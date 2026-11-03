//! Native Teach configuration, bounded MCAP recording and authority-aware control.
//!
//! No Python process, ROS node, robot connection or disk worker starts at import.
//! The shared C++ core is compiled into this crate; robot adapters remain local.
use anyhow::{anyhow, bail, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::cell::Cell;
use std::ffi::{c_char, c_void, CStr, CString};
use std::marker::PhantomData;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr::NonNull;

mod cartesian;
mod config;
mod runtime;
pub use cartesian::{CartesianAdapter, CartesianController, CartesianLimits};
pub use config::{TeachCamera, TeachPortal, TeachStream, TeachPointCloud};
pub use runtime::TeachRuntime;

type AdapterCall = unsafe extern "C" fn(*mut c_void, *const c_char, *const c_char) -> *const c_char;
type AdapterFree = unsafe extern "C" fn(*mut c_void, *const c_char);
type SampleCall = unsafe extern "C" fn(*mut c_void, *const c_char, *const u8, usize) -> i32;
extern "C" {
    fn horus_teach_monotonic_ns() -> u64;
    fn horus_teach_config(config: *const c_char, robot: *const c_char) -> *mut c_char;
    fn horus_teach_free(value: *mut c_char);
    fn horus_teach_session_create(
        config: *const c_char,
        robot: *const c_char,
        context: *mut c_void,
        callback: Option<AdapterCall>,
        release: Option<AdapterFree>,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_teach_session_call(
        session: *mut c_void,
        operation: *const c_char,
        value: *const c_char,
    ) -> *mut c_char;
    fn horus_teach_session_append(
        session: *mut c_void,
        metadata: *const c_char,
        data: *const u8,
        size: usize,
    ) -> *mut c_char;
    fn horus_teach_session_destroy(session: *mut c_void);
    fn horus_teach_recorder_create(
        config: *const c_char,
        robot: *const c_char,
        episode: *const c_char,
        task: *const c_char,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_teach_recorder_call(
        recorder: *mut c_void,
        operation: *const c_char,
        value: *const c_char,
    ) -> *mut c_char;
    fn horus_teach_recorder_append(
        recorder: *mut c_void,
        metadata: *const c_char,
        data: *const u8,
        size: usize,
    ) -> *mut c_char;
    fn horus_teach_recorder_destroy(recorder: *mut c_void);
    fn horus_teach_episode_validate(path: *const c_char) -> *mut c_char;
    fn horus_teach_episode_recover(path: *const c_char) -> *mut c_char;
    fn horus_teach_episode_read(
        path: *const c_char,
        options: *const c_char,
        context: *mut c_void,
        callback: SampleCall,
    ) -> *mut c_char;
}
/// Timestamp fresh controller feedback in the same monotonic clock domain as
/// the native watchdog. Do not use Unix time or an arbitrary Instant origin.
pub fn monotonic_seconds() -> f64 {
    unsafe { horus_teach_monotonic_ns() as f64 * 1e-9 }
}
/// Recovers complete records into a new interrupted/unacceptable episode.
/// Never edits the source evidence or recovers a capture with a held lock.
pub fn recover_episode(path: &str) -> Result<String> {
    let path = CString::new(path)?;
    let value = unsafe { response(horus_teach_episode_recover(path.as_ptr()))? };
    Ok(value
        .as_str()
        .ok_or_else(|| anyhow!("Invalid recovery result"))?
        .to_owned())
}
fn cjson(value: &Value) -> Result<CString> {
    Ok(CString::new(serde_json::to_vec(value)?)?)
}
unsafe fn response(value: *mut c_char) -> Result<Value> {
    if value.is_null() {
        bail!("Native Teach returned no result (allocation failure)");
    }
    let result = serde_json::from_slice::<Value>(CStr::from_ptr(value).to_bytes());
    horus_teach_free(value);
    let result = result?;
    if result["ok"] != true {
        bail!(
            "{}",
            result["error"].as_str().unwrap_or("Native Teach failed")
        );
    }
    Ok(result["value"].clone())
}
unsafe fn created(ptr: *mut c_void, error: *mut c_char) -> Result<NonNull<c_void>> {
    let message = if error.is_null() {
        "Native Teach initialization failed".to_owned()
    } else {
        let value = CStr::from_ptr(error).to_string_lossy().into_owned();
        horus_teach_free(error);
        value
    };
    NonNull::new(ptr).ok_or_else(|| anyhow!(message))
}

#[derive(Clone, Debug)]
pub struct TeachConfig {
    value: Value,
    digest: String,
}
impl TeachConfig {
    pub fn new(recording_root: impl Into<String>) -> Result<Self> {
        Self::from_value(json!({"recording_root": recording_root.into()}))
    }
    pub fn from_value(value: Value) -> Result<Self> {
        let input = cjson(&value)?;
        let output = unsafe { response(horus_teach_config(input.as_ptr(), std::ptr::null()))? };
        Ok(Self {
            value: output["config"].clone(),
            digest: output["digest"].as_str().unwrap().to_owned(),
        })
    }
    pub fn to_value(&self) -> &Value {
        &self.value
    }
    pub fn digest(&self) -> &str {
        &self.digest
    }
    pub fn registration(&self, robot: &str) -> Result<Value> {
        let config = cjson(&self.value)?;
        let robot = CString::new(robot)?;
        Ok(
            unsafe { response(horus_teach_config(config.as_ptr(), robot.as_ptr()))? }
                ["registration"]
                .clone(),
        )
    }
    /// Validates the entire new profile atomically; does not modify this value.
    pub fn with_options(&self, options: Value) -> Result<Self> {
        let mut value = self.value.clone();
        let options = options
            .as_object()
            .ok_or_else(|| anyhow!("Teach options must be an object"))?;
        for (key, item) in options {
            value[key] = item.clone();
        }
        Self::from_value(value)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct AdapterCapabilities {
    pub adapter_id: String,
    pub qualification: String,
    pub joint_names: Vec<String>,
    pub command_frame: String,
    pub calibration_id: String,
    pub action_representation: String,
    pub controller_watchdog: String,
    pub profile_revision: String,
    pub joint_units: Value,
    pub command_boundary: String,
    pub hold_description: String,
    pub command_period_seconds: Option<f64>,
    pub authority_contract: String,
    pub supports_tracking_pause: bool,
}
impl Default for AdapterCapabilities {
    fn default() -> Self {
        Self {
            adapter_id: "observe".into(),
            qualification: "observation".into(),
            joint_names: vec![],
            command_frame: String::new(),
            calibration_id: String::new(),
            action_representation: "none".into(),
            controller_watchdog: "unavailable".into(),
            profile_revision: "unspecified".into(),
            joint_units: json!({}),
            command_boundary: "unavailable".into(),
            hold_description: "unavailable".into(),
            command_period_seconds: None,
            authority_contract: "unavailable".into(),
            supports_tracking_pause: false,
        }
    }
}
/// Callbacks must not block on disk/network. Hardware adapters must enforce
/// authority-scoped commands/hold and have their own controller watchdog.
pub trait TeachAdapter: Send {
    fn capabilities(&self) -> AdapterCapabilities;
    fn ready(&mut self) -> Result<(bool, String)>;
    fn anchor(&mut self, sample: &Value) -> Result<()>;
    fn apply(&mut self, sample: &Value) -> Result<Value>;
    fn hold(&mut self, context: &Value) -> Result<bool>;
    /// Confirm measured hold, discard pending targets and retain the active anchor/authority.
    /// Only a subsequently accepted fresh apply may resume motion.
    fn pause(&mut self, _context: &Value) -> Result<bool> {
        bail!("Tracking pause unavailable")
    }
}
pub struct ObserveAdapter;
impl TeachAdapter for ObserveAdapter {
    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities::default()
    }
    fn ready(&mut self) -> Result<(bool, String)> {
        Ok((false, "Observation-only adapter".into()))
    }
    fn anchor(&mut self, _: &Value) -> Result<()> {
        bail!("Observation-only adapter cannot engage")
    }
    fn apply(&mut self, _: &Value) -> Result<Value> {
        bail!("Observation-only adapter cannot send commands")
    }
    fn hold(&mut self, _: &Value) -> Result<bool> {
        Ok(false)
    }
}
struct AdapterBox(Box<dyn TeachAdapter>);
unsafe extern "C" fn adapter_call(
    context: *mut c_void,
    op: *const c_char,
    input: *const c_char,
) -> *const c_char {
    let result = catch_unwind(AssertUnwindSafe(|| -> Result<Value> {
        let adapter = &mut (*(context as *mut AdapterBox)).0;
        let input: Value = serde_json::from_slice(CStr::from_ptr(input).to_bytes())?;
        match CStr::from_ptr(op).to_str()? {
            "capabilities" => Ok(serde_json::to_value(adapter.capabilities())?),
            "ready" => {
                let (ready, reason) = adapter.ready()?;
                Ok(json!({"ready":ready,"reason":reason}))
            }
            "anchor" => {
                adapter.anchor(&input)?;
                Ok(Value::Null)
            }
            "apply" => adapter.apply(&input),
            "hold" => Ok(json!(adapter.hold(&input)?)),
            "pause" => Ok(json!(adapter.pause(&input)?)),
            _ => bail!("Unknown adapter operation"),
        }
    }));
    let output = match result {
        Ok(Ok(value)) => json!({"ok":true,"value":value}),
        Ok(Err(error)) => json!({"ok":false,"error":error.to_string()}),
        Err(_) => json!({"ok":false,"error":"Teach adapter panicked"}),
    };
    cjson(&output)
        .map(CString::into_raw)
        .unwrap_or(std::ptr::null_mut())
}
unsafe extern "C" fn adapter_free(_: *mut c_void, value: *const c_char) {
    if !value.is_null() {
        drop(CString::from_raw(value as *mut c_char));
    }
}

pub struct TeachSession {
    handle: NonNull<c_void>,
    _adapter: Option<Box<AdapterBox>>,
    _not_sync: PhantomData<Cell<()>>,
}
// Unique ownership; callbacks are Send and the engine serializes its workers.
unsafe impl Send for TeachSession {}
impl TeachSession {
    pub fn new(
        config: &TeachConfig,
        robot: &str,
        adapter: Option<Box<dyn TeachAdapter>>,
    ) -> Result<Self> {
        let mut adapter = adapter.map(|v| Box::new(AdapterBox(v)));
        let config = cjson(config.to_value())?;
        let robot = CString::new(robot)?;
        let context = adapter
            .as_mut()
            .map(|v| &mut **v as *mut AdapterBox as *mut c_void)
            .unwrap_or(std::ptr::null_mut());
        let callback = adapter.as_ref().map(|_| adapter_call as AdapterCall);
        let release = adapter.as_ref().map(|_| adapter_free as AdapterFree);
        let mut error = std::ptr::null_mut();
        let ptr = unsafe {
            horus_teach_session_create(
                config.as_ptr(),
                robot.as_ptr(),
                context,
                callback,
                release,
                &mut error,
            )
        };
        Ok(Self {
            handle: unsafe { created(ptr, error)? },
            _adapter: adapter,
            _not_sync: PhantomData,
        })
    }
    fn call(&mut self, op: &str, value: Value) -> Result<Value> {
        let op = CString::new(op)?;
        let value = cjson(&value)?;
        unsafe {
            response(horus_teach_session_call(
                self.handle.as_ptr(),
                op.as_ptr(),
                value.as_ptr(),
            ))
        }
    }
    pub fn state(&mut self) -> Result<Value> {
        self.call("state", Value::Null)
    }
    pub fn request(&mut self, request: Value) -> Result<Value> {
        self.call("request", request)
    }
    pub fn input(&mut self, sample: Value) -> Result<bool> {
        Ok(self.call("input", sample)? == true)
    }
    pub fn update_lease(&mut self, owner: &str, generation: &str, ttl: f64) -> Result<()> {
        if !ttl.is_finite() {
            bail!("Lease TTL must be finite");
        }
        self.call(
            "lease",
            json!({"owner":owner,"generation":generation,"ttl":ttl}),
        )?;
        Ok(())
    }
    pub fn note_stream(&mut self, id: &str) -> Result<()> {
        self.call("note_stream", json!(id))?;
        Ok(())
    }
    pub fn missing_streams(&mut self, control: bool) -> Result<Vec<String>> {
        Ok(serde_json::from_value(
            self.call("missing_streams", json!(control))?,
        )?)
    }
    pub fn readiness(&mut self) -> Result<(bool, String)> {
        let v = self.call("readiness", Value::Null)?;
        Ok((
            v["ready"] == true,
            v["reason"].as_str().unwrap_or("").into(),
        ))
    }
    pub fn tick(&mut self) -> Result<()> {
        self.call("tick", Value::Null)?;
        Ok(())
    }
    pub fn close(&mut self) -> Result<()> {
        self.call("close", Value::Null)?;
        Ok(())
    }
    pub fn append(&mut self, sample: &Sample<'_>) -> Result<bool> {
        let metadata = cjson(&sample.metadata())?;
        Ok(unsafe {
            response(horus_teach_session_append(
                self.handle.as_ptr(),
                metadata.as_ptr(),
                sample.payload.as_ptr(),
                sample.payload.len(),
            ))?
        } == true)
    }
}
impl Drop for TeachSession {
    fn drop(&mut self) {
        unsafe {
            horus_teach_session_destroy(self.handle.as_ptr());
        }
    }
}

/// Raw payload is borrowed until append returns; the bounded native queue owns
/// one copy afterwards. Receipt clocks must be supplied by the recorder host.
pub struct Sample<'a> {
    pub stream_id: &'a str,
    pub payload: &'a [u8],
    pub received_unix_ns: Option<u64>,
    pub received_monotonic_ns: Option<u64>,
    pub source_ns: Option<u64>,
    pub source_sequence: Option<u64>,
    pub encoding: &'a str,
    pub schema: &'a str,
    pub schema_encoding: &'a str,
}
impl<'a> Sample<'a> {
    pub fn new(stream_id: &'a str, payload: &'a [u8]) -> Self {
        Self {
            stream_id,
            payload,
            received_unix_ns: None,
            received_monotonic_ns: None,
            source_ns: None,
            source_sequence: None,
            encoding: "cdr",
            schema: "",
            schema_encoding: "",
        }
    }
    fn metadata(&self) -> Value {
        let mut v = json!({"stream_id":self.stream_id,"source_ns":self.source_ns,"source_sequence":self.source_sequence,
            "encoding":self.encoding,"schema":self.schema,"schema_encoding":self.schema_encoding});
        if let Some(time) = self.received_unix_ns {
            v["received_unix_ns"] = json!(time);
        }
        if let Some(time) = self.received_monotonic_ns {
            v["received_monotonic_ns"] = json!(time);
        }
        v
    }
}
pub struct EpisodeRecorder {
    handle: NonNull<c_void>,
    _not_sync: PhantomData<Cell<()>>,
}
unsafe impl Send for EpisodeRecorder {}
impl EpisodeRecorder {
    pub fn new(config: &TeachConfig, robot: &str, episode_id: &str, task: &str) -> Result<Self> {
        let config = cjson(config.to_value())?;
        let robot = CString::new(robot)?;
        let episode = CString::new(episode_id)?;
        let task = CString::new(task)?;
        let mut error = std::ptr::null_mut();
        let ptr = unsafe {
            horus_teach_recorder_create(
                config.as_ptr(),
                robot.as_ptr(),
                episode.as_ptr(),
                task.as_ptr(),
                &mut error,
            )
        };
        Ok(Self {
            handle: unsafe { created(ptr, error)? },
            _not_sync: PhantomData,
        })
    }
    fn call(&mut self, op: &str, value: Value) -> Result<Value> {
        let op = CString::new(op)?;
        let value = cjson(&value)?;
        unsafe {
            response(horus_teach_recorder_call(
                self.handle.as_ptr(),
                op.as_ptr(),
                value.as_ptr(),
            ))
        }
    }
    pub fn append(&mut self, sample: &Sample<'_>) -> Result<bool> {
        let metadata = cjson(&sample.metadata())?;
        Ok(unsafe {
            response(horus_teach_recorder_append(
                self.handle.as_ptr(),
                metadata.as_ptr(),
                sample.payload.as_ptr(),
                sample.payload.len(),
            ))?
        } == true)
    }
    pub fn event(&mut self, kind: &str, data: Value) -> Result<bool> {
        Ok(self.call("event", json!({"kind":kind,"data":data}))? == true)
    }
    pub fn status(&mut self) -> Result<Value> {
        self.call("status", Value::Null)
    }
    pub fn finish(&mut self, outcome: &str, interrupted: bool) -> Result<Value> {
        self.call(
            "finish",
            json!({"outcome":outcome,"interrupted":interrupted}),
        )
    }
}
impl Drop for EpisodeRecorder {
    fn drop(&mut self) {
        unsafe {
            horus_teach_recorder_destroy(self.handle.as_ptr());
        }
    }
}

pub struct Episode {
    path: CString,
    pub manifest: Value,
}
impl Episode {
    pub fn open(path: &str) -> Result<Self> {
        let path = CString::new(path)?;
        let v = unsafe { response(horus_teach_episode_validate(path.as_ptr()))? };
        Ok(Self {
            path,
            manifest: v["manifest"].clone(),
        })
    }
    pub fn validate(&self) -> Result<Vec<String>> {
        let v = unsafe { response(horus_teach_episode_validate(self.path.as_ptr()))? };
        Ok(serde_json::from_value(v["issues"].clone())?)
    }
    /// Streaming reader; return false to stop. Data cannot outlive the callback.
    pub fn samples<F: FnMut(&Value, &[u8]) -> Result<bool>>(
        &self,
        streams: &[&str],
        start_ns: u64,
        end_ns: u64,
        visitor: F,
    ) -> Result<()> {
        struct Reader<F> {
            visitor: F,
            error: Option<anyhow::Error>,
        }
        unsafe extern "C" fn receive<F: FnMut(&Value, &[u8]) -> Result<bool>>(
            ctx: *mut c_void,
            meta: *const c_char,
            data: *const u8,
            len: usize,
        ) -> i32 {
            let reader = &mut *(ctx as *mut Reader<F>);
            let result = catch_unwind(AssertUnwindSafe(|| -> Result<bool> {
                let meta = serde_json::from_slice(CStr::from_ptr(meta).to_bytes())?;
                (reader.visitor)(
                    &meta,
                    if len == 0 {
                        &[]
                    } else {
                        std::slice::from_raw_parts(data, len)
                    },
                )
            }));
            match result {
                Ok(Ok(v)) => i32::from(v),
                Ok(Err(e)) => {
                    reader.error = Some(e);
                    0
                }
                Err(_) => {
                    reader.error = Some(anyhow!("Episode visitor panicked"));
                    0
                }
            }
        }
        let mut reader = Reader {
            visitor,
            error: None,
        };
        let options = cjson(&json!({"streams":streams,"start_ns":start_ns,"end_ns":end_ns}))?;
        unsafe {
            response(horus_teach_episode_read(
                self.path.as_ptr(),
                options.as_ptr(),
                &mut reader as *mut _ as *mut c_void,
                receive::<F>,
            ))?;
        }
        if let Some(e) = reader.error {
            return Err(e);
        }
        Ok(())
    }
}
