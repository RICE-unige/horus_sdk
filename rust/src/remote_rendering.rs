//! Native remote-rendering v3 contracts, framing, clock alignment and newest-only queues.
//! This module does not start a GPU renderer or a media server.
pub mod atlas;
pub mod datasets;
pub mod encoder;
pub mod frame;
pub mod gpu;
pub mod publisher;
pub mod scene;
pub mod stereo;
pub mod video;
use anyhow::{anyhow, bail, ensure, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::VecDeque;
use std::ffi::{c_char, CStr, CString};
use std::sync::{Condvar, Mutex};
use std::time::Duration;

pub const PROTOCOL_VERSION: &str = "horus.remote_render.v3";
pub const VIEW_STATE_VERSION: &str = "horus.remote_render.view_state.v3";
pub const FRAME_METADATA_VERSION: &str = "horus.remote_render.frame.v3";
pub const ROS_DIAGNOSTIC_FORMAT_VERSION: &str = "horus.remote_render.v3.ros";
pub const WEBRTC_FORMAT_VERSION: &str = "horus.remote_render.v3.webrtc";
pub const DEFAULT_VIEW_STATE_TOPIC: &str = "/horus/remote_render/v3/view_state";
pub const DEFAULT_ROS_DIAGNOSTIC_TOPIC: &str = "/horus/remote_render/v3/stereo";
pub const DEFAULT_STATUS_TOPIC: &str = "/horus/remote_render/v3/status";
pub const MAXIMUM_METADATA_BYTES: usize = 64 * 1024;
pub const MAXIMUM_IMAGE_BYTES: usize = 32 * 1024 * 1024;
pub type Matrix4 = [[f64; 4]; 4];

unsafe extern "C" {
    fn horus_remote_monotonic_time_ns() -> u64;
    fn horus_remote_contract(operation: *const c_char, value: *const c_char) -> *mut c_char;
    fn horus_teach_free(value: *mut c_char);
}
/// Host monotonic time for render/encode timing. Do not use for ROS header stamps.
pub fn monotonic_time_ns() -> u64 {
    unsafe { horus_remote_monotonic_time_ns() }
}
fn contract(operation: &str, value: &Value) -> Result<Value> {
    let operation = CString::new(operation)?;
    let value = CString::new(serde_json::to_string(value)?)?;
    let result = unsafe { horus_remote_contract(operation.as_ptr(), value.as_ptr()) };
    ensure!(
        !result.is_null(),
        "Native remote contract returned no result"
    );
    let parsed = unsafe { serde_json::from_slice::<Value>(CStr::from_ptr(result).to_bytes()) };
    unsafe { horus_teach_free(result) };
    let result = parsed?;
    ensure!(
        result["ok"] == true,
        "{}",
        result["error"]
            .as_str()
            .unwrap_or("Invalid remote contract")
    );
    Ok(result["value"].clone())
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct EyeFov {
    pub left: f64,
    pub right: f64,
    pub up: f64,
    pub down: f64,
}
impl EyeFov {
    pub fn renderer_projection(&self) -> Result<[f64; 4]> {
        ensure!(
            [self.left, self.right, self.up, self.down]
                .iter()
                .all(|x| x.is_finite()),
            "FOV must be finite"
        );
        ensure!(
            self.left < 0.0 && self.right > 0.0 && self.down < 0.0 && self.up > 0.0,
            "FOV must cross both axes"
        );
        let x = self.right - self.left;
        let y = self.up - self.down;
        let p = [
            2.0 / x,
            2.0 / y,
            -(self.right + self.left) / x,
            -(self.up + self.down) / y,
        ];
        ensure!(
            x.is_finite() && y.is_finite() && p.iter().all(|v| v.is_finite()),
            "FOV range overflow"
        );
        Ok(p)
    }
}
pub fn eye_fov_from_unity_projection(m00: f64, m11: f64, m02: f64, m12: f64) -> Result<EyeFov> {
    ensure!(
        [m00, m11, m02, m12].iter().all(|x| x.is_finite()) && m00 > 0.0 && m11 > 0.0,
        "Invalid projection scales"
    );
    let fov = EyeFov {
        left: (-1.0 + m02) / m00,
        right: (1.0 + m02) / m00,
        up: (1.0 + m12) / m11,
        down: (-1.0 + m12) / m11,
    };
    fov.renderer_projection()?;
    Ok(fov)
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct EyeView {
    pub position: Vector3,
    pub rotation: Quaternion,
    pub fov: EyeFov,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct WorkspaceState {
    pub revision: u64,
    pub position: Vector3,
    pub rotation: Quaternion,
    pub scale: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ViewState {
    pub session_id: String,
    pub view_sequence: u64,
    pub sample_time_ns: u64,
    pub predicted_display_time_ns: u64,
    pub predicted_display_period_ns: u64,
    pub reference_space_id: String,
    pub workspace: WorkspaceState,
    pub left_eye: EyeView,
    pub right_eye: EyeView,
    pub viewport_width: u32,
    pub viewport_height: u32,
    pub refresh_rate_hz: f64,
    #[serde(default)]
    pub view_stream_id: String,
}
impl ViewState {
    pub fn from_value(value: &Value) -> Result<Self> {
        Ok(serde_json::from_value(contract("view", value)?)?)
    }
    pub fn from_json(text: &str) -> Result<Self> {
        ensure!(
            text.len() <= MAXIMUM_METADATA_BYTES,
            "View metadata exceeds 64 KiB"
        );
        Self::from_value(&serde_json::from_str(text)?)
    }
    pub fn to_value(&self) -> Result<Value> {
        let mut j = serde_json::to_value(self)?;
        j["version"] = VIEW_STATE_VERSION.into();
        contract("view", &j)
    }
    pub fn to_json(&self) -> Result<String> {
        Ok(serde_json::to_string(&self.to_value()?)?)
    }
    pub fn eye_matrices(&self, near: f64, far: f64) -> Result<StereoMatrices> {
        ensure!(near.is_finite() && far.is_finite(), "Invalid depth range");
        Ok(serde_json::from_value(contract(
            "matrices",
            &serde_json::json!({"view":self.to_value()?,"near":near,"far":far}),
        )?)?)
    }
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StereoMatrices {
    pub left_view: Matrix4,
    pub right_view: Matrix4,
    pub left_projection: Matrix4,
    pub right_projection: Matrix4,
    pub left_vp: Matrix4,
    pub right_vp: Matrix4,
}
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RenderedFrameMetadata {
    pub session_id: String,
    pub frame_id: u64,
    pub source_view_sequence: u64,
    pub predicted_display_time_ns: u64,
    pub workspace_revision: u64,
    pub render_start_ns: u64,
    pub render_complete_ns: u64,
    pub encode_start_ns: u64,
    pub capture_time_ns: u64,
    pub stereo_layout: String,
    pub width_per_eye: u32,
    pub height_per_eye: u32,
    pub codec: String,
    pub color_space: String,
    pub color_range: String,
    pub depth_representation: String,
    pub visibility_mask_height: u32,
    pub frame_marker_height: u32,
    pub frame_marker: u16,
    pub video_height: u32,
    pub depth_near_m: f64,
    pub depth_far_m: f64,
    pub view_stream_id: String,
}
impl RenderedFrameMetadata {
    pub fn from_value(value: &Value) -> Result<Self> {
        Ok(serde_json::from_value(contract("metadata", value)?)?)
    }
    pub fn from_json(text: &str) -> Result<Self> {
        ensure!(
            text.len() <= MAXIMUM_METADATA_BYTES,
            "Frame metadata exceeds 64 KiB"
        );
        Self::from_value(&serde_json::from_str(text)?)
    }
    pub fn to_value(&self) -> Result<Value> {
        let mut j = serde_json::to_value(self)?;
        j["version"] = FRAME_METADATA_VERSION.into();
        contract("metadata", &j)
    }
    pub fn to_json(&self) -> Result<String> {
        Ok(serde_json::to_string(&self.to_value()?)?)
    }
}
pub fn pack_ros_diagnostic_frame(
    metadata: &RenderedFrameMetadata,
    image: &[u8],
) -> Result<Vec<u8>> {
    let metadata = metadata.to_json()?;
    ensure!(
        !metadata.is_empty()
            && metadata.len() <= MAXIMUM_METADATA_BYTES
            && !image.is_empty()
            && image.len() <= MAXIMUM_IMAGE_BYTES,
        "Diagnostic frame exceeds limits"
    );
    let mut out = Vec::with_capacity(16 + metadata.len() + image.len());
    out.extend_from_slice(b"HRV3\x01\0\0\0");
    out.extend_from_slice(&(metadata.len() as u32).to_le_bytes());
    out.extend_from_slice(&(image.len() as u32).to_le_bytes());
    out.extend_from_slice(metadata.as_bytes());
    out.extend_from_slice(image);
    Ok(out)
}
/// Decodes bounded metadata and borrows image bytes without copying the image.
pub fn unpack_ros_diagnostic_frame(bytes: &[u8]) -> Result<(RenderedFrameMetadata, &[u8])> {
    ensure!(
        bytes.len() >= 16 && &bytes[..8] == b"HRV3\x01\0\0\0",
        "Invalid diagnostic header"
    );
    let m = u32::from_le_bytes(bytes[8..12].try_into()?) as usize;
    let i = u32::from_le_bytes(bytes[12..16].try_into()?) as usize;
    ensure!(
        m > 0
            && m <= MAXIMUM_METADATA_BYTES
            && i > 0
            && i <= MAXIMUM_IMAGE_BYTES
            && bytes.len() == 16 + m + i,
        "Invalid diagnostic frame lengths"
    );
    Ok((
        RenderedFrameMetadata::from_json(std::str::from_utf8(&bytes[16..16 + m])?)?,
        &bytes[16 + m..],
    ))
}
#[derive(Debug, Clone, Copy)]
pub struct ClockSample {
    pub client_send_ns: i64,
    pub server_receive_ns: i64,
    pub server_send_ns: i64,
    pub client_receive_ns: i64,
}
impl ClockSample {
    pub fn round_trip_ns(&self) -> f64 {
        ((self.client_receive_ns as i128 - self.client_send_ns as i128)
            - (self.server_send_ns as i128 - self.server_receive_ns as i128))
            .max(0) as f64
    }
    pub fn server_minus_client_ns(&self) -> f64 {
        ((self.server_receive_ns as i128 - self.client_send_ns as i128)
            + (self.server_send_ns as i128 - self.client_receive_ns as i128)) as f64
            / 2.0
    }
}
fn median(mut values: Vec<f64>) -> Result<f64> {
    ensure!(!values.is_empty(), "Clock alignment has no samples");
    values.sort_by(f64::total_cmp);
    let n = values.len();
    Ok(if n % 2 == 1 {
        values[n / 2]
    } else {
        values[n / 2 - 1] / 2.0 + values[n / 2] / 2.0
    })
}
pub struct ClockAlignment {
    maximum: usize,
    samples: VecDeque<ClockSample>,
}
impl ClockAlignment {
    pub fn new(maximum_samples: usize) -> Result<Self> {
        ensure!(
            (3..=65536).contains(&maximum_samples),
            "Clock sample bound must be 3..65536"
        );
        Ok(Self {
            maximum: maximum_samples,
            samples: VecDeque::new(),
        })
    }
    pub fn add(&mut self, s: ClockSample) -> Result<()> {
        ensure!(
            s.client_send_ns >= 0
                && s.server_receive_ns >= 0
                && s.server_send_ns >= s.server_receive_ns
                && s.client_receive_ns >= s.client_send_ns,
            "Invalid clock sample ordering"
        );
        if self.samples.len() == self.maximum {
            self.samples.pop_front();
        }
        self.samples.push_back(s);
        Ok(())
    }
    pub fn sample_count(&self) -> usize {
        self.samples.len()
    }
    pub fn offset_ns(&self) -> Result<f64> {
        let mut ranked: Vec<_> = self.samples.iter().collect();
        ranked.sort_by(|a, b| a.round_trip_ns().total_cmp(&b.round_trip_ns()));
        let count = (ranked.len() / 3).max(1);
        median(
            ranked
                .into_iter()
                .take(count)
                .map(|s| s.server_minus_client_ns())
                .collect(),
        )
    }
    pub fn round_trip_ns(&self) -> Result<f64> {
        median(
            self.samples
                .iter()
                .map(ClockSample::round_trip_ns)
                .collect(),
        )
    }
}
impl Default for ClockAlignment {
    fn default() -> Self {
        Self::new(32).expect("valid default")
    }
}
pub struct ViewStreamSequenceGate {
    timeout: f64,
    last_received: f64,
    active: Option<String>,
    last_sequence: u64,
    last_sample: u64,
    revision: u64,
    retired: VecDeque<String>,
}
impl ViewStreamSequenceGate {
    pub fn new(idle_timeout: f64) -> Result<Self> {
        ensure!(
            idle_timeout.is_finite() && idle_timeout > 0.0,
            "Idle timeout must be finite and positive"
        );
        Ok(Self {
            timeout: idle_timeout,
            last_received: 0.0,
            active: None,
            last_sequence: 0,
            last_sample: 0,
            revision: 0,
            retired: VecDeque::new(),
        })
    }
    pub fn revision(&self) -> u64 {
        self.revision
    }
    pub fn accept(&mut self, state: &ViewState, now: f64) -> Result<Option<&'static str>> {
        ensure!(
            now.is_finite() && now >= 0.0 && (self.active.is_none() || now >= self.last_received),
            "Use a monotonic receipt clock"
        );
        let idle = now - self.last_received >= self.timeout;
        if let Some(active) = &self.active {
            if state.view_stream_id != *active {
                if self.retired.contains(&state.view_stream_id) {
                    return Ok(Some("retired_view_stream"));
                }
                if !idle {
                    return Ok(Some("view_stream_busy"));
                }
                if self.retired.len() == 128 {
                    self.retired.pop_front();
                }
                self.retired.push_back(active.clone());
            } else {
                if state.sample_time_ns <= self.last_sample {
                    return Ok(Some("stale_sample"));
                }
                if state.view_sequence <= self.last_sequence
                    && (!state.view_stream_id.is_empty() || !idle)
                {
                    return Ok(Some("stale_sequence"));
                }
            }
        }
        self.active = Some(state.view_stream_id.clone());
        self.last_sequence = state.view_sequence;
        self.last_sample = state.sample_time_ns;
        self.last_received = now;
        self.revision += 1;
        Ok(None)
    }
}
impl Default for ViewStreamSequenceGate {
    fn default() -> Self {
        Self::new(2.0).expect("valid default")
    }
}
pub fn select_newest_view(states: &[ViewState]) -> Result<&ViewState> {
    let first = states
        .first()
        .ok_or_else(|| anyhow!("At least one view is required"))?;
    let mut newest = first;
    for s in states {
        ensure!(s.session_id == first.session_id, "Mixed view sessions");
        if s.view_sequence > newest.view_sequence {
            newest = s;
        }
    }
    Ok(newest)
}
struct QueueState<T> {
    value: Option<T>,
    closed: bool,
    replaced: u64,
}
pub struct LatestValueQueue<T> {
    state: Mutex<QueueState<T>>,
    ready: Condvar,
}
impl<T> Default for LatestValueQueue<T> {
    fn default() -> Self {
        Self {
            state: Mutex::new(QueueState {
                value: None,
                closed: false,
                replaced: 0,
            }),
            ready: Condvar::new(),
        }
    }
}
impl<T> LatestValueQueue<T> {
    pub fn put(&self, value: T) -> Result<()> {
        let mut s = self.state.lock().map_err(|_| anyhow!("Queue poisoned"))?;
        if s.closed {
            bail!("Queue is closed");
        }
        s.replaced += u64::from(s.value.is_some());
        s.value = Some(value);
        self.ready.notify_one();
        Ok(())
    }
    pub fn get(&self, timeout: Option<Duration>) -> Result<Option<T>> {
        let s = self.state.lock().map_err(|_| anyhow!("Queue poisoned"))?;
        let mut s = match timeout {
            Some(t) => {
                self.ready
                    .wait_timeout_while(s, t, |s| s.value.is_none() && !s.closed)
                    .map_err(|_| anyhow!("Queue poisoned"))?
                    .0
            }
            None => self
                .ready
                .wait_while(s, |s| s.value.is_none() && !s.closed)
                .map_err(|_| anyhow!("Queue poisoned"))?,
        };
        Ok(s.value.take())
    }
    pub fn close(&self) -> Result<()> {
        let mut s = self.state.lock().map_err(|_| anyhow!("Queue poisoned"))?;
        s.closed = true;
        self.ready.notify_all();
        Ok(())
    }
    pub fn replaced_count(&self) -> Result<u64> {
        Ok(self
            .state
            .lock()
            .map_err(|_| anyhow!("Queue poisoned"))?
            .replaced)
    }
}
pub mod gaussian;
