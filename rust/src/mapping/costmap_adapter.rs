use anyhow::{bail, Result};
use serde::Serialize;
use serde_json::Value;

#[derive(Clone, Debug, Serialize)]
pub struct CostmapAdapterOptions {
    pub input_topic: String,
    pub input_format: String,
    pub updates_topic: String,
    pub snapshot_service: String,
    pub output_topic: String,
    pub publish_hz: f64,
    pub resync_seconds: f64,
    pub volatile_full: bool,
}
impl CostmapAdapterOptions {
    pub fn new(input_topic: impl Into<String>, output_topic: impl Into<String>) -> Self {
        Self {
            input_topic: input_topic.into(),
            input_format: "occupancy".into(),
            updates_topic: String::new(),
            snapshot_service: String::new(),
            output_topic: output_topic.into(),
            publish_hz: 5.0,
            resync_seconds: 2.0,
            volatile_full: false,
        }
    }
}
#[cfg(feature = "ros2")]
mod ffi {
    pub use std::ffi::{c_char, c_void, CStr, CString};
    extern "C" {
        pub fn horus_native_costmap_create(
            options: *const c_char,
            error: *mut *mut c_char,
        ) -> *mut c_void;
        pub fn horus_native_costmap_status(adapter: *mut c_void) -> *mut c_char;
        pub fn horus_native_costmap_destroy(adapter: *mut c_void);
        pub fn horus_native_free(value: *mut c_char);
    }
    pub unsafe fn take(text: *mut c_char) -> String {
        if text.is_null() {
            return "Native costmap returned no result".into();
        }
        let result = CStr::from_ptr(text).to_string_lossy().into_owned();
        horus_native_free(text);
        result
    }
}
/// Private ROS executor; no Python runtime. Drop cancels and joins only its worker.
pub struct CostmapAdapter {
    #[cfg(feature = "ros2")]
    handle: std::ptr::NonNull<ffi::c_void>,
}
// The runtime serializes callbacks and status reads and owns its worker/context.
unsafe impl Send for CostmapAdapter {}
unsafe impl Sync for CostmapAdapter {}
impl CostmapAdapter {
    pub fn new(options: CostmapAdapterOptions) -> Result<Self> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = options;
            bail!("Enable ros2 for native costmap adaptation");
        }
        #[cfg(feature = "ros2")]
        {
            let text = ffi::CString::new(serde_json::to_vec(&options)?)?;
            let mut error = std::ptr::null_mut();
            let raw = unsafe { ffi::horus_native_costmap_create(text.as_ptr(), &mut error) };
            let Some(handle) = std::ptr::NonNull::new(raw) else {
                bail!("{}", unsafe { ffi::take(error) });
            };
            Ok(Self { handle })
        }
    }
    pub fn status(&self) -> Result<Value> {
        #[cfg(not(feature = "ros2"))]
        {
            bail!("Enable ros2 for native costmap adaptation");
        }
        #[cfg(feature = "ros2")]
        {
            let value: Value = serde_json::from_str(&unsafe {
                ffi::take(ffi::horus_native_costmap_status(self.handle.as_ptr()))
            })?;
            if value["ok"] != true {
                bail!("{}", value["error"]);
            }
            Ok(value["value"].clone())
        }
    }
}
impl Drop for CostmapAdapter {
    fn drop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_native_costmap_destroy(self.handle.as_ptr());
        }
    }
}
