//! Native ROS endpoint for application-owned stereo renderers.
use super::{
    encoder::EncodedH264Frame, stereo::StereoFrameRender, ViewState, DEFAULT_ROS_DIAGNOSTIC_TOPIC,
    DEFAULT_STATUS_TOPIC, DEFAULT_VIEW_STATE_TOPIC,
};
use anyhow::{bail, Result};
use serde::Serialize;
use serde_json::Value;

#[derive(Clone, Debug, Serialize)]
pub struct RemotePublisherOptions {
    pub session_id: String,
    pub mode: String,
    pub view_topic: String,
    pub stream_topic: String,
    pub status_topic: String,
    pub maximum_view_age_seconds: f64,
}
impl Default for RemotePublisherOptions {
    fn default() -> Self {
        Self {
            session_id: String::new(),
            mode: "diagnostic".into(),
            view_topic: DEFAULT_VIEW_STATE_TOPIC.into(),
            stream_topic: DEFAULT_ROS_DIAGNOSTIC_TOPIC.into(),
            status_topic: DEFAULT_STATUS_TOPIC.into(),
            maximum_view_age_seconds: 2.0,
        }
    }
}
#[cfg(feature = "ros2")]
mod ffi {
    pub use std::ffi::{c_char, c_void, CStr, CString};
    unsafe extern "C" {
        pub fn horus_remote_create(json: *const c_char, error: *mut *mut c_char) -> *mut c_void;
        pub fn horus_remote_destroy(p: *mut c_void);
        pub fn horus_remote_stop(p: *mut c_void);
        pub fn horus_remote_next_view(
            p: *mut c_void,
            timeout: u32,
            error: *mut *mut c_char,
        ) -> *mut c_char;
        pub fn horus_remote_status(p: *mut c_void, error: *mut *mut c_char) -> *mut c_char;
        pub fn horus_remote_take_keyframe(p: *mut c_void) -> i32;
        pub fn horus_remote_publish(
            p: *mut c_void,
            kind: i32,
            json: *const c_char,
            data: *const u8,
            n: usize,
            error: *mut *mut c_char,
        ) -> i32;
        pub fn horus_native_free(p: *mut c_char);
    }
    pub unsafe fn take(p: *mut c_char) -> String {
        if p.is_null() {
            return "Remote runtime returned no result".into();
        }
        let text = CStr::from_ptr(p).to_string_lossy().into_owned();
        horus_native_free(p);
        text
    }
}
pub struct RemotePublisher {
    #[cfg(feature = "ros2")]
    handle: std::ptr::NonNull<ffi::c_void>,
}
// The runtime serializes ROS publication, queues and lifecycle; no global ROS shutdown.
unsafe impl Send for RemotePublisher {}
unsafe impl Sync for RemotePublisher {}
impl RemotePublisher {
    pub fn new(options: RemotePublisherOptions) -> Result<Self> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = options;
            bail!("Enable ros2 for remote publication");
        }
        #[cfg(feature = "ros2")]
        {
            let text = ffi::CString::new(serde_json::to_vec(&options)?)?;
            let mut error = std::ptr::null_mut();
            let p = unsafe { ffi::horus_remote_create(text.as_ptr(), &mut error) };
            let Some(handle) = std::ptr::NonNull::new(p) else {
                bail!("{}", unsafe { ffi::take(error) });
            };
            Ok(Self { handle })
        }
    }
    pub fn next_view(&self, timeout_ms: u32) -> Result<Option<ViewState>> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = timeout_ms;
            bail!("Enable ros2 for remote publication");
        }
        #[cfg(feature = "ros2")]
        {
            let mut error = std::ptr::null_mut();
            let p = unsafe {
                ffi::horus_remote_next_view(self.handle.as_ptr(), timeout_ms, &mut error)
            };
            if p.is_null() {
                bail!("{}", unsafe { ffi::take(error) });
            }
            let value: Value = serde_json::from_str(&unsafe { ffi::take(p) })?;
            if value.is_null() {
                Ok(None)
            } else {
                Ok(Some(ViewState::from_value(&value)?))
            }
        }
    }
    fn publish(&self, kind: i32, json: &str, data: &[u8]) -> Result<bool> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = (kind, json, data);
            bail!("Enable ros2 for remote publication");
        }
        #[cfg(feature = "ros2")]
        {
            let text = ffi::CString::new(json)?;
            let mut error = std::ptr::null_mut();
            let result = unsafe {
                ffi::horus_remote_publish(
                    self.handle.as_ptr(),
                    kind,
                    text.as_ptr(),
                    data.as_ptr(),
                    data.len(),
                    &mut error,
                )
            };
            if result < 0 {
                bail!("{}", unsafe { ffi::take(error) });
            }
            Ok(result != 0)
        }
    }
    pub fn publish_diagnostic(&self, frame: &StereoFrameRender) -> Result<bool> {
        self.publish(0, &frame.metadata().to_json()?, &frame.diagnostic_packet()?)
    }
    pub fn publish_rgb(&self, frame: &StereoFrameRender) -> Result<bool> {
        self.publish(1, &frame.metadata().to_json()?, frame.video_rgb())
    }
    pub fn publish_h264(&self, frame: &EncodedH264Frame) -> Result<bool> {
        self.publish(2, &frame.metadata_json, &frame.data)
    }
    pub fn take_keyframe_request(&self) -> bool {
        #[cfg(feature = "ros2")]
        unsafe {
            return ffi::horus_remote_take_keyframe(self.handle.as_ptr()) != 0;
        }
        #[cfg(not(feature = "ros2"))]
        {
            false
        }
    }
    pub fn status(&self) -> Result<Value> {
        #[cfg(not(feature = "ros2"))]
        {
            bail!("Enable ros2 for remote publication");
        }
        #[cfg(feature = "ros2")]
        {
            let mut error = std::ptr::null_mut();
            let p = unsafe { ffi::horus_remote_status(self.handle.as_ptr(), &mut error) };
            if p.is_null() {
                bail!("{}", unsafe { ffi::take(error) });
            }
            Ok(serde_json::from_str(&unsafe { ffi::take(p) })?)
        }
    }
    pub fn stop(&self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_remote_stop(self.handle.as_ptr());
        }
    }
}
impl Drop for RemotePublisher {
    fn drop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_remote_destroy(self.handle.as_ptr());
        }
    }
}
