use anyhow::{bail, Result};
use serde::Serialize;
use serde_json::Value;

#[derive(Clone, Debug, Serialize)]
pub struct MeshSourceAdapterOptions {
    pub input_topic: String,
    pub input_type: String,
    pub output_topic: String,
    pub frame_id: String,
    pub max_pages: u32,
    pub max_payload_bytes: usize,
    pub max_cache_bytes: usize,
    pub triangles_per_page: usize,
    pub max_input_points: usize,
    pub blocks_per_axis: u32,
    pub pages_per_second: f64,
    pub bytes_per_second: f64,
    pub use_sim_time: bool,
    pub color_mode: String,
    pub fallback_color: [f32; 4],
}
impl MeshSourceAdapterOptions {
    pub fn new(input_topic: impl Into<String>, output_topic: impl Into<String>) -> Self {
        Self {
            input_topic: input_topic.into(),
            input_type: "Marker".into(),
            output_topic: output_topic.into(),
            frame_id: "map".into(),
            max_pages: 1024,
            max_payload_bytes: 1024 * 1024,
            max_cache_bytes: 256 * 1024 * 1024,
            triangles_per_page: 4096,
            max_input_points: 3000000,
            blocks_per_axis: 2,
            pages_per_second: 60.,
            bytes_per_second: 4. * 1024. * 1024.,
            use_sim_time: false,
            color_mode: "auto".into(),
            fallback_color: [0.70, 0.72, 0.78, 1.],
        }
    }
}
#[cfg(feature = "ros2")]
mod ffi {
    pub use std::ffi::{c_char, c_void, CStr, CString};
    extern "C" {
        pub fn horus_native_mesh_source_create(
            options: *const c_char,
            error: *mut *mut c_char,
        ) -> *mut c_void;
        pub fn horus_native_mesh_source_status(handle: *mut c_void) -> *mut c_char;
        pub fn horus_native_mesh_source_destroy(handle: *mut c_void);
        pub fn horus_native_free(text: *mut c_char);
    }
    pub unsafe fn take(text: *mut c_char) -> anyhow::Result<String> {
        if text.is_null() {
            anyhow::bail!("Native mesh adapter returned no result");
        }
        let value = CStr::from_ptr(text).to_str().map(str::to_owned);
        horus_native_free(text);
        Ok(value?)
    }
}
/// In-process ROS source adapter. Owns its executor, TF buffer, retained map and
/// paced publisher. No Python SDK wrapper and no change to the caller's ROS node.
pub struct MeshSourceAdapter {
    #[cfg(feature = "ros2")]
    handle: std::ptr::NonNull<ffi::c_void>,
}
// Private runtime serializes callbacks/status and joins its worker on destruction.
unsafe impl Send for MeshSourceAdapter {}
unsafe impl Sync for MeshSourceAdapter {}
impl MeshSourceAdapter {
    pub fn new(options: MeshSourceAdapterOptions) -> Result<Self> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = options;
            bail!("Enable ros2 for mesh source adaptation");
        }
        #[cfg(feature = "ros2")]
        {
            let input = ffi::CString::new(serde_json::to_vec(&options)?)?;
            let mut error = std::ptr::null_mut();
            let raw = unsafe { ffi::horus_native_mesh_source_create(input.as_ptr(), &mut error) };
            let Some(handle) = std::ptr::NonNull::new(raw) else {
                bail!("{}", unsafe { ffi::take(error)? });
            };
            Ok(Self { handle })
        }
    }
    pub fn status(&self) -> Result<Value> {
        #[cfg(not(feature = "ros2"))]
        {
            bail!("Enable ros2 for mesh source adaptation");
        }
        #[cfg(feature = "ros2")]
        {
            Ok(serde_json::from_str(&unsafe {
                ffi::take(ffi::horus_native_mesh_source_status(self.handle.as_ptr()))?
            })?)
        }
    }
}
impl Drop for MeshSourceAdapter {
    fn drop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_native_mesh_source_destroy(self.handle.as_ptr());
        }
    }
}
