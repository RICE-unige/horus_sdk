//! Owned native ROS 2 context. Only a small C ABI crosses the middleware boundary.
//! JSON construction remains Rust/serde; no Python or ROS command-line subprocesses.
use serde_json::{json, Value};

#[cfg(feature = "ros2")]
mod enabled {
    use super::*;
    use std::ffi::{c_char, c_void, CStr, CString};
    use std::ptr::NonNull;

    unsafe extern "C" {
        fn horus_native_create_with_options(
            options: *const c_char,
            error: *mut *mut c_char,
        ) -> *mut c_void;
        fn horus_native_register(
            handle: *mut c_void,
            payload: *const c_char,
            timeout: f64,
            keep_alive: i32,
            show_status: i32,
        ) -> *mut c_char;
        fn horus_native_unregister(
            handle: *mut c_void,
            robot: *const c_char,
            timeout: f64,
        ) -> *mut c_char;
        fn horus_native_status(handle: *mut c_void) -> *mut c_char;
        fn horus_native_stop(handle: *mut c_void);
        fn horus_native_destroy(handle: *mut c_void);
        fn horus_native_free(text: *mut c_char);
    }

    pub struct NativeRegistry(NonNull<c_void>);
    // The C++ runtime synchronizes all operations; destruction requires exclusive
    // ownership. Rust references/Arc keep it alive during concurrent stop/status.
    unsafe impl Send for NativeRegistry {}
    unsafe impl Sync for NativeRegistry {}

    fn take_text(pointer: *mut c_char) -> Result<String, String> {
        if pointer.is_null() {
            return Err("Native runtime could not allocate a result".into());
        }
        // The ABI returns NUL-terminated owned UTF-8; free even on invalid UTF-8.
        unsafe {
            let value = CStr::from_ptr(pointer)
                .to_str()
                .map(str::to_owned)
                .map_err(|e| e.to_string());
            horus_native_free(pointer);
            value
        }
    }
    fn take_json(pointer: *mut c_char) -> Value {
        take_text(pointer)
            .and_then(|text| serde_json::from_str(&text).map_err(|e| e.to_string()))
            .unwrap_or_else(|error| json!({"success": false, "error": error}))
    }

    impl NativeRegistry {
        pub fn new(options: &crate::bridge::RegistryOptions) -> Result<Self, String> {
            let mut error = std::ptr::null_mut();
            let options =
                CString::new(serde_json::to_string(options).map_err(|e| e.to_string())?).unwrap();
            let handle = unsafe { horus_native_create_with_options(options.as_ptr(), &mut error) };
            NonNull::new(handle).map(Self).ok_or_else(|| {
                if error.is_null() {
                    "Failed to create native ROS context".into()
                } else {
                    take_text(error).unwrap_or_else(|e| e)
                }
            })
        }
        pub fn register(
            &self,
            payloads: &Value,
            timeout: f64,
            keep_alive: bool,
            show_status: bool,
        ) -> Value {
            let encoded = CString::new(payloads.to_string()).expect("JSON escapes embedded NUL");
            take_json(unsafe {
                horus_native_register(
                    self.0.as_ptr(),
                    encoded.as_ptr(),
                    timeout,
                    i32::from(keep_alive),
                    i32::from(show_status),
                )
            })
        }
        pub fn unregister(&self, name: &str, timeout: f64) -> Value {
            let Ok(name) = CString::new(name) else {
                return json!({"success":false,"error":"Robot name contains NUL"});
            };
            take_json(unsafe { horus_native_unregister(self.0.as_ptr(), name.as_ptr(), timeout) })
        }
        pub fn status(&self) -> Value {
            take_json(unsafe { horus_native_status(self.0.as_ptr()) })
        }
        pub fn stop(&self) {
            unsafe { horus_native_stop(self.0.as_ptr()) }
        }
    }
    impl Drop for NativeRegistry {
        fn drop(&mut self) {
            unsafe { horus_native_destroy(self.0.as_ptr()) }
        }
    }
}
#[cfg(feature = "ros2")]
pub use enabled::NativeRegistry;

#[cfg(not(feature = "ros2"))]
pub struct NativeRegistry;
#[cfg(not(feature = "ros2"))]
impl NativeRegistry {
    pub fn new(_: &crate::bridge::RegistryOptions) -> Result<Self, String> {
        Err("Build the Rust SDK with --features ros2 for live registration".into())
    }
    pub fn register(&self, _: &Value, _: f64, _: bool, _: bool) -> Value {
        unreachable!()
    }
    pub fn unregister(&self, _: &str, _: f64) -> Value {
        unreachable!()
    }
    pub fn status(&self) -> Value {
        json!({"app_connected":false,"registration_subscribers":0})
    }
    pub fn stop(&self) {}
}
