//! Batched ROS TF publication with retained static transforms and no Python process.
use anyhow::{bail, Result};
use serde::Serialize;

#[derive(Clone, Debug)]
pub struct Transform {
    pub parent: String,
    pub child: String,
    pub translation: [f64; 3],
    /// ROS quaternion order: x, y, z, w.
    pub rotation: [f64; 4],
    /// None uses ROS time. Some(0) is valid simulation time, not "now".
    pub stamp_ns: Option<i64>,
}
impl Transform {
    pub fn new(parent: impl Into<String>, child: impl Into<String>) -> Self {
        Self {
            parent: parent.into(),
            child: child.into(),
            translation: [0.; 3],
            rotation: [0., 0., 0., 1.],
            stamp_ns: None,
        }
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct PublisherOptions {
    pub topic: String,
    pub static_topic: String,
    pub use_sim_time: bool,
}
impl Default for PublisherOptions {
    fn default() -> Self {
        Self {
            topic: "/tf".into(),
            static_topic: "/tf_static".into(),
            use_sim_time: false,
        }
    }
}
#[cfg(feature = "ros2")]
mod ffi {
    pub use std::ffi::{c_char, c_void, CStr, CString};
    #[repr(C)]
    pub struct Transform {
        pub parent: *const c_char,
        pub child: *const c_char,
        pub translation: [f64; 3],
        pub rotation: [f64; 4],
        pub stamp_ns: i64,
    }
    extern "C" {
        pub fn horus_native_tf_create(
            options: *const c_char,
            error: *mut *mut c_char,
        ) -> *mut c_void;
        pub fn horus_native_tf_publish(
            handle: *mut c_void,
            transforms: *const Transform,
            count: usize,
            is_static: i32,
            error: *mut *mut c_char,
        ) -> i32;
        pub fn horus_native_tf_destroy(handle: *mut c_void);
        pub fn horus_native_tf_subscribers(
            handle: *mut c_void,
            is_static: i32,
            error: *mut *mut c_char,
        ) -> i64;
        fn horus_native_free(error: *mut c_char);
    }
    pub unsafe fn take(error: *mut c_char) -> String {
        if error.is_null() {
            return "TF publisher failed without error details".into();
        }
        let text = CStr::from_ptr(error).to_string_lossy().into_owned();
        horus_native_free(error);
        text
    }
}
/// Private ROS context; dropping this publisher does not stop other ROS nodes.
pub struct Publisher {
    #[cfg(feature = "ros2")]
    handle: std::ptr::NonNull<ffi::c_void>,
}
impl Publisher {
    /// Matched DDS subscriptions, not proof of message delivery.
    pub fn subscriber_count(&self, is_static: bool) -> Result<usize> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = is_static;
            bail!("Enable ros2 for TF publication");
        }
        #[cfg(feature = "ros2")]
        {
            let mut error = std::ptr::null_mut();
            let count = unsafe {
                ffi::horus_native_tf_subscribers(self.handle.as_ptr(), is_static.into(), &mut error)
            };
            if count < 0 {
                bail!("{}", unsafe { ffi::take(error) });
            }
            Ok(count as usize)
        }
    }
    pub fn new(options: PublisherOptions) -> Result<Self> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = options;
            bail!("Enable ros2 for TF publication");
        }
        #[cfg(feature = "ros2")]
        {
            let text = ffi::CString::new(serde_json::to_vec(&options)?)?;
            let mut error = std::ptr::null_mut();
            let raw = unsafe { ffi::horus_native_tf_create(text.as_ptr(), &mut error) };
            let Some(handle) = std::ptr::NonNull::new(raw) else {
                bail!("{}", unsafe { ffi::take(error) });
            };
            Ok(Self { handle })
        }
    }
    pub fn publish(&mut self, transforms: &[Transform], is_static: bool) -> Result<()> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = (transforms, is_static);
            bail!("Enable ros2 for TF publication");
        }
        #[cfg(feature = "ros2")]
        {
            anyhow::ensure!(transforms.len() <= 4096, "TF batch exceeds 4096 transforms");
            let mut names = Vec::with_capacity(transforms.len());
            for t in transforms {
                anyhow::ensure!(
                    t.stamp_ns.map_or(true, |v| v >= 0),
                    "TF stamp must be nonnegative"
                );
                names.push((
                    ffi::CString::new(t.parent.as_str())?,
                    ffi::CString::new(t.child.as_str())?,
                ));
            }
            let batch: Vec<_> = transforms
                .iter()
                .zip(&names)
                .map(|(t, (parent, child))| ffi::Transform {
                    parent: parent.as_ptr(),
                    child: child.as_ptr(),
                    translation: t.translation,
                    rotation: t.rotation,
                    stamp_ns: t.stamp_ns.unwrap_or(-1),
                })
                .collect();
            let mut error = std::ptr::null_mut();
            if unsafe {
                ffi::horus_native_tf_publish(
                    self.handle.as_ptr(),
                    batch.as_ptr(),
                    batch.len(),
                    is_static.into(),
                    &mut error,
                )
            } == 0
            {
                bail!("{}", unsafe { ffi::take(error) });
            }
            Ok(())
        }
    }
}
impl Drop for Publisher {
    fn drop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_native_tf_destroy(self.handle.as_ptr());
        }
    }
}
