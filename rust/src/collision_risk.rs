//! Advisory collision-risk visualization, not a robot safety controller.
use anyhow::{ensure, Result};
use serde::Serialize;
#[derive(Clone, Debug, Default)]
pub struct CollisionObservation {
    pub min_distance_m: Option<f64>,
    pub direction: [f64; 3],
}
pub fn risk_from_distance(distance: Option<f64>, threshold: f64) -> Result<f64> {
    ensure!(threshold.is_finite(), "Threshold must be finite");
    Ok(distance.filter(|d| d.is_finite()).map_or(0., |d| {
        ((threshold.max(1e-6) - d) / threshold.max(1e-6)).clamp(0., 1.)
    }))
}
fn normalize(x: f64, y: f64) -> [f64; 3] {
    let length = x.hypot(y);
    if length > 1e-6 {
        [x / length, y / length, 0.]
    } else {
        [0.; 3]
    }
}
pub fn analyze_laser_scan(
    ranges: &[f64],
    angle_min: f64,
    increment: f64,
    minimum: f64,
    maximum: f64,
) -> Result<CollisionObservation> {
    ensure!(
        [angle_min, increment, minimum, maximum]
            .iter()
            .all(|v| v.is_finite()),
        "Laser parameters must be finite"
    );
    let mut out = CollisionObservation::default();
    let mut angle = angle_min;
    for &d in ranges {
        if d.is_finite()
            && d >= minimum
            && d <= maximum
            && out.min_distance_m.map_or(true, |v| d < v)
        {
            out.min_distance_m = Some(d);
            out.direction = normalize(angle.cos(), angle.sin());
        }
        angle += increment;
    }
    Ok(out)
}
pub fn analyze_point_cloud(points: &[[f64; 3]], z_window: f64) -> Result<CollisionObservation> {
    ensure!(z_window.is_finite(), "Z window must be finite");
    let mut out = CollisionObservation::default();
    for &[x, y, z] in points {
        if ![x, y, z].iter().all(|v| v.is_finite()) || z.abs() > z_window.max(0.) {
            continue;
        }
        let d = x.hypot(y);
        if out.min_distance_m.map_or(true, |v| d < v) {
            out.min_distance_m = Some(d);
            out.direction = normalize(x, y);
        }
    }
    Ok(out)
}
#[derive(Clone, Debug, Serialize)]
pub struct CollisionRiskOptions {
    pub robot_name: String,
    pub source_type: String,
    pub source_topic: String,
    pub output_topic: String,
    pub threshold_m: f64,
    pub publish_hz: f64,
    pub smoothing_alpha: f64,
    pub z_window_m: f64,
}
impl Default for CollisionRiskOptions {
    fn default() -> Self {
        Self {
            robot_name: String::new(),
            source_type: "laser_scan".into(),
            source_topic: String::new(),
            output_topic: String::new(),
            threshold_m: 1.2,
            publish_hz: 10.,
            smoothing_alpha: 0.35,
            z_window_m: 0.35,
        }
    }
}
#[cfg(feature = "ros2")]
mod ffi {
    pub use std::ffi::{c_char, c_void, CStr, CString};
    extern "C" {
        pub fn horus_collision_create(
            options: *const c_char,
            error: *mut *mut c_char,
        ) -> *mut c_void;
        pub fn horus_collision_stop(handle: *mut c_void);
        pub fn horus_collision_destroy(handle: *mut c_void);
        pub fn horus_native_free(error: *mut c_char);
    }
}
/// Background ROS subscription and risk publisher. Drop stops only its own context.
pub struct CollisionRiskAnalyzer {
    #[cfg(feature = "ros2")]
    handle: std::ptr::NonNull<ffi::c_void>,
}
impl CollisionRiskAnalyzer {
    pub fn new(options: CollisionRiskOptions) -> Result<Self> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = options;
            anyhow::bail!("Enable ros2 for CollisionRiskAnalyzer");
        }
        #[cfg(feature = "ros2")]
        {
            let text = ffi::CString::new(serde_json::to_vec(&options)?)?;
            let mut error = std::ptr::null_mut();
            let raw = unsafe { ffi::horus_collision_create(text.as_ptr(), &mut error) };
            let Some(handle) = std::ptr::NonNull::new(raw) else {
                let text = if error.is_null() {
                    "Collision analyzer creation failed".into()
                } else {
                    unsafe { ffi::CStr::from_ptr(error).to_string_lossy().into_owned() }
                };
                unsafe { ffi::horus_native_free(error) };
                anyhow::bail!(text);
            };
            Ok(Self { handle })
        }
    }
    pub fn stop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_collision_stop(self.handle.as_ptr())
        }
    }
}
impl Drop for CollisionRiskAnalyzer {
    fn drop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            ffi::horus_collision_destroy(self.handle.as_ptr())
        }
    }
}
