use super::*;
#[cfg(feature = "ros2")]
extern "C" {
    fn horus_teach_ros_create(
        session: *mut c_void,
        config: *const c_char,
        robot: *const c_char,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_teach_ros_status(runtime: *mut c_void) -> *mut c_char;
    fn horus_teach_ros_destroy(runtime: *mut c_void);
}

/// Explicitly started ROS recorder/control worker. Drop stops and joins its
/// private executor before releasing the session, adapter or recorder.
pub struct TeachRuntime {
    #[cfg(feature = "ros2")]
    handle: NonNull<c_void>,
    session: TeachSession,
}
unsafe impl Send for TeachRuntime {}
impl TeachRuntime {
    pub fn new(
        config: &TeachConfig,
        robot: &str,
        adapter: Option<Box<dyn TeachAdapter>>,
    ) -> Result<Self> {
        #[cfg(not(feature = "ros2"))]
        {
            let _ = (config, robot, adapter);
            bail!("TeachRuntime requires the ros2 feature");
        }
        #[cfg(feature = "ros2")]
        {
            let session = TeachSession::new(config, robot, adapter)?;
            let config = cjson(config.to_value())?;
            let robot = CString::new(robot)?;
            let mut error = std::ptr::null_mut();
            let ptr = unsafe {
                horus_teach_ros_create(
                    session.handle.as_ptr(),
                    config.as_ptr(),
                    robot.as_ptr(),
                    &mut error,
                )
            };
            Ok(Self {
                handle: unsafe { created(ptr, error)? },
                session,
            })
        }
    }
    pub fn status(&self) -> Result<Value> {
        #[cfg(feature = "ros2")]
        {
            unsafe { response(horus_teach_ros_status(self.handle.as_ptr())) }
        }
        #[cfg(not(feature = "ros2"))]
        {
            bail!("ROS support is disabled")
        }
    }
    // Native session serialization also protects Rust adapter callbacks from
    // concurrent ROS/control-worker entry. The reference cannot escape Drop.
    pub fn session(&mut self) -> &mut TeachSession {
        &mut self.session
    }
}
impl Drop for TeachRuntime {
    fn drop(&mut self) {
        #[cfg(feature = "ros2")]
        unsafe {
            horus_teach_ros_destroy(self.handle.as_ptr());
        }
    }
}
