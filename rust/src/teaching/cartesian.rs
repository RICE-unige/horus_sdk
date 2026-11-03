use super::*;
extern "C" {
    fn horus_teach_cartesian_create(
        cap: *const c_char,
        limits: *const c_char,
        hands: *const c_char,
        offsets: *const c_char,
        context: *mut c_void,
        callback: AdapterCall,
        release: AdapterFree,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_teach_cartesian_call(
        adapter: *mut c_void,
        operation: *const c_char,
        value: *const c_char,
    ) -> *mut c_char;
    fn horus_teach_cartesian_destroy(adapter: *mut c_void);
}
#[derive(Clone, Debug, Serialize)]
pub struct CartesianLimits {
    pub minimum: [f64; 3],
    pub maximum: [f64; 3],
    pub max_speed_m_s: f64,
    pub max_rotation_rad_s: f64,
    pub feedback_timeout_s: f64,
    pub translation_scale: f64,
    pub floor_to_command_rotation: [f64; 4],
}
impl CartesianLimits {
    pub fn new(
        minimum: [f64; 3],
        maximum: [f64; 3],
        max_speed_m_s: f64,
        max_rotation_rad_s: f64,
    ) -> Self {
        Self {
            minimum,
            maximum,
            max_speed_m_s,
            max_rotation_rad_s,
            feedback_timeout_s: 0.2,
            translation_scale: 1.0,
            floor_to_command_rotation: [0.0, 0.0, 0.0, 1.0],
        }
    }
}
/// The driver supplies fresh measured poses and authority-scoped target/hold
/// receipts. It owns IK, balance, collision constraints and its local watchdog.
pub trait CartesianController: Send {
    fn feedback(&mut self) -> Result<Value>;
    fn publish(&mut self, targets: &Value) -> Result<Value>;
    fn hold(&mut self, context: &Value) -> Result<bool>;
}
struct ControllerBox(Box<dyn CartesianController>);
unsafe extern "C" fn call(
    ctx: *mut c_void,
    op: *const c_char,
    value: *const c_char,
) -> *const c_char {
    let result = catch_unwind(AssertUnwindSafe(|| -> Result<Value> {
        let controller = &mut (*(ctx as *mut ControllerBox)).0;
        let value = serde_json::from_slice(CStr::from_ptr(value).to_bytes())?;
        match CStr::from_ptr(op).to_str()? {
            "feedback" => controller.feedback(),
            "publish" => controller.publish(&value),
            "hold" => Ok(json!(controller.hold(&value)?)),
            _ => bail!("Unknown Cartesian operation"),
        }
    }));
    let output = match result {
        Ok(Ok(v)) => json!({"ok":true,"value":v}),
        Ok(Err(e)) => json!({"ok":false,"error":e.to_string()}),
        Err(_) => json!({"ok":false,"error":"Cartesian controller panicked"}),
    };
    cjson(&output)
        .map(CString::into_raw)
        .unwrap_or(std::ptr::null_mut())
}
pub struct CartesianAdapter {
    handle: NonNull<c_void>,
    capabilities: AdapterCapabilities,
    _controller: Box<ControllerBox>,
    _not_sync: PhantomData<Cell<()>>,
}
unsafe impl Send for CartesianAdapter {}
impl CartesianAdapter {
    pub fn new(
        capabilities: AdapterCapabilities,
        limits: CartesianLimits,
        hands: &[&str],
        offsets: Value,
        controller: Box<dyn CartesianController>,
    ) -> Result<Self> {
        let cap = cjson(&serde_json::to_value(&capabilities)?)?;
        let limits = cjson(&serde_json::to_value(limits)?)?;
        let hands = cjson(&json!(hands))?;
        let offsets = cjson(&offsets)?;
        let mut controller = Box::new(ControllerBox(controller));
        let mut error = std::ptr::null_mut();
        let ptr = unsafe {
            horus_teach_cartesian_create(
                cap.as_ptr(),
                limits.as_ptr(),
                hands.as_ptr(),
                offsets.as_ptr(),
                &mut *controller as *mut ControllerBox as *mut c_void,
                call,
                adapter_free,
                &mut error,
            )
        };
        Ok(Self {
            handle: unsafe { created(ptr, error)? },
            capabilities,
            _controller: controller,
            _not_sync: PhantomData,
        })
    }
    fn call(&mut self, op: &str, v: &Value) -> Result<Value> {
        let op = CString::new(op)?;
        let value = cjson(v)?;
        unsafe {
            response(horus_teach_cartesian_call(
                self.handle.as_ptr(),
                op.as_ptr(),
                value.as_ptr(),
            ))
        }
    }
}
impl TeachAdapter for CartesianAdapter {
    fn capabilities(&self) -> AdapterCapabilities {
        self.capabilities.clone()
    }
    fn ready(&mut self) -> Result<(bool, String)> {
        let v = self.call("ready", &Value::Null)?;
        Ok((
            v["ready"] == true,
            v["reason"].as_str().unwrap_or("").into(),
        ))
    }
    fn anchor(&mut self, sample: &Value) -> Result<()> {
        self.call("anchor", sample)?;
        Ok(())
    }
    fn apply(&mut self, sample: &Value) -> Result<Value> {
        self.call("apply", sample)
    }
    fn hold(&mut self, context: &Value) -> Result<bool> {
        Ok(self.call("hold", context)? == true)
    }
}
impl Drop for CartesianAdapter {
    fn drop(&mut self) {
        unsafe {
            horus_teach_cartesian_destroy(self.handle.as_ptr());
        }
    }
}
