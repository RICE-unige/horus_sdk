use anyhow::{anyhow, bail, Result};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::ffi::{c_char, CStr, CString};

extern "C" {
    fn horus_description_compile(xml: *const c_char, options: *const c_char) -> *mut c_char;
    fn horus_teach_free(value: *mut c_char);
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Artifact {
    pub manifest: Value,
    pub payload: Value,
    pub payload_json: String,
    pub encoded_payload: String,
}

/// Compile resolved URDF XML with native mesh import and fixed-link grouping.
/// Paths/package search roots and pose mode use the Python configuration keys.
pub fn compile_urdf(xml: &str, options: &Value) -> Result<Artifact> {
    compile(Some(xml), options)
}

pub fn compile_file(options: &Value) -> Result<Artifact> {
    compile(None, options)
}

/// Resolves files, or a ROS String topic / robot_description parameter.
pub fn resolve(options: &Value) -> Result<Artifact> {
    if options["urdf_path"].as_str().is_some_and(|s| !s.is_empty()) {
        return compile_file(options);
    }
    #[cfg(feature = "ros2")]
    {
        extern "C" {
            fn horus_native_description_resolve(options: *const c_char) -> *mut c_char;
            fn horus_native_free(value: *mut c_char);
        }
        let input = CString::new(serde_json::to_vec(options)?)?;
        let ptr = unsafe { horus_native_description_resolve(input.as_ptr()) };
        if ptr.is_null() {
            bail!("Description source returned no result");
        }
        let result = unsafe { serde_json::from_slice::<Value>(CStr::from_ptr(ptr).to_bytes()) };
        unsafe { horus_native_free(ptr) };
        let result = result?;
        if result["ok"] != true {
            bail!("{}", result["error"]);
        }
        Ok(serde_json::from_value(result["value"].clone())?)
    }
    #[cfg(not(feature = "ros2"))]
    bail!("Build with ROS 2 for robot description topic/parameter sources")
}

fn compile(xml: Option<&str>, options: &Value) -> Result<Artifact> {
    let xml = xml.map(CString::new).transpose()?;
    let options = CString::new(serde_json::to_vec(options)?)?;
    let output = unsafe {
        horus_description_compile(
            xml.as_ref().map_or(std::ptr::null(), |s| s.as_ptr()),
            options.as_ptr(),
        )
    };
    if output.is_null() {
        bail!("Native model compiler returned no result");
    }
    let result = unsafe { serde_json::from_slice::<Value>(CStr::from_ptr(output).to_bytes()) };
    unsafe { horus_teach_free(output) };
    let result = result?;
    if result["ok"] != true {
        return Err(anyhow!(
            "{}",
            result["error"]
                .as_str()
                .unwrap_or("Model compilation failed")
        ));
    }
    Ok(serde_json::from_value(result["value"].clone())?)
}
