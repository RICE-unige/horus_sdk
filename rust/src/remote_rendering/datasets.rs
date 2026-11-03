//! Real-data resolution and bounded, full-resolution point streaming.
use super::{encoder::failure, scene::SceneAsset};
use anyhow::{anyhow, ensure, Result};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::{
    ffi::{c_char, c_void, CStr, CString},
    path::{Path, PathBuf},
    ptr::NonNull,
};
unsafe extern "C" {
    fn horus_dataset_request(request: *const c_char, error: *mut *mut c_char) -> *mut c_char;
    fn horus_dataset_cow(path: *const c_char, scale: f64, error: *mut *mut c_char) -> *mut c_void;
    fn horus_dataset_stream(
        descriptor: *const c_char,
        size: usize,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_dataset_next(
        stream: *mut c_void,
        done: *mut i32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_dataset_write_lod(
        stream: *mut c_void,
        path: *const c_char,
        voxel: f32,
        maximum: usize,
        error: *mut *mut c_char,
    ) -> u64;
    fn horus_dataset_destroy(stream: *mut c_void);
    fn horus_teach_free(text: *mut c_char);
}
fn path(p: &Path) -> Result<&str> {
    p.to_str()
        .ok_or_else(|| anyhow!("Dataset path is not UTF-8"))
}
fn request(j: Value) -> Result<Value> {
    let text = CString::new(j.to_string())?;
    let mut error = std::ptr::null_mut();
    let p = unsafe { horus_dataset_request(text.as_ptr(), &mut error) };
    ensure!(!p.is_null(), "{}", failure(error));
    let text = unsafe { CStr::from_ptr(p) }.to_string_lossy().into_owned();
    unsafe { horus_teach_free(p) };
    Ok(serde_json::from_str(&text)?)
}
pub fn scene_catalog(cache_root: impl AsRef<Path>) -> Result<Value> {
    request(json!({"op":"catalog","cache_root":path(cache_root.as_ref())?}))
}
pub fn resolve_scene_asset(
    id: &str,
    source: impl AsRef<Path>,
    cache_root: impl AsRef<Path>,
) -> Result<PathBuf> {
    Ok(PathBuf::from(serde_json::from_value::<String>(request(
        json!({"op":"resolve","scene_id":id,"path":path(source.as_ref())?,"cache_root":path(cache_root.as_ref())?}),
    )?)?))
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlignedScan {
    pub filename: String,
    pub alignment: [[f32; 4]; 4],
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PreparedPointScene {
    pub scene_id: String,
    pub root: PathBuf,
    pub scans: Vec<AlignedScan>,
    pub point_count: u64,
    pub center: [f32; 3],
    pub source_bounds_min: [f64; 3],
    pub source_bounds_max: [f64; 3],
    pub world_scale: f64,
    pub depth_near_m: f64,
    pub depth_far_m: f64,
    pub initial_position: [f64; 3],
    pub initial_pitch_degrees: f64,
    pub fingerprint: String,
}
pub fn prepare_eth3d_scene(
    id: &str,
    source: impl AsRef<Path>,
    dataset_scale: f64,
    cache_root: impl AsRef<Path>,
) -> Result<PreparedPointScene> {
    Ok(serde_json::from_value(request(
        json!({"op":"eth3d","scene_id":id,"path":path(source.as_ref())?,"dataset_scale":dataset_scale,"cache_root":path(cache_root.as_ref())?}),
    )?)?)
}
pub fn prepare_cow_lady_scene(source: impl AsRef<Path>, world_scale: f64) -> Result<SceneAsset> {
    let p = CString::new(path(source.as_ref())?)?;
    let mut e = std::ptr::null_mut();
    let h = unsafe { horus_dataset_cow(p.as_ptr(), world_scale, &mut e) };
    Ok(SceneAsset {
        handle: NonNull::new(h).ok_or_else(|| failure(e))?,
    })
}
pub struct PointChunkStream {
    pub(super) handle: NonNull<c_void>,
    pub(super) done: bool,
}
// Owned stream can move threads; next/write require exclusive mutable access.
unsafe impl Send for PointChunkStream {}
impl PointChunkStream {
    fn new(j: Value, chunk_size: usize) -> Result<Self> {
        let s = CString::new(j.to_string())?;
        let mut e = std::ptr::null_mut();
        let h = unsafe { horus_dataset_stream(s.as_ptr(), chunk_size, &mut e) };
        Ok(Self {
            handle: NonNull::new(h).ok_or_else(|| failure(e))?,
            done: false,
        })
    }
    pub fn open(source: impl AsRef<Path>, chunk_size: usize) -> Result<Self> {
        Self::new(json!({"file":path(source.as_ref())?}), chunk_size)
    }
    pub fn from_scene(scene: &PreparedPointScene, chunk_size: usize) -> Result<Self> {
        Self::new(serde_json::to_value(scene)?, chunk_size)
    }
    /// Explicit opt-in; never used by normal full-resolution scene streaming.
    pub fn write_lod(
        &mut self,
        output: impl AsRef<Path>,
        voxel_size_m: f32,
        maximum_voxels: usize,
    ) -> Result<u64> {
        let p = CString::new(path(output.as_ref())?)?;
        let mut e = std::ptr::null_mut();
        let n = unsafe {
            horus_dataset_write_lod(
                self.handle.as_ptr(),
                p.as_ptr(),
                voxel_size_m,
                maximum_voxels,
                &mut e,
            )
        };
        self.done = true;
        ensure!(n > 0, "{}", failure(e));
        Ok(n)
    }
}
impl Iterator for PointChunkStream {
    type Item = Result<SceneAsset>;
    fn next(&mut self) -> Option<Self::Item> {
        if self.done {
            return None;
        }
        let mut done = 0;
        let mut e = std::ptr::null_mut();
        let p = unsafe { horus_dataset_next(self.handle.as_ptr(), &mut done, &mut e) };
        if done != 0 {
            self.done = true;
            return None;
        }
        Some(match NonNull::new(p) {
            Some(handle) => Ok(SceneAsset { handle }),
            None => {
                self.done = true;
                Err(failure(e))
            }
        })
    }
}
impl Drop for PointChunkStream {
    fn drop(&mut self) {
        unsafe { horus_dataset_destroy(self.handle.as_ptr()) }
    }
}
