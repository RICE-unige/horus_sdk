//! Transactional non-ROS Marker/nvblox conversion using borrowed binary FFI inputs.
use super::{decode_live_header, BlockGeometry, LiveMeshOptions, MeshPageUpdate};
use anyhow::{anyhow, ensure, Result};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::ffi::{c_char, c_void, CStr, CString};
use std::ptr::NonNull;
use std::sync::Arc;

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct MeshPose {
    pub translation: [f64; 3],
    pub rotation: [f64; 4],
}
impl Default for MeshPose {
    fn default() -> Self {
        Self {
            translation: [0.; 3],
            rotation: [0., 0., 0., 1.],
        }
    }
}
#[derive(Clone, Debug)]
pub struct TriangleMarker {
    pub name_space: String,
    pub frame: String,
    pub id: i32,
    pub action: i32,
    pub marker_type: i32,
    pub stamp_ns: i64,
    pub frame_locked: bool,
    pub lifetime_seconds: f64,
    pub pose: MeshPose,
    pub scale: [f64; 3],
    pub color: [u8; 4],
    pub points: Vec<[f64; 3]>,
    pub colors: Vec<[u8; 4]>,
}
impl Default for TriangleMarker {
    fn default() -> Self {
        Self {
            name_space: String::new(),
            frame: "map".into(),
            id: 0,
            action: 0,
            marker_type: 11,
            stamp_ns: 0,
            frame_locked: false,
            lifetime_seconds: 0.,
            pose: MeshPose::default(),
            scale: [1.; 3],
            color: [255; 4],
            points: Vec::new(),
            colors: Vec::new(),
        }
    }
}
pub struct NvbloxBlockUpdate<'a> {
    pub index: [i32; 3],
    pub geometry: Option<&'a BlockGeometry>,
}
#[derive(Clone, Debug)]
pub struct MeshSourceOptions {
    pub map: LiveMeshOptions,
    pub triangles_per_page: usize,
    pub max_input_points: usize,
    pub blocks_per_axis: u32,
}
impl Default for MeshSourceOptions {
    fn default() -> Self {
        Self {
            map: LiveMeshOptions::default(),
            triangles_per_page: 4096,
            max_input_points: 3_000_000,
            blocks_per_axis: 2,
        }
    }
}
#[repr(C)]
struct MarkerView {
    name_space: *const c_char,
    frame: *const c_char,
    id: i32,
    action: i32,
    marker_type: i32,
    stamp_ns: i64,
    frame_locked: i32,
    lifetime_seconds: f64,
    pose: MeshPose,
    transform: MeshPose,
    scale: [f64; 3],
    color: [u8; 4],
    points: *const f64,
    point_count: usize,
    colors: *const u8,
    color_count: usize,
}
#[repr(C)]
struct BlockView {
    index: [i32; 3],
    deleted: i32,
    vertices: *const f32,
    vertex_count: usize,
    triangles: *const u32,
    index_count: usize,
    colors: *const u8,
    color_count: usize,
}
unsafe extern "C" {
    fn horus_mesh_source_create(options: *const c_char, error: *mut *mut c_char) -> *mut c_void;
    fn horus_mesh_source_destroy(handle: *mut c_void);
    fn horus_mesh_source_markers(
        handle: *mut c_void,
        markers: *const MarkerView,
        count: usize,
        now: f64,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_mesh_source_blocks(
        handle: *mut c_void,
        blocks: *const BlockView,
        count: usize,
        clear: i32,
        frame: *const c_char,
        stamp: i64,
        transform: MeshPose,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_mesh_source_pages(
        handle: *mut c_void,
        operation: *const c_char,
        request: *const c_char,
        now: f64,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_mesh_source_status(handle: *mut c_void, error: *mut *mut c_char) -> *mut c_char;
    fn horus_mesh_batch_size(batch: *mut c_void) -> usize;
    fn horus_mesh_batch_page(batch: *mut c_void, index: usize, bytes: *mut usize) -> *const u8;
    fn horus_mesh_batch_destroy(batch: *mut c_void);
    fn horus_teach_free(text: *mut c_char);
}
fn take(text: *mut c_char) -> Result<String> {
    ensure!(!text.is_null(), "Native source returned no result");
    let result = unsafe { CStr::from_ptr(text).to_str().map(str::to_owned) };
    unsafe { horus_teach_free(text) };
    Ok(result?)
}
fn failure(text: *mut c_char) -> anyhow::Error {
    match take(text) {
        Ok(s) => anyhow!(s),
        Err(e) => e,
    }
}
struct Batch(NonNull<c_void>);
impl Drop for Batch {
    fn drop(&mut self) {
        unsafe {
            horus_mesh_batch_destroy(self.0.as_ptr());
        }
    }
}
fn pages(raw: *mut c_void, error: *mut c_char) -> Result<Vec<Arc<MeshPageUpdate>>> {
    let batch = Batch(NonNull::new(raw).ok_or_else(|| failure(error))?);
    let count = unsafe { horus_mesh_batch_size(batch.0.as_ptr()) };
    let mut result = Vec::with_capacity(count);
    for i in 0..count {
        let mut length = 0;
        let ptr = unsafe { horus_mesh_batch_page(batch.0.as_ptr(), i, &mut length) };
        ensure!(!ptr.is_null(), "Missing native mesh page");
        let bytes = unsafe { std::slice::from_raw_parts(ptr, length) };
        let h = decode_live_header(bytes).map_err(anyhow::Error::msg)?;
        result.push(Arc::new(MeshPageUpdate {
            chunk_id: h.chunk_id,
            epoch: h.epoch,
            revision: h.revision,
            operation: h.operation,
            payload: Arc::from(bytes),
            vertex_count: h.vertex_count as usize,
            triangle_count: h.index_count as usize / 3,
        }));
    }
    Ok(result)
}
/// Retains full source geometry. Geometry crosses FFI as borrowed arrays, not JSON.
/// Each returned encoded page is copied once into a Rust-owned immutable Arc.
pub struct MeshSourceState {
    handle: NonNull<c_void>,
    frame: String,
}
unsafe impl Send for MeshSourceState {}
impl MeshSourceState {
    pub fn new(o: MeshSourceOptions) -> Result<Self> {
        let value=CString::new(json!({"frame_id":o.map.frame_id,"max_pages":o.map.max_pages,"max_payload_bytes":o.map.max_payload_bytes,
            "max_cache_bytes":o.map.max_cache_bytes,"initial_epoch":o.map.initial_epoch,"triangles_per_page":o.triangles_per_page,
            "max_input_points":o.max_input_points,"blocks_per_axis":o.blocks_per_axis}).to_string())?;
        let mut error = std::ptr::null_mut();
        let raw = unsafe { horus_mesh_source_create(value.as_ptr(), &mut error) };
        Ok(Self {
            handle: NonNull::new(raw).ok_or_else(|| failure(error))?,
            frame: o.map.frame_id,
        })
    }
    pub fn apply_markers<F>(
        &mut self,
        markers: &[TriangleMarker],
        now: f64,
        mut lookup: F,
    ) -> Result<Vec<Arc<MeshPageUpdate>>>
    where
        F: FnMut(&str, i64) -> Result<MeshPose>,
    {
        ensure!(markers.len() <= 1024, "Marker batch exceeds 1024");
        let strings: Vec<_> = markers
            .iter()
            .map(|m| {
                Ok((
                    CString::new(m.name_space.as_str())?,
                    CString::new(m.frame.as_str())?,
                ))
            })
            .collect::<Result<_>>()?;
        let mut transforms = HashMap::new();
        let mut views = Vec::with_capacity(markers.len());
        for (m, (namespace, frame)) in markers.iter().zip(&strings) {
            let mut transform = MeshPose::default();
            if m.action == 0 && !m.points.is_empty() && m.frame != self.frame {
                let key = (m.frame.as_str(), m.stamp_ns);
                if let Some(p) = transforms.get(&key) {
                    transform = *p;
                } else {
                    transform = lookup(key.0, key.1)?;
                    transforms.insert(key, transform);
                }
            }
            views.push(MarkerView {
                name_space: namespace.as_ptr(),
                frame: frame.as_ptr(),
                id: m.id,
                action: m.action,
                marker_type: m.marker_type,
                stamp_ns: m.stamp_ns,
                frame_locked: i32::from(m.frame_locked),
                lifetime_seconds: m.lifetime_seconds,
                pose: m.pose,
                transform,
                scale: m.scale,
                color: m.color,
                points: m.points.as_ptr().cast(),
                point_count: m.points.len(),
                colors: m.colors.as_ptr().cast(),
                color_count: m.colors.len(),
            });
        }
        let mut error = std::ptr::null_mut();
        let raw = unsafe {
            horus_mesh_source_markers(
                self.handle.as_ptr(),
                views.as_ptr(),
                views.len(),
                now,
                &mut error,
            )
        };
        pages(raw, error)
    }
    pub fn apply_blocks<F>(
        &mut self,
        blocks: &[NvbloxBlockUpdate<'_>],
        clear: bool,
        frame: &str,
        stamp: i64,
        mut lookup: F,
    ) -> Result<Vec<Arc<MeshPageUpdate>>>
    where
        F: FnMut(&str, i64) -> Result<MeshPose>,
    {
        ensure!(blocks.len() <= 65536, "Too many nvblox blocks");
        let transform = if frame == self.frame {
            MeshPose::default()
        } else {
            lookup(frame, stamp)?
        };
        let frame = CString::new(frame)?;
        let views: Vec<_> = blocks
            .iter()
            .map(|b| {
                let g = b.geometry;
                BlockView {
                    index: b.index,
                    deleted: i32::from(g.is_none()),
                    vertices: g.map_or(std::ptr::null(), |g| g.vertices.as_ptr().cast()),
                    vertex_count: g.map_or(0, |g| g.vertices.len()),
                    triangles: g.map_or(std::ptr::null(), |g| g.triangles.as_ptr()),
                    index_count: g.map_or(0, |g| g.triangles.len()),
                    colors: g.map_or(std::ptr::null(), |g| g.colors.as_ptr().cast()),
                    color_count: g.map_or(0, |g| g.colors.len()),
                }
            })
            .collect();
        let mut error = std::ptr::null_mut();
        let raw = unsafe {
            horus_mesh_source_blocks(
                self.handle.as_ptr(),
                views.as_ptr(),
                views.len(),
                i32::from(clear),
                frame.as_ptr(),
                stamp,
                transform,
                &mut error,
            )
        };
        pages(raw, error)
    }
    fn query(&self, operation: &str, request: &str, now: f64) -> Result<Vec<Arc<MeshPageUpdate>>> {
        let operation = CString::new(operation)?;
        let request = CString::new(request)?;
        let mut error = std::ptr::null_mut();
        let raw = unsafe {
            horus_mesh_source_pages(
                self.handle.as_ptr(),
                operation.as_ptr(),
                request.as_ptr(),
                now,
                &mut error,
            )
        };
        pages(raw, error)
    }
    pub fn expire(&mut self, now: f64) -> Result<Vec<Arc<MeshPageUpdate>>> {
        self.query("expire", "", now)
    }
    pub fn snapshot_updates(&self) -> Result<Vec<Arc<MeshPageUpdate>>> {
        self.query("snapshot", "", 0.)
    }
    pub fn repair_updates(&self, request: &Value) -> Result<Vec<Arc<MeshPageUpdate>>> {
        self.query("repair", &request.to_string(), 0.)
    }
    pub fn status(&self) -> Result<Value> {
        let mut error = std::ptr::null_mut();
        let raw = unsafe { horus_mesh_source_status(self.handle.as_ptr(), &mut error) };
        if raw.is_null() {
            return Err(failure(error));
        }
        Ok(serde_json::from_str(&take(raw)?)?)
    }
}
impl Drop for MeshSourceState {
    fn drop(&mut self) {
        unsafe {
            horus_mesh_source_destroy(self.handle.as_ptr());
        }
    }
}
