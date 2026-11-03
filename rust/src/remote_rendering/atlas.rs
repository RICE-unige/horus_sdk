//! Bounded multiview point atlases and reconstruction. No implicit point decimation.
use super::{datasets::PointChunkStream, encoder::failure, scene::SceneAsset};
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use std::{
    ffi::{c_char, c_void, CString},
    ptr::NonNull,
};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RemoteRenderViewSpec {
    pub name: String,
    pub position: [f64; 3],
    pub yaw_degrees: f64,
    pub pitch_degrees: f64,
    pub atlas_x: f64,
    pub atlas_y: f64,
    pub atlas_width: f64,
    pub atlas_height: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AtlasOptions {
    pub width: u32,
    pub height: u32,
    pub point_radius: i32,
    pub hole_fill_iterations: i32,
    pub world_scale: f64,
    pub near_base: f64,
    pub far_base: f64,
    pub vertical_fov_degrees: f64,
}
impl Default for AtlasOptions {
    fn default() -> Self {
        Self {
            width: 640,
            height: 480,
            point_radius: 1,
            hole_fill_iterations: 2,
            world_scale: 1.,
            near_base: 0.3,
            far_base: 20.,
            vertical_fov_degrees: 52.,
        }
    }
}
unsafe extern "C" {
    fn horus_atlas_render(
        points: *const f32,
        colors: *const u8,
        count: usize,
        stream: *mut c_void,
        views: *const c_char,
        options: *const c_char,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_atlas_rgb(handle: *const c_void, count: *mut usize) -> *const u8;
    fn horus_atlas_depth(handle: *const c_void, count: *mut usize) -> *const f32;
    fn horus_atlas_unproject(
        handle: *const c_void,
        views: *const c_char,
        columns: u32,
        rows: u32,
        scale: f64,
        fov: f64,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_atlas_destroy(handle: *mut c_void);
}
pub struct MultiviewAtlas {
    handle: NonNull<c_void>,
    pub width: u32,
    pub height: u32,
}
// Once constructed, buffers are immutable and owned by this handle.
unsafe impl Send for MultiviewAtlas {}
unsafe impl Sync for MultiviewAtlas {}
impl MultiviewAtlas {
    fn render(
        points: &[[f32; 3]],
        colors: &[[u8; 3]],
        stream: *mut c_void,
        views: &[RemoteRenderViewSpec],
        options: &AtlasOptions,
    ) -> Result<Self> {
        ensure!(points.len() == colors.len(), "Point/color count mismatch");
        let v = CString::new(serde_json::to_vec(views)?)?;
        let o = CString::new(serde_json::to_vec(options)?)?;
        let mut e = std::ptr::null_mut();
        let p = unsafe {
            horus_atlas_render(
                points.as_ptr().cast(),
                colors.as_ptr().cast(),
                points.len(),
                stream,
                v.as_ptr(),
                o.as_ptr(),
                &mut e,
            )
        };
        Ok(Self {
            handle: NonNull::new(p).ok_or_else(|| failure(e))?,
            width: options.width,
            height: options.height,
        })
    }
    pub fn from_points(
        points: &[[f32; 3]],
        colors: &[[u8; 3]],
        views: &[RemoteRenderViewSpec],
        options: &AtlasOptions,
    ) -> Result<Self> {
        Self::render(points, colors, std::ptr::null_mut(), views, options)
    }
    /// Consumes the remaining stream; errors do not leave a reusable partial iterator.
    pub fn from_stream(
        stream: &mut PointChunkStream,
        views: &[RemoteRenderViewSpec],
        options: &AtlasOptions,
    ) -> Result<Self> {
        ensure!(!stream.done, "Point stream already consumed");
        stream.done = true;
        Self::render(&[], &[], stream.handle.as_ptr(), views, options)
    }
    pub fn rgb(&self) -> &[u8] {
        let mut n = 0;
        let p = unsafe { horus_atlas_rgb(self.handle.as_ptr(), &mut n) };
        unsafe { std::slice::from_raw_parts(p, n) }
    }
    pub fn depth(&self) -> &[f32] {
        let mut n = 0;
        let p = unsafe { horus_atlas_depth(self.handle.as_ptr(), &mut n) };
        unsafe { std::slice::from_raw_parts(p, n) }
    }
    pub fn unproject(
        &self,
        views: &[RemoteRenderViewSpec],
        columns: u32,
        rows: u32,
        world_scale: f64,
        vertical_fov_degrees: f64,
    ) -> Result<SceneAsset> {
        let v = CString::new(serde_json::to_vec(views)?)?;
        let mut e = std::ptr::null_mut();
        let p = unsafe {
            horus_atlas_unproject(
                self.handle.as_ptr(),
                v.as_ptr(),
                columns,
                rows,
                world_scale,
                vertical_fov_degrees,
                &mut e,
            )
        };
        Ok(SceneAsset {
            handle: NonNull::new(p).ok_or_else(|| failure(e))?,
        })
    }
    pub fn pack_rgbd(&self, near: f32, far: f32) -> Result<Vec<u8>> {
        let depth = super::frame::encode_luma_depth(
            self.depth(),
            self.width,
            self.height,
            near,
            far,
            super::frame::LumaDepthFormat::Static8,
        )?;
        let stride = self.width as usize * 3;
        let mut out = Vec::with_capacity(self.rgb().len() * 2);
        for y in 0..self.height as usize {
            out.extend_from_slice(&self.rgb()[y * stride..(y + 1) * stride]);
            out.extend_from_slice(&depth[y * stride..(y + 1) * stride]);
        }
        Ok(out)
    }
}
impl Drop for MultiviewAtlas {
    fn drop(&mut self) {
        unsafe { horus_atlas_destroy(self.handle.as_ptr()) }
    }
}
