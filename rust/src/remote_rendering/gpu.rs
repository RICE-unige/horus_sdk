//! In-process bindings to the same CUDA point and textured-mesh engines as Python.
//! An explicit prebuilt HORUS CUDA library is required; no Python wrapper is used.
use super::EyeView;
use anyhow::{anyhow, ensure, Result};
use std::ffi::{c_char, c_void, CStr, CString};
use std::path::Path;
use std::ptr::NonNull;

#[repr(C)]
#[derive(Clone, Copy, Default)]
pub(super) struct NativeEye {
    pub(super) position: [f64; 3],
    pub(super) rotation: [f64; 4],
    pub(super) fov: [f64; 4],
}
unsafe extern "C" {
    fn horus_gpu_points_create(
        path: *const c_char,
        n: usize,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_gpu_mesh_create(
        path: *const c_char,
        n: usize,
        f: usize,
        vertices: *const f32,
        faces: *const u32,
        colors: *const u8,
        uvs: *const f32,
        rects: *const u32,
        atlas: *const u8,
        w: u32,
        h: u32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_gpu_upload(
        handle: *mut c_void,
        offset: usize,
        count: usize,
        points: *const f32,
        colors: *const u8,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_gpu_render(
        handle: *mut c_void,
        eyes: *const NativeEye,
        count: usize,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        radius: i32,
        rgb: *mut u8,
        rgb_size: usize,
        depth: *mut f32,
        depth_size: usize,
        elapsed: *mut f32,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_gpu_destroy(handle: *mut c_void);
    fn horus_teach_free(error: *mut c_char);
}
fn error(value: *mut c_char) -> anyhow::Error {
    if value.is_null() {
        return anyhow!("Native GPU operation failed");
    }
    let message = unsafe { CStr::from_ptr(value).to_string_lossy().into_owned() };
    unsafe { horus_teach_free(value) };
    anyhow!(message)
}
fn path(path: &Path) -> Result<CString> {
    ensure!(path.is_absolute(), "Use an absolute CUDA library path");
    Ok(CString::new(path.to_str().ok_or_else(|| {
        anyhow!("CUDA library path is not UTF-8")
    })?)?)
}
pub struct MeshInput<'a> {
    pub vertices: &'a [[f32; 3]],
    pub faces: &'a [[u32; 3]],
    pub colors: &'a [[u8; 3]],
    pub uvs: Option<&'a [[f32; 2]]>,
    pub texture_rects: Option<&'a [[u32; 4]]>,
    pub texture_atlas: Option<&'a [[u8; 3]]>,
    pub texture_width: u32,
    pub texture_height: u32,
}
impl<'a> MeshInput<'a> {
    pub fn new(vertices: &'a [[f32; 3]], faces: &'a [[u32; 3]], colors: &'a [[u8; 3]]) -> Self {
        Self {
            vertices,
            faces,
            colors,
            uvs: None,
            texture_rects: None,
            texture_atlas: None,
            texture_width: 1,
            texture_height: 1,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct GpuRenderOptions {
    pub width: u32,
    pub height: u32,
    pub near_m: f32,
    pub far_m: f32,
    pub point_radius: i32,
}
impl Default for GpuRenderOptions {
    fn default() -> Self {
        Self {
            width: 640,
            height: 480,
            near_m: 0.05,
            far_m: 100.0,
            point_radius: 1,
        }
    }
}
impl GpuRenderOptions {
    pub(super) fn pixels(&self, views: usize) -> Result<usize> {
        ensure!(
            (1..=12).contains(&views)
                && (1..=16384).contains(&self.width)
                && (1..=16384).contains(&self.height),
            "Invalid view dimensions"
        );
        ensure!(
            self.near_m.is_finite()
                && self.far_m.is_finite()
                && self.near_m > 0.0
                && self.far_m > self.near_m
                && (0..=8).contains(&self.point_radius),
            "Invalid depth range or radius"
        );
        let pixels = views as u64 * self.width as u64 * self.height as u64;
        ensure!(
            pixels <= 256 * 1024 * 1024 / 7,
            "GPU output exceeds 256 MiB host frame budget"
        );
        Ok(pixels as usize)
    }
}
pub struct RenderedViews {
    pub width: u32,
    pub height: u32,
    pub view_count: usize,
    pub rgb: Vec<u8>,
    pub depth: Vec<f32>,
    pub cuda_ms: f32,
}
pub struct CudaRenderer {
    pub(super) handle: NonNull<c_void>,
}
// The shared native engine serializes every upload/render and owns its CUDA state.
unsafe impl Send for CudaRenderer {}
unsafe impl Sync for CudaRenderer {}
impl CudaRenderer {
    pub fn points(library: impl AsRef<Path>, count: usize) -> Result<Self> {
        let library = path(library.as_ref())?;
        let mut failure = std::ptr::null_mut();
        let raw = unsafe { horus_gpu_points_create(library.as_ptr(), count, &mut failure) };
        Ok(Self {
            handle: NonNull::new(raw).ok_or_else(|| error(failure))?,
        })
    }
    pub fn mesh(library: impl AsRef<Path>, m: &MeshInput<'_>) -> Result<Self> {
        ensure!(m.colors.len() == m.faces.len(), "Face color count mismatch");
        ensure!(
            m.uvs.is_none_or(|uv| uv.len() == m.vertices.len()),
            "UV count mismatch"
        );
        ensure!(
            m.texture_rects.is_none_or(|r| r.len() == m.faces.len()),
            "Texture rect count mismatch"
        );
        ensure!(
            m.texture_atlas
                .is_none_or(|a| a.len() as u64 == m.texture_width as u64 * m.texture_height as u64),
            "Atlas shape mismatch"
        );
        let library = path(library.as_ref())?;
        let mut failure = std::ptr::null_mut();
        let raw = unsafe {
            horus_gpu_mesh_create(
                library.as_ptr(),
                m.vertices.len(),
                m.faces.len(),
                m.vertices.as_ptr().cast(),
                m.faces.as_ptr().cast(),
                m.colors.as_ptr().cast(),
                m.uvs.map_or(std::ptr::null(), |v| v.as_ptr().cast()),
                m.texture_rects
                    .map_or(std::ptr::null(), |v| v.as_ptr().cast()),
                m.texture_atlas
                    .map_or(std::ptr::null(), |v| v.as_ptr().cast()),
                m.texture_width,
                m.texture_height,
                &mut failure,
            )
        };
        Ok(Self {
            handle: NonNull::new(raw).ok_or_else(|| error(failure))?,
        })
    }
    pub fn upload(&self, points: &[[f32; 3]], colors: &[[u8; 3]], offset: usize) -> Result<()> {
        ensure!(points.len() == colors.len(), "Point color count mismatch");
        let mut failure = std::ptr::null_mut();
        let code = unsafe {
            horus_gpu_upload(
                self.handle.as_ptr(),
                offset,
                points.len(),
                points.as_ptr().cast(),
                colors.as_ptr().cast(),
                &mut failure,
            )
        };
        if code != 0 {
            return Err(error(failure));
        }
        Ok(())
    }
    pub fn render_views(
        &self,
        eyes: &[EyeView],
        options: GpuRenderOptions,
    ) -> Result<RenderedViews> {
        let n = options.pixels(eyes.len())?;
        let mut out = RenderedViews {
            width: options.width,
            height: options.height,
            view_count: eyes.len(),
            rgb: vec![0; n * 3],
            depth: vec![0.0; n],
            cuda_ms: 0.0,
        };
        out.cuda_ms = self.render_into(eyes, options, &mut out.rgb, &mut out.depth)?;
        Ok(out)
    }
    /// Reuses caller-owned frame buffers across renders; no per-frame image allocation.
    pub fn render_into(
        &self,
        eyes: &[EyeView],
        options: GpuRenderOptions,
        rgb: &mut [u8],
        depth: &mut [f32],
    ) -> Result<f32> {
        let n = options.pixels(eyes.len())?;
        ensure!(
            rgb.len() == n * 3 && depth.len() == n,
            "Output frame shape mismatch"
        );
        let mut views = [NativeEye::default(); 12];
        for (target, e) in views.iter_mut().zip(eyes) {
            *target = NativeEye {
                position: [e.position.x, e.position.y, e.position.z],
                rotation: [e.rotation.x, e.rotation.y, e.rotation.z, e.rotation.w],
                fov: [e.fov.left, e.fov.right, e.fov.up, e.fov.down],
            };
        }
        let mut failure = std::ptr::null_mut();
        let mut elapsed = 0.0;
        let code = unsafe {
            horus_gpu_render(
                self.handle.as_ptr(),
                views.as_ptr(),
                eyes.len(),
                options.width,
                options.height,
                options.near_m,
                options.far_m,
                options.point_radius,
                rgb.as_mut_ptr(),
                rgb.len(),
                depth.as_mut_ptr(),
                depth.len(),
                &mut elapsed,
                &mut failure,
            )
        };
        if code != 0 {
            return Err(error(failure));
        }
        Ok(elapsed)
    }
}
impl Drop for CudaRenderer {
    fn drop(&mut self) {
        unsafe {
            horus_gpu_destroy(self.handle.as_ptr());
        }
    }
}
