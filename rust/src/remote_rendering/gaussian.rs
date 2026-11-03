//! Original 3DGS assets and an in-process gsplat renderer. No Python runtime.
use super::{
    gpu::{GpuRenderOptions, NativeEye, RenderedViews},
    EyeView,
};
use anyhow::{anyhow, ensure, Result};
use serde::{Deserialize, Serialize};
use std::{
    ffi::{c_char, c_void, CStr, CString},
    path::Path,
    ptr::NonNull,
};

unsafe extern "C" {
    fn horus_gaussian_scene_load(
        path: *const c_char,
        scale: f64,
        down: i32,
        q: f64,
        n: u64,
        bytes: u64,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_gaussian_scene_destroy(p: *mut c_void);
    fn horus_gaussian_scene_array(p: *const c_void, kind: i32, count: *mut usize) -> *const c_void;
    fn horus_gaussian_scene_metadata(p: *const c_void) -> *mut c_char;
    fn horus_gaussian_renderer_create(
        path: *const c_char,
        scene: *const c_void,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_gaussian_renderer_destroy(p: *mut c_void);
    fn horus_gaussian_renderer_render(
        p: *mut c_void,
        eyes: *const NativeEye,
        count: usize,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        rgb: *mut u8,
        nrgb: usize,
        depth: *mut f32,
        nd: usize,
        elapsed: *mut f32,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_teach_free(p: *mut c_char);
}
fn native_string(p: *mut c_char) -> Result<String> {
    ensure!(!p.is_null(), "Native Gaussian operation failed");
    let text = unsafe { CStr::from_ptr(p).to_string_lossy().into_owned() };
    unsafe { horus_teach_free(p) };
    Ok(text)
}
fn failure(p: *mut c_char) -> anyhow::Error {
    anyhow!(native_string(p).unwrap_or_else(|e| e.to_string()))
}
fn path(p: &Path) -> Result<CString> {
    Ok(CString::new(
        p.to_str().ok_or_else(|| anyhow!("Path is not UTF-8"))?,
    )?)
}
#[derive(Clone, Copy, Debug)]
pub struct GaussianLoadOptions {
    pub world_scale: f64,
    pub canonicalize_y_down: bool,
    pub floor_quantile: f64,
    pub maximum_gaussians: u64,
    pub maximum_decoded_bytes: u64,
}
impl Default for GaussianLoadOptions {
    fn default() -> Self {
        Self {
            world_scale: 1.,
            canonicalize_y_down: false,
            floor_quantile: 0.995,
            maximum_gaussians: 20_000_000,
            maximum_decoded_bytes: 8 * 1024 * 1024 * 1024,
        }
    }
}
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct GaussianMetadata {
    pub sh_degree: u32,
    pub discarded_nonfinite: usize,
    pub world_bounds_min: [f64; 3],
    pub world_bounds_max: [f64; 3],
    pub source_from_world_translation: [f64; 3],
    pub source_from_world_rotation: [f64; 4],
}
pub struct GaussianScene {
    handle: NonNull<c_void>,
    metadata: GaussianMetadata,
}
// All scene arrays are immutable and borrowed for the owner's lifetime.
unsafe impl Send for GaussianScene {}
unsafe impl Sync for GaussianScene {}
impl GaussianScene {
    pub fn load(path_value: impl AsRef<Path>, o: GaussianLoadOptions) -> Result<Self> {
        let name = path(path_value.as_ref())?;
        let mut error = std::ptr::null_mut();
        let raw = unsafe {
            horus_gaussian_scene_load(
                name.as_ptr(),
                o.world_scale,
                o.canonicalize_y_down as i32,
                o.floor_quantile,
                o.maximum_gaussians,
                o.maximum_decoded_bytes,
                &mut error,
            )
        };
        let handle = NonNull::new(raw).ok_or_else(|| failure(error))?;
        let metadata = (|| {
            let text = native_string(unsafe { horus_gaussian_scene_metadata(raw) })?;
            Ok::<_, anyhow::Error>(serde_json::from_str(&text)?)
        })();
        match metadata {
            Ok(metadata) => Ok(Self { handle, metadata }),
            Err(e) => {
                unsafe { horus_gaussian_scene_destroy(raw) };
                Err(e)
            }
        }
    }
    pub fn metadata(&self) -> &GaussianMetadata {
        &self.metadata
    }
    fn array<T>(&self, kind: i32) -> &[T] {
        let mut n = 0;
        let raw = unsafe { horus_gaussian_scene_array(self.handle.as_ptr(), kind, &mut n) };
        if n == 0 {
            return &[];
        }
        assert!(!raw.is_null());
        unsafe { std::slice::from_raw_parts(raw.cast::<T>(), n) }
    }
    pub fn means(&self) -> &[[f32; 3]] {
        self.array(0)
    }
    pub fn scales(&self) -> &[[f32; 3]] {
        self.array(1)
    }
    /// Normalized wxyz (not the xyzw convention of eye poses).
    pub fn quaternions(&self) -> &[[f32; 4]] {
        self.array(2)
    }
    pub fn opacities(&self) -> &[f32] {
        self.array(3)
    }
    /// Flat Gaussian/coefficient/RGB order; coefficients=(sh_degree+1)^2.
    pub fn spherical_harmonics(&self) -> &[[f32; 3]] {
        self.array(4)
    }
}
impl Drop for GaussianScene {
    fn drop(&mut self) {
        unsafe { horus_gaussian_scene_destroy(self.handle.as_ptr()) }
    }
}
pub struct GaussianRenderer {
    pub(super) handle: NonNull<c_void>,
}
// Native mutex serializes render calls; its CUDA stream/device is explicitly guarded.
unsafe impl Send for GaussianRenderer {}
unsafe impl Sync for GaussianRenderer {}
impl GaussianRenderer {
    pub fn new(library: impl AsRef<Path>, scene: &GaussianScene) -> Result<Self> {
        ensure!(
            library.as_ref().is_absolute(),
            "Use an absolute Gaussian plugin path"
        );
        let name = path(library.as_ref())?;
        let mut error = std::ptr::null_mut();
        let raw = unsafe {
            horus_gaussian_renderer_create(name.as_ptr(), scene.handle.as_ptr(), &mut error)
        };
        Ok(Self {
            handle: NonNull::new(raw).ok_or_else(|| failure(error))?,
        })
    }
    pub fn render_views(&self, eyes: &[EyeView], o: GpuRenderOptions) -> Result<RenderedViews> {
        let n = o.pixels(eyes.len())?;
        let mut out = RenderedViews {
            width: o.width,
            height: o.height,
            view_count: eyes.len(),
            rgb: vec![0; n * 3],
            depth: vec![0.; n],
            cuda_ms: 0.,
        };
        out.cuda_ms = self.render_into(eyes, o, &mut out.rgb, &mut out.depth)?;
        Ok(out)
    }
    pub fn render_into(
        &self,
        eyes: &[EyeView],
        o: GpuRenderOptions,
        rgb: &mut [u8],
        depth: &mut [f32],
    ) -> Result<f32> {
        let n = o.pixels(eyes.len())?;
        ensure!(
            rgb.len() == n * 3 && depth.len() == n,
            "Gaussian output shape mismatch"
        );
        let mut views = [NativeEye::default(); 12];
        for (v, e) in views.iter_mut().zip(eyes) {
            *v = NativeEye {
                position: [e.position.x, e.position.y, e.position.z],
                rotation: [e.rotation.x, e.rotation.y, e.rotation.z, e.rotation.w],
                fov: [e.fov.left, e.fov.right, e.fov.up, e.fov.down],
            };
        }
        let mut error = std::ptr::null_mut();
        let mut elapsed = 0.;
        let code = unsafe {
            horus_gaussian_renderer_render(
                self.handle.as_ptr(),
                views.as_ptr(),
                eyes.len(),
                o.width,
                o.height,
                o.near_m,
                o.far_m,
                rgb.as_mut_ptr(),
                rgb.len(),
                depth.as_mut_ptr(),
                depth.len(),
                &mut elapsed,
                &mut error,
            )
        };
        if code != 0 {
            return Err(failure(error));
        }
        Ok(elapsed)
    }
}
impl Drop for GaussianRenderer {
    fn drop(&mut self) {
        unsafe { horus_gaussian_renderer_destroy(self.handle.as_ptr()) }
    }
}
