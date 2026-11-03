//! Device-resident render/pack/NVENC. Only compressed packets cross to the CPU.
use super::{
    encoder::{failure, packets, EncodedH264Frame},
    gaussian::GaussianRenderer,
    gpu::{CudaRenderer, GpuRenderOptions},
    ViewState,
};
use anyhow::{anyhow, ensure, Result};
use std::{
    ffi::{c_char, c_void, CString},
    path::Path,
    ptr::NonNull,
};
unsafe extern "C" {
    fn horus_gpu_video_create(
        path: *const c_char,
        w: u32,
        h: u32,
        dh: u32,
        fps: u32,
        kbps: u32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_gpu_video_destroy(p: *mut c_void);
    fn horus_gpu_video_keyframe(p: *mut c_void);
    fn horus_gpu_video_encode(
        p: *mut c_void,
        renderer: *mut c_void,
        gaussian: i32,
        view: *const c_char,
        seq: u64,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        radius: i32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_gpu_video_close(p: *mut c_void, error: *mut *mut c_char) -> *mut c_void;
}
pub struct GpuVideoEncoder {
    handle: NonNull<c_void>,
}
// Native mutex protects codec state and frame buffers; keyframe requests are atomic.
unsafe impl Send for GpuVideoEncoder {}
unsafe impl Sync for GpuVideoEncoder {}
impl GpuVideoEncoder {
    pub fn new(
        library: impl AsRef<Path>,
        width_per_eye: u32,
        height: u32,
        depth_height: u32,
        fps: u32,
        bitrate_kbps: u32,
    ) -> Result<Self> {
        ensure!(
            library.as_ref().is_absolute(),
            "Use an absolute CUDA video plugin path"
        );
        let path = CString::new(
            library
                .as_ref()
                .to_str()
                .ok_or_else(|| anyhow!("CUDA video path is not UTF-8"))?,
        )?;
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_gpu_video_create(
                path.as_ptr(),
                width_per_eye,
                height,
                depth_height,
                fps,
                bitrate_kbps,
                &mut error,
            )
        };
        Ok(Self {
            handle: NonNull::new(p).ok_or_else(|| failure(error))?,
        })
    }
    fn encode(
        &self,
        renderer: NonNull<c_void>,
        gaussian: bool,
        view: &ViewState,
        sequence: u64,
        o: GpuRenderOptions,
    ) -> Result<Vec<EncodedH264Frame>> {
        o.pixels(2)?;
        let json = CString::new(view.to_json()?)?;
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_gpu_video_encode(
                self.handle.as_ptr(),
                renderer.as_ptr(),
                gaussian as i32,
                json.as_ptr(),
                sequence,
                o.width,
                o.height,
                o.near_m,
                o.far_m,
                o.point_radius,
                &mut error,
            )
        };
        packets(p, error)
    }
    pub fn encode_cuda(
        &self,
        renderer: &CudaRenderer,
        view: &ViewState,
        sequence: u64,
        o: GpuRenderOptions,
    ) -> Result<Vec<EncodedH264Frame>> {
        self.encode(renderer.handle, false, view, sequence, o)
    }
    pub fn encode_gaussian(
        &self,
        renderer: &GaussianRenderer,
        view: &ViewState,
        sequence: u64,
        o: GpuRenderOptions,
    ) -> Result<Vec<EncodedH264Frame>> {
        self.encode(renderer.handle, true, view, sequence, o)
    }
    pub fn request_keyframe(&self) {
        unsafe { horus_gpu_video_keyframe(self.handle.as_ptr()) }
    }
    pub fn close(&self) -> Result<Vec<EncodedH264Frame>> {
        let mut error = std::ptr::null_mut();
        let p = unsafe { horus_gpu_video_close(self.handle.as_ptr(), &mut error) };
        packets(p, error)
    }
}
impl Drop for GpuVideoEncoder {
    fn drop(&mut self) {
        unsafe { horus_gpu_video_destroy(self.handle.as_ptr()) }
    }
}
