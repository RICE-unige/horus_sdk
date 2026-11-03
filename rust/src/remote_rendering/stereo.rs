//! Native point reference rendering and exact stereo RGB/depth carriers.
use super::gpu::{GpuRenderOptions, NativeEye, RenderedViews};
use super::{EyeView, RenderedFrameMetadata, ViewState};
use anyhow::{anyhow, ensure, Result};
use serde::Serialize;
use std::ffi::{c_char, c_void, CStr, CString};
use std::ptr::NonNull;

unsafe extern "C" {
    fn horus_stereo_assemble(
        rgb: *const u8,
        rn: usize,
        depth: *const f32,
        dn: usize,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        context: *const c_char,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_stereo_data(result: *const c_void, which: i32, size: *mut usize) -> *const u8;
    fn horus_stereo_metadata(result: *const c_void) -> *const c_char;
    fn horus_stereo_visibility(result: *const c_void) -> f64;
    fn horus_stereo_diagnostic(result: *const c_void, error: *mut *mut c_char) -> *mut c_void;
    fn horus_stereo_destroy(result: *mut c_void);
    fn horus_cpu_points(
        points: *const f32,
        colors: *const u8,
        count: usize,
        eyes: *const NativeEye,
        eye_count: usize,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        radius: i32,
        rgb: *mut u8,
        rn: usize,
        depth: *mut f32,
        dn: usize,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_teach_free(p: *mut c_char);
    fn horus_frame_buffer_data(p: *const c_void, size: *mut usize) -> *const u8;
    fn horus_frame_buffer_destroy(p: *mut c_void);
}
fn failure(p: *mut c_char) -> anyhow::Error {
    if p.is_null() {
        return anyhow!("Native stereo operation failed");
    }
    let text = unsafe { CStr::from_ptr(p).to_string_lossy().into_owned() };
    unsafe { horus_teach_free(p) };
    anyhow!(text)
}
#[derive(Clone, Debug, Serialize)]
pub struct StereoFrameContext {
    pub session_id: String,
    pub view_stream_id: String,
    pub codec: String,
    pub sequence: u64,
    pub source_view_sequence: u64,
    pub workspace_revision: u64,
    pub predicted_display_time_ns: u64,
    pub render_start_ns: u64,
    pub render_complete_ns: u64,
    pub capture_time_ns: u64,
    pub depth_strip_height: u32,
    pub marker_height: u32,
}
impl Default for StereoFrameContext {
    fn default() -> Self {
        Self {
            session_id: "preview".into(),
            view_stream_id: String::new(),
            codec: "png".into(),
            sequence: 0,
            source_view_sequence: 0,
            workspace_revision: 0,
            predicted_display_time_ns: 0,
            render_start_ns: 0,
            render_complete_ns: 0,
            capture_time_ns: 0,
            depth_strip_height: 0,
            marker_height: 0,
        }
    }
}
impl StereoFrameContext {
    pub fn from_view(state: &ViewState, sequence: u64) -> Self {
        Self {
            session_id: state.session_id.clone(),
            view_stream_id: state.view_stream_id.clone(),
            sequence,
            source_view_sequence: state.view_sequence,
            workspace_revision: state.workspace.revision,
            predicted_display_time_ns: state.predicted_display_time_ns,
            ..Default::default()
        }
    }
}
/// Owns the native buffers. Borrowed image slices cannot outlive this frame.
pub struct StereoFrameRender {
    handle: NonNull<c_void>,
    metadata: RenderedFrameMetadata,
}
// All accessors are read-only and the native frame owns every buffer.
unsafe impl Send for StereoFrameRender {}
unsafe impl Sync for StereoFrameRender {}
impl Drop for StereoFrameRender {
    fn drop(&mut self) {
        unsafe { horus_stereo_destroy(self.handle.as_ptr()) }
    }
}
impl StereoFrameRender {
    pub fn metadata(&self) -> &RenderedFrameMetadata {
        &self.metadata
    }
    fn data(&self, which: i32) -> &[u8] {
        let mut n = 0;
        let p = unsafe { horus_stereo_data(self.handle.as_ptr(), which, &mut n) };
        if n == 0 {
            return &[];
        }
        unsafe { std::slice::from_raw_parts(p, n) }
    }
    pub fn stereo_rgba(&self) -> &[u8] {
        self.data(0)
    }
    pub fn video_rgb(&self) -> &[u8] {
        self.data(1)
    }
    pub fn visibility_fraction(&self) -> f64 {
        unsafe { horus_stereo_visibility(self.handle.as_ptr()) }
    }
    pub fn diagnostic_packet(&self) -> Result<Vec<u8>> {
        let mut error = std::ptr::null_mut();
        let p = unsafe { horus_stereo_diagnostic(self.handle.as_ptr(), &mut error) };
        let p = NonNull::new(p).ok_or_else(|| failure(error))?;
        struct Buffer(NonNull<c_void>);
        impl Drop for Buffer {
            fn drop(&mut self) {
                unsafe { horus_frame_buffer_destroy(self.0.as_ptr()) }
            }
        }
        let buffer = Buffer(p);
        let mut n = 0;
        let data = unsafe { horus_frame_buffer_data(buffer.0.as_ptr(), &mut n) };
        Ok(if n == 0 {
            Vec::new()
        } else {
            unsafe { std::slice::from_raw_parts(data, n) }.to_vec()
        })
    }
}
pub fn assemble_stereo_frame(
    rgb: &[u8],
    depth: &[f32],
    options: GpuRenderOptions,
    context: &StereoFrameContext,
) -> Result<StereoFrameRender> {
    let n = options.pixels(2)?;
    ensure!(
        rgb.len() == n * 3 && depth.len() == n,
        "Stereo input shape mismatch"
    );
    let context = CString::new(serde_json::to_string(context)?)?;
    let mut error = std::ptr::null_mut();
    let raw = unsafe {
        horus_stereo_assemble(
            rgb.as_ptr(),
            rgb.len(),
            depth.as_ptr(),
            depth.len(),
            options.width,
            options.height,
            options.near_m,
            options.far_m,
            context.as_ptr(),
            &mut error,
        )
    };
    let handle = NonNull::new(raw).ok_or_else(|| failure(error))?;
    let text = unsafe { CStr::from_ptr(horus_stereo_metadata(handle.as_ptr())) };
    let parsed = text
        .to_str()
        .map_err(anyhow::Error::from)
        .and_then(RenderedFrameMetadata::from_json);
    match parsed {
        Ok(metadata) => Ok(StereoFrameRender { handle, metadata }),
        Err(error) => {
            unsafe { horus_stereo_destroy(handle.as_ptr()) };
            Err(error)
        }
    }
}
/// CPU reference implementation with the same camera contract as the CUDA path.
/// This processes every point and skips only non-finite and out-of-view samples.
pub fn render_colored_points(
    points: &[[f32; 3]],
    colors: &[[u8; 3]],
    eyes: &[EyeView],
    o: GpuRenderOptions,
) -> Result<RenderedViews> {
    ensure!(points.len() == colors.len(), "Point/color count mismatch");
    let n = o.pixels(eyes.len())?;
    let mut views = [NativeEye::default(); 12];
    for (e, out) in eyes.iter().zip(views.iter_mut()) {
        *out = NativeEye {
            position: [e.position.x, e.position.y, e.position.z],
            rotation: [e.rotation.x, e.rotation.y, e.rotation.z, e.rotation.w],
            fov: [e.fov.left, e.fov.right, e.fov.up, e.fov.down],
        };
    }
    let mut rgb = vec![0; n * 3];
    let mut depth = vec![0.; n];
    let mut error = std::ptr::null_mut();
    let ok = unsafe {
        horus_cpu_points(
            points.as_ptr().cast(),
            colors.as_ptr().cast(),
            points.len(),
            views.as_ptr(),
            eyes.len(),
            o.width,
            o.height,
            o.near_m,
            o.far_m,
            o.point_radius,
            rgb.as_mut_ptr(),
            rgb.len(),
            depth.as_mut_ptr(),
            depth.len(),
            &mut error,
        )
    };
    if ok != 1 {
        return Err(failure(error));
    }
    Ok(RenderedViews {
        width: o.width,
        height: o.height,
        view_count: eyes.len(),
        rgb,
        depth,
        cuda_ms: 0.,
    })
}
