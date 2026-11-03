//! Optional native NVENC encoding. Host RGB input, hardware H.264 output; no Python process.
use anyhow::{anyhow, ensure, Result};
use std::ffi::{c_char, c_void, CStr, CString};
use std::ptr::NonNull;

unsafe extern "C" {
    fn horus_encoder_compiled() -> i32;
    fn horus_encoder_create(
        w: u32,
        h: u32,
        fps: u32,
        kbps: u32,
        gpu: u32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_encoder_destroy(p: *mut c_void);
    fn horus_encoder_keyframe(p: *mut c_void);
    fn horus_encoder_encode(
        p: *mut c_void,
        rgb: *const u8,
        n: usize,
        json: *const c_char,
        seq: u64,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_encoder_close(p: *mut c_void, error: *mut *mut c_char) -> *mut c_void;
    fn horus_async_encoder_create(
        w: u32,
        h: u32,
        fps: u32,
        kbps: u32,
        gpu: u32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_async_encoder_destroy(p: *mut c_void);
    fn horus_async_encoder_keyframe(p: *mut c_void);
    fn horus_async_encoder_submit(
        p: *mut c_void,
        rgb: *const u8,
        n: usize,
        json: *const c_char,
        seq: u64,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_async_encoder_poll(
        p: *mut c_void,
        n: usize,
        timeout: u32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_async_encoder_close(p: *mut c_void, error: *mut *mut c_char) -> *mut c_void;
    fn horus_async_encoder_replaced(p: *const c_void) -> u64;
    fn horus_encoder_result_count(p: *const c_void) -> usize;
    fn horus_encoder_result_data(p: *const c_void, i: usize, n: *mut usize) -> *const u8;
    fn horus_encoder_result_metadata(p: *const c_void, i: usize) -> *const c_char;
    fn horus_encoder_result_info(p: *const c_void, i: usize, seq: *mut u64, index: *mut u64)
        -> i32;
    fn horus_encoder_result_destroy(p: *mut c_void);
    fn horus_teach_free(p: *mut c_char);
}
pub(super) fn failure(p: *mut c_char) -> anyhow::Error {
    if p.is_null() {
        return anyhow!("Native encoder failed");
    }
    let text = unsafe { CStr::from_ptr(p).to_string_lossy().into_owned() };
    unsafe { horus_teach_free(p) };
    anyhow!(text)
}
#[derive(Clone, Copy, Debug)]
pub struct H264EncoderOptions {
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub bitrate_kbps: u32,
    pub gpu_id: u32,
}
impl Default for H264EncoderOptions {
    fn default() -> Self {
        Self {
            width: 1280,
            height: 512,
            fps: 30,
            bitrate_kbps: 12000,
            gpu_id: 0,
        }
    }
}
#[derive(Debug)]
pub struct EncodedH264Frame {
    pub data: Vec<u8>,
    pub metadata_json: String,
    pub source_sequence: u64,
    pub frame_index: u64,
    pub keyframe: bool,
}
struct Packets(NonNull<c_void>);
impl Drop for Packets {
    fn drop(&mut self) {
        unsafe { horus_encoder_result_destroy(self.0.as_ptr()) }
    }
}
pub(super) fn packets(p: *mut c_void, error: *mut c_char) -> Result<Vec<EncodedH264Frame>> {
    let owned = Packets(NonNull::new(p).ok_or_else(|| failure(error))?);
    let p = owned.0.as_ptr();
    let count = unsafe { horus_encoder_result_count(p) };
    ensure!(count <= 32, "Encoder packet limit exceeded");
    (0..count)
        .map(|i| {
            let mut size = 0;
            let data = unsafe { horus_encoder_result_data(p, i, &mut size) };
            let metadata = unsafe { horus_encoder_result_metadata(p, i) };
            let (mut seq, mut index) = (0, 0);
            let keyframe = unsafe { horus_encoder_result_info(p, i, &mut seq, &mut index) };
            ensure!(
                !data.is_null() && !metadata.is_null() && keyframe >= 0,
                "Invalid encoder packet"
            );
            Ok(EncodedH264Frame {
                data: unsafe { std::slice::from_raw_parts(data, size) }.to_vec(),
                metadata_json: unsafe { CStr::from_ptr(metadata) }.to_str()?.to_owned(),
                source_sequence: seq,
                frame_index: index,
                keyframe: keyframe != 0,
            })
        })
        .collect()
}
pub struct H264Encoder {
    handle: NonNull<c_void>,
}
// All codec calls are serialized by the native mutex; keyframe requests are atomic.
unsafe impl Send for H264Encoder {}
unsafe impl Sync for H264Encoder {}
impl H264Encoder {
    pub fn compiled() -> bool {
        unsafe { horus_encoder_compiled() != 0 }
    }
    pub fn new(o: H264EncoderOptions) -> Result<Self> {
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_encoder_create(
                o.width,
                o.height,
                o.fps,
                o.bitrate_kbps,
                o.gpu_id,
                &mut error,
            )
        };
        Ok(Self {
            handle: NonNull::new(p).ok_or_else(|| failure(error))?,
        })
    }
    pub fn request_keyframe(&self) {
        unsafe { horus_encoder_keyframe(self.handle.as_ptr()) }
    }
    pub fn encode_rgb(
        &self,
        rgb: &[u8],
        metadata_json: &str,
        source_sequence: u64,
    ) -> Result<Vec<EncodedH264Frame>> {
        let json = CString::new(metadata_json)?;
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_encoder_encode(
                self.handle.as_ptr(),
                rgb.as_ptr(),
                rgb.len(),
                json.as_ptr(),
                source_sequence,
                &mut error,
            )
        };
        packets(p, error)
    }
    pub fn close(&self) -> Result<Vec<EncodedH264Frame>> {
        let mut error = std::ptr::null_mut();
        let p = unsafe { horus_encoder_close(self.handle.as_ptr(), &mut error) };
        packets(p, error)
    }
}
impl Drop for H264Encoder {
    fn drop(&mut self) {
        unsafe { horus_encoder_destroy(self.handle.as_ptr()) }
    }
}

/// Bounded background encoding: one waiting raw frame and backpressured encoded
/// output. Only unencoded obsolete frames are replaced; H.264 references stay intact.
pub struct AsyncH264Encoder {
    handle: NonNull<c_void>,
}
unsafe impl Send for AsyncH264Encoder {}
unsafe impl Sync for AsyncH264Encoder {}
impl AsyncH264Encoder {
    pub fn new(o: H264EncoderOptions) -> Result<Self> {
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_async_encoder_create(
                o.width,
                o.height,
                o.fps,
                o.bitrate_kbps,
                o.gpu_id,
                &mut error,
            )
        };
        Ok(Self {
            handle: NonNull::new(p).ok_or_else(|| failure(error))?,
        })
    }
    pub fn submit_rgb(&self, rgb: &[u8], metadata_json: &str, source_sequence: u64) -> Result<()> {
        let json = CString::new(metadata_json)?;
        let mut error = std::ptr::null_mut();
        let code = unsafe {
            horus_async_encoder_submit(
                self.handle.as_ptr(),
                rgb.as_ptr(),
                rgb.len(),
                json.as_ptr(),
                source_sequence,
                &mut error,
            )
        };
        if code != 0 {
            return Err(failure(error));
        }
        Ok(())
    }
    pub fn request_keyframe(&self) {
        unsafe { horus_async_encoder_keyframe(self.handle.as_ptr()) }
    }
    pub fn poll_encoded(
        &self,
        maximum_frames: usize,
        timeout_ms: u32,
    ) -> Result<Vec<EncodedH264Frame>> {
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_async_encoder_poll(self.handle.as_ptr(), maximum_frames, timeout_ms, &mut error)
        };
        packets(p, error)
    }
    pub fn replaced_count(&self) -> u64 {
        unsafe { horus_async_encoder_replaced(self.handle.as_ptr()) }
    }
    pub fn close(&self) -> Result<Vec<EncodedH264Frame>> {
        let mut error = std::ptr::null_mut();
        let p = unsafe { horus_async_encoder_close(self.handle.as_ptr(), &mut error) };
        packets(p, error)
    }
}
impl Drop for AsyncH264Encoder {
    fn drop(&mut self) {
        unsafe { horus_async_encoder_destroy(self.handle.as_ptr()) }
    }
}
