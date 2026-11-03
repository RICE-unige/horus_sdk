//! HRRF v1/v2 depth frames and image helpers. Binary inputs are borrowed; no JSON images.
use anyhow::{anyhow, ensure, Result};
use std::ffi::{c_char, c_void, CStr};

pub const FRAME_MARKER_WIDTH: u32 = 384;
pub const FRAME_MARKER_HEIGHT: u32 = 8;
const MAX_PIXELS: usize = 64 * 1024 * 1024;
#[repr(C)]
#[derive(Clone, Copy)]
struct NativeInfo {
    capture_time_ns: u64,
    sequence: u32,
    width: u32,
    height: u32,
    color_width: u32,
    color_height: u32,
    view_count: u32,
    mono_atlas: u32,
    near_m: f32,
    far_m: f32,
    eyes: [[f32; 11]; 2],
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrameEye {
    pub projection: [f32; 4],
    pub position: [f32; 3],
    pub rotation: [f32; 4],
}
impl Default for FrameEye {
    fn default() -> Self {
        Self {
            projection: [1., 1., 0., 0.],
            position: [0.; 3],
            rotation: [0., 0., 0., 1.],
        }
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct DepthFrameInfo {
    pub capture_time_ns: u64,
    pub sequence: u32,
    pub width: u32,
    pub height: u32,
    pub color_width: u32,
    pub color_height: u32,
    pub view_count: u32,
    pub mono_atlas: bool,
    pub near_m: f32,
    pub far_m: f32,
    pub eyes: [FrameEye; 2],
}
impl Default for DepthFrameInfo {
    fn default() -> Self {
        Self {
            capture_time_ns: 0,
            sequence: 0,
            width: 0,
            height: 0,
            color_width: 0,
            color_height: 0,
            view_count: 2,
            mono_atlas: false,
            near_m: 0.05,
            far_m: 100.,
            eyes: [FrameEye::default(); 2],
        }
    }
}
impl DepthFrameInfo {
    fn native(&self) -> NativeInfo {
        let mut eyes = [[0.; 11]; 2];
        for (target, source) in eyes.iter_mut().zip(self.eyes.iter()) {
            target[..4].copy_from_slice(&source.projection);
            target[4..7].copy_from_slice(&source.position);
            target[7..].copy_from_slice(&source.rotation);
        }
        NativeInfo {
            capture_time_ns: self.capture_time_ns,
            sequence: self.sequence,
            width: self.width,
            height: self.height,
            color_width: self.color_width,
            color_height: self.color_height,
            view_count: self.view_count,
            mono_atlas: u32::from(self.mono_atlas),
            near_m: self.near_m,
            far_m: self.far_m,
            eyes,
        }
    }
    fn from_native(n: NativeInfo) -> Self {
        let eyes = n.eyes.map(|e| FrameEye {
            projection: e[..4].try_into().unwrap(),
            position: e[4..7].try_into().unwrap(),
            rotation: e[7..].try_into().unwrap(),
        });
        Self {
            capture_time_ns: n.capture_time_ns,
            sequence: n.sequence,
            width: n.width,
            height: n.height,
            color_width: n.color_width,
            color_height: n.color_height,
            view_count: n.view_count,
            mono_atlas: n.mono_atlas != 0,
            near_m: n.near_m,
            far_m: n.far_m,
            eyes,
        }
    }
}
pub struct DepthFrame {
    pub info: DepthFrameInfo,
    /// Top-left contiguous [eye][row][column], including a duplicated mono eye.
    pub depth_codes: Vec<u16>,
}
#[repr(i32)]
#[derive(Clone, Copy)]
pub enum DepthReduction {
    Minimum,
    Surfel,
    Reprojection,
}
unsafe extern "C" {
    fn horus_luma_encode(
        input: *const f32,
        n: usize,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        format: i32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_luma_decode(
        input: *const u8,
        n: usize,
        w: u32,
        h: u32,
        near: f32,
        far: f32,
        format: i32,
        out: *mut f32,
        count: usize,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_dynamic_pose(
        input: *mut u8,
        n: usize,
        w: u32,
        h: u32,
        pose: *mut f32,
        sequence: *mut u32,
        range: f32,
        encode: i32,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_dynamic_pack(
        color: *const u8,
        nc: usize,
        depth: *const f32,
        nd: usize,
        w: u32,
        h: u32,
        pose: *const f32,
        sequence: u32,
        near: f32,
        far: f32,
        range: f32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_depth_quantize(
        input: *const f32,
        n: usize,
        near: f32,
        far: f32,
        out: *mut u16,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_depth_dequantize(
        input: *const u16,
        n: usize,
        near: f32,
        far: f32,
        out: *mut f32,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_depth_reduce(
        input: *const f32,
        n: usize,
        sw: u32,
        sh: u32,
        w: u32,
        h: u32,
        mode: i32,
        absolute: f32,
        relative: f32,
        out: *mut f32,
        count: usize,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_frame_pack(
        input: *const f32,
        n: usize,
        info: *const NativeInfo,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_frame_unpack(
        input: *const u8,
        n: usize,
        info: *mut NativeInfo,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_frame_buffer_data(handle: *const c_void, size: *mut usize) -> *const u8;
    fn horus_frame_buffer_destroy(handle: *mut c_void);
    fn horus_frame_marker(
        input: *mut u8,
        n: usize,
        w: u32,
        h: u32,
        sequence: *mut u32,
        encode: i32,
        error: *mut *mut c_char,
    ) -> i32;
    fn horus_teach_free(error: *mut c_char);
}
fn failure(error: *mut c_char) -> anyhow::Error {
    if error.is_null() {
        return anyhow!("Native frame operation failed");
    }
    let message = unsafe { CStr::from_ptr(error).to_string_lossy().into_owned() };
    unsafe { horus_teach_free(error) };
    anyhow!(message)
}
fn check(ok: i32, error: *mut c_char) -> Result<()> {
    if ok == 0 {
        Err(failure(error))
    } else {
        Ok(())
    }
}
fn result_bytes(handle: *mut c_void, error: *mut c_char) -> Result<Vec<u8>> {
    ensure!(!handle.is_null(), "{}", failure(error));
    struct Buffer(*mut c_void);
    impl Drop for Buffer {
        fn drop(&mut self) {
            unsafe { horus_frame_buffer_destroy(self.0) }
        }
    }
    let guard = Buffer(handle);
    let mut len = 0;
    let bytes = unsafe { horus_frame_buffer_data(guard.0, &mut len) };
    ensure!(!bytes.is_null(), "Native frame buffer is null");
    Ok(unsafe { std::slice::from_raw_parts(bytes, len) }.to_vec())
}
pub fn quantize_depth(depth: &[f32], near_m: f32, far_m: f32) -> Result<Vec<u16>> {
    ensure!(depth.len() <= MAX_PIXELS, "Depth buffer exceeds limit");
    let mut out = vec![0; depth.len()];
    let mut error = std::ptr::null_mut();
    check(
        unsafe {
            horus_depth_quantize(
                depth.as_ptr(),
                depth.len(),
                near_m,
                far_m,
                out.as_mut_ptr(),
                &mut error,
            )
        },
        error,
    )?;
    Ok(out)
}
pub fn dequantize_depth(codes: &[u16], near_m: f32, far_m: f32) -> Result<Vec<f32>> {
    ensure!(codes.len() <= MAX_PIXELS, "Depth buffer exceeds limit");
    let mut out = vec![0.; codes.len()];
    let mut error = std::ptr::null_mut();
    check(
        unsafe {
            horus_depth_dequantize(
                codes.as_ptr(),
                codes.len(),
                near_m,
                far_m,
                out.as_mut_ptr(),
                &mut error,
            )
        },
        error,
    )?;
    Ok(out)
}
pub fn reduce_depth(
    depth: &[f32],
    source: [u32; 2],
    target: [u32; 2],
    mode: DepthReduction,
    edge_absolute_m: f32,
    edge_relative: f32,
) -> Result<Vec<f32>> {
    let count = u64::from(target[0]) * u64::from(target[1]);
    ensure!(
        target[0] > 0 && target[1] > 0 && count <= MAX_PIXELS as u64,
        "Invalid target dimensions"
    );
    ensure!(
        depth.len() as u64 == u64::from(source[0]) * u64::from(source[1]),
        "Source depth shape mismatch"
    );
    ensure!(
        target[0] <= source[0] && target[1] <= source[1],
        "Cannot upsample depth"
    );
    let mut out = vec![0.; count as usize];
    let mut error = std::ptr::null_mut();
    check(
        unsafe {
            horus_depth_reduce(
                depth.as_ptr(),
                depth.len(),
                source[0],
                source[1],
                target[0],
                target[1],
                mode as i32,
                edge_absolute_m,
                edge_relative,
                out.as_mut_ptr(),
                out.len(),
                &mut error,
            )
        },
        error,
    )?;
    Ok(out)
}
pub fn pack_depth_frame(depth: &[f32], info: &DepthFrameInfo) -> Result<Vec<u8>> {
    let mut error = std::ptr::null_mut();
    let handle =
        unsafe { horus_frame_pack(depth.as_ptr(), depth.len(), &info.native(), &mut error) };
    result_bytes(handle, error)
}
pub fn unpack_depth_frame(bytes: &[u8]) -> Result<DepthFrame> {
    let mut info = DepthFrameInfo::default().native();
    let mut error = std::ptr::null_mut();
    let handle = unsafe { horus_frame_unpack(bytes.as_ptr(), bytes.len(), &mut info, &mut error) };
    let raw = result_bytes(handle, error)?;
    Ok(DepthFrame {
        info: DepthFrameInfo::from_native(info),
        depth_codes: raw
            .chunks_exact(2)
            .map(|b| u16::from_le_bytes([b[0], b[1]]))
            .collect(),
    })
}
pub fn embed_frame_marker(
    rgb: &mut [u8],
    width: u32,
    height: u32,
    mut sequence: u32,
) -> Result<()> {
    let mut error = std::ptr::null_mut();
    check(
        unsafe {
            horus_frame_marker(
                rgb.as_mut_ptr(),
                rgb.len(),
                width,
                height,
                &mut sequence,
                1,
                &mut error,
            )
        },
        error,
    )
}
pub fn decode_frame_marker(rgb: &[u8], width: u32, height: u32) -> Result<u16> {
    let mut sequence = 0;
    let mut error = std::ptr::null_mut();
    // The native decode branch never mutates the image.
    check(
        unsafe {
            horus_frame_marker(
                rgb.as_ptr().cast_mut(),
                rgb.len(),
                width,
                height,
                &mut sequence,
                0,
                &mut error,
            )
        },
        error,
    )?;
    Ok(sequence as u16)
}
pub fn pack_ros_debug_frame(jpeg: &[u8], frame: &[u8]) -> Result<Vec<u8>> {
    ensure!(
        !jpeg.is_empty()
            && !frame.is_empty()
            && jpeg.len() <= 4 * 1024 * 1024
            && frame.len() <= 4 * 1024 * 1024,
        "Invalid ROS debug frame component lengths"
    );
    let mut out = Vec::with_capacity(16 + jpeg.len() + frame.len());
    out.extend_from_slice(b"HRRD\x01\0\0\0");
    out.extend_from_slice(&(frame.len() as u32).to_le_bytes());
    out.extend_from_slice(&(jpeg.len() as u32).to_le_bytes());
    out.extend_from_slice(frame);
    out.extend_from_slice(jpeg);
    Ok(out)
}
/// Returns borrowed (jpeg, HRRF frame data) slices.
pub fn unpack_ros_debug_frame(bytes: &[u8]) -> Result<(&[u8], &[u8])> {
    ensure!(
        bytes.len() >= 16 && &bytes[..5] == b"HRRD\x01",
        "Invalid ROS debug frame header"
    );
    let f = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
    let j = u32::from_le_bytes(bytes[12..16].try_into().unwrap()) as usize;
    ensure!(
        f > 0 && j > 0 && f <= 4 * 1024 * 1024 && j <= 4 * 1024 * 1024 && bytes.len() == 16 + f + j,
        "Invalid ROS debug frame lengths"
    );
    Ok((&bytes[16 + f..], &bytes[16..16 + f]))
}

#[repr(i32)]
#[derive(Clone, Copy)]
pub enum LumaDepthFormat {
    Static8,
    Dynamic16,
}
#[derive(Clone, Copy, Debug)]
pub struct DynamicCameraPose {
    pub position: [f32; 3],
    pub rotation: [f32; 4],
}
impl Default for DynamicCameraPose {
    fn default() -> Self {
        Self {
            position: [0.; 3],
            rotation: [0., 0., 0., 1.],
        }
    }
}
impl DynamicCameraPose {
    fn flat(self) -> [f32; 7] {
        let mut f = [0.; 7];
        f[..3].copy_from_slice(&self.position);
        f[3..].copy_from_slice(&self.rotation);
        f
    }
}
pub fn encode_luma_depth(
    depth: &[f32],
    width: u32,
    height: u32,
    near_m: f32,
    far_m: f32,
    format: LumaDepthFormat,
) -> Result<Vec<u8>> {
    let mut error = std::ptr::null_mut();
    let handle = unsafe {
        horus_luma_encode(
            depth.as_ptr(),
            depth.len(),
            width,
            height,
            near_m,
            far_m,
            format as i32,
            &mut error,
        )
    };
    result_bytes(handle, error)
}
pub fn decode_luma_depth(
    rgb: &[u8],
    width: u32,
    height: u32,
    near_m: f32,
    far_m: f32,
    format: LumaDepthFormat,
) -> Result<Vec<f32>> {
    let count = u64::from(width) * u64::from(height);
    ensure!(
        width > 0 && height > 0 && count <= 32 * 1024 * 1024 && rgb.len() as u64 == count * 3,
        "Invalid encoded luma dimensions"
    );
    let mut out = vec![0.; count as usize];
    let mut error = std::ptr::null_mut();
    check(
        unsafe {
            horus_luma_decode(
                rgb.as_ptr(),
                rgb.len(),
                width,
                height,
                near_m,
                far_m,
                format as i32,
                out.as_mut_ptr(),
                out.len(),
                &mut error,
            )
        },
        error,
    )?;
    Ok(out)
}
pub fn metadata_rows(width: u32) -> Result<u32> {
    ensure!(
        width >= 8,
        "Dynamic metadata requires at least 8 pixels width"
    );
    let blocks = width / 8;
    Ok(36_u32.div_ceil(blocks) * 8)
}
pub fn encode_dynamic_pose_metadata(
    rgb: &mut [u8],
    width: u32,
    height: u32,
    pose: DynamicCameraPose,
    mut sequence: u32,
    position_range_m: f32,
) -> Result<u32> {
    let mut error = std::ptr::null_mut();
    let mut pose = pose.flat();
    check(
        unsafe {
            horus_dynamic_pose(
                rgb.as_mut_ptr(),
                rgb.len(),
                width,
                height,
                pose.as_mut_ptr(),
                &mut sequence,
                position_range_m,
                1,
                &mut error,
            )
        },
        error,
    )?;
    metadata_rows(width)
}
pub fn decode_dynamic_pose_metadata(
    rgb: &[u8],
    width: u32,
    height: u32,
    position_range_m: f32,
) -> Result<(u16, DynamicCameraPose)> {
    let mut error = std::ptr::null_mut();
    let mut pose = [0.; 7];
    let mut sequence = 0;
    // Decode is read-only on the input image in the C ABI.
    check(
        unsafe {
            horus_dynamic_pose(
                rgb.as_ptr().cast_mut(),
                rgb.len(),
                width,
                height,
                pose.as_mut_ptr(),
                &mut sequence,
                position_range_m,
                0,
                &mut error,
            )
        },
        error,
    )?;
    Ok((
        sequence as u16,
        DynamicCameraPose {
            position: pose[..3].try_into().unwrap(),
            rotation: pose[3..].try_into().unwrap(),
        },
    ))
}
pub fn pack_dynamic_rgbd(
    color: &[u8],
    depth: &[f32],
    size: [u32; 2],
    pose: DynamicCameraPose,
    sequence: u32,
    near_m: f32,
    far_m: f32,
    position_range_m: f32,
) -> Result<Vec<u8>> {
    let mut error = std::ptr::null_mut();
    let handle = unsafe {
        horus_dynamic_pack(
            color.as_ptr(),
            color.len(),
            depth.as_ptr(),
            depth.len(),
            size[0],
            size[1],
            pose.flat().as_ptr(),
            sequence,
            near_m,
            far_m,
            position_range_m,
            &mut error,
        )
    };
    result_bytes(handle, error)
}
