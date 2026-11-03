//! Full-resolution native scene import. Geometry is borrowed from an owned native
//! asset, so uploading a large map does not create an additional Rust-side copy.
use super::gpu::MeshInput;
use anyhow::{anyhow, ensure, Result};
use std::ffi::{c_char, c_void, CStr, CString};
use std::path::Path;
use std::ptr::NonNull;

unsafe extern "C" {
    fn horus_scene_load(
        path: *const c_char,
        scale: f64,
        vertices: u64,
        triangles: u64,
        pixels: u64,
        lighting: i32,
        error: *mut *mut c_char,
    ) -> *mut c_void;
    fn horus_scene_destroy(scene: *mut c_void);
    fn horus_scene_array(scene: *const c_void, kind: i32, count: *mut usize) -> *const c_void;
    fn horus_scene_texture_size(scene: *const c_void, height: i32) -> u32;
    fn horus_teach_free(error: *mut c_char);
}
#[derive(Clone, Copy, Debug)]
pub struct SceneLoadOptions {
    pub world_scale: f64,
    pub maximum_vertices: u64,
    pub maximum_triangles: u64,
    pub maximum_texture_pixels: u64,
    pub bake_lighting: bool,
}
impl Default for SceneLoadOptions {
    fn default() -> Self {
        Self {
            world_scale: 1.0,
            maximum_vertices: 100_000_000,
            maximum_triangles: 50_000_000,
            maximum_texture_pixels: 64 * 1024 * 1024,
            bake_lighting: true,
        }
    }
}
pub struct SceneAsset {
    pub(super) handle: NonNull<c_void>,
}
// The imported buffers are immutable, owned by this handle and freed only on drop.
unsafe impl Send for SceneAsset {}
unsafe impl Sync for SceneAsset {}
impl SceneAsset {
    pub fn load(path: impl AsRef<Path>, options: SceneLoadOptions) -> Result<Self> {
        let path = CString::new(
            path.as_ref()
                .to_str()
                .ok_or_else(|| anyhow!("Scene path is not UTF-8"))?,
        )?;
        let mut error = std::ptr::null_mut();
        let p = unsafe {
            horus_scene_load(
                path.as_ptr(),
                options.world_scale,
                options.maximum_vertices,
                options.maximum_triangles,
                options.maximum_texture_pixels,
                options.bake_lighting as i32,
                &mut error,
            )
        };
        let handle = NonNull::new(p).ok_or_else(|| {
            if error.is_null() {
                return anyhow!("Native scene import failed");
            }
            let message = unsafe { CStr::from_ptr(error).to_string_lossy().into_owned() };
            unsafe { horus_teach_free(error) };
            anyhow!(message)
        })?;
        Ok(Self { handle })
    }
    // Private: each accessor below selects the matching native element type.
    fn array<T>(&self, kind: i32) -> &[T] {
        let mut count = 0;
        let ptr = unsafe { horus_scene_array(self.handle.as_ptr(), kind, &mut count) };
        if count == 0 {
            return &[];
        }
        assert!(
            !ptr.is_null(),
            "Native scene returned a null nonempty buffer"
        );
        unsafe { std::slice::from_raw_parts(ptr.cast::<T>(), count) }
    }
    pub fn vertices(&self) -> &[[f32; 3]] {
        self.array(0)
    }
    pub fn point_colors(&self) -> &[[u8; 3]] {
        self.array(1)
    }
    pub fn faces(&self) -> &[[u32; 3]] {
        self.array(2)
    }
    pub fn face_colors(&self) -> &[[u8; 3]] {
        self.array(3)
    }
    pub fn uvs(&self) -> &[[f32; 2]] {
        self.array(4)
    }
    pub fn texture_rects(&self) -> &[[u32; 4]] {
        self.array(5)
    }
    pub fn texture_atlas(&self) -> &[[u8; 3]] {
        self.array(6)
    }
    pub fn texture_width(&self) -> u32 {
        unsafe { horus_scene_texture_size(self.handle.as_ptr(), 0) }
    }
    pub fn texture_height(&self) -> u32 {
        unsafe { horus_scene_texture_size(self.handle.as_ptr(), 1) }
    }
    pub fn mesh_input(&self) -> Result<MeshInput<'_>> {
        ensure!(
            !self.faces().is_empty(),
            "Point-only scene has no triangle mesh"
        );
        Ok(MeshInput {
            vertices: self.vertices(),
            faces: self.faces(),
            colors: self.face_colors(),
            uvs: Some(self.uvs()),
            texture_rects: Some(self.texture_rects()),
            texture_atlas: Some(self.texture_atlas()),
            texture_width: self.texture_width(),
            texture_height: self.texture_height(),
        })
    }
}
impl Drop for SceneAsset {
    fn drop(&mut self) {
        unsafe { horus_scene_destroy(self.handle.as_ptr()) }
    }
}
