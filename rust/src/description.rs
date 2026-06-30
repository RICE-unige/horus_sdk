//! Native visual-mesh baking for robot descriptions (STL).
//!
//! Parses binary or ASCII STL meshes referenced by a URDF's `<visual>`
//! elements into a deduplicated indexed mesh and packs it into a `MeshAsset`
//! (base64 float/int buffers), matching the shape of the Python
//! `RobotDescriptionModels.MeshAsset`.
//!
//! Scope: STL only. The Python baker additionally handles DAE/OBJ (via external
//! converters), decimation, and `package://` resolution via a mesh root; those
//! remain Python-only (see the Implementation Status docs). Mesh references are
//! resolved relative to the URDF, via `file://`, or as absolute paths.

use once_cell::sync::Lazy;
use regex::Regex;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

const BASE64_ALPHABET: &[u8; 64] =
    b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

/// Standard base64 encoding (matches Python base64.b64encode).
pub fn base64_encode(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for chunk in data.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = *chunk.get(1).unwrap_or(&0) as u32;
        let b2 = *chunk.get(2).unwrap_or(&0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        out.push(BASE64_ALPHABET[((n >> 18) & 63) as usize] as char);
        out.push(BASE64_ALPHABET[((n >> 12) & 63) as usize] as char);
        out.push(if chunk.len() > 1 {
            BASE64_ALPHABET[((n >> 6) & 63) as usize] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            BASE64_ALPHABET[(n & 63) as usize] as char
        } else {
            '='
        });
    }
    out
}

/// A baked, indexed mesh ready for the registration payload.
#[derive(Debug, Clone)]
pub struct MeshAsset {
    pub mesh_id: String,
    pub vertex_count: usize,
    pub triangle_count: usize,
    pub positions_b64: String,
    pub normals_b64: String,
    pub indices_b64: String,
    pub bounds_min: [f32; 3],
    pub bounds_max: [f32; 3],
}

impl MeshAsset {
    pub fn to_value(&self) -> Value {
        json!({
            "mesh_id": self.mesh_id,
            "vertex_count": self.vertex_count,
            "triangle_count": self.triangle_count,
            "positions_b64": self.positions_b64,
            "normals_b64": self.normals_b64,
            "indices_b64": self.indices_b64,
            "bounds_min": self.bounds_min,
            "bounds_max": self.bounds_max,
        })
    }

    pub fn encoded_bytes(&self) -> usize {
        self.positions_b64.len() + self.normals_b64.len() + self.indices_b64.len()
    }
}

struct Triangle {
    normal: [f32; 3],
    verts: [[f32; 3]; 3],
}

fn is_binary_stl(bytes: &[u8]) -> bool {
    if bytes.len() < 84 {
        return false;
    }
    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    bytes.len() == 84 + count * 50
}

fn read_f32(bytes: &[u8], offset: usize) -> f32 {
    f32::from_le_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

fn parse_binary_stl(bytes: &[u8]) -> Vec<Triangle> {
    let count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
    let mut triangles = Vec::with_capacity(count);
    let mut offset = 84;
    for _ in 0..count {
        let normal = [
            read_f32(bytes, offset),
            read_f32(bytes, offset + 4),
            read_f32(bytes, offset + 8),
        ];
        let v0 = [
            read_f32(bytes, offset + 12),
            read_f32(bytes, offset + 16),
            read_f32(bytes, offset + 20),
        ];
        let v1 = [
            read_f32(bytes, offset + 24),
            read_f32(bytes, offset + 28),
            read_f32(bytes, offset + 32),
        ];
        let v2 = [
            read_f32(bytes, offset + 36),
            read_f32(bytes, offset + 40),
            read_f32(bytes, offset + 44),
        ];
        triangles.push(Triangle {
            normal,
            verts: [v0, v1, v2],
        });
        offset += 50;
    }
    triangles
}

fn parse_ascii_stl(bytes: &[u8]) -> Vec<Triangle> {
    let text = match std::str::from_utf8(bytes) {
        Ok(text) => text,
        Err(_) => return Vec::new(),
    };
    let tokens: Vec<&str> = text.split_whitespace().collect();
    let mut triangles = Vec::new();
    let mut i = 0;
    while i < tokens.len() {
        if tokens[i] == "facet" && i + 4 < tokens.len() && tokens[i + 1] == "normal" {
            let normal = [
                tokens[i + 2].parse().unwrap_or(0.0),
                tokens[i + 3].parse().unwrap_or(0.0),
                tokens[i + 4].parse().unwrap_or(0.0),
            ];
            let mut verts = [[0.0f32; 3]; 3];
            let mut vert_index = 0;
            let mut j = i + 5;
            while j < tokens.len() && tokens[j] != "endfacet" && vert_index < 3 {
                if tokens[j] == "vertex" && j + 3 < tokens.len() {
                    verts[vert_index] = [
                        tokens[j + 1].parse().unwrap_or(0.0),
                        tokens[j + 2].parse().unwrap_or(0.0),
                        tokens[j + 3].parse().unwrap_or(0.0),
                    ];
                    vert_index += 1;
                    j += 4;
                } else {
                    j += 1;
                }
            }
            if vert_index == 3 {
                triangles.push(Triangle { normal, verts });
            }
            i = j;
        } else {
            i += 1;
        }
    }
    triangles
}

fn face_normal(verts: &[[f32; 3]; 3]) -> [f32; 3] {
    let u = [
        verts[1][0] - verts[0][0],
        verts[1][1] - verts[0][1],
        verts[1][2] - verts[0][2],
    ];
    let v = [
        verts[2][0] - verts[0][0],
        verts[2][1] - verts[0][1],
        verts[2][2] - verts[0][2],
    ];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 1e-12 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0.0, 0.0, 1.0]
    }
}

fn bake_triangles(triangles: &[Triangle], mesh_id: impl Into<String>) -> Option<MeshAsset> {
    if triangles.is_empty() {
        return None;
    }
    let mut positions: Vec<f32> = Vec::new();
    let mut normals: Vec<f32> = Vec::new();
    let mut indices: Vec<i32> = Vec::new();
    let mut lookup: HashMap<[u32; 6], i32> = HashMap::new();
    let mut bounds_min = [f32::INFINITY; 3];
    let mut bounds_max = [f32::NEG_INFINITY; 3];

    for triangle in triangles {
        let normal = if triangle.normal.iter().all(|c| *c == 0.0) {
            face_normal(&triangle.verts)
        } else {
            triangle.normal
        };
        for vertex in &triangle.verts {
            let key = [
                vertex[0].to_bits(),
                vertex[1].to_bits(),
                vertex[2].to_bits(),
                normal[0].to_bits(),
                normal[1].to_bits(),
                normal[2].to_bits(),
            ];
            let index = *lookup.entry(key).or_insert_with(|| {
                let id = (positions.len() / 3) as i32;
                positions.extend_from_slice(vertex);
                normals.extend_from_slice(&normal);
                id
            });
            indices.push(index);
            for axis in 0..3 {
                bounds_min[axis] = bounds_min[axis].min(vertex[axis]);
                bounds_max[axis] = bounds_max[axis].max(vertex[axis]);
            }
        }
    }

    let mut position_bytes = Vec::with_capacity(positions.len() * 4);
    for value in &positions {
        position_bytes.extend_from_slice(&value.to_le_bytes());
    }
    let mut normal_bytes = Vec::with_capacity(normals.len() * 4);
    for value in &normals {
        normal_bytes.extend_from_slice(&value.to_le_bytes());
    }
    let mut index_bytes = Vec::with_capacity(indices.len() * 4);
    for value in &indices {
        index_bytes.extend_from_slice(&value.to_le_bytes());
    }

    Some(MeshAsset {
        mesh_id: mesh_id.into(),
        vertex_count: positions.len() / 3,
        triangle_count: triangles.len(),
        positions_b64: base64_encode(&position_bytes),
        normals_b64: base64_encode(&normal_bytes),
        indices_b64: base64_encode(&index_bytes),
        bounds_min,
        bounds_max,
    })
}

/// Parse and bake an STL file into a MeshAsset.
pub fn bake_stl_file(path: &Path, mesh_id: impl Into<String>) -> Option<MeshAsset> {
    let bytes = std::fs::read(path).ok()?;
    let triangles = if is_binary_stl(&bytes) {
        parse_binary_stl(&bytes)
    } else {
        parse_ascii_stl(&bytes)
    };
    bake_triangles(&triangles, mesh_id)
}

fn resolve_mesh_path(filename: &str, urdf_path: &str) -> Option<PathBuf> {
    let trimmed = filename.trim();
    if let Some(rest) = trimmed.strip_prefix("file://") {
        return Some(PathBuf::from(rest));
    }
    if trimmed.starts_with("package://") {
        // package:// resolution needs a mesh root / ament index (Python only).
        return None;
    }
    let candidate = Path::new(trimmed);
    if candidate.is_absolute() {
        return Some(candidate.to_path_buf());
    }
    Path::new(urdf_path)
        .parent()
        .map(|dir| dir.join(trimmed))
}

static VISUAL_BLOCK: Lazy<Option<Regex>> =
    Lazy::new(|| Regex::new(r#"<visual\b[\s\S]*?</visual>"#).ok());
static MESH_FILENAME: Lazy<Option<Regex>> =
    Lazy::new(|| Regex::new(r#"<mesh\b[^>]*filename\s*=\s*["']([^"']+)["']"#).ok());

/// Bake every resolvable STL mesh referenced by the URDF's visual elements.
pub fn bake_visual_meshes(urdf: &str, urdf_path: &str) -> Vec<MeshAsset> {
    let (Some(visual_re), Some(mesh_re)) = (VISUAL_BLOCK.as_ref(), MESH_FILENAME.as_ref()) else {
        return Vec::new();
    };
    let mut assets = Vec::new();
    for (block_index, block) in visual_re.find_iter(urdf).enumerate() {
        for capture in mesh_re.captures_iter(block.as_str()) {
            let filename = &capture[1];
            if !filename.to_ascii_lowercase().ends_with(".stl") {
                continue;
            }
            let Some(path) = resolve_mesh_path(filename, urdf_path) else {
                continue;
            };
            let stem = Path::new(filename)
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("mesh");
            let mesh_id = format!("{stem}_{block_index}");
            if let Some(asset) = bake_stl_file(&path, mesh_id) {
                assets.push(asset);
            }
        }
    }
    assets
}
