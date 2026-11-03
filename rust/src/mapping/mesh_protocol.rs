//! HMSH v2 byte encoding, with one reserved output allocation and no mesh resampling.
#[derive(Clone, Debug, Default)]
pub struct BlockGeometry {
    pub vertices: Vec<[f32; 3]>,
    pub colors: Vec<[u8; 4]>,
    pub triangles: Vec<u32>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MeshOperation {
    Upsert = 0,
    Delete = 1,
    Clear = 2,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LiveHeader {
    pub chunk_id: u32,
    pub page_key: [i32; 3],
    pub epoch: u64,
    pub revision: u64,
    pub operation: MeshOperation,
    pub has_color: bool,
    pub vertex_count: u32,
    pub index_count: u32,
}
pub const HMSH_V2_HEADER_SIZE: usize = 52;

pub fn encode_live_payload(
    chunk_id: u32,
    page_key: [i32; 3],
    epoch: u64,
    revision: u64,
    operation: MeshOperation,
    geometry: Option<&BlockGeometry>,
    max_payload_bytes: usize,
) -> Result<Vec<u8>, String> {
    let (vertices, indices, color) = geometry.map_or((0, 0, false), |g| {
        (
            g.vertices.len(),
            g.triangles.len(),
            g.colors.len() == g.vertices.len(),
        )
    });
    if vertices > u32::MAX as usize || indices > u32::MAX as usize {
        return Err("HMSH counts exceed wire range".into());
    }
    if let Some(g) = geometry {
        if operation != MeshOperation::Upsert
            || g.triangles.len() % 3 != 0
            || g.triangles.iter().any(|i| *i as usize >= vertices)
            || g.vertices.iter().flatten().any(|v| !v.is_finite())
            || (!g.colors.is_empty() && !color)
        {
            return Err("Invalid HMSH geometry".into());
        }
    }
    let size = vertices
        .checked_mul(if color { 16 } else { 12 })
        .and_then(|v| indices.checked_mul(4).and_then(|i| v.checked_add(i)))
        .and_then(|v| v.checked_add(HMSH_V2_HEADER_SIZE))
        .ok_or("HMSH size overflow")?;
    if size > max_payload_bytes {
        return Err("HMSH page exceeds budget; split pages without reducing map quality".into());
    }
    let mut out = Vec::with_capacity(size);
    out.extend_from_slice(&0x484d5348u32.to_le_bytes());
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&chunk_id.to_le_bytes());
    for value in page_key {
        out.extend_from_slice(&value.to_le_bytes());
    }
    out.extend_from_slice(&epoch.to_le_bytes());
    out.extend_from_slice(&revision.to_le_bytes());
    out.extend_from_slice(&[operation as u8, u8::from(color), 0, 0]);
    out.extend_from_slice(&(vertices as u32).to_le_bytes());
    out.extend_from_slice(&(indices as u32).to_le_bytes());
    if let Some(g) = geometry {
        for (i, vertex) in g.vertices.iter().enumerate() {
            for value in vertex {
                out.extend_from_slice(&value.to_le_bytes());
            }
            if color {
                out.extend_from_slice(&g.colors[i]);
            }
        }
        for index in &g.triangles {
            out.extend_from_slice(&index.to_le_bytes());
        }
    }
    Ok(out)
}
pub fn decode_live_header(data: &[u8]) -> Result<LiveHeader, String> {
    if data.len() < HMSH_V2_HEADER_SIZE {
        return Err("Truncated HMSH header".into());
    }
    let u32_at = |n| u32::from_le_bytes(data[n..n + 4].try_into().unwrap());
    let u64_at = |n| u64::from_le_bytes(data[n..n + 8].try_into().unwrap());
    if u32_at(0) != 0x484d5348 || u32_at(4) != 2 {
        return Err("Not HMSH v2".into());
    }
    let operation = match data[40] {
        0 => MeshOperation::Upsert,
        1 => MeshOperation::Delete,
        2 => MeshOperation::Clear,
        _ => return Err("Invalid HMSH operation".into()),
    };
    if data[41] > 1 || data[42] != 0 || data[43] != 0 {
        return Err("Invalid HMSH flags".into());
    }
    Ok(LiveHeader {
        chunk_id: u32_at(8),
        page_key: [u32_at(12) as i32, u32_at(16) as i32, u32_at(20) as i32],
        epoch: u64_at(24),
        revision: u64_at(32),
        operation,
        has_color: data[41] != 0,
        vertex_count: u32_at(44),
        index_count: u32_at(48),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn wire_bytes_match_python_and_roundtrip() {
        let geometry = BlockGeometry {
            vertices: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
            colors: vec![[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 255]],
            triangles: vec![0, 1, 2],
        };
        let bytes = encode_live_payload(
            7,
            [-2, 3, 0],
            11,
            12,
            MeshOperation::Upsert,
            Some(&geometry),
            1024,
        )
        .unwrap();
        let expected = include_str!("fixtures/hmsh_v2_triangle.hex").trim();
        let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
        assert_eq!(hex, expected);
        let header = decode_live_header(&bytes).unwrap();
        assert_eq!(header.page_key, [-2, 3, 0]);
        assert_eq!(header.vertex_count, 3);
        assert!(header.has_color);
        assert!(
            encode_live_payload(7, [0; 3], 0, 0, MeshOperation::Upsert, Some(&geometry), 100)
                .is_err()
        );
        let mut invalid = geometry.clone();
        invalid.triangles[0] = 3;
        assert!(
            encode_live_payload(7, [0; 3], 0, 0, MeshOperation::Upsert, Some(&invalid), 1024)
                .is_err()
        );
        assert!(decode_live_header(&bytes[..51]).is_err());
        let clear = encode_live_payload(0, [0; 3], 1, 0, MeshOperation::Clear, None, 52).unwrap();
        assert_eq!(clear.len(), 52);
        assert_eq!(
            decode_live_header(&clear).unwrap().operation,
            MeshOperation::Clear
        );
    }
}
