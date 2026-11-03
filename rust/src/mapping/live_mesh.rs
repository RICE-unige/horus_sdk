//! Bounded map state with exact deduplication and immutable shared repair pages.
use super::{encode_live_payload, BlockGeometry, MeshOperation, HMSH_V2_HEADER_SIZE};
use serde_json::{json, Value};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
    time::{SystemTime, UNIX_EPOCH},
};

#[derive(Clone, Debug)]
pub struct MeshPageUpdate {
    pub chunk_id: u32,
    pub epoch: u64,
    pub revision: u64,
    pub operation: MeshOperation,
    pub payload: Arc<[u8]>,
    pub vertex_count: usize,
    pub triangle_count: usize,
}
#[derive(Clone, Debug)]
pub struct LiveMeshOptions {
    pub frame_id: String,
    pub max_pages: u32,
    pub max_payload_bytes: usize,
    pub max_cache_bytes: usize,
    pub initial_epoch: Option<u64>,
}
impl Default for LiveMeshOptions {
    fn default() -> Self {
        Self {
            frame_id: "map".into(),
            max_pages: 1024,
            max_payload_bytes: 1024 * 1024,
            max_cache_bytes: 256 * 1024 * 1024,
            initial_epoch: None,
        }
    }
}
#[derive(Clone)]
pub struct LiveMeshState {
    options: LiveMeshOptions,
    epoch: u64,
    keys: BTreeMap<String, u32>,
    current: BTreeMap<u32, Arc<MeshPageUpdate>>,
    free: BTreeSet<u32>,
    next_id: u32,
    cached_bytes: usize,
    unchanged_pages: u64,
    suppressed_bytes: u64,
}
impl LiveMeshState {
    pub fn new(options: LiveMeshOptions) -> Result<Self, String> {
        if options.frame_id.is_empty()
            || !(1..=1024).contains(&options.max_pages)
            || options.max_payload_bytes < HMSH_V2_HEADER_SIZE
            || options.max_cache_bytes < options.max_payload_bytes
        {
            return Err("Invalid live mesh frame or capacity limits".into());
        }
        let epoch = match options.initial_epoch {
            Some(value) => value,
            None => SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_err(|e| e.to_string())?
                .as_nanos()
                .try_into()
                .map_err(|_| "Epoch exceeds uint64")?,
        };
        if epoch == u64::MAX {
            return Err("Map epoch exhausted".into());
        }
        Ok(Self {
            options,
            epoch,
            keys: BTreeMap::new(),
            current: BTreeMap::new(),
            free: BTreeSet::new(),
            next_id: 1,
            cached_bytes: 0,
            unchanged_pages: 0,
            suppressed_bytes: 0,
        })
    }
    pub fn epoch(&self) -> u64 {
        self.epoch
    }
    pub fn page_count(&self) -> usize {
        self.keys.len()
    }
    pub fn cached_bytes(&self) -> usize {
        self.cached_bytes
    }
    pub fn unchanged_pages(&self) -> u64 {
        self.unchanged_pages
    }
    pub fn suppressed_bytes(&self) -> u64 {
        self.suppressed_bytes
    }

    pub fn replace(
        &mut self,
        key: &str,
        geometry: Option<&BlockGeometry>,
    ) -> Result<Vec<Arc<MeshPageUpdate>>, String> {
        if let Some(g) = geometry {
            if g.vertices.is_empty() || g.triangles.is_empty() {
                return Err("Incomplete mesh geometry".into());
            }
        }
        let existing = self.keys.get(key).copied();
        if geometry.is_none() && existing.is_none() {
            return Ok(Vec::new());
        }
        let id = existing.unwrap_or_else(|| self.free.first().copied().unwrap_or(self.next_id));
        if id > self.options.max_pages {
            return Err("Mesh page capacity exceeded; previous map retained".into());
        }
        let old = self.current.get(&id);
        let revision = old
            .map_or(0, |u| u.revision)
            .checked_add(1)
            .ok_or("Revision exhausted; clear the map")?;
        let operation = if geometry.is_some() {
            MeshOperation::Upsert
        } else {
            MeshOperation::Delete
        };
        let mut payload = encode_live_payload(
            id,
            [0; 3],
            self.epoch,
            old.map_or(revision, |u| u.revision),
            operation,
            geometry,
            self.options.max_payload_bytes,
        )?;
        if old.is_some_and(|u| u.payload.as_ref() == payload.as_slice()) {
            self.unchanged_pages = self.unchanged_pages.saturating_add(1);
            self.suppressed_bytes = self.suppressed_bytes.saturating_add(payload.len() as u64);
            return Ok(Vec::new());
        }
        payload[32..40].copy_from_slice(&revision.to_le_bytes());
        let bytes = self.cached_bytes - old.map_or(0, |u| u.payload.len());
        let size = bytes
            .checked_add(payload.len())
            .ok_or("Mesh cache size overflow")?;
        if size > self.options.max_cache_bytes {
            return Err("Mesh cache capacity exceeded; previous map retained".into());
        }
        let update = Arc::new(MeshPageUpdate {
            chunk_id: id,
            epoch: self.epoch,
            revision,
            operation,
            payload: payload.into(),
            vertex_count: geometry.map_or(0, |g| g.vertices.len()),
            triangle_count: geometry.map_or(0, |g| g.triangles.len() / 3),
        });
        if existing.is_none() {
            if !self.free.remove(&id) {
                self.next_id += 1;
            }
            self.keys.insert(key.into(), id);
        }
        if geometry.is_none() {
            self.keys.remove(key);
            self.free.insert(id);
        }
        self.current.insert(id, update.clone());
        self.cached_bytes = size;
        Ok(vec![update])
    }
    /// Atomic batch: copy bounded metadata, share unchanged page buffers.
    pub fn replace_many<'a>(
        &mut self,
        changes: impl IntoIterator<Item = (&'a str, Option<&'a BlockGeometry>)>,
    ) -> Result<Vec<Arc<MeshPageUpdate>>, String> {
        let mut staged = self.clone();
        let mut updates = Vec::new();
        for (key, geometry) in changes {
            updates.extend(staged.replace(key, geometry)?);
        }
        *self = staged;
        Ok(updates)
    }
    pub fn snapshot_clear_update(&self) -> Arc<MeshPageUpdate> {
        Arc::new(MeshPageUpdate {
            chunk_id: 0,
            epoch: self.epoch,
            revision: 0,
            operation: MeshOperation::Clear,
            payload: encode_live_payload(
                0,
                [0; 3],
                self.epoch,
                0,
                MeshOperation::Clear,
                None,
                self.options.max_payload_bytes,
            )
            .expect("validated header budget")
            .into(),
            vertex_count: 0,
            triangle_count: 0,
        })
    }
    pub fn clear(&mut self) -> Result<Arc<MeshPageUpdate>, String> {
        self.epoch = self.epoch.checked_add(1).ok_or("Map epoch exhausted")?;
        self.keys.clear();
        self.current.clear();
        self.free.clear();
        self.next_id = 1;
        self.cached_bytes = 0;
        Ok(self.snapshot_clear_update())
    }
    pub fn snapshot_updates(&self) -> Vec<Arc<MeshPageUpdate>> {
        self.current.values().cloned().collect()
    }
    pub fn manifest(&self) -> Value {
        json!({"version":1,"epoch":self.epoch.to_string(),"frame_id":self.options.frame_id,
            "pages":self.current.values().map(|u| json!({"chunk_id":u.chunk_id,
                "revision":u.revision.to_string(),"operation":u.operation as u8})).collect::<Vec<_>>(),
            "triangles":self.current.values().map(|u|u.triangle_count).sum::<usize>()})
    }
    pub fn repair_updates(&self, request: &Value) -> Vec<Arc<MeshPageUpdate>> {
        let kind = request.get("request").and_then(Value::as_str);
        if !matches!(kind, Some("snapshot" | "pages")) {
            return Vec::new();
        }
        if self.current.is_empty() {
            return vec![self.snapshot_clear_update()];
        }
        let epoch = request.get("epoch").and_then(|v| {
            v.as_str()
                .map(str::to_owned)
                .or_else(|| v.as_u64().map(|n| n.to_string()))
        });
        if kind == Some("snapshot") || epoch.as_deref() != Some(&self.epoch.to_string()) {
            return self.snapshot_updates();
        }
        let Some(ids) = request.get("ids").and_then(Value::as_array) else {
            return Vec::new();
        };
        if ids.len() > self.options.max_pages as usize {
            return Vec::new();
        }
        let mut selected = BTreeSet::new();
        for value in ids {
            let Some(id) = value
                .as_u64()
                .filter(|v| (1..=u64::from(self.options.max_pages)).contains(v))
            else {
                return Vec::new();
            };
            selected.insert(id as u32);
        }
        selected
            .into_iter()
            .filter_map(|id| self.current.get(&id).cloned())
            .collect()
    }
}
