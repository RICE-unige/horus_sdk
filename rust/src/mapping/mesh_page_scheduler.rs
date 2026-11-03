use super::{MeshOperation, MeshPageUpdate};
use std::{
    collections::{HashMap, VecDeque},
    sync::Arc,
};

#[derive(Clone, Debug)]
pub struct MeshSchedulerOptions {
    pub pages_per_second: f64,
    pub bytes_per_second: f64,
    pub max_pages: usize,
    pub max_payload_bytes: usize,
}
impl Default for MeshSchedulerOptions {
    fn default() -> Self {
        Self {
            pages_per_second: 30.0,
            bytes_per_second: 4.0 * 1024.0 * 1024.0,
            max_pages: 1024,
            max_payload_bytes: 1024 * 1024,
        }
    }
}

/// Caller-driven pacing with shared immutable pages; times are monotonic seconds.
pub struct MeshPageScheduler {
    options: MeshSchedulerOptions,
    epoch: Option<u64>,
    clear: Option<Arc<MeshPageUpdate>>,
    pages: HashMap<u32, Arc<MeshPageUpdate>>,
    order: VecDeque<u32>,
    pending_bytes: usize,
    page_tokens: f64,
    byte_tokens: f64,
    last_time: f64,
}
impl MeshPageScheduler {
    pub fn new(options: MeshSchedulerOptions, now: f64) -> Result<Self, String> {
        if !now.is_finite()
            || !options.pages_per_second.is_finite()
            || !options.bytes_per_second.is_finite()
            || options.pages_per_second <= 0.0
            || options.bytes_per_second <= 0.0
            || options.max_pages == 0
            || options.max_payload_bytes == 0
            || options
                .max_pages
                .checked_add(1)
                .and_then(|p| p.checked_mul(options.max_payload_bytes))
                .is_none()
        {
            return Err("Invalid mesh scheduler capacity, rate or clock".into());
        }
        Ok(Self {
            options,
            epoch: None,
            clear: None,
            pages: HashMap::new(),
            order: VecDeque::new(),
            pending_bytes: 0,
            page_tokens: 0.0,
            byte_tokens: 0.0,
            last_time: now,
        })
    }
    pub fn pending_count(&self) -> usize {
        self.pages.len() + usize::from(self.clear.is_some())
    }
    pub fn pending_bytes(&self) -> usize {
        self.pending_bytes
    }
    pub fn enqueue(&mut self, update: Arc<MeshPageUpdate>) -> Result<bool, String> {
        if update.payload.len() > self.options.max_payload_bytes {
            return Err("Mesh page exceeds scheduler payload capacity".into());
        }
        if self.epoch.is_some_and(|epoch| update.epoch < epoch) {
            return Ok(false);
        }
        let new_epoch = self.epoch.is_none_or(|epoch| update.epoch > epoch);
        if new_epoch {
            self.pages.clear();
            self.order.clear();
            self.clear = None;
            self.pending_bytes = 0;
            self.epoch = Some(update.epoch);
        }
        if update.operation == MeshOperation::Clear {
            if !new_epoch && !self.pages.is_empty() {
                return Ok(false);
            }
            if let Some(old) = &self.clear {
                self.pending_bytes -= old.payload.len();
            }
            self.pending_bytes += update.payload.len();
            self.clear = Some(update);
            return Ok(true);
        }
        if let Some(previous) = self.pages.get(&update.chunk_id) {
            if update.revision <= previous.revision {
                return Ok(false);
            }
            self.pending_bytes -= previous.payload.len();
        } else {
            if self.pages.len() >= self.options.max_pages {
                return Err("Mesh scheduler page capacity exceeded".into());
            }
            self.order.push_back(update.chunk_id);
        }
        self.pending_bytes += update.payload.len();
        self.pages.insert(update.chunk_id, update);
        Ok(true)
    }
    pub fn pop_ready(&mut self, now: f64) -> Result<Option<Arc<MeshPageUpdate>>, String> {
        if !now.is_finite() {
            return Err("Mesh scheduler clock must be finite".into());
        }
        let elapsed = (now - self.last_time).max(0.0);
        self.last_time = self.last_time.max(now);
        let update = self
            .clear
            .as_ref()
            .or_else(|| self.order.front().and_then(|id| self.pages.get(id)));
        self.page_tokens = (self.page_tokens + elapsed * self.options.pages_per_second).min(4.0);
        let bytes = update.map_or(0, |page| page.payload.len());
        let capacity = (self.options.bytes_per_second * 0.1).max(bytes as f64);
        self.byte_tokens =
            (self.byte_tokens + elapsed * self.options.bytes_per_second).min(capacity);
        if update.is_none() || self.page_tokens < 1.0 || self.byte_tokens < bytes as f64 {
            return Ok(None);
        }
        self.page_tokens -= 1.0;
        self.byte_tokens -= bytes as f64;
        self.pending_bytes -= bytes;
        Ok(self
            .clear
            .take()
            .or_else(|| self.order.pop_front().and_then(|id| self.pages.remove(&id))))
    }
}
