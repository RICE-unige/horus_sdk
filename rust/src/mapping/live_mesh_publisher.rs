use super::{
    BlockGeometry, LiveMeshOptions, LiveMeshState, MeshPageScheduler, MeshPageUpdate,
    MeshSchedulerOptions,
};
use std::{collections::HashMap, sync::Arc, time::Instant};

#[derive(Clone, Debug)]
pub struct LiveMeshPublisherOptions {
    pub topic: String,
    pub map: LiveMeshOptions,
    pub pages_per_second: f64,
    pub bytes_per_second: f64,
}
impl Default for LiveMeshPublisherOptions {
    fn default() -> Self {
        Self {
            topic: "/horus/maps/live".into(),
            map: LiveMeshOptions::default(),
            pages_per_second: 60.0,
            bytes_per_second: 4.0 * 1024.0 * 1024.0,
        }
    }
}

/// Caller-driven ROS publisher. Geometry must already be paged in `map.frame_id`.
/// `pump` services bounded repair/publication work; no global ROS context is owned.
pub struct LiveMeshPublisher {
    state: LiveMeshState,
    queue: MeshPageScheduler,
    transport: native::Transport,
    start: Instant,
    last_manifest: f64,
    sent: HashMap<u32, (u64, u64, f64)>,
    dropped_requests: u64,
}
impl LiveMeshPublisher {
    pub fn new(options: LiveMeshPublisherOptions) -> Result<Self, String> {
        let state = LiveMeshState::new(options.map.clone())?;
        let queue = MeshPageScheduler::new(
            MeshSchedulerOptions {
                pages_per_second: options.pages_per_second,
                bytes_per_second: options.bytes_per_second,
                max_pages: options.map.max_pages as usize,
                max_payload_bytes: options.map.max_payload_bytes,
            },
            0.0,
        )?;
        let transport = native::Transport::new(&options.topic, options.map.max_payload_bytes)?;
        Ok(Self {
            state,
            queue,
            transport,
            start: Instant::now(),
            last_manifest: f64::NEG_INFINITY,
            sent: HashMap::new(),
            dropped_requests: 0,
        })
    }
    pub fn state(&self) -> &LiveMeshState {
        &self.state
    }
    pub fn pending_count(&self) -> usize {
        self.queue.pending_count()
    }
    pub fn dropped_repair_requests(&self) -> u64 {
        self.dropped_requests
    }
    fn enqueue(&mut self, updates: Vec<Arc<MeshPageUpdate>>) -> Result<usize, String> {
        let count = updates.len();
        for update in updates {
            self.queue.enqueue(update)?;
        }
        Ok(count)
    }
    pub fn replace(
        &mut self,
        key: &str,
        geometry: Option<&BlockGeometry>,
    ) -> Result<usize, String> {
        let updates = self.state.replace(key, geometry)?;
        self.enqueue(updates)
    }
    pub fn replace_many(
        &mut self,
        changes: &[(&str, Option<&BlockGeometry>)],
    ) -> Result<usize, String> {
        let updates = self.state.replace_many(changes.iter().copied())?;
        self.enqueue(updates)
    }
    pub fn clear(&mut self) -> Result<(), String> {
        self.queue.enqueue(self.state.clear()?)?;
        self.sent.clear();
        self.last_manifest = f64::NEG_INFINITY;
        Ok(())
    }
    pub fn pump(&mut self) -> Result<usize, String> {
        let now = self.start.elapsed().as_secs_f64();
        let status = self.transport.poll()?;
        if let Some(error) = status["error"].as_str().filter(|e| !e.is_empty()) {
            return Err(error.into());
        }
        self.dropped_requests = status["dropped_requests"]
            .as_u64()
            .ok_or("Missing repair diagnostics")?;
        if let Some(request) = status["request"].as_str() {
            let request = serde_json::from_str(request).unwrap_or(serde_json::Value::Null);
            for update in self.state.repair_updates(&request) {
                if self.sent.get(&update.chunk_id).is_some_and(|old| {
                    old.0 == update.epoch && old.1 == update.revision && now - old.2 < 2.0
                }) {
                    continue;
                }
                self.queue.enqueue(update)?;
            }
        }
        if now - self.last_manifest >= 2.0 {
            self.transport
                .manifest(&self.state.manifest().to_string())?;
            self.last_manifest = now;
        }
        if status["subscribers"].as_u64().unwrap_or(0) == 0 {
            return Ok(0);
        }
        let mut count = 0;
        while count < 4 {
            let Some(update) = self.queue.pop_ready(now)? else {
                break;
            };
            if let Err(error) = self.transport.publish(&update.payload) {
                self.queue.enqueue(update)?;
                return Err(error);
            }
            self.sent
                .insert(update.chunk_id, (update.epoch, update.revision, now));
            count += 1;
        }
        Ok(count)
    }
}

#[cfg(feature = "ros2")]
mod native {
    use serde_json::Value;
    use std::{
        ffi::{c_char, c_void, CStr, CString},
        ptr::NonNull,
    };
    unsafe extern "C" {
        fn horus_native_mesh_create(
            topic: *const c_char,
            limit: usize,
            error: *mut *mut c_char,
        ) -> *mut c_void;
        fn horus_native_mesh_publish(
            handle: *mut c_void,
            data: *const u8,
            size: usize,
            error: *mut *mut c_char,
        ) -> i32;
        fn horus_native_mesh_manifest(
            handle: *mut c_void,
            json: *const c_char,
            error: *mut *mut c_char,
        ) -> i32;
        fn horus_native_mesh_poll(handle: *mut c_void) -> *mut c_char;
        fn horus_native_mesh_destroy(handle: *mut c_void);
        fn horus_native_free(text: *mut c_char);
    }
    pub struct Transport(NonNull<c_void>);
    // The native runtime owns its executor thread and synchronizes ROS access.
    // Moving the sole owner is safe; &mut methods prevent concurrent pump/drop.
    unsafe impl Send for Transport {}
    fn take(pointer: *mut c_char) -> Result<String, String> {
        if pointer.is_null() {
            return Err("Native mesh transport returned no result".into());
        }
        // Owned C-ABI result. Release even if UTF-8 validation fails.
        unsafe {
            let result = CStr::from_ptr(pointer)
                .to_str()
                .map(str::to_owned)
                .map_err(|e| e.to_string());
            horus_native_free(pointer);
            result
        }
    }
    fn check(result: i32, error: *mut c_char) -> Result<(), String> {
        if result == 1 {
            if !error.is_null() {
                let _ = take(error);
            }
            Ok(())
        } else {
            Err(take(error).unwrap_or_else(|e| e))
        }
    }
    impl Transport {
        pub fn new(topic: &str, limit: usize) -> Result<Self, String> {
            let topic = CString::new(topic).map_err(|_| "Map topic contains NUL")?;
            let mut error = std::ptr::null_mut();
            let handle = unsafe { horus_native_mesh_create(topic.as_ptr(), limit, &mut error) };
            NonNull::new(handle)
                .map(Self)
                .ok_or_else(|| take(error).unwrap_or_else(|e| e))
        }
        pub fn publish(&mut self, bytes: &[u8]) -> Result<(), String> {
            let mut error = std::ptr::null_mut();
            let result = unsafe {
                horus_native_mesh_publish(self.0.as_ptr(), bytes.as_ptr(), bytes.len(), &mut error)
            };
            check(result, error)
        }
        pub fn manifest(&mut self, text: &str) -> Result<(), String> {
            let value = CString::new(text).map_err(|_| "Manifest contains NUL")?;
            let mut error = std::ptr::null_mut();
            let result =
                unsafe { horus_native_mesh_manifest(self.0.as_ptr(), value.as_ptr(), &mut error) };
            check(result, error)
        }
        pub fn poll(&mut self) -> Result<Value, String> {
            let text = take(unsafe { horus_native_mesh_poll(self.0.as_ptr()) })?;
            serde_json::from_str(&text).map_err(|e| e.to_string())
        }
    }
    impl Drop for Transport {
        fn drop(&mut self) {
            unsafe { horus_native_mesh_destroy(self.0.as_ptr()) }
        }
    }
}
#[cfg(not(feature = "ros2"))]
mod native {
    pub struct Transport;
    impl Transport {
        pub fn new(_: &str, _: usize) -> Result<Self, String> {
            Err("Enable ros2 to publish a live mesh".into())
        }
        pub fn publish(&mut self, _: &[u8]) -> Result<(), String> {
            unreachable!()
        }
        pub fn manifest(&mut self, _: &str) -> Result<(), String> {
            unreachable!()
        }
        pub fn poll(&mut self) -> Result<serde_json::Value, String> {
            unreachable!()
        }
    }
}
