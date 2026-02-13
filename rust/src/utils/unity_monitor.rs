use parking_lot::RwLock;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

type Callback = Arc<dyn Fn(String, bool) + Send + Sync>;

#[derive(Default)]
pub struct UnityConnectionMonitor {
    unity_port: u16,
    disconnect_grace: Duration,
    is_monitoring: AtomicBool,
    callbacks: RwLock<Vec<Callback>>,
    connected_clients: RwLock<HashSet<String>>,
    last_seen: RwLock<HashMap<String, Instant>>,
}

impl UnityConnectionMonitor {
    pub fn new(unity_port: u16, disconnect_grace_s: f64) -> Self {
        Self {
            unity_port,
            disconnect_grace: Duration::from_secs_f64(disconnect_grace_s.max(0.0)),
            is_monitoring: AtomicBool::new(false),
            callbacks: RwLock::new(Vec::new()),
            connected_clients: RwLock::new(HashSet::new()),
            last_seen: RwLock::new(HashMap::new()),
        }
    }

    pub fn unity_port(&self) -> u16 {
        self.unity_port
    }

    pub fn set_connection_callback<F>(&self, callback: F)
    where
        F: Fn(String, bool) + Send + Sync + 'static,
    {
        self.callbacks.write().push(Arc::new(callback));
    }

    pub fn start_monitoring(&self) {
        self.is_monitoring.store(true, Ordering::Relaxed);
    }

    pub fn stop_monitoring(&self) {
        self.is_monitoring.store(false, Ordering::Relaxed);
    }

    pub fn record_seen(&self, ip: &str) {
        if !self.is_monitoring.load(Ordering::Relaxed) {
            return;
        }

        let now = Instant::now();
        self.last_seen.write().insert(ip.to_string(), now);

        let mut connected = self.connected_clients.write();
        if connected.insert(ip.to_string()) {
            self.emit(ip.to_string(), true);
        }
    }

    pub fn sweep_disconnected(&self) {
        if !self.is_monitoring.load(Ordering::Relaxed) {
            return;
        }

        let now = Instant::now();
        let stale: Vec<String> = self
            .last_seen
            .read()
            .iter()
            .filter_map(|(ip, ts)| {
                if now.duration_since(*ts) > self.disconnect_grace {
                    Some(ip.clone())
                } else {
                    None
                }
            })
            .collect();

        if stale.is_empty() {
            return;
        }

        let mut connected = self.connected_clients.write();
        let mut last_seen = self.last_seen.write();
        for ip in stale {
            last_seen.remove(&ip);
            if connected.remove(&ip) {
                self.emit(ip, false);
            }
        }
    }

    pub fn get_connected_clients(&self) -> HashSet<String> {
        self.connected_clients.read().clone()
    }

    pub fn is_unity_connected(&self) -> bool {
        !self.connected_clients.read().is_empty()
    }

    fn emit(&self, ip: String, connected: bool) {
        let callbacks = self.callbacks.read().clone();
        for callback in callbacks {
            callback(ip.clone(), connected);
        }
    }
}

