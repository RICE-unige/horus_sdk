use crate::core::types::EventPriority;
use once_cell::sync::Lazy;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event {
    pub topic: String,
    pub data: serde_json::Value,
    pub priority: EventPriority,
    pub source: Option<String>,
    pub event_id: String,
    pub timestamp: f64,
}

impl Event {
    pub fn new(
        topic: impl Into<String>,
        data: serde_json::Value,
        priority: EventPriority,
        source: impl Into<String>,
    ) -> Self {
        Self {
            topic: topic.into(),
            data,
            priority,
            source: Some(source.into()),
            event_id: Uuid::new_v4().to_string(),
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_else(|_| Duration::from_secs(0))
                .as_secs_f64(),
        }
    }
}

pub type EventCallback = Arc<dyn Fn(Event) + Send + Sync + 'static>;

#[derive(Clone)]
struct EventSubscription {
    callback: EventCallback,
    topic_filter: String,
    priority_filter: EventPriority,
    subscription_id: String,
    active: bool,
}

impl EventSubscription {
    fn new(callback: EventCallback, topic_filter: String, priority_filter: EventPriority) -> Self {
        Self {
            callback,
            topic_filter,
            priority_filter,
            subscription_id: Uuid::new_v4().to_string(),
            active: true,
        }
    }

    fn matches(&self, event: &Event) -> bool {
        if !self.active {
            return false;
        }
        if event.priority < self.priority_filter {
            return false;
        }
        matches_topic(&event.topic, &self.topic_filter)
    }
}

fn matches_topic(event_topic: &str, topic_filter: &str) -> bool {
    if topic_filter == "*" {
        return true;
    }
    if event_topic == topic_filter {
        return true;
    }
    if topic_filter.ends_with('*') {
        let prefix = &topic_filter[..topic_filter.len() - 1];
        return event_topic.starts_with(prefix);
    }

    let filter_parts: Vec<&str> = topic_filter.split('.').collect();
    let event_parts: Vec<&str> = event_topic.split('.').collect();
    if filter_parts.is_empty() {
        return false;
    }

    if filter_parts.last() == Some(&"*") {
        let prefix_parts = &filter_parts[..filter_parts.len() - 1];
        if event_parts.len() < prefix_parts.len() {
            return false;
        }
        return prefix_parts
            .iter()
            .zip(event_parts.iter())
            .all(|(fp, ep)| *fp == "+" || *fp == *ep);
    }

    if filter_parts.len() != event_parts.len() {
        return false;
    }
    filter_parts
        .iter()
        .zip(event_parts.iter())
        .all(|(fp, ep)| *fp == "+" || *fp == *ep)
}

#[derive(Default)]
struct EventBusStats {
    events_published: AtomicUsize,
    events_processed: AtomicUsize,
    dropped_events: AtomicUsize,
    active_subscriptions: AtomicUsize,
}

pub struct EventBus {
    max_queue_size: usize,
    running: Arc<AtomicBool>,
    queue: Arc<(Mutex<Vec<Event>>, Condvar)>,
    subscriptions: Arc<RwLock<HashMap<String, Vec<EventSubscription>>>>,
    stats: Arc<EventBusStats>,
    worker: Mutex<Option<JoinHandle<()>>>,
}

impl EventBus {
    pub fn new(max_queue_size: usize) -> Self {
        let queue = Arc::new((Mutex::new(Vec::<Event>::new()), Condvar::new()));
        let subscriptions = Arc::new(RwLock::new(HashMap::<String, Vec<EventSubscription>>::new()));
        let stats = Arc::new(EventBusStats::default());
        let running = Arc::new(AtomicBool::new(true));

        let queue_for_thread = Arc::clone(&queue);
        let subscriptions_for_thread = Arc::clone(&subscriptions);
        let stats_for_thread = Arc::clone(&stats);
        let running_for_thread = Arc::clone(&running);
        let worker = thread::spawn(move || {
            loop {
                let maybe_event = {
                    let (lock, cvar) = &*queue_for_thread;
                    let mut guard = lock.lock().unwrap_or_else(|e| e.into_inner());
                    while guard.is_empty() {
                        let res = cvar
                            .wait_timeout(guard, Duration::from_millis(100))
                            .unwrap_or_else(|e| e.into_inner());
                        guard = res.0;
                        if guard.is_empty() && !running_for_thread.load(Ordering::Relaxed) {
                            return;
                        }
                    }
                    guard.sort_by(|a, b| b.priority.cmp(&a.priority));
                    guard.pop()
                };

                let Some(event) = maybe_event else {
                    continue;
                };

                let callbacks: Vec<EventCallback> = {
                    let subs = subscriptions_for_thread.read().unwrap_or_else(|e| e.into_inner());
                    subs.values()
                        .flat_map(|v| v.iter())
                        .filter(|s| s.matches(&event))
                        .map(|s| Arc::clone(&s.callback))
                        .collect()
                };

                for cb in callbacks {
                    cb(event.clone());
                }
                stats_for_thread
                    .events_processed
                    .fetch_add(1, Ordering::Relaxed);
            }
        });

        Self {
            max_queue_size,
            running,
            queue,
            subscriptions,
            stats,
            worker: Mutex::new(Some(worker)),
        }
    }

    pub fn publish_event(&self, event: Event) {
        let (lock, cvar) = &*self.queue;
        let mut guard = lock.lock().unwrap_or_else(|e| e.into_inner());
        if guard.len() >= self.max_queue_size {
            if let Some(idx) = guard.iter().position(|e| e.priority == EventPriority::Low) {
                guard.remove(idx);
            } else if !guard.is_empty() {
                guard.remove(0);
            }
            self.stats.dropped_events.fetch_add(1, Ordering::Relaxed);
        }
        guard.push(event);
        self.stats
            .events_published
            .fetch_add(1, Ordering::Relaxed);
        cvar.notify_one();
    }

    pub fn publish_simple(
        &self,
        topic: impl Into<String>,
        data: serde_json::Value,
        priority: EventPriority,
        source: impl Into<String>,
    ) {
        self.publish_event(Event::new(topic, data, priority, source));
    }

    pub fn subscribe(
        &self,
        topic_filter: impl Into<String>,
        callback: EventCallback,
        priority_filter: EventPriority,
    ) -> String {
        let topic_filter = topic_filter.into();
        let subscription = EventSubscription::new(callback, topic_filter.clone(), priority_filter);
        let subscription_id = subscription.subscription_id.clone();
        let mut subs = self.subscriptions.write().unwrap_or_else(|e| e.into_inner());
        subs.entry(topic_filter).or_default().push(subscription);
        let active_count: usize = subs.values().map(|v| v.len()).sum();
        self.stats
            .active_subscriptions
            .store(active_count, Ordering::Relaxed);
        subscription_id
    }

    pub fn unsubscribe(&self, subscription_id: &str) -> bool {
        let mut subs = self.subscriptions.write().unwrap_or_else(|e| e.into_inner());
        let mut found = false;
        let keys: Vec<String> = subs.keys().cloned().collect();
        for key in keys {
            if let Some(list) = subs.get_mut(&key) {
                let before = list.len();
                list.retain(|s| s.subscription_id != subscription_id);
                if list.len() != before {
                    found = true;
                }
            }
            let remove_key = subs.get(&key).map(|v| v.is_empty()).unwrap_or(false);
            if remove_key {
                subs.remove(&key);
            }
        }
        let active_count: usize = subs.values().map(|v| v.len()).sum();
        self.stats
            .active_subscriptions
            .store(active_count, Ordering::Relaxed);
        found
    }

    pub fn unsubscribe_all(&self, topic_filter: Option<&str>) {
        let mut subs = self.subscriptions.write().unwrap_or_else(|e| e.into_inner());
        match topic_filter {
            Some(filter) => {
                subs.remove(filter);
            }
            None => subs.clear(),
        }
        let active_count: usize = subs.values().map(|v| v.len()).sum();
        self.stats
            .active_subscriptions
            .store(active_count, Ordering::Relaxed);
    }

    pub fn get_stats(&self) -> HashMap<String, usize> {
        let queue_size = self
            .queue
            .0
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .len();
        HashMap::from([
            (
                "events_published".to_string(),
                self.stats.events_published.load(Ordering::Relaxed),
            ),
            (
                "events_processed".to_string(),
                self.stats.events_processed.load(Ordering::Relaxed),
            ),
            (
                "dropped_events".to_string(),
                self.stats.dropped_events.load(Ordering::Relaxed),
            ),
            (
                "active_subscriptions".to_string(),
                self.stats.active_subscriptions.load(Ordering::Relaxed),
            ),
            ("queue_size".to_string(), queue_size),
        ])
    }

    pub fn get_subscriptions(&self) -> HashMap<String, usize> {
        self.subscriptions
            .read()
            .unwrap_or_else(|e| e.into_inner())
            .iter()
            .map(|(k, v)| (k.clone(), v.len()))
            .collect()
    }
}

impl Drop for EventBus {
    fn drop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        self.queue.1.notify_all();
        if let Ok(mut handle_guard) = self.worker.lock() {
            if let Some(handle) = handle_guard.take() {
                let _ = handle.join();
            }
        }
    }
}

static GLOBAL_EVENT_BUS: Lazy<EventBus> = Lazy::new(|| EventBus::new(1000));

pub fn get_event_bus() -> &'static EventBus {
    &GLOBAL_EVENT_BUS
}

pub fn publish_now(
    topic: impl Into<String>,
    data: serde_json::Value,
    priority: EventPriority,
    source: impl Into<String>,
) {
    get_event_bus().publish_simple(topic, data, priority, source);
}

pub async fn publish(
    topic: impl Into<String>,
    data: serde_json::Value,
    priority: EventPriority,
    source: impl Into<String>,
) {
    publish_now(topic, data, priority, source);
}

pub fn subscribe_now(
    topic_filter: impl Into<String>,
    callback: EventCallback,
    priority_filter: EventPriority,
) -> String {
    get_event_bus().subscribe(topic_filter, callback, priority_filter)
}

pub async fn subscribe(
    topic_filter: impl Into<String>,
    callback: EventCallback,
    priority_filter: EventPriority,
) -> String {
    subscribe_now(topic_filter, callback, priority_filter)
}

pub fn unsubscribe_now(subscription_id: &str) -> bool {
    get_event_bus().unsubscribe(subscription_id)
}

pub async fn unsubscribe(subscription_id: &str) -> bool {
    unsubscribe_now(subscription_id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;

    #[test]
    fn topic_matching_supports_plus_and_star() {
        assert!(matches_topic("robot.status.ready", "robot.status.*"));
        assert!(matches_topic("robot.status.ready", "robot.+.ready"));
        assert!(!matches_topic("robot.status.ready", "robot.ready"));
    }

    #[test]
    fn publish_and_subscribe_flow() {
        let bus = EventBus::new(16);
        let seen = Arc::new(AtomicUsize::new(0));
        let seen_cb = Arc::clone(&seen);
        let sub = bus.subscribe(
            "robot.*",
            Arc::new(move |_| {
                seen_cb.fetch_add(1, Ordering::Relaxed);
            }),
            EventPriority::Low,
        );
        bus.publish_simple(
            "robot.status",
            serde_json::json!({"ok": true}),
            EventPriority::Normal,
            "test",
        );
        std::thread::sleep(Duration::from_millis(80));
        assert!(seen.load(Ordering::Relaxed) >= 1);
        assert!(bus.unsubscribe(&sub));
    }
}
