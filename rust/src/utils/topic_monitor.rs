use once_cell::sync::Lazy;
use parking_lot::RwLock;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crate::utils::topic_status::get_topic_status_board;

#[derive(Debug)]
pub struct TopicSubscriptionMonitor {
    debounce: Duration,
    poll_interval: Duration,
    watched_topics: RwLock<HashSet<String>>,
    topic_modes: RwLock<HashMap<String, String>>,
    is_subscribed: RwLock<HashMap<String, bool>>,
    zero_since: RwLock<HashMap<String, Option<Instant>>>,
    running: AtomicBool,
}

impl Default for TopicSubscriptionMonitor {
    fn default() -> Self {
        Self::new(2.0, 2.0)
    }
}

impl TopicSubscriptionMonitor {
    pub fn new(debounce_s: f64, poll_hz: f64) -> Self {
        let poll_hz = if poll_hz <= 0.0 { 0.1 } else { poll_hz };
        Self {
            debounce: Duration::from_secs_f64(debounce_s.max(0.0)),
            poll_interval: Duration::from_secs_f64(1.0 / poll_hz),
            watched_topics: RwLock::new(HashSet::new()),
            topic_modes: RwLock::new(HashMap::new()),
            is_subscribed: RwLock::new(HashMap::new()),
            zero_since: RwLock::new(HashMap::new()),
            running: AtomicBool::new(false),
        }
    }

    pub fn start(&self) {
        self.running.store(true, Ordering::Relaxed);
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }

    pub fn watch_topics(&self, topics: &[String], modes: Option<&HashMap<String, String>>) {
        let board = get_topic_status_board();
        let mut watched = self.watched_topics.write();
        let mut per_topic_modes = self.topic_modes.write();
        let mut subscribed = self.is_subscribed.write();
        let mut zero_since = self.zero_since.write();

        for topic in topics {
            if topic.trim().is_empty() {
                continue;
            }
            if watched.insert(topic.clone()) {
                board.on_unsubscribe(topic);
            }
            subscribed.entry(topic.clone()).or_insert(false);
            zero_since.entry(topic.clone()).or_insert(None);

            if let Some(map) = modes {
                if let Some(mode) = map.get(topic) {
                    let normalized = if mode == "sdk_sub" {
                        "backend_pub"
                    } else {
                        "backend_sub"
                    };
                    per_topic_modes.insert(topic.clone(), normalized.to_string());
                } else {
                    per_topic_modes
                        .entry(topic.clone())
                        .or_insert_with(|| "backend_sub".to_string());
                }
            } else {
                per_topic_modes
                    .entry(topic.clone())
                    .or_insert_with(|| "backend_sub".to_string());
            }
        }
    }

    pub fn unwatch_topics(&self, topics: &[String], emit_unsubscribed: bool) {
        let board = get_topic_status_board();
        let mut watched = self.watched_topics.write();
        let mut per_topic_modes = self.topic_modes.write();
        let mut subscribed = self.is_subscribed.write();
        let mut zero_since = self.zero_since.write();

        for topic in topics {
            watched.remove(topic);
            per_topic_modes.remove(topic);
            zero_since.remove(topic);
            let was_subscribed = subscribed.remove(topic).unwrap_or(false);
            if emit_unsubscribed && was_subscribed {
                board.on_unsubscribe(topic);
            }
        }
    }

    pub fn mark_backend_link(&self, topic: &str, backend_link_up: bool, publisher_count: usize) {
        if topic.trim().is_empty() {
            return;
        }
        if !self.watched_topics.read().contains(topic) {
            return;
        }

        let now = Instant::now();
        let mut subscribed = self.is_subscribed.write();
        let mut zero_since = self.zero_since.write();
        let current = subscribed.get(topic).copied().unwrap_or(false);

        if backend_link_up && publisher_count > 0 {
            if !current {
                subscribed.insert(topic.to_string(), true);
                get_topic_status_board().on_subscribe(topic);
            }
            zero_since.insert(topic.to_string(), None);
            return;
        }

        if current {
            if !backend_link_up {
                subscribed.insert(topic.to_string(), false);
                zero_since.insert(topic.to_string(), None);
                get_topic_status_board().on_unsubscribe(topic);
                return;
            }

            if publisher_count == 0 {
                let timer = zero_since.entry(topic.to_string()).or_insert(None);
                match timer {
                    Some(started_at) => {
                        if now.duration_since(*started_at) >= self.debounce {
                            subscribed.insert(topic.to_string(), false);
                            *timer = None;
                            get_topic_status_board().on_unsubscribe(topic);
                        }
                    }
                    None => {
                        *timer = Some(now);
                    }
                }
            }
        } else {
            zero_since.insert(topic.to_string(), None);
        }
    }

    pub fn snapshot_topics(&self) -> Vec<String> {
        self.watched_topics.read().iter().cloned().collect()
    }

    pub fn poll_interval(&self) -> Duration {
        self.poll_interval
    }
}

static TOPIC_MONITOR: Lazy<TopicSubscriptionMonitor> = Lazy::new(TopicSubscriptionMonitor::default);

pub fn get_topic_monitor() -> &'static TopicSubscriptionMonitor {
    &TOPIC_MONITOR
}

