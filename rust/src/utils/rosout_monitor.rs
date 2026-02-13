use once_cell::sync::Lazy;
use parking_lot::RwLock;
use regex::Regex;
use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};

use crate::utils::topic_status::get_topic_status_board;

#[derive(Debug)]
pub struct RosoutSubscriptionMonitor {
    running: AtomicBool,
    sessions: RwLock<HashMap<String, HashSet<String>>>,
    topic_states: RwLock<HashMap<String, bool>>,
    connection_pattern: Regex,
    register_pattern: Regex,
    disconnect_pattern: Regex,
}

impl Default for RosoutSubscriptionMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl RosoutSubscriptionMonitor {
    pub fn new() -> Self {
        Self {
            running: AtomicBool::new(false),
            sessions: RwLock::new(HashMap::new()),
            topic_states: RwLock::new(HashMap::new()),
            connection_pattern: Regex::new(r"Connection from (.+)")
                .expect("connection regex must compile"),
            register_pattern: Regex::new(r"RegisterSubscriber\(([^,]+),\s*([^)]+)\)\s*OK")
                .expect("register regex must compile"),
            disconnect_pattern: Regex::new(r"Disconnected from (.+)")
                .expect("disconnect regex must compile"),
        }
    }

    pub fn start(&self) {
        self.running.store(true, Ordering::Relaxed);
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
        let mut topic_states = self.topic_states.write();
        for (topic, is_subscribed) in topic_states.iter_mut() {
            if *is_subscribed {
                *is_subscribed = false;
                get_topic_status_board().on_unsubscribe(topic);
            }
        }
    }

    pub fn ingest_log(&self, logger_name: &str, message: &str) {
        if !self.running.load(Ordering::Relaxed) {
            return;
        }
        if logger_name != "UnityEndpoint" {
            return;
        }

        if let Some(cap) = self.connection_pattern.captures(message) {
            if let Some(ip_match) = cap.get(1) {
                let ip = ip_match.as_str().trim().to_string();
                if !ip.is_empty() {
                    self.sessions.write().entry(ip).or_default();
                }
            }
            return;
        }

        if let Some(cap) = self.register_pattern.captures(message) {
            let Some(topic_match) = cap.get(1) else {
                return;
            };
            let topic = topic_match.as_str().trim().to_string();
            if topic.is_empty() {
                return;
            }

            let mut sessions = self.sessions.write();
            if let Some((_, topics)) = sessions.iter_mut().last() {
                topics.insert(topic.clone());
            }
            drop(sessions);

            let mut topic_states = self.topic_states.write();
            if !topic_states.get(&topic).copied().unwrap_or(false) {
                topic_states.insert(topic.clone(), true);
                get_topic_status_board().on_subscribe(&topic);
            }
            return;
        }

        if let Some(cap) = self.disconnect_pattern.captures(message) {
            let Some(ip_match) = cap.get(1) else {
                return;
            };
            let ip = ip_match.as_str().trim().to_string();
            if ip.is_empty() {
                return;
            }

            if let Some(topics) = self.sessions.write().remove(&ip) {
                let mut topic_states = self.topic_states.write();
                for topic in topics {
                    if topic_states.get(&topic).copied().unwrap_or(false) {
                        topic_states.insert(topic.clone(), false);
                        get_topic_status_board().on_unsubscribe(&topic);
                    }
                }
            }
        }
    }
}

static ROSOUT_MONITOR: Lazy<RosoutSubscriptionMonitor> = Lazy::new(RosoutSubscriptionMonitor::new);

pub fn get_rosout_monitor() -> &'static RosoutSubscriptionMonitor {
    &ROSOUT_MONITOR
}

