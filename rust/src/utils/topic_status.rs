use once_cell::sync::Lazy;
use parking_lot::RwLock;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicSubscriptionState {
    Subscribed,
    Unsubscribed,
}

impl TopicSubscriptionState {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Subscribed => "SUBSCRIBED",
            Self::Unsubscribed => "UNSUBSCRIBED",
        }
    }
}

#[derive(Debug, Clone)]
struct TopicState {
    state: TopicSubscriptionState,
    last_changed: f64,
}

#[derive(Debug, Default)]
pub struct TopicStatusBoard {
    states: RwLock<HashMap<String, TopicState>>,
    silent: AtomicBool,
    running: AtomicBool,
}

impl TopicStatusBoard {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start(&self) {
        self.running.store(true, Ordering::Relaxed);
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }

    pub fn set_silent(&self, silent: bool) {
        self.silent.store(silent, Ordering::Relaxed);
    }

    pub fn on_subscribe(&self, topic: &str) {
        self.update(topic, TopicSubscriptionState::Subscribed);
    }

    pub fn on_unsubscribe(&self, topic: &str) {
        self.update(topic, TopicSubscriptionState::Unsubscribed);
    }

    pub fn snapshot(&self) -> HashMap<String, String> {
        self.states
            .read()
            .iter()
            .map(|(topic, state)| (topic.clone(), state.state.as_str().to_string()))
            .collect()
    }

    pub fn state_for(&self, topic: &str) -> Option<TopicSubscriptionState> {
        self.states.read().get(topic).map(|s| s.state)
    }

    pub fn last_changed_for(&self, topic: &str) -> Option<f64> {
        self.states.read().get(topic).map(|s| s.last_changed)
    }

    fn update(&self, topic: &str, state: TopicSubscriptionState) {
        if topic.trim().is_empty() {
            return;
        }
        if !self.running.load(Ordering::Relaxed) {
            self.start();
        }

        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();

        let mut states = self.states.write();
        let should_emit = states
            .get(topic)
            .map(|entry| entry.state != state)
            .unwrap_or(true);
        states.insert(
            topic.to_string(),
            TopicState {
                state,
                last_changed: now,
            },
        );

        if should_emit && !self.silent.load(Ordering::Relaxed) {
            let action = if state == TopicSubscriptionState::Subscribed {
                "Subscribed"
            } else {
                "Unsubscribed"
            };
            println!("  * Topic: {topic} - {action}");
        }
    }
}

static TOPIC_STATUS_BOARD: Lazy<TopicStatusBoard> = Lazy::new(TopicStatusBoard::new);

pub fn get_topic_status_board() -> &'static TopicStatusBoard {
    &TOPIC_STATUS_BOARD
}

