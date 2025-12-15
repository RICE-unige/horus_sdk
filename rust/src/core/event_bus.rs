use crate::core::types::EventPriority;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::sync::{mpsc, RwLock};
use uuid::Uuid;

/// Event class for EventBus communication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event {
    pub topic: String,
    pub data: serde_json::Value,
    pub priority: EventPriority,
    pub source: String,
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
            source: source.into(),
            event_id: Uuid::new_v4().to_string(),
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs_f64(),
        }
    }
}

/// Event subscription callback type
pub type EventCallback = Arc<dyn Fn(Event) -> () + Send + Sync>;

/// Manages a single event subscription
struct EventSubscription {
    callback: EventCallback,
    topic_filter: String,
    priority_filter: EventPriority,
    subscription_id: String,
    active: bool,
}

impl EventSubscription {
    fn new(
        callback: EventCallback,
        topic_filter: String,
        priority_filter: EventPriority,
    ) -> Self {
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

        if !self.matches_topic(&event.topic, &self.topic_filter) {
            return false;
        }

        if (event.priority as u8) < (self.priority_filter as u8) {
            return false;
        }

        true
    }

    fn matches_topic(&self, event_topic: &str, topic_filter: &str) -> bool {
        if topic_filter == "*" {
            return true;
        }

        if event_topic == topic_filter {
            return true;
        }

        // Simple wildcard matching
        if topic_filter.ends_with('*') {
            let prefix = &topic_filter[..topic_filter.len() - 1];
            return event_topic.starts_with(prefix);
        }

        // Split by '.' and match parts
        let event_parts: Vec<&str> = event_topic.split('.').collect();
        let filter_parts: Vec<&str> = topic_filter.split('.').collect();

        if filter_parts.is_empty() {
            return false;
        }

        // Handle trailing wildcard in parts
        if filter_parts.last() == Some(&"*") {
            let prefix_parts = &filter_parts[..filter_parts.len() - 1];
            if event_parts.len() < prefix_parts.len() {
                return false;
            }

            for (i, filter_part) in prefix_parts.iter().enumerate() {
                if *filter_part != "+" && *filter_part != event_parts[i] {
                    return false;
                }
            }
            return true;
        }

        // Exact match with + wildcards
        if event_parts.len() != filter_parts.len() {
            return false;
        }

        for (event_part, filter_part) in event_parts.iter().zip(filter_parts.iter()) {
            if *filter_part != "+" && *filter_part != *event_part {
                return false;
            }
        }

        true
    }
}

/// Thread-safe event bus for HORUS SDK component communication
///
/// Enables real-time data flow between ROS robots and Meta Quest 3 application
/// through topic-based publish/subscribe messaging with priority handling.
pub struct EventBus {
    subscriptions: Arc<RwLock<HashMap<String, Vec<Arc<RwLock<EventSubscription>>>>>>,
    event_tx: mpsc::UnboundedSender<Event>,
    stats: Arc<RwLock<EventBusStats>>,
    max_queue_size: usize,
}

#[derive(Debug, Clone, Default)]
struct EventBusStats {
    events_published: usize,
    events_processed: usize,
    dropped_events: usize,
}

impl EventBus {
    /// Create a new EventBus with specified max queue size
    pub fn new(max_queue_size: usize) -> Self {
        let (event_tx, mut event_rx) = mpsc::unbounded_channel::<Event>();
        let subscriptions = Arc::new(RwLock::new(HashMap::new()));
        let stats = Arc::new(RwLock::new(EventBusStats::default()));

        // Spawn event processing task
        let subs_clone = Arc::clone(&subscriptions);
        let stats_clone = Arc::clone(&stats);
        tokio::spawn(async move {
            while let Some(event) = event_rx.recv().await {
                // Dispatch event to all matching subscriptions
                let subs = subs_clone.read().await;
                let mut matching_subs = Vec::new();

                for sub_list in subs.values() {
                    for sub in sub_list {
                        let sub_lock = sub.read().await;
                        if sub_lock.matches(&event) {
                            matching_subs.push(Arc::clone(&sub_lock.callback));
                        }
                    }
                }
                drop(subs);

                // Invoke callbacks
                for callback in matching_subs {
                    let event_clone = event.clone();
                    tokio::spawn(async move {
                        callback(event_clone);
                    });
                }

                // Update stats
                let mut stats = stats_clone.write().await;
                stats.events_processed += 1;
            }
        });

        Self {
            subscriptions,
            event_tx,
            stats,
            max_queue_size,
        }
    }

    /// Publish an event to the bus
    pub async fn publish(&self, event: Event) {
        let mut stats = self.stats.write().await;
        
        // Note: UnboundedSender doesn't have a size limit, but we track conceptually
        if stats.events_published - stats.events_processed > self.max_queue_size {
            stats.dropped_events += 1;
            return;
        }

        if self.event_tx.send(event).is_ok() {
            stats.events_published += 1;
        }
    }

    /// Publish with convenience parameters
    pub async fn publish_simple(
        &self,
        topic: impl Into<String>,
        data: serde_json::Value,
        priority: EventPriority,
        source: impl Into<String>,
    ) {
        let event = Event::new(topic, data, priority, source);
        self.publish(event).await;
    }

    /// Subscribe to events matching topic filter
    pub async fn subscribe(
        &self,
        topic_filter: String,
        callback: EventCallback,
        priority_filter: EventPriority,
    ) -> String {
        let subscription = Arc::new(RwLock::new(EventSubscription::new(
            callback,
            topic_filter.clone(),
            priority_filter,
        )));

        let subscription_id = subscription.read().await.subscription_id.clone();

        let mut subs = self.subscriptions.write().await;
        subs.entry(topic_filter)
            .or_insert_with(Vec::new)
            .push(subscription);

        subscription_id
    }

    /// Unsubscribe from events
    pub async fn unsubscribe(&self, subscription_id: &str) -> bool {
        let mut subs = self.subscriptions.write().await;

        for (topic, sub_list) in subs.iter_mut() {
            sub_list.retain(|sub| {
                let sub_lock = tokio::task::block_in_place(|| {
                    tokio::runtime::Handle::current().block_on(sub.read())
                });
                sub_lock.subscription_id != subscription_id
            });

            if sub_list.is_empty() {
                subs.remove(topic);
                return true;
            }
        }

        false
    }

    /// Get event bus statistics
    pub async fn get_stats(&self) -> HashMap<String, usize> {
        let stats = self.stats.read().await;
        let mut result = HashMap::new();
        result.insert("events_published".to_string(), stats.events_published);
        result.insert("events_processed".to_string(), stats.events_processed);
        result.insert("dropped_events".to_string(), stats.dropped_events);
        result.insert(
            "active_subscriptions".to_string(),
            self.subscriptions.read().await.len(),
        );
        result
    }

    /// Get current subscription counts by topic
    pub async fn get_subscriptions(&self) -> HashMap<String, usize> {
        let subs = self.subscriptions.read().await;
        subs.iter()
            .map(|(topic, sub_list)| (topic.clone(), sub_list.len()))
            .collect()
    }
}

// Global event bus instance
static GLOBAL_EVENT_BUS: once_cell::sync::Lazy<EventBus> =
    once_cell::sync::Lazy::new(|| EventBus::new(1000));

/// Get the global HORUS event bus instance
pub fn get_event_bus() -> &'static EventBus {
    &GLOBAL_EVENT_BUS
}

/// Convenience function to publish to global event bus
pub async fn publish(
    topic: impl Into<String>,
    data: serde_json::Value,
    priority: EventPriority,
    source: impl Into<String>,
) {
    get_event_bus()
        .publish_simple(topic, data, priority, source)
        .await;
}

/// Convenience function to subscribe to global event bus
pub async fn subscribe(
    topic_filter: String,
    callback: EventCallback,
    priority_filter: EventPriority,
) -> String {
    get_event_bus()
        .subscribe(topic_filter, callback, priority_filter)
        .await
}

/// Convenience function to unsubscribe from global event bus
pub async fn unsubscribe(subscription_id: &str) -> bool {
    get_event_bus().unsubscribe(subscription_id).await
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_event_creation() {
        let event = Event::new("test.topic", serde_json::json!({}), EventPriority::Normal, "test");
        assert_eq!(event.topic, "test.topic");
        assert!(!event.event_id.is_empty());
    }

    #[tokio::test]
    async fn test_event_bus_publish_subscribe() {
        let bus = EventBus::new(100);
        let received = Arc::new(RwLock::new(false));
        let received_clone = Arc::clone(&received);

        let callback: EventCallback = Arc::new(move |_event| {
            let received = Arc::clone(&received_clone);
            tokio::spawn(async move {
                *received.write().await = true;
            });
        });

        bus.subscribe("test.*".to_string(), callback, EventPriority::Low)
            .await;

        bus.publish_simple("test.message", serde_json::json!({}), EventPriority::Normal, "test")
            .await;

        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        assert!(*received.read().await);
    }
}
