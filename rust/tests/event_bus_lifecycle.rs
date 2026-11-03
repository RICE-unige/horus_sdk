use horus::core::event_bus::EventBus;
use horus::core::types::EventPriority;
use serde_json::json;
use std::sync::{mpsc, Arc};
use std::time::Duration;

#[test]
fn event_priority_overflow_restart_and_callback_failure() {
    let bus = Arc::new(EventBus::new(3));
    bus.stop();
    let (tx, rx) = mpsc::channel();
    bus.subscribe(
        "robot.+",
        Arc::new(move |e| {
            tx.send(e.data).unwrap();
        }),
        EventPriority::Low,
    );
    let failed = bus.subscribe(
        "robot.+",
        Arc::new(|_| panic!("expected callback failure")),
        EventPriority::Low,
    );
    for (value, priority) in [
        (0, EventPriority::Low),
        (1, EventPriority::Normal),
        (2, EventPriority::High),
        (3, EventPriority::High),
    ] {
        bus.publish_simple("robot.status", json!(value), priority, "test");
    }
    assert_eq!(bus.get_stats()["active_subscriptions"], 2);
    assert_eq!(bus.get_stats()["dropped_events"], 1);
    bus.start();
    for value in [2, 3, 1] {
        assert_eq!(
            rx.recv_timeout(Duration::from_secs(2)).unwrap(),
            json!(value)
        );
    }
    bus.stop();
    assert!(bus.unsubscribe(&failed));
    let (stopped, wait) = mpsc::channel();
    let weak = Arc::downgrade(&bus);
    bus.subscribe(
        "stop",
        Arc::new(move |_| {
            weak.upgrade().unwrap().stop();
            stopped.send(()).unwrap();
        }),
        EventPriority::Low,
    );
    bus.publish_simple("stop", json!(null), EventPriority::Critical, "test");
    bus.start();
    wait.recv_timeout(Duration::from_secs(2)).unwrap();
    bus.publish_simple("robot.status", json!(4), EventPriority::Normal, "test");
    bus.start();
    assert_eq!(rx.recv_timeout(Duration::from_secs(2)).unwrap(), json!(4));
    bus.stop();
    bus.unsubscribe_all(None);
    assert_eq!(bus.get_stats()["active_subscriptions"], 0);
}

#[test]
fn concurrent_event_publication_does_not_lose_messages() {
    let bus = Arc::new(EventBus::new(4096));
    let (tx, rx) = mpsc::channel();
    bus.subscribe(
        "*",
        Arc::new(move |e| {
            tx.send(e.event_id).unwrap();
        }),
        EventPriority::Low,
    );
    let threads: Vec<_> = (0..4)
        .map(|_| {
            let bus = bus.clone();
            std::thread::spawn(move || {
                for _ in 0..500 {
                    bus.publish_simple("event", json!(null), EventPriority::Normal, "test");
                }
            })
        })
        .collect();
    for thread in threads {
        thread.join().unwrap();
    }
    let mut seen = std::collections::HashSet::new();
    for _ in 0..2000 {
        seen.insert(rx.recv_timeout(Duration::from_secs(5)).unwrap());
    }
    bus.stop();
    assert_eq!(seen.len(), 2000);
    assert_eq!(bus.get_stats()["events_processed"], 2000);
}
