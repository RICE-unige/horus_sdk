use horus::mapping::{MeshOperation, MeshPageScheduler, MeshPageUpdate, MeshSchedulerOptions};
use serde_json::{json, Value};
use std::sync::Arc;

fn options() -> MeshSchedulerOptions {
    MeshSchedulerOptions {
        pages_per_second: 10.0,
        bytes_per_second: 1000.0,
        max_pages: 4,
        max_payload_bytes: 2000,
    }
}
fn page(a: &Value) -> Arc<MeshPageUpdate> {
    Arc::new(MeshPageUpdate {
        chunk_id: a[0].as_u64().unwrap() as u32,
        epoch: a[1].as_u64().unwrap(),
        revision: a[2].as_u64().unwrap(),
        operation: match a[3].as_u64().unwrap() {
            1 => MeshOperation::Delete,
            2 => MeshOperation::Clear,
            _ => MeshOperation::Upsert,
        },
        payload: vec![0; a[4].as_u64().unwrap() as usize].into(),
        vertex_count: 3,
        triangle_count: 1,
    })
}
#[test]
fn shared_scheduler_contract() {
    let cases: Value =
        serde_json::from_str(include_str!("../../contracts/fixtures/mesh_scheduler.json")).unwrap();
    for test in cases.as_array().unwrap() {
        let mut queue = MeshPageScheduler::new(options(), 0.0).unwrap();
        for event in test["events"].as_array().unwrap() {
            if !event["enqueue"].is_null() {
                assert_eq!(
                    queue.enqueue(page(&event["enqueue"])).unwrap(),
                    event["accepted"].as_bool().unwrap()
                );
            } else {
                let ready = queue.pop_ready(event["pop"].as_f64().unwrap()).unwrap();
                let output = ready.map_or(Value::Null, |p| {
                    json!([
                        p.chunk_id,
                        p.epoch,
                        p.revision,
                        p.operation as u8,
                        p.payload.len()
                    ])
                });
                assert_eq!(output, event["ready"], "{}", test["name"]);
            }
            if let Some(n) = event["count"].as_u64() {
                assert_eq!(queue.pending_count(), n as usize);
            }
            if let Some(n) = event["bytes"].as_u64() {
                assert_eq!(queue.pending_bytes(), n as usize);
            }
        }
    }
}
#[test]
fn capacity_errors_are_atomic_and_geometry_is_shared() {
    let mut queue = MeshPageScheduler::new(options(), 0.0).unwrap();
    let shared = page(&json!([1, 1, 1, 0, 100]));
    queue.enqueue(shared.clone()).unwrap();
    assert!(Arc::ptr_eq(
        &queue.pop_ready(1.0).unwrap().unwrap(),
        &shared
    ));
    for id in 1..=4 {
        queue.enqueue(page(&json!([id, 1, 2, 0, 100]))).unwrap();
    }
    for invalid in [json!([5, 1, 2, 0, 100]), json!([1, 9, 2, 0, 2001])] {
        assert!(queue.enqueue(page(&invalid)).is_err());
        assert_eq!(queue.pending_count(), 4);
        assert_eq!(queue.pending_bytes(), 400);
    }
    for bad_time in [f64::INFINITY, f64::NAN] {
        assert!(queue.pop_ready(bad_time).is_err());
        assert_eq!(queue.pending_count(), 4);
        assert!(MeshPageScheduler::new(options(), bad_time).is_err());
    }
}
