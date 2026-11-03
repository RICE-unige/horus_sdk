use horus::mapping::{BlockGeometry, LiveMeshOptions, LiveMeshState, MeshOperation};
use serde_json::json;
use std::sync::Arc;

fn triangle() -> BlockGeometry {
    BlockGeometry {
        vertices: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
        colors: vec![[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 255]],
        triangles: vec![0, 1, 2],
    }
}
fn state() -> LiveMeshState {
    LiveMeshState::new(LiveMeshOptions {
        max_pages: 2,
        max_payload_bytes: 120,
        max_cache_bytes: 240,
        initial_epoch: Some(11),
        ..Default::default()
    })
    .unwrap()
}
#[test]
fn native_map_state_matches_python_manifest_and_shares_buffers() {
    let mut map = state();
    let g = triangle();
    let first = map.replace("a", Some(&g)).unwrap()[0].clone();
    assert_eq!(first.payload.len(), 112);
    assert!(map.replace("a", Some(&g)).unwrap().is_empty());
    assert_eq!(map.unchanged_pages(), 1);
    assert_eq!(map.suppressed_bytes(), 112);
    map.replace("b", Some(&g)).unwrap();
    let snapshot = map.snapshot_updates();
    assert!(Arc::ptr_eq(&first, &snapshot[0]));
    let deleted = map.replace("a", None).unwrap();
    assert_eq!(deleted[0].operation, MeshOperation::Delete);
    assert_eq!(deleted[0].revision, 2);
    assert_eq!(map.cached_bytes(), 164);
    assert_eq!(map.page_count(), 1);
    let new = map.replace("c", Some(&g)).unwrap();
    assert_eq!(new[0].chunk_id, 1);
    assert_eq!(new[0].revision, 3);
    let expected: serde_json::Value = serde_json::from_str(include_str!(
        "../src/mapping/fixtures/live_mesh_manifest.json"
    ))
    .unwrap();
    assert_eq!(map.manifest(), expected);
    let repair = map.repair_updates(&json!({"request":"pages","epoch":"11","ids":[1,1]}));
    assert_eq!(repair.len(), 1);
    assert!(Arc::ptr_eq(&repair[0], &new[0]));
    assert_eq!(
        map.repair_updates(&json!({"request":"pages","epoch":10}))
            .len(),
        2
    );
    assert!(map
        .repair_updates(&json!({"request":"pages","epoch":11,"ids":[true]}))
        .is_empty());
    assert!(map
        .repair_updates(&json!({"request":"pages","epoch":11,"ids":[1,2,1]}))
        .is_empty());
    assert_eq!(map.clear().unwrap().epoch, 12);
    assert_eq!(map.cached_bytes(), 0);
    assert_eq!(
        map.repair_updates(&json!({"request":"snapshot"}))[0].operation,
        MeshOperation::Clear
    );
    assert_eq!(first.payload.len(), 112); // Independent retained snapshot remains valid.
}
#[test]
fn map_validation_and_batch_failure_leave_previous_state_unchanged() {
    let mut map = state();
    let g = triangle();
    map.replace("a", Some(&g)).unwrap();
    let before = map.manifest();
    let mut invalid = g.clone();
    invalid.triangles[0] = 10;
    assert!(map
        .replace_many([("a", None), ("bad", Some(&invalid))])
        .is_err());
    assert_eq!(map.manifest(), before);
    let mut changed = g.clone();
    changed.colors[0] = [0, 0, 0, 255];
    assert_eq!(map.replace("a", Some(&changed)).unwrap()[0].revision, 2);
    map.replace("b", Some(&g)).unwrap();
    let full = map.manifest();
    assert!(map.replace("c", Some(&g)).is_err());
    assert_eq!(map.manifest(), full);
    let mut limited = LiveMeshState::new(LiveMeshOptions {
        max_payload_bytes: 120,
        max_cache_bytes: 120,
        initial_epoch: Some(11),
        ..Default::default()
    })
    .unwrap();
    limited.replace("a", Some(&g)).unwrap();
    assert!(limited.replace("b", Some(&g)).is_err());
    assert_eq!(limited.page_count(), 1);
    assert_eq!(limited.cached_bytes(), 112);
    invalid.vertices[0][0] = f32::NAN;
    assert!(limited.replace("a", Some(&invalid)).is_err());
    assert!(LiveMeshState::new(LiveMeshOptions {
        max_pages: 0,
        ..Default::default()
    })
    .is_err());
}
