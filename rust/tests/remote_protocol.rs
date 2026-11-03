use horus::remote_rendering::*;
use serde_json::Value;
use std::sync::Arc;
use std::time::Duration;
fn fixture() -> Value {
    serde_json::from_str(include_str!(
        "../../contracts/fixtures/remote_protocol.json"
    ))
    .unwrap()
}
fn unhex(v: &Value) -> Vec<u8> {
    v.as_str()
        .unwrap()
        .as_bytes()
        .chunks_exact(2)
        .map(|p| u8::from_str_radix(std::str::from_utf8(p).unwrap(), 16).unwrap())
        .collect()
}
#[test]
fn python_view_and_true_stereo_matrices() {
    let f = fixture();
    let v = ViewState::from_value(&f["view"]).unwrap();
    assert_eq!(v.viewport_width, 1832);
    assert_eq!(v.view_stream_id, "stream-a");
    let round = ViewState::from_json(&v.to_json().unwrap()).unwrap();
    assert_eq!(round.sample_time_ns, v.sample_time_ns);
    let matrices = serde_json::to_value(v.eye_matrices(0.1, 40.).unwrap()).unwrap();
    for (name, expected) in f["matrices"].as_object().unwrap() {
        for r in 0..4 {
            for c in 0..4 {
                assert!(
                    (matrices[name][r][c].as_f64().unwrap() - expected[r][c].as_f64().unwrap())
                        .abs()
                        < 2e-7
                );
            }
        }
    }
    let p = eye_fov_from_unity_projection(1.31, 1.47, 0.08, -0.04)
        .unwrap()
        .renderer_projection()
        .unwrap();
    assert!((p[2] + 0.08).abs() < 1e-12);
    assert!((p[3] - 0.04).abs() < 1e-12);
    let mut bad = f["view"].clone();
    bad["sample_time_ns"] = (-1).into();
    assert!(ViewState::from_value(&bad).is_err());
    bad = f["view"].clone();
    bad["workspace"]["scale"] = 0.into();
    assert!(ViewState::from_value(&bad).is_err());
    bad = f["view"].clone();
    bad["left_eye"]["rotation"] = serde_json::json!([0, 0, 0, 0]);
    assert!(ViewState::from_value(&bad).is_err());
}
#[test]
fn bounded_framing_borrows_payload_and_rejects_truncation() {
    let f = fixture();
    let bytes = unhex(&f["wire_hex"]);
    let (metadata, image) = unpack_ros_diagnostic_frame(&bytes).unwrap();
    assert_eq!(image, unhex(&f["image_hex"]));
    assert_eq!(metadata.to_value().unwrap(), f["metadata"]);
    assert!(
        image.as_ptr() >= bytes.as_ptr()
            && image.as_ptr() < unsafe { bytes.as_ptr().add(bytes.len()) }
    );
    let native = pack_ros_diagnostic_frame(&metadata, image).unwrap();
    assert_eq!(unpack_ros_diagnostic_frame(&native).unwrap().0, metadata);
    for n in 0..bytes.len() {
        assert!(unpack_ros_diagnostic_frame(&bytes[..n]).is_err());
    }
    let mut bad = bytes.clone();
    bad[5] = 1;
    assert!(unpack_ros_diagnostic_frame(&bad).is_err());
    bad = bytes.clone();
    bad[8..12].fill(255);
    assert!(unpack_ros_diagnostic_frame(&bad).is_err());
    let mut value = f["metadata"].clone();
    value["codec"] = "h264".into();
    assert!(RenderedFrameMetadata::from_value(&value).is_err());
    value["color_range"] = "limited".into();
    value["frame_marker_height"] = 8.into();
    assert!(RenderedFrameMetadata::from_value(&value).is_ok());
}
#[test]
fn clock_and_stream_reconnect_match_python() {
    let f = fixture();
    let mut clock = ClockAlignment::default();
    assert!(clock.offset_ns().is_err());
    for s in f["clock_samples"].as_array().unwrap() {
        clock
            .add(ClockSample {
                client_send_ns: s[0].as_i64().unwrap(),
                server_receive_ns: s[1].as_i64().unwrap(),
                server_send_ns: s[2].as_i64().unwrap(),
                client_receive_ns: s[3].as_i64().unwrap(),
            })
            .unwrap();
    }
    assert_eq!(clock.offset_ns().unwrap(), f["offset_ns"].as_f64().unwrap());
    assert_eq!(
        clock.round_trip_ns().unwrap(),
        f["round_trip_ns"].as_f64().unwrap()
    );
    let v = ViewState::from_value(&f["view"]).unwrap();
    let mut gate = ViewStreamSequenceGate::default();
    assert_eq!(gate.accept(&v, 100.).unwrap(), None);
    assert_eq!(gate.accept(&v, 101.).unwrap(), Some("stale_sample"));
    let mut next = v.clone();
    next.view_stream_id = "stream-b".into();
    next.view_sequence = 1;
    next.sample_time_ns = 10;
    assert_eq!(gate.accept(&next, 101.).unwrap(), Some("view_stream_busy"));
    assert_eq!(gate.accept(&next, 103.).unwrap(), None);
    assert_eq!(gate.accept(&v, 106.).unwrap(), Some("retired_view_stream"));
    assert_eq!(gate.revision(), 2);
    assert!(gate.accept(&next, 1.).is_err());
}
#[test]
fn latest_queue_replaces_and_close_wakes_waiter() {
    let q = Arc::new(LatestValueQueue::default());
    q.put(vec![1]).unwrap();
    q.put(vec![2, 3]).unwrap();
    assert_eq!(q.replaced_count().unwrap(), 1);
    assert_eq!(q.get(None).unwrap(), Some(vec![2, 3]));
    assert_eq!(q.get(Some(Duration::from_millis(1))).unwrap(), None);
    let other = q.clone();
    let t = std::thread::spawn(move || other.get(None).unwrap());
    q.close().unwrap();
    assert_eq!(t.join().unwrap(), None);
    assert!(q.put(vec![4]).is_err());
}
