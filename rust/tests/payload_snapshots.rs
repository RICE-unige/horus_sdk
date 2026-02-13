use horus::bridge::{RobotRegistryClient, build_global_visualizations_payload, build_robot_config_dict};
use horus::core::types::RobotType;
use horus::robot::{Robot, RobotDimensions};
use horus::sensors::Camera;
use serde_json::Value;
use std::fs;
use std::sync::Arc;

fn fixture(name: &str) -> Value {
    let path = format!("../contracts/fixtures/{name}");
    let raw = fs::read_to_string(path).expect("fixture must exist");
    serde_json::from_str(&raw).expect("fixture must be valid json")
}

#[test]
fn single_robot_registration_snapshot() {
    let robot = Robot::with_dimensions(
        "test_bot",
        RobotType::Wheeled,
        RobotDimensions::new(0.8, 0.6, 0.4),
    );
    let dataviz = robot.create_dataviz(None);
    let payload = build_robot_config_dict(&robot, &dataviz, None);
    let actual = serde_json::to_value(payload).expect("serialize");
    let expected = fixture("single_robot_registration_snapshot.json");

    assert_eq!(actual["action"], expected["action"]);
    assert_eq!(actual["robot_name"], expected["robot_name"]);
    assert_eq!(actual["robot_type"], expected["robot_type"]);
    assert_eq!(actual["control"], expected["control"]);
    let actual_dims = &actual["dimensions"];
    let expected_dims = &expected["dimensions"];
    for key in ["length", "width", "height"] {
        let a = actual_dims[key].as_f64().expect("actual dimension float");
        let e = expected_dims[key].as_f64().expect("expected dimension float");
        assert!((a - e).abs() < 1e-5, "{key} mismatch: actual={a} expected={e}");
    }
}

#[test]
fn multi_robot_global_visualization_snapshot() {
    let robot_a = Robot::new("bot_a", RobotType::Wheeled);
    let mut dataviz_a = robot_a.create_dataviz(None);
    dataviz_a.add_occupancy_grid("/map", "map", None);

    let robot_b = Robot::new("bot_b", RobotType::Wheeled);
    let mut dataviz_b = robot_b.create_dataviz(None);
    dataviz_b.add_occupancy_grid("/map", "map", None);

    let global = build_global_visualizations_payload(&[dataviz_a, dataviz_b]);
    let expected = fixture("multi_robot_global_viz_snapshot.json");
    assert_eq!(
        global.len(),
        expected["global_visualizations_count"]
            .as_u64()
            .expect("count") as usize
    );
    let first = serde_json::to_value(&global[0]).expect("serialize");
    assert_eq!(first["type"], expected["global_visualization"]["type"]);
    assert_eq!(first["topic"], expected["global_visualization"]["topic"]);
    assert_eq!(first["scope"], expected["global_visualization"]["scope"]);
    assert_eq!(first["frame"], expected["global_visualization"]["frame"]);
}

#[test]
fn camera_webrtc_fields_snapshot() {
    let mut robot = Robot::new("camera_bot", RobotType::Wheeled);
    let mut camera = Camera::new(
        "front_camera",
        "camera_bot/camera_link",
        "/camera_bot/camera/image_raw/compressed",
    );
    camera.streaming_type = "webrtc".to_string();
    camera.minimap_streaming_type.clear();
    camera.teleop_streaming_type.clear();
    camera.startup_mode.clear();
    robot.add_sensor(Arc::new(camera)).expect("sensor add");

    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let camera_cfg = serde_json::to_value(
        payload.sensors[0]
            .camera_config
            .clone()
            .expect("camera config"),
    )
    .expect("serialize");

    let expected = fixture("camera_webrtc_fields_snapshot.json");
    assert_eq!(camera_cfg["streaming_type"], expected["streaming_type"]);
    assert_eq!(
        camera_cfg["minimap_streaming_type"],
        expected["minimap_streaming_type"]
    );
    assert_eq!(
        camera_cfg["teleop_streaming_type"],
        expected["teleop_streaming_type"]
    );
    assert_eq!(camera_cfg["startup_mode"], expected["startup_mode"]);
    assert_eq!(
        camera_cfg["webrtc_client_signal_topic"],
        expected["webrtc_client_signal_topic"]
    );
    assert_eq!(
        camera_cfg["webrtc_server_signal_topic"],
        expected["webrtc_server_signal_topic"]
    );
    assert_eq!(
        camera_cfg["webrtc_bitrate_kbps"],
        expected["webrtc_bitrate_kbps"]
    );
    assert_eq!(camera_cfg["webrtc_framerate"], expected["webrtc_framerate"]);
}
