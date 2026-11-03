use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::Robot;
use horus::sensors::Camera;
use horus::sensors::WebRtcTransportConfig;
use serde_json::Value;
use std::fs;
use std::sync::Arc;

fn fixture(name: &str) -> Value {
    let path = format!("../contracts/fixtures/{name}");
    let raw = fs::read_to_string(path).expect("fixture must exist");
    serde_json::from_str(&raw).expect("fixture must be valid json")
}

#[test]
fn camera_transport_profile_defaults() {
    let camera = Camera::new(
        "front_camera",
        "test_bot/camera_link",
        "/test_bot/camera/image_raw",
    );
    let expected = fixture("camera_transport_defaults.json");
    assert_eq!(camera.streaming_type, expected["streaming_type"]);
    assert_eq!(
        camera.minimap_streaming_type,
        expected["minimap_streaming_type"]
    );
    assert_eq!(
        camera.teleop_streaming_type,
        expected["teleop_streaming_type"]
    );
    assert_eq!(camera.startup_mode, expected["startup_mode"]);
}

#[test]
fn separate_media_budgets_survive_registration() {
    let mut robot = Robot::new("budget_bot", RobotType::Wheeled);
    let mut camera = Camera::new("front", "budget_bot/camera", "/budget_bot/rgb");
    camera.configure_webrtc_transport(WebRtcTransportConfig {
        minimap_bitrate_kbps: Some(1500), minimap_framerate: Some(15),
        teleop_bitrate_kbps: Some(3000), teleop_framerate: Some(30),
        minimap_priority: Some(2), teleop_priority: Some(8),
        ..Default::default()
    });
    robot.add_sensor(Arc::new(camera)).expect("sensor");
    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&robot, &robot.create_dataviz(None), None, None);
    let profile = payload.sensors[0].camera_config.as_ref().expect("camera profile");
    assert_eq!(profile.webrtc_minimap_bitrate_kbps, 1500);
    assert_eq!(profile.webrtc_minimap_framerate, 15);
    assert_eq!(profile.webrtc_teleop_bitrate_kbps, 3000);
    assert_eq!(profile.webrtc_teleop_framerate, 30);
    assert_eq!(profile.webrtc_minimap_priority, 2);
    assert_eq!(profile.webrtc_teleop_priority, 8);
}

#[test]
fn camera_transport_profile_validation() {
    assert!(Camera::try_new_with_profiles(
        "front_camera",
        "test_bot/camera_link",
        "/test_bot/camera/image_raw",
        "ros",
        "invalid",
        "webrtc",
        "minimap",
    )
    .is_err());
    assert!(Camera::try_new_with_profiles(
        "front_camera",
        "test_bot/camera_link",
        "/test_bot/camera/image_raw",
        "ros",
        "ros",
        "invalid",
        "minimap",
    )
    .is_err());
    assert!(Camera::try_new_with_profiles(
        "front_camera",
        "test_bot/camera_link",
        "/test_bot/camera/image_raw",
        "ros",
        "ros",
        "webrtc",
        "invalid",
    )
    .is_err());
}

#[test]
fn registration_payload_includes_profile_fields() {
    let mut robot = Robot::new("test_bot", RobotType::Wheeled);
    let camera = Camera::try_new_with_profiles(
        "front_camera",
        "test_bot/camera_link",
        "/test_bot/camera/image_raw/compressed",
        "ros",
        "ros",
        "webrtc",
        "minimap",
    )
    .expect("camera config should be valid");
    robot.add_sensor(Arc::new(camera)).expect("sensor add");

    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let camera_config = config.sensors[0]
        .camera_config
        .clone()
        .expect("camera config expected");

    assert_eq!(camera_config.streaming_type, "ros");
    assert_eq!(camera_config.minimap_streaming_type, "ros");
    assert_eq!(camera_config.teleop_streaming_type, "webrtc");
    assert_eq!(camera_config.startup_mode, "minimap");
}

#[test]
fn camera_topic_profiles_merge_same_topic_as_both() {
    let mut robot = Robot::new("profile_bot", RobotType::Wheeled);
    let mut camera = Camera::try_new_with_profiles(
        "front_camera",
        "profile_bot/camera_link",
        "/profile_bot/camera/image_raw/compressed",
        "ros",
        "ros",
        "webrtc",
        "minimap",
    )
    .expect("camera config should be valid");
    camera.minimap_topic = "/profile_bot/camera/image_raw/compressed".to_string();
    camera.teleop_topic = "/profile_bot/camera/image_raw/compressed".to_string();
    robot.add_sensor(Arc::new(camera)).expect("sensor add");

    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let profiles = client.extract_camera_topic_profiles(&config);

    assert_eq!(
        profiles["/profile_bot/camera/image_raw/compressed"]["source_mode"],
        "both"
    );
}

#[test]
fn registration_payload_legacy_streaming_fallback() {
    let mut robot = Robot::new("legacy_bot", RobotType::Wheeled);
    let mut camera = Camera::new(
        "front_camera",
        "legacy_bot/camera_link",
        "/legacy_bot/camera/image_raw/compressed",
    );
    camera.streaming_type = "webrtc".to_string();
    camera.minimap_streaming_type.clear();
    camera.teleop_streaming_type.clear();
    camera.startup_mode.clear();
    robot.add_sensor(Arc::new(camera)).expect("sensor add");

    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let camera_config = config.sensors[0]
        .camera_config
        .clone()
        .expect("camera config expected");
    let expected = fixture("camera_transport_legacy_fallback.json");

    assert_eq!(camera_config.streaming_type, expected["streaming_type"]);
    assert_eq!(
        camera_config.minimap_streaming_type,
        expected["minimap_streaming_type"]
    );
    assert_eq!(
        camera_config.teleop_streaming_type,
        expected["teleop_streaming_type"]
    );
    assert_eq!(camera_config.startup_mode, expected["startup_mode"]);
}

#[test]
fn robot_manager_defaults_snapshot() {
    let robot = Robot::new("defaults_bot", RobotType::Wheeled);
    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let actual = serde_json::to_value(config.robot_manager_config).expect("serialize");
    let expected = fixture("robot_manager_config_defaults.json");
    assert_eq!(actual, expected);
}
