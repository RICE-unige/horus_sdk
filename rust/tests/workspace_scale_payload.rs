use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::Robot;
use serde_json::Value;
use std::fs;

fn fixture(name: &str) -> Value {
    let path = format!("../contracts/fixtures/{name}");
    let raw = fs::read_to_string(path).expect("fixture must exist");
    serde_json::from_str(&raw).expect("fixture must be valid json")
}

fn build_robot_and_dataviz() -> (Robot, horus::DataViz) {
    let robot = Robot::new("scale_bot", RobotType::Wheeled);
    let dataviz = robot.create_dataviz(None);
    (robot, dataviz)
}

#[test]
fn workspace_scale_serialized_when_valid() {
    let (robot, dataviz) = build_robot_and_dataviz();
    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, Some(0.25));

    let expected = fixture("workspace_scale_valid.json");
    assert_eq!(
        config
            .workspace_config
            .as_ref()
            .map(|cfg| cfg.position_scale as f64),
        expected["workspace_config"]["position_scale"].as_f64()
    );
}

#[test]
fn workspace_scale_omitted_when_missing() {
    let (robot, dataviz) = build_robot_and_dataviz();
    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, None);
    assert!(config.workspace_config.is_none());
}

#[test]
fn workspace_scale_omitted_when_invalid() {
    let (robot, dataviz) = build_robot_and_dataviz();
    let client = RobotRegistryClient::new();
    let invalid_values = [
        0.0,
        -0.1,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for value in invalid_values {
        let config = client.build_robot_config_dict(&robot, &dataviz, None, Some(value));
        assert!(config.workspace_config.is_none());
    }
}

#[test]
fn robot_register_forwards_workspace_scale() {
    let (mut robot, dataviz) = build_robot_and_dataviz();
    let (success, result) = robot.register_with_horus(Some(dataviz), false, false, Some(0.33));
    assert!(success);
    let actual = result["payload"]["workspace_config"]["position_scale"]
        .as_f64()
        .expect("position scale should exist");
    assert!((actual - 0.33).abs() < 1e-6);
}

