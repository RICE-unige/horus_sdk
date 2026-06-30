use horus::bridge::{build_robot_config_dict, validate_field_teammate_safety};
use horus::core::types::RobotType;
use horus::robot::Robot;
use serde_json::Value;
use std::fs;

fn fixture(name: &str) -> Value {
    let raw = fs::read_to_string(format!("../contracts/fixtures/{name}")).expect("fixture exists");
    serde_json::from_str(&raw).expect("fixture is valid json")
}

#[test]
fn field_teammate_payload_matches_contract_fixture() {
    let teammate = Robot::field_teammate("field_teammate_1");
    let dataviz = teammate.create_dataviz(None);
    let payload = build_robot_config_dict(&teammate, &dataviz, None);
    let actual = serde_json::to_value(&payload).expect("serialize");
    let expected = fixture("field_teammate_hololens.json");

    assert_eq!(actual["robot_type"], "human");
    assert_eq!(actual["entity_kind"], "field_teammate");
    assert_eq!(actual["capabilities"], expected["capabilities"]);
    assert_eq!(actual["control"]["teleop"]["enabled"], Value::Bool(false));
    assert_eq!(
        actual["control"]["tasks"]["go_to_point"]["enabled"],
        Value::Bool(false)
    );
    assert_eq!(
        actual["control"]["tasks"]["waypoint"]["enabled"],
        Value::Bool(false)
    );
    assert_eq!(
        actual["robot_manager_config"]["sections"],
        expected["robot_manager_config"]["sections"]
    );
    assert_eq!(actual["field_teammate_config"], expected["field_teammate_config"]);

    validate_field_teammate_safety(&payload).expect("default field teammate is safe");
}

#[test]
fn regular_robot_is_controllable_and_marked_robot() {
    let robot = Robot::new("rover", RobotType::Wheeled);
    let dataviz = robot.create_dataviz(None);
    let payload = build_robot_config_dict(&robot, &dataviz, None);

    assert_eq!(payload.entity_kind, "robot");
    assert!(payload.capabilities.controllable);
    assert!(payload.capabilities.teleoperable);
    assert!(payload.capabilities.taskable);
    assert!(!payload.capabilities.guidable);
    // A plain robot is not a field teammate, so the safety check is a no-op.
    validate_field_teammate_safety(&payload).expect("robot passes");
}

#[test]
fn validator_rejects_tampered_field_teammate() {
    let teammate = Robot::field_teammate("ft");
    let dataviz = teammate.create_dataviz(None);
    let mut payload = build_robot_config_dict(&teammate, &dataviz, None);

    // Tamper with the serialized payload to re-enable teleop.
    payload.control.teleop.enabled = true;
    assert!(validate_field_teammate_safety(&payload).is_err());

    // Re-enabling a robot-control capability is also rejected.
    let mut payload2 = build_robot_config_dict(&teammate, &dataviz, None);
    payload2.capabilities.controllable = true;
    assert!(validate_field_teammate_safety(&payload2).is_err());
}
