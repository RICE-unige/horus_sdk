use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::Robot;
use serde_json::{Value, json};
use std::collections::HashMap;
use std::fs;

fn fixture(name: &str) -> Value {
    let path = format!("../contracts/fixtures/{name}");
    let raw = fs::read_to_string(path).expect("fixture must exist");
    serde_json::from_str(&raw).expect("fixture must be valid json")
}

#[test]
fn occupancy_grid_serialized_as_global_visualization() {
    let robot = Robot::new("test_bot", RobotType::Wheeled);
    let mut dataviz = robot.create_dataviz(None);

    let mut render_options: HashMap<String, Value> = HashMap::new();
    render_options.insert("show_unknown_space".to_string(), json!(false));
    render_options.insert("position_scale".to_string(), json!(0.15));
    render_options.insert("position_offset".to_string(), json!([1.0, 2.0, 3.0]));
    render_options.insert(
        "rotation_offset_euler".to_string(),
        json!({ "x": 0.0, "y": 90.0, "z": 0.0 }),
    );
    dataviz.add_occupancy_grid("/map", "map", Some(render_options));

    let client = RobotRegistryClient::new();
    let config = client.build_robot_config_dict(&robot, &dataviz, None, None);

    let occupancy_entries: Vec<_> = config
        .global_visualizations
        .iter()
        .filter(|entry| entry.viz_type == "occupancy_grid")
        .collect();
    assert_eq!(occupancy_entries.len(), 1);

    let occupancy_entry = occupancy_entries[0];
    assert_eq!(occupancy_entry.scope, "global");
    assert_eq!(occupancy_entry.topic, "/map");
    assert_eq!(occupancy_entry.frame.as_deref(), Some("map"));

    let occupancy = occupancy_entry.occupancy.clone().expect("occupancy payload");
    assert!(!occupancy.show_unknown_space);
    assert_eq!(occupancy.position_scale, Some(0.15));
    assert_eq!(
        serde_json::to_value(occupancy.position_offset.expect("position offset")).expect("serialize"),
        json!({"x": 1.0, "y": 2.0, "z": 3.0})
    );
    assert_eq!(
        serde_json::to_value(occupancy.rotation_offset_euler.expect("rotation")).expect("serialize"),
        json!({"x": 0.0, "y": 90.0, "z": 0.0})
    );

    assert!(
        config
            .visualizations
            .iter()
            .all(|entry| entry.viz_type != "occupancy_grid")
    );
}

#[test]
fn global_visualization_dedupes_across_multiple_robots() {
    let robot_a = Robot::new("bot_a", RobotType::Wheeled);
    let mut dataviz_a = robot_a.create_dataviz(None);
    dataviz_a.add_occupancy_grid("/map", "map", None);

    let robot_b = Robot::new("bot_b", RobotType::Wheeled);
    let mut dataviz_b = robot_b.create_dataviz(None);
    dataviz_b.add_occupancy_grid("/map", "map", None);

    let client = RobotRegistryClient::new();
    let payload = client.build_global_visualizations_payload(&[dataviz_a, dataviz_b]);
    let occupancy_entries: Vec<_> = payload
        .iter()
        .filter(|entry| entry.viz_type == "occupancy_grid")
        .collect();
    assert_eq!(occupancy_entries.len(), 1);

    let expected = fixture("global_occupancy_dedupe.json");
    assert_eq!(payload.len(), expected["count"].as_u64().unwrap_or_default() as usize);
    let entry = occupancy_entries[0];
    assert_eq!(entry.topic, expected["global_visualizations"][0]["topic"]);
    assert_eq!(entry.scope, expected["global_visualizations"][0]["scope"]);
}

