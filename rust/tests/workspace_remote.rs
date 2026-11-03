use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::dataviz::{DataViz, RenderOptions};
use horus::robot::Robot;
use serde_json::Value;

#[test]
fn workspace_and_remote_view_match_python() {
    let fixture: Value = serde_json::from_str(include_str!("../../contracts/fixtures/remote_render.json")).unwrap();
    let client = RobotRegistryClient::new();
    let entity = Robot::workspace_visualization("map_view");
    assert_eq!(entity.get_entity_kind(), "workspace_visualization");
    assert!(entity.create_dataviz(None).visualizations.is_empty());
    for test in fixture["cases"].as_array().unwrap() {
        let mut viz = DataViz::new("remote");
        let options: RenderOptions = serde_json::from_value(test["options"].clone()).unwrap();
        viz.add_remote_rendered_map("/remote/map", "world", Some(options)).unwrap();
        let mut expected = fixture["defaults"].as_object().unwrap().clone();
        expected.extend(test["expected"].as_object().unwrap().clone());
        if let Some(absent) = test["absent"].as_array() {
            for key in absent { expected.remove(key.as_str().unwrap()); }
        }
        for explicit_global in [false, true] {
            let global = explicit_global.then(|| client.build_global_visualizations_payload(&[viz.clone()]));
            let wire = serde_json::to_value(client.build_robot_config_dict(&entity, &viz, global, None)).unwrap();
            assert_eq!(wire["entity_kind"], "workspace_visualization");
            assert!(wire["capabilities"].as_object().unwrap().values().all(|value| value == false));
            assert_eq!(wire["robot_manager_config"]["enabled"], false);
            assert!(wire["robot_manager_config"]["sections"].as_object().unwrap().values().all(|value| value == false));
            assert_eq!(wire["control"]["drive_topic"], "");
            assert_eq!(wire["control"]["teleop"]["enabled"], false);
            assert!(wire["control"]["tasks"].as_object().unwrap().values().all(|value| value["enabled"] == false));
            assert!(wire["sensors"].as_array().unwrap().is_empty());
            assert!(wire["visualizations"].as_array().unwrap().is_empty());
            assert_eq!(wire["global_visualizations"].as_array().unwrap().len(), 1);
            let map = &wire["global_visualizations"][0];
            assert_eq!(map["type"], "remote_render");
            assert_eq!(map["frame"], "world");
            assert_eq!(map["remote_render"], Value::Object(expected.clone()), "{}", test["name"]);
        }
        viz.add_remote_rendered_map("/remote/map", "world", None).unwrap();
        assert_eq!(viz.visualizations.len(), 1);
    }
    for options in fixture["invalid"].as_array().unwrap() {
        let mut viz = DataViz::new("invalid");
        assert!(viz.add_remote_rendered_map("/remote/map", "world", Some(serde_json::from_value(options.clone()).unwrap())).is_err());
        assert!(viz.visualizations.is_empty());
    }
    let ordinary = Robot::new("robot", RobotType::Wheeled);
    assert_eq!(ordinary.get_entity_kind(), "robot");
    assert!(!ordinary.create_dataviz(None).visualizations.is_empty());
}
