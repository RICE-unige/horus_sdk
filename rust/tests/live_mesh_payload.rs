use horus::{
    bridge::RobotRegistryClient,
    core::types::RobotType,
    dataviz::{DataViz, RenderOptions},
    robot::Robot,
};
use serde_json::{json, Value};

#[test]
fn live_mesh_registration_matches_python_and_cpp() {
    let client = RobotRegistryClient::new();
    let robot = Robot::new("mesh_test", RobotType::Wheeled);
    let fixtures: Value = serde_json::from_str(include_str!(
        "../../contracts/fixtures/live_mesh_registration.json"
    ))
    .unwrap();
    for test in fixtures.as_array().unwrap() {
        let options: RenderOptions = serde_json::from_value(test["options"].clone()).unwrap();
        let mut viz = DataViz::new("mesh");
        match test["helper"].as_str().unwrap() {
            "live" => viz.add_live_mesh("/horus/maps/live", "map", Some(options)),
            "nvblox" => viz.add_nvblox_mesh("/horus/maps/live", "map", Some(options)),
            _ => viz.add_3d_mesh("/horus/maps/live", "map", Some(options)),
        }
        let global = client.build_global_visualizations_payload(std::slice::from_ref(&viz));
        assert_eq!(
            serde_json::to_value(&global[0]).unwrap(),
            test["expected"],
            "{}",
            test["name"]
        );
        let payload = client.build_robot_config_dict(&robot, &viz, None, None);
        assert_eq!(
            serde_json::to_value(&payload.global_visualizations[0]).unwrap(),
            test["expected"]
        );
    }
}

#[test]
fn oversized_integers_do_not_wrap_and_decimal_budgets_are_accepted() {
    let mut viz = DataViz::new("mesh");
    viz.add_live_mesh(
        "/mesh",
        "map",
        Some(RenderOptions::from([
            ("max_chunk_renderers".into(), json!("inf")),
            ("max_chunk_uploads_per_frame".into(), json!(1e40)),
            ("max_triangle_uploads_per_frame".into(), json!("4567.8")),
            ("max_triangles".into(), json!(u64::MAX)),
        ])),
    );
    let payload = RobotRegistryClient::new().build_global_visualizations_payload(&[viz]);
    let mesh = payload[0].mesh.as_ref().unwrap();
    assert_eq!(mesh["max_chunk_renderers"], 1024);
    assert_eq!(mesh["max_chunk_uploads_per_frame"], 32);
    assert_eq!(mesh["max_triangle_uploads_per_frame"], 4567);
    assert_eq!(mesh["max_triangles"], i32::MAX);
}

#[test]
fn independent_adaptive_policy_survives_native_mesh_registration() {
    let policy = json!({"enabled": true, "initial_triangle_budget": 500000,
        "regions": [{"id": "robot", "frame": "rover/base_link", "shape": "sphere",
                     "size": [2, 2, 2], "priority": 2}]});
    let mut viz = DataViz::new("adaptive");
    viz.add_live_mesh("/map_adaptive", "map",
        Some(RenderOptions::from([("adaptive".into(), policy.clone())])));
    let payload = RobotRegistryClient::new().build_global_visualizations_payload(&[viz]);
    assert_eq!(payload[0].mesh.as_ref().unwrap()["adaptive"], policy);
}

#[test]
fn optional_display_policy_survives_mesh_registration() {
    let display = json!({"lod_enabled": true, "spacewarp_enabled": true, "lod_error_pixels": 1.0});
    let mut viz = DataViz::new("display");
    viz.add_live_mesh("/mesh_lod", "map", Some(RenderOptions::from([("display".into(), display.clone())])));
    let payload = RobotRegistryClient::new().build_global_visualizations_payload(&[viz]);
    assert_eq!(payload[0].mesh.as_ref().unwrap()["display"], display);
}

#[test]
fn map_render_options_are_preserved_in_all_languages() {
    let cases: Value = serde_json::from_str(include_str!("../../contracts/fixtures/map_render_options.json")).unwrap();
    let client = RobotRegistryClient::new();
    let robot = Robot::new("mesh_test", RobotType::Wheeled);
    for test in cases.as_array().unwrap() {
        let mut viz = DataViz::new("maps");
        let options: RenderOptions = serde_json::from_value(test["options"].clone()).unwrap();
        let kind = test["type"].as_str().unwrap();
        match kind {
            "point_cloud" => viz.add_3d_map("/map", "odom", Some(options)),
            "gaussian_splat" => viz.add_gaussian_splat_map("/map", "odom", Some(options), "/preview"),
            _ => viz.add_3d_octomap("/map", "odom", Some(options)),
        }
        for payload in [
            client.build_global_visualizations_payload(std::slice::from_ref(&viz)),
            client.build_robot_config_dict(&robot, &viz, None, None).global_visualizations,
        ] {
            let wire = serde_json::to_value(&payload[0]).unwrap();
            for (key, expected) in test["fields"].as_object().unwrap() {
                assert_eq!(&wire[kind][key], expected, "{kind}/{key}");
            }
            if let Some(absent) = test["absent"].as_array() {
                for key in absent { assert!(wire[kind].get(key.as_str().unwrap()).is_none()); }
            }
        }
    }
}
