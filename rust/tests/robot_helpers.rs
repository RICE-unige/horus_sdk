use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::{
    EntityCapabilities, GoToPointTaskConfig, NavigationTaskConfig, Robot, WaypointTaskConfig,
};
use serde_json::{json, Value};

#[test]
fn robot_helpers_match_python_and_preserve_other_tasks() {
    let expected: Value =
        serde_json::from_str(include_str!("../../contracts/fixtures/robot_helpers.json")).unwrap();
    let mut robot = Robot::new("native_helpers", RobotType::Drone);
    robot
        .metadata
        .insert("task_config".into(), json!({"custom":{"enabled":true}}));
    robot.configure_waypoint_task(WaypointTaskConfig {
        enabled: false,
        path_topic: Some("/route".into()),
        frame_id: Some("map".into()),
        ..Default::default()
    });
    robot.configure_go_to_point_task(GoToPointTaskConfig {
        goal_topic: Some("/goal".into()),
        frame_id: Some("world".into()),
        min_altitude_m: Some(1.0),
        max_altitude_m: Some(5.0),
        allow_unbounded_placement: Some(true),
        ..Default::default()
    });
    robot.configure_capabilities(EntityCapabilities {
        teleoperable: false,
        guidable: true,
        ..EntityCapabilities::for_robot()
    });
    robot.configure_workspace_experiment(true, " experiment.v1 ");
    assert!(!robot.get_capabilities().teleoperable && robot.get_capabilities().guidable);
    assert!(robot.get_field_teammate_config().is_none());
    let client = RobotRegistryClient::new();
    let wire = |robot: &Robot| {
        serde_json::to_value(client.build_robot_config_dict(
            robot,
            &robot.create_dataviz(None),
            None,
            None,
        ))
        .unwrap()
    };
    let data = wire(&robot);
    assert_eq!(data["control"]["tasks"], expected["tasks"]);
    assert_eq!(data["workspace_config"], expected["workspace"]);
    assert_eq!(data["capabilities"], expected["capabilities"]);
    robot.configure_go_to_point_task(Default::default());
    assert_eq!(
        wire(&robot)["control"]["tasks"]["go_to_point"]["allow_unbounded_placement"],
        false
    );
    assert_eq!(
        wire(&robot)["control"]["tasks"]["waypoint"],
        expected["tasks"]["waypoint"]
    );
    robot.configure_navigation_tasks(NavigationTaskConfig {
        allow_unbounded_placement: Some(true),
        ..Default::default()
    });
    assert_eq!(
        robot.metadata["task_config"]["custom"],
        json!({"enabled":true})
    );
    assert_eq!(
        wire(&robot)["control"]["tasks"]["go_to_point"]["allow_unbounded_placement"],
        true
    );
    robot.configure_workspace_experiment(false, " ");
    assert_eq!(
        wire(&robot)["workspace_config"]["experiment"],
        json!({"enabled":false,"contract_version":"experiment.v1"})
    );
}
