use horus::core::types::RobotType;
use horus::robot::Robot;
use horus::utils::{TopicStatusBoard, get_topic_status_board};
use uuid::Uuid;

#[test]
fn board_state_transitions() {
    let board = TopicStatusBoard::new();
    board.set_silent(true);
    board.on_subscribe("/robot1/camera/image_raw");
    board.on_unsubscribe("/robot1/camera/image_raw");
    board.on_subscribe("/robot1/camera/image_raw");

    let snapshot = board.snapshot();
    assert_eq!(
        snapshot.get("/robot1/camera/image_raw").map(|s| s.as_str()),
        Some("SUBSCRIBED")
    );
}

#[test]
fn robot_tf_topic_status_flow() {
    let unique = Uuid::new_v4().simple().to_string();
    let robot_name = format!("drone_{unique}");
    let topic = format!("/{robot_name}/tf");

    let board = get_topic_status_board();
    board.set_silent(true);

    let mut robot = Robot::new(robot_name.clone(), RobotType::Aerial);
    let mut dataviz = horus::DataViz::new("dv");
    dataviz.add_robot_transform(&robot_name, &topic, "world", None);

    let (ok, _) = robot.register_with_horus(Some(dataviz), false, false, None);
    assert!(ok);
    assert_eq!(board.state_for(&topic).map(|s| s.as_str()), Some("SUBSCRIBED"));

    let (ok2, _) = robot.unregister_from_horus();
    assert!(ok2);
    assert_eq!(
        board.state_for(&topic).map(|s| s.as_str()),
        Some("UNSUBSCRIBED")
    );
}

