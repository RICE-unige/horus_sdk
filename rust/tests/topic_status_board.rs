use horus::utils::{get_topic_status_board, TopicStatusBoard};
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
    board.on_subscribe(&topic);
    assert_eq!(
        board.state_for(&topic).map(|s| s.as_str()),
        Some("SUBSCRIBED")
    );

    board.on_unsubscribe(&topic);
    assert_eq!(
        board.state_for(&topic).map(|s| s.as_str()),
        Some("UNSUBSCRIBED")
    );
}
