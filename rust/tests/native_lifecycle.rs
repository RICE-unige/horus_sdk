use horus::bridge::RobotRegistryClient;

#[test]
fn stop_before_initialization_is_latched() {
    let client = RobotRegistryClient::new();
    client.stop();
    assert!(client.connect().unwrap_err().contains("stopped"));
    assert_eq!(client.status()["stopped"], true);
    assert_eq!(client.status()["initialized"], false);
}

#[test]
fn shared_registry_handles_do_not_hold_a_global_operation_lock() {
    let first = horus::bridge::get_robot_registry_client();
    let second = horus::bridge::get_robot_registry_client();
    assert!(std::sync::Arc::ptr_eq(&first, &second));
    horus::bridge::shutdown_robot_registry();
    assert_eq!(first.status()["stopped"], true);
    let restarted = horus::bridge::get_robot_registry_client();
    assert!(!std::sync::Arc::ptr_eq(&first, &restarted));
    horus::bridge::shutdown_robot_registry();
}
#[tokio::test]
async fn client_shutdown_does_not_stop_another_client() {
    let first = horus::client::Client::new("ros2", false);
    let second = horus::client::Client::new("ros2", false);
    first.shutdown().await;
    assert_eq!(first.registry().unwrap().status()["stopped"], true);
    assert_eq!(second.registry().unwrap().status()["stopped"], false);
    assert!(horus::client::Client::new("unknown", false)
        .registry()
        .is_err());
    first.shutdown().await;
}
