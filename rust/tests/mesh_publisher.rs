use horus::mapping::{LiveMeshPublisher, LiveMeshPublisherOptions};

#[test]
fn publisher_can_be_owned_by_an_application_worker() {
    fn require_send<T: Send>() {}
    require_send::<LiveMeshPublisher>();
}

#[test]
fn publication_requires_ros_feature_and_valid_topic() {
    let options = LiveMeshPublisherOptions {
        topic: "relative/topic".into(),
        ..Default::default()
    };
    let Err(error) = LiveMeshPublisher::new(options) else { panic!("Invalid publisher was accepted"); };
    #[cfg(feature = "ros2")]
    assert!(error.contains("absolute"));
    #[cfg(not(feature = "ros2"))]
    assert!(error.contains("ros2"));
}

#[cfg(feature = "ros2")]
#[test]
fn undelivered_pages_are_retained_and_shutdown_is_owned() {
    use horus::mapping::BlockGeometry;
    for _ in 0..2 {
        let mut publisher = LiveMeshPublisher::new(LiveMeshPublisherOptions {
            topic: "/horus_test_rust_lifecycle".into(), ..Default::default()
        }).unwrap();
        let triangle = BlockGeometry { vertices: vec![[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]],
            triangles: vec![0,1,2], ..Default::default() };
        assert_eq!(publisher.replace("page", Some(&triangle)).unwrap(), 1);
        assert_eq!(publisher.replace("page", Some(&triangle)).unwrap(), 0);
        assert_eq!(publisher.pump().unwrap(), 0);
        assert_eq!(publisher.pending_count(), 1);
        publisher.clear().unwrap();
        assert_eq!(publisher.state().page_count(), 0);
        assert_eq!(publisher.pending_count(), 1);
        publisher.pump().unwrap();
    }
}
