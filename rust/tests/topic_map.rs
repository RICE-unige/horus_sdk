use horus::core::topic_map::{TopicInfo, TopicMap};
use horus::core::types::{TopicDirection, TopicType};
use std::sync::Arc;

#[test]
fn topic_catalogue_keeps_metadata_activity_and_type_counts() {
    let map = TopicMap::new();
    let camera = map.add_topic_from_type_str(
        "/robot1/camera/image",
        "sensor_msgs/msg/Image",
        TopicDirection::Publish,
    );
    assert_eq!(camera.get_horus_event_topic(), "robot1.sensor.camera");
    assert!(camera.is_sensor_topic() && camera.is_robot_topic());
    let mut lidar = TopicInfo::new("/points", TopicType::PointCloud, TopicDirection::Subscribe);
    lidar.robot_namespace = Some("g1".into());
    lidar.frame_id = Some("lidar_link".into());
    lidar.qos_profile = Some(serde_json::json!({"reliability": "best_effort"}));
    map.add_topic_info(lidar.clone());
    assert_eq!(
        map.get_topic("/points").unwrap().qos_profile,
        lidar.qos_profile
    );
    assert_eq!(map.get_topics_by_robot("g1").len(), 1);
    assert_eq!(map.get_sensor_topics(None).len(), 2);
    assert_eq!(map.get_sensor_topics(Some("robot1")).len(), 1);
    assert_eq!(map.create_topic_mapping("robot1")["camera"], camera.name);
    assert_eq!(
        map.add_topic("/tf_static", TopicType::TfStatic, TopicDirection::Publish)
            .get_horus_event_topic(),
        "global.tf"
    );
    assert_eq!(
        map.add_topic_from_type_str(
            "/map",
            "nav_msgs/msg/OccupancyGrid",
            TopicDirection::Publish
        )
        .get_horus_event_topic(),
        "global.map"
    );
    map.update_topic_activity(&camera.name, false);
    map.update_topic_activity("/absent", false);
    let stats = map.get_stats();
    assert_eq!(stats["total_topics"], 4);
    assert_eq!(stats["active_topics"], 3);
    assert_eq!(stats["sensor_topics"], 2);
    assert_eq!(stats["robot_count"], 2);
    assert_eq!(stats["topic_types"]["tf2_msgs/TFMessage"], 1);
    assert!(map.remove_topic("/points"));
    assert!(!map.remove_topic("/points"));
    assert_eq!(map.get_robot_namespaces().len(), 1);
}

#[test]
fn simultaneous_insert_remove_and_snapshot() {
    let map = Arc::new(TopicMap::new());
    let writer = Arc::clone(&map);
    let producer = std::thread::spawn(move || {
        for _ in 0..2000 {
            assert_eq!(
                writer
                    .add_topic(
                        "/robot2/scan",
                        TopicType::LaserScan,
                        TopicDirection::Publish
                    )
                    .name,
                "/robot2/scan"
            );
        }
    });
    for _ in 0..2000 {
        map.remove_topic("/robot2/scan");
        map.get_stats();
        map.get_all_topics();
    }
    producer.join().unwrap();
}
