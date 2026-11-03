use horus::tf::{Publisher, PublisherOptions};

#[test]
fn tf_requires_ros_or_publishes_validated_batches() {
    #[cfg(not(feature = "ros2"))]
    assert!(Publisher::new(PublisherOptions::default()).is_err());
    #[cfg(feature = "ros2")]
    {
        use horus::tf::Transform;
        let mut publisher = Publisher::new(PublisherOptions {
            topic: "/rust_tf_test/tf".into(),
            static_topic: "/rust_tf_test/tf_static".into(),
            ..Default::default()
        })
        .unwrap();
        let good = Transform::new("map", "robot/base");
        publisher.publish(&[good.clone()], false).unwrap();
        publisher
            .publish(&[Transform::new("robot/base", "robot/camera")], true)
            .unwrap();
        assert!(publisher
            .publish(&[good.clone(), good.clone()], false)
            .is_err());
        for stamp in [Some(-1), Some(i64::MAX)] {
            let mut bad = good.clone();
            bad.stamp_ns = stamp;
            assert!(publisher.publish(&[bad], false).is_err());
        }
        for name in ["", "/base", "base link", "base\0link", "map"] {
            let mut bad = good.clone();
            bad.child = name.into();
            assert!(publisher.publish(&[bad], false).is_err());
        }
        let mut bad = good.clone();
        bad.translation[0] = f64::NAN;
        assert!(publisher.publish(&[bad], false).is_err());
        let mut bad = good;
        bad.rotation = [0.; 4];
        assert!(publisher.publish(&[bad], false).is_err());
        publisher.publish(&[], false).unwrap();
    }
}
