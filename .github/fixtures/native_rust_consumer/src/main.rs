use horus::bridge::RobotRegistryClient;
use horus::robot::Robot;

fn main() -> Result<(), String> {
    assert!(!horus::remote_rendering::datasets::scene_catalog("").map_err(|e|e.to_string())?.is_null());
    assert_eq!(horus::experiments::workload_from_dict(&serde_json::json!({"camera":{"streams":2}})).map_err(|e|e.to_string())?.stream_count(),2);
    assert_eq!(horus::collision_risk::risk_from_distance(Some(0.5),1.).map_err(|e|e.to_string())?,0.5);
    let _ = horus::remote_rendering::encoder::H264Encoder::compiled();
    assert!(horus::remote_rendering::scene::SceneAsset::load("/missing/scene.ply",Default::default()).is_err());
    assert_eq!(horus::remote_rendering::frame::quantize_depth(&[0.1,10.],0.1,10.)
        .map_err(|e| e.to_string())?, vec![1,65535]);
    let projection = horus::remote_rendering::eye_fov_from_unity_projection(1.,1.,0.,0.)
        .map_err(|e| e.to_string())?;
    assert_eq!(projection.left, -1.);
    assert!(horus::remote_rendering::gpu::CudaRenderer::points("/missing/horus.so",1).is_err());
    let model = horus::description::compile_urdf(
        "<robot name='installed'><link name='base'/></robot>",
        &serde_json::json!({}),
    ).map_err(|e| e.to_string())?;
    assert_eq!(model.manifest["link_count"], 1);
    let config = horus::teaching::TeachConfig::new("/tmp/horus-installed-rust-consumer")
        .map_err(|e| e.to_string())?;
    let mut session = horus::teaching::TeachSession::new(&config, "robot", None)
        .map_err(|e| e.to_string())?;
    assert_eq!(session.state().map_err(|e| e.to_string())?["control_ready"], false);
    session.close().map_err(|e| e.to_string())?;
    let entity = Robot::workspace_visualization("consumer_map");
    let mut view = entity.create_dataviz(None);
    view.add_remote_rendered_map("/consumer/map", "map", None)?;
    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&entity, &view, None, None);
    assert_eq!(payload.entity_kind, "workspace_visualization");
    assert!(!payload.control.teleop.enabled);
    assert_eq!(payload.global_visualizations.len(), 1);
    assert!(payload.global_visualizations[0].remote_render.is_some());
    #[cfg(feature = "ros2")]
    {
        let mut collision=horus::collision_risk::CollisionRiskAnalyzer::new(horus::collision_risk::CollisionRiskOptions{robot_name:"consumer".into(),source_topic:"/installed_rust_consumer/scan".into(),..Default::default()}).map_err(|e|e.to_string())?;
        collision.stop();
        let mut tf = horus::tf::Publisher::new(horus::tf::PublisherOptions {
            topic: "/installed_rust_consumer/tf".into(),
            static_topic: "/installed_rust_consumer/tf_static".into(),
            ..Default::default()
        }).map_err(|e| e.to_string())?;
        tf.publish(&[], false).map_err(|e| e.to_string())?;
        let _ = tf.subscriber_count(false).map_err(|e| e.to_string())?;
        let remote = horus::remote_rendering::publisher::RemotePublisher::new(Default::default())
            .map_err(|e| e.to_string())?;
        assert_eq!(remote.status().map_err(|e| e.to_string())?["state"],"waiting_for_view");
        remote.stop();
        use horus::mapping::{LiveMeshPublisher, LiveMeshPublisherOptions};
        let mut publisher = LiveMeshPublisher::new(LiveMeshPublisherOptions {
            topic: "/rust_consumer_mesh".into(),
            ..Default::default()
        })?;
        assert_eq!(publisher.pump()?, 0);
    }
    client.stop();
    Ok(())
}
