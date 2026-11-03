use horus::description::{compile_file, compile_urdf};
use serde_json::json;

#[test]
fn articulated_model_has_poses_and_bindings() {
    let path = format!(
        "{}/../contracts/fixtures/robot_model/articulated.urdf",
        env!("CARGO_MANIFEST_DIR")
    );
    let artifact=compile_file(&json!({"urdf_path":path,"base_frame":"base","body_mesh_mode":"runtime_high_mesh","visual_link_pose_source":"tf"})).unwrap();
    assert_eq!(
        artifact.payload["visual_links"].as_array().unwrap().len(),
        2
    );
    assert_eq!(artifact.payload["mesh_assets"][0]["triangle_count"], 2);
    assert!(
        (artifact.payload["visual_links"][1]["origin_xyz"][0]
            .as_f64()
            .unwrap()
            + 1.0)
            .abs()
            < 1e-6
    );
    assert_eq!(
        artifact.payload["joints"][1]["axis_xyz"],
        json!([0.0, 1.0, 0.0])
    );
    assert!(compile_urdf(
        "<robot><link name='a'/><link name='a'/></robot>",
        &json!({})
    )
    .is_err());
}
