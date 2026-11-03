use horus::remote_rendering::scene::{SceneAsset, SceneLoadOptions};
#[test]
fn full_scene_import_and_borrowed_buffers() {
    let root =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../contracts/fixtures/remote_scene");
    let points = SceneAsset::load(
        root.join("points.ply"),
        SceneLoadOptions {
            world_scale: 2.,
            bake_lighting: false,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(points.vertices().len(), 3);
    assert!(points.faces().is_empty());
    assert_eq!(points.vertices()[0], [2., 4., 6.]);
    assert_eq!(
        points.point_colors(),
        &[[255, 0, 0], [0, 255, 0], [0, 0, 255]]
    );
    assert!(points.mesh_input().is_err());
    let mesh = SceneAsset::load(
        root.join("triangle.obj"),
        SceneLoadOptions {
            bake_lighting: false,
            ..Default::default()
        },
    )
    .unwrap();
    assert_eq!(mesh.faces().len(), 1);
    assert_eq!(mesh.face_colors()[0], [255, 0, 0]);
    assert_eq!(mesh.uvs()[0], [0., 1.]);
    assert_eq!(
        mesh.mesh_input().unwrap().vertices.as_ptr(),
        mesh.vertices().as_ptr()
    );
    assert_eq!(mesh.texture_atlas().len(), 256);
    assert!(SceneAsset::load(
        root.join("triangle.obj"),
        SceneLoadOptions {
            maximum_vertices: 2,
            ..Default::default()
        }
    )
    .is_err());
    assert!(SceneAsset::load(root.join("missing.obj"), Default::default()).is_err());
}
