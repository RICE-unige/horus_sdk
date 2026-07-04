use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::description::{bake_stl_file, base64_encode};
use horus::robot::{Robot, RobotDescriptionConfig};

const CORNERS: [[f32; 3]; 8] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [0.0, 1.0, 1.0],
];

const TRIS: [([f32; 3], [usize; 3]); 12] = [
    ([0.0, 0.0, -1.0], [0, 1, 2]),
    ([0.0, 0.0, -1.0], [0, 2, 3]),
    ([0.0, 0.0, 1.0], [4, 6, 5]),
    ([0.0, 0.0, 1.0], [4, 7, 6]),
    ([0.0, -1.0, 0.0], [0, 5, 1]),
    ([0.0, -1.0, 0.0], [0, 4, 5]),
    ([0.0, 1.0, 0.0], [3, 2, 6]),
    ([0.0, 1.0, 0.0], [3, 6, 7]),
    ([-1.0, 0.0, 0.0], [0, 3, 7]),
    ([-1.0, 0.0, 0.0], [0, 7, 4]),
    ([1.0, 0.0, 0.0], [1, 5, 6]),
    ([1.0, 0.0, 0.0], [1, 6, 2]),
];

fn cube_binary_stl() -> Vec<u8> {
    let mut out = vec![0u8; 80];
    out.extend_from_slice(&(TRIS.len() as u32).to_le_bytes());
    for (normal, idx) in &TRIS {
        for c in normal {
            out.extend_from_slice(&c.to_le_bytes());
        }
        for &i in idx {
            for c in &CORNERS[i] {
                out.extend_from_slice(&c.to_le_bytes());
            }
        }
        out.extend_from_slice(&[0u8, 0u8]);
    }
    out
}

fn cube_ascii_stl() -> String {
    let mut text = String::from("solid cube\n");
    for (normal, idx) in &TRIS {
        text.push_str(&format!("facet normal {} {} {}\n", normal[0], normal[1], normal[2]));
        text.push_str("outer loop\n");
        for &i in idx {
            let v = CORNERS[i];
            text.push_str(&format!("vertex {} {} {}\n", v[0], v[1], v[2]));
        }
        text.push_str("endloop\nendfacet\n");
    }
    text.push_str("endsolid cube\n");
    text
}

#[test]
fn base64_matches_reference() {
    assert_eq!(base64_encode(b"abc"), "YWJj");
    assert_eq!(base64_encode(b"ab"), "YWI=");
    assert_eq!(base64_encode(b"a"), "YQ==");
}

#[test]
fn binary_and_ascii_stl_agree() {
    let dir = std::env::temp_dir().join(format!("horus_stl_fmt_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let bin = dir.join("cube_bin.stl");
    let asc = dir.join("cube_ascii.stl");
    std::fs::write(&bin, cube_binary_stl()).unwrap();
    std::fs::write(&asc, cube_ascii_stl()).unwrap();

    let bin_asset = bake_stl_file(&bin, "cube").expect("binary bake");
    let asc_asset = bake_stl_file(&asc, "cube").expect("ascii bake");

    assert_eq!(bin_asset.triangle_count, 12);
    assert_eq!(bin_asset.vertex_count, 24);
    assert_eq!(bin_asset.bounds_min, [0.0, 0.0, 0.0]);
    assert_eq!(bin_asset.bounds_max, [1.0, 1.0, 1.0]);
    // Same geometry, same baked buffers regardless of STL encoding.
    assert_eq!(bin_asset.positions_b64, asc_asset.positions_b64);
    assert_eq!(bin_asset.indices_b64, asc_asset.indices_b64);
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn bakes_stl_visual_mesh_into_manifest() {
    let dir = std::env::temp_dir().join(format!("horus_stl_manifest_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("cube.stl"), cube_binary_stl()).unwrap();
    let urdf_path = dir.join("robot.urdf");
    std::fs::write(
        &urdf_path,
        r#"<robot name="m"><link name="base_link"><visual><geometry><mesh filename="cube.stl"/></geometry></visual></link></robot>"#,
    )
    .unwrap();

    let mut robot = Robot::new("mesh_bot", RobotType::Wheeled);
    let mut cfg = RobotDescriptionConfig::new(urdf_path.display().to_string(), "base_link");
    cfg.body_mesh_mode = "runtime_high_mesh".to_string();
    robot.configure_robot_description(cfg);
    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let manifest = payload.robot_description_manifest.expect("manifest");

    assert_eq!(manifest["supports_visual_meshes"], true);
    assert_eq!(manifest["mesh_asset_count"].as_i64(), Some(1));
    assert!(manifest["mesh_asset_encoded_bytes"].as_i64().unwrap() > 0);

    let parsed: serde_json::Value =
        serde_json::from_str(&payload.robot_description_payload_json.expect("payload json")).unwrap();
    let asset = &parsed["mesh_assets"][0];
    assert_eq!(asset["triangle_count"].as_i64(), Some(12));
    assert_eq!(asset["vertex_count"].as_i64(), Some(24));
    let bmax = asset["bounds_max"].as_array().unwrap();
    assert_eq!(bmax[0].as_f64(), Some(1.0));
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn collision_only_skips_mesh_baking() {
    let dir = std::env::temp_dir().join(format!("horus_stl_skip_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("cube.stl"), cube_binary_stl()).unwrap();
    let urdf_path = dir.join("robot.urdf");
    std::fs::write(
        &urdf_path,
        r#"<robot name="m"><link name="base_link"><visual><geometry><mesh filename="cube.stl"/></geometry></visual></link></robot>"#,
    )
    .unwrap();

    let mut robot = Robot::new("collision_bot", RobotType::Wheeled);
    let mut cfg = RobotDescriptionConfig::new(urdf_path.display().to_string(), "base_link");
    cfg.body_mesh_mode = "collision_only".to_string();
    robot.configure_robot_description(cfg);
    let dataviz = robot.create_dataviz(None);
    let client = RobotRegistryClient::new();
    let payload = client.build_robot_config_dict(&robot, &dataviz, None, None);
    let manifest = payload.robot_description_manifest.expect("manifest");

    assert_eq!(manifest["supports_visual_meshes"], false);
    assert_eq!(manifest["mesh_asset_count"].as_i64(), Some(0));
    let _ = std::fs::remove_dir_all(&dir);
}
