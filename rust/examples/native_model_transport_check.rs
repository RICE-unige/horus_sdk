use horus::bridge::{RegistryOptions, RobotRegistryClient};
use horus::core::types::RobotType;
use horus::robot::{Robot, RobotDescriptionConfig};
use serde_json::json;
fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("Expected URDF path"))?;
    if path == "--source" {
        let options = serde_json::from_str(
            &std::env::args()
                .nth(2)
                .ok_or_else(|| anyhow::anyhow!("Expected source options"))?,
        )?;
        println!(
            "{}",
            serde_json::to_string(&horus::description::resolve(&options)?)?
        );
        return Ok(());
    }
    if std::env::args().nth(2).as_deref() == Some("--artifact") {
        let artifact = horus::description::compile_file(
            &json!({"urdf_path":path,"base_frame":"base","body_mesh_mode":"runtime_high_mesh","visual_link_pose_source":"tf"}),
        )?;
        println!("{}", serde_json::to_string(&artifact)?);
        return Ok(());
    }
    let mut robot = Robot::new("native_model", RobotType::Wheeled);
    let mut config = RobotDescriptionConfig::new(path, "base");
    config.body_mesh_mode = "runtime_high_mesh".into();
    config.chunk_size_bytes = 1024;
    config.visual_link_pose_source = "tf".into();
    robot.configure_robot_description(config);
    let client = RobotRegistryClient::with_options(RegistryOptions {
        auto_launch: false,
        ..Default::default()
    });
    let viz = robot.create_dataviz(None);
    let (ok, result) = client.register_robot(&mut robot, &viz, 8., false, false, None);
    if !ok {
        anyhow::bail!("{result}");
    }
    std::thread::sleep(std::time::Duration::from_secs(10));
    Ok(())
}
