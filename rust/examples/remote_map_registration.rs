mod common;
use horus::robot::Robot;
use serde_json::json;

fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args().collect();
    anyhow::ensure!(
        args.len() == 2,
        "Usage: remote_map_registration CONFIG_JSON"
    );
    let config: serde_json::Value = serde_json::from_slice(&std::fs::read(&args[1])?)?;
    let mode = config["mode"].as_str().unwrap_or("");
    anyhow::ensure!(
        matches!(mode, "diagnostic" | "rgb" | "h264"),
        "Invalid mode"
    );
    let options = json!({
        "protocol_version":"horus.remote_render.v3",
        "transport":if mode == "diagnostic" {"ros_compressed"} else {"webrtc"},
        "image_type":if mode == "h264" {"h264"} else {"raw"},
        "frame_data_topic":"",
        "session_id":config["session_id"], "viewer_pose_topic":config["view_topic"],
        "status_topic":config["status_topic"], "ros_compressed_topic":config["stream_topic"],
        "viewport_width":config["width"], "viewport_height":config["height"],
        "framerate":config["fps"], "bitrate_kbps":config["bitrate_kbps"],
        "maximum_frame_age_ms":config["maximum_frame_age_ms"],
    });
    anyhow::ensure!(
        options.as_object().unwrap().values().all(|v| !v.is_null()),
        "Use the complete remote_render_config.json template"
    );
    let resource = Robot::workspace_visualization("workspace_remote_map");
    let mut viz = resource.create_dataviz(None);
    viz.add_remote_rendered_map(
        config["stream_topic"]
            .as_str()
            .ok_or_else(|| anyhow::anyhow!("Missing stream_topic"))?,
        "map",
        Some(serde_json::from_value(options)?),
    )
    .map_err(anyhow::Error::msg)?;
    let scale = config["workspace_scale"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("Missing workspace_scale"))?;
    common::register_with_options(
        &mut common::RegistrationSet {
            robots: vec![resource],
            datavizs: vec![viz],
        },
        scale,
        config["keep_alive"].as_bool().unwrap_or(true),
        horus::bridge::RegistryOptions {
            auto_launch: config["auto_launch_bridge"].as_bool().unwrap_or(true),
            ..Default::default()
        },
    );
    Ok(())
}
