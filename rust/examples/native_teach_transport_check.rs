use horus::teaching::{AdapterCapabilities, Episode, TeachAdapter, TeachConfig, TeachRuntime};
use serde_json::{json, Value};

// Test-only echo adapter, with no hardware or command publisher.
struct InputFixture;
impl TeachAdapter for InputFixture {
    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            adapter_id: "input_fixture".into(),
            qualification: "simulation".into(),
            joint_names: vec!["fixture_joint".into()],
            command_frame: "fixture_base".into(),
            calibration_id: "fixture-v1".into(),
            controller_watchdog: "no-hardware-fixture".into(),
            authority_contract: "fixture-session-lease".into(),
            ..Default::default()
        }
    }
    fn ready(&mut self) -> anyhow::Result<(bool, String)> {
        Ok((true, String::new()))
    }
    fn anchor(&mut self, _: &Value) -> anyhow::Result<()> {
        Ok(())
    }
    fn apply(&mut self, sample: &Value) -> anyhow::Result<Value> {
        Ok(json!({"fixture_input":sample,"execution":"none"}))
    }
    fn hold(&mut self, _: &Value) -> anyhow::Result<bool> {
        Ok(true)
    }
}
fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() == 3 && args[1] == "--validate" {
        let issues = Episode::open(&args[2])?.validate()?;
        println!("{}", json!(issues));
        if !issues.is_empty() {
            anyhow::bail!("Episode validation failed");
        }
        return Ok(());
    }
    let v2 = args.len() == 3 && args[2] == "--input-v2";
    if args.len() != 2 && !v2 {
        anyhow::bail!("Provide a temporary recording directory [--input-v2]");
    }
    let mut config = TeachConfig::from_value(
        json!({"recording_root":args[1],"minimum_free_bytes":1024,"provenance":"fixture",
        "streams":[{"stream_id":"state","topic":"/horus_native_teach/state","message_type":"std_msgs/msg/String"},
            {"stream_id":"image","topic":"/horus_native_teach/image","message_type":"sensor_msgs/msg/Image","role":"camera"},
            {"stream_id":"info","topic":"/horus_native_teach/info","message_type":"sensor_msgs/msg/CameraInfo","reliability":"reliable","durability":"transient_local"},
            {"stream_id":"static","topic":"/horus_native_teach/tf_static","message_type":"tf2_msgs/msg/TFMessage","reliability":"reliable","durability":"transient_local"}]}),
    )?;
    if v2 {
        config = config.with_options(json!({"input_schema_version":2,
            "required_input_features":["head_pose","left_stick","right_stick","left_buttons","right_buttons"],
            "clutch_binding":"both_primary", "adapter_id":"input_fixture", "workflow":"collection", "joint_names":["fixture_joint"],
            "joint_groups":{"fixture":["fixture_joint"]},
            "command_frame":"fixture_base", "calibration_id":"fixture-v1",
            "grasp_bindings":{"grasp":"left_grip","pinch":"right_trigger"}}))?;
    }
    let adapter: Option<Box<dyn TeachAdapter>> = if v2 {
        Some(Box::new(InputFixture))
    } else {
        None
    };
    let runtime = TeachRuntime::new(&config, "native_teach", adapter)?;
    for _ in 0..140 {
        let status = runtime.status()?;
        if status["running"] != true {
            anyhow::bail!("{status}");
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    println!("{}", runtime.status()?);
    Ok(())
}
