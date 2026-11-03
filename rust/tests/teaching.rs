use anyhow::Result;
use horus::teaching::*;
use serde_json::{json, Value};
use std::sync::{Arc, Mutex};
use std::time::Duration;

#[test]
fn spatial_view_matches_python_and_rejects_wrong_sources() -> Result<()> {
    let mut stream = TeachStream::new("depth", "/robot/front/points", "sensor_msgs/msg/PointCloud2");
    stream.required = false;
    let base = TeachConfig::new("/tmp/episodes")?.with_stream(stream)?;
    let config = base.with_spatial_view(TeachPointCloud::new("depth"))?;
    assert_eq!(config.digest(), "25c4fbfbee7b73fd598a923b7ac5e46b20527f3fa67fc9e8d85ec55c64627e91");
    assert_eq!(config.registration("robot")?["spatial_views"][0]["topic"], "/robot/front/points");
    assert!(config.with_spatial_view(TeachPointCloud::new("depth")).is_err());
    assert!(base.with_spatial_view(TeachPointCloud::new("unknown")).is_err());
    let mut oversized = TeachPointCloud::new("depth");
    oversized.max_points = 2_000_001;
    assert!(base.with_spatial_view(oversized).is_err());
    Ok(())
}

#[test]
fn portal_configuration_matches_python_and_cpp() -> Result<()> {
    let mut camera = TeachCamera::new("front");
    camera.portal = Some(TeachPortal::default());
    let config = TeachConfig::new("/data/teach")?.with_camera(camera.clone())?;
    assert_eq!(config.digest(), "ab34bb8486283d80f600f8f77e8dd22df07a3e787b6f2aac52965014f7175c5f");
    assert_eq!(config.registration("robot")?["cameras"][0]["portal"]["mode"], "stereo");
    camera.portal.as_mut().unwrap().crop = 0.2;
    assert!(TeachConfig::new("/data/teach")?.with_camera(camera).is_err());
    Ok(())
}

#[test]
fn python_configuration_contract() -> Result<()> {
    let fixture: Value =
        serde_json::from_str(include_str!("../../contracts/fixtures/teaching.json"))?;
    for case in fixture["cases"].as_array().unwrap() {
        let config = TeachConfig::from_value(case["input"].clone())?;
        assert_eq!(config.to_value(), &case["normalized"]);
        assert_eq!(config.digest(), case["digest"].as_str().unwrap());
        assert_eq!(config.registration("teach_robot")?, case["registration"]);
    }
    assert!(TeachConfig::new("relative").is_err());
    assert!(
        TeachConfig::from_value(json!({"recording_root":"/tmp/test","publish_rate_hz":0})).is_err()
    );
    let config = TeachConfig::new("/tmp/native-teach")?.with_stream(TeachStream::new(
        "image",
        "/camera",
        "sensor_msgs/msg/Image",
    ))?;
    let mut camera = TeachCamera::new("front");
    camera.stream_id = "image".into();
    camera.required_for_control = true;
    let config = config.with_camera(camera)?;
    assert_eq!(config.to_value()["cameras"][0]["stream_id"], "image");
    assert!(config
        .with_stream(TeachStream::new("image", "/another", "std_msgs/msg/String"))
        .is_err());
    Ok(())
}

struct PanickingAdapter;
impl TeachAdapter for PanickingAdapter {
    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities::default()
    }
    fn ready(&mut self) -> Result<(bool, String)> {
        panic!("Test driver failure")
    }
    fn anchor(&mut self, _: &Value) -> Result<()> {
        unreachable!()
    }
    fn apply(&mut self, _: &Value) -> Result<Value> {
        unreachable!()
    }
    fn hold(&mut self, _: &Value) -> Result<bool> {
        Ok(false)
    }
}
#[test]
fn adapter_panic_is_contained_at_ffi_boundary() -> Result<()> {
    let config = TeachConfig::new("/tmp/horus-teach-panic-test")?;
    let mut session = TeachSession::new(&config, "robot", Some(Box::new(PanickingAdapter)))?;
    let state = session.state()?;
    assert_eq!(state["control_ready"], false);
    let readiness = session.readiness()?;
    assert!(!readiness.0 && readiness.1.contains("panicked"));
    Ok(())
}

struct CartesianFixture {
    outputs: Arc<Mutex<Vec<Value>>>,
}
impl CartesianController for CartesianFixture {
    fn feedback(&mut self) -> Result<Value> {
        Ok(
            json!({"ready":true,"received_monotonic":monotonic_seconds(),"poses":{"left":{"position":[0,0,0],"rotation":[0,0,0,1]}}}),
        )
    }
    fn publish(&mut self, value: &Value) -> Result<Value> {
        self.outputs.lock().unwrap().push(value.clone());
        Ok(json!({"ok":true,"boundary":"accepted"}))
    }
    fn hold(&mut self, _: &Value) -> Result<bool> {
        Ok(true)
    }
}
#[test]
fn cartesian_callbacks_and_local_clock() -> Result<()> {
    let outputs = Arc::new(Mutex::new(Vec::new()));
    let cap = AdapterCapabilities {
        action_representation: "cartesian_pose_absolute".into(),
        authority_contract: "lease".into(),
        command_boundary: "accepted".into(),
        ..Default::default()
    };
    let mut adapter = CartesianAdapter::new(
        cap,
        CartesianLimits::new([-1.; 3], [1.; 3], 1.0, 1.0),
        &["left"],
        json!({}),
        Box::new(CartesianFixture {
            outputs: outputs.clone(),
        }),
    )?;
    assert!(adapter.ready()?.0);
    let sample = json!({"left":{"position":[0,0,0],"rotation":[0,0,0,1]},"sequence":1,
        "authority":{"robot_name":"robot","owner":"operator","session_id":"s1","authority_generation":"g1"}});
    adapter.anchor(&sample)?;
    std::thread::sleep(Duration::from_millis(2));
    adapter.apply(&sample)?;
    assert_eq!(outputs.lock().unwrap().len(), 1);
    assert!(adapter.hold(&json!({}))?);
    assert!(adapter.apply(&sample).is_err());
    Ok(())
}
#[test]
fn native_recording_roundtrip_and_overflow() -> Result<()> {
    let root = std::env::temp_dir().join(format!("horus-rust-teach-{}", uuid::Uuid::new_v4()));
    let config = TeachConfig::from_value(
        json!({"recording_root":root.to_str().unwrap(),"minimum_free_bytes":1024,
        "streams":[{"stream_id":"state","topic":"/state","message_type":"std_msgs/msg/String"}]}),
    )?;
    let id = uuid::Uuid::new_v4().to_string();
    let mut recorder = EpisodeRecorder::new(&config, "teach_robot", &id, "Roundtrip")?;
    let payload = [0, 1, 0, 0, 2, 0, 0, 0, b'a', 0];
    let mut sample = Sample::new("state", &payload);
    sample.schema = "string data\n";
    sample.schema_encoding = "ros2msg";
    assert!(recorder.append(&sample)?);
    assert!(recorder.event("check", json!({"value":42}))?);
    let manifest = recorder.finish("success", false)?;
    assert_eq!(manifest["quality"], "acceptable");
    assert_eq!(recorder.finish("abort", false)?, manifest);
    let path = root.join(&id);
    let episode = Episode::open(path.to_str().unwrap())?;
    assert!(episode.validate()?.is_empty());
    let mut received = 0;
    episode.samples(&["state"], 0, u64::MAX, |meta, bytes| {
        assert_eq!(meta["schema_encoding"], "ros2msg");
        assert_eq!(bytes, &payload);
        received += 1;
        Ok(true)
    })?;
    assert_eq!(received, 1);
    assert!(EpisodeRecorder::new(&config, "teach_robot", &id, "Duplicate").is_err());
    let small = config.with_options(json!({"queue_budget_bytes":1024}))?;
    let mut recorder = EpisodeRecorder::new(
        &small,
        "teach_robot",
        &uuid::Uuid::new_v4().to_string(),
        "Overflow",
    )?;
    assert!(!recorder.event("huge", json!("x".repeat(4096)))?);
    assert_eq!(recorder.finish("abort", true)?["quality"], "unacceptable");
    {
        let collision_id = uuid::Uuid::new_v4().to_string();
        let mut collision =
            EpisodeRecorder::new(&config, "teach_robot", &collision_id, "Conflict")?;
        assert!(collision.event("preserved", json!({"value":42}))?);
        let staging = root.join(format!("{collision_id}.inprogress"));
        let destination = root.join(collision_id);
        std::fs::create_dir(&destination)?;
        assert!(collision.finish("success", false).is_err());
        assert_eq!(std::fs::read_dir(destination)?.count(), 0);
        assert!(staging.join("data.mcap").is_file());
        assert!(Episode::open(staging.to_str().unwrap())?
            .validate()?
            .is_empty());
    }
    std::fs::remove_dir_all(root)?;
    Ok(())
}
struct Fixture {
    counts: Arc<Mutex<[usize; 3]>>,
}
impl TeachAdapter for Fixture {
    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities {
            adapter_id: "fixture".into(),
            qualification: "simulation".into(),
            joint_names: vec!["joint".into()],
            command_frame: "base".into(),
            calibration_id: "v1".into(),
            controller_watchdog: "local".into(),
            authority_contract: "lease".into(),
            ..Default::default()
        }
    }
    fn ready(&mut self) -> Result<(bool, String)> {
        Ok((true, String::new()))
    }
    fn anchor(&mut self, sample: &Value) -> Result<()> {
        assert_eq!(sample["authority"]["owner"], "operator");
        if sample["input_schema_version"] == 2 {
            assert_eq!(sample["head_tracking_valid"], true);
            assert_eq!(sample["left_stick"], json!([0.2, -0.3]));
            assert_eq!(sample["tools"]["left_grasp"], 0.6);
        }
        self.counts.lock().unwrap()[0] += 1;
        Ok(())
    }
    fn apply(&mut self, _: &Value) -> Result<Value> {
        self.counts.lock().unwrap()[1] += 1;
        Ok(json!({"boundary":"accepted"}))
    }
    fn hold(&mut self, _: &Value) -> Result<bool> {
        self.counts.lock().unwrap()[2] += 1;
        Ok(true)
    }
}
fn request(session: &mut TeachSession, config: &TeachConfig, action: &str) -> Result<Value> {
    request_extra(session, config, action, json!({}))
}
fn request_extra(
    session: &mut TeachSession,
    config: &TeachConfig,
    action: &str,
    extra: Value,
) -> Result<Value> {
    let state = session.state()?;
    let mut request = json!({"request_id":uuid::Uuid::new_v4().to_string(),"config_hash":config.digest(),"action":action,
        "owner":"operator","session_id":state["session_id"],"expected_revision":state["revision"],"authority_generation":"g1",
        "alignment_id":state["alignment_id"]});
    request
        .as_object_mut()
        .unwrap()
        .extend(extra.as_object().unwrap().clone());
    session.request(request)
}
fn align(session: &mut TeachSession, config: &TeachConfig) -> Result<()> {
    let result = request_extra(session, config, "align", json!({"alignment": {
        "schema_version":1, "alignment_id":uuid::Uuid::new_v4().to_string(),
        "tracking_origin_id":uuid::Uuid::new_v4().to_string(), "frame_id":"quest_tracking_flu",
        "child_frame_id":"teach_calibrated_floor_flu", "position":[0,0,0], "rotation":[0,0,0,1]}}))?;
    assert_eq!(result["accepted"], true);
    Ok(())
}
fn tracking_input(sample: &mut Value, state: &Value) {
    sample["frame_id"] = json!("quest_tracking_flu");
    sample["alignment_id"] = state["alignment_id"].clone();
    sample["tracking_origin_id"] = state["tracking_origin_id"].clone();
}

#[test]
fn optional_client_diagnostics_survive_recording_without_changing_stop_contract() -> Result<()> {
    let root = std::env::temp_dir().join(format!("horus-teach-diagnostics-{}", uuid::Uuid::new_v4()));
    let config = TeachConfig::from_value(json!({"recording_root":root,"minimum_free_bytes":1024}))?;
    let mut session = TeachSession::new(&config, "robot", None)?;
    assert_eq!(request(&mut session, &config, "open")?["accepted"], true);
    let calibration = json!({"schema_version":1,"alignment_id":uuid::Uuid::new_v4().to_string(),
        "tracking_origin_id":uuid::Uuid::new_v4().to_string(),"frame_id":"quest_tracking_flu",
        "child_frame_id":"teach_calibrated_floor_flu","position":[0,0,0],"rotation":[0,0,0,1]});
    let diagnostic = json!({"build_guid":"tested-build","tracking_origin":"Stage","unknown":false});
    assert_eq!(request_extra(&mut session, &config, "align", json!({"alignment":calibration,
        "client_diagnostics":diagnostic}))?["accepted"], true);
    let id = uuid::Uuid::new_v4().to_string();
    assert_eq!(request_extra(&mut session, &config, "record", json!({"episode_id":id}))?["accepted"], true);
    for _ in 0..500 {
        session.tick()?;
        if session.state()?["capture"] == "recording" { break; }
        std::thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(session.state()?["capture"], "recording");
    assert_eq!(request_extra(&mut session, &config, "invalidate_alignment", json!({"expected_revision":-100,
        "client_diagnostics":{"build_guid":"tested-build","invalidation_reason":"Headset recentered; align again"}}))?["accepted"], true);
    assert_eq!(session.state()?["alignment_id"], "");
    assert_eq!(request(&mut session, &config, "finish")?["accepted"], true);
    for _ in 0..500 {
        session.tick()?;
        if session.state()?["capture"] == "saved" { break; }
        std::thread::sleep(Duration::from_millis(2));
    }
    assert_eq!(session.state()?["capture"], "saved");
    let episode = Episode::open(root.join(id).to_str().unwrap())?;
    let mut found = 0;
    episode.samples(&["__events"], 0, u64::MAX, |_, bytes| {
        let event: Value = serde_json::from_slice(bytes)?;
        let kind = event["kind"].as_str().unwrap_or("");
        if ["alignment", "recording_started", "alignment_invalidated"].contains(&kind) {
            let context = &event["data"]["client_diagnostics"];
            assert_eq!(context["build_guid"], "tested-build");
            assert!(context.get("unknown").is_none());
            if kind == "alignment_invalidated" {
                assert_eq!(context["invalidation_reason"], "Headset recentered; align again");
            }
            found += 1;
        }
        Ok(true)
    })?;
    assert_eq!(found, 3);
    assert_eq!(request_extra(&mut session, &config, "invalidate_alignment", json!({"client_diagnostics":[]}))?["accepted"], true);
    session.close()?;
    std::fs::remove_dir_all(root)?;
    Ok(())
}

#[test]
fn v2_negotiated_input_and_negative_contract() -> Result<()> {
    let fixture: Value =
        serde_json::from_str(include_str!("../../contracts/fixtures/teaching.json"))?;
    let config = TeachConfig::from_value(json!({"recording_root":"/tmp/horus-teach-v2-contract",
        "adapter_id":"fixture", "provenance":"fixture", "workflow":"practice",
        "joint_names":["joint"], "joint_groups":{"arm":["joint"]}, "command_frame":"base", "calibration_id":"v1",
        "input_schema_version":2, "required_input_features":fixture["input_v2"]["features"], "clutch_binding":"both_primary",
        "grasp_bindings":{"left_grasp":"left_grip","right_grasp":"right_grip","left_pinch":"left_trigger","right_pinch":"right_trigger"}}))?;
    let negotiation =
        json!({"input_schema_version":2,"input_features":fixture["input_v2"]["features"]});
    for fault in fixture["input_v2"]["invalid"].as_array().unwrap() {
        let counts = Arc::new(Mutex::new([0; 3]));
        let mut session = TeachSession::new(
            &config,
            "teach_robot",
            Some(Box::new(Fixture {
                counts: counts.clone(),
            })),
        )?;
        assert_eq!(request(&mut session, &config, "open")?["accepted"], false);
        assert_eq!(session.state()?["owner"], "");
        assert_eq!(
            request_extra(&mut session, &config, "open", negotiation.clone())?["accepted"],
            true
        );
        session.update_lease("operator", "g1", 3.0)?;
        align(&mut session, &config)?;
        let armed = request(&mut session, &config, "engage")?;
        assert_eq!(armed["accepted"], true);
        let mut sample = fixture["input_v2"]["sample"].clone();
        tracking_input(&mut sample, &armed);
        sample.as_object_mut().unwrap().extend(
            json!({"owner":"operator", "session_id":armed["session_id"],
            "input_epoch":armed["input_epoch"], "challenge":armed["challenge"], "sequence":1})
            .as_object()
            .unwrap()
            .clone(),
        );
        assert!(session.input(sample.clone())?);
        session.tick()?;
        assert_eq!(counts.lock().unwrap()[0], 1);
        sample["sequence"] = json!(2);
        assert!(session.input(sample.clone())?);
        session.tick()?;
        assert_eq!(counts.lock().unwrap()[1], 1);
        sample["sequence"] = json!(3);
        if let Some(patch) = fault["patch"].as_object() {
            sample.as_object_mut().unwrap().extend(patch.clone());
        }
        if let Some(key) = fault["remove"].as_str() {
            sample.as_object_mut().unwrap().remove(key);
        }
        assert!(!session.input(sample)?);
        session.tick()?;
        assert_eq!(session.state()?["control"], "holding");
        let cause = session.state()?["last_hold_reason"].clone();
        assert!(!cause.as_str().unwrap().is_empty());
        assert_eq!(request(&mut session, &config, "heartbeat")?["last_hold_reason"], cause);
        assert_eq!(counts.lock().unwrap()[1], 1);
        assert_eq!(request(&mut session, &config, "close")?["accepted"], true);
        assert_eq!(session.state()?["input_features"], json!([]));
    }
    Ok(())
}
#[test]
fn continuous_input_requires_negotiation_and_stops_without_auto_resume() -> Result<()> {
    let contract = TeachConfig::from_value(json!({"recording_root":"/data/teach", "input_schema_version":2, "engagement_mode":"continuous"}))?;
    assert_eq!(contract.digest(), "b4a500fb99e6ba3460b1212b74b2bb990be0e4371a3328f88bcf1b6d056690fe");
    let fixture: Value = serde_json::from_str(include_str!("../../contracts/fixtures/teaching.json"))?;
    let config = TeachConfig::from_value(json!({"recording_root":"/tmp/horus-teach-continuous-contract",
        "adapter_id":"fixture", "provenance":"fixture", "workflow":"practice",
        "joint_names":["joint"], "joint_groups":{"arm":["joint"]}, "command_frame":"base", "calibration_id":"v1",
        "input_schema_version":2, "required_input_features":fixture["input_v2"]["features"],
        "engagement_mode":"continuous", "grasp_bindings":{"left_grasp":"left_grip"}}))?;
    for fault in [json!({"left_primary":true,"right_primary":true}), json!({"right_secondary":true}),
        json!({"left_buttons_valid":false}), json!({"tracking_valid":false}), json!({"ui_focus":true}),
        json!({"clutch":false}), json!({"engagement_mode":"hold_to_run"})] {
        let counts = Arc::new(Mutex::new([0; 3]));
        let mut session = TeachSession::new(&config, "teach_robot", Some(Box::new(Fixture {counts: counts.clone()})))?;
        let mut negotiation = json!({"input_schema_version":2,"input_features":fixture["input_v2"]["features"]});
        assert_eq!(request_extra(&mut session, &config, "open", negotiation.clone())?["accepted"], false);
        negotiation["engagement_mode"] = json!("continuous");
        assert_eq!(request_extra(&mut session, &config, "open", negotiation)?["accepted"], true);
        align(&mut session, &config)?;
        session.update_lease("operator", "g1", 3.0)?;
        let armed = request(&mut session, &config, "engage")?;
        assert_eq!(armed["accepted"], true);
        let mut sample = fixture["input_v2"]["sample"].clone();
        tracking_input(&mut sample, &armed);
        sample.as_object_mut().unwrap().extend(json!({"owner":"operator", "session_id":armed["session_id"],
            "input_epoch":armed["input_epoch"], "challenge":armed["challenge"], "sequence":1,
            "engagement_mode":"continuous", "left_primary":false,"right_primary":false}).as_object().unwrap().clone());
        assert!(session.input(sample.clone())?);
        session.tick()?;
        assert_eq!(counts.lock().unwrap()[0], 1);
        sample["sequence"] = json!(2);
        assert!(session.input(sample.clone())?);
        session.tick()?;
        assert_eq!(counts.lock().unwrap()[1], 1);
        sample["sequence"] = json!(3);
        let mut bad = sample.clone();
        bad.as_object_mut().unwrap().extend(fault.as_object().unwrap().clone());
        assert!(!session.input(bad)?);
        session.tick()?;
        assert_eq!(session.state()?["control"], "holding");
        assert!(!session.input(sample)?);
        assert_eq!(counts.lock().unwrap()[1], 1);
    }
    Ok(())
}

struct RecoveryFixture {
    inner: Fixture,
    pauses: Arc<Mutex<usize>>,
}
impl TeachAdapter for RecoveryFixture {
    fn capabilities(&self) -> AdapterCapabilities {
        AdapterCapabilities { supports_tracking_pause: true, ..self.inner.capabilities() }
    }
    fn ready(&mut self) -> Result<(bool, String)> { self.inner.ready() }
    fn anchor(&mut self, value: &Value) -> Result<()> { self.inner.anchor(value) }
    fn apply(&mut self, value: &Value) -> Result<Value> { self.inner.apply(value) }
    fn hold(&mut self, value: &Value) -> Result<bool> { self.inner.hold(value) }
    fn pause(&mut self, value: &Value) -> Result<bool> {
        assert_eq!(value["owner"], "operator");
        *self.pauses.lock().unwrap() += 1;
        Ok(true)
    }
}

#[test]
fn recovery_callback_preserves_anchor_but_explicit_stop_is_latched() -> Result<()> {
    let contract = TeachConfig::new("/data/teach")?.with_options(json!({"provenance":"fixture",
        "input_schema_version":2,"engagement_mode":"continuous","tracking_recovery":"auto_resume"}))?;
    assert_eq!(contract.digest(), "4073155edb3d2526d447ef4c370f81606329f467b8aa2171989db15bed9a6127");
    let fixture: Value = serde_json::from_str(include_str!("../../contracts/fixtures/teaching.json"))?;
    let config = TeachConfig::from_value(json!({"recording_root":"/tmp/horus-teach-recovery-contract",
        "adapter_id":"fixture", "provenance":"fixture", "workflow":"practice",
        "joint_names":["joint"], "joint_groups":{"arm":["joint"]}, "command_frame":"base", "calibration_id":"v1",
        "input_schema_version":2, "required_input_features":fixture["input_v2"]["features"],
        "engagement_mode":"continuous", "tracking_recovery":"auto_resume", "grasp_bindings":{"left_grasp":"left_grip"}}))?;
    assert!(config.with_options(json!({"provenance":"real"})).is_err());
    assert_eq!(config.registration("teach_robot")?["tracking_recovery"], "auto_resume");
    let counts = Arc::new(Mutex::new([0; 3]));
    let pauses = Arc::new(Mutex::new(0));
    let mut session = TeachSession::new(&config, "teach_robot", Some(Box::new(RecoveryFixture {
        inner: Fixture { counts: counts.clone() }, pauses: pauses.clone()
    })))?;
    let negotiation = json!({"input_schema_version":2,"input_features":fixture["input_v2"]["features"],
        "engagement_mode":"continuous", "tracking_recovery":"auto_resume"});
    assert_eq!(request_extra(&mut session, &config, "open", negotiation)?["accepted"], true);
    align(&mut session, &config)?;
    session.update_lease("operator", "g1", 3.0)?;
    let armed = request(&mut session, &config, "engage")?;
    assert_eq!(armed["accepted"], true);
    let mut sample = fixture["input_v2"]["sample"].clone();
    tracking_input(&mut sample, &armed);
    sample.as_object_mut().unwrap().extend(json!({"owner":"operator", "session_id":armed["session_id"],
        "input_epoch":armed["input_epoch"], "challenge":armed["challenge"], "sequence":1,
        "engagement_mode":"continuous", "tracking_recovery":"auto_resume",
        "left_primary":false,"right_primary":false}).as_object().unwrap().clone());
    assert!(session.input(sample.clone())?); session.tick()?;
    for seq in [2, 4, 6] {
        let mut lost = sample.clone(); lost["sequence"] = json!(seq);
        lost["left_tracking_valid"] = json!(false); lost["left"] = Value::Null;
        lost["challenge"] = session.state()?["challenge"].clone();
        assert!(!session.input(lost)?);
        assert_eq!(session.state()?["control"], "paused");
        assert_eq!(session.state()?["input_epoch"], armed["input_epoch"]);
        sample["sequence"] = json!(seq + 1);
        sample["challenge"] = session.state()?["challenge"].clone();
        assert!(session.input(sample.clone())?); session.tick()?;
    }
    assert_eq!(*counts.lock().unwrap(), [1, 3, 0]);
    assert_eq!(*pauses.lock().unwrap(), 3);
    sample["sequence"] = json!(8); sample["right_secondary"] = json!(true); sample["challenge"] = json!("old");
    assert!(!session.input(sample.clone())?);
    assert_eq!(session.state()?["control"], "holding");
    sample["sequence"] = json!(9); sample["right_secondary"] = json!(false);
    sample["challenge"] = session.state()?["challenge"].clone();
    assert!(!session.input(sample)?);
    assert_eq!(*counts.lock().unwrap(), [1, 3, 1]);
    Ok(())
}

#[test]
fn native_adapter_lifecycle() -> Result<()> {
    let config = TeachConfig::from_value(
        json!({"recording_root":"/tmp/horus-teach-contract","adapter_id":"fixture","provenance":"fixture","workflow":"practice",
        "joint_names":["joint"],"joint_groups":{"arm":["joint"]},"command_frame":"base","calibration_id":"v1"}),
    )?;
    let counts = Arc::new(Mutex::new([0; 3]));
    let mut session = TeachSession::new(
        &config,
        "teach_robot",
        Some(Box::new(Fixture {
            counts: counts.clone(),
        })),
    )?;
    assert_eq!(request(&mut session, &config, "open")?["accepted"], true);
    assert_eq!(request(&mut session, &config, "engage")?["accepted"], false);
    session.update_lease("operator", "g1", 3.0)?;
    assert_eq!(request(&mut session, &config, "engage")?["accepted"], false);
    align(&mut session, &config)?;
    let state = request(&mut session, &config, "engage")?;
    assert_eq!(state["control"], "armed");
    let pose = json!({"position":[0,0,0],"rotation":[0,0,0,1]});
    let mut sample = json!({"session_id":state["session_id"],"input_epoch":state["input_epoch"],"challenge":state["challenge"],"owner":"operator",
        "sequence":0,"tracking_valid":true,"clutch":true,"ui_focus":false,"left":pose,"right":pose});
    tracking_input(&mut sample, &state);
    assert!(session.input(sample.clone())?);
    session.tick()?;
    assert_eq!(*counts.lock().unwrap(), [1, 0, 0]);
    assert!(!session.input(sample.clone())?);
    sample["sequence"] = json!(1);
    assert!(session.input(sample)?);
    session.tick()?;
    assert_eq!(*counts.lock().unwrap(), [1, 1, 0]);
    std::thread::sleep(Duration::from_millis(220));
    session.tick()?;
    assert_eq!(session.state()?["control"], "holding");
    assert_eq!(*counts.lock().unwrap(), [1, 1, 1]);
    session.close()?;
    Ok(())
}
