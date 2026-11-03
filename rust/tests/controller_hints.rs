use anyhow::Result;
use horus::bridge::RobotRegistryClient;
use horus::core::types::RobotType;
use horus::robot::Robot;
use horus::{ControllerControl as C, ControllerHint, ControllerHintContext, ControllerHints};
use serde_json::{json, Value};
use std::collections::HashMap;

#[test]
fn typed_metadata_and_real_registration_match_python_without_control_changes() -> Result<()> {
    let fixture: Value = serde_json::from_str(include_str!(
        "../../contracts/fixtures/controller_hints_registration.json"
    ))?;
    let hints = ControllerHints::from_value(fixture["controller_hints"].clone())?;
    assert_eq!(hints.to_value()?, fixture["controller_hints"]);
    assert_eq!(hints.profiles.len(), 4);
    let mut robot = Robot::new("hint_demo", RobotType::Wheeled);
    let client = RobotRegistryClient::new();
    let wire = |r: &Robot| {
        serde_json::to_value(client.build_robot_config_dict(r, &r.create_dataviz(None), None, None))
            .unwrap()
    };
    let before = wire(&robot);
    assert!(before.get("controller_hints").is_none());
    robot.configure_controller_hints(&hints)?;
    let after = wire(&robot);
    assert_eq!(after["controller_hints"], hints.to_value()?);
    assert_eq!(after["control"], before["control"]);
    robot.clear_controller_hints();
    assert!(wire(&robot).get("controller_hints").is_none());
    let unicode: Value = serde_json::from_str(include_str!(
        "../../contracts/fixtures/controller_hints_unicode.json"
    ))?;
    assert_eq!(
        ControllerHints::from_value(unicode.clone())?.to_value()?,
        unicode
    );
    Ok(())
}
#[test]
fn scoped_updates_are_read_only_and_validate_declared_actions() -> Result<()> {
    let fixture: Value = serde_json::from_str(include_str!(
        "../../contracts/fixtures/controller_hints_registration.json"
    ))?;
    let hints = ControllerHints::from_value(fixture["controller_hints"].clone())?;
    let context = ControllerHintContext::from_json(
        &json!({"schema_version":1,"robot_name":"hint_demo",
        "config_hash":hints.digest()?,"profile_id":"inspection","mode":"teach",
        "context_id":"00000000-0000-4000-8000-000000000001","context_sequence":2})
        .to_string(),
    )?;
    let update = hints
        .reply(
            &context,
            "hint_demo",
            HashMap::from([("tool_mode".into(), "Mode: inspect".into())]),
            1,
        )?
        .to_value()?;
    assert_eq!(update["context_sequence"], 2);
    assert!(update.get("control").is_none());
    assert!(update.get("authority").is_none());
    assert!(hints.reply(&context, "other", HashMap::new(), 0).is_err());
    assert!(hints
        .reply(
            &context,
            "hint_demo",
            HashMap::from([("unknown".into(), "Label".into())]),
            0
        )
        .is_err());
    assert!(hints
        .reply(
            &context,
            "hint_demo",
            HashMap::from([("tool_mode".into(), "bad\n".into())]),
            0
        )
        .is_err());
    assert!(hints
        .reply(&context, "hint_demo", HashMap::new(), -1)
        .is_err());
    assert!(ControllerHint::new(C::LeftPrimary, "bad id", "Label").is_err());
    assert!(ControllerHint::new(C::LeftPrimary, "action", "x".repeat(25)).is_err());
    let mut bad = hints.to_value()?;
    bad["config_hash"] = json!("0".repeat(64));
    assert!(ControllerHints::from_value(bad).is_err());
    Ok(())
}
