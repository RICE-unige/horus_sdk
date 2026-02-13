use horus::bridge::queued_reason_from_ack;
use serde_json::Value;
use std::fs;

fn fixture() -> Value {
    let raw = fs::read_to_string("../contracts/fixtures/queued_status_reasons.json")
        .expect("fixture must exist");
    serde_json::from_str(&raw).expect("fixture must be valid json")
}

#[test]
fn queued_reason_normalization() {
    let expected = fixture();
    for (input, output) in expected.as_object().expect("object fixture") {
        let ack = serde_json::json!({
            "success": true,
            "robot_id": input,
        });
        assert_eq!(
            queued_reason_from_ack(&ack),
            output.as_str().map(ToString::to_string)
        );
    }
}

