use horus::utils::cli::render_snapshot;
use serde_json::json;
#[test]
fn snapshot_is_bounded_and_does_not_invent_streaming() {
    let state = json!({"registration_state":"Accepted", "heartbeat_seen":true,
        "topics":[{"topic":"/camera", "link":"MEDIA", "transport":"webrtc"}]});
    for width in [48, 80, 120] {
        let text = render_snapshot(&state, width, 24, 0, false).unwrap();
        assert!(text.contains("Accepted") && text.contains("DISCONNECTED"));
        assert!(text.contains("WebRTC") && text.contains("MEDIA"));
        assert!(!text.contains("streaming") && !text.contains('\x1b'));
        assert!(text.lines().all(|line| line.len() == width as usize));
    }
    assert!(render_snapshot(&state, 80, 24, 0, true)
        .unwrap()
        .contains("\x1b[34m"));
}
