#!/usr/bin/env python3
"""Check native remote-map example registration against a local ACK peer."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("executables", nargs="+")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("native_remote_registration_peer")
    acknowledgements = node.create_publisher(String, "/horus/registration_ack", 32)
    received, lock = [], threading.Lock()

    def receive(message):
        payload = json.loads(message.data)
        if payload.get("robot_name") != "workspace_remote_map":
            return
        with lock:
            received.append(payload)
        acknowledgements.publish(String(data=json.dumps(dict(
            success=True, robot_name="workspace_remote_map", robot_id="native-map-test"))))

    node.create_subscription(String, "/horus/registration", receive, QoSProfile(
        depth=32, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    template = json.loads((Path(__file__).resolve().parents[2] /
                           "cpp/examples/remote_render_config.json").read_text())
    try:
        with tempfile.TemporaryDirectory(prefix="horus-native-map-registration-") as directory:
            config_path = Path(directory) / "config.json"
            for index, executable in enumerate(args.executables):
                for mode in ("diagnostic", "rgb", "h264"):
                    config = dict(template, mode=mode, session_id=f"native-map-{index}-{mode}",
                                  auto_launch_bridge=False, keep_alive=False)
                    config_path.write_text(json.dumps(config))
                    result = subprocess.run([executable, str(config_path)], text=True,
                                            capture_output=True, timeout=15)
                    assert result.returncode == 0, result.stdout + result.stderr
                    with lock:
                        matched = [p for p in received if p.get("action") == "register"
                                   and p.get("global_visualizations")
                                   and p["global_visualizations"][0]["remote_render"]["session_id"] == config["session_id"]]
                    assert matched, "No registration matched the renderer session"
                    for payload in matched:
                        assert payload["entity_kind"] == "workspace_visualization"
                        assert not payload["robot_manager_config"]["enabled"]
                        assert not payload["sensors"] and not payload["visualizations"]
                        assert all(v is False for v in payload["capabilities"].values())
                        assert len(payload["global_visualizations"]) == 1
                        remote = payload["global_visualizations"][0]["remote_render"]
                        assert remote["transport"] == ("ros_compressed" if mode == "diagnostic" else "webrtc")
                        assert remote["image_type"] == ("h264" if mode == "h264" else "raw")
                        for field, key in (("viewer_pose_topic", "view_topic"), ("stream_topic", "stream_topic"),
                                           ("ros_compressed_topic", "stream_topic"), ("status_topic", "status_topic"),
                                           ("viewport_width", "width"), ("viewport_height", "height"),
                                           ("framerate", "fps"), ("bitrate_kbps", "bitrate_kbps"),
                                           ("maximum_frame_age_ms", "maximum_frame_age_ms")):
                            assert remote[field] == config[key], (field, remote, config)
                    print(json.dumps(dict(executable=executable, mode=mode, result="passed",
                                          workspace_entity=True, renderer_settings_match=True)), flush=True)
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
