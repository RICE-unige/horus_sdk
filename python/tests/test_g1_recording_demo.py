"""The recorded G1 demo stays observation-only and preserves source geometry."""

import importlib
import json
from pathlib import Path
import sys

import pytest

EXAMPLES = Path(__file__).parents[1] / "examples"
sys.path.insert(0, str(EXAMPLES))
registration = importlib.import_module("g1_recording_registration")
replay = importlib.import_module("tools.g1_recording_replay")
demo = importlib.import_module("tools.g1_recording_demo")


def camera_info():
    return {"session": "recording", "width": 1280, "height": 720,
            "k": [912.8, 0, 649.5, 0, 912.4, 384.6, 0, 0, 1]}


def test_full_registration_has_only_recorded_observation_layers():
    from horus.bridge.robot_registry import RobotRegistryClient
    robot, viz = registration.make_registration(camera_info())
    assert robot.get_capabilities() == dict(controllable=False, teleoperable=False,
        taskable=False, guidable=False, observable=True, communicative=False)
    assert robot.get_ros_binding()["base_frame"] == "floor_anchor"
    model = robot.metadata["robot_description_config"]
    assert model["base_frame"] == "pelvis"
    assert model["visual_link_pose_source"] == "tf"
    assert model["body_mesh_mode"] == "runtime_high_mesh"
    topics = {v.data_source.topic for v in viz.visualizations}
    assert topics == {"/tf", registration.TOPIC + "/walked_path", registration.TOPIC + "/mesh",
                      registration.TOPIC + "/scan", registration.TOPIC + "/camera/compressed"}
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.node, client.ros_initialized = None, False
    payload = client._build_global_visualizations_payload([viz])
    assert len(payload) == 1 and payload[0]["mesh"]["indexed_live"]
    assert not payload[0]["mesh"]["static_mesh"]
    scan = next(v for v in viz.visualizations if v.data_source.topic.endswith("/scan"))
    options = client._serialize_visualization_payload(scan)["point_cloud"]
    assert options["color_mode"] == "rgb_height"
    assert (options["height_min_m"], options["height_max_m"]) == (-1.0, 2.0)
    camera = robot.sensors[0]
    assert camera.frame_id == registration.CAMERA_FRAME
    assert camera.metadata["image_type"] == "compressed"
    assert camera.metadata.get("view_rotation_offset") in (None, {"x": 0.0, "y": 0.0, "z": 0.0})


@pytest.mark.parametrize("mode,low,high,expected", [
    ("height", -2, 5, ("height", -2, 5)),
    ("rgb", -1, 2, ("rgb", -1, 2)),
    ("invalid", float("nan"), float("inf"), ("rgb", -1, 2)),
    (" RGB_HEIGHT ", 2, 1, ("rgb_height", 2, 5)),
])
def test_lidar_palette_payload_is_finite_and_explicit(mode, low, high, expected):
    from horus.bridge.robot_registry import RobotRegistryClient
    _, viz = registration.make_registration(camera_info())
    scan = next(v for v in viz.visualizations if v.data_source.topic.endswith("/scan"))
    scan.render_options.update(color_mode=mode, height_min_m=low, height_max_m=high)
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    options = client._serialize_visualization_payload(scan)["point_cloud"]
    assert (options["color_mode"], options["height_min_m"], options["height_max_m"]) == expected


def test_whitelist_cannot_replay_commands_clock_or_wrong_odometry():
    assert set(replay.SOURCES) == {"pelvis", "joints", "camera_pose", "camera_info", "rgb", "scan", "mesh"}
    assert all("cmd" not in topic and "clock" not in topic and "odometry" not in topic
               for topic, _ in replay.SOURCES.values())


def test_busy_bridge_is_never_stopped(monkeypatch):
    class Busy:
        def __enter__(self): return self
        def __exit__(self, *_): pass
        def settimeout(self, *_): pass
        def connect_ex(self, *_): return 0
    monkeypatch.setattr(demo.socket, "socket", Busy)
    with pytest.raises(RuntimeError, match="Nothing has been stopped"):
        demo.ensure_port_free()


def test_camera_encoder_preserves_size_color_and_row_padding():
    pytest.importorskip("sensor_msgs.msg")
    import cv2
    import numpy as np
    from sensor_msgs.msg import Image
    # Red RGB pixels with four bytes of unrelated row padding.
    row = bytes([230, 20, 10] * 16) + b"pad!"
    msg = Image(width=16, height=12, encoding="rgb8", step=52, data=row * 12)
    encoded = replay.jpeg_message(msg)
    pixels = cv2.imdecode(np.frombuffer(encoded.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert pixels.shape == (12, 16, 3)
    assert abs(float(pixels[:, :, 2].mean()) - 230) < 4
    assert float(pixels[:, :, 0].mean()) < 15
    assert msg.encoding == "rgb8" and bytes(msg.data) == row * 12
    msg.data = msg.data[:-1]
    with pytest.raises(ValueError, match="byte count"):
        replay.jpeg_message(msg)


def test_body_and_floor_anchor_do_not_reorient_or_shift_recorded_body():
    pytest.importorskip("geometry_msgs.msg")
    from geometry_msgs.msg import PoseStamped
    from builtin_interfaces.msg import Time
    pose = PoseStamped()
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 2, -1, 0.8
    pose.pose.orientation.w = 1
    original = replay.pose_transform(pose, registration.ROBOT + "/pelvis", Time(sec=8))
    floor = replay.floor_transform(pose, 0.8, Time(sec=8))
    assert original.transform.translation.z == 0.8
    assert floor.transform.translation.z == pytest.approx(-0.05)
    assert original.transform.translation.x == floor.transform.translation.x == 2
    assert original.transform.rotation == pose.pose.orientation
    assert original.header.frame_id == floor.header.frame_id == "map"
    pose.header.frame_id = "wrong_map"
    with pytest.raises(ValueError, match="unexpected frame"):
        replay.validate_frame("pelvis", pose)


@pytest.mark.parametrize("dataset", ["first", "later"])
def test_camera_mount_uses_saved_rig_and_requires_actual_parent(tmp_path, dataset):
    import numpy as np
    from tools.g1_recording_data import camera_mount
    from scipy.spatial.transform import Rotation
    mount = np.eye(4)
    mount[:3, :3] = Rotation.from_euler('xyz', [0.1, -0.4, 0.7]).as_matrix()
    mount[:3, 3] = [0.04, -0.08, 0.15]
    (tmp_path / 'input').mkdir()
    (tmp_path / 'input/manifest.json').write_text(json.dumps({
        'nominal_camera_from_lidar': np.linalg.inv(mount).tolist()}))
    (tmp_path / 'configuration.json').write_text(json.dumps({'sensor_from_camera': mount.tolist()}))
    path = tmp_path / ('map_input/bag' if dataset == 'first' else 'outputs')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('<robot><link name="mid360_link"/></robot>')
    parent, actual, provenance = camera_mount(path, dataset, urdf)
    assert parent == 'mid360_link' and Path(provenance).is_file()
    np.testing.assert_allclose(actual, mount, atol=1e-12)
    urdf.write_text('<robot><link name="unrelated_lidar"/></robot>')
    with pytest.raises(ValueError, match='mid360_link'):
        camera_mount(path, dataset, urdf)
