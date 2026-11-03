"""Local hospital configuration and adapter contracts; no simulator is required."""

from pathlib import Path
import signal
import argparse
import sys
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("rclpy")
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/hospital"))
sys.path.insert(0, str(ROOT / "python/examples"))

from configuration import mapping_parameters, navigation_parameters, prepare_description, write_configuration, navigation_topic_overrides
from ros_support import encode, HospitalSupport, navigation_cloud
import ros_support
from kit_runtime import local_physics_pose, vector_in_body, quaternion
import runner
from check_frames import frames_ready
from check_streams import INDEXED_MESH_QOS


def test_large_mesh_probe_uses_bounded_reliable_delivery():
    from rclpy.qos import ReliabilityPolicy
    assert INDEXED_MESH_QOS.reliability == ReliabilityPolicy.RELIABLE
    assert INDEXED_MESH_QOS.depth == 1


def test_local_discovery_covers_the_fleet_without_using_lan_multicast():
    ns = {"d":"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    root = ET.parse(ROOT/"simulation/hospital/fastdds-local.xml").getroot()
    udp = root.find("d:transport_descriptors/d:transport_descriptor[d:type='UDPv4']",ns)
    assert int(udp.find("d:maxInitialPeersRange",ns).text) == 100
    assert [v.text for v in root.findall(".//d:address",ns)] == ["127.0.0.1","127.0.0.1"]
    assert root.find("d:transport_descriptors/d:transport_descriptor[d:type='SHM']",ns) is not None
    rtps = root.find("d:participant/d:rtps",ns)
    assert rtps.find("d:useBuiltinTransports",ns).text == "false"
    assert rtps.find("d:builtin/d:metatrafficUnicastLocatorList/d:locator",ns) is not None


def test_frame_check_waits_for_all_robot_and_camera_transforms():
    from builtin_interfaces.msg import Time
    available = set()
    buffer = SimpleNamespace(can_transform=lambda parent, child, stamp: child in available)
    truth = {name: SimpleNamespace(header=SimpleNamespace(stamp=Time()))
             for name in ("carter1", "carter2", "carter3")}
    assert not frames_ready(buffer, truth, ["base_link"])
    available.update(name+"/"+link for name in truth for link in ("base_link", "camera_optical"))
    assert frames_ready(buffer, truth, ["base_link"])
    assert not frames_ready(buffer, {}, ["base_link"])


def test_projected_camera_center_is_in_front_of_each_physical_robot():
    from hospital_fleet_registration import PROJECTED_DISTANCE
    from kit_runtime import HospitalRuntime, ROBOTS

    messages = []
    runtime = SimpleNamespace(
        initial=[(0, 0, 0)] * len(ROBOTS),
        local_static=[SimpleNamespace(publish=lambda _: None) for _ in ROBOTS],
        static=SimpleNamespace(publish=messages.append),
    )
    HospitalRuntime.publish_static(runtime)
    cameras = [value for value in messages[0].transforms
               if value.child_frame_id.endswith("/camera_optical")]
    assert len(cameras) == len(ROBOTS)
    for name, camera in zip(ROBOTS, cameras):
        assert camera.child_frame_id == f"{name}/camera_optical"
        assert camera.header.frame_id == f"{name}/base_link"
        rotation = camera.transform.rotation
        inverse = (rotation.w, -rotation.x, -rotation.y, -rotation.z)
        forward = vector_in_body((0, 0, PROJECTED_DISTANCE), inverse)
        mount = camera.transform.translation
        center = np.array((mount.x, mount.y, mount.z)) + forward
        assert center == pytest.approx((.35 + PROJECTED_DISTANCE, 0, .58))
        assert center[0] > .82 / 2  # Outside the registered robot's front surface.


def test_authoritative_pose_does_not_integrate_wheels_spinning_at_a_wall():
    position,rotation = local_physics_pose((10.8,0,0),quaternion(0),(9.5,0,0))
    assert position == pytest.approx((1.3,0,0))
    assert rotation == pytest.approx((1,0,0,0))
    # Arbitrarily large wheel encoder travel is not an input to the pose conversion.
    assert local_physics_pose((10.8,0,0),quaternion(0),(9.5,0,0))[0] == position


def test_spawn_transform_is_applied_exactly_once():
    import math
    position,rotation = local_physics_pose((4,1,.02),quaternion(math.pi),(4,-1,math.pi/2))
    assert position == pytest.approx((2,0,.02))
    assert rotation == pytest.approx(quaternion(math.pi/2))
    assert vector_in_body((0,1,0),quaternion(math.pi/2)) == pytest.approx((1,0,0))


def test_mapping_retains_coverage_and_uses_three_calibrated_cameras():
    values = mapping_parameters()["/**"]["ros__parameters"]
    assert values["num_cameras"] == 3
    assert values["voxel_size"] == .05
    assert values["global_frame"] == "global_odom"
    assert values["map_clearing_frame_id"] == "carter1/base_link"
    assert values["map_clearing_radius_m"] < 0
    assert values["layer_visualization_exclusion_height_m"] < 0
    assert values["static_mapper"]["workspace_bounds_type"] == "unbounded"


def test_mapping_profile_changes_only_requested_spacing(tmp_path):
    import yaml
    original = mapping_parameters(.03)
    coarser = mapping_parameters(.05)
    original["/**"]["ros__parameters"]["voxel_size"] = .05
    assert original == coarser
    write_configuration(tmp_path, .03)
    assert yaml.safe_load((tmp_path/"nvblox.yaml").read_text())["/**"]["ros__parameters"]["voxel_size"] == .03


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf"])
def test_invalid_mapping_resolution_is_rejected(value):
    with pytest.raises(argparse.ArgumentTypeError):
        runner.positive_resolution(value)


def test_navigation_preserves_local_collision_checks():
    values = navigation_parameters()
    assert values["controller_server"]["ros__parameters"]["FollowPath"]["use_collision_detection"]
    for key in ("local_costmap", "global_costmap"):
        params = values[key][key]["ros__parameters"]
        assert "obstacle_layer" in params["plugins"]
        assert "inflation_layer" in params["plugins"]
        assert "depth_layer" in params["plugins"]
        assert params["depth_layer"]["depth"]["data_type"] == "PointCloud2"


def test_scan_and_depth_topics_are_rewritten_independently(tmp_path):
    from launch import LaunchContext
    from nav2_common.launch import RewrittenYaml
    import yaml
    write_configuration(tmp_path)
    rewritten = Path(RewrittenYaml(source_file=str(tmp_path/"navigation.yaml"),root_key="carter2",
        param_rewrites=navigation_topic_overrides("carter2"),convert_types=True).perform(LaunchContext()))
    try:
        data = yaml.safe_load(rewritten.read_text())["carter2"]
        for name in ("local_costmap","global_costmap"):
            values = data[name][name]["ros__parameters"]
            assert values["obstacle_layer"]["scan"]["topic"] == "/carter2/slam/scan"
            assert values["depth_layer"]["depth"]["topic"] == "/carter2/camera/obstacles"
            assert values["static_layer"]["map_topic"] == "/carter2/map"
    finally:
        rewritten.unlink()


def test_navigation_cloud_keeps_floor_returns_for_clearing_and_preserves_raw_depth():
    from sensor_msgs.msg import CameraInfo
    depth = np.ones((4,4),dtype="<f4")
    depth[0,0] = np.nan
    message = Image(width=4,height=4,step=16,encoding="32FC1",data=depth.tobytes())
    original = bytes(message.data)
    info = CameraInfo(width=4,height=4,k=[2.,0.,2.,0.,2.,2.,0.,0.,1.])
    cloud = navigation_cloud(message,info,stride=1)
    points = np.frombuffer(cloud.data,dtype="<f4").reshape(-1,3)
    assert len(points) == 15 and cloud.is_dense
    assert points[-1] == pytest.approx((.5,.5,1.))
    assert bytes(message.data) == original


def test_generated_yaml_is_complete(tmp_path):
    write_configuration(tmp_path)
    import yaml
    assert {p.name for p in tmp_path.iterdir()} == {"slam.yaml", "navigation.yaml", "nvblox.yaml"}
    assert yaml.safe_load((tmp_path/"slam.yaml").read_text())["/**"]["ros__parameters"]["scan_share_topic"] == "/hospital/localized_scan"


def test_description_fix_is_scoped_and_preserves_geometry(tmp_path):
    source = tmp_path / "source.urdf"
    source.write_text('<robot name="carter"><link name="base"/><joint name="joint_caster_base" type="revolute"><origin xyz="1 2 3"/></joint></robot>')
    original = source.read_bytes()
    output = prepare_description(source, tmp_path/"output.urdf")
    joint = ET.parse(output).getroot().find("joint")
    assert joint.get("type") == "continuous"
    assert joint.find("origin").get("xyz") == "1 2 3"
    assert source.read_bytes() == original


def test_unexpected_invalid_joint_is_not_silently_repaired(tmp_path):
    source = tmp_path / "source.urdf"
    source.write_text('<robot><joint name="arm" type="revolute"/></robot>')
    with pytest.raises(ValueError, match="Unexpected"):
        prepare_description(source, tmp_path/"output.urdf")


@pytest.mark.parametrize("encoding,channels", [("rgb8", 3), ("bgr8", 3), ("rgba8", 4), ("bgra8", 4)])
def test_jpeg_encoding_handles_row_padding_and_rgb_order(encoding, channels):
    data = np.zeros((24, 32, channels), dtype=np.uint8)
    data[:, :, 0 if encoding.startswith("rgb") else 2] = 240
    padded = np.zeros((24, 32*channels+8), dtype=np.uint8)
    padded[:, :32*channels] = data.reshape(24, -1)
    image = Image(width=32, height=24, step=padded.shape[1], encoding=encoding, data=padded.tobytes())
    result = encode(image)
    decoded = cv2.imdecode(np.frombuffer(result.data, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (24, 32, 3)
    assert decoded[:, :, 2].mean() > 230
    assert decoded[:, :, :2].mean() < 10


def test_jpeg_rejects_short_payload():
    with pytest.raises(ValueError, match="payload"):
        encode(Image(width=10, height=10, step=30, encoding="rgb8", data=b"short"))


def test_jpeg_adapter_is_demand_driven():
    submitted = []
    subscribers = [0]
    support = SimpleNamespace(
        camera_pubs={"carter1": SimpleNamespace(get_subscription_count=lambda: subscribers[0])},
        pending={}, pool=SimpleNamespace(submit=lambda *args: submitted.append(args) or object()),
    )
    frame = Image(width=1, height=1, step=3, encoding="rgb8", data=b"\x01\x02\x03")
    HospitalSupport.image(support, "carter1", frame)
    assert submitted == []
    subscribers[0] = 1
    HospitalSupport.image(support, "carter1", frame)
    HospitalSupport.image(support, "carter1", frame)
    assert len(submitted) == 1


def test_missing_process_identity_cannot_authorize_a_signal(monkeypatch):
    calls = []
    monkeypatch.setattr(runner, "process_identity", lambda _: None)
    monkeypatch.setattr(runner.os, "killpg", lambda *args: calls.append(args))
    runner.owned_stop(99999999, None, signal.SIGTERM)
    assert not calls


def test_wait_can_be_cancelled_during_startup():
    class Processes:
        def verify(self):
            pass
    with pytest.raises(InterruptedError):
        runner.wait_for(lambda: False, Processes(), 5, "test", cancelled=lambda: True)


def test_registration_refresh_preserves_other_children_and_existing_log(tmp_path, monkeypatch):
    calls = []
    simulation = SimpleNamespace(pid=11)
    registration = SimpleNamespace(pid=22, wait=lambda timeout: calls.append(("wait",timeout)))
    processes = runner.Processes(tmp_path, {})
    sim_row = dict(name="simulation",start_ticks="1",command=["sim"],finite=False)
    reg_row = dict(name="registration",start_ticks="2",command=["register"],finite=False)
    processes.children = [(simulation,sim_row),(registration,reg_row)]
    monkeypatch.setattr(runner,"owned_stop",lambda *args: calls.append(("signal",*args)))
    monkeypatch.setattr(processes,"start",lambda *args,**kwargs: calls.append(("start",args,kwargs)))
    processes.restart("registration")
    assert processes.children == [(simulation,sim_row)]
    assert calls[0] == ("signal",22,"2",signal.SIGINT,tmp_path)
    assert calls[-1] == ("start",("registration",["register"]),{"finite":False,"append":True})


def test_refresh_command_rejects_old_supervisor_before_sending_a_signal(monkeypatch):
    monkeypatch.setattr(sys,"argv",["hospital-demo","refresh-registration"])
    monkeypatch.setattr(runner,"live_state",lambda: {"phase":"local_streams_ready"})
    monkeypatch.setattr(runner,"owned_stop",lambda *args: pytest.fail("Unsafe signal to old supervisor"))
    with pytest.raises(SystemExit,match="predates registration refresh"):
        runner.main()


@pytest.mark.parametrize("profile", ["full", "lod", "spacewarp", "combined"])
def test_render_profile_refreshes_only_registration(tmp_path, monkeypatch, profile):
    import json
    calls = []
    monkeypatch.setattr(sys, "argv", ["hospital-demo", "render-profile", profile])
    monkeypatch.setattr(runner, "live_state", lambda: {"phase": "local_streams_ready",
        "registration_refresh": True, "run": str(tmp_path), "pid": 123, "start_ticks": "77"})
    monkeypatch.setattr(runner, "owned_stop", lambda *args: calls.append(args))
    runner.main()
    assert calls == [(123, "77", signal.SIGUSR1)]
    value = json.loads((tmp_path / "map-render-profile.json").read_text())
    assert value["lod_enabled"] is True
    assert value["force_full_detail"] == (profile in ("full", "spacewarp"))
    assert value["spacewarp_enabled"] == (profile in ("spacewarp", "combined"))


def test_scans_wait_until_all_cooperative_mappers_are_active():
    from sensor_msgs.msg import LaserScan
    sent = []
    fake = SimpleNamespace(slam_ready={"carter1", "carter2"},
        scan_pubs={"carter1": SimpleNamespace(publish=sent.append)})
    scan = LaserScan()
    scan.header.frame_id = "carter1/laser"
    HospitalSupport.local_scan(fake, "carter1", scan)
    assert not sent
    fake.slam_ready.add("carter3")
    HospitalSupport.local_scan(fake, "carter1", scan)
    assert len(sent) == 1 and sent[0].header.frame_id == "laser"
    assert scan.header.frame_id == "carter1/laser"


def test_slam_readiness_retries_lost_response_without_opening_scan_gate(monkeypatch):
    calls = []
    lost = SimpleNamespace(done=lambda: False,cancel=lambda: calls.append("cancel"))
    retry = object()
    client = SimpleNamespace(service_is_ready=lambda: True,
        remove_pending_request=lambda future: calls.append(("remove",future)),
        call_async=lambda request: retry)
    fake = SimpleNamespace(slam_ready=set(),slam_futures={"carter1":(lost,0)},
        slam_clients={"carter1":client},
        get_logger=lambda: SimpleNamespace(warning=lambda *args,**kwargs: None))
    monkeypatch.setattr(ros_support.time,"monotonic",lambda: 4.)
    HospitalSupport.check_slam_ready(fake)
    assert not fake.slam_ready and not fake.slam_futures
    assert calls == [("remove",lost),"cancel"]
    HospitalSupport.check_slam_ready(fake)
    assert fake.slam_futures == {"carter1":(retry,4.)}


def test_slam_readiness_requires_an_active_lifecycle_state():
    client = SimpleNamespace(service_is_ready=lambda: True)
    for state in (2,3):
        future = SimpleNamespace(done=lambda: True,
            result=lambda: SimpleNamespace(current_state=SimpleNamespace(id=state)))
        fake = SimpleNamespace(slam_ready=set(),slam_futures={"carter1":(future,0)},
            slam_clients={"carter1":client})
        HospitalSupport.check_slam_ready(fake)
        assert ("carter1" in fake.slam_ready) == (state==3)
        assert not fake.slam_futures


@pytest.mark.parametrize("media_mode", ["local", "direct", "relay"])
def test_registration_matches_actual_topics_and_preserves_model_quality(tmp_path, monkeypatch, media_mode):
    from hospital_fleet_registration import build_fleet
    from horus.bridge.robot_registry import RobotRegistryClient
    model = tmp_path / "carter.urdf"
    model.write_text('<robot name="carter"><link name="base_link"/></robot>')
    monkeypatch.setenv("HORUS_HOSPITAL_URDF", str(model))
    remote_media, relay_media = media_mode == "direct", media_mode == "relay"
    robots, views = build_fleet(remote_media=remote_media, relay_media=relay_media)
    assert [r.name for r in robots] == ["carter1", "carter2", "carter3", "hospital_shared_maps"]
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized, client.node = False, None
    client._resolve_robot_description_artifact = lambda _: None
    globals_ = client._build_global_visualizations_payload(views)
    for robot, view in zip(robots[:3], views[:3]):
        config = client._build_robot_config_dict(robot, view, global_visualizations=globals_)
        assert config["control"]["drive_topic"] == f"/{robot.name}/teleop_cmd_vel"
        assert config["control"]["teleop"]["axes"]["invert_angular_z"] is True
        assert config["control"]["teleop"]["axes"]["invert_linear_x"] is False
        assert config["control"]["tasks"]["go_to_point"]["goal_topic"] == f"/{robot.name}/horus/goal_pose"
        assert config["control"]["tasks"]["go_to_point"]["frame_id"] == "global_odom"
        description = robot.get_metadata("robot_description_config")
        assert description["visual_mesh_triangle_budget"] == 500000
        assert description["body_mesh_mode"] == "runtime_high_mesh"
        assert description["visual_link_pose_source"] == "tf"
        camera = next(sensor["camera_config"] for sensor in config["sensors"] if sensor["type"] == "camera")
        assert camera["image_type"] == "compressed"
        assert camera["minimap_image_type"] == camera["teleop_image_type"] == ("raw" if remote_media else "compressed")
        assert camera["minimap_streaming_type"] == ("webrtc" if remote_media or relay_media else "ros")
        assert camera["teleop_streaming_type"] == "webrtc"
        if remote_media or relay_media:
            topic = f"/horus/remote_media/{robot.name}/h264" if relay_media else f"/{robot.name}/camera/color/image_raw"
            prefix = "/horus/webrtc" if relay_media else "/horus/remote/webrtc"
            assert camera["minimap_topic"] == camera["teleop_topic"] == topic
            assert camera["webrtc_client_signal_topic"] == prefix + "/client_signal"
            assert camera["webrtc_server_signal_topic"] == prefix + "/server_signal"
        assert camera["image_scale"] == .11
        assert camera["fov"] == 82.26
        assert camera["show_frustum"] is True
        assert camera["use_tf"] is True
        assert camera["projected_position_offset"] == {"x": 0., "y": 0., "z": 0.}
        assert camera["view_position_offset"] == {"x": 0., "y": 0., "z": 0.}
        assert camera["view_rotation_offset"] == {"x": 0., "y": 0., "z": 0.}
        assert camera["projection_target_frame"] == ""
        camera_sensor = next(sensor for sensor in config["sensors"] if sensor["type"] == "camera")
        assert camera_sensor["frame"] == f"{robot.name}/camera_optical"
        import math
        half_width = .11 * 640 / 360 / 2
        assert 2 * math.degrees(math.atan(half_width / camera["focal_length_scale"])) == pytest.approx(82.26)
        assert camera["focal_length_scale"] < .275
        trail = next(v for v in config["visualizations"] if v["type"] == "odometry_trail")
        assert trail["topic"] == f"/{robot.name}/odom"
        assert trail["frame"] == f"{robot.name}/odom" and trail["enabled"]
        assert trail["trail"]["max_points"] == 48
        assert trail["trail"]["history_seconds"] == 3.2
        paths = [v for v in config["visualizations"] if v["type"] == "path"]
        assert [v["topic"] for v in paths] == [f"/{robot.name}/plan_global"]
        assert all(v["topic"] != f"/{robot.name}/trajectory" for v in config["visualizations"])
    mesh = next(v for v in globals_ if v["type"] == "mesh")
    assert mesh["topic"] == "/horus/hospital/mesh"
    assert mesh["transport_lane"] == "bulk_strict"
    assert mesh["mesh"]["indexed_live"] is True
    assert mesh["mesh"]["max_triangles"] == 2000000
