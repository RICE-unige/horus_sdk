import importlib
import json
import math
import os
import subprocess
import time
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / "examples"))
scene = importlib.import_module("tools.field_teammate_scene")


def manifest(low=-1.6, high=0.4, epoch="12"):
    return {"epoch": epoch, "frame_id": "partner_scene/source", "source_frame_id": "map",
            "height_bounds_m": [low, high], "triangles": 1000}


def test_observed_geometry_datum_locks_once_and_preserves_the_source_frame():
    datum = scene.SceneDatum()
    assert not datum.observe(manifest(-1.58))
    assert not datum.observe(manifest(-1.60))
    assert datum.observe(manifest(-1.59))
    assert datum.floor_z == -1.60 and datum.source_frame == "map"
    assert datum.observe(manifest(-2.0))
    assert datum.floor_z == -1.60  # Walking must not make the complete workspace move.


def test_new_map_clear_resets_the_visual_datum_even_before_geometry_returns():
    datum = scene.SceneDatum()
    for _ in range(3):
        datum.observe(manifest())
    cleared = manifest(epoch="13")
    cleared["height_bounds_m"] = None
    assert not datum.observe(cleared)
    assert datum.floor_z is None and not datum.samples


@pytest.mark.parametrize("value", [None, [0, math.nan], [0, math.inf], [False, 2], [0], [0, .1]])
def test_invalid_or_insufficient_geometry_does_not_invent_a_floor(value):
    datum = scene.SceneDatum()
    record = manifest()
    record["height_bounds_m"] = value
    for _ in range(4):
        assert not datum.observe(record)
    assert datum.floor_z is None


def pose(x=0.0, z=0.0, sec=5, frame="map"):
    stamp_ns = round(sec * 1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id=frame,
        stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)), pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=0.0, z=z),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def test_native_camera_and_depth_frames_must_agree_not_the_legacy_profile_height():
    assert scene.camera_pose_is_consistent(pose(.05), pose(), "map")
    assert not scene.camera_pose_is_consistent(pose(z=1.61), pose(), "map")
    assert not scene.camera_pose_is_consistent(pose(x=3), pose(), "map")
    assert not scene.camera_pose_is_consistent(pose(sec=6), pose(), "map")
    assert not scene.camera_pose_is_consistent(pose(frame="other"), pose(), "map")


def test_uniform_datum_preserves_distances_and_puts_the_native_head_above_geometry():
    low = -1.6
    camera, wall = (0.0, 0.0, .1), (1.0, .5, -.8)
    placed_camera = (camera[0], camera[1], camera[2] - low)
    placed_wall = (wall[0], wall[1], wall[2] - low)
    assert placed_camera[2] == pytest.approx(1.7)
    assert math.dist(placed_camera, placed_wall) == pytest.approx(math.dist(camera, wall))
    assert scene.scene_frames("partner") == (
        "partner_scene/floor", "partner_scene/source", "partner_scene/camera")


def tracking_packet(t, kind="head", position=(0., 0., 0.), orientation=(0., 0., 0., 1.),
                    revision=1, valid=True, session="relay", name="partner", stamp=None):
    stamp_ns = round((1000 + t if stamp is None else stamp) * 1e9)
    return {"schema": "horus.field_tracking.v1", "session_id": session,
        "header": {"frame_id": "map", "stamp": {"sec": stamp_ns // 1_000_000_000,
                                                  "nanosec": stamp_ns % 1_000_000_000}},
        "child_frame_id": f"{name}/{kind}",
        "pose_source": "unity_head" if kind == "head" else "hololens_locatable_pv_camera",
        "source_timestamp_sec": t, "tracking_origin_revision": revision,
        "tracking_valid": valid, "position": position if valid else None,
        "orientation_xyzw": orientation if valid else None}


def head(tracking, t, **kwargs):
    return tracking.observe_head(tracking_packet(t, **kwargs), t, 1000 + t)


def camera(tracking, t, **kwargs):
    return tracking.observe_camera(tracking_packet(t, "pv_camera", **kwargs), t, 1000 + t)


def align(tracking, start=10., position=(0., 0., 0.), revision=1, session="relay", depth=True):
    for i in range(3):
        t = start + .12 * i
        head(tracking, t, position=position, revision=revision, session=session)
        if depth:
            tracking.observe_depth(pose(x=position[0], z=position[2], sec=1000 + t), t)
        camera(tracking, t, position=(position[0] + .06, position[1], position[2]),
               revision=revision, session=session)


def aligned_tracker():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    align(tracking)
    assert tracking.aligned
    return tracking


def test_startup_requires_fresh_native_head_pv_and_depth_in_one_origin():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    align(tracking, depth=False)
    assert not tracking.aligned
    assert head(tracking, 10.28) is None
    tracking.observe_depth(pose(sec=1000 + 10.28), 10.28)
    assert tracking.aligned
    result = head(tracking, 10.32)
    assert result[0].position == (0., 0., 0.)
    assert result[1][0] == pytest.approx((.06, 0., 0.))


@pytest.mark.parametrize("recover", [False, True])
def test_thirty_fps_camera_calibrates_over_two_hundred_ms_at_startup_and_revision_recovery(recover):
    tracking = aligned_tracker() if recover else scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    start, revision = (10.28, 2) if recover else (10., 1)
    for i in range(7):
        t = start + i / 30
        head(tracking, t, revision=revision)
        tracking.observe_depth(pose(sec=1000 + t), t)
        camera(tracking, t, position=(.06, 0., 0.), revision=revision)
        assert tracking.aligned is (i == 6)
    times = [t for t, _ in tracking.candidates]
    assert times == pytest.approx([start, start + .1, start + .2])
    assert head(tracking, start + .24, revision=revision) is not None


@pytest.mark.parametrize("position,orientation", [
    ((.12, 0., 0.), (0., 0., 0., 1.)),
    ((.06, 0., 0.), (0., 0., math.sin(.075), math.cos(.075))),
    ((.5, 0., 0.), (0., 0., 0., 1.)),
])
def test_inconsistent_intermediate_camera_samples_reset_time_spaced_calibration(position, orientation):
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    for i in range(7):
        t = 10 + i / 30
        head(tracking, t)
        tracking.observe_depth(pose(sec=1000 + t), t)
        camera(tracking, t, position=position if i == 1 else (.06, 0., 0.),
               orientation=orientation if i == 1 else (0., 0., 0., 1.))
        assert not tracking.aligned
    assert not tracking.candidates or tracking.candidates[0][0] >= 10 + 2 / 30
    for i in range(7, 30):
        t = 10 + i / 30
        head(tracking, t)
        tracking.observe_depth(pose(sec=1000 + t), t)
        camera(tracking, t, position=(.06, 0., 0.))
        if tracking.aligned:
            break
    assert tracking.aligned


def test_unaligned_highrate_stream_bounds_paired_times_to_the_camera_buffer():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    for i in range(180):
        t = 10 + i / 30
        head(tracking, t)
        camera(tracking, t, position=(.06, 0., 0.))
        assert not tracking.aligned  # No depth observation yet.
        assert tracking.paired_times <= {sample.source_time for sample, _ in tracking.cameras}
        assert len(tracking.paired_times) <= tracking.cameras.maxlen


def test_two_second_depth_hiatus_keeps_every_fresh_head_and_camera_tf():
    tracking = aligned_tracker()
    for i in range(1, 61):
        t = 10.24 + i / 30
        yaw = .4 * (t - 10.24)
        position = (.2 * (t - 10.24), 0., -.2)
        q = (0., 0., math.sin(yaw / 2), math.cos(yaw / 2))
        sample, pv = head(tracking, t, position=position, orientation=q)
        assert sample.stamp_ns == round((1000 + t) * 1e9)
        assert sample.position == position
        expected = (position[0] + .06 * math.cos(yaw), .06 * math.sin(yaw), position[2])
        assert pv[0] == pytest.approx(expected)
        assert pv[1] == pytest.approx(q)
        if i % 4 == 0:
            camera(tracking, t, position=expected, orientation=q)
        assert tracking.status(t) == "live"
    # A delayed depth callback is not a coordinate disagreement.
    tracking.observe_depth(pose(sec=1010.24), 12.24)
    assert head(tracking, 12.28, position=(.41, 0., -.2), orientation=q) is not None


UNITY_HEAD = (.393877, -.539827, 1.153283)
NATIVE_PV = (.314766, -.543861, -.410723)


def native_depth(t, position=NATIVE_PV):
    message = pose(x=position[0], z=position[2], sec=1000 + t)
    message.pose.position.y = position[1]
    return message


def native_tracker():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    assert head(tracking, 10., position=UNITY_HEAD) is None
    tracking.observe_depth(native_depth(10.), 10.)
    sample = camera(tracking, 10., position=NATIVE_PV)
    assert sample is not None
    return tracking


def test_real_unity_native_origin_offset_uses_exact_pv_through_two_second_depth_gap():
    tracking = native_tracker()
    q = (0., 0., math.sin(.1), math.cos(.1))
    for i in range(1, 17):
        t = 10 + i / 8
        assert head(tracking, t, position=UNITY_HEAD) is None
        sample = camera(tracking, t, position=NATIVE_PV, orientation=q)
        assert sample is not None
        assert sample.position == NATIVE_PV
        assert sample.orientation == pytest.approx(q)
        assert sample.stamp_ns == round((1000 + t) * 1e9)
        assert tracking.pose_source == "native_pv" and tracking.status(t) == "live"
        assert tracking.extrinsic is None and not tracking.aligned
    assert tracking.status(12.6) == "waiting_for_fresh_native_camera_pose"


def test_unity_only_origin_jump_does_not_block_continuous_valid_native_pv():
    tracking = native_tracker()
    moved_head = (UNITY_HEAD[0], UNITY_HEAD[1], UNITY_HEAD[2] + 2)
    assert head(tracking, 10.04, position=moved_head) is None
    assert tracking.blocked and tracking.native_aligned
    sample = camera(tracking, 10.125, position=NATIVE_PV)
    assert sample is not None and sample.position == NATIVE_PV
    assert tracking.status(10.125) == "live" and tracking.pose_source == "native_pv"


def test_native_pv_revision_or_invalidity_requires_new_depth_validation():
    tracking = native_tracker()
    head(tracking, 10.04, position=UNITY_HEAD, revision=2)
    assert not tracking.native_aligned
    assert camera(tracking, 10.125, position=NATIVE_PV, revision=1) is None
    assert camera(tracking, 10.125, position=NATIVE_PV, revision=2) is None
    tracking.observe_depth(native_depth(10.), 10.125)
    assert not tracking.native_aligned
    tracking.observe_depth(native_depth(10.125), 10.125)
    assert camera(tracking, 10.25, position=NATIVE_PV, revision=2) is not None
    assert camera(tracking, 10.375, revision=2, valid=False) is None
    assert not tracking.native_aligned
    head(tracking, 10.4, position=UNITY_HEAD, revision=2)
    assert camera(tracking, 10.5, position=NATIVE_PV, revision=2) is None
    tracking.observe_depth(native_depth(10.5), 10.5)
    assert camera(tracking, 10.625, position=NATIVE_PV, revision=2) is not None


def test_safe_highrate_head_mode_does_not_double_publish_delayed_native_pv():
    tracking = aligned_tracker()
    assert head(tracking, 10.28) is not None
    packet = tracking_packet(10.29, "pv_camera", position=(.06, 0., 0.), stamp=1010.26)
    assert tracking.observe_camera(packet, 10.29, 1010.29) is None
    assert tracking.pose_source == "measured_head_with_observed_rigid_pv_extrinsic"
    assert head(tracking, 10.32) is not None


def test_tracking_loss_never_replays_pose_and_requires_new_alignment():
    tracking = aligned_tracker()
    assert head(tracking, 10.28, valid=False) is None
    assert not tracking.aligned and tracking.state == "waiting_for_head_tracking"
    assert head(tracking, 10.32) is None
    tracking.observe_depth(pose(sec=1010.24), 10.32)  # Queued before invalidation.
    align(tracking, start=10.36, depth=False)
    assert not tracking.aligned
    tracking.observe_depth(pose(sec=1010.60), 10.60)
    assert head(tracking, 10.64) is not None


def test_origin_revision_change_rejects_old_pv_and_requires_fresh_depth():
    tracking = aligned_tracker()
    assert head(tracking, 10.28, revision=2) is None
    assert not tracking.aligned
    for t in (10.32, 10.44, 10.56):
        head(tracking, t, revision=2)
        camera(tracking, t, revision=1)
    assert not tracking.candidates
    align(tracking, start=10.60, revision=2, depth=False)
    assert not tracking.aligned
    tracking.observe_depth(pose(sec=1010.24), 10.84)
    assert not tracking.aligned
    tracking.observe_depth(pose(sec=1010.84), 10.84)
    assert head(tracking, 10.88, revision=2) is not None
    assert head(tracking, 10.90, revision=1) is None
    assert head(tracking, 10.92, revision=1, valid=False) is None
    assert tracking.aligned and tracking.origin == ("relay", 2)


@pytest.mark.parametrize("revision", [1, 2])
def test_untrusted_head_jump_stops_head_driving_but_preserves_authoritative_native_pose(revision):
    tracking = aligned_tracker()
    assert head(tracking, 10.28, position=(4., 0., 0.), revision=revision) is None
    align(tracking, start=10.32, position=(4., 0., 0.), revision=revision)
    assert tracking.blocked and not tracking.aligned
    assert tracking.status(10.6) == "live"
    assert tracking.pose_source == "native_pv"
    tracking.observe_geometry("13", "map")  # Main owns the actual map reset, not this adapter.
    align(tracking, start=11., position=(4., 0., 0.), revision=revision)
    assert head(tracking, 11.28, position=(4., 0., 0.), revision=revision) is not None


def test_rotational_head_jump_disables_head_driving_not_a_valid_native_camera():
    tracking = aligned_tracker()
    assert head(tracking, 10.28, orientation=(0., 0., 1., 0.)) is None
    assert tracking.blocked


def test_asynchronous_depth_disagreement_cannot_latch_a_valid_native_camera_forever():
    tracking = native_tracker()
    assert head(tracking, 10.25, position=UNITY_HEAD) is None
    tracking.observe_depth(native_depth(10.25, (NATIVE_PV[0] + 1., *NATIVE_PV[1:])), 10.25)
    sample = camera(tracking, 10.25, position=NATIVE_PV)
    assert sample is not None and sample.position == NATIVE_PV
    assert tracking.native_depth_residual_m == pytest.approx(1.)
    assert tracking.native_aligned and tracking.status(10.25) == "live"
    for i in range(1, 25):
        t = 10.25 + i / 8
        head(tracking, t, position=UNITY_HEAD)
        assert camera(tracking, t, position=NATIVE_PV) is not None


def test_network_arrival_burst_uses_capture_clock_for_native_motion_not_receive_interval():
    tracking = native_tracker()
    moved = (NATIVE_PV[0] + .25, *NATIVE_PV[1:])
    head(tracking, 10.125, position=UNITY_HEAD)
    packet = tracking_packet(10.125, "pv_camera", position=moved, stamp=1010.001)
    sample = tracking.observe_camera(packet, 10.125, 1010.125)
    assert sample is not None and sample.position == moved
    assert tracking.native_motion_discontinuities == 0


def test_same_revision_native_delivery_gap_does_not_demand_an_unrelated_new_depth_frame():
    tracking = native_tracker()
    head(tracking, 12.125, position=UNITY_HEAD)
    sample = camera(tracking, 12.125, position=NATIVE_PV)
    assert sample is not None and sample.position == NATIVE_PV
    assert tracking.status(12.125) == "live"


def test_a_tracked_native_discontinuity_is_reported_not_silently_frozen():
    tracking = native_tracker()
    head(tracking, 10.125, position=UNITY_HEAD)
    moved = (NATIVE_PV[0] + 3., *NATIVE_PV[1:])
    sample = camera(tracking, 10.125, position=moved)
    assert sample is not None and sample.position == moved
    assert tracking.native_motion_discontinuities == 1


def test_legitimate_walk_during_same_origin_gap_revalidates_from_head_and_native_pv():
    tracking = aligned_tracker()
    assert head(tracking, 11.74, position=(1.5, 0., 0.)) is None
    assert not tracking.blocked and not tracking.aligned
    align(tracking, start=11.78, position=(1.5, 0., 0.), depth=False)
    assert tracking.aligned and tracking.native_aligned
    tracking.observe_depth(pose(x=1.5, sec=1012.02), 12.02)
    assert head(tracking, 12.06, position=(1.5, 0., 0.)) is not None


def test_changed_origin_during_unobserved_motion_cannot_realign_to_the_old_map():
    tracking = aligned_tracker()
    assert head(tracking, 11.74, position=(1.5, 0., 0.), revision=2) is None
    align(tracking, start=11.78, position=(1.5, 0., 0.), revision=2)
    assert tracking.blocked and not tracking.aligned
    tracking.observe_geometry("13", "map")
    align(tracking, start=12.1, position=(1.5, 0., 0.), revision=2)
    assert tracking.aligned


def test_incompatible_legacy_source_clocks_never_guess_a_head_pv_extrinsic():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    for i in range(5):
        t = 10 + .12 * i
        head(tracking, t)
        packet = tracking_packet(1_700_000_000 + t, "pv_camera", stamp=1000 + t)
        tracking.observe_camera(packet, t, 1000 + t)
        tracking.observe_depth(pose(sec=1000 + t), t)
    assert not tracking.aligned and tracking.extrinsic is None
    assert tracking.state == "waiting_for_shared_capture_clock"


def test_pv_extrinsic_preserves_nonidentity_rotation_and_local_translation():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    angle = .15
    pv_q = (0., math.sin(angle / 2), 0., math.cos(angle / 2))
    for i in range(3):
        t = 10 + .12 * i
        head(tracking, t)
        camera(tracking, t, position=(.04, .02, -.03), orientation=pv_q)
        tracking.observe_depth(pose(sec=1000 + t), t)
    assert tracking.aligned
    yaw_q = (0., 0., math.sqrt(.5), math.sqrt(.5))
    _, pv = head(tracking, 10.48, orientation=yaw_q)
    assert pv[0] == pytest.approx((-.02, .04, -.03))
    assert scene.rotation_distance(pv[1], scene.quaternion_product(yaw_q, pv_q)) < 1e-7


def test_unstable_or_unmatched_head_pv_samples_cannot_calibrate_an_extrinsic():
    tracking = scene.SceneTracking("partner")
    tracking.observe_geometry("12", "map")
    for i in range(6):
        t = 10 + .12 * i
        head(tracking, t)
        tracking.observe_depth(pose(sec=1000 + t), t)
        camera(tracking, t, position=(.06 if i % 2 else .12, 0., 0.))
    assert not tracking.aligned
    camera(tracking, 10.64)  # 40 ms away from the nearest measured head.
    assert not tracking.aligned


def test_stale_duplicate_and_retired_head_samples_cannot_refresh_tf():
    tracking = aligned_tracker()
    assert head(tracking, 10.24) is None
    assert tracking.last_arrival == 10.24
    assert tracking.status(10.8) == "waiting_for_fresh_native_camera_pose"
    packet = tracking_packet(10.28)
    assert tracking.observe_head(packet, 11., 1011.) is None
    tracking.observe_geometry("13", "map")
    align(tracking, start=11.1, session="new-relay")
    assert tracking.aligned
    assert head(tracking, 11.4, session="relay") is None
    assert tracking.origin[0] == "new-relay"


@pytest.mark.parametrize("change", [
    {"tracking_origin_revision": None}, {"tracking_origin_revision": True},
    {"tracking_valid": "true"}, {"source_timestamp_sec": math.nan},
    {"position": [0., 0., math.inf]}, {"orientation_xyzw": [0., 0., 0., 0.]},
    {"child_frame_id": "partner/base"}, {"pose_source": "head_pose_fallback"},
])
def test_atomic_tracking_schema_rejects_invalid_or_cosmetic_poses(change):
    tracking = aligned_tracker()
    packet = dict(tracking_packet(10.28), **change)
    assert tracking.observe_head(packet, 10.28, 1010.28) is None
    assert not tracking.aligned


@pytest.mark.skipif(os.environ.get("HORUS_FIELD_ROS_TEST") != "1", reason="Requires an isolated ROS test run")
@pytest.mark.parametrize("native_fallback", [False, True])
def test_real_ros_presentation_tf_uses_native_camera_and_one_metric_datum(native_fallback):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    import json

    name = f"field_scene_test_{os.getpid()}"
    script = Path(__file__).parents[1] / "examples/tools/field_teammate_scene.py"
    environment = dict(os.environ, ROS_DOMAIN_ID="219", ROS_LOCALHOST_ONLY="1",
                       ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    process = subprocess.Popen([sys.executable, str(script), "--name", name], env=environment,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    context = Context()
    rclpy.init(context=context, domain_id=219)
    node = rclpy.create_node("scene_tf_test", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        buffer = Buffer(node=node)
        listener = TransformListener(buffer, node)
        camera = node.create_publisher(String, f"/{name}/tracking/pv_pose", 10)
        head_pub = node.create_publisher(String, f"/{name}/tracking/head_pose", 10)
        depth = node.create_publisher(PoseStamped, f"/{name}/research/long_throw/pose", 1)
        mesh = node.create_publisher(String, "/horus/field/mesh/indexed_manifest",
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        message = PoseStamped()
        message.header.frame_id = "map"
        pv_position = NATIVE_PV if native_fallback else (1., .5, .15)
        head_position = UNITY_HEAD if native_fallback else pv_position
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pv_position
        message.pose.orientation.w = 1.0
        record = manifest(low=-1.265, high=.685)
        floor_frame, source_frame, camera_frame = scene.scene_frames(name)
        record["frame_id"] = source_frame
        def publish_tracking(revision=1, with_depth=True):
            stamp = node.get_clock().now().to_msg()
            ros_time = stamp.sec + stamp.nanosec / 1e9
            source_time = time.monotonic()
            packet = tracking_packet(source_time, name=name, stamp=ros_time, revision=revision,
                                     position=head_position)
            message.header.stamp = stamp
            head_pub.publish(String(data=json.dumps(packet)))
            packet["child_frame_id"] = f"{name}/pv_camera"
            packet["pose_source"] = "hololens_locatable_pv_camera"
            packet["position"] = pv_position
            camera.publish(String(data=json.dumps(packet)))
            if with_depth:
                depth.publish(message)
            return round(ros_time * 1e9)

        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            assert process.poll() is None, process.stderr.read()
            mesh.publish(String(data=json.dumps(record)))
            publish_tracking()
            executor.spin_once(timeout_sec=.1)
            time.sleep(.05)
            if buffer.can_transform(floor_frame, camera_frame, rclpy.time.Time()):
                break
        result = buffer.lookup_transform(floor_frame, camera_frame, rclpy.time.Time())
        translation = result.transform.translation
        assert translation.x == pytest.approx(pv_position[0])
        assert translation.y == pytest.approx(pv_position[1])
        assert translation.z == pytest.approx(pv_position[2] + 1.265)
        anchor = buffer.lookup_transform(f"{name}_scene/world", floor_frame, rclpy.time.Time())
        assert anchor.transform.translation.z == pytest.approx(0)
        assert anchor.transform.rotation.w == pytest.approx(1)
        assert not buffer.can_transform("map", camera_frame, rclpy.time.Time())
        # Real ROS callbacks: fresh measured head poses continue throughout a
        # depth hiatus longer than the previous hard 1.5-second cutoff.
        from tf2_msgs.msg import TFMessage
        received = []
        received_heads = []
        statuses = []
        def receive_tf(message):
            for tf in message.transforms:
                stamp_ns = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
                if tf.child_frame_id == camera_frame:
                    received.append(stamp_ns)
                elif tf.child_frame_id == f"{name}_scene/head":
                    received_heads.append(stamp_ns)
        subscription = node.create_subscription(TFMessage, "/tf", receive_tf, 100)
        status_subscription = node.create_subscription(String, f"/{name}/scene/status",
            lambda msg: statuses.append(json.loads(msg.data)), 10)
        deadline = time.monotonic() + 2.1
        while time.monotonic() < deadline:
            publish_tracking(with_depth=False)
            for _ in range(3):
                executor.spin_once(timeout_sec=.01)
            time.sleep(.02)
        latest = buffer.lookup_transform(floor_frame, camera_frame, rclpy.time.Time())
        latest_ns = latest.header.stamp.sec * 1_000_000_000 + latest.header.stamp.nanosec
        assert node.get_clock().now().nanoseconds - latest_ns < 350_000_000
        assert len(set(received)) > 25
        if native_fallback:
            assert not received_heads
            assert any(status["camera_pose_source"] == "native_pv" and status["state"] == "live"
                       for status in statuses)
        # A real origin revision cannot reuse the retained pre-gap depth sample.
        cutoff = publish_tracking(revision=2, with_depth=False)
        deadline = time.monotonic() + .35
        while time.monotonic() < deadline:
            publish_tracking(revision=2, with_depth=False)
            executor.spin_once(timeout_sec=.02)
            time.sleep(.02)
        assert not any(stamp >= cutoff for stamp in received)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            publish_tracking(revision=2)
            executor.spin_once(timeout_sec=.03)
            time.sleep(.04)
            if (any(stamp >= cutoff for stamp in received) and
                    (native_fallback or any(stamp >= cutoff for stamp in received_heads))):
                break
        assert any(stamp >= cutoff for stamp in received)
        if native_fallback:
            assert not received_heads
        else:
            assert any(stamp >= cutoff for stamp in received_heads)
        node.destroy_subscription(subscription)
        node.destroy_subscription(status_subscription)
        listener.unregister()
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        process.stderr.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
