"""Synthetic demo contract and kinematics, without DDS or hardware."""
import sys
from pathlib import Path
from copy import deepcopy
import time

import numpy as np
import pytest

pytest.importorskip("scipy")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
from tools.teach_synthetic import (NAME, HANDS, JOINT_NAMES, SyntheticHumanoid,
                                  description, forward_arm, inverse_arm, camera_image)
from humanoid_teach_demo import make_registration


def bundle(model):
    return dict(authority=dict(robot_name=NAME, owner="operator", session_id="session", authority_generation="one"),
                targets=deepcopy(model.feedback()["poses"]), tools={"left_grasp": 1., "right_grasp": 0.})


@pytest.mark.parametrize("input_version,mode", [(1, "hold_to_run"), (2, "hold_to_run"), (2, "continuous")])
def test_demo_bakes_visible_colored_articulated_geometry(tmp_path, input_version, mode):
    from horus.description.robot_description_resolver import RobotDescriptionResolver
    path = tmp_path / "demo.urdf"
    path.write_text(description())
    robot, view = make_registration(path, tmp_path / "recordings", input_version, mode)
    artifact = RobotDescriptionResolver().resolve_for_robot(robot)
    assert artifact is not None
    assert artifact.manifest.supports_visual_meshes
    assert artifact.manifest.mesh_asset_count >= 5
    assert artifact.manifest.joint_count >= len(JOINT_NAMES)
    from test_registration_payload_contract import _build_client
    payload = _build_client()._build_robot_config_dict(robot, view)
    assert payload["teaching"].get("input_schema_version", 1) == input_version
    assert payload["teaching"].get("engagement_mode", "hold_to_run") == mode
    if input_version == 2:
        assert "head_pose" in payload["teaching"]["required_input_features"]
        assert payload["teaching"]["clutch_binding"] == "both_primary"
    assert len(payload["teaching"]["cameras"]) == 3


@pytest.mark.parametrize("hand", HANDS)
def test_measured_pose_matches_inverse_targets(hand):
    for x, z in ((.32, 1.06), (.4, 1.2), (.2, 1.4)):
        y = .24 if hand == "left" else -.24
        target = [x, y, z]
        joints = inverse_arm(target, [0, 0, 0, 1], hand)
        measured = forward_arm(joints, hand)
        assert np.allclose(measured["position"], target)
        assert abs(measured["rotation"][3]) == pytest.approx(1)


def test_commands_require_current_authority_and_hold_is_scoped():
    model = SyntheticHumanoid()
    command = bundle(model)
    with pytest.raises(RuntimeError): model.publish(command)
    model.update_lease("operator", "one", 3)
    command["targets"]["left"]["position"][0] += .03
    before = model.snapshot()
    receipt = model.publish(command)
    assert receipt["boundary"] == "scheduled"
    model.step(.03)
    assert model.snapshot() != before
    assert not model.hold(dict(command["authority"], session_id="old"))
    assert model.mode == "tracking"
    assert model.hold(command["authority"])
    held = model.snapshot()
    model.step(.03)
    assert model.snapshot() == held
    model.update_lease("another", "two", 3)
    with pytest.raises(RuntimeError): model.publish(command)


def test_independent_watchdog_stops_synthetic_servo():
    model = SyntheticHumanoid()
    model.update_lease("operator", "one", 3)
    model.publish(bundle(model))
    model.last_command = time.monotonic() - .3
    before = model.snapshot()
    model.step(.03)
    assert model.snapshot() == before and model.mode == "holding"


def test_cameras_are_distinct_rgb_and_animate():
    model = SyntheticHumanoid()
    front = camera_image(model, "front", 40)
    assert front.shape == (240, 320, 3) and front.dtype == np.uint8
    assert not np.array_equal(front, camera_image(model, "left", 40))
    assert not np.array_equal(front, camera_image(model, "front", 80))


@pytest.mark.parametrize("mode", ["mono", "stereo", "estimated"])
def test_portal_registration_preserves_original_recording(tmp_path, mode):
    robot, view = make_registration(tmp_path / "test.urdf", tmp_path, front_portal=mode)
    from test_registration_payload_contract import _build_client
    payload = _build_client()._build_robot_config_dict(robot, view)
    assert payload["teaching"]["cameras"][0]["portal"]["mode"] == mode
    camera = payload["sensors"][0]["camera_config"]
    assert camera["minimap_topic"] == "/teach_synthetic/front/image_raw"
    if mode == "stereo":
        assert camera["teleop_stereo_layout"] == "dual_topic"
        assert camera["teleop_right_topic"] == "/teach_synthetic/front/right/image_raw"
    elif mode == "estimated":
        assert camera["teleop_stereo_layout"] == "side_by_side"
        assert camera["teleop_topic"] == "/teach_synthetic/front/portal_sbs/compressed"
        assert camera["teleop_image_type"] == "compressed"
    from horus.teaching import TeachConfig
    config = TeachConfig.from_dict(robot.get_metadata("teaching_config"))
    assert next(s for s in config.streams if s.stream_id == "front").topic == "/teach_synthetic/front/image_raw"
    assert all("portal_sbs" not in s.topic for s in config.streams)


def test_stereo_fixture_has_horizontal_depth_disparity():
    from tools.teach_synthetic import stereo_portal_fixture
    left, right = stereo_portal_fixture(0)
    for color in ((65, 150, 235), (225, 180, 60), (55, 210, 170)):
        ly, lx = np.where(np.all(left == color, axis=2))
        ry, rx = np.where(np.all(right == color, axis=2))
        assert lx.mean() > rx.mean()
        # Occlusion changes visible pixel centroids, but not vertical projection.
        assert (ly.min(), ly.max()) == (ry.min(), ry.max())
