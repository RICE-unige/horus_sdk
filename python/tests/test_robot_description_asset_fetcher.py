"""Tests for robot-description demo helpers and profile mapping."""

import importlib.util
import os
import tempfile
from pathlib import Path

import pytest

from horus.description.robot_description_resolver import RobotDescriptionResolver
from horus.robot import Robot, RobotType


def _load_module(module_path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, str(module_path))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec: {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_resolver_mesh_proxy_without_scale_does_not_square_fallback_size():
    resolver = RobotDescriptionResolver()
    raw_payload = """
<robot name="mesh_scale_default_bot">
  <link name="base_link">
    <collision>
      <geometry>
        <mesh filename="package://test/meshes/link.stl"/>
      </geometry>
    </collision>
  </link>
</robot>
    """

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
        handle.write(raw_payload)
        urdf_path = handle.name

    try:
        robot = Robot(
            name="mesh_scale_default_bot",
            robot_type=RobotType.LEGGED,
            dimensions=(0.42, 0.28, 0.21),
        )
        robot.configure_robot_description(urdf_path=urdf_path, base_frame="base_link")
        artifact = resolver.resolve_for_robot(robot)
        assert artifact is not None
        collision = artifact.payload_dict["links"][0]["collisions"][0]
        assert collision["type"] == "mesh_proxy"
        assert collision["proxy_size_xyz"] == pytest.approx([0.147, 0.098, 0.0525], abs=1e-6)
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_robot_profile_mapping_matches_expected_names_and_base_frames():
    demo = _load_module(
        Path(__file__).resolve().parents[1] / "examples" / "legacy" / "sdk_robot_description_demo.py",
        "sdk_robot_description_demo_for_profile_tests",
    )

    classic_args = demo.build_parser().parse_args([])
    classic_entries = demo._build_robot_profile_entries(classic_args)
    assert [entry["name"] for entry in classic_entries] == ["jackal", "go1"]
    assert [entry["base_frame"] for entry in classic_entries] == ["base_link", "base"]

    real_args = demo.build_parser().parse_args(["--robot-profile", "real_models"])
    real_entries = demo._build_robot_profile_entries(real_args)
    assert [entry["name"] for entry in real_entries] == ["anymal_c", "jackal", "go1", "h1"]
    assert [entry["base_frame"] for entry in real_entries] == ["base", "base_link", "base", "pelvis"]
