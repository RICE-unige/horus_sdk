"""Real-world URDF shape handling regression tests for Robot Description V1."""

import os
import tempfile

import pytest

from horus.description.robot_description_resolver import RobotDescriptionResolver
from horus.robot import Robot, RobotType


def _write_temp_file(content: str, suffix: str) -> str:
    with tempfile.NamedTemporaryFile("w", suffix=suffix, delete=False, encoding="utf-8") as handle:
        handle.write(content.strip())
        return handle.name


def _build_robot(name: str, urdf_path: str, base_frame: str = "base_link") -> Robot:
    robot = Robot(name=name, robot_type=RobotType.WHEELED)
    robot.configure_robot_description(urdf_path=urdf_path, base_frame=base_frame)
    return robot


def test_base_frame_falls_back_to_root_link_when_configured_base_missing():
    urdf = """
<robot name="fallback_bot">
  <link name="chassis"/>
  <joint name="root_to_sensor" type="fixed">
    <parent link="chassis"/>
    <child link="sensor_mount"/>
    <origin xyz="0.4 0.0 0.2" rpy="0 0 0"/>
  </joint>
  <link name="sensor_mount">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
</robot>
    """
    urdf_path = _write_temp_file(urdf, ".urdf")
    try:
        resolver = RobotDescriptionResolver()
        robot = _build_robot("fallback_bot", urdf_path, base_frame="base_link")
        artifact = resolver.resolve_for_robot(robot)
        assert artifact is not None
        assert artifact.payload_dict["base_frame"] == "chassis"
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_mesh_collisions_become_deterministic_mesh_proxy_with_package_uri():
    urdf = """
<robot name="mesh_bot">
  <link name="base_link">
    <collision>
      <origin xyz="0.1 0.0 0.2" rpy="0 0 0"/>
      <geometry>
        <mesh filename="package://mesh_bot_description/meshes/body.dae" scale="1.5 0.5 2.0"/>
      </geometry>
    </collision>
  </link>
</robot>
    """
    urdf_path = _write_temp_file(urdf, ".urdf")
    try:
        resolver = RobotDescriptionResolver()
        robot = _build_robot("mesh_bot", urdf_path)
        artifact = resolver.resolve_for_robot(robot)
        assert artifact is not None
        links = artifact.payload_dict["links"]
        assert len(links) == 1
        collisions = links[0]["collisions"]
        assert len(collisions) == 1
        collision = collisions[0]
        assert collision["type"] == "mesh_proxy"
        assert collision["mesh_uri"].startswith("package://mesh_bot_description/")
        assert len(collision["proxy_size_xyz"]) == 3
        assert collision["proxy_size_xyz"][0] > 0.03
        assert collision["proxy_size_xyz"][1] > 0.03
        assert collision["proxy_size_xyz"][2] > 0.03
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_joint_axis_normalization_and_degenerate_fallback():
    urdf = """
<robot name="axis_bot">
  <link name="base_link"/>
  <joint name="degenerate_axis_joint" type="revolute">
    <parent link="base_link"/>
    <child link="link_a"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 0"/>
  </joint>
  <link name="link_a"/>
  <joint name="non_normalized_axis_joint" type="revolute">
    <parent link="link_a"/>
    <child link="link_b"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 3 4"/>
  </joint>
  <link name="link_b"/>
</robot>
    """
    urdf_path = _write_temp_file(urdf, ".urdf")
    try:
        resolver = RobotDescriptionResolver()
        robot = _build_robot("axis_bot", urdf_path)
        artifact = resolver.resolve_for_robot(robot)
        assert artifact is not None
        joints = {joint["name"]: joint for joint in artifact.payload_dict["joints"]}
        assert joints["degenerate_axis_joint"]["axis_xyz"] == pytest.approx([0.0, 0.0, 1.0], abs=1e-6)
        assert joints["non_normalized_axis_joint"]["axis_xyz"] == pytest.approx([0.0, 0.6, 0.8], abs=1e-6)
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_xacro_failure_surfaces_actionable_error(monkeypatch):
    xacro_payload = """
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_bot">
  <xacro:property name="foo" value="1"/>
</robot>
    """
    xacro_path = _write_temp_file(xacro_payload, ".xacro")
    try:
        resolver = RobotDescriptionResolver()

        def _fake_expand(_path: str):
            return "", "xacro tool not found. Install xacro or provide a resolved .urdf file."

        monkeypatch.setattr(resolver, "_expand_xacro", _fake_expand)
        robot = _build_robot("xacro_bot", xacro_path)
        artifact = resolver.resolve_for_robot(robot)
        assert artifact is None
        assert "xacro tool not found" in resolver.last_error
    finally:
        if os.path.isfile(xacro_path):
            os.remove(xacro_path)

