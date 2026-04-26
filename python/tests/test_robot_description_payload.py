"""Tests for robot-description manifest payload wiring."""

import os
import tempfile
import math

import pytest

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client


def test_robot_description_manifest_is_attached_when_urdf_config_present():
    urdf = """
<robot name="tiny_bot">
  <link name="base_link">
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><box size="0.4 0.3 0.2"/></geometry>
    </collision>
  </link>
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_left"/>
    <origin xyz="0.1 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="wheel_left">
    <collision>
      <geometry><cylinder radius="0.1" length="0.04"/></geometry>
    </collision>
  </link>
</robot>
    """.strip()

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
        handle.write(urdf)
        urdf_path = handle.name

    try:
        robot = Robot(name="atlas", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(urdf_path=urdf_path, base_frame="base_link")
        dataviz = robot.create_dataviz()

        client = _build_client()
        config = client._build_robot_config_dict(robot, dataviz)

        assert "robot_description_manifest" in config
        manifest = config["robot_description_manifest"]
        assert manifest["version"] == "v2"
        assert manifest["source"] == "ros"
        assert manifest["base_frame"] == "base_link"
        assert manifest["supports_collision"] is True
        assert manifest["supports_joints"] is True
        assert manifest["supports_visual_meshes"] is False
        assert manifest["mesh_asset_count"] == 0
        assert manifest["is_transparent"] is False
        assert manifest["collision_count"] == 2
        assert manifest["joint_count"] == 1
        assert manifest["link_count"] == 2
        assert str(manifest["description_id"]).startswith("sha256:")
        assert manifest["encoding"] == "json+gzip+base64"
        assert int(manifest["chunk_size_bytes"]) >= 1024
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_robot_description_manifest_not_present_when_disabled():
    robot = Robot(name="nova", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    client = _build_client()
    config = client._build_robot_config_dict(robot, dataviz)
    assert "robot_description_manifest" not in config


def test_robot_description_manifest_transparency_flag_can_be_enabled():
    urdf = """
<robot name="tiny_bot">
  <link name="base_link">
    <collision>
      <geometry><box size="0.2 0.2 0.1"/></geometry>
    </collision>
  </link>
</robot>
    """.strip()

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
        handle.write(urdf)
        urdf_path = handle.name

    try:
        robot = Robot(name="atlas_transparent", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(
            urdf_path=urdf_path,
            base_frame="base_link",
            is_transparent=True,
        )
        dataviz = robot.create_dataviz()

        client = _build_client()
        config = client._build_robot_config_dict(robot, dataviz)

        assert "robot_description_manifest" in config
        manifest = config["robot_description_manifest"]
        assert manifest["is_transparent"] is True
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)


def test_robot_description_payload_is_flattened_into_base_frame():
    urdf = """
<robot name="pose_bot">
  <link name="base_link">
    <collision>
      <geometry><box size="0.3 0.2 0.1"/></geometry>
    </collision>
  </link>
  <joint name="arm_joint" type="fixed">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="1.0 0.0 0.0" rpy="0 0 1.5707963"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="arm_link">
    <collision>
      <origin xyz="0.2 0 0" rpy="0 0 0"/>
      <geometry><box size="0.2 0.1 0.1"/></geometry>
    </collision>
  </link>
</robot>
    """.strip()

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False, encoding="utf-8") as handle:
        handle.write(urdf)
        urdf_path = handle.name

    try:
        robot = Robot(name="atlas", robot_type=RobotType.WHEELED)
        robot.configure_robot_description(urdf_path=urdf_path, base_frame="base_link")
        client = _build_client()
        artifact_container = client._resolve_robot_description_artifact(robot)
        assert artifact_container is not None
        artifact = artifact_container["artifact"]
        payload = artifact.payload_dict

        links = {link["name"]: link for link in payload["links"]}
        arm_collisions = links["arm_link"]["collisions"]
        assert len(arm_collisions) == 1
        arm_collision = arm_collisions[0]

        # arm_link is translated by +1.0m X and rotated +90deg yaw, so local +0.2m X becomes +0.2m Y in base.
        assert arm_collision["origin_xyz"][0] == pytest.approx(1.0, abs=1e-4)
        assert arm_collision["origin_xyz"][1] == pytest.approx(0.2, abs=1e-4)
        assert arm_collision["origin_xyz"][2] == pytest.approx(0.0, abs=1e-4)

        joints = {joint["name"]: joint for joint in payload["joints"]}
        arm_joint = joints["arm_joint"]
        assert arm_joint["origin_xyz"][0] == pytest.approx(1.0, abs=1e-4)
        assert arm_joint["origin_xyz"][1] == pytest.approx(0.0, abs=1e-4)
        assert arm_joint["origin_xyz"][2] == pytest.approx(0.0, abs=1e-4)
        assert arm_joint["origin_rpy"][2] == pytest.approx(math.pi / 2.0, abs=1e-4)
    finally:
        if os.path.isfile(urdf_path):
            os.remove(urdf_path)
