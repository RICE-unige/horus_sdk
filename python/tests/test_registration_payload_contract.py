"""Payload-shape guardrails for HORUS MR registration compatibility."""

from __future__ import annotations

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotDimensions, RobotType
from horus.sensors import Camera, LaserScan


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client


def test_registration_payload_keeps_core_horus_mr_sections():
    robot = Robot(
        name="contract_bot",
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.8, width=0.5, height=0.4),
    )
    robot.configure_ros_binding(tf_mode="prefixed", topic_mode="prefixed")
    robot.configure_workspace_tutorial("robot_description_onboarding_v1")
    robot.add_sensor(Camera(name="front_camera", frame_id="contract_bot/front_camera", topic="/contract_bot/front/image_raw"))
    robot.add_sensor(LaserScan(name="scan", frame_id="contract_bot/scan", topic="/contract_bot/scan"))

    dataviz = robot.create_dataviz()
    config = _build_client()._build_robot_config_dict(
        robot,
        dataviz,
        workspace_scale=0.1,
        compass_enabled=True,
    )

    assert config["action"] == "register"
    assert config["robot_name"] == "contract_bot"
    assert config["robot_type"] == "wheeled"
    assert config["ros_binding"]["logical_name"] == "contract_bot"
    assert config["dimensions"] == {"length": 0.8, "width": 0.5, "height": 0.4}
    assert len(config["sensors"]) == 2
    assert isinstance(config["visualizations"], list)
    assert isinstance(config["global_visualizations"], list)
    assert config["control"]["drive_topic"] == "/contract_bot/cmd_vel"
    assert config["control"]["teleop"]["robot_profile"] == "wheeled"
    assert "go_to_point" in config["control"]["tasks"]
    assert "waypoint" in config["control"]["tasks"]
    assert config["robot_manager_config"]["enabled"] is True
    assert config["workspace_config"]["position_scale"] == 0.1
    assert config["workspace_config"]["compass"]["enabled"] is True
    assert config["workspace_config"]["tutorial"] == {
        "enabled": True,
        "preset_id": "robot_description_onboarding_v1",
    }


def test_drone_and_aerial_robot_type_semantics_are_preserved():
    drone = Robot(name="drone_alpha", robot_type=RobotType.DRONE)
    aerial = Robot(name="aerial_alpha", robot_type=RobotType.AERIAL)

    drone_config = _build_client()._build_robot_config_dict(drone, drone.create_dataviz())
    aerial_config = _build_client()._build_robot_config_dict(aerial, aerial.create_dataviz())

    assert drone_config["robot_type"] == "drone"
    assert drone_config["control"]["teleop"]["robot_profile"] == "drone"
    assert aerial_config["robot_type"] == "aerial"
    assert aerial_config["control"]["teleop"]["robot_profile"] == "aerial"
