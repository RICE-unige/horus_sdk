"""Tests for workspace-scale registration payload behavior."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType
from horus.bridge import robot_registry as robot_registry_module


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    # Prevent __del__ from trying to tear down uninitialized ROS state.
    client.ros_initialized = False
    client.node = None
    return client


def _build_robot_and_dataviz():
    robot = Robot(name="scale_bot", robot_type=RobotType.WHEELED)
    dataviz = robot.create_dataviz()
    return robot, dataviz


def test_workspace_scale_serialized_when_valid():
    robot, dataviz = _build_robot_and_dataviz()
    client = _build_client()

    config = client._build_robot_config_dict(robot, dataviz, workspace_scale=0.25)

    assert "workspace_config" in config
    assert config["workspace_config"]["position_scale"] == 0.25


def test_workspace_scale_omitted_when_missing():
    robot, dataviz = _build_robot_and_dataviz()
    client = _build_client()

    config = client._build_robot_config_dict(robot, dataviz)

    assert "workspace_config" not in config


def test_workspace_scale_omitted_when_invalid():
    robot, dataviz = _build_robot_and_dataviz()
    client = _build_client()

    invalid_values = (0.0, -0.1, float("nan"), float("inf"), float("-inf"), "bad")
    for value in invalid_values:
        config = client._build_robot_config_dict(robot, dataviz, workspace_scale=value)
        assert "workspace_config" not in config


def test_robot_register_forwards_workspace_scale(monkeypatch):
    class FakeRegistry:
        def __init__(self):
            self.received_workspace_scale = None

        def register_robot(
            self,
            robot,
            dataviz,
            timeout_sec: float = 10.0,
            keep_alive: bool = False,
            show_dashboard: bool = True,
            workspace_scale=None,
        ):
            self.received_workspace_scale = workspace_scale
            return True, {"robot_id": "R1", "assigned_color": "#00FF00"}

    fake_registry = FakeRegistry()
    monkeypatch.setattr(
        robot_registry_module,
        "get_robot_registry_client",
        lambda: fake_registry,
    )

    robot, dataviz = _build_robot_and_dataviz()
    success, result = robot.register_with_horus(
        dataviz=dataviz,
        keep_alive=False,
        show_dashboard=False,
        workspace_scale=0.33,
    )

    assert success is True
    assert result["robot_id"] == "R1"
    assert fake_registry.received_workspace_scale == 0.33
