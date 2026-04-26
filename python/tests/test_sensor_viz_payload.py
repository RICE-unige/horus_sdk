"""Tests for sensor viz_config serialization precedence."""

from horus.dataviz import DataViz
from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType
from horus.sensors import LaserScan


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    return client


def _build_scan(topic: str, color: str = "red", point_size: float = 0.05) -> LaserScan:
    return LaserScan(
        name="front_2d_lidar",
        frame_id="base_link",
        topic=topic,
        color=color,
        point_size=point_size,
    )


def test_sensor_viz_payload_uses_dataviz_sensor_color_and_point_size():
    client = _build_client()

    robot_a = Robot(name="carter1", robot_type=RobotType.WHEELED)
    scan_a = _build_scan("/carter1/front_2d_lidar/scan")
    robot_a.add_sensor(scan_a)
    dataviz_a = robot_a.create_dataviz()
    dataviz_a.add_sensor_visualization(
        scan_a,
        robot_name="carter1",
        render_options={"point_size": 0.012},
    )

    robot_b = Robot(name="carter2", robot_type=RobotType.WHEELED)
    scan_b = _build_scan("/carter2/front_2d_lidar/scan")
    robot_b.add_sensor(scan_b)
    dataviz_b = robot_b.create_dataviz()
    dataviz_b.add_sensor_visualization(
        scan_b,
        robot_name="carter2",
        render_options={"point_size": 0.018},
    )

    config_a = client._build_robot_config_dict(robot_a, dataviz_a)
    config_b = client._build_robot_config_dict(robot_b, dataviz_b)

    viz_a = config_a["sensors"][0]["viz_config"]
    viz_b = config_b["sensors"][0]["viz_config"]

    assert viz_a["color"] != viz_b["color"]
    assert viz_a["point_size"] == 0.012
    assert viz_b["point_size"] == 0.018


def test_sensor_viz_payload_preserves_sensor_defaults_without_dataviz_override():
    client = _build_client()

    robot = Robot(name="legacy_bot", robot_type=RobotType.WHEELED)
    scan = _build_scan("/legacy/front_2d_lidar/scan", color="red", point_size=0.025)
    robot.add_sensor(scan)
    dataviz = DataViz(name="legacy_bot_empty_viz")

    config = client._build_robot_config_dict(robot, dataviz)
    viz = config["sensors"][0]["viz_config"]

    assert viz["color"] == "red"
    assert viz["point_size"] == 0.025


def test_sensor_viz_payload_allows_dataviz_override_to_replace_sensor_defaults():
    client = _build_client()

    robot = Robot(name="override_bot", robot_type=RobotType.WHEELED)
    scan = _build_scan("/override/front_2d_lidar/scan", color="red", point_size=0.025)
    robot.add_sensor(scan)
    dataviz = robot.create_dataviz()
    dataviz.add_sensor_visualization(
        scan,
        robot_name="override_bot",
        render_options={"color": "#11AAEE", "point_size": 0.031},
    )

    config = client._build_robot_config_dict(robot, dataviz)
    viz = config["sensors"][0]["viz_config"]

    assert viz["color"] == "#11AAEE"
    assert viz["point_size"] == 0.031
