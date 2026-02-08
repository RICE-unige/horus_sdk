"""Tests for camera MiniMap/Teleop transport profile serialization."""

import pytest

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import Robot, RobotType
from horus.sensors import Camera


def _build_config(robot):
    dataviz = robot.create_dataviz()
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    # Prevent __del__ from trying to tear down uninitialized ROS state.
    client.ros_initialized = False
    client.node = None
    return client._build_robot_config_dict(robot, dataviz)


def test_camera_transport_profile_defaults():
    camera = Camera(
        name="front_camera",
        frame_id="test_bot/camera_link",
        topic="/test_bot/camera/image_raw",
    )

    assert camera.streaming_type == "ros"
    assert camera.minimap_streaming_type == "ros"
    assert camera.teleop_streaming_type == "webrtc"
    assert camera.startup_mode == "minimap"


def test_camera_transport_profile_validation():
    with pytest.raises(ValueError):
        Camera(
            name="front_camera",
            frame_id="test_bot/camera_link",
            topic="/test_bot/camera/image_raw",
            minimap_streaming_type="invalid",
        )

    with pytest.raises(ValueError):
        Camera(
            name="front_camera",
            frame_id="test_bot/camera_link",
            topic="/test_bot/camera/image_raw",
            teleop_streaming_type="invalid",
        )

    with pytest.raises(ValueError):
        Camera(
            name="front_camera",
            frame_id="test_bot/camera_link",
            topic="/test_bot/camera/image_raw",
            startup_mode="invalid",
        )


def test_registration_payload_includes_profile_fields():
    robot = Robot(name="test_bot", robot_type=RobotType.WHEELED)
    camera = Camera(
        name="front_camera",
        frame_id="test_bot/camera_link",
        topic="/test_bot/camera/image_raw/compressed",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        startup_mode="minimap",
    )
    robot.add_sensor(camera)

    config = _build_config(robot)
    camera_config = config["sensors"][0]["camera_config"]

    assert camera_config["streaming_type"] == "ros"
    assert camera_config["minimap_streaming_type"] == "ros"
    assert camera_config["teleop_streaming_type"] == "webrtc"
    assert camera_config["startup_mode"] == "minimap"


def test_registration_payload_legacy_streaming_fallback():
    robot = Robot(name="legacy_bot", robot_type=RobotType.WHEELED)
    camera = Camera(
        name="front_camera",
        frame_id="legacy_bot/camera_link",
        topic="/legacy_bot/camera/image_raw/compressed",
        streaming_type="webrtc",
    )
    # Simulate legacy/unspecified profile values and ensure fallback to
    # legacy streaming_type is preserved.
    camera.minimap_streaming_type = ""
    camera.teleop_streaming_type = ""
    camera.startup_mode = ""
    robot.add_sensor(camera)

    config = _build_config(robot)
    camera_config = config["sensors"][0]["camera_config"]

    assert camera_config["streaming_type"] == "webrtc"
    assert camera_config["minimap_streaming_type"] == "webrtc"
    assert camera_config["teleop_streaming_type"] == "webrtc"
    assert camera_config["startup_mode"] == "minimap"
