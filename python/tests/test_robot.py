"""Basic tests for HORUS SDK Robot module"""

import pytest


def test_robot_imports():
    """Test that robot module imports work correctly"""
    try:
        from horus.color import ColorManager
        from horus.dataviz import DataViz
        from horus.robot import Robot, RobotType
        from horus.sensors import Camera, LaserScan, Lidar3D

        assert True
    except ImportError as e:
        pytest.fail(f"Failed to import robot modules: {e}")


def test_robot_type_enum():
    """Test RobotType enum values"""
    from horus.robot import RobotType

    # Test that basic robot types exist
    assert hasattr(RobotType, "WHEELED")
    assert hasattr(RobotType, "LEGGED")
    assert hasattr(RobotType, "AERIAL")


def test_sensor_type_enum():
    """Test SensorType enum values"""
    from horus.sensors import SensorType

    # Test that basic sensor types exist
    assert hasattr(SensorType, "CAMERA")
    assert hasattr(SensorType, "LIDAR_3D")
    assert hasattr(SensorType, "LASER_SCAN")
    assert hasattr(SensorType, "IMU")
    assert hasattr(SensorType, "GPS")
    assert hasattr(SensorType, "ODOMETRY")

    # Test enum values
    assert SensorType.CAMERA.value == "camera"
    assert SensorType.LASER_SCAN.value == "laser_scan"
    assert SensorType.LIDAR_3D.value == "lidar_3d"


def test_basic_robot_creation():
    """Test basic robot creation without backend dependencies"""
    from horus.robot import Robot, RobotType

    # Create a simple robot without requiring ROS2 backend
    robot = Robot(name="test_robot", robot_type=RobotType.WHEELED)

    assert robot.name == "test_robot"
    assert robot.robot_type == RobotType.WHEELED
    assert robot.metadata == {}  # Default empty metadata after __post_init__
    assert robot.sensors == []  # No sensors by default

    # Test robot methods
    assert robot.get_type_str() == "wheeled"
    assert robot.get_sensor_count() == 0
    assert not robot.has_sensors()
    assert not robot.is_registered_with_horus()


def test_robot_metadata():
    """Test robot metadata functionality"""
    from horus.robot import Robot, RobotType

    robot = Robot(name="metadata_test", robot_type=RobotType.LEGGED)

    # Test metadata operations
    robot.add_metadata("test_key", "test_value")
    assert robot.get_metadata("test_key") == "test_value"
    assert robot.get_metadata("nonexistent", "default") == "default"

    # Test metadata is stored in the dict
    assert "test_key" in robot.metadata
    assert robot.metadata["test_key"] == "test_value"


def test_version_import():
    """Test that version can be imported"""
    from horus.utils.branding import __version__

    assert __version__ is not None
    assert isinstance(__version__, str)
