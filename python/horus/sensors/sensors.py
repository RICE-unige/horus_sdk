"""
Sensor system for robot data visualization in HORUS SDK
"""

from enum import Enum
from typing import Optional, Dict, Any, List, Union
from dataclasses import dataclass, field
from abc import ABC, abstractmethod


class SensorType(Enum):
    """Sensor type classifications"""

    CAMERA = "camera"
    LASER_SCAN = "laser_scan"
    LIDAR_3D = "lidar_3d"
    IMU = "imu"
    GPS = "gps"
    ODOMETRY = "odometry"


@dataclass
class BaseSensor(ABC):
    """
    Base sensor class for all sensor types

    Args:
        name: Sensor identifier
        sensor_type: Type of sensor
        frame_id: ROS frame ID for the sensor
        topic: ROS topic name for sensor data
        enabled: Whether sensor is active
        metadata: Additional sensor information
    """

    name: str
    sensor_type: SensorType
    frame_id: str
    topic: str
    enabled: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        """Validate sensor configuration"""
        if not self.name:
            raise ValueError("Sensor name cannot be empty")
        if not self.topic:
            raise ValueError("Sensor topic cannot be empty")
        if not self.frame_id:
            raise ValueError("Sensor frame_id cannot be empty")

    def __str__(self) -> str:
        return f"{self.__class__.__name__}(name='{self.name}', topic='{self.topic}')"

    def enable(self) -> None:
        """Enable sensor"""
        self.enabled = True

    def disable(self) -> None:
        """Disable sensor"""
        self.enabled = False

    def add_metadata(self, key: str, value: Any) -> None:
        """Add metadata to sensor"""
        self.metadata[key] = value

    def get_metadata(self, key: str, default: Any = None) -> Any:
        """Get metadata value by key"""
        return self.metadata.get(key, default)


@dataclass
class Camera(BaseSensor):
    """
    Camera sensor with vision-specific properties

    Args:
        is_stereo: Whether camera is stereo (dual camera)
        resolution: Camera resolution as (width, height)
        fps: Frames per second
        fov: Field of view in degrees
        encoding: Image encoding format
    """

    is_stereo: bool = False
    resolution: tuple[int, int] = (640, 480)
    fps: int = 30
    fov: float = 60.0
    encoding: str = "bgr8"

    def __init__(
        self,
        name: str,
        frame_id: str,
        topic: str,
        is_stereo: bool = False,
        resolution: tuple[int, int] = (640, 480),
        fps: int = 30,
        fov: float = 60.0,
        encoding: str = "bgr8",
        **kwargs,
    ):
        super().__init__(name, SensorType.CAMERA, frame_id, topic, **kwargs)
        self.is_stereo = is_stereo
        self.resolution = resolution
        self.fps = fps
        self.fov = fov
        self.encoding = encoding

    def get_camera_type(self) -> str:
        """Get camera type as string"""
        return "stereo" if self.is_stereo else "mono"

    def get_resolution_str(self) -> str:
        """Get resolution as string"""
        return f"{self.resolution[0]}x{self.resolution[1]}"


@dataclass
class LaserScan(BaseSensor):
    """
    2D Laser scanner sensor

    Args:
        min_angle: Minimum scan angle in radians
        max_angle: Maximum scan angle in radians
        angle_increment: Angular resolution in radians
        min_range: Minimum range in meters
        max_range: Maximum range in meters
        range_resolution: Range resolution in meters
    """

    min_angle: float = -3.14159  # -π
    max_angle: float = 3.14159  # π
    angle_increment: float = 0.005
    min_range: float = 0.1
    max_range: float = 30.0
    range_resolution: float = 0.01

    def __init__(
        self,
        name: str,
        frame_id: str,
        topic: str,
        min_angle: float = -3.14159,
        max_angle: float = 3.14159,
        angle_increment: float = 0.005,
        min_range: float = 0.1,
        max_range: float = 30.0,
        range_resolution: float = 0.01,
        **kwargs,
    ):
        super().__init__(name, SensorType.LASER_SCAN, frame_id, topic, **kwargs)
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.angle_increment = angle_increment
        self.min_range = min_range
        self.max_range = max_range
        self.range_resolution = range_resolution

    def get_scan_range_degrees(self) -> float:
        """Get scan range in degrees"""
        return (self.max_angle - self.min_angle) * 180.0 / 3.14159

    def get_num_points(self) -> int:
        """Get approximate number of scan points"""
        return int((self.max_angle - self.min_angle) / self.angle_increment)


@dataclass
class Lidar3D(BaseSensor):
    """
    3D LiDAR sensor

    Args:
        vertical_fov: Vertical field of view in degrees
        horizontal_fov: Horizontal field of view in degrees
        vertical_resolution: Vertical angular resolution in degrees
        horizontal_resolution: Horizontal angular resolution in degrees
        min_range: Minimum range in meters
        max_range: Maximum range in meters
        points_per_second: Points per second capability
        num_layers: Number of vertical layers/channels
    """

    vertical_fov: float = 40.0
    horizontal_fov: float = 360.0
    vertical_resolution: float = 0.4
    horizontal_resolution: float = 0.4
    min_range: float = 0.5
    max_range: float = 100.0
    points_per_second: int = 1000000
    num_layers: int = 64

    def __init__(
        self,
        name: str,
        frame_id: str,
        topic: str,
        vertical_fov: float = 40.0,
        horizontal_fov: float = 360.0,
        vertical_resolution: float = 0.4,
        horizontal_resolution: float = 0.4,
        min_range: float = 0.5,
        max_range: float = 100.0,
        points_per_second: int = 1000000,
        num_layers: int = 64,
        **kwargs,
    ):
        super().__init__(name, SensorType.LIDAR_3D, frame_id, topic, **kwargs)
        self.vertical_fov = vertical_fov
        self.horizontal_fov = horizontal_fov
        self.vertical_resolution = vertical_resolution
        self.horizontal_resolution = horizontal_resolution
        self.min_range = min_range
        self.max_range = max_range
        self.points_per_second = points_per_second
        self.num_layers = num_layers

    def get_point_cloud_size(self) -> int:
        """Get approximate point cloud size"""
        h_points = int(self.horizontal_fov / self.horizontal_resolution)
        v_points = int(self.vertical_fov / self.vertical_resolution)
        return h_points * v_points

    def get_lidar_type(self) -> str:
        """Get LiDAR type description"""
        return f"{self.num_layers}-layer 3D LiDAR"


# Type alias for sensor types
SensorInstance = Union[Camera, LaserScan, Lidar3D]
