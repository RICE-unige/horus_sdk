"""Core DataViz model types used by the HORUS SDK compatibility facade."""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, Optional

from ..sensors import SensorInstance

class DataSourceType(Enum):
    """Types of data sources for visualization"""

    # Sensor-based data
    SENSOR = "sensor"

    # Robot-specific data
    ROBOT_STATE = "robot_state"
    ROBOT_TRANSFORM = "robot_transform"
    ROBOT_GLOBAL_PATH = "robot_global_path"
    ROBOT_LOCAL_PATH = "robot_local_path"
    ROBOT_TRAJECTORY = "robot_trajectory"
    ROBOT_ODOMETRY = "robot_odometry"
    ROBOT_COLLISION_RISK = "robot_collision_risk"

    # Environmental/World data (robot-independent)
    OCCUPANCY_GRID = "occupancy_grid"
    COSTMAP = "costmap"
    MAP_3D = "map_3d"
    GLOBAL_NAVIGATION_PATH = "global_navigation_path"

    # Shared/Global data
    TF_TREE = "tf_tree"
    GLOBAL_MARKERS = "global_markers"
    COORDINATE_FRAME = "coordinate_frame"


class VisualizationType(Enum):
    """Types of data visualization rendering"""

    CAMERA_FEED = "camera_feed"
    LASER_SCAN = "laser_scan"
    POINT_CLOUD = "point_cloud"
    OCCUPANCY_GRID = "occupancy_grid"
    TRAJECTORY = "trajectory"
    PATH = "path"
    VELOCITY_DATA = "velocity_data"
    ODOMETRY_TRAIL = "odometry_trail"
    COLLISION_RISK = "collision_risk"
    MARKERS = "markers"
    TRANSFORM_TREE = "transform_tree"
    COORDINATE_AXES = "coordinate_axes"
    MESH = "mesh"
    OCTOMAP = "octomap"
    HEATMAP = "heatmap"
    SEMANTIC_BOX = "semantic_box"


@dataclass
class DataSource(ABC):
    """
    Abstract base class for data sources

    Args:
        name: Unique identifier for the data source
        source_type: Type of data source
        topic: ROS topic or data channel
        frame_id: Reference frame for the data
        robot_name: Associated robot (None for robot-independent data)
    """

    name: str
    source_type: DataSourceType
    topic: str
    frame_id: str = "map"
    robot_name: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if not self.name:
            raise ValueError("Data source name cannot be empty")
        if not self.topic:
            raise ValueError("Data source topic cannot be empty")

    def is_robot_specific(self) -> bool:
        """Check if this data source is tied to a specific robot"""
        return self.robot_name is not None

    def add_metadata(self, key: str, value: Any) -> None:
        """Add metadata to data source"""
        self.metadata[key] = value

    def get_metadata(self, key: str, default: Any = None) -> Any:
        """Get metadata value by key"""
        return self.metadata.get(key, default)


class SensorDataSource(DataSource):
    """Data source from a robot sensor"""

    def __init__(self, sensor: SensorInstance, **kwargs):
        super().__init__(
            name=sensor.name,
            source_type=DataSourceType.SENSOR,
            topic=sensor.topic,
            frame_id=sensor.frame_id,
            **kwargs,
        )
        self.sensor = sensor


class RobotDataSource(DataSource):
    """Data source from robot state/transforms"""

    def __init__(
        self,
        name: str,
        source_type: DataSourceType,
        topic: str,
        robot_name: str,
        frame_id: str = "map",
        **kwargs,
    ):
        super().__init__(name, source_type, topic, frame_id, robot_name, **kwargs)


class EnvironmentDataSource(DataSource):
    """Data source from environment/world (robot-independent)"""

    def __init__(
        self,
        name: str,
        source_type: DataSourceType,
        topic: str,
        frame_id: str = "map",
        **kwargs,
    ):
        super().__init__(name, source_type, topic, frame_id, None, **kwargs)


@dataclass
class VisualizationConfig:
    """
    Configuration for a specific visualization

    Args:
        viz_type: Type of visualization rendering
        data_source: Associated data source
        display_name: Human-readable name for display
        enabled: Whether visualization is active
        render_options: Rendering-specific options
        layer_priority: Display layer priority (higher = front)
    """

    viz_type: VisualizationType
    data_source: DataSource
    display_name: str = ""
    enabled: bool = True
    render_options: Dict[str, Any] = field(default_factory=dict)
    layer_priority: int = 0

    def __post_init__(self):
        """Set default display name if not provided"""
        if not self.display_name:
            source_desc = (
                f"{self.data_source.robot_name}:" if self.data_source.robot_name else ""
            )
            self.display_name = (
                f"{source_desc}{self.data_source.name} ({self.viz_type.value})"
            )

    def is_robot_specific(self) -> bool:
        """Check if this visualization is tied to a specific robot"""
        return self.data_source.is_robot_specific()
