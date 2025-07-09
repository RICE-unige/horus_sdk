"""
Data visualization system for robot sensors and environmental data in HORUS SDK
"""

from typing import List, Dict, Any, Optional, Union
from dataclasses import dataclass, field
from enum import Enum
from abc import ABC, abstractmethod

from ..sensors import SensorInstance, SensorType


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
    MARKERS = "markers"
    TRANSFORM_TREE = "transform_tree"
    COORDINATE_AXES = "coordinate_axes"
    MESH = "mesh"
    HEATMAP = "heatmap"


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


@dataclass
class DataViz:
    """
    Data visualization manager for robot sensors and environmental data

    This class can manage visualizations for:
    - Robot-specific data (sensors, transforms, paths)
    - Robot-independent data (maps, global markers, TF tree)
    - Mixed scenarios with multiple robots and shared data

    Args:
        name: Name/identifier for this visualization collection
        visualizations: List of visualization configurations
        color_manager: Color management system for unique robot colors
    """

    name: str
    visualizations: List[VisualizationConfig] = field(default_factory=list)
    color_manager: Optional["ColorManager"] = None

    def __post_init__(self):
        """Validate DataViz configuration"""
        if not self.name:
            raise ValueError("DataViz name cannot be empty")

        # Initialize color manager if not provided
        if self.color_manager is None:
            from ..color import ColorManager

            self.color_manager = ColorManager()

    # Sensor-based visualizations
    def add_sensor_visualization(
        self,
        sensor: SensorInstance,
        robot_name: str,
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add a sensor-based visualization with automatic color assignment"""
        if render_options is None:
            render_options = {}

        # Create sensor data source
        data_source = SensorDataSource(sensor, robot_name=robot_name)

        # Determine visualization type
        viz_type = self._get_viz_type_for_sensor(sensor)

        # Auto-assign color based on sensor type and robot
        if "color" not in render_options:
            if viz_type == VisualizationType.LASER_SCAN:
                color = self.color_manager.get_laser_scan_color(robot_name)
                render_options["color"] = color.to_hex()
                render_options["alpha"] = color.a
            else:
                # For other sensors, use base robot color
                color = self.color_manager.get_robot_color(robot_name)
                render_options["color"] = color.to_hex()

        # Create and add visualization
        viz_config = VisualizationConfig(
            viz_type=viz_type, data_source=data_source, render_options=render_options
        )

        self._add_or_update_visualization(viz_config)

    # Robot-specific visualizations
    def add_robot_transform(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add robot transform visualization with automatic color assignment"""
        if render_options is None:
            render_options = {}

        # Auto-assign color if not specified
        if "color" not in render_options:
            color = self.color_manager.get_transform_color(robot_name)
            render_options["color"] = color.to_hex()

        data_source = RobotDataSource(
            name=f"{robot_name}_transform",
            source_type=DataSourceType.ROBOT_TRANSFORM,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.COORDINATE_AXES,
            data_source=data_source,
            render_options=render_options,
        )

        self._add_or_update_visualization(viz_config)

    def add_robot_global_path(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add robot global path planning visualization with automatic color assignment"""
        if render_options is None:
            render_options = {}

        # Auto-assign color for global path
        if "color" not in render_options:
            color = self.color_manager.get_path_color(robot_name, "global")
            render_options["color"] = color.to_hex()
            render_options["alpha"] = color.a
            render_options["line_width"] = render_options.get("line_width", 3)

        data_source = RobotDataSource(
            name=f"{robot_name}_global_path",
            source_type=DataSourceType.ROBOT_GLOBAL_PATH,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.PATH,
            data_source=data_source,
            render_options=render_options,
            layer_priority=3,  # Global paths on top of maps
        )

        self._add_or_update_visualization(viz_config)

    def add_robot_local_path(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add robot local path planning visualization with automatic color assignment"""
        if render_options is None:
            render_options = {}

        # Auto-assign color for local path (lighter than global)
        if "color" not in render_options:
            color = self.color_manager.get_path_color(robot_name, "local")
            render_options["color"] = color.to_hex()
            render_options["alpha"] = color.a
            render_options["line_width"] = render_options.get("line_width", 2)
            render_options["line_style"] = render_options.get("line_style", "dashed")

        data_source = RobotDataSource(
            name=f"{robot_name}_local_path",
            source_type=DataSourceType.ROBOT_LOCAL_PATH,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.PATH,
            data_source=data_source,
            render_options=render_options,
            layer_priority=4,  # Local paths on top of global paths
        )

        self._add_or_update_visualization(viz_config)

    def add_robot_trajectory(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add robot trajectory visualization (executed path history)"""
        if render_options is None:
            render_options = {}

        # Auto-assign color for trajectory (even more transparent)
        if "color" not in render_options:
            base_color = self.color_manager.get_robot_color(robot_name)
            render_options["color"] = base_color.to_hex()
            render_options["alpha"] = 0.5  # More transparent for history
            render_options["line_width"] = render_options.get("line_width", 1)

        data_source = RobotDataSource(
            name=f"{robot_name}_trajectory",
            source_type=DataSourceType.ROBOT_TRAJECTORY,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.TRAJECTORY,
            data_source=data_source,
            render_options=render_options,
            layer_priority=1,  # Trajectories in background
        )

        self._add_or_update_visualization(viz_config)

    # Environment/World visualizations (robot-independent)
    def add_occupancy_grid(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add occupancy grid map visualization"""
        data_source = EnvironmentDataSource(
            name="occupancy_grid",
            source_type=DataSourceType.OCCUPANCY_GRID,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.OCCUPANCY_GRID,
            data_source=data_source,
            render_options=render_options or {},
            layer_priority=-10,  # Background layer
        )

        self._add_or_update_visualization(viz_config)

    def add_3d_map(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add 3D map visualization"""
        data_source = EnvironmentDataSource(
            name="map_3d",
            source_type=DataSourceType.MAP_3D,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.POINT_CLOUD,
            data_source=data_source,
            render_options=render_options or {},
            layer_priority=-5,  # Background layer
        )

        self._add_or_update_visualization(viz_config)

    def add_global_navigation_path(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add global navigation path visualization (robot-independent)"""
        if render_options is None:
            render_options = {}

        # Default styling for global navigation paths
        if "color" not in render_options:
            render_options["color"] = "#00FF00"  # Green for global navigation
        if "line_width" not in render_options:
            render_options["line_width"] = 4
        if "alpha" not in render_options:
            render_options["alpha"] = 0.8

        data_source = EnvironmentDataSource(
            name="global_navigation_path",
            source_type=DataSourceType.GLOBAL_NAVIGATION_PATH,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.PATH,
            data_source=data_source,
            render_options=render_options,
            layer_priority=5,  # Foreground layer
        )

        self._add_or_update_visualization(viz_config)

    # Convenience alias for backward compatibility
    def add_navigation_path(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add global navigation path visualization (alias for add_global_navigation_path)"""
        return self.add_global_navigation_path(topic, frame_id, render_options)

    def add_tf_tree(
        self,
        topic: str = "/tf",
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add TF tree visualization"""
        data_source = EnvironmentDataSource(
            name="tf_tree",
            source_type=DataSourceType.TF_TREE,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.TRANSFORM_TREE,
            data_source=data_source,
            render_options=render_options or {},
        )

        self._add_or_update_visualization(viz_config)

    # Query and management methods
    def get_robot_visualizations(self, robot_name: str) -> List[VisualizationConfig]:
        """Get all visualizations for a specific robot"""
        return [
            viz
            for viz in self.visualizations
            if viz.data_source.robot_name == robot_name
        ]

    def get_global_visualizations(self) -> List[VisualizationConfig]:
        """Get all robot-independent visualizations"""
        return [viz for viz in self.visualizations if not viz.is_robot_specific()]

    def get_visualizations_by_type(
        self, viz_type: VisualizationType
    ) -> List[VisualizationConfig]:
        """Get all visualizations of a specific type"""
        return [viz for viz in self.visualizations if viz.viz_type == viz_type]

    def get_enabled_visualizations(self) -> List[VisualizationConfig]:
        """Get all enabled visualizations sorted by layer priority"""
        enabled = [viz for viz in self.visualizations if viz.enabled]
        return sorted(enabled, key=lambda v: v.layer_priority, reverse=True)

    def enable_robot_visualizations(self, robot_name: str) -> None:
        """Enable all visualizations for a specific robot"""
        for viz in self.get_robot_visualizations(robot_name):
            viz.enabled = True

    def disable_robot_visualizations(self, robot_name: str) -> None:
        """Disable all visualizations for a specific robot"""
        for viz in self.get_robot_visualizations(robot_name):
            viz.enabled = False

    def remove_robot_visualizations(self, robot_name: str) -> int:
        """Remove all visualizations for a specific robot"""
        initial_count = len(self.visualizations)
        self.visualizations = [
            viz
            for viz in self.visualizations
            if viz.data_source.robot_name != robot_name
        ]
        return initial_count - len(self.visualizations)

    def get_summary(self) -> Dict[str, Any]:
        """Get a comprehensive summary of all visualizations"""
        summary = {
            "name": self.name,
            "total_visualizations": len(self.visualizations),
            "enabled_visualizations": len(self.get_enabled_visualizations()),
            "robot_specific": len(
                [v for v in self.visualizations if v.is_robot_specific()]
            ),
            "global": len(self.get_global_visualizations()),
            "by_robot": {},
            "by_type": {},
            "by_data_source": {},
        }

        # Group by robot
        for viz in self.visualizations:
            if viz.is_robot_specific():
                robot = viz.data_source.robot_name
                if robot not in summary["by_robot"]:
                    summary["by_robot"][robot] = 0
                summary["by_robot"][robot] += 1

        # Group by visualization type
        for viz in self.visualizations:
            viz_type = viz.viz_type.value
            if viz_type not in summary["by_type"]:
                summary["by_type"][viz_type] = 0
            summary["by_type"][viz_type] += 1

        # Group by data source type
        for viz in self.visualizations:
            source_type = viz.data_source.source_type.value
            if source_type not in summary["by_data_source"]:
                summary["by_data_source"][source_type] = 0
            summary["by_data_source"][source_type] += 1

        return summary

    def _add_or_update_visualization(self, viz_config: VisualizationConfig) -> None:
        """Add new visualization or update existing one"""
        # Check if similar visualization already exists
        for i, existing in enumerate(self.visualizations):
            if (
                existing.data_source.name == viz_config.data_source.name
                and existing.viz_type == viz_config.viz_type
            ):
                # Update existing
                self.visualizations[i] = viz_config
                return

        # Add new visualization
        self.visualizations.append(viz_config)

    def _get_viz_type_for_sensor(self, sensor: SensorInstance) -> VisualizationType:
        """Determine appropriate visualization type for sensor"""
        from ..sensors import Camera, LaserScan, Lidar3D

        if isinstance(sensor, Camera):
            return VisualizationType.CAMERA_FEED
        elif isinstance(sensor, LaserScan):
            return VisualizationType.LASER_SCAN
        elif isinstance(sensor, Lidar3D):
            return VisualizationType.POINT_CLOUD
        else:
            return VisualizationType.MARKERS

    def __str__(self) -> str:
        return f"DataViz(name='{self.name}', visualizations={len(self.visualizations)})"

    def __repr__(self) -> str:
        return f"DataViz(name='{self.name}', visualizations={len(self.visualizations)})"
