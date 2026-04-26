"""
Data visualization system for robot sensors and environmental data in HORUS SDK
"""

import colorsys
import hashlib
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from ..sensors import SensorInstance
from .models import (
    DataSource,
    DataSourceType,
    EnvironmentDataSource,
    RobotDataSource,
    SensorDataSource,
    VisualizationConfig,
    VisualizationType,
)

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
    color_manager: Optional[Any] = None

    def __post_init__(self):
        """Validate DataViz configuration"""
        if not self.name:
            raise ValueError("DataViz name cannot be empty")

        # Initialize color manager if not provided
        if self.color_manager is None:
            from ..color.color_manager import ColorManager

            self.color_manager = ColorManager()

    @staticmethod
    def _deterministic_robot_hex_color(robot_name: str) -> str:
        """
        Generate a stable, robot-unique color independent of DataViz instance state.
        """
        normalized_name = str(robot_name or "").strip().lower().encode("utf-8")
        digest = hashlib.sha256(normalized_name).hexdigest()
        # Lift channel floor so colors remain visible in MR overlays.
        r = 64 + (int(digest[0:2], 16) // 2)
        g = 64 + (int(digest[2:4], 16) // 2)
        b = 64 + (int(digest[4:6], 16) // 2)
        return f"#{r:02x}{g:02x}{b:02x}"

    @staticmethod
    def _deterministic_robot_laser_hex_color(robot_name: str) -> str:
        """
        Generate a vivid, high-separation laser color from robot name.
        """
        normalized_name = str(robot_name or "").strip().lower().encode("utf-8")
        digest = hashlib.sha256(normalized_name).digest()
        hue = ((digest[0] << 8) | digest[1]) / 65535.0
        saturation = 0.82 + (digest[2] / 255.0) * 0.10
        value = 0.86 + (digest[3] / 255.0) * 0.10
        red, green, blue = colorsys.hsv_to_rgb(
            hue % 1.0,
            min(0.95, saturation),
            min(0.96, value),
        )
        return "#{:02x}{:02x}{:02x}".format(
            int(round(red * 255.0)),
            int(round(green * 255.0)),
            int(round(blue * 255.0)),
        )

    @staticmethod
    def _deterministic_robot_nav_path_hex_color(robot_name: str, role: str) -> str:
        """
        Generate a stable robot-specific nav-path color with global/local gradient variants.
        """
        normalized_name = str(robot_name or "").strip().lower().encode("utf-8")
        digest = hashlib.sha256(normalized_name).digest()
        hue = (digest[0] / 255.0 + (digest[1] / 255.0) * 0.12) % 1.0

        if str(role).strip().lower() == "local":
            saturation = 0.54
            value = 0.97
        else:
            saturation = 0.82
            value = 0.90

        red, green, blue = colorsys.hsv_to_rgb(hue, saturation, value)
        return "#{:02x}{:02x}{:02x}".format(
            int(round(red * 255.0)),
            int(round(green * 255.0)),
            int(round(blue * 255.0)),
        )

    # Sensor-based visualizations
    def add_sensor_visualization(
        self,
        sensor: SensorInstance,
        robot_name: str,
        enabled: bool = True,
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add a sensor-based visualization with automatic color assignment."""
        if render_options is None:
            render_options = {}

        # Create sensor data source
        data_source = SensorDataSource(sensor, robot_name=robot_name)

        # Determine visualization type
        viz_type = self._get_viz_type_for_sensor(sensor)

        # Auto-assign color based on sensor type and robot
        if "color" not in render_options:
            if viz_type == VisualizationType.LASER_SCAN:
                render_options["color"] = self._deterministic_robot_laser_hex_color(robot_name)
                render_options["alpha"] = 0.8
            else:
                # Sensor colors must stay stable across separate DataViz instances.
                render_options["color"] = self._deterministic_robot_hex_color(robot_name)

        # Create and add visualization
        viz_config = VisualizationConfig(
            viz_type=viz_type,
            data_source=data_source,
            enabled=bool(enabled),
            render_options=render_options,
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
        """Add robot global path planning visualization with automatic color
        assignment"""
        if render_options is None:
            render_options = {}

        # Auto-assign color for global path
        if "color" not in render_options:
            render_options["color"] = self._deterministic_robot_nav_path_hex_color(
                robot_name, "global"
            )
            render_options["alpha"] = 0.9
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
        """Add robot local path planning visualization with automatic color
        assignment"""
        if render_options is None:
            render_options = {}

        # Auto-assign color for local path (lighter than global)
        if "color" not in render_options:
            render_options["color"] = self._deterministic_robot_nav_path_hex_color(
                robot_name, "local"
            )
            render_options["alpha"] = 0.72
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

    def add_robot_velocity_data(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add robot planar velocity text visualization backed by odometry."""
        if render_options is None:
            render_options = {}

        if "color" not in render_options:
            color = self.color_manager.get_robot_color(robot_name)
            render_options["color"] = color.to_hex()
        render_options.setdefault("units", "m/s")
        render_options.setdefault("text_back_offset_m", 0.36)
        render_options.setdefault("floor_offset_m", 0.01)
        render_options.setdefault("update_hz", 10.0)

        data_source = RobotDataSource(
            name=f"{robot_name}_velocity_data",
            source_type=DataSourceType.ROBOT_ODOMETRY,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.VELOCITY_DATA,
            data_source=data_source,
            render_options=render_options,
            layer_priority=6,
        )

        self._add_or_update_visualization(viz_config)

    def add_robot_odometry_trail(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add short decimated odometry trail visualization."""
        if render_options is None:
            render_options = {}

        if "color" not in render_options:
            render_options["color"] = self._deterministic_robot_hex_color(robot_name)
        render_options.setdefault("max_points", 48)
        render_options.setdefault("history_seconds", 3.2)
        render_options.setdefault("min_spacing_m", 0.07)
        render_options.setdefault("line_width_m", 0.0096)
        render_options.setdefault("trail_back_offset_m", 0.44)

        data_source = RobotDataSource(
            name=f"{robot_name}_odometry_trail",
            source_type=DataSourceType.ROBOT_ODOMETRY,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.ODOMETRY_TRAIL,
            data_source=data_source,
            render_options=render_options,
            layer_priority=2,
        )

        self._add_or_update_visualization(viz_config)

    def add_robot_collision_risk(
        self,
        robot_name: str,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add collision risk visualization driven by SDK-published risk signal."""
        if render_options is None:
            render_options = {}

        if "color" not in render_options:
            render_options["color"] = "#FF6A00"
        render_options.setdefault("threshold_m", 0.45)
        render_options.setdefault("radius_m", 0.45)
        render_options.setdefault("source", "laser_scan")
        render_options.setdefault("alpha_min", 0.0)
        render_options.setdefault("alpha_max", 0.55)

        data_source = RobotDataSource(
            name=f"{robot_name}_collision_risk",
            source_type=DataSourceType.ROBOT_COLLISION_RISK,
            topic=topic,
            robot_name=robot_name,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.COLLISION_RISK,
            data_source=data_source,
            render_options=render_options,
            layer_priority=7,
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

    def add_3d_mesh(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add global 3D mesh map visualization."""
        data_source = EnvironmentDataSource(
            name="map_3d_mesh",
            source_type=DataSourceType.MAP_3D,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.MESH,
            data_source=data_source,
            render_options=render_options or {},
            layer_priority=-4,
        )

        self._add_or_update_visualization(viz_config)

    def add_3d_octomap(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add global OctoMap visualization metadata."""
        data_source = EnvironmentDataSource(
            name="map_3d_octomap",
            source_type=DataSourceType.MAP_3D,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.OCTOMAP,
            data_source=data_source,
            render_options=render_options or {},
            layer_priority=-4,
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
        """Add global navigation path visualization (alias for
        add_global_navigation_path)"""
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

    def add_semantic_box(
        self,
        semantic_id: str,
        label: str,
        center: Tuple[float, float, float],
        size: Tuple[float, float, float],
        frame_id: str = "map",
        rotation_offset_euler: Optional[Tuple[float, float, float]] = None,
        enabled: bool = True,
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add a static semantic perception box as a global map overlay."""
        semantic_id = str(semantic_id or "").strip()
        if not semantic_id:
            raise ValueError("semantic_id cannot be empty")

        label = str(label or "").strip()
        if not label:
            raise ValueError("label cannot be empty")

        if center is None or len(center) != 3:
            raise ValueError("center must be a 3-tuple")
        if size is None or len(size) != 3:
            raise ValueError("size must be a 3-tuple")

        options = dict(render_options or {})
        options["id"] = semantic_id
        options["label"] = label
        options["center"] = [float(center[0]), float(center[1]), float(center[2])]
        options["size"] = [float(size[0]), float(size[1]), float(size[2])]
        options["rotation_offset_euler"] = [
            float(rotation_offset_euler[0]) if rotation_offset_euler else 0.0,
            float(rotation_offset_euler[1]) if rotation_offset_euler else 0.0,
            float(rotation_offset_euler[2]) if rotation_offset_euler else 0.0,
        ]

        data_source = EnvironmentDataSource(
            name=f"semantic_box_{semantic_id}",
            source_type=DataSourceType.GLOBAL_MARKERS,
            topic=f"/horus/semantic_boxes/{semantic_id}",
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.SEMANTIC_BOX,
            data_source=data_source,
            enabled=bool(enabled),
            render_options=options,
            layer_priority=4,
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
        summary: Dict[str, Any] = {
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
        # Use sensor type matching instead of isinstance for exhaustive checking
        sensor_type = sensor.sensor_type

        if sensor_type.value == "camera":
            return VisualizationType.CAMERA_FEED
        elif sensor_type.value == "laser_scan":
            return VisualizationType.LASER_SCAN
        elif sensor_type.value == "lidar_3d":
            return VisualizationType.POINT_CLOUD
        else:
            # Handle any other sensor types
            return VisualizationType.MARKERS

    def __str__(self) -> str:
        return f"DataViz(name='{self.name}', visualizations={len(self.visualizations)})"

    def __repr__(self) -> str:
        return f"DataViz(name='{self.name}', visualizations={len(self.visualizations)})"
