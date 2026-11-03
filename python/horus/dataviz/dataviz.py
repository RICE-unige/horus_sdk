"""
Data visualization system for robot sensors and environmental data in HORUS SDK
"""

import colorsys
import hashlib
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Union

from ..sensors import SensorInstance
from .models import (
    DataSource,
    DataSourceType,
    EnvironmentDataSource,
    MapRenderTarget,
    MapUpdateMode,
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

    # Recognized declarative transport lanes for 3D-map visualizations. These mirror the bridge's
    # OutboundMessagePolicy tokens and are honored verbatim by the bridge when forwarded from Unity.
    _VALID_TRANSPORT_LANES = (
        "strict",
        "replaceable",
        "bulk_strict",
        "bulk_replaceable",
    )

    @classmethod
    def _apply_transport_lane(
        cls,
        render_options: Dict[str, Any],
        transport_lane: Optional[str],
    ) -> None:
        """
        Optionally stamp an explicit transport lane onto a map visualization's render options.

        Default (transport_lane=None) is a no-op, preserving the bridge's heuristic lane
        classification. An explicit caller value already present in render_options is respected
        and not overwritten.
        """
        if transport_lane is None or "transport_lane" in render_options:
            return
        normalized = str(transport_lane).strip().lower()
        if normalized not in cls._VALID_TRANSPORT_LANES:
            raise ValueError(
                "transport_lane must be one of "
                f"{cls._VALID_TRANSPORT_LANES}, got '{transport_lane}'"
            )
        render_options["transport_lane"] = normalized

    @staticmethod
    def _normalize_map_option(value: Any, option_type: Any, option_name: str):
        if isinstance(value, option_type):
            return value
        try:
            return option_type(str(value).strip().lower())
        except ValueError as exception:
            choices = ", ".join(item.value for item in option_type)
            raise ValueError(
                f"{option_name} must be one of ({choices}), got '{value}'"
            ) from exception

    @staticmethod
    def _coerce_bool(value: Any, default: bool = False) -> bool:
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"1", "true", "yes", "on"}:
                return True
            if normalized in {"0", "false", "no", "off"}:
                return False
        return bool(value)

    # Environment/World visualizations (robot-independent)
    def add_occupancy_grid(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
        transport_lane: Optional[str] = None,
    ) -> None:
        """Add occupancy grid map visualization"""
        render_options = dict(render_options or {})
        self._apply_transport_lane(render_options, transport_lane)
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

    def add_costmap(
        self,
        topic: str,
        frame_id: str = "map",
        *,
        alpha: float = 0.65,
        show_unknown_space: bool = False,
        height_offset: float = 0.01,
        stale_timeout: float = 5.0,
        tf_topic: str = "/tf",
    ) -> None:
        """Add a full nav_msgs/OccupancyGrid cost layer in a workspace fixed frame.

        Nav2 publishers must set ``always_send_full_costmap: true``. Raw 0..255
        costmaps and incremental ``*_updates`` messages are not this wire type.
        Free cells are transparent; costs use the RViz costmap palette. Offset
        is in workspace metres (after scaling). A zero timeout retains static
        grids indefinitely; otherwise stale layers are hidden, not shown as live.
        """
        for name, value in (("topic", topic), ("frame_id", frame_id), ("tf_topic", tf_topic)):
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"{name} must be a non-empty string")
        if not isinstance(show_unknown_space, bool):
            raise ValueError("show_unknown_space must be a boolean")
        values = {"alpha": alpha, "height_offset": height_offset, "stale_timeout": stale_timeout}
        for name, value in values.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        if not 0.0 <= alpha <= 1.0:
            raise ValueError("alpha must be between 0 and 1")
        if not 0.0 <= height_offset <= 1.0 or stale_timeout < 0.0:
            raise ValueError("height_offset must be between 0 and 1; stale_timeout must be non-negative")
        topic = topic.strip()
        self._add_or_update_visualization(VisualizationConfig(
            viz_type=VisualizationType.COSTMAP,
            data_source=EnvironmentDataSource(
                name=f"costmap:{topic}", source_type=DataSourceType.COSTMAP,
                topic=topic, frame_id=frame_id.strip(),
            ),
            render_options={**values, "show_unknown_space": show_unknown_space,
                            "tf_topic": tf_topic.strip(), "transport_lane": "bulk_replaceable"},
            layer_priority=-9,
        ))

    def add_3d_map(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
        transport_lane: Optional[str] = None,
        render_target: Union[MapRenderTarget, str] = MapRenderTarget.QUEST,
        update_mode: Optional[Union[MapUpdateMode, str]] = None,
    ) -> None:
        """Add a Quest-rendered or remotely rendered 3D map.

        ``render_target="quest"`` consumes geometry from ``topic`` and renders it
        on the headset. ``render_target="remote"`` treats ``topic`` as the color
        source for the synchronized, pose-adaptive WebRTC RGB-D renderer.
        """
        target = self._normalize_map_option(
            render_target, MapRenderTarget, "render_target"
        )
        if target is MapRenderTarget.REMOTE:
            if update_mode is not None:
                raise ValueError(
                    "remote rendering is always pose-adaptive; update_mode only "
                    "applies to Quest-rendered geometry"
                )
            if transport_lane is not None:
                raise ValueError(
                    "transport_lane applies to Quest geometry transport, not remote RGB-D streams"
                )
            self.add_remote_rendered_map(
                stream_topic=topic,
                frame_id=frame_id,
                render_options=render_options,
            )
            return

        render_options = dict(render_options or {})
        if update_mode is not None:
            mode = self._normalize_map_option(
                update_mode, MapUpdateMode, "update_mode"
            )
        else:
            mode = (
                MapUpdateMode.STATIC
                if self._coerce_bool(render_options.get("map_static_mode"), True)
                else MapUpdateMode.REFRESH
            )
        render_options["render_target"] = target.value
        render_options["update_mode"] = mode.value
        render_options["map_static_mode"] = mode is MapUpdateMode.STATIC
        self._apply_transport_lane(render_options, transport_lane)
        data_source = EnvironmentDataSource(
            name="map_3d",
            source_type=DataSourceType.MAP_3D,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.POINT_CLOUD,
            data_source=data_source,
            render_options=render_options,
            layer_priority=-5,  # Background layer
        )

        self._add_or_update_visualization(viz_config)

    def add_3d_mesh(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
        transport_lane: Optional[str] = None,
    ) -> None:
        """Add global 3D mesh map visualization."""
        render_options = dict(render_options or {})
        self._apply_transport_lane(render_options, transport_lane)
        data_source = EnvironmentDataSource(
            name="map_3d_mesh",
            source_type=DataSourceType.MAP_3D,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.MESH,
            data_source=data_source,
            render_options=render_options,
            layer_priority=-4,
        )

        self._add_or_update_visualization(viz_config)

    def add_nvblox_mesh(
        self,
        topic: str = "/horus/nvblox/mesh",
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add a live nvblox mesh using the bounded native Quest path.

        The nvblox adapter publishes lossless spatial page replacements on
        ``<topic>/indexed``. These defaults preserve every triangle while bounding
        main-thread upload work so live mapping cannot monopolize a Quest frame.
        Callers may override any value through ``render_options``.
        """
        self.add_live_mesh(topic, frame_id, render_options)

    def add_live_mesh(
        self,
        topic: str = "/horus/maps/live",
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Register any HMSH v2 map adapter with revision-based repair.

        This registers the adapter output, not its raw source topic. It does not
        start a source adapter; use ``python -m horus.mapping.ros_mesh`` for ROS
        TRIANGLE_LIST maps, or another HMSH v2 producer for other map formats.
        """
        options: Dict[str, Any] = {
            "max_triangles": 2_000_000,
            "use_vertex_colors": True,
            "double_sided": False,
            "source_coordinate_space": "enu",
            "use_indexed_binary": True,
            "indexed_live": True,
            "static_mesh": False,
            "max_chunk_renderers": 1024,
            "max_chunk_uploads_per_frame": 2,
            "max_triangle_uploads_per_frame": 20_000,
            "max_upload_milliseconds_per_frame": 2.5,
            "enable_perf_diagnostics": True,
        }
        options.update(dict(render_options or {}))
        if "display" in options:
            from .mesh_display import mesh_display_options
            options["display"] = mesh_display_options(options["display"])
        self.add_3d_mesh(
            topic=topic,
            frame_id=frame_id,
            render_options=options,
            transport_lane="bulk_strict",
        )

    def add_adaptive_mesh(
        self,
        topic: str,
        frame_id: str = "map",
        interests: Optional[List[Dict[str, Any]]] = None,
        initial_triangle_budget: int = 500_000,
        min_triangle_budget: int = 100_000,
        max_triangle_budget: int = 2_000_000,
        memory_budget_mib: int = 192,
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Register independent adaptive views from ``horus.mapping.adaptive_mesh_ros``.

        The server retains original geometry and supplies a colour-aware context
        approximation outside the interest areas. A frame-bound region follows
        TF; a region without ``frame`` becomes a movable, two-hand-resizable volume.
        Budgets are policy settings, not guaranteed headset triangle capacities.
        """
        import math
        if not topic.startswith("/") or topic.endswith("/") or not frame_id.strip():
            raise ValueError("adaptive mesh requires an absolute topic and a map frame")
        if not 1000 <= min_triangle_budget <= initial_triangle_budget <= max_triangle_budget:
            raise ValueError("adaptive triangle budgets must satisfy min <= initial <= max")
        if not 16 <= memory_budget_mib <= 384:
            raise ValueError("adaptive headset memory budget must be 16..384 MiB")
        regions = list(interests) if interests is not None else [
            {"id": "inspection", "shape": "box", "center": [0, 0, 1], "size": [2, 2, 2]}]
        if len(regions) > 16:
            raise ValueError("at most 16 interest regions per headset")
        ids = set()
        for region in regions:
            if not isinstance(region, dict) or not isinstance(region.get("id"), str) or not region["id"] or region["id"] in ids:
                raise ValueError("interest regions require unique IDs")
            ids.add(region["id"])
            if region.get("shape", "box") not in ("box", "sphere"):
                raise ValueError("interest shape must be box or sphere")
            for name, default in (("center", [0, 0, 0]), ("size", [2, 2, 2]), ("rotation", [0, 0, 0, 1])):
                value = region.get(name, default)
                if not isinstance(value, (list, tuple)) or len(value) != len(default) or not all(
                    not isinstance(x, bool) and math.isfinite(float(x)) for x in value):
                    raise ValueError(f"interest {name} requires {len(default)} finite numbers")
                if name == "size" and not all(.1 <= float(x) <= 30 for x in value):
                    raise ValueError("interest size must be 0.1..30 metres")
                if name == "rotation" and sum(float(x) ** 2 for x in value) < 1e-12:
                    raise ValueError("interest rotation must be nonzero")
            for name, default, low, high in (("priority", 1, 0, 100), ("min_fraction", .25, 0, 1)):
                value = region.get(name, default)
                if isinstance(value, bool) or not math.isfinite(float(value)) or not low <= float(value) <= high:
                    raise ValueError(f"interest {name} must be between {low} and {high}")
            if region.get("shape", "box") == "sphere" and len(set(float(x) for x in region.get("size", [2, 2, 2]))) != 1:
                raise ValueError("sphere diameters must be equal on all three axes")
        options = dict(render_options or {})
        options["adaptive"] = {
            "enabled": True, "initial_triangle_budget": initial_triangle_budget,
            "min_triangle_budget": min_triangle_budget, "max_triangle_budget": max_triangle_budget,
            "memory_budget_mib": memory_budget_mib, "frame_headroom": .15,
            "regions": regions,
        }
        self.add_live_mesh(topic, frame_id, options)

    def add_3d_octomap(
        self,
        topic: str,
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
        transport_lane: Optional[str] = None,
    ) -> None:
        """Add global OctoMap visualization metadata."""
        render_options = dict(render_options or {})
        self._apply_transport_lane(render_options, transport_lane)
        data_source = EnvironmentDataSource(
            name="map_3d_octomap",
            source_type=DataSourceType.MAP_3D,
            topic=topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.OCTOMAP,
            data_source=data_source,
            render_options=render_options,
            layer_priority=-4,
        )

        self._add_or_update_visualization(viz_config)

    def add_remote_rendered_map(
        self,
        stream_topic: str = "/horus/remote_render/map_portal",
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Add a versioned workspace-level remote renderer.

        V3 WebRTC is the production default: exact display-time stereo views
        are rendered from the complete PC-resident scene. The older RGB-D v2
        contract remains available only when requested explicitly.
        """
        options = dict(render_options or {})
        options["render_target"] = MapRenderTarget.REMOTE.value
        protocol_version = str(
            options.get("protocol_version", "horus.remote_render.v3")
            or "horus.remote_render.v3"
        ).strip()
        if protocol_version not in {
            "remote_stereo_rgbd_v2",
            "horus.remote_render.v3",
        }:
            raise ValueError(
                "remote render protocol_version must be "
                "'remote_stereo_rgbd_v2' or 'horus.remote_render.v3'"
            )
        options["protocol_version"] = protocol_version
        default_transport = (
            "webrtc"
            if protocol_version == "horus.remote_render.v3"
            else "ros_compressed"
        )
        transport = str(
            options.get("transport", default_transport) or default_transport
        ).strip().lower()
        if transport not in {"webrtc", "ros_compressed"}:
            raise ValueError("remote render transport must be 'webrtc' or 'ros_compressed'")
        options["transport"] = transport
        if protocol_version == "horus.remote_render.v3":
            image_type = str(
                options.get(
                    "image_type",
                    "h264" if transport == "webrtc" else "raw",
                )
                or "raw"
            ).strip().lower()
            if image_type not in {"raw", "h264"}:
                raise ValueError(
                    "remote render image_type must be 'raw' or 'h264'"
                )
            options["image_type"] = image_type
            options["format_version"] = (
                "horus.remote_render.v3.webrtc"
                if transport == "webrtc"
                else "horus.remote_render.v3.ros"
            )
            options.setdefault(
                "viewer_pose_topic",
                "/horus/remote_render/v3/view_state",
            )
            options.setdefault(
                "ros_compressed_topic",
                "/horus/remote_render/v3/stereo",
            )
            options.setdefault("status_topic", "/horus/remote_render/v3/status")
            options.setdefault("session_id", "")
            options.setdefault("reference_space_id", "workspace-local")
            options.setdefault("workspace_revision", 0)
            options.setdefault("viewport_width", 1024)
            options.setdefault("viewport_height", 1024)
            options.setdefault("display_refresh_rate_hz", 72.0)
            options.setdefault("maximum_frame_age_ms", 250)
            options.setdefault("viewer_pose_rate_hz", 72.0)
            if transport == "webrtc":
                # V3 frame identity rides inside the encoded access unit. It
                # does not use the independent v2 depth data channel.
                options["frame_data_topic"] = ""
        else:
            options["format_version"] = (
                "remote_stereo_rgbd_ros_v2"
                if transport == "ros_compressed"
                else "remote_stereo_rgbd_webrtc_v2"
            )
            options.setdefault("viewer_pose_topic", "/horus/remote_render/viewer_pose")
            options.setdefault("viewer_pose_rate_hz", 60.0)
            options.setdefault(
                "ros_compressed_topic",
                "/horus/remote_render/rgbd",
            )
            options.setdefault("status_topic", "/horus/remote_render/agent_status")
        options.setdefault("frame_data_topic", "/horus/remote_render/frame_data")
        options.setdefault("client_signal_topic", "/horus/webrtc/client_signal")
        options.setdefault("server_signal_topic", "/horus/webrtc/server_signal")
        options.setdefault("encoder", "nvenc")
        options.setdefault("bitrate_kbps", 12000)
        options.setdefault("framerate", 60)
        options.setdefault("flip_y", False)

        data_source = EnvironmentDataSource(
            name="remote_rendered_map",
            source_type=DataSourceType.REMOTE_RENDER,
            topic=stream_topic,
            frame_id=frame_id,
        )
        self._add_or_update_visualization(
            VisualizationConfig(
                viz_type=VisualizationType.REMOTE_RENDER,
                data_source=data_source,
                render_options=options,
                layer_priority=-4,
            )
        )

    def add_gaussian_splat_map(
        self,
        manifest_topic: str = "/horus/gaussian_splat/manifest",
        frame_id: str = "map",
        render_options: Optional[Dict[str, Any]] = None,
        preview_topic: str = "/map_gaussian_splat_preview",
        transport_lane: Optional[str] = None,
    ) -> None:
        """Add a global Gaussian Splat map visualization."""
        options = dict(render_options or {})
        self._apply_transport_lane(options, transport_lane)
        options.setdefault("manifest_topic", manifest_topic)
        options.setdefault("chunk_begin_topic", "/horus/gaussian_splat/chunk_begin")
        options.setdefault("chunk_item_topic", "/horus/gaussian_splat/chunk_item")
        options.setdefault("chunk_end_topic", "/horus/gaussian_splat/chunk_end")
        options.setdefault("binary_chunks", False)
        if not isinstance(options["binary_chunks"], bool):
            raise ValueError("binary_chunks must be a boolean")
        options.setdefault("asset_format", "3dgs_ply")
        options.setdefault("source_coordinate_space", "colmap")
        options.setdefault("render_mode", "splats")
        options.setdefault("max_splats", 350000)
        options.setdefault("render_scale", 0.5)
        options.setdefault("sh_order", 2)
        options.setdefault("half_precision_sh", True)
        options.setdefault("adaptive_sort", True)
        options.setdefault("sort_passes", 2)
        options.setdefault("opacity_scale", 1.0)
        options.setdefault("splat_scale", 1.0)
        options.setdefault("contribution_cull_threshold", 0.1)
        options.setdefault("high_precision_rt", False)
        options.setdefault("preserve_source_detail", False)
        if not isinstance(options["preserve_source_detail"], bool):
            raise ValueError("preserve_source_detail must be a boolean")
        if options["preserve_source_detail"]:
            # max_splats becomes an admission limit, never a prefix selection.
            options.update(render_scale=1.0, sh_order=3, half_precision_sh=False,
                           adaptive_sort=False, sort_passes=4,
                           contribution_cull_threshold=0.0, high_precision_rt=True)
        options.setdefault("pointcloud_fallback", True)
        options.setdefault("fallback_topic", preview_topic)
        options.setdefault("fallback_frame", frame_id)
        options.setdefault(
            "fallback_point_cloud",
            {
                "point_size": 0.035,
                "max_points_per_frame": 0,
                "base_sample_stride": 1,
                "render_all_points": True,
                "auto_point_size_by_workspace_scale": True,
                "min_point_size": 0.0015,
                "max_point_size": 0.012,
                "point_shape": "circle",
                "enable_view_frustum_culling": False,
                "enable_subpixel_culling": False,
                "min_screen_radius_px": 0.0,
                "visible_points_budget": 120000,
                "max_visible_points_budget": 180000,
                "map_static_mode": True,
                "render_mode": "opaque_fast",
            },
        )

        data_source = EnvironmentDataSource(
            name="gaussian_splat",
            source_type=DataSourceType.MAP_3D,
            topic=manifest_topic,
            frame_id=frame_id,
        )

        viz_config = VisualizationConfig(
            viz_type=VisualizationType.GAUSSIAN_SPLAT,
            data_source=data_source,
            render_options=options,
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
