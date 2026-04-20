"""
Robot object system for HORUS SDK
"""

from dataclasses import dataclass, field
from enum import Enum
import importlib
import inspect
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    from ..dataviz import DataViz
    from ..sensors import SensorInstance, SensorType


class RobotType(Enum):
    """Robot type classifications"""

    WHEELED = "wheeled"
    LEGGED = "legged"
    AERIAL = "aerial"
    DRONE = "drone"


@dataclass
class RobotDimensions:
    """Robot dimensions in meters (length, width, height)."""

    length: float
    width: float
    height: float


@dataclass
class Robot:
    """
    Base robot object with type and identification

    Args:
        name: Robot identifier/namespace (unique)
        robot_type: Classification of robot (wheeled, legged, aerial, drone)
        metadata: Optional additional robot information
        sensors: List of sensors attached to this robot
    """

    name: str
    robot_type: RobotType
    metadata: Optional[Dict[str, Any]] = None
    sensors: List["SensorInstance"] = field(default_factory=list)
    dimensions: Optional[RobotDimensions] = None

    _DEFAULT_LOGICAL_NAME = "robot"
    _ROS_BINDING_METADATA_KEY = "ros_binding_config"
    _LOCAL_BODY_MODEL_METADATA_KEY = "local_body_model_config"
    _WORKSPACE_TUTORIAL_METADATA_KEY = "workspace_tutorial_config"

    def __post_init__(self):
        """Validate robot configuration after initialization"""
        self.name = str(self.name or "").strip() or self._DEFAULT_LOGICAL_NAME

        if not isinstance(self.robot_type, RobotType):
            raise TypeError("robot_type must be a RobotType enum")

        # Initialize metadata if not provided
        if self.metadata is None:
            self.metadata = {}

        if self.dimensions is not None:
            if isinstance(self.dimensions, RobotDimensions):
                pass
            elif isinstance(self.dimensions, (list, tuple)):
                if len(self.dimensions) == 2:
                    length, width = self.dimensions
                    height = 0.0
                elif len(self.dimensions) == 3:
                    length, width, height = self.dimensions
                else:
                    raise TypeError(
                        "dimensions must be length/width or length/width/height"
                    )
                self.dimensions = RobotDimensions(
                    length=float(length),
                    width=float(width),
                    height=float(height),
                )
            elif isinstance(self.dimensions, dict):
                self.dimensions = RobotDimensions(
                    length=float(self.dimensions.get("length", 0.0)),
                    width=float(self.dimensions.get("width", 0.0)),
                    height=float(self.dimensions.get("height", 0.0)),
                )
            else:
                raise TypeError(
                    "dimensions must be RobotDimensions, tuple/list, or dict"
                )

            if (self.dimensions.length < 0 or
                    self.dimensions.width < 0 or
                    self.dimensions.height < 0):
                raise ValueError("dimensions must be non-negative")

    def __str__(self) -> str:
        """String representation of the robot"""
        return f"Robot(name='{self.name}', type={self.robot_type.value})"

    def __repr__(self) -> str:
        """Detailed representation of the robot"""
        return (
            f"Robot(name='{self.name}', robot_type={self.robot_type}, "
            f"metadata={self.metadata})"
        )

    def get_type_str(self) -> str:
        """Get robot type as string"""
        return self.robot_type.value

    def add_metadata(self, key: str, value: Any) -> None:
        """Add metadata to the robot"""
        self.metadata[key] = value

    def get_metadata(self, key: str, default: Any = None) -> Any:
        """Get metadata value by key"""
        return self.metadata.get(key, default)

    @staticmethod
    def _normalize_binding_mode(value: Any, default: str) -> str:
        normalized = str(value or "").strip().lower()
        return normalized if normalized in {"prefixed", "flat"} else default

    @staticmethod
    def _normalize_frame_token(value: Any, default: str = "") -> str:
        normalized = str(value or "").strip().strip("/")
        return normalized or default

    @staticmethod
    def _normalize_topic_prefix(value: Any) -> str:
        raw = str(value or "").strip()
        if not raw:
            return ""
        return "/" + raw.strip("/")

    @staticmethod
    def _normalize_topic_leaf(value: Any, default: str = "") -> str:
        normalized = str(value or "").strip().strip("/")
        return normalized or default

    def configure_ros_binding(
        self,
        tf_mode: str = "prefixed",
        topic_mode: str = "prefixed",
        base_frame: str = "base_link",
        tf_prefix: str = "",
        topic_prefix: str = "",
    ) -> None:
        """Configure how HORUS logical robot identity binds to ROS TF/topics."""
        resolved_tf_mode = self._normalize_binding_mode(tf_mode, "prefixed")
        resolved_topic_mode = self._normalize_binding_mode(topic_mode, "prefixed")
        resolved_base_frame = self._normalize_frame_token(base_frame, "base_link")

        if resolved_tf_mode == "prefixed":
            resolved_tf_prefix = self._normalize_frame_token(tf_prefix, self.name)
        else:
            resolved_tf_prefix = self._normalize_frame_token(tf_prefix)

        if resolved_topic_mode == "prefixed":
            resolved_topic_prefix = self._normalize_topic_prefix(topic_prefix or f"/{self.name}")
        else:
            resolved_topic_prefix = self._normalize_topic_prefix(topic_prefix)

        self.add_metadata(
            self._ROS_BINDING_METADATA_KEY,
            {
                "logical_name": self.name,
                "tf_mode": resolved_tf_mode,
                "topic_mode": resolved_topic_mode,
                "base_frame": resolved_base_frame,
                "tf_prefix": resolved_tf_prefix,
                "topic_prefix": resolved_topic_prefix,
            },
        )

    def get_ros_binding(self) -> Dict[str, str]:
        """Return the resolved ROS binding configuration for this robot."""
        raw = self.metadata.get(self._ROS_BINDING_METADATA_KEY, {})
        if not isinstance(raw, dict):
            raw = {}

        robot_desc_cfg = self.metadata.get("robot_description_config", {})
        desc_base_frame = ""
        if isinstance(robot_desc_cfg, dict):
            desc_base_frame = self._normalize_frame_token(
                robot_desc_cfg.get("base_frame"),
                "",
            )

        tf_mode = self._normalize_binding_mode(raw.get("tf_mode"), "prefixed")
        topic_mode = self._normalize_binding_mode(raw.get("topic_mode"), "prefixed")
        base_frame = self._normalize_frame_token(
            raw.get("base_frame"),
            desc_base_frame or "base_link",
        )

        if tf_mode == "prefixed":
            tf_prefix = self._normalize_frame_token(raw.get("tf_prefix"), self.name)
        else:
            tf_prefix = self._normalize_frame_token(raw.get("tf_prefix"))

        if topic_mode == "prefixed":
            topic_prefix = self._normalize_topic_prefix(raw.get("topic_prefix") or f"/{self.name}")
        else:
            topic_prefix = self._normalize_topic_prefix(raw.get("topic_prefix"))

        return {
            "logical_name": self.name,
            "tf_mode": tf_mode,
            "topic_mode": topic_mode,
            "base_frame": base_frame,
            "tf_prefix": tf_prefix,
            "topic_prefix": topic_prefix,
        }

    def resolve_tf_frame(self, frame_name: Optional[str] = None) -> str:
        """Resolve a frame name according to the configured TF binding mode."""
        binding = self.get_ros_binding()
        frame_token = self._normalize_frame_token(frame_name, binding["base_frame"])
        prefix = self._normalize_frame_token(binding.get("tf_prefix", ""))

        if prefix and (frame_token == prefix or frame_token.startswith(prefix + "/")):
            return frame_token

        if prefix:
            return f"{prefix}/{frame_token}"
        return frame_token

    def resolve_topic(self, topic_name: Optional[str] = None) -> str:
        """Resolve a topic according to the configured topic binding mode."""
        binding = self.get_ros_binding()
        raw_topic = str(topic_name or "").strip()
        if raw_topic.startswith("/"):
            return raw_topic

        topic_leaf = self._normalize_topic_leaf(raw_topic)
        prefix = self._normalize_topic_prefix(binding.get("topic_prefix", ""))
        if prefix:
            if not topic_leaf:
                return prefix
            prefixed_leaf = prefix.strip("/")
            if topic_leaf == prefixed_leaf or topic_leaf.startswith(prefixed_leaf + "/"):
                return "/" + topic_leaf
            return f"{prefix}/{topic_leaf}"

        return f"/{topic_leaf}" if topic_leaf else "/"

    # Sensor management methods
    def add_sensor(self, sensor: "SensorInstance") -> None:
        """Add a sensor to this robot"""
        # Check if sensor with same name already exists
        for existing_sensor in self.sensors:
            if existing_sensor.name == sensor.name:
                raise ValueError(f"Sensor with name '{sensor.name}' already exists")

        self.sensors.append(sensor)

    def remove_sensor(self, sensor_name: str) -> bool:
        """Remove a sensor by name. Returns True if sensor was found and removed."""
        for i, sensor in enumerate(self.sensors):
            if sensor.name == sensor_name:
                del self.sensors[i]
                return True
        return False

    def get_sensor(self, sensor_name: str) -> Optional["SensorInstance"]:
        """Get sensor by name"""
        for sensor in self.sensors:
            if sensor.name == sensor_name:
                return sensor
        return None

    def get_sensors_by_type(self, sensor_type: "SensorType") -> List["SensorInstance"]:
        """Get all sensors of a specific type"""
        return [sensor for sensor in self.sensors if sensor.sensor_type == sensor_type]

    def get_sensor_count(self) -> int:
        """Get total number of sensors"""
        return len(self.sensors)

    def has_sensors(self) -> bool:
        """Check if robot has any sensors"""
        return len(self.sensors) > 0

    def configure_workspace_tutorial(
        self,
        preset_id: str,
        enabled: bool = True,
    ) -> None:
        """Configure an opt-in workspace tutorial preset for MR onboarding."""
        normalized_preset_id = str(preset_id or "").strip()
        if not normalized_preset_id:
            raise ValueError("preset_id must be a non-empty string")

        self.add_metadata(
            self._WORKSPACE_TUTORIAL_METADATA_KEY,
            {
                "enabled": bool(enabled),
                "preset_id": normalized_preset_id,
            },
        )

    def create_dataviz(self, dataviz_name: Optional[str] = None) -> "DataViz":
        """
        Create a DataViz instance for this robot with all its sensors

        Args:
            dataviz_name: Name for the DataViz instance (defaults to robot name +
                         "_viz")

        Returns:
            DataViz instance configured with this robot's sensors
        """
        from ..dataviz import DataViz

        if dataviz_name is None:
            dataviz_name = f"{self.name}_viz"

        # Create DataViz instance
        dataviz = DataViz(name=dataviz_name)

        # Add all robot sensors to DataViz
        for sensor in self.sensors:
            dataviz.add_sensor_visualization(sensor, self.name)

        # Use the configured base frame when available so TF visualization matches
        # robots that do not use base_link (for example, go1/base).
        # Add robot transform visualization
        dataviz.add_robot_transform(
            robot_name=self.name,
            topic="/tf",
            frame_id=self.resolve_tf_frame(),
        )

        return dataviz

    def add_path_planning_to_dataviz(
        self,
        dataviz: "DataViz",
        global_path_topic: Optional[str] = None,
        local_path_topic: Optional[str] = None,
        trajectory_topic: Optional[str] = None,
    ) -> None:
        """
        Add path planning visualizations to a DataViz instance

        Args:
            dataviz: DataViz instance to add visualizations to
            global_path_topic: Topic for global path planning (optional)
            local_path_topic: Topic for local path planning (optional)
            trajectory_topic: Topic for executed trajectory (optional)
        """
        # Add global path if topic provided
        if global_path_topic:
            dataviz.add_robot_global_path(
                robot_name=self.name, topic=global_path_topic, frame_id="map"
            )

        # Add local path if topic provided
        if local_path_topic:
            dataviz.add_robot_local_path(
                robot_name=self.name, topic=local_path_topic, frame_id="map"
            )

        # Add trajectory if topic provided
        if trajectory_topic:
            dataviz.add_robot_trajectory(
                robot_name=self.name, topic=trajectory_topic, frame_id="map"
            )

    def add_navigation_safety_to_dataviz(
        self,
        dataviz: "DataViz",
        odom_topic: Optional[str] = None,
        collision_risk_topic: Optional[str] = None,
        include_velocity: bool = True,
        include_trail: bool = True,
        include_collision: bool = True,
    ) -> None:
        """
        Add navigation safety visualizations (velocity/trail/collision risk).

        Args:
            dataviz: DataViz instance to add visualizations to.
            odom_topic: Topic for nav_msgs/Odometry input.
            collision_risk_topic: Topic for SDK-published collision risk JSON.
            include_velocity: Include floor velocity text visualization.
            include_trail: Include short odometry trail visualization.
            include_collision: Include collision risk halo visualization.
        """
        resolved_odom_topic = odom_topic or f"/{self.name}/odom"
        resolved_collision_topic = collision_risk_topic or f"/{self.name}/collision_risk"

        if include_velocity:
            dataviz.add_robot_transform(
                robot_name=self.name,
                topic=resolved_odom_topic,
                frame_id="map",
            )

        if include_trail:
            dataviz.add_robot_trajectory(
                robot_name=self.name,
                topic=resolved_odom_topic,
                frame_id="map",
            )

        if include_collision:
            if hasattr(dataviz, "add_robot_collision_risk"):
                dataviz.add_robot_collision_risk(
                    robot_name=self.name,
                    topic=resolved_collision_topic,
                    frame_id=self.resolve_tf_frame(),
                )

    def configure_robot_description(
        self,
        urdf_path: str,
        base_frame: str = "base_link",
        source: str = "ros",
        ros_param_node: str = "",
        ros_param_name: str = "robot_description",
        chunk_size_bytes: int = 12000,
        is_transparent: bool = False,
        include_visual_meshes: bool = True,
        visual_mesh_triangle_budget: int = 90000,
        body_mesh_mode: str = "preview_mesh",
        enabled: bool = True,
    ) -> None:
        """
        Configure robot description resolution for MR collision/joint visualization.

        Args:
            urdf_path: Absolute or workspace-relative URDF file path.
            base_frame: Link frame used as MR anchoring root.
            source: Description source type (v1: "ros").
            ros_param_node: Optional ROS node name for fallback parameter fetch.
            ros_param_name: ROS parameter name (default: robot_description).
            chunk_size_bytes: Chunk size used for SDK->MR transport.
            is_transparent: Render collision body as transparent in MR.
            include_visual_meshes: Include baked visual meshes in the payload.
            visual_mesh_triangle_budget: Triangle budget for preview/runtime-high baked meshes.
            body_mesh_mode: collision_only, preview_mesh, runtime_high_mesh, or max_quality_mesh alias.
            enabled: Enable robot description manifest + chunk transport.
        """
        normalized_body_mesh_mode = str(body_mesh_mode or "preview_mesh").strip().lower()
        if normalized_body_mesh_mode == "max_quality_mesh":
            normalized_body_mesh_mode = "runtime_high_mesh"
        if normalized_body_mesh_mode not in {"collision_only", "preview_mesh", "runtime_high_mesh"}:
            normalized_body_mesh_mode = "preview_mesh"

        if normalized_body_mesh_mode == "collision_only":
            include_visual_meshes = False

        self.add_metadata(
            "robot_description_config",
            {
                "enabled": bool(enabled),
                "source": str(source or "ros"),
                "urdf_path": str(urdf_path or ""),
                "base_frame": str(base_frame or "base_link"),
                "ros_param_node": str(ros_param_node or ""),
                "ros_param_name": str(ros_param_name or "robot_description"),
                "chunk_size_bytes": int(max(1024, min(64000, chunk_size_bytes))),
                "is_transparent": bool(is_transparent),
                "include_visual_meshes": bool(include_visual_meshes),
                "visual_mesh_triangle_budget": int(max(2000, min(500000, visual_mesh_triangle_budget))),
                "body_mesh_mode": normalized_body_mesh_mode,
            },
        )

    def configure_local_body_model(
        self,
        robot_model_id: str,
        enabled: bool = True,
    ) -> None:
        """
        Configure a bundled MR-side robot body model by stable model identifier.

        Args:
            robot_model_id: Stable identifier for a model shipped with Horus MR.
            enabled: Whether the bundled model should be advertised to MR.
        """
        normalized_model_id = str(robot_model_id or "").strip().lower()
        self.add_metadata(
            self._LOCAL_BODY_MODEL_METADATA_KEY,
            {
                "enabled": bool(enabled) and bool(normalized_model_id),
                "robot_model_id": normalized_model_id,
            },
        )

    def create_full_dataviz(
        self,
        dataviz_name: Optional[str] = None,
        global_path_topic: Optional[str] = None,
        local_path_topic: Optional[str] = None,
        trajectory_topic: Optional[str] = None,
    ) -> "DataViz":
        """
        Create a complete DataViz instance with sensors and path planning

        Args:
            dataviz_name: Name for the DataViz instance
            global_path_topic: Topic for global path planning
            local_path_topic: Topic for local path planning
            trajectory_topic: Topic for executed trajectory

        Returns:
            DataViz instance with sensors and path planning configured
        """
        # Create base DataViz with sensors
        dataviz = self.create_dataviz(dataviz_name)

        # Add path planning visualizations
        self.add_path_planning_to_dataviz(
            dataviz, global_path_topic, local_path_topic, trajectory_topic
        )

        return dataviz

    def register_with_horus(
        self,
        dataviz: Optional["DataViz"] = None,
        keep_alive: bool = True,
        show_dashboard: bool = True,
        workspace_scale: Optional[float] = None,
        compass_enabled: Optional[bool] = None,
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Register this robot with the HORUS backend system

        Args:
            dataviz: DataViz instance (creates one if not provided)
            keep_alive: If True, keep the connection dashboard running
            show_dashboard: If False, skip the dashboard UI during registration
            workspace_scale: Optional global workspace position scale.
            compass_enabled: Optional workspace-scoped Compass availability toggle.

        Returns:
            Tuple of (success, registration_data)
        """
        # Create DataViz if not provided
        if dataviz is None:
            dataviz = self.create_dataviz()

        # Create registry client and register
        registry = _get_registry_client()
        success, result = _invoke_register_robot(
            registry,
            self,
            dataviz,
            keep_alive=keep_alive,
            show_dashboard=show_dashboard,
            workspace_scale=workspace_scale,
            compass_enabled=compass_enabled,
            wait_for_app_before_register=True,
        )

        if success:
            # Store registration data
            self.add_metadata("horus_robot_id", result.get("robot_id"))
            self.add_metadata("horus_color", result.get("assigned_color") or result.get("color"))
            self.add_metadata("horus_registered", True)

            # Store topics for reference (rosout monitor will track actual subscriptions)
            topics = []
            for viz in dataviz.get_enabled_visualizations():
                topic = viz.data_source.topic
                if topic and topic not in topics:
                    topics.append(topic)
            self.add_metadata("horus_topics", topics)

            if topics:
                try:
                    from ..utils.topic_status import get_topic_status_board

                    board = get_topic_status_board()
                    for topic in topics:
                        board.on_subscribe(topic)
                except Exception:
                    pass

                # Hand topics to the ROS graph monitor so it can reconcile real state
                try:
                    from ..utils.topic_monitor import get_topic_monitor
                    monitor = get_topic_monitor()
                    monitor.watch_topics(topics)
                    monitor.start()
                except Exception:
                    pass

        return success, result

    def unregister_from_horus(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Unregister this robot from the HORUS backend system

        Returns:
            Tuple of (success, result_data)
        """
        robot_id = self.get_metadata("horus_robot_id")
        if not robot_id:
            return False, {"error": "Robot not registered with HORUS"}

        topics = self.get_metadata("horus_topics") or []

        # Create registry client and unregister
        registry = _get_registry_client()
        success, result = registry.unregister_robot(robot_id)

        if success:
            if topics:
                try:
                    from ..utils.topic_status import get_topic_status_board

                    board = get_topic_status_board()
                    for topic in topics:
                        board.on_unsubscribe(topic)
                except Exception:
                    pass

                try:
                    from ..utils.topic_monitor import get_topic_monitor

                    monitor = get_topic_monitor()
                    monitor.unwatch_topics(topics, emit_unsubscribed=False)
                except Exception:
                    pass

            # Clear registration metadata
            self.metadata.pop("horus_robot_id", None)
            self.metadata.pop("horus_color", None)
            self.metadata.pop("horus_registered", None)
            self.metadata.pop("horus_topics", None)

        return success, result

    def is_registered_with_horus(self) -> bool:
        """Check if robot is registered with HORUS system"""
        return bool(self.get_metadata("horus_registered", False))

    def get_horus_id(self) -> Optional[str]:
        """Get HORUS-assigned robot ID"""
        result = self.get_metadata("horus_robot_id")
        return result if isinstance(result, str) else None

    def get_horus_color(self) -> Optional[str]:
        """Get HORUS-assigned color"""
        result = self.get_metadata("horus_color")
        return result if isinstance(result, str) else None


def register_robots(
    robots,
    keep_alive: bool = True,
    show_dashboard: bool = True,
    timeout_sec: float = 10.0,
    workspace_scale: Optional[float] = None,
    compass_enabled: Optional[bool] = None,
    wait_for_app_before_register: bool = True,
    datavizs=None,
):
    """Register multiple robots using a shared registry session."""
    registry = _get_registry_client()
    return _invoke_register_robots(
        registry,
        robots,
        datavizs=datavizs,
        timeout_sec=timeout_sec,
        keep_alive=keep_alive,
        show_dashboard=show_dashboard,
        workspace_scale=workspace_scale,
        compass_enabled=compass_enabled,
        wait_for_app_before_register=wait_for_app_before_register,
    )


def _get_registry_client():
    robot_registry_module = importlib.import_module("horus.bridge.robot_registry")

    get_client = getattr(robot_registry_module, "get_robot_registry_client", None)
    if callable(get_client):
        return get_client()

    registry_cls = getattr(robot_registry_module, "RobotRegistryClient", None)
    if registry_cls is None:
        raise ImportError("No RobotRegistryClient available in horus.bridge.robot_registry")

    return registry_cls()


def _invoke_register_robot(
    registry,
    robot: "Robot",
    dataviz: "DataViz",
    keep_alive: bool,
    show_dashboard: bool,
    workspace_scale: Optional[float],
    compass_enabled: Optional[bool],
    wait_for_app_before_register: bool,
):
    method = registry.register_robot
    kwargs: Dict[str, Any] = {}

    try:
        parameters = inspect.signature(method).parameters
    except (TypeError, ValueError):
        parameters = {}

    if "keep_alive" in parameters:
        kwargs["keep_alive"] = keep_alive
    if "show_dashboard" in parameters:
        kwargs["show_dashboard"] = show_dashboard
    if workspace_scale is not None and "workspace_scale" in parameters:
        kwargs["workspace_scale"] = workspace_scale
    if compass_enabled is not None and "compass_enabled" in parameters:
        kwargs["compass_enabled"] = compass_enabled
    if "wait_for_app_before_register" in parameters:
        kwargs["wait_for_app_before_register"] = wait_for_app_before_register

    return method(robot, dataviz, **kwargs)


def _invoke_register_robots(
    registry,
    robots,
    datavizs,
    timeout_sec: float,
    keep_alive: bool,
    show_dashboard: bool,
    workspace_scale: Optional[float],
    compass_enabled: Optional[bool],
    wait_for_app_before_register: bool,
):
    method = registry.register_robots
    kwargs: Dict[str, Any] = {}

    try:
        parameters = inspect.signature(method).parameters
    except (TypeError, ValueError):
        parameters = {}

    if datavizs is not None and "datavizs" in parameters:
        kwargs["datavizs"] = datavizs
    if "timeout_sec" in parameters:
        kwargs["timeout_sec"] = timeout_sec
    if "keep_alive" in parameters:
        kwargs["keep_alive"] = keep_alive
    if "show_dashboard" in parameters:
        kwargs["show_dashboard"] = show_dashboard
    if workspace_scale is not None and "workspace_scale" in parameters:
        kwargs["workspace_scale"] = workspace_scale
    if compass_enabled is not None and "compass_enabled" in parameters:
        kwargs["compass_enabled"] = compass_enabled
    if "wait_for_app_before_register" in parameters:
        kwargs["wait_for_app_before_register"] = wait_for_app_before_register

    return method(robots, **kwargs)

