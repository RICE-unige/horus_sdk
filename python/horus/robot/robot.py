"""
Robot object system for HORUS SDK
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

if TYPE_CHECKING:
    from ..dataviz import DataViz
    from ..sensors import SensorInstance, SensorType


class RobotType(Enum):
    """Robot type classifications"""

    WHEELED = "wheeled"
    LEGGED = "legged"
    AERIAL = "aerial"


@dataclass
class Robot:
    """
    Base robot object with type and identification

    Args:
        name: Robot identifier/namespace (unique)
        robot_type: Classification of robot (wheeled, legged, aerial)
        metadata: Optional additional robot information
        sensors: List of sensors attached to this robot
    """

    name: str
    robot_type: RobotType
    metadata: Optional[Dict[str, Any]] = None
    sensors: List["SensorInstance"] = field(default_factory=list)

    def __post_init__(self):
        """Validate robot configuration after initialization"""
        if not self.name:
            raise ValueError("Robot name cannot be empty")

        if not isinstance(self.robot_type, RobotType):
            raise TypeError("robot_type must be a RobotType enum")

        # Initialize metadata if not provided
        if self.metadata is None:
            self.metadata = {}

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

        # Add robot transform visualization
        dataviz.add_robot_transform(
            robot_name=self.name,
            topic=f"/{self.name}/tf",
            frame_id=f"{self.name}_base_link",
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
        self, dataviz: Optional["DataViz"] = None
    ) -> Tuple[bool, Dict[str, Any]]:
        """
        Register this robot with the HORUS backend system

        Args:
            dataviz: DataViz instance (creates one if not provided)

        Returns:
            Tuple of (success, registration_data)
        """
        from ..bridge.robot_registry import RobotRegistryClient

        # Create DataViz if not provided
        if dataviz is None:
            dataviz = self.create_dataviz()

        # Create registry client and register
        registry = RobotRegistryClient()
        success, result = registry.register_robot(self, dataviz)

        if success:
            # Store registration data
            self.add_metadata("horus_robot_id", result.get("robot_id"))
            self.add_metadata("horus_color", result.get("assigned_color"))
            self.add_metadata("horus_registered", True)

        return success, result

    def unregister_from_horus(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Unregister this robot from the HORUS backend system

        Returns:
            Tuple of (success, result_data)
        """
        from ..bridge.robot_registry import RobotRegistryClient

        robot_id = self.get_metadata("horus_robot_id")
        if not robot_id:
            return False, {"error": "Robot not registered with HORUS"}

        # Create registry client and unregister
        registry = RobotRegistryClient()
        success, result = registry.unregister_robot(robot_id)

        if success:
            # Clear registration data
            self.add_metadata("horus_robot_id", None)
            self.add_metadata("horus_color", None)
            self.add_metadata("horus_registered", False)

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
