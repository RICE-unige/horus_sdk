"""
Topic mapping and management system for HORUS SDK

This module provides ROS topic discovery, mapping, and namespace management
to enable seamless communication between robots and the Quest 3 application.
"""

import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Set, Union

from .event_bus import EventPriority, get_event_bus


class TopicType(Enum):
    """ROS topic message types relevant to HORUS"""
    
    # Sensor data topics
    IMAGE = "sensor_msgs/Image"
    POINT_CLOUD = "sensor_msgs/PointCloud2"
    LASER_SCAN = "sensor_msgs/LaserScan"
    IMU = "sensor_msgs/Imu"
    GPS = "sensor_msgs/NavSatFix"
    
    # Navigation topics
    ODOMETRY = "nav_msgs/Odometry"
    PATH = "nav_msgs/Path"
    OCCUPANCY_GRID = "nav_msgs/OccupancyGrid"
    GOAL = "geometry_msgs/PoseStamped"
    
    # Robot control topics
    TWIST = "geometry_msgs/Twist"
    JOINT_STATE = "sensor_msgs/JointState"
    
    # Transform topics
    TF = "tf2_msgs/TFMessage"
    TF_STATIC = "tf2_msgs/TFMessage"
    
    # HORUS-specific topics
    ROBOT_STATUS = "horus_interfaces/RobotStatus"
    ROBOT_COMMAND = "horus_interfaces/RobotCommand"
    ROBOT_CONFIG = "horus_interfaces/RobotConfig"
    
    # Generic/Unknown
    UNKNOWN = "unknown"


class TopicDirection(Enum):
    """Topic data direction"""
    PUBLISH = "publish"
    SUBSCRIBE = "subscribe"
    BIDIRECTIONAL = "bidirectional"


@dataclass
class TopicInfo:
    """
    Information about a ROS topic
    
    Args:
        name: Full topic name (e.g., "/robot1/camera/image_raw")
        topic_type: ROS message type
        direction: Data flow direction
        robot_namespace: Robot namespace if topic belongs to a robot
        sensor_name: Sensor name if topic is sensor data
        frame_id: TF frame ID associated with topic
        qos_profile: ROS QoS profile information
        last_seen: Timestamp when topic was last discovered
        active: Whether topic is currently active
        metadata: Additional topic information
    """
    name: str
    topic_type: TopicType = TopicType.UNKNOWN
    direction: TopicDirection = TopicDirection.PUBLISH
    robot_namespace: Optional[str] = None
    sensor_name: Optional[str] = None
    frame_id: Optional[str] = None
    qos_profile: Optional[Dict[str, Any]] = None
    last_seen: float = field(default_factory=time.time)
    active: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        """Validate and process topic information"""
        if not self.name:
            raise ValueError("Topic name cannot be empty")
        
        # Extract robot namespace from topic name
        if self.robot_namespace is None:
            self.robot_namespace = self._extract_robot_namespace()
        
        # Extract sensor name from topic name
        if self.sensor_name is None:
            self.sensor_name = self._extract_sensor_name()
    
    def _extract_robot_namespace(self) -> Optional[str]:
        """Extract robot namespace from topic name"""
        parts = self.name.strip('/').split('/')
        if len(parts) >= 1:
            # Common patterns: /robot1/... or /carter/... or /tb3_0/...
            first_part = parts[0]
            if any(keyword in first_part.lower() for keyword in ['robot', 'carter', 'tb3', 'turtlebot', 'husky', 'jackal']):
                return first_part
        return None
    
    def _extract_sensor_name(self) -> Optional[str]:
        """Extract sensor name from topic name"""
        parts = self.name.strip('/').split('/')
        
        # Common sensor topic patterns
        sensor_keywords = ['camera', 'lidar', 'laser', 'scan', 'imu', 'gps', 'depth']
        
        for part in parts:
            if any(keyword in part.lower() for keyword in sensor_keywords):
                return part
        
        return None
    
    def is_sensor_topic(self) -> bool:
        """Check if this is a sensor data topic"""
        sensor_types = {TopicType.IMAGE, TopicType.POINT_CLOUD, TopicType.LASER_SCAN, 
                       TopicType.IMU, TopicType.GPS}
        return self.topic_type in sensor_types
    
    def is_robot_topic(self) -> bool:
        """Check if this topic belongs to a specific robot"""
        return self.robot_namespace is not None
    
    def get_horus_event_topic(self) -> str:
        """Convert ROS topic to HORUS event bus topic"""
        if self.robot_namespace:
            if self.sensor_name:
                return f"{self.robot_namespace}.sensor.{self.sensor_name}"
            elif self.topic_type == TopicType.ROBOT_STATUS:
                return f"{self.robot_namespace}.status"
            elif self.topic_type == TopicType.ODOMETRY:
                return f"{self.robot_namespace}.odometry"
            elif self.topic_type == TopicType.PATH:
                return f"{self.robot_namespace}.path"
            else:
                return f"{self.robot_namespace}.data"
        else:
            # Global topics
            if self.topic_type == TopicType.TF:
                return "global.tf"
            elif self.topic_type == TopicType.OCCUPANCY_GRID:
                return "global.map"
            else:
                return "global.data"


class TopicMap:
    """
    ROS topic discovery and mapping system for HORUS SDK
    
    Manages discovery, classification, and mapping of ROS topics to enable
    seamless integration between robots and the Quest 3 application.
    """
    
    def __init__(self, auto_discovery: bool = True):
        self._topics: Dict[str, TopicInfo] = {}
        self._robot_namespaces: Set[str] = set()
        self._lock = threading.RLock()
        self._auto_discovery = auto_discovery
        self._discovery_thread: Optional[threading.Thread] = None
        self._running = False
        self._event_bus = get_event_bus()
        
        # Topic type mapping for common ROS message types
        self._type_mapping = {
            "sensor_msgs/msg/Image": TopicType.IMAGE,
            "sensor_msgs/Image": TopicType.IMAGE,
            "sensor_msgs/msg/PointCloud2": TopicType.POINT_CLOUD,
            "sensor_msgs/PointCloud2": TopicType.POINT_CLOUD,
            "sensor_msgs/msg/LaserScan": TopicType.LASER_SCAN,
            "sensor_msgs/LaserScan": TopicType.LASER_SCAN,
            "sensor_msgs/msg/Imu": TopicType.IMU,
            "sensor_msgs/Imu": TopicType.IMU,
            "nav_msgs/msg/Odometry": TopicType.ODOMETRY,
            "nav_msgs/Odometry": TopicType.ODOMETRY,
            "nav_msgs/msg/Path": TopicType.PATH,
            "nav_msgs/Path": TopicType.PATH,
            "geometry_msgs/msg/Twist": TopicType.TWIST,
            "geometry_msgs/Twist": TopicType.TWIST,
            "tf2_msgs/msg/TFMessage": TopicType.TF,
            "tf2_msgs/TFMessage": TopicType.TF,
        }
    
    def start(self):
        """Start topic discovery if auto-discovery is enabled"""
        if not self._auto_discovery or self._running:
            return
            
        self._running = True
        self._discovery_thread = threading.Thread(target=self._discovery_loop, daemon=True)
        self._discovery_thread.start()
        
    def stop(self):
        """Stop topic discovery"""
        self._running = False
        if self._discovery_thread:
            self._discovery_thread.join(timeout=1.0)
    
    def _discovery_loop(self):
        """Main topic discovery loop"""
        while self._running:
            try:
                self._discover_topics()
                time.sleep(2.0)  # Discover every 2 seconds
            except Exception as e:
                print(f"TopicMap discovery error: {e}")
                time.sleep(5.0)  # Wait longer on error
    
    def _discover_topics(self):
        """Discover active ROS topics (stub - needs ROS2 integration)"""
        # This would integrate with ROS2 to discover actual topics
        # For now, we'll provide the framework structure
        pass
    
    def add_topic(self, name: str, topic_type: Union[str, TopicType], direction: TopicDirection = TopicDirection.PUBLISH, **kwargs) -> TopicInfo:
        """
        Add a topic to the map
        
        Args:
            name: Topic name (e.g., "/robot1/camera/image_raw")
            topic_type: ROS message type or TopicType enum
            direction: Data flow direction
            **kwargs: Additional TopicInfo parameters
            
        Returns:
            TopicInfo object for the added topic
        """
        if isinstance(topic_type, str):
            topic_type = self._type_mapping.get(topic_type, TopicType.UNKNOWN)
        
        topic_info = TopicInfo(name=name, topic_type=topic_type, direction=direction, **kwargs)
        
        with self._lock:
            self._topics[name] = topic_info
            
            # Track robot namespaces
            if topic_info.robot_namespace:
                self._robot_namespaces.add(topic_info.robot_namespace)
        
        # Publish topic discovery event
        self._event_bus.publish(
            "topicmap.topic.discovered",
            {
                "topic_name": name,
                "topic_type": topic_type.value,
                "robot_namespace": topic_info.robot_namespace,
                "sensor_name": topic_info.sensor_name
            },
            priority=EventPriority.NORMAL,
            source="TopicMap"
        )
        
        return topic_info
    
    def remove_topic(self, name: str) -> bool:
        """
        Remove a topic from the map
        
        Args:
            name: Topic name to remove
            
        Returns:
            True if topic was found and removed
        """
        with self._lock:
            if name in self._topics:
                topic_info = self._topics.pop(name)
                
                # Publish topic removal event
                self._event_bus.publish(
                    "topicmap.topic.removed",
                    {"topic_name": name, "robot_namespace": topic_info.robot_namespace},
                    priority=EventPriority.NORMAL,
                    source="TopicMap"
                )
                
                return True
        return False
    
    def get_topic(self, name: str) -> Optional[TopicInfo]:
        """Get topic information by name"""
        with self._lock:
            return self._topics.get(name)
    
    def get_topics_by_robot(self, robot_namespace: str) -> List[TopicInfo]:
        """Get all topics for a specific robot"""
        with self._lock:
            return [topic for topic in self._topics.values() 
                   if topic.robot_namespace == robot_namespace]
    
    def get_topics_by_type(self, topic_type: TopicType) -> List[TopicInfo]:
        """Get all topics of a specific type"""
        with self._lock:
            return [topic for topic in self._topics.values() 
                   if topic.topic_type == topic_type]
    
    def get_sensor_topics(self, robot_namespace: Optional[str] = None) -> List[TopicInfo]:
        """Get all sensor topics, optionally filtered by robot"""
        with self._lock:
            topics = [topic for topic in self._topics.values() if topic.is_sensor_topic()]
            if robot_namespace:
                topics = [topic for topic in topics if topic.robot_namespace == robot_namespace]
            return topics
    
    def get_robot_namespaces(self) -> Set[str]:
        """Get all discovered robot namespaces"""
        with self._lock:
            return self._robot_namespaces.copy()
    
    def get_all_topics(self) -> List[TopicInfo]:
        """Get all discovered topics"""
        with self._lock:
            return list(self._topics.values())
    
    def update_topic_activity(self, name: str, active: bool):
        """Update topic activity status"""
        with self._lock:
            if name in self._topics:
                self._topics[name].active = active
                self._topics[name].last_seen = time.time()
                
                # Publish activity update event
                self._event_bus.publish(
                    "topicmap.topic.activity",
                    {"topic_name": name, "active": active},
                    priority=EventPriority.LOW,
                    source="TopicMap"
                )
    
    def create_topic_mapping(self, robot_name: str) -> Dict[str, str]:
        """
        Create topic mapping for a robot to use in HORUS registration
        
        Args:
            robot_name: Robot namespace
            
        Returns:
            Dictionary mapping sensor names to ROS topics
        """
        mapping = {}
        robot_topics = self.get_topics_by_robot(robot_name)
        
        for topic in robot_topics:
            if topic.is_sensor_topic() and topic.sensor_name:
                mapping[topic.sensor_name] = topic.name
        
        return mapping
    
    def get_stats(self) -> Dict[str, Any]:
        """Get topic map statistics"""
        with self._lock:
            return {
                "total_topics": len(self._topics),
                "active_topics": sum(1 for t in self._topics.values() if t.active),
                "robot_count": len(self._robot_namespaces),
                "sensor_topics": len([t for t in self._topics.values() if t.is_sensor_topic()]),
                "topic_types": {tt.value: len([t for t in self._topics.values() if t.topic_type == tt]) 
                               for tt in TopicType}
            }


# Global topic map instance
_global_topic_map: Optional[TopicMap] = None


def get_topic_map() -> TopicMap:
    """Get the global topic map instance"""
    global _global_topic_map
    if _global_topic_map is None:
        _global_topic_map = TopicMap()
        _global_topic_map.start()
    return _global_topic_map
