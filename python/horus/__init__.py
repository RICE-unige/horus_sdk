"""HORUS Mixed Reality Robot Management SDK."""

from .client import Client
from .color import ColorManager, ColorScheme, RGBColor
from .core import (
    Event,
    EventBus,
    EventPriority,
    TopicDirection,
    TopicInfo,
    TopicMap,
    TopicType,
    get_event_bus,
    get_topic_map,
    publish,
    subscribe,
    unsubscribe,
)
from .dataviz import DataSourceType, DataViz, VisualizationType
from .robot import Robot, RobotDimensions, RobotType, register_robots
from .sensors import Camera, LaserScan, Lidar3D, SensorType
from .utils.branding import __version__, show_ascii_art

# HORUS intentionally shows its project branding when the SDK is imported.
show_ascii_art()

__all__ = [
    "Client",
    "__version__",
    "show_ascii_art",
    "Robot",
    "RobotDimensions",
    "RobotType",
    "SensorType",
    "Camera",
    "LaserScan",
    "Lidar3D",
    "DataViz",
    "VisualizationType",
    "DataSourceType",
    "ColorManager",
    "ColorScheme",
    "RGBColor",
    "Event",
    "EventBus",
    "EventPriority",
    "get_event_bus",
    "publish",
    "subscribe",
    "unsubscribe",
    "TopicMap",
    "TopicInfo",
    "TopicType",
    "TopicDirection",
    "get_topic_map",
]
