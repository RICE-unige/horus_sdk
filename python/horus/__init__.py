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
from .robot import (
    EntityCapabilities,
    FieldTeammate,
    FieldTeammateConfig,
    Robot,
    RobotDimensions,
    RobotType,
    is_registration_cancelled,
    register_robots,
)
from .sensors import Camera, LaserScan, Lidar3D, SensorType
import os

from .utils.branding import __version__, show_ascii_art

# HORUS intentionally shows its project branding when the SDK is imported.
if os.getenv("HORUS_SDK_NO_BANNER", "").strip().lower() not in {"1", "true", "yes", "on"}:
    show_ascii_art()

__all__ = [
    "Client",
    "__version__",
    "show_ascii_art",
    "Robot",
    "FieldTeammate",
    "EntityCapabilities",
    "FieldTeammateConfig",
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
    "is_registration_cancelled",
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
