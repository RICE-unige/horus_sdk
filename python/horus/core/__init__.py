"""
HORUS SDK Core Module

Pure logic components that are ROS-agnostic and unit-testable.
Contains low-level infrastructure like EventBus, TopicMap, and exceptions.
"""

from .event_bus import Event, EventBus, EventPriority, get_event_bus, publish, subscribe, unsubscribe
from .topic_map import TopicDirection, TopicInfo, TopicMap, TopicType, get_topic_map

# Import stub implementations for remaining components
# from .exceptions import HorusException

__all__ = [
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
    "get_topic_map"
    # Future components:
    # 'HorusException'
]
