"""Deprecated compatibility path for topic-map types."""

from ._compat import warn_deprecated_module
from .core import TopicDirection, TopicInfo, TopicMap, TopicType, get_topic_map

warn_deprecated_module("horus.topics", "horus.core")

__all__ = [
    "TopicMap",
    "TopicInfo",
    "TopicType",
    "TopicDirection",
    "get_topic_map",
]
