"""Deprecated compatibility path for DataViz types."""

from .._compat import warn_deprecated_module
from ..dataviz import DataSourceType, DataViz, VisualizationType

warn_deprecated_module("horus.robot.dataviz", "horus.dataviz")

__all__ = ["DataViz", "VisualizationType", "DataSourceType"]
