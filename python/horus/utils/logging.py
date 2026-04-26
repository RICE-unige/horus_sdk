"""Standard logging compatibility utilities for HORUS SDK."""

from __future__ import annotations

import logging

from .._compat import warn_deprecated_module

warn_deprecated_module("horus.utils.logging", "the standard logging module")


def get_logger(name: str = "horus") -> logging.Logger:
    """Return a standard Python logger for SDK integrations."""
    return logging.getLogger(name)


__all__ = ["get_logger"]
