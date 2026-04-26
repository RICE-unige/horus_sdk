"""Compatibility helpers for legacy HORUS SDK import paths."""

from __future__ import annotations

import warnings


def warn_deprecated_module(old_module: str, replacement: str | None = None) -> None:
    """Emit one standard deprecation warning for legacy module imports."""
    message = f"{old_module} is a deprecated compatibility import path."
    if replacement:
        message += f" Use {replacement} instead."
    warnings.warn(message, DeprecationWarning, stacklevel=2)
