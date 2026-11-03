"""Explicit, backwards-compatible native mesh presentation options."""

import math


def mesh_display_options(value):
    if not isinstance(value, dict):
        raise ValueError("mesh display must be a dictionary")
    allowed = {"lod_enabled", "force_full_detail", "lod_error_pixels", "lod_memory_mib", "spacewarp_enabled"}
    if set(value) - allowed:
        raise ValueError(f"Unknown mesh display options: {sorted(set(value) - allowed)}")
    result = {"lod_enabled": False, "force_full_detail": False,
              "lod_error_pixels": 1.0, "lod_memory_mib": 96, "spacewarp_enabled": False}
    result.update(value)
    for key in ("lod_enabled", "force_full_detail", "spacewarp_enabled"):
        if not isinstance(result[key], bool):
            raise ValueError(f"{key} must be a boolean")
    error = result["lod_error_pixels"]
    if isinstance(error, bool) or not isinstance(error, (float, int)) or not math.isfinite(error) or not .25 <= error <= 4:
        raise ValueError("lod_error_pixels must be finite and between 0.25 and 4")
    memory = result["lod_memory_mib"]
    if isinstance(memory, bool) or not isinstance(memory, int) or not 16 <= memory <= 192:
        raise ValueError("lod_memory_mib must be an integer between 16 and 192")
    return result
