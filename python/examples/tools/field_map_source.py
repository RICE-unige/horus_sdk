"""Keep Field registration and the PC viewer on the same reconstruction."""

from __future__ import annotations


def resolve_map_source(estimated: bool, requested: str | None = None) -> str:
    source = requested or ("estimated" if estimated else "measured")
    if source not in ("measured", "estimated"):
        raise ValueError("Map source must be measured or estimated")
    if source == "estimated" and not estimated:
        raise ValueError("The estimated map source requires --estimated-depth")
    return source
