#!/usr/bin/env python3
"""Register the SDK costmap adapters' outputs; no robot commands or fake data."""

from horus.robot import WorkspaceVisualization, is_registration_cancelled, register_entities


def main():
    maps = WorkspaceVisualization("navigation_costmaps")
    viz = maps.create_dataviz()
    viz.add_costmap("/horus/costmaps/global", frame_id="map", height_offset=0.008)
    viz.add_costmap("/horus/costmaps/local", frame_id="map", height_offset=0.012)
    success, result = register_entities(
        [maps], datavizs=[viz], workspace_scale=0.1, compass_enabled=False, keep_alive=True,
    )
    if not success and not is_registration_cancelled(result):
        raise SystemExit(f"Costmap registration failed: {result}")


if __name__ == "__main__":
    main()
