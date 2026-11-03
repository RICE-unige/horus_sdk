#!/usr/bin/env python3
"""Display existing Nav2 local/global costmaps without registering a fake robot.

Both costmap nodes need always_send_full_costmap=true and publish_frequency>0.
Run in the same ROS domain as Nav2. This script does not command the robot.
"""

from horus.robot import WorkspaceVisualization, is_registration_cancelled, register_entities


def main():
    maps = WorkspaceVisualization("navigation_costmaps")
    viz = maps.create_dataviz()
    viz.add_costmap("/global_costmap/costmap", frame_id="map", height_offset=0.008)
    viz.add_costmap("/local_costmap/costmap", frame_id="map", height_offset=0.012)
    success, result = register_entities(
        [maps], datavizs=[viz], workspace_scale=0.1, compass_enabled=False, keep_alive=True,
    )
    if not success and not is_registration_cancelled(result):
        raise SystemExit(f"Costmap registration failed: {result}")


if __name__ == "__main__":
    main()
