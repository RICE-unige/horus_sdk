#!/usr/bin/env python3
"""Register any HMSH v2 map output; run its source adapter separately."""

import argparse

from horus.robot import WorkspaceVisualization, is_registration_cancelled, register_entities


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/horus/maps/live")
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args()
    resource = WorkspaceVisualization("live_map")
    dataviz = resource.create_dataviz("live_map")
    dataviz.add_live_mesh(topic=args.topic, frame_id=args.frame_id)
    success, result = register_entities([resource], datavizs=[dataviz],
        workspace_scale=0.1, compass_enabled=False, keep_alive=True)
    if not success and not is_registration_cancelled(result):
        raise SystemExit(f"Live map registration failed: {result}")


if __name__ == "__main__":
    main()
