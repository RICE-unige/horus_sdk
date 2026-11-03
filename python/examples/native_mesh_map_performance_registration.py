#!/usr/bin/env python3
"""Register an isolated native Quest mesh-map performance scene.

This intentionally registers one tiny TF-only anchor and the global mesh layer.
Robot models, cameras, sensors, and remote rendering are omitted so Quest metrics
measure the native mesh path instead of unrelated showcase content.
"""

import argparse

from horus.robot import (
    Robot,
    RobotDimensions,
    RobotType,
    is_registration_cancelled,
    register_robots,
)


def parse_float(value: str) -> float:
    return float(str(value).strip().replace(",", "."))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-scale", type=parse_float, default=0.1)
    parser.add_argument("--mesh-max-triangles", type=int, default=2_000_000)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    anchor = Robot(
        name="native_map_anchor",
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.08, width=0.08, height=0.08),
    )
    anchor.configure_ros_binding(
        tf_mode="flat",
        topic_mode="flat",
        base_frame="native_map_anchor",
    )

    dataviz = anchor.create_dataviz()
    dataviz.add_3d_mesh(
        "/map_3d_mesh",
        frame_id="map",
        render_options={
            "max_triangles": int(args.mesh_max_triangles),
            "use_vertex_colors": True,
            "double_sided": False,
            "source_coordinate_space": "enu",
            "use_indexed_binary": True,
            "static_mesh": True,
            "max_chunk_renderers": 128,
            "max_chunk_uploads_per_frame": 2,
            "max_triangle_uploads_per_frame": 20_000,
            "static_finalize_delay_seconds": 0.75,
            "enable_perf_diagnostics": True,
        },
    )

    success, result = register_robots(
        [anchor],
        datavizs=[dataviz],
        workspace_scale=float(args.workspace_scale),
        compass_enabled=False,
        keep_alive=True,
    )
    if success:
        return 0
    if is_registration_cancelled(result):
        print("HORUS registration monitor stopped.")
        return 0
    raise SystemExit(f"HORUS registration failed: {result}")


if __name__ == "__main__":
    raise SystemExit(main())
