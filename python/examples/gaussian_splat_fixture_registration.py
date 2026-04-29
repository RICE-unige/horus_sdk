#!/usr/bin/env python3
"""Register fake robots plus the Gaussian Splat fixture preview with HORUS.

Pair this with:
    python3 python/examples/tools/publish_gaussian_splat_fixture.py

The publisher emits matching TF/odom for splat_rover_1 and splat_rover_2. Current
HORUS builds render the sampled PointCloud2 preview, while the publisher also
emits /horus/gaussian_splat/manifest for the future native splat renderer.
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots


ROBOT_SPECS = [
    ("splat_rover_1", RobotDimensions(length=0.70, width=0.48, height=0.30)),
    ("splat_rover_2", RobotDimensions(length=0.70, width=0.48, height=0.30)),
]


robots = []
datavizs = []

for name, dimensions in ROBOT_SPECS:
    robot = Robot(
        name=name,
        robot_type=RobotType.WHEELED,
        dimensions=dimensions,
    )
    robot.configure_ros_binding(base_frame="base_link")
    robot.configure_robot_manager(enabled=True, status=True, data_viz=True, teleop=False, tasks=False)

    dataviz = robot.create_dataviz()
    robot.add_navigation_safety_to_dataviz(
        dataviz,
        odom_topic=f"/{name}/odom",
        include_velocity=True,
        include_trail=True,
        include_collision=False,
    )

    robots.append(robot)
    datavizs.append(dataviz)

world_layers = datavizs[0]
world_layers.add_3d_map(
    "/map_gaussian_splat_preview",
    frame_id="map",
    render_options={
        "point_size": 0.035,
        "max_points_per_frame": 0,
        "base_sample_stride": 1,
        "render_all_points": True,
        "auto_point_size_by_workspace_scale": True,
        "min_point_size": 0.0015,
        "max_point_size": 0.012,
        "point_shape": "circle",
        "enable_view_frustum_culling": False,
        "enable_subpixel_culling": False,
        "min_screen_radius_px": 0.0,
        "visible_points_budget": 120000,
        "max_visible_points_budget": 180000,
        "map_static_mode": True,
        "render_mode": "opaque_fast",
        "color": "#6ED7FF",
    },
)

success, result = register_robots(
    robots,
    datavizs=datavizs,
    workspace_scale=0.1,
    compass_enabled=False,
    keep_alive=True,
)

if not success:
    raise SystemExit(f"HORUS registration failed: {result}")
