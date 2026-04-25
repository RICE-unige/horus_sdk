#!/usr/bin/env python3
"""Register robots plus global occupancy, point-cloud, mesh, and octomap layers.

Pair this with the map publishers listed in the root README.

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/global_maps_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

robots = []
datavizs = []

for name in ("test_bot_1", "test_bot_2"):
    robot = Robot(
        name=name,
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.8, width=0.55, height=0.45),
    )
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic=f"/{name}/cmd_vel",
        robot_profile="wheeled",
    )

    camera = Camera(
        name="front_camera",
        frame_id=f"{name}/camera_link",
        topic=f"/{name}/camera/image_raw/compressed",
        resolution=(160, 90),
        fps=6,
        encoding="jpeg",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="ros",
        minimap_image_type="compressed",
        teleop_image_type="compressed",
    )
    camera.add_metadata("image_type", "compressed")
    camera.configure_projected_view(
        image_scale=1.0,
        focal_length_scale=0.55,
        show_frustum=True,
        frustum_color="#E6E6E0A0",
    )
    camera.configure_minimap_view(
        size=10.0,
        position_offset=(0.0, 2.0, 0.0),
        face_camera=True,
        rotation_offset=(90.0, 0.0, 0.0),
    )
    robot.add_sensor(camera)

    dataviz = robot.create_dataviz()
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic=f"/{name}/global_path",
        local_path_topic=f"/{name}/local_path",
    )
    robot.add_navigation_safety_to_dataviz(
        dataviz,
        odom_topic=f"/{name}/odom",
        collision_risk_topic=f"/{name}/collision_risk",
    )
    datavizs.append(dataviz)
    robots.append(robot)

world_layers = datavizs[0]
world_layers.add_occupancy_grid(
    "/map",
    frame_id="map",
    render_options={"color_free": "#222222", "color_occupied": "#F4F4F4", "alpha": 0.85},
)
world_layers.add_3d_map(
    "/map_3d",
    frame_id="map",
    render_options={"point_size": 0.04, "max_points_per_frame": 0, "render_all_points": True, "auto_point_size_by_workspace_scale": False, "enable_view_frustum_culling": False, "enable_subpixel_culling": False, "color": "#6ED7FF"},
)
world_layers.add_3d_mesh(
    "/map_3d_mesh",
    frame_id="map",
    render_options={"max_triangles": 60000, "use_vertex_colors": True},
)
world_layers.add_3d_octomap(
    "/map_3d_octomap_mesh",
    frame_id="map",
    render_options={"render_mode": "surface_mesh", "native_topic": "/map_3d_octomap"},
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
