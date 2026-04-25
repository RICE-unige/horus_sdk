#!/usr/bin/env python3
"""Register a live Unitree Go1 in HORUS MR.

Expected live ROS graph:
    /tf with unitree_go1/base, unitree_go1/trunk, unitree_go1/camera_face
    /tf_static with unitree_go1/laser and unitree_go1/camera_optical_left_face
    /unitree_go1/cmd_vel
    /unitree_go1/scan
    /unitree_go1/front_camera/left/color/image_rect/compressed

This uses the real Go1 URDF and visual meshes from:
    /home/omotoye/Unitree_ros2_to_real/ros2_ws/src/go1_description

Run the support relay in a separate terminal. It publishes the missing front
camera optical TF and also handles HORUS MR Stand Up, Sit Down, and collision
alert visualization:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/unitree_go1_high_mode_relay.py

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/unitree_go1_registration.py
"""

from pathlib import Path

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera, LaserScan

ROBOT_NAME = "unitree_go1"
GO1_DESCRIPTION_ROOT = Path("/home/omotoye/Unitree_ros2_to_real/ros2_ws/src/go1_description")
GO1_URDF = GO1_DESCRIPTION_ROOT / "urdf" / "go1.urdf"

if not GO1_URDF.exists():
    raise SystemExit(f"Missing Go1 URDF: {GO1_URDF}")
if not (GO1_DESCRIPTION_ROOT / "meshes" / "trunk.dae").exists():
    raise SystemExit(f"Missing Go1 visual meshes under: {GO1_DESCRIPTION_ROOT / 'meshes'}")

robot = Robot(
    name=ROBOT_NAME,
    robot_type=RobotType.LEGGED,
    dimensions=RobotDimensions(length=0.54, width=0.30, height=0.18),
)
robot.configure_ros_binding(base_frame="trunk")
robot.configure_robot_description(
    urdf_path=str(GO1_URDF),
    base_frame="trunk",
    source="ros",
    include_visual_meshes=True,
    visual_mesh_triangle_budget=220000,
    body_mesh_mode="runtime_high_mesh",
)
robot.configure_robot_manager()
robot.configure_teleop(
    command_topic=f"/{ROBOT_NAME}/cmd_vel",
    robot_profile="legged",
    linear_xy_max_mps=0.25,
    linear_z_max_mps=0.0,
    angular_z_max_rps=1.2,
)

front_camera = Camera(
    name="front_camera",
    frame_id=f"{ROBOT_NAME}/camera_optical_left_face",
    topic=f"/{ROBOT_NAME}/front_camera/left/color/image_rect/compressed",
    resolution=(928, 800),
    fps=30,
    fov=120.0,
    encoding="jpeg",
    streaming_type="ros",
    minimap_streaming_type="ros",
    teleop_streaming_type="ros",
    minimap_topic=f"/{ROBOT_NAME}/front_camera/left/color/image_rect/compressed",
    minimap_image_type="compressed",
    teleop_topic=f"/{ROBOT_NAME}/front_camera/left/color/image_rect/compressed",
    teleop_image_type="compressed",
)
front_camera.configure_projected_view(
    position_offset=(0.0, 0.0, 0.0),
    image_scale=0.0875,
    focal_length_scale=0.0875,
    show_frustum=True,
    frustum_color="#E6E6E0A0",
)
front_camera.configure_immersive_view(ros_flip_x=True, ros_flip_y=True)
front_camera.configure_minimap_view(size=1.0)
robot.add_sensor(front_camera)

front_scan = LaserScan(
    name="front_laser_scan",
    frame_id=f"{ROBOT_NAME}/laser",
    topic=f"/{ROBOT_NAME}/scan",
    min_angle=-2.3561945,
    max_angle=2.3561945,
    angle_increment=0.004363323,
    min_range=0.023,
    max_range=60.0,
    color="#6ED7FF",
    point_size=0.2,
)
robot.add_sensor(front_scan)

dataviz = robot.create_dataviz()
dataviz.add_sensor_visualization(
    front_scan,
    ROBOT_NAME,
    render_options={"color": "#6ED7FF", "point_size": 0.2, "alpha": 0.9},
)
dataviz.add_robot_collision_risk(
    robot_name=ROBOT_NAME,
    topic=f"/{ROBOT_NAME}/collision_risk",
    frame_id=f"{ROBOT_NAME}/trunk",
    render_options={
        "threshold_m": 1.2,
        "radius_m": 0.32,
        "source": "laser_scan",
        "alpha_min": 0.0,
        "alpha_max": 0.48,
        "color": "#FF6A00",
    },
)

success, result = register_robots(
    [robot],
    datavizs=[dataviz],
    workspace_scale=0.1,
    compass_enabled=False,
    keep_alive=True,
)

if not success:
    raise SystemExit(f"HORUS registration failed: {result}")
