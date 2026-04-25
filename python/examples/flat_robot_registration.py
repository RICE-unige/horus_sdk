#!/usr/bin/env python3
"""Register one robot that uses un-namespaced ROS topics and flat TF frames.

Pair this with:
    python3 python/examples/legacy/fake_tf_single_flat.py

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/flat_robot_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

robot = Robot(
    name="robot",
    robot_type=RobotType.WHEELED,
    dimensions=RobotDimensions(length=0.75, width=0.5, height=0.35),
)
robot.configure_ros_binding(tf_mode="flat", topic_mode="flat", base_frame="base_link")
robot.configure_robot_manager()
robot.configure_teleop(
    command_topic="/cmd_vel",
    robot_profile="wheeled",
)
robot.configure_navigation_tasks(
    goal_topic="/goal_pose",
    cancel_topic="/goal_cancel",
    goal_status_topic="/goal_status",
    waypoint_path_topic="/waypoint_path",
    waypoint_status_topic="/waypoint_status",
    frame_id="map",
)

camera = Camera(
    name="front_camera",
    frame_id="camera_link",
    topic="/camera/image_raw",
    resolution=(160, 90),
    fps=6,
    encoding="rgb8",
    streaming_type="ros",
    minimap_streaming_type="ros",
    teleop_streaming_type="ros",
    startup_mode="minimap",
    minimap_image_type="raw",
    teleop_image_type="raw",
)
camera.add_metadata("image_type", "raw")
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
robot.add_path_planning_to_dataviz(dataviz, global_path_topic="/global_path", local_path_topic="/local_path")
robot.add_navigation_safety_to_dataviz(dataviz, odom_topic="/odom", collision_risk_topic="/collision_risk")

success, result = register_robots(
    [robot],
    datavizs=[dataviz],
    workspace_scale=0.1,
    compass_enabled=False,
    keep_alive=True,
)

if not success:
    raise SystemExit(f"HORUS registration failed: {result}")
