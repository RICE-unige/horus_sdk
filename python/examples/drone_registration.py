#!/usr/bin/env python3
"""Register a small drone team with teleop, go-to, waypoint, takeoff, and land UI.

Pair this with:
    python3 python/examples/legacy/fake_tf_drone_ops_suite.py

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/drone_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

robots = []
datavizs = []

for name in ("drone_1", "drone_2", "drone_3"):
    robot = Robot(
        name=name,
        robot_type=RobotType.DRONE,
        dimensions=RobotDimensions(length=0.46, width=0.46, height=0.18),
    )
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic=f"/{name}/cmd_vel",
        robot_profile="drone",
    )
    robot.configure_navigation_tasks(
        goal_topic=f"/{name}/goal_pose",
        cancel_topic=f"/{name}/goal_cancel",
        goal_status_topic=f"/{name}/goal_status",
        waypoint_path_topic=f"/{name}/waypoint_path",
        waypoint_status_topic=f"/{name}/waypoint_status",
        frame_id="map",
        min_altitude_m=0.0,
        max_altitude_m=10.0,
    )

    camera = Camera(
        name="front_camera",
        frame_id=f"{name}/camera_link",
        topic=f"/{name}/camera/image_raw/compressed",
        resolution=(160, 90),
        fps=6,
        encoding="jpeg",
        minimap_image_type="compressed",
        teleop_image_type="compressed",
    )
    camera.configure_projected_view(image_scale=0.06, focal_length_scale=0.09)
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

    robots.append(robot)
    datavizs.append(dataviz)

success, result = register_robots(
    robots,
    datavizs=datavizs,
    workspace_scale=0.1,
    compass_enabled=False,
    keep_alive=True,
)

if not success:
    raise SystemExit(f"HORUS registration failed: {result}")
