#!/usr/bin/env python3
"""Register a small HORUS MR ground-robot fleet.

Pair this with the legacy fake runtime while developing locally:
    python3 python/examples/legacy/fake_tf_ops_suite.py

If running from source instead of an installed SDK, keep the ROS paths:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/ops_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

robots = []
datavizs = []

for name in ("atlas", "nova", "orion"):
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
    robot.configure_navigation_tasks(
        goal_topic=f"/{name}/goal_pose",
        cancel_topic=f"/{name}/goal_cancel",
        goal_status_topic=f"/{name}/goal_status",
        waypoint_path_topic=f"/{name}/waypoint_path",
        waypoint_status_topic=f"/{name}/waypoint_status",
        frame_id="map",
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
