#!/usr/bin/env python3
"""Register robots with mono minimap cameras and stereo teleop camera streams.

Pair this with:
    python3 python/examples/legacy/fake_stereo_camera_multi.py

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/stereo_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

robots = []
datavizs = []

for index in range(1, 5):
    name = f"stereo_bot_{index}"
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

    eye_resolution = (1920, 1080) if index == 1 else (960, 540)
    camera = Camera(
        name="front_stereo_camera",
        frame_id=f"{name}/camera_link",
        topic=f"/{name}/camera/minimap/image_raw/compressed",
        is_stereo=True,
        resolution=(eye_resolution[0] * 2, eye_resolution[1]),
        fps=90 if index == 1 else 10,
        encoding="jpeg",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        minimap_topic=f"/{name}/camera/minimap/image_raw/compressed",
        minimap_image_type="compressed",
        minimap_max_fps=30,
        teleop_topic=f"/{name}/camera/teleop/image_raw/compressed",
        teleop_image_type="compressed",
        teleop_stereo_layout="side_by_side",
        stereo_layout="side_by_side",
        startup_mode="minimap",
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
