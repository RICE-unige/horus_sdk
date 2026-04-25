#!/usr/bin/env python3
"""Register a ground fleet with global semantic perception boxes enabled by default.

Pair this with:
    python3 python/examples/legacy/fake_tf_ops_suite.py --robot-count 4

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/semantic_perception_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

robots = []
datavizs = []

for name in ("atlas", "nova", "orion", "luna"):
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
        minimap_image_type="compressed",
        teleop_image_type="compressed",
    )
    camera.configure_projected_view(image_scale=0.072, focal_length_scale=0.108)
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

semantic_layer = datavizs[0]
semantic_layer.add_semantic_box(
    "person_1",
    "person",
    center=(1.6, 0.8, 0.9),
    size=(0.45, 0.45, 1.8),
    render_options={"color": "#00C853", "confidence": 0.92},
)
semantic_layer.add_semantic_box(
    "person_2",
    "person",
    center=(-1.2, 1.4, 0.9),
    size=(0.45, 0.45, 1.8),
    render_options={"color": "#00C853", "confidence": 0.78},
)
semantic_layer.add_semantic_box(
    "equipment_1",
    "equipment",
    center=(0.3, -1.6, 0.4),
    size=(0.8, 0.6, 0.8),
    render_options={"color": "#FFD54F", "confidence": 0.85},
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
