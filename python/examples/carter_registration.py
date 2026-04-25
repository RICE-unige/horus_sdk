#!/usr/bin/env python3
"""Register a live NVIDIA Carter fleet with HORUS MR.

Expected live topics:
    /tf
    /tf_static
    /shared_map
    /<robot>/cmd_vel
    /<robot>/chassis/odom
    /<robot>/front_2d_lidar/scan
    /<robot>/front_stereo_camera/left/image_raw/compressed
    /rviz/<robot>/plan
    /rviz/<robot>/local_plan

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/carter_registration.py
"""

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera, LaserScan

ROBOT_NAMES = ("carter1", "carter2", "carter3")


def build_carter(robot_name: str):
    robot = Robot(
        name=robot_name,
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(length=0.82, width=0.56, height=0.60),
    )
    robot.configure_local_body_model("nova_carter")
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic=f"/{robot_name}/cmd_vel",
        robot_profile="wheeled",
    )
    robot.configure_navigation_tasks(
        goal_topic=f"/{robot_name}/goal_pose",
        cancel_topic=f"/{robot_name}/goal_cancel",
        goal_status_topic=f"/{robot_name}/goal_status",
        waypoint_path_topic=f"/{robot_name}/waypoint_path",
        waypoint_status_topic=f"/{robot_name}/waypoint_status",
        frame_id="map",
        position_tolerance_m=0.20,
        yaw_tolerance_deg=12.0,
    )

    camera_frame = f"{robot_name}/front_stereo_camera_left_optical"
    camera = Camera(
        name="front_camera",
        frame_id=camera_frame,
        topic=f"/{robot_name}/front_stereo_camera/left/image_raw/compressed",
        resolution=(1280, 720),
        fps=20,
        encoding="jpeg",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        startup_mode="minimap",
        minimap_image_type="compressed",
        teleop_image_type="compressed",
    )
    camera.configure_projected_view(
        position_offset=(0.04, 0.28, 0.0),
        rotation_offset=(0.0, 0.0, 0.0),
        image_scale=1.037,
        focal_length_scale=0.55,
        show_frustum=True,
        frustum_color="#E6E6E0A0",
    )
    camera.configure_webrtc_transport(
        bitrate_kbps=2000,
        framerate=20,
        stun_server_url="stun:stun.l.google.com:19302",
    )
    camera.configure_minimap_view(
        size=10.0,
        position_offset=(0.0, 2.0, 0.0),
        face_camera=True,
        rotation_offset=(90.0, 0.0, 0.0),
    )
    robot.add_sensor(camera)

    scan = LaserScan(
        name="front_2d_lidar",
        frame_id=f"{robot_name}/front_2d_lidar",
        topic=f"/{robot_name}/front_2d_lidar/scan",
        min_angle=-3.14159,
        max_angle=3.14159,
        angle_increment=0.005,
        min_range=0.1,
        max_range=30.0,
        point_size=0.025,
    )
    robot.add_sensor(scan)

    dataviz = robot.create_dataviz()
    dataviz.add_sensor_visualization(camera, robot_name=robot_name, enabled=True)
    dataviz.add_sensor_visualization(scan, robot_name=robot_name, enabled=True)
    dataviz.add_robot_transform(
        robot_name=robot_name,
        topic="/tf",
        frame_id=f"{robot_name}/base_link",
    )
    dataviz.add_robot_velocity_data(
        robot_name=robot_name,
        topic=f"/{robot_name}/chassis/odom",
        frame_id="global_odom",
    )
    dataviz.add_robot_odometry_trail(
        robot_name=robot_name,
        topic=f"/{robot_name}/chassis/odom",
        frame_id="global_odom",
    )
    dataviz.add_robot_global_path(
        robot_name=robot_name,
        topic=f"/rviz/{robot_name}/plan",
        frame_id="global_odom",
    )
    dataviz.add_robot_local_path(
        robot_name=robot_name,
        topic=f"/rviz/{robot_name}/local_plan",
        frame_id="global_odom",
    )
    return robot, dataviz


robots, datavizs = zip(*(build_carter(name) for name in ROBOT_NAMES))
datavizs[0].add_occupancy_grid(
    topic="/shared_map",
    frame_id="map",
    render_options={"color_free": "#222222", "color_occupied": "#F4F4F4", "alpha": 0.85},
)

success, result = register_robots(
    list(robots),
    datavizs=list(datavizs),
    workspace_scale=0.1,
    keep_alive=True,
    wait_for_app_before_register=False,
)

if not success:
    raise SystemExit(f"HORUS registration failed: {result}")
