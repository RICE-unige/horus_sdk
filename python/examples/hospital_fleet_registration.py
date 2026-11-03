#!/usr/bin/env python3
"""Register the local, physics-driven hospital demo. Start with ./hospital-demo."""

import os
import json
import math
from pathlib import Path

from horus.robot import Robot, RobotDimensions, RobotType, WorkspaceVisualization
from horus.robot import is_registration_cancelled, register_entities
from horus.sensors import Camera, LaserScan

ROBOTS = ("carter1", "carter2", "carter3")
CAMERA_RESOLUTION = (640, 360)
CAMERA_HORIZONTAL_FOV_DEG = 82.26
PROJECTED_IMAGE_HEIGHT = 0.11
# Keep the plane centered on the optical axis with the calibrated field of view.
PROJECTED_DISTANCE = (
    PROJECTED_IMAGE_HEIGHT * CAMERA_RESOLUTION[0] / CAMERA_RESOLUTION[1]
    / (2 * math.tan(math.radians(CAMERA_HORIZONTAL_FOV_DEG / 2)))
)


def build_fleet(*, remote_media=False, relay_media=False,
                client_signal_topic=None, server_signal_topic=None):
    model_root = Path(os.environ.get("HORUS_HOSPITAL_MODEL_ROOT",
        "~/.cache/horus/robot_description_sources/nova_carter")).expanduser()
    urdf = Path(os.environ.get("HORUS_HOSPITAL_URDF", str(
        model_root / "nova_carter_description/urdf/horus_nova_carter_static_shell.urdf")))
    if not urdf.is_file():
        raise FileNotFoundError(f"Carter description missing: {urdf}. Run ./hospital-demo check.")
    entities, views = [], []
    for name in ROBOTS:
        robot = Robot(name=name, robot_type=RobotType.WHEELED,
                      dimensions=RobotDimensions(length=0.82, width=0.60, height=0.72))
        robot.configure_ros_binding(base_frame="base_link")
        robot.configure_robot_description(source="local", urdf_path=str(urdf), mesh_root=str(model_root),
            robot_description_topic=f"/{name}/robot_description", base_frame="base_link",
            include_visual_meshes=True, body_mesh_mode="runtime_high_mesh", visual_mesh_triangle_budget=500000,
            visual_link_pose_source="tf")
        robot.configure_robot_manager()
        robot.configure_teleop(command_topic=f"/{name}/teleop_cmd_vel", robot_profile="wheeled",
            linear_xy_max_mps=0.6, angular_z_max_rps=1.2, publish_rate_hz=30,
            invert_angular_z=True)  # Stick right must produce clockwise (negative ROS yaw).
        robot.configure_navigation_tasks(goal_topic=f"/{name}/horus/goal_pose", cancel_topic=f"/{name}/horus/goal_cancel",
            goal_status_topic=f"/{name}/horus/goal_status", waypoint_path_topic=f"/{name}/horus/waypoint_path",
            waypoint_status_topic=f"/{name}/horus/waypoint_status", frame_id="global_odom",
            position_tolerance_m=0.20, yaw_tolerance_deg=12.0)
        camera = Camera(name="front_camera", frame_id=f"{name}/camera_optical",
            topic=f"/{name}/camera/color/image_raw/compressed", resolution=CAMERA_RESOLUTION, fps=15,
            encoding="jpeg", fov=CAMERA_HORIZONTAL_FOV_DEG, streaming_type="ros", minimap_streaming_type="ros",
            teleop_streaming_type="webrtc", minimap_image_type="compressed", teleop_image_type="compressed",
            startup_mode="minimap", minimap_max_fps=15, metadata={"image_type": "compressed"})
        camera.configure_projected_view(position_offset=(0, 0, 0), rotation_offset=(0, 0, 0),
            image_scale=PROJECTED_IMAGE_HEIGHT, focal_length_scale=PROJECTED_DISTANCE, show_frustum=True)
        camera.configure_minimap_view(size=1.0, position_offset=(0, 0.9, 0), face_camera=True)
        camera.configure_webrtc_transport(bitrate_kbps=1500, framerate=15,
            minimap_bitrate_kbps=1500, minimap_framerate=15,
            teleop_bitrate_kbps=3000, teleop_framerate=30)
        if remote_media or relay_media:
            # Native relay aliases forward encoded RTP, without an operator encoder.
            camera.minimap_topic = camera.teleop_topic = (
                f"/horus/remote_media/{name}/h264" if relay_media else f"/{name}/camera/color/image_raw")
            camera.minimap_image_type = camera.teleop_image_type = "compressed" if relay_media else "raw"
            camera.minimap_streaming_type = camera.teleop_streaming_type = "webrtc"
            prefix = "/horus/webrtc" if relay_media else "/horus/remote/webrtc"
            camera.configure_webrtc_transport(client_signal_topic=client_signal_topic or prefix + "/client_signal",
                server_signal_topic=server_signal_topic or prefix + "/server_signal", bitrate_kbps=1500, framerate=15,
                minimap_bitrate_kbps=1500, minimap_framerate=15,
                teleop_bitrate_kbps=3000, teleop_framerate=30)
        scan = LaserScan(name="laser", frame_id=f"{name}/laser", topic=f"/{name}/scan",
            min_angle=-3.14159265, max_angle=3.12413936, angle_increment=0.017453293,
            min_range=0.1, max_range=20.0, point_size=0.015)
        robot.add_sensor(camera)
        robot.add_sensor(scan)
        view = robot.create_dataviz()
        view.add_robot_velocity_data(name, f"/{name}/odom", frame_id=f"{name}/odom")
        view.add_robot_odometry_trail(name, f"/{name}/odom", frame_id=f"{name}/odom")
        view.add_robot_global_path(name, f"/{name}/plan_global", frame_id="global_odom",
                                   render_options={"color": "#48C8F0", "line_width": 2})
        entities.append(robot)
        views.append(view)
    maps = WorkspaceVisualization("hospital_shared_maps")
    view = maps.create_dataviz()
    view.add_occupancy_grid("/shared_map", frame_id="map", render_options={"alpha": 0.7})
    options = {}
    run = os.environ.get("HORUS_HOSPITAL_RUN")
    profile = Path(run) / "map-render-profile.json" if run else None
    if profile is not None and profile.is_file():
        options["display"] = json.loads(profile.read_text())
    view.add_nvblox_mesh("/horus/hospital/mesh", frame_id="global_odom", render_options=options)
    entities.append(maps)
    views.append(view)
    return entities, views


def main():
    media_mode = os.environ.get("HORUS_HOSPITAL_REMOTE_MEDIA", "0").lower()
    entities, views = build_fleet(
        remote_media=media_mode in {"1", "direct"}, relay_media=media_mode == "relay",
        client_signal_topic=os.environ.get("HORUS_REMOTE_MEDIA_CLIENT_SIGNAL_TOPIC"),
        server_signal_topic=os.environ.get("HORUS_REMOTE_MEDIA_SERVER_SIGNAL_TOPIC"))
    success, result = register_entities(entities, datavizs=views, workspace_scale=0.1,
        compass_enabled=False, keep_alive=True, wait_for_app_before_register=False)
    if not success and not is_registration_cancelled(result):
        raise SystemExit(f"Hospital registration failed: {result}")


if __name__ == "__main__":
    main()
