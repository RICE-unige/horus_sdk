#!/usr/bin/env python3
"""Register synthetic HORUS experiment robots and visualizations."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

SDK_ROOT = Path(__file__).resolve().parents[3]
PYTHON_ROOT = SDK_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from horus.dataviz import DataViz
from horus.experiments.workloads import WorkloadConfig, load_workload_config
from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera, Lidar3D


def camera_is_compressed(workload: WorkloadConfig) -> bool:
    encoding = str(workload.camera.encoding or "").strip().lower()
    return encoding in {
        "compressed",
        "compressed_image",
        "jpeg",
        "jpg",
        "ros_compressed",
        "sensor_msgs/compressedimage",
        "sensor_msgs/msg/compressedimage",
    }


def camera_uses_webrtc(workload: WorkloadConfig) -> bool:
    transport = str(workload.transport or "").strip().lower()
    encoding = str(workload.camera.encoding or "").strip().lower()
    return "webrtc" in transport or encoding in {"h264", "webrtc"}


def robot_pointcloud_enabled(workload: WorkloadConfig) -> bool:
    value = workload.extra.get("robot_pointcloud", True)
    return value is not False and str(value).strip().lower() not in {"0", "false", "no", "off"}


def navigation_enabled(workload: WorkloadConfig) -> bool:
    extra = dict(getattr(workload, "extra", {}) or {})
    nested = extra.get("extra")
    if isinstance(nested, dict):
        extra.update(nested)
    value = extra.get("navigation", False)
    return value is True or str(value).strip().lower() in {"1", "true", "yes", "on"}


def camera_topic(robot_name: str, camera_index: int, workload: WorkloadConfig) -> str:
    base = f"/{robot_name}/camera_{camera_index}/image_raw"
    if camera_uses_webrtc(workload):
        return f"/{robot_name}/camera_{camera_index}/webrtc/image_raw"
    if camera_is_compressed(workload):
        return f"{base}/compressed"
    return base


def build_camera(robot_name: str, camera_index: int, workload: WorkloadConfig) -> Camera:
    width, height = parse_resolution(workload.camera.resolution)
    compressed = camera_is_compressed(workload)
    use_webrtc = camera_uses_webrtc(workload)
    topic = camera_topic(robot_name, camera_index, workload)
    image_type = "compressed" if compressed else "raw"
    transport = "webrtc" if use_webrtc else "ros"

    camera = Camera(
        name=f"camera_{camera_index}",
        frame_id=f"{robot_name}/camera_{camera_index}_link",
        topic=topic,
        resolution=(width, height),
        fps=max(1, int(round(workload.camera.fps or 15))),
        encoding="jpeg" if compressed else "rgb8",
        streaming_type=transport,
        minimap_streaming_type="ros",
        teleop_streaming_type=transport,
        startup_mode="minimap",
        minimap_topic=topic,
        minimap_image_type=image_type,
        teleop_topic=topic,
        teleop_image_type=image_type,
    )
    camera.add_metadata("image_type", image_type)
    camera.add_metadata("experiment_stream", True)
    camera.add_metadata("transport_profile", workload.transport)
    if hasattr(camera, "configure_projected_view"):
        camera.configure_projected_view(
            image_scale=1.0,
            focal_length_scale=0.55,
            show_frustum=True,
            frustum_color="#A7D8FFA0",
        )
    if hasattr(camera, "configure_minimap_view"):
        camera.configure_minimap_view(
            size=8.0,
            position_offset=(0.0, 1.6 + 0.18 * camera_index, 0.0),
            face_camera=True,
            rotation_offset=(90.0, 0.0, 0.0),
        )
    return camera


def parse_resolution(value: str) -> tuple[int, int]:
    parts = str(value or "").lower().split("x", 1)
    if len(parts) != 2:
        return 640, 480
    try:
        return max(1, int(parts[0])), max(1, int(parts[1]))
    except ValueError:
        return 640, 480


def robot_type_for_index(index: int, workload: WorkloadConfig) -> RobotType:
    profile = str(workload.robot_profile or "").lower()
    if "uav" in profile or "drone" in profile or "aerial" in profile:
        return RobotType.DRONE
    if "legged" in profile:
        return RobotType.LEGGED
    if "mixed" in profile:
        return [RobotType.WHEELED, RobotType.LEGGED, RobotType.DRONE, RobotType.WHEELED][index % 4]
    return RobotType.WHEELED


def dimensions_for(robot_type: RobotType) -> RobotDimensions:
    if robot_type == RobotType.DRONE:
        return RobotDimensions(length=0.46, width=0.46, height=0.18)
    if robot_type == RobotType.LEGGED:
        return RobotDimensions(length=0.70, width=0.34, height=0.46)
    return RobotDimensions(length=0.80, width=0.55, height=0.42)


def cameras_for_robot(robot_index: int, workload: WorkloadConfig) -> list[int]:
    streams = max(0, int(workload.camera.streams or 0))
    robots = max(1, int(workload.robot_count or 1))
    return [stream // robots for stream in range(streams) if stream % robots == robot_index]


def build_robot(robot_index: int, workload: WorkloadConfig) -> tuple[Robot, DataViz]:
    name = f"exp_robot_{robot_index}"
    robot_type = robot_type_for_index(robot_index, workload)
    robot = Robot(name=name, robot_type=robot_type, dimensions=dimensions_for(robot_type))
    if max(1, int(workload.robot_count or 1)) == 1:
        robot.configure_ros_binding(
            tf_mode="flat",
            topic_mode="flat",
            base_frame=f"{name}/base_link",
        )
    else:
        robot.configure_ros_binding(
            tf_mode="prefixed",
            topic_mode="prefixed",
            base_frame="base_link",
            tf_prefix=name,
            topic_prefix=name,
        )
    robot.configure_workspace_experiment(enabled=True)
    enable_navigation = navigation_enabled(workload)
    robot.configure_robot_manager(
        teleop=enable_navigation,
        tasks=enable_navigation,
    )
    if enable_navigation:
        robot.configure_teleop(
            command_topic=f"/{name}/cmd_vel",
            robot_profile="drone" if robot_type == RobotType.DRONE else robot_type.value,
            publish_rate_hz=30.0,
            linear_xy_max_mps=1.0,
            linear_z_max_mps=0.5 if robot_type == RobotType.DRONE else 0.0,
            angular_z_max_rps=1.0,
        )
        robot.configure_navigation_tasks(
            goal_topic=f"/{name}/goal_pose",
            cancel_topic=f"/{name}/goal_cancel",
            goal_status_topic=f"/{name}/goal_status",
            waypoint_path_topic=f"/{name}/global_path",
            waypoint_status_topic=f"/{name}/goal_status",
            frame_id="map",
            position_tolerance_m=0.25,
            yaw_tolerance_deg=12.0,
            min_altitude_m=0.2 if robot_type == RobotType.DRONE else None,
            max_altitude_m=8.0 if robot_type == RobotType.DRONE else None,
        )

    for camera_index in cameras_for_robot(robot_index, workload):
        robot.add_sensor(build_camera(name, camera_index, workload))

    if robot_pointcloud_enabled(workload) and workload.pointcloud.points > 0 and workload.pointcloud.hz > 0:
        robot.add_sensor(
            Lidar3D(
                name="lidar_3d",
                frame_id=f"{name}/lidar_link",
                topic=f"/{name}/points",
                points_per_second=int(workload.pointcloud.points * workload.pointcloud.hz),
            )
        )

    dataviz = robot.create_dataviz(f"{name}_viz")
    if enable_navigation:
        robot.add_path_planning_to_dataviz(
            dataviz,
            global_path_topic=f"/{name}/global_path",
            local_path_topic=f"/{name}/local_path",
            trajectory_topic=f"/{name}/trajectory",
        )
    if enable_navigation and hasattr(robot, "add_navigation_safety_to_dataviz"):
        robot.add_navigation_safety_to_dataviz(
            dataviz,
            odom_topic=f"/{name}/odom",
            collision_risk_topic=f"/{name}/collision_risk",
        )
    return robot, dataviz


def attach_global_layers(dataviz: DataViz, workload: WorkloadConfig) -> None:
    dataviz.add_tf_tree("/tf", frame_id="map")
    representation = str(workload.map.representation or "none").lower()
    if representation != "none" and workload.map.triangles > 0:
        dataviz.add_3d_mesh(
            "/horus/experiment/map_mesh",
            frame_id="map",
            render_options={
                "max_triangles": max(1000, int(workload.map.triangles)),
                "chunk_max_triangles": max(1000, math.ceil(max(1, workload.map.triangles) / max(1, workload.map.chunks or 1))),
                "use_vertex_colors": True,
                "alpha": 0.95,
                "double_sided": True,
                "transport": "marker",
                "source_coordinate_space": "enu",
                "color": "#59B8D1",
            },
        )
    if "pointcloud" in representation:
        dataviz.add_3d_map(
            "/horus/experiment/map_points",
            frame_id="map",
            render_options={
                "point_size": 0.018,
                "max_points_per_frame": 0,
                "render_all_points": True,
                "auto_point_size_by_workspace_scale": True,
                "min_point_size": 0.002,
                "max_point_size": 0.018,
                "point_shape": "circle",
                "enable_view_frustum_culling": False,
                "enable_subpixel_culling": False,
                "color": "#7DD6FF",
            },
        )


def build_registration(workload: WorkloadConfig) -> tuple[list[Robot], list[DataViz]]:
    robots: list[Robot] = []
    datavizs: list[DataViz] = []
    for robot_index in range(max(1, int(workload.robot_count or 1))):
        robot, dataviz = build_robot(robot_index, workload)
        robots.append(robot)
        datavizs.append(dataviz)
    attach_global_layers(datavizs[0], workload)
    return robots, datavizs


def registration_preview(robots: list[Robot], datavizs: list[DataViz], workload: WorkloadConfig) -> dict[str, Any]:
    representation = str(workload.map.representation or "none").lower()
    robot_pointcloud = (
        robot_pointcloud_enabled(workload)
        and int(workload.pointcloud.points or 0) > 0
        and float(workload.pointcloud.hz or 0.0) > 0.0
    )
    return {
        "experiment": workload.experiment,
        "condition": workload.condition,
        "robots": [robot.name for robot in robots],
        "robot_count": len(robots),
        "visualization_count": sum(len(dataviz.visualizations) for dataviz in datavizs),
        "camera_streams": int(workload.camera.streams or 0),
        "robot_pointcloud_enabled": robot_pointcloud,
        "pointcloud_points_per_robot": int(workload.pointcloud.points or 0) if robot_pointcloud else 0,
        "map_pointcloud_points": int(workload.pointcloud.points or 0) if "pointcloud" in representation else 0,
        "map_representation": workload.map.representation,
        "map_triangles": int(workload.map.triangles or 0),
        "navigation_enabled": navigation_enabled(workload),
        "topics": {
            "tf": "/tf",
            "map_mesh": "/horus/experiment/map_mesh",
            "map_points": "/horus/experiment/map_points",
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--no-wait-for-app", action="store_true")
    parser.add_argument("--workspace-scale", type=float, default=0.1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    workload = load_workload_config(Path(args.config))
    robots, datavizs = build_registration(workload)
    if args.dry_run:
        print(json.dumps(registration_preview(robots, datavizs, workload), indent=2, sort_keys=True))
        return 0
    success, result = register_robots(
        robots,
        datavizs=datavizs,
        timeout_sec=args.timeout,
        keep_alive=not args.once,
        show_dashboard=not args.quiet,
        workspace_scale=args.workspace_scale,
        compass_enabled=False,
        wait_for_app_before_register=not args.no_wait_for_app,
    )
    if not success:
        raise SystemExit(f"HORUS experiment registration failed: {result}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
