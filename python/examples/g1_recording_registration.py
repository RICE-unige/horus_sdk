#!/usr/bin/env python3
"""Observation-only G1: articulated body, RGB, LiDAR, walked path and nvblox."""

import argparse
import json
import math
from pathlib import Path

ROBOT = "g1_recorded"
TOPIC = "/horus/g1_recording"
CAMERA_FRAME = ROBOT + "/rgb_optical"
URDF = (Path.home() / "g1_operations/runtime/workspaces/ros/install/"
        "unitree_g1_description/share/unitree_g1_description/urdf/"
        "g1_29dof_with_hand_rev_1_0_fixed.urdf")


def make_registration(camera_info, urdf=URDF):
    from horus.robot import Robot, RobotDimensions, RobotType
    from horus.sensors import Camera, Lidar3D

    robot = Robot(ROBOT, RobotType.LEGGED,
                  dimensions=RobotDimensions(length=0.55, width=0.55, height=1.32))
    # The upright selection box follows a presentation anchor. The actual body
    # follows the recorded pelvis pose, with all recorded joints applied by RSP.
    robot.configure_ros_binding(tf_prefix=ROBOT, topic_mode="flat", base_frame="floor_anchor")
    robot.configure_robot_description(
        source="local", urdf_path=str(urdf), mesh_root=str(urdf.parent.parent.parent),
        base_frame="pelvis", body_mesh_mode="runtime_high_mesh",
        visual_link_pose_source="tf", visual_mesh_triangle_budget=2_000_000,
        chunk_size_bytes=64000,
    )
    robot.configure_capabilities(controllable=False, teleoperable=False, taskable=False,
                                 guidable=False, observable=True, communicative=False)
    robot.configure_robot_manager(status=True, data_viz=True, teleop=False, tasks=False)
    robot.configure_teleop(enabled=False)
    robot.configure_navigation_tasks(go_to_point_enabled=False, waypoint_enabled=False)
    robot.add_metadata("data_mode", "recorded_physical_robot")
    robot.add_metadata("source_session", camera_info["session"])

    width, height = camera_info["width"], camera_info["height"]
    camera_topic = TOPIC + "/camera/compressed"
    camera = Camera(
        "Recorded RGB", CAMERA_FRAME, camera_topic, resolution=(width, height), fps=30,
        fov=math.degrees(2 * math.atan(width / (2 * camera_info["k"][0]))),
        encoding="jpeg", streaming_type="ros", minimap_streaming_type="ros",
        teleop_streaming_type="ros", minimap_topic=camera_topic,
        minimap_image_type="compressed", teleop_topic=camera_topic,
        teleop_image_type="compressed", minimap_max_fps=30,
    )
    camera.add_metadata("camera_info_topic", TOPIC + "/camera/info")
    camera.add_metadata("image_type", "compressed")
    camera.configure_projected_view(image_scale=0.07,
        focal_length_scale=0.07 * camera_info["k"][4] / height, show_frustum=True)
    camera.configure_minimap_view(size=0.4)
    robot.add_sensor(camera)
    lidar = Lidar3D("Recorded LiDAR scan", "map", TOPIC + "/scan", min_range=0.0,
                    max_range=100.0, points_per_second=100000)
    robot.add_sensor(lidar)
    viz = robot.create_dataviz()
    viz.add_sensor_visualization(camera, robot.name)
    viz.add_sensor_visualization(lidar, robot.name,
        render_options={"color": "#63D5E6", "point_size": 0.012,
                        "color_mode": "rgb_height", "height_min_m": -1.0, "height_max_m": 2.0,
                        "enable_subpixel_culling": False, "map_static_mode": False})
    viz.add_robot_transform(robot.name, "/tf", frame_id="floor_anchor")
    viz.add_robot_local_path(robot.name, TOPIC + "/walked_path", frame_id="map",
                            render_options={"color": "#FFC247", "line_width": 2})
    if camera_info.get("occupancy"):
        viz.add_occupancy_grid(TOPIC + "/occupancy", frame_id="map", transport_lane="bulk_replaceable")
    viz.add_nvblox_mesh(TOPIC + "/mesh",
                       render_options={"static_mesh": camera_info.get("map_mode") == "static"})
    return robot, viz


def main():
    from horus.robot import register_robots, is_registration_cancelled
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-info", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, default=URDF)
    args = parser.parse_args()
    robot, viz = make_registration(json.loads(args.camera_info.read_text()), args.urdf)
    ok, result = register_robots([robot], datavizs=[viz], workspace_scale=0.10,
                                compass_enabled=False, keep_alive=True)
    return 0 if ok or is_registration_cancelled(result) else 1


if __name__ == "__main__":
    raise SystemExit(main())
