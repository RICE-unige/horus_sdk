#!/usr/bin/env python3
"""Register a single robot using flat TF frames and root control topics."""

import argparse
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..", "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera


def build_parser():
    parser = argparse.ArgumentParser(
        description="Register a single robot with flat TF/topic binding for HORUS."
    )
    parser.add_argument("--robot-name", default="robot", help="HORUS logical/display name.")
    parser.add_argument("--base-frame", default="base_link", help="Flat ROS base frame.")
    parser.add_argument("--camera-frame", default="camera_link", help="Flat camera frame.")
    parser.add_argument("--camera-topic", default="/camera/image_raw")
    parser.add_argument("--workspace-scale", type=float, default=None)
    parser.add_argument("--keep-alive", dest="keep_alive", action="store_true", default=True)
    parser.add_argument("--no-keep-alive", dest="keep_alive", action="store_false")
    parser.add_argument("--length", type=float, default=0.8)
    parser.add_argument("--width", type=float, default=0.6)
    parser.add_argument("--height", type=float, default=0.4)
    return parser


def main():
    args = build_parser().parse_args()

    robot = Robot(
        name=args.robot_name,
        robot_type=RobotType.WHEELED,
        dimensions=RobotDimensions(
            length=float(args.length),
            width=float(args.width),
            height=float(args.height),
        ),
    )
    robot.configure_ros_binding(
        tf_mode="flat",
        topic_mode="flat",
        base_frame=args.base_frame,
    )

    camera = Camera(
        name="front_camera",
        frame_id=args.camera_frame,
        topic=args.camera_topic,
        resolution=(160, 90),
        fps=6,
        encoding="rgb8",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="webrtc",
        startup_mode="minimap",
    )
    robot.add_sensor(camera)

    dataviz = robot.create_full_dataviz(
        global_path_topic="/global_path",
        local_path_topic="/local_path",
    )
    robot.add_navigation_safety_to_dataviz(
        dataviz,
        odom_topic="/odom",
        collision_risk_topic="/collision_risk",
    )
    register_robots(
        [robot],
        datavizs=[dataviz],
        keep_alive=args.keep_alive,
        workspace_scale=args.workspace_scale,
    )


if __name__ == "__main__":
    main()
