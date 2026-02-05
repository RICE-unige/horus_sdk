#!/usr/bin/env python3
"""
Example of registering multiple robots with Horus using the Python SDK.
This demo sends only the robot transform (TF) using the robot name as the TF prefix
and includes robot dimensions for sizing the interactor surface.

Usage:
  python3 sdk_registration_demo.py
  python3 sdk_registration_demo.py --robot-count 4
  python3 sdk_registration_demo.py --robot-names test_bot_1,test_bot_2
"""

import sys
import os
import argparse

# Ensure we can import 'horus' package regardless of where script is run from
script_dir = os.path.dirname(os.path.abspath(__file__))
package_root = os.path.join(script_dir, "..")
if package_root not in sys.path:
    sys.path.insert(0, package_root)

try:
    from horus.robot import Robot, RobotDimensions, RobotType, register_robots
    from horus.utils import cli
except ImportError:
    # Fallback/Debug
    print(f"Failed to import horus from {package_root}")
    raise

def build_parser():
    parser = argparse.ArgumentParser(
        description="Register multiple robots with Horus for MR testing."
    )
    parser.add_argument(
        "--robot-name",
        default="test_bot",
        help="Robot name (single) or prefix (multi).",
    )
    parser.add_argument(
        "--robot-names",
        default="",
        help="Comma-separated list of robot names (overrides --robot-name/--robot-count).",
    )
    parser.add_argument("--robot-count", type=int, default=3, help="Number of robots")
    parser.add_argument(
        "--keep-alive",
        dest="keep_alive",
        action="store_true",
        default=True,
        help="Keep dashboard open after registration (default: on).",
    )
    parser.add_argument(
        "--no-keep-alive",
        dest="keep_alive",
        action="store_false",
        help="Exit after registering robots.",
    )
    parser.add_argument("--length", type=float, default=0.8, help="Robot length in meters")
    parser.add_argument("--width", type=float, default=0.6, help="Robot width in meters")
    parser.add_argument("--height", type=float, default=0.4, help="Robot height in meters")
    return parser


def resolve_robot_names(args):
    if args.robot_names:
        names = [name.strip() for name in args.robot_names.split(",") if name.strip()]
        if names:
            return names

    if args.robot_count <= 1:
        return [args.robot_name]

    return [f"{args.robot_name}_{idx + 1}" for idx in range(args.robot_count)]

def main():
    args = build_parser().parse_args()
    robot_names = resolve_robot_names(args)

    cli.print_step("Defining Robot Configuration...")
    robots = []
    for idx, name in enumerate(robot_names):
        length = args.length + (0.1 * idx)
        width = args.width + (0.05 * idx)
        height = args.height + (0.03 * idx)
        robots.append(
            Robot(
                # Name should match the TF prefix (e.g. test_bot_1/base_link)
                name=name,
                robot_type=RobotType.WHEELED,
                dimensions=RobotDimensions(
                    length=length,
                    width=width,
                    height=height,
                ),
            )
        )

    cli.print_step(f"Registering {len(robots)} robot(s)...")
    success, result = register_robots(
        robots,
        keep_alive=args.keep_alive,
        show_dashboard=True,
    )
    if not success:
        if isinstance(result, dict) and result.get("error") == "Cancelled":
            cli.print_info("Registration cancelled by user.")
            return
        cli.print_error(f"Registration failed: {result}")
        return


if __name__ == "__main__":
    main()
