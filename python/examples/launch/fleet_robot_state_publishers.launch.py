#!/usr/bin/env python3
"""Bring up a heterogeneous 4-robot fleet as a normal ROS 2 robot_description workflow.

For each robot this launches, under its own namespace:
  * robot_state_publisher  -- loads the URDF (expanding xacro when needed), publishes the
                              latched ``/<ns>/robot_description`` topic + the
                              ``robot_description`` parameter, and broadcasts ``/tf`` with a
                              per-robot ``frame_prefix`` so frames are ``<ns>/<link>``.
  * joint_state_publisher  -- publishes default joint states so every movable link gets TF.
  * static_transform_publisher -- anchors ``world -> <ns>/<root_link>`` at an x-offset so the
                              robots are spaced apart (handy in RViz; HORUS anchors per robot).

Fleet (heterogeneous): jackal (wheeled), go1 + anymal_c (legged), h1 (humanoid).

Run (ROS 2 sourced):
    ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py

Fetch the URDFs + meshes first (once):
    python3 python/examples/tools/fetch_robot_description_assets.py

Then register the fleet with HORUS MR in another terminal:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

ASSETS_DIR = Path(__file__).resolve().parents[1] / ".local_assets" / "robot_descriptions"

# (namespace, urdf basename, xacro fallback basename, root link, x-offset metres)
FLEET = [
    ("jackal", "jackal.urdf", "jackal.urdf.xacro", "base_link", 0.0),
    ("go1", "go1.urdf", None, "base", 1.5),
    ("anymal_c", "anymal_c.urdf", None, "base", 3.0),
    ("h1", "h1.urdf", None, "pelvis", 4.5),
]


def _load_robot_description(urdf_name: str, xacro_name) -> str:
    """Return URDF XML, preferring an expanded .urdf and expanding xacro if needed."""
    urdf_path = ASSETS_DIR / urdf_name
    if urdf_path.is_file():
        return urdf_path.read_text(encoding="utf-8")

    if xacro_name:
        xacro_path = ASSETS_DIR / xacro_name
        if xacro_path.is_file():
            import xacro  # provided by the ROS 2 environment

            return xacro.process_file(str(xacro_path)).toxml()

    raise FileNotFoundError(
        f"No URDF for {urdf_name} in {ASSETS_DIR}. Run "
        "python3 python/examples/tools/fetch_robot_description_assets.py first."
    )


def generate_launch_description() -> LaunchDescription:
    actions = []
    for namespace, urdf_name, xacro_name, root_link, x_offset in FLEET:
        robot_description = _load_robot_description(urdf_name, xacro_name)
        frame_prefix = f"{namespace}/"

        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": frame_prefix,
                        # Latch the description so late SDK subscribers receive it immediately.
                        "publish_frequency": 30.0,
                    }
                ],
            )
        )
        actions.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                namespace=namespace,
                name="joint_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            )
        )
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{namespace}_world_anchor",
                output="log",
                arguments=[
                    "--x", str(x_offset),
                    "--y", "0",
                    "--z", "0",
                    "--frame-id", "world",
                    "--child-frame-id", f"{frame_prefix}{root_link}",
                ],
            )
        )

    return LaunchDescription(actions)
