#!/usr/bin/env python3
"""Bring up a heterogeneous 5-robot showroom as a normal ROS 2 workflow.

By default this launches one static TF anchor per robot:
``world -> <ns>/<base>``. HORUS MR gets the detailed robot body from the
SDK-baked URDF payload, so publishing every URDF link frame is unnecessary for
the Quest showroom and can create hundreds of frame labels/axes.

Set ``publish_full_tf:=true`` to also launch one robot_state_publisher per robot.
That opt-in path publishes ``/<ns>/robot_description`` and the full prefixed TF tree
for RViz-style validation.

Fleet: Unitree G1/H1, ANYmal C, Boston Dynamics Spot, and Jackal.

Run (ROS 2 sourced):
    ros2 launch python/examples/launch/fleet_robot_state_publishers.launch.py

Fetch the URDFs + meshes first (once):
    python3 python/examples/tools/fetch_robot_description_assets.py

Then register the fleet with HORUS MR in another terminal:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/fleet_robot_description_registration.py
"""

from pathlib import Path
import sys

from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

EXAMPLES_DIR = Path(__file__).resolve().parents[1]
if str(EXAMPLES_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLES_DIR))

from robot_description_showroom_specs import SHOWROOM_FLEET

ASSETS_DIR = Path(__file__).resolve().parents[1] / ".local_assets" / "robot_descriptions"
SUPPORT_NODE = Path(__file__).resolve().parents[1] / "tools" / "showroom_tf_support_node.py"
FLEET = SHOWROOM_FLEET


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
    publish_full_tf = LaunchConfiguration("publish_full_tf")
    actions = [
        DeclareLaunchArgument(
            "publish_full_tf",
            default_value="false",
            description="Launch robot_state_publisher for every showroom robot and publish the full URDF TF tree.",
        ),
        ExecuteProcess(
            cmd=[sys.executable, str(SUPPORT_NODE)],
            name="showroom_tf_support",
            output="screen",
            condition=IfCondition(publish_full_tf),
        )
    ]
    for spec in FLEET:
        robot_description = _load_robot_description(spec.urdf_name, spec.xacro_name)
        frame_prefix = f"{spec.name}/"

        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{spec.name}_showroom_anchor",
                output="screen",
                arguments=[
                    "--x", f"{float(spec.x):.6f}",
                    "--y", f"{float(spec.y):.6f}",
                    "--z", f"{float(spec.z):.6f}",
                    "--qx", "0.0",
                    "--qy", "0.0",
                    "--qz", "0.0",
                    "--qw", "1.0",
                    "--frame-id", "world",
                    "--child-frame-id", f"{spec.name}/{spec.urdf_root_frame}",
                ],
                condition=UnlessCondition(publish_full_tf),
            )
        )

        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=spec.name,
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
                condition=IfCondition(publish_full_tf),
            )
        )

    return LaunchDescription(actions)
