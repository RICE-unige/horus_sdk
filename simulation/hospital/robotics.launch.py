"""Bring up three independent cooperative SLAM/Nav2 stacks and one RGB-D mapper."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from configuration import navigation_topic_overrides


def generate_launch_description():
    run = Path(os.environ["HORUS_HOSPITAL_RUN"])
    slam_launch = Path(get_package_share_directory("slam_toolbox")) / "launch/online_async_decentralized_multirobot_launch.py"
    actions = []
    lifecycle = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
    for name in ("carter1", "carter2", "carter3"):
        urdf = Path(os.environ["HORUS_HOSPITAL_URDF"]).read_text()
        actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", namespace=name, output="screen",
            parameters=[{"use_sim_time": True, "robot_description": urdf,
                         "frame_prefix": f"{name}/", "publish_frequency": 30.0}]))
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam_launch)),
            launch_arguments={"namespace": name, "slam_params_file": str(run / "config/slam.yaml"),
                              "use_sim_time": "true", "autostart": "true"}.items()))
        parameters = RewrittenYaml(source_file=str(run / "config/navigation.yaml"), root_key=name,
            param_rewrites=navigation_topic_overrides(name), convert_types=True)
        for node, package in (("controller_server", "nav2_controller"),
                               ("planner_server", "nav2_planner"),
                               ("behavior_server", "nav2_behaviors"),
                               ("bt_navigator", "nav2_bt_navigator")):
            actions.append(Node(package=package, executable=node, name=node, namespace=name,
                output="screen", parameters=[parameters],
                remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "nav_cmd_vel")]))
        actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="navigation_lifecycle", namespace=name, output="screen",
            parameters=[{"use_sim_time": True, "autostart": True, "node_names": lifecycle,
                         "bond_timeout": 10.0}]))
    remappings = []
    for index, name in enumerate(("carter1", "carter2", "carter3")):
        for kind in ("color", "depth"):
            remappings += [(f"camera_{index}/{kind}/image", f"/{name}/camera/{kind}/image_raw"),
                           (f"camera_{index}/{kind}/camera_info", f"/{name}/camera/camera_info")]
    actions.append(Node(package="nvblox_ros", executable="nvblox_node", name="hospital_mapper",
        output="screen", parameters=[str(run / "config/nvblox.yaml")], remappings=remappings))
    # A failed child must stop the launch process so the supervisor cannot report a partial fleet as healthy.
    def required_exited(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason=f"Required process {event.process_name} exited ({event.returncode})"))]
    required_exit = RegisterEventHandler(OnProcessExit(on_exit=required_exited))
    return LaunchDescription([required_exit, *actions])
