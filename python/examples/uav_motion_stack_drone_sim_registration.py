#!/usr/bin/env python3
"""Register the uav_motion_stack Baby-K drone sim with HORUS MR.

Run on arancino while the simulator container is running:

    cd ~/horus_sdk
    source /opt/ros/jazzy/setup.bash
    source ~/horus_ros2/install/setup.bash
    python3 python/examples/uav_motion_stack_drone_sim_registration.py
"""

import os
from pathlib import Path
import sys
import threading

SDK_PYTHON_DIR = Path(__file__).resolve().parents[1]
if str(SDK_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(SDK_PYTHON_DIR))

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera, Lidar3D


ROBOT_NAME = "seed_pdt_drone"
BASE_FRAME = "base_link"
MAP_FRAME = "map"


def start_horus_tf_aliases():
    """Republish the sim TF frames under the HORUS robot prefix."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster

    if not rclpy.ok():
        rclpy.init(args=None)

    node = rclpy.create_node("horus_uav_motion_stack_tf_aliases")
    broadcaster = TransformBroadcaster(node)
    latest = {}

    aliases = {
        ("drone/map", "odom"): (f"{ROBOT_NAME}/map", f"{ROBOT_NAME}/odom"),
        ("odom", "base_link"): (f"{ROBOT_NAME}/odom", f"{ROBOT_NAME}/base_link"),
        ("base_link", "base_link_FRD"): (
            f"{ROBOT_NAME}/base_link",
            f"{ROBOT_NAME}/base_link_FRD",
        ),
        ("base_link", "baby_k_0/OakD-Lite/base_link/IMX214"): (
            f"{ROBOT_NAME}/base_link",
            f"{ROBOT_NAME}/oak_rgb_frame",
        ),
        ("base_link", "baby_k_0/OakD-Lite/base_link/StereoOV7251"): (
            f"{ROBOT_NAME}/base_link",
            f"{ROBOT_NAME}/oak_depth_frame",
        ),
    }

    def remember(msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip("/")
            child = transform.child_frame_id.strip("/")
            if (parent, child) in aliases:
                latest[(parent, child)] = transform.transform

    def publish_aliases():
        stamp = node.get_clock().now().to_msg()
        output = []

        root = TransformStamped()
        root.header.stamp = stamp
        root.header.frame_id = MAP_FRAME
        root.child_frame_id = f"{ROBOT_NAME}/map"
        root.transform.rotation.w = 1.0
        output.append(root)

        for source, target in aliases.items():
            if source not in latest:
                continue
            alias = TransformStamped()
            alias.header.stamp = stamp
            alias.header.frame_id = target[0]
            alias.child_frame_id = target[1]
            alias.transform = latest[source]
            output.append(alias)

        broadcaster.sendTransform(output)

    static_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(TFMessage, "/tf", remember, 100)
    node.create_subscription(TFMessage, "/tf_static", remember, static_qos)
    node.create_timer(0.05, publish_aliases)

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    return node, thread, rclpy


os.environ.setdefault("ROS_DOMAIN_ID", "17")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

tf_node, tf_thread, rclpy = start_horus_tf_aliases()

robot = Robot(
    name=ROBOT_NAME,
    robot_type=RobotType.DRONE,
    dimensions=RobotDimensions(length=0.46, width=0.46, height=0.18),
)
robot.configure_ros_binding(
    tf_mode="prefixed",
    topic_mode="flat",
    base_frame=BASE_FRAME,
    tf_prefix=ROBOT_NAME,
)
robot.configure_robot_manager()
robot.configure_teleop(
    command_topic="/teleop/velocity_increments",
    robot_profile="drone",
    linear_xy_max_mps=1.0,
    linear_z_max_mps=0.5,
    angular_z_max_rps=0.8,
)
robot.configure_navigation_tasks(
    goal_topic="/move_base_simple/goal",
    cancel_topic=f"/{ROBOT_NAME}/goal_cancel",
    goal_status_topic="/move_manager/status",
    waypoint_path_topic="/trajectory_path",
    waypoint_status_topic="/trajectory_interpolator/status",
    frame_id=MAP_FRAME,
    min_altitude_m=0.0,
    max_altitude_m=10.0,
)

camera = Camera(
    name="oak_rgb",
    frame_id="oak_rgb_frame",
    topic="/camera",
    resolution=(640, 480),
    fps=30,
    fov=69.0,
    encoding="rgb8",
    streaming_type="ros",
    minimap_streaming_type="ros",
    teleop_streaming_type="ros",
    startup_mode="minimap",
    minimap_image_type="raw",
    teleop_image_type="raw",
)
camera.add_metadata("camera_info_topic", "/camera_info")
camera.add_metadata("image_type", "raw")
camera.configure_projected_view(
    image_scale=1.0,
    focal_length_scale=0.55,
    projection_target_frame=robot.resolve_tf_frame(BASE_FRAME),
    show_frustum=True,
    frustum_color="#E6E6E0A0",
)
camera.configure_minimap_view(
    size=10.0,
    position_offset=(0.0, 1.6, 0.0),
    face_camera=True,
    rotation_offset=(90.0, 0.0, 0.0),
)
robot.add_sensor(camera)

robot.add_sensor(
    Lidar3D(
        name="oak_depth_cloud",
        frame_id="oak_depth_frame",
        topic="/depth_camera/points",
        horizontal_fov=75.0,
        vertical_fov=58.0,
        max_range=12.0,
        points_per_second=150000,
    )
)

dataviz = robot.create_dataviz()
robot.add_path_planning_to_dataviz(
    dataviz,
    global_path_topic="/path_planner/optimized_path",
    local_path_topic="/trajectory_interpolator/transformed_path",
    trajectory_topic="/trajectory_path",
)
robot.add_navigation_safety_to_dataviz(
    dataviz,
    odom_topic="/model/baby_k_0/odometry",
    collision_risk_topic=f"/{ROBOT_NAME}/collision_risk",
    include_collision=False,
)
dataviz.add_3d_octomap(
    "/octomap_full",
    frame_id=MAP_FRAME,
    render_options={"render_mode": "voxels", "alpha": 0.35, "max_depth": 16},
)
dataviz.add_tf_tree("/tf", frame_id=MAP_FRAME)

try:
    success, result = register_robots(
        [robot],
        datavizs=[dataviz],
        workspace_scale=0.1,
        compass_enabled=False,
        keep_alive=True,
        wait_for_app_before_register=False,
    )

    if not success:
        raise SystemExit(f"HORUS registration failed: {result}")
finally:
    tf_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    tf_thread.join(timeout=1.0)
