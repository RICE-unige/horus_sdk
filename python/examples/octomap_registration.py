#!/usr/bin/env python3
"""Register the octomap map showcase for HORUS MR.

Fetch the sample URDFs once:
    python3 python/examples/tools/fetch_robot_description_assets.py

Pair this with:
    python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode octomap --map-3d-profile realistic

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/octomap_registration.py
"""

from pathlib import Path

from horus.robot import Robot, RobotDimensions, RobotType, register_robots
from horus.sensors import Camera

ASSET_DIR = Path(__file__).resolve().parent / ".local_assets" / "robot_descriptions"
ROBOT_MODELS = [
    ("anymal_c", RobotType.LEGGED, RobotDimensions(0.95, 0.55, 0.70), "base", "anymal_c.urdf"),
    ("jackal", RobotType.WHEELED, RobotDimensions(0.51, 0.43, 0.25), "base_link", "jackal.urdf"),
    ("go1", RobotType.LEGGED, RobotDimensions(0.65, 0.32, 0.45), "base", "go1.urdf"),
    ("h1", RobotType.LEGGED, RobotDimensions(0.55, 0.38, 1.25), "pelvis", "h1.urdf"),
]


def require_urdf(urdf_file: str) -> Path:
    urdf_path = ASSET_DIR / urdf_file
    if not urdf_path.exists():
        raise SystemExit(
            f"Missing {urdf_path}. Run: python3 python/examples/tools/fetch_robot_description_assets.py"
        )
    return urdf_path


def build_camera(robot_name: str) -> Camera:
    camera = Camera(
        name="front_camera",
        frame_id=f"{robot_name}/camera_link",
        topic=f"/{robot_name}/camera/image_raw/compressed",
        resolution=(160, 90),
        fps=6,
        encoding="jpeg",
        streaming_type="ros",
        minimap_streaming_type="ros",
        teleop_streaming_type="ros",
        minimap_image_type="compressed",
        teleop_image_type="compressed",
    )
    camera.add_metadata("image_type", "compressed")
    camera.configure_projected_view(
        image_scale=1.0,
        focal_length_scale=0.55,
        show_frustum=True,
        frustum_color="#E6E6E0A0",
    )
    camera.configure_minimap_view(
        size=10.0,
        position_offset=(0.0, 2.0, 0.0),
        face_camera=True,
        rotation_offset=(90.0, 0.0, 0.0),
    )
    return camera


robots = []
datavizs = []

for name, robot_type, dimensions, base_frame, urdf_file in ROBOT_MODELS:
    robot = Robot(name=name, robot_type=robot_type, dimensions=dimensions)
    robot.configure_ros_binding(base_frame=base_frame)
    robot.configure_robot_description(
        urdf_path=str(require_urdf(urdf_file)),
        base_frame=base_frame,
        source="ros",
        include_visual_meshes=True,
        visual_mesh_triangle_budget=90000,
        body_mesh_mode="preview_mesh",
    )
    robot.configure_robot_manager()
    robot.configure_teleop(
        command_topic=f"/{name}/cmd_vel",
        robot_profile=robot_type.value,
    )
    robot.configure_navigation_tasks(
        goal_topic=f"/{name}/goal_pose",
        cancel_topic=f"/{name}/goal_cancel",
        goal_status_topic=f"/{name}/goal_status",
        waypoint_path_topic=f"/{name}/waypoint_path",
        waypoint_status_topic=f"/{name}/waypoint_status",
        frame_id="map",
    )
    robot.add_sensor(build_camera(name))

    dataviz = robot.create_dataviz()
    robot.add_path_planning_to_dataviz(
        dataviz,
        global_path_topic=f"/{name}/global_path",
        local_path_topic=f"/{name}/local_path",
    )
    robot.add_navigation_safety_to_dataviz(
        dataviz,
        odom_topic=f"/{name}/odom",
        collision_risk_topic=f"/{name}/collision_risk",
    )
    robots.append(robot)
    datavizs.append(dataviz)

world_layers = datavizs[0]
world_layers.add_3d_octomap(
    "/map_3d_octomap_mesh",
    frame_id="map",
    render_options={
        "render_mode": "surface_mesh",
        "native_topic": "/map_3d_octomap",
        "max_triangles": 120000,
    },
)

success, result = register_robots(
    robots,
    datavizs=datavizs,
    workspace_scale=0.1,
    compass_enabled=False,
    keep_alive=True,
)

if not success:
    raise SystemExit(f"HORUS registration failed: {result}")
