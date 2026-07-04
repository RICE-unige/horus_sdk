#!/usr/bin/env python3
"""Register the dense mesh map showcase for HORUS MR.

Fetch the sample URDFs once:
    python3 python/examples/tools/fetch_robot_description_assets.py

Synthetic mesh source:
    python3 python/examples/legacy/fake_tf_robot_description_suite.py \
        --robot-profile real_models \
        --map-3d-mode mesh \
        --map-3d-profile realistic \
        --map-3d-mesh-voxel-size 0.07 \
        --map-3d-mesh-max-voxels 220000 \
        --map-3d-mesh-max-triangles 220000 \
        --map-3d-mesh-update-policy snapshot

Real Voxblox Cow & Lady source:
    python3 python/examples/tools/fetch_voxblox_cow_lady.py

ROS 2-native Cow & Lady ground-truth PLY path:
    source /opt/ros/jazzy/setup.bash
    python3 python/examples/tools/ply_to_horus_mesh_marker.py \
        --topic /map_3d_mesh \
        --frame-id map \
        --mode voxel_surface \
        --voxel-size 0.02 \
        --republish-interval 0
    The publisher waits for the HORUS bridge/headset subscriber before sending the map.
    It does not publish DELETEALL by default, so late clear messages cannot erase dense
    chunks after loading.
    Use --voxel-size 0.03 for a lighter balanced profile if Quest performance becomes
    the limiting factor.

Maximum-density point-map preview:
    python3 python/examples/tools/ply_to_horus_mesh_marker.py \
        --topic /map_3d_mesh \
        --frame-id map \
        --mode triangle_shell \
        --shape triad \
        --triangle-size 0.025 \
        --max-triangles 0 \
        --republish-interval 0

Run upstream Voxblox on the downloaded data.bag, then relay its mesh topic:
    python3 python/examples/tools/voxblox_mesh_to_horus_marker.py \
        --ros-api ros1 \
        --input-topic /voxblox_node/mesh \
        --output-topic /map_3d_mesh \
        --republish-interval 2.0

Bridge /map_3d_mesh into ROS 2 if Voxblox is running in ROS 1.

From a source checkout:
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/mesh_map_registration.py \
        workspace_scale=0,2
"""

import argparse
from pathlib import Path

from horus.robot import (
    Robot,
    RobotDimensions,
    RobotType,
    is_registration_cancelled,
    register_robots,
)
from horus.sensors import Camera

ASSET_DIR = Path(__file__).resolve().parent / ".local_assets" / "robot_descriptions"
ROBOT_MODELS = [
    ("anymal_c", RobotType.LEGGED, RobotDimensions(0.95, 0.55, 0.70), "base", "anymal_c.urdf"),
    ("jackal", RobotType.WHEELED, RobotDimensions(0.51, 0.43, 0.25), "base_link", "jackal.urdf"),
    ("go1", RobotType.LEGGED, RobotDimensions(0.65, 0.32, 0.45), "base", "go1.urdf"),
    ("h1", RobotType.LEGGED, RobotDimensions(0.55, 0.38, 1.25), "pelvis", "h1.urdf"),
]


def parse_float(value: str) -> float:
    return float(str(value).strip().replace(",", "."))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace-scale", type=parse_float, default=0.1)
    parser.add_argument("--mesh-max-triangles", type=int, default=2_000_000)
    parser.add_argument(
        "overrides",
        nargs="*",
        help="Optional key=value overrides, e.g. workspace_scale=0,2.",
    )
    args = parser.parse_args()
    for override in args.overrides:
        if "=" not in override:
            raise SystemExit(f"Expected key=value override, got: {override}")
        key, value = override.split("=", 1)
        normalized_key = key.strip().replace("-", "_")
        if normalized_key == "workspace_scale":
            args.workspace_scale = parse_float(value)
        elif normalized_key == "mesh_max_triangles":
            args.mesh_max_triangles = int(parse_float(value))
        else:
            raise SystemExit(f"Unknown override: {key}")
    return args


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


args = parse_args()

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
world_layers.add_3d_mesh(
    "/map_3d_mesh",
    frame_id="map",
    render_options={
        "max_triangles": int(args.mesh_max_triangles),
        "use_vertex_colors": True,
        "source_coordinate_space": "enu",
    },
)

success, result = register_robots(
    robots,
    datavizs=datavizs,
    workspace_scale=float(args.workspace_scale),
    compass_enabled=False,
    keep_alive=True,
)

if not success:
    if is_registration_cancelled(result):
        print("HORUS registration monitor stopped.")
        raise SystemExit(0)
    raise SystemExit(f"HORUS registration failed: {result}")
