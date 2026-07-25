"""Tests for semantic Robot/Camera configuration helpers."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.robot import (
    DeadmanPolicy,
    Robot,
    RobotType,
    TeleopProfile,
    TeleopResponseMode,
    WorkspaceVisualization,
)
from horus.sensors import Camera, CameraTransport


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client._robot_description_resolver = None
    client._robot_description_by_robot = {}
    client._robot_description_by_id = {}
    return client


def _build_config(robot):
    return _build_client()._build_robot_config_dict(robot, robot.create_dataviz())


def test_robot_manager_helper_keeps_payload_shape():
    robot = Robot(name="ui_bot", robot_type=RobotType.WHEELED)
    robot.configure_robot_manager(status=True, data_viz=False, teleop=True, tasks=False)

    config = _build_config(robot)
    manager = config["robot_manager_config"]

    assert manager["enabled"] is True
    assert manager["sections"] == {
        "status": True,
        "data_viz": False,
        "teleop": True,
        "tasks": False,
    }


def test_teleop_helper_keeps_payload_shape_and_defaults():
    robot = Robot(name="teleop_bot", robot_type=RobotType.WHEELED)
    robot.configure_teleop(
        command_topic="/teleop_bot/custom_cmd_vel",
        robot_profile=TeleopProfile.WHEELED,
        response_mode=TeleopResponseMode.ANALOG,
        deadman_policy=DeadmanPolicy.EITHER_GRIP_TRIGGER,
        linear_xy_max_mps=0.35,
        linear_z_max_mps=0.0,
        angular_z_max_rps=0.8,
        invert_angular_z=True,
    )

    teleop = _build_config(robot)["control"]["teleop"]

    assert teleop["enabled"] is True
    assert teleop["command_topic"] == "/teleop_bot/custom_cmd_vel"
    assert teleop["robot_profile"] == "wheeled"
    assert teleop["response_mode"] == "analog"
    assert teleop["deadman"]["policy"] == "either_grip_trigger"
    assert teleop["axes"]["linear_xy_max_mps"] == 0.35
    assert teleop["axes"]["linear_z_max_mps"] == 0.0
    assert teleop["axes"]["angular_z_max_rps"] == 0.8
    assert teleop["axes"]["invert_angular_z"] is True


def test_navigation_task_helpers_keep_payload_shape():
    robot = Robot(name="task_bot", robot_type=RobotType.WHEELED)
    robot.set_metadata("task_config", {"label_pose": {"enabled": True}})
    robot.configure_navigation_tasks(
        goal_topic="/task_bot/nav_goal",
        cancel_topic="/task_bot/nav_cancel",
        goal_status_topic="/task_bot/nav_status",
        waypoint_path_topic="/task_bot/route",
        waypoint_status_topic="/task_bot/route_status",
        frame_id="map",
        position_tolerance_m=0.35,
        yaw_tolerance_deg=25.0,
    )

    assert robot.metadata["task_config"]["label_pose"] == {"enabled": True}

    tasks = _build_config(robot)["control"]["tasks"]

    assert tasks["go_to_point"]["goal_topic"] == "/task_bot/nav_goal"
    assert tasks["go_to_point"]["cancel_topic"] == "/task_bot/nav_cancel"
    assert tasks["go_to_point"]["status_topic"] == "/task_bot/nav_status"
    assert tasks["go_to_point"]["position_tolerance_m"] == 0.35
    assert tasks["go_to_point"]["yaw_tolerance_deg"] == 25.0
    assert tasks["waypoint"]["path_topic"] == "/task_bot/route"
    assert tasks["waypoint"]["status_topic"] == "/task_bot/route_status"
    assert tasks["waypoint"]["position_tolerance_m"] == 0.35
    assert tasks["waypoint"]["yaw_tolerance_deg"] == 25.0


def test_camera_view_helpers_keep_payload_shape():
    robot = Robot(name="camera_bot", robot_type=RobotType.WHEELED)
    camera = Camera(
        name="front_camera",
        frame_id="camera_bot/front_camera",
        topic="/camera_bot/front/image_raw/compressed",
        encoding="jpeg",
        streaming_type=CameraTransport.ROS,
        minimap_image_type="compressed",
        teleop_image_type="compressed",
    )
    camera.configure_projected_view(image_scale=0.25, focal_length_scale=0.5)
    camera.configure_minimap_view(size=2.5, position_offset=(0.0, 1.0, 0.0))
    camera.configure_immersive_view(ros_flip_x=True, ros_flip_y=True)
    camera.configure_webrtc_transport(bitrate_kbps=1500, framerate=20)
    robot.add_sensor(camera)

    sensor = _build_config(robot)["sensors"][0]
    camera_config = sensor["camera_config"]

    assert camera_config["image_scale"] == 0.25
    assert camera_config["focal_length_scale"] == 0.5
    assert camera_config["overhead_size"] == 2.5
    assert camera_config["overhead_position_offset"] == {"x": 0.0, "y": 1.0, "z": 0.0}
    assert camera_config["immersive_ros_flip_x"] is True
    assert camera_config["immersive_ros_flip_y"] is True
    assert camera_config["webrtc_bitrate_kbps"] == 1500
    assert camera_config["webrtc_framerate"] == 20


def test_workspace_remote_render_is_global_and_has_no_robot_controls():
    client = _build_client()
    resource = WorkspaceVisualization("workspace_remote_map")
    dataviz = resource.create_dataviz()
    dataviz.add_remote_rendered_map(
        render_options={
            "status_topic": "/horus/remote_render/agent_status",
            "frame_data_topic": "/horus/remote_render/frame_data",
        }
    )
    global_visualizations = client._build_global_visualizations_payload([dataviz])
    config = client._build_robot_config_dict(
        resource,
        dataviz,
        global_visualizations=global_visualizations,
        workspace_scale=0.1,
    )

    assert config["entity_kind"] == "workspace_visualization"
    assert config["sensors"] == []
    assert config["visualizations"] == []
    assert config["robot_manager_config"]["enabled"] is False
    assert config["control"]["teleop"]["enabled"] is False
    assert config["control"]["tasks"]["go_to_point"]["enabled"] is False
    assert len(config["global_visualizations"]) == 1
    remote = config["global_visualizations"][0]
    assert remote["type"] == "remote_render"
    assert remote["scope"] == "global"
    assert remote["topic"] == "/horus/remote_render/map_portal"
    assert remote["remote_render"]["format_version"] == (
        "remote_stereo_rgbd_ros_v2"
    )
    assert remote["remote_render"]["viewer_pose_topic"] == "/horus/remote_render/viewer_pose"
    assert remote["remote_render"]["viewer_pose_rate_hz"] == 60.0
    assert remote["remote_render"]["transport"] == "ros_compressed"
    assert remote["remote_render"]["ros_compressed_topic"] == (
        "/horus/remote_render/rgbd"
    )
    assert remote["remote_render"]["frame_data_topic"] == "/horus/remote_render/frame_data"
    assert remote["remote_render"]["encoder"] == "nvenc"
    assert remote["remote_render"]["bitrate_kbps"] == 12000
    assert remote["remote_render"]["client_signal_topic"] == "/horus/webrtc/client_signal"
    assert remote["remote_render"]["server_signal_topic"] == "/horus/webrtc/server_signal"
    assert remote["remote_render"]["status_topic"] == "/horus/remote_render/agent_status"
    assert client._collect_topics(dataviz) == [
        "/horus/remote_render/rgbd",
        "/horus/remote_render/agent_status",
    ]
    assert "/horus/remote_render/map_portal" not in client._collect_topics(dataviz)


def test_workspace_remote_render_pose_contract_is_serialized():
    client = _build_client()
    resource = WorkspaceVisualization("workspace_remote_map")
    dataviz = resource.create_dataviz()
    dataviz.add_remote_rendered_map(
        render_options={
            "viewer_pose_topic": "/test/remote_map/viewer_pose",
            "viewer_pose_rate_hz": 72.0,
            "frame_data_topic": "/test/remote_map/frame_data",
        }
    )

    remote = client._build_global_visualizations_payload([dataviz])[0]["remote_render"]

    assert remote["format_version"] == "remote_stereo_rgbd_ros_v2"
    assert remote["transport"] == "ros_compressed"
    assert remote["viewer_pose_topic"] == "/test/remote_map/viewer_pose"
    assert remote["viewer_pose_rate_hz"] == 72.0
    assert remote["frame_data_topic"] == "/test/remote_map/frame_data"


def test_metadata_aliases_remain_available():
    robot = Robot(name="compat_bot", robot_type=RobotType.WHEELED)
    camera = Camera(name="camera", frame_id="compat_bot/camera", topic="/compat_bot/camera")

    robot.set_metadata("custom_robot_key", "robot_value")
    camera.set_metadata("custom_sensor_key", "sensor_value")

    assert robot.get_metadata("custom_robot_key") == "robot_value"
    assert camera.get_metadata("custom_sensor_key") == "sensor_value"
