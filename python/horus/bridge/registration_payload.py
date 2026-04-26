"""Registration payload builders for HORUS MR compatibility."""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional, Tuple


def build_robot_config_dict(
    client,
    robot,
    dataviz,
    global_visualizations: Optional[list] = None,
    workspace_scale: Optional[float] = None,
    compass_enabled: Optional[bool] = None,
) -> Dict:
    """Build simple JSON dictionary for robot config"""
    dimensions = None
    if getattr(robot, "dimensions", None):
        dimensions = {
            "length": float(robot.dimensions.length),
            "width": float(robot.dimensions.width),
            "height": float(robot.dimensions.height),
        }

    def _coerce_bool(value, default):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ("1", "true", "yes", "on"):
                return True
            if normalized in ("0", "false", "no", "off"):
                return False
        if isinstance(value, (int, float)):
            return bool(value)
        return default

    def _coerce_float(value, default):
        if value is None:
            return float(default)
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(default)

    def _coerce_int(value, default):
        if value is None:
            return int(default)
        try:
            return int(value)
        except (TypeError, ValueError):
            return int(default)

    def _coerce_vec3(value, default_xyz):
        x_default, y_default, z_default = default_xyz
        if isinstance(value, dict):
            x = _coerce_float(value.get("x"), x_default)
            y = _coerce_float(value.get("y"), y_default)
            z = _coerce_float(value.get("z"), z_default)
            return {"x": x, "y": y, "z": z}
        if isinstance(value, (list, tuple)) and len(value) == 3:
            x = _coerce_float(value[0], x_default)
            y = _coerce_float(value[1], y_default)
            z = _coerce_float(value[2], z_default)
            return {"x": x, "y": y, "z": z}
        return {
            "x": float(x_default),
            "y": float(y_default),
            "z": float(z_default),
        }

    def _coerce_text(value, default):
        if value is None:
            return str(default)
        text = str(value).strip()
        return text if text else str(default)

    def _build_camera_config(sensor):
        metadata = sensor.metadata or {}
        def _normalize_transport(value, fallback):
            normalized = str(value).strip().lower() if value is not None else ""
            if normalized in ("ros", "webrtc"):
                return normalized
            return fallback

        def _normalize_image_type(value, fallback):
            normalized = str(value).strip().lower() if value is not None else ""
            if normalized in ("raw", "compressed"):
                return normalized
            return fallback

        def _normalize_layout(value, fallback):
            normalized = str(value).strip().lower() if value is not None else ""
            if normalized in ("sbs", "side_by_side", "side-by-side"):
                return "side_by_side"
            if normalized in ("dual_topic", "dual", "two_topics", "two_topic"):
                return "dual_topic"
            if normalized in ("mono", ""):
                return "mono" if normalized == "mono" else fallback
            return fallback

        legacy_streaming_type = _normalize_transport(
            metadata.get("streaming_type"),
            "",
        )
        if not legacy_streaming_type:
            legacy_streaming_type = _normalize_transport(
                getattr(sensor, "streaming_type", ""),
                "ros",
            )
        if not legacy_streaming_type:
            legacy_streaming_type = "ros"

        minimap_streaming_type = _normalize_transport(
            metadata.get("minimap_streaming_type"),
            "",
        )
        if not minimap_streaming_type:
            minimap_streaming_type = _normalize_transport(
                getattr(sensor, "minimap_streaming_type", ""),
                "",
            )
        if not minimap_streaming_type:
            minimap_streaming_type = _normalize_transport(legacy_streaming_type, "ros")

        teleop_streaming_type = _normalize_transport(
            metadata.get("teleop_streaming_type"),
            "",
        )
        if not teleop_streaming_type:
            teleop_streaming_type = _normalize_transport(
                getattr(sensor, "teleop_streaming_type", ""),
                "",
            )
        if not teleop_streaming_type:
            teleop_streaming_type = _normalize_transport(legacy_streaming_type, "webrtc")

        startup_mode = str(
            metadata.get("startup_mode", getattr(sensor, "startup_mode", "minimap"))
        ).strip().lower()
        if startup_mode not in ("minimap", "teleop"):
            startup_mode = "minimap"

        is_stereo = bool(getattr(sensor, "is_stereo", False))
        raw_stereo_layout = str(
            metadata.get("stereo_layout", getattr(sensor, "stereo_layout", ""))
        ).strip().lower()
        if is_stereo:
            if raw_stereo_layout in ("dual_topic", "dual", "two_topics", "two_topic"):
                stereo_layout = "dual_topic"
            else:
                stereo_layout = "side_by_side"
        else:
            stereo_layout = "mono"

        right_topic = str(
            metadata.get("right_topic", getattr(sensor, "right_topic", ""))
        ).strip()

        minimap_topic = str(
            metadata.get("minimap_topic", getattr(sensor, "minimap_topic", ""))
        ).strip()
        if not minimap_topic:
            minimap_topic = str(getattr(sensor, "topic", "")).strip()

        teleop_topic = str(
            metadata.get("teleop_topic", getattr(sensor, "teleop_topic", ""))
        ).strip()
        if not teleop_topic:
            teleop_topic = minimap_topic

        image_type = _normalize_image_type(metadata.get("image_type"), "raw")
        minimap_image_type = _normalize_image_type(
            metadata.get("minimap_image_type", getattr(sensor, "minimap_image_type", "")),
            image_type,
        )
        teleop_image_type = _normalize_image_type(
            metadata.get("teleop_image_type", getattr(sensor, "teleop_image_type", "")),
            image_type,
        )

        minimap_max_fps = _coerce_int(
            metadata.get("minimap_max_fps", getattr(sensor, "minimap_max_fps", 30)),
            30,
        )
        minimap_max_fps = max(1, min(30, minimap_max_fps))

        teleop_stereo_layout = _normalize_layout(
            metadata.get("teleop_stereo_layout", getattr(sensor, "teleop_stereo_layout", "")),
            stereo_layout,
        )
        teleop_right_topic = str(
            metadata.get("teleop_right_topic", getattr(sensor, "teleop_right_topic", ""))
        ).strip()
        if not teleop_right_topic:
            teleop_right_topic = right_topic
        if not is_stereo:
            teleop_stereo_layout = "mono"
            teleop_right_topic = ""

        return {
            "streaming_type": legacy_streaming_type,
            "minimap_streaming_type": minimap_streaming_type,
            "teleop_streaming_type": teleop_streaming_type,
            "minimap_topic": minimap_topic,
            "teleop_topic": teleop_topic,
            "minimap_image_type": minimap_image_type,
            "teleop_image_type": teleop_image_type,
            "minimap_max_fps": minimap_max_fps,
            "teleop_stereo_layout": teleop_stereo_layout,
            "teleop_right_topic": teleop_right_topic,
            "startup_mode": startup_mode,
            "is_stereo": is_stereo,
            "stereo_layout": stereo_layout,
            "right_topic": right_topic,
            "image_type": image_type,
            "display_mode": str(metadata.get("display_mode", "projected")).lower(),
            "use_tf": _coerce_bool(metadata.get("use_tf"), True),
            "projection_target_frame": str(metadata.get("projection_target_frame", "")),
            "webrtc_client_signal_topic": str(
                metadata.get("webrtc_client_signal_topic", "/horus/webrtc/client_signal")
            ),
            "webrtc_server_signal_topic": str(
                metadata.get("webrtc_server_signal_topic", "/horus/webrtc/server_signal")
            ),
            "webrtc_bitrate_kbps": _coerce_int(
                metadata.get("webrtc_bitrate_kbps"),
                2000,
            ),
            "webrtc_framerate": _coerce_int(
                metadata.get("webrtc_framerate"),
                max(30, min(90, _coerce_int(getattr(sensor, "fps", None), 30))),
            ),
            "webrtc_stun_server_url": str(
                metadata.get("webrtc_stun_server_url", "stun:stun.l.google.com:19302")
            ),
            "webrtc_turn_server_url": str(
                metadata.get("webrtc_turn_server_url", "")
            ),
            "webrtc_turn_username": str(
                metadata.get("webrtc_turn_username", "")
            ),
            "webrtc_turn_credential": str(
                metadata.get("webrtc_turn_credential", "")
            ),
            "image_scale": _coerce_float(metadata.get("image_scale"), 1.0),
            "focal_length_scale": _coerce_float(metadata.get("focal_length_scale"), 0.5),
            "immersive_ros_flip_x": _coerce_bool(metadata.get("immersive_ros_flip_x"), False),
            "immersive_ros_flip_y": _coerce_bool(metadata.get("immersive_ros_flip_y"), False),
            "view_position_offset": _coerce_vec3(
                metadata.get("view_position_offset"), (0.0, 0.0, 0.0)
            ),
            "view_rotation_offset": _coerce_vec3(
                metadata.get("view_rotation_offset"), (0.0, 0.0, 0.0)
            ),
            "projected_position_offset": _coerce_vec3(
                metadata.get("projected_position_offset"), (0.0, 0.0, 0.0)
            ),
            "projected_scale_multiplier": _coerce_vec3(
                metadata.get("projected_scale_multiplier"), (1.0, 1.0, 1.0)
            ),
            "show_frustum": _coerce_bool(metadata.get("show_frustum"), True),
            "frustum_color": str(metadata.get("frustum_color", "#FFFF00")),
            "overhead_size": _coerce_float(metadata.get("overhead_size"), 1.0),
            "overhead_position_offset": _coerce_vec3(
                metadata.get("overhead_position_offset"), (0.0, 2.0, 0.0)
            ),
            "overhead_face_camera": _coerce_bool(
                metadata.get("overhead_face_camera"), True
            ),
            "overhead_rotation_offset": _coerce_vec3(
                metadata.get("overhead_rotation_offset"), (90.0, 0.0, 0.0)
            ),
            "resolution": {
                "width": int(getattr(sensor, "resolution", (640, 480))[0]),
                "height": int(getattr(sensor, "resolution", (640, 480))[1]),
            },
            "fps": int(getattr(sensor, "fps", 30)),
            "fov": _coerce_float(getattr(sensor, "fov", 60.0), 60.0),
            "encoding": str(getattr(sensor, "encoding", "bgr8")),
        }

    def _build_sensor_viz_override_index():
        overrides = {}
        robot_name_key = str(getattr(robot, "name", "") or "").strip()

        for visualization in getattr(dataviz, "visualizations", []) or []:
            data_source = getattr(visualization, "data_source", None)
            if data_source is None:
                continue

            render_options = getattr(visualization, "render_options", {}) or {}
            if not isinstance(render_options, dict) or not render_options:
                continue

            source_robot_name = str(
                getattr(data_source, "robot_name", "") or robot_name_key
            ).strip()
            if source_robot_name != robot_name_key:
                continue

            override = {}
            color_override = render_options.get("color")
            if color_override not in (None, ""):
                override["color"] = str(color_override)

            point_size_override = render_options.get("point_size")
            if point_size_override not in (None, ""):
                override["point_size"] = _coerce_float(point_size_override, 0.05)

            if not override:
                continue

            topic_key = str(getattr(data_source, "topic", "") or "").strip()
            name_key = str(getattr(data_source, "name", "") or "").strip()
            if topic_key:
                overrides[(source_robot_name, "topic", topic_key)] = override
            if name_key:
                overrides.setdefault((source_robot_name, "name", name_key), override)

        return overrides

    sensor_viz_override_index = _build_sensor_viz_override_index()

    def _build_sensor_viz_config(sensor):
        viz_config = {
            "color": getattr(sensor, "color", "white"),
            "point_size": getattr(sensor, "point_size", 0.05),
        }

        robot_name_key = str(getattr(robot, "name", "") or "").strip()
        topic_key = str(getattr(sensor, "topic", "") or "").strip()
        name_key = str(getattr(sensor, "name", "") or "").strip()

        override = None
        if topic_key:
            override = sensor_viz_override_index.get((robot_name_key, "topic", topic_key))
        if override is None and name_key:
            override = sensor_viz_override_index.get((robot_name_key, "name", name_key))

        if isinstance(override, dict):
            color_override = override.get("color")
            if color_override not in (None, ""):
                viz_config["color"] = str(color_override)

            if "point_size" in override:
                viz_config["point_size"] = _coerce_float(
                    override.get("point_size"),
                    viz_config["point_size"],
                )

        return viz_config

    sensor_payloads = []
    for sensor in robot.sensors:
        payload = {
            "name": sensor.name,
            "type": sensor.sensor_type.value,
            "topic": sensor.topic,
            "frame": sensor.frame_id,
            "metadata": sensor.metadata or {},
            "viz_config": _build_sensor_viz_config(sensor),
        }
        if sensor.sensor_type.value == "camera":
            payload["camera_config"] = _build_camera_config(sensor)
        sensor_payloads.append(payload)

    def _build_robot_manager_config():
        robot_metadata = getattr(robot, "metadata", {}) or {}
        manager_metadata = robot_metadata.get("robot_manager_config")
        if not isinstance(manager_metadata, dict):
            manager_metadata = {}

        sections_metadata = manager_metadata.get("sections")
        if not isinstance(sections_metadata, dict):
            sections_metadata = {}

        return {
            "enabled": _coerce_bool(manager_metadata.get("enabled"), True),
            # Editor fallback path keeps setup friction low while the runtime path
            # can be provided explicitly when using Resources/Addressables.
            "prefab_asset_path": str(
                manager_metadata.get("prefab_asset_path", "Assets/Prefabs/UI/RobotManager.prefab")
            ),
            "prefab_resource_path": str(manager_metadata.get("prefab_resource_path", "")),
            "sections": {
                "status": _coerce_bool(sections_metadata.get("status"), True),
                "data_viz": _coerce_bool(sections_metadata.get("data_viz"), True),
                "teleop": _coerce_bool(sections_metadata.get("teleop"), True),
                "tasks": _coerce_bool(sections_metadata.get("tasks"), True),
            },
        }

    def _build_teleop_control():
        robot_metadata = getattr(robot, "metadata", {}) or {}
        teleop_metadata = robot_metadata.get("teleop_config")
        if not isinstance(teleop_metadata, dict):
            teleop_metadata = {}

        allowed_profiles = {"wheeled", "legged", "aerial", "drone", "custom"}
        default_profile = str(getattr(robot, "get_type_str", lambda: "wheeled")()).strip().lower()
        if default_profile not in allowed_profiles:
            default_profile = "wheeled"

        allowed_response_modes = {"analog", "discrete"}
        allowed_deadman_policies = {
            "either_index_trigger",
            "left_index_trigger",
            "right_index_trigger",
            "either_grip_trigger",
        }

        command_topic = _coerce_text(
            teleop_metadata.get("command_topic"),
            robot.resolve_topic("cmd_vel"),
        )
        raw_input_topic = _coerce_text(
            teleop_metadata.get("raw_input_topic"),
            f"/horus/teleop/{robot.name}/joy",
        )
        head_pose_topic = _coerce_text(
            teleop_metadata.get("head_pose_topic"),
            f"/horus/teleop/{robot.name}/head_pose",
        )

        response_mode = _coerce_text(teleop_metadata.get("response_mode"), "analog").lower()
        if response_mode not in allowed_response_modes:
            response_mode = "analog"

        robot_profile = _coerce_text(teleop_metadata.get("robot_profile"), default_profile).lower()
        if robot_profile not in allowed_profiles:
            robot_profile = default_profile

        publish_rate_hz = _coerce_float(teleop_metadata.get("publish_rate_hz"), 30.0)
        publish_rate_hz = max(5.0, min(120.0, publish_rate_hz))

        deadman_metadata = teleop_metadata.get("deadman")
        if not isinstance(deadman_metadata, dict):
            deadman_metadata = {}

        deadman_policy = _coerce_text(
            deadman_metadata.get("policy"),
            "either_grip_trigger",
        ).lower()
        if deadman_policy not in allowed_deadman_policies:
            deadman_policy = "either_grip_trigger"

        deadman_timeout_ms = _coerce_int(deadman_metadata.get("timeout_ms"), 200)
        deadman_timeout_ms = max(50, min(2000, deadman_timeout_ms))

        axes_metadata = teleop_metadata.get("axes")
        if not isinstance(axes_metadata, dict):
            axes_metadata = {}

        deadzone = _coerce_float(axes_metadata.get("deadzone"), 0.15)
        deadzone = max(0.0, min(0.5, deadzone))
        expo = _coerce_float(axes_metadata.get("expo"), 1.7)
        expo = max(1.0, min(3.0, expo))
        linear_xy_max_mps = _coerce_float(axes_metadata.get("linear_xy_max_mps"), 1.0)
        linear_xy_max_mps = max(0.0, min(5.0, linear_xy_max_mps))
        linear_z_max_mps = _coerce_float(axes_metadata.get("linear_z_max_mps"), 0.8)
        linear_z_max_mps = max(0.0, min(5.0, linear_z_max_mps))
        angular_z_max_rps = _coerce_float(axes_metadata.get("angular_z_max_rps"), 1.2)
        angular_z_max_rps = max(0.0, min(6.0, angular_z_max_rps))

        discrete_metadata = teleop_metadata.get("discrete")
        if not isinstance(discrete_metadata, dict):
            discrete_metadata = {}

        discrete_threshold = _coerce_float(discrete_metadata.get("threshold"), 0.6)
        discrete_threshold = max(0.1, min(1.0, discrete_threshold))
        linear_xy_step_mps = _coerce_float(discrete_metadata.get("linear_xy_step_mps"), 0.6)
        linear_xy_step_mps = max(0.0, min(5.0, linear_xy_step_mps))
        linear_z_step_mps = _coerce_float(discrete_metadata.get("linear_z_step_mps"), 0.4)
        linear_z_step_mps = max(0.0, min(5.0, linear_z_step_mps))
        angular_z_step_rps = _coerce_float(discrete_metadata.get("angular_z_step_rps"), 0.9)
        angular_z_step_rps = max(0.0, min(6.0, angular_z_step_rps))

        return {
            "enabled": _coerce_bool(teleop_metadata.get("enabled"), True),
            "command_topic": command_topic,
            "raw_input_topic": raw_input_topic,
            "head_pose_topic": head_pose_topic,
            "robot_profile": robot_profile,
            "response_mode": response_mode,
            "publish_rate_hz": publish_rate_hz,
            "custom_passthrough_only": _coerce_bool(
                teleop_metadata.get("custom_passthrough_only"),
                False,
            ),
            "deadman": {
                "policy": deadman_policy,
                "timeout_ms": deadman_timeout_ms,
            },
            "axes": {
                "deadzone": deadzone,
                "expo": expo,
                "linear_xy_max_mps": linear_xy_max_mps,
                "linear_z_max_mps": linear_z_max_mps,
                "angular_z_max_rps": angular_z_max_rps,
            },
            "discrete": {
                "threshold": discrete_threshold,
                "linear_xy_step_mps": linear_xy_step_mps,
                "linear_z_step_mps": linear_z_step_mps,
                "angular_z_step_rps": angular_z_step_rps,
            },
        }

    def _build_task_control():
        robot_metadata = getattr(robot, "metadata", {}) or {}
        task_metadata = robot_metadata.get("task_config")
        if not isinstance(task_metadata, dict):
            task_metadata = {}

        go_to_point_metadata = task_metadata.get("go_to_point")
        if not isinstance(go_to_point_metadata, dict):
            go_to_point_metadata = {}

        goal_topic = _coerce_text(
            go_to_point_metadata.get("goal_topic"),
            robot.resolve_topic("goal_pose"),
        )
        cancel_topic = _coerce_text(
            go_to_point_metadata.get("cancel_topic"),
            robot.resolve_topic("goal_cancel"),
        )
        status_topic = _coerce_text(
            go_to_point_metadata.get("status_topic"),
            robot.resolve_topic("goal_status"),
        )
        frame_id = _coerce_text(go_to_point_metadata.get("frame_id"), "map")
        if not frame_id:
            frame_id = "map"

        position_tolerance_m = _coerce_float(go_to_point_metadata.get("position_tolerance_m"), 0.20)
        position_tolerance_m = max(0.01, min(10.0, position_tolerance_m))
        yaw_tolerance_deg = _coerce_float(go_to_point_metadata.get("yaw_tolerance_deg"), 12.0)
        yaw_tolerance_deg = max(0.1, min(180.0, yaw_tolerance_deg))
        min_altitude_m = _coerce_float(go_to_point_metadata.get("min_altitude_m"), 0.0)
        min_altitude_m = max(0.0, min(100.0, min_altitude_m))
        max_altitude_m = _coerce_float(go_to_point_metadata.get("max_altitude_m"), 10.0)
        max_altitude_m = max(min_altitude_m + 0.1, min(100.0, max_altitude_m))

        waypoint_metadata = task_metadata.get("waypoint")
        if not isinstance(waypoint_metadata, dict):
            waypoint_metadata = {}

        waypoint_path_topic = _coerce_text(
            waypoint_metadata.get("path_topic"),
            robot.resolve_topic("waypoint_path"),
        )
        waypoint_status_topic = _coerce_text(
            waypoint_metadata.get("status_topic"),
            robot.resolve_topic("waypoint_status"),
        )
        waypoint_frame_id = _coerce_text(waypoint_metadata.get("frame_id"), "map")
        if not waypoint_frame_id:
            waypoint_frame_id = "map"

        waypoint_position_tolerance_m = _coerce_float(waypoint_metadata.get("position_tolerance_m"), 0.20)
        waypoint_position_tolerance_m = max(0.01, min(10.0, waypoint_position_tolerance_m))
        waypoint_yaw_tolerance_deg = _coerce_float(waypoint_metadata.get("yaw_tolerance_deg"), 12.0)
        waypoint_yaw_tolerance_deg = max(0.1, min(180.0, waypoint_yaw_tolerance_deg))

        return {
            "go_to_point": {
                "enabled": _coerce_bool(go_to_point_metadata.get("enabled"), True),
                "goal_topic": goal_topic,
                "cancel_topic": cancel_topic,
                "status_topic": status_topic,
                "frame_id": frame_id,
                "position_tolerance_m": position_tolerance_m,
                "yaw_tolerance_deg": yaw_tolerance_deg,
                "min_altitude_m": min_altitude_m,
                "max_altitude_m": max_altitude_m,
            },
            "waypoint": {
                "enabled": _coerce_bool(waypoint_metadata.get("enabled"), True),
                "path_topic": waypoint_path_topic,
                "status_topic": waypoint_status_topic,
                "frame_id": waypoint_frame_id,
                "position_tolerance_m": waypoint_position_tolerance_m,
                "yaw_tolerance_deg": waypoint_yaw_tolerance_deg,
            },
        }

    robot_visualizations = []
    fallback_global_visualizations = []
    for visualization in dataviz.visualizations:
        payload = client._serialize_visualization_payload(visualization)
        if not payload:
            continue

        if payload.get("scope") == "global":
            fallback_global_visualizations.append(payload)
            continue

        robot_visualizations.append(payload)

    if global_visualizations is None:
        global_visualizations = []
        seen_global = set()
        for payload in fallback_global_visualizations:
            if str(payload.get("type", "")) == "semantic_box":
                semantic_payload = payload.get("semantic_box") or {}
                key = (
                    str(payload.get("type", "")),
                    str(semantic_payload.get("id", "")),
                    str(payload.get("frame", "")),
                )
            else:
                key = (
                    str(payload.get("type", "")),
                    str(payload.get("topic", "")),
                    str(payload.get("frame", "")),
                )
            if key in seen_global:
                continue
            seen_global.add(key)
            global_visualizations.append(payload)

    drive_topic = robot.resolve_topic("cmd_vel")
    teleop_control = _build_teleop_control()
    task_control = _build_task_control()
    if isinstance(teleop_control, dict):
        command_override = str(teleop_control.get("command_topic", "")).strip()
        if command_override:
            drive_topic = command_override

    description_container = client._resolve_robot_description_artifact(robot)

    config = {
        "action": "register",
        "robot_name": robot.name,
        "robot_type": robot.get_type_str(),
        "ros_binding": robot.get_ros_binding(),
        "sensors": sensor_payloads,
        "visualizations": robot_visualizations,
        "global_visualizations": global_visualizations,
        "control": {
            "drive_topic": drive_topic,
            "teleop": teleop_control,
            "tasks": task_control,
        },
        "robot_manager_config": _build_robot_manager_config(),
        "timestamp": time.time()
    }

    local_body_config = robot.get_metadata("local_body_model_config", {})
    if isinstance(local_body_config, dict):
        robot_model_id = str(local_body_config.get("robot_model_id", "") or "").strip().lower()
        local_body_enabled = bool(local_body_config.get("enabled", False))
        if local_body_enabled and robot_model_id:
            config["robot_model_id"] = robot_model_id
            config["has_visual_mesh_model"] = True

    if description_container is not None:
        artifact = description_container.get("artifact")
        if artifact is not None:
            config["robot_description_manifest"] = artifact.to_manifest_payload()
            if bool(getattr(artifact.manifest, "supports_visual_meshes", False)):
                config["has_visual_mesh_model"] = True
            # Keep inline payloads only when they stay reasonably small.
            # Mesh-backed descriptions use the existing chunk transport path.
            if len(str(artifact.payload_json or "")) <= 250000:
                config["robot_description_payload_json"] = artifact.payload_json

    if dimensions is not None:
        config["dimensions"] = dimensions

    normalized_workspace_scale = None
    if workspace_scale is not None:
        try:
            candidate = float(workspace_scale)
        except (TypeError, ValueError):
            candidate = None

        if candidate is not None and math.isfinite(candidate) and candidate > 0.0:
            normalized_workspace_scale = candidate

    workspace_config: Dict[str, Any] = {}
    if normalized_workspace_scale is not None:
        workspace_config["position_scale"] = normalized_workspace_scale

    if compass_enabled is not None:
        workspace_config["compass"] = {
            "enabled": _coerce_bool(compass_enabled, False),
        }

    workspace_tutorial_metadata = robot.get_metadata("workspace_tutorial_config", {})
    if isinstance(workspace_tutorial_metadata, dict):
        tutorial_preset_id = _coerce_text(
            workspace_tutorial_metadata.get("preset_id"),
            "",
        )
        if tutorial_preset_id:
            workspace_config["tutorial"] = {
                "enabled": _coerce_bool(workspace_tutorial_metadata.get("enabled"), True),
                "preset_id": tutorial_preset_id,
            }

    if workspace_config:
        config["workspace_config"] = workspace_config

    return config
