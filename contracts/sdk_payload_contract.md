# HORUS SDK Payload Contract (Python Parity Baseline)

This document freezes the payload/registration contract implemented in `python/horus/bridge/robot_registry.py` and used by Unity.

Scope:
- Included: implemented Python behavior in `python/horus/**` and `python/examples/**`.
- Excluded: Python stubs and `examples/demo_unity_viz.py`.

## Registration Envelope

`action`:
- `"register"` for registration requests.

`robot_name`:
- Required string.

`robot_type`:
- Required string (`"wheeled"`, `"legged"`, `"aerial"`).

`timestamp`:
- Seconds since Unix epoch as float.

## Sensors

Each sensor entry contains:
- `name`, `type`, `topic`, `frame`.
- `metadata` (dict passthrough).
- `viz_config`:
  - `color` default `"white"` except LaserScan defaults to sensor color.
  - `point_size` default `0.05` unless sensor metadata/fields override.
- `camera_config` only for `type == "camera"`.

### Camera Transport Profile Rules

Defaults:
- `streaming_type`: `ros`
- `minimap_streaming_type`: `ros`
- `teleop_streaming_type`: `webrtc`
- `startup_mode`: `minimap`

Fallback behavior:
- `minimap_streaming_type` falls back to `streaming_type` when empty/invalid.
- `teleop_streaming_type` falls back to `streaming_type` when empty/invalid.
- `startup_mode` falls back to `minimap` when empty/invalid.

Allowed transport values:
- `ros`, `webrtc`.

Allowed startup values:
- `minimap`, `teleop`.

## Visualizations

`visualizations`:
- Robot-scoped only.
- Any global visualization (for example occupancy grid) is excluded from this list.

`global_visualizations`:
- Global-scoped visualizations.
- Dedupe key: `(type, topic, frame)`.
- First occurrence wins; order of first appearance is preserved.

Occupancy payload:
- `occupancy.show_unknown_space`: bool coercion.
- `occupancy.position_scale`: float coercion.
- `occupancy.position_offset`: vec3 coercion from dict/list.
- `occupancy.rotation_offset_euler`: vec3 coercion from dict/list.

## Robot Manager Config

`robot_manager_config` defaults:
- `enabled = true`
- `prefab_asset_path = "Assets/Prefabs/UI/RobotManager.prefab"`
- `prefab_resource_path = ""`
- `sections.status = true`
- `sections.data_viz = true`
- `sections.teleop = true`
- `sections.tasks = true`

## Teleoperation Control Contract

`control.drive_topic`:
- Default: `/<robot>/cmd_vel`.
- If `robot.metadata["teleop_config"]["command_topic"]` is set, `drive_topic` mirrors it.

`control.teleop` defaults:
- `enabled = true`
- `command_topic = /<robot>/cmd_vel`
- `raw_input_topic = /horus/teleop/<robot>/joy`
- `head_pose_topic = /horus/teleop/<robot>/head_pose`
- `robot_profile = robot_type` (`wheeled|legged|aerial|custom`, fallback `wheeled`)
- `response_mode = analog` (`analog|discrete`)
- `publish_rate_hz = 30.0` (clamped to `[5.0, 120.0]`)
- `custom_passthrough_only = false`

Deadman defaults:
- `policy = either_index_trigger` (`either_index_trigger|left_index_trigger|right_index_trigger|either_grip_trigger`)
- `timeout_ms = 200` (clamped to `[50, 2000]`)

Axis shaping defaults:
- `deadzone = 0.15` (clamped to `[0.0, 0.5]`)
- `expo = 1.7` (clamped to `[1.0, 3.0]`)
- `linear_xy_max_mps = 1.0`
- `linear_z_max_mps = 0.8`
- `angular_z_max_rps = 1.2`

Discrete response defaults:
- `threshold = 0.6` (clamped to `[0.1, 1.0]`)
- `linear_xy_step_mps = 0.6`
- `linear_z_step_mps = 0.4`
- `angular_z_step_rps = 0.9`

## Workspace Scale

`workspace_config.position_scale` is included only when all are true:
- value is provided
- value parses as float
- value is finite
- value is strictly greater than `0.0`

Otherwise `workspace_config` is omitted.

## Queue Status Semantics

Ack payloads with `success=true` and `robot_id` prefixed by `"Queued"` are interpreted as queued registration.

Normalized reason extraction:
- `"Queued"` -> `"Waiting for Workspace"`
- `"Queued(...)"` -> content inside parentheses
- trims optional leading `:` or `-`

Expected reasons currently exercised:
- `Waiting for Workspace`
- `Waiting for TF`
