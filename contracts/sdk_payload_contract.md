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

