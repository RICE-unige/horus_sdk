---
title: Troubleshooting
sidebar_position: 1
---

# Troubleshooting

## No registration ACK

Check the runtime topics first:

```bash
ros2 topic list | grep horus
ros2 topic echo /horus/registration_ack
```

If nothing appears:

- confirm the app is connected to the bridge host
- confirm `horus_unity_bridge` is running
- confirm the SDK shell sourced `~/horus_ws/install/setup.bash`

## Bridge auto-start does not work

Try the registration script from a shell where both ROS and the workspace are active:

```bash
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
```

If you still need to debug startup, launch the bridge manually and rerun the SDK example.

## Camera panel is blank

Check:

- the camera topic exists and is publishing the expected image type
- the `frame_id` matches the real camera frame
- `image_type`, `minimap_image_type`, and `teleop_image_type` match the actual ROS message content

For live systems, also check whether the example expects WebRTC for teleop or ROS-only transport.

## Robot appears but does not move

Verify the command and task topics that the example registered:

```bash
ros2 topic list | grep cmd_vel
ros2 topic list | grep goal_pose
ros2 topic list | grep waypoint_path
```

The SDK can register a correct robot while the paired fake runtime or live robot graph is still missing the command subscribers.

## Global map or semantic layer is missing

Check that:

- the first `DataViz` instance in the batch is carrying the intended global layer
- the required topic is publishing
- the MR workspace has already been accepted

## Pointcloud map points are too large

Check the pointcloud render options in the registration script:

```python
render_options={
    "point_size": 0.035,
    "auto_point_size_by_workspace_scale": True,
    "min_point_size": 0.0015,
    "max_point_size": 0.012,
    "point_shape": "circle",
    "render_mode": "opaque_fast",
}
```

If the layer still looks like large flat panels, confirm the workspace scale in the registration payload and the map frame used by the PointCloud2 publisher.

## Gaussian splat does not appear

Start with ROS transport and cache state:

```bash
ros2 topic echo /horus/gaussian_splat/manifest --once
ros2 topic hz /horus/gaussian_splat/chunk_item
ros2 topic echo /horus/registration_ack
```

Then check HORUS MR logs for:

- manifest received
- cache path and hash validation
- chunk progress
- runtime PLY load complete
- active splat renderer count

Keep `pointcloud_fallback=True` while debugging. If the fallback pointcloud appears but the splat does not, set `render_mode="debug_points"` in `python/examples/gaussian_splat_fixture_registration.py`.

## Gaussian splat moves with the headset

Use the small fixture first:

```bash
python3 python/examples/tools/publish_gaussian_splat_fixture.py --profile small
PYTHONPATH=python:$PYTHONPATH python3 python/examples/gaussian_splat_fixture_registration.py
```

Then test render modes in this order:

1. `debug_points`
2. `mono_center_eye`
3. `no_covariance`
4. `splats`

Watch HORUS MR logs for `[GaussianSplatURP]` descriptor lines and `[GaussianSplat.XR]` stereo matrix lines. Plausible Quest stereo logs should show a texture-array target and a non-zero eye separation.

## WSL2 networking

If the headset cannot reach the bridge ports from WSL, use mirrored networking:

```ini
[wsl2]
networkingMode=mirrored
```

Then restart WSL:

```powershell
wsl --shutdown
```
