---
title: Gaussian Splat Fixture
sidebar_position: 5
---

# Gaussian Splat Fixture

The Gaussian splat fixture is the safest way to test HORUS Gaussian Splat DataViz without changing a live robot graph. It keeps the normal HORUS workflow:

- SDK registration declares a `gaussian_splat` DataViz layer
- the fixture publishes a manifest and ROS chunk topics
- HORUS MR caches the received PLY asset
- the map is anchored under the accepted workspace map transform
- a PointCloud2 preview remains available as fallback

:::warning[Experimental renderer]

Gaussian splatting is still an experimental HORUS MR path. Validate the small fixture first, then move to denser assets only after the layer stays fixed in the workspace while moving the headset.

:::

## Files

| File | Role |
| --- | --- |
| `python/examples/tools/publish_gaussian_splat_fixture.py` | publishes the fixture manifest, ROS chunks, fallback pointcloud, TF, and odometry |
| `python/examples/gaussian_splat_fixture_registration.py` | registers two fake robots plus the global Gaussian splat DataViz layer |

## Run It

Terminal A:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH

python3 python/examples/tools/publish_gaussian_splat_fixture.py --profile small
```

Terminal B:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH

python3 python/examples/gaussian_splat_fixture_registration.py
```

Expected registration behavior:

- two fake robots appear after workspace acceptance
- `/horus/gaussian_splat/manifest` is published
- `/horus/gaussian_splat/chunk_begin`, `/chunk_item`, and `/chunk_end` carry the asset
- `/map_gaussian_splat_preview` publishes the fallback PointCloud2 layer

## Render Modes

Set `render_options["render_mode"]` in `gaussian_splat_fixture_registration.py`.

| Mode | Use it when |
| --- | --- |
| `splats` | validating the normal renderer |
| `debug_points` | checking whether the loaded asset is anchored to the workspace |
| `mono_center_eye` | isolating XR stereo matrix problems |
| `no_covariance` | checking whether splat covariance expansion causes stretching or swimming |

Start with `debug_points` if the layer appears to move with the headset. If debug points are stable but `splats` are not, the issue is in covariance, projection, or composite rendering rather than registration or ROS transport.

## Useful ROS Checks

```bash
ros2 topic echo /horus/gaussian_splat/manifest --once
ros2 topic hz /horus/gaussian_splat/chunk_item
ros2 topic echo /horus/registration_ack
ros2 topic list | grep gaussian
```

The manifest should report `visualization_type: gaussian_splat`, `asset_format: 3dgs_ply`, a SHA-256 hash, content length, chunk count, `frame_id: map`, and the fallback preview topic.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Robots appear but no splat appears | Confirm chunk topics are flowing and watch HORUS MR logs for cache/hash/load messages |
| Fallback pointcloud appears but splat does not | Keep the fallback enabled, then test `render_mode="debug_points"` |
| Splat follows the headset | Test `debug_points`, then `mono_center_eye`, then `no_covariance` |
| Dense profile never appears | Return to `--profile small` and confirm the cache path and hash validation first |
