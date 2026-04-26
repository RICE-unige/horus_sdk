---
title: "Tutorial 2: Cameras and Views"
sidebar_position: 3
toc_max_heading_level: 2
---

# Tutorial 2: Cameras and Views

This tutorial explains how a `Camera` registration becomes three different operator experiences in HORUS MR:

- the projected camera in the workspace
- the minimap camera panel
- the teleop-facing image surface

:::note[Separate stream description from view description]

The `Camera(...)` constructor describes the ROS stream. The `configure_*_view(...)` calls describe how HORUS should render that stream for operators.

:::

## Goal

By the end of this step, you should understand:

- which camera fields describe the ROS stream itself
- which fields describe HORUS rendering behavior
- why projected-view sizing and minimap sizing are separate
- how to reason about frame alignment before you touch live robots

## Step 1: describe the stream

```python
from horus.sensors import Camera

camera = Camera(
    name="front_camera",
    frame_id="atlas/camera_link",
    topic="/atlas/camera/image_raw/compressed",
    resolution=(160, 90),
    fps=6,
    encoding="jpeg",
    streaming_type="ros",
    minimap_streaming_type="ros",
    teleop_streaming_type="ros",
    minimap_image_type="compressed",
    teleop_image_type="compressed",
)
```

### What matters here

- `frame_id`: where the camera lives in TF. If this is wrong, the projected view will be attached to the wrong part of the robot even if the image topic is correct.
- `topic`: the ROS image topic the runtime must publish.
- `resolution`, `fps`, `encoding`: descriptive metadata for the stream.
- `streaming_type`, `minimap_streaming_type`, `teleop_streaming_type`: transport choices per operator surface. The simple examples keep all of them on ROS. The Carter example moves teleop to WebRTC while leaving the minimap view on ROS.

## Step 2: describe the projected workspace view

```python
camera.configure_projected_view(
    image_scale=1.0,
    focal_length_scale=0.55,
    show_frustum=True,
    frustum_color="#E6E6E0A0",
)
```

This controls the image plane shown in the 3D workspace:

- `image_scale`: apparent size of the projected image
- `focal_length_scale`: apparent projection depth and framing
- `show_frustum`: a debugging and spatial-awareness aid

## Step 3: describe the minimap panel

```python
camera.configure_minimap_view(
    size=10.0,
    position_offset=(0.0, 2.0, 0.0),
    face_camera=True,
    rotation_offset=(90.0, 0.0, 0.0),
)
```

This controls the floating panel used in minimap teleop:

- `size`: panel dimensions
- `position_offset`: panel placement relative to the robot
- `face_camera`: keeps the panel readable from the headset
- `rotation_offset`: usually what fixes panel orientation in the mini-map

:::tip[Validate frame correctness before tuning aesthetics]

If the image is attached to the wrong frame, do not tune `image_scale` first. Fix the TF anchor first, then tune projected-view size and minimap size.

:::

## What to validate before moving on

Run the full reference workflow:

```bash
# Terminal A
python3 python/examples/legacy/fake_tf_ops_suite.py

# Terminal B
python3 python/examples/ops_registration.py
```

In HORUS MR, verify:

- the projected camera is attached to the correct robot pose
- the minimap camera panel is visible and readable
- the image orientation is correct
- the camera feels intentionally sized, not guessed

## Why this matters before live robots

If camera alignment is wrong in simulation, it becomes much harder to debug on a live system because you will not know whether the problem is:

- the ROS topic
- the TF frame
- the transport mode
- the projected-view sizing
- the teleop or minimap view configuration

## What comes next

Continue with [Tutorial 3: Teleop and tasks](operator-controls.md), where the robot becomes actionable instead of just visible.
