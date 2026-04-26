---
title: Sensors
sidebar_position: 2
---

# Sensors

## When to use it

Use SDK sensor objects when the MR app needs to know how a camera or lidar stream should be attached, transported, and rendered.

## Minimal example

```python
from horus.sensors import Camera

camera = Camera(
    name="front_camera",
    frame_id="atlas/camera_link",
    topic="/atlas/camera/image_raw/compressed",
    resolution=(160, 90),
    fps=6,
    encoding="jpeg",
    minimap_image_type="compressed",
    teleop_image_type="compressed",
)
```

## Realistic example

```python
from horus.sensors import Camera, LaserScan

front_camera = Camera(
    name="front_camera",
    frame_id="carter1/front_stereo_camera_left_optical",
    topic="/carter1/front_stereo_camera/left/image_raw/compressed",
    resolution=(1280, 720),
    fps=20,
    encoding="jpeg",
    streaming_type="ros",
    minimap_streaming_type="ros",
    teleop_streaming_type="webrtc",
    startup_mode="minimap",
    minimap_image_type="compressed",
    teleop_image_type="compressed",
)
front_camera.configure_projected_view(
    image_scale=1.037,
    focal_length_scale=0.55,
    show_frustum=True,
    frustum_color="#E6E6E0A0",
)
front_camera.configure_minimap_view(
    size=10.0,
    position_offset=(0.0, 2.0, 0.0),
    face_camera=True,
    rotation_offset=(90.0, 0.0, 0.0),
)
front_camera.configure_webrtc_transport(
    bitrate_kbps=2000,
    framerate=20,
    stun_server_url="stun:stun.l.google.com:19302",
)

front_scan = LaserScan(
    name="front_2d_lidar",
    frame_id="carter1/front_2d_lidar",
    topic="/carter1/front_2d_lidar/scan",
    min_angle=-3.14159,
    max_angle=3.14159,
    angle_increment=0.005,
    min_range=0.1,
    max_range=30.0,
    point_size=0.025,
)
```

## Common failure and fix

- **Failure:** a camera panel appears with the wrong size, wrong orientation, or no image at all.
- **Fix:** validate the `frame_id`, actual image topic, and image type together. For teleop/minimap differences, also verify `minimap_streaming_type`, `teleop_streaming_type`, and the view-specific image type fields.
