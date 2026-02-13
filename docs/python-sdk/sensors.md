---
title: Sensors
sidebar_position: 2
---

# Sensors

## When to use it

Use SDK sensor classes to define source topics, frames, and rendering metadata consumed by bridge/MR runtime contracts.

## Minimal example

```python
from horus.sensors import Camera

camera = Camera(
    name="FrontCam",
    topic="/robot/camera/image_raw",
    frame_id="camera_link"
)
```

## Realistic example

```python
from horus.sensors import Camera

camera = Camera(
    name="FrontCam",
    topic="/robot/camera/image_compressed",
    frame_id="camera_link",
    image_type="compressed",
    minimap_streaming_type="ros",
    teleop_streaming_type="webrtc",
    startup_mode="minimap"
)

camera.add_metadata("webrtc_client_signal_topic", "/robot/webrtc/client_signal")
camera.add_metadata("webrtc_server_signal_topic", "/robot/webrtc/server_signal")
camera.add_metadata("webrtc_bitrate_kbps", 1200)
camera.add_metadata("webrtc_framerate", 15)
```

## Common failure and fix

- **Failure:** camera transport fields conflict or are missing.
- **Fix:** keep legacy `streaming_type` plus profile fields for compatibility, and validate values are `ros` or `webrtc`.
