---
title: Camera Transport Profiles
sidebar_position: 2
---

# Camera Transport Profiles

## Policy model

SDK camera payload supports:

- legacy `streaming_type`
- `minimap_streaming_type`
- `teleop_streaming_type`
- `startup_mode`

Recommended baseline:

- MiniMap = `ros`
- Teleop = `webrtc`
- Startup = `minimap`

## Example CLI

```bash
python3 python/examples/sdk_registration_demo.py \
  --robot-count 4 \
  --with-camera \
  --camera-streaming-type ros \
  --camera-minimap-streaming-type ros \
  --camera-teleop-streaming-type webrtc \
  --camera-startup-mode minimap
```

## WebRTC tuning options

```bash
python3 python/examples/sdk_registration_demo.py \
  --with-camera \
  --camera-teleop-streaming-type webrtc \
  --webrtc-bitrate-kbps 1200 \
  --webrtc-framerate 15 \
  --webrtc-stun-server-url stun:stun.l.google.com:19302
```

## Compatibility guidance

Keep `streaming_type` populated while newer profile fields are used. This preserves compatibility with older clients while enabling dual-profile behavior in current MR runtime.
