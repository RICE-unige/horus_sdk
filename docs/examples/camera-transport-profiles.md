---
title: Camera Transport Profiles
sidebar_position: 2
---

# Camera Transport Profiles

:::tip[Use this after the camera tutorial]

This page is reference-oriented. If you are learning why the camera settings exist, start with [Tutorial 2: Cameras and views](../tutorials/cameras-and-views.md) and return here when you need to choose an exact transport profile.

:::

The SDK supports separate camera transport policy for projected view, minimap view, and teleop view. This is critical in HORUS because awareness and control do not have the same transport requirements.

## Fields that matter

| Field | Purpose |
| --- | --- |
| `streaming_type` | compatibility fallback transport |
| `minimap_streaming_type` | transport used by the minimap camera panel |
| `teleop_streaming_type` | transport used by teleop view |
| `startup_mode` | initial camera mode the app should prefer |
| `minimap_image_type` | image encoding for the minimap path |
| `teleop_image_type` | image encoding for the teleop path |

## Separate WebRTC media profiles

Keep one camera identity while selecting different overview and teleop budgets:

```python
camera.configure_webrtc_transport(
    bitrate_kbps=1500,
    framerate=15,
    minimap_bitrate_kbps=1500,
    minimap_framerate=15,
    teleop_bitrate_kbps=3000,
    teleop_framerate=30,
    minimap_priority=1,
    teleop_priority=4,
)
```

These optional fields are also available on C++ `Camera::WebRtcTransportOptions` and
Rust `WebRtcTransportConfig`. Unspecified values preserve the existing shared
bitrate/FPS setting. FPS requests are 1-60; they cannot make a source publish
faster than it actually acquires frames. Resolution and camera calibration
are unchanged. Congestion adaptation may lower compression bitrate; record
actual delivered quality and frame rate rather than reporting requested FPS.
Priorities are optional integer weights 1-16. Unspecified values use the MR
defaults (overview 1, teleop 4). They affect an explicitly configured shared
link budget, not the robot's control lease or a network/router setting.

When the native sender advertises profile updates, switching projected,
minimap or immersive views of the same stream reuses its WebRTC session.
An explicit ROS profile remains ROS, including the overhead teleop view.
The hospital remote registration uses native WebRTC for both overview and
immersive views, with separate 15/30 FPS requests.

With the optional native connector relay, already encoded video stays in the
native RTP path between the source and Quest. It is not decoded/re-encoded or
republished as an operator-PC ROS image. See the connector's
`docs/performance.md` for native relay, fleet control and isolated GCC setup.

## Recommended profiles in current examples

### Ground ops

- example: `python/examples/ops_registration.py`
- policy: ROS for minimap, ROS for teleop
- use when: fake-runtime development, general validation, simple local setups

### Carter live workflow

- example: `python/examples/carter_registration.py`
- policy: ROS for minimap, WebRTC for teleop
- use when: live fleets where teleop needs a lower-latency camera path

### Stereo workflow

- example: `python/examples/stereo_registration.py`
- policy: mono minimap plus stereo teleop
- use when: you want a headset teleop view that is richer than the low-cost awareness stream

## Practical guidance

- keep `streaming_type` populated for compatibility
- set minimap and teleop transport explicitly in new examples
- tune projected and minimap view geometry with the same care as the transport fields
- document whether a given live workflow needs WebRTC support before someone treats it as ROS-only
