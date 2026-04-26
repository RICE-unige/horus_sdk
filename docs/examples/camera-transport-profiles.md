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
