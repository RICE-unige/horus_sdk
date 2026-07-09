# Camera Transport Plan for HORUS MR

Status: deferred implementation note.

For the Napoli connectivity pilot, use the ROS 2 camera path only. The goal of
that pilot is to prove robot-to-cloud-to-machine topic flow before adding MR
runtime complexity.

## Decision

HORUS should use two camera paths depending on the operator mode:

- Monitoring, minimap, and projected camera views use ROS 2 image topics through
  the HORUS Connector Zenoh path.
- Immersive teleoperation uses direct WebRTC media into the HORUS Unity client.

This matches the SDK camera transport model:

- `minimap_streaming_type="ros"`
- `teleop_streaming_type="webrtc"`

## Runtime Behavior

Normal monitoring mode:

```text
robot camera topic
  -> ROS 2 / Zenoh
  -> horus_ros2 / HorusLink
  -> HORUS minimap or projected view
```

Teleoperation mode:

```text
robot camera
  -> HORUS Connector robot WebRTC encoder
  -> cloud signaling and ICE/TURN path when needed
  -> HORUS Unity WebRTC receiver
  -> direct texture rendering in MR
```

When direct WebRTC teleoperation is active for a robot, the machine-side
WebRTC-to-ROS image republisher should be stopped or suspended for that robot.
This avoids duplicate decoding work and prevents peer ownership conflicts in the
same signaling room.

When teleoperation ends, the direct WebRTC session closes and the ROS camera path
continues serving minimap and projected views.

## Why This Split

ROS 2 is better for low-rate contextual camera views because it keeps the image
inside the existing robot graph and is simple to inspect with ROS tools.

WebRTC is better for teleoperation video because it is designed for low-latency
media, jitter handling, packet loss recovery, NAT traversal, and hardware
encode/decode paths.

The intended architecture is therefore:

```text
ROS 2 / Zenoh: state, TF, maps, minimap camera, commands, metadata
WebRTC: immersive teleoperation camera stream
```

## Implementation Work

Future work should add one clean direct-HORUS WebRTC integration path:

1. Add a HORUS Unity receiver mode that can join a HORUS Connector signaling
   room as the machine-side video peer, receive the robot offer, answer it, and
   render the media track directly.
2. Or add a small signaling gateway that bridges HORUS Unity's HorusLink
   signaling topics (`/horus/webrtc/client_signal` and
   `/horus/webrtc/server_signal`) to the HORUS Connector cloud signaling relay.
3. Update the Connector signaling model so minimap/monitoring tools and direct
   MR teleoperation do not compete for the same `machine` peer slot.
4. Keep the existing WebRTC-to-ROS republisher as a compatibility and debugging
   path for non-MR tools.

## Current Pilot Scope

For the Napoli pilot, keep the setup simple:

```text
Napoli robot ROS 2 topics
  -> HORUS Connector robot
  -> cloud hub
  -> HORUS Connector machine
  -> local ROS 2 topic list / echo / visualization
```

Do not depend on HORUS MR direct WebRTC for the first connectivity test.
