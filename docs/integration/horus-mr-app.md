---
title: HORUS MR Integration
sidebar_position: 2
---

# HORUS MR Integration

## Role of `horus` MR app

The MR app consumes SDK registration payloads, enforces workspace-gated activation, and maps runtime visual behavior to operator interactions.

## End-to-end flow

```bash
# 1. backend runtime
cd ~/horus/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch horus_backend horus_complete_backend.launch.py

# 2. fake data
cd ~/horus/sdk
python3 python/examples/fake_tf_publisher.py --robot-count 4 --with-camera --publish-occupancy-grid

# 3. sdk registration
python3 python/examples/sdk_registration_demo.py --robot-count 4 --with-camera --with-occupancy-grid --workspace-scale 0.1
```

## Runtime semantics expected in MR

- startup camera mode policy is MiniMap-first by default
- transport profile fields are available for MiniMap/Teleop selection logic
- occupancy map remains hidden until workspace acceptance
- registration dedupe/batching behavior reduces accept-time freezes

## Integration checklist

- Registration ACK arrives per robot.
- First frame appears for active camera streams.
- Dashboard and MR visual state agree after workspace acceptance.
