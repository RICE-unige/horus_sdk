---
title: Runtime Flow
sidebar_position: 2
---

# Runtime Flow

## End-to-end sequence

1. SDK creates robot + sensor + dataviz objects.
2. SDK builds registration payload (robot-scoped + global visualization fields).
3. Payload published to `/horus/registration`.
4. Bridge/backend forwards/handles runtime state.
5. MR app accepts workspace, then activates runtime entities.
6. SDK dashboard reconciles link/data states from monitor + app connectivity signals.

## Canonical flow command set

```bash
# terminal 1: backend
cd ~/horus/ros2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch horus_backend horus_complete_backend.launch.py

# terminal 2: fake data
cd ~/horus/sdk
python3 python/examples/fake_tf_publisher.py --robot-count 6 --with-camera --publish-occupancy-grid

# terminal 3: registration
cd ~/horus/sdk
python3 python/examples/sdk_registration_demo.py --robot-count 6 --with-camera --with-occupancy-grid --workspace-scale 0.1
```

## Failure domains

- **SDK model/serialization issue:** malformed or missing fields in payload.
- **Bridge/runtime issue:** transport setup or topic wiring fails.
- **MR lifecycle issue:** workspace not accepted yet; entities intentionally gated.

## Operational truth source

Dashboard status is derived from the combined signal path:

- app connectivity state,
- monitor subscription linkage,
- publisher presence/activity.

This avoids false-positive “active” states during pre-workspace or disconnected phases.
