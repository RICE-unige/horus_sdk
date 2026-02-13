---
title: Topic Monitoring Dashboard
sidebar_position: 3
---

# Topic Monitoring Dashboard

## Core vs data groups

- **Core topics:** registration, ack, heartbeat
- **Data topics:** TF, camera, and sensor streams

## Link/Data semantics

| Condition | Link | Data |
|---|---|---|
| App disconnected | OFF | IDLE |
| App connected, no subscription | OFF | IDLE |
| Subscription active, publisher present | ON | ACTIVE |
| Subscription active, publisher missing/silent | ON | STALE |

## Example run

```bash
python3 python/examples/fake_tf_publisher.py --robot-count 4 --with-camera
python3 python/examples/sdk_registration_demo.py --robot-count 4 --with-camera --keep-alive
```

## Validation checklist

1. Disconnect app: all data rows return to `IDLE`.
2. Connect app pre-workspace: rows should not falsely report active data.
3. Accept workspace + publishers active: data rows move to `ACTIVE`.
