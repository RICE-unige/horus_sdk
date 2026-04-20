---
title: Registration Flows
sidebar_position: 1
---

# Registration Flows

Run these commands from the SDK repository when using a source checkout:

```bash
cd ~/horus_sdk
source /opt/ros/humble/setup.bash
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python
```

## Single robot flow

```bash
python3 python/examples/sdk_registration_demo.py \
  --robot-name test_bot \
  --with-camera \
  --workspace-scale 0.1
```

## Multi-robot flow

```bash
python3 python/examples/sdk_registration_demo.py \
  --robot-count 10 \
  --with-camera \
  --with-occupancy-grid \
  --workspace-scale 0.1
```

## Typical operations flow

Use this for Robot Manager, camera, teleop, and task-panel validation:

```bash
python3 python/examples/sdk_typical_ops_demo.py \
  --robot-count 10 \
  --workspace-scale 0.1
```

Pair it with:

```bash
python3 python/examples/fake_tf_ops_suite.py \
  --robot-count 10 \
  --rate 30 \
  --static-camera \
  --publish-compressed-images \
  --task-path-publish-rate 5
```

## Keep-alive flow for observability

```bash
python3 python/examples/sdk_registration_demo.py \
  --robot-count 4 \
  --with-camera \
  --keep-alive \
  --workspace-scale 0.1
```

## Payload-level E2E check

```bash
python3 python/examples/e2e_registration_check.py
```

## Notes

- Keep robot names stable when testing retries, dedupe behavior, task state, or multi-operator authority behavior.
- If the fake runtime uses explicit `--robot-names`, registration must use the same names.
- For stress tests, pair registration demos with fake runtime controls such as camera resolution variation, map options, robot counts, and task path publishing rate.
