---
title: Quickstart
sidebar_position: 2
---

# Quickstart

This is the shortest realistic path from a source checkout to a working HORUS MR session.

:::info[What this page is for]

If this is your first time with the SDK, finish this page once, then continue with the [tutorial track](../tutorials/summary.md). The quickstart proves that the stack works. The tutorials explain how the registration model is structured and how to build your own integration cleanly.

:::

The primary registration script is `python/examples/ops_registration.py`. Its paired fake runtime still lives under `python/examples/legacy/` because that folder contains the simulator and validation tooling.

## 1. Prepare the shell

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash  # or humble
source ~/horus_ws/install/setup.bash
export PYTHONPATH=python:$PYTHONPATH
```

## 2. Start the paired fake runtime

Terminal A:

```bash
python3 python/examples/legacy/fake_tf_ops_suite.py
```

This runtime publishes TF, compressed front cameras, teleop command handling, nav-task status, odometry, paths, and collision-risk signals for a small wheeled fleet.

## 3. Register the fleet

Terminal B:

```bash
python3 python/examples/ops_registration.py
```

The SDK registration process keeps itself alive and will attempt to auto-start `horus_unity_bridge` if your current ROS shell can resolve it.

:::tip[Use quickstart as a sanity baseline]

Do not start by editing `ops_registration.py`. Run it once unchanged. That gives you a known-good baseline before you introduce your own robot names, topics, frames, or camera choices.

:::

## 4. Open HORUS MR

In the headset:

1. Launch the HORUS app.
2. Connect to the machine running `horus_ros2`.
3. Accept the workspace.
4. Select a robot and open Robot Manager.

Expected results:

- robots appear after workspace acceptance
- Robot Manager opens for a selected robot only
- the front camera appears in the projected panel and minimap camera panel
- `Teleop`, `Go-To Point`, and `Draw Waypoint` controls are active

## 5. Sanity checks

In any ROS shell:

```bash
ros2 topic echo /horus/registration_ack
ros2 topic list | grep atlas
```

If registration succeeds, you should see ACK traffic and the expected per-robot topics for `atlas`, `nova`, and `orion`.

## Next steps

- [Tutorial summary](../tutorials/summary.md)
- [Curated examples](../examples/registration-flows.md)
- [Registration guide](../python-sdk/registration.md)
- [HORUS ROS 2 integration](../integration/horus-ros2.md)
