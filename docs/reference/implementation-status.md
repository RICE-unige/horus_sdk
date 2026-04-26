---
title: Implementation Status
sidebar_position: 1
---

# Implementation Status

## Python track

Status: **Production baseline**

- curated registration examples for wheeled, drone, legged, stereo, robot-description, map, semantic, Carter, and Unitree workflows
- registration payload serialization with keep-alive and ACK handling
- camera transport profiles for ROS and WebRTC-backed flows
- robot-scoped and global DataViz support
- topic monitoring and dashboard state

## C++ track

Status: **Parity in progress**

- parity work is intentionally secondary to the Python integration track
- contract fixtures remain shared, but day-to-day onboarding should use Python examples

## Rust track

Status: **Parity in progress**

- payload and fixture work continues, but the Python SDK remains the reference implementation

## Current known stub/non-primary areas

Status: Stub (not usable yet)

- `python/horus/bridge/ros2.py`
- `python/horus/bridge/unity_tcp.py`
- `python/horus/topics.py`
- `python/horus/robot/status.py`
- `python/horus/robot/teleop.py`
- `python/horus/robot/task.py`
- `python/horus/robot/dataviz.py`
- `python/horus/plugins/rosbot.py`
- `python/horus/core/exceptions.py`
