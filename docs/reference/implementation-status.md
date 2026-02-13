---
title: Implementation Status
sidebar_position: 1
---

# Implementation Status

## Python track

Status: **Production baseline**

- registration payload serialization
- camera transport profiles
- global visualization payloads
- workspace scale forwarding
- dashboard/topic monitoring

## C++ track

Status: **Parity in progress**

- payload builders and registration flows advancing toward Python parity
- examples and benchmark scaffolds exist
- parity validation is CI-backed but still evolving

## Rust track

Status: **Parity in progress**

- typed payload pathways and fixtures implemented
- monitoring helpers and examples expanded
- parity tests continue to track Python contract

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
