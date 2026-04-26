---
title: "Tutorial 7: Live Robot Checklist"
sidebar_position: 8
toc_max_heading_level: 2
---

# Tutorial 7: Live Robot Checklist

This tutorial explains how to move from the fake-runtime workflow to a real ROS graph without treating the SDK examples like magic.

The reference scripts are:

- `python/examples/carter_registration.py`
- `python/examples/unitree_go1_registration.py`

:::warning[Do not start here]

Live robot integrations are the last step, not the first one. If the quickstart, camera alignment, operator controls, and DataViz layers are not already stable in a fake-runtime workflow, the live graph will only make the failure harder to diagnose.

:::

## Goal

By the end of this step, you should understand:

- what must already be stable before you attempt a live robot integration
- how the Carter and Unitree examples map the earlier tutorial concepts onto real topics
- which failures are usually SDK-contract problems versus runtime or TF problems

## The live-integration rule

Do not start from the live examples first.

A clean live registration normally comes after:

1. a working quickstart
2. a working understanding of the `Robot` model
3. correct camera registration
4. correct teleop and task registration
5. correct DataViz and robot-description behavior

If those are not stable already, the live graph will only make the failure harder to diagnose.

## Carter as a reference

The Carter example shows how the same SDK shape scales to a real multi-robot graph:

- compressed front camera
- lidar
- occupancy grid
- odometry
- global and local plans
- WebRTC-capable teleop camera path

The key lesson is that live integration is not a different model. It is the same model with real topic names, real frame names, and real transport decisions.

## Unitree as a reference

The Unitree Go1 example adds a few live-only realities:

- a real URDF and visual mesh source
- a real LaserScan stream
- legged action relays
- live collision risk derived from the scan

It is a good example of where the SDK contract ends and project-specific relays begin.

## Pre-flight checklist

Before you register a live robot, verify:

- the base frame exists and is stable in TF
- the camera frame exists and is the frame you actually want to anchor
- the image topic, encoding, and transport assumptions match reality
- teleop command topics are the real command sinks
- task topics are the real action entry points
- collision or safety overlays have a meaningful runtime source

## What the SDK should own versus what it should not own

The SDK should own:

- robot identity
- topic names
- frame names
- operator-facing control declarations
- DataViz declarations

The SDK should not try to hide:

- missing TF edges
- project-specific command relays
- transport bridge issues
- robot-specific high-level service logic

If a live robot needs a relay or helper node, keep that explicit like the Unitree example does.

## Use the example catalog after this page

Once you are here, the [example catalog](../examples/registration-flows.md) becomes more useful because you now understand what each registration file is actually teaching.
