---
title: Field Teammate
sidebar_position: 6
---

# Field Teammate

A **field teammate** is a person represented inside the HORUS workspace alongside
the robot team — for example, someone in the field wearing a HoloLens. They are
registered through the same pipeline as a robot, but they are a *guidable,
non-controllable* entity. Safety is **capability-driven and default-deny**: the
MR runtime gates every command path on explicit capability flags rather than
inferring permission from the robot type.

This is the SDK foundation for the "On the Map, In the Team" line of work: an
operator can see a teammate's pose, heading, view cone, first-person video, and
localization confidence, and send spatial guidance — but can never *drive* a
human.

## The contract at a glance

| | Robot | Field teammate |
|--|-------|----------------|
| `robot_type` | wheeled / legged / aerial / drone | `human` |
| `entity_kind` | `robot` | `field_teammate` |
| `controllable` / `teleoperable` / `taskable` | `true` | **`false`** |
| `guidable` / `observable` / `communicative` | guidance off | **`true`** |
| MR affordances | teleop + navigation tasks | guidance + view cone + FPV + confidence |

## Minimal example

```python
from horus.robot import FieldTeammate

teammate = FieldTeammate("field_teammate_1")
success, result = teammate.register_with_horus(keep_alive=False)
```

`FieldTeammate` constructs a `RobotType.HUMAN` entity, stamps the default-deny
capability contract, records the wearable/topic contract, and disables teleop
and the navigation tasks. The HoloLens is the first supported wearable; pass
`wearable_type="aria"` (or `quest_pro` / `generic`) for others.

## Customizing the contract

```python
teammate = FieldTeammate(
    "scout",
    wearable_type="hololens2",
    first_person_video_topic="/scout/head_cam/compressed",
    guidance_request_topic="/scout/orders",
)
```

Any topic left unset falls back to a per-entity default derived from the
teammate's name (for example `/scout/guidance/response`). You can also turn an
existing `Robot` into a teammate with `robot.configure_field_teammate(...)`.

## Capability-driven safety

Safety does **not** come from the `human` type tag — it comes from the
capability flags, and it is enforced twice:

1. **At construction.** `FieldTeammate` / `configure_field_teammate` force
   `controllable`, `teleoperable`, and `taskable` off and disable teleop and the
   navigation tasks.
2. **At serialization (fail-closed).** The registration serializer re-checks the
   built payload and raises `FieldTeammateSafetyError` if a field teammate would
   expose teleop, a navigation task, or a robot-control capability — even if
   metadata was mutated directly afterwards.

```python
from horus.bridge.registration_payload import FieldTeammateSafetyError

teammate = FieldTeammate("ft")
teammate.configure_teleop(enabled=True)  # tamper
# Building the registration payload now raises FieldTeammateSafetyError.
```

The serialized payload carries `entity_kind`, a `capabilities` block, and a
versioned `field_teammate_config` (`contract_version: "field_teammate.v1"`) with
the wearable, frames, topics, and interaction permissions (acknowledge / clarify
/ reject / complete).

## Exercising it without a headset

The mock companion node publishes everything a HoloLens app would — pose (TF),
status, localization confidence, a first-person frame — and answers operator
guidance requests. It derives its topics from the same `FieldTeammateConfig`, so
it never drifts from the contract:

```bash
# Validate the contract offline (no app, no bridge):
PYTHONPATH=python python3 python/examples/field_teammate_registration.py --dry-run

# Publish an offline teammate feed (ROS 2 sourced):
PYTHONPATH=python python3 python/examples/tools/mock_field_teammate_node.py --name field_teammate_1
```

See `python/examples/FIELD_TEAMMATE_DEMO.md` for the full walkthrough.

## Study analysis scaffolding

`horus.experiments.field_teammate_study` provides a hardware-independent harness
for the three-condition study (voice+video → on-the-map → in-the-team). It
records study events as newline-delimited JSON and reduces per-dyad sessions into
per-condition summaries with t-based 95% confidence intervals, treating the dyad
as the unit of analysis.

```python
from horus.experiments import (
    FieldTeammateStudyRecorder,
    StudyCondition,
    build_study_report,
    load_sessions,
)

with FieldTeammateStudyRecorder("dyad1.ndjson", dyad_id="dyad1",
                                condition=StudyCondition.IN_THE_TEAM,
                                scenario="search") as rec:
    rec.session_start()
    rec.guidance_request("g1")
    rec.guidance_response("g1", "acknowledge")
    rec.session_end()

report = build_study_report(load_sessions(["dyad1.ndjson"]))
```

## Native parity

The capability contract is at parity in the C++ and Rust SDKs:
`make_field_teammate` / `Robot::field_teammate`, the `entity_kind` /
`capabilities` / `field_teammate_config` payload fields, and a
`validate_field_teammate_safety` fail-closed check. All three are verified
against the shared `contracts/fixtures/field_teammate_hololens.json` fixture.
