# Field-teammate demo — "On the Map, In the Team"

Represents a person in the field (wearing a HoloLens-class headset) inside the
same HORUS workspace as the robot team — as a **field teammate**, not a
controllable robot. The teammate is registered through the existing SDK
pipeline, but safety is **capability-driven and default-deny**: teleop, the
navigation tasks, and every robot-control capability are denied, and the
serializer **fails closed** if any of them are re-enabled.

| Aspect | Robot | Field teammate |
|--------|-------|----------------|
| `robot_type` | wheeled / legged / aerial / drone | `human` |
| `entity_kind` | `robot` | `field_teammate` |
| controllable / teleoperable / taskable | true | **false** |
| guidable / observable / communicative | guidance off | **true** |
| MR affordances | teleop + tasks | guidance + view-cone + FPV + confidence |

## What this demo exercises

- **`FieldTeammate` wrapper** — constructs a `RobotType.HUMAN` entity, stamps the
  capability default-deny contract, and disables teleop + navigation tasks.
- **Capability contract in the payload** — `entity_kind`, `capabilities`, and
  `field_teammate_config` are emitted by the registration serializer. The MR
  runtime gates command paths on the capabilities, never on `robot_type`.
- **Fail-closed serializer** — re-enabling teleop or a task (even by mutating
  metadata directly) raises `FieldTeammateSafetyError` before registration.
- **First-person view** — an optional compressed-over-WebRTC FPV camera on the
  teammate's `camera` frame.
- **Mock HoloLens feed** — a node that publishes pose/status/confidence/FPV and
  answers guidance requests, so the whole loop runs without a headset.

## Run it

All commands from the `horus_sdk` repo root, ROS 2 sourced.

```bash
# 1) Register the teammate with HORUS MR.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py
#    --name field_teammate_1     teammate entity name/namespace
#    --wearable hololens2        hololens2 | aria | quest_pro | generic
#    --no-fpv-camera             register without the FPV camera sensor

# 2) In another terminal: publish an offline HoloLens feed (pose, status,
#    confidence, FPV) and auto-answer operator guidance requests.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/mock_field_teammate_node.py --name field_teammate_1
```

## Validate offline (no ROS graph, no HORUS bridge)

Builds and prints the baked registration payload, then asserts the default-deny
contract holds:

```bash
PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py --dry-run
```

Expected tail:

```
  "entity_kind": "field_teammate",
  "capabilities": { "controllable": false, "teleoperable": false, "taskable": false, ... }
  ...
DRYRUN_OK (default-deny contract verified)
```

## Topic contract

The mock node and the SDK derive these from the same `FieldTeammateConfig`, so
they never drift. For `--name field_teammate_1`:

**Teammate publishes:** `/tf` (`map → field_teammate_1/base → field_teammate_1/camera`),
`/field_teammate_1/status`, `/field_teammate_1/localization_confidence`,
`/field_teammate_1/fpv/image_raw/compressed`, `/field_teammate_1/guidance/response`,
`/field_teammate_1/guidance/state`.

**Teammate subscribes:** `/field_teammate_1/guidance/request`,
`/field_teammate_1/guidance/annotation`, `/field_teammate_1/guidance/route`,
`/field_teammate_1/guidance/warning`, `/field_teammate_1/audio/message`.

## Notes

- **`RobotType` has no humanoid class**, so a teammate registers as `human`; the
  capability contract — not the type tag — is what keeps it non-controllable.
- **Confidence-gated guidance**: the mock publishes a localization confidence
  that sweeps the HIGH/MEDIUM/LOW bands so the MR side can degrade guidance
  precision safely.
- **On-device** (HoloLens companion app → ROS 2 → HORUS) is the live path; the
  mock node stands in for the headset for development and study dry-runs.
- The contract fixture is `contracts/fixtures/field_teammate_hololens.json`; the
  serialized-payload guardrails live in `python/tests/test_field_teammate_payload.py`.
