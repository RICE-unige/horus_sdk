# Field-teammate Demo: On the Map, In the Team

Represents a person in the field, wearing a HoloLens-class headset, inside the
same HORUS workspace as the robot team. The teammate is registered through the
existing SDK pipeline as a field teammate, not as a controllable robot.

Safety is capability-driven and default-deny: teleop, navigation tasks, and
robot-control capabilities are denied, and the serializer fails closed if any
of them are re-enabled.

| Aspect | Robot | Field teammate |
|--------|-------|----------------|
| `robot_type` | wheeled / legged / aerial / drone | `human` |
| `entity_kind` | `robot` | `field_teammate` |
| controllable / teleoperable / taskable | true | false |
| guidable / observable / communicative | guidance off | true |
| MR affordances | teleop + tasks | guidance + view-cone + FPV + confidence |

## What this demo exercises

- `FieldTeammate` wrapper: constructs a `RobotType.HUMAN` entity, stamps the
  capability default-deny contract, and disables teleop plus navigation tasks.
- Capability contract in the payload: `entity_kind`, `capabilities`, and
  `field_teammate_config` are emitted by the registration serializer. The MR
  runtime gates command paths on capabilities, never on `robot_type`.
- Fail-closed serializer: re-enabling teleop or a task, even by mutating
  metadata directly, raises `FieldTeammateSafetyError` before registration.
- First-person view: an optional compressed ROS FPV camera on the teammate's
  animated head/camera frame. The mock uses a real cached walking video by
  default, not generated placeholder imagery.
- Human body model: the SDK advertises a profile (`height`, `sex`,
  `body_model`) and leaves the default body rendering to the MR runtime. This is
  the intended path for Meta Avatars or another headset-side human renderer.
  The local MakeHuman mesh modes are explicit debug fallbacks only.
- Mock HoloLens feed: a node that publishes pose/status/confidence/FPV and
  answers guidance requests, so the whole loop runs without a headset. The mock
  uses ROS compressed images; the live HoloLens path can switch to WebRTC later.

## Run It

All commands from the `horus_sdk` repo root, ROS 2 sourced.

```bash
# One-command offline demo: starts the mock pose/FPV feed, then registers the
# teammate with HORUS MR.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py --mock-feed
#    --name field_teammate_1      teammate entity name/namespace
#    --wearable hololens2         hololens2 | aria | quest_pro | generic
#    --no-fpv-camera              register without the FPV camera sensor
#    --no-human-model             register without the human model
#    --profile-height 1.75        profile height in meters
#    --profile-sex unspecified    female | male | unspecified
#    --human-model-source meta_avatar
#    --workspace-scale 0.1        default workspace scale for the demo
#    --mock-feed-rate 15          mock pose/status/FPV publish rate in Hz
#    --mock-fpv-source real       real | synthetic | auto
```

For a real HoloLens companion app, or if you want to inspect the mock feed
separately, run the feed and registration in two terminals:

```bash
# Terminal 1: publish an offline HoloLens feed.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/mock_field_teammate_node.py \
  --name field_teammate_1 \
  --profile-height 1.75 \
  --profile-sex unspecified \
  --fpv-source real

# Terminal 2: register the teammate.
PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py
```

## Live HoloLens Endpoint

The HoloLens does **not** connect to `horus_ros2` or HorusLink directly. The
live path is:

```text
HORUS Lenses app on HoloLens
  -> headset pose/PV stream ports
  -> offboard relay on this machine
  -> ROS 2 topics
  -> horus_ros2 bridge
  -> HORUS MR operators
```

The HoloLens app displays the device IP and configured ports in its status
panel. The default companion-app profile exposes:

| Stream | Default port |
|--------|--------------|
| Personal video | `3810` |
| Spatial/head input | `3814` |
| Unity message queue / guidance back-channel | `3816` |

> [!IMPORTANT]
> **Ownership split:** the **mock** feed (`tools/mock_field_teammate_node.py`)
> lives in this SDK as a development fixture for the contract. The **live
> relay is owned by `horus_connector`** (`scripts/field_teammate_hololens_relay.py`,
> exposed as the `teammate` role) — the SDK ships no copy. The managed path is:
>
> ```bash
> cd ~/horus_connector
> ./horus setup teammate     # once: HoloLens host, name, video profile
> ./horus doctor teammate    # connectivity check
> ./horus launch teammate    # relay + Zenoh transport
> ```

Before running a live study trial, validate that the machine can reach the
HoloLens endpoint (direct invocation, from the connector checkout):

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/horus_connector
python3 scripts/field_teammate_hololens_relay.py \
  --name field_teammate_1 \
  --hololens-host <HOLOLENS_IP_FROM_STATUS_PANEL> \
  --check-endpoint
```

To inspect the endpoint and ROS topic contract without opening sockets:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/horus_connector
python3 scripts/field_teammate_hololens_relay.py \
  --name field_teammate_1 \
  --hololens-host <HOLOLENS_IP_FROM_STATUS_PANEL> \
  --dry-run
```

To publish the live HoloLens pose and camera stream into ROS 2 (or simply use
`./horus launch teammate`):

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/horus_connector
python3 scripts/field_teammate_hololens_relay.py \
  --name field_teammate_1 \
  --hololens-host <HOLOLENS_IP_FROM_STATUS_PANEL>
```

HoloLens runtime video profiles are selected by the relay over the UMQ control
socket:

```bash
# Keep the deployed app's built-in video config.
--video-profile app

# Tested quality profile: usable quality with lower frame rate.
--video-profile balanced        # 640x360@30, JPEG q55, app auto capture mode

# Tested high-rate profile: lowest latency target for local ROS viewing. This is the default.
--video-profile fast60          # 640x360@60, JPEG q25, VideoConferencing mode

# Tested high-resolution profile: better detail but not preferred for latency.
--video-profile hd720           # 1280x720@30, JPEG q45
```

One-command live registration path (spawns the connector relay, resolved via
`HORUS_CONNECTOR_ROOT`, default `~/horus_connector`):

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py \
  --live-hololens \
  --hololens-host <HOLOLENS_IP_FROM_STATUS_PANEL>
```

Manual overrides can be used for one-off tests (from the connector checkout):

```bash
cd ~/horus_connector
python3 scripts/field_teammate_hololens_relay.py \
  --name field_teammate_1 \
  --hololens-host <HOLOLENS_IP_FROM_STATUS_PANEL> \
  --video-profile fast60 \
  --video-mode video_conferencing \
  --video-width 640 \
  --video-height 360 \
  --video-fps 60 \
  --video-quality 25
```

Expected ROS outputs:

```bash
ros2 topic hz /field_teammate_1/fpv/image_raw/compressed
ros2 topic echo /field_teammate_1/status --once
ros2 topic echo /field_teammate_1/localization_confidence --once
ros2 run tf2_ros tf2_echo map field_teammate_1/base
```

The relay publishes the same topic contract as the mock node:
`/tf`, `/field_teammate_1/status`,
`/field_teammate_1/localization_confidence`, and
`/field_teammate_1/fpv/image_raw/compressed`. That contract is
`field_teammate.v1` — the fixture
`contracts/fixtures/field_teammate_hololens.json` is the source of truth, the
SDK freezes the topic names in
`test_field_teammate_payload.py::test_field_teammate_topic_contract_frozen`,
and the connector relay pins `FIELD_TEAMMATE_CONTRACT_VERSION` to it. The relay
publishes compressed FPV by default for framerate. Add `--raw-hololens-image`
on `field_teammate_registration.py`, or `--raw-image` on the connector's
`field_teammate_hololens_relay.py`, only when a decoded
`/field_teammate_1/fpv/image_raw` helper topic is needed for local ROS viewers.
The connector transports the compressed FPV stream and camera metadata; raw
images and `/field_teammate_1/audio/message` are local-only unless
`config/zenoh_teammate.json5` is deliberately extended.
The built-in HoloLens app stream
uses JPEG frames and Unity XR camera pose. A future `hl2ss` adapter can replace
that capture backend for hardware H.264 or research-mode sensors without
changing the ROS contract.

## Validate Offline

Builds and prints the baked registration payload, then asserts the default-deny
contract holds:

```bash
PYTHONPATH=python:$PYTHONPATH python3 python/examples/field_teammate_registration.py --dry-run
```

Expected tail:

```text
  "entity_kind": "field_teammate",
  "capabilities": { "controllable": false, "teleoperable": false, "taskable": false, ... }
  ...
DRYRUN_OK (default-deny contract verified)
```

## Topic Contract

The mock node and the SDK derive these from the same `FieldTeammateConfig`, so
they never drift. For `--name field_teammate_1`:

Teammate publishes: `/tf` (`map -> field_teammate_1/base`, body skeleton
frames under `field_teammate_1/*`, and
`field_teammate_1/head -> field_teammate_1/camera`),
`/field_teammate_1/status`, `/field_teammate_1/localization_confidence`,
`/field_teammate_1/fpv/image_raw/compressed`, `/field_teammate_1/guidance/response`,
`/field_teammate_1/guidance/state`.

Teammate subscribes: `/field_teammate_1/guidance/request`,
`/field_teammate_1/guidance/annotation`, `/field_teammate_1/guidance/route`,
`/field_teammate_1/guidance/warning`, `/field_teammate_1/audio/message`.
The current teammate Zenoh profile transports the guidance topics. Audio remains
local to the relay until the profile explicitly allow-lists it.

## Notes

- `RobotType` has no humanoid class, so a teammate registers as `human`; the
  capability contract, not the type tag, keeps it non-controllable.
- Confidence-gated guidance: the mock publishes a localization confidence that
  sweeps the HIGH/MEDIUM/LOW bands so the MR side can degrade guidance
  precision safely.
- Human model source: the default `meta_avatar` mode keeps the human body as an
  MR-side runtime asset. The SDK registration sends the teammate contract,
  profile, TF, FPV camera, guidance, status, and confidence topics, but it does
  not send an SDK body mesh. Use `skinned_profile_v1` or `makehuman_static` only
  when explicitly debugging local mesh payloads without Meta Avatars installed.
- Mock FPV source: `--fpv-source real` downloads and caches frames from
  `Walking in the sands.webm` by `sgu18ify` on Wikimedia Commons, licensed
  CC BY 3.0. Use `--mock-fpv-video` / `--fpv-video` to point at another local
  or remote field video, or `synthetic` only when a network-free placeholder is
  explicitly desired.
- Mock motion: the teammate uses a bounded random-walk with observation pauses.
  Skeleton TF, head pose, and the FPV camera frame are published so a Meta
  Avatar or future HoloLens body-tracking renderer can bind to the same contract
  without changing the registration API.
- On-device HoloLens companion app to ROS 2 to HORUS is the live path; the mock
  node stands in for the headset for development and study dry-runs.
- `horus_ros2` needs no HoloLens-specific socket. It receives the relay output
  as ordinary ROS 2 topics, exactly like robot drivers and sensors.
- The contract fixture is `contracts/fixtures/field_teammate_hololens.json`; the
  serialized-payload guardrails live in `python/tests/test_field_teammate_payload.py`.
