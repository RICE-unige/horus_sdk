---
title: Teach Input and Adapter Contract
sidebar_position: 13
---

# Teach Input and Adapter Contract

Teach v2 adds measured head pose, sticks, buttons and per-input validity to the
existing controller-pose workflow. It is opt-in and supported by the Python,
C++ and Rust hosts. It does not provide robot-specific inverse kinematics,
locomotion, finger retargeting, balance or a physical emergency stop.

## Enable v2

Use these fields in your existing `TeachConfig`; keep your own adapter identity,
joint list, command frame, calibration, cameras, streams and storage directory:

```python
from dataclasses import replace

config = replace(
    existing_config,
    input_schema_version=2,
    required_input_features=(
        "head_pose", "left_stick", "right_stick", "left_buttons", "right_buttons",
    ),
    clutch_binding="both_primary",
    hold_binding="right_secondary",
    grasp_bindings={
        "left_grasp": "left_grip", "right_grasp": "right_grip",
        "left_pinch": "left_trigger", "right_pinch": "right_trigger",
    },
)
```

The `grasp_bindings` keys are user-defined logical tool channels. HORUS adds
their measured values to `sample["tools"]`; their physical meaning belongs to
the adapter. A user can select other supported clutch/hold bindings and require
only the features that their profile uses. `controlled_hands` independently
declares which controller poses must remain tracked. A required control input
must not be treated as optional merely to avoid a tracking fault.

C++ accepts the same keys through `TeachConfig(recording_root, options)` or
`with_options(options)`. Rust uses `TeachConfig::from_value` or `with_options`.
Both run the native implementation, not a Python subprocess. See
[native Teach](../native-sdk/models-and-teach.md#teach-configuration-and-runtime).

Version 1 remains the default. Its normalized configuration, hash and wire
messages do not gain v2 fields. Version 2 configuration changes the hash, so
the registration and recording runtime must use the same resolved profile.
Registration schema and episode schema remain version 1; they are independent
of `input_schema_version`.

## Negotiation

### Continuous Engagement

Opt into countdown-based control with these additional configuration fields:

```python
config = replace(config, input_schema_version=2,
    engagement_mode="continuous", engage_countdown_seconds=3.0,
    disengage_binding="both_primary")
```

The default remains `hold_to_run`, preserving existing registrations and hashes.
Continuous mode requires v2 input. Countdown duration is 3-10 seconds; the stop
binding is `both_primary` (Quest X+A), `left_primary`, or `right_primary`.
The same keys and semantics are available in C++ and Rust.

In continuous mode, **Engage** requests a lease and starts the visible countdown.
No motion input is sent during the countdown. After it elapses the app requests
engagement; only the matching accepted host acknowledgement enables input.
The first accepted pose anchors control. Buttons need not be held afterwards.
X+A, **Disengage**, or the configured secondary Hold button stops control.
By default, tracking/focus/camera/authority loss, disconnect, alignment changes
and recorder failures stop control; recovery requires a new Engage. The optional
explicit tracking-recovery policy below is the only exception.
Hiding the Teach controls or switching away from their tab also cancels the
countdown and disengages continuous control. Reopen Teach and press Engage to resume.

`engagement_mode` is included in v2 open negotiation, host state and samples.
The host rejects mismatched modes. In continuous mode, `clutch=true` is the
explicitly acknowledged local engagement gate, not a claim that X+A are held.
Raw button fields are always measured unchanged. The host requires readable
stop controls and rejects an asserted stop chord. Adapters must derive the mode
from their trusted configuration and honor it too; an older adapter that
requires both primary buttons held cannot be used unchanged. Keep its local
watchdog, authority checks and robot-specific limits.

### Tracking Recovery

For a simulator or fixture, a qualified adapter can opt into waiting for tracking
without requiring another Engage:

```python
config = replace(config, provenance="simulation", input_schema_version=2,
    engagement_mode="continuous", tracking_recovery="auto_resume")
```

`tracking_recovery` defaults to `manual`; omitting it preserves existing profile
hashes. `auto_resume` is rejected for hold-to-run and input v1. The adapter must
explicitly declare `supports_tracking_pause=True` and
implement bounded, nonblocking `pause(context) -> bool`. This callback discards
pending targets and confirms measured hold **without** revoking the current
authority or resetting the anchor. False leaves `pause_requested`; True enters
`paused`. A healthy paused adapter must still report `ready=True`. Exceptions or
an actual worker/readiness fault trigger latched Hold, not automatic recovery.

Well-formed missing tracking, required-input unavailability, temporary UI focus,
an input watchdog gap or an expired transport challenge initiates pause. No
cached pose is substituted and no pending target is replayed. The host invalidates
pre-pause challenges; after confirmation, a new increasing-sequence input with a
fresh challenge, valid tracking, input focus and unchanged authority can resume.
The same input epoch and anchor are retained. Recordings continue and contain
`tracking_pause` / `tracking_resumed` events, separate from accepted intent.

Explicit Stop/Hold/X+A, clutch permission removal, lost lease, genuine worker
fault, missing required video, collection-recorder failure and lost operator
presence still latch a stop. Headset app suspension, tracking-origin changes,
alignment loss or a changed/stale host connection also require re-engagement.
This policy does not turn device tracking into a physical safety system.

Python, C++ and Rust share this simulation/fixture contract. Registration, v2 open requests, each
input sample and host state carry `tracking_recovery`; an older client that does
not negotiate it is rejected. `pause_reason` explains the paused host state.
The headset continues reporting measured validity while paused and keeps the
existing engagement intent; it never starts a new engagement automatically.

#### Physical Recovery (Python)

The Python SDK also supports physical recovery with **two independent opt-ins**:
the locally configured profile requests it, and the installed hardware adapter
declares that its controller implements it. Keep `provenance="real"`:

```python
from dataclasses import replace

# Apply to the integration's existing, validated profile and capabilities.
config = replace(config, provenance="real", input_schema_version=2,
    engagement_mode="continuous", tracking_recovery="auto_resume")
capabilities = replace(capabilities, supports_tracking_pause=True,
    supports_physical_tracking_recovery=True)
```

The adapter must already declare `qualification="hardware"`, the exact configured
joint order, command frame and calibration, an independent controller watchdog,
and controller-scoped authority. `supports_physical_tracking_recovery` defaults
to `False`; missing, truthy non-boolean or false values cannot authorize physical
auto-resume. Manual profiles still require Engage even if both capabilities are
true. No camera, lease, feedback or recording requirement is relaxed.

Implement `pause(context)` to discard queued targets and confirm the actual hold
of **all** controlled channels under the supplied authority. Report `False`
until confirmed; do not treat submitting a hold request as confirmation. Keep
the anchor while paused. The first fresh resumed `apply(sample)` must use the
controller's bounded recovery from measured hold state, preserving joint,
velocity and other robot-specific constraints. Recovery blending belongs in
the robot adapter, not in the generic SDK; HORUS does not invent a duration or
claim to verify physical safety. The adapter must report faults through
`ready()` or an exception, and must not interpret worker failure as recoverable
Quest tracking loss. Stop, ownership loss and shutdown still revoke control.

Capabilities are recorded in the episode's `adapter_contract` event; provenance
and the requested policy remain in the profile. Startup, viewing, recording and
arming do not themselves send an `apply` command. The first accepted sample
anchors without motion. Software tests use non-actuating adapter doubles and do
not qualify a physical robot. C++/Rust currently retain simulation/fixture-only
automatic recovery; this physical extension is Python-only.

### Input Capabilities

The SDK advertises `input_schema_version: 2` and `required_input_features` in
the robot's `teaching` registration. On session open the new MR app sends:

```json
{
  "input_schema_version": 2,
  "input_features": ["head_pose", "left_stick", "right_stick", "left_buttons", "right_buttons"]
}
```

These fields extend the existing request identity, owner, configuration hash,
session and revision fields. The host rejects a missing/wrong version, unknown
or duplicate capabilities, or a missing required capability before assigning
session ownership. Reordering the same feature set is harmless; changing it in
an open session is rejected. Close or owner expiry clears the negotiation.
Host state echoes the version, negotiated features and required features. MR
rejects incompatible state and stops sending control input.

An older APK cannot open a v2 profile successfully. Update both the SDK and the
app; do not weaken the profile or fabricate missing input values as a workaround.
Declared support means the client knows how to measure a feature, not that its
device is currently tracked. Each sample carries separate validity flags.

## Sample Fields

All v2 samples retain v1 identity, freshness, poses, analog values and clutch
fields. Field names below are exact and flat, not nested under a new container.

| Fields | Meaning |
| --- | --- |
| `input_schema_version` | Integer `2`. |
| `head`, `left`, `right` | Pose objects with `position: [x,y,z]` in metres and `rotation: [x,y,z,w]`, normalized quaternion. |
| `head_tracking_valid`, `left_tracking_valid`, `right_tracking_valid` | Boolean: position **and** rotation are tracked and finite. |
| `left_stick`, `right_stick` | Two normalized components `[x-right,y-forward]`, each in `[-1,1]`. Not robot velocity. |
| `left_stick_valid`, `right_stick_valid` | Boolean: the stick measurement was available. |
| `left_primary`, `left_secondary`, `left_stick_click` | Measured left button booleans. |
| `right_primary`, `right_secondary`, `right_stick_click` | Measured right button booleans. |
| `left_buttons_valid`, `right_buttons_valid` | Boolean: all three listed buttons on that controller were readable. |
| `left_trigger`, `right_trigger`, `left_grip`, `right_grip` | Measured analog values in `[0,1]`. |
| `left_trigger_valid`, `right_trigger_valid`, `left_grip_valid`, `right_grip_valid` | Explicit measurement validity. |
| `engagement_mode` | `hold_to_run` (default) or explicitly configured `continuous`. |
| `tracking_recovery` | `manual` (default) or negotiated `auto_resume`; physical Python adapters require a separate explicit capability. |
| `tracking_valid`, `clutch`, `ui_focus` | Overall tracking gate, local control permission (held clutch or acknowledged continuous engagement), and conflicting UI focus. |
| `session_id`, `owner`, `input_epoch`, `sequence`, `challenge` | Existing session/authority association, increasing sequence and fresh host challenge. |
| `alignment_id`, `tracking_origin_id` | Current host-acknowledged alignment and XR-origin UUIDs. |

An untracked pose has its validity flag false. Unity JSON may represent an
unavailable pose as empty/null fields; never interpret it as a measured origin.
Unavailable sticks/analogs may contain zero with validity false. Valid zero
means a real measured neutral value; the flag distinguishes these cases.

## Frames and Time

- Raw headset input uses `frame_id="quest_tracking_flu"`: forward, left, up.
  The host applies the acknowledged tracking-to-calibrated transform exactly
  once, then passes `frame_id="teach_calibrated_floor_flu"` and
  `source_frame_id="quest_tracking_flu"` to the robot adapter. Do not apply
  the alignment a second time in an adapter.
- All three poses use the same floor-level root established by explicit
  **Align** and a positive host acknowledgement. Opening Teach starts video
  viewing immediately but never aligns or engages automatically. A real tracking
  reference change/recenter, connection/session or registration-profile replacement
  invalidates alignment and requires **Align** again. Ordinary head motion and
  brief tracking loss in the same reference space do not erase acknowledged
  alignment. Video viewing does not depend on control readiness or floor calibration.
  Repositioning the same Unity XR rig in the virtual world only rebases the
  avatar/camera presentation; it does not change the raw XR calibration. Replacing
  the tracking-space object, selecting a different native reference space, or a
  genuine native-origin delta still requires alignment. An identity notification
  for the selected space and notifications for unselected spaces do not invalidate
  it. An explicit recenter always invalidates, including when its mode is unchanged.
  The frame does not follow the moving head. Head yaw is not subtracted from
  hand poses. Head-relative retargeting, axis remapping and robot-base transforms
  belong in the adapter and must use its calibration.
- Unity right/up/forward position maps to `[z,-x,y]`; its relative quaternion
  maps to `[-z,x,-y,w]`. The robot embodiment is full scale independently of
  workspace scale. Camera panel positions remain calibrated Unity-space metres.
  Pose conversion ignores visual transform scale; raising a controller by 0.5 m
  raises its reported FLU z by 0.5 m even if a visual parent is scaled.
- `source_monotonic_seconds` is Unity's process-monotonic sample time;
  `clock_id` is `quest_monotonic` and `pose_time_semantics` is
  `unity_xr_latest_available`. This is not sensor exposure time or Unix time.
  Do not subtract it from a ROS or host timestamp without clock calibration.
- On Quest, native reference-space/recenter events distinguish an actual origin
  change from a same-mode tracking notification. Other XR providers without those
  native signals retain fail-closed handling of origin-update notifications.
- App suspension and a short recorder-status delay stop control but do not alone
  erase calibration. Returning to fresh, owned state still requires explicit
  **Engage** after a latched stop. A disconnected/replaced session, real origin
  change or incompatible host state requires **Align** as well. Invalid/missing
  tracking never replays a previously valid pose, and no lease or watchdog is bypassed.

### Alignment Protocol

After `open`, send the existing request envelope with `action="align"` and:

```json
{
  "alignment": {
    "schema_version": 1,
    "alignment_id": "00112233-4455-4677-8899-aabbccddeeff",
    "tracking_origin_id": "10213243-5465-4768-899a-abbccddeeff0",
    "frame_id": "quest_tracking_flu",
    "child_frame_id": "teach_calibrated_floor_flu",
    "position": [0, 0, 0],
    "rotation": [0, 0, 0, 1]
  }
}
```

The IDs above are illustrative; generate new canonical UUIDs. The transform is
`T_tracking_from_calibrated`, with metric position and an XYZW quaternion. It is
finite, normalized and gravity-aligned (yaw only, within one degree tolerance).
The host rejects alignment while control is active or a hold is unconfirmed.
State advertises `alignment_schema_version=1`, `alignment_required=true` and the
current IDs. Engage must include the current `alignment_id`; every raw input
must include both IDs. `invalidate_alignment` stops control and clears the proof.
An old host without the acknowledgement contract cannot enable the updated app.

Requests may include optional `client_diagnostics` containing `build_guid`,
`app_version`, `tracking_origin`, `tracking_origin_id` and `invalidation_reason`.
These are recorder evidence only, never authorization or a replacement for the
acknowledged alignment. Python, C++ and Rust hosts retain known string fields
up to 256 UTF-8 bytes each, ignore malformed/unknown fields, and do not let
diagnostic validation block a valid stop request. Alignment and recording-start
events retain the session's client-build context, including when Align precedes
Record. Client invalidation events retain the exact client reason alongside the
existing generic host reason. Older clients without metadata remain supported;
their build/callback provenance is unknown and must not be inferred retroactively.

The full-scale avatar is presentation, not a new localization estimate. Its
floor placement comes from loaded visual bounds; workspace scale and cosmetic
map lift do not alter input coordinates or measured camera extrinsics.

### Measured Spatial Views

```python
from horus.teaching import TeachConfig, TeachPointCloud, TeachStream

config = TeachConfig(
    recording_root="/absolute/dataset",
    streams=(TeachStream("front_depth", "/robot/front/points",
                         "sensor_msgs/msg/PointCloud2", required=False),),
    spatial_views=(TeachPointCloud("front_depth", point_size_m=0.004,
                                  max_points=1_000_000, max_age_seconds=1.0),),
)
```

Register the actual model base frame and camera TF, including the optical-frame
rotation. Up to two unique spatial views are supported. The app uses the measured
XYZ/color data and the camera-to-model-base TF to render a metric, binocular
point cloud beside the full-scale avatar. This is not RGB plane stretching,
monocular estimated depth, an inferred camera mount or dense surface completion.
Use measured calibration where available and label nominal URDF mounts honestly
in the stream's calibration metadata.

The optional **Depth: Off / On** control starts off. Alignment does not enable it
automatically. This control only subscribes/renders the headset depth view; it
does not change host-side recording of a declared stream. **Align** also places
the camera windows at the operator's eye level, preserving their registered
relative layout, independently of avatar visibility and the floor-based control
frame. Live video remains visible while waiting for recorder health; engagement
still requires the normal camera and recorder readiness checks.

Each Teach camera's `width_m` defines its physical panel width; there is no
smaller wrist-camera role override. Set the same width for front, left wrist and
right wrist when all three should have equal prominence. Panel height follows
the actual camera aspect ratio, preserving image proportions rather than
stretching unlike source resolutions. Positions remain independently configurable.

No points are silently downsampled within the configured limit; over-budget
updates are rejected. Missing alignment, missing TF or stale data hides the
cloud. Freshness uses local monotonic receipt including transport-queue and
upload delay, not subtraction of unsynchronized source stamps. Static relative
TF does not expire because global localization is stale; dynamic relative edges
must be fresh. Optional views do not gate control, but a required recording stream
still follows the normal recording-health contract. Original observations and
source timestamps are recorded unchanged. GPU/memory capacity remains device-
and workload-dependent; the point limit is not a frame-rate guarantee.

## Control and Freshness

Open, registration, alignment and recording do not command motion. **Engage**
requires a current positive robot lease, a ready adapter, fresh required
cameras, acknowledged current-session alignment, and healthy recording for a collection profile. The app requires
release then re-press of the configured clutch in `hold_to_run` mode. The first accepted sample
anchors measured state; subsequent samples call the adapter.

With the shown hold-to-run Quest profile, X and A together are the clutch; B requests Hold.
With continuous mode, X+A stops instead; pressing them never starts control.
There is no built-in A-to-quit or Damp action. Grips and triggers remain separate
grasp/pinch measurements. Stick locomotion is not enabled implicitly.

The host validates version, frame/clock labels, finite normalized poses, axis
ranges, strict boolean flags, required tracking/features, measurable clutch and
Hold controls, mode-specific clutch/stop agreement, and each configured grasp axis. It
rejects malformed samples before adapter callbacks and requests Hold. Required
tracking loss is never replaced with a previously valid pose. Duplicate or
out-of-order sequences and old epochs cannot drive the adapter.

Each state update supplies a short-lived challenge. Input timeout defaults to
200 ms (configurable 50-500 ms), evaluated using host monotonic receipt time;
this bounds transport staleness without pretending clocks are synchronized.
The control mailbox is latest-only. Lease expiry, stale transport, clutch
release after engagement (hold-to-run), the stop chord (continuous), UI focus or recorder failure in collection mode
also request Hold. Operator presence expires after five seconds without a
valid heartbeat/input. Re-engagement requires fresh authority and a new epoch.
For explicitly configured simulation recovery, only the transient conditions
listed above pause/resume the existing epoch instead; explicit stop and genuine
fault conditions retain the default latched behavior.

Adapters receive poses transformed once into `teach_calibrated_floor_flu`, with
`source_frame_id` identifying the raw tracking frame, plus `tools` and `authority`.
Measured buttons, analog values, validity flags and timestamps remain unchanged.
They must independently enforce scoped authority and a controller-local
watchdog. `hold(context)` must report true only when its declared fallback is
confirmed; false remains `hold_requested`, not a successful stop. Callbacks
must be bounded and nonblocking. SDK checks cannot qualify a physical controller.

## Recording and Qualification

Accepted engaged samples are saved as `operator_intent` events with all v2
fields. Held/viewing samples are not a continuous raw-XR recording stream.
Adapter results, lease/hold events, measured robot feedback and configured
camera streams provide separate evidence. A return value must state the actual
observable command boundary; publication is not execution.

Use [the v2 synthetic demo](teaching-demo.md#v2-input-profile) for software
rehearsal and [the native reader](teaching-data.md) for capture validation.
Native MCAP remains the recording format. No viewer, training framework,
robot repository or cloud service is required. A real robot adapter, its
watchdog/limits, camera calibration and any GR00T/LeRobot export recipe need
their own qualification; they are not provided by the synthetic fixture.
