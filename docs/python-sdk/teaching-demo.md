# Humanoid Teach Demo

For opt-in stereo windows with feathered edges, see
[Teach Camera Portals](./teaching-camera-portals.md). The normal demo stays flat
unless `--front-portal` is supplied.

## Continuous Control

For countdown engagement without holding X+A, run:

```bash
cd ~/horus_sdk
./teach-demo --continuous
```

This selects the v2 synthetic profile. Open **Teleop > Teach**: your local
workspace/minimap parks to the left, while the Robot Manager opens at your
right and stays fixed in the room. Use its top grab handle to reposition it
with a near or distance grab while disengaged. The buttons do not drag the panel.
During the countdown grabbing is locked; on engagement the panel parks once
at your right and remains stationary, even when you move your head. After
disengagement is confirmed, grabbing is available again in the same position.
The full-scale avatar and camera views stay ahead. This local parking
also applies in shared workspaces without relocating other operators' views;
exiting Teach restores the original layout.

Select **Record**, then **Engage**. Once the lease and required inputs are ready,
the panel counts down three seconds before requesting engagement. Keep the
controllers in a comfortable starting pose. Movement remains enabled without
holding buttons until **Disengage**, X+A, B, or a safety condition stops it.
To resume, select **Engage** again; releasing the stop buttons does not resume.
The existing no-flag demo below remains hold-to-run.

If control stops unexpectedly, the headset logs the specific local condition or
recorder transition. Recorder state includes `last_hold_reason`, which survives
subsequent healthy heartbeats. Continuous mode has no fixed engagement duration;
it still requires tracked inputs, fresh required video, a valid lease and a
responsive controller. A stop never automatically resumes motion.

State also exposes `input_health` (latest received/accepted sequence, accepted
input age in milliseconds, and callback lock wait) and `last_hold_input`, the
snapshot retained at a stop. The headset logs its largest input publish gap
when the control state changes. These distinguish missing or delayed delivery
from an input rejected by the host; they do not measure motor response latency.
The hold snapshot is not overwritten by late packets or healthy heartbeats.
Do not increase the input timeout to conceal a transport fault.

This hardware-free example exercises the HORUS Teach workflow: humanoid
registration, life-size embodiment, three live camera panels, controller-driven
arms and grippers, per-robot leases and independent saved episodes.

The robot is a self-authored articulated model. Its RGB images are animated
diagnostic views, **not rendered robot observations**. Kinematics are synthetic,
with no dynamics, collision avoidance, balance controller or hardware command
publisher. Every episode is marked `provenance: fixture`; these recordings are
for checking the software, not training a policy or claiming robot performance.

## One-Time Setup

Requires ROS 2 Jazzy, the current HORUS bridge and a Teach-enabled HORUS MR build.
From the SDK checkout, create an isolated environment if it is not already set up:

```bash
python3 -m venv --system-site-packages ~/.cache/horus/teach-demo-venv
~/.cache/horus/teach-demo-venv/bin/python -m pip install 'mcap==1.3.1' 'scipy>=1.11,<2'
```

The checkout's SDK is used directly. The example mesh is included; no model
download, proprietary robot software, cloud service or dataset viewer is needed.

## Launch

Stop any other SDK demo/bridge using ports 10000 and 10001, then run:

```bash
cd ~/horus_sdk
./teach-demo
```

The launcher sources ROS Jazzy and `~/horus_ws/install/local_setup.bash`, selects
Fast DDS and local-only ROS domain **58**, starts the synthetic publishers and
recorder, and lets the SDK start the normal bridge. Do not start a ROS daemon or
bridge manually. ROS local-only discovery does not prevent the Quest connecting
to the bridge over LAN. Use the PC's LAN address shown in the SDK dashboard and
port **10000**. The second transport lane uses port 10001 automatically.

The demo waits for your workspace; there is no fixed run duration. To choose a
recording folder, use `./teach-demo --recording-root /absolute/folder`. A different
isolated ROS domain can be selected with `HORUS_TEACH_DOMAIN_ID`, and an existing
Python environment with `HORUS_TEACH_PYTHON`.

## In the Headset

1. Connect HORUS to the PC and create/open a workspace. Wait for
   `teach_synthetic` and its colored robot body to appear.
2. Open its **Robot Manager > Teleop > Teach**. Wear the headset with both
   controllers tracked and a configured floor boundary.
3. Camera screens open immediately. Stand in the intended orientation, then
   select **Align** while control is disengaged. Wait for the host acknowledgement
   `Aligned | Robot scale: 100%` before engaging; there is no automatic initial
   alignment. The full-scale copy uses the robot model's visual floor bounds.
   Check its orientation and the live-video count. If floor tracking
   is unavailable, video remains available but alignment and engagement stay
   blocked. The workspace robot is not moved or reparented. **Avatar** hides
   only the life-size copy.
4. Select a task, then **Record**. Wait for `Capture: recording`.
5. Select **Engage**. HORUS requests the robot lease and waits for its grant.
   Release both grips, then hold both grips to anchor your current hand poses.
   Move your hands slowly over a small distance. The synthetic arms follow
   relative motion; trigger values control the two grippers.
6. Release either grip to hold. **Hold** or the right controller's secondary
   button also requests hold. To resume, select **Engage** again and re-anchor.
   Limits, stale streams, tracking loss, lease loss or recorder failure stop
   collection-mode motion rather than continuing with invalid data.
7. Select **Success**, **Failure** or **Unknown**, then **Finish & Save**.
   Wait for `Capture: saved` and inspect `Quality`. Saving requests hold first.
   Task outcome and recording quality are independent.
8. Select **Record** for a new episode, or **Exit** after saving. Stop the terminal
   with Ctrl+C when finished. An interrupted recording is not relabeled complete.

If a reach/speed limit causes `faulted`, release the grips and select **Hold**.
After confirmed hold, **Engage** and re-anchor before making smaller movements.
If the recorder restarts or tracking origin changes, **Align** again. A missing
camera remains labeled unavailable instead of being presented as live.

An optional measured point-cloud view is configured through
`TeachConfig.spatial_views`; see [Teach input and spatial views](teaching-input.md).
It needs real source data and a connected camera-to-model-base TF chain. Missing
depth never produces substitute geometry. This synthetic demo does not enable a
physical depth camera or a robot controller.

If the recorder is ready but Teach shows `Video: 0/N live`, check delivery to
the headset rather than only the source publisher. The current HORUS bridge
accepts both best-effort sensor-data and reliable publishers for ROS
`sensor_msgs/Image` and `sensor_msgs/CompressedImage` topics. An older bridge
can reject best-effort cameras with `RELIABILITY_QOS_POLICY`; rebuild/restart
that bridge before retrying. Required camera previews must be fresh on the
headset before **Engage** can proceed. Recorder readiness alone is insufficient.

Teach waits for a request-channel acknowledgement during connection setup.
After a connection changes, repeat unfinished actions explicitly; an old
engagement or recording request is not automatically replayed into a new
connection.

## V2 Input Profile

To include measured head pose, sticks, raw buttons and explicit per-input
validity, use a v2-enabled MR build and run:

```bash
cd ~/horus_sdk
./teach-demo --input-schema-version 2
```

The workflow is unchanged except for the controls: release then hold **X and A**
together to clutch, and use the **grips** for the synthetic grippers. Release
either primary button or press **B** to request Hold. Triggers are separate
pinch channels. Head pose, sticks and these channels are captured in accepted
operator-intent events; this stationary fixture does not walk or implement
head-relative retargeting. Its simple grippers illustrate grasp, not a physical
dexterous hand. An old APK is rejected rather than given invented inputs.

See [Teach Input and Adapter Contract](teaching-input.md) for exact field names,
frames, validity, capability negotiation and the Python/C++/Rust configuration.

## Saved Data

Default location: `~/horus-data/teach-synthetic/<episode-uuid>/`.
Each episode contains `data.mcap`, `manifest.json` and recovery metadata.
It records three RGB streams (320 x 240, nominal 10 Hz), measured synthetic
joints and TF (nominal 30 Hz), accepted target events, controller intent,
lease/anchor/hold events and timing. Rates are targets, not measured guarantees.
The camera-info topics contain fixture intrinsics, not a physical calibration.

Validate an episode without opening any viewer:

```bash
cd ~/horus_sdk
PYTHONPATH=python ~/.cache/horus/teach-demo-venv/bin/python -m horus.teaching validate \
  ~/horus-data/teach-synthetic/EPISODE_UUID
```

See [Teach Recordings](teaching-data.md) for schema, timing semantics, recovery
and the streaming reader. Use your own data tools; HORUS does not install or
require Rerun and does not include a recorded-data viewing UI. Native MCAP is
currently supported; LeRobot/RLDS exports are not implemented.

## Adapting to a Robot

The example keeps all robot-specific code in
`python/examples/tools/teach_synthetic.py`. The SDK configuration declares
camera layout, streams, joint names/units, command frame, clutch/grasp bindings,
recording location and adapter identity. No G1-specific joint layout or fixed
camera count is built into HORUS Teach.

A physical controller must implement the adapter contract, current measured
feedback, authority-scoped commands/hold, an independent watchdog and the robot's
own IK, balance and collision constraints. Merely changing the robot's name or
topics does **not** qualify this synthetic controller for real hardware.
