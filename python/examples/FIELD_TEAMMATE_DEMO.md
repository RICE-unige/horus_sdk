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

### Which Quest app to open

Open **HORUS**, not **HORUS Map Lab**. Map Lab is a separate rendering benchmark;
it is not required for Field reception, mapping, adaptive viewing or registration.
Connect HORUS to the **PC's LAN address** on port `10000`, not the HoloLens address.

Normal measured/estimated viewing transfers all geometry from the selected map.
Upload time and memory safeguards are not automatic downsampling. Only the
explicit `--adaptive` mode selects coarse context outside high-detail interests.
The 3 cm and 5 cm profiles instead reconstruct coarser maps on the PC; they are
not the same geometry as a 1 cm scan.

To compare with Rerun, compare the **same source**: measured with measured, or
estimated with estimated. `--estimated-depth` selects the estimated source on
both viewers by default; `--map-source measured` explicitly selects the measured
one while both reconstructions continue to be saved. The terminal prints the
selected source/topic and effective voxel sizes.

```bash
cd ~/horus_sdk
# Complete 1 cm measured map, without inferred depth or adaptive detail.
./field-demo --mapping-profile quality --visualizer rerun
# Full estimated source, matching Rerun's initially selected estimated map.
./field-demo --estimated-depth --visualizer rerun
```

`field-demo` now selects `--mesh-render-profile combined`: the approved
distance-based display LOD and SpaceWarp profile for the map. Source voxels,
colour and complete map coverage are unchanged. LOD can approximate distant
display geometry and SpaceWarp can introduce synthesis artifacts; this is not
a claim of identical per-eye rendering. The head, camera and path do not receive
mesh display options. Use `--mesh-render-profile off` for a native baseline, or
`lod` / `spacewarp` to select only one technique. Direct invocation of the
registration script remains opt-in and defaults to `off`.

For a fresh complete 1 cm measured scan on the lab network:

```bash
./field-demo 10.0.3.42 --mapping-profile quality --mapping-memory unified \
  --mesh-render-profile combined --visualizer none
```

`unified` requires the already-prepared optional mapper overlay; it changes PC
map allocation, not map resolution. Connect normal Quest HORUS to the PC's
current LAN address on port `10000`. Do not start a second Field launch while
one is already running. The normal shutdown saves this new scan independently
of previous maps.

Stop/save the previous scan before switching commands. These commands use the
saved HoloLens address; a positional IP overrides it for one run. The unchanged
`quality` profile uses 1 cm measured / 2 cm estimated voxels. It does not promise
1 cm estimated reconstruction. Reconstruction coverage, headset delivery and
visual quality are distinct: an incomplete delivery must not be reported as a
lower-resolution reconstruction. Mesh diagnostics report `inventory_triangles`,
`resident_triangles`, `inventory_complete` and `missing_pages`. Counts can differ
temporarily while a live map changes; compare matching settled revisions.

The HoloLens does **not** connect to `horus_ros2` or HorusLink directly. The
live path is:

```text
HORUS Field app on HoloLens
  -> headset pose/PV stream ports
  -> offboard relay on this machine
  -> ROS 2 topics
  -> horus_ros2 bridge
  -> HORUS MR operators
```

### Local reception and measured-map test

On the prepared Ubuntu workstation, the launcher supplies the ROS environment,
domain20, the private sensor-client runtime and SDK bridge auto-start. No separate
bridge command is needed. The saved headset address can be overridden as the
first argument.

On this prepared workstation, omitting the address uses the machine-local
`HORUS_FIELD_HOLOLENS_HOST` saved in
`python/examples/.local_assets/field-runtime.sh`. A positional address overrides
that default for one run; for example, `./field-demo 10.0.3.42 --receive-only`.
Use this override when changing networks, or update that single saved setting
to keep the new default. The address is local configuration, not a public SDK
default. No separate ROS or bridge setup is needed.

```bash
cd ~/horus_sdk

# Check PC dependencies without opening headset connections or starting a mapper.
./field-demo --check

# First establish real tracking and camera reception; keep the terminal open.
./field-demo 192.168.1.165 --receive-only
```

The HoloLens must be awake with HORUS Field open. "Waiting for relay" is normal
until the persistent relay connects; a short port check does not keep it connected.
The HoloLens address passed to `field-demo` is the sensor source, not the Quest
bridge address. After changing networks, enter the PC's current LAN address
shown by the SDK dashboard in the Quest connection panel.
The SDK's "Waiting for Horus App" refers to the **Quest operator app**, not the
HoloLens. PC sensor reception runs independently of Quest registration.

If a background reception session was started on this workstation, stop it before
launching another demo or switching to mapping:

```bash
systemctl --user stop horus-field-reception.service
# If a background mapping session is already running, stop/save it instead:
# systemctl --user stop horus-field-mapping.service
```

For complete measured mapping, stop the receive-only launch with `Ctrl+C`, enable
Research Mode under **System > Research Mode** in HoloLens Device Portal, and reboot
the headset if requested. The installed HORUS Field build must contain its Research
Mode server and required sensor capabilities. Long-throw and remote-configuration
ports (`3805`, `3809`) must respond. Then:

```bash
./field-demo 192.168.1.165
# Or open the live Rerun dashboard instead of RViz:
./field-demo 192.168.1.165 --visualizer rerun
# Measured map plus a separate RGB-guided depth-completion map:
./field-demo 192.168.1.165 --estimated-depth --visualizer rerun --estimated-range 10
# Quest only, without opening a PC map viewer:
./field-demo 192.168.1.165 --no-view
```

### Full-coverage resolution profiles

| Profile | Measured voxels | Estimated voxels when enabled |
| --- | --- | --- |
| `quality` (unchanged default) | 1 cm | 2 cm |
| `balanced` | 2 cm | 2 cm |
| `3cm` | 3 cm | 3 cm |
| `performance` | 5 cm | 5 cm |

For a fresh **2 cm** comparison with estimated depth and Rerun:

```bash
cd ~/horus_sdk
# Stop the previous scan with Ctrl+C and wait for its save before this command.
./field-demo 192.168.1.165 --mapping-profile balanced --estimated-depth --visualizer rerun
```

Both map layers use 2 cm voxels. Complete observed coverage, RGB and tracking /
depth-validation checks are retained. The existing estimated per-image range
default remains 10 m; that is not a radius limit on the growing global map.
Stop before starting a different profile; each run gets its own new output folder.
For measured-only mapping, omit `--estimated-depth`. On Quest, use the normal
HORUS app, connect to the PC on port `10000` and create a workspace as usual.

For a fresh **3 cm** comparison, stop/save the 2 cm run first, then use:

```bash
./field-demo 192.168.1.165 --mapping-profile 3cm --estimated-depth --visualizer rerun
```

Both measured and estimated maps use 3 cm. Sensor, tracking, RGB, coverage and
range settings are otherwise unchanged; the 2 cm map is kept in its original
session folder. No new APK is required. Neither comparison changes the default.

### Full-coverage 5 cm comparison

Use the named `performance` profile to test a lighter reconstruction. It uses
**5 cm voxels**, retains RGB colours, and keeps the complete observed map: no
height cutoff, moving-radius exclusion, or map clearing. Sensor confidence,
calibration, tracking checks and accepted physical depth range are unchanged.
It is coarser geometry than the 1 cm profile, not a claim of identical fine detail.
Original recordings and earlier maps are never rewritten.

```bash
cd ~/horus_sdk
# Local dependency/profile check only; no headset connection or mapper starts.
./field-demo --check --mapping-profile performance --visualizer rerun
# Start a fresh real measured scan, with the map visible on the PC and Quest.
./field-demo 192.168.1.165 --mapping-profile performance --visualizer rerun
```

Keep the terminal open. On the HoloLens, leave HORUS Field in the foreground,
with Research Mode and camera access enabled. On the Quest, open the normal
HORUS app, connect to the PC's LAN address on port `10000`, and create a workspace.
The launcher starts the bridge, registration, camera/pose relay, travelled path,
mapper and mesh adapter; do not start a second relay or bridge manually.
Scan slowly and look at the floor, walls and objects from several viewpoints.
The terminal prints `Mapping profile: performance; measured voxels 5 cm`.

Stop with `Ctrl+C` and wait for saving to finish. The timestamped output folder
is printed on startup; on this prepared PC it is under
`/media/omotoye/New Volume/HORUS/data/field_mapping/`. `launch.json` records the
selected profile, and `scan/session.json` records the effective measured voxel
size and profile. The unchanged `quality` profile remains the default until
the comparison is accepted:

```bash
# Separate new 1 cm measured scan for comparison; stop the previous scan first.
./field-demo 192.168.1.165 --mapping-profile quality --visualizer rerun
# Optional longer-range provisional completion, separate from physical mapping:
./field-demo 192.168.1.165 --mapping-profile performance --estimated-depth --estimated-range 10 --visualizer rerun
```

With `performance --estimated-depth`, **both** measured and estimated maps use
5 cm voxels. The estimated layer still needs physical anchors and is not measured
ten-metre depth. Quest displays the selected source, not both overlapping maps;
use the measured-only command first for a clear profile comparison. `quality` retains
1 cm measured and 2 cm provisional voxels; `balanced` uses 2 cm for both. No new Quest APK is required for these
PC-side profiles, and this does not enable adaptive-detail or CloudXR rendering.

Use the HoloLens address shown by HORUS Field if it differs from this example.
This launches the canonical connector relay, existing measured nvblox pipeline,
full-geometry map adapter, measured trajectory publisher, teammate/workspace
registration, live reception checker, and the selected PC viewer. The default
measured mode does not launch mock feeds, neural inference, remote/cloud
transports or voice signaling. Inference starts only with `--estimated-depth`.
Use the separate voice workflow below for calls.
A teammate relay must not be launched twice, and only one measured mapper may
own a ROS domain in this demo. Duplicates are refused rather than stopping an
existing demo. Do not run a standalone HL2SS server alongside HORUS Field:
the Field app must own its own camera and sensor endpoints.

`--estimated-depth` uses the physical long-throw image and calibrated RGB image
together, not an RGB-only replacement. The measured map remains separate and
authoritative. Rerun retains both reconstructions, with the hybrid map selected
initially. HORUS displays that same provisional map on
`/horus/field/estimated_mesh/indexed`; it does not silently substitute the
measured-only source. Both native maps are still generated and saved separately.
Physical depth anchors are
preserved. Estimated pixels need agreement with depth withheld from the model
prompt and recurring evidence across distinct physical observations.

The default completion backend is LingBot-Depth v0.5. The optional
`--depth-backend promptda_small` and `promptda_large` alternatives were not
accepted on this workstation's sparse HoloLens validation sample, so they are
not recommended for this profile. They remain explicit comparison options,
subject to the same rejection gates. Completion runtime and pinned weights
must pass `./field-demo --check --estimated-depth --visualizer rerun` first.

`--estimated-range 10` sets the provisional inference and fusion limit to ten
metres. It does not extend the physical sensor or establish ten-metre accuracy;
far geometry needs RGB evidence and is unverified outside the observed depth
region. Sparse/misaligned or stale input is rejected, not silently fused.
If status reports `camera_frames_not_aligned`, stop the scan, reopen HORUS Field
while tracking is valid, and start a fresh scan. Do not combine maps across an
unverified tracking-origin reset. Saving a hybrid run writes separate
`final.ply`/`final.nvblx` and `final_estimated.ply`/`final_estimated.nvblx` exports.

To view the teammate and map in HORUS MR, connect the Quest app to this PC's LAN
address on port `10000` and create the workspace. The SDK registers the non-controllable
teammate with a head/HoloLens marker, real projected FPV camera,
travelled path and shared map. Native geometry comes from
`/horus_measured_mapper/mesh`; `/horus/field/mesh/indexed` carries indexed live-mesh updates
into HORUS without triangle decimation. No occupancy-grid or synthetic-map substitute
is used in this test.

The demo registers one selected workspace mesh, because the current HORUS global
map path displays one mesh source at a time. Without `--estimated-depth` that is
the measured map; with it, the estimated map is selected, matching Rerun's initial
tab. The terminal and `launch.json` identify the exact source and topic. To keep
generating both maps but inspect the measured reconstruction instead, use:

```bash
./field-demo --estimated-depth --map-source measured --visualizer rerun --mapping-memory unified
```

Rerun starts on its measured tab for that command. `--map-source estimated`
requires `--estimated-depth`; unavailable sources are rejected instead of
replaced with a different map. Uniform profiles send the original indexed
triangles and colours from the selected source. Only explicit `--adaptive`
enables coarser context outside detailed interest areas. Use the normal **HORUS**
Quest application; **HORUS Map Lab** is a separate benchmark, not this demo.

The travelled path is `nav_msgs/msg/Path` on `/field_teammate_1/trajectory`, in
`field_teammate_1_scene/source`. It uses the same long-throw camera poses as Rerun's trajectory, including
their true height and source timestamps; it is not a planned route or a control
command. The path is workspace-anchored, not attached to the moving head.
It uses the same thin robot-local trail renderer as the G1 recording demo, not
the thicker global navigation-path overlay.
Like Rerun, it admits points after 2 cm of motion and retains at most 20,000
positions. It publishes at most twice per second while moving and refreshes an
unchanged path every five seconds for Quest reconnection. This does not change
mesh resolution or depth fusion. It works with `--no-view`, independently of Rerun.
With `--name another_teammate`, both pose and trajectory topics use that name.

### Workspace placement and dense-map batching

The mapping demo does not infer a full person's pose from a headset position.
It registers a head-sized selection box and a head/HoloLens mesh at the native
PV camera pose. Legacy floor-plus-profile-height head poses and unaligned hand
frames are not used for this mapping representation. Receive-only and mock
registration retain their independent body-profile options.

Namespaced presentation frames keep the scene coherent. An explicit identity
anchor gives the MR fixed frame its own static TF sample:

```text
field_teammate_1_scene/world
  -> field_teammate_1_scene/floor (identity)
    -> field_teammate_1_scene/source
      -> field_teammate_1_scene/camera
```

The visual datum is the lowest observed startup mesh height, locked after three
stable observations. It is **not a detected or surveyed physical floor**: a scan
that has not observed the floor cannot establish its real height. The datum
does not keep shifting as more geometry arrives. The native map, travelled path
and camera receive the same translation; distances, scale, colours, source
vertices and raw ROS `map` coordinates are unchanged. The aliases do not rewrite
the robot or sensor TF tree. Start a fresh scan after a tracking-origin reset.

Camera poses must agree with fresh native depth poses before the marker moves.
`scene/status` reports `camera_frames_not_aligned` or
`waiting_for_depth_tracking` rather than substituting a guessed human height.
If the old HoloLens build loses its Research Mode tracking origin, the PC can
still receive RGB and pose packets while valid depth/mapping stops. The scene
correction does not repair that device-side tracking-recovery bug; its UWP
update is a separate step.

The measured RGB-D launch also checks shared camera origins **before fusion**.
Depth and colour stop altering the reconstruction when their fresh camera centres
disagree, rather than adding displaced surfaces or painting from the wrong pose.
`research/long_throw/status` reports `camera_frames_not_aligned` or
`waiting_for_shared_camera_origin`. The extra `tracking_pose` topic is diagnostic
native Research Mode tracking, not the validated pose used by the path or camera
marker. This check cannot repair an existing corrupted volume, prove loop-closure
accuracy, or recover the old app's world origin. Reopen HORUS Field and start a
fresh scan after an origin mismatch; retain the previous scan for diagnosis.

The SDK processes nvblox encoding on one worker, independently of the ROS
publish/repair timer. While it is busy, pending observations coalesce by block
identity, keeping unrelated blocks, deletions and clear boundaries. Only obsolete
pending versions are replaced; no spatial downsampling is applied. Quest uploads
indexed pages into a hidden buffer in bounded slices and swaps each complete page
into view. The previous page remains visible during upload. Diagnostic logs report
resident geometry, queued bytes, worker time and actual transfer work. GPU/API
allocation calls remain atomic, so the time budget is soft, not a hard real-time
guarantee. A growing full-detail map can still exceed the headset's render budget.

On shutdown, native mesh/volume exports stage in the PC's local temporary
directory, then copy sequentially to the scan's `exports/` directory. This avoids
nvblox's many small PLY writes blocking on NTFS. Allow temporary disk space for
the exported files. Completed files receive checksums and a session-manifest
entry; a timeout/error is **not** a successful save. Failed native staging and
destination `.partial` files remain available for diagnosis. Do not treat a lone
PLY filename as proof that the complete mesh or restorable volume was saved.

Field mesh pages group eight native blocks per axis instead of four. With the
current 1 cm voxel profile, that is a 0.64 m spatial group instead of 0.32 m.
This reduces renderer count without downsampling or changing fusion accuracy;
individual pages remain subject to transport/upload limits. On a captured
partial real scan, the grouping retained all 448,839 triangles and reduced 254
pages to 71. These are replay/page-count results, not a measured headset FPS
gain or a universal map-size limit. Restart the demo to apply launcher changes.
Use a matching current HORUS MR build: older builds can keep the fallback mesh
frame if chunks arrive before their manifest, and can reject later path samples
against a static TF timestamp. Updating the SDK alone does not correct those MR
renderer/time-resolution behaviours.
Larger pages can resend more unchanged neighbours when one block changes: the
same capture generated about 9% more update bytes before transport coalescing,
while generating 68% fewer page updates. The existing byte-paced, latest-revision
scheduler remains enabled. This is a renderer/traffic tradeoff, not a bandwidth
compression claim.

`field_teammate_mapping_registration.py` alone registers existing streams; it
does not start the sensors, mapper, trajectory publisher or mesh adapters. Use
the `./field-demo` commands above for the complete launch. Do not run a second
registration alongside that launch. A new launch starts a new path/map session;
existing saved scans are not overwritten.

Every five seconds the terminal reports **received** TF/video/depth/path/mesh rates and
waiting/live/invalid/stale states. An unchanged valid mesh is reported as `retained`,
not falsely counted as new frames. The saved JSON includes image dimensions,
depth encoding/valid pixels/median metres, calibration model, timestamp age, camera
and composed head positions, and native-vs-head-pose-fallback counts. An estimated
`synthetic_pinhole` preview is not accepted as calibrated map colour. Fresh calibration
is downloaded into each scan's directory; old headset/IP caches are not silently reused.
ROS header age is not capture-to-PC latency: the older relay stamps pose/video
messages on arrival. These bring-up logs are not validated experiment runs.

On this workstation outputs are under:

```text
/media/omotoye/New Volume/HORUS/data/field_mapping/field_<timestamp>/
  launch.json           # actual processes, endpoints and configuration
  reception.ndjson      # PC-side stream checks, including missing data
  relay.log             # camera/control connection details
  path.log              # measured pose-to-Path publisher
  scene.log             # shared presentation datum and native camera alignment
  mapping.log           # native depth/fusion/RViz, for full mapping
  registration.log      # Quest connection and registration acknowledgements
  scan/exports/         # PLY mesh and restorable nvblox volume after saving
  exit.json             # process result and explicit data-validation result
```

`Ctrl+C` attempts to save the measured map before stopping only this launch's
children. A failed/timed-out save is reported, not called a successful export.
Original logs are kept even if streams are missing. Exit code2 means the requested
sensor set was not validated at shutdown; inspect `reception.ndjson` instead of
interpreting process startup as a successful sensor test.

### Depth throughput and tracking diagnostics

The completion worker has one 8 Hz inference budget and a bounded latest-pair
queue. There is no second image-timestamp limiter that halves a 9-10 FPS camera.
The configured budget is not a promise of delivered FPS: input availability,
independent validation and tracking still determine accepted output.

Large native/HMSH mesh messages are checked by a separately supervised
`mesh_health` process (`mesh_reception.ndjson`). The main sensor monitor consumes
its small summaries, preserving actual mesh counters and age. Rerun likewise
uses separate sensor and mesh processes sharing one recording; full-resolution
geometry is retained rather than decimated to keep the camera responsive.

Long-throw status reports `received_rate_hz` and `published_rate_hz` at the
source, plus processing/publish durations. Completion reports `output_rate_hz`
and actual `inference_rate_hz`. `model_fps` and the legacy `effective_fps` are
inverse processing timings, **not** actual delivered rates. Compare source
counters, completion counters and received counts over the same stable interval.

The exact temporal evidence filter can use an optional project-local C++ index.
It preserves the same voxel keys, distinct-observation votes, history eviction
and measured anchors. With the existing compiler and Torch/pybind11 headers it
is cached under `HORUS_FIELD_NATIVE_CACHE`, or beside `HF_HOME` in `native-cache`.
`temporal_backend=auto` falls back to the original NumPy rule if unavailable;
status identifies the active backend. No system package or driver is installed.

`waiting_for_depth_stream` is different from `camera_frames_not_aligned`:
the latter means the two locatable camera poses disagree, even if physical
depth keeps arriving. Completion pauses rather than injecting wrong geometry.
Reopening an old Field app after tracking loss may temporarily restore a common
origin, but is not a permanent app fix. Do not join pre-reset and post-reset
scans without verified spatial alignment. A proper app-side shared-coordinate
recovery update requires rebuilding and deploying the HoloLens UWP app with a
supported Windows toolchain; PC throughput optimization does not repair that
installed-app lifecycle defect.

**Physical check, 26 October 2026:** the initial test received tracking but no JPEGs.
After camera permission was granted in HoloLens Settings and the PC was restarted,
the headset at `10.0.3.42` supplied real tracking at roughly18-24Hz and decoded
640x360 video at roughly9FPS, with camera intrinsics and native PV poses. Keep
HORUS Field open and its camera permission enabled. A subsequent real mapping run
received calibrated320x288metric depth at5Hz and produced mesh updates near2Hz;
Rerun received the reconstructed geometry, and saving a PLY mesh plus a restorable
nvblox volume succeeded. The indexed HORUS mesh stream was also received and
validated on the PC. This does not by itself qualify the Quest display or mapping
accuracy. Received rates do not establish physical tracking accuracy, cross-camera world
alignment, Quest display rate or image-to-display latency. Check camera orientation
and that room geometry remains fixed while moving the headset before collecting
research data.

### Canonical live UI and voice test

Use the managed connector launch when testing the physical HoloLens, unified
main panel, or live voice. The connector and registration names must match
exactly because the entity name is also the voice room.

```bash
# Terminal 1: HoloLens pose/PV/control relay plus voice signaling.
cd ~/horus_connector
./horus setup teammate       # only when the HoloLens IP/name changed
./horus doctor teammate
./horus stop
./horus launch teammate
```

In a second terminal, source ROS and register the name configured as
`FIELD_TEAMMATE_NAME` in `~/horus_connector/.env`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
set -a
source ~/horus_connector/.env
set +a
cd ~/horus_sdk
export PYTHONPATH="$PWD/python${PYTHONPATH:+:$PYTHONPATH}"
python3 python/examples/field_teammate_registration.py --name "$FIELD_TEAMMATE_NAME"
```

Keep both terminals running. Connect HORUS MR to its normal bridge on port
`10000`, accept the workspace, start live voice on Quest, and accept it from
the HoloLens main panel. Stop registration with `Ctrl+C`, then run
`./horus stop` in the connector checkout.

The `--live-hololens` SDK convenience flag below starts the stream relay
directly. It is useful for stream/registration debugging, but the managed
`./horus launch teammate` path above is preferred for voice because it also
owns signaling lifecycle and automatic HoloLens voice configuration.

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
## Adaptive 1 cm / 3 cm Quest Viewing

Use the adaptive mode for a fresh, complete-coverage scan:

```bash
cd ~/horus_sdk
./field-demo <HoloLens_IP> --adaptive --estimated-depth --visualizer rerun
```

Stop the preceding `field-demo` with Ctrl+C first so it can save its scan. The
launcher starts its own relay, bridge registration, source mapping, trajectory,
mesh adapter and adaptive server; do not start a second bridge manually. Each
run gets a fresh output directory and preserves previous maps.
On shutdown, `adaptive_original/` stores the original mesh pages with checksums,
source inventory verification and the existing visual datum for saved replay.
This CPU-side checkpoint does not rely on a successful large PLY export.
Incomplete captures are marked and rejected by the saved-map replay loader.

This mode retains 1 cm source geometry on the PC. When estimated depth is enabled,
both mappers use 1 cm voxels, and the Quest's adaptive view selects the explicitly
provisional estimated map; the measured source is retained separately. Without
`--estimated-depth`, Quest displays the measured source. The 3 cm context is a
colour-aware, boundary-preserving display approximation of the same source, not
an independently aligned reconstruction or a promise of 3 cm physical accuracy.
Fine geometry and colours are unchanged. Boundary/detail constraints can retain
more triangles than a uniform 3 cm reconstruction.
`--map-source measured` also selects the measured source for adaptive viewing
while leaving both original reconstructions running and saved.

The scene camera/head marker does not wait for a new depth image on every pose
update. Native camera/depth alignment is checked first; subsequent depth-delivery
pauses do not by themselves stop valid tracking. When time-matched head and PV
samples establish a safe rigid camera offset, head tracking can drive the camera
frame at the head-pose rate. If Unity's head origin differs from the native map
origin, the scene uses the exact native PV poses instead, at their actual source
rate. It does not subtract an assumed human height or invent intervening poses.
Real tracking/origin changes still require validation before resuming. The
`/<teammate>/scene/status` message identifies `camera_pose_source`.
Network arrival bursts or asynchronous depth callbacks must not permanently
block native PV poses. Native motion discontinuities and the latest depth/PV
residual are diagnostic fields; explicit invalid tracking or an origin revision
still invalidates alignment. Native-PV fallback is limited to the actual PV
pose rate, not an interpolated or claimed higher-rate head feed.

The adaptive server's `<base>/status` distinguishes `source_pages`,
`received_requests`, `clients`, `waiting_for_subscribers` and `sent_pages`.
A valid source with zero client requests is not a successfully delivered Quest
map. Registration must retain the nested adaptive policy through Unity's
registration-copy and deferred-workspace paths.
Client identities remain 32 hexadecimal characters. A digit-leading identity
uses the ROS namespace token `c_<identity>`; letter-leading identities retain
their existing namespace. Both the provider and Quest use this rule, since ROS
topic segments cannot begin with a digit. Reconnecting must not stop the map
provider because a newly generated identity happens to start with a number.

Connect the updated HORUS Quest app to this PC on port 10000 and create a
workspace. A detailed region follows the Field teammate. The cyan inspection
volume can be moved by grabbing a corner with a controller ray or nearby hand;
use two grabs to resize. Its requested bounds are independent of the teammate.
Releasing either grab and grabbing again retains the chosen size and position.
When less detail fits than requested, a green inner boundary shows the granted
inspection extent after its pages arrive; the cyan handles retain your request.
The renderer replaces whole spatial pages, so a delivered detail boundary may
extend slightly beyond the requested shape. Overlapping areas are counted once.
The server's `regions_ready` counts fully satisfied requests. A large requested
box may therefore report zero fully ready regions while its smaller granted
area already contains fine pages. Inspect `granted_regions`, `fine_pages` and
the delivered inventory together; do not interpret the requested cyan box as
a guarantee that every enclosed page is currently at full detail.

The headset requests its own view and adjusts its triangle admission budget from
valid XR application GPU timings and frame deadlines. It does not change image
resolution, MSAA, text, recorded maps or other operators' selected regions.
Refinement uploads are staged and repairs/reconnects preserve complete context.
Requested detail is not reported as ready before its inventory arrives. When
minimum detail or the complete context cannot fit the resource policy, the status
reports that conflict rather than deleting parts of the map.
This is a real limit: if the complete context consumes the current budget, no
fine region can be granted, even for a small requested box. The default maximum
admission budget is two million triangles, not a measured universal Quest limit.
Do not raise it blindly or infer an exact 9:1 reduction from the 1 cm/3 cm label.
Two overlapping 2 m regions can also cover most of a small scan, making the
surrounding-detail contrast less apparent; a region moving away must return
its old pages to context unless another region still covers them.

Adaptive display does not reduce the PC memory required for its original maps.
Both 1 cm mappers plus depth inference have exhausted this workstation's 16 GB
GPU memory during live tests. After a mapper exits, its retained mesh can still
be visible, but that is not evidence of continuing fusion. Keep the original
mesh checkpoint and check mapper liveness/logs before calling a run successful;
an unpublished GPU volume cannot be recovered after that process has died.
The launch records actual nested mapper exits in the scan's
`mapper_exit_<node>.json`. An unexpected exit stops this demo with an error,
saves any surviving mapper and the last adaptive mesh, and skips the unavailable
native volume instead of waiting for a dead save service. Normal Ctrl+C still
saves before stopping the mapper processes.

On a workstation with the optional HORUS mapper memory overlay built, use:

```bash
./field-demo <HoloLens_IP> --adaptive --estimated-depth --visualizer rerun --mapping-memory unified
```

This selects the distinct `horus_nvblox_memory` wrapper for both maps. It keeps
the same voxel, sensor, inference, range and coverage settings, while allocating
map layers in CUDA unified memory. Host RAM can back those layers; paging can
slow fusion, and scratch/hash/model allocations can still exhaust GPU memory.
It is not an unlimited-capacity or unchanged-FPS guarantee. The normal default
remains the stock device-backed mapper. Missing overlay dependencies fail in
preflight rather than silently ignoring the requested memory policy. The
machine-local `HORUS_NVBLOX_MEMORY_PREFIX` supplies package discovery only to
this command; other ROS sessions and system packages are not changed. Build and
removal instructions are in `horus_field_mapping/tools/nvblox_memory_overlay/README.md`.

Local preflight, without starting a scan:

```bash
./field-demo --check --adaptive --estimated-depth --visualizer rerun
```

The first adaptive preflight builds a small project-specific native helper from
the SDK's pinned MIT meshoptimizer sources using the installed C++ compiler. It
does not download dependencies or install system libraries. A 1 cm source still
requires PC GPU/RAM for reconstruction; adaptive display does not remove that
cost. The existing quality, balanced, 3cm and performance uniform profiles remain
unchanged. Do not combine `--adaptive` with a coarser source profile.

## Generic SDK Registration

Any HMSH v2 producer can use the same adaptive server:

```bash
python3 -m horus.mapping.adaptive_mesh_ros \
  --input-topic /my_map --output-topic /my_map_adaptive --frame-id map
```

Register its output using `dataviz.add_adaptive_mesh(...)`, with `interests` for
the application. A region with a `frame` follows that robot's TF; one without a
frame is operator-controlled. Positions and sizes use real map metres, not the
miniaturized workspace units. `priority`, `min_fraction`, triangle bounds and
memory policy are configurable. The source frame must match the mesh coordinates.
Original map publishers remain unchanged and the normal `add_live_mesh` path is
still available.
Rust and C++ clients can supply the same nested `adaptive` object through their
existing live-mesh render options; their registration serializers preserve it.
