# Mapping visualization checks on Quest 3

Run these from the Ubuntu PC connected to the Quest. Each command starts one
map source, its required adapter, and SDK registration. The SDK starts the
HORUS bridge automatically. No robot simulation, ROS daemon command, experiment
runner, cloud relay, or manual environment exports are required.

## Start here

```bash
cd ~/horus_sdk
./mapping-demo indexed
```

1. Keep this terminal running. There is no experiment timer.
2. Open HORUS on the Quest, connect to this PC's LAN IP on port **10000**,
   then create and accept a workspace. The SDK dashboard prints the IP.
3. The reconstructed G1 laboratory mesh should appear as a workspace map. These
   isolated demos intentionally register no robots or Robot Manager panels.
4. Walk around, crouch, rotate your head, and resize the workspace. The map
   should stay fixed relative to the workspace, retain its colors and surfaces,
   and remain visible to both eyes without stretching or losing previous pages.
5. Press **Ctrl+C** to stop this demo. Close and reopen HORUS before switching
   map paths so a retained map from a previous registration is not mistaken
   for the next result. Start the next command and reconnect/create a workspace.

The wrapper selects local-only ROS domain 17, Fast DDS and current-workspace
bridge auto-start for its child processes. It does not fall back to a different
helper checkout. This does **not** restrict the Quest's TCP/WebRTC connection to the
PC. The PC and Quest still need ordinary LAN reachability. Stop an unrelated
HORUS demo normally before starting: the wrapper will not kill existing services
or change their ROS domain. An already-running bridge must use the same domain.

Use the app and SDK mapping fixes from **2026-10-12** for these checks. The remote
launcher now keeps loopback in its generated DDS profile when local-only discovery
is selected. A connected Quest cannot exchange registration/view messages with a
bridge whose ROS process cannot discover the launcher. Restart the whole demo,
including its owned bridge, after updating; restarting only the GPU renderer does
not replace the bridge's inherited profile. Explicit user-supplied DDS profiles
are respected and must permit communication between these local processes.

## Commands: native Quest rendering

### Full recorded G1 workspace

For the articulated G1, projected camera, LiDAR scan, walked path and the same
nvblox recording together, use this separate launcher:

Use the HORUS app build containing the recorded-G1 visualization fixes
(2026-10-12 or later). Earlier builds can display the model/camera while failing
to subscribe to the mesh and `lidar_3d` scan, and can separate fixed palm joints
from the moving finger TF chains. An SDK-only restart cannot fix those app paths.

The bridge also needs the 2026-10-12 TF/occupancy fixes: partial TF packets must
not replace other transforms on the same topic, and a saved occupancy snapshot
must reach clients that subscribe after its publication. Restart a previously
running demo after rebuilding the bridge. Those bridge fixes work with the
existing recorded-G1 app build; no further APK update is required for them.

```bash
cd ~/horus_sdk
./g1-recording-demo
```

The launcher now supports both physical recordings and their completed maps:

| Command (run from `~/horus_sdk`) | Demonstration |
| --- | --- |
| `./g1-recording-demo first --map static` | September 29 first walk: finished RGB mesh (115,205 faces), saved 1 cm occupancy grid, moving articulated G1, projected RGB, LiDAR and walked path. |
| `./g1-recording-demo later --map static` | October 5 walk: finished RGB mesh (227,954 faces), saved 1 cm occupancy grid and the same moving robot/sensor layers. |
| `./g1-recording-demo later --map live` | Later recording's archived incremental nvblox updates plus robot/sensors and the saved completed occupancy grid. This is also the no-argument default. |

Run only one at a time. **Static means the 3D map is already complete; the robot,
camera, scan and path still replay when you press Enter.** The completed mesh is
loaded on request after workspace connection and remains unchanged during the walk.
Its original RGB and triangle indices are preserved; it is not a point-cloud shell.
There is no archived incremental mesh output for the first walk. `first --map live`
fails explicitly rather than pretending that the static map is a live reconstruction.
Neither mode starts an estimator or nvblox reconstruction process.

1. Stop the previous demo with **Ctrl+C**. This launcher refuses to reuse an
   existing bridge on port 10000 and does not kill it. Keep New Volume mounted.
2. Open a fresh HORUS workspace, connected to the PC on port **10000**. The SDK
   starts the bridge. No ROS daemon command or environment exports are needed.
3. Wait until the G1 body appears. The recording holds its first pose while
   registration/model transfer finishes; the walk has not started yet. Camera
   and scan previews may not appear until playback starts because subscribers
   can connect after the initial preview messages.
4. Press **Enter in the launch terminal** when ready. Alternatively, run
   `./g1-recording-demo --start` in a second terminal. Starting before registration
   succeeds is rejected; retry after the workspace is ready.
5. Watch the approximately **128-second first walk** or **125-second later walk**.
   In `--map live`, the 3D map develops from the archived incremental updates.
   In `--map static`, wait for the completed map to load before starting the walk.
   In an uninterrupted session, the final model pose, image, scan, complete
   walked path and map remain visible. If the app reconnects after playback has
   finished, restart the demo to replay camera and scan messages for that client.
   **Ctrl+C** stops this demo. Relaunch for another replay; no automatic map reset.

The rainbow LiDAR layer is a changing scan and the colored surface is the retained
nvblox map; both can be visible together. The Robot Manager point-cloud toggle
controls the scan without replacing or disabling the shared mesh.
LiDAR uses an opt-in height palette: purple at -1 m through blue, cyan, green,
yellow to red at 2 m, clamped outside that range. Heights are the recorded map Z,
before workspace scaling. These are false colours for reading geometry, not RGB
measurements or intensity. Source RGB, if present, is kept. Map colours remain
unchanged. A fresh demo registration is required to pick up the palette setting.

The topic names below refer to the later recording. The first walk uses the
matched mapper-input and display-pose files described after the table.

| Layer | Recorded source and treatment |
| --- | --- |
| G1 with Dex3 hands | Full-detail URDF: 43 moving joints, 44 baked mesh groups, 418,206 triangles. Fixed visual links are merged, not dropped. Recorded joints drive `robot_state_publisher`. |
| Robot motion | `/g1_spatial/pelvis_pose`, original positions/orientations, not sensor odometry. The separate upright selection anchor does not alter body poses. |
| Projected RGB and overhead view | `/g1_spatial/mapping/rgb`, 1280 x 720, JPEG quality 90; recorded CameraInfo and nominal optical mounting transform. Camera is a fixed child of the model's `mid360_link`, not an independently updated world frame. No resizing or flipping. JPEG is lossy. |
| Point-cloud sensing | Complete `/g1_spatial/map_scan` XYZ scan with a height rainbow. A changing sensor scan, not the separate multi-million-point aggregate; no added subsampling. |
| Walked path | Every recorded pelvis pose accumulated into `nav_msgs/Path`, published at up to 2 Hz plus the final full path. Uses the supported robot-local-path display role, not a planned goal. |
| Colored nvblox map | All 1,151 incremental updates through the existing full-detail retained-page adapter; same recording as `./mapping-demo nvblox`. |
| Completed 3D maps | Direct saved-mesh exports in `--map static`: full RGB surfaces, spatially ordered HMSH v2 pages, selective repair, no periodic full-map resend. |
| Occupancy grid | Saved actual occupancy values (-1 unknown, 0..100), not ESDF or a costmap. First: 1841 x 888; later: 2342 x 2492; both 0.01 m/cell. |

Dynamic streams follow **one selected derived-bag receipt timeline at 1x**;
outgoing ROS timestamps are refreshed. The first walk uses the matched display
pelvis/joint/camera poses and transforms the mapper's sensor-local scans into the
same map coordinates without additional point sampling. It does not align the
original camera and state bags by their incompatible receipt-clock start times.
The optical mount comes from each bundle's saved configuration. It matches all
first-walk saved camera poses when composed with the body/joint transforms;
it is a nominal rig transform, not a newly measured calibration. Root poses
retain their recorded approximately 14 Hz rate, not synthetic 60 Hz samples.
Static camera mounting therefore follows the same displayed robot hierarchy
even between image deliveries.
This is a physical-capture reconstruction, not a live robot, new mapping run or
camera-latency benchmark. Model materials come from the URDF; map RGB comes from
the reconstruction. No usable battery/command-feedback stream is present. The
occupancy origin is composed with the matching saved `world_from_map` exactly
once; the NPZ already has ROS row order and is not flipped like a PGM image.
**Occupancy is the saved final map even during incremental 3D playback**, not a
claim that the 2D map was known at the beginning of the recording. Its publish
stamp is a playback stamp, not the unavailable original snapshot timestamp.
The grid is published once with transient-local durability. The bridge delivers
that snapshot on subscription/reconnection; periodic multi-megabyte resends are
not needed. The shared occupancy layer remains separate from the mesh layer.
Controls, navigation tasks and recorded `/clock` are absent.

Use `./g1-recording-demo --check` for source/model preflight without starting ROS
nodes. Logs and `replay-summary.json` are under
`~/.cache/horus/g1-recording-demo/replay-*`. The summary records source counts,
path samples, final map blocks and maximum replay scheduling delay. Replay status
is also available on `/horus/g1_recording/replay_status`. Source data and the G1
project stay read-only. The existing map-only demos below are unchanged.

For any of the three commands, append `--check` to validate the selected recording, robot
model, occupancy and static mesh cache without starting a bridge or ROS nodes.
`maps-ready.json` records the completed mesh/page count and occupancy dimensions.
The two finished mesh caches are already prepared on this PC under
`~/.cache/horus/g1-recording-demo/assets/`; source and cache SHA-256 are verified
before use. To regenerate these derived caches only (never the source maps):

```bash
cd ~/horus_sdk
source python/examples/.local_assets/mapping-runtime.sh
export PYTHONPATH="$PWD/python:$PWD/python/examples:$PYTHONPATH"
"$HORUS_REMOTE_PYTHON" -m tools.g1_completed_maps --dataset first --prepare
"$HORUS_REMOTE_PYTHON" -m tools.g1_completed_maps --dataset later --prepare
```

This preparation reuses NumPy/plyfile in the existing local runtime. It neither
installs packages nor reconstructs a map. Bundle paths can be overridden with
`HORUS_G1_FIRST_RESULT` and `HORUS_G1_LATER_RESULT`; a different bundle requires
matching preparation and frame/timeline validation, not guessed frame aliases.

### Map-only commands

Run one row at a time from `~/horus_sdk`.

| Command | Source and path | Expected result |
| --- | --- | --- |
| `./mapping-demo mesh` | G1 real-capture reconstruction, 227,954 triangles as ROS Markers | Saved laboratory map, progressively loaded chunks |
| `./mapping-demo indexed` | Same G1 triangles and source RGB, indexed binary payload | Same complete surface as `mesh`, compact upload path |
| `./mapping-demo mesh --dataset spot` | Physical Spot five-camera reconstruction, 29,802 triangles | Smaller stationary lab capture, original source colors |
| `./mapping-demo nvblox` | G1 recorded `nvblox_msgs/Mesh`, 1,151 updates at recorded timing | Map builds over about 125 seconds, then holds its final state; updates retain earlier blocks |
| `./mapping-demo octomap` | Freiburg real Kinect office `ColorOcTree`, original 2 cm resolution | Colored occupied-voxel surfaces, not a 2D grid |
| `./mapping-demo voxblox-marker` | G1 saved mesh through the Voxblox Marker compatibility relay | Tests Marker input with real geometry, **not** a new Voxblox reconstruction or `voxblox_msgs` input |
| `./mapping-demo points` | ETH Cow and Lady physical reference scan, all 643,343 points | Static measured laboratory point map, original RGB |
| `./mapping-demo points --dataset g1` | All 3,752,449 points in the saved G1 aggregate | Full existing 1 cm aggregate; heavy native rendering test, no additional reduction |
| `./mapping-demo points-refresh` | Recorded G1 registered LiDAR scans in the map frame | Moving individual scans at recorded timing, **not** an accumulated map; final scan held after replay |
| `./mapping-demo gaussian` | Existing 50,000-splat subset of real-photo Playroom reconstruction | Native Gaussian appearance; first transfer may wait for the next 30-second replay |
| `./mapping-demo gaussian-points` | Same Gaussian asset, debug points | Fixed point centers for diagnosing stereo anchoring |
| `./mapping-demo gaussian-no-covariance` | Same Gaussian asset, covariance disabled | Diagnostic appearance without full ellipsoid expansion |
| `./mapping-demo gaussian-mono` | Same asset, center-eye diagnostic | Mono comparison only, not a stereo-quality acceptance test |

**No generated scene is used by these commands.** Developer fixtures remain
available only with an explicit `--synthetic` flag for regression tests. Missing
real data causes an error, never a synthetic fallback. Meshes and point clouds
retain all primitives and source RGB in the selected export. Existing TSDF/point
voxelization is part of the source reconstruction, not raw sensor ground truth.
Unobserved G1 surfaces may be grey; no artificial color is added to disguise this.
The native Gaussian default
is an **existing 50,000-splat subset**, deliberately identified as a small fixture;
the launcher does not create another subset or prune its spherical harmonics.
Point-cloud fallback is disabled in these Gaussian checks so a failed splat load
cannot appear to succeed by displaying points instead.
These native Gaussian demos use binary `UInt8MultiArray` chunks. Both registration
and the publisher select `binary_chunks`; the app must retain that setting when
cloning/resolving registration. Existing SDK registrations still default to the
original text/base64 transport. Do not enable only one end of binary transport.

The native point renderer draws contiguous point vertices rather than a separate
six-vertex instance per point. The Freiburg adapter spatially groups the same
colored surface triangles into larger pages, and this static demo releases CPU
mesh copies after upload. No point, occupied surface, or source color is removed.
Freiburg is much more expensive than the G1 surface despite its smaller physical
extent: this export has **2,125,042 triangles**, versus **227,954** for G1. The
spatial/page optimization reduces Freiburg from 709 to 178 pages, not to fewer
triangles. Scene extent alone is not a headset capacity estimate. Sustained
headset frame time and memory still need checking for each selected scene.

To use another existing Gaussian PLY, preserving every splat in that file:

```bash
./mapping-demo gaussian --ply /absolute/path/to/map.ply
```

Native demo admission is limited to two million splats. Larger inputs are rejected,
not silently truncated. This is not a promise that every admitted scene fits Quest
memory or frame-time limits. Test a small fixture first; use the offboard renderer
for substantially larger scenes.

## Commands: PC rendering, Quest display

The renderer uses the PC's CUDA GPU. The Quest sends stereo view poses and
displays the returned frames. Native and remote mesh/point commands now use the
same source exports by default. This permits same-source comparisons, but keep
viewpoint, scale and measurement procedure fixed before comparing performance.

| Command | What runs |
| --- | --- |
| `./mapping-demo remote-mesh` | Full G1 227,954-triangle mesh with interpolated source vertex RGB, V3 stereo, NVENC/WebRTC |
| `./mapping-demo remote-mesh --dataset spot` | Full Spot 29,802-triangle captured mesh, same remote path |
| `./mapping-demo remote-points` | Full Cow and Lady point cloud, V3 stereo, NVENC/WebRTC, source voxel size zero |
| `./mapping-demo remote-points --dataset g1` | All 3,752,449 points from the saved G1 aggregate |
| `./mapping-demo remote-gaussian` | Full Playroom Gaussian PLY, V3 stereo, NVENC/WebRTC |
| `./mapping-demo remote-diagnostic` | Same G1 mesh through lower-rate ROS diagnostic frame transport |

Wait for **renderer ready**, then connect/create the workspace. A first GPU
extension compilation can take several minutes. The renderer reports its log
path as soon as preparation starts. Remote preparation happens before SDK bridge
startup. No new map geometry is downsampled by these commands.

The current balanced WebRTC profile requests 1152 x 1152 color per eye and
60 FPS at 35 Mbps. Those are configured targets, **not guaranteed delivered FPS**.
The V3 carrier contains reduced-resolution depth/mask information and lossy H.264
video. Remote rendering preserves source geometry but is not lossless image/depth
transport. CloudXR, independent simultaneous viewer streams, and arbitrary live
GPU scene updates are not implemented by these commands.

This Ubuntu installation has an ignored machine-local runtime configuration at
`python/examples/.local_assets/mapping-runtime.sh`. It points to the existing
CUDA/Torch interpreter and isolated dependencies on New Volume without modifying
the shared interpreter. Keep that volume mounted. On another computer, prepare
the remote dependencies described in [the remote guide](REMOTE_MAP_RENDERING_DEMO.md)
and configure `HORUS_REMOTE_PYTHON` for that machine.

The local ROS bridge must be built with WebRTC support. This PC's bridge has
been rebuilt with it enabled. The mapping launcher checks the selected package's
`build_capabilities.json` before starting a WebRTC demo. For another source build:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/horus_ws
colcon build --packages-select horus_unity_bridge --symlink-install --cmake-args \
  -DENABLE_WEBRTC=ON -DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON
source install/setup.bash
```

The fetch option downloads pinned dependencies if no installed libdatachannel is
available. An explicitly enabled WebRTC build now fails if required development
dependencies are missing instead of silently producing a non-WebRTC bridge.
Stop an older running bridge normally before using the rebuilt version. The
preflight describes the local installed binary, not a bridge on another machine.

## Checks and logs

Check prerequisites without starting publishers or registration:

```bash
./mapping-demo indexed --check
./mapping-demo remote-gaussian --check
```

`--check` validates imports, source assets, CUDA/codec availability and the local
WebRTC build capability when needed. It does not
replace actual GPU rendering, bridge delivery, or headset display verification.

- Native source/adapter logs: `~/.cache/horus/mapping-demo/<path>-<unique-run>/`.
- Remote renderer logs: `~/.cache/horus/remote_map/<unique-run>/agent.log`.
- Keep terminal errors and the source log if a map does not appear; do not run
  several paths together to diagnose an empty workspace.
- Check complete coverage after initial transfer and after an update. Transport
  delivery alone does not prove that the Quest rendered every page.
- Compare stationary and moving-head views, both eyes, colors, workspace scale,
  memory growth and sustained frame timing. Startup success is not a performance
  measurement or evidence that stereo reprojection is correct.

## Real-data provenance and preparation

The prepared manifest is `~/.cache/horus/real_mapping/manifest.json`, on New
Volume on this PC. It records source paths, hashes, original counts and conversion
notes. Original captures/PLY/NPZ/native archives are not modified. Static source
positions retain their session origin; relabelling to `map` is for one isolated
workspace, **not** alignment between G1, Spot and public scenes. Do not overlay
these unrelated recordings as a common physical environment.

These updates do not change an already-running demo. Stop its terminal with
Ctrl+C before running these commands; a process started before this update
will still be publishing its old generated scene.

| Capture | Source, interpretation and limitations |
| --- | --- |
| G1 | Physical Dex3 G1 walk on 2026-10-05, FAST-LIVO/nvblox replay output `g1_operations/artifacts/spatial/20261006_live_like_walk/20261006T011621Z_fast-livo_7d69e5`. `g1_spatial/map`, LiDAR-only 5 cm TSDF geometry, RGB appearance, 1 cm point aggregate. Nominal camera/body extrinsics are not independently calibrated. |
| Spot | `spot_operation/maps/live_acceptance_20260908/live_acceptance_20260908.ply`: real stationary five-camera Spot capture, 4 cm nvblox reconstruction, session-local `spot/vision`. Not walking SLAM or complete-building coverage. |
| Cow and Lady | [ETH official dataset](https://projects.asl.ethz.ch/datasets/voxblox/), [persistent record](https://doi.org/10.3929/ethz-b-000721636), CC BY 4.0. Uses every vertex in the reference laser PLY. Existing original ROS 1 RGB-D bag is at `~/.cache/horus/voxblox_cow_lady/data.bag`; it is not replayed by the static point example. |
| Freiburg | [Official OctoMap dataset repository](http://ais.informatik.uni-freiburg.de/projects/datasets/octomap/), `freiburg1_360.ot.gz`: physical Kinect office capture aligned by RGBDSLAM. Original `ColorOcTree` bytes/resolution are preserved; surface conversion omits internal occupied faces, not observed exterior geometry. Source is a completed map, not a bag or fresh reconstruction. |
| Playroom | Real-photo Deep Blending scene reconstructed with [3D Gaussian Splatting](https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/). Remote default uses 1,916,379 splats; native default is the explicitly labelled existing 50,000 subset. Do not compare their performance as equal geometry. |

Native `nvblox` replays only the recorded mesh topic; `points-refresh` replays
only the registered scan topic. Neither runs robot controllers, publishes recorded
commands, replays `/clock`, nor accesses hardware. Original relative message timing
is retained; header stamps use replay wall time for this visualization-only test.
Playback targets the original timing; CPU backpressure can stretch it rather
than reducing source geometry. Playback begins when a subscriber exists. The map adapter may subscribe before
you put on the headset; its retained map inventory supports later joining.
Restart the command to watch construction from the beginning. No automatic loops
clear completed maps. These runs measure rendering/transport, not original
sensor-to-display latency or SLAM accuracy.

The saved G1 PLY and recorded mesh stream are different snapshots. Accumulating
all 1,151 recorded updates produces 2,067 blocks and 177,664 triangle-index triples;
574 repeat vertex indices and have zero drawable area. The existing adapter
rejects those degenerate triples, leaving 177,090 drawable triangles in 2,065 blocks;
the separately exported PLY contains 227,954 triangles. Do not interpret this
difference as HORUS dropping geometry. The replay adapter groups 4 x 4 x 4 source
blocks into pages (160 final pages), preserving every nondegenerate recorded triangle while
remaining below the renderer-page limit. The Freiburg exterior surface contains
2,125,042 triangles in 709 chunks; allow about 40 seconds for the initial stream.

The local files are already prepared. On another installation with the same
recordings, use a Python environment with NumPy and plyfile (the existing remote
runtime on this PC provides them):

```bash
cd ~/horus_sdk
source python/examples/.local_assets/mapping-runtime.sh
"$HORUS_REMOTE_PYTHON" python/examples/tools/prepare_real_mapping_data.py \
  --g1-session ~/g1_operations/artifacts/spatial/20261006_live_like_walk/20261006T011621Z_fast-livo_7d69e5 \
  --spot-mesh ~/spot_operation/maps/live_acceptance_20260908/live_acceptance_20260908.ply
```

Preparation refuses to overwrite an existing manifest. It exports arrays without
sampling, reuses the locally cached Cow and Lady PLY and downloads the 8 MB public
OctoMap archive only if absent. Upstream Freiburg currently serves HTTP only;
recorded SHA-256 verifies this local copy, not upstream authenticity. Check dataset
terms before redistribution. No original robot bag is copied into the SDK.

### Remaining representation boundaries

#### Newer G1 inventory, not selected automatically

The G1 Little Task handoff subsequently supplied newer 2026-10-09 outputs under
`/home/omotoye/g1_spatial_validation/live_audit_20261009/`:

- `final_report_livo/20261009T002738Z_fast-livo_e5af85`: the mesh header contains
  169,072 vertices and 226,325 faces. Its source summary reports 6,702,789 measured
  point voxels, including 1,153,799 with observed RGB. This static export has no
  duplicate output bag.
- `two_hz_livo_clean/20261009T002201Z_fast-livo_8b0ab8/outputs`: recorded output bag;
  metadata lists 48,246 messages over 123.759 seconds, including 238
  `nvblox_msgs/msg/Mesh` updates. This is the output duration, not the duration of
  the approximately 128-second physical-source input fixture.
- `two_hz_lio_clean/20261009T002458Z_fast-lio_f931e3`: a separate FAST-LIO geometry
  comparison, with its own static exports and trajectory.

These are reconstructions from recorded physical sensors, not new live captures.
The originating task's acceptance checks are documented in that folder's
`README.md`; they do not establish HORUS ingestion or Quest performance. Each
result has its own initial datum: pair mesh/points and poses from the same run.
The current launcher manifest intentionally retains the earlier G1 dataset used
in the verified tests above; none of the commands silently switches to these
newer results. No large recording was copied or replayed for this inventory update.

#### Consumer coverage

The three VLA inventories are in `/home/omotoye/The Workshop/`, named
`g1-updates-horus-real-mapping-data-2026-10-09.md`,
`g1-little-task-horus-real-mapping-data-2026-10-09.md` and
`spot-horus-real-mapping-data-2026-10-09.md`. They list more recordings and native
maps, including an experimental G1 RGB-only Gaussian reconstruction with 6,079,913
splats. Its parameter conventions/metric scale are not qualified for these tests;
it is not silently substituted for the verified Playroom input.

Local verification on 2026-10-09 rendered the full G1/Spot meshes and both
point sets on CUDA. WebRTC decode checks passed on repeat for the G1 mesh and
Cow and Lady cloud, with zero invalid/unmatched frame markers in those runs.
An earlier G1 run under concurrent workstation workloads had 13 invalid markers
among 970 decoded frames. That failure remains a transport-robustness caveat,
not erased by a passing repeat. None of these PC checks certifies worn-Quest
stereo registration or sustained headset frame rate.

Spot bags contain real RGB/depth/TF but no saved PointCloud2/nvblox Mesh stream.
Use its saved PLY for now; generating those topics requires the existing Spot
mapper, not merely bag playback. G1 supplies the recorded incremental mesh test.

Actual Voxblox `voxblox_msgs/Mesh` replay is still a coverage gap: neither robot
inventory contains that type. The real ETH RGB-D bag is available for a future
Voxblox reconstruction, but `voxblox-marker` tests only the compatible Marker
input using measured-derived geometry. A `.nvblx` file is **nvblox**, not Voxblox.
The older nvblox quickstart uses simulated data and is not part of this real-data
test sequence. No command here claims CloudXR, live sensor reconstruction or
physical headset acceptance merely because a PC transport check passed.
