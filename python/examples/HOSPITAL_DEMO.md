# Local Hospital Fleet

Three physically driven Nova Carter robots in NVIDIA's hospital environment,
with independent navigation, one front RGB-D camera and a 360-beam scan each.
Cooperative slam_toolbox produces the 2D maps; nvblox fuses the three calibrated
depth cameras into a coloured 3D mesh. The SDK registers all three robots and
the shared maps with HORUS.
The simulator runs headless to leave GPU capacity for sensing and mapping;
do not open a second Isaac Sim instance for this demo.

## Start

On this laptop:

```bash
cd ~/horus_sdk
./hospital-demo check
./hospital-demo start
./hospital-demo status
```

The current trial uses **5 cm** voxels and a forward-facing spotlight attached
to each physical robot. The light illuminates the actual simulated camera
image and therefore the colour input to mapping; it is not an image overlay or
a new Quest rendering light. It follows the chassis without copied-pose updates.

To compare the previous 3 cm profile, start a fresh run explicitly:

```bash
./hospital-demo stop
./hospital-demo start --voxel-size 0.03
```

Use `--voxel-size 0.05` for the 5 cm trial, or simply omit it. Voxel sizes are in
metres and are recorded in the run configuration/status. They cannot be changed
inside an existing TSDF volume. Coarser voxels generally reduce mesh load but
also reduce geometric detail; this is an explicit reconstruction tradeoff.

No separate ROS setup, domain exports, bridge launch or registration command is
needed. Initial startup includes asset loading and robot-model preparation.
Wait for `local_streams_ready` in the status output. This means actual local
sensor/map receipts passed, not that a headset has connected.

Open **HORUS** on the Quest, connect to **192.168.1.22:10000**, and create a
workspace. Use the laptop's current LAN address instead if the Wi-Fi changes.
Keep both devices on the same LAN. Port 10001 is the companion bulk-data lane;
the app opens it automatically. No remote connector is used in this local stage.

You should see Carter 1, 2 and 3 with their full robot bodies, scans, projected
front-camera views, shared occupancy grid and coloured 3D map. Select a robot
to open Robot Manager. Teleoperation drives only that robot. Go-to-point and
waypoint tasks use its own Nav2 stack; cancel stops that robot's task. Starting
manual teleoperation cancels its navigation task. In Robot Manager > DataViz,
**Odometry Trail** switches the short travelled trail on or off independently
of **Nav Path**, which switches the planned route. The trail uses existing
odometry and retains at most 48 points / 3.2 seconds; it is not an accumulating
full-mission path. There is no automatic tour on ordinary startup.

Hospital steering uses `invert_angular_z=True`: moving the turning stick left
produces positive ROS yaw (counterclockwise), and right produces negative yaw. Forward
motion and other registrations' axis conventions are unchanged.

The projected/minimap camera uses ROS JPEG. Immersive teleoperation requests
the native HORUS WebRTC stream. The PC setup does not require CloudXR, inference,
HoloLens, a cloud server or external robot hardware.

For an explicitly configured remote source, `HORUS_HOSPITAL_REMOTE_MEDIA=relay`
registers both views against local encoded relay topics
`/horus/remote_media/carterN/h264`, using the normal operator bridge signaling.
This mode requires the connector's source sender and `HORUS_MEDIA_RELAY_CONFIG`,
plus the operator bridge's matching `webrtc.preencoded_topics` list; the SDK
registration alone does not create that transport. `HORUS_BRIDGE_CONFIG` can
select a separate bridge YAML without changing the default local setup.
`HORUS_HOSPITAL_REMOTE_MEDIA=direct` instead requests the source cameras through
remote signaling and requires source-to-Quest ICE reachability. Merely putting
the two PCs on Tailscale does not establish that direct Quest route.
Unset these variables for the unchanged ordinary local demo.
The projected-camera scale is 0.11, half the previous 0.22 profile. Its plane is
centered on each robot's optical axis, with a configured plane distance of about
0.112 m and visible projection lines. The distance matches the 82.26-degree
horizontal field of view and 16:9 image aspect ratio. Subsequent workspace resizing
scales both the plane and distance. Image resolution and field of view are unchanged;
the overhead view keeps its own size. Projection rays start at each robot's
`camera_optical` TF, mounted at `(0.35, 0, 0.58)` metres relative to `base_link`,
not at the interaction box. The updated Quest app waits for that frame before
showing a projected view rather than drawing from a fallback origin.
Use `refresh-registration` below to apply
these registration settings to a running local demo without resetting its map.

## Check And Stop

```bash
cd ~/horus_sdk
./hospital-demo verify
./hospital-demo stop
```

`verify` records 20 seconds of actual ROS receipts without moving the robots.
It reports missing streams and saves rates, timing, calibration, sample images,
map dimensions and indexed mesh receipts (including the HSMH wire header) in
the current run directory. It checks the actual map stream sent to HORUS rather
than subscribing a second consumer to nvblox's much larger internal Mesh message.
These are **PC
receipt rates**, not Quest display FPS or measured immersive WebRTC latency.

`stop` stops only owned demo processes and waits for shutdown. It does not kill
unrelated processes, change networking or delete saved runs. A fresh start
creates fresh simulation/mapping state; old logs and captures remain available.
An occupied demo port is reported as an error rather than forcibly reclaimed.

After editing only the SDK registration, update it without resetting the running
simulation, navigation, bridge or map:

```bash
./hospital-demo refresh-registration
```

This restarts only the owned registration child and appends to its existing log.
The command reports that a restart is required if an older running supervisor
does not support this operation; it never sends that old process an unknown signal.

## Profiles And Frames

| Item | Configuration |
| --- | --- |
| Simulator | Shared Isaac Sim 6.2; official hospital and Carter physics |
| ROS | Jazzy; loopback discovery up to 100 participants plus shared memory, domain 87 |
| Robots | `carter1`, `carter2`, `carter3` |
| Physics | 60 Hz simulation time; physical wheel drive and explicitly ideal simulator odometry |
| Camera | 640 x 360 RGB + metric depth, nominal 15 Hz simulation time; JPEG quality 80 |
| Scan | 360 rays, 10 Hz simulation time, 0.1-20 m |
| Occupancy | Cooperative scan-based SLAM; 5 cm cells |
| 3D map | Native nvblox TSDF; 5 cm trial by default, selectable spacing, 8 m depth integration range |
| Mesh | Lossless indexed spatial pages with repair/replay; no triangle decimation |
| Navigation | Per-robot Nav2, obstacle/inflation layers and collision checks |
| Limits | Teleop up to 0.6 m/s and 1.2 rad/s; navigation 0.35 m/s; 20 cm goal tolerance |

The nvblox depth range is explicit. It does **not** clear already mapped regions
by radius or height. No claim is made that unobserved space is reconstructed.
The simulation can run slower than wall time under load; use the measured
`simulation_to_wall_ratio` and per-topic receipt rates instead of treating
configured rates as achieved FPS.

Public transforms use `map -> global_odom -> carterN/odom -> carterN/base_link`.
Each robot also has prefixed laser, optical and articulated visual-link frames.
Its Nav2/SLAM stack has a **private** TF topic with local `map`, `odom`,
`base_link` names. Only Carter 1's map alignment is relayed into public TF;
there are not three competing public `map -> global_odom` broadcasters.
The initial spawn transforms provide the known shared odometry alignment; this
demo does not estimate unknown inter-robot starting poses.

The 3D map lives in continuous `global_odom`, avoiding abrupt loop-closure
repositioning of previously fused surfaces. It is not a loop-closure-corrected
reconstruction: the pipeline does not retroactively reintegrate the TSDF after
a SLAM correction. Depth rendering and TSDF fusion still determine map accuracy;
an ideal simulator pose does not make the reconstructed surface ground truth.
The authoritative odometry uses the simulator's physical chassis pose, expressed
in each robot's known spawn frame. Wheel rotation alone is not authoritative:
wheels can slip or keep spinning against a wall while the chassis stays still.
The alternative wheel-plus-IMU estimate remains on `/carterN/wheel_odom` for slip
diagnostics. `/carterN/ground_truth/odom` exposes the independent world-frame
diagnostic, and `/carterN/imu` exposes the simulated inertial measurements.
The robots are driven through joint velocity targets, never teleported. This
demonstration explicitly assumes ideal simulated odometry; it is **not** a
ground-truth-free localization or real odometry-noise benchmark.

The indexed `max_triangles` setting is a per-page validation limit, not a total
resident triangle budget. All delivered pages can therefore exceed two million
triangles in aggregate. Adapter source geometry is preserved; this is not a
promise of unlimited headset capacity. Uniform 3 cm or 5 cm is not an adaptive profile.

## Files And Dependencies

The entrypoint is `hospital-demo`; its implementation and generated profile
definitions are in `simulation/hospital/`. The compact registration is
`python/examples/hospital_fleet_registration.py`.

Runs are stored under `/media/omotoye/New Volume/HORUS/data/hospital/` with
separate logs, configuration, source scene layer, normalized URDF, process
identities and sensor-verification results. `current.json` locates the current
run. Existing NVIDIA scene/robot assets and the original cached URDF are not
modified. Only the disposable desktop viewport is disabled; all sensor render
products retain their configured resolution and colour/depth outputs.

This workstation uses an isolated cooperative slam_toolbox overlay at
`/media/omotoye/New Volume/HORUS/tools/hospital-slam-toolbox/install`.
It is based on Jazzy revision `02afdde003313a10b8d21461a92d9e5f4f0bc5f2`
with the upstream multi-robot PR #592 and local compatibility/lifecycle fixes.
The stock system package alone does not supply this cooperative executable.
The overlay does not replace `/opt/ros/jazzy`. Its complete patch, source checksums
and repeatable build script are kept in `simulation/hospital/`. To rebuild it
with the already installed native build dependencies:

```bash
bash simulation/hospital/build_cooperative_slam.sh
```

The script pins the source revision and verifies patched-file checksums. It
does not reset an unrelated source tree or install system packages. Original
upstream copyright and licence notices are retained in the patch.

Paths can be relocated through `HORUS_ISAAC_HOME`, `HORUS_ISAAC_ASSET_ROOT`,
`HORUS_HOSPITAL_SLAM_PREFIX`, `HORUS_HOSPITAL_MODEL_ROOT` and
`HORUS_HOSPITAL_DATA_ROOT`; ordinary operation on this laptop needs none of them.
This is a workstation demo, not an automatic installer for a new machine.

## Maintainer Checks

The focused contracts run without Isaac Sim:

```bash
cd ~/horus_sdk
source simulation/hospital/ros_env.sh
python3 -m pytest python/tests/test_hospital_demo.py -q -o addopts=
```

The following test deliberately **moves all three simulated robots**, checks
independence against physical ground truth, then exercises go-to-point,
cancellation and two-waypoint navigation. Run it only when no operator is
controlling the demo:

```bash
python3 simulation/hospital/check_motion.py --output /tmp/hospital-motion.json
```

For a separate adversarial physical-contact fixture, stop the demo first:

```bash
./hospital-demo stop
bash simulation/hospital/check_contacts.sh /tmp/hospital-contact-check
```

Use a new output directory for each contact check. This places the robots
before simulation starts, then physically drives one against an existing
hospital wall and another against a second robot. It checks actual PhysX
contacts, penetration and displayed-pose agreement, including wheels spinning
while blocked. It starts no HORUS bridge or registration, and is not the normal
operator demo.

Navigation additionally uses a private depth-derived voxel obstacle layer,
because the 64 cm laser plane passes above parts of these robots. Its 5 Hz,
stride-4 point cloud is only for Nav2; nvblox still receives the original full
640 x 360 depth images at the original mapping resolution. The asymmetric
footprint follows the wheel-axle base frame, which is not the body's centre.

An opt-in test reads real streams through both bridge lanes without claiming to
be a headset or fabricating registration acknowledgements:

```bash
HORUS_HOSPITAL_LINK_TEST=1 python3 -m pytest \
  python/tests/integration/test_hospital_link.py -q -s -o addopts=
```

Direct Tailscale, multiple operator PCs and cloud deployment are deliberately
deferred until this local baseline is accepted.
# Optional Native Map Rendering Profiles

With the LOD/SpaceWarp-capable Quest build, these commands switch only SDK
registration. They do not reset, export, stop or reconstruct the live map:

```bash
cd ~/horus_sdk
./hospital-demo render-profile full
./hospital-demo render-profile lod
./hospital-demo render-profile spacewarp
./hospital-demo render-profile combined
```

Use one command at a time. `full` restores source geometry and native frames.
For comparable tests it prewarms the same bounded LOD cache, without displaying
it. `lod` uses colour-aware, locked-border simplified pages when their projected
error fits a one-pixel threshold in both eyes. Original LOD0/source geometry is
retained; lower LODs are a visual approximation, not lossless geometry. Up to
96 MiB of extra LOD geometry is admitted, with full-detail fallback when unavailable.

`spacewarp` keeps full geometry and requests Meta Application SpaceWarp.
`combined` requests both. SpaceWarp synthesizes display frames: for example,
36 real application frames plus generated frames is not 72 native application FPS.
It requires the supported Vulkan/OpenXR runtime, is off in existing registrations,
and is suspended during Teach or active teleoperation. Thin edges, occlusion
boundaries and transparent surfaces can still show reprojection artifacts.
No profile lowers text resolution, MSAA, global render scale or source map coverage.
Allow the background LOD cache to warm before comparing profiles. Inspect
`MeshRenderHealth` for selected triangles/cache bytes and requested/API/native
SpaceWarp state; resident source triangles are not actual per-pass draw counts.

