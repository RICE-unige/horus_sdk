# Remote 3D Map Rendering

HORUS remote rendering keeps dense geometry on the PC and sends a
pose-synchronized stereo view to the Quest. The PC rasterizes textured
triangles, anisotropic Gaussian splats, or scan points at full source density
with the NVIDIA GPU. The Quest hardware-decodes a fixed-size video carrier and
composites one stereo surface; it never receives or draws source geometry.

This is a first-class workspace visualization. It is not a robot camera, does
not appear in Robot Manager, and does not require the Unity Editor at runtime.

## Architecture

The current experimental runtime has one primary path and two rollback/diagnostic paths:

- V3 `webrtc` is the default. It sends per-eye H.264 with a 60 FPS target;
  frame identity and quantized metric depth travel with the same encoded
  frame.
- V3 `ros` is a low-rate PNG diagnostic carrier for isolating transport faults.
- V2 RGB-D remains available explicitly with `--protocol v2` for rollback.

V3 follows a CloudXR-style split: predicted OpenXR eye views go to the PC,
rendering and H.264 encoding stay on the PC, and the Quest performs only
hardware decode plus a bounded GPU composition pass. No Unity depth mesh is
rebuilt per frame.

The V3 depth atlas stores 16-bit linear metric depth as four 4-bit
luma symbols. This reduces the range-conversion errors of a single 8-bit
depth value, but H.264 is lossy: this is not lossless depth transport. Quest
reconstructs each point from the source eye projection and pose, then places it
under the workspace map root. Workspace scale is applied exactly once, so the
map is composed in the workspace frame rather than attached to the current eye.
Occlusion changes can still expose surfaces absent from an older source view;
the architecture does not guarantee complete coverage under arbitrary motion.

Encoded H.264 access units are delivered in FIFO order. Only unencoded renders
may be replaced by newer work. On a bridge queue overflow or an encoder-index
gap, the bridge waits for the next IDR instead of forwarding a broken prediction
chain. Periodic IDRs are configured once per target second. Receiver PLI/FIR
requests and track startup also request a fresh IDR through
`/horus/remote_render/map_portal/request_keyframe`. The bridge coalesces requests
to at most two per second, and the encoder emits SPS/PPS with the next IDR.

The Quest pairs metadata with a marker decoded from an immutable video snapshot,
not the arrival order of pre-decode metadata. Three snapshot slots bound memory;
GPU readback transfers only the eight-byte identity. Workspace changes invalidate
pending snapshots. Depth samples address texel centers within their own eye.
The marker uses 32-pixel-wide, macroblock-aligned complementary symbols on the
standard profiles (8 pixels on small diagnostic carriers). The decoder requires
high contrast, so scene pixels at the opposite image edge cannot qualify on the
magic byte alone. Rebuild the APK together with the SDK for this carrier update.

Network bytes, decoded frames, and accepted map frames are separate progress
signals. Polling an unchanged texture does not reset stream health. After the
initial six-second negotiation grace, a decoder stall of 2.5 seconds or an
accepted-frame stall of five seconds can trigger a rate-limited session recovery.
This is fault recovery, not a substitute for meeting the frame-time budget.
`HORUS_REMOTE_RENDER_V3` diagnostics include the decoded-frame count, image
marker, latest metadata frame/view, presentation age, and decoder statistics.

The implementation is not the NVIDIA CloudXR runtime. PC rendering and
independent decoder tests do not establish Quest motion-to-photon performance;
a worn-headset sustained-motion test remains required after client changes.

### Current limits

V3 currently renders a loaded, static source asset for one active viewer per
agent. It is not yet a live nvblox mesh consumer or an independent multi-operator
render farm. Retaining full source geometry does not make the transmitted image
or depth lossless: depth has lower spatial resolution than color, the identity
strip occupies carrier rows, and H.264 can alter depth symbols. Thin surfaces,
depth discontinuities, transparency, and newly exposed regions during motion
therefore remain device-acceptance cases, not guaranteed full-fidelity results.

The Quest compositor draws a procedural reprojection grid. It avoids rebuilding
a CPU mesh every frame, but still has a GPU triangle/fragment cost. Local HORUS
text and controls retain their own rendering resolution.

For static textured meshes, the CUDA renderer now reuses one texture atlas,
its mip chain, and its antialias topology cache across eyes and frames.
Loading/render stream dependencies are explicit. The GPU regression test
compares cached and uncached stereo color/depth output; it does not establish
wireless or headset performance.

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy
- the matching `horus_ros2` workspace built and sourced
- an NVIDIA GPU with a working CUDA toolchain
- a HORUS APK built from matching MR, SDK, and ROS 2 revisions
- `nvdiffrast` for textured meshes and `gsplat` for Gaussian scenes

Inspect the PC rendering dependency plan without installing packages or starting CUDA:

```bash
cd ~/horus_sdk
python3 python/examples/tools/install_remote_render_dependencies.py --plan
```

On the shared workstation, dependencies are provisioned centrally. The helper
defaults to a read-only plan, pins Torch 2.11.0 from the CUDA 13.0 wheel index,
and refuses installation into the shared Isaac/Lab environment or base Python.
An explicitly selected, separate HORUS-only virtual environment can use
`--install` (and optionally `--force`); neither option is for the shared runtime.

The selected NVENC overlay must contain the isolated, MIT-licensed
PyNvVideoCodec 2.2.2 adapter rebuilt from its SHA-256-pinned NVIDIA source.
It exposes the upstream output-delay argument, H.264 Baseline profile and explicit
colour-VUI signalling for the RGB/depth atlas;
stock PyNvVideoCodec is not a substitute. CUDA's `nvcc` and a C++ toolchain
are required. Do not copy an old compiled overlay into a new Torch/CUDA runtime.

For a separately approved build, the coordinator selects the new overlay first:

```bash
export HORUS_NVENC_ROOT="/path/to/the/new/runtime-cache/nvenc-color-vui-v1"
# Run only after the coordinator provisions the matching compiler/build dependencies.
python3 python/examples/tools/install_remote_nvenc.py
```

Its source, wheel, and provenance receipt live under
`$HORUS_NVENC_ROOT` (or the legacy default
`~/.cache/horus/remote_render/nvenc-low-latency-v2/` outside the shared profile),
not in the repository.

### Shared Isaac 6.2 consumer profile

The machine-local `python/examples/.local_assets/mapping-runtime.sh` opts into
the tracked `python/examples/tools/mapping_runtime_shared.sh`, which selects
`~/isaac/env_isaacsim/bin/python` for `./mapping-demo remote-*`.
Native map playback still uses the normal Jazzy/system-Python path. This
profile does not launch Sim/Lab, activate or edit its shared source, install
packages, or accept a license on the user's behalf.

Fresh artifacts default to
`~/.cache/horus/remote_render/isaacsim-6.2-py312-torch2.11-cu130/`:

| Item | Central provisioning requirement |
|---|---|
| Python / Torch | Canonical Python 3.12, Torch 2.11.0+cu130; do not upgrade Lab's Torch using the HORUS helper |
| `site-packages/` | gsplat 1.5.3, plyfile 1.1.3, ninja 1.13.0, trimesh 4.9.0 and their compatible non-Torch dependencies |
| nvdiffrast | Rebuild pinned commit `253ac4fcea7de5f396371124af597e6cc957bfae` against the canonical Torch/CUDA |
| `torch-extensions/` | Fresh gsplat/nvdiffrast JIT output; no copied CUDA 12.8 extensions |
| `native/` | Rebuild HORUS point/mesh `.cu` kernels using the approved CUDA 13 compiler |
| `nvenc-color-vui-v1/` | Patched 2.2.2 with output-delay, Baseline and explicit colour VUI; preserve prior NVENC caches and retain source/wheel/provenance receipts |

The shared profile selects the qualified colour-VUI adapter. Its H.264 stream
declares limited-range BT.601; auxiliary depth and identity symbols retain their
existing receiver contract. An older adapter that silently omits the VUI option
is rejected. Explicit `HORUS_NVENC_ROOT` overrides must select a compatible build.

ROS imports (`rclpy`, `sensor_msgs`, `std_msgs`, `visualization_msgs`) must resolve
from the existing Jazzy/HORUS overlay. Media also requires NumPy, Rich, OpenCV,
Pillow and gsplat's jaxtyping dependency. For central overlay provisioning use
`--no-deps` and supply compatible prerequisites separately, so pip does not
install another Torch into the overlay. The pinned NVENC source declares
setuptools, wheel, scikit-build, CMake, Ninja and NumPy as build dependencies.

The profile removes the superseded October 8 HORUS Python overlay and CUDA 12.8
selectors while preserving unrelated ROS paths. When the approved toolkit path
is known, `HORUS_REMOTE_CUDA_HOME` selects it explicitly. A missing explicitly
selected compiler, Python or NVENC overlay fails rather than falling back to
the old environment. The normal CUDA compiler search still needs verification
if no explicit toolkit was selected.

`HORUS_REMOTE_PYTHON`, `HORUS_REMOTE_CACHE_ROOT` and `HORUS_NVENC_ROOT` remain
explicit overrides for other installations; users are responsible for matching
their chosen ABI. Existing caches, recordings and data manifests are preserved.
The full Sim 6.2 asset pack is not a dependency of recorded HORUS map rendering.

Provisioning and exclusive GPU acceptance are separate gates. CPU path tests
do not qualify CUDA 13 support, rendering correctness, NVENC, FPS or headset
performance. G1 Demonstrations and its policy environments are not migrated or
resumed by this profile.

For direct Python launches after provisioning, use the profile's
`$HORUS_REMOTE_PYTHON` rather than assuming system `python3` has the GPU overlay.

## Profiles

### Compare a saved HMSH map with headset rendering

The saved-map launcher can render the same verified fixture on the PC. Stop
the local replay first, then run:

```bash
cd ~/horus_sdk
./replay-map /path/to/saved-map/measured --render-target remote --profile quality
```

The launcher selects the existing GPU runtime, starts the renderer and bridge,
and registers a remote workspace map. Connect the Quest to the PC on port
10000 and create a workspace. No new scan is required. To return to local
rendering, stop the remote replay and omit `--render-target remote`.

Every source page is checksum-verified. All vertices, triangles, winding and RGB
colours are retained; the same source-floor transform is applied. RGBA sources
must be opaque: non-opaque alpha is rejected, not silently discarded. Derived
RGB arrays are verified before cache reuse and originals are never overwritten.
Use `--remote-cache /path/on/a/larger/disk` to choose their location.

This compares full source geometry, not identical output pixels. The remote
path still uses the stereo video/depth encoding described above. Compare
moving-head stability, image detail and latency as well as Quest FPS. This
is a static snapshot, not live remote nvblox updates.

Color dimensions are per eye. Source map density is independent of this table.

| Profile | Per-eye color | WebRTC target | Bitrate | ROS diagnostic |
|---|---:|---:|---:|---:|
| `fast` | 896x896 | 60 FPS | 20 Mbps | up to 15 FPS |
| `balanced` | 1152x1152 | 60 FPS | 35 Mbps | up to 15 FPS |
| `quality` | 1344x1344 | 60 FPS | 50 Mbps | up to 15 FPS |

Start with `balanced`. Actual frame rate depends on scene complexity, CUDA
render time, video encoding, Wi-Fi, and headset load.

The profile changes only the fixed stereo image sent to the Quest. It does not
remove triangles, points, splats, textures, or spherical-harmonic coefficients
from the PC scene. Point-cloud voxel reduction is disabled by default
(`--source-voxel-size 0.0`) and can only be enabled explicitly. Output pixels
and H.264 bitrate still bound the detail visible through the headset, so use
`quality` when the PC, encoder, and wireless link sustain it.

Textured meshes and Gaussian scenes hand CUDA tensors directly to NVENC.
Gaussian rendering retains its original anisotropic splats, antialiasing, and
spherical-harmonic colors; it no longer downloads stereo color/depth images to
CPU memory and uploads them again for WebRTC. ROS diagnostics still use CPU PNG
encoding, and the point-cloud fallback still incurs CPU image transfers.

## Common setup

Run this setup before an example:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash

export ROS_DOMAIN_ID=10
export PYTHONPATH="$PWD/python:$PYTHONPATH"
```

Run one registration at a time. Stop it with `Ctrl+C` before starting another.
The launcher starts the PC render agent, waits for renderer initialization,
registers the workspace visualization, and starts the HORUS bridge through the
normal SDK lifecycle.

The default command uses V3 WebRTC, NVENC, and full source geometry:

```bash
python3 python/examples/remote_map_portal_registration.py \
  --scene synthetic \
  --profile balanced \
  --workspace-scale 0.1
```

Use the ROS diagnostic carrier when isolating WebRTC behavior:

```bash
python3 python/examples/remote_map_portal_registration.py \
  --transport ros \
  --scene synthetic \
  --profile balanced \
  --workspace-scale 0.1
```

## Remote rendering v3 acceptance path

`horus.remote_render.v3` sends the OpenXR predicted display time, independent
left/right eye poses, and each eye's asymmetric field of view to the PC. WebRTC
returns an identity-tagged side-by-side stereo H.264 frame. ROS returns the same
views as a PNG diagnostic and should not be used for performance measurements.

Run the geometric conformance scene:

```bash
python3 python/examples/remote_map_portal_registration.py \
  --protocol v3 \
  --transport ros \
  --scene synthetic \
  --profile balanced \
  --workspace-scale 0.1
```

Run the textured Sponza acceptance scene:

```bash
python3 python/examples/remote_textured_mesh_map_registration.py \
  --protocol v3 \
  --transport ros \
  --scene sponza_mesh \
  --profile balanced \
  --workspace-scale 0.1
```

During both runs, verify slow and fast head rotation, lateral translation,
fore/aft translation, asymmetric eye projection, workspace redraw, and
workspace scale changes. The map must remain world-locked, the eyes must not
be swapped or duplicated, and frames from an earlier workspace revision must
not reappear.

The automated renderer gates are:

```bash
python3 -m pytest -q \
  python/tests/test_remote_rendering_v3_protocol.py \
  python/tests/test_remote_rendering_v3_agent.py \
  -o addopts=

python/tests/integration/run_remote_rendering_v3_ros_probe.sh
HORUS_V3_INTEGRATION_SCENE=sponza_mesh \
  python/tests/integration/run_remote_rendering_v3_ros_probe.sh
```

The opt-in PC throughput/continuity test uses the real GPU, NVENC, ROS publisher,
and ROS subscriber, without connecting to a robot or headset. It checks that the
last view is delivered after pose input stops and that encoded access units have
no missing indices. The separate round-trip test checks depth symbols through
NVENC and FFmpeg decode:

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/python:$PYTHONPATH"
ROS_DOMAIN_ID=217 CYCLONEDDS_URI= HORUS_TEST_GPU_VIDEO=1 python3 -m pytest -q -s \
  python/tests/test_remote_rendering_live_pipeline.py \
  python/tests/test_remote_rendering_video_roundtrip.py -o addopts=
```

This does **not** measure Quest FPS, wireless delivery, hardware decoder latency,
or motion-to-photon latency. For device acceptance, record Quest frame time and
`HORUS_REMOTE_RENDER_V3` counters alongside `HORUS_REMOTE_RENDER_DIAG` on the PC.
Check rotation, translation, a stationary head, reconnect, and workspace redraw.
Do not infer headset performance from the PC throughput number.

### Local pipeline measurements

The RTX 4090 check on 2026-09-17 used the automated moving-eye trajectory above,
1152x1152 color per eye, 60 Hz pose input, and the complete loaded source scene.
Each run drove poses for 12 seconds, excluded the first two seconds from the
throughput calculation, and then stopped input for three seconds to test drain.

| Scene | Loaded geometry | PC output FPS | Missing encoded indices | Final pose delivered |
|---|---:|---:|---:|---|
| Sponza | 262,267 triangles | 59.83 | 0 | Yes |
| Gaussian Train | 1,074,761 splats | 45.70 | 0 | Yes |

These are short, scene/view-specific regression checks, not operating-envelope
benchmarks. They stop at the local ROS subscriber, before the WebRTC bridge,
wireless network, or Quest decoder/compositor. Neither demonstrates 60 FPS for
all viewpoints or sustained thermal behavior. The separate NVENC/decode test
passed the depth-symbol checks; headset motion stability remains a device gate.

To repeat the Gaussian run, add `HORUS_RENDER_SCENE=gaussian_train` to the test
environment above. The assets must already be downloaded.

## Scene coordinates

Every source adapter produces one canonical HORUS scene:

- Y is up.
- the physical floor is at Y=0.
- X and Z form the workspace floor plane.
- the startup preview camera validates the PC renderer only; it never defines
  live workspace alignment.
- Quest poses are already expressed in the remote-map workspace frame and are
  used directly for live rendering.

This prevents a dataset-specific preview angle from tilting the map and
prevents a first-view anchor from making the scene follow the headset.

## Textured triangle meshes

Fetch all mesh showcases:

```bash
python3 python/examples/tools/fetch_remote_render_advanced_maps.py --asset mesh
```

### Khronos Sponza

```bash
python3 python/examples/remote_textured_mesh_map_registration.py \
  --scene sponza_mesh \
  --profile balanced \
  --workspace-scale 0.1
```

Sponza preserves its indexed triangles, UVs, source textures, and material
tints. Textures are packed without downsampling into a padded, mip-compatible
atlas.

Source:
<https://github.com/KhronosGroup/glTF-Sample-Assets/tree/main/Models/Sponza>

### San Miguel 2.0

```bash
python3 python/examples/remote_textured_mesh_map_registration.py \
  --scene san_miguel_mesh \
  --profile balanced \
  --workspace-scale 0.1
```

Source:
<https://casual-effects.com/data>

Retain and follow the license bundled with each downloaded mesh dataset.

## Gaussian splat scenes

Gaussian splatting is experimental in this release. The scenes are useful for
continued renderer development, but the textured-mesh path is the validated
showcase for this PR.

Fetch all four trained 30,000-iteration scenes:

```bash
python3 python/examples/tools/fetch_remote_render_advanced_maps.py \
  --asset gaussian
```

The loader preserves every anisotropic Gaussian, opacity, scale, rotation, and
available spherical-harmonic coefficient. It does not convert the model to
points or triangles.

### Deep Blending Dr Johnson

```bash
python3 python/examples/remote_gaussian_splat_map_registration.py \
  --scene gaussian_drjohnson \
  --profile balanced \
  --workspace-scale 0.1
```

### Deep Blending Playroom

```bash
python3 python/examples/remote_gaussian_splat_map_registration.py \
  --scene gaussian_playroom \
  --profile balanced \
  --workspace-scale 0.1
```

### Tanks and Temples Train

```bash
python3 python/examples/remote_gaussian_splat_map_registration.py \
  --scene gaussian_train \
  --profile balanced \
  --workspace-scale 0.1
```

### Tanks and Temples Truck

```bash
python3 python/examples/remote_gaussian_splat_map_registration.py \
  --scene gaussian_truck \
  --profile balanced \
  --workspace-scale 0.1
```

The first Gaussian launch can take several minutes while CUDA extensions are
compiled. Later launches reuse the user cache.

Dataset and license:
<https://huggingface.co/datasets/Voxel51/gaussian_splatting> (Apache-2.0)

## Point and reconstructed-surface scenes

These examples exercise the same remote renderer with real scan data.

### Voxblox Cow and Lady

Download the extras without the 4.6 GB ROS 1 bag and prepare the triangle
surface:

```bash
python3 python/examples/tools/fetch_voxblox_cow_lady.py --extras-only
mkdir -p ~/.cache/horus/voxblox_cow_lady/extras
python3 -m zipfile -e \
  ~/.cache/horus/voxblox_cow_lady/voxblox_cow_extras.zip \
  ~/.cache/horus/voxblox_cow_lady/extras
python3 python/examples/tools/prepare_cow_lady_surface.py
```

Run:

```bash
python3 python/examples/remote_map_portal_registration.py \
  --scene cow_lady \
  --profile balanced \
  --workspace-scale 0.1
```

Dataset:
<https://www.research-collection.ethz.ch/entities/researchdata/ded5ea04-6ec7-42e9-a5ca-6062cf83507c>

### ETH3D Courtyard

```bash
python3 python/examples/tools/fetch_eth3d_courtyard.py

python3 python/examples/remote_map_portal_registration.py \
  --scene eth3d_courtyard \
  --profile balanced \
  --workspace-scale 0.1
```

### ETH3D showcase set

Fetch Delivery Area, Electro, Facade, Playground, and Terrains:

```bash
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 python/examples/tools/fetch_remote_render_maps.py
```

Run any scene by ID:

```bash
python3 python/examples/remote_map_portal_registration.py \
  --scene delivery_area \
  --profile balanced \
  --workspace-scale 0.1

python3 python/examples/remote_map_portal_registration.py \
  --scene electro \
  --profile balanced \
  --workspace-scale 0.1

python3 python/examples/remote_map_portal_registration.py \
  --scene facade \
  --profile balanced \
  --workspace-scale 0.1

python3 python/examples/remote_map_portal_registration.py \
  --scene playground \
  --profile balanced \
  --workspace-scale 0.1

python3 python/examples/remote_map_portal_registration.py \
  --scene terrains \
  --profile balanced \
  --workspace-scale 0.1
```

The full scans remain unchanged under `~/.cache/horus/eth3d_remote_maps` and
are rendered in full by default. `--source-voxel-size` is an explicit opt-in
host-side point-cloud LOD and is never applied automatically.

ETH3D is licensed under CC BY-NC-SA 4.0. Cite:

> T. Schops et al., "A Multi-View Stereo Benchmark with High-Resolution Images
> and Multi-Camera Videos," CVPR, 2017.

Dataset: <https://www.eth3d.net/datasets>

## SDK API

Use `add_remote_rendered_map` when a PC process owns the render source:

```python
dataviz.add_remote_rendered_map(
    stream_topic="/horus/remote_render/map_portal",
    frame_id="map",
    render_options={
        "protocol_version": "horus.remote_render.v3",
        "transport": "webrtc",
        "viewer_pose_topic": "/horus/remote_render/v3/view_state",
    },
)
```

V3 WebRTC is the launcher default. V2 and the V3 ROS diagnostic remain
available explicitly and never replace V3 silently.

`add_3d_map(..., render_target="quest")` remains available for bounded geometry
that fits the headset memory and triangle budgets. For remote maps, use either
`add_remote_rendered_map(...)` or
`add_3d_map(..., render_target="remote")`. Quest-only `static` and `refresh`
update modes do not apply to remote rendering.

## Runtime topics and channels

The default V3 WebRTC route uses these channels:

| Topic or channel | Direction | Purpose |
|---|---|---|
| `/horus/remote_render/v3/view_state` | Quest to PC | predicted display time and exact per-eye views |
| `/horus/remote_render/map_portal` | PC renderer to bridge | fixed-size stereo RGB and metric-depth atlas for NVENC |
| H.264 video track | bridge to Quest | hardware-decoded stereo atlas with integrated frame identity |
| `/horus/remote_render/v3/status` | PC | renderer health, source count, timing, and rejection counters |

The V3 ROS diagnostic route replaces the H.264 track with
`/horus/remote_render/v3/stereo`. The legacy V2 topics
`/horus/remote_render/viewer_pose`, `/horus/remote_render/frame_data`,
`/horus/remote_render/rgbd`, and the `horus.remote-render.v1` data channel are
used only when `--protocol v2` is selected explicitly.

The render agent and Quest both use newest-frame semantics. Stale work is
discarded rather than queued, and the previous complete stereo frame remains
visible until a newer identity-matched frame is ready.

The Quest receiver permits metadata and decoded video to arrive in either order.
It retains at most three owned video snapshots, matches their embedded frame
identities exactly, and expires unmatched snapshots at the configured maximum
frame age. It never associates delayed metadata with the decoder's latest
unrelated texture. Color filtering is clamped within each eye's content region
to avoid blending with the other eye, marker, or depth strip.

Timing-log compatibility names need care: `source_to_media_ms` ends at metadata
arrival, not necessarily decoded video arrival. `media_to_capture_ms` is signed
and can be negative when metadata arrives after capture. `readback_ms` spans
capture to pair acceptance, including metadata wait; it is not a GPU execution
timer. Use total view-sample-to-acceptance age and actual presentation counters
alongside these stages, rather than adding unrelated clock domains or claiming
motion-to-photon timing.

This remains the V3 contract: depth is spatially reduced and carried inside a
lossy video stream. These receiver fixes do not provide protected full-resolution
depth, live resident scene-page updates, or independent concurrent viewers from
one agent. Treat those capabilities as unfinished, not as consequences of
successful V3 transport tests.

V3 is a matched transport contract. An APK containing the four-symbol depth
decoder must be used with an SDK renderer that advertises
`linear_u16_nibbles_in_video`; older depth encodings are rejected rather than
displayed with incorrect geometry.

## Diagnostics

Check the PC renderer:

```bash
ros2 topic hz /horus/remote_render/v3/view_state
ros2 topic echo --once /horus/remote_render/v3/status
```

For the default WebRTC carrier, confirm that the raw stereo source reaches the
local bridge. This topic is intentionally local to the PC and is not decoded by
the Quest as ROS data:

```bash
ros2 topic hz /horus/remote_render/map_portal
```

For a V3 ROS diagnostic run:

```bash
ros2 topic hz /horus/remote_render/v3/stereo
```

The render agent writes:

```text
~/.cache/horus/remote_map/preview.png
~/.cache/horus/remote_map/agent.log
```

The preview confirms that the PC renderer loaded the scene. It is not streamed
to Quest and does not influence map placement.

Before treating a WebRTC run as valid, bridge session telemetry
must show:

- a connected peer and open video track;
- increasing encoded video units;
- increasing incoming and pushed frame counts without sustained queue growth;
- NVENC selected when `--encoder nvenc` is requested.

The Quest keeps the last accepted stereo frame visible while a newer frame is
rendered, encoded, transported, decoded, and identity-checked. V3 never builds
a map mesh or point cloud on the Quest.

If a fixed CycloneDDS interface from an old network no longer exists, clear it
before launching:

```bash
unset CYCLONEDDS_URI
ros2 daemon stop
ros2 daemon start
```
