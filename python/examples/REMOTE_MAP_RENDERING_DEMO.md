# Remote 3D Map Rendering

HORUS remote rendering keeps dense geometry on the PC and sends a
pose-synchronized RGB-D view to the Quest. The PC rasterizes textured
triangles, anisotropic Gaussian splats, or scan points with the NVIDIA GPU.
The Quest receives color plus metric depth and reconstructs a bounded spatial
surface in the workspace; it never receives the source scene geometry.

This is a first-class workspace visualization. It is not a robot camera, does
not appear in Robot Manager, and does not require the Unity Editor at runtime.

## Architecture

The runtime has one rendering design and two carriers:

- `ros` is the validated default. It packs matching JPEG color and lossless
  metric depth into one replace-latest ROS message at up to 15 FPS.
- `webrtc` is experimental. H.264 carries color at up to 60 FPS and a paired
  data channel carries metric depth, source pose, and projection.

Both transports use the same renderer, frame protocol, Quest reprojection
shader, scene coordinates, and SDK registration. They are not different
rendering modes.

The WebRTC implementation is retained for the next remote-rendering
improvement branch, but it is not the release default. Its depth frames are
fragmented into bounded data-channel messages by the HORUS ROS 2 bridge and
reassembled on Quest.

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy
- the matching `horus_ros2` workspace built and sourced
- an NVIDIA GPU with a working CUDA toolchain
- a HORUS APK built from matching MR, SDK, and ROS 2 revisions
- `nvdiffrast` for textured meshes and `gsplat` for Gaussian scenes

Install the PC rendering dependencies once:

```bash
cd ~/horus_sdk
python3 python/examples/tools/install_remote_render_dependencies.py
```

The installer verifies CUDA-enabled PyTorch before returning.

## Profiles

Color dimensions are for the single predicted center-eye render. Quest
reprojects that render independently for each display eye.

| Profile | Color | Metric depth | Experimental WebRTC target | Bitrate | ROS default |
|---|---:|---:|---:|---:|---:|
| `fast` | 896x896 | 448x448 | 60 FPS | 20 Mbps | up to 15 FPS |
| `balanced` | 1152x1152 | 384x384 | 60 FPS | 35 Mbps | up to 15 FPS |
| `quality` | 1344x1344 | 448x448 | 60 FPS | 50 Mbps | up to 15 FPS |

Start with `balanced`. Actual frame rate depends on scene complexity, CUDA
render time, video encoding, Wi-Fi, and headset load.

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

The default transport is the validated ROS-compressed path:

```bash
python3 python/examples/remote_map_portal_registration.py \
  --scene synthetic \
  --profile balanced \
  --workspace-scale 0.1
```

The legacy `ros_debug` spelling remains accepted as an alias. To explicitly
try the experimental WebRTC carrier, add:

```text
--transport webrtc
```

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

The full scans remain unchanged under
`~/.cache/horus/eth3d_remote_maps`. The default 12.5 mm PC render cache is a
deterministic source LOD; use `--source-voxel-size` only when deliberately
trading PC GPU memory against source density.

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
        "transport": "ros_compressed",
        "ros_compressed_topic": "/horus/remote_render/rgbd",
        "viewer_pose_topic": "/horus/remote_render/viewer_pose",
    },
)
```

The SDK derives the protocol version from `transport`; callers do not select
an incompatible format manually. Omitting `transport` also selects
`ros_compressed`.

`add_3d_map(..., render_target="quest")` remains available for bounded geometry
that fits the headset memory and triangle budgets. For remote maps, use either
`add_remote_rendered_map(...)` or
`add_3d_map(..., render_target="remote")`. Quest-only `static` and `refresh`
update modes do not apply to remote rendering.

## Runtime topics and channels

| Topic or channel | Direction | Purpose |
|---|---|---|
| `/horus/remote_render/viewer_pose` | Quest to PC | current and predicted eye poses and projections |
| `/horus/remote_render/map_portal` | PC to bridge | raw RGB source for H.264 encoding |
| `/horus/remote_render/frame_data` | PC to bridge | matching metric depth and frame metadata |
| H.264 video track | bridge to Quest | WebRTC color |
| `horus.remote-render.v1` | bridge to Quest | fragmented WebRTC depth and metadata |
| `/horus/remote_render/rgbd` | PC to Quest | paired ROS color, depth, pose, and projection |
| `/horus/remote_render/agent_status` | PC | renderer health and timing |

The render agent uses replace-latest ROS QoS for high-rate source frames.
Quest retains the previous complete frame until a newer exact color/depth pair
is ready. The experimental WebRTC carrier additionally bounds its bridge
data-channel queue.

## Diagnostics

Check the PC renderer:

```bash
ros2 topic hz /horus/remote_render/viewer_pose
ros2 topic echo --once /horus/remote_render/agent_status
```

For the default ROS carrier:

```bash
ros2 topic hz /horus/remote_render/rgbd
```

For an experimental WebRTC run:

```bash
ros2 topic hz /horus/remote_render/map_portal
ros2 topic hz /horus/remote_render/frame_data
```

The render agent writes:

```text
~/.cache/horus/remote_map/preview.png
~/.cache/horus/remote_map/agent.log
```

The preview confirms that the PC renderer loaded the scene. It is not streamed
to Quest and does not influence map placement.

Before treating an experimental WebRTC run as valid, bridge session telemetry
must show:

- a connected peer and open video track;
- increasing encoded video units;
- an open `horus.remote-render.v1` data channel;
- increasing auxiliary sent frames without sustained queue growth.

The Quest keeps the last complete spatial frame visible while a newer frame is
rendered, encoded, transported, and paired. It never displays the server's
startup preview or a color frame without its matching metric depth.

If a fixed CycloneDDS interface from an old network no longer exists, clear it
before launching:

```bash
unset CYCLONEDDS_URI
ros2 daemon stop
ros2 daemon start
```

## Design references

The remote-rendering work was informed by public documentation and examples
from Azure Remote Rendering, the 3D Streaming Toolkit, and ALVR. These projects
were consulted as conceptual references for remote camera synchronization,
video delivery, and local composition. The HORUS implementation was developed
independently and does not include source code from these projects.

- Azure Remote Rendering:
  <https://github.com/Azure/azure-remote-rendering>
- 3D Streaming Toolkit:
  <https://github.com/3DStreamingToolkit/3DStreamingToolkit>
- ALVR: <https://github.com/alvr-org/ALVR>
