---
title: "Tutorial 6: Global Maps"
sidebar_position: 7
toc_max_heading_level: 2
---

# Tutorial 6: Global Maps

This tutorial explains how shared world layers are added on top of the robot-description flow.

For isolated Quest checks from a source checkout, run `./mapping-demo indexed`
from the SDK root. It starts the source, adapter (where needed), and SDK
registration, including automatic bridge startup. The complete command list and
expected results are in `python/examples/MAPPING_VISUALIZATION_TESTING.md`.

The curated SDK now keeps each major map family in its own registration example so the behavior is easy to isolate:

- occupancy grid
- pointcloud
- dense mesh
- octomap
- Gaussian splat fixture with pointcloud fallback

For live triangle meshes with change detection and reconnect repair, see
[Live mesh delivery](../python-sdk/live-mesh.md). This source-independent SDK
adapter is separate from the occupancy-grid and raw point-cloud paths.

## Goal

By the end of this step, you should understand:

- why map layers are documented separately from the base robot-description registration
- which map family matches which operator need
- how the SDK expresses these as workspace-level DataViz layers

## Occupancy grid

Use this when you want the clearest 2D navigation context:

```python
world_layers.add_occupancy_grid(
    "/map",
    frame_id="map",
    render_options={"color_free": "#222222", "color_occupied": "#F4F4F4", "alpha": 0.85},
)
```

Reference script:

- `python/examples/occupancy_map_registration.py`

## Pointcloud

Use this when the source data is a dense point set and you want direct spatial evidence instead of a meshed surface:

```python
world_layers.add_3d_map(
    "/map_3d",
    frame_id="map",
    render_options={
        "point_size": 0.035,
        "auto_point_size_by_workspace_scale": True,
        "min_point_size": 0.0015,
        "max_point_size": 0.012,
        "point_shape": "circle",
        "render_mode": "opaque_fast",
        "render_all_points": True,
        "color": "#6ED7FF",
    },
)
```

Reference script:

- `python/examples/pointcloud_map_registration.py`

`render_all_points=True` retains every accepted point; it does not apply the
adaptive visible-point budget. The GPU renderer checks both eye frusta in XR.
Its full-detail admission budget defaults to 512 MiB of estimated map buffers,
configured on `PointCloudVisualizerGPU`, not through this registration payload.
An update that exceeds this budget is rejected with a resource error and the
previous map remains visible. This is not a whole-application RAM guarantee:
camera textures, robot models, and other maps need memory too. CPU submission
timings in the renderer log are not GPU execution timings.

## Dense mesh

Use this when you want a more solid environmental surface:

```python
world_layers.add_3d_mesh(
    "/map_3d_mesh",
    frame_id="map",
    render_options={
        "max_triangles": 2000000,
        "use_vertex_colors": True,
        "double_sided": False,
        "use_indexed_binary": True,
        "static_mesh": True,
        "max_chunk_renderers": 128,
        "max_chunk_uploads_per_frame": 2,
        "max_triangle_uploads_per_frame": 20000,
        "enable_perf_diagnostics": True,
    },
)
```

For dense static maps, publish compact indexed chunks rather than
`visualization_msgs/Marker` triangle lists. The provided publisher Morton-orders
triangles before chunking, so each Unity renderer has spatially useful bounds for
frustum culling. A 20,000-triangle chunk stays below the 16-bit worst-case vertex
limit and keeps the 1.2-million-triangle Cow & Lady surface to about 60 renderers.

```bash
python3 python/examples/tools/ply_to_horus_mesh_marker.py \
  --mode voxel_surface \
  --voxel-size 0.02 \
  --transport indexed_binary \
  --chunk-max-triangles 20000 \
  --republish-interval 0
```

The triangle budget normally permits one 20,000-triangle chunk per frame with this profile,
then releases the CPU-side mesh copies after the static snapshot settles. Use
`static_mesh=False` only for maps whose geometry genuinely changes after load.

Reference script:

- `python/examples/mesh_map_registration.py`

### Isolated Quest performance test

Use the dedicated benchmark registration when measuring the native map renderer.
It registers one tiny TF-only anchor and no robot models, cameras, sensor panels,
or remote-rendering components, so OVR Metrics results describe the mesh path.

Start a static anchor transform:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id map \
  --child-frame-id native_map_anchor
```

Start the indexed Cow & Lady publisher in a second terminal:

```bash
python3 python/examples/tools/ply_to_horus_mesh_marker.py \
  --topic /map_3d_mesh \
  --frame-id map \
  --mode voxel_surface \
  --voxel-size 0.02 \
  --transport indexed_binary \
  --chunk-max-triangles 20000 \
  --republish-interval 0
```

Start the isolated registration in a third terminal:

```bash
PYTHONPATH=python:$PYTHONPATH \
python3 python/examples/native_mesh_map_performance_registration.py \
  --workspace-scale 0.1 \
  --mesh-max-triangles 2000000
```

Capture a baseline before starting the mesh publisher, then capture the same
headset motion after the map has loaded and the Unity log reports that all static
GPU mesh chunks were finalized. In OVR Metrics Tool's Basic preset, compare FPS,
App GPU time, CPU/GPU utilization, stale frames, memory, and thermal throttling.

Static finalization is opt-in (`static_mesh=True`), not a global map default.
It uploads exact-size vertex/index buffers and releases their CPU-readable copy
after the map settles. It does not decimate geometry, alter colors, or lower
headset resolution. A later update recreates only its affected chunk. Indexed
uploads obey both the chunk-count and triangle budget per frame; one indivisible
chunk can exceed the triangle budget, so publisher chunk sizing still matters.
Dynamic maps keep reusable CPU buffers to avoid repeated allocation. Keep the
registered triangle/chunk limits high enough for the supplied map: these remain
explicit capacity limits, not a guarantee that arbitrary scenes fit Quest memory.

### Live nvblox mesh

Use the dedicated nvblox path for a map being reconstructed by Isaac ROS nvblox.
It consumes `nvblox_msgs/msg/Mesh` block updates on the PC and sends indexed
spatial page replacements to Quest. Normal updates send only changed pages;
reconnect or repair can replay retained pages. It does not decimate triangles.
Positions use float32 and colors use RGBA8 on the wire, so this is not a claim
of mathematically lossless conversion from arbitrary source attributes.
Clear, block deletion, map generations, stale-update
rejection, and reconnect snapshot replay are part of the transport contract.

The SDK-level registration is intentionally small:

```python
world_layers.add_nvblox_mesh(
    topic="/horus/nvblox/mesh",
    frame_id="map",
)
```

For a live Spot/nvblox graph, verify the source and start HORUS:

```bash
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export ROS_DOMAIN_ID=10
ros2 topic type /nvblox_node/mesh

cd ~/horus_sdk
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 python/examples/nvblox_mesh_map_registration.py \
  --input-topic /nvblox_node/mesh \
  --frame-id map \
  --workspace-scale 0.1 \
  --input-reliability auto \
  --max-publish-mib-per-second 8
```

The default profile merges each 4x4x4 neighborhood of nvblox blocks into one
renderer. This reduces draw calls without dropping geometry. Unchanged blocks
are not re-encoded, changed pages are coalesced fairly, and a byte-rate budget
limits output as well as the page-rate budget. A page larger than one frame's
allowance accumulates credit rather than starving behind smaller pages.

Quest prepares indexed geometry on a background worker. Unity/GPU uploads stay
on the main thread with chunk-count, triangle-count, and elapsed-time budgets.
The 2.5 ms target is checked between uploads, not a hard upper bound: one upload
cannot be interrupted. Larger `--blocks-per-axis` reduces renderer count but
increases individual page size and worst-case upload time. Measure both before
changing this setting.

The default `auto` input reliability waits for the source endpoint, chooses
reliable delivery when every source publisher supports it, and otherwise uses
compatible best-effort delivery. Source durability is also matched. Explicit
`reliable` or `best_effort` overrides are available. Best-effort input can lose
incremental blocks before HORUS sees them; downstream repair cannot reconstruct
those blocks. A source snapshot is also needed when an adapter starts after an
incremental-only producer has already built its map.

The adapter publishes an inventory on `<topic>/indexed_manifest`. Quest compares
epoch/page/revision entries with applied geometry and requests missing pages on
`<topic>/indexed_request`. Same-generation replay does not clear other viewers;
deletions are retained as bounded revisioned tombstones. Rejected/invalid updates
preserve the last valid map. `IsIndexedMapComplete` and `IndexedMissingPages` on
the MR renderer describe completion against the latest received inventory,
not proof that the upstream reconstruction supplied every block. With V2 manifest
delivery, a replacement epoch is staged invisibly until its received inventory
is complete, then swaps as a whole. The last complete map remains visible during
staging. Initial loads and legacy producers without inventories can still appear
progressively. Combined resident and staged geometry must fit the renderer's
estimated 512 MiB admission budget; an incomplete replacement is not reported as
complete. This requires both the updated SDK adapter and MR runtime.

MR mesh, point, robot-body and decoded-camera uploads share a soft per-frame CPU
budget. It bounds admission, not the execution time of a single Unity upload or
the GPU frame time, and does not reduce the geometry or text resolution.

For a repeatable test without Spot, run the official NVIDIA Isaac ROS nvblox
quickstart bag. The first command downloads the compact 314 MB NGC asset once;
the second starts bag playback, GPU nvblox reconstruction, the HORUS adapter,
bridge, and registration:

```bash
cd ~/horus_sdk
python3 python/examples/tools/fetch_nvblox_quickstart.py

source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
export ROS_DOMAIN_ID=10
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 python/examples/nvblox_quickstart_demo.py \
  --workspace-scale 0.1
```

The downloaded bag is stored under
`~/.cache/horus/nvblox_quickstart`; its source/version are recorded in
`manifest.json`. The nvblox launch log is written beside it.

For live Wi-Fi operation, keep nvblox's mesh publication rate modest (start at
5 Hz) and set its `layer_streamer_bandwidth_limit_mbps` to 30. HORUS preserves
the newest accepted revision of every page, so adapter byte pacing does not
require lowering mesh quality. Source bandwidth settings must preserve the
producer's own incremental-delivery contract.

## Voxblox compatibility adapter

`python/examples/tools/voxblox_mesh_to_horus_marker.py` accepts ROS 1 or ROS 2
Voxblox meshes and triangle markers. Encoded Voxblox coordinates use the same
`2 / 65535` block-relative scale as its RViz visualizer, preserving source RGB.
Triangle markers have their pose and scale baked into the output geometry.

The default `--max-triangles 0` does not truncate source triangles. An explicit
nonzero cap rejects an oversized update as a whole. Malformed blocks, colors,
coordinates or mixed-frame marker snapshots leave the previous accepted map
unchanged. Identical blocks do not trigger a new immediate snapshot; periodic
compatibility replay still occurs at `--republish-interval` (2 seconds by default).

This is the legacy complete-Marker transport, not the indexed incremental-page
protocol used by the nvblox adapter. Large live maps should be measured for
snapshot bandwidth and receive/upload cost; an uncapped source does not imply
unlimited headset capacity.

## Octomap

Use this when the runtime already provides an octomap-oriented representation and you want HORUS to mirror that layer directly:

```python
world_layers.add_3d_octomap(
    "/map_3d_octomap_mesh",
    frame_id="map",
    render_options={
        "render_mode": "surface_mesh",
        "native_topic": "/map_3d_octomap",
        "max_triangles": 120000,
    },
)
```

Reference script:

- `python/examples/octomap_registration.py`

The standalone `uav_sim_octomap_marker_relay` defaults to `--voxel-scale 1.0`
and `--color-mode auto`: source RGB is used for full ColorOcTree messages;
ordinary/binary occupancy trees fall back to axis colors because their payloads
do not carry RGB. `--color-mode axis` explicitly selects that visual style.
Exposed-face extraction respects mixed octree cell sizes, and coplanar merging
does not cross source-color boundaries. This legacy relay still sends complete
Marker snapshots, unlike the incremental nvblox path.

## Gaussian splat fixture

Use this when you want to validate the experimental 3D Gaussian Splat visualization path while keeping the normal HORUS ROS workflow. The fixture publisher emits:

- a Gaussian splat manifest
- ROS chunk topics for the binary PLY asset
- a PointCloud2 fallback preview
- TF and odometry for two fake robots so workspace registration remains realistic

```python
world_layers.add_gaussian_splat_map(
    manifest_topic="/horus/gaussian_splat/manifest",
    frame_id="map",
    preview_topic="/map_gaussian_splat_preview",
    render_options={
        "render_mode": "splats",
        "max_splats": 350000,
        "render_scale": 0.5,
        "pointcloud_fallback": True,
    },
)
```

Reference scripts:

- `python/examples/tools/publish_gaussian_splat_fixture.py`
- `python/examples/gaussian_splat_fixture_registration.py`

During XR debugging, change `render_mode` to `debug_points`, `mono_center_eye`, or `no_covariance` in the registration script. Return to `splats` after the layer stays fixed in the accepted workspace while moving the headset.

For an explicit source-detail profile, set `preserve_source_detail=True` in
`render_options`. The matching MR runtime uses full-resolution splat targets,
float32 harmonic storage, all available harmonic orders without SH LOD,
four-pass sorting every frame, and no contribution-threshold culling. `max_splats`
then rejects an oversized asset rather than selecting a prefix. Reannounce the
manifest after changing load settings so cached data is reloaded with the new
precision/capacity. Existing registrations retain their defaults. This profile
costs more GPU time/memory; it is not a claim of mathematically lossless rendering,
unlimited capacity, or verified Quest XR behavior. A configured point-cloud
fallback is still a distinct representation and must not be measured as splats.

## Why these are separate examples

This is one of the places where the curated examples are intentionally not over-generalized.

The point is not to build one script with ten flags. The point is to give you one clean registration per map family so you can copy the one that matches your runtime.

## Validation path

All of these builds on the same robot-description fake runtime:

```bash
python3 python/examples/tools/fetch_robot_description_assets.py
```

Then choose one:

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --publish-occupancy-grid
python3 python/examples/occupancy_map_registration.py
```

```bash
python3 python/examples/legacy/fake_tf_robot_description_suite.py --robot-profile real_models --map-3d-mode pointcloud --map-3d-profile realistic
python3 python/examples/pointcloud_map_registration.py
```

Gaussian splat fixture:

```bash
python3 python/examples/tools/publish_gaussian_splat_fixture.py --profile small
PYTHONPATH=python:$PYTHONPATH python3 python/examples/gaussian_splat_fixture_registration.py
```

## What comes next

The final tutorial moves from simulated infrastructure to real ROS graphs. Continue with [Tutorial 7: Live robot checklist](live-robot-checklist.md).
