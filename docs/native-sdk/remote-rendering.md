---
title: Native Remote Rendering
sidebar_position: 5
---

# Native Remote Rendering

The C++ and Rust SDKs expose scene import, v3 remote-view exchange, stereo frame
assembly, direct CUDA point/textured-mesh rendering, and optional native H.264
encoding. The HORUS ROS bridge owns WebRTC signaling and network delivery.
Registration alone does not start rendering: a renderer must consume view states
and publish rendered frames.

## View and frame contracts

C++ headers are under `horus/remote_rendering/`; Rust APIs are under
`horus::remote_rendering`. Both provide:

- Per-eye view/FOV and workspace state, including viewer-incarnation IDs.
- Positive-forward stereo view/projection matrices, preserving asymmetric FOV.
- Validated rendered-frame metadata and bounded `HRV3` diagnostic framing.
- Reconnect-aware pose sequence gates and newest-only queues with replacement counts.
- Bounded NTP-style clock alignment using low-round-trip samples.
- HRRF v1/v2 lossless depth frames, stereo/mono atlases and HRRD diagnostic carriers.
- Depth quantization, minimum/center/edge-aware reduction, RGB frame markers,
  static luma depth and dynamic v5 pose/depth atlas encoding and decoding.

```cpp
#include <horus/remote_rendering/protocol.hpp>

auto view = horus::remote_rendering::ViewState::from_json(view_json);
auto left_vp = horus::remote_rendering::eye_view_projection_matrix(
    view.left_eye, 0.05, 100.0);
```

```rust
use horus::remote_rendering::ViewState;

let view = ViewState::from_json(&view_json)?;
let matrices = view.eye_matrices(0.05, 100.0)?;
```

Diagnostic frames allow 64 KiB metadata and 32 MiB encoded image data. Decoders
borrow the image slice rather than copying it; keep the packet alive. Image
bytes remain opaque: successful packet decoding does not validate the PNG.
The native protocol accepts non-negative integer timestamps through `i64::MAX`
and rejects nonfinite numeric values. Queues replace obsolete waiting work and
drain a pending value after close; they never queue an unbounded video backlog.

### Existing RGB-D streams

Use `horus/remote_rendering/frame.hpp` in C++ or
`horus::remote_rendering::frame` in Rust. `pack_depth_frame` takes explicit
capture time and one or two top-left depth images. Stereo rows are flipped and
laid out horizontally on the wire, matching the existing Quest raw-texture
contract. The decoder returns top-left per-eye depth codes. Shared mono-atlas
packing requires both quantized depth images to match; it does not silently
discard a different right-eye view.

Zero depth codes indicate invalid pixels. Gzip output is deterministic, CRC is
checked before decoding, and both compressed and expanded wire depth are capped
at 4 MiB. A shared mono atlas can expand to two returned eye arrays. C++ diagnostic
views and Rust `unpack_ros_debug_frame` borrow their packet slices. RGB image data
is never routed through JSON. The older luma helpers preserve their original
8-bit paired-pixel / 16-bit four-plane encodings and their sampling behavior;
they are compatibility APIs, not the recommended format for new integrations.

```rust
use horus::remote_rendering::frame::{DepthFrameInfo, pack_depth_frame};

let info = DepthFrameInfo {
    width: 2, height: 2, color_width: 640, color_height: 480,
    view_count: 1, capture_time_ns: 1_780_000_000_000_000_000,
    ..Default::default()
};
let packet = pack_depth_frame(&[1.0, 2.0, f32::INFINITY, 3.0], &info)?;
```

## CUDA backends

No CUDA installation or compilation occurs when an SDK is imported or a
registry is created. Build the existing backend sources explicitly with your
CUDA toolkit. From the SDK checkout, for example:

```bash
mkdir -p build/cuda
nvcc -O3 --shared -Xcompiler=-fPIC -arch=sm_89 \
  python/horus/remote_rendering/cuda_point_renderer.cu \
  -o build/cuda/libhorus_cuda_points.so
nvcc -O3 --shared -Xcompiler=-fPIC -arch=sm_89 \
  python/horus/remote_rendering/cuda_mesh_renderer.cu \
  -o build/cuda/libhorus_cuda_mesh.so
```

`sm_89` is the architecture used for the local RTX 4090 qualification. Select
the architecture(s) appropriate to deployment; these commands do not produce
a universal GPU binary. The `.cu` files contain native CUDA, not Python code.
Supply the absolute library path to `CudaRenderer` (C++) or
`CudaRenderer::points` / `CudaRenderer::mesh` (Rust). Keep the library installed
alongside your deployment; the Rust crate does not bundle prebuilt GPU binaries.

Point uploads accept borrowed position/RGB arrays and an explicit offset.
The renderer refuses to run until every point is uploaded, including when
uploads overlap or arrive out of order. Mesh inputs include positions, triangle
indices, per-face tints and optional UVs/texture atlas rectangles. Defaults use
a white one-pixel texture. No mesh decimation is performed.

`render_views` returns owned RGB/depth buffers. `render_into` reuses caller-owned
buffers to avoid repeated image allocation. Input validation checks shape,
index/texture bounds, finite geometry, projection and depth range before native
calls. The point backend accepts up to 12 views; the mesh backend accepts two.
Host frame storage is limited to 256 MiB per call. GPU allocation failures
surface as errors; these limits are not measured frame-rate guarantees.

## Scene import and CPU reference

`load_scene(path, options)` in C++ and `SceneAsset::load(path, options)` in Rust
load full indexed geometry, node transforms, material colours, UVs and textures
through Assimp. OBJ/glTF/GLB meshes and coloured point-only PLY are supported;
PNG/JPEG, WebP/TIFF and supported 8-bit BMP/TGA/GIF/PNM/PSD textures can be external or embedded. No asset is downloaded and no
geometry is silently decimated. Explicit vertex/triangle/texture budgets reject
oversized output. Assimp's temporary import storage is additional to those budgets.

Node instances remain distinct geometry. Mirrored node transforms correct triangle
winding. Identical textures share atlas space with padded edges. Source scale is
explicit; no dataset-specific coordinate correction is guessed. The default
`bake_lighting` matches the existing simple directional/ambient shading approach;
disable it for unchanged material tints. As with the CUDA mesh backend, this is
RGB diffuse texture rendering, not a PBR engine: transparency, normal/roughness
maps and animated/skinned scenes are not reproduced. Non-triangle line geometry
and missing or unsupported textures report errors.

Rust's immutable scene owns native storage and lends slices to the renderer;
there is no second complete Rust-side copy of the geometry. C++ `mesh_input()`
similarly borrows its scene. Keep the scene alive while using those views.
`render_colored_points` provides a full-point CPU reference; stereo assembly adds
visibility/depth strips, markers and validated frame metadata. It is a reference
path, not a promised replacement for GPU rendering at production frame rates.

## ROS frame publisher

`RemotePublisher` uses a private ROS context and an executor thread. Each renderer
calls `next_view(timeout_ms)` and receives the newest waiting valid view, then
publishes its completed frame. The three explicit modes are:

| Mode | Published source | Delivery |
| --- | --- | --- |
| `diagnostic` | HRV3 PNG in `sensor_msgs/CompressedImage` | Direct ROS diagnostic path |
| `rgb` | Stereo RGB-D/marker atlas in `sensor_msgs/Image` | Bridge encodes and sends WebRTC |
| `h264` | Annex-B H.264 in `sensor_msgs/CompressedImage` | Bridge forwards encoded WebRTC media |

Configure the publisher topics to match `add_remote_rendered_map`. Use frame
metadata from the accepted view; set render/capture times from the process's
monotonic clock. The publisher rejects obsolete sessions, viewer incarnations,
workspace revisions, future source sequences, expired views and duplicate frame
IDs. `stop()` wakes waiting consumers and does not shut down unrelated ROS nodes.
Source-view freshness is measured at that pose's receipt, not from the arrival of
an unrelated newer pose. A bounded receipt history retains at most 8,192 accepted
views and expires entries at the configured deadline. After a dropped H.264
reference or workspace/viewer change, dependent frames wait for an IDR; retrying
an already published frame does not break the reference chain.
For H.264, forward `take_keyframe_request()` to the encoder. Status reports local
source activity, not confirmation of received or displayed headset frames.

## Optional native NVENC

Build C++ with `-DHORUS_ENABLE_NVENC=ON` or Rust with `--features "ros2 nvenc"`.
These options require FFmpeg development libraries (`libavcodec >= 60`,
`libavutil >= 58`) and NVIDIA encoding hardware/drivers at runtime. They are off
by default; importing the SDK never installs or downloads them.

`H264Encoder` accepts a complete RGB video atlas, uses reusable host surfaces,
correlates encoded packets with frame metadata and supports explicit IDR requests.
The low-latency configuration disables B-frames/lookahead and uses a single-frame
rate-control budget. `close()` flushes remaining packets and is idempotent; errors
are returned rather than silently switching to CPU encoding. Viewer/session or
workspace revision changes automatically start a new keyframe chain.

`AsyncH264Encoder` owns a worker and one replaceable waiting RGB input. It applies
backpressure when encoded output is not consumed; it never discards already
encoded reference frames. Call `poll_encoded` regularly and forward every returned
packet in order. `replaced_count` exposes skipped waiting inputs; `close` finishes
the newest pending input, joins the worker and returns remaining packets. Rust
copies borrowed input into owned native storage before returning; C++ can move
its vector. Neither path borrows a frame after its owner has released it.

Both encoders currently copy RGB into a host encoder surface before GPU encoding.
This is **not** a device-resident or zero-copy media path.

## Native launchers

Both languages provide `remote_map_registration` and `native_remote_render_agent`.
They share the configuration template at `cpp/examples/remote_render_config.json`.
The registration creates a workspace visualization, not a fake robot or a control
interface. Start registration first, connect HORUS and create a workspace; then
run the renderer in another terminal with the same ROS environment and config:

```bash
source /opt/ros/jazzy/setup.bash
# Terminal 1, from the SDK checkout; use your own scene in terminal 2.
./build/cpp-ros/examples/remote_map_registration cpp/examples/remote_render_config.json
```

```bash
source /opt/ros/jazzy/setup.bash
# Terminal 2: choose the point or mesh CUDA library to match the scene.
./build/cpp-ros/examples/native_remote_render_agent \
  /absolute/path/to/scene.ply "$PWD/build/cuda/libhorus_cuda_points.so" \
  cpp/examples/remote_render_config.json
```

Rust provides the same executable arguments:

```bash
cargo build --manifest-path rust/Cargo.toml --features "ros2 nvenc" --examples
# Run in separate terminals, both with ROS sourced.
./rust/target/debug/examples/remote_map_registration cpp/examples/remote_render_config.json
./rust/target/debug/examples/native_remote_render_agent \
  /absolute/path/to/scene.obj "$PWD/build/cuda/libhorus_cuda_mesh.so" \
  cpp/examples/remote_render_config.json
```

The template defaults to lossless diagnostic PNG over ROS. To use WebRTC, select
`rgb` (bridge encoding) or `h264` (native NVENC), and set `stream_topic` to
`/horus/remote_render/image` in the shared config before starting either process.
WebRTC also requires a media-enabled HORUS bridge. Diagnostic mode does not need
NVENC. `world_scale` scales source geometry; `workspace_scale` sets its presentation
scale in MR. These are different quantities. No dataset alignment is inferred.

The renderer uploads full scene geometry once, frees its CPU scene copy, reuses
RGB/depth output buffers, limits rendering to the configured rate and consumes
only new valid views. `duration_seconds: 0` runs until interrupted. SIGINT/SIGTERM
flush and stop the renderer; Ctrl+C stops registration. The source status reports
accepted/published/replaced/discarded counts and errors, not headset delivery.
Qualification covers actual CUDA rendering, diagnostic/RGB/H264 ROS output,
host-monotonic timing, decoder IDs, viewer/revision restart and graceful shutdown.

## Native Launcher Options

Native launchers accept `scene_kind: "gaussian"` with the Gaussian backend library
in place of the point/mesh library. For GPU-resident H.264 set `mode: "h264"` and
`video_library` to the absolute CUDA video plugin path. Omitting it retains the
host-input encoder. `canonicalize_y_down` explicitly enables the Gaussian dataset's
floor/axis conversion; it is off by default. Scene coordinates are never inferred
from the filename.

## Gaussian and Device Video

Original 3DGS PLY files (ASCII and binary little/big endian) retain positions,
quaternions, scale, opacity and spherical harmonics through degree four. Non-finite
rows are counted and rejected from the scene; no finite splats are sampled away.
The optional GPU plugin uses gsplat's covariance rasterization and spherical
harmonics, not a point approximation. Both APIs expose reusable host output and
an integrated device-output path through `GpuVideoEncoder`.

Build optional plugins explicitly. Substitute your dependency directories and GPU
architecture; nothing is downloaded during these builds:

```bash
cmake -S rust/native/gaussian -B build/gaussian \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89 \
  -DCMAKE_PREFIX_PATH=/absolute/path/to/libtorch \
  -DGSPLAT_SOURCE_DIR=/absolute/path/to/gsplat
cmake --build build/gaussian --parallel 2
cmake -S rust/native/cuda_video -B build/cuda-video \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89
cmake --build build/cuda-video --parallel 2
```

The Gaussian plugin was tested with gsplat 1.5.3, LibTorch 2.11.0 CUDA 12.8 and
CUDA 12.8. Use mutually compatible CUDA/LibTorch builds; the standalone LibTorch
distribution needs no Python SDK runtime. The video plugin requires CUDA and
FFmpeg libavcodec 60+/libavutil 58+. Point and mesh CUDA libraries must include
the current device-output entry points. Rebuild older libraries rather than
silently falling back to host copies.

`GpuVideoEncoder` renders, assembles stereo/depth strips and frame markers, converts
RGB to limited-range BT.709 NV12 and submits CUDA surfaces to NVENC. Only encoded
packets return to host memory. Device-to-device copies and synchronization remain;
this is not a claim of zero-copy or unlimited throughput. It retains the active
CUDA context, uses bounded reusable surfaces, and correlates packets with source
poses. Reconnects and explicit recovery requests force IDR frames.

H.264 is lossy, including the encoded depth strip. Qualification includes decoded
depth and frame IDs, not only colour appearance. The high-frequency test fixture
needed 64 Mbit/s to meet its depth-error bound; 16 Mbit/s failed on forced IDRs.
That is test evidence, not a recommended universal bitrate. Measure depth fidelity,
GPU load and network capacity for your scene/resolution; diagnostic PNG remains
the lossless reference. No cloud or Quest throughput figure is inferred here.

## Real Datasets and Atlases

`scene_catalog` retains source and attribution metadata from the Python catalogue.
`resolve_scene_asset` resolves locally cached assets only. Fetching datasets remains
an explicit preparation step with the existing download tools, not a side effect
of importing either SDK.

`prepare_eth3d_scene` reads scan alignment matrices and computes shared bounds,
floor placement, scale and a fingerprint. `PointChunkStream` reads binary PLY or
HRL in bounded chunks, preserves every finite point and handles padded scalar
properties and either byte order. `prepare_cow_lady_scene` applies the same
quantile-based centering and coordinate conversion as Python. Its full-scene
result needs memory proportional to the cloud; use streaming for large scans.

`render_multiview_atlas` (C++) / `MultiviewAtlas` (Rust) renders full point sets or
streams into bounded views. `unproject_multiview_surfels` / `unproject` reconstructs
the same float32 texture-coordinate grid used by the Quest representation.
`pack_atlas_rgbd` / `pack_rgbd` creates the legacy static colour/depth carrier.
The Python Cow/Courtyard convenience builders can be composed from these APIs
and the catalogue's view presets. These are CPU reference/preparation helpers;
the native CUDA agent is the interactive rendering path.

`write_point_lod` / `PointChunkStream::write_lod` is **explicit opt-in**. It retains
the first original point/color per voxel and exports deterministic HRL1, without
overwriting existing output. Normal loading and rendering never invoke it.
The native paths do not depend on Python's NumPy caches or runtime JIT builders.

Tests include byte-exact Python-generated HRRF/luma/pose fixtures, stereo fixtures, malformed/truncated carriers,
clock/reconnect/queue lifecycle checks and direct Python/native GPU pixel/depth
comparison, scene transform/texture/PLY checks, actual GPU H.264 decode/marker/IDR
checks, and isolated DDS frame exchange/restart/stale-view tests in each language.
These do not establish network latency or headset visual acceptance.
