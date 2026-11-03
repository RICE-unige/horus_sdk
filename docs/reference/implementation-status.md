---
title: Implementation Status
sidebar_position: 1
---

# Implementation Status

## Python track

Status: **Primary operational SDK; release candidate under validation**

- curated registration examples for wheeled, drone, legged, stereo, robot-description, map, semantic, Carter, and Unitree workflows
- registration payload serialization with keep-alive and ACK handling
- camera transport profiles for ROS and WebRTC-backed flows
- robot-scoped and global DataViz support
- separate global/local planning paths and executed trajectory history, plus shared workspace navigation paths (requires the corresponding MR runtime update)
- global map support for occupancy grids, pointclouds, meshes, octomaps, and experimental Gaussian splat fixtures
- topic monitoring and dashboard state
- independent costmap layers; the SDK adapter normalizes full/raw Nav2 maps and incremental updates to the headset's full-grid format, with bounded buffering and snapshot recovery
- humanoid Teach configuration, controller-driven synthetic demo, configurable camera views, engagement/hold state, lease checks and native MCAP episode recording
- optional Teach task keyboard/catalogue, per-task episode counts, custom action handlers and durable-save hooks; currently Python-host/MR functionality, with native extension parity pending
- field teammate (HoloLens-class) registration with a capability-driven, default-deny safety contract and a fail-closed serializer (`entity_kind`, `capabilities`, `field_teammate_config`)
- HRI study analysis scaffolding ("On the Map, In the Team") with a dyad-level, confidence-interval reducer

## Experimental areas

Status: **Active validation**

- Gaussian Splat DataViz registration, fixture publishing, ROS chunk transfer, cache validation, and pointcloud fallback are available for testing.
- Quest/XR Gaussian Splat rendering is still being validated; use the small fixture and diagnostic render modes before testing dense assets.
- Teach's synthetic workflow and recording checks are automated. Physical robot retargeting, controller comfort and hardware stop behavior need robot-specific qualification. No built-in recorded-data viewer is provided; users choose their own dataset tools.
- Remote point, mesh and Gaussian rendering is currently a static-scene, single-viewer agent. Live reconstruction and independently rendered simultaneous operators are not yet implemented.
- Durable mission restoration, after-action replay, fleet resource admission and generated collision shapes are not completed release features.
- Native C++/Rust Teach configuration, sessions, Cartesian adapters and MCAP recording are implemented; physical controller qualification remains robot-specific.

## C++ track

Status: **Native SDK functional coverage implemented; release qualification in progress**

- C++ builds typed registration payloads for HORUS MR without the Python runtime
- ROS-enabled builds perform live registration, accepted/queued ACK handling, replay, keep-alive and unregister without invoking the Python SDK
- each registry owns its ROS context and executor; cancellation does not shut down the caller's ROS context
- optional bridge auto-start tracks only its own child process; startup failures return a bounded log tail
- costmap registration/reconstruction, ROS snapshot/update recovery, HMSH v2 mesh encoding, retained-map state, paced publication/repair, Marker/nvblox source conversion and ROS topic/status monitoring are implemented
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, Gaussian Splat fixtures, and semantic boxes
- field teammate capability contract at parity: `make_field_teammate`, `entity_kind`/`capabilities`/`field_teammate_config` in the payload, and `validate_field_teammate_safety` as the fail-closed backstop (verified against the shared `field_teammate_hololens.json` fixture)
- experiments subsystem: NDJSON/CSV metric writers, a monotonic-anchored clock, and the field-teammate HRI study reducer (dyad-level aggregation with t-based confidence intervals)
- colour manager: per-scheme palettes with cached assignment and an MD5-deterministic fallback that matches Python/Rust byte-for-byte
- native URDF/xacro/topic/parameter model compilation includes joint/collision poses, articulated visual bindings, materials, shared mesh assets and paced gzip chunk delivery
- native Teach configuration, lease-aware sessions, Cartesian adapters, bounded MCAP capture, episode validation and interrupted-capture recovery are available
- `cpp/examples/` mirrors the curated Python scenarios by basename, with `sdk_registration_demo.cpp` kept as the short ops-style payload demo

## Rust track

Status: **Native SDK functional coverage implemented; release qualification in progress**

- Rust builds typed registration payloads with `serde_json` while matching the Python payload contract
- the `ros2` feature uses a bundled C++ ROS runtime through a small C ABI; no Python SDK runtime wrapper
- live registration, accepted/queued ACK handling, replay, keep-alive, unregister and owned bridge startup use the same tested transport as C++
- costmap reconstruction and HMSH v2 encoding use owned Rust buffers with borrowed inputs/output; retained pages and paced queues share immutable storage. ROS costmap recovery, Marker/nvblox source conversion and topic/status monitoring use the shared native runtime
- camera transport profiles, teleop/task controls, ROS binding, workspace config, local body model metadata, and Robot Manager config are supported
- robot and global DataViz payloads cover transforms, paths, velocity, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, Gaussian Splat fixtures, and semantic boxes
- field teammate capability contract at parity: `Robot::field_teammate`, `entity_kind`/`capabilities`/`field_teammate_config` in the payload, and `validate_field_teammate_safety` as the fail-closed backstop (verified against the shared `field_teammate_hololens.json` fixture)
- experiments subsystem: NDJSON/CSV metric writers, a monotonic-anchored clock, and the field-teammate HRI study reducer (dyad-level aggregation with t-based confidence intervals); colour manager already present
- native robot-model compilation/delivery and Teach use the same tested C++ engine through a bounded C ABI, with owned Rust handles, fallible configuration and panic-contained adapter callbacks; no Python SDK subprocess is used
- `rust/examples/` mirrors the curated Python scenarios by basename, with `sdk_registration_demo.rs` kept as the short ops-style payload demo

## Native parity acceptance

Matching JSON is necessary but not sufficient. Native functional coverage means an
equivalent usable operation, not identical method names, runtime backends or
example orchestration. The matrix below identifies implementations and evidence;
it is not a guarantee for every robot, platform or optional dependency combination.

| API group | Python | C++ / Rust |
| --- | --- | --- |
| Registration, ACK, timeout, unregister, replay | Implemented | Implemented with ROS; isolated DDS peer tested |
| Owned bridge startup and cleanup | Implemented | Implemented; child ownership/failure tests |
| Camera, robot/global DataViz and field-teammate configuration | Implemented | Shared payload fixtures; not every hardware combination tested |
| Status/topic dashboard | Implemented | Implemented; paginated terminal, ROS graph links, heartbeat/presence expiry and transport evidence; no fabricated received FPS |
| Costmap configuration and reconstruction | Implemented | Implemented; cross-language fixture and patch tests |
| Incremental costmap ROS subscription/recovery adapter | Implemented | Implemented; occupancy/raw Nav2 snapshot, late-join and recovery DDS tests |
| HMSH v2 binary mesh encoding | Implemented | Implemented; byte-exact Python fixture |
| Live mesh retained state, unchanged-page suppression, repair inventory | Implemented | Implemented; atomic batches and immutable shared pages |
| Live/nvblox mesh configuration and upload budgets | Implemented | Shared Python/C++/Rust wire fixtures |
| Live mesh paced scheduler and ROS page publication/repair | Implemented | Implemented; raw-source adapters also perform paging and TF conversion |
| Map-only workspace entity and remote-view registration | Implemented | Implemented; shared v2/v3 payload tests, external renderer still required |
| Raw nvblox/Marker map conversion, TF lookup and source QoS discovery | Implemented | Implemented; Marker, MarkerArray and nvblox delayed-TF/geometry/color/repair DDS tests in both languages |
| URDF poses, visual/collision binding, materials and asset delivery | Implemented | Implemented; source/material/pose contracts and paced DDS delivery; expanded 8-bit texture decoding |
| Teach configuration, engagement and episode recording | Implemented | Implemented; shared configuration/hash fixtures, CDR capture and Python/native episode interoperability |
| Optional task management/counts, custom Teach actions and save hooks | Implemented with the matching MR update | Not yet implemented; existing native recording/control APIs remain available |
| Remote v3 view/frame contracts, stereo matrices, clocks and sequence gates | Implemented | Implemented; Python-generated fixture, malformed-frame and queue-lifecycle tests |
| Existing HRRF v1/v2 and dynamic v5 RGB-D frame helpers | Implemented | Implemented; exact Python-produced depth/marker/pose/atlas fixtures, bounded gzip decoding |
| CUDA point and textured-mesh rendering | Implemented | Direct native bindings; explicit backend library, reusable host and device outputs |
| CPU point rendering, stereo assembly and diagnostic PNG | Implemented | Implemented; full-point and pixel/metadata fixtures |
| Remote frame source/pose exchange | Implemented | Native newest-view ROS publisher, stale/duplicate rejection and shutdown; actual DDS tested in diagnostic/RGB/H.264 modes; bridge owns WebRTC |
| H.264 encoding | GPU-resident and host/async paths | Host and optional device-resident NVENC; actual decode, marker, reconnect-IDR and overload tests; lossy depth requires bitrate qualification |
| Generic point/textured-mesh scene loading | Implemented | Native Assimp import; transforms/materials, expanded texture formats and ASCII/binary PLY; borrowed Rust geometry |
| Point/mesh/Gaussian renderer launcher | Implemented | Native CUDA launchers and paired registration; diagnostic/RGB/H264 output, restart and shutdown tested locally |
| Remote Gaussian rendering | Implemented with documented restrictions | Original 3DGS PLY, SH0..4 and optional native gsplat backend; Python/native pixel/depth comparisons |
| Real-data catalogue, alignment and point streaming | Implemented | Shared catalogue, Cow/ETH3D preparation, bounded PLY/HRL streaming, explicit LOD; cross-language data checks |
| Multiview atlases and surfel reconstruction | Implemented | Bounded CPU atlas, float32 sampling and RGB-D packing; streamed or owned input |
| Robot helper methods, EventBus and TopicMap | Implemented | Native configuration/accessors and bounded event lifecycle tested; TopicMap is an in-memory catalogue, not a ROS subscription API |
| TF publication | ROS examples publish TF | Native batched publisher, retained static TF, explicit or ROS-clock stamps; `fake_tf_publisher` publishes real ROS messages rather than console-only updates |
| Collision-risk scan/cloud analysis and ROS publisher | Implemented | Native advisory analyzer, private context, bounded input, endian/stride checks; not a robot safety controller |
| Metrics writers, manifests, workloads, field-teammate reducer and colour allocation | Implemented | Implemented with focused tests; explicit source commit metadata and platform-specific host descriptions |

The native ROS implementation is qualified locally on Ubuntu with ROS 2 Jazzy. No-ROS builds support configuration, CPU kernels and explicit CUDA bindings, not live registration. Windows, macOS, other ROS distributions, physical robot behavior and headset performance require separate qualification. `show_dashboard` renders a paginated terminal view on a TTY and changed status summaries in redirected output. Graph discovery is not evidence that sensor payloads reached the headset; the dashboard labels unmeasured delivery explicitly.

The languages intentionally do not duplicate Python-specific tooling such as
NumPy caches, runtime CUDA JIT compilation, download scripts, experiment CLI
orchestration or synthetic multi-sensor scenario generators. Use the existing
preparation tools with native runtime APIs, or compose native scenarios with TF,
ROS, metrics and workload helpers. The native textured-mesh renderer uses the
CUDA backend; it does not expose Python's alternative nvdiffrast implementation.
Neither renderer is a general animated PBR engine. Deprecated empty Python
modules below are not features to reproduce. Do not infer operational native
support merely from a matching example filename.

Both languages also expose standalone Marker/nvblox conversion without ROS;
Rust borrows binary input arrays across its C ABI and owns returned encoded pages.
The collision analyzer publishes advisory risk in the sensor frame. It does not
gate robot motion or transform obstacles into a global planning frame. Native
sensor subscriptions accept best-effort and reliable publishers. Native workload
loaders preserve unknown fields under `extra`; loading does not mutate the input.

See [Robot Models and Teach](../native-sdk/models-and-teach.md) for dependencies,
source selection, model budgets, adapter responsibilities and recording limits.
Use Rust's `try_build_robot_config_dict` for user-provided model/Teach configuration;
it returns errors. The legacy infallible builder panics on invalid configuration.
Native registration uses the fallible path and reports the failure.

Rust's shared-registry accessor now returns an `Arc` rather than a mutex guard: a keep-alive call cannot hold the global lock and block shutdown. `shutdown_robot_registry()` stops the previous instance and allows a fresh one. Explicit native registry instances cannot be restarted after `stop()`; construct a new instance. Existing C++ shared-registry users should use an explicit new client after shutdown.

`Client.registry()` now exposes an independently owned registry in each native
language. Its `auto_launch` setting controls that registry's bridge launch.
`Client.shutdown()` stops only that instance, not another client's global
registry. Unlike the Python client, construction is lazy and does not launch
the legacy backend or start its monitoring UI.

Native kernel benchmarks are described in **Native Performance**. They measure CPU work only, not network or Quest performance.

## Compatibility imports

Use `horus.bridge.robot_registry`, `horus.core`, and `horus.dataviz` for new code. The old `horus.bridge.ros2`, `horus.topics`, and `horus.robot.dataviz` modules re-export the active implementations and emit deprecation warnings.

`horus.bridge.unity_tcp`, `horus.robot.status`, `horus.robot.teleop`, `horus.robot.task`, and `horus.plugins.rosbot` are deprecated empty compatibility placeholders, not additional operational APIs. Use the robot registration/configuration methods documented in this site instead.

| Source module | Status |
| --- | --- |
| `python/horus/bridge/ros2.py` | Deprecated re-export of the active registry |
| `python/horus/topics.py` | Deprecated re-export of core topic types |
| `python/horus/robot/dataviz.py` | Deprecated re-export of active DataViz configuration |
| `python/horus/bridge/unity_tcp.py` | Empty compatibility placeholder |
| `python/horus/robot/status.py` | Empty compatibility placeholder |
| `python/horus/robot/teleop.py` | Empty compatibility placeholder |
| `python/horus/robot/task.py` | Empty compatibility placeholder |
| `python/horus/plugins/rosbot.py` | Empty compatibility placeholder |

`python/horus/core/exceptions.py` defines the SDK exception hierarchy:
`HorusError`, `HorusConfigurationError`, `HorusConnectionError`,
`HorusRegistrationError`, and `HorusBridgeError`. It is not a stub.
