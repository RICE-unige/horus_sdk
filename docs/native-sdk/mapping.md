---
title: Native Mapping
sidebar_position: 3
---

# Native Mapping

C++ and Rust provide costmap reconstruction, HMSH v2 encoding and retained live
mesh state without requiring ROS or the Python SDK. They preserve the existing
float32/RGBA8 representation and do not downsample geometry.

ROS-enabled builds also provide `LiveMeshPublisher`: paced page publication,
retained inventories, bounded repair requests and late-join recovery. The
publisher does not start a bridge or register a visualization automatically.

## Publishing transforms

Both native SDKs provide a reusable TF publisher: `horus::tf::Publisher` in C++
and `horus::tf::Publisher` in Rust. It sends typed batches to `/tf`, or retained
static transforms to `/tf_static`. Topics and `use_sim_time` are configurable.
Frames use ROS coordinates and xyzw quaternions. Supply a nonnegative nanosecond
stamp to preserve source time, including zero; leaving it unset uses the ROS
clock. Process uptime is never substituted for a message stamp.

All transforms in a batch are validated before publication. Up to 4,096
transforms per batch and 4,096 retained static children are supported. Retained
static frames are republished together so a late subscriber receives earlier
fixed frames too. The caller must supply a coherent tree and serialize calls;
this publisher does not reconcile conflicting TF sources. `subscriber_count`
reports matched DDS subscriptions, not proof of reception or rendering.

`fake_tf_publisher` is a transform-only example in both languages. It publishes
a moving robot base plus fixed camera and lidar offsets, requires ROS, and does
not generate images, scans, registrations or motion commands.

## Map-only and remote views

Use `make_workspace_visualization` (C++) or `Robot::workspace_visualization`
(Rust) when the registration represents a shared map, not a physical robot.
Its `create_dataviz` does not add a robot TF stream. The wire contract disables
robot controls, sensors and the Robot Manager while retaining global maps.

```cpp
#include <horus/robot/robot.hpp>

auto workspace = horus::robot::make_workspace_visualization("map_view");
auto view = workspace.create_dataviz();
view->add_remote_rendered_map("/remote/map", "map");
```

```rust
use horus::robot::Robot;

fn main() -> Result<(), String> {
    let workspace = Robot::workspace_visualization("map_view");
    let mut view = workspace.create_dataviz(None);
    view.add_remote_rendered_map("/remote/map", "map", None)?;
    Ok(())
}
```

Register the entity and its view with the normal registry API. Remote-view
configuration defaults to v3 WebRTC and also supports explicit v2 or ROS
diagnostic transport. It matches Python's media topics, frame identity,
viewport, encoder, frame-age and caller overrides. It does **not** start a
rendering process. Both native SDKs provide a separate point/mesh rendering
agent and a paired registration example. See [Remote Rendering](./remote-rendering.md)
for the CUDA backend, shared configuration and supported output modes.

## Retained mesh state

Every key denotes one complete mesh section in a fixed map frame. Replacing a
key compares its encoded content; unchanged geometry and colors emit no update.
A deletion emits a retained tombstone. Reusing a slot increments its revision,
and clearing the map increments its generation.

```cpp
#include <horus/mapping/live_mesh.hpp>

horus::mapping::LiveMeshState map;
horus::mapping::BlockGeometry section;
section.vertices = {{0,0,0}, {1,0,0}, {0,1,0}};
section.triangles = {0,1,2};
auto updates = map.replace("section-a", &section);
auto inventory = map.manifest_json();
auto repair = map.repair_updates(R"({"request":"snapshot"})");
```

```rust
use horus::mapping::{BlockGeometry, LiveMeshOptions, LiveMeshState};

fn main() -> Result<(), String> {
let mut map = LiveMeshState::new(LiveMeshOptions::default())?;
let section = BlockGeometry {
    vertices: vec![[0.,0.,0.], [1.,0.,0.], [0.,1.,0.]],
    triangles: vec![0,1,2],
    ..Default::default()
};
let updates = map.replace("section-a", Some(&section))?;
let inventory = map.manifest();
let repair = map.repair_updates(&serde_json::json!({"request":"snapshot"}));
Ok(())
}
```

`replace_many` commits a batch only after every change validates. Snapshot and
repair results share immutable pages (`shared_ptr` / `Arc`), not copies of all
mesh bytes. C++ callers must serialize state mutation; Rust requires a mutable
borrow. Returned pages can outlive later map edits.

Default limits are 1,024 slots, 1 MiB per page and 256 MiB retained encoded data.
Exceeding a limit returns an error and retains the previous valid map. Split
large sections into pages; limits do not authorize silent geometry removal.
The cache budget does not include caller-held old snapshots, source geometry,
temporary encoding or outgoing ROS queues. Release stale handles and bound
your outgoing queues independently.

## Costmaps

`CostmapState` accepts full occupancy data or raw Nav2 costs and applies bounded
rectangular patches. It checks geometry, frame, payload length and timestamp
ordering before mutation. Patch operations touch affected rows only. The output
is a borrowed `span` / slice; do not retain it across mutation.

## Native ROS publication

Configure the SDK visualization with `DataViz.add_live_mesh` or
`add_nvblox_mesh`, then register it using your normal registry client. These
helpers preserve Python's indexed-live flags, upload budgets and caller
overrides. The topic names a converted HMSH producer, **not** raw nvblox data.

```cpp
#include <horus/mapping/live_mesh_publisher.hpp>

horus::mapping::LiveMeshPublisher publisher;
// section is a complete page in the configured map frame.
publisher.replace("section-a", &section);
// Call regularly from your application loop (for example every 10 ms).
publisher.pump();
```

```rust
use horus::mapping::{LiveMeshPublisher, LiveMeshPublisherOptions};

// Inside a Result-returning function, using the section defined above:
let mut publisher = LiveMeshPublisher::new(LiveMeshPublisherOptions::default())?;
publisher.replace("section-a", Some(&section))?;
publisher.pump()?;
```

The default topic family is `/horus/maps/live/indexed`, `indexed_manifest`
and `indexed_request`. Publication is limited to 60 pages/s, 4 MiB/s and
four pages per `pump` call. These are configurable **transport budgets**, not
triangle reduction. The queue replaces superseded unsent revisions without
changing FIFO position, and retains complete pending pages while no subscriber
exists. Pages are shared across the cache and scheduler; ROS serialization
still requires a middleware copy.

Manifests publish every two seconds. Repair processing handles at most one
request per pump; its incoming queue holds up to 16 requests of 256 KiB each.
`dropped_repair_requests()` reports overflow, so failed recovery is not silent.
Recent matching revisions are coalesced for two seconds, then may be resent.
The publisher owns a private ROS context; destroying it does not shut down the
application's other ROS nodes. Calls must be serialized by the application.

`MeshPageScheduler` is also available separately for custom transports. Its
clock inputs are monotonic seconds; nonfinite values are rejected. There is no
hard real-time guarantee for DDS publication.

## Source-adapter boundary

The native `CostmapAdapter` subscribes to full OccupancyGrid or raw Nav2
costmaps and their incremental updates, publishing the normalized full-grid
format expected by HORUS MR. It supports retained-map late joining, optional
snapshot-service recovery, bounded patch journals and periodic resynchronization.
The adapter owns its ROS context; destroying it does not stop the caller's node.
Both language wrappers expose `status()` with readiness and error details.
Missing Nav2/map message dependencies produce an explicit startup error.

```cpp
horus::mapping::CostmapAdapterOptions options;
options.input_topic = "/local_costmap/costmap";
options.updates_topic = "/local_costmap/costmap_updates";
options.output_topic = "/horus/local_costmap";
horus::mapping::CostmapAdapter adapter(options);
```

```rust
let mut options = horus::mapping::CostmapAdapterOptions::new(
    "/local_costmap/costmap", "/horus/local_costmap");
options.updates_topic = "/local_costmap/costmap_updates".into();
let adapter = horus::mapping::CostmapAdapter::new(options)?;
```

Register the output topic as a costmap layer. For raw Nav2 input, select
`input_format = "raw"` and configure the raw update topic and GetCostmap
snapshot service. A stream without sequence numbers cannot prove that no
incremental update was lost; periodic authoritative snapshots limit this risk.

Native `MeshSourceAdapter` accepts Marker, MarkerArray or nvblox Mesh sources,
converts geometry into a fixed map frame using TF, and publishes paged HMSH
updates. It discovers publisher reliability/durability, defers messages waiting
for TF, retains unchanged sections and handles deletion/clear without replacing
the whole map. It does not register a robot or launch a bridge.

```cpp
#include <horus/mapping/mesh_source_adapter.hpp>

horus::mapping::MeshSourceAdapterOptions source;
source.input_topic = "/nvblox_node/mesh";
source.input_type = "nvblox";
source.output_topic = "/horus/maps/live";
source.frame_id = "map";
horus::mapping::MeshSourceAdapter adapter(source);
```

```rust
use horus::mapping::{MeshSourceAdapter, MeshSourceAdapterOptions};

let mut source = MeshSourceAdapterOptions::new("/nvblox_node/mesh", "/horus/maps/live");
source.input_type = "nvblox".into();
source.frame_id = "map".into();
let adapter = MeshSourceAdapter::new(source)?;
```

Register `/horus/maps/live` with `add_live_mesh` or `add_nvblox_mesh` while
keeping the adapter alive. The nvblox message package must be present at native
build time; absent support returns an explicit error. Marker geometry must be
TRIANGLE_LIST, not frame-locked. Transforms are looked up at the source stamp;
zero requests the latest transform. Simulation clocks require `use_sim_time`.

The TF wait queue is bounded to 64 messages / 128 MiB / five seconds. Overflow
or expiry is reported and requires an authoritative source snapshot; a missing
delta is never silently treated as a complete map. Best-effort sources carry
the same limitation. Defaults allow three million input points per message,
4,096 triangles per Marker page and two nvblox blocks per page axis. These are
validation/paging limits, not geometry decimation. Source geometry and encoded
pages have separate cache accounting; middleware, staging and outgoing buffers
are additional memory costs.

`status()` distinguishes a subscription from discovered source publishers and
includes receipt age, rejected messages, TF backlog, source/encoded bytes,
pending pages and unchanged suppression. Receipt age does not claim rendering.

Both languages expose the non-ROS `MeshSourceState` conversion kernel, accepting
a caller-provided transform lookup. Rust borrows binary geometry across its C
ABI; returned encoded pages are copied once into Rust-owned immutable `Arc`
storage. `LiveMeshPublisher` also accepts already-paged
geometry in one fixed map frame for custom source integrations. The
[live mesh delivery contract](../python-sdk/live-mesh.md) remains common to all
languages. A successful ROS publish does not prove the headset displayed it.
