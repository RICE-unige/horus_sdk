---
title: Live Mesh Delivery
---

# Live Mesh Delivery

The SDK's live-map cache compares complete section content, sends only changed
sections, and retains current revisions for missing-page repair and reconnects.
No triangles are decimated. The existing HMSH representation uses float32
coordinates and RGBA8 colors; it is not a bit-exact archive of ROS float64 values.

## Standard ROS maps

For `visualization_msgs/msg/Marker` maps, start the adapter before mapping begins:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 -m horus.mapping.ros_mesh --input-topic /your_mapper/mesh \
  --output-topic /horus/maps/live --frame-id map
```

For `MarkerArray`, add `--input-type MarkerArray`. Use the same ROS domain and
middleware environment as your source. The adapter does not start a simulation,
change the source publisher, or generate substitute geometry.

In another terminal, with the same ROS environment:

```bash
cd ~/horus_sdk
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/python:$PYTHONPATH"
python3 python/examples/live_mesh_map_registration.py \
  --topic /horus/maps/live --frame-id map
```

Registration starts the HORUS bridge through the normal SDK workflow. In an
existing robot/team registration, use `world_layers.add_live_mesh(topic, frame_id)`.
Register the **adapter output**, not the raw source as a second mesh layer.
This opt-in requires matching updated MR and bridge builds. `add_3d_mesh()` stays
on the legacy path. `add_nvblox_mesh()` uses the explicit live subscription policy
with the nvblox-specific decoder.

## Data contract

- One map has one fixed frame, one authoritative adapter, and its own output topic.
  Use separate topics for independent maps; merge sources upstream only with
  explicit frame alignment and unique section identities.
- Standard input supports `TRIANGLE_LIST`, keyed by `(namespace, id)`. Marker pose,
  scale and colors are applied. Foreign frames require TF at the marker stamp;
  unavailable TF is retried in source order for up to five seconds, then reported
  as a rejected update rather than misplaced. The TF-wait queue is bounded to
  64 messages / three million points; a source replay is required after overflow
  or timeout. MarkerArrays are limited to 1024 markers / three million points.
- Omitted IDs are unchanged. `DELETE` removes one marker. `DELETEALL` alone resets
  the generation. A complete `MarkerArray` beginning with `DELETEALL` is compared
  as a snapshot, including removal of absent IDs.
- Identical geometry with a refreshed timestamp is suppressed only after its
  fixed-frame placement is resolved. Changed color, scale or placement is sent.
- Finite lifetimes refresh without geometry replay and expire with an explicit
  delete. Use `--ros-args -p use_sim_time:=true` for simulated lifetimes. Transport
  pacing uses a steady clock even when simulation time is paused.
- Frame-locked animated markers and non-triangle types are rejected, not silently
  converted to static maps. Keep them on their appropriate dynamic visualizer.
- Large markers are split into pages, not downsampled. Defaults: 4096 triangles
  per page, 1024 page slots, 256 MiB encoded-state cache, 4 MiB/s geometry output.
  Capacity failures are counted and reported; the last valid map is preserved,
  not silently truncated. Cache limits exclude Python decoding overhead.
- Pending replacements coalesce per page. A slow link may skip intermediate
  revisions but retains every section's latest state. Small revision inventories
  and explicit repair replace full-map timer broadcasts.
- Repair geometry currently broadcasts on the map topic. Up-to-date Quest clients
  ignore applied revisions; per-client repair bandwidth is not yet implemented.
  Simultaneous requests are coalesced for one second. A just-sent revision is
  allowed two seconds to arrive before being retransmitted; receivers retry
  missing pages via the inventory rather than relying on a single request.

Input reliability/durability are discovered when the source first appears.
Changing source QoS later requires restarting the adapter. Reliable input is
recommended. Missing upstream deltas cannot be reconstructed: start before mapping
or obtain a full source snapshot. Source resets must send a clear or authoritative
snapshot; publisher disappearance alone is not proof of a map reset.

## Monitor and extend

`<output-topic>/status` is a `std_msgs/msg/String` JSON report with received and
rejected updates, unchanged pages, suppressed encoded bytes, cache and queue bytes,
sent pages/bytes and repair pages. Suppressed bytes count avoided HMSH payloads,
not measured wire traffic or frame-rate improvement. A synchronized stationary
map sends no geometry until changed or explicitly requested.

Other producers can use `horus.mapping.LiveMeshState`, with immutable
`BlockGeometry` sections, or implement the same HMSH v2 contract. Output topics:

| Suffix | Type | Delivery |
| --- | --- | --- |
| `/indexed` | `std_msgs/msg/UInt8MultiArray` | Reliable, volatile page updates |
| `/indexed_manifest` | `std_msgs/msg/String` | Reliable, transient-local depth-one inventory |
| `/indexed_request` | `std_msgs/msg/String` | Snapshot or selected-page repair requests |

Identity is map generation plus page slot and monotonically increasing revision.
Deletion records are retained until slot reuse or generation reset. New observers
request current state; they do not rely on ROS subscriber counts or historical
transport queues. Exact duplicate comparison excludes revision metadata only.

Put the adapter near a remote producer to save upstream bandwidth too. On a local
PC it saves downstream map traffic only. This does not reduce the GPU cost of
resident geometry and is not a generic filter for TF, camera frames, commands or
raw sensor PointCloud2 streams.
