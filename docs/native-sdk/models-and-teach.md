---
title: Robot Models and Teach
sidebar_position: 4
---

# Robot Models and Teach

C++ and Rust expose the same native compiler, Teach session and MCAP recording
engine. These paths do not launch or embed the Python SDK. Live ROS integration
requires a ROS-enabled build; file compilation and episode APIs work without ROS.
The qualified host is Linux with ROS 2 Jazzy. Physical robot control still needs
an application-specific adapter and hardware qualification.

## Build Dependencies

Both languages require CMake and a C++20 compiler for the shared engine, plus
nlohmann-json, Eigen3, OpenSSL Crypto, zstd, Assimp, tinyxml2, zlib, libpng,
libjpeg, libwebp and libtiff development packages. MCAP 2.1.3 and meshoptimizer 1.3 sources are
vendored with their MIT licenses and provenance; they are not fetched at build
time. ROS builds additionally use rclcpp, rcl_interfaces, std_msgs, rosbag2_cpp
and rosidl_typesupport_introspection_cpp. Costmap adaptation also requires
nav_msgs, map_msgs and nav2_msgs.

On Ubuntu 24.04:

~~~bash
sudo apt-get install cmake g++ nlohmann-json3-dev libeigen3-dev libssl-dev \
  libzstd-dev libassimp-dev libtinyxml2-dev zlib1g-dev libpng-dev libjpeg-dev \
  libwebp-dev libtiff-dev
~~~

Resolved URDFs need no external compiler. Xacro inputs require ROS's xacro
executable in the sourced environment. Its subprocess is bounded by a timeout
and output-size limit; invalid descriptions return an error.

## Robot Models

### C++

~~~cpp
#include <horus/description/compiler.hpp>
#include <horus/robot/robot.hpp>

horus::robot::Robot robot("rover", horus::core::RobotType::WHEELED);
horus::robot::RobotDescriptionOptions description;
description.source = "local";
description.urdf_path = "/data/robots/rover.urdf";
description.mesh_root = "/data/robots/packages";
description.base_frame = "base_link";
description.body_mesh_mode = "runtime_high_mesh";
description.visual_link_pose_source = "tf";
robot.configure_robot_description(description);
~~~

### Rust

~~~rust
use horus::robot::{Robot, RobotDescriptionConfig};
use horus::core::types::RobotType;

let mut robot = Robot::new("rover", RobotType::Wheeled);
let mut description = RobotDescriptionConfig::new("/data/robots/rover.urdf", "base_link");
description.source = "local".into();
description.mesh_root = "/data/robots/packages".into();
description.body_mesh_mode = "runtime_high_mesh".into();
description.visual_link_pose_source = "tf".into();
robot.configure_robot_description(description);
~~~

Use normal robot registration afterward. Keep the registry alive so it can
answer model requests and repair transfers. The publisher removes large model
data from registration messages, retains the compressed artifact, and sends
paced chunks on the requested reply topics. Identical mesh assets share storage.

For a retained description topic, leave the URDF path empty, use source
"topic" and set robot_description_topic. For a ROS parameter, use source "ros"
and ros_param_node; the default parameter is robot_description. These sources
are not interchangeable fallbacks: a missing explicit parameter must not load
another robot's topic.

The compiler resolves package://, file:// and relative meshes, computes
joint and collision poses, groups fixed links, and preserves separate movable
link bindings. Assimp handles supported formats such as STL, OBJ and DAE.
URDF colors, embedded vertex/material colors and Gazebo fallback colors are
baked into vertex colors. PNG/JPEG, WebP/TIFF and supported 8-bit BMP/TGA/GIF/PNM/PSD textures supply an alpha-weighted mean color
in linear RGB, not a full UV texture. Unsupported or missing textures report an
error rather than silently becoming white.

The description module's compile_file and compile_urdf functions also return
artifacts without registering. Their options use the same configuration keys.
The direct compiler accepts preserve_source_mesh=true to disable geometric
simplification; lossless vertex deduplication and index/cache optimization still
run. Otherwise it applies an attribute-aware, error-bounded triangle budget.
The manifest reports source/output triangle counts, the requested budget,
whether that budget was met, and simplification error. A requested count is not
a guarantee when reaching it would exceed the shape-error bound.

This compiler does not publish robot TF. Supply the robot's actual TF tree;
visual_link_pose_source="tf" follows those frames.

## Teach Configuration and Runtime

Teach configuration alone never starts motion or recording. A runtime services
the headset requests and subscribes to configured data streams. Register the
same configuration on the humanoid robot.

C++ and Rust enforce the same explicit
[alignment protocol and measured spatial-view contract](../python-sdk/teaching-input.md#alignment-protocol)
as Python. `TeachPointCloud` / `with_spatial_view` binds a declared PointCloud2
stream; the default limit is one million points, with no silent downsampling.
The runtime retains CameraInfo and transient-local TF child frames for each
new episode, preserving original CDR and timestamps and marking snapshots.
Adapters receive calibrated FLU poses after the host applies alignment exactly
once. An Engage request without the current acknowledged alignment is rejected.

### C++

~~~cpp
#include <horus/teaching/runtime.hpp>
#include <horus/robot/robot.hpp>

horus::teaching::TeachConfig config("/data/teach", {
    {"streams", {{{"stream_id", "joints"}, {"topic", "/humanoid/joint_states"},
                  {"message_type", "sensor_msgs/msg/JointState"}}}}
});
horus::robot::Robot robot("humanoid", horus::core::RobotType::HUMANOID);
robot.configure_teaching(config);
horus::teaching::TeachRuntime runtime(config, "humanoid");
~~~

### Rust

~~~rust
use horus::teaching::{TeachConfig, TeachRuntime, TeachStream};
use horus::robot::Robot;
use horus::core::types::RobotType;

let config = TeachConfig::new("/data/teach")?
    .with_stream(TeachStream::new("joints", "/humanoid/joint_states",
                                 "sensor_msgs/msg/JointState"))?;
let mut robot = Robot::new("humanoid", RobotType::Humanoid);
robot.configure_teaching(&config)?;
let runtime = TeachRuntime::new(&config, "humanoid", None)?;
~~~

These examples are observation-only. Implement TeachAdapter to connect an
actual controller. The Cartesian adapter supports measured anchoring, frame
rotation, reach and speed limits. Adapters report readiness and command
acceptance and must implement bounded, nonblocking command and hold callbacks.
They must enforce control authority and a robot-side watchdog independently.
Registration is not proof that a physical controller is safe.

The native session implements lease expiry, engagement/hold, clutch release,
focus and tracking checks, sequence rejection, stream freshness and idempotent
requests. Rust callback panics are contained at the C ABI boundary and reported
as failures. Slow or blocked user callbacks remain the adapter author's
responsibility.

Both native hosts support opt-in Teach input v2. Add `input_schema_version: 2`,
and optionally `engagement_mode: "continuous"`, `engage_countdown_seconds: 3.0`
and `disengage_binding: "both_primary"` for timed engagement with X+A stop.
Continuous mode requires a matching mode in open negotiation and input samples;
`clutch` becomes the explicit permission gate and raw buttons stay measured.
The default is unchanged hold-to-run. See the [engagement contract](../python-sdk/teaching-input.md#continuous-engagement).

Simulation/fixture continuous v2 profiles may additionally set
`tracking_recovery: "auto_resume"`. Adapters must declare
`supports_tracking_pause=true` and implement `pause(context)` with measured hold
confirmation and pending-target disposal while preserving the anchor/authority.
The default virtual/trait method reports unsupported; existing adapters retain
manual recovery. `pause_requested` and `paused` accept only fresh validated
recovery input, never cached commands. Real-robot profiles cannot enable this
policy. See [tracking recovery](../python-sdk/teaching-input.md#tracking-recovery)
for mandatory negotiation, recording events and conditions that still latch Stop.
Configure
`required_input_features`, and your clutch/grasp bindings through the normal
configuration options. Head/controller poses, sticks, buttons and per-input
validity reach native adapter callbacks and MCAP events without Python. The
session negotiates features before ownership and rejects legacy clients for a
v2 profile. Version 1 remains unchanged. The
[input contract](../python-sdk/teaching-input.md) defines exact fields, axes,
timestamps and authority/freshness checks shared by all three SDKs.

## Episodes

The recorder writes MCAP CDR data with schemas, source timestamps where present,
receipt timestamps, events, checksums and a manifest compatible with Python.
Disk writes use a bounded worker queue. Overflow or incomplete data marks an
episode unacceptable; it does not silently claim a successful capture.
Finalization is durable and refuses to overwrite an existing episode.

Publication uses an atomic no-overwrite directory rename when supported.
On NTFS/FUSE filesystems that reject it, the recorder reserves a new destination,
hard-links the durable artifacts and publishes `manifest.json` last. Staging
data remains available until publication finishes; existing episodes are never
replaced. Filesystems lacking both operations report an error and retain staging.
After a power loss during fallback publication, inspect partial destination and
`.inprogress` directories rather than deleting either automatically.

Episode provides streaming reads and validation in either language.
The recover_episode function reads an interrupted .inprogress capture into a new
episode, preserves the original evidence, refuses an active writer, and marks
the result interrupted/unacceptable. Recovery does not prove that the lost tail
was empty. No recorded-data viewer is included; choose external analysis tools.

## Automated Checks

The native suites cover shared Python configuration/hash fixtures, invalid
models, joint/collision poses, model materials, recorder overflow, recovery,
leases and adapter failures. The isolated ROS tests exercise C++ and Rust
against real DDS peers: model replay on custom topics, topic/parameter/xacro
sources, CDR Image/String capture and cross-language episode validation.
They start no physical robot and are not headset or hardware acceptance tests.

See [Implementation Status](../reference/implementation-status.md) for remaining
native API gaps.
