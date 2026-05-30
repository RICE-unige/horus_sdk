---
title: C++ SDK
sidebar_position: 1
---

# C++ SDK

The C++ SDK builds the same MR registration contract as the Python SDK without depending on the Python runtime. Use it when your robot process is already C++, when registration is part of a ROS 2 node, or when startup and payload generation overhead need to stay low.

## What it sends

`RobotRegistryClient::build_robot_config_dict(...)` produces the `/horus/registration` payload consumed by `horus_unity_bridge` and HORUS MR:

| Area | C++ support |
| --- | --- |
| Robot identity | name, type, dimensions, HORUS color/id metadata |
| ROS binding | prefixed or flat topics and TF frames |
| Robot Manager | status, DataViz, teleop, and task panel sections |
| Controls | teleop defaults/overrides, go-to-point, waypoint topics |
| Cameras | minimap/teleop transport profiles, WebRTC settings, stereo fields, view/projection offsets |
| DataViz | robot transforms, paths, velocity text, odometry trails, collision risk, occupancy, pointcloud, mesh, octomap, semantic boxes |
| Workspace | position scale, compass metadata, tutorial preset, local body model id |

## Build and test

```bash
cd ~/horus_sdk/cpp
cmake -S . -B build_no_ros -DHORUS_ENABLE_ROS2=OFF
cmake --build build_no_ros --parallel
ctest --test-dir build_no_ros --output-on-failure
```

Enable ROS 2 integration when building inside a sourced ROS shell:

```bash
source /opt/ros/jazzy/setup.bash
source ~/horus_ws/install/setup.bash
cmake -S . -B build_ros -DHORUS_ENABLE_ROS2=ON
cmake --build build_ros --parallel
```

## Minimal example

```cpp
#include "horus/bridge/robot_registry.hpp"
#include "horus/robot/robot.hpp"
#include "horus/robot/sensors.hpp"

#include <memory>

int main() {
    horus::robot::Robot robot("atlas", horus::core::RobotType::WHEELED);
    robot.configure_ros_binding("prefixed", "prefixed", "base_link");
    robot.configure_robot_manager(true, true, true, true);

    auto camera = std::make_shared<horus::robot::Camera>(
        "front_camera",
        "atlas/camera_link",
        "/atlas/camera/image_raw/compressed");
    camera->set_minimap_topic("/atlas/camera/minimap/compressed");
    camera->set_teleop_topic("/atlas/camera/image_raw/compressed");
    camera->set_minimap_image_type("compressed");
    camera->set_teleop_image_type("compressed");
    robot.add_sensor(camera);

    auto dataviz = robot.create_dataviz();
    robot.add_path_planning_to_dataviz(
        *dataviz,
        robot.resolve_topic("global_path"),
        robot.resolve_topic("local_path"),
        robot.resolve_topic("trajectory"));
    robot.add_navigation_safety_to_dataviz(*dataviz);
    dataviz->add_occupancy_grid("/map", "map");
    dataviz->add_3d_octomap("/atlas/octomap_mesh", "map");

    horus::bridge::RobotRegistryClient client;
    auto payload = client.build_robot_config_dict(robot, *dataviz, std::nullopt, 0.25);
    return payload.robot_name == "atlas" ? 0 : 1;
}
```

## Full demo

The native parity demo lives at `cpp/examples/sdk_registration_demo.cpp`.

```bash
cd ~/horus_sdk/cpp
./build_no_ros/examples/sdk_registration_demo --robot-count 4 --workspace-scale 0.25
```

The demo configures camera view profiles, teleop/task payloads, Robot Manager metadata, navigation safety visualizations, global 3D map payloads, semantic boxes, workspace compass/tutorial metadata, and an optional local body model id.
