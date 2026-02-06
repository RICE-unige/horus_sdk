# HORUS C++ SDK Examples

This directory contains example programs demonstrating how to use the HORUS C++ SDK.

## Building Examples

### With Installed SDK

If you've installed the HORUS C++ SDK system-wide:

```bash
cd examples
mkdir build && cd build
cmake ..
make
```

### With Source Build

If you're building from the repository:

```bash
cd cpp
mkdir build && cd build
cmake ..
make

# Examples will be in build/examples/
./examples/sdk_registration_demo
```

## Examples

### sdk_registration_demo

**File:** `sdk_registration_demo.cpp`

**Description:** Demonstrates how to:
- Create a robot with the HORUS SDK
- Add sensors (LaserScan, Camera) to the robot
- Subscribe to events using the EventBus
- Publish events
- View event bus statistics

**Usage:**
```bash
./sdk_registration_demo
```

**Expected Output:**
```
[HORUS ASCII Art]
🤖 HORUS Mixed Reality Robot Management SDK v0.1.0
🏗️  Developed at RICE Lab, University of Genoa
============================================================

📋 Defining Robot Configuration...

✅ Created robot: SdkBot_Professional
✅ Added sensor: Front Lidar
✅ Added sensor: Realsense RGB Camera

📊 Robot Configuration Summary:
  Name: SdkBot_Professional
  Type: wheeled
  Sensors: 2

📡 Setting up event subscriptions...
✅ Subscribed to robot events

📤 Publishing robot status event...
📬 Received event: robot.status.ready

🔗 Registration with HORUS backend:
  Note: Full HORUS backend integration requires ROS 2 runtime
  This demo shows the robot configuration setup

📈 Event Bus Statistics:
  Events Published: 1
  Events Processed: 1
  Active Subscriptions: 1

✨ Demo completed successfully!
```

## Creating Your Own Example

1. Create a new `.cpp` file in this directory
2. Add it to `CMakeLists.txt`:

```cmake
add_executable(my_example my_example.cpp)
target_link_libraries(my_example PRIVATE horus::horus_cpp_sdk)
```

3. Build and run:

```bash
cd build
cmake ..
make
./my_example
```

## API Reference

For complete API documentation, see the [C++ SDK README](../README.md).

## Common Use Cases

### Creating a Wheeled Robot
```cpp
horus::robot::Robot robot("my_robot", horus::core::RobotType::WHEELED);
```

### Adding a Laser Scanner
```cpp
auto laser = std::make_shared<horus::robot::LaserScan>(
    "front_laser", "laser_frame", "/scan");
robot.add_sensor(laser);
```

### Subscribing to Events
```cpp
auto sub_id = horus::core::subscribe("robot.*", 
    [](const horus::core::Event& event) {
        std::cout << "Event: " << event.topic << std::endl;
    });
```

### Publishing Events
```cpp
horus::core::publish("robot.status", data, 
                    horus::core::EventPriority::NORMAL, "source");
```

## Troubleshooting

### Cannot Find horus_cpp_sdk Package

Make sure the SDK is installed:
```bash
cd cpp
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
sudo cmake --install .
```

### Link Errors

Ensure you're linking against the library:
```cmake
target_link_libraries(your_target PRIVATE horus::horus_cpp_sdk)
```

## Further Reading

- [C++ SDK Documentation](../README.md)
- [HORUS SDK Repository](https://github.com/RICE-unige/horus_sdk)
