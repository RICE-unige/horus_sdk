# HORUS C++ SDK

```
██╗  ██╗ ██████╗ ██████╗ ██╗   ██╗███████╗
██║  ██║██╔═══██╗██╔══██╗██║   ██║██╔════╝
███████║██║   ██║██████╔╝██║   ██║███████╗
██╔══██║██║   ██║██╔══██╗██║   ██║╚════██║
██║  ██║╚██████╔╝██║  ██║╚██████╔╝███████║
╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝
```

**Mixed Reality Robot Management SDK**

High-performance C++ implementation of the HORUS SDK for integrating ROS robots with Meta Quest mixed reality applications.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://isocpp.org/)
[![ROS](https://img.shields.io/badge/ROS-2%20Humble-blue.svg)](https://docs.ros.org/en/humble/)

## 🚀 Quick Start

### Installation

**Via Conan:**
```bash
conan install --requires=horus-cpp-sdk/0.1.0
```

**Via vcpkg:**
```bash
vcpkg install horus-cpp-sdk
```

**From Source:**
```bash
git clone https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk/cpp
mkdir build && cd build
cmake ..
cmake --build .
sudo cmake --install .
```

### Usage in CMake Projects

```cmake
find_package(horus_cpp_sdk REQUIRED)
target_link_libraries(your_target PRIVATE horus::horus_cpp_sdk)
```

## ✨ Features

- **Thread-Safe Event Bus**: Priority-based publish/subscribe messaging system
- **ROS 2 Integration**: Native ROS 2 Humble support with topic discovery
- **Robot Management**: Comprehensive robot and sensor modeling
- **DataViz System**: Flexible visualization configuration
- **Color Management**: Automatic color assignment for multi-robot scenarios
- **Modern C++17**: Smart pointers, move semantics, and thread-safe design
- **High Performance**: Zero-copy messaging, lock-free counters, parallel processing

## Building

### Prerequisites

- CMake 3.10+
- C++17 compliant compiler (GCC 7+, Clang 5+, MSVC 2017+)
- ROS 2 Humble (optional but recommended)

### Build Instructions

```bash
# Clone the repository
cd horus_sdk/cpp

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build
cmake --build .

# Install (optional)
sudo cmake --install .
```

## 📖 Usage Examples

### Basic Robot Setup

```cpp
#include <horus/utils/branding.hpp>
#include <horus/robot/robot.hpp>
#include <horus/robot/sensors.hpp>
#include <horus/core/event_bus.hpp>

int main() {
    // Display HORUS branding
    horus::utils::show_ascii_art();
    
    // Create a wheeled robot
    horus::robot::Robot robot("my_robot", horus::core::RobotType::WHEELED);
    robot.add_metadata("manufacturer", "RobotCo");
    
    // Add sensors
    auto laser = std::make_shared<horus::robot::LaserScan>(
        "front_laser", "base_laser_link", "/scan");
    laser->set_range_min(0.1f);
    laser->set_range_max(10.0f);
    robot.add_sensor(laser);
    
    auto camera = std::make_shared<horus::robot::Camera>(
        "rgb_camera", "camera_link", "/camera/image_raw");
    camera->set_resolution(1920, 1080);
    camera->set_framerate(30.0f);
    robot.add_sensor(camera);
    
    std::cout << "Robot configured with " 
              << robot.get_sensor_count() << " sensors" << std::endl;
    
    return 0;
}
```

### Event Bus Communication

```cpp
#include <horus/core/event_bus.hpp>
#include <iostream>
#include <thread>

int main() {
    // Subscribe to robot events
    auto sub_id = horus::core::subscribe(
        "robot.*",
        [](const horus::core::Event& event) {
            std::cout << "📬 Event: " << event.topic 
                     << " from " << event.source << std::endl;
        },
        horus::core::EventPriority::LOW
    );
    
    // Publish different priority events
    horus::core::publish("robot.status.ready", std::any{}, 
                        horus::core::EventPriority::NORMAL, "main");
    
    horus::core::publish("robot.emergency.stop", std::any{},
                        horus::core::EventPriority::CRITICAL, "safety");
    
    // Allow time for processing
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Check statistics
    auto& bus = horus::core::get_event_bus();
    auto stats = bus.get_statistics();
    std::cout << "📊 Published: " << stats.published_events
              << ", Processed: " << stats.processed_events << std::endl;
    
    // Cleanup
    horus::core::unsubscribe(sub_id);
    
    return 0;
}
```

### Multi-Sensor Robot

```cpp
#include <horus/robot/robot.hpp>
#include <horus/robot/sensors.hpp>
#include <memory>
#include <vector>

int main() {
    horus::robot::Robot robot("advanced_robot", horus::core::RobotType::AERIAL);
    
    // Add multiple cameras
    std::vector<std::string> camera_names = {"front", "back", "left", "right"};
    for (const auto& name : camera_names) {
        auto cam = std::make_shared<horus::robot::Camera>(
            name + "_cam", name + "_link", "/" + name + "/image");
        cam->set_resolution(640, 480);
        robot.add_sensor(cam);
    }
    
    // Add 3D lidar
    auto lidar = std::make_shared<horus::robot::Lidar3D>(
        "velodyne", "lidar_link", "/points");
    lidar->set_channels(32);
    lidar->set_range_max(100.0f);
    robot.add_sensor(lidar);
    
    std::cout << "Configured " << robot.get_name() 
              << " with " << robot.get_sensor_count() 
              << " sensors" << std::endl;
    
    return 0;
}
```

### Complete Examples

See the [examples/](examples/) directory for complete working examples:
- `sdk_registration_demo.cpp` - Full robot registration workflow
- See [examples/README.md](examples/README.md) for detailed documentation

## Architecture

### Core Components

- **EventBus**: Thread-safe event system with priority queues
- **TopicMap**: ROS topic discovery and management
- **Robot**: Robot modeling with sensors and metadata
- **Sensors**: Camera, LaserScan, Lidar3D implementations
- **DataViz**: Visualization configuration management
- **ColorManager**: Automatic color assignment

### Thread Safety

All core components are thread-safe using:
- `std::recursive_mutex` for internal synchronization
- `std::shared_ptr` for resource management
- `std::atomic` for lock-free counters

## API Documentation

### EventBus

```cpp
// Get global event bus
auto& bus = horus::core::get_event_bus();

// Publish event
bus.publish("topic.name", data, priority, "source");

// Subscribe to events
auto sub_id = bus.subscribe("topic.*", callback, priority_filter);

// Unsubscribe
bus.unsubscribe(sub_id);
```

### Robot

```cpp
// Create robot
horus::robot::Robot robot("robot_name", horus::core::RobotType::WHEELED);

// Add metadata
robot.add_metadata("key", value);

// Add sensor
robot.add_sensor(sensor_ptr);

// Get sensor count
size_t count = robot.get_sensor_count();
```

## 🚄 Performance Considerations

- **Zero-copy messaging**: Use move semantics for large data
- **Priority queues**: High-priority events processed first
- **Thread pool**: Callbacks executed in parallel
- **Lock-free counters**: Atomic operations for statistics
- **Smart pointers**: Efficient memory management
- **Mutex granularity**: Fine-grained locking for concurrency

## 📚 Documentation

- **API Reference**: See header files in `include/horus/`
- **Examples**: Complete examples in [examples/](examples/)

## 🤝 Contributing

Contributions welcome! Please see the main repository for guidelines.

## 📄 License

Apache License 2.0 - See [LICENSE](../LICENSE) file for details

## 🔗 Links

- [Main Repository](https://github.com/RICE-unige/horus_sdk)
- [Python SDK](../python/)
- [Rust SDK](../rust/)

## License

Apache License 2.0 - See LICENSE file for details
