# HORUS Rust SDK

```
██╗  ██╗ ██████╗ ██████╗ ██╗   ██╗███████╗
██║  ██║██╔═══██╗██╔══██╗██║   ██║██╔════╝
███████║██║   ██║██████╔╝██║   ██║███████╗
██╔══██║██║   ██║██╔══██╗██║   ██║╚════██║
██║  ██║╚██████╔╝██║  ██║╚██████╔╝███████║
╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝
```

**Mixed Reality Robot Management SDK**

High-performance Rust implementation of the HORUS SDK for integrating ROS robots with Meta Quest mixed reality applications.

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/Rust-1.70+-orange.svg)](https://www.rust-lang.org/)
[![ROS](https://img.shields.io/badge/ROS-2%20Humble-blue.svg)](https://docs.ros.org/en/humble/)

## 🚀 Quick Start

### Installation

**Add to Cargo.toml:**
```toml
[dependencies]
horus-sdk = "0.1.0"
tokio = { version = "1.35", features = ["full"] }
serde_json = "1.0"
```

**Or via Cargo CLI:**
```bash
cargo add horus-sdk
cargo add tokio --features full
```

**From Source:**
```bash
git clone https://github.com/RICE-unige/horus_sdk.git
cd horus_sdk/rust
cargo build --release
```

## ✨ Features

- **Async Event Bus**: Tokio-based async publish/subscribe messaging system
- **ROS 2 Integration**: Native ROS 2 support via r2r or rclrs
- **Zero-Cost Abstractions**: Leverages Rust's ownership system for memory safety
- **Type Safety**: Strong typing with compile-time guarantees
- **Concurrent Processing**: Efficient async/await for event handling
- **Serde Integration**: JSON serialization for ROS communication
- **Memory Safe**: No null pointers, buffer overflows, or data races
- **High Performance**: Zero-copy operations, parallel processing

## Building

### Prerequisites

- Rust 1.70+ (2021 edition)
- Cargo
- ROS 2 Humble (optional)

### Build Instructions

```bash
# Clone the repository
cd horus_sdk/rust

# Build
cargo build --release

# Run tests
cargo test

# Build documentation
cargo doc --open
```

## 📖 Usage Examples

### Basic Robot Setup

```rust
use horus::{Robot, RobotType, LaserScan, Camera};
use std::sync::Arc;

#[tokio::main]
async fn main() {
    // Display HORUS branding
    horus::utils::show_ascii_art();
    
    // Create a wheeled robot
    let mut robot = Robot::new("my_robot", RobotType::Wheeled);
    robot.add_metadata("manufacturer", serde_json::json!("RobotCo"));
    
    // Add sensors
    let mut laser = LaserScan::new("front_laser", "base_laser_link", "/scan");
    laser.set_range_min(0.1);
    laser.set_range_max(10.0);
    robot.add_sensor(Arc::new(laser));
    
    let mut camera = Camera::new("rgb_camera", "camera_link", "/camera/image_raw");
    camera.set_resolution(1920, 1080);
    camera.set_framerate(30.0);
    robot.add_sensor(Arc::new(camera));
    
    println!("Robot configured with {} sensors", robot.get_sensor_count());
}
```

### Event Bus Communication

```rust
use horus::{subscribe, publish, unsubscribe, get_event_bus, EventPriority};
use std::sync::Arc;
use std::time::Duration;

#[tokio::main]
async fn main() {
    // Subscribe to robot events
    let sub_id = subscribe(
        "robot.*".to_string(),
        Arc::new(|event| {
            println!("📬 Event: {} from {}", event.topic, event.source);
        }),
        EventPriority::Low,
    ).await;
    
    // Publish different priority events
    publish(
        "robot.status.ready",
        serde_json::json!({"ready": true}),
        EventPriority::Normal,
        "main",
    ).await;
    
    publish(
        "robot.emergency.stop",
        serde_json::json!({"emergency": true}),
        EventPriority::Critical,
        "safety",
    ).await;
    
    // Allow time for processing
    tokio::time::sleep(Duration::from_millis(100)).await;
    
    // Check statistics
    let bus = get_event_bus();
    let stats = bus.get_statistics().await;
    println!("📊 Published: {}, Processed: {}", 
            stats.published_events, stats.processed_events);
    
    // Cleanup
    unsubscribe(&sub_id).await;
}
```

### Multi-Sensor Robot with Color Management

```rust
use horus::{Robot, RobotType, Camera, Lidar3D, ColorManager, ColorScheme};
use std::sync::Arc;

#[tokio::main]
async fn main() {
    let mut robot = Robot::new("advanced_robot", RobotType::Aerial);
    
    // Add multiple cameras
    for name in ["front", "back", "left", "right"] {
        let mut cam = Camera::new(
            &format!("{}_cam", name),
            &format!("{}_link", name),
            &format!("/{}/image", name),
        );
        cam.set_resolution(640, 480);
        robot.add_sensor(Arc::new(cam));
    }
    
    // Add 3D lidar
    let mut lidar = Lidar3D::new("velodyne", "lidar_link", "/points");
    lidar.set_channels(32);
    lidar.set_range_max(100.0);
    robot.add_sensor(Arc::new(lidar));
    
    // Assign colors
    let color_mgr = ColorManager::new(ColorScheme::Bright);
    let robot_color = color_mgr.get_color(&robot.get_name());
    println!("Robot color: #{}", robot_color.to_hex());
    
    println!("Configured {} with {} sensors", 
            robot.get_name(), robot.get_sensor_count());
}
```

### Async Event Processing

```rust
use horus::{subscribe, publish, EventPriority};
use std::sync::Arc;
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn main() {
    // Subscribe with async callback
    let sub_id = subscribe(
        "sensor.data".to_string(),
        Arc::new(|event| {
            // This runs in parallel via tokio::spawn
            if let Some(data) = event.data.as_ref() {
                println!("Processing sensor data asynchronously");
            }
        }),
        EventPriority::Normal,
    ).await;
    
    // Publish events in a loop
    for i in 0..10 {
        publish(
            "sensor.data",
            serde_json::json!({"reading": i}),
            EventPriority::Normal,
            "sensor",
        ).await;
        
        sleep(Duration::from_millis(100)).await;
    }
    
    horus::unsubscribe(&sub_id).await;
}
```

### Complete Examples

See the [examples/](examples/) directory for complete working examples:
- `sdk_registration_demo.rs` - Full robot registration workflow
- See [examples/README.md](examples/README.md) for detailed documentation

## Architecture

### Core Components

- **EventBus**: Async event system with tokio channels
- **TopicMap**: ROS topic discovery with async/await
- **Robot**: Robot modeling with trait-based sensors
- **Sensors**: Camera, LaserScan, Lidar3D with trait implementation
- **DataViz**: Visualization configuration
- **ColorManager**: Color assignment with MD5 hashing

### Async Design

All I/O operations use `async/await`:
- Non-blocking event processing
- Parallel callback execution with `tokio::spawn`
- `Arc<RwLock<T>>` for shared mutable state
- Zero-copy where possible

## API Documentation

### EventBus

```rust
// Get global event bus
let bus = horus::get_event_bus();

// Publish event
bus.publish_simple("topic.name", data, priority, "source").await;

// Subscribe to events
let sub_id = bus.subscribe(
    "topic.*".to_string(),
    callback,
    priority_filter
).await;

// Unsubscribe
bus.unsubscribe(&sub_id).await;
```

### Robot

```rust
// Create robot
let mut robot = Robot::new("robot_name", RobotType::Wheeled);

// Add metadata
robot.add_metadata("key", serde_json::json!("value"));

// Add sensor
robot.add_sensor(Arc::new(sensor));

// Get sensor count
let count = robot.get_sensor_count();
```

## 🚄 Performance Optimizations

- **Zero-copy**: Leverages Rust's ownership for efficient data transfer
- **Async I/O**: Non-blocking operations with tokio
- **Compile-time checks**: No runtime overhead for type safety
- **SIMD**: Potential for vectorized operations in sensor processing
- **Parallel callbacks**: Event handlers run concurrently via `tokio::spawn`
- **Lock-free reads**: `Arc<RwLock<T>>` allows multiple concurrent readers

## Safety Guarantees

- **Memory safety**: No null pointers or buffer overflows
- **Thread safety**: `Send + Sync` traits ensure safe concurrency
- **Data races**: Prevented by ownership system
- **Resource leaks**: Automatic cleanup with RAII

## 📦 Dependencies

Key dependencies:
- `tokio`: Async runtime
- `serde`: Serialization
- `uuid`: Unique IDs
- `once_cell`: Lazy static initialization
- `parking_lot`: Efficient synchronization primitives
- `md5`: Color generation

## 📚 Documentation

- **API Reference**: `cargo doc --open`
- **Examples**: Complete examples in [examples/](examples/)

## 📦 Testing

```bash
# Run all tests
cargo test

# Run with output
cargo test -- --nocapture

# Run specific test
cargo test event_bus

# Run documentation tests
cargo test --doc
```

## 🔨 Development

```bash
# Check code
cargo check

# Format code
cargo fmt

# Lint code
cargo clippy

# Build documentation
cargo doc --no-deps --open
```

## 🤝 Contributing

Contributions welcome! Please see the main repository for guidelines.

## 📄 License

Apache License 2.0 - See [LICENSE](../LICENSE) file for details

## 🏛️ Research Lab

Developed at **RICE Lab** (Robots and Intelligent Systems for Citizens and the Environment)
University of Genoa, Italy

## 🔗 Links

- [Main Repository](https://github.com/RICE-unige/horus_sdk)
- [Python SDK](../python/)
- [C++ SDK](../cpp/)
- [crates.io](https://crates.io/crates/horus-sdk) (after publication)
- [docs.rs](https://docs.rs/horus-sdk) (after publication)
