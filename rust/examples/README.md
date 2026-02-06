# HORUS Rust SDK Examples

This directory contains example programs demonstrating how to use the HORUS Rust SDK.

## Running Examples

### From Repository

```bash
# Run specific example
cargo run --example sdk_registration_demo

# Run with release optimizations
cargo run --release --example sdk_registration_demo
```

### With Installed Crate

If you've added `horus-sdk` to your `Cargo.toml`:

```bash
cargo run --example sdk_registration_demo
```

## Examples

### sdk_registration_demo

**File:** `sdk_registration_demo.rs`

**Description:** Demonstrates how to:
- Create a robot with the HORUS SDK
- Add sensors (LaserScan, Camera) to the robot
- Subscribe to events using the async EventBus
- Publish events
- View event bus statistics

**Usage:**
```bash
cargo run --example sdk_registration_demo
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

1. Create a new `.rs` file in this directory (e.g., `my_example.rs`)
2. Add it to `Cargo.toml`:

```toml
[[example]]
name = "my_example"
path = "examples/my_example.rs"
```

3. Add async runtime if needed:

```rust
#[tokio::main]
async fn main() {
    // Your code here
}
```

4. Run it:

```bash
cargo run --example my_example
```

## API Reference

For complete API documentation:

```bash
cargo doc --open
```

Or visit [docs.rs/horus-sdk](https://docs.rs/horus-sdk) after publication.

## Common Use Cases

### Creating a Wheeled Robot
```rust
let mut robot = Robot::new("my_robot", RobotType::Wheeled);
```

### Adding a Laser Scanner
```rust
let laser = LaserScan::new("front_laser", "laser_frame", "/scan");
robot.add_sensor(Arc::new(laser));
```

### Subscribing to Events
```rust
let sub_id = horus::subscribe(
    "robot.*".to_string(),
    Arc::new(|event| {
        println!("Event: {}", event.topic);
    }),
    EventPriority::Low,
).await;
```

### Publishing Events
```rust
horus::publish(
    "robot.status",
    serde_json::json!({"status": "ready"}),
    EventPriority::Normal,
    "source",
).await;
```

## Async Programming

All examples use `tokio` as the async runtime. Key points:

- Use `#[tokio::main]` for the main function
- Use `.await` for async operations
- Event callbacks run in parallel via `tokio::spawn`

### Example Template

```rust
use horus::{Robot, RobotType};

#[tokio::main]
async fn main() {
    // Display branding
    horus::utils::show_ascii_art();
    
    // Your async code here
    let robot = Robot::new("my_robot", RobotType::Wheeled);
    
    // Subscribe to events
    let sub_id = horus::subscribe(
        "robot.*".to_string(),
        Arc::new(|event| {
            println!("Event: {}", event.topic);
        }),
        EventPriority::Low,
    ).await;
    
    // Publish events
    horus::publish(
        "robot.status",
        serde_json::json!({"ready": true}),
        EventPriority::Normal,
        "demo",
    ).await;
    
    // Cleanup
    horus::unsubscribe(&sub_id).await;
}
```

## Troubleshooting

### Tokio Runtime Errors

Make sure you have `#[tokio::main]` on your async main function and tokio is in your dependencies:

```toml
[dependencies]
horus-sdk = "0.1.0"
tokio = { version = "1.35", features = ["full"] }
```

### Event Not Received

Give the event system time to process:

```rust
tokio::time::sleep(Duration::from_millis(100)).await;
```

## Further Reading

- [Rust SDK Documentation](../README.md)
- [HORUS SDK Repository](https://github.com/RICE-unige/horus_sdk)
- [Tokio Documentation](https://tokio.rs)
