#!/usr/bin/env rust
//! Example of registering a custom robot with HORUS using the Rust SDK
//! 
//! Usage:
//!   cargo run --example sdk_registration_demo

use horus::{Robot, RobotType, Camera, LaserScan, EventPriority};
use horus::utils::show_ascii_art;
use std::sync::Arc;
use std::time::Duration;

#[tokio::main]
async fn main() {
    // Display HORUS branding
    show_ascii_art();
    
    println!("\n📋 Defining Robot Configuration...\n");
    
    // 1. Define your robot
    let mut my_robot = Robot::new("SdkBot_Professional", RobotType::Wheeled);
    
    println!("✅ Created robot: {}", my_robot.name);
    
    // 2. Add Sensors
    let front_laser = LaserScan::new(
        "Front Lidar",
        "laser_frame",
        "/scan",
    );
    
    my_robot.add_sensor(Arc::new(front_laser));
    println!("✅ Added sensor: Front Lidar");
    
    let camera = Camera::new(
        "Realsense RGB",
        "camera_link",
        "/camera/color/image_raw",
    );
    
    my_robot.add_sensor(Arc::new(camera));
    println!("✅ Added sensor: Realsense RGB Camera");
    
    // 3. Add metadata
    my_robot.add_metadata("description", serde_json::json!("Professional wheeled robot"));
    my_robot.add_metadata("max_speed", serde_json::json!(2.5));
    
    println!("\n📊 Robot Configuration Summary:");
    println!("  Name: {}", my_robot.name);
    println!("  Type: {}", my_robot.get_type_str());
    println!("  Sensors: {}", my_robot.get_sensor_count());
    
    // 4. Subscribe to events
    println!("\n📡 Setting up event subscriptions...");
    
    let sub_id = horus::subscribe(
        "robot.*".to_string(),
        Arc::new(|event| {
            println!("📬 Received event: {}", event.topic);
        }),
        EventPriority::Low,
    ).await;
    
    println!("✅ Subscribed to robot events");
    
    // 5. Publish a status event
    println!("\n📤 Publishing robot status event...");
    
    horus::publish(
        "robot.status.ready",
        serde_json::json!({"status": "ready"}),
        EventPriority::Normal,
        "demo",
    ).await;
    
    // Give the event system time to process
    tokio::time::sleep(Duration::from_millis(100)).await;
    
    // 6. Registration with HORUS backend would happen here
    println!("\n🔗 Registration with HORUS backend:");
    println!("  Note: Full HORUS backend integration requires ROS 2 runtime");
    println!("  This demo shows the robot configuration setup");
    
    // Show event bus statistics
    let stats = horus::get_event_bus().get_stats().await;
    println!("\n📈 Event Bus Statistics:");
    println!("  Events Published: {}", stats.get("events_published").unwrap_or(&0));
    println!("  Events Processed: {}", stats.get("events_processed").unwrap_or(&0));
    println!("  Active Subscriptions: {}", stats.get("active_subscriptions").unwrap_or(&0));
    
    println!("\n✨ Demo completed successfully!");
    
    // Cleanup
    horus::unsubscribe(&sub_id).await;
}
