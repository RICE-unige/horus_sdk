//! HORUS Mixed Reality Robot Management SDK - Rust implementation
//!
//! This crate provides a high-performance Rust implementation of the HORUS SDK
//! for integrating ROS robots with Meta Quest mixed reality applications.

pub mod core;
pub mod robot;
pub mod sensors;
pub mod dataviz;
pub mod color;
pub mod bridge;
pub mod client;
pub mod utils;

// Re-export commonly used types
pub use core::{
    types::*,
    event_bus::{Event, EventBus, get_event_bus, publish, subscribe, unsubscribe},
    topic_map::{TopicMap, TopicInfo, get_topic_map},
};

pub use robot::Robot;
pub use sensors::{Sensor, Camera, LaserScan, Lidar3D};
pub use dataviz::DataViz;
pub use color::{ColorManager, RGBColor};
pub use client::Client;

/// SDK version
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_version() {
        assert!(!VERSION.is_empty());
    }
}
