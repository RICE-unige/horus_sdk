//! HORUS Mixed Reality Robot Management SDK - Rust implementation
//!
//! This crate provides a high-performance Rust implementation of the HORUS SDK
//! for integrating ROS robots with Meta Quest mixed reality applications.

pub mod bridge;
pub mod client;
pub mod color;
pub mod core;
pub mod dataviz;
pub mod robot;
pub mod sensors;
pub mod utils;

// Re-export commonly used types
pub use core::{
    event_bus::{get_event_bus, publish, subscribe, unsubscribe, Event, EventBus},
    topic_map::{get_topic_map, TopicInfo, TopicMap},
    types::*,
};

pub use client::Client;
pub use color::{ColorManager, RGBColor};
pub use dataviz::DataViz;
pub use robot::{
    register_robots, NavigationTaskConfig, Robot, RobotDescriptionConfig, RobotDimensions,
    RobotManagerConfig, TeleopConfig,
};
pub use sensors::{
    Camera, ImmersiveViewConfig, LaserScan, Lidar3D, MinimapViewConfig, ProjectedViewConfig,
    Sensor, WebRtcTransportConfig,
};

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
