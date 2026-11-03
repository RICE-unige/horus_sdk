use crate::bridge::{RegistryOptions, RobotRegistryClient};
use std::sync::Arc;

/// Owns an independent registry. Initialization remains lazy until registration.
pub struct Client {
    backend_type: String,
    auto_launch: bool,
    registry: Arc<RobotRegistryClient>,
}

impl Client {
    pub fn new(backend: impl Into<String>, auto_launch: bool) -> Self {
        Self {
            backend_type: backend.into(),
            auto_launch,
            registry: Arc::new(RobotRegistryClient::with_options(RegistryOptions {
                auto_launch,
                ..RegistryOptions::default()
            })),
        }
    }

    pub async fn shutdown(&self) {
        self.registry.stop();
    }

    pub fn registry(&self) -> Result<Arc<RobotRegistryClient>, String> {
        if self.backend_type != "ros2" {
            return Err("Native registration requires backend=ros2".into());
        }
        Ok(self.registry.clone())
    }

    pub fn backend_type(&self) -> &str {
        &self.backend_type
    }

    pub fn auto_launch(&self) -> bool {
        self.auto_launch
    }
}
