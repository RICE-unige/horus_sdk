/// Robot registry client for HORUS backend
pub struct RobotRegistryClient;

impl RobotRegistryClient {
    pub fn new() -> Self {
        Self
    }
}

impl Default for RobotRegistryClient {
    fn default() -> Self {
        Self::new()
    }
}
