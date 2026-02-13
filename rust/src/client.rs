/// Main SDK client for orchestration
pub struct Client {
    backend_type: String,
    auto_launch: bool,
}

impl Client {
    pub fn new(backend: impl Into<String>, auto_launch: bool) -> Self {
        Self {
            backend_type: backend.into(),
            auto_launch,
        }
    }

    pub async fn shutdown(&self) {
        // TODO: Implement shutdown logic
    }

    pub fn backend_type(&self) -> &str {
        &self.backend_type
    }

    pub fn auto_launch(&self) -> bool {
        self.auto_launch
    }
}
