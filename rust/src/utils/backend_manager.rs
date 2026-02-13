use std::collections::HashMap;
use std::net::{SocketAddr, TcpStream};
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct BackendConfig {
    pub package: String,
    pub launch_file: String,
    pub launch_command: Vec<String>,
    pub tcp_port: u16,
    pub unity_port: u16,
}

#[derive(Debug)]
pub struct BackendManager {
    pub backend_type: String,
    pub backend_process: Option<Child>,
    backend_configs: HashMap<String, BackendConfig>,
    shutdown_called: bool,
}

impl Default for BackendManager {
    fn default() -> Self {
        Self::new("ros2")
    }
}

impl BackendManager {
    pub fn new(backend_type: impl Into<String>) -> Self {
        let backend_type = backend_type.into();
        let mut backend_configs = HashMap::new();
        backend_configs.insert(
            "ros2".to_string(),
            BackendConfig {
                package: "horus_backend".to_string(),
                launch_file: "horus_complete_backend.launch.py".to_string(),
                launch_command: vec![
                    "ros2".to_string(),
                    "launch".to_string(),
                    "horus_backend".to_string(),
                    "horus_complete_backend.launch.py".to_string(),
                ],
                tcp_port: 8080,
                unity_port: 10000,
            },
        );

        Self {
            backend_type,
            backend_process: None,
            backend_configs,
            shutdown_called: false,
        }
    }

    pub fn launch_backend(&mut self) -> Result<(), String> {
        if self.is_backend_running() {
            return Ok(());
        }
        self.start_backend_process()?;
        self.wait_for_backend_ready(Duration::from_secs(30))
    }

    pub fn is_backend_running(&self) -> bool {
        self.config()
            .map(|cfg| is_port_open(cfg.tcp_port))
            .unwrap_or(false)
    }

    pub fn is_unity_endpoint_running(&self) -> bool {
        self.config()
            .map(|cfg| is_port_open(cfg.unity_port))
            .unwrap_or(false)
    }

    pub fn test_connection(&self) -> bool {
        self.is_backend_running()
    }

    pub fn test_unity_endpoint(&self) -> bool {
        self.is_unity_endpoint_running()
    }

    pub fn get_port(&self) -> Option<u16> {
        self.config().map(|cfg| cfg.tcp_port)
    }

    pub fn get_unity_port(&self) -> Option<u16> {
        self.config().map(|cfg| cfg.unity_port)
    }

    pub fn stop_backend(&mut self) {
        if self.shutdown_called {
            return;
        }
        self.shutdown_called = true;

        if let Some(mut child) = self.backend_process.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }

    fn config(&self) -> Option<&BackendConfig> {
        self.backend_configs.get(&self.backend_type)
    }

    fn start_backend_process(&mut self) -> Result<(), String> {
        let cfg = self
            .config()
            .ok_or_else(|| format!("Unknown backend type '{}'", self.backend_type))?;
        if cfg.launch_command.is_empty() {
            return Err("Launch command is empty".to_string());
        }

        let mut cmd = Command::new(&cfg.launch_command[0]);
        cmd.args(&cfg.launch_command[1..])
            .stdout(Stdio::null())
            .stderr(Stdio::null());

        let child = cmd
            .spawn()
            .map_err(|e| format!("Failed to spawn backend process: {e}"))?;
        self.backend_process = Some(child);
        Ok(())
    }

    fn wait_for_backend_ready(&self, timeout: Duration) -> Result<(), String> {
        let Some(cfg) = self.config() else {
            return Err(format!("Unknown backend type '{}'", self.backend_type));
        };

        let start = Instant::now();
        while start.elapsed() < timeout {
            if is_port_open(cfg.tcp_port) {
                return Ok(());
            }
            thread::sleep(Duration::from_secs(1));
        }
        Err(format!(
            "Backend failed to start after {} seconds",
            timeout.as_secs()
        ))
    }
}

fn is_port_open(port: u16) -> bool {
    let addr = SocketAddr::from(([127, 0, 0, 1], port));
    TcpStream::connect_timeout(&addr, Duration::from_millis(300)).is_ok()
}

