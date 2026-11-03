#[derive(Debug, Clone)]
pub struct ConnectionDashboard {
    ip: String,
    port: u16,
    bridge_status: String,
    app_link_state: String,
    registration_state: String,
    sub_status: String,
    topic_rows: Vec<serde_json::Value>,
    show_counts: bool,
}

impl ConnectionDashboard {
    pub fn new(
        ip: impl Into<String>,
        port: u16,
        bridge_status: impl Into<String>,
        show_counts: bool,
    ) -> Self {
        Self {
            ip: ip.into(),
            port,
            bridge_status: bridge_status.into(),
            app_link_state: "Waiting for Horus App...".to_string(),
            registration_state: "Idle".to_string(),
            sub_status: String::new(),
            topic_rows: Vec::new(),
            show_counts,
        }
    }

    pub fn update_status(&mut self, status: impl Into<String>) {
        self.sub_status = status.into();
        self.render();
    }

    pub fn update_app_link(&mut self, status: impl Into<String>) {
        self.app_link_state = status.into();
        self.render();
    }

    pub fn update_registration(&mut self, status: impl Into<String>) {
        self.registration_state = status.into();
        self.render();
    }

    pub fn update_bridge(&mut self, status: impl Into<String>) {
        self.bridge_status = status.into();
        self.render();
    }

    pub fn update_topics(&mut self, rows: Vec<serde_json::Value>) {
        self.topic_rows = rows;
        self.render();
    }

    pub fn tick(&self) {
        self.render();
    }

    pub fn render(&self) {
        let state = serde_json::json!({
            "app_connected": self.app_link_state.to_ascii_lowercase().starts_with("connected"),
            "heartbeat_seen": self.app_link_state.to_ascii_lowercase().starts_with("disconnected"),
            "bridge_state": format!("{} | {}:{}", self.bridge_status, self.ip, self.port),
            "registration_state": self.registration_state,
            "error": self.sub_status,
            "topics": self.topic_rows,
            "show_counts": self.show_counts,
        });
        match render_snapshot(&state, 100, 24, 0, false) {
            Ok(text) => print!("{text}"),
            Err(error) => eprintln!("HORUS dashboard: {error}"),
        }
    }
}

/// Render `RobotRegistryClient::status()` without acquiring sensor subscriptions.
/// Live registration uses the same renderer with terminal sizing and pagination.
pub fn render_snapshot(
    snapshot: &serde_json::Value,
    width: u32,
    rows: u32,
    page: u32,
    color: bool,
) -> anyhow::Result<String> {
    use std::ffi::{c_char, CStr, CString};
    extern "C" {
        fn horus_dashboard_render(
            snapshot: *const c_char,
            width: u32,
            rows: u32,
            page: u32,
            color: i32,
        ) -> *mut c_char;
        fn horus_teach_free(value: *mut c_char);
    }
    let input = CString::new(serde_json::to_vec(snapshot)?)?;
    let result = unsafe { horus_dashboard_render(input.as_ptr(), width, rows, page, color.into()) };
    if result.is_null() {
        anyhow::bail!("Native dashboard allocation failed");
    }
    let json = unsafe {
        let parsed = serde_json::from_slice::<serde_json::Value>(CStr::from_ptr(result).to_bytes());
        horus_teach_free(result);
        parsed?
    };
    if json["ok"] != true {
        anyhow::bail!("{}", json["error"]);
    }
    json["value"]
        .as_str()
        .map(str::to_owned)
        .ok_or_else(|| anyhow::anyhow!("Native dashboard returned invalid text"))
}

pub fn parse_bool_flag(value: Option<&str>, default: bool) -> bool {
    match value {
        Some(v) => match v.trim().to_ascii_lowercase().as_str() {
            "1" | "true" | "yes" | "on" => true,
            "0" | "false" | "no" | "off" => false,
            _ => default,
        },
        None => default,
    }
}

pub fn print_step(message: impl AsRef<str>) {
    println!("* {}", message.as_ref());
}

pub fn print_success(message: impl AsRef<str>) {
    println!("+ {}", message.as_ref());
}

pub fn print_error(message: impl AsRef<str>) {
    eprintln!("- {}", message.as_ref());
}

pub fn print_info(message: impl AsRef<str>) {
    println!("{}", message.as_ref());
}
