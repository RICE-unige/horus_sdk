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
        println!(
            "[dashboard] bridge={} ip={} port={} app={} registration={} detail={} rows={} counts={}",
            self.bridge_status,
            self.ip,
            self.port,
            self.app_link_state,
            self.registration_state,
            self.sub_status,
            self.topic_rows.len(),
            self.show_counts
        );
    }
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

