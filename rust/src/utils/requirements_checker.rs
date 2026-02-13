use std::process::Command;

#[derive(Debug, Clone, Default)]
pub struct RequirementReport {
    pub ros2_available: bool,
    pub commands: Vec<(String, bool)>,
}

#[derive(Debug, Clone, Default)]
pub struct RequirementsChecker;

impl RequirementsChecker {
    pub fn new() -> Self {
        Self
    }

    pub fn check_all(&self) -> Result<RequirementReport, String> {
        let mut report = RequirementReport {
            ros2_available: command_available("ros2"),
            commands: Vec::new(),
        };
        for command in ["ros2", "ss", "netstat"] {
            report
                .commands
                .push((command.to_string(), command_available(command)));
        }
        Ok(report)
    }
}

fn command_available(command: &str) -> bool {
    Command::new("sh")
        .arg("-c")
        .arg(format!("command -v {command} >/dev/null 2>&1"))
        .status()
        .map(|status| status.success())
        .unwrap_or(false)
}

