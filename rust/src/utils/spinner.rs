use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

pub struct Spinner {
    message: String,
    delay: Duration,
    style: Vec<&'static str>,
    running: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

impl Spinner {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
            delay: Duration::from_millis(100),
            style: vec!["-", "\\", "|", "/"],
            running: Arc::new(AtomicBool::new(false)),
            handle: None,
        }
    }

    pub fn start(&mut self) {
        if self.running.swap(true, Ordering::Relaxed) {
            return;
        }
        let running = Arc::clone(&self.running);
        let message = self.message.clone();
        let style = self.style.clone();
        let delay = self.delay;
        self.handle = Some(thread::spawn(move || {
            let mut idx = 0usize;
            while running.load(Ordering::Relaxed) {
                print!("\r  \x1b[96m{}\x1b[0m {}...", style[idx % style.len()], message);
                let _ = std::io::stdout().flush();
                idx = idx.wrapping_add(1);
                thread::sleep(delay);
            }
        }));
    }

    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
        print!("\r{}\r", " ".repeat(self.message.len() + 16));
        let _ = std::io::stdout().flush();
    }
}
