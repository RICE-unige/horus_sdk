use clap::Parser;
use std::time::Duration;

#[derive(Debug, Parser)]
#[command(about = "Fake TF publisher parity scaffold (Rust).")]
struct Args {
    #[arg(long, default_value = "test_bot")]
    robot_name: String,
    #[arg(long, default_value_t = 10.0)]
    rate_hz: f64,
    #[arg(long, default_value_t = 5)]
    cycles: u32,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let period = Duration::from_secs_f64((1.0 / args.rate_hz.max(0.1)).max(0.001));
    println!(
        "Publishing fake TF updates for '{}' at {:.2} Hz ({} cycles)",
        args.robot_name, args.rate_hz, args.cycles
    );
    for idx in 0..args.cycles {
        println!(
            "[fake_tf] robot={} frame={}/base_link seq={}",
            args.robot_name,
            args.robot_name,
            idx + 1
        );
        tokio::time::sleep(period).await;
    }
}

