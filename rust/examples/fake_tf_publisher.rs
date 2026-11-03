use clap::Parser;
use horus::tf::{Publisher, PublisherOptions, Transform};
use std::time::{Duration, Instant};

#[derive(Debug, Parser)]
#[command(about = "Publish a moving robot base and retained camera/lidar TF on ROS 2.")]
struct Args {
    #[arg(long, default_value = "test_bot")]
    robot_name: String,
    #[arg(long, default_value_t = 10.0)]
    rate_hz: f64,
    #[arg(long, default_value_t = 20)]
    cycles: u32,
}

#[tokio::main(worker_threads = 1)]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();
    anyhow::ensure!(
        args.rate_hz.is_finite()
            && (0.1..=240.).contains(&args.rate_hz)
            && args.cycles > 0
            && !args.robot_name.is_empty()
            && args
                .robot_name
                .bytes()
                .all(|c| c.is_ascii_alphanumeric() || c == b'_'),
        "Use an alphanumeric robot name, rate 0.1..240 and a positive cycle count"
    );
    let mut publisher = Publisher::new(PublisherOptions::default())?;
    let base_frame = format!("{}/base_link", args.robot_name);
    let mut camera = Transform::new(&base_frame, format!("{}/camera_link", args.robot_name));
    camera.translation = [0.3, 0., 0.5];
    publisher.publish(&[camera], true)?;
    let mut lidar = Transform::new(&base_frame, format!("{}/laser_link", args.robot_name));
    lidar.translation = [0., 0., 0.4];
    publisher.publish(&[lidar], true)?;
    let start = Instant::now();
    let stop = async {
        #[cfg(unix)]
        {
            let mut terminate =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
            tokio::select! { result = tokio::signal::ctrl_c() => result?, _ = terminate.recv() => {} }
        }
        #[cfg(not(unix))]
        tokio::signal::ctrl_c().await?;
        Ok::<_, std::io::Error>(())
    };
    tokio::pin!(stop);
    let mut published = 0;
    for idx in 0..args.cycles {
        let angle = f64::from(idx) / args.rate_hz * 0.3;
        let mut base = Transform::new("map", &base_frame);
        base.translation = [angle.cos(), angle.sin(), 0.2];
        base.rotation = [0., 0., (angle / 2.).sin(), (angle / 2.).cos()];
        publisher.publish(&[base], false)?;
        published += 1;
        let deadline = start + Duration::from_secs_f64(f64::from(idx + 1) / args.rate_hz);
        tokio::select! {
            result = &mut stop => { result?; break; }
            _ = tokio::time::sleep_until(deadline.into()) => {}
        }
    }
    println!(
        "Published {published} TF updates for {} on /tf with two retained frames on /tf_static",
        args.robot_name
    );
    Ok(())
}
