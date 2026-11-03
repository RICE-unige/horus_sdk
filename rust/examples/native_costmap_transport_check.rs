use horus::mapping::{CostmapAdapter, CostmapAdapterOptions};
fn main() -> anyhow::Result<()> {
    let mut options =
        CostmapAdapterOptions::new("/horus_native_costmap/full", "/horus_native_costmap/output");
    if std::env::args().nth(1).as_deref() == Some("raw") {
        options.input_format = "raw".into();
    }
    options.updates_topic = "/horus_native_costmap/updates".into();
    options.snapshot_service = "/horus_native_costmap/snapshot".into();
    options.publish_hz = 20.0;
    options.resync_seconds = 0.5;
    let adapter = CostmapAdapter::new(options)?;
    for _ in 0..160 {
        let status = adapter.status()?;
        anyhow::ensure!(status["running"] == true, "{status}");
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    println!("{}", adapter.status()?);
    Ok(())
}
