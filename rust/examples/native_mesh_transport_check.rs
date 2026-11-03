use horus::mapping::{BlockGeometry, LiveMeshPublisher, LiveMeshPublisherOptions};
use std::time::{Duration, Instant};

// Isolated ROS contract fixture, not a robot demo or registration launcher.
fn main() -> Result<(), String> {
    let topic = std::env::args()
        .nth(1)
        .ok_or("Usage: native_mesh_transport_check /absolute/map/topic")?;
    let mut options = LiveMeshPublisherOptions {
        topic,
        ..Default::default()
    };
    options.map.initial_epoch = Some(11);
    let mut publisher = LiveMeshPublisher::new(options)?;
    let mut triangle = BlockGeometry {
        vertices: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
        colors: vec![[255, 0, 0, 255], [0, 255, 0, 255], [0, 0, 255, 255]],
        triangles: vec![0, 1, 2],
    };
    if publisher.replace("triangle", Some(&triangle))? != 1
        || publisher.replace("triangle", Some(&triangle))? != 0
    {
        return Err("Mesh deduplication failed".into());
    }
    let start = Instant::now();
    let mut phase = 0;
    while start.elapsed().as_secs_f64() < 8.0 {
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed >= 4.0 && phase == 0 {
            triangle.colors[0] = [255, 255, 255, 255];
            publisher.replace("triangle", Some(&triangle))?;
            phase += 1;
        }
        if elapsed >= 5.0 && phase == 1 {
            publisher.replace("triangle", None)?;
            phase += 1;
        }
        if elapsed >= 6.0 && phase == 2 {
            publisher.clear()?;
            phase += 1;
        }
        publisher.pump()?;
        std::thread::sleep(Duration::from_millis(10));
    }
    if publisher.state().page_count() != 0
        || publisher.pending_count() != 0
        || publisher.dropped_repair_requests() != 0
    {
        return Err("Mesh lifecycle or repair overflow check failed".into());
    }
    println!("Native mesh lifecycle passed");
    Ok(())
}
