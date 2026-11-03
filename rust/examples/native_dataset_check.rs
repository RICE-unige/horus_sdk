use horus::remote_rendering::datasets::*;
fn main() -> anyhow::Result<()> {
    let a: Vec<_> = std::env::args().collect();
    anyhow::ensure!(
        a.len() == 4,
        "Usage: native_dataset_check MODE SOURCE OUTPUT_LOD"
    );
    let mut out = serde_json::json!({});
    let mut points = Vec::new();
    let mut colors = Vec::new();
    let mut append = |s: horus::remote_rendering::scene::SceneAsset| {
        points.extend_from_slice(s.vertices());
        colors.extend_from_slice(s.point_colors());
    };
    if a[1] == "cow" {
        append(prepare_cow_lady_scene(&a[2], 1.7)?);
    } else {
        let scene = if a[1] == "eth3d" {
            Some(prepare_eth3d_scene("electro", &a[2], 1.7, "")?)
        } else {
            None
        };
        if let Some(s) = &scene {
            out["scene"] = serde_json::to_value(s)?;
        }
        let stream = if let Some(s) = &scene {
            PointChunkStream::from_scene(s, 17)?
        } else {
            PointChunkStream::open(&a[2], 17)?
        };
        for chunk in stream {
            append(chunk?);
        }
        let mut stream = if let Some(s) = &scene {
            PointChunkStream::from_scene(s, 13)?
        } else {
            PointChunkStream::open(&a[2], 13)?
        };
        let count = stream.write_lod(&a[3], 0.05, 20000000)?;
        out["lod_count"] = count.into();
        let mut n = 0;
        for chunk in PointChunkStream::open(&a[3], 7)? {
            n += chunk?.vertices().len();
        }
        anyhow::ensure!(n as u64 == count, "LOD roundtrip mismatch");
    }
    out["points"] = serde_json::to_value(points)?;
    out["colors"] = serde_json::to_value(colors)?;
    out["catalog"] = scene_catalog("/scene-cache")?;
    println!("{out}");
    Ok(())
}
