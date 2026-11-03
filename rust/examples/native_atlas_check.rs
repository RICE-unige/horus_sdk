use horus::remote_rendering::atlas::*;
use serde_json::{json, Value};
fn main() -> anyhow::Result<()> {
    let j: Value = serde_json::from_reader(std::io::stdin())?;
    let points: Vec<[f32; 3]> = serde_json::from_value(j["points"].clone())?;
    let colors: Vec<[u8; 3]> = serde_json::from_value(j["colors"].clone())?;
    let views: Vec<RemoteRenderViewSpec> = serde_json::from_value(j["views"].clone())?;
    let o: AtlasOptions = serde_json::from_value(j["options"].clone())?;
    let a = MultiviewAtlas::from_points(&points, &colors, &views, &o)?;
    let s = a.unproject(&views, 12, 8, o.world_scale, o.vertical_fov_degrees)?;
    let depth: Vec<Option<f32>> = a
        .depth()
        .iter()
        .map(|&v| v.is_finite().then_some(v))
        .collect();
    println!(
        "{}",
        json!({"rgb":a.rgb(),"depth":depth,"points":s.vertices(),"colors":s.point_colors(),"packed":a.pack_rgbd((o.near_base*o.world_scale) as f32,(o.far_base*o.world_scale) as f32)?})
    );
    Ok(())
}
