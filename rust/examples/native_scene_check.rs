use horus::remote_rendering::scene::{SceneAsset, SceneLoadOptions};
fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("usage: native_scene_check SCENE"))?;
    let s = SceneAsset::load(
        path,
        SceneLoadOptions {
            bake_lighting: false,
            ..Default::default()
        },
    )?;
    println!(
        "{}",
        serde_json::json!({"vertices":s.vertices(),"faces":s.faces(),"point_colors":s.point_colors(),
        "face_colors":s.face_colors(),"uvs":s.uvs(),"texture_rects":s.texture_rects(),
        "atlas":s.texture_atlas(),"width":s.texture_width(),"height":s.texture_height()})
    );
    Ok(())
}
