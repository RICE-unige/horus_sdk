use anyhow::{ensure, Result};
use horus::remote_rendering::{
    gaussian::{GaussianLoadOptions, GaussianRenderer, GaussianScene},
    gpu::GpuRenderOptions,
    ViewState,
};
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(
        args.len() == 4 || args.len() == 6,
        "Usage: native_gaussian_check PLY OPTIONS_JSON OUTPUT_JSON [PLUGIN OUTPUT_PREFIX]"
    );
    let cfg: serde_json::Value = serde_json::from_slice(&std::fs::read(&args[2])?)?;
    let s = GaussianScene::load(
        &args[1],
        GaussianLoadOptions {
            world_scale: cfg["world_scale"].as_f64().unwrap_or(1.),
            canonicalize_y_down: cfg["canonicalize_y_down"].as_bool().unwrap_or(false),
            ..Default::default()
        },
    )?;
    let mut j = serde_json::to_value(s.metadata())?;
    j["means"] = serde_json::to_value(s.means())?;
    j["scales"] = serde_json::to_value(s.scales())?;
    j["quaternions"] = serde_json::to_value(s.quaternions())?;
    j["opacities"] = serde_json::to_value(s.opacities())?;
    j["spherical_harmonics"] = serde_json::to_value(s.spherical_harmonics())?;
    if args.len() == 6 {
        let view = ViewState::from_value(&cfg["view"])?;
        let renderer = GaussianRenderer::new(&args[4], &s)?;
        let r = renderer.render_views(
            &[view.left_eye, view.right_eye],
            GpuRenderOptions {
                width: cfg["width"].as_u64().unwrap_or(256) as u32,
                height: cfg["height"].as_u64().unwrap_or(192) as u32,
                ..Default::default()
            },
        )?;
        std::fs::write(format!("{}.rgb", args[5]), r.rgb)?;
        let bytes: Vec<_> = r.depth.iter().flat_map(|v| v.to_le_bytes()).collect();
        std::fs::write(format!("{}.depth", args[5]), bytes)?;
        j["elapsed_ms"] = serde_json::json!(r.cuda_ms);
    }
    std::fs::write(&args[3], serde_json::to_vec(&j)?)?;
    Ok(())
}
