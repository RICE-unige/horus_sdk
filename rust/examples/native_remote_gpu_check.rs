use anyhow::{ensure, Result};
use horus::remote_rendering::{gpu::*, EyeFov, EyeView, Quaternion, Vector3};
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(args.len() == 3, "Pass point and mesh CUDA library paths");
    let points = [[-0.5, 0.0, 2.0], [0.0, 0.0, 2.0], [0.5, 0.0, 2.0]];
    let colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]];
    let left = EyeView {
        position: Vector3 {
            x: -0.1,
            y: 0.0,
            z: 0.0,
        },
        rotation: Quaternion {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        },
        fov: EyeFov {
            left: -1.0,
            right: 1.0,
            up: 1.0,
            down: -1.0,
        },
    };
    let mut right = left;
    right.position.x = 0.1;
    let eyes = [left, right];
    let options = GpuRenderOptions {
        width: 32,
        height: 24,
        near_m: 0.1,
        far_m: 10.0,
        point_radius: 1,
    };
    let point = CudaRenderer::points(&args[1], points.len())?;
    point.upload(&points[..2], &colors[..2], 0)?;
    ensure!(
        point.render_views(&eyes, options).is_err(),
        "Partial upload rendered"
    );
    point.upload(&points[2..], &colors[2..], 2)?;
    let p = point.render_views(&eyes, options)?;
    let vertices = [
        [-1.0, -1.0, 2.0],
        [1.0, -1.0, 2.0],
        [1.0, 1.0, 2.0],
        [-1.0, 1.0, 2.0],
    ];
    let faces = [[0, 1, 2], [0, 2, 3]];
    let tints = [[255, 80, 20], [20, 80, 255]];
    let mesh = CudaRenderer::mesh(&args[2], &MeshInput::new(&vertices, &faces, &tints))?;
    let m = mesh.render_views(&eyes, options)?;
    let encode = |r: RenderedViews| serde_json::json!({"rgb":r.rgb,"depth":r.depth.iter().map(|x|if x.is_finite(){Some(*x)}else{None}).collect::<Vec<_>>(),"cuda_ms":r.cuda_ms});
    println!(
        "{}",
        serde_json::json!({"points":encode(p),"mesh":encode(m)})
    );
    Ok(())
}
