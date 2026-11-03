use horus::remote_rendering::{
    gaussian::{GaussianLoadOptions, GaussianRenderer, GaussianScene},
    gpu::{CudaRenderer, GpuRenderOptions},
    scene::{SceneAsset, SceneLoadOptions},
    stereo::{assemble_stereo_frame, StereoFrameContext},
    video::GpuVideoEncoder,
    ViewState,
};
use std::io::Write;
fn main() -> anyhow::Result<()> {
    let a: Vec<_> = std::env::args().collect();
    anyhow::ensure!(a.len()==7,"Usage: native_device_video_check SCENE KIND RENDER_PLUGIN VIDEO_PLUGIN VIEW_JSON OUTPUT_PREFIX");
    let mut view = ViewState::from_json(&std::fs::read_to_string(&a[5])?)?;
    let (geometry, gaussian) = if a[2] == "gaussian" {
        let scene = GaussianScene::load(&a[1], GaussianLoadOptions::default())?;
        (None, Some(GaussianRenderer::new(&a[3], &scene)?))
    } else {
        let scene = SceneAsset::load(
            &a[1],
            SceneLoadOptions {
                bake_lighting: false,
                ..Default::default()
            },
        )?;
        let renderer = if scene.faces().is_empty() {
            let r = CudaRenderer::points(&a[3], scene.vertices().len())?;
            r.upload(scene.vertices(), scene.point_colors(), 0)?;
            r
        } else {
            CudaRenderer::mesh(&a[3], &scene.mesh_input()?)?
        };
        (Some(renderer), None)
    };
    let o = GpuRenderOptions {
        width: 256,
        height: 192,
        near_m: 0.05,
        far_m: 100.,
        point_radius: 2,
    };
    let encoder = GpuVideoEncoder::new(&a[4], o.width, o.height, 48, 30, 64000)?;
    let mut output = std::fs::File::create(format!("{}.h264", a[6]))?;
    let mut packets = Vec::new();
    let mut save =
        |frames: Vec<horus::remote_rendering::encoder::EncodedH264Frame>| -> anyhow::Result<()> {
            for f in frames {
                output.write_all(&f.data)?;
                packets.push(serde_json::json!({"frame_index":f.frame_index,"source_sequence":f.source_sequence,
                "keyframe":f.keyframe,"metadata":serde_json::from_str::<serde_json::Value>(&f.metadata_json)?}));
            }
            Ok(())
        };
    for i in 0..12 {
        view.workspace.revision = if i < 5 { 0 } else { 1 };
        view.view_sequence = i + 1;
        if i == 0 {
            let r = if let Some(g) = &gaussian {
                g.render_views(&[view.left_eye, view.right_eye], o)?
            } else {
                geometry
                    .as_ref()
                    .unwrap()
                    .render_views(&[view.left_eye, view.right_eye], o)?
            };
            let mut c = StereoFrameContext::from_view(&view, 100 + i);
            c.codec = "h264".into();
            c.marker_height = 16;
            c.depth_strip_height = 48;
            let reference = assemble_stereo_frame(&r.rgb, &r.depth, o, &c)?;
            std::fs::write(format!("{}.rgb", a[6]), reference.video_rgb())?;
        }
        if i == 7 {
            encoder.request_keyframe();
        }
        save(if let Some(g) = &gaussian {
            encoder.encode_gaussian(g, &view, 100 + i, o)?
        } else {
            encoder.encode_cuda(geometry.as_ref().unwrap(), &view, 100 + i, o)?
        })?;
    }
    save(encoder.close()?)?;
    anyhow::ensure!(
        encoder.close()?.is_empty() && packets.len() == 12,
        "Incomplete device video"
    );
    println!("{}", serde_json::to_string(&packets)?);
    Ok(())
}
