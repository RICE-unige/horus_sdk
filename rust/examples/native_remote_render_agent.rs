use horus::remote_rendering::{
    encoder::{AsyncH264Encoder, H264EncoderOptions},
    gpu::{CudaRenderer, GpuRenderOptions},
    gaussian::{GaussianScene, GaussianLoadOptions, GaussianRenderer},
    monotonic_time_ns,
    publisher::{RemotePublisher, RemotePublisherOptions},
    scene::{SceneAsset, SceneLoadOptions},
    stereo::{assemble_stereo_frame, StereoFrameContext},
    video::GpuVideoEncoder,
};
use serde::Deserialize;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};

#[derive(Deserialize)]
#[serde(default)]
struct Config {
    scene_kind: String,
    canonicalize_y_down: bool,
    mode: String,
    session_id: String,
    view_topic: String,
    stream_topic: String,
    status_topic: String,
    maximum_view_age_seconds: f64,
    width: u32,
    height: u32,
    near_m: f32,
    far_m: f32,
    point_radius: i32,
    fps: u32,
    depth_strip_height: u32,
    duration_seconds: f64,
    world_scale: f64,
    bake_lighting: bool,
    bitrate_kbps: u32,
    video_library: String,
}
impl Default for Config {
    fn default() -> Self {
        let p = RemotePublisherOptions::default();
        Self {
            scene_kind: "geometry".into(),
            canonicalize_y_down: false,
            mode: p.mode,
            session_id: p.session_id,
            view_topic: p.view_topic,
            stream_topic: p.stream_topic,
            status_topic: p.status_topic,
            maximum_view_age_seconds: p.maximum_view_age_seconds,
            width: 640,
            height: 480,
            near_m: 0.05,
            far_m: 100.,
            point_radius: 1,
            fps: 30,
            depth_strip_height: 120,
            duration_seconds: 0.,
            world_scale: 1.,
            bake_lighting: true,
            bitrate_kbps: 12000,
            video_library: String::new(),
        }
    }
}
#[tokio::main(worker_threads = 2)]
async fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args().collect();
    anyhow::ensure!(
        args.len() == 4,
        "Usage: native_remote_render_agent SCENE CUDA_LIBRARY CONFIG_JSON"
    );
    let c: Config = serde_json::from_slice(&std::fs::read(&args[3])?)?;
    anyhow::ensure!(
        matches!(c.mode.as_str(), "diagnostic" | "rgb" | "h264"),
        "Invalid mode"
    );
    let video_height = c.height as u64
        + if c.mode == "diagnostic" {
            0
        } else {
            c.depth_strip_height as u64 + 16
        };
    anyhow::ensure!(
        c.near_m.is_finite()
            && c.far_m.is_finite()
            && c.near_m > 0.0
            && c.far_m > c.near_m
            && (0..=8).contains(&c.point_radius)
            && c.width as u64 * 2 * video_height
                <= horus::remote_rendering::MAXIMUM_IMAGE_BYTES as u64 / 3,
        "Invalid depth/point settings or frame exceeds publication budget"
    );
    anyhow::ensure!(
        (1..=120).contains(&c.fps)
            && c.duration_seconds.is_finite()
            && (0.0..=86400.0).contains(&c.duration_seconds)
            && (192..=4096).contains(&c.width)
            && c.width % 4 == 0
            && (16..=4096).contains(&c.height)
            && c.height % 2 == 0
            && c.depth_strip_height <= 4096
            && c.depth_strip_height % 2 == 0,
        "Invalid render dimensions, frame rate or duration"
    );
    let (renderer, gaussian, vertices, triangles, gaussians) = if c.scene_kind == "gaussian" {
        let scene = GaussianScene::load(&args[1],GaussianLoadOptions{world_scale:c.world_scale,
            canonicalize_y_down:c.canonicalize_y_down,..Default::default()})?;
        (None,Some(GaussianRenderer::new(&args[2],&scene)?),0,0,scene.means().len())
    } else {
    anyhow::ensure!(c.scene_kind == "geometry", "scene_kind must be geometry or gaussian");
    let scene = SceneAsset::load(
        &args[1],
        SceneLoadOptions {
            world_scale: c.world_scale,
            bake_lighting: c.bake_lighting,
            ..Default::default()
        },
    )?;
    let vertices = scene.vertices().len();
    let triangles = scene.faces().len();
    let renderer = if scene.faces().is_empty() {
        let renderer = CudaRenderer::points(&args[2], vertices)?;
        renderer.upload(scene.vertices(), scene.point_colors(), 0)?;
        renderer
    } else {
        CudaRenderer::mesh(&args[2], &scene.mesh_input()?)?
    };
    drop(scene);
    (Some(renderer),None,vertices,triangles,0)
    };
    anyhow::ensure!(c.video_library.is_empty() || c.mode == "h264", "video_library requires h264 mode");
    let device_encoder = if !c.video_library.is_empty() {
        Some(GpuVideoEncoder::new(&c.video_library,c.width,c.height,c.depth_strip_height,c.fps,c.bitrate_kbps)?)
    } else {None};
    let encoder = if c.mode == "h264" && device_encoder.is_none() {
        Some(AsyncH264Encoder::new(H264EncoderOptions {
            width: c.width * 2,
            height: c.height + c.depth_strip_height + 16,
            fps: c.fps,
            bitrate_kbps: c.bitrate_kbps,
            ..Default::default()
        })?)
    } else {
        None
    };
    let publisher = RemotePublisher::new(RemotePublisherOptions {
        mode: c.mode.clone(),
        session_id: c.session_id,
        view_topic: c.view_topic,
        stream_topic: c.stream_topic,
        status_topic: c.status_topic,
        maximum_view_age_seconds: c.maximum_view_age_seconds,
    })?;
    let stopping = Arc::new(AtomicBool::new(false));
    let stop = stopping.clone();
    let signal = tokio::spawn(async move {
        #[cfg(unix)]
        {
            let mut terminate =
                tokio::signal::unix::signal(tokio::signal::unix::SignalKind::terminate())?;
            tokio::select! {
                result = tokio::signal::ctrl_c() => {result?;},
                _ = terminate.recv() => {},
            }
        }
        #[cfg(not(unix))]
        tokio::signal::ctrl_c().await?;
        stop.store(true, Ordering::Release);
        Ok::<(), std::io::Error>(())
    });
    let options = GpuRenderOptions {
        width: c.width,
        height: c.height,
        near_m: c.near_m,
        far_m: c.far_m,
        point_radius: c.point_radius,
    };
    let host_pixels=if device_encoder.is_some() {0} else {c.width as usize*c.height as usize*2};
    let mut rgb = vec![0; host_pixels * 3];
    let mut depth = vec![0.; host_pixels];
    let start = Instant::now();
    let mut next_frame = start;
    let mut sequence = 0;
    let forward =
        |frames: Vec<horus::remote_rendering::encoder::EncodedH264Frame>| -> anyhow::Result<()> {
            for packet in frames {
                if !publisher.publish_h264(&packet)? {
                    if let Some(e) = &encoder {
                        e.request_keyframe();
                    }
                    if let Some(e) = &device_encoder {e.request_keyframe();}
                }
            }
            Ok(())
        };
    while !stopping.load(Ordering::Acquire)
        && (c.duration_seconds == 0. || start.elapsed().as_secs_f64() < c.duration_seconds)
    {
        if let Some(e) = &encoder {
            forward(e.poll_encoded(8, 0)?)?;
        }
        std::thread::sleep(next_frame.saturating_duration_since(Instant::now()));
        let Some(view) = publisher.next_view(50)? else {
            anyhow::ensure!(
                publisher.status()?["running"] == true,
                "Remote transport stopped"
            );
            continue;
        };
        sequence += 1;
        let mut context = StereoFrameContext::from_view(&view, sequence);
        let render_start = Instant::now();
        if let Some(e) = &device_encoder {
            if publisher.take_keyframe_request() {e.request_keyframe();}
            forward(if let Some(g)=&gaussian {e.encode_gaussian(g,&view,sequence,options)?}
                else {e.encode_cuda(renderer.as_ref().unwrap(),&view,sequence,options)?})?;
            next_frame=render_start+Duration::from_nanos(1_000_000_000/c.fps as u64);
            continue;
        }
        context.render_start_ns = monotonic_time_ns();
        if let Some(renderer) = &gaussian {
            renderer.render_into(&[view.left_eye,view.right_eye], options, &mut rgb, &mut depth)?;
        } else {
        renderer.as_ref().unwrap().render_into(
            &[view.left_eye, view.right_eye],
            options,
            &mut rgb,
            &mut depth,
        )?;
        }
        context.render_complete_ns = monotonic_time_ns();
        context.capture_time_ns = context.render_complete_ns;
        context.codec = if c.mode == "diagnostic" {
            "png"
        } else {
            "h264"
        }
        .into();
        context.marker_height = if c.mode == "diagnostic" { 0 } else { 16 };
        context.depth_strip_height = if c.mode == "diagnostic" {
            0
        } else {
            c.depth_strip_height
        };
        let frame = assemble_stereo_frame(&rgb, &depth, options, &context)?;
        if let Some(e) = &encoder {
            if publisher.take_keyframe_request() {
                e.request_keyframe();
            }
            e.submit_rgb(frame.video_rgb(), &frame.metadata().to_json()?, sequence)?;
        } else if c.mode == "rgb" {
            publisher.publish_rgb(&frame)?;
        } else {
            publisher.publish_diagnostic(&frame)?;
        }
        next_frame = render_start + Duration::from_nanos(1_000_000_000 / c.fps as u64);
    }
    if let Some(e) = &encoder {
        forward(e.close()?)?;
    }
    if let Some(e) = &device_encoder {forward(e.close()?)?;}
    publisher.stop();
    signal.abort();
    let mut status = publisher.status()?;
    status["scene_vertices"] = vertices.into();
    status["scene_triangles"] = triangles.into();
    status["scene_gaussians"] = gaussians.into();
    status["device_video"] = device_encoder.is_some().into();
    status["encoded_inputs_replaced"] = encoder.as_ref().map_or(0, |e| e.replaced_count()).into();
    println!("{status}");
    Ok(())
}
