use horus::remote_rendering::{
    encoder::{H264Encoder, H264EncoderOptions},
    gpu::GpuRenderOptions,
    publisher::{RemotePublisher, RemotePublisherOptions},
    stereo::{assemble_stereo_frame, render_colored_points, StereoFrameContext},
};
use std::sync::Arc;
fn main() -> anyhow::Result<()> {
    let mode = std::env::args().nth(1).unwrap_or("diagnostic".into());
    let publisher = Arc::new(RemotePublisher::new(RemotePublisherOptions {
        mode: mode.clone(),
        view_topic: "/native_remote/view".into(),
        stream_topic: "/native_remote/image".into(),
        status_topic: "/native_remote/status".into(),
        maximum_view_age_seconds: 0.5,
        ..Default::default()
    })?);
    let encoder = if mode == "h264" {
        Some(H264Encoder::new(H264EncoderOptions {
            width: 384,
            height: 64,
            ..Default::default()
        })?)
    } else {
        None
    };
    let options = GpuRenderOptions {
        width: 192,
        height: 48,
        near_m: 0.1,
        far_m: 10.,
        point_radius: 0,
    };
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
    let mut sequence = 0;
    while std::time::Instant::now() < deadline {
        let Some(view) = publisher.next_view(100)? else {
            continue;
        };
        let rendered = render_colored_points(
            &[[-0.5, 0., 2.], [0., 0., 2.], [0.5, 0., 2.]],
            &[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            &[view.left_eye, view.right_eye],
            options,
        )?;
        sequence += 1;
        let mut context = StereoFrameContext::from_view(&view, sequence);
        context.codec = if mode == "diagnostic" { "png" } else { "h264" }.into();
        context.marker_height = if mode == "diagnostic" { 0 } else { 16 };
        let frame = assemble_stereo_frame(&rendered.rgb, &rendered.depth, options, &context)?;
        match mode.as_str() {
            "diagnostic" => {
                if publisher.publish_diagnostic(&frame)? {
                    assert!(!publisher.publish_diagnostic(&frame)?);
                }
            }
            "rgb" => {
                if publisher.publish_rgb(&frame)? {
                    assert!(!publisher.publish_rgb(&frame)?);
                }
            }
            _ => {
                let encoder = encoder.as_ref().unwrap();
                if publisher.take_keyframe_request() {
                    encoder.request_keyframe();
                }
                for packet in
                    encoder.encode_rgb(frame.video_rgb(), &frame.metadata().to_json()?, sequence)?
                {
                    if publisher.publish_h264(&packet)? {
                        assert!(!publisher.publish_h264(&packet)?);
                    }
                }
            }
        }
    }
    if let Some(encoder) = encoder {
        for packet in encoder.close()? {
            publisher.publish_h264(&packet)?;
        }
    }
    let clone = Arc::clone(&publisher);
    let waiter = std::thread::spawn(move || while clone.next_view(60000).unwrap().is_some() {});
    publisher.stop();
    waiter.join().unwrap();
    publisher.stop();
    assert!(publisher.next_view(0)?.is_none());
    println!("{}", publisher.status()?);
    Ok(())
}
