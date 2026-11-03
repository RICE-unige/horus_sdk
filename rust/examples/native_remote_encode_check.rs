use horus::remote_rendering::{
    encoder::{AsyncH264Encoder, EncodedH264Frame, H264Encoder, H264EncoderOptions},
    gpu::GpuRenderOptions,
    stereo::{assemble_stereo_frame, StereoFrameContext},
};
use std::io::Write;
fn main() -> anyhow::Result<()> {
    let path = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("Expected output H.264 path"))?;
    let mode = std::env::args().nth(2).unwrap_or("sync".into());
    let burst = mode == "burst";
    let settings = H264EncoderOptions {
        width: 384,
        height: 64,
        ..Default::default()
    };
    let encoder = if mode == "sync" {
        Some(H264Encoder::new(settings)?)
    } else {
        None
    };
    let background = if mode != "sync" {
        Some(AsyncH264Encoder::new(settings)?)
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
    let mut file = std::fs::File::create(path)?;
    let mut packets = Vec::new();
    let mut save = |encoded: Vec<EncodedH264Frame>| -> anyhow::Result<()> {
        for f in encoded {
            file.write_all(&f.data)?;
            packets.push(serde_json::json!({"frame_index":f.frame_index,"source_sequence":f.source_sequence,
                "keyframe":f.keyframe,"metadata":serde_json::from_str::<serde_json::Value>(&f.metadata_json)?,"bytes":f.data.len()}));
        }
        Ok(())
    };
    let count = if burst { 200 } else { 12 };
    for i in 0..count {
        let mut rgb = vec![0; 192 * 48 * 2 * 3];
        for p in 0..192 * 48 * 2 {
            rgb[3 * p] = if p < 192 * 48 { 200 } else { 40 };
            rgb[3 * p + 1] = 30 + (i % 12) as u8 * 5;
            rgb[3 * p + 2] = if p < 192 * 48 { 40 } else { 200 };
        }
        let context = StereoFrameContext {
            codec: "h264".into(),
            sequence: 100 + i,
            marker_height: 16,
            workspace_revision: if i < 5 { 0 } else { 1 },
            ..Default::default()
        };
        let frame = assemble_stereo_frame(&rgb, &vec![2.; 192 * 48 * 2], options, &context)?;
        if i == 7 {
            if let Some(e) = &encoder {
                e.request_keyframe();
            }
            if let Some(e) = &background {
                e.request_keyframe();
            }
        }
        if let Some(e) = &background {
            e.submit_rgb(frame.video_rgb(), &frame.metadata().to_json()?, 100 + i)?;
            if !burst {
                let encoded = e.poll_encoded(1, 5000)?;
                anyhow::ensure!(encoded.len() == 1, "Async encoder timed out");
                save(encoded)?;
            } else if i < 10 {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        } else {
            let encoded = encoder.as_ref().unwrap().encode_rgb(
                frame.video_rgb(),
                &frame.metadata().to_json()?,
                100 + i,
            )?;
            anyhow::ensure!(encoded.len() == 1, "Unexpected encode delay");
            save(encoded)?;
        }
    }
    if let Some(e) = &background {
        save(e.close()?)?;
        anyhow::ensure!(
            e.close()?.is_empty() && e.poll_encoded(1, 0)?.is_empty(),
            "Async close not idempotent"
        );
        anyhow::ensure!(e.poll_encoded(0, 0).is_err(), "Invalid poll accepted");
        anyhow::ensure!(
            packets.len() as u64 + e.replaced_count() == count,
            "Frame accounting mismatch"
        );
        if burst {
            anyhow::ensure!(
                e.replaced_count() > 0 && packets.last().unwrap()["source_sequence"] == 299,
                "Backpressure lost newest input"
            );
        }
    } else {
        save(encoder.as_ref().unwrap().close()?)?;
    }
    anyhow::ensure!(packets[0]["keyframe"] == true, "No initial IDR");
    if !burst {
        anyhow::ensure!(
            packets.len() == 12 && packets[5]["keyframe"] == true && packets[7]["keyframe"] == true,
            "IDR recovery failed"
        );
    }
    println!("{}", serde_json::to_string(&packets)?);
    Ok(())
}
