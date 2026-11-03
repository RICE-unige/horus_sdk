use horus::remote_rendering::gpu::GpuRenderOptions;
use horus::remote_rendering::stereo::*;
use horus::remote_rendering::{unpack_ros_diagnostic_frame, EyeFov, EyeView, Quaternion, Vector3};
use serde_json::Value;
fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect()
}
fn depths(value: &Value) -> Vec<f32> {
    value
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().map_or(f32::INFINITY, |v| v as f32))
        .collect()
}
#[test]
fn stereo_pixels_metadata_and_cpu_projection_match_python() {
    let fixture: Value =
        serde_json::from_str(include_str!("../../contracts/fixtures/stereo_render.json")).unwrap();
    for test in fixture["cases"].as_array().unwrap() {
        let o = GpuRenderOptions {
            width: test["width"].as_u64().unwrap() as u32,
            height: test["height"].as_u64().unwrap() as u32,
            near_m: 0.25,
            far_m: 8.,
            point_radius: 0,
        };
        let mut c = StereoFrameContext {
            session_id: "native".into(),
            sequence: 42,
            source_view_sequence: 42,
            workspace_revision: 3,
            predicted_display_time_ns: 100,
            render_start_ns: 10,
            render_complete_ns: 20,
            capture_time_ns: 30,
            depth_strip_height: test["strip"].as_u64().unwrap() as u32,
            marker_height: test["marker"].as_u64().unwrap() as u32,
            codec: test["codec"].as_str().unwrap().into(),
            ..Default::default()
        };
        let rgb: Vec<u8> = serde_json::from_value(test["rgb"].clone()).unwrap();
        let d = depths(&test["depth"]);
        let frame = assemble_stereo_frame(&rgb, &d, o, &c).unwrap();
        assert_eq!(hex(frame.stereo_rgba()), test["rgba_hex"].as_str().unwrap());
        assert_eq!(hex(frame.video_rgb()), test["video_hex"].as_str().unwrap());
        assert_eq!(frame.metadata().to_value().unwrap(), test["metadata"]);
        assert!((frame.visibility_fraction() - test["visibility"].as_f64().unwrap()).abs() < 1e-12);
        if c.codec == "png" {
            let packet = frame.diagnostic_packet().unwrap();
            let (metadata, png) = unpack_ros_diagnostic_frame(&packet).unwrap();
            assert_eq!(metadata.to_value().unwrap(), test["metadata"]);
            assert_eq!(&png[..8], b"\x89PNG\r\n\x1a\n");
        }
        assert!(assemble_stereo_frame(&rgb[..rgb.len() - 1], &d, o, &c).is_err());
        c.capture_time_ns = 19;
        assert!(assemble_stereo_frame(&rgb, &d, o, &c).is_err());
    }
    let points: Vec<[f32; 3]> = serde_json::from_value(fixture["points"].clone()).unwrap();
    let colors: Vec<[u8; 3]> = serde_json::from_value(fixture["colors"].clone()).unwrap();
    let eyes: Vec<_> = [-0.1, 0.1]
        .into_iter()
        .map(|x| EyeView {
            position: Vector3 { x, y: 0., z: 0. },
            rotation: Quaternion {
                x: 0.,
                y: 0.,
                z: 0.,
                w: 1.,
            },
            fov: EyeFov {
                left: -4. / 3.,
                right: 4. / 3.,
                up: 1.,
                down: -1.,
            },
        })
        .collect();
    let frame = render_colored_points(
        &points,
        &colors,
        &eyes,
        GpuRenderOptions {
            width: 32,
            height: 24,
            near_m: 0.25,
            far_m: 8.,
            point_radius: 1,
        },
    )
    .unwrap();
    assert_eq!(hex(&frame.rgb), fixture["cpu_rgb_hex"].as_str().unwrap());
    assert_eq!(frame.depth, depths(&fixture["cpu_depth"]));
}
