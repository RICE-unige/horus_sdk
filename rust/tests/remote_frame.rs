use horus::remote_rendering::frame::*;
use serde_json::Value;
fn fixture() -> Value {
    serde_json::from_str(include_str!("../../contracts/fixtures/remote_frame.json")).unwrap()
}
fn values(j: &Value) -> Vec<f32> {
    j.as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().map(|f| f as f32).unwrap_or(f32::INFINITY))
        .collect()
}
fn unhex(s: &str) -> Vec<u8> {
    s.as_bytes()
        .chunks_exact(2)
        .map(|v| u8::from_str_radix(std::str::from_utf8(v).unwrap(), 16).unwrap())
        .collect()
}
#[test]
fn python_hrff_frames_are_byte_exact_and_corruption_is_rejected() {
    let f = fixture();
    let common = &f["common"];
    let mut info = DepthFrameInfo {
        capture_time_ns: common["capture_time_ns"].as_u64().unwrap(),
        sequence: common["sequence"].as_u64().unwrap() as u32,
        color_width: 640,
        color_height: 480,
        near_m: 0.1,
        far_m: 10.,
        ..Default::default()
    };
    for eye in 0..2 {
        info.eyes[eye] = FrameEye {
            projection: values(&f["projections"][eye]).try_into().unwrap(),
            position: values(&f["positions"][eye]).try_into().unwrap(),
            rotation: values(&f["rotations"][eye]).try_into().unwrap(),
        };
    }
    for case in f["cases"].as_array().unwrap() {
        info.width = 4;
        info.height = 3;
        info.view_count = case["view_count"].as_u64().unwrap() as u32;
        info.mono_atlas = case["mono_atlas"].as_bool().unwrap();
        let mut wire = unhex(case["wire_hex"].as_str().unwrap());
        let depth = values(&case["depth"]);
        assert_eq!(pack_depth_frame(&depth, &info).unwrap(), wire);
        let frame = unpack_depth_frame(&wire).unwrap();
        assert_eq!(frame.info.capture_time_ns, info.capture_time_ns);
        assert_eq!(frame.info.eyes[0], info.eyes[0]);
        let expected: Vec<u16> = serde_json::from_value(case["codes"].clone()).unwrap();
        assert_eq!(frame.depth_codes, expected);
        assert_eq!(quantize_depth(&depth, 0.1, 10.).unwrap(), expected);
        let decoded = dequantize_depth(&expected, 0.1, 10.).unwrap();
        for i in 0..depth.len() {
            if expected[i] == 0 {
                assert!(decoded[i].is_infinite());
            } else {
                assert!((decoded[i] - depth[i]).abs() < 0.0002);
            }
        }
        for i in 0..wire.len() {
            assert!(unpack_depth_frame(&wire[..i]).is_err());
        }
        *wire.last_mut().unwrap() ^= 1;
        assert!(unpack_depth_frame(&wire).is_err());
    }
    assert!(pack_depth_frame(&[1.], &info).is_err());
    let mut depth = values(&f["cases"][2]["depth"]);
    *depth.last_mut().unwrap() = 1.;
    assert!(pack_depth_frame(&depth, &info).is_err());
}
#[test]
fn python_depth_reductions_markers_and_borrowed_diagnostic_carrier() {
    let f = fixture();
    let source = values(&f["source"]);
    for (index, mode) in [
        DepthReduction::Minimum,
        DepthReduction::Surfel,
        DepthReduction::Reprojection,
    ]
    .into_iter()
    .enumerate()
    {
        let case = &f["reductions"][index];
        let size = [
            case["size"][0].as_u64().unwrap() as u32,
            case["size"][1].as_u64().unwrap() as u32,
        ];
        assert_eq!(
            reduce_depth(&source, [8, 6], size, mode, 0.08, 0.035).unwrap(),
            values(&case["expected"])
        );
    }
    assert!(reduce_depth(
        &source,
        [8, 6],
        [3, 2],
        DepthReduction::Minimum,
        0.08,
        0.035
    )
    .is_err());
    assert!(quantize_depth(&[1.], 1., 1.).is_err());
    let mut rgb = vec![127; 400 * 20 * 3];
    embed_frame_marker(&mut rgb, 400, 20, 0x1234abcd).unwrap();
    let row = unhex(f["marker_row_hex"].as_str().unwrap());
    for y in 0..20 {
        if y < 8 || y >= 12 {
            assert_eq!(&rgb[y * 1200..(y + 1) * 1200], &row);
        } else {
            assert!(rgb[y * 1200..(y + 1) * 1200].iter().all(|v| *v == 127));
        }
    }
    assert_eq!(decode_frame_marker(&rgb, 400, 20).unwrap(), 0xabcd);
    assert!(decode_frame_marker(&vec![0; rgb.len()], 400, 20).is_err());
    let carrier = pack_ros_debug_frame(&[0xff, 0xd8, 0xff, 0xd9], &[1, 2, 3]).unwrap();
    assert_eq!(
        unpack_ros_debug_frame(&carrier).unwrap(),
        (&[0xff, 0xd8, 0xff, 0xd9][..], &[1, 2, 3][..])
    );
    for i in 0..carrier.len() {
        assert!(unpack_ros_debug_frame(&carrier[..i]).is_err());
    }
}

#[test]
fn python_legacy_luma_dynamic_pose_and_combined_rgbd_contracts() {
    let f = fixture();
    let source = values(&f["source"]);
    for (i, format) in [LumaDepthFormat::Static8, LumaDepthFormat::Dynamic16]
        .into_iter()
        .enumerate()
    {
        let encoded = encode_luma_depth(&source, 8, 6, 0.1, 10., format).unwrap();
        assert_eq!(
            encoded,
            unhex(f["luma"][i]["encoded_hex"].as_str().unwrap())
        );
        let decoded = decode_luma_depth(&encoded, 8, 6, 0.1, 10., format).unwrap();
        let expected = values(&f["luma"][i]["decoded"]);
        assert!(decoded
            .iter()
            .zip(expected.iter())
            .all(|(a, b)| a == b || (a - b).abs() < 1e-6));
    }
    let p = &f["dynamic_pose"];
    let pose = DynamicCameraPose {
        position: values(&p["position"]).try_into().unwrap(),
        rotation: values(&p["rotation"]).try_into().unwrap(),
    };
    let mut atlas = vec![71; 72 * 32 * 3];
    assert_eq!(
        encode_dynamic_pose_metadata(&mut atlas, 72, 32, pose, 0xabcd, 40.).unwrap(),
        32
    );
    assert!(atlas == unhex(p["metadata_hex"].as_str().unwrap()));
    let (seq, decoded) = decode_dynamic_pose_metadata(&atlas, 72, 32, 40.).unwrap();
    assert_eq!(seq, 0xabcd);
    for i in 0..3 {
        assert!(
            (decoded.position[i] - p["decoded_position"][i].as_f64().unwrap() as f32).abs() < 1e-5
        );
    }
    for i in 0..4 {
        assert!(
            (decoded.rotation[i] - p["decoded_rotation"][i].as_f64().unwrap() as f32).abs() < 1e-6
        );
    }
    let packed = pack_dynamic_rgbd(
        &vec![127; 72 * 32 * 3],
        &vec![1.5; 72 * 32],
        [72, 32],
        pose,
        0xabcd,
        0.1,
        10.,
        40.,
    )
    .unwrap();
    assert!(packed == unhex(p["packed_hex"].as_str().unwrap()));
    assert!(metadata_rows(4).is_err());
    assert!(encode_dynamic_pose_metadata(&mut atlas, 72, 16, pose, 1, 40.).is_err());
    assert!(decode_luma_depth(&atlas, 72, 32, 0., 0., LumaDepthFormat::Dynamic16).is_err());
}
