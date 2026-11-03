use horus::remote_rendering::encoder::{AsyncH264Encoder, H264Encoder, H264EncoderOptions};
#[test]
fn invalid_encoder_config_and_missing_feature_fail_explicitly() {
    for width in [0, 383, 8193] {
        assert!(H264Encoder::new(H264EncoderOptions {
            width,
            ..Default::default()
        })
        .is_err());
        assert!(AsyncH264Encoder::new(H264EncoderOptions {
            width,
            ..Default::default()
        }).is_err());
    }
    if !H264Encoder::compiled() {
        match AsyncH264Encoder::new(H264EncoderOptions::default()) {
            Err(e) => assert!(e.to_string().contains("not built")),
            Ok(_) => panic!("Async NVENC silently enabled without build feature"),
        }
        match H264Encoder::new(H264EncoderOptions::default()) {
            Err(e) => assert!(e.to_string().contains("not built")),
            Ok(_) => panic!("NVENC silently enabled without build feature"),
        }
    }
}
