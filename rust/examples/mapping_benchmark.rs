use horus::mapping::{
    encode_live_payload, BlockGeometry, CostmapState, GridGeometry, MeshOperation,
};
use std::{hint::black_box, time::Instant};

fn main() {
    let mut map = CostmapState::new();
    map.full(
        GridGeometry {
            frame: "map".into(),
            width: 1024,
            height: 1024,
            resolution: 0.05,
            origin: [0., 0., 0., 0., 0., 0., 1.],
        },
        vec![0; 1024 * 1024],
        0,
    )
    .unwrap();
    let patch: Vec<u8> = (0..128 * 128).map(|i| (i % 256) as u8).collect();
    let geometry = BlockGeometry {
        vertices: (0..30000)
            .map(|i| [(i % 256) as f32 / 16., (i / 256) as f32 / 16., 1.])
            .collect(),
        colors: vec![[255, 128, 0, 255]; 30000],
        triangles: (0..30000).collect(),
    };
    let mut stamp = 0;
    for round in 0..6 {
        let start = Instant::now();
        for _ in 0..1000 {
            stamp += 1;
            map.patch_raw("map", [64, 64], [128, 128], black_box(&patch), stamp)
                .unwrap();
        }
        black_box(map.data());
        let patch_us = start.elapsed().as_secs_f64() * 1e6 / 1000.;
        let start = Instant::now();
        let mut encoded = Vec::new();
        for _ in 0..100 {
            encoded = encode_live_payload(
                7,
                [0; 3],
                1,
                1,
                MeshOperation::Upsert,
                Some(black_box(&geometry)),
                1048576,
            )
            .unwrap();
            black_box(&encoded);
        }
        let encode_us = start.elapsed().as_secs_f64() * 1e6 / 100.;
        if round > 0 {
            println!(
                "{}",
                serde_json::json!({"language":"rust","round":round,
            "patch_us":patch_us,"encode_us":encode_us,"wire_bytes":encoded.len(),
            "checksum":encoded.iter().map(|v|u64::from(*v)).sum::<u64>()})
            );
        }
    }
}
