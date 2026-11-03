use horus::{
    collision_risk::*,
    experiments::{manifest::*, workloads::*, CsvMetricWriter},
};
use serde_json::json;
#[test]
fn helpers() -> anyhow::Result<()> {
    assert_eq!(
        default_run_id(
            "E0 Baseline",
            "one / two",
            2,
            Some(chrono::DateTime::from_timestamp(0, 0).unwrap())
        ),
        "e0_baseline_one_two_r02_19700101_000000"
    );
    let w = workload_from_dict(
        &json!({"robot_count":3,"camera":{"streams":2,"fps":30},"pointcloud":{"points":10,"hz":2},"map":{"representation":"mesh","hz":1},"seed":42,"extra":{"test":true}}),
    )?;
    assert_eq!(w.stream_count(), 8);
    assert_eq!(w.extra["seed"], 42);
    assert_eq!(w.extra["test"], true);
    assert_eq!(workload_from_dict(&w.to_dict()?)?.to_dict()?, w.to_dict()?);
    let dir =
        std::env::temp_dir().join(format!("horus-native-experiment-{}", uuid::Uuid::new_v4()));
    let m = ExperimentManifest::from_identity(
        &RunIdentity {
            run_id: "run".into(),
            experiment: "E0".into(),
            condition: "base".into(),
            repetition: 2,
        },
        json!({"notes":"test"}),
    )?;
    assert_eq!(
        serde_json::from_slice::<serde_json::Value>(&std::fs::read(m.save(&dir)?)?)?,
        m.to_dict()
    );
    let path = dir.join("metric.csv");
    let mut writer =
        CsvMetricWriter::create(&path, "run,one", "E0", "a\nb", &["fps", "note", "fps"])?;
    writer.write(
        json!({"fps":30,"note":"say \"hi\"\r"}).as_object().unwrap(),
        Some(0),
    )?;
    writer.flush()?;
    drop(writer);
    assert_eq!(std::fs::read_to_string(path)?,"timestamp_ns,run_id,experiment,condition,fps,note\n0,\"run,one\",E0,\"a\nb\",30,\"say \"\"hi\"\"\r\"\n");
    let s = analyze_laser_scan(&[f64::NAN, 2., 0.5, f64::INFINITY], 0., 0.5, 0., 10.)?;
    assert_eq!(s.min_distance_m, Some(0.5));
    assert!((s.direction[0] - 1f64.cos()).abs() < 1e-9);
    let c = analyze_point_cloud(&[[0.1, 0., 2.], [0.3, 0.4, 0.], [f64::NAN, 0., 0.]], 0.35)?;
    assert_eq!(c.min_distance_m, Some(0.5));
    assert!((c.direction[0] - 0.6).abs() < 1e-9);
    assert_eq!(risk_from_distance(Some(0.5), 1.)?, 0.5);
    std::fs::remove_dir_all(dir)?;
    Ok(())
}
