use horus::bridge::RobotRegistryClient;
use horus::dataviz::{CostmapOptions, DataViz, RenderOptions};
use serde_json::{json, Value};

#[test]
fn full_costmap_contract_and_independent_layers() {
    let mut viz = DataViz::new("maps");
    viz.add_costmap(
        " /local_costmap/costmap ",
        " odom ",
        CostmapOptions::default(),
    )
    .unwrap();
    viz.add_costmap("/global_costmap/costmap", "map", CostmapOptions::default())
        .unwrap();
    let payload = RobotRegistryClient::new().build_global_visualizations_payload(&[viz]);
    assert_eq!(payload.len(), 2);
    let expected: Value =
        serde_json::from_str(include_str!("../../contracts/fixtures/costmap.json")).unwrap();
    assert_eq!(serde_json::to_value(&payload[0]).unwrap(), expected);
}

#[test]
fn costmap_options_rejected_before_mutation() {
    let mut viz = DataViz::new("maps");
    for alpha in [f64::NAN, f64::INFINITY, -1., 1.01] {
        assert!(viz
            .add_costmap(
                "/costmap",
                "map",
                CostmapOptions {
                    alpha,
                    ..Default::default()
                }
            )
            .is_err());
    }
    assert!(viz
        .add_costmap(" ", "map", CostmapOptions::default())
        .is_err());
    assert!(viz.visualizations.is_empty());
}

#[test]
fn map_transport_lane_is_explicit_and_invalid_values_are_omitted() {
    let client = RobotRegistryClient::new();
    for (input, expected) in [(" BULK_STRICT ", Some("bulk_strict")), ("unknown", None)] {
        let mut viz = DataViz::new("maps");
        viz.add_3d_mesh(
            "/mesh",
            "map",
            Some(RenderOptions::from([(
                "transport_lane".into(),
                json!(input),
            )])),
        );
        let payload = client.build_global_visualizations_payload(&[viz]);
        assert_eq!(payload[0].transport_lane.as_deref(), expected);
    }
}
