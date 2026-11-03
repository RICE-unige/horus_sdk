use horus::collision_risk::*;
fn main() -> anyhow::Result<()> {
    let source_type = std::env::args()
        .nth(1)
        .ok_or_else(|| anyhow::anyhow!("Expected source type"))?;
    let mut analyzer = CollisionRiskAnalyzer::new(CollisionRiskOptions {
        robot_name: "parity".into(),
        source_type,
        source_topic: "/parity/source".into(),
        output_topic: "/parity/risk".into(),
        smoothing_alpha: 1.,
        ..Default::default()
    })?;
    std::thread::sleep(std::time::Duration::from_secs(7));
    analyzer.stop();
    analyzer.stop();
    Ok(())
}
