use horus::mapping::{MeshSourceAdapter, MeshSourceAdapterOptions};
fn main() -> anyhow::Result<()> {
    let args: Vec<_> = std::env::args().collect();
    anyhow::ensure!(
        args.len() >= 4,
        "input_type input_topic output_topic required"
    );
    let mut o = MeshSourceAdapterOptions::new(&args[2], &args[3]);
    o.input_type = args[1].clone();
    o.triangles_per_page = 1;
    o.blocks_per_axis = 1;
    let adapter = MeshSourceAdapter::new(o)?;
    std::thread::sleep(std::time::Duration::from_secs(12));
    println!("{}", adapter.status()?);
    Ok(())
}
