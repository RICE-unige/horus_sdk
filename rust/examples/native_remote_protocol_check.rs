use anyhow::{ensure, Result};
use horus::remote_rendering::{pack_ros_diagnostic_frame, RenderedFrameMetadata};
fn main() -> Result<()> {
    let args: Vec<_> = std::env::args().collect();
    ensure!(args.len() == 2, "Pass a protocol fixture JSON file");
    let fixture: serde_json::Value = serde_json::from_slice(&std::fs::read(&args[1])?)?;
    let metadata = RenderedFrameMetadata::from_value(&fixture["metadata"])?;
    let hex = fixture["image_hex"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("Missing image"))?;
    let mut image = Vec::new();
    for p in hex.as_bytes().chunks_exact(2) {
        image.push(u8::from_str_radix(std::str::from_utf8(p)?, 16)?);
    }
    let bytes = pack_ros_diagnostic_frame(&metadata, &image)?;
    for b in bytes {
        print!("{b:02x}");
    }
    println!();
    Ok(())
}
