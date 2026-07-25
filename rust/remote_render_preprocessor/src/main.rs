use std::collections::HashMap;
use std::env;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, ErrorKind, Read, Write};
use std::path::PathBuf;

const INPUT_RECORD_BYTES: usize = 15;
const OUTPUT_MAGIC: &[u8; 4] = b"HRL1";

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
struct VoxelKey {
    x: i32,
    y: i32,
    z: i32,
}

#[derive(Clone, Copy)]
struct Point {
    position: [f32; 3],
    color: [u8; 3],
}

fn parse_args() -> Result<(f32, PathBuf), String> {
    let mut args = env::args().skip(1);
    let voxel_size = args
        .next()
        .ok_or_else(|| "usage: horus-remote-render-preprocessor VOXEL_SIZE OUTPUT".to_owned())?
        .parse::<f32>()
        .map_err(|error| format!("invalid voxel size: {error}"))?;
    let output = PathBuf::from(
        args.next()
            .ok_or_else(|| "missing output path".to_owned())?,
    );
    if args.next().is_some() {
        return Err("unexpected extra arguments".to_owned());
    }
    if !voxel_size.is_finite() || voxel_size <= 0.0 {
        return Err("voxel size must be finite and positive".to_owned());
    }
    Ok((voxel_size, output))
}

fn read_f32(bytes: &[u8]) -> f32 {
    f32::from_le_bytes(bytes.try_into().expect("four-byte float"))
}

fn write_f32(writer: &mut impl Write, value: f32) -> io::Result<()> {
    writer.write_all(&value.to_le_bytes())
}

fn voxel_key(position: [f32; 3], inverse_voxel: f32) -> VoxelKey {
    VoxelKey {
        x: (position[0] * inverse_voxel).floor() as i32,
        y: (position[1] * inverse_voxel).floor() as i32,
        z: (position[2] * inverse_voxel).floor() as i32,
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let (voxel_size, output_path) =
        parse_args().map_err(|message| io::Error::new(ErrorKind::InvalidInput, message))?;
    let inverse_voxel = 1.0 / voxel_size;
    let mut input = BufReader::with_capacity(8 * 1024 * 1024, io::stdin().lock());
    let mut voxels: HashMap<VoxelKey, Point> = HashMap::new();
    let mut record = [0_u8; INPUT_RECORD_BYTES];
    let mut input_count = 0_u64;

    loop {
        let mut filled = 0;
        while filled < record.len() {
            match input.read(&mut record[filled..]) {
                Ok(0) if filled == 0 => break,
                Ok(0) => {
                    return Err(
                        io::Error::new(ErrorKind::UnexpectedEof, "truncated point record").into(),
                    );
                }
                Ok(count) => filled += count,
                Err(error) if error.kind() == ErrorKind::Interrupted => continue,
                Err(error) => return Err(error.into()),
            }
        }
        if filled == 0 {
            break;
        }
        let position = [
            read_f32(&record[0..4]),
            read_f32(&record[4..8]),
            read_f32(&record[8..12]),
        ];
        if !position.iter().all(|value| value.is_finite()) {
            continue;
        }
        let key = voxel_key(position, inverse_voxel);
        voxels.entry(key).or_insert(Point {
            position,
            color: [record[12], record[13], record[14]],
        });
        input_count += 1;
    }

    let mut ordered: Vec<(VoxelKey, Point)> = voxels.into_iter().collect();
    ordered.sort_unstable_by_key(|(key, _)| *key);
    let temporary_path = output_path.with_extension("hrl.part");
    let output_file = File::create(&temporary_path)?;
    let mut output = BufWriter::with_capacity(8 * 1024 * 1024, output_file);
    output.write_all(OUTPUT_MAGIC)?;
    output.write_all(&1_u32.to_le_bytes())?;
    write_f32(&mut output, voxel_size)?;
    output.write_all(&(ordered.len() as u64).to_le_bytes())?;
    for (_, point) in &ordered {
        for value in point.position {
            write_f32(&mut output, value)?;
        }
        output.write_all(&point.color)?;
    }
    output.flush()?;
    drop(output);
    std::fs::rename(&temporary_path, &output_path)?;
    eprintln!(
        "HORUS remote-render LOD: {input_count} input points -> {} voxels ({voxel_size:.6} m)",
        ordered.len()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voxel_keys_use_floor_on_both_sides_of_the_origin() {
        assert_eq!(
            voxel_key([0.0124, -0.0001, -0.0125], 80.0),
            VoxelKey { x: 0, y: -1, z: -1 }
        );
    }

    #[test]
    fn voxel_keys_are_stable_within_a_cell() {
        assert_eq!(
            voxel_key([1.001, 2.002, 3.003], 10.0),
            voxel_key([1.099, 2.099, 3.099], 10.0)
        );
    }
}
