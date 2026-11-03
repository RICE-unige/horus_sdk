use anyhow::Result;
use horus::mapping::*;
fn tf(frame: &str, _stamp: i64) -> Result<MeshPose> {
    assert_eq!(frame, "sensor");
    Ok(MeshPose {
        translation: [10., 0., 1.],
        ..Default::default()
    })
}
#[test]
fn source_batches_transform_preserve_and_expire() -> Result<()> {
    let mut state = MeshSourceState::new(MeshSourceOptions {
        map: LiveMeshOptions {
            initial_epoch: Some(42),
            ..Default::default()
        },
        triangles_per_page: 1,
        ..Default::default()
    })?;
    let mut marker = TriangleMarker {
        name_space: "room".into(),
        frame: "sensor".into(),
        id: 2,
        pose: MeshPose {
            translation: [0., 2., 0.],
            ..Default::default()
        },
        scale: [2., 1., 1.],
        color: [10, 20, 30, 255],
        points: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
        ..Default::default()
    };
    let updates = state.apply_markers(&[marker.clone()], 0., tf)?;
    assert_eq!(updates.len(), 1);
    assert_eq!(updates[0].vertex_count, 3);
    // Interleaved vertex/color output, with scale -> local pose -> map transform.
    let bytes = &updates[0].payload;
    assert_eq!(f32::from_le_bytes(bytes[52..56].try_into().unwrap()), 10.);
    assert_eq!(f32::from_le_bytes(bytes[56..60].try_into().unwrap()), 2.);
    assert_eq!(f32::from_le_bytes(bytes[60..64].try_into().unwrap()), 1.);
    assert_eq!(&bytes[64..68], &[10, 20, 30, 255]);
    assert!(state.apply_markers(&[marker.clone()], 0., tf)?.is_empty());
    let snapshot = state.snapshot_updates()?[0].payload.clone();
    let mut invalid = marker.clone();
    invalid.id = 3;
    invalid.points[0][0] = f64::NAN;
    marker.points[0][0] = 5.;
    assert!(state
        .apply_markers(&[marker.clone(), invalid], 0., tf)
        .is_err());
    assert_eq!(state.snapshot_updates()?[0].payload, snapshot);
    marker.points[0][0] = 0.;
    let clear = TriangleMarker {
        action: 3,
        ..Default::default()
    };
    assert!(state
        .apply_markers(&[clear, marker.clone()], 0., tf)?
        .is_empty());
    marker.lifetime_seconds = 1.;
    state.apply_markers(&[marker], 0., tf)?;
    assert!(state.expire(0.5)?.is_empty());
    assert_eq!(state.expire(1.)?[0].operation, MeshOperation::Delete);
    assert_eq!(state.status()?["page_count"], 0);
    Ok(())
}
#[test]
fn nvblox_partial_deletion_and_clear() -> Result<()> {
    let mut state = MeshSourceState::new(MeshSourceOptions::default())?;
    let geometry = BlockGeometry {
        vertices: vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]],
        triangles: vec![0, 1, 2],
        colors: vec![[10, 20, 30, 255]; 3],
    };
    let blocks = [
        NvbloxBlockUpdate {
            index: [-1, 0, 0],
            geometry: Some(&geometry),
        },
        NvbloxBlockUpdate {
            index: [0, 0, 0],
            geometry: Some(&geometry),
        },
    ];
    assert_eq!(state.apply_blocks(&blocks, false, "map", 0, tf)?.len(), 2);
    assert!(state.apply_blocks(&blocks, false, "map", 0, tf)?.is_empty());
    let deletion = NvbloxBlockUpdate {
        index: [0, 0, 0],
        geometry: None,
    };
    assert_eq!(
        state.apply_blocks(&[deletion], false, "map", 0, tf)?.len(),
        1
    );
    assert_eq!(state.status()?["page_count"], 1);
    assert!(state.apply_blocks(&blocks, false, "sensor", 0, tf).is_err());
    assert_eq!(
        state.apply_blocks(&[], true, "map", 0, tf)?[0].operation,
        MeshOperation::Clear
    );
    assert_eq!(state.status()?["source_bytes"], 0);
    Ok(())
}
