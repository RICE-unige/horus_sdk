#!/usr/bin/env python3
"""Generate HRRF/depth/marker compatibility cases from the operational Python API."""
import json
from pathlib import Path
import numpy as np
from horus.remote_rendering import frame_protocol as p
from horus.remote_rendering import dynamic_stream as dynamic
from horus.remote_rendering import synthetic_scene as s


def finite_list(values):
    return [float(x) if np.isfinite(x) else None for x in np.asarray(values).ravel()]


def main():
    depth = np.array([np.inf, .1, .5, 1., 2., 4., 8., 10., -1., 11., np.nan, -np.inf,
                      .2, .3, .4, .5, 1.1, 3.3, 5.5, 7.7, 9., 10., 0., np.inf], dtype=np.float32).reshape(2, 3, 4)
    positions = [[-.03, 1.5, .2], [.03, 1.5, .2]]
    rotations = [[0., 0., 0., 1.], [0., 0., 0., 1.]]
    projections = [[1.2, 1.4, .1, -.05], [1.2, 1.4, -.1, -.05]]
    common = dict(sequence=0x1234abcd, capture_time_ns=1780000000000000000,
                  color_width=640, color_height=480, near_m=.1, far_m=10.)
    cases = []
    for name in ("single", "stereo", "mono"):
        d = depth[:1] if name == "single" else (np.repeat(depth[:1], 2, axis=0) if name == "mono" else depth)
        if name == "single":
            wire = p.pack_remote_frame(d[0], projection=projections[0], position=positions[0],
                                       rotation=rotations[0], **common)
        else:
            wire = p.pack_stereo_remote_frame(d, projections=projections, positions=positions,
                                              rotations=rotations, mono_atlas=name == "mono", **common)
        cases.append(dict(name=name, depth=finite_list(d), width=4, height=3,
                          codes=np.concatenate([p.quantize_depth(x, near_m=.1, far_m=10.).ravel() for x in d]).tolist(),
                          wire_hex=wire.hex(), view_count=len(d), mono_atlas=name == "mono"))
    source = np.arange(48, dtype=np.float32).reshape(6, 8) / 10 + 1
    source[1, 2] = np.inf
    source[4, 5] = np.inf
    reductions = []
    for name, fn, size in (("minimum", p.downsample_depth_min, [4, 3]),
                           ("surfel", p.downsample_depth_for_surfels, [3, 2]),
                           ("reprojection", p.downsample_depth_for_reprojection, [3, 2])):
        reductions.append(dict(name=name, size=size, expected=finite_list(fn(source, *size))))
    rgb = np.full((20, 400, 3), 127, np.uint8)
    p.embed_frame_marker(rgb, common["sequence"])
    # Compact marker fixture: expected rows, all other rows retain the initial value.
    result = dict(common=common, positions=positions, rotations=rotations, projections=projections,
                  cases=cases, source=finite_list(source), reductions=reductions,
                  marker_row_hex=rgb[0].tobytes().hex())
    result["luma"] = []
    for name, encoder, decoder in (("static8", s.encode_luma_depth, s.decode_luma_depth),
                                   ("dynamic16", dynamic.encode_dynamic_depth, dynamic.decode_dynamic_depth)):
        encoded = encoder(source, near_m=.1, far_m=10.)
        result["luma"].append(dict(name=name, encoded_hex=encoded.tobytes().hex(),
                                    decoded=finite_list(decoder(encoded, near_m=.1, far_m=10.))))
    pose = dynamic.DynamicCameraPose((1.2, -2.4, .7), (.1, .3, -.2, .9))
    atlas = np.full((32, 72, 3), 71, np.uint8)
    dynamic.encode_dynamic_pose_metadata(atlas, (pose,), sequence=0xabcd, position_range_m=40.)
    _, poses = dynamic.decode_dynamic_pose_metadata(atlas, position_range_m=40.)
    packed = dynamic.pack_dynamic_rgbd(np.full((32, 72, 3), 127, np.uint8),
                                np.full((32, 72), 1.5, np.float32), (pose,), sequence=0xabcd,
                                near_m=.1, far_m=10., position_range_m=40.)
    result["dynamic_pose"] = dict(position=pose.position, rotation=pose.rotation,
        decoded_position=poses[0].position, decoded_rotation=poses[0].rotation,
        metadata_hex=atlas.tobytes().hex(), packed_hex=packed.tobytes().hex())
    destination = Path(__file__).resolve().parents[2] / "contracts/fixtures/remote_frame.json"
    destination.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
