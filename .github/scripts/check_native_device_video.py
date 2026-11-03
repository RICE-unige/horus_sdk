#!/usr/bin/env python3
"""Decode real CUDA/NVENC frames and compare against host-packed native output."""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from horus.remote_rendering.frame_protocol import decode_frame_marker


def fixtures(root):
    points = [(x / 10, y / 10, 2, 200, 60, 30)
              for x in range(-15, 16) for y in range(-12, 13)]
    common = "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\n"
    colors = "property uchar red\nproperty uchar green\nproperty uchar blue\n"
    (root / "points.ply").write_text(common.format(len(points)) + colors + "end_header\n" +
                                     "\n".join(" ".join(map(str, p)) for p in points) + "\n")
    (root / "mesh.ply").write_text(common.format(4) + colors +
        "element face 2\nproperty list uchar int vertex_indices\nend_header\n" +
        "-2 -2 2 200 60 30\n2 -2 2 200 60 30\n2 2 2 200 60 30\n-2 2 2 200 60 30\n3 0 1 2\n3 0 2 3\n")
    names = ["scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3",
             "opacity", "f_dc_0", "f_dc_1", "f_dc_2"]
    (root / "gaussian.ply").write_text(common.format(len(points)) +
        "".join(f"property float {n}\n" for n in names) + "end_header\n" +
        "\n".join(" ".join(map(str, p[:3])) + " -2.7 -2.7 -3 1 0 0 0 3 1 -1 -1.2"
                  for p in points) + "\n")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("binaries", nargs="+")
    p.add_argument("--output-dir", type=Path)
    for name in ["points", "mesh", "gaussian", "video"]:
        p.add_argument(f"--{name}-plugin", required=True)
    a = p.parse_args()
    view = json.loads((Path(__file__).parents[2] / "contracts/fixtures/remote_protocol.json").read_text())["view"]
    for eye, x in [("left_eye", -0.03), ("right_eye", 0.03)]:
        view[eye]["position"] = dict(x=x, y=0, z=0)
        view[eye]["rotation"] = dict(x=0, y=0, z=0, w=1)
    if a.output_dir:
        a.output_dir.mkdir(parents=True, exist_ok=True)
    with (nullcontext(a.output_dir) if a.output_dir else tempfile.TemporaryDirectory(prefix="horus-device-video-")) as d:
        root = Path(d)
        fixtures(root)
        config = root / "view.json"
        config.write_text(json.dumps(view))
        for binary in a.binaries:
            for kind in ["points", "mesh", "gaussian"]:
                prefix = root / "video"
                cmd = [binary, str(root / f"{kind}.ply"), kind, getattr(a, kind + "_plugin"),
                       a.video_plugin, str(config), str(prefix)]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
                assert result.returncode == 0, result.stderr
                packets = json.loads(result.stdout)
                assert len(packets) == 12
                path = str(prefix) + ".h264"
                probe = subprocess.run(["ffprobe", "-v", "error", "-show_streams", "-show_frames",
                                        "-of", "json", path], capture_output=True, check=True, timeout=30)
                info = json.loads(probe.stdout)
                stream = info["streams"][0]
                assert "Baseline" in stream["profile"] and stream["has_b_frames"] == 0
                assert stream["width"] == 512 and stream["height"] == 256
                frames = info["frames"]
                assert len(frames) == 12 and all(frames[i]["key_frame"] for i in [0, 5, 7])
                raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-f", "rawvideo",
                                      "-pix_fmt", "rgb24", "pipe:1"], capture_output=True, check=True, timeout=30)
                pixels = np.frombuffer(raw.stdout, np.uint8).reshape(12, 256, 512, 3)
                reference = np.fromfile(str(prefix) + ".rgb", np.uint8).reshape(256, 512, 3)
                assert np.count_nonzero(reference[:192]) > 1000, "Empty reference"
                errors = []
                for i, (packet, image) in enumerate(zip(packets, pixels)):
                    assert packet["frame_index"] == i and packet["source_sequence"] == 100 + i
                    assert packet["metadata"]["frame_id"] == 100 + i
                    assert decode_frame_marker(image[-16:]) == 100 + i
                    error = np.abs(image[:240].astype(float) - reference[:240])
                    # H.264 4:2:0 blends chroma on tiny point silhouettes. Bound
                    # that image-wide loss separately from flat-region color accuracy.
                    assert error.mean() < 10, (binary, kind, error.mean(),
                        error[:192].mean(axis=(0,1)).tolist(),error[192:].mean(axis=(0,1)).tolist())
                    center = reference[2:190, 2:510]
                    flat = np.ones(center.shape[:2], dtype=bool)
                    for dy in range(-2, 3):
                        for dx in range(-2, 3):
                            flat &= (center == reference[2+dy:190+dy, 2+dx:510+dx]).all(axis=2)
                    if flat.any():
                        assert np.abs(image[2:190, 2:510].astype(float)-center)[flat].mean() < 3
                    # Robust depth uses four grayscale nibbles. Ignore one-pixel codec boundaries;
                    # exact recovered nibbles prove the GPU atlas has the same metric depth codes.
                    def nibbles(v):
                        return np.rint((v[194:238, :, :].mean(axis=2) - 32) * 15 / 192).clip(0, 15)
                    actual, expected = nibbles(image), nibbles(reference)
                    match = (actual == expected).mean()
                    assert match >= .99, (binary, kind, "depth nibbles", match)
                    errors.append(float(error.mean()))
                print(json.dumps({"binary": binary, "kind": kind, "frames": 12, "result": "passed",
                                  "markers": "exact", "IDR_recovery": "passed",
                                  "mean_rgb_depth_error_8bit": float(np.mean(errors))}), flush=True)


if __name__ == "__main__":
    main()
