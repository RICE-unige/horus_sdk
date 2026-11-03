#!/usr/bin/env python3
"""Real GPU qualification: decode native NVENC output and check each frame/IDR."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
from horus.remote_rendering.frame_protocol import decode_frame_marker


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("binaries", nargs="+")
    parser.add_argument("--mode", choices=["sync", "async", "burst"], default="sync")
    args = parser.parse_args()
    for binary in args.binaries:
        with tempfile.TemporaryDirectory(prefix="horus-native-encode-") as directory:
            path = Path(directory) / "video.h264"
            completed = subprocess.run([binary, str(path), args.mode], text=True, capture_output=True,
                                       timeout=30, check=True)
            packets = json.loads(completed.stdout)
            if args.mode == "burst":
                assert 1 <= len(packets) < 200 and packets[-1]["source_sequence"] == 299
            else:
                assert len(packets) == 12
            probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0",
                                    "-show_streams", "-show_frames", "-of", "json", str(path)],
                                   capture_output=True, timeout=30, check=True)
            info = json.loads(probe.stdout)
            stream = info["streams"][0]
            assert stream["codec_name"] == "h264"
            assert "Baseline" in stream["profile"], stream["profile"]
            assert stream["width"] == 384 and stream["height"] == 64
            assert stream["has_b_frames"] == 0
            frames = info["frames"]
            assert len(frames) == len(packets) and frames[0]["key_frame"]
            if args.mode != "burst":
                assert frames[5]["key_frame"] and frames[7]["key_frame"]
            decoded = subprocess.run(["ffmpeg", "-v", "error", "-i", str(path), "-f", "rawvideo",
                                      "-pix_fmt", "rgb24", "pipe:1"], capture_output=True,
                                     timeout=30, check=True)
            pixels = np.frombuffer(decoded.stdout, dtype=np.uint8).reshape(len(frames), 64, 384, 3)
            errors = []
            for i, (packet, image) in enumerate(zip(packets, pixels)):
                sequence = packet["source_sequence"]
                assert packet["frame_index"] == i
                if args.mode != "burst":
                    assert sequence == 100 + i
                if i:
                    assert sequence > packets[i-1]["source_sequence"]
                assert packet["metadata"]["frame_id"] == sequence
                assert decode_frame_marker(image[-16:]) == sequence
                green = 30 + ((sequence - 100) % 12) * 5
                for crop, expected in ((image[8:40, 16:176], [200, green, 40]),
                                       (image[8:40, 208:368], [40, green, 200])):
                    error = np.abs(crop.astype(float) - expected).mean()
                    errors.append(float(error))
                    assert error < 10, f"{binary}: wrong colors / eye layout: {error}"
            print(json.dumps({"binary": binary, "mode": args.mode, "result": "passed", "frames": len(frames),
                              "marker_ids": "exact", "keyframe_request": "passed" if args.mode != "burst" else "not_assessed",
                              "profile": stream["profile"], "mean_rgb_error": float(np.mean(errors))}))


if __name__ == "__main__":
    main()
