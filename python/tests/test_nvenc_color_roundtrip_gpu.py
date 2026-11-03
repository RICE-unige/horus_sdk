"""Opt-in repeated default-decoder RGB/depth/latency contract acceptance."""

import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np
import pytest

from horus.remote_rendering.frame_protocol import decode_frame_marker
from horus.remote_rendering.nvenc_encoder import GpuH264Encoder


@pytest.mark.skipif(os.environ.get("HORUS_TEST_GPU_VIDEO") != "1",
                    reason="requires a separately cleared NVENC GPU window")
@pytest.mark.parametrize("repeat", range(3))
def test_declared_nvenc_colours_depth_and_immediate_output(repeat, tmp_path):
    output = Path(os.environ.get("HORUS_TEST_GPU_VIDEO_OUTPUT", str(tmp_path)))
    output.mkdir(parents=True, exist_ok=True)
    prefix = output / f"nvenc-colour-repeat-{repeat}"
    if prefix.with_suffix(".json").exists() or prefix.with_suffix(".h264").exists():
        raise RuntimeError("Refusing to overwrite previous acceptance evidence")
    width, height, strip, count = 256, 256, 64, 16
    video = np.zeros((height + strip + 16, width * 2, 3), np.uint8)
    left, right = np.array([220, 40, 30]), np.array([25, 180, 220])
    video[:height, :width] = left
    video[:height, width:] = right
    grays = np.array([0, 16, 32, 64, 128, 224, 235, 255], np.uint8)
    video[:32, :width] = np.repeat(grays, 32)[None, :, None]
    levels = np.rint(np.linspace(32, 224, 16)).astype(np.uint8)
    video[height:height + strip] = np.repeat(levels, 32)[None, :, None]
    original = video.copy()
    receipt = {"repeat": repeat, "status": "running", "latency_scope": "local submit-to-packet, not end-to-end",
               "expected_rgb": [left.tolist(), right.tolist()], "frames": [],
               "decoder_range_override": False, "input_sha256": hashlib.sha256(video).hexdigest()}
    packets = []

    def save():
        prefix.with_suffix(".json").write_text(json.dumps(receipt, indent=2) + "\n")

    def decode(data, frames):
        result = subprocess.run(
            ["ffmpeg", "-v", "error", "-threads", "2", "-f", "h264", "-i", "pipe:0",
             "-frames:v", str(frames), "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
            input=data, capture_output=True, check=True, timeout=20,
        )
        return np.frombuffer(result.stdout, np.uint8).reshape(-1, 336, 512, 3)

    try:
        started = time.perf_counter()
        with GpuH264Encoder(width, height, strip, fps=60, bitrate_kbps=12000) as encoder:
            receipt["constructor_ms"] = (time.perf_counter() - started) * 1000
            for index in range(count):
                sequence = repeat * 100 + index
                metadata = json.dumps({"frame_id": sequence})
                if index == 8:
                    encoder.request_keyframe()
                started = time.perf_counter()
                emitted = encoder.encode_rgb_numpy(video, metadata, sequence)
                receipt["frames"].append({"sequence": sequence, "outputs": len(emitted),
                                          "submit_to_packet_ms": (time.perf_counter() - started) * 1000})
                packets.extend(emitted)
                save()
                assert len(emitted) == 1, "Every input must emit immediately without an extra pose"
                assert emitted[0].source_sequence == sequence
                assert emitted[0].frame_index == index
                assert emitted[0].metadata_json == metadata
        data = b"".join(packet.data for packet in packets)
        prefix.with_suffix(".h264").write_bytes(data)
        receipt["stream_sha256"] = hashlib.sha256(data).hexdigest()
        probe = subprocess.run(
            ["ffprobe", "-v", "error", "-show_entries",
             "stream=codec_name,profile,width,height,color_range,color_space,color_transfer,color_primaries",
             "-of", "json", str(prefix.with_suffix(".h264"))],
            capture_output=True, text=True, check=True, timeout=10,
        )
        receipt["stream"] = stream = json.loads(probe.stdout)["streams"][0]
        decoded = decode(data, count)
        receipt["decoded_frames"] = len(decoded)
        measured = np.array([np.median(decoded[:, 64:240, 20:230], axis=(0, 1, 2)),
                             np.median(decoded[:, 64:240, 280:490], axis=(0, 1, 2))])
        receipt["measured_rgb"] = measured.tolist()
        depth_rgb = np.array([np.median(decoded[:, height + 8:height + strip - 8,
                                                    i * 32 + 8:i * 32 + 24, 0]) for i in range(16)])
        recovered = 16 + depth_rgb * (219 / 255)
        receipt["decoded_depth_rgb"] = depth_rgb.tolist()
        receipt["recovered_depth_symbols"] = recovered.tolist()
        latencies = [frame["submit_to_packet_ms"] for frame in receipt["frames"][1:]]
        receipt["submit_to_packet_ms"] = {"p50": float(np.percentile(latencies, 50)),
                                           "p95": float(np.percentile(latencies, 95)), "samples": len(latencies)}
        save()
        assert stream["codec_name"] == "h264" and "Baseline" in stream["profile"]
        assert (stream["width"], stream["height"]) == (512, 336)
        assert stream["color_range"] == "tv"
        assert stream["color_space"] == stream["color_primaries"] == stream["color_transfer"] == "smpte170m"
        assert len(decoded) == count
        np.testing.assert_allclose(measured, np.stack((left, right)), atol=12, rtol=0)
        measured_gray = np.array([np.median(decoded[:, 8:24, i * 32 + 8:i * 32 + 24, 0])
                                  for i in range(8)])
        np.testing.assert_allclose(measured_gray, grays, atol=3, rtol=0)
        np.testing.assert_allclose(recovered, levels, atol=3, rtol=0)
        np.testing.assert_array_equal(np.rint((recovered - 32) / 192 * 15), np.arange(16))
        assert [decode_frame_marker(frame[-16:]) for frame in decoded] == [repeat * 100 + i for i in range(count)]
        np.testing.assert_array_equal(video, original)
        independent = decode(packets[8].data, 1)
        assert len(independent) == 1
        assert decode_frame_marker(independent[0, -16:]) == repeat * 100 + 8
        receipt["requested_idr_independently_decoded"] = True
        receipt["status"] = "passed"
    except Exception as exc:
        receipt["status"] = "failed"
        receipt["error"] = f"{type(exc).__name__}: {exc}"
        if packets and not prefix.with_suffix(".h264").exists():
            prefix.with_suffix(".h264").write_bytes(b"".join(packet.data for packet in packets))
        raise
    finally:
        save()
