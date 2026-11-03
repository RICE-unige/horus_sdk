"""Opt-in tests through the real CUDA encoder and an independent H.264 decoder."""

import os
import subprocess

import numpy as np
import pytest

from horus.remote_rendering.nvenc_encoder import GpuH264Encoder


@pytest.mark.skipif(os.environ.get("HORUS_TEST_GPU_VIDEO") != "1",
                    reason="requires NVENC and ffmpeg")
def test_receiver_request_generates_independently_decodable_idr():
    width, height, strip = 256, 256, 64
    video = np.full((height + strip + 16, width * 2, 3), 100, np.uint8)
    with GpuH264Encoder(width, height, strip, fps=60, bitrate_kbps=12000) as encoder:
        encoder.encode_rgb_numpy(video, '{"frame_id":0}', 0)
        encoder.encode_rgb_numpy(video, '{"frame_id":1}', 1)
        encoder.request_keyframe()
        packet = encoder.encode_rgb_numpy(video, '{"frame_id":2}', 2)[0]
    # The receiver has lost every earlier reference frame and SPS/PPS.
    result = subprocess.run(
        ["ffmpeg", "-v", "error", "-f", "h264", "-i", "pipe:0",
         "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
        input=packet.data, capture_output=True, check=True, timeout=30,
    )
    assert len(result.stdout) == video.size


@pytest.mark.skipif(os.environ.get("HORUS_TEST_GPU_VIDEO") != "1",
                    reason="requires NVENC and ffmpeg")
def test_nvenc_depth_symbols_match_quest_limited_range_decoder():
    width, height, strip = 256, 256, 64
    video = np.zeros((height + strip + 16, width * 2, 3), np.uint8)
    levels = np.rint(np.linspace(32, 224, 16)).astype(np.uint8)
    video[height:height + strip] = np.repeat(levels, 32)[None, :, None]
    packets = []
    with GpuH264Encoder(width, height, strip, fps=60, bitrate_kbps=12000) as encoder:
        for sequence in range(8):
            output = encoder.encode_rgb_numpy(video, '{"frame_id":1}', sequence)
            assert len(output) == 1, "Each input must produce its access unit without extra poses"
            assert output[0].source_sequence == sequence
            packets.extend(output)
    result = subprocess.run(
        ["ffmpeg", "-v", "error", "-f", "h264", "-i", "pipe:0",
         "-frames:v", "1", "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
        input=b"".join(packet.data for packet in packets),
        capture_output=True, check=True, timeout=30,
    )
    decoded = np.frombuffer(result.stdout, np.uint8).reshape(video.shape)
    measured = np.array([
        np.median(decoded[height + 8:height + strip - 8, i * 32 + 8:i * 32 + 24, 0])
        for i in range(16)
    ])
    print("depth symbols", levels.tolist(), "decoded RGB", measured.tolist())
    # Auxiliary inputs preserve their limited-range code points. Match the
    # existing receiver's inverse expansion after hardware RGB decode.
    recovered = 16 + measured * (219 / 255)
    np.testing.assert_allclose(recovered, levels, atol=3)
    np.testing.assert_array_equal(
        np.rint((recovered - 32) / 192 * 15).astype(int), np.arange(16)
    )
