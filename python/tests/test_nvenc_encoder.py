from __future__ import annotations

from collections import deque, OrderedDict
import threading
import time
from types import SimpleNamespace
import sys

import pytest

from horus.remote_rendering.nvenc_encoder import (
    EncodedH264Frame, GpuH264Encoder, _load_low_latency_codec,
)


def test_nvenc_rejects_adapter_that_silently_ignores_h264_profile(monkeypatch):
    monkeypatch.setitem(sys.modules, "PyNvVideoCodec", SimpleNamespace(
        HORUS_OUTPUT_DELAY_OPTION=1))
    with pytest.raises(RuntimeError, match="install_remote_nvenc"):
        _load_low_latency_codec()


def test_nvenc_accepts_adapter_with_explicit_profile_support(monkeypatch):
    codec = SimpleNamespace(HORUS_OUTPUT_DELAY_OPTION=1, HORUS_H264_PROFILE_OPTION=1,
                            HORUS_H264_COLOR_VUI_OPTION=1)
    monkeypatch.setitem(sys.modules, "PyNvVideoCodec", codec)
    assert _load_low_latency_codec() is codec


def test_nvenc_rejects_previous_adapter_without_explicit_colour_vui(monkeypatch):
    monkeypatch.setitem(sys.modules, "PyNvVideoCodec", SimpleNamespace(
        HORUS_OUTPUT_DELAY_OPTION=1, HORUS_H264_PROFILE_OPTION=1))
    with pytest.raises(RuntimeError, match="install_remote_nvenc"):
        _load_low_latency_codec()


def _packet_encoder() -> GpuH264Encoder:
    encoder = GpuH264Encoder.__new__(GpuH264Encoder)
    encoder._pending_metadata = OrderedDict(
        ((7, ('{"frame_id":7}', 107)), (8, ('{"frame_id":8}', 108)))
    )
    encoder.encoded_frames = 0
    encoder.encoded_bytes = 0
    return encoder


def test_nvenc_packets_keep_the_metadata_for_the_encoded_input_frame():
    encoder = _packet_encoder()

    frames = encoder._resolve_packets(
        ({"timestamp": 7, "data": b"\x00\x00\x00\x01\x65", "picture_type": 3},)
    )

    assert len(frames) == 1
    assert frames[0].frame_index == 7
    assert frames[0].metadata_json == '{"frame_id":7}'
    assert frames[0].source_sequence == 107
    assert frames[0].data.endswith(b"\x65")
    assert list(encoder._pending_metadata) == [8]
    assert encoder.encoded_frames == 1
    assert encoder.encoded_bytes == 5


def test_nvenc_rejects_an_unmatched_delayed_packet():
    encoder = _packet_encoder()

    with pytest.raises(RuntimeError, match="unknown frame index"):
        encoder._resolve_packets(
            ({"timestamp": 99, "data": b"bad", "picture_type": 0},)
        )


def test_async_nvenc_handoff_encodes_without_blocking_the_submitter():
    encoder = GpuH264Encoder.__new__(GpuH264Encoder)
    encoder._async_condition = threading.Condition()
    encoder._async_pending = None
    encoder._async_outputs = deque()
    encoder._async_error = None
    encoder._async_stop = False
    encoder._async_thread = None
    encoder.dropped_pending_frames = 0
    encoded = []

    def encode(colors, masks, metadata, source_sequence):
        time.sleep(0.02)
        encoded.append((colors, masks, metadata, source_sequence))
        return []

    encoder.encode_cuda_views = encode
    encoder.start_async()
    started = time.perf_counter()
    encoder.submit_cuda_views("colors", "masks", "metadata", 12)
    elapsed = time.perf_counter() - started

    deadline = time.monotonic() + 1.0
    while not encoded and time.monotonic() < deadline:
        time.sleep(0.005)
    with encoder._async_condition:
        encoder._async_stop = True
        encoder._async_condition.notify_all()
    encoder._async_thread.join(timeout=1.0)

    assert elapsed < 0.01
    assert encoded == [("colors", "masks", "metadata", 12)]


def test_async_nvenc_handoff_replaces_only_unencoded_work():
    encoder = GpuH264Encoder.__new__(GpuH264Encoder)
    encoder._async_condition = threading.Condition()
    encoder._async_pending = ("old", "mask", "old-metadata", 11)
    encoder._async_outputs = deque()
    encoder._async_error = None
    encoder._async_stop = False
    encoder._async_thread = object()
    encoder.dropped_pending_frames = 0

    encoder.submit_cuda_views("new", "mask", "new-metadata", 12)

    assert encoder._async_pending == ("new", "mask", "new-metadata", 12)
    assert encoder.dropped_pending_frames == 1


def test_slow_consumer_preserves_encoded_chain_and_bounds_backlog():
    encoder = GpuH264Encoder.__new__(GpuH264Encoder)
    encoder._async_condition = threading.Condition()
    encoder._async_pending = None
    encoder._async_outputs = deque()
    encoder._async_error = None
    encoder._async_stop = False
    encoder._async_thread = None
    encoder.dropped_pending_frames = 0
    produced = []

    def encode(colors, masks, metadata, sequence):
        produced.append(sequence)
        return [EncodedH264Frame(b"au", metadata, sequence, len(produced) - 1, 0)]

    encoder.encode_cuda_views = encode
    encoder.start_async()
    try:
        for sequence in range(20):
            encoder.submit_cuda_views(None, None, "{}", sequence)
            time.sleep(0.002)
        assert len(encoder._async_outputs) <= encoder.MAX_ENCODED_BACKLOG
        assert encoder._async_error is None
        consumed = []
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            consumed.extend(encoder.poll_encoded(timeout=0.05))
            if consumed and consumed[-1].source_sequence == 19:
                break
        assert [packet.source_sequence for packet in consumed] == produced
        assert [packet.frame_index for packet in consumed] == list(range(len(produced)))
        assert consumed[-1].source_sequence == 19  # drained even without another pose
        assert encoder.dropped_pending_frames > 0
    finally:
        with encoder._async_condition:
            encoder._async_stop = True
            encoder._async_condition.notify_all()
        encoder._async_thread.join(timeout=1)
