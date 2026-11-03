"""Zero-copy NVIDIA H.264 encoding for the remote-render video atlas."""

from __future__ import annotations

from collections import deque, OrderedDict
from dataclasses import dataclass
import os
import threading
import time
from pathlib import Path
import sys
from typing import Any

import numpy as np


FRAME_MARKER_MAGIC = 0xA5
FRAME_MARKER_BLOCK_SIZE = 8
FRAME_MARKER_BITS = 24
FRAME_MARKER_WIDTH = FRAME_MARKER_BITS * 2 * FRAME_MARKER_BLOCK_SIZE
FRAME_MARKER_HEIGHT = 16


def _limited_code_to_rgb() -> np.ndarray:
    # Auxiliary bytes are limited-range code points, not scene RGB values.
    levels = np.arange(256, dtype=np.float64)
    return np.rint(np.clip((levels - 16.0) * (255.0 / 219.0), 0, 255)).astype(np.uint8)


def _load_low_latency_codec():
    root = Path(os.environ.get("HORUS_NVENC_ROOT", str(
        Path.home() / ".cache/horus/remote_render/nvenc-low-latency-v2"))).expanduser()
    target = root / "site-packages"
    explicit = "HORUS_NVENC_ROOT" in os.environ
    if explicit and not (target / "PyNvVideoCodec").is_dir():
        raise RuntimeError(f"The selected HORUS NVENC overlay is not provisioned: {target}")
    loaded = sys.modules.get("PyNvVideoCodec")
    if explicit and loaded is not None:
        origin = getattr(loaded, "__file__", None)
        if not origin or not Path(origin).resolve().is_relative_to(target.resolve()):
            raise RuntimeError("A different NVENC overlay is already loaded; start a fresh process with the selected HORUS_NVENC_ROOT.")
    if target.is_dir():
        sys.path.insert(0, str(target))
    try:
        import PyNvVideoCodec as codec
    finally:
        if target.is_dir():
            sys.path.remove(str(target))
    if (getattr(codec, "HORUS_OUTPUT_DELAY_OPTION", 0) != 1 or
            getattr(codec, "HORUS_H264_PROFILE_OPTION", 0) != 1 or
            getattr(codec, "HORUS_H264_COLOR_VUI_OPTION", 0) != 1):
        raise RuntimeError(
            "Remote rendering needs the low-latency NVENC adapter. Run "
            "'python3 python/examples/tools/install_remote_nvenc.py' once. "
            "It leaves the stock PyNvVideoCodec installation unchanged."
        )
    return codec


@dataclass(frozen=True)
class EncodedH264Frame:
    data: bytes
    metadata_json: str
    source_sequence: int
    frame_index: int
    picture_type: int


class _CudaArrayView:
    def __init__(self, tensor: Any) -> None:
        element_size = int(tensor.element_size())
        self.__cuda_array_interface__ = {
            "shape": tuple(int(value) for value in tensor.shape),
            "strides": tuple(
                int(tensor.stride(index)) * element_size
                for index in range(tensor.ndim)
            ),
            "data": (int(tensor.data_ptr()), False),
            "typestr": "|u1",
            "version": 3,
        }


class _CudaFrame:
    def __init__(self, tensor: Any) -> None:
        self._view = _CudaArrayView(tensor)

    def cuda(self) -> _CudaArrayView:
        return self._view


class GpuH264Encoder:
    """Pack stereo views on CUDA and encode them without a raw-frame readback."""

    MAX_ENCODED_BACKLOG = 4

    def __init__(
        self,
        width_per_eye: int,
        height_per_eye: int,
        visibility_mask_height: int,
        *,
        fps: int,
        bitrate_kbps: int,
        gpu_id: int = 0,
    ) -> None:
        try:
            nvc = _load_low_latency_codec()
            import torch
        except ImportError as exc:
            raise RuntimeError(
                "Direct GPU WebRTC encoding requires PyNvVideoCodec and CUDA "
                "PyTorch. Run 'python3 python/examples/tools/"
                "install_remote_render_dependencies.py'."
            ) from exc
        if not torch.cuda.is_available():
            raise RuntimeError("Direct GPU WebRTC encoding requires an NVIDIA GPU")

        self.width_per_eye = int(width_per_eye)
        self.height_per_eye = int(height_per_eye)
        self.visibility_mask_height = int(visibility_mask_height)
        self.frame_marker_height = FRAME_MARKER_HEIGHT
        self.width = self.width_per_eye * 2
        self.height = (
            self.height_per_eye
            + self.visibility_mask_height
            + self.frame_marker_height
        )
        self.fps = max(1, int(fps))
        self.bitrate_kbps = max(1000, int(bitrate_kbps))
        if min(
            self.width_per_eye,
            self.height_per_eye,
            self.visibility_mask_height,
        ) <= 0:
            raise ValueError("encoder dimensions must be positive")

        self._torch = torch
        self._nvc = nvc
        self._device = torch.device(f"cuda:{int(gpu_id)}")
        self._auxiliary_rgb_levels = torch.tensor(
            _limited_code_to_rgb(), dtype=torch.uint8, device=self._device,
        )
        self._surface = torch.empty(
            (self.height, self.width, 4),
            dtype=torch.uint8,
            device=self._device,
        )
        # PyNvVideoCodec's ABGR input is a word-ordered A8B8G8R8 value.
        # On this little-endian host its byte layout is RGBA, which matches
        # the renderer's RGB tensors without swapping red and blue.
        self._surface[..., 3].fill_(255)
        if self.width < FRAME_MARKER_WIDTH:
            raise ValueError(
                f"encoder width must be at least {FRAME_MARKER_WIDTH} pixels"
            )
        # Macroblock-aligned solid symbols survive CBR quantization much better
        # than 8-pixel stripes beside a detailed color/depth atlas.
        self._marker_block_size = 32 if self.width >= FRAME_MARKER_BITS * 64 else 8
        self._marker_width = FRAME_MARKER_BITS * 2 * self._marker_block_size
        self._marker_levels_cpu = torch.empty(
            (self._marker_width,),
            dtype=torch.uint8,
            pin_memory=True,
        )
        self._marker_levels_numpy = self._marker_levels_cpu.numpy()
        self._marker_levels_gpu = torch.empty(
            (self._marker_width,),
            dtype=torch.uint8,
            device=self._device,
        )
        self._frame = _CudaFrame(self._surface)
        self._copy_stream = torch.cuda.Stream(device=self._device)
        self._copy_ready_event = torch.cuda.Event()
        bitrate_bps = self.bitrate_kbps * 1000
        self._encoder = nvc.CreateEncoder(
            self.width,
            self.height,
            "ABGR",
            False,
            codec="h264",
            horus_h264_profile="baseline",
            horus_h264_color_range="limited",
            preset="P1",
            tuning_info="ultra_low_latency",
            rc="cbr",
            fps=self.fps,
            gop=self.fps,
            idrperiod=self.fps,
            bf=0,
            extra_output_delay=0,
            lookahead=0,
            bitrate=bitrate_bps,
            maxbitrate=bitrate_bps,
            vbvbufsize=max(100_000, bitrate_bps // self.fps),
            gpu_id=int(gpu_id),
            multipass="disabled",
            colorspace="bt601",
            repeatspspps=1,
        )
        self._pending_metadata: OrderedDict[int, tuple[str, int]] = OrderedDict()
        self._next_frame_index = 0
        self.last_encode_ms = 0.0
        self.encoded_frames = 0
        self.encoded_bytes = 0
        self.dropped_pending_frames = 0
        self._async_condition = threading.Condition()
        self._async_pending: tuple[Any, Any, str, int] | None = None
        self._async_outputs: deque[EncodedH264Frame] = deque()
        self._async_error: BaseException | None = None
        self._async_stop = False
        self._async_thread: threading.Thread | None = None
        self._keyframe_requested = False

    def request_keyframe(self) -> None:
        """Coalesce receiver recovery requests; NVENC is touched only by its worker."""
        with self._async_condition:
            self._keyframe_requested = True

    def start_async(self) -> None:
        """Start the bounded render-to-NVENC handoff worker."""
        with self._async_condition:
            if self._async_thread is not None:
                return
            self._async_stop = False
            self._async_thread = threading.Thread(
                target=self._async_encode_loop,
                name="horus-remote-map-nvenc",
                daemon=True,
            )
            self._async_thread.start()

    def submit_cuda_views(
        self,
        colors: Any,
        depth_codes: Any,
        metadata_json: str,
        source_sequence: int,
    ) -> None:
        """Submit the newest completed GPU render without blocking on NVENC."""
        with self._async_condition:
            self._raise_async_error_locked()
            if self._async_thread is None:
                raise RuntimeError("asynchronous NVENC worker is not running")
            if self._async_pending is not None:
                self.dropped_pending_frames += 1
            self._async_pending = (
                colors,
                depth_codes,
                metadata_json,
                int(source_sequence),
            )
            # Both the encoder and output consumer wait on this condition.
            # notify() can wake only the consumer and strand a raw input.
            self._async_condition.notify_all()

    def poll_encoded(self, max_frames: int = 1, timeout: float = 0.0) -> list[EncodedH264Frame]:
        """Return ordered access units, optionally waiting for the first one."""
        if max_frames <= 0:
            raise ValueError("max_frames must be positive")
        with self._async_condition:
            if timeout > 0:
                self._async_condition.wait_for(
                    lambda: self._async_outputs or self._async_error or self._async_stop,
                    timeout=timeout,
                )
            self._raise_async_error_locked()
            count = min(int(max_frames), len(self._async_outputs))
            outputs = [self._async_outputs.popleft() for _ in range(count)]
            self._async_condition.notify_all()
            return outputs

    def _raise_async_error_locked(self) -> None:
        if self._async_error is not None:
            raise RuntimeError("asynchronous NVENC worker failed") from self._async_error

    def _async_encode_loop(self) -> None:
        while True:
            with self._async_condition:
                # Only raw renders may be replaced. An encoded P-frame can be
                # a reference for every following frame until the next IDR.
                while not self._async_stop and (
                    self._async_pending is None
                    or len(self._async_outputs) >= self.MAX_ENCODED_BACKLOG
                ):
                    self._async_condition.wait()
                if self._async_stop:
                    return
                pending = self._async_pending
                self._async_pending = None
            try:
                assert pending is not None
                outputs = self.encode_cuda_views(*pending)
            except BaseException as exception:
                with self._async_condition:
                    self._async_error = exception
                    self._async_stop = True
                    self._async_condition.notify_all()
                return
            if outputs:
                with self._async_condition:
                    self._async_outputs.extend(outputs)
                    if len(self._async_outputs) > 16:
                        self._async_error = RuntimeError(
                            "asynchronous NVENC output exceeded its bounded queue"
                        )
                        self._async_stop = True
                    self._async_condition.notify_all()

    def encode_cuda_views(
        self,
        colors: Any,
        depth_codes: Any,
        metadata_json: str,
        source_sequence: int,
    ) -> list[EncodedH264Frame]:
        torch = self._torch
        expected_colors = (
            2,
            self.height_per_eye,
            self.width_per_eye,
            3,
        )
        expected_masks = (
            2,
            self.visibility_mask_height,
            self.width_per_eye,
        )
        if tuple(colors.shape) != expected_colors or colors.dtype != torch.uint8:
            raise ValueError(f"colors must be CUDA uint8 with shape {expected_colors}")
        if (
            tuple(depth_codes.shape) != expected_masks
            or depth_codes.dtype != torch.uint8
        ):
            raise ValueError(
                "depth_codes must be CUDA uint8 with shape "
                f"{expected_masks}"
            )
        if not colors.is_cuda or not depth_codes.is_cuda:
            raise ValueError("direct encoder inputs must remain on CUDA")

        with torch.cuda.stream(self._copy_stream):
            top = self._surface[: self.height_per_eye]
            top[:, : self.width_per_eye, 0:3].copy_(colors[0])
            top[:, self.width_per_eye :, 0:3].copy_(colors[1])
            mask_atlas = self._surface[self.height_per_eye :]
            mask_atlas = mask_atlas[: self.visibility_mask_height]
            mask_atlas[:, : self.width_per_eye, 0:3].copy_(
                depth_codes[0, ..., None].expand(-1, -1, 3)
            )
            mask_atlas[:, self.width_per_eye :, 0:3].copy_(
                depth_codes[1, ..., None].expand(-1, -1, 3)
            )
            self._write_frame_marker(source_sequence)
            self._prepare_auxiliary_rgb()
            self._copy_ready_event.record(self._copy_stream)
        self._copy_ready_event.synchronize()
        return self._encode_surface(metadata_json, source_sequence)

    def encode_rgb_numpy(
        self,
        video_rgb: np.ndarray,
        metadata_json: str,
        source_sequence: int,
    ) -> list[EncodedH264Frame]:
        rgb = np.ascontiguousarray(video_rgb, dtype=np.uint8)
        expected = (self.height, self.width, 3)
        if rgb.shape != expected:
            raise ValueError(f"video_rgb must have shape {expected}")
        with self._torch.cuda.stream(self._copy_stream):
            source = self._torch.from_numpy(rgb).to(
                self._device,
                non_blocking=False,
            )
            self._surface[..., 0:3].copy_(source)
            self._write_frame_marker(source_sequence)
            self._prepare_auxiliary_rgb()
            self._copy_ready_event.record(self._copy_stream)
        self._copy_ready_event.synchronize()
        return self._encode_surface(metadata_json, source_sequence)

    def _prepare_auxiliary_rgb(self) -> None:
        # NVENC now performs a declared limited-range RGB-to-YUV conversion.
        # Undo that range on depth/identity inputs so the receiver still sees
        # the original code points after its existing inverse expansion.
        auxiliary = self._surface[self.height_per_eye :, :, 0:3]
        rgb = self._auxiliary_rgb_levels[auxiliary[..., 0].long()]
        auxiliary.copy_(rgb[..., None].expand(-1, -1, 3))

    def _write_frame_marker(self, source_sequence: int) -> None:
        """Write an H.264-resistant frame identity into the auxiliary strip."""
        word = (FRAME_MARKER_MAGIC << 16) | (int(source_sequence) & 0xFFFF)
        bits = np.asarray(
            [(word >> (FRAME_MARKER_BITS - 1 - index)) & 1
             for index in range(FRAME_MARKER_BITS)],
            dtype=np.uint8,
        )
        pairs = np.column_stack((bits, 1 - bits)).reshape(-1)
        self._marker_levels_numpy[:] = np.repeat(
            np.where(pairs != 0, 224, 32).astype(np.uint8),
            self._marker_block_size,
        )
        self._marker_levels_gpu.copy_(
            self._marker_levels_cpu,
            non_blocking=True,
        )
        marker = self._surface[-self.frame_marker_height :]
        marker[..., 0:3].fill_(16)
        marker[:, :self._marker_width, 0:3].copy_(
            self._marker_levels_gpu[None, :, None].expand(
                self.frame_marker_height,
                self._marker_width,
                3,
            )
        )

    def _encode_surface(
        self,
        metadata_json: str,
        source_sequence: int,
    ) -> list[EncodedH264Frame]:
        if not metadata_json:
            raise ValueError("encoded remote-render frames require metadata")
        frame_index = self._next_frame_index
        self._next_frame_index += 1
        self._pending_metadata[frame_index] = (
            metadata_json,
            int(source_sequence),
        )

        params = self._nvc.NV_ENC_PIC_PARAMS()
        params.frameIdx = frame_index
        params.inputTimeStamp = frame_index
        params.inputDuration = 1
        with self._async_condition:
            keyframe_requested = self._keyframe_requested
            self._keyframe_requested = False
        if keyframe_requested:
            # NV_ENC_PIC_FLAG_FORCEIDR | NV_ENC_PIC_FLAG_OUTPUT_SPSPPS.
            params.encodePicFlags = 0x2 | 0x4
        started = time.perf_counter()
        try:
            packets = self._encoder.Encode(self._frame, params)
        except Exception:
            self._pending_metadata.pop(frame_index, None)
            raise
        self.last_encode_ms = (time.perf_counter() - started) * 1000.0
        return self._resolve_packets(packets)

    def _resolve_packets(self, packets: Any) -> list[EncodedH264Frame]:
        outputs: list[EncodedH264Frame] = []
        for packet in packets or ():
            frame_index = int(packet["timestamp"])
            pending = self._pending_metadata.pop(frame_index, None)
            if pending is None:
                raise RuntimeError(
                    f"NVENC returned unknown frame index {frame_index}"
                )
            metadata, source_sequence = pending
            data = bytes(packet["data"])
            outputs.append(
                EncodedH264Frame(
                    data=data,
                    metadata_json=metadata,
                    source_sequence=source_sequence,
                    frame_index=frame_index,
                    picture_type=int(packet.get("picture_type", -1)),
                )
            )
            self.encoded_frames += 1
            self.encoded_bytes += len(data)
        if len(self._pending_metadata) > 16:
            raise RuntimeError("NVENC output queue exceeded its bounded latency window")
        return outputs

    def close(self) -> None:
        thread = getattr(self, "_async_thread", None)
        condition = getattr(self, "_async_condition", None)
        if thread is not None and condition is not None:
            with condition:
                self._async_stop = True
                self._async_pending = None
                condition.notify_all()
            thread.join(timeout=5.0)
            if thread.is_alive():
                raise RuntimeError("asynchronous NVENC worker did not stop")
            self._async_thread = None
        if getattr(self, "_encoder", None) is not None:
            try:
                self._encoder.EndEncode()
            finally:
                self._encoder = None
        self._pending_metadata.clear()
        if hasattr(self, "_async_outputs"):
            self._async_outputs.clear()

    def __enter__(self) -> "GpuH264Encoder":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
