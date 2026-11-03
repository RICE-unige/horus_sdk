"""Host-side, non-metric stereo preview. Never use this output as measured depth.

Inference is optional and local. Original observations are not modified; publish
the returned RGB side-by-side image on a separate presentation-only topic.
"""
from __future__ import annotations

import io
import math
import threading
import time

import numpy as np

MODEL_ID = "depth-anything/Depth-Anything-V2-Small-hf"
MODEL_REVISION = "5426e4f0f36572d16453bbda7a8389317b1bef99"
MAX_PIXELS = 3840 * 2160
MAX_DIMENSION = 8192


def image_rgb(message, compressed=False):
    """Validate bounds before decoding; support ROS padding without changing input."""
    import cv2
    from PIL import Image

    if compressed:
        if not 0 < len(message.data) <= 16 * 1024 * 1024:
            raise ValueError("Compressed preview image exceeds 16 MiB")
        data = bytes(message.data)
        with Image.open(io.BytesIO(data)) as header:
            if not 0 < header.width <= MAX_DIMENSION or not 0 < header.height <= MAX_DIMENSION or header.width * header.height > MAX_PIXELS:
                raise ValueError("Preview image exceeds 3840x2160 pixel budget")
        decoded = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if decoded is None:
            raise ValueError("Invalid compressed image")
        return cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
    if message.encoding not in {"rgb8", "bgr8", "rgba8", "bgra8", "mono8"}:
        raise ValueError("Portal input requires rgb8, bgr8, rgba8, bgra8 or mono8")
    w, h = int(message.width), int(message.height)
    channels = {"mono8": 1, "rgba8": 4, "bgra8": 4}.get(message.encoding, 3)
    if not 0 < w <= MAX_DIMENSION or not 0 < h <= MAX_DIMENSION or w * h > MAX_PIXELS or not w * channels <= message.step <= w * channels + 65536:
        raise ValueError("Invalid image dimensions/stride")
    if len(message.data) != h * message.step or len(message.data) > 64 * 1024 * 1024:
        raise ValueError("Image payload does not match stride and height")
    pixels = np.frombuffer(message.data, np.uint8).reshape(h, message.step)[:, :w * channels].reshape(h, w, channels)
    if channels == 1:
        return np.repeat(pixels, 3, axis=2)
    return np.ascontiguousarray(pixels[:, :, :3][:, :, ::-1] if message.encoding.startswith("bgr") else pixels[:, :, :3])


class RelativeStereoWarp:
    """Small depth-dependent binocular disparity, not calibrated reconstruction.

    Smooth the normalization range, not moving image pixels. Inverse warping
    cannot recover occluded surfaces; keep disparity restrained and fade edges.
    """
    def __init__(self, strength=0.008):
        if not math.isfinite(strength) or not 0 <= strength <= 0.015:
            raise ValueError("Per-eye disparity must be 0-0.015 of image width")
        self.strength = strength
        self.bounds = None

    def render(self, rgb, relative_inverse_depth):
        import cv2
        if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8 or not 0 < rgb.shape[0] * rgb.shape[1] <= MAX_PIXELS or max(rgb.shape[:2]) > MAX_DIMENSION:
            raise ValueError("Expected bounded uint8 RGB image")
        depth = np.asarray(relative_inverse_depth, dtype=np.float32)
        if depth.ndim != 2 or not depth.size or depth.size > MAX_PIXELS or not np.isfinite(depth).all():
            raise ValueError("Depth estimate must be a bounded, finite 2D array")
        lo, hi = np.percentile(depth, (2, 98))
        h, w = rgb.shape[:2]
        if hi - lo < 1e-6 or self.strength == 0:
            return np.concatenate((rgb, rgb), axis=1)
        if self.bounds is None:
            self.bounds = (lo, hi)
        else:
            self.bounds = tuple(.8 * old + .2 * new for old, new in zip(self.bounds, (lo, hi)))
        lo, hi = self.bounds
        normalized = np.clip((depth - lo) / max(hi - lo, 1e-6), 0, 1)
        disparity = cv2.resize(np.asarray((normalized - .5) * (2 * self.strength * w), np.float32),
                               (w, h), interpolation=cv2.INTER_LINEAR)
        x, y = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        eyes = []
        for sign in (1, -1):
            source_x = x.copy()
            for _ in range(3):
                shift = cv2.remap(disparity, source_x, y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
                source_x = x - sign * shift
            eyes.append(cv2.remap(rgb, source_x, y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE))
        return np.concatenate(eyes, axis=1)


def inference_shape(height, width, longest_edge):
    if height <= 0 or width <= 0:
        raise ValueError("Invalid inference dimensions")
    scale = longest_edge / max(height, width)
    return {"height": max(14, round(height * scale / 14) * 14),
            "width": max(14, round(width * scale / 14) * 14)}


class RelativeDepthEstimator:
    def __init__(self, cache_dir, device="cpu", size=294, download=False):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        if size < 140 or size > 518 or size % 14:
            raise ValueError("Inference size must be a multiple of 14 between 140 and 518")
        if device != "cpu" and device != "cuda":
            raise ValueError("Device must be cpu or cuda")
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable; choose --device cpu")
        torch.set_num_threads(min(4, torch.get_num_threads()))
        options = dict(revision=MODEL_REVISION, cache_dir=str(cache_dir), local_files_only=not download,
                       trust_remote_code=False)
        self.processor = AutoImageProcessor.from_pretrained(MODEL_ID, use_fast=False, **options)
        self.model = AutoModelForDepthEstimation.from_pretrained(MODEL_ID, use_safetensors=True, **options).to(device).eval()
        self.device, self.size = device, size

    def __call__(self, rgb):
        import torch
        inputs = self.processor(images=rgb, return_tensors="pt", keep_aspect_ratio=False,
                                size=inference_shape(*rgb.shape[:2], self.size))
        with torch.inference_mode():
            depth = self.model(**{k: v.to(self.device) for k, v in inputs.items()}).predicted_depth
        return depth[0].float().cpu().numpy()


class LatestPreviewWorker:
    """One in-flight frame + one pending frame, rate/age bounded, no FIFO backlog."""
    def __init__(self, process, publish, on_error, fps=10, max_age=.5):
        if not math.isfinite(fps) or not 1 <= fps <= 30 or not math.isfinite(max_age) or not .05 <= max_age <= 2:
            raise ValueError("FPS must be 1-30 and max age 0.05-2 seconds")
        self.process, self.publish, self.on_error = process, publish, on_error
        self.period, self.max_age = 1 / fps, max_age
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.received = self.replaced = self.expired = self.published = 0
        self.thread = threading.Thread(target=self._run, name="horus-camera-portal", daemon=False)
        self.thread.start()

    def submit(self, message):
        with self.condition:
            if self.closed:
                return
            self.received += 1
            self.replaced += int(self.pending is not None)
            self.pending = (message, time.monotonic())
            self.condition.notify()

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify()
        self.thread.join()

    def _run(self):
        next_frame = 0.
        while True:
            with self.condition:
                while not self.closed and (self.pending is None or time.monotonic() < next_frame):
                    self.condition.wait(None if self.pending is None else max(.001, next_frame - time.monotonic()))
                if self.closed:
                    return
                message, received = self.pending
                self.pending = None
            next_frame = time.monotonic() + self.period
            if time.monotonic() - received > self.max_age:
                self.expired += 1
                continue
            try:
                result = self.process(message)
                with self.condition:
                    if self.closed:
                        return
                    if time.monotonic() - received > self.max_age:
                        self.expired += 1
                    else:
                        self.publish(message, result)
                        self.published += 1
            except Exception as exc:
                self.on_error(exc)


def main():
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description="Local estimated stereo preview; not measured depth or training observations")
    parser.add_argument("--input", required=True, help="Original RGB ROS topic (never modified)")
    parser.add_argument("--output", required=True, help="Separate derived SBS CompressedImage topic")
    parser.add_argument("--compressed", action="store_true", help="Input is CompressedImage instead of Image")
    parser.add_argument("--model-cache", type=Path, default=Path.home() / ".cache/horus/portal-depth")
    parser.add_argument("--download-model", action="store_true", help="Explicitly allow the pinned model download")
    parser.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    parser.add_argument("--size", type=int, default=294, help="Depth inference longest edge; RGB output resolution is preserved")
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--max-age", type=float, default=.5)
    parser.add_argument("--strength", type=float, default=.008)
    args = parser.parse_args()
    if not args.input.startswith("/") or not args.output.startswith("/") or args.input.rstrip("/") == args.output.rstrip("/"):
        parser.error("Use distinct absolute input and output topics")
    import cv2
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.executors import ExternalShutdownException
    from sensor_msgs.msg import Image, CompressedImage
    estimator = RelativeDepthEstimator(args.model_cache, args.device, args.size, args.download_model)
    warp = RelativeStereoWarp(args.strength)
    cv2.setNumThreads(2)
    rclpy.init()
    node = rclpy.create_node("horus_estimated_camera_portal")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher = node.create_publisher(CompressedImage, args.output, qos)
    last_error = [0.]

    def process(message):
        rgb = image_rgb(message, args.compressed)
        sbs = warp.render(rgb, estimator(rgb))
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(sbs, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise ValueError("Failed to encode stereo preview")
        return encoded.tobytes()

    def publish(source, data):
        result = CompressedImage()
        result.header = source.header
        result.format = "rgb8; jpeg compressed bgr8"
        result.data = data
        publisher.publish(result)

    def error(exc):
        if time.monotonic() - last_error[0] > 5:
            node.get_logger().warning(f"Portal frame rejected: {exc}")
            last_error[0] = time.monotonic()

    worker = None
    try:
        worker = LatestPreviewWorker(process, publish, error, args.fps, args.max_age)
        node.create_subscription(CompressedImage if args.compressed else Image, args.input, worker.submit, qos)
        node.create_timer(5., lambda: node.get_logger().info(
            f"ESTIMATED stereo: received={worker.received} published={worker.published} "
            f"replaced={worker.replaced} age_dropped={worker.expired}; not measured depth"))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if worker is not None:
            worker.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
