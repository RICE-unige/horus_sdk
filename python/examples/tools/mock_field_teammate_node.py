#!/usr/bin/env python3
"""Offline stand-in for a HoloLens field-teammate companion app.

This node publishes everything a real HoloLens companion would publish for a
field teammate -- pose (TF), status, localization confidence, a first-person
video frame, and guidance state -- and answers operator guidance requests with
acknowledge / complete responses. It lets the full "On the Map / In the Team"
contract be exercised end to end without a headset.

The topic names are derived from the SDK's ``FieldTeammateConfig`` so this mock
can never drift from the registration contract.

Run (ROS 2 sourced, from the repo root):
    PYTHONPATH=python:$PYTHONPATH python3 python/examples/tools/mock_field_teammate_node.py --name field_teammate_1
"""

from __future__ import annotations

import argparse
import base64
from io import BytesIO
import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys

# Keep the import quiet; this is a runtime tool, not an interactive session.
os.environ.setdefault("HORUS_SDK_NO_BANNER", "1")

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Float32, String
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster
except Exception as exc:  # pragma: no cover - depends on a sourced ROS 2 env
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    raise SystemExit(1)

from horus.robot import FieldTeammateConfig

from field_teammate_articulated_assets import (
    FieldTeammateProfile,
    field_teammate_body_link_layout,
)

try:
    from PIL import Image, ImageDraw
except Exception:  # pragma: no cover - optional runtime dependency
    Image = None
    ImageDraw = None

# A genuinely valid 1x1 JPEG so downstream decoders accept the FPV frame.
_PLACEHOLDER_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAP//////////////////////////////"
    "////////////////////////////////////////////////////2wBDAf//////"
    "////////////////////////////////////////////////////////////////"
    "////////////////wAARCAABAAEDASIAAhEBAxEB/8QAFAABAAAAAAAAAAAAAAAAAA"
    "AAAv/EABQQAQAAAAAAAAAAAAAAAAAAAAD/xAAUAQEAAAAAAAAAAAAAAAAAAAAA/8QA"
    "FBEBAAAAAAAAAAAAAAAAAAAAAP/aAAwDAQACEQMRAD8AvwA//9k="
)

DEFAULT_REAL_FPV_VIDEO_URL = (
    "https://commons.wikimedia.org/wiki/Special:Redirect/file/Walking_in_the_sands.webm"
)
DEFAULT_REAL_FPV_ATTRIBUTION = (
    "Walking in the sands.webm by sgu18ify, Wikimedia Commons, CC BY 3.0"
)


def _yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        (sr * cp * cy) - (cr * sp * sy),
        (cr * sp * cy) + (sr * cp * sy),
        (cr * cp * sy) - (sr * sp * cy),
        (cr * cp * cy) + (sr * sp * sy),
    )


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _lerp_channel(a: int, b: int, t: float) -> int:
    return int(a + (b - a) * max(0.0, min(1.0, t)))


def _lerp_color(a: tuple[int, int, int], b: tuple[int, int, int], t: float) -> tuple[int, int, int]:
    return (
        _lerp_channel(a[0], b[0], t),
        _lerp_channel(a[1], b[1], t),
        _lerp_channel(a[2], b[2], t),
    )


def _is_url(value: str) -> bool:
    lowered = str(value or "").strip().lower()
    return lowered.startswith("http://") or lowered.startswith("https://")


def _cache_video_file(source: str, cache_dir: Path) -> Path:
    cache_dir.mkdir(parents=True, exist_ok=True)
    if not source:
        source = DEFAULT_REAL_FPV_VIDEO_URL

    if not _is_url(source):
        local = Path(source).expanduser()
        if not local.is_file():
            raise FileNotFoundError(f"FPV video file does not exist: {local}")
        return local

    suffix = Path(source.split("?", 1)[0]).suffix or ".webm"
    destination = cache_dir / f"source{suffix}"
    if destination.is_file() and destination.stat().st_size > 1024:
        return destination

    print(f"[fpv] downloading real video source: {source}")
    process = subprocess.run(
        [
            "curl",
            "-L",
            "--fail",
            "--retry",
            "2",
            "--connect-timeout",
            "20",
            "--max-time",
            "180",
            "-o",
            str(destination),
            source,
        ],
        check=False,
        text=True,
    )
    if process.returncode != 0 or not destination.is_file() or destination.stat().st_size <= 1024:
        raise RuntimeError(f"Failed to download FPV video source: {source}")
    return destination


def _prepare_real_fpv_frames(
    source: str,
    cache_dir: Path,
    *,
    width: int = 640,
    height: int = 360,
    target_fps: float = 15.0,
) -> list[bytes]:
    frames_dir = cache_dir / "frames"
    manifest_path = frames_dir / "manifest.json"
    if manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            files = [frames_dir / item for item in manifest.get("frames", [])]
            frames = [path.read_bytes() for path in files if path.is_file()]
            if frames:
                return frames
        except Exception:
            pass

    try:
        import cv2  # type: ignore
    except Exception as exc:
        raise RuntimeError("OpenCV is required for the real FPV demo stream.") from exc

    video_path = _cache_video_file(source, cache_dir)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"Failed to open FPV video: {video_path}")

    source_fps = float(capture.get(cv2.CAP_PROP_FPS) or 0.0)
    stride = max(1, int(round(source_fps / max(1.0, target_fps)))) if source_fps > 0.0 else 2
    frames_dir.mkdir(parents=True, exist_ok=True)
    for old_frame in frames_dir.glob("frame_*.jpg"):
        try:
            old_frame.unlink()
        except OSError:
            pass

    encoded_names: list[str] = []
    index = 0
    saved = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % stride != 0:
            index += 1
            continue

        frame_h, frame_w = frame.shape[:2]
        target_aspect = width / float(height)
        source_aspect = frame_w / float(max(1, frame_h))
        if source_aspect > target_aspect:
            crop_w = int(frame_h * target_aspect)
            x0 = max(0, (frame_w - crop_w) // 2)
            frame = frame[:, x0:x0 + crop_w]
        else:
            crop_h = int(frame_w / target_aspect)
            y0 = max(0, (frame_h - crop_h) // 2)
            frame = frame[y0:y0 + crop_h, :]

        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        output = frames_dir / f"frame_{saved:04d}.jpg"
        if not cv2.imwrite(str(output), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82]):
            raise RuntimeError(f"Failed writing decoded FPV frame: {output}")
        encoded_names.append(output.name)
        saved += 1
        index += 1

    capture.release()
    if not encoded_names:
        raise RuntimeError(f"No frames decoded from FPV video: {video_path}")

    manifest_path.write_text(
        json.dumps(
            {
                "source": source or DEFAULT_REAL_FPV_VIDEO_URL,
                "attribution": DEFAULT_REAL_FPV_ATTRIBUTION,
                "width": width,
                "height": height,
                "target_fps": target_fps,
                "frames": encoded_names,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    return [(frames_dir / name).read_bytes() for name in encoded_names]


class MockFieldTeammateNode(Node):
    def __init__(
        self,
        name: str,
        wearable: str,
        rate_hz: float,
        *,
        profile_height: float = 1.75,
        profile_sex: str = "unspecified",
        fpv_source: str = "real",
        fpv_video: str = "",
        fpv_cache_dir: str = "",
    ) -> None:
        super().__init__("mock_field_teammate")
        self._name = name
        self._profile = FieldTeammateProfile(height_m=profile_height, sex=profile_sex)
        self._contract = FieldTeammateConfig.from_values(
            name=name,
            wearable_type=wearable,
            profile_height_m=self._profile.normalized_height_m,
            profile_sex=self._profile.normalized_sex,
            body_model="meta_avatar",
        )
        topics = self._contract.to_payload()["topics"]

        self._map_frame = "map"
        self._base_frame = self._contract.base_frame
        self._camera_frame = self._contract.camera_frame

        self._tf_broadcaster = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(String, topics["status"], 10)
        self._confidence_pub = self.create_publisher(Float32, topics["localization_confidence"], 10)
        self._fpv_pub = self.create_publisher(CompressedImage, topics["first_person_video"], 10)
        self._guidance_state_pub = self.create_publisher(String, topics["guidance_state"], 10)
        self._guidance_response_pub = self.create_publisher(String, topics["guidance_response"], 10)

        self._guidance_request_sub = self.create_subscription(
            String, topics["guidance_request"], self._on_guidance_request, 10
        )
        for key in ("guidance_annotation", "guidance_route", "guidance_warning", "audio"):
            self.create_subscription(
                String, topics[key], self._make_logger(key), 10
            )

        self._rng = random.Random()
        self._phase = self._rng.uniform(0.0, math.tau)
        self._x = self._rng.uniform(-0.4, 0.4)
        self._y = self._rng.uniform(-0.4, 0.4)
        self._yaw = self._rng.uniform(-math.pi, math.pi)
        self._target_x = self._x
        self._target_y = self._y
        self._target_speed = 0.45
        self._pause_remaining = 0.0
        self._walk_distance = 0.0
        self._frame_index = 0
        self._rate_hz = max(1.0, float(rate_hz))
        self._frame_period = 1.0 / self._rate_hz
        self._timer = self.create_timer(self._frame_period, self._tick)
        self._state = "idle"
        self._body_layout = field_teammate_body_link_layout(self._profile)
        self._fpv_source = str(fpv_source or "real").strip().lower()
        if self._fpv_source not in {"real", "synthetic", "auto"}:
            self._fpv_source = "real"
        self._fpv_frames: list[bytes] = []
        if self._fpv_source != "synthetic":
            cache_dir = Path(fpv_cache_dir).expanduser() if fpv_cache_dir else (
                Path.home() / ".cache" / "horus" / "field_teammate_fpv" / "walking_in_the_sands"
            )
            try:
                self._fpv_frames = _prepare_real_fpv_frames(
                    fpv_video or DEFAULT_REAL_FPV_VIDEO_URL,
                    cache_dir,
                    target_fps=self._rate_hz,
                )
                self.get_logger().info(
                    f"Using real FPV video frames ({len(self._fpv_frames)} cached): {DEFAULT_REAL_FPV_ATTRIBUTION}"
                )
            except Exception as exc:
                if self._fpv_source == "real":
                    raise
                self.get_logger().warning(f"Real FPV stream unavailable; falling back to synthetic frames: {exc}")
        self._choose_new_target()

        self.get_logger().info(
            f"Mock field teammate '{name}' ({wearable}) publishing on {self._base_frame}; "
            f"answering guidance on {topics['guidance_request']}."
        )

    def _make_logger(self, kind: str):
        def _handler(msg: String) -> None:
            self.get_logger().info(f"received {kind}: {msg.data[:160]}")

        return _handler

    def _on_guidance_request(self, msg: String) -> None:
        self.get_logger().info(f"guidance request: {msg.data[:160]}")
        self._state = "acting"
        self._publish_guidance_state()
        self._guidance_response_pub.publish(
            String(data=json.dumps({"teammate": self._name, "response": "acknowledge"}))
        )

    def _publish_guidance_state(self) -> None:
        self._guidance_state_pub.publish(
            String(data=json.dumps({"teammate": self._name, "state": self._state}))
        )

    def _choose_new_target(self) -> None:
        self._target_x = self._rng.uniform(-1.25, 1.25)
        self._target_y = self._rng.uniform(-1.00, 1.00)
        self._target_speed = self._rng.uniform(0.28, 0.72)

    def _update_motion(self, dt: float) -> tuple[float, float, float, str]:
        self._phase += dt

        if self._pause_remaining > 0.0:
            self._pause_remaining = max(0.0, self._pause_remaining - dt)
            self._yaw += _wrap_angle(self._yaw + 0.25 * math.sin(self._phase * 1.7) - self._yaw) * min(1.0, dt * 2.0)
            return self._x, self._y, self._yaw, "observing"

        dx = self._target_x - self._x
        dy = self._target_y - self._y
        distance = math.hypot(dx, dy)
        if distance < 0.08:
            self._pause_remaining = self._rng.uniform(0.6, 2.2)
            self._choose_new_target()
            return self._x, self._y, self._yaw, "observing"

        target_yaw = math.atan2(dy, dx)
        max_turn = 2.4 * dt
        yaw_delta = max(-max_turn, min(max_turn, _wrap_angle(target_yaw - self._yaw)))
        self._yaw = _wrap_angle(self._yaw + yaw_delta)

        speed = self._target_speed * max(0.35, math.cos(_wrap_angle(target_yaw - self._yaw)))
        step = min(distance, max(0.0, speed) * dt)
        self._x += math.cos(target_yaw) * step
        self._y += math.sin(target_yaw) * step
        self._walk_distance += step
        return self._x, self._y, self._yaw, "walking"

    def _build_fpv_frame(self, confidence: float, state: str) -> bytes:
        if self._fpv_frames:
            return self._fpv_frames[self._frame_index % len(self._fpv_frames)]

        if Image is None or ImageDraw is None:
            return _PLACEHOLDER_JPEG

        width, height = 640, 360
        image = Image.new("RGB", (width, height), (97, 143, 185))
        draw = ImageDraw.Draw(image)

        bob = math.sin(self._walk_distance * 11.0)
        sway = math.sin(self._walk_distance * 4.5)
        horizon = int(132 + 10 * bob + 9 * math.sin(self._yaw * 0.65))

        for y in range(0, max(1, horizon)):
            t = y / max(1, horizon)
            draw.line((0, y, width, y), fill=_lerp_color((76, 124, 182), (174, 208, 229), t))

        sun_x = int(width * 0.74 + 18 * math.sin(self._phase * 0.2))
        sun_y = int(42 + 5 * math.sin(self._phase * 0.3))
        draw.ellipse((sun_x - 18, sun_y - 18, sun_x + 18, sun_y + 18), fill=(246, 222, 132))

        draw.rectangle((0, horizon - 6, width, horizon + 8), fill=(71, 116, 63))
        for i in range(18):
            tx = int((i * 61 + self._phase * 8) % (width + 90) - 45)
            trunk_h = 22 + (i % 4) * 7
            draw.rectangle((tx - 2, horizon - trunk_h, tx + 2, horizon + 2), fill=(80, 68, 48))
            draw.ellipse((tx - 18, horizon - trunk_h - 18, tx + 18, horizon - trunk_h + 13), fill=(57, 103, 55))

        ground_top = horizon + 8
        for y in range(ground_top, height):
            t = (y - ground_top) / max(1, height - ground_top)
            draw.line((0, y, width, y), fill=_lerp_color((66, 112, 50), (33, 74, 37), t))

        vanishing_x = int(width * 0.50 + 42 * math.sin(self._yaw) + 18 * sway)
        path_bottom_half = 112
        path_top_half = 18
        path = [
            (vanishing_x - path_top_half, ground_top + 6),
            (vanishing_x + path_top_half, ground_top + 6),
            (width // 2 + path_bottom_half + int(18 * sway), height),
            (width // 2 - path_bottom_half + int(18 * sway), height),
        ]
        draw.polygon(path, fill=(122, 113, 78))
        for row in range(9):
            y = ground_top + 22 + row * 28 + int((self._walk_distance * 35) % 28)
            if y >= height:
                continue
            spread = (y - ground_top) / max(1, height - ground_top)
            half = int(path_top_half + spread * (path_bottom_half - path_top_half))
            cx_row = int(vanishing_x + spread * ((width // 2 + int(18 * sway)) - vanishing_x))
            draw.line((cx_row - half, y, cx_row + half, y + 4), fill=(98, 89, 61), width=2)

        for offset in (-3, -2, -1, 1, 2, 3):
            base_x = vanishing_x + offset * 52
            bottom_x = width // 2 + int(offset * 125 + 30 * sway)
            color = (43 + abs(offset) * 5, 96 + abs(offset) * 8, 42)
            draw.line((base_x, ground_top + 8, bottom_x, height), fill=color, width=3)
            draw.line((base_x + 18, ground_top + 8, bottom_x + 52, height), fill=(77, 127, 60), width=2)

        for i in range(70):
            t = ((i * 37 + int(self._walk_distance * 120)) % 100) / 100.0
            side = -1 if i % 2 == 0 else 1
            y = int(ground_top + t * (height - ground_top))
            spread = (y - ground_top) / max(1, height - ground_top)
            x = int(vanishing_x + side * (34 + spread * 290) + 24 * math.sin(i + self._phase))
            blade_h = int(7 + spread * 26)
            draw.line((x, y, x + side * 5, y - blade_h), fill=(93, 154, 65), width=1)

        # A small forearm/tablet hint makes the stream read like a wearable POV
        # instead of a generic texture, without adding privacy-sensitive imagery.
        arm_y = height - 58 + int(5 * bob)
        draw.polygon(
            [
                (width - 150, height),
                (width - 45, height),
                (width - 62, arm_y),
                (width - 124, arm_y - 8),
            ],
            fill=(82, 66, 54),
        )
        draw.rounded_rectangle((width - 138, arm_y - 30, width - 50, arm_y + 26), radius=8, fill=(22, 30, 36), outline=(118, 182, 197), width=2)
        draw.line((width - 122, arm_y - 10, width - 66, arm_y - 10), fill=(118, 220, 255), width=2)
        draw.line((width - 122, arm_y + 6, width - 78, arm_y + 6), fill=(210, 231, 225), width=2)

        cx, cy = width // 2, height // 2
        draw.line((cx - 38, cy, cx - 8, cy), fill=(120, 220, 255), width=3)
        draw.line((cx + 8, cy, cx + 38, cy), fill=(120, 220, 255), width=3)
        draw.line((cx, cy - 30, cx, cy - 8), fill=(120, 220, 255), width=3)
        draw.ellipse((cx - 58, cy - 58, cx + 58, cy + 58), outline=(120, 220, 255), width=2)

        heading_deg = int(math.degrees(self._yaw)) % 360
        conf_w = int(170 * max(0.0, min(1.0, confidence)))
        draw.rectangle((18, 18, 250, 88), fill=(12, 17, 24))
        draw.text((30, 28), f"HORUS FIELD FPV  {self._frame_index:04d}", fill=(150, 230, 255))
        draw.text((30, 48), f"{state.upper()}  yaw {heading_deg:03d} deg", fill=(235, 238, 228))
        draw.rectangle((30, 72, 200, 80), outline=(120, 220, 255))
        draw.rectangle((30, 72, 30 + conf_w, 80), fill=(120, 220, 255))

        draw.rectangle((width - 178, 18, width - 18, 78), fill=(12, 17, 24))
        draw.text((width - 166, 30), f"x {self._x:+.2f}  y {self._y:+.2f}", fill=(235, 238, 228))
        draw.text((width - 166, 52), "mock HoloLens stream", fill=(255, 204, 110))

        buffer = BytesIO()
        image.save(buffer, format="JPEG", quality=82, optimize=False)
        return buffer.getvalue()

    def _make_link_tf(
        self,
        now,
        child_leaf: str,
        xyz: tuple[float, float, float],
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        *,
        parent_frame: str | None = None,
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = parent_frame or self._base_frame
        transform.child_frame_id = f"{self._name}/{child_leaf}" if "/" not in child_leaf else child_leaf
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        qx, qy, qz, qw = _rpy_to_quaternion(*rpy)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform

    def _build_body_transforms(self, now, motion_state: str) -> tuple[list[TransformStamped], float, float]:
        moving = motion_state == "walking"
        gait = self._walk_distance * 10.5
        gait_amp = 1.0 if moving else 0.18
        bob = (0.024 * gait_amp) * abs(math.sin(gait))
        torso_pitch = (0.035 * gait_amp) * math.sin(gait + 0.35)
        head_yaw = 0.34 * math.sin(self._phase * 0.72) + (0.09 * gait_amp) * math.sin(gait * 0.5)
        head_pitch = 0.16 * math.sin(self._phase * 0.93 + 0.7)
        transforms: list[TransformStamped] = []
        adjusted_layout = {
            link: [float(xyz[0]), float(xyz[1]), float(xyz[2])]
            for link, xyz in self._body_layout.items()
        }
        if "pelvis" in adjusted_layout:
            adjusted_layout["pelvis"][2] += bob

        rotations: dict[str, tuple[float, float, float]] = {
            "pelvis": (0.018 * gait_amp * math.sin(gait + 0.15), 0.0, 0.018 * gait_amp * math.sin(gait)),
            "torso": (0.0, torso_pitch, 0.0),
            "chest_vest": (0.0, torso_pitch * 0.45, 0.0),
            "neck": (0.0, head_pitch * 0.25, head_yaw * 0.40),
            "head": (0.0, head_pitch, head_yaw),
        }

        for side_name, side in (("left", 1.0), ("right", -1.0)):
            arm_phase = gait + (math.pi if side > 0 else 0.0)
            arm_swing = (0.42 * gait_amp) * math.sin(arm_phase)
            rotations[f"{side_name}_upper_arm"] = (
                side * (0.18 + 0.05 * math.sin(gait)),
                arm_swing,
                side * 0.08,
            )
            rotations[f"{side_name}_forearm"] = (
                side * 0.06,
                -0.32 + (0.20 * gait_amp) * max(0.0, -math.sin(arm_phase + 0.35)),
                0.0,
            )
            rotations[f"{side_name}_hand"] = (
                0.04 * math.sin(gait + 0.90),
                side * 0.08,
                0.06 * math.sin(gait + 0.30),
            )

            leg_phase = gait + (0.0 if side > 0 else math.pi)
            leg_lift = max(0.0, math.sin(leg_phase))
            rotations[f"{side_name}_thigh"] = (0.0, (0.46 * gait_amp) * math.sin(leg_phase), 0.0)
            rotations[f"{side_name}_shin"] = (0.0, (-0.56 * gait_amp) * leg_lift, 0.0)
            rotations[f"{side_name}_foot"] = (0.0, (-0.20 * gait_amp) * math.sin(leg_phase + 0.35), 0.0)
            if f"{side_name}_foot" in adjusted_layout:
                adjusted_layout[f"{side_name}_foot"][0] += 0.035 * gait_amp * math.sin(leg_phase)
                adjusted_layout[f"{side_name}_foot"][2] += 0.016 * gait_amp * leg_lift

        skeleton = [
            ("pelvis", None),
            ("torso", "pelvis"),
            ("chest_vest", "torso"),
            ("neck", "chest_vest"),
            ("head", "neck"),
            ("left_upper_arm", "chest_vest"),
            ("left_forearm", "left_upper_arm"),
            ("left_hand", "left_forearm"),
            ("right_upper_arm", "chest_vest"),
            ("right_forearm", "right_upper_arm"),
            ("right_hand", "right_forearm"),
            ("left_thigh", "pelvis"),
            ("left_shin", "left_thigh"),
            ("left_foot", "left_shin"),
            ("right_thigh", "pelvis"),
            ("right_shin", "right_thigh"),
            ("right_foot", "right_shin"),
        ]

        for link, parent in skeleton:
            if link not in adjusted_layout:
                continue
            x, y, z = adjusted_layout[link]
            if parent and parent in adjusted_layout:
                px, py, pz = adjusted_layout[parent]
                xyz = (x - px, y - py, z - pz)
                parent_frame = f"{self._name}/{parent}"
            else:
                xyz = (x, y, z)
                parent_frame = self._base_frame
            transforms.append(
                self._make_link_tf(
                    now,
                    link,
                    xyz,
                    rotations.get(link, (0.0, 0.0, 0.0)),
                    parent_frame=parent_frame,
                )
            )

        return transforms, head_yaw, head_pitch

    def _tick(self) -> None:
        now = self.get_clock().now().to_msg()
        x, y, yaw, motion_state = self._update_motion(self._frame_period)
        if self._state != "acting":
            self._state = motion_state
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)

        base_tf = TransformStamped()
        base_tf.header.stamp = now
        base_tf.header.frame_id = self._map_frame
        base_tf.child_frame_id = self._base_frame
        base_tf.transform.translation.x = x
        base_tf.transform.translation.y = y
        base_tf.transform.translation.z = 0.0
        base_tf.transform.rotation.x = qx
        base_tf.transform.rotation.y = qy
        base_tf.transform.rotation.z = qz
        base_tf.transform.rotation.w = qw

        body_transforms, _, _ = self._build_body_transforms(now, motion_state)

        camera_tf = TransformStamped()
        camera_tf.header.stamp = now
        camera_tf.header.frame_id = f"{self._name}/head"
        camera_tf.child_frame_id = self._camera_frame
        camera_tf.transform.translation.x = 0.105
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = 0.018
        # Keep the TF orientation on the known-good forward-facing camera path.
        # Image roll is adjusted by the registration metadata's view_rotation_offset;
        # do not encode panel-roll fixes as TF yaw/pitch changes.
        cqx, cqy, cqz, cqw = _rpy_to_quaternion(-math.pi * 0.5, 0.0, 0.0)
        camera_tf.transform.rotation.x = cqx
        camera_tf.transform.rotation.y = cqy
        camera_tf.transform.rotation.z = cqz
        camera_tf.transform.rotation.w = cqw

        self._tf_broadcaster.sendTransform([base_tf, *body_transforms, camera_tf])

        # Localization confidence oscillates through the HIGH/MEDIUM/LOW bands so
        # the confidence-gated guidance path can be exercised.
        confidence = 0.75 + 0.25 * math.sin(self._phase * 0.5)
        self._confidence_pub.publish(Float32(data=float(confidence)))

        self._status_pub.publish(
            String(
                data=json.dumps(
                    {
                        "teammate": self._name,
                        "state": self._state,
                        "battery": 0.9,
                        "localization_confidence": round(confidence, 3),
                        "map_position": {"x": round(x, 3), "y": round(y, 3), "z": 0.0},
                        "heading_yaw_rad": round(yaw, 3),
                        "view_frame": self._camera_frame,
                        "profile": {
                            "height_m": round(self._profile.normalized_height_m, 3),
                            "sex": self._profile.normalized_sex,
                        },
                        "fpv_source": "real" if self._fpv_frames else "synthetic",
                    }
                )
            )
        )

        fpv = CompressedImage()
        fpv.header.stamp = now
        fpv.header.frame_id = self._camera_frame
        fpv.format = "jpeg"
        fpv.data = self._build_fpv_frame(confidence, self._state)
        self._fpv_pub.publish(fpv)
        self._frame_index += 1

        self._publish_guidance_state()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--name", default="field_teammate_1", help="Teammate entity name/namespace.")
    parser.add_argument("--wearable", default="hololens2", help="Wearable device type.")
    parser.add_argument("--rate", type=float, default=15.0, help="Publish rate in Hz.")
    parser.add_argument("--profile-height", type=float, default=1.75, help="Human height in meters.")
    parser.add_argument(
        "--profile-sex",
        choices=("female", "male", "unspecified"),
        default="unspecified",
        help="Human profile sex used for body proportions.",
    )
    parser.add_argument(
        "--fpv-source",
        choices=("real", "synthetic", "auto"),
        default="real",
        help="FPV source. Default streams cached frames from a real CC-BY walking video.",
    )
    parser.add_argument(
        "--fpv-video",
        default="",
        help="Optional local video path or URL for the real FPV stream.",
    )
    parser.add_argument(
        "--fpv-cache-dir",
        default="",
        help="Optional cache directory for downloaded/extracted FPV frames.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = MockFieldTeammateNode(
        args.name,
        args.wearable,
        args.rate,
        profile_height=args.profile_height,
        profile_sex=args.profile_sex,
        fpv_source=args.fpv_source,
        fpv_video=args.fpv_video,
        fpv_cache_dir=args.fpv_cache_dir,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
