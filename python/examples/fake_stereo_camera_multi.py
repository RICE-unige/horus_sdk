#!/usr/bin/env python3
"""Multi-robot fake TF publisher with high-quality SBS stereo camera streams."""

import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from fake_tf_teleop_common import (
    TeleopDrivenFakeTFPublisher,
    build_teleop_parser,
    parse_resolution_list,
    resolve_robot_names,
)


def _draw_rect(data, width, height, x0, y0, x1, y1, color):
    x0 = max(0, min(width - 1, int(x0)))
    y0 = max(0, min(height - 1, int(y0)))
    x1 = max(0, min(width - 1, int(x1)))
    y1 = max(0, min(height - 1, int(y1)))
    if x1 < x0 or y1 < y0:
        return

    r, g, b = color
    for y in range(y0, y1 + 1):
        row = y * width * 3
        for x in range(x0, x1 + 1):
            p = row + x * 3
            data[p] = r
            data[p + 1] = g
            data[p + 2] = b


def _draw_disc(data, width, height, cx, cy, radius, color):
    radius = max(1, int(radius))
    min_x = max(0, int(cx - radius))
    max_x = min(width - 1, int(cx + radius))
    min_y = max(0, int(cy - radius))
    max_y = min(height - 1, int(cy + radius))
    rr = radius * radius
    r, g, b = color
    for y in range(min_y, max_y + 1):
        dy = y - cy
        row = y * width * 3
        for x in range(min_x, max_x + 1):
            dx = x - cx
            if (dx * dx) + (dy * dy) > rr:
                continue
            p = row + x * 3
            data[p] = r
            data[p + 1] = g
            data[p + 2] = b


def _shade(color, factor):
    factor = max(0.0, factor)
    return (
        max(0, min(255, int(color[0] * factor))),
        max(0, min(255, int(color[1] * factor))),
        max(0, min(255, int(color[2] * factor))),
    )


def render_eye_scene(width, height, phase, eye_offset_x, base_color):
    data = bytearray(width * height * 3)
    horizon = int(height * 0.48)

    # Sky and floor gradients.
    for y in range(height):
        row = y * width * 3
        if y < horizon:
            t = y / max(1, horizon)
            r = int(22 + (64 * t))
            g = int(40 + (90 * t))
            b = int(70 + (130 * t))
        else:
            t = (y - horizon) / max(1, height - horizon - 1)
            r = int(36 + (38 * t))
            g = int(30 + (32 * t))
            b = int(24 + (24 * t))
        for x in range(width):
            p = row + x * 3
            data[p] = r
            data[p + 1] = g
            data[p + 2] = b

    fx = width * 0.95
    fy = height * 0.95
    cx = width * 0.5
    cy = height * 0.5

    static_objects = []

    # Grid markers on floor to make parallax obvious.
    for gz in range(2, 16):
        z = gz * 0.9
        for gx in range(-6, 7):
            static_objects.append(
                {
                    "kind": "disc",
                    "x": gx * 0.75,
                    "y": 0.0,
                    "z": z,
                    "size": 0.035,
                    "color": (155, 155, 155),
                }
            )

    # Simple room-like features.
    for idx in range(6):
        x = -3.0 + idx * 1.2
        static_objects.append(
            {
                "kind": "box",
                "x": x,
                "y": 0.4,
                "z": 5.0 + (idx % 2) * 2.2,
                "size": 0.38 + (idx % 3) * 0.08,
                "color": _shade(base_color, 0.75 + idx * 0.05),
            }
        )

    animated_objects = []
    for idx in range(4):
        angle = phase * 0.55 + idx * (math.pi * 0.5)
        radius = 1.8 + (0.35 * math.sin(phase * 0.35 + idx))
        x = math.cos(angle) * radius
        y = 0.35 + 0.25 * math.sin(phase * 0.8 + idx * 0.6)
        z = 3.5 + 0.7 * idx + 0.4 * math.sin(phase * 0.5 + idx)
        animated_objects.append(
            {
                "kind": "disc",
                "x": x,
                "y": y,
                "z": z,
                "size": 0.24 + 0.02 * idx,
                "color": (
                    min(255, base_color[0] + 22 * idx),
                    min(255, base_color[1] + 14 * (3 - idx)),
                    min(255, base_color[2] + 18 * idx),
                ),
            }
        )

    objects = static_objects + animated_objects
    objects.sort(key=lambda obj: obj["z"], reverse=True)

    for obj in objects:
        z = obj["z"]
        if z <= 0.1:
            continue

        x = obj["x"] - eye_offset_x
        y = obj["y"]
        u = int(cx + (fx * x / z))
        v = int(cy + (fy * (-y) / z))
        if u < -100 or u > (width + 100) or v < -100 or v > (height + 100):
            continue

        radius_px = max(1, int(fx * obj["size"] / z))
        shade = 0.72 + (0.28 * max(0.0, 1.0 - (z / 16.0)))
        color = _shade(obj["color"], shade)

        if obj["kind"] == "box":
            _draw_rect(
                data,
                width,
                height,
                u - radius_px,
                v - int(radius_px * 1.2),
                u + radius_px,
                v + int(radius_px * 1.1),
                color,
            )
        else:
            _draw_disc(data, width, height, u, v, radius_px, color)

    # Horizon highlight.
    _draw_rect(data, width, height, 0, horizon - 1, width - 1, horizon + 1, (168, 178, 186))
    return bytes(data)


class StereoTeleopFakeTFPublisher(TeleopDrivenFakeTFPublisher):
    def __init__(
        self,
        *,
        stereo_robot_count: int,
        stereo_baseline: float,
        stereo_eye_resolution: tuple[int, int],
        stereo_input_mode: str,
        highres_robot_name: str,
        highres_eye_resolution: tuple[int, int],
        highres_image_rate_hz: float,
        **kwargs,
    ):
        base_image_rate_hz = max(0.1, float(kwargs.get("image_rate_hz", 10.0)))
        self.highres_robot_name = (highres_robot_name or "").strip()
        self.highres_eye_resolution = (
            max(64, int(highres_eye_resolution[0])),
            max(64, int(highres_eye_resolution[1])),
        )
        self.highres_image_rate_hz = max(0.1, float(highres_image_rate_hz))
        self.default_image_rate_hz = base_image_rate_hz
        kwargs["image_rate_hz"] = max(base_image_rate_hz, self.highres_image_rate_hz)

        self.stereo_robot_count = max(0, int(stereo_robot_count))
        self.stereo_baseline = max(0.02, float(stereo_baseline))
        self.stereo_input_mode = (
            "dual_topic" if str(stereo_input_mode).strip().lower() == "dual_topic" else "sbs"
        )
        eye_width = max(64, int(stereo_eye_resolution[0]))
        eye_height = max(64, int(stereo_eye_resolution[1]))
        self.stereo_eye_resolution = (eye_width, eye_height)
        self.stereo_robots = []
        self.highres_robot = ""
        self.minimap_image_publishers = {}
        self.minimap_compressed_image_publishers = {}
        self.teleop_image_publishers = {}
        self.teleop_compressed_image_publishers = {}
        self.teleop_right_image_publishers = {}
        self.teleop_right_compressed_image_publishers = {}
        self._robot_minimap_image_size = {}
        self._right_image_sequences = {}
        self._teleop_left_image_sequences = {}
        self._minimap_image_sequences = {}
        self._right_compressed_sequences = {}
        self._minimap_compressed_sequences = {}
        self._robot_target_image_rate_hz = {}
        self._robot_next_minimap_time_s = {}
        self._robot_next_teleop_time_s = {}
        self._robot_frame_index = {}
        super().__init__(**kwargs)
        self._initialize_dual_stream_publishers()
        if self.highres_robot:
            highres_size = self._robot_image_size.get(self.highres_robot, self.highres_eye_resolution)
            if self.stereo_input_mode == "sbs" and self.highres_robot in self.stereo_robots:
                highres_size = (max(1, int(highres_size[0] // 2)), int(highres_size[1]))
            self.get_logger().info(
                "High-load stereo camera profile: "
                f"{self.highres_robot} eye={highres_size[0]}x{highres_size[1]} @ "
                f"{self._robot_target_image_rate_hz.get(self.highres_robot, self.highres_image_rate_hz):.1f} Hz"
            )

    def _initialize_dual_stream_publishers(self):
        if not (self.publish_raw_images or self.publish_compressed_images):
            return

        image_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
            reliability=(
                ReliabilityPolicy.RELIABLE
                if self.image_reliable
                else ReliabilityPolicy.BEST_EFFORT
            ),
        )

        for name in self.robot_names:
            if self.publish_raw_images:
                self.minimap_image_publishers[name] = self.create_publisher(
                    Image, f"/{name}/camera/minimap/image_raw", image_qos
                )
                teleop_topic = (
                    f"/{name}/camera/teleop/left/image_raw"
                    if self.stereo_input_mode == "dual_topic" and name in self.stereo_robots
                    else f"/{name}/camera/teleop/image_raw"
                )
                self.teleop_image_publishers[name] = self.create_publisher(Image, teleop_topic, image_qos)

            if self.publish_compressed_images:
                self.minimap_compressed_image_publishers[name] = self.create_publisher(
                    CompressedImage, f"/{name}/camera/minimap/image_raw/compressed", image_qos
                )
                teleop_topic = (
                    f"/{name}/camera/teleop/left/image_raw/compressed"
                    if self.stereo_input_mode == "dual_topic" and name in self.stereo_robots
                    else f"/{name}/camera/teleop/image_raw/compressed"
                )
                self.teleop_compressed_image_publishers[name] = self.create_publisher(
                    CompressedImage, teleop_topic, image_qos
                )

            if self.stereo_input_mode != "dual_topic" or name not in self.stereo_robots:
                continue

            if self.publish_raw_images:
                self.teleop_right_image_publishers[name] = self.create_publisher(
                    Image,
                    f"/{name}/camera/teleop/right/image_raw",
                    image_qos,
                )
            if self.publish_compressed_images:
                self.teleop_right_compressed_image_publishers[name] = self.create_publisher(
                    CompressedImage,
                    f"/{name}/camera/teleop/right/image_raw/compressed",
                    image_qos,
                )

        self.get_logger().info(
            "Publishing camera dual-stream topics: "
            "minimap=/camera/minimap/* (mono, <=30Hz), teleop=/camera/teleop/* (stereo/high-rate)."
        )

    def _initialize_robot_image_sizes(self):
        super()._initialize_robot_image_sizes()
        eye_width, eye_height = self.stereo_eye_resolution

        self.stereo_robots = self.robot_names[: min(self.stereo_robot_count, len(self.robot_names))]
        if self.highres_robot_name and self.highres_robot_name in self.robot_names:
            self.highres_robot = self.highres_robot_name
        elif self.robot_names:
            self.highres_robot = self.robot_names[0]
        else:
            self.highres_robot = ""

        for name in self.robot_names:
            if name == self.highres_robot:
                self._robot_image_size[name] = self.highres_eye_resolution
                self._robot_target_image_rate_hz[name] = self.highres_image_rate_hz
            else:
                self._robot_image_size[name] = (eye_width, eye_height)
                self._robot_target_image_rate_hz[name] = self.default_image_rate_hz
            self._robot_minimap_image_size[name] = (eye_width, eye_height)
            self._robot_next_minimap_time_s[name] = 0.0
            self._robot_next_teleop_time_s[name] = 0.0
            self._robot_frame_index[name] = 0

        if self.stereo_input_mode == "sbs":
            for name in self.stereo_robots:
                src_eye_width, src_eye_height = self._robot_image_size.get(name, (eye_width, eye_height))
                self._robot_image_size[name] = (src_eye_width * 2, src_eye_height)

    def _prepare_image_sequences(self, phase_count=12):
        phase_count = max(8, int(phase_count))
        self._image_sequences = {}
        self._right_image_sequences = {}

        for idx, name in enumerate(self.robot_names):
            width, height = self._robot_image_size.get(name, self.stereo_eye_resolution)
            base_color = self._image_palette[idx % len(self._image_palette)]
            is_stereo_robot = name in self.stereo_robots
            teleop_sequence = []
            teleop_left_sequence = []
            minimap_sequence = []
            right_sequence = []

            for phase_idx in range(phase_count):
                phase = (phase_idx / phase_count) * (math.pi * 2.0) + (idx * 0.35)
                if is_stereo_robot:
                    if self.stereo_input_mode == "sbs":
                        eye_width = max(1, width // 2)
                        eye_height = height
                    else:
                        eye_width = width
                        eye_height = height

                    half_baseline = self.stereo_baseline * 0.5
                    left = render_eye_scene(eye_width, eye_height, phase, -half_baseline, base_color)
                    right = render_eye_scene(eye_width, eye_height, phase, half_baseline, base_color)
                    teleop_left_sequence.append(left)
                    if self.stereo_input_mode == "dual_topic":
                        teleop_sequence.append(left)
                        right_sequence.append(right)
                    else:
                        merged = bytearray(width * height * 3)
                        for y in range(height):
                            dst_row = y * width * 3
                            src_row = y * eye_width * 3
                            merged[dst_row : dst_row + eye_width * 3] = left[
                                src_row : src_row + eye_width * 3
                            ]
                            merged[dst_row + eye_width * 3 : dst_row + width * 3] = right[
                                src_row : src_row + eye_width * 3
                            ]
                        teleop_sequence.append(bytes(merged))
                else:
                    mono_frame = render_eye_scene(width, height, phase, 0.0, base_color)
                    teleop_sequence.append(mono_frame)
                    teleop_left_sequence.append(mono_frame)

                minimap_width, minimap_height = self._robot_minimap_image_size.get(name, self.stereo_eye_resolution)
                minimap_sequence.append(
                    render_eye_scene(minimap_width, minimap_height, phase, 0.0, base_color)
                )

            self._image_sequences[name] = teleop_sequence
            self._teleop_left_image_sequences[name] = teleop_left_sequence
            self._minimap_image_sequences[name] = minimap_sequence
            if right_sequence:
                self._right_image_sequences[name] = right_sequence

    def _prepare_compressed_sequences(self):
        self._compressed_sequences = {}
        self._right_compressed_sequences = {}
        self._minimap_compressed_sequences = {}

        for name, sequence in self._image_sequences.items():
            width, height = self._robot_image_size.get(name, self.stereo_eye_resolution)
            encoded = []
            for frame in sequence:
                payload = self._encode_jpeg(frame, width, height)
                if payload:
                    encoded.append(payload)
            if encoded:
                self._compressed_sequences[name] = encoded

        for name, sequence in self._minimap_image_sequences.items():
            width, height = self._robot_minimap_image_size.get(name, self.stereo_eye_resolution)
            encoded = []
            for frame in sequence:
                payload = self._encode_jpeg(frame, width, height)
                if payload:
                    encoded.append(payload)
            if encoded:
                self._minimap_compressed_sequences[name] = encoded

        if self.stereo_input_mode != "dual_topic":
            return

        for name, sequence in self._right_image_sequences.items():
            eye_width, eye_height = self._robot_image_size.get(name, self.stereo_eye_resolution)
            encoded = []
            for frame in sequence:
                payload = self._encode_jpeg(frame, eye_width, eye_height)
                if payload:
                    encoded.append(payload)
            if encoded:
                self._right_compressed_sequences[name] = encoded

    def _on_image_timer(self):
        if not (self.publish_raw_images or self.publish_compressed_images):
            return

        stamp = self.get_clock().now().to_msg()
        now_s = time.monotonic()

        for idx, robot in enumerate(self.robots):
            robot_name = robot.name
            target_rate_hz = max(0.1, float(self._robot_target_image_rate_hz.get(robot_name, self.default_image_rate_hz)))
            minimap_rate_hz = min(30.0, target_rate_hz)

            next_minimap_time_s = self._robot_next_minimap_time_s.get(robot_name, 0.0)
            next_teleop_time_s = self._robot_next_teleop_time_s.get(robot_name, 0.0)
            publish_minimap = (next_minimap_time_s <= 0.0) or (now_s + 1e-6 >= next_minimap_time_s)
            publish_teleop = (next_teleop_time_s <= 0.0) or (now_s + 1e-6 >= next_teleop_time_s)
            if not publish_minimap and not publish_teleop:
                continue

            teleop_sequence = self._image_sequences.get(robot_name)
            minimap_sequence = self._minimap_image_sequences.get(robot_name)
            frame_index = int(self._robot_frame_index.get(robot_name, 0))
            if teleop_sequence:
                frame_idx = frame_index % len(teleop_sequence)
            else:
                frame_idx = 0

            teleop_width, teleop_height = self._robot_image_size.get(robot_name, self.stereo_eye_resolution)
            minimap_width, minimap_height = self._robot_minimap_image_size.get(robot_name, self.stereo_eye_resolution)
            teleop_raw_payload = teleop_sequence[frame_idx] if teleop_sequence else self._build_fake_image(
                robot_index=idx,
                elapsed=0.0,
                width=teleop_width,
                height=teleop_height,
            )
            minimap_raw_payload = (
                minimap_sequence[frame_idx % len(minimap_sequence)]
                if minimap_sequence
                else self._build_fake_image(
                    robot_index=idx,
                    elapsed=0.0,
                    width=minimap_width,
                    height=minimap_height,
                )
            )

            if publish_minimap and self.publish_raw_images:
                publisher = self.minimap_image_publishers.get(robot_name)
                if publisher is not None:
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = f"{robot_name}/camera_link"
                    msg.height = minimap_height
                    msg.width = minimap_width
                    msg.encoding = "rgb8"
                    msg.is_bigendian = 0
                    msg.step = minimap_width * 3
                    msg.data = minimap_raw_payload
                    publisher.publish(msg)

            if publish_minimap and self.publish_compressed_images:
                compressed_publisher = self.minimap_compressed_image_publishers.get(robot_name)
                if compressed_publisher is not None:
                    compressed_sequence = self._minimap_compressed_sequences.get(robot_name)
                    if compressed_sequence:
                        compressed_payload = compressed_sequence[frame_idx % len(compressed_sequence)]
                    else:
                        compressed_payload = self._encode_jpeg(minimap_raw_payload, minimap_width, minimap_height)
                    if compressed_payload:
                        msg = CompressedImage()
                        msg.header.stamp = stamp
                        msg.header.frame_id = f"{robot_name}/camera_link"
                        msg.format = "jpeg"
                        msg.data = compressed_payload
                        compressed_publisher.publish(msg)

            if publish_teleop and self.publish_raw_images:
                teleop_raw_publisher = self.teleop_image_publishers.get(robot_name)
                if teleop_raw_publisher is not None:
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = f"{robot_name}/camera_link"
                    msg.height = teleop_height
                    msg.width = teleop_width
                    msg.encoding = "rgb8"
                    msg.is_bigendian = 0
                    msg.step = teleop_width * 3
                    msg.data = teleop_raw_payload
                    teleop_raw_publisher.publish(msg)

            if publish_teleop and self.publish_compressed_images:
                teleop_compressed_publisher = self.teleop_compressed_image_publishers.get(robot_name)
                if teleop_compressed_publisher is not None:
                    compressed_sequence = self._compressed_sequences.get(robot_name)
                    if compressed_sequence:
                        compressed_payload = compressed_sequence[frame_idx % len(compressed_sequence)]
                    else:
                        compressed_payload = self._encode_jpeg(teleop_raw_payload, teleop_width, teleop_height)
                    if compressed_payload:
                        msg = CompressedImage()
                        msg.header.stamp = stamp
                        msg.header.frame_id = f"{robot_name}/camera_link"
                        msg.format = "jpeg"
                        msg.data = compressed_payload
                        teleop_compressed_publisher.publish(msg)

            if publish_teleop and self.stereo_input_mode == "dual_topic" and robot_name in self.stereo_robots:
                right_sequence = self._right_image_sequences.get(robot_name)
                if right_sequence:
                    right_frame = right_sequence[frame_idx % len(right_sequence)]
                    eye_width, eye_height = self._robot_image_size.get(robot_name, self.stereo_eye_resolution)

                    if self.publish_raw_images:
                        right_raw_publisher = self.teleop_right_image_publishers.get(robot_name)
                        if right_raw_publisher is not None:
                            msg = Image()
                            msg.header.stamp = stamp
                            msg.header.frame_id = f"{robot_name}/camera_right_link"
                            msg.height = eye_height
                            msg.width = eye_width
                            msg.encoding = "rgb8"
                            msg.is_bigendian = 0
                            msg.step = eye_width * 3
                            msg.data = right_frame
                            right_raw_publisher.publish(msg)

                    if self.publish_compressed_images:
                        right_compressed_publisher = self.teleop_right_compressed_image_publishers.get(robot_name)
                        if right_compressed_publisher is not None:
                            right_compressed_sequence = self._right_compressed_sequences.get(robot_name)
                            if right_compressed_sequence:
                                compressed_payload = right_compressed_sequence[frame_idx % len(right_compressed_sequence)]
                            else:
                                compressed_payload = self._encode_jpeg(right_frame, eye_width, eye_height)
                            if compressed_payload:
                                msg = CompressedImage()
                                msg.header.stamp = stamp
                                msg.header.frame_id = f"{robot_name}/camera_right_link"
                                msg.format = "jpeg"
                                msg.data = compressed_payload
                                right_compressed_publisher.publish(msg)

            if publish_minimap:
                self._robot_next_minimap_time_s[robot_name] = now_s + (1.0 / max(0.1, minimap_rate_hz))
            if publish_teleop:
                self._robot_next_teleop_time_s[robot_name] = now_s + (1.0 / target_rate_hz)

            self._robot_frame_index[robot_name] = frame_index + 1

    def _build_fake_image(self, robot_index, elapsed, width, height):
        base_color = self._image_palette[robot_index % len(self._image_palette)]
        robot_name = self.robot_names[robot_index]

        if robot_name not in self.stereo_robots or self.stereo_input_mode == "dual_topic":
            return render_eye_scene(width, height, elapsed, 0.0, base_color)

        eye_width = width // 2
        eye_height = height
        half_baseline = self.stereo_baseline * 0.5

        left = render_eye_scene(eye_width, eye_height, elapsed, -half_baseline, base_color)
        right = render_eye_scene(eye_width, eye_height, elapsed, half_baseline, base_color)

        merged = bytearray(width * height * 3)
        for y in range(height):
            dst_row = y * width * 3
            src_row = y * eye_width * 3
            merged[dst_row : dst_row + eye_width * 3] = left[src_row : src_row + eye_width * 3]
            merged[dst_row + eye_width * 3 : dst_row + width * 3] = right[src_row : src_row + eye_width * 3]
        return bytes(merged)


def main():
    parser = build_teleop_parser(default_robot_count=4)
    parser.description = "Publish multi-robot fake TF + high-quality SBS stereo camera data."
    parser.set_defaults(
        robot_name="stereo_bot",
        image_rate=10.0,
        image_width=960,
        image_height=540,
        vary_image_resolution=False,
        publish_images=False,
        publish_compressed_images=True,
    )
    parser.add_argument(
        "--stereo-robot-count",
        type=int,
        default=-1,
        help="Number of robots (from the front of the robot list) that publish stereo (default: all robots).",
    )
    parser.add_argument(
        "--stereo-input-mode",
        choices=["sbs", "dual_topic"],
        default="sbs",
        help="Stereo stream mode: SBS single-topic or dual_topic left/right topics.",
    )
    parser.add_argument(
        "--stereo-baseline",
        type=float,
        default=0.12,
        help="Stereo baseline in meters for fake left/right eye separation.",
    )
    parser.add_argument(
        "--stereo-eye-resolution",
        default="960x540",
        help="Per-eye resolution for stereo cameras (SBS total width is doubled).",
    )
    parser.add_argument(
        "--highres-robot",
        default="",
        help="Robot name to publish with a high-load camera profile (default: first robot).",
    )
    parser.add_argument(
        "--highres-eye-resolution",
        default="1920x1080",
        help="Per-eye resolution for the high-load robot (default: 1920x1080).",
    )
    parser.add_argument(
        "--highres-image-rate",
        type=float,
        default=90.0,
        help="Image publish rate for the high-load robot (default: 90.0 Hz).",
    )
    args = parser.parse_args()
    robot_names = resolve_robot_names(args)

    parsed_eye_res = parse_resolution_list(args.stereo_eye_resolution)
    if parsed_eye_res:
        eye_resolution = parsed_eye_res[0]
    else:
        eye_resolution = (960, 540)

    parsed_highres_eye_res = parse_resolution_list(args.highres_eye_resolution)
    if parsed_highres_eye_res:
        highres_eye_resolution = parsed_highres_eye_res[0]
    else:
        highres_eye_resolution = (1920, 1080)

    if int(args.stereo_robot_count) < 0:
        args.stereo_robot_count = len(robot_names)

    requested_highres_robot = (args.highres_robot or "").strip()
    highres_robot_name = requested_highres_robot if requested_highres_robot else (robot_names[0] if robot_names else "")
    if highres_robot_name and highres_robot_name not in robot_names:
        print(
            f"[fake_stereo_camera_multi] WARN: --highres-robot '{highres_robot_name}' is not in robot list; using '{robot_names[0] if robot_names else ''}'."
        )
        highres_robot_name = robot_names[0] if robot_names else ""

    effective_image_rate = max(float(args.image_rate), float(args.highres_image_rate))
    args.image_width = eye_resolution[0]
    args.image_height = eye_resolution[1]
    args.image_rate = effective_image_rate
    args.vary_image_resolution = False

    rclpy.init()
    node = StereoTeleopFakeTFPublisher(
        command_timeout_s=args.command_timeout,
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        rate_hz=args.rate,
        height=args.height,
        scale=args.scale,
        mode="teleop",
        radius=args.radius,
        omega=args.omega,
        area_size=args.area_size,
        min_distance=args.min_distance,
        max_speed=args.max_speed,
        jitter=0.0,
        seed=args.seed,
        publish_static_frames=not args.no_static_frames,
        publish_camera=args.static_camera,
        publish_images=args.publish_images,
        publish_compressed_images=args.publish_compressed_images,
        image_rate_hz=args.image_rate,
        image_width=args.image_width,
        image_height=args.image_height,
        image_reliable=not args.image_best_effort,
        jpeg_quality=args.jpeg_quality,
        vary_image_resolution=False,
        image_resolutions=[eye_resolution],
        publish_occupancy_grid=args.publish_occupancy_grid,
        occupancy_topic=args.occupancy_topic,
        occupancy_rate_hz=args.occupancy_rate,
        occupancy_resolution=args.occupancy_resolution,
        occupancy_width=args.occupancy_width,
        occupancy_height=args.occupancy_height,
        occupancy_unknown_ratio=args.occupancy_unknown_ratio,
        occupancy_obstacle_count=args.occupancy_obstacle_count,
        stereo_robot_count=args.stereo_robot_count,
        stereo_baseline=args.stereo_baseline,
        stereo_eye_resolution=eye_resolution,
        stereo_input_mode=args.stereo_input_mode,
        highres_robot_name=highres_robot_name,
        highres_eye_resolution=highres_eye_resolution,
        highres_image_rate_hz=args.highres_image_rate,
    )

    executor = None
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
