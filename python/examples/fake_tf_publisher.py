import argparse
import io
import math
import random
import sys
import time

try:
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CompressedImage, Image
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)

try:
    import cv2
except Exception:
    cv2 = None

try:
    import numpy as np
except Exception:
    np = None

try:
    from PIL import Image as PILImage
except Exception:
    PILImage = None


def quaternion_from_yaw(yaw_rad):
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def parse_resolution_list(raw_value):
    if not raw_value:
        return []

    resolutions = []
    for token in raw_value.split(","):
        item = token.strip().lower()
        if not item:
            continue
        if "x" not in item:
            continue
        left, right = item.split("x", 1)
        try:
            width = max(16, int(left))
            height = max(16, int(right))
        except ValueError:
            continue
        resolutions.append((width, height))
    return resolutions


class RobotState:
    def __init__(self, name, x, y, vx, vy, yaw):
        self.name = name
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.yaw = yaw
        self.noise_ax = 0.0
        self.noise_ay = 0.0
        self.target_x = x
        self.target_y = y

    def speed(self):
        return math.hypot(self.vx, self.vy)


class FakeTFPublisher(Node):
    def __init__(
        self,
        robot_names,
        map_frame,
        base_frame,
        rate_hz,
        height,
        scale,
        mode,
        radius,
        omega,
        area_size,
        min_distance,
        max_speed,
        jitter,
        seed,
        publish_static_frames,
        publish_camera,
        publish_images,
        publish_compressed_images,
        image_rate_hz,
        image_width,
        image_height,
        image_reliable,
        jpeg_quality,
        vary_image_resolution,
        image_resolutions,
    ):
        super().__init__("horus_fake_tf_publisher")
        self.robot_names = robot_names
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.rate_hz = max(rate_hz, 0.1)
        self.height = height
        self.scale = scale
        self.mode = mode
        self.radius = radius
        self.omega = omega
        self.area_size = max(area_size, 0.1)
        self.min_distance = max(min_distance, 0.1)
        self.max_speed = max(max_speed, 0.01)
        self.jitter = max(jitter, 0.0)
        self.publish_static_frames = publish_static_frames
        self.publish_camera = publish_camera
        self.publish_raw_images = publish_images
        self.publish_compressed_images = publish_compressed_images
        self.image_rate_hz = max(image_rate_hz, 0.1)
        self.image_width = max(16, int(image_width))
        self.image_height = max(16, int(image_height))
        self.image_reliable = image_reliable
        self.jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self.vary_image_resolution = vary_image_resolution
        self.image_resolutions = image_resolutions or []
        self.start_time = time.time()
        self.last_update_time = self.start_time
        self.random = random.Random(seed if seed is not None else time.time())
        self._image_sequences = {}
        self._compressed_sequences = {}
        self._robot_image_size = {}
        self._image_frame_index = 0
        self._image_palette = (
            (220, 70, 70),   # red-ish
            (70, 190, 90),   # green-ish
            (70, 130, 230),  # blue-ish
            (210, 180, 70),  # amber-ish
            (180, 80, 200),  # magenta-ish
            (70, 200, 200),  # cyan-ish
        )

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)

        self.robots = self._init_robots()
        self._initialize_robot_image_sizes()

        self._tf_callback_group = ReentrantCallbackGroup()
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(
            period,
            self._on_timer,
            callback_group=self._tf_callback_group,
        )

        if self.publish_static_frames:
            self._publish_static_frames()

        self.image_publishers = {}
        self.compressed_image_publishers = {}
        self.image_timer = None

        if self.publish_compressed_images and not self._has_jpeg_encoder():
            self.get_logger().warning(
                "Compressed image requested, but no JPEG encoder found (requires OpenCV+NumPy or Pillow). Falling back to raw images."
            )
            self.publish_compressed_images = False
            if not self.publish_raw_images:
                self.publish_raw_images = True

        if self.publish_raw_images or self.publish_compressed_images:
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
                    topic = f"/{name}/camera/image_raw"
                    self.image_publishers[name] = self.create_publisher(Image, topic, image_qos)
                if self.publish_compressed_images:
                    compressed_topic = f"/{name}/camera/image_raw/compressed"
                    self.compressed_image_publishers[name] = self.create_publisher(
                        CompressedImage, compressed_topic, image_qos
                    )

            self._prepare_image_sequences(phase_count=16)
            if self.publish_compressed_images:
                self._prepare_compressed_sequences()
            self._image_callback_group = ReentrantCallbackGroup()
            image_period = 1.0 / self.image_rate_hz
            self.image_timer = self.create_timer(
                image_period,
                self._on_image_timer,
                callback_group=self._image_callback_group,
            )

        self.get_logger().info(
            f"Publishing fake TF for {len(self.robots)} robot(s) on /tf at {self.rate_hz} Hz"
        )
        self.get_logger().info(
            f"Robots: {', '.join(self.robot_names)} | mode={self.mode} | area={self.area_size}m | min_dist={self.min_distance}m"
        )
        if self.publish_raw_images:
            qos_mode = "RELIABLE" if self.image_reliable else "BEST_EFFORT"
            self.get_logger().info(
                f"Publishing raw camera images on /<robot>/camera/image_raw at {self.image_rate_hz} Hz (rgb8, {qos_mode})"
            )
        if self.publish_compressed_images:
            qos_mode = "RELIABLE" if self.image_reliable else "BEST_EFFORT"
            self.get_logger().info(
                f"Publishing compressed camera images on /<robot>/camera/image_raw/compressed at {self.image_rate_hz} Hz (jpeg quality={self.jpeg_quality}, {qos_mode})"
            )
        if self.publish_raw_images or self.publish_compressed_images:
            detail = ", ".join(
                f"{name}:{size[0]}x{size[1]}"
                for name, size in self._robot_image_size.items()
            )
            self.get_logger().info(f"Per-robot camera resolution: {detail}")

    def _initialize_robot_image_sizes(self):
        if self.vary_image_resolution and self.image_resolutions:
            for idx, name in enumerate(self.robot_names):
                self._robot_image_size[name] = self.image_resolutions[idx % len(self.image_resolutions)]
            return

        for name in self.robot_names:
            self._robot_image_size[name] = (self.image_width, self.image_height)

    def _init_robots(self):
        robots = []
        attempts = 0
        max_attempts = 1000

        for name in self.robot_names:
            placed = False
            while not placed and attempts < max_attempts:
                attempts += 1
                x = self.random.uniform(-self.area_size, self.area_size)
                y = self.random.uniform(-self.area_size, self.area_size)
                if all(self._distance_xy(x, y, r.x, r.y) >= self.min_distance for r in robots):
                    placed = True
                    angle = self.random.uniform(0.0, math.tau)
                    speed = self.max_speed * 0.5
                    vx = math.cos(angle) * speed
                    vy = math.sin(angle) * speed
                    robot = RobotState(name, x, y, vx, vy, angle)
                    robot.target_x, robot.target_y = self._sample_patrol_target(x, y)
                    robots.append(robot)
            if not placed:
                angle = self.random.uniform(0.0, math.tau)
                speed = self.max_speed * 0.5
                vx = math.cos(angle) * speed
                vy = math.sin(angle) * speed
                robot = RobotState(name, 0.0, 0.0, vx, vy, angle)
                robot.target_x, robot.target_y = self._sample_patrol_target(0.0, 0.0)
                robots.append(robot)

        return robots

    def _distance_xy(self, ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)

    def _sample_patrol_target(self, around_x, around_y):
        max_offset = self.area_size * 0.6
        target_x = around_x + self.random.uniform(-max_offset, max_offset)
        target_y = around_y + self.random.uniform(-max_offset, max_offset)
        clamp = self.area_size * 0.9
        target_x = max(-clamp, min(clamp, target_x))
        target_y = max(-clamp, min(clamp, target_y))
        return target_x, target_y

    def _publish_static_frames(self):
        now = self.get_clock().now().to_msg()
        transforms = []

        for name in self.robot_names:
            parent = f"{name}/{self.base_frame}"
            static_children = [
                ("left_wheel", (0.0, 0.25, -0.05), 0.0),
                ("right_wheel", (0.0, -0.25, -0.05), 0.0),
                ("lidar_link", (0.2, 0.0, 0.2), 0.0),
                ("imu_link", (0.0, 0.0, 0.15), 0.0),
            ]
            if self.publish_camera or self.publish_raw_images or self.publish_compressed_images:
                static_children.append(("camera_link", (0.25, 0.0, 0.2), 0.0))

            for child, (x, y, z), yaw in static_children:
                msg = TransformStamped()
                msg.header.stamp = now
                msg.header.frame_id = parent
                msg.child_frame_id = f"{name}/{child}"
                msg.transform.translation.x = x * self.scale
                msg.transform.translation.y = y * self.scale
                msg.transform.translation.z = z * self.scale
                qx, qy, qz, qw = quaternion_from_yaw(yaw)
                msg.transform.rotation.x = qx
                msg.transform.rotation.y = qy
                msg.transform.rotation.z = qz
                msg.transform.rotation.w = qw
                transforms.append(msg)

        if transforms:
            self.tf_static_pub.publish(TFMessage(transforms=transforms))
            self.get_logger().info("Published static wheel/sensor TF frames")

    def _on_timer(self):
        now = self.get_clock().now().to_msg()
        current_time = time.time()
        dt = current_time - self.last_update_time
        if dt <= 0.0:
            dt = 1.0 / self.rate_hz
        dt = min(dt, 0.2)
        self.last_update_time = current_time

        if self.mode == "circle":
            self._update_circle(current_time)
        else:
            self._update_wander(dt)

        transforms = []
        for robot in self.robots:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = f"{robot.name}/{self.base_frame}"
            tf.transform.translation.x = robot.x * self.scale
            tf.transform.translation.y = robot.y * self.scale
            tf.transform.translation.z = self.height * self.scale
            qx, qy, qz, qw = quaternion_from_yaw(robot.yaw)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            transforms.append(tf)

        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def _update_circle(self, current_time):
        elapsed = current_time - self.start_time
        count = max(len(self.robots), 1)
        for idx, robot in enumerate(self.robots):
            angle = elapsed * self.omega + (math.tau * idx / count)
            robot.x = math.cos(angle) * self.radius
            robot.y = math.sin(angle) * self.radius
            robot.yaw = angle + math.pi * 0.5

    def _update_wander(self, dt):
        separation_gain = 1.2
        boundary_gain = 1.0
        target_gain = 0.9
        noise_gain = 1.0
        noise_smoothing = min(1.0, dt * 0.8)
        velocity_damping = max(0.0, 1.0 - (0.45 * dt))
        reach_distance = max(0.25, self.min_distance * 0.5)

        accelerations = []
        for robot in self.robots:
            target_dx = robot.target_x - robot.x
            target_dy = robot.target_y - robot.y
            target_dist = math.hypot(target_dx, target_dy)
            if target_dist < reach_distance:
                robot.target_x, robot.target_y = self._sample_patrol_target(robot.x, robot.y)
                target_dx = robot.target_x - robot.x
                target_dy = robot.target_y - robot.y
                target_dist = math.hypot(target_dx, target_dy)

            if target_dist > 1e-4:
                ax = (target_dx / target_dist) * target_gain
                ay = (target_dy / target_dist) * target_gain
            else:
                ax = 0.0
                ay = 0.0

            noise_angle = self.random.uniform(0.0, math.tau)
            noise_ax = math.cos(noise_angle) * self.jitter
            noise_ay = math.sin(noise_angle) * self.jitter
            robot.noise_ax += (noise_ax - robot.noise_ax) * noise_smoothing
            robot.noise_ay += (noise_ay - robot.noise_ay) * noise_smoothing
            ax += robot.noise_ax * noise_gain
            ay += robot.noise_ay * noise_gain

            for other in self.robots:
                if other is robot:
                    continue
                dx = robot.x - other.x
                dy = robot.y - other.y
                dist = math.hypot(dx, dy)
                if dist < self.min_distance:
                    if dist < 1e-4:
                        rand_angle = self.random.uniform(0.0, math.tau)
                        dx = math.cos(rand_angle)
                        dy = math.sin(rand_angle)
                        dist = 1.0
                    repulsion = (self.min_distance - dist) / self.min_distance
                    ax += (dx / dist) * repulsion * separation_gain
                    ay += (dy / dist) * repulsion * separation_gain

            if abs(robot.x) > self.area_size:
                ax += -math.copysign(boundary_gain, robot.x)
            if abs(robot.y) > self.area_size:
                ay += -math.copysign(boundary_gain, robot.y)

            accelerations.append((ax, ay))

        for robot, (ax, ay) in zip(self.robots, accelerations):
            robot.vx += ax * dt
            robot.vy += ay * dt
            robot.vx *= velocity_damping
            robot.vy *= velocity_damping
            speed = robot.speed()
            if speed > self.max_speed:
                scale = self.max_speed / speed
                robot.vx *= scale
                robot.vy *= scale

            robot.x += robot.vx * dt
            robot.y += robot.vy * dt

        self._resolve_collisions(dt)
        self._apply_bounds()

        for robot in self.robots:
            speed = robot.speed()
            if speed > 1e-3:
                desired_yaw = math.atan2(robot.vy, robot.vx)
                delta = math.atan2(
                    math.sin(desired_yaw - robot.yaw),
                    math.cos(desired_yaw - robot.yaw),
                )
                max_turn = 1.8 * dt
                if delta > max_turn:
                    delta = max_turn
                elif delta < -max_turn:
                    delta = -max_turn
                robot.yaw += delta

    def _resolve_collisions(self, dt):
        count = len(self.robots)
        max_push = max(0.02, self.max_speed * max(dt, 1.0 / self.rate_hz) * 0.6)
        for i in range(count):
            for j in range(i + 1, count):
                a = self.robots[i]
                b = self.robots[j]
                dx = a.x - b.x
                dy = a.y - b.y
                dist = math.hypot(dx, dy)
                if dist < 1e-4:
                    angle = self.random.uniform(0.0, math.tau)
                    dx = math.cos(angle)
                    dy = math.sin(angle)
                    dist = 1.0
                if dist < self.min_distance:
                    overlap = self.min_distance - dist
                    push = min(overlap * 0.5, max_push)
                    nx = dx / dist
                    ny = dy / dist
                    a.x += nx * push
                    a.y += ny * push
                    b.x -= nx * push
                    b.y -= ny * push

                    # Damp approach along collision normal to avoid repeated jittery corrections.
                    rel_vx = a.vx - b.vx
                    rel_vy = a.vy - b.vy
                    rel_n = (rel_vx * nx) + (rel_vy * ny)
                    if rel_n < 0.0:
                        impulse = -0.5 * rel_n
                        a.vx += nx * impulse
                        a.vy += ny * impulse
                        b.vx -= nx * impulse
                        b.vy -= ny * impulse

    def _apply_bounds(self):
        for robot in self.robots:
            if robot.x > self.area_size:
                robot.x = self.area_size
                robot.vx *= -0.3
            elif robot.x < -self.area_size:
                robot.x = -self.area_size
                robot.vx *= -0.3

            if robot.y > self.area_size:
                robot.y = self.area_size
                robot.vy *= -0.3
            elif robot.y < -self.area_size:
                robot.y = -self.area_size
                robot.vy *= -0.3

    def _on_image_timer(self):
        if not (self.publish_raw_images or self.publish_compressed_images):
            return

        stamp = self.get_clock().now().to_msg()

        for idx, robot in enumerate(self.robots):
            sequence = self._image_sequences.get(robot.name)
            if sequence:
                frame_idx = (self._image_frame_index + idx) % len(sequence)
            else:
                frame_idx = 0
            width, height = self._robot_image_size.get(robot.name, (self.image_width, self.image_height))
            raw_payload = sequence[frame_idx] if sequence else self._build_fake_image(
                robot_index=idx,
                elapsed=0.0,
                width=width,
                height=height,
            )

            if self.publish_raw_images:
                publisher = self.image_publishers.get(robot.name)
                if publisher is not None:
                    msg = Image()
                    msg.header.stamp = stamp
                    msg.header.frame_id = f"{robot.name}/camera_link"
                    msg.height = height
                    msg.width = width
                    msg.encoding = "rgb8"
                    msg.is_bigendian = 0
                    msg.step = width * 3
                    msg.data = raw_payload
                    publisher.publish(msg)

            if self.publish_compressed_images:
                compressed_publisher = self.compressed_image_publishers.get(robot.name)
                if compressed_publisher is not None:
                    compressed_sequence = self._compressed_sequences.get(robot.name)
                    if compressed_sequence:
                        compressed_payload = compressed_sequence[frame_idx % len(compressed_sequence)]
                    else:
                        compressed_payload = self._encode_jpeg(raw_payload, width, height)
                    if compressed_payload:
                        msg = CompressedImage()
                        msg.header.stamp = stamp
                        msg.header.frame_id = f"{robot.name}/camera_link"
                        msg.format = "jpeg"
                        msg.data = compressed_payload
                        compressed_publisher.publish(msg)

        self._image_frame_index += 1

    def _prepare_image_sequences(self, phase_count=16):
        phase_count = max(2, int(phase_count))

        for idx, name in enumerate(self.robot_names):
            width, height = self._robot_image_size.get(name, (self.image_width, self.image_height))
            max_x = max(width - 1, 1)
            max_y = max(height - 1, 1)
            base = self._build_base_image(robot_index=idx, width=width, height=height)
            sequence = []
            for phase in range(phase_count):
                frame = bytearray(base)
                marker_x = int((phase / phase_count) * max_x)
                marker_phase = (phase * 3 + idx * 5) % phase_count
                marker_y = int((marker_phase / phase_count) * max_y)
                self._draw_cross_marker(frame, width, height, marker_x, marker_y, thickness=2)
                sequence.append(bytes(frame))
            self._image_sequences[name] = sequence

    def _prepare_compressed_sequences(self):
        self._compressed_sequences = {}
        for name, sequence in self._image_sequences.items():
            width, height = self._robot_image_size.get(name, (self.image_width, self.image_height))
            encoded = []
            for frame in sequence:
                payload = self._encode_jpeg(frame, width, height)
                if payload:
                    encoded.append(payload)
            if encoded:
                self._compressed_sequences[name] = encoded

    def _has_jpeg_encoder(self):
        return (cv2 is not None and np is not None) or (PILImage is not None)

    def _encode_jpeg(self, rgb_bytes, width, height):
        if rgb_bytes is None:
            return None

        if cv2 is not None and np is not None:
            try:
                rgb = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((height, width, 3))
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                ok, encoded = cv2.imencode(
                    ".jpg",
                    bgr,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if ok:
                    return encoded.tobytes()
            except Exception:
                pass

        if PILImage is not None:
            try:
                image = PILImage.frombytes("RGB", (width, height), rgb_bytes)
                with io.BytesIO() as buffer:
                    image.save(buffer, format="JPEG", quality=self.jpeg_quality, optimize=False)
                    return buffer.getvalue()
            except Exception:
                pass

        return None

    def _build_base_image(self, robot_index, width, height):
        base_r, base_g, base_b = self._image_palette[robot_index % len(self._image_palette)]
        data = bytearray(width * height * 3)
        max_w = max(width - 1, 1)
        max_h = max(height - 1, 1)

        for y in range(height):
            row_start = y * width * 3
            y_term = (y * 120) // max_h
            for x in range(width):
                pixel = row_start + x * 3
                x_term = (x * 120) // max_w
                stripe = 20 if ((x // 24) % 2 == 0) else 0

                data[pixel] = min(255, (base_r // 2) + x_term + stripe)
                data[pixel + 1] = min(255, (base_g // 2) + y_term + stripe)
                data[pixel + 2] = min(255, (base_b // 2) + ((x_term + y_term) // 2))

        return bytes(data)

    def _draw_cross_marker(self, data, width, height, marker_x, marker_y, thickness=2):
        min_x = max(0, marker_x - thickness)
        max_x = min(width - 1, marker_x + thickness)
        min_y = max(0, marker_y - thickness)
        max_y = min(height - 1, marker_y + thickness)

        for y in range(height):
            if y < min_y or y > max_y:
                continue
            row_start = y * width * 3
            for x in range(width):
                pixel = row_start + x * 3
                data[pixel] = 255
                data[pixel + 1] = 255
                data[pixel + 2] = 255

        for y in range(height):
            row_start = y * width * 3
            for x in range(min_x, max_x + 1):
                pixel = row_start + x * 3
                data[pixel] = 255
                data[pixel + 1] = 255
                data[pixel + 2] = 255

    def _build_fake_image(self, robot_index, elapsed, width, height):
        base_r, base_g, base_b = self._image_palette[robot_index % len(self._image_palette)]

        horiz_shift = int((elapsed * 45.0) + (robot_index * 23)) % max(width, 1)
        marker_x = int((elapsed * 32.0) + (robot_index * 37)) % max(width, 1)
        marker_y = int((elapsed * 24.0) + (robot_index * 19)) % max(height, 1)

        data = bytearray(width * height * 3)
        max_w = max(width - 1, 1)
        max_h = max(height - 1, 1)

        for y in range(height):
            row_start = y * width * 3
            y_term = (y * 255) // max_h
            for x in range(width):
                pixel = row_start + x * 3
                x_term = (((x + horiz_shift) % width) * 255) // max_w
                stripe = 24 if (((x + horiz_shift) // 18) % 2) else 0

                r = min(255, (base_r // 2) + (x_term // 2) + stripe)
                g = min(255, (base_g // 2) + (y_term // 2) + stripe)
                b = min(255, (base_b // 2) + ((x_term + y_term) // 4))

                if abs(x - marker_x) <= 2 or abs(y - marker_y) <= 2:
                    r = 255
                    g = 255
                    b = 255

                data[pixel] = r
                data[pixel + 1] = g
                data[pixel + 2] = b

        return bytes(data)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Publish fake TF frames for Horus MR testing."
    )
    parser.add_argument(
        "--robot-name",
        default="test_bot",
        help="Robot name (single) or prefix (multi).",
    )
    parser.add_argument(
        "--robot-names",
        default="",
        help="Comma-separated list of robot names (overrides --robot-name/--robot-count).",
    )
    parser.add_argument("--robot-count", type=int, default=10, help="Number of robots")
    parser.add_argument("--map-frame", default="map", help="Fixed map frame name")
    parser.add_argument("--base-frame", default="base_link", help="Base frame name (no prefix)")
    parser.add_argument("--rate", type=float, default=30.0, help="Publish rate (Hz)")
    parser.add_argument("--height", type=float, default=0.0, help="Z height in meters")
    parser.add_argument("--scale", type=float, default=1.0, help="Position scale multiplier")
    parser.add_argument(
        "--mode",
        choices=["wander", "circle"],
        default="wander",
        help="Motion mode: wander (random) or circle.",
    )
    parser.add_argument("--radius", type=float, default=1.0, help="Circle radius in meters")
    parser.add_argument("--omega", type=float, default=0.5, help="Angular speed (rad/s)")
    parser.add_argument(
        "--area-size",
        type=float,
        default=6.0,
        help="Half-size of square motion area in meters",
    )
    parser.add_argument(
        "--min-distance",
        type=float,
        default=0.8,
        help="Minimum distance between robots in meters",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.45,
        help="Maximum robot speed in meters/second",
    )
    parser.add_argument(
        "--jitter",
        type=float,
        default=0.2,
        help="Random wander acceleration strength",
    )
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument(
        "--no-static-frames",
        action="store_true",
        help="Disable static wheel/sensor frames",
    )
    parser.add_argument("--static-camera", action="store_true", help="Include a camera_link frame")
    parser.add_argument(
        "--publish-images",
        dest="publish_images",
        action="store_true",
        default=False,
        help="Publish fake raw RGB images on /<robot>/camera/image_raw.",
    )
    parser.add_argument(
        "--no-publish-images",
        dest="publish_images",
        action="store_false",
        help="Disable fake raw image publishing.",
    )
    parser.add_argument(
        "--publish-compressed-images",
        dest="publish_compressed_images",
        action="store_true",
        default=True,
        help="Publish fake JPEG compressed images on /<robot>/camera/image_raw/compressed.",
    )
    parser.add_argument(
        "--no-publish-compressed-images",
        dest="publish_compressed_images",
        action="store_false",
        help="Disable fake compressed image publishing.",
    )
    parser.add_argument(
        "--image-rate",
        type=float,
        default=6.0,
        help="Camera image publish rate in Hz.",
    )
    parser.add_argument(
        "--image-width",
        type=int,
        default=160,
        help="Fake image width in pixels.",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=90,
        help="Fake image height in pixels.",
    )
    parser.add_argument(
        "--vary-image-resolution",
        action="store_true",
        default=True,
        help="Vary image resolution across robots (default: on).",
    )
    parser.add_argument(
        "--no-vary-image-resolution",
        dest="vary_image_resolution",
        action="store_false",
        help="Use a single shared image resolution for all robots.",
    )
    parser.add_argument(
        "--image-resolutions",
        default="160x90,192x108,224x126,256x144,320x180,426x240",
        help="Comma-separated WxH list used when varying resolution.",
    )
    parser.add_argument(
        "--image-best-effort",
        action="store_true",
        help="Use BEST_EFFORT QoS for image topics (default is RELIABLE).",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=65,
        help="JPEG quality for compressed images (30-95).",
    )
    return parser


def resolve_robot_names(args):
    if args.robot_names:
        names = [name.strip() for name in args.robot_names.split(",") if name.strip()]
        if names:
            return names

    if args.robot_count <= 1:
        return [args.robot_name]

    return [f"{args.robot_name}_{idx + 1}" for idx in range(args.robot_count)]


def main():
    parser = build_parser()
    args = parser.parse_args()

    rclpy.init()
    robot_names = resolve_robot_names(args)
    parsed_resolutions = parse_resolution_list(args.image_resolutions)
    if not parsed_resolutions:
        parsed_resolutions = [(args.image_width, args.image_height)]
    node = FakeTFPublisher(
        robot_names=robot_names,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        rate_hz=args.rate,
        height=args.height,
        scale=args.scale,
        mode=args.mode,
        radius=args.radius,
        omega=args.omega,
        area_size=args.area_size,
        min_distance=args.min_distance,
        max_speed=args.max_speed,
        jitter=args.jitter,
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
        vary_image_resolution=args.vary_image_resolution,
        image_resolutions=parsed_resolutions,
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
            try:
                executor.shutdown()
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
