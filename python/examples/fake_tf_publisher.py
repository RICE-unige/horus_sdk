import argparse
import math
import random
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
except Exception as exc:
    print(f"ERROR: ROS 2 Python dependencies not available: {exc}")
    sys.exit(1)


def quaternion_from_yaw(yaw_rad):
    half = yaw_rad * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class RobotState:
    def __init__(self, name, x, y, vx, vy, yaw):
        self.name = name
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.yaw = yaw

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
        self.start_time = time.time()
        self.last_update_time = self.start_time
        self.random = random.Random(seed if seed is not None else time.time())

        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)

        self.robots = self._init_robots()

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._on_timer)

        if self.publish_static_frames:
            self._publish_static_frames()

        self.get_logger().info(
            f"Publishing fake TF for {len(self.robots)} robot(s) on /tf at {self.rate_hz} Hz"
        )
        self.get_logger().info(
            f"Robots: {', '.join(self.robot_names)} | mode={self.mode} | area={self.area_size}m | min_dist={self.min_distance}m"
        )

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
                    robots.append(RobotState(name, x, y, vx, vy, angle))
            if not placed:
                angle = self.random.uniform(0.0, math.tau)
                speed = self.max_speed * 0.5
                vx = math.cos(angle) * speed
                vy = math.sin(angle) * speed
                robots.append(RobotState(name, 0.0, 0.0, vx, vy, angle))

        return robots

    def _distance_xy(self, ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)

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
            if self.publish_camera:
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
        separation_gain = 2.0
        boundary_gain = 1.5

        accelerations = []
        for robot in self.robots:
            angle = self.random.uniform(0.0, math.tau)
            ax = math.cos(angle) * self.jitter
            ay = math.sin(angle) * self.jitter

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
            speed = robot.speed()
            if speed > self.max_speed:
                scale = self.max_speed / speed
                robot.vx *= scale
                robot.vy *= scale

            robot.x += robot.vx * dt
            robot.y += robot.vy * dt

        self._resolve_collisions()
        self._apply_bounds()

        for robot in self.robots:
            speed = robot.speed()
            if speed > 1e-3:
                robot.yaw = math.atan2(robot.vy, robot.vx)

    def _resolve_collisions(self):
        count = len(self.robots)
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
                    push = overlap * 0.5
                    nx = dx / dist
                    ny = dy / dist
                    a.x += nx * push
                    a.y += ny * push
                    b.x -= nx * push
                    b.y -= ny * push

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
    parser.add_argument("--robot-count", type=int, default=1, help="Number of robots")
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
        default=2.0,
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
        default=0.6,
        help="Maximum robot speed in meters/second",
    )
    parser.add_argument(
        "--jitter",
        type=float,
        default=0.8,
        help="Random wander acceleration strength",
    )
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument(
        "--no-static-frames",
        action="store_true",
        help="Disable static wheel/sensor frames",
    )
    parser.add_argument("--static-camera", action="store_true", help="Include a camera_link frame")
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
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
