"""Local display adapters, bounded camera encoding, and explicit velocity arbitration."""

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan, CameraInfo, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException
from std_msgs.msg import String
from lifecycle_msgs.srv import GetState

ROBOTS = ("carter1", "carter2", "carter3")


def encode(message):
    if message.encoding not in ("rgb8", "rgba8", "bgr8", "bgra8"):
        raise ValueError(f"Unsupported camera encoding {message.encoding}")
    channels = 4 if "a8" in message.encoding else 3
    if message.step < message.width * channels or len(message.data) < message.step * message.height:
        raise ValueError("Camera dimensions and stride exceed the payload")
    image = np.ndarray((message.height, message.width, channels), dtype=np.uint8,
                       buffer=message.data, strides=(message.step, channels, 1))
    if message.encoding.startswith("rgb"):
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR if channels == 4 else cv2.COLOR_RGB2BGR)
    elif channels == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    ok, payload = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise ValueError("JPEG encoder failed")
    return CompressedImage(header=message.header, format="jpeg", data=payload.tobytes())


def navigation_cloud(message, info, stride=4):
    """Project real depth for navigation; never replace/downsample mapping input."""
    if message.encoding != "32FC1" or message.width != info.width or message.height != info.height:
        raise ValueError("Depth format/dimensions do not match calibration")
    if message.step < message.width*4 or len(message.data) < message.step*message.height:
        raise ValueError("Depth stride exceeds payload")
    fx,fy,cx,cy = info.k[0],info.k[4],info.k[2],info.k[5]
    if not all(math.isfinite(v) for v in (fx,fy,cx,cy)) or min(fx,fy) <= 0:
        raise ValueError("Invalid depth intrinsics")
    depth = np.ndarray((message.height,message.width),np.dtype(">f4" if message.is_bigendian else "<f4"),
                       buffer=message.data,strides=(message.step,4))[::stride,::stride]
    v,u = np.mgrid[0:message.height:stride,0:message.width:stride]
    valid = np.isfinite(depth) & (depth>.05) & (depth<8.0)
    z = depth[valid]
    points = np.column_stack(((u[valid]-cx)*z/fx,(v[valid]-cy)*z/fy,z)).astype("<f4")
    fields = [PointField(name=name,offset=index*4,datatype=PointField.FLOAT32,count=1)
              for index,name in enumerate(("x","y","z"))]
    return PointCloud2(header=deepcopy(message.header),height=1,width=len(points),fields=fields,
        is_bigendian=False,point_step=12,row_step=len(points)*12,data=points.tobytes(),is_dense=True)


class HospitalSupport(Node):
    def __init__(self):
        super().__init__("hospital_display_and_control")
        self.pool = ThreadPoolExecutor(max_workers=3, thread_name_prefix="hospital-jpeg")
        self.pending = {}
        self.manual = {}
        self.navigation = {}
        self.calibration = {}
        self.last_depth = {}
        self.paths = {name: deque(maxlen=2000) for name in ROBOTS}
        self.subs = []
        latch = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub = self.create_publisher(OccupancyGrid, "/shared_map", latch)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.subs.append(self.create_subscription(OccupancyGrid, "/carter1/map", self.map_pub.publish, latch))
        self.subs.append(self.create_subscription(TFMessage, "/carter1/tf", self.forward_map_tf, 10))
        self.camera_pubs = {name: self.create_publisher(CompressedImage, f"/{name}/camera/color/image_raw/compressed", 2) for name in ROBOTS}
        self.scan_pubs = {name: self.create_publisher(LaserScan, f"/{name}/slam/scan", 5) for name in ROBOTS}
        self.command_pubs = {name: self.create_publisher(Twist, f"/{name}/cmd_vel", 2) for name in ROBOTS}
        self.path_pubs = {name: self.create_publisher(Path, f"/{name}/trajectory", 2) for name in ROBOTS}
        self.plan_pubs = {name: self.create_publisher(Path, f"/{name}/plan_global", 2) for name in ROBOTS}
        self.obstacle_pubs = {name: self.create_publisher(PointCloud2, f"/{name}/camera/obstacles", 2) for name in ROBOTS}
        self.cancel_pubs = {name: self.create_publisher(String, f"/{name}/horus/goal_cancel", 2) for name in ROBOTS}
        self.buffers = {name: Buffer() for name in ROBOTS}
        self.info_pubs = {name: [self.create_publisher(CameraInfo, f"/{name}/camera/{kind}/camera_info", 2)
                                 for kind in ("color", "depth")] for name in ROBOTS}
        self.origins = {}
        self.slam_ready = set()
        self.slam_futures = {}
        self.slam_clients = {name: self.create_client(GetState, f"/{name}/slam_toolbox/get_state") for name in ROBOTS}
        self.readiness_timer = self.create_timer(.5, self.check_slam_ready)
        self.subs.append(self.create_subscription(TFMessage, "/tf_static", self.static_frames, latch))
        for name in ROBOTS:
            self.subs += [
                self.create_subscription(Image, f"/{name}/camera/color/image_raw", lambda m, n=name: self.image(n, m), 1),
                self.create_subscription(Image, f"/{name}/camera/depth/image_raw", lambda m,n=name: self.depth(n,m), 1),
                self.create_subscription(LaserScan, f"/{name}/scan", lambda m, n=name: self.local_scan(n, m), 2),
                self.create_subscription(Twist, f"/{name}/teleop_cmd_vel", lambda m, n=name: self.velocity(n, m, True), 2),
                self.create_subscription(Twist, f"/{name}/nav_cmd_vel", lambda m, n=name: self.velocity(n, m, False), 2),
                self.create_subscription(Odometry, f"/{name}/odom", lambda m, n=name: self.odometry(n, m), 2),
                self.create_subscription(CameraInfo, f"/{name}/camera/camera_info", lambda m, n=name: self.camera_info(n, m), 2),
                self.create_subscription(TFMessage, f"/{name}/tf", lambda m, n=name: self.local_transforms(n, m, False), 10),
                self.create_subscription(TFMessage, f"/{name}/tf_static", lambda m, n=name: self.local_transforms(n, m, True), latch),
                self.create_subscription(Path, f"/{name}/plan", lambda m, n=name: self.plan(n, m), 2),
            ]
        self.timer = self.create_timer(0.02, self.flush)

    def forward_map_tf(self, message):
        transforms = [value for value in message.transforms
                      if value.header.frame_id == "map" and value.child_frame_id == "global_odom"]
        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    def local_scan(self, name, message):
        if len(self.slam_ready) != len(ROBOTS):
            return
        local = deepcopy(message)
        local.header.frame_id = "laser"
        self.scan_pubs[name].publish(local)

    def check_slam_ready(self):
        for name, client in self.slam_clients.items():
            if name in self.slam_ready:
                continue
            pending = self.slam_futures.get(name)
            if pending is not None:
                future, started = pending
                if not future.done():
                    if time.monotonic()-started >= 3:
                        client.remove_pending_request(future)
                        future.cancel()
                        del self.slam_futures[name]
                        self.get_logger().warning(f"{name}: SLAM readiness reply timed out; retrying",throttle_duration_sec=5)
                    continue
                try:
                    if future.result().current_state.id == 3:
                        self.slam_ready.add(name)
                except Exception as error:
                    self.get_logger().warning(f"{name}: SLAM state unavailable: {error}")
                del self.slam_futures[name]
            elif client.service_is_ready():
                self.slam_futures[name] = (client.call_async(GetState.Request()), time.monotonic())

    def image(self, name, message):
        if (self.camera_pubs[name].get_subscription_count() > 0
                and name not in self.pending):
            self.pending[name] = self.pool.submit(encode, message)

    def camera_info(self, name, message):
        self.calibration[name] = message
        for publisher in self.info_pubs[name]:
            publisher.publish(message)

    def depth(self, name, message):
        info = self.calibration.get(name)
        timestamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        previous = self.last_depth.get(name,-math.inf)
        if info is None or 0 <= timestamp-previous < .2:
            return
        try:
            cloud = navigation_cloud(message,info)
            cloud.header.frame_id = "camera_optical"  # Nav2 has an isolated, local-name TF tree.
            self.obstacle_pubs[name].publish(cloud)
            self.last_depth[name] = timestamp
        except ValueError as error:
            self.get_logger().warning(f"{name}: navigation depth rejected: {error}",throttle_duration_sec=5)

    def static_frames(self, message):
        for value in message.transforms:
            if value.header.frame_id == "global_odom" and value.child_frame_id.endswith("/odom"):
                self.origins[value.child_frame_id.split("/")[0]] = value.transform

    def odometry(self, name, message):
        origin = self.origins.get(name)
        if origin is None:
            return
        q = origin.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        pose = PoseStamped()
        pose.header = deepcopy(message.header)
        pose.header.frame_id = "global_odom"
        p = message.pose.pose.position
        pose.pose.position.x = origin.translation.x + math.cos(yaw)*p.x - math.sin(yaw)*p.y
        pose.pose.position.y = origin.translation.y + math.sin(yaw)*p.x + math.cos(yaw)*p.y
        pose.pose.position.z = p.z
        pose.pose.orientation.w = 1.0
        previous = self.paths[name][-1] if self.paths[name] else None
        if previous and math.hypot(previous.pose.position.x - pose.pose.position.x,
                                   previous.pose.position.y - pose.pose.position.y) < 0.04:
            return
        self.paths[name].append(pose)
        self.path_pubs[name].publish(Path(header=pose.header, poses=list(self.paths[name])))

    def velocity(self, name, message, manual):
        if not all(math.isfinite(value) for value in (message.linear.x, message.angular.z)):
            return
        now = time.monotonic()
        if manual and now - self.manual.get(name, (None, -math.inf))[1] >= .35:
            self.cancel_pubs[name].publish(String(data="manual_override"))
            self.navigation.pop(name, None)
        (self.manual if manual else self.navigation)[name] = (message, now)

    def local_transforms(self, name, message, static):
        for value in message.transforms:
            if static:
                self.buffers[name].set_transform_static(value, "hospital")
            else:
                self.buffers[name].set_transform(value, "hospital")

    def plan(self, name, message):
        if not message.header.frame_id:
            return
        try:
            value = self.buffers[name].lookup_transform("global_odom", message.header.frame_id, rclpy.time.Time())
        except TransformException:
            return
        output = deepcopy(message)
        output.header.frame_id = "global_odom"
        q = value.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        for pose in output.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            pose.header.frame_id = "global_odom"
            pose.pose.position.x = value.transform.translation.x + cosine*x - sine*y
            pose.pose.position.y = value.transform.translation.y + sine*x + cosine*y
            pose.pose.position.z += value.transform.translation.z
            old = pose.pose.orientation
            half_sine, half_cosine = math.sin(yaw/2), math.cos(yaw/2)
            old.z, old.w = half_cosine*old.z + half_sine*old.w, half_cosine*old.w - half_sine*old.z
        self.plan_pubs[name].publish(output)

    def flush(self):
        for name, future in list(self.pending.items()):
            if future.done():
                del self.pending[name]
                try:
                    self.camera_pubs[name].publish(future.result())
                except Exception as error:
                    self.get_logger().error(f"{name}: camera rejected: {error}")
        now = time.monotonic()
        for name in ROBOTS:
            manual, manual_time = self.manual.get(name, (None, -math.inf))
            navigation, nav_time = self.navigation.get(name, (None, -math.inf))
            command = manual if now - manual_time < 0.35 else navigation if now - nav_time < 0.35 else Twist()
            self.command_pubs[name].publish(command)

    def destroy_node(self):
        for publisher in self.command_pubs.values():
            publisher.publish(Twist())
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    rclpy.init()
    node = HospitalSupport()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
