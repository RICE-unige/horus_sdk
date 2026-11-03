"""One paced bag clock for G1 poses, joints, optical camera, scan and live mesh.

Only the explicit observation whitelist is opened. No recorded command or clock
topic is published. Coordinates are unchanged; frame labels are isolated.
"""

from collections import Counter
from copy import copy, deepcopy
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from g1_recording_registration import CAMERA_FRAME, ROBOT, TOPIC

SOURCES = {
    "pelvis": ("/g1_spatial/pelvis_pose", "geometry_msgs/msg/PoseStamped"),
    "joints": ("/g1_spatial/joint_states", "sensor_msgs/msg/JointState"),
    "camera_pose": ("/g1_spatial/mapping/camera_pose", "geometry_msgs/msg/PoseStamped"),
    "camera_info": ("/g1_spatial/mapping/camera_info", "sensor_msgs/msg/CameraInfo"),
    "rgb": ("/g1_spatial/mapping/rgb", "sensor_msgs/msg/Image"),
    "scan": ("/g1_spatial/map_scan", "sensor_msgs/msg/PointCloud2"),
    "mesh": ("/g1_spatial_nvblox/mesh", "nvblox_msgs/msg/Mesh"),
}
MAP_FRAME = "g1_spatial/map"
OPTICAL_FRAME = "g1_spatial/rgb_optical"


def read_bag(path, *, include_mesh=True):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path), storage_id="mcap"), ConverterOptions("", ""))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    sources = {key: spec for key, spec in SOURCES.items() if key != "mesh" or include_mesh}
    for topic, expected in sources.values():
        if types.get(topic) != expected:
            raise ValueError(f"{topic}: expected {expected}, found {types.get(topic)}")
    selected = {topic: (key, get_message(type_)) for key, (topic, type_) in sources.items()}
    reader.set_filter(StorageFilter(topics=list(selected)))
    previous = None
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if previous is not None and stamp < previous:
            raise ValueError("Bag receipt timeline goes backwards")
        previous = stamp
        key, type_ = selected[topic]
        yield key, deserialize_message(data, type_), stamp


def validate_frame(key, message):
    expected = OPTICAL_FRAME if key in ("rgb", "camera_info") else MAP_FRAME
    if message.header.frame_id != expected:
        raise ValueError(f"{key}: unexpected frame {message.header.frame_id!r}; expected {expected!r}")


def inspect(path, urdf, dataset="later", map_mode="live"):
    from tools.g1_recording_data import camera_mount, read_source
    required = set(SOURCES) - ({"mesh"} if map_mode == "static" else set())
    first = {}
    for key, message, stamp in read_source(path, urdf, dataset, map_mode):
        validate_frame(key, message)
        first.setdefault(key, (message, stamp))
        if set(first) == required:
            break
    if set(first) != required:
        raise ValueError(f"Missing data: {required - set(first)}")
    root = ET.parse(urdf).getroot()
    movable = {j.attrib["name"] for j in root.findall("joint") if j.get("type") != "fixed"}
    recorded = set(first["joints"][0].name)
    if movable != recorded:
        raise ValueError(f"Joint/model mismatch: missing={movable-recorded}, extra={recorded-movable}")
    info = first["camera_info"][0]
    image = first["rgb"][0]
    if (image.width, image.height) != (info.width, info.height):
        raise ValueError("Recorded RGB dimensions do not match CameraInfo")
    if info.k[0] <= 0 or info.k[4] <= 0 or not all(math.isfinite(v) for v in info.k):
        raise ValueError("Invalid camera intrinsics")
    camera_parent, _, mount_source = camera_mount(path, dataset, urdf)
    return first, {"session": str(Path(path).parent), "width": info.width, "height": info.height,
                   "k": list(info.k), "joint_count": len(movable), "bag": str(path),
                   "camera_parent": camera_parent, "camera_mount_source": mount_source,
                   "dataset": dataset, "map_mode": map_mode}


def jpeg_message(image):
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    if image.encoding not in ("rgb8", "bgr8") or image.step < image.width * 3:
        raise ValueError(f"Unsupported recorded RGB layout: {image.encoding}/{image.step}")
    if len(image.data) != image.step * image.height:
        raise ValueError("Recorded RGB byte count does not match row stride")
    pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
    pixels = pixels[:, :image.width * 3].reshape(image.height, image.width, 3)
    if image.encoding == "rgb8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return CompressedImage(header=deepcopy(image.header), format="rgb8; jpeg compressed bgr8",
                           data=encoded.tobytes())


def pose_transform(pose, child, stamp):
    from geometry_msgs.msg import TransformStamped
    tf = TransformStamped(child_frame_id=child)
    tf.header.frame_id, tf.header.stamp = "map", stamp
    p = pose.pose.position
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
    tf.transform.rotation = deepcopy(pose.pose.orientation)
    return tf


def floor_transform(pose, initial_height, stamp):
    tf = pose_transform(pose, ROBOT + "/floor_anchor", stamp)
    tf.transform.translation.z -= initial_height + 0.05
    q = pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    tf.transform.rotation.x = tf.transform.rotation.y = 0.0
    tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return tf


def run(path, urdf, ready_file, dataset="later", map_mode="live"):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Path as RosPath
    from sensor_msgs.msg import CameraInfo, CompressedImage, JointState, PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage
    from nvblox_msgs.msg import Mesh

    from tools.g1_recording_data import camera_mount, matrix_pose, read_source
    first, info = inspect(path, urdf, dataset, map_mode)
    rclpy.init()
    node = rclpy.create_node("horus_g1_recording_replay")
    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers = {
        "joints": node.create_publisher(JointState, TOPIC + "/joint_states", 10),
        "rgb": node.create_publisher(CompressedImage, TOPIC + "/camera/compressed", retained),
        "camera_info": node.create_publisher(CameraInfo, TOPIC + "/camera/info", retained),
        "scan": node.create_publisher(PointCloud2, TOPIC + "/scan", retained),
    }
    if map_mode == "live":
        publishers["mesh"] = node.create_publisher(Mesh, TOPIC + "/mesh/source", 64)
    tf_pub = node.create_publisher(TFMessage, "/tf", 100)
    static_tf_pub = node.create_publisher(TFMessage, "/tf_static", retained)
    parent, mount, _ = camera_mount(path, dataset, urdf)
    from geometry_msgs.msg import PoseStamped
    camera_tf = pose_transform(PoseStamped(pose=matrix_pose(mount)), CAMERA_FRAME,
                               node.get_clock().now().to_msg())
    camera_tf.header.frame_id = ROBOT + "/" + parent
    static_tf_pub.publish(TFMessage(transforms=[camera_tf]))
    path_pub = node.create_publisher(RosPath, TOPIC + "/walked_path", retained)
    status_pub = node.create_publisher(String, TOPIC + "/replay_status", retained)
    state = {"phase": "waiting", "requested": False, "registered": False, "elapsed_s": 0.0}
    counts, blocks, latest = Counter(), {}, {}
    trail = RosPath()
    trail.header.frame_id = "map"
    initial_height = first["pelvis"][0].pose.position.z
    last_path_time, max_lag = 0.0, 0.0

    def status():
        status_pub.publish(String(data=json.dumps({**state, "counts": counts,
            "map_blocks": len(blocks), "path_samples": len(trail.poses),
            "max_schedule_lag_s": round(max_lag, 4)})))

    def ack(message):
        try:
            data = json.loads(message.data)
            if data.get("robot_name") == ROBOT:
                state["registered"] = (data.get("success") is True and
                    not str(data.get("robot_id", "")).lower().startswith("queued"))
        except (ValueError, AttributeError):
            return

    def start(_request, response):
        if state["phase"] != "waiting":
            response.message = "Replay already started; relaunch to replay from the beginning."
        elif not state["registered"]:
            response.message = "Create a workspace and wait for G1 registration first."
        elif ((map_mode == "live" and publishers["mesh"].get_subscription_count() == 0)
              or publishers["joints"].get_subscription_count() == 0):
            response.message = "Waiting for the mesh adapter or robot state publisher."
        else:
            state["requested"] = True
            response.success, response.message = True, "Starting recorded walk; final scene will stay visible."
        return response

    node.create_subscription(String, "/horus/registration_ack", ack, 10)
    node.create_service(Trigger, TOPIC + "/start", start)
    node.create_timer(1.0, status)

    def publish(key, message, *, keep_path=True):
        nonlocal last_path_time
        validate_frame(key, message)
        now = node.get_clock().now().to_msg()
        latest[key] = message
        if key == "camera_pose":
            # Preserve the recorded observation for auditing, not as a second
            # independently timed map->camera edge competing with the robot tree.
            return
        if key == "pelvis":
            child = ROBOT + "/pelvis"
            transforms = [pose_transform(message, child, now)]
            transforms.append(floor_transform(message, initial_height, now))
            if keep_path:
                pose = deepcopy(message)
                pose.header.frame_id, pose.header.stamp = "map", now
                trail.poses.append(pose)
            if time.monotonic() - last_path_time >= 0.5:
                trail.header.stamp = now
                path_pub.publish(trail)
                last_path_time = time.monotonic()
            tf_pub.publish(TFMessage(transforms=transforms))
        else:
            # Large point/mesh arrays are immutable here: copy only the header,
            # not every ROS Point object or multi-megabyte data buffer.
            outgoing = jpeg_message(message) if key == "rgb" else copy(message)
            outgoing.header = deepcopy(message.header)
            outgoing.header.stamp = now
            outgoing.header.frame_id = CAMERA_FRAME if key in ("rgb", "camera_info") else "map"
            if key == "mesh":
                if len(outgoing.block_indices) != len(outgoing.blocks):
                    raise ValueError("Incomplete nvblox block update")
                if outgoing.clear:
                    blocks.clear()
                if counts[key] == 0:
                    outgoing.clear = True
                for index, block in zip(outgoing.block_indices, outgoing.blocks):
                    block_key = (index.x, index.y, index.z)
                    if block.vertices:
                        blocks[block_key] = (index, block)
                    else:
                        blocks.pop(block_key, None)
            publishers[key].publish(outgoing)

    def hold_once():
        for key in ("pelvis", "joints"):
            publish(key, latest.get(key, first[key][0]), keep_path=False)

    try:
        for key in ("camera_info", "rgb", "scan", "pelvis", "camera_pose", "joints"):
            publish(key, first[key][0], keep_path=False)
        Path(ready_file).write_text(json.dumps(info) + "\n")
        print("Ready; holding first pose. Wait for the model, then press Enter in the launcher.", flush=True)
        tick = 0.0
        while rclpy.ok() and not state["requested"]:
            if time.monotonic() >= tick:
                hold_once()
                tick = time.monotonic() + 0.1
            rclpy.spin_once(node, timeout_sec=0.02)
        state["phase"] = "playing"
        timeline_start, playback_start = None, None
        for key, message, stamp in read_source(path, urdf, dataset, map_mode):
            if not rclpy.ok():
                break
            if timeline_start is None:
                timeline_start = stamp
                playback_start = time.monotonic()
            due = playback_start + (stamp - timeline_start) / 1e9
            while rclpy.ok() and time.monotonic() < due:
                rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, due-time.monotonic())))
            max_lag = max(max_lag, time.monotonic()-due)
            state["elapsed_s"] = round((stamp-timeline_start)/1e9, 3)
            publish(key, message)
            counts[key] += 1
            rclpy.spin_once(node, timeout_sec=0)
        state["phase"] = "complete"
        trail.header.stamp = node.get_clock().now().to_msg()
        path_pub.publish(trail)
        status()
        Path(ready_file).with_name("replay-summary.json").write_text(json.dumps({**info,
            "counts": counts, "map_blocks": len(blocks), "path_samples": len(trail.poses),
            "max_schedule_lag_s": max_lag}, indent=2) + "\n")
        print(f"Replay complete: {dict(counts)}. Holding final scene; Ctrl+C stops.", flush=True)
        # Keep poses fresh. A slow full inventory also repairs an adapter restart;
        # Quest page repair itself remains revision-based, not periodic bulk resend.
        repair, repair_index, next_repair = [], 0, time.monotonic() + 10
        while rclpy.ok():
            hold_once()
            if time.monotonic() >= next_repair:
                if not repair:
                    repair, repair_index = list(blocks.values()), 0
                part = repair[repair_index:repair_index+32]
                if part:
                    msg = Mesh(block_size_m=latest["mesh"].block_size_m, clear=False)
                    msg.header.frame_id, msg.header.stamp = "map", node.get_clock().now().to_msg()
                    msg.block_indices, msg.blocks = [p[0] for p in part], [p[1] for p in part]
                    publishers["mesh"].publish(msg)
                repair_index += len(part)
                done = repair_index >= len(repair)
                next_repair = time.monotonic() + (10 if done else 0.2)
                if done:
                    repair = []
            until = time.monotonic() + 0.1
            while rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, until-time.monotonic())))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--ready-file", type=Path, required=True)
    parser.add_argument("--dataset", choices=("first", "later"), default="later")
    parser.add_argument("--map", dest="map_mode", choices=("static", "live"), default="live")
    args = parser.parse_args()
    run(args.bag, args.urdf, args.ready_file, args.dataset, args.map_mode)
