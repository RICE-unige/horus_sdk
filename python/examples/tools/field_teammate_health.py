"""Bounded live reception checks; hardware correctness is not inferred from counts."""
from collections import deque
import argparse
import json
import math
from pathlib import Path
import time
from examples.tools.field_teammate_scene import scene_frames


MR_MESH_TOPIC = "/horus/field/mesh/indexed"
MESH_STREAMS = ("mesh", "MR mesh", "estimated mesh", "estimated MR mesh")


def merge_mesh_snapshot(streams, remote, now):
    """Merge only measured worker counters, ageing them rather than inventing arrivals."""
    if not remote:
        return
    elapsed = max(0., now - remote["time_monotonic"])
    for name in MESH_STREAMS:
        if name not in streams or name not in remote["streams"]:
            continue
        row = dict(remote["streams"][name])
        if "arrival_age_sec" in row:
            row["arrival_age_sec"] = round(row["arrival_age_sec"] + elapsed, 3)
            if row["arrival_age_sec"] > 6 and row["state"] == "live":
                row["state"] = "retained"
        if "stamp_age_sec" in row:
            row["stamp_age_sec"] = round(row["stamp_age_sec"] + elapsed, 3)
        if elapsed > 10:
            row["state"] = "monitor_stale"
            row["rate_hz"] = 0.
        streams[name] = row


def safe_numbers(value):
    if isinstance(value, dict):
        return {key: safe_numbers(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_numbers(item) for item in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


class Reception:
    def __init__(self):
        self.samples = {}

    def observe(self, name, *, valid=True, details=None, stamp=None, now=None):
        now = time.monotonic() if now is None else now
        row = self.samples.setdefault(name, {"count": 0, "invalid_count": 0, "arrivals": deque(maxlen=120)})
        row["count"] += 1
        row["invalid_count"] += int(not valid)
        row["valid"] = valid
        row["details"] = details or {}
        row["arrivals"].append(now)
        row["stamp"] = stamp

    def snapshot(self, required, now=None):
        now = time.monotonic() if now is None else now
        result = {}
        for name in required:
            row = self.samples.get(name)
            if row is None:
                result[name] = {"state": "waiting", "count": 0, "rate_hz": 0}
                continue
            arrivals = [value for value in row["arrivals"] if now - value <= 10]
            elapsed = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0
            age = now - row["arrivals"][-1]
            stale_after = 6 if "mesh" in name or "depth" in name else 3
            state = "stale" if age > stale_after else "live" if row["valid"] else "invalid"
            if state == "stale" and ("mesh" in name or name == "travelled path") and row["valid"]:
                state = "retained"
            result[name] = {"state": state, "count": row["count"], "invalid_count": row["invalid_count"],
                "rate_hz": round((len(arrivals) - 1) / elapsed, 2) if elapsed > 0 else 0,
                "arrival_age_sec": round(age, 3), "details": row["details"]}
            if row["stamp"] is not None:
                result[name]["stamp_age_sec"] = round(time.time() - row["stamp"], 3)
        return result


def pose_valid(pose):
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    return all(math.isfinite(value) for value in values) and abs(sum(value * value for value in values[3:]) - 1) < .02


def depth_details(message):
    import numpy as np
    if (message.encoding != "32FC1" or not 0 < message.width <= 4096 or not 0 < message.height <= 4096 or
            message.step < message.width * 4 or len(message.data) != message.step * message.height):
        raise ValueError("Depth must be a complete 32FC1 image with valid row stride")
    depth = np.ndarray((message.height, message.width), dtype=">f4" if message.is_bigendian else "<f4",
                       buffer=bytes(message.data), strides=(message.step, 4))
    finite = depth[np.isfinite(depth) & (depth > 0)]
    return {"encoding": message.encoding, "width": message.width, "height": message.height,
            "valid_pixels": int(finite.size), "median_metres": float(np.median(finite)) if finite.size else None}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="field_teammate_1")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--receive-only", action="store_true")
    parser.add_argument("--estimated-depth", action="store_true")
    parser.add_argument("--mesh-worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--mesh-status-topic", default="", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.mesh_worker and (not args.mesh_status_topic or args.receive_only):
        parser.error("--mesh-worker requires --mesh-status-topic and mapping mode")
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image
    from std_msgs.msg import String, UInt8MultiArray
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    from horus.mapping.mesh_protocol import decode_live_header, HMSH_V2_HEADER

    rclpy.init()
    node = rclpy.create_node("horus_field_mesh_reception_check" if args.mesh_worker else "horus_field_reception_check")
    health = Reception()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    prefix = f"/{args.name}"
    floor_frame, scene_source, scene_camera = scene_frames(args.name)
    required = ["head TF", "video", "camera info", "camera pose", "relay status"]
    if not args.receive_only:
        required += ["depth", "depth info", "depth pose", "depth status", "travelled path", "scene alignment", "mesh", "MR mesh"]
    if args.estimated_depth:
        required += ["estimated depth", "estimated status", "estimated mesh", "estimated MR mesh"]
    if args.mesh_worker:
        required = [name for name in required if name in MESH_STREAMS]
    subscriptions = []
    mesh_states = {}
    tf_buffer = Buffer() if not args.mesh_worker else None
    tf_listener = TransformListener(tf_buffer, node) if tf_buffer is not None else None
    remote_mesh = None
    mesh_status_pub = node.create_publisher(String, args.mesh_status_topic, qos) if args.mesh_worker else None

    def receive_mesh_status(message):
        nonlocal remote_mesh
        try:
            value = json.loads(message.data)
            if (value.get("source") == "isolated_mesh_monitor" and
                    isinstance(value.get("streams"), dict) and
                    isinstance(value.get("time_monotonic"), (int, float)) and
                    math.isfinite(value["time_monotonic"])):
                remote_mesh = value
        except (ValueError, TypeError):
            pass

    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

    def observe(name, message, valid=True, details=None):
        health.observe(name, valid=valid, details=details, stamp=stamp(message) if hasattr(message, "header") else None)

    def subscribe(kind, topic, callback):
        subscriptions.append(node.create_subscription(kind, topic, callback, qos))

    def image(message):
        import cv2
        import numpy as np
        try:
            pixels = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            valid = pixels is not None
            observe("video", message, valid, {"width": pixels.shape[1], "height": pixels.shape[0]} if valid else {})
        except Exception as error:
            observe("video", message, False, {"error": str(error)})

    def info(name, message):
        valid = (message.width > 0 and message.height > 0 and math.isfinite(message.k[0]) and
                 math.isfinite(message.k[4]) and message.k[0] > 0 and message.k[4] > 0 and
                 message.distortion_model != "synthetic_pinhole")
        observe(name, message, valid, {"model": message.distortion_model, "fx": message.k[0], "fy": message.k[4]})

    def status(name, message):
        try:
            value = json.loads(message.data)
            valid = value.get("state") in ("live", "streaming")
            observe(name, message, valid, value)
        except (ValueError, TypeError) as error:
            observe(name, message, False, {"error": str(error)})

    def pose(name, message):
        position = message.pose.position
        observe(name, message, pose_valid(message.pose) and message.header.frame_id == "map",
            {"frame": message.header.frame_id, "position_m": [position.x, position.y, position.z]})

    def trajectory(message):
        valid = (message.header.frame_id == scene_source and 0 < len(message.poses) <= 20_000 and
                 all(p.header.frame_id == scene_source and pose_valid(p.pose)
                     for p in (message.poses[0], message.poses[-1])))
        observe("travelled path", message, valid, {"frame": message.header.frame_id,
                                                  "points": len(message.poses)})

    def depth(message, name="depth"):
        try:
            details = depth_details(message)
            observe(name, message, details["valid_pixels"] > 0, details)
        except ValueError as error:
            observe(name, message, False, {"error": str(error)})

    def mr_mesh(message, name="MR mesh"):
        try:
            state = mesh_states.setdefault(name, {"pages": {}, "epoch": None})
            pages = state["pages"]
            update = decode_live_header(bytes(message.data))
            expected = HMSH_V2_HEADER.size + update.vertex_count * (16 if update.has_color else 12) + update.index_count * 4
            if expected != len(message.data) or update.operation not in (0, 1, 2) or update.index_count % 3:
                raise ValueError("Invalid HMSH geometry length or operation")
            if update.epoch != state["epoch"] or update.operation == 2:
                pages.clear()
                state["epoch"] = update.epoch
            if update.operation == 0:
                if update.chunk_id not in pages and len(pages) >= 1024:
                    raise ValueError("Unexpected page capacity")
                pages[update.chunk_id] = update.index_count // 3
            elif update.operation == 1:
                pages.pop(update.chunk_id, None)
            triangles = sum(pages.values())
            observe(name, message, triangles > 0,
                {"bytes": len(message.data), "triangles_retained": triangles, "pages": len(pages)})
        except (ValueError, AttributeError) as error:
            observe(name, message, False, {"error": str(error)})

    if not args.mesh_worker:
        if not args.receive_only:
            subscribe(RosPath, prefix + "/trajectory", trajectory)
            subscribe(String, prefix + "/scene/status", lambda msg: status("scene alignment", msg))
        subscribe(CompressedImage, prefix + "/fpv/image_raw/compressed", image)
        subscribe(CameraInfo, prefix + "/fpv/camera_info", lambda msg: info("camera info", msg))
        subscribe(PoseStamped, prefix + "/fpv/camera_pose", lambda msg: pose("camera pose", msg))
        subscribe(String, prefix + "/status", lambda msg: status("relay status", msg))
        if args.mesh_status_topic:
            subscribe(String, args.mesh_status_topic, receive_mesh_status)
    def head_tf(message):
        for transform in message.transforms:
            if transform.child_frame_id == (f"{args.name}/camera" if args.receive_only else scene_camera):
                translation, rotation = transform.transform.translation, transform.transform.rotation
                numbers = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w]
                valid = all(math.isfinite(value) for value in numbers) and abs(sum(v * v for v in numbers[3:]) - 1) < .02
                observe("head TF", transform, valid, {"parent": transform.header.frame_id})
    if not args.mesh_worker:
        subscribe(TFMessage, "/tf", head_tf)
    if not args.receive_only:
        from nvblox_msgs.msg import Mesh
        depth_root = prefix + "/research/long_throw"
        if not args.mesh_worker:
            subscribe(Image, depth_root + "/depth/image", depth)
            subscribe(CameraInfo, depth_root + "/depth/camera_info", lambda msg: info("depth info", msg))
            subscribe(PoseStamped, depth_root + "/pose", lambda msg: pose("depth pose", msg))
            subscribe(String, depth_root + "/status", lambda msg: status("depth status", msg))
        if args.mesh_worker or not args.mesh_status_topic:
            subscribe(Mesh, "/horus_measured_mapper/mesh", lambda msg: observe("mesh", msg,
                msg.header.frame_id == "map" and msg.block_size_m > 0 and len(msg.blocks) == len(msg.block_indices),
                {"blocks_in_update": len(msg.blocks), "block_size_m": msg.block_size_m, "clear": msg.clear}))
            subscribe(UInt8MultiArray, MR_MESH_TOPIC, mr_mesh)
        if args.estimated_depth:
            learned_root = prefix + "/research/learned_depth"
            if not args.mesh_worker:
                subscribe(Image, learned_root + "/depth/image", lambda msg: depth(msg, "estimated depth"))
                subscribe(String, learned_root + "/status", lambda msg: status("estimated status", msg))
            if args.mesh_worker or not args.mesh_status_topic:
                subscribe(Mesh, "/horus_hybrid_mapper/mesh", lambda msg: observe("estimated mesh", msg,
                    msg.header.frame_id == "map" and msg.block_size_m > 0 and len(msg.blocks) == len(msg.block_indices),
                    {"blocks_in_update": len(msg.blocks), "block_size_m": msg.block_size_m}))
                subscribe(UInt8MultiArray, "/horus/field/estimated_mesh/indexed", lambda msg: mr_mesh(msg, "estimated MR mesh"))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("a", buffering=1) as output:
        def report():
            now = time.monotonic()
            streams = health.snapshot(required, now=now)
            if args.mesh_worker:
                payload = {"source": "isolated_mesh_monitor", "time_monotonic": now, "streams": streams}
                mesh_status_pub.publish(String(data=json.dumps(safe_numbers(payload), allow_nan=False)))
                output.write(json.dumps(safe_numbers(payload), allow_nan=False) + "\n")
                return
            merge_mesh_snapshot(streams, remote_mesh, now)
            chain_valid = False
            head_position = None
            try:
                transform = tf_buffer.lookup_transform(
                    "map" if args.receive_only else floor_frame,
                    f"{args.name}/camera" if args.receive_only else scene_camera, rclpy.time.Time())
                chain_valid = abs(node.get_clock().now().nanoseconds / 1e9 - stamp(transform)) < 3
                t = transform.transform.translation
                head_position = [t.x, t.y, t.z]
            except Exception:
                pass  # The snapshot remains waiting/stale until TF really arrives.
            snapshot = {"time_utc": time.time(), "streams": streams, "map_to_head_tf_fresh": chain_valid,
                        "map_to_head_position_m": head_position,
                        "head_reference_frame": "map" if args.receive_only else floor_frame}
            snapshot["all_streams_live"] = all(row["state"] == "live" for row in snapshot["streams"].values())
            snapshot["all_required_data_present"] = chain_valid and all(
                row["state"] in ("live", "retained") for row in snapshot["streams"].values())
            output.write(json.dumps(safe_numbers(snapshot), allow_nan=False) + "\n")
            print(" | ".join(f"{key}: {row['state']} {row['rate_hz']:.1f}Hz" for key, row in snapshot["streams"].items()), flush=True)

        node.create_timer(1 if args.mesh_worker else 5, report)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            tf_listener.unregister()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
