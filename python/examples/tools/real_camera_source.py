"""Replay real RGB-D and motion data on the robot-site PC at recorded speed."""

import argparse
from array import array
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

from real_camera_data import DATASET_PAGE, DATASET_URL, INTRINSICS, loop_period, nearest_index, pointcloud, read_index


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--role", choices=("all", "video", "jpeg", "motion", "bulk"), default="all")
    parser.add_argument("--jpeg-quality", type=int, default=75)
    parser.add_argument("--epoch-wall-ns", type=int, default=0)
    args = parser.parse_args()
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("jpeg-quality must be between 1 and 100")
    args.output.mkdir(parents=True, exist_ok=True)
    if args.role == "all":
        # ROS serialization holds Python's GIL; separate processes prevent a
        # full cloud publication from pausing video or high-rate recorded TF.
        epoch = time.time_ns()+15_000_000_000
        children = [subprocess.Popen([sys.executable, __file__, "--dataset", str(args.dataset),
            "--output", str(args.output / role), "--role", role, "--epoch-wall-ns", str(epoch),
            "--jpeg-quality", str(args.jpeg_quality)])
            for role in ("video", "jpeg", "motion", "bulk")]
        stopped = threading.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stopped.set())
        try:
            while not stopped.wait(.2):
                if any(child.poll() is not None for child in children):
                    raise RuntimeError("Replay worker exited; see source journal")
        finally:
            for child in children:
                if child.poll() is None:
                    child.send_signal(signal.SIGTERM)
            for child in children:
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=5)
        return
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, PointCloud2, PointField
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    cv2.setNumThreads(1)
    rgb_index = read_index(args.dataset / "rgb.txt")
    depth_index = read_index(args.dataset / "depth.txt")
    poses = read_index(args.dataset / "groundtruth.txt")
    accel = read_index(args.dataset / "accelerometer.txt") if (args.dataset / "accelerometer.txt").exists() else []
    rgb = [cv2.cvtColor(cv2.imread(str(args.dataset / row[1])), cv2.COLOR_BGR2RGB) for row in rgb_index] if args.role != "motion" else []
    depth = [cv2.imread(str(args.dataset / row[1]), cv2.IMREAD_UNCHANGED) for row in depth_index] if args.role == "bulk" else []
    if any(x is None or x.shape != (480, 640) or x.dtype != np.uint16 for x in depth):
        raise ValueError("Expected original 640x480 16-bit Kinect depth")
    times = [row[0] for row in rgb_index]
    depth_times = [row[0] for row in depth_index]
    period = loop_period(times)
    provenance = dict(dataset="TUM RGB-D freiburg1_desk", url=DATASET_URL,
        homepage=DATASET_PAGE, license="CC BY 4.0", rgb_frames=len(rgb_index), depth_frames=len(depth_index),
        pose_samples=len(poses), accelerometer_samples=len(accel), period_s=period,
        recorded_rgb_hz=(len(times)-1)/(times[-1]-times[0]), width=640, height=480,
        playback_speed=1, camera_streams="Three phase-offset replays of one recording, not three independent robots",
        mixed_load="Original motion cadence; lossless full-resolution depth at 2Hz; all valid cloud pixels at 0.2Hz",
        depth_scale=5000, camera_intrinsics=INTRINSICS, jpeg_quality=args.jpeg_quality,
        jpeg_encoding="Live OpenCV/libjpeg encoding in a separate replay process; quality explicitly configured",
        clock="Replay publication wall time; original timestamp in source CSV")
    (args.output / "dataset.json").write_text(json.dumps(provenance, indent=2))
    rclpy.init()
    node = rclpy.create_node("horus_real_camera_source_"+args.role)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    cameras = [node.create_publisher(Image, f"/horus/dataset/cam{i+1}/image_raw", sensor_qos) for i in range(3)] if args.role == "video" else []
    compressed = [node.create_publisher(CompressedImage,
        f"/horus/dataset/cam{i+1}/image_raw/compressed", sensor_qos) for i in range(3)] if args.role == "jpeg" else []
    tf_pub = node.create_publisher(TFMessage, "/horus/dataset/tf", 32) if args.role == "motion" else None
    depth_pub = node.create_publisher(CompressedImage, "/horus/dataset/depth_png", 2) if args.role == "bulk" else None
    cloud_pub = node.create_publisher(PointCloud2, "/horus/dataset/points", 2) if args.role == "bulk" else None
    imu_pub = node.create_publisher(Imu, "/horus/dataset/imu", sensor_qos) if args.role == "motion" else None
    info_pub = node.create_publisher(CameraInfo, "/horus/dataset/camera_info", 2) if args.role == "video" else None
    status_pub = node.create_publisher(String, "/horus/dataset/source_status", 10)
    clock_pub = node.create_publisher(String, "/horus/dataset/clock_reply", 10)
    state = {"role": args.role, "condition": "video_only", "transport": "webrtc", "cameras": 0,
             "jpeg_quality": args.jpeg_quality, "pid": os.getpid(), "counts": {}, "errors": [],
             "skipped_camera_deadlines": [0]*3}
    lock = threading.Lock()
    events_file = (args.output / "source_events.csv").open("x", buffering=65536)
    events = csv.writer(events_file)
    events.writerow(["kind", "stream", "publish_wall_ns", "publish_monotonic_ns", "recorded_timestamp", "cycle", "index", "bytes", "sha256"])

    def record(kind, stream, stamp, recorded, cycle, index, size, digest=""):
        with lock:
            key = f"{kind}/{stream}"
            state["counts"][key] = state["counts"].get(key, 0)+1
            events.writerow([kind, stream, stamp, time.monotonic_ns(), recorded, cycle, index, size, digest])

    def set_condition(message):
        value = json.loads(message.data)
        if value.get("condition") in {"video_only", "motion_only", "depth_only", "cloud_only", "mixed"}:
            state["condition"] = value["condition"]
        if value.get("transport") in {"webrtc", "ros"}:
            state["transport"] = value["transport"]
        if type(value.get("cameras")) is int and 0 <= value["cameras"] <= 3:
            state["cameras"] = value["cameras"]
        if isinstance(value.get("trial_id"), str):
            state["trial_id"] = value["trial_id"]

    def clock(message):
        received = time.time_ns()
        value = json.loads(message.data)
        if type(value.get("sent_ns")) is int and isinstance(value.get("id"), str):
            clock_pub.publish(String(data=json.dumps(dict(id=value["id"], sent_ns=value["sent_ns"],
                source_received_ns=received, source_sent_ns=time.time_ns()))))

    node.create_subscription(String, "/horus/dataset/condition", set_condition, 10)
    if args.role == "motion":
        node.create_subscription(String, "/horus/dataset/clock_request", clock, 10)

    def stamp(header, ns, frame="tum_camera_optical"):
        header.stamp.sec, header.stamp.nanosec = divmod(ns, 1_000_000_000)
        header.frame_id = frame

    start = time.monotonic()+(args.epoch_wall_ns-time.time_ns())/1e9 if args.epoch_wall_ns else time.monotonic()+1

    def camera_worker(stream):
        publisher = compressed[stream] if args.role == "jpeg" else cameras[stream]
        phase = stream*period/3
        index = int(np.searchsorted(np.asarray(times)-times[0], phase))
        epoch = start-phase
        cycle = 0
        while not stop.is_set():
            target = epoch+cycle*period+times[index]-times[0]
            if stop.wait(max(0, target-time.monotonic())):
                break
            now = time.monotonic()
            if now-target > .05:
                state["skipped_camera_deadlines"][stream] += 1
            elif (stream < state["cameras"] and publisher.get_subscription_count()
                  and state["transport"] == ("ros" if args.role == "jpeg" else "webrtc")):
                image = rgb[index]
                ns = time.time_ns()
                if args.role == "jpeg":
                    ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                                              [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality])
                    if not ok:
                        raise RuntimeError("Live JPEG encoding failed")
                    msg = CompressedImage(format="rgb8; jpeg compressed bgr8", data=array("B", encoded.tobytes()))
                else:
                    msg = Image(width=640, height=480, encoding="rgb8", step=1920, data=array("B", image.tobytes()))
                stamp(msg.header, ns, f"tum_cam{stream+1}_optical")
                publisher.publish(msg)
                record("jpeg" if args.role == "jpeg" else "rgb", stream+1, ns, times[index], cycle,
                       index, len(msg.data), hashlib.sha256(msg.data).hexdigest() if args.role == "jpeg" else "")
            index += 1
            if index == len(times):
                index, cycle = 0, cycle+1

    def motion_worker():
        rows = sorted([(row[0], "tf", row) for row in poses] + [(row[0], "imu", row) for row in accel])
        cycle = 0
        while not stop.is_set():
            for index, (recorded, kind, row) in enumerate(rows):
                if not times[0] <= recorded < times[0]+period:
                    continue
                target = start+cycle*period+recorded-times[0]
                if stop.wait(max(0, target-time.monotonic())):
                    return
                if state["condition"] == "video_only":
                    continue
                ns = time.time_ns()
                if kind == "tf":
                    msg = TransformStamped(child_frame_id="tum_cam1_optical")
                    stamp(msg.header, ns, "tum_world")
                    vals = [float(x) for x in row[1:]]
                    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = vals[:3]
                    q = msg.transform.rotation
                    q.x, q.y, q.z, q.w = vals[3:7]
                    tf_pub.publish(TFMessage(transforms=[msg]))
                else:
                    msg = Imu()
                    stamp(msg.header, ns)
                    msg.orientation_covariance[0] = -1
                    msg.angular_velocity_covariance[0] = -1
                    a = msg.linear_acceleration
                    a.x, a.y, a.z = [float(x) for x in row[1:4]]
                    imu_pub.publish(msg)
                record(kind, 1, ns, recorded, cycle, index, 0)
            cycle += 1

    def bulk_worker():
        next_cloud = 0.
        while not stop.wait(.5):
            if state["condition"] not in {"mixed", "depth_only", "cloud_only"}:
                continue
            elapsed = max(0, time.monotonic()-start)
            cycle = int(elapsed // period)
            index = min(int(np.searchsorted(np.asarray(times)-times[0], elapsed % period)), len(times)-1)
            di = nearest_index(depth_times, times[index])
            if di is None:
                continue
            ok, encoded = cv2.imencode(".png", depth[di])
            if not ok:
                raise RuntimeError("Depth PNG encoding failed")
            ns = time.time_ns()
            msg = CompressedImage(format="png", data=array("B", encoded.tobytes()))
            stamp(msg.header, ns)
            if state["condition"] != "cloud_only":
                depth_pub.publish(msg)
                record("depth", 1, ns, depth_times[di], cycle, di, len(msg.data), hashlib.sha256(msg.data).hexdigest())
            if state["condition"] != "depth_only" and time.monotonic() >= next_cloud:
                cloud = pointcloud(depth[di], rgb[index])
                ns = time.time_ns()
                msg = PointCloud2(height=1, width=len(cloud), point_step=16, row_step=len(cloud)*16,
                    is_dense=True, data=array("B", cloud.tobytes()), fields=[
                        PointField(name=name, offset=i*4, datatype=PointField.FLOAT32 if i < 3 else PointField.UINT32, count=1)
                        for i, name in enumerate(("x", "y", "z", "rgb"))])
                stamp(msg.header, ns, "tum_cam1_optical")
                cloud_pub.publish(msg)
                record("cloud", 1, ns, times[index], cycle, index, cloud.nbytes, hashlib.sha256(msg.data).hexdigest())
                next_cloud = time.monotonic()+5

    def guarded(target, *values):
        try:
            target(*values)
        except Exception as error:
            state["errors"].append(repr(error))
            stop.set()

    workers = [threading.Thread(target=guarded, args=(camera_worker, i), daemon=True) for i in range(3)] if args.role in {"video", "jpeg"} else []
    if args.role == "motion":
        workers.append(threading.Thread(target=guarded, args=(motion_worker,), daemon=True))
    if args.role == "bulk":
        workers.append(threading.Thread(target=guarded, args=(bulk_worker,), daemon=True))
    for worker in workers:
        worker.start()
    last_status = 0.
    print(json.dumps(provenance), flush=True)
    try:
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic()-last_status >= 1:
                with lock:
                    status = dict(state, source_wall_ns=time.time_ns(), source_monotonic_ns=time.monotonic_ns())
                    status["counts"] = dict(state["counts"])
                    status_pub.publish(String(data=json.dumps(status)))
                    events_file.flush()
                    (args.output / "status.json").write_text(json.dumps(status, indent=2))
                if info_pub is not None:
                    info = CameraInfo(width=640, height=480)
                    fx, fy, cx, cy = INTRINSICS
                    info.k = [fx, 0., cx, 0., fy, cy, 0., 0., 1.]
                    stamp(info.header, time.time_ns())
                    info_pub.publish(info)
                last_status = time.monotonic()
    finally:
        stop.set()
        for worker in workers:
            worker.join(timeout=5)
        events_file.close()
        node.destroy_node()
        rclpy.shutdown()
    if state["errors"]:
        raise RuntimeError(state["errors"])


if __name__ == "__main__":
    main()
