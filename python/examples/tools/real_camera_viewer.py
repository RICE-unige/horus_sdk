"""Measure genuine HORUS remote video and ROS sensor receipt; visualize in Rerun."""

import argparse
import asyncio
from collections import deque
import csv
import hashlib
import json
import os
import pickle
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid

import cv2
import numpy as np

from real_camera_data import ClockEstimate, Rate, decode_depth_png, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--cameras", type=int, choices=(0, 1, 3), default=3)
    parser.add_argument("--transport", choices=("webrtc", "ros"), default="webrtc")
    parser.add_argument("--webrtc-decoder", choices=("aiortc", "gstreamer"), default="gstreamer")
    parser.add_argument("--quality-samples", type=int, default=0,
                        help="Save bounded decoded PNG samples for source-matched quality checks")
    parser.add_argument("--decoder-threads", type=int, choices=(0,1,2,4), default=1,
                        help="H.264 decoder threads per camera; 0 preserves FFmpeg auto threading")
    parser.add_argument("--condition", choices=("video_only", "motion_only", "depth_only", "cloud_only", "mixed"), default="mixed")
    parser.add_argument("--duration", type=float, default=90)
    parser.add_argument("--warmup", type=float, default=10)
    parser.add_argument("--rerun", action="store_true")
    parser.add_argument("--viewer-port", type=int, default=9891)
    args = parser.parse_args()
    if args.duration <= 0 or args.warmup < 0:
        parser.error("duration must be positive and warmup non-negative")
    args.output.mkdir(parents=True, exist_ok=True)
    asyncio.run(run(args))


async def run(args):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from aiortc import RTCPeerConnection, RTCConfiguration, RTCSessionDescription, RTCRtpReceiver
    from aiortc.rtcdtlstransport import RTCDtlsTransport, X509_DIGEST_ALGORITHMS, State
    from aiortc.codecs.h264 import H264Decoder

    original_decoder_init = H264Decoder.__init__

    def decoder_init(self):
        original_decoder_init(self)
        # Auto slicing creates one worker per CPU on this host, per camera.
        # Small real-time frames should not wait for dozens of worker barriers.
        self.codec.thread_count = args.decoder_threads

    H264Decoder.__init__ = decoder_init

    # This diagnostic's OpenSSL path still verifies all advertised supported
    # fingerprints, while accepting libdatachannel's legacy X509 version.
    def verify(self, parameters):
        certificate = self._ssl.get_peer_certificate()
        supported = [f for f in parameters.fingerprints if f.algorithm.lower() in X509_DIGEST_ALGORITHMS]
        if not supported or any(certificate.digest(f.algorithm.replace("-", "")).decode().upper() != f.value.upper()
                                for f in supported):
            self._set_state(State.FAILED)

    RTCDtlsTransport._validate_peer_identity = verify
    cv2.setNumThreads(1)
    rclpy.init()
    node = rclpy.create_node("horus_real_camera_viewer_"+uuid.uuid4().hex[:8])
    sensor_qos = QoSProfile(depth=16, reliability=ReliabilityPolicy.BEST_EFFORT)
    signal_pub = node.create_publisher(String, "/horus/webrtc/client_signal", 10)
    condition_pub = node.create_publisher(String, "/horus/dataset/condition", 10)
    clock_pub = node.create_publisher(String, "/horus/dataset/clock_request", 10)
    stop = threading.Event()
    ui_queue = queue.Queue(maxsize=32)
    latest_images, ui_counts, rates = {}, {}, {}
    jpeg_pending = {}
    jpeg_condition = threading.Condition()
    quality_saved, quality_last = {}, {}
    compressed_receipts = {}
    compressed_seen = {}
    decoded, packets, traces, clock_samples, statuses = {}, {}, {}, [], []
    peers, answers, tasks, errors = {}, {}, [], []
    receivers, arrivals = {}, {}
    clock = ClockEstimate()
    pending_clocks = {}
    source_state = {}
    source_rates = {}
    source_roles = {}
    trial_id = uuid.uuid4().hex
    measure_start = measure_end = None
    events_file = (args.output / "received_events.csv").open("x", buffering=65536)
    events = csv.writer(events_file)
    events.writerow(["kind", "stream", "receipt_wall_ns", "receipt_monotonic_ns", "source_publish_wall_ns", "bytes", "sha256"])
    ui_errors = []
    ui_ready = threading.Event()
    event_lock = threading.Lock()
    seen_sensors = {}
    last_pose_display = 0.
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())

    def rate_tick(name):
        rates.setdefault(name, Rate()).tick(time.monotonic())

    def emit_ui(kind, value):
        if not args.rerun:
            return
        try:
            ui_queue.put_nowait((kind, value))
        except queue.Full:
            ui_counts["sensor_updates_skipped_in_viewer"] = ui_counts.get("sensor_updates_skipped_in_viewer", 0)+1

    def source_status(message):
        value = json.loads(message.data)
        role = value.get("role", "all")
        previous = source_roles.get(role, {})
        elapsed = (value.get("source_monotonic_ns", 0)-previous.get("source_monotonic_ns", 0))/1e9
        if 0 < elapsed < 5:
            for name, count in value.get("counts", {}).items():
                source_rates[name] = max(0, count-previous.get("counts", {}).get(name, count))/elapsed
        source_roles[role] = value
        source_state.update(value)
        source_state["counts"] = {key: count for state in source_roles.values() for key, count in state.get("counts", {}).items()}
        source_state["errors"] = [error for state in source_roles.values() for error in state.get("errors", [])]
        source_state["skipped_camera_deadlines"] = source_roles.get("video", value).get("skipped_camera_deadlines", [])
        statuses.append(dict(receipt_wall_ns=time.time_ns(), **source_state))

    def clock_reply(message):
        now = time.time_ns()
        value = json.loads(message.data)
        sent = pending_clocks.pop(value.get("id"), None)
        if sent is not None and value.get("sent_ns") == sent:
            if clock.add(sent, now, value["source_received_ns"], value["source_sent_ns"]):
                clock_samples.append(dict(received_ns=now, sent_ns=sent, **{k: value[k] for k in
                    ("source_received_ns", "source_sent_ns")}))

    def sensor(kind, message):
        nonlocal last_pose_display
        now, mono = time.time_ns(), time.monotonic_ns()
        header = message.transforms[0].header if kind == "tf" else message.header
        stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec
        size = len(message.data) if hasattr(message, "data") else 0
        with event_lock:
            events.writerow([kind, 1, now, mono, stamp, size,
                             hashlib.sha256(message.data).hexdigest() if kind in {"depth", "cloud"} else ""])
        seen = seen_sensors.setdefault(kind, {})
        if stamp in seen:
            return
        seen[stamp] = None
        if len(seen) > 16384:
            del seen[next(iter(seen))]
        rate_tick(kind)
        # Display motion at 30Hz and omit invisible IMU UI copies; counters and
        # CSV still observe every received sample independently of this choice.
        if kind == "tf":
            if mono/1e9-last_pose_display < 1/30:
                return
            last_pose_display = mono/1e9
        if kind != "imu":
            emit_ui(kind, message)

    def answer(message):
        value = json.loads(message.data)
        if value.get("session_id") in answers:
            loop.call_soon_threadsafe(answers[value["session_id"]].put_nowait, value)

    subscriptions = [
        node.create_subscription(String, "/horus/webrtc/server_signal", answer, 10),
        node.create_subscription(String, "/horus/dataset/source_status", source_status, 10),
        node.create_subscription(String, "/horus/dataset/clock_reply", clock_reply, 10),
    ]

    def compressed_image(name, message):
        now, mono = time.time_ns(), time.monotonic_ns()
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        payload = bytes(message.data)
        with event_lock:
            events.writerow(["jpeg", name, now, mono, stamp, len(payload), hashlib.sha256(payload).hexdigest()])
        compressed_receipts[name].append((mono/1e9, len(payload)))
        seen = compressed_seen.setdefault(name, {})
        if stamp in seen:
            ui_counts["duplicate_jpeg_messages"] = ui_counts.get("duplicate_jpeg_messages", 0)+1
            return
        seen[stamp] = None
        if len(seen) > 16384:
            del seen[next(iter(seen))]
        # JPEG decoding and display never run on the TF/clock executor. Only
        # complete camera frames may supersede stale display frames here.
        with jpeg_condition:
            if name in jpeg_pending:
                ui_counts["jpeg_decode_superseded"] = ui_counts.get("jpeg_decode_superseded", 0)+1
            jpeg_pending[name] = (now, mono, stamp, payload)
            jpeg_condition.notify()

    def save_quality(name, stamp, frame, mono):
        if (measure_start is None or stamp is None or quality_saved.get(name, 0) >= args.quality_samples
                or mono/1e9-quality_last.get(name, 0) < max(2, args.duration/max(1,args.quality_samples))):
            return
        rgb = frame if isinstance(frame, np.ndarray) else frame.to_ndarray(format="rgb24")
        path = args.output / "quality"
        path.mkdir(exist_ok=True)
        ok, image = cv2.imencode(".png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        if not ok:
            raise ValueError("Decoded quality sample PNG encoding failed")
        (path / f"{name}-{stamp}.png").write_bytes(image.tobytes())
        quality_saved[name] = quality_saved.get(name, 0)+1
        quality_last[name] = mono/1e9

    def decode_jpeg():
        try:
            while not stop.is_set():
                with jpeg_condition:
                    jpeg_condition.wait_for(lambda: jpeg_pending or stop.is_set(), timeout=.1)
                    pending = dict(jpeg_pending)
                    jpeg_pending.clear()
                for name, (received, receipt_mono, stamp, payload) in pending.items():
                    image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
                    if image is None or image.shape != (480, 640, 3):
                        ui_counts["invalid_jpeg_frames"] = ui_counts.get("invalid_jpeg_frames", 0)+1
                        continue
                    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                    now, mono = time.time_ns(), time.monotonic_ns()
                    decoded[name].append(mono/1e9)
                    rate_tick(f"decoded/{name}")
                    with event_lock:
                        events.writerow(["video", name, now, mono, stamp, 0, ""])
                    traces[name].append(dict(receipt_wall_ns=now, receipt_monotonic_ns=mono,
                        transport_receipt_wall_ns=received, ros_stamp_ns=stamp))
                    if args.rerun:
                        latest_images[name] = rgb
                    save_quality(name, stamp, rgb, mono)
        except Exception as error:
            errors.append(repr(error))
            stop.set()

    jpeg_worker = None
    if args.transport == "ros":
        for i in range(1, args.cameras+1):
            name = f"cam{i}"
            decoded[name], traces[name], compressed_receipts[name] = [], [], []
            subscriptions.append(node.create_subscription(CompressedImage,
                f"/horus/dataset/{name}/image_raw/compressed",
                lambda m, name=name: compressed_image(name, m), sensor_qos))
        jpeg_worker = threading.Thread(target=decode_jpeg, daemon=True)
        jpeg_worker.start()
    if args.condition != "video_only":
        subscriptions.extend([
            node.create_subscription(TFMessage, "/horus/dataset/tf", lambda m: sensor("tf", m), 32),
            node.create_subscription(Imu, "/horus/dataset/imu", lambda m: sensor("imu", m), sensor_qos),
            node.create_subscription(CompressedImage, "/horus/dataset/depth_png", lambda m: sensor("depth", m), 2),
            node.create_subscription(PointCloud2, "/horus/dataset/points", lambda m: sensor("cloud", m), 2),
        ])

    def visualize():
        process = None
        try:
            # Separate environments keep Arrow/NumPy2 out of ROS/OpenCV's
            # NumPy1 decoder. Only display copies cross this private pipe.
            environment = dict(os.environ)
            environment.pop("PYTHONPATH", None)
            process = subprocess.Popen([os.environ["HORUS_RERUN_PYTHON"],
                str(Path(__file__).with_name("real_camera_rerun.py")),
                "--output", str(args.output.resolve()), "--port", str(args.viewer_port),
                "--cameras", str(args.cameras), "--transport", args.transport], stdin=subprocess.PIPE, env=environment)

            def send(kind, data):
                pickle.dump((kind, time.monotonic()-visual_start, data), process.stdin, protocol=4)
                process.stdin.flush()

            poses = deque(maxlen=2000)
            last_stats = 0
            ui_ready.set()
            while not stop.is_set() or not ui_queue.empty():
                if process.poll() is not None:
                    raise RuntimeError(f"Rerun adapter exited: {process.returncode}")
                for name, frame in list(latest_images.items()):
                    if latest_images.get(name) is frame:
                        latest_images.pop(name, None)
                    rgb = frame if isinstance(frame, np.ndarray) else frame.to_ndarray(format="rgb24")
                    ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                                           [cv2.IMWRITE_JPEG_QUALITY, 95])
                    if not ok:
                        raise RuntimeError("Rerun display JPEG encoding failed")
                    send("image", dict(name=name, jpeg=jpeg.tobytes()))
                    ui_counts[name] = ui_counts.get(name, 0)+1
                try:
                    kind, message = ui_queue.get(timeout=.005)
                except queue.Empty:
                    kind = None
                if kind == "tf":
                    transform = message.transforms[0]
                    p, q = transform.transform.translation, transform.transform.rotation
                    pos = [p.x, p.y, p.z]
                    poses.append((transform.header.stamp.sec*1e9+transform.header.stamp.nanosec, pos, [q.x,q.y,q.z,q.w]))
                    send("pose", dict(position=pos, quaternion=[q.x,q.y,q.z,q.w]))
                elif kind == "depth":
                    try:
                        image = decode_depth_png(message.data)
                    except (ValueError, cv2.error):
                        count = ui_counts.get("invalid_depth_frames", 0)+1
                        ui_counts["invalid_depth_frames"] = count
                        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                        if count <= 3:
                            (args.output / f"invalid-depth-{stamp}.png").write_bytes(bytes(message.data))
                        if count <= 5 or count % 60 == 0:
                            print(f"Invalid depth PNG received: source_stamp={stamp}, bytes={len(message.data)}, count={count}", flush=True)
                        continue
                    send("depth", image)
                elif kind == "cloud":
                    raw = np.frombuffer(message.data, dtype=[("xyz","<f4",3),("rgb","<u4")])
                    color = raw["rgb"]
                    colors = np.column_stack([(color >> 16) & 255, (color >> 8) & 255, color & 255]).astype(np.uint8)
                    stamp = message.header.stamp.sec*1e9+message.header.stamp.nanosec
                    pose = min(poses, key=lambda p: abs(p[0]-stamp)) if poses else None
                    send("cloud", dict(points=raw["xyz"], colors=colors,
                        position=pose[1] if pose else None, quaternion=pose[2] if pose else None))
                if time.monotonic()-last_stats > .5:
                    numbers = {name: tracker.hz(time.monotonic()) for name, tracker in list(rates.items())}
                    for i in range(1, args.cameras+1):
                        numbers[f"source/cam{i}"] = source_rates.get(f"{'jpeg' if args.transport == 'ros' else 'rgb'}/{i}", 0)
                    best = clock.best(time.time_ns())
                    text = ("# Arancino → HORUS → This PC\n\n"
                        "**Real recording:** TUM RGB-D / freiburg1_desk · 640×480 · native ~30 Hz\n\n"
                        f"**Load:** {args.condition} · {args.cameras} camera streams\n\n"
                        + ("**Video:** ROS CompressedImage / JPEG → Zenoh → local JPEG decode.\n\n" if args.transport == "ros"
                           else "**Video:** source GPU H.264 → WAN WebRTC → native packet relay → local WebRTC decode.\n\n")
                        + "**Sensors:** recorded motion + lossless depth + full valid-pixel coloured cloud through Zenoh.\n\n"
                        + (f"**Clock uncertainty bound:** {best[2]/1e6:.1f} ms\n\n" if best else "Waiting for clock response.\n\n"))
                    text += "Three cameras are phase-offset replays of one recording. Viewer logging is not measured screen presentation FPS.\n\n"
                    text += f"**Source scheduling skips:** {source_state.get('skipped_camera_deadlines', [])}"
                    text += f"\n\n**Invalid depth payloads:** {ui_counts.get('invalid_depth_frames', 0)}"
                    send("status", dict(text=text, rates=numbers))
                    last_stats = time.monotonic()
        except Exception as error:
            ui_errors.append(repr(error))
            ui_ready.set()
        finally:
            if process:
                try:
                    process.stdin.close()
                except BrokenPipeError:
                    pass
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)

    visual_start = time.monotonic()
    worker = threading.Thread(target=visualize, daemon=True) if args.rerun else None
    if worker:
        worker.start()

    def publish(session, **value):
        signal_pub.publish(String(data=json.dumps(dict(session_id=session, **value))))

    async def spin():
        next_ping = 0
        while not stop.is_set():
            if time.monotonic() >= next_ping:
                token = uuid.uuid4().hex
                now = time.time_ns()
                pending_clocks.clear()
                pending_clocks[token] = now
                clock_pub.publish(String(data=json.dumps(dict(id=token, sent_ns=now))))
                condition_pub.publish(String(data=json.dumps(dict(condition=args.condition,
                    transport=args.transport, cameras=args.cameras, trial_id=trial_id))))
                next_ping = time.monotonic()+1
            await asyncio.sleep(.01)

    def spin_ros():
        try:
            while not stop.is_set():
                rclpy.spin_once(node, timeout_sec=.01)
        except Exception as error:
            errors.append(repr(error))
            stop.set()

    ros_worker = threading.Thread(target=spin_ros, daemon=True)
    ros_worker.start()

    async def video(name, track):
        while not stop.is_set():
            frame = await track.recv()
            now, mono = time.time_ns(), time.monotonic_ns()
            metadata = None
            for side in frame.side_data:
                raw = bytes(side)
                marker = raw.find(b"HORUS-RR-V3-META")
                if marker >= 0:
                    try:
                        value, _ = json.JSONDecoder().raw_decode(raw[marker+16:].decode("utf-8"))
                        if value.get("kind") == "horus_camera_frame_v1":
                            metadata = value
                    except (ValueError, UnicodeDecodeError):
                        pass
            decoded[name].append(mono/1e9)
            rate_tick(f"decoded/{name}")
            stamp = metadata["ros_stamp_ns"] if metadata else None
            with event_lock:
                events.writerow(["video", name, now, mono, stamp, 0, ""])
            if metadata:
                receiver = receivers[name]
                mapper = receiver._RTCRtpReceiver__timestamp_mapper
                absolute = ((int(frame.pts*frame.time_base*90000)+mapper._origin) & 0xffffffff
                            if frame.pts is not None and mapper._origin is not None else None)
                arrival = arrivals[name].get(absolute, {})
                traces[name].append(dict(receipt_wall_ns=now, receipt_monotonic_ns=mono,
                    first_rtp_wall_ns=arrival.get("first"), last_rtp_wall_ns=arrival.get("last"),
                    decoder_queue_size=receiver._RTCRtpReceiver__decoder_queue.qsize(),
                    decoded_queue_size=track._queue.qsize(), **metadata))
            if args.rerun:
                latest_images[name] = frame
            save_quality(name, stamp, frame, mono)

    def native_frame(name, image, metadata, now, mono):
        decoded[name].append(mono/1e9)
        rate_tick(f"decoded/{name}")
        stamp = metadata.get("ros_stamp_ns") if metadata else None
        with event_lock:
            events.writerow(["video", name, now, mono, stamp, 0, ""])
        if metadata:
            traces[name].append(dict(receipt_wall_ns=now, receipt_monotonic_ns=mono, **metadata))
        if args.rerun:
            latest_images[name] = image
        save_quality(name, stamp, image, mono)

    def native_packet(name, timestamp, sequence, marker, size):
        packets[name].append((time.monotonic(), timestamp, sequence, marker, size))

    tasks.append(asyncio.create_task(spin()))
    try:
        deadline = time.monotonic()+20
        while (args.transport == "webrtc" and not signal_pub.get_subscription_count()) or not source_state:
            if time.monotonic() > deadline:
                raise RuntimeError("Source telemetry or HORUS signaling unavailable")
            await asyncio.sleep(.05)
        deadline = time.monotonic()+25
        while not all(source_roles.get(role, {}).get("trial_id") == trial_id
                      for role in ("video", "jpeg", "motion", "bulk")):
            if time.monotonic() > deadline:
                raise RuntimeError("Not all replay workers acknowledged this trial; refusing mixed/stale conditions")
            await asyncio.sleep(.05)
        for i in range(1, args.cameras+1) if args.transport == "webrtc" else ():
            name = f"cam{i}"
            decoded[name], traces[name], packets[name] = [], [], []
            if args.webrtc_decoder == "gstreamer":
                from real_camera_gstreamer import NativeObserver
                session = "real_camera_"+uuid.uuid4().hex
                answers[session] = asyncio.Queue()
                peer = NativeObserver(lambda image, metadata, now, mono, name=name:
                    native_frame(name, image, metadata, now, mono),
                    lambda timestamp, sequence, marker, size, name=name:
                    native_packet(name, timestamp, sequence, marker, size),
                    errors.append, decoder_threads=args.decoder_threads)
                peers[session] = peer
                publish(session, type="offer", sdp=await peer.offer(),
                    stream_topic=f"/horus/dataset/{name}/h264", image_type="compressed",
                    bitrate_kbps=2500, framerate=30, priority=4, stun_servers=[], wait_for_complete_gathering=True)
                while True:
                    response = await asyncio.wait_for(answers[session].get(), 20)
                    if response["type"] == "error":
                        raise RuntimeError(response)
                    if response["type"] == "answer":
                        await peer.answer(response["sdp"])
                        break
                continue
            peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
            session = "real_camera_"+uuid.uuid4().hex
            answers[session] = asyncio.Queue()
            peers[session] = peer

            @peer.on("track")
            def track_received(track, name=name):
                if track.kind == "video":
                    tasks.append(asyncio.create_task(video(name, track)))

            transceiver = peer.addTransceiver("video", direction="recvonly")
            receivers[name], arrivals[name] = transceiver.receiver, {}
            transceiver.setCodecPreferences([c for c in RTCRtpReceiver.getCapabilities("video").codecs if c.mimeType.lower() == "video/h264"])
            original = transceiver.receiver._handle_rtp_packet

            async def received(packet, *values, original=original, name=name, **kwargs):
                packets[name].append((time.monotonic(), packet.timestamp, packet.sequence_number, packet.marker, len(packet.payload)))
                now = time.time_ns()
                entry = arrivals[name].setdefault(packet.timestamp, dict(first=now, last=now))
                entry["last"] = now
                if len(arrivals[name]) > 512:
                    del arrivals[name][next(iter(arrivals[name]))]
                return await original(packet, *values, **kwargs)

            transceiver.receiver._handle_rtp_packet = received
            await peer.setLocalDescription(await peer.createOffer())
            publish(session, type="offer", sdp=peer.localDescription.sdp,
                stream_topic=f"/horus/dataset/{name}/h264", image_type="compressed",
                bitrate_kbps=2500, framerate=30, priority=4, stun_servers=[], wait_for_complete_gathering=True)
            while True:
                response = await asyncio.wait_for(answers[session].get(), 20)
                if response["type"] == "error":
                    raise RuntimeError(response)
                if response["type"] == "answer":
                    await peer.setRemoteDescription(RTCSessionDescription(response["sdp"], "answer"))
                    break
        deadline = time.monotonic()+15
        while not all(decoded.values()):
            if time.monotonic() > deadline:
                raise RuntimeError("Not all camera streams decoded")
            await asyncio.sleep(.05)
        await asyncio.sleep(args.warmup)
        measure_start = time.monotonic()
        measure_start_wall = time.time_ns()
        while not stop.is_set() and time.monotonic()-measure_start < args.duration:
            failures = [repr(t.exception()) for t in tasks if t.done() and not t.cancelled() and t.exception()]
            if failures or ui_errors or errors or source_state.get("errors"):
                raise RuntimeError(failures+ui_errors+errors+source_state.get("errors", []))
            await asyncio.sleep(.1)
        measure_end = time.monotonic()
        stats = [stat.__dict__ for peer in peers.values() for stat in (await peer.getStats()).values()
                 if stat.type == "inbound-rtp"]
        result = dict(condition=args.condition, transport=args.transport, decoder_threads=args.decoder_threads,
            webrtc_decoder=args.webrtc_decoder,
            cameras=args.cameras, width=640, height=480,
            measurement_start_monotonic_s=measure_start, measurement_end_monotonic_s=measure_end,
            measurement_start_wall_ns=measure_start_wall, measurement_end_wall_ns=time.time_ns(),
            duration_s=measure_end-measure_start,
            requested_duration_s=args.duration,
            completed=not stop.is_set() and measure_end-measure_start >= args.duration,
            decoded={name: summary([t for t in values if measure_start <= t <= measure_end]) for name, values in decoded.items()},
            rtp={name: dict(packets=len([p for p in values if measure_start <= p[0] <= measure_end]),
                unique_frame_timestamps=len({p[1] for p in values if measure_start <= p[0] <= measure_end}),
                payload_bytes=sum(p[4] for p in values if measure_start <= p[0] <= measure_end)) for name, values in packets.items()},
            compressed={name: dict(messages=len([p for p in values if measure_start <= p[0] <= measure_end]),
                payload_bytes=sum(p[1] for p in values if measure_start <= p[0] <= measure_end))
                for name, values in compressed_receipts.items()},
            clock=clock.best(time.time_ns()), inbound_stats=stats,
            viewer_logged=ui_counts, quality_samples=quality_saved,
            source_errors=source_state.get("errors", []), errors=errors)
        (args.output / "result.json").write_text(json.dumps(result, indent=2, default=str))
        print(json.dumps(result, default=str), flush=True)
    except Exception as error:
        errors.append(repr(error))
        (args.output / "failure.json").write_text(json.dumps(dict(errors=errors, source=source_state), indent=2))
        raise
    finally:
        condition_pub.publish(String(data=json.dumps(dict(condition="video_only", transport=args.transport,
            cameras=0, trial_id="idle_"+trial_id))))
        for session, peer in peers.items():
            publish(session, type="stop")
            await peer.close()
        stop.set()
        with jpeg_condition:
            jpeg_condition.notify_all()
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        await asyncio.to_thread(ros_worker.join, 5)
        if jpeg_worker:
            await asyncio.to_thread(jpeg_worker.join, 5)
        if worker:
            await asyncio.to_thread(worker.join, 15)
        (args.output / "source-status.json").write_text(json.dumps(statuses, indent=2))
        (args.output / "source-traces.json").write_text(json.dumps(traces))
        (args.output / "clock-samples.json").write_text(json.dumps(clock_samples))
        events_file.close()
        node.destroy_node()
        rclpy.shutdown()
        H264Decoder.__init__ = original_decoder_init


if __name__ == "__main__":
    main()
