"""Owned two-hop RTP check: genuine paced input, three decoders, no ROS video relay."""

import argparse
from array import array
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--real-source", action="store_true")
    parser.add_argument("--profile-switch", action="store_true")
    parser.add_argument("--shared-encoders", action="store_true")
    parser.add_argument("--mixed-profiles", action="store_true")
    parser.add_argument("--reconnect", action="store_true")
    parser.add_argument("--link-budget-kbps", type=int, default=0)
    parser.add_argument("--operators", type=int, choices=(1, 2, 3), default=1)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("native_relay_input_check")
    stopped = threading.Event()
    children, streams, publishers = [], [], []
    prefix = "/horus/native_check"
    budgets = []
    budget_sub = node.create_subscription(String, "/horus/remote/media/bandwidth_budget",
        lambda message: budgets.append(json.loads(message.data)), 10)
    for index in (1, 2, 3):
        name = f"carter{index}"
        topic = f"/{name}/camera/color/image_raw" if args.real_source else f"{prefix}/{name}/rgb"
        streams.append(dict(id=name, input_topic=topic,
                            output_topic=f"{prefix}/{name}/h264", bitrate_kbps=1500, framerate=30))
        if not args.real_source:
            publishers.append(node.create_publisher(Image, topic,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)))
    logs = []
    try:
        config = args.output / "relay.json"
        config.write_text(json.dumps(dict(streams=streams,
            client_signal_topic="/horus/remote/webrtc/client_signal" if args.real_source else prefix+"/source/client_signal",
            server_signal_topic="/horus/remote/webrtc/server_signal" if args.real_source else prefix+"/source/server_signal")))
        binary = str(args.binary.resolve())

        def launch(label, arguments):
            stream = (args.output / (label + ".log")).open("w")
            logs.append(stream)
            process = subprocess.Popen([binary, "--ros-args", *arguments],
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(process)

        if not args.real_source:
            launch("source", ["-p", f"webrtc.client_signal_topic:={prefix}/source/client_signal",
                "-p", f"webrtc.server_signal_topic:={prefix}/source/server_signal",
                "-p", "webrtc.default_encoder:=x264enc", "-p",
                f"webrtc.share_encoders:={'true' if args.shared_encoders else 'false'}", "-p",
                f"webrtc.link_budget_kbps:={args.link_budget_kbps}", "-p",
                f"webrtc.bulk_budget_file:={args.output / 'bulk-budget.txt'}"])
        for operator in range(args.operators):
            suffix = "" if operator == 0 else f"/operator{operator+1}"
            launch(f"relay{operator+1}", ["-p", f"webrtc.client_signal_topic:={prefix}{suffix}/client_signal",
                "-p", f"webrtc.server_signal_topic:={prefix}{suffix}/server_signal",
                "-p", f"webrtc.relay_config:={config}"])
        counts = [0, 0, 0]

        def produce():
            deadline = time.monotonic()
            while not stopped.is_set():
                for index, publisher in enumerate(publishers):
                    if publisher.get_subscription_count():
                        shade = counts[index] % 128 + 64
                        pixel = bytes((shade, 64 + index * 50, 192))
                        message = Image(width=640, height=360, step=640*3, encoding="rgb8",
                            data=array("B", pixel * (640*360)))
                        message.header.stamp = node.get_clock().now().to_msg()
                        publisher.publish(message)
                        counts[index] += 1
                rclpy.spin_once(node, timeout_sec=0)
                deadline += 1/30
                now = time.monotonic()
                if now > deadline + 1/30:
                    deadline = now
                stopped.wait(max(0, deadline - now))

        worker = threading.Thread(target=produce)
        worker.start()
        time.sleep(2)
        assert all(process.poll() is None for process in children), "Native node startup failed"
        env = dict(os.environ, HORUS_HOSPITAL_WEBRTC_TEST="1", HORUS_TEST_RTC_NATIVE_RELAY="1",
            HORUS_TEST_RTC_CLIENT_TOPIC=prefix+"/client_signal",
            HORUS_TEST_RTC_SERVER_TOPIC=prefix+"/server_signal",
            HORUS_TEST_RTC_TOPIC_TEMPLATE=prefix+"/{name}/h264",
            HORUS_TEST_RTC_IMAGE_TYPE="compressed", HORUS_TEST_RTC_DUPLICATE_OFFERS="1",
            PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
        if args.profile_switch or args.link_budget_kbps:
            env["HORUS_TEST_RTC_TRACE"] = str(args.output / "rtp-trace.json")
            env["HORUS_TEST_RTC_REQUIRE_TRACE"] = "1"
        if args.profile_switch:
            env["HORUS_TEST_RTC_PROFILE_SWITCH"] = "1"
        if not args.real_source and not args.profile_switch:
            env["HORUS_TEST_RTC_MIN_FPS"] = "27"
            env["HORUS_TEST_RTC_FPS"] = "30"
        elif args.profile_switch:
            env["HORUS_TEST_RTC_FPS"] = "30"
        probes = []
        for operator in range(args.operators):
            suffix = "" if operator == 0 else f"/operator{operator+1}"
            probe_env = dict(env, HORUS_TEST_RTC_CLIENT_TOPIC=prefix+suffix+"/client_signal",
                HORUS_TEST_RTC_SERVER_TOPIC=prefix+suffix+"/server_signal")
            if args.mixed_profiles and operator == 0:
                probe_env["HORUS_TEST_RTC_PROFILE_SWITCH"] = "1"
                probe_env["HORUS_TEST_RTC_MIN_FPS"] = "0"
            probe_env["HORUS_TEST_RTC_PRIORITY"] = "4" if operator == 0 else "1"
            if args.profile_switch or args.link_budget_kbps:
                probe_env["HORUS_TEST_RTC_TRACE"] = str(args.output / f"rtp-trace-{operator+1}.json")
            stream = (args.output / ("decoder.log" if operator == 0 else f"decoder-{operator+1}.log")).open("w")
            logs.append(stream)
            process = subprocess.Popen([os.sys.executable, "-m", "pytest", "-q", "-s", "-o", "addopts=",
                str(Path(__file__).with_name("test_hospital_webrtc.py"))], env=probe_env,
                stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            probes.append(process); children.append(process)
        # Inspect source demand while all consumers are active, not after stop.
        time.sleep(10)
        source_subscriptions = {stream["input_topic"]: node.count_subscribers(stream["input_topic"])
                                for stream in streams}
        codes = [probe.wait(timeout=90) for probe in probes]
        reconnect_codes = []
        if args.reconnect:
            time.sleep(25)
            if not args.real_source:
                assert all(node.count_subscribers(stream["input_topic"]) == 0 for stream in streams), "Idle source encoders were not released"
            for operator in range(args.operators):
                suffix = "" if operator == 0 else f"/operator{operator+1}"
                probe_env = dict(env, HORUS_TEST_RTC_CLIENT_TOPIC=prefix+suffix+"/client_signal",
                    HORUS_TEST_RTC_SERVER_TOPIC=prefix+suffix+"/server_signal")
                stream = (args.output / f"reconnect-{operator+1}.log").open("w")
                logs.append(stream)
                process = subprocess.Popen([os.sys.executable, "-m", "pytest", "-q", "-s", "-o", "addopts=",
                    str(Path(__file__).with_name("test_hospital_webrtc.py"))], env=probe_env,
                    stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                children.append(process)
                reconnect_codes.append(process)
            reconnect_codes = [probe.wait(timeout=90) for probe in reconnect_codes]
        stopped.set()
        worker.join(timeout=5)
        # Alias topics are logical native streams, not publishers carrying video.
        aliases = {stream["output_topic"]: len(node.get_publishers_info_by_topic(stream["output_topic"]))
                   for stream in streams}
        assert all(value == 0 for value in aliases.values()), aliases
        report = dict(returncodes=codes, source_frames=counts, source_subscriptions=source_subscriptions,
                      encoded_ros_publishers=aliases, real_source=args.real_source,
                      shared_encoders=args.shared_encoders, operators=args.operators,
                      reconnect_returncodes=reconnect_codes, mixed_profiles=args.mixed_profiles)
        if args.link_budget_kbps:
            active = [value for value in budgets if len(value.get("streams", {})) == 3 * args.operators]
            (args.output / "budget-status.json").write_text(json.dumps(budgets, indent=2))
            assert active, "No aggregate budget for all active streams"
            for value in active:
                assert value["video_kbps"] + value["bulk_kbps"] + value["reserved_kbps"] <= args.link_budget_kbps
                assert value["video_kbps"] == sum(value["streams"].values())
                assert value["bulk_kbps"] >= 256
            expected = {1, 4} if args.operators > 1 else {4}
            settled = [value for value in active if {profile["priority"] for profile in value["profiles"].values()} == expected]
            assert settled, "Requested priorities never propagated through native relays"
            if args.operators > 1 and args.link_budget_kbps < args.operators * 4500 + 768:
                value = settled[-1]
                high = [value["streams"][key] for key, profile in value["profiles"].items() if profile["priority"] == 4]
                low = [value["streams"][key] for key, profile in value["profiles"].items() if profile["priority"] == 1]
                assert sum(high) / len(high) > sum(low) / len(low), "No usable priority allocation under shared load"
            assert int((args.output / "bulk-budget.txt").read_text()) > 0
            report["budget_samples"] = len(active)
            report["last_active_budget"] = active[-1]
        (args.output / "result.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
        assert all(code == 0 for code in codes), f"Decoder verification failed; see {args.output / 'decoder.log'}"
        assert all(code == 0 for code in reconnect_codes), "Reconnect verification failed"
        if args.shared_encoders and not args.real_source and not args.mixed_profiles and not args.link_budget_kbps:
            assert all(count == 1 for count in source_subscriptions.values()), source_subscriptions
    finally:
        stopped.set()
        if "worker" in locals():
            worker.join(timeout=5)
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(children):
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for stream in logs:
            stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
