"""Verify the running fleet authority using an isolated, unbound probe robot.

Commands use a unique /horus/transport_check topic, never a hardware topic.
Received ROS messages must travel from the fleet authority back over the real
connector. This is a functional check, not synchronized one-way latency.
"""
import argparse
import json
from pathlib import Path
import struct
import time
import uuid

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from test_teach_demo_link import Peer, tlv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("horus_control_transport_check")
    name = "transport_probe_" + uuid.uuid4().hex[:12]
    topic = f"/horus/transport_check/{name}/cmd_vel"
    samples, status = [], {}
    subscriptions = [node.create_subscription(Twist, topic,
        lambda value: samples.append((time.monotonic(), value.linear.x)), 10),
        node.create_subscription(String, "/horus/remote/control/proxy/status",
            lambda value: status.update(json.loads(value.data)), 10)]
    peers = [Peer(args.port), Peer(args.port)]
    offsets = {}

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            for peer in peers:
                peer.poll(.002)
            rclpy.spin_once(node, timeout_sec=.002)

    def wait(predicate, timeout=12):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            pump(.02)
        assert predicate(), (status, samples)

    def request(peer, app, action):
        peer.publish(3, dict(robot_name=name, app_id=app, action=action,
            role="private", panel_open=True, teleop_active=True))

    def holder(app):
        return any(lease.get("robot_name") == name and lease.get("holder_app_id") == app
                   for lease in peers[0].value(1).get("leases", []))

    def motion(peer, speed, seconds=.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic_ns() // 1000000
            prefix = struct.pack("<QQ", now + offsets[peer] + 170, now)
            peer.send(4, 1, prefix + struct.pack("<6d", speed, 0, 0, 0, 0, 0), flags=79)
            pump(1/30)

    try:
        wait(lambda: any(link.get("clock_valid") for link in status.get("links", [])))
        for peer in peers:
            wait(lambda: bool(peer.hellos))
            assert any(struct.unpack("<I", hello.get(22, b"\0" * 4))[0] & 1 for hello in peer.hellos)
            now = time.monotonic_ns() // 1000000
            peer.send(0, 5, b"HLT1" + struct.pack("<QQ", now, 0))
            wait(lambda: bool(peer.clock_responses))
            sent, received, remote = peer.clock_responses[-1]
            assert received - sent < 100, (sent, received)
            offsets[peer] = remote - (sent + received) // 2
            peer.channel(1, "/horus/multi_operator/control_lease_state")
            peer.channel(2, "/horus/multi_operator/control_topic_catalog", publish=True)
            peer.channel(3, "/horus/multi_operator/control_lease_request", publish=True)
            peer.publish(2, dict(role="private", op="upsert", replace_client_catalog=True,
                robots=[dict(robot_name=name, protected_topics=[topic])]))
            peer.send(0, 2, tlv(1, b"\x05") + tlv(2, struct.pack("<H", 1)) +
                tlv(4, struct.pack("<H", 4)) + tlv(5, topic.encode()) +
                tlv(6, b"geometry_msgs/msg/Twist") + tlv(7, b"\x01") + tlv(8, b"\x02"))
            peer.wait(lambda: 4 in peer.acks)
        pump(.25)
        request(peers[0], "probe_one", "acquire")
        wait(lambda: holder("probe_one"))
        now = time.monotonic_ns() // 1000000
        expired = struct.pack("<QQ", now + offsets[peers[0]] - 1000, now - 2000)
        peers[0].send(4, 1, expired + struct.pack("<6d", .625, 0, 0, 0, 0, 0), flags=79)
        pump(.3)
        assert not any(abs(value - .625) < 1e-6 for _, value in samples), samples
        request(peers[1], "probe_two", "acquire")
        pump(.3)
        motion(peers[1], .75)
        assert not any(abs(value-.75) < 1e-6 for _, value in samples), samples
        motion(peers[0], .25, 1)
        wait(lambda: any(abs(value-.25) < 1e-6 for _, value in samples))
        wait(lambda: samples and abs(samples[-1][1]) < 1e-6)
        request(peers[0], "probe_one", "release")
        pump(.25)
        request(peers[1], "probe_two", "acquire")
        wait(lambda: holder("probe_two"))
        motion(peers[1], .5, 1)
        wait(lambda: any(abs(value-.5) < 1e-6 for _, value in samples))
        wait(lambda: samples and abs(samples[-1][1]) < 1e-6)
        request(peers[1], "probe_two", "release")
        pump(.3)
        result = dict(probe_robot=name, probe_topic=topic, received_source_commands=len(samples),
            speeds=sorted(set(value for _, value in samples)), status=status,
            denied_peer_never_published=True, expiry_stopped_motion=True,
            source_authority_handoff_verified=True,
            timed_local_input_verified=True, expired_local_input_rejected=True,
            qualification="Real authority/connector round trip on an unbound topic; not hardware or one-way latency")
        args.output.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))
    finally:
        for peer in peers:
            peer.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
