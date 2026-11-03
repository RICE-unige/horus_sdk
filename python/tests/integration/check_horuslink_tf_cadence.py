"""Read-only TF cadence at an actual HorusLink client, separate from bulk parsing.

Receipt gaps are not synchronized source age or Quest display measurements.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import time

from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from test_teach_demo_link import Peer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=10000)
    parser.add_argument("--duration", type=float, default=45)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", nargs="+", required=True)
    args = parser.parse_args()
    peer = Peer(args.port)
    samples = defaultdict(list)
    try:
        peer.channel(1, "/tf", message="tf2_msgs/msg/TFMessage")
        start, consumed = time.monotonic(), 0
        while time.monotonic() - start < args.duration:
            peer.poll(.005)
            messages = peer.messages.get(1, [])
            for payload in messages[consumed:]:
                message = deserialize_message(b"\x00\x01\x00\x00" + payload, TFMessage)
                now = time.monotonic()
                for transform in message.transforms:
                    if transform.child_frame_id in args.frames:
                        samples[transform.child_frame_id].append(now)
            consumed = len(messages)
        report = {"qualification": __doc__, "duration_s": args.duration, "frames": {}}
        for frame in args.frames:
            times = samples[frame]
            assert len(times) > 1, (frame, len(times))
            gaps = sorted(b - a for a, b in zip(times, times[1:]))
            report["frames"][frame] = dict(received=len(times), receipt_hz=(len(times) - 1) / (times[-1] - times[0]),
                p95_gap_ms=gaps[min(len(gaps) - 1, int(len(gaps) * .95))] * 1000, max_gap_ms=max(gaps) * 1000)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))
    finally:
        peer.close()


if __name__ == "__main__":
    main()
