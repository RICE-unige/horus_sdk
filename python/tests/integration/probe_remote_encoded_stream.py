"""Read-only live H.264/identity probe; does not publish poses or robot commands."""

import argparse
from fractions import Fraction
import json
import time

import av
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def marker_from_rgb(rgb, *, diagnostic=False):
    words = []
    candidates = []
    block_size = 32 if rgb.shape[1] >= 1536 else 8
    for row in (0, rgb.shape[0] - 8):
        word = 0
        contrast_valid = True
        minimum_contrast = 255.0
        for bit in range(24):
            x = bit * block_size * 2 + (block_size - 8) // 2
            first = rgb[row + 1:row + 8:2, x + 1:x + 8:2].mean()
            second = rgb[row + 1:row + 8:2, x + block_size + 1:x + block_size + 8:2].mean()
            contrast_valid = contrast_valid and abs(first - second) >= 32
            minimum_contrast = min(minimum_contrast, abs(first - second))
            word = (word << 1) | int(first > second)
        if contrast_valid and word >> 16 == 0xA5:
            words.append(word & 0xFFFF)
        candidates.append((hex(word), float(minimum_contrast)))
    if diagnostic:
        return candidates
    return words[0] if words and all(w == words[0] for w in words) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('horus_encoded_identity_probe')
    decoder = av.CodecContext.create('h264', 'r')
    metadata = {}
    counts = {'received': 0, 'decoded': 0, 'mismatches': 0, 'waiting_keyframe': 0}
    start = time.monotonic()

    def receive(message):
        counts['received'] += 1
        data = json.loads(message.header.frame_id.split(':', 1)[1])
        index = counts['received']
        metadata[index] = data
        packet = av.Packet(bytes(message.data))
        packet.pts = packet.dts = index
        packet.time_base = Fraction(1, 60)
        try:
            frames = decoder.decode(packet)
        except av.AVError:
            counts['waiting_keyframe'] += 1
            return
        for frame in frames:
            expected = metadata.get(frame.pts, {})
            actual = marker_from_rgb(frame.to_ndarray(format='rgb24'))
            counts['decoded'] += 1
            matches = actual == expected.get('frame_marker')
            counts['mismatches'] += not matches
            if not matches or counts['decoded'] % 60 == 1:
                stamp = message.header.stamp
                print(json.dumps({'image_marker': actual,
                    'expected_marker': expected.get('frame_marker'),
                    'frame': expected.get('frame_id'),
                    'source_view': expected.get('source_view_sequence'),
                    'publish_age_ms': (time.time() - stamp.sec - stamp.nanosec / 1e9) * 1000,
                    **counts}), flush=True)
        while len(metadata) > 128:
            metadata.pop(next(iter(metadata)))

    node.create_subscription(CompressedImage, '/horus/remote_render/map_portal', receive,
        QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(counts), flush=True)
    return 0 if counts['decoded'] and not counts['mismatches'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
