"""Opt-in live bridge probe. Receives video only; never sends poses or commands."""

import argparse
import asyncio
from collections import OrderedDict
import json
import time
import uuid

from aiortc import RTCPeerConnection, RTCConfiguration, RTCSessionDescription, RTCRtpReceiver
from aiortc.rtcdtlstransport import RTCDtlsTransport, X509_DIGEST_ALGORITHMS, State
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from probe_remote_encoded_stream import marker_from_rgb


def validate_legacy_certificate(self, parameters):
    # libdatachannel's installed TLS backend emits an X509 v2 certificate.
    # Keep fingerprint verification, using OpenSSL rather than cryptography's
    # stricter certificate-version parser. This affects this diagnostic only.
    certificate = self._ssl.get_peer_certificate()
    fingerprints = [f for f in parameters.fingerprints
                    if f.algorithm.lower() in X509_DIGEST_ALGORITHMS]
    if not fingerprints or any(certificate.digest(f.algorithm.replace('-', '')).decode().upper()
                               != f.value.upper() for f in fingerprints):
        self._set_state(State.FAILED)


async def run(seconds):
    RTCDtlsTransport._validate_peer_identity = validate_legacy_certificate
    rclpy.init()
    node = rclpy.create_node('horus_bridge_video_probe')
    publisher = node.create_publisher(String, '/horus/webrtc/client_signal', 10)
    peer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    session = 'diagnostic_' + uuid.uuid4().hex
    answer = asyncio.Queue()
    metadata = OrderedDict()
    counts = {'decoded': 0, 'invalid_marker': 0, 'unmatched': 0}
    ages = []
    tasks = []
    inbound_stats = []

    def inspect_frame(frame):
        rgb = frame.to_ndarray(format='rgb24')
        marker = marker_from_rgb(rgb)
        diagnostics = marker_from_rgb(rgb, diagnostic=True) if marker is None else None
        return marker, diagnostics

    def signal(data):
        publisher.publish(String(data=json.dumps({'session_id': session, **data})))

    def receive_signal(message):
        value = json.loads(message.data)
        if value.get('session_id') == session:
            answer.put_nowait(value)

    def receive_metadata(message):
        value = json.loads(message.header.frame_id.split(':', 1)[1])
        stamp = message.header.stamp
        metadata[value['frame_marker']] = (value, stamp.sec + stamp.nanosec / 1e9)
        while len(metadata) > 512:
            metadata.popitem(last=False)

    signal_subscription = node.create_subscription(String, '/horus/webrtc/server_signal', receive_signal, 10)
    node.create_subscription(CompressedImage, '/horus/remote_render/map_portal', receive_metadata,
        QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT))

    async def spin():
        while True:
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(0.001)

    async def video(track):
        while True:
            frame = await track.recv()
            decoded_at = time.time()
            # RGB conversion must not block ICE/RTP reception on this event loop.
            marker, diagnostics = await asyncio.to_thread(inspect_frame, frame)
            if marker is None:
                print(json.dumps({'marker_failure': diagnostics}), flush=True)
            counts['decoded'] += 1
            counts['invalid_marker'] += marker is None
            expected = metadata.get(marker)
            # This diagnostic compares two independent arrivals: RTP and ROS.
            # Allow the reference ROS callback to catch up without charging that
            # wait to measured decoder latency.
            deadline = time.monotonic() + .25
            while marker is not None and expected is None and time.monotonic() < deadline:
                await asyncio.sleep(.002)
                expected = metadata.get(marker)
            counts['unmatched'] += expected is None
            age = (decoded_at - expected[1]) * 1000 if expected else None
            if age is not None:
                ages.append(age)
            if counts['decoded'] % 120 == 1:
                print(json.dumps({**counts, 'marker': marker, 'publish_age_ms': age,
                    'frame': expected[0]['frame_id'] if expected else None}), flush=True)

    @peer.on('track')
    def on_track(track):
        if track.kind == 'video':
            tasks.append(asyncio.create_task(video(track)))

    tasks.append(asyncio.create_task(spin()))
    transceiver = peer.addTransceiver('video', direction='recvonly')
    transceiver.setCodecPreferences([codec for codec in RTCRtpReceiver.getCapabilities('video').codecs
        if codec.mimeType.lower() == 'video/h264'])
    try:
        await peer.setLocalDescription(await peer.createOffer())
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() == 0 or signal_subscription.get_publisher_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('Bridge signaling endpoints were not discovered within 10 seconds')
            await asyncio.sleep(.05)
        print(json.dumps({'signal_subscribers': publisher.get_subscription_count(),
            'signal_publishers': signal_subscription.get_publisher_count()}), flush=True)
        signal({'type': 'offer', 'sdp': peer.localDescription.sdp,
            'stream_topic': '/horus/remote_render/map_portal', 'image_type': 'h264',
            'encoder': 'passthrough', 'bitrate_kbps': 35000, 'framerate': 60,
            'stun_servers': [], 'wait_for_complete_gathering': True})
        while True:
            message = await asyncio.wait_for(answer.get(), 15)
            if message['type'] == 'error':
                raise RuntimeError(message)
            if message['type'] == 'answer':
                await peer.setRemoteDescription(RTCSessionDescription(message['sdp'], 'answer'))
                break
        await asyncio.sleep(seconds)
        inbound_stats = [vars(value) for value in (await peer.getStats()).values()
            if value.type == 'inbound-rtp']
    finally:
        signal({'type': 'stop'})
        await peer.close()
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        node.destroy_node()
        rclpy.shutdown()
    ages.sort()
    print(json.dumps({**counts, 'age_p50_ms': ages[len(ages) // 2] if ages else None,
        'age_p95_ms': ages[int(len(ages) * .95)] if ages else None,
        'age_max_ms': max(ages) if ages else None,
        'inbound_rtp': inbound_stats}, default=str), flush=True)
    return 0 if counts['decoded'] and not counts['invalid_marker'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=90)
    raise SystemExit(asyncio.run(run(parser.parse_args().seconds)))
