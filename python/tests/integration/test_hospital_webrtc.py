"""Decode three actual hospital camera sessions concurrently; no robot commands."""

import asyncio
import json
import ipaddress
import os
import time
import uuid
from pathlib import Path

import pytest

if os.environ.get("HORUS_HOSPITAL_WEBRTC_TEST") != "1":
    pytest.skip("Requires the owned hospital demo and test-only aiortc environment", allow_module_level=True)

from aiortc import RTCPeerConnection, RTCConfiguration, RTCIceServer, RTCSessionDescription, RTCRtpReceiver
from aiortc.rtcdtlstransport import RTCDtlsTransport
import rclpy
from std_msgs.msg import String
from probe_remote_webrtc_stream import validate_legacy_certificate
from aiortc.codecs.h264 import H264Decoder


def test_three_immersive_camera_sessions_decode_real_hospital_frames():
    async def run():
        # Match the installed native TLS backend without disabling fingerprint checks.
        RTCDtlsTransport._validate_peer_identity = validate_legacy_certificate
        rclpy.init()
        node = rclpy.create_node("hospital_webrtc_check")
        client_topic = os.environ.get("HORUS_TEST_RTC_CLIENT_TOPIC", "/horus/webrtc/client_signal")
        server_topic = os.environ.get("HORUS_TEST_RTC_SERVER_TOPIC", "/horus/webrtc/server_signal")
        raw_source = os.environ.get("HORUS_TEST_RTC_RAW_SOURCE") == "1"
        public_path = os.environ.get("HORUS_TEST_RTC_PUBLIC_PATH") == "1"

        def public_candidates(sdp):
            lines = []
            for line in sdp.splitlines():
                if line.startswith("a=candidate:"):
                    address = ipaddress.ip_address(line.split()[4])
                    if address in ipaddress.ip_network("100.64.0.0/10") or address in ipaddress.ip_network("fd7a:115c:a1e0::/48"):
                        continue
                lines.append(line)
            return "\r\n".join(lines) + "\r\n"
        publisher = node.create_publisher(String, client_topic, 10)
        queues, peers, tasks, frames, connected, image_stats = {}, {}, [], {}, {}, {}
        profile_capabilities, profile_windows = set(), []
        rtp_trace = {}
        decode_trace = []
        source_traces = {}
        original_decode = H264Decoder.decode
        if os.environ.get("HORUS_TEST_RTC_TRACE"):
            def trace_decode(self, encoded_frame):
                result = original_decode(self, encoded_frame)
                decode_trace.append(dict(time=time.monotonic(), decoder=id(self),
                    timestamp=encoded_frame.timestamp, size=len(encoded_frame.data), frames=len(result)))
                return result
            H264Decoder.decode = trace_decode

        def receive(message):
            value = json.loads(message.data)
            if value.get("type") == "ready" and value.get("profile_updates"):
                profile_capabilities.add(value.get("session_id"))
            queue = queues.get(value.get("session_id"))
            if queue is not None:
                queue.put_nowait(value)

        subscription = node.create_subscription(String, server_topic, receive, 10)

        async def spin():
            while True:
                rclpy.spin_once(node, timeout_sec=0)
                await asyncio.sleep(.001)

        def signal(session, **values):
            publisher.publish(String(data=json.dumps(dict(session_id=session, **values))))

        async def video(name, track):
            while True:
                frame = await track.recv()
                frames[name].append((time.monotonic(), frame.width, frame.height))
                for side in frame.side_data:
                    data = bytes(side)
                    marker = data.find(b"HORUS-RR-V3-META")
                    if marker >= 0:
                        try:
                            trace, _ = json.JSONDecoder().raw_decode(data[marker + 16:].decode("utf-8"))
                        except (ValueError, UnicodeDecodeError):
                            continue
                        if trace.get("kind") == "horus_camera_frame_v1":
                            source_traces.setdefault(name, []).append(trace)
                if len(frames[name]) == 1:
                    rgb = await asyncio.to_thread(frame.to_ndarray, format="rgb24")
                    assert rgb.shape == (360,640,3) and rgb.dtype.name == "uint8"
                    # A dark corridor is valid camera content, not decoder failure.
                    image_stats[name] = dict(first_frame_mean=float(rgb.mean()),
                                            first_frame_std=float(rgb.std()))

        tasks.append(asyncio.create_task(spin()))
        try:
            deadline = time.monotonic()+10
            while publisher.get_subscription_count() == 0 or subscription.get_publisher_count() == 0:
                assert time.monotonic() < deadline, "Bridge signaling unavailable"
                await asyncio.sleep(.05)
            template = os.environ.get("HORUS_TEST_RTC_TOPIC_TEMPLATE", "")
            if template and os.environ.get("HORUS_TEST_RTC_NATIVE_RELAY") != "1":
                # The signaling bridge can outlive a restarted media relay.
                # Wait for its publishers, not frames: do not prewarm the video.
                deadline = time.monotonic() + 15
                while not all(node.get_publishers_info_by_topic(template.format(name=name))
                              for name in ("carter1", "carter2", "carter3")):
                    assert time.monotonic() < deadline, "Camera source publishers unavailable"
                    await asyncio.sleep(.05)
            for name in ("carter1", "carter2", "carter3"):
                peer = RTCPeerConnection(RTCConfiguration(iceServers=(
                    [RTCIceServer(urls="stun:stun.l.google.com:19302")] if public_path else [])))
                session = "hospital_check_"+uuid.uuid4().hex
                peers[session] = peer
                queues[session] = asyncio.Queue()
                frames[name] = []

                @peer.on("connectionstatechange")
                def connection_state(peer=peer, name=name):
                    if peer.connectionState == "connected":
                        connected.setdefault(name, time.monotonic())

                @peer.on("track")
                def on_track(track, name=name):
                    if track.kind == "video":
                        tasks.append(asyncio.create_task(video(name, track)))

                transceiver = peer.addTransceiver("video", direction="recvonly")
                if os.environ.get("HORUS_TEST_RTC_TRACE"):
                    receiver = transceiver.receiver
                    original = receiver._handle_rtp_packet
                    async def trace(packet, *args, original=original, name=name, **kwargs):
                        rtp_trace.setdefault(name, []).append(dict(time=time.monotonic(),
                            timestamp=packet.timestamp, sequence=packet.sequence_number,
                            marker=packet.marker, payload_head=packet.payload[:2].hex()))
                        return await original(packet, *args, **kwargs)
                    receiver._handle_rtp_packet = trace
                transceiver.setCodecPreferences([c for c in RTCRtpReceiver.getCapabilities("video").codecs
                                                if c.mimeType.lower() == "video/h264"])
                await peer.setLocalDescription(await peer.createOffer())
                offer = dict(type="offer", sdp=public_candidates(peer.localDescription.sdp) if public_path else peer.localDescription.sdp,
                    stream_topic=os.environ.get("HORUS_TEST_RTC_TOPIC_TEMPLATE", "").format(name=name) or
                        f"/{name}/camera/color/image_raw" + ("" if raw_source else "/compressed"),
                    image_type=os.environ.get("HORUS_TEST_RTC_IMAGE_TYPE") or ("raw" if raw_source else "compressed"),
                    bitrate_kbps=1500, framerate=int(os.environ.get("HORUS_TEST_RTC_FPS", "15")),
                    priority=int(os.environ.get("HORUS_TEST_RTC_PRIORITY", "1")),
                    stun_servers=["stun:stun.l.google.com:19302"] if public_path else [],
                    wait_for_complete_gathering=True)
                signal(session, **offer)
                if os.environ.get("HORUS_TEST_RTC_DUPLICATE_OFFERS") == "1":
                    await asyncio.sleep(.02)
                    signal(session, **offer)
                while True:
                    answer = await asyncio.wait_for(queues[session].get(), 15)
                    assert answer["type"] != "error", answer
                    if answer["type"] == "answer":
                        await peer.setRemoteDescription(RTCSessionDescription(
                            public_candidates(answer["sdp"]) if public_path else answer["sdp"], "answer"))
                        break
            if os.environ.get("HORUS_TEST_RTC_PROFILE_SWITCH") == "1":
                # Measure steady-state profile rates, not initial ICE/IDR wait.
                ready_until = time.monotonic() + 3
                while not all(frames.values()) and time.monotonic() < ready_until:
                    await asyncio.sleep(.02)
                assert all(frames.values()), "No initial decoded video before profile measurement"
                for fps in (30, 15, 30):
                    for session in peers:
                        assert session in profile_capabilities, "Native profile capability was not negotiated"
                        signal(session, type="profile", bitrate_kbps=1500, framerate=fps,
                            priority=int(os.environ.get("HORUS_TEST_RTC_PRIORITY", "1")))
                    await asyncio.sleep(1)
                    start = time.monotonic()
                    await asyncio.sleep(5)
                    end = time.monotonic()
                    rates = {name: sum(start <= t <= end for t, _, _ in values) / (end-start)
                             for name, values in frames.items()}
                    profile_windows.append(dict(requested_fps=fps, received_fps=rates,
                                                start=start, end=end))
                    for name, rate in rates.items():
                        assert fps * .85 <= rate <= fps * 1.15, (name, fps, rate)
                print("PROFILE_WINDOWS " + json.dumps(profile_windows))
            else:
                await asyncio.sleep(20)
            errors = [t.exception() for t in tasks if t.done() and not t.cancelled() and t.exception()]
            assert not errors, errors
            stats = [await peer.getStats() for peer in peers.values()]
            for name, values in frames.items():
                if not values:
                    print(json.dumps(dict(camera=name, decoded=0)))
                    continue
                hz = (len(values)-1)/(values[-1][0]-values[0][0])
                startup = values[0][0]-connected[name]
                print(json.dumps(dict(camera=name, decoded=len(values), decoded_receipt_fps=hz,
                                      connected_to_first_frame_s=startup, **image_stats[name])))
            print("RTC_STATS " + json.dumps([stat.__dict__ for report in stats for stat in report.values()
                  if stat.type == "inbound-rtp"], default=str))
            print("SOURCE_TRACES " + json.dumps({name: len(values) for name, values in source_traces.items()}))
            if os.environ.get("HORUS_TEST_RTC_REQUIRE_TRACE") == "1":
                assert all(source_traces.get(name) for name in frames), source_traces
                for values in source_traces.values():
                    assert all(value["stamp_clock"] == "ros_unsynchronized" for value in values)
                    assert all(value["adapter_monotonic_ns"] > 0 and value["ros_stamp_ns"] > 0 for value in values)
            for name, values in frames.items():
                assert len(values) >= 60, (name, len(values))
                minimum_fps = float(os.environ.get("HORUS_TEST_RTC_MIN_FPS", "0"))
                if minimum_fps:
                    hz = (len(values) - 1) / (values[-1][0] - values[0][0])
                    assert hz >= minimum_fps, (name, hz, minimum_fps)
                assert all((w,h) == (640,360) for _,w,h in values)
                startup = values[0][0]-connected[name]
                assert startup < 2, (name, "Connected video startup exceeded two seconds", startup)
        finally:
            if os.environ.get("HORUS_TEST_RTC_TRACE"):
                Path(os.environ["HORUS_TEST_RTC_TRACE"]).write_text(json.dumps(
                    dict(rtp=rtp_trace, profiles=profile_windows, decode=decode_trace, source_frames=source_traces)))
            for session, peer in peers.items():
                signal(session, type="stop")
                await peer.close()
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            H264Decoder.decode = original_decode
            node.destroy_node()
            rclpy.shutdown()

    asyncio.run(run())
