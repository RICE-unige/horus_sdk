"""Time-bounded native WebRTC receiver for the real-camera diagnostic."""

import asyncio
import json
import threading
import time

import gi
import numpy as np

gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")
gi.require_version("GstRtp", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst, GstRtp, GstSdp, GstVideo, GstWebRTC


class NativeObserver:
    def __init__(self, on_frame, on_packet, on_error, decoder_threads=1):
        Gst.init(None)
        for name in ("webrtcbin", "nicesrc", "nicesink", "rtph264depay", "h264parse", "avdec_h264"):
            if Gst.ElementFactory.find(name) is None:
                raise RuntimeError(f"Missing GStreamer {name}; use the connector's validated plugin environment")
        self.on_frame, self.on_packet, self.on_error = on_frame, on_packet, on_error
        self.closed = False
        self.metadata = {}
        self.pipeline = Gst.Pipeline.new(None)
        self.peer = Gst.ElementFactory.make("webrtcbin", "peer")
        self.peer.set_property("bundle-policy", GstWebRTC.WebRTCBundlePolicy.MAX_BUNDLE)
        self.peer.set_property("latency", 30)
        self.pipeline.add(self.peer)
        rtpbin = self.peer.get_by_name("rtpbin")
        if rtpbin:
            rtpbin.connect("new-jitterbuffer", self.jitterbuffer)
        self.decoder = Gst.parse_bin_from_description(
            "rtph264depay request-keyframe=true wait-for-keyframe=true ! "
            "video/x-h264,alignment=au ! h264parse name=parser ! avdec_h264 max-threads="+str(decoder_threads)+
            " ! videoconvert ! video/x-raw,format=RGB ! "
            "appsink name=frames emit-signals=true sync=false max-buffers=1 drop=true", True)
        self.pipeline.add(self.decoder)
        self.peer.connect("pad-added", self.pad_added)
        self.decoder.get_by_name("parser").get_static_pad("src").add_probe(
            Gst.PadProbeType.BUFFER, self.encoded)
        self.decoder.get_by_name("frames").connect("new-sample", self.decoded)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message::error", self.bus_error)
        self.mainloop = GLib.MainLoop()
        self.thread = threading.Thread(target=self.mainloop.run, daemon=True)
        caps = Gst.Caps.from_string("application/x-rtp,media=video,encoding-name=H264,clock-rate=90000,"
                                   "payload=96,packetization-mode=(string)1,profile-level-id=(string)42e01f,"
                                   "rtcp-fb-nack=(boolean)true,rtcp-fb-nack-pli=(boolean)true")
        transceiver = self.peer.emit("add-transceiver", GstWebRTC.WebRTCRTPTransceiverDirection.RECVONLY, caps)
        if transceiver and transceiver.find_property("do-nack"):
            transceiver.set_property("do-nack", True)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.closed = True
            self.pipeline.set_state(Gst.State.NULL)
            self.bus.remove_signal_watch()
            raise RuntimeError("Native observer pipeline could not start")
        self.thread.start()

    def bus_error(self, _bus, message):
        error, details = message.parse_error()
        self.on_error(f"GStreamer receiver: {error}: {details}")

    def jitterbuffer(self, _rtpbin, buffer, _session, _ssrc):
        if buffer.find_property("rtx-next-seqnum"):
            buffer.set_property("rtx-next-seqnum", False)

    def pad_added(self, _peer, pad):
        if pad.get_direction() != Gst.PadDirection.SRC or self.closed:
            return
        sink = self.decoder.get_static_pad("sink")
        if sink.is_linked():
            return
        pad.add_probe(Gst.PadProbeType.BUFFER, self.rtp)
        result = pad.link(sink)
        if result != Gst.PadLinkReturn.OK:
            self.on_error(f"Native observer RTP link failed: {result}")

    def rtp(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is not None and not self.closed:
            ok, rtp = GstRtp.RTPBuffer.map(buffer, Gst.MapFlags.READ)
            if ok:
                self.on_packet(rtp.get_timestamp(), rtp.get_seq(), rtp.get_marker(), rtp.get_payload_len())
                rtp.unmap()
        return Gst.PadProbeReturn.OK

    def encoded(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is not None and not self.closed:
            raw = buffer.extract_dup(0, buffer.get_size())
            marker = raw.find(b"HORUS-RR-V3-META")
            if marker >= 0:
                try:
                    metadata, _ = json.JSONDecoder().raw_decode(raw[marker+16:].decode("utf-8", errors="replace"))
                    if metadata.get("kind") == "horus_camera_frame_v1":
                        self.metadata[buffer.pts] = metadata
                        if len(self.metadata) > 512:
                            del self.metadata[next(iter(self.metadata))]
                except ValueError:
                    pass
        return Gst.PadProbeReturn.OK

    def decoded(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None or self.closed:
            return Gst.FlowReturn.OK
        try:
            buffer = sample.get_buffer()
            video = GstVideo.VideoInfo.new_from_caps(sample.get_caps())
            if video.width != 640 or video.height != 480:
                raise ValueError("Native observer received unexpected frame dimensions")
            raw = buffer.extract_dup(0, buffer.get_size())
            image = np.ndarray((video.height, video.width, 3), dtype=np.uint8,
                buffer=raw, offset=video.offset[0], strides=(video.stride[0], 3, 1)).copy()
            self.on_frame(image, self.metadata.pop(buffer.pts, None), time.time_ns(), time.monotonic_ns())
        except Exception as error:
            self.on_error(repr(error))
        return Gst.FlowReturn.OK

    async def offer(self):
        loop = asyncio.get_running_loop()
        ready = loop.create_future()

        def created(promise, *_):
            try:
                reply = promise.get_reply()
                if reply.has_field("error"):
                    raise RuntimeError(str(reply.get_value("error")))
                description = reply.get_value("offer")

                def local_set(result, *_):
                    reply = result.get_reply()
                    if reply is not None and reply.has_field("error"):
                        loop.call_soon_threadsafe(ready.set_exception, RuntimeError(str(reply.get_value("error"))))
                    else:
                        loop.call_soon_threadsafe(ready.set_result, True)

                self.peer.emit("set-local-description", description,
                    Gst.Promise.new_with_change_func(local_set, None, None))
            except Exception as error:
                loop.call_soon_threadsafe(ready.set_exception, error)

        self.peer.emit("create-offer", None, Gst.Promise.new_with_change_func(created, None, None))
        await asyncio.wait_for(ready, 10)
        deadline = time.monotonic()+10
        while self.peer.get_property("ice-gathering-state") != GstWebRTC.WebRTCICEGatheringState.COMPLETE:
            if time.monotonic() > deadline:
                state = self.peer.get_property("ice-gathering-state")
                raise TimeoutError(f"Native observer ICE gathering did not complete: {state}")
            await asyncio.sleep(.02)
        return self.peer.get_property("local-description").sdp.as_text()

    async def answer(self, text):
        result, sdp = GstSdp.SDPMessage.new_from_text(text)
        if result != GstSdp.SDPResult.OK:
            raise ValueError("Invalid native receiver SDP answer")
        # libdatachannel inherits transport attributes at session level;
        # GStreamer needs them expanded per media, as in the native HORUS relay.
        for index in range(sdp.medias_len()):
            media = sdp.get_media(index)
            for name in ("setup", "ice-ufrag", "ice-pwd"):
                value = sdp.get_attribute_val(name)
                if value and not media.get_attribute_val(name):
                    media.add_attribute(name, value)
        description = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdp)
        self.peer.emit("set-remote-description", description, Gst.Promise.new())

    async def getStats(self):
        return {}

    async def close(self):
        self.closed = True
        self.pipeline.set_state(Gst.State.NULL)
        self.bus.remove_signal_watch()
        self.mainloop.quit()
        await asyncio.to_thread(self.thread.join, 3)
