"""Join replay and received identities without assuming synchronized PC clocks."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from real_camera_data import ClockEstimate, payload_integrity, summary, window_summary


def analyze(run, source_events):
    result = json.loads((run / "result.json").read_text())
    traces = json.loads((run / "source-traces.json").read_text())
    estimates = json.loads((run / "clock-samples.json").read_text())
    clock = ClockEstimate()
    samples = []
    for value in estimates:
        clock.add(value["sent_ns"], value["received_ns"], value["source_received_ns"], value["source_sent_ns"])
        best = clock.best(value["received_ns"])
        if best:
            samples.append(best)
    if not samples:
        raise ValueError("No bounded cross-PC clock estimate; cannot compare aligned source windows")
    start, end = result["measurement_start_wall_ns"], result["measurement_end_wall_ns"]
    offset = min(samples, key=lambda x: x[2])[1]
    if isinstance(source_events, Path):
        source_events = [source_events]
    events = [row for path in source_events for row in csv.DictReader(path.open())]
    received = list(csv.DictReader((run / "received_events.csv").open()))
    transport = result.get("transport", "webrtc")
    start_mono = result.get("measurement_start_monotonic_s")
    end_mono = result.get("measurement_end_monotonic_s")
    cameras = {}
    for camera, decoded in result["decoded"].items():
        decoded = dict(decoded, window_fps=decoded["count"]/result["duration_s"]) if "count" in decoded else decoded
        source = [e for e in events if e["kind"] == ("jpeg" if transport == "ros" else "rgb") and e["stream"] == camera[3:]
                  and start+offset <= int(e["publish_wall_ns"]) <= end+offset]
        source_stats = summary([int(e["publish_monotonic_ns"])/1e9 for e in source])
        source_stats["window_fps"] = len(source)/result["duration_s"]
        eligible = {int(e["publish_wall_ns"]) for e in source
                    if start+offset+2e9 <= int(e["publish_wall_ns"]) <= end+offset-2e9}
        delivered = {int(t["ros_stamp_ns"]) for t in traces[camera]}
        ages, uncertainty = [], []
        for trace in traces[camera]:
            now = trace["receipt_wall_ns"]
            if not start <= now <= end:
                continue
            recent = [s for s in samples if 0 <= now-s[0] <= 15e9]
            if recent:
                estimate = min(recent, key=lambda s: s[2])
                ages.append((now+estimate[1]-trace["ros_stamp_ns"])/1e6)
                uncertainty.append(estimate[2]/1e6)
        video = [int(e["receipt_monotonic_ns"])/1e9 for e in received if e["kind"] == "video" and e["stream"] == camera]
        if start_mono is not None and end_mono is not None:
            decoded = window_summary(video, start_mono, end_mono)
        payload = result.get("compressed" if transport == "ros" else "rtp", {}).get(camera, {}).get("payload_bytes", 0)
        cameras[camera] = dict(source=source_stats, decoded=decoded,
            eligible_source_frames=len(eligible), matching_unique_decoded_frames=len(eligible & delivered),
            source_identity_delivery_percent=100*len(eligible & delivered)/len(eligible) if eligible else None,
            source_trace_count=len(traces[camera]),
            received_rtp_payload_kbps=result.get("rtp", {}).get(camera, {}).get("payload_bytes", 0)*8/1000/result["duration_s"],
            received_payload_kbps=payload*8/1000/result["duration_s"],
            source_publication_payload_kbps=sum(int(e["bytes"]) for e in source)*8/1000/result["duration_s"],
            replay_publish_to_decode_ms={f"p{p}": float(np.percentile(ages, p)) for p in (50,95,99)} if ages else {},
            max_clock_uncertainty_ms=max(uncertainty) if uncertainty else None,
            latency_definition="Replay ROS publication to this PC decoder output, offset-estimated; not glass-to-glass")
    sensors = {}
    for kind in ("tf", "imu", "depth", "cloud"):
        raw = [e for e in received if e["kind"] == kind and start <= int(e["receipt_wall_ns"]) <= end]
        seen = {}
        for event in raw:
            seen.setdefault(event["source_publish_wall_ns"], event)
        values = list(seen.values())
        source = [e for e in events if e["kind"] == kind and start+offset <= int(e["publish_wall_ns"]) <= end+offset]
        receipt_stats = summary([int(e["receipt_monotonic_ns"])/1e9 for e in values])
        receipt_stats["window_fps"] = len(values)/result["duration_s"]
        if start_mono is not None and end_mono is not None:
            receipt_stats = window_summary([int(e["receipt_monotonic_ns"])/1e9 for e in values], start_mono, end_mono)
        source_stats = summary([int(e["publish_monotonic_ns"])/1e9 for e in source])
        source_stats["window_fps"] = len(source)/result["duration_s"]
        ages = []
        for event in values:
            now = int(event["receipt_wall_ns"])
            recent = [s for s in samples if 0 <= now-s[0] <= 15e9]
            if recent:
                estimate = min(recent, key=lambda s: s[2])
                ages.append((now+estimate[1]-int(event["source_publish_wall_ns"]))/1e6)
        sensors[kind] = dict(received=receipt_stats,
            raw_received_messages=len(raw), duplicate_messages=len(raw)-len(values),
            source=source_stats, replay_stamp_to_receipt_ms={f"p{p}": float(np.percentile(ages, p))
                for p in (50,95,99)} if ages else {},
            payload_bytes_received=sum(int(e["bytes"]) for e in values))
    return dict(condition=result["condition"], transport=transport,
        webrtc_decoder=result.get("webrtc_decoder", "aiortc") if transport == "webrtc" else None,
        payload_measurement=("RTP payload after receiver jitter/repair; excludes transport headers"
            if transport == "webrtc" and result.get("webrtc_decoder") == "gstreamer"
            else "Received RTP payload; excludes transport headers" if transport == "webrtc"
            else "Received CompressedImage data; excludes ROS/Zenoh/transport headers"),
        camera_count=result["cameras"], duration_s=result["duration_s"],
        cameras=cameras, sensors=sensors, inbound_stats=result["inbound_stats"],
        payload_integrity=payload_integrity(events, received),
        clock_method="Recent minimum RTT NTP-style request/reply; one-way bounds assume no clock step",
        notes=["Three feeds are phase-offset replays of one real recording.",
               "Decode rate is not Rerun display presentation FPS or Quest FPS.",
               "Companion depth/cloud use explicitly selected 2Hz/0.2Hz load, retaining all valid pixels.",
               "Loop boundaries repeat a 20.4-second recording; trials are not independent scene recordings."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--source-events", required=True, nargs="+", type=Path)
    args = parser.parse_args()
    result = analyze(args.run, args.source_events)
    (args.run / "analysis.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
