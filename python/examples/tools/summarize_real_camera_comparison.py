"""Summarize serial real-data transport trials without hiding failed trials."""

import argparse
import json
from pathlib import Path

import numpy as np


def resources(path, start, end):
    rows = [json.loads(line) for line in path.open()] if path.is_file() else []
    rows = [r for r in rows if start <= r["wall_ns"] <= end]
    cpu = [sum(p.get("cpu_percent_one_core") or 0 for p in r["processes"].values()) for r in rows]
    rss = [sum(p["rss_bytes"] for p in r["processes"].values())/1024**2 for r in rows]
    gpu = [max(float(g.split(",")[0]) for g in r["gpu_percent_memory_percent_mib"])
           for r in rows if r["gpu_percent_memory_percent_mib"]]
    net = None
    if len(rows) > 1:
        first, last = rows[0], rows[-1]
        a, b = first["network_host_totals"].get("tailscale0"), last["network_host_totals"].get("tailscale0")
        if a and b:
            seconds = (last["wall_ns"]-first["wall_ns"])/1e9
            net = {key.replace("_bytes", "_mbps"): (b[key]-a[key])*8/1e6/seconds for key in a}
    return dict(samples=len(rows), cpu_percent_one_core_median=float(np.median(cpu)) if cpu else None,
        cpu_percent_one_core_p95=float(np.percentile(cpu,95)) if cpu else None,
        sum_process_rss_peak_mib=max(rss) if rss else None,
        whole_gpu_utilization_median=float(np.median(gpu)) if gpu else None, host_tailscale=net)


def summarize(root):
    runs = []
    quality_path = root / "quality.json"
    quality = json.loads(quality_path.read_text()) if quality_path.exists() else dict(samples=[], unmatched_samples=[])
    for directory in sorted(root.iterdir()):
        if not directory.is_dir() or not directory.name.startswith(("webrtc-", "ros-")):
            continue
        if not (directory / "analysis.json").exists():
            runs.append(dict(run=directory.name, status="failed or not analyzed",
                failure=json.loads((directory / "failure.json").read_text()) if (directory / "failure.json").exists() else None))
            continue
        analysis = json.loads((directory / "analysis.json").read_text())
        result = json.loads((directory / "result.json").read_text())
        start, end = result["measurement_start_wall_ns"], result["measurement_end_wall_ns"]
        offset = result["clock"][1] if result.get("clock") else 0
        qs = [r["psnr_rgb_db"] for r in quality["samples"] if r["run"] == directory.name and r["psnr_rgb_db"] is not None]
        runs.append(dict(run=directory.name, status="collected" if result.get("completed", True) else "interrupted", **analysis,
            local_resources=resources(root / "local-resources.jsonl", start, end),
            remote_resources=resources(root / "remote-resources.jsonl", start+offset, end+offset),
            decoded_quality=dict(matched_samples=len(qs), psnr_rgb_db_mean=float(np.mean(qs)) if qs else None),
            decoder_warnings=result.get("viewer_logged", {})))
    return dict(runs=runs, unmatched_quality_samples=quality["unmatched_samples"],
        caveats=["Diagnostic repetitions of one real recorded sequence; not a powered statistical study.",
                 "One-way times are frame-stamp-to-PC-decode estimates with clock uncertainty, not Quest/glass-to-glass latency.",
                 "JPEG75 and adaptive H.264 are not quality/bitrate matched; consult PSNR and actual payload rates.",
                 "RTP payload is not wire bitrate; the native observer taps after jitter/repair and has no RTCP loss estimate.",
                 "CPU totals include matched benchmark processes; sum RSS counts shared pages more than once.",
                 "GPU and Tailscale counters are whole-host measurements, not exclusively camera traffic."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, type=Path)
    args = parser.parse_args()
    result = summarize(args.root)
    (args.root / "comparison.json").write_text(json.dumps(result, indent=2)+"\n")
    lines = ["# Real Camera Transport Comparison", "", "| Trial | Camera decoded FPS | Payload Mbit/s | Video p95 age ms | Longest video silence ms | TF Hz | TF p95 age ms | Longest TF silence ms |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for run in result["runs"]:
        if run["status"] != "collected":
            lines.append(f"| {run['run']} | Failed/not analyzed | | | | | | |")
            continue
        cameras = list(run["cameras"].values())
        fps = ", ".join(f"{c['decoded']['window_fps']:.2f}" for c in cameras)
        bitrate = sum(c["received_payload_kbps"] for c in cameras)/1000
        ages = [c["replay_publish_to_decode_ms"]["p95"] for c in cameras if "p95" in c["replay_publish_to_decode_ms"]]
        p95 = f"{max(ages):.0f}" if len(ages) == len(cameras) and ages else "N/A"
        gap = max((c["decoded"]["max_silence_ms"] for c in cameras), default=0)
        tf = run["sensors"]["tf"]
        tf_age = tf['replay_stamp_to_receipt_ms'].get('p95')
        tf_age = f"{tf_age:.0f}" if tf_age is not None else "N/A"
        lines.append(f"| {run['run']} | {fps} | {bitrate:.2f} | {p95} | {gap:.0f} | {tf['received']['window_fps']:.2f} | {tf_age} | {tf['received']['max_silence_ms']:.0f} |")
    lines += ["", "## Quality And Resources", "",
        "| Trial | Decoder | Matched quality samples | Mean RGB PSNR dB | Local CPU % | Source CPU % | Clock uncertainty ms |",
        "|---|---|---:|---:|---:|---:|---:|"]
    def number(value):
        return f"{value:.1f}" if value is not None else "N/A"
    for run in result["runs"]:
        if run["status"] != "collected":
            continue
        quality = run["decoded_quality"]
        uncertainty = [c["max_clock_uncertainty_ms"] for c in run["cameras"].values()
                       if c["max_clock_uncertainty_ms"] is not None]
        lines.append(f"| {run['run']} | {run.get('webrtc_decoder') or 'JPEG/OpenCV'} | "
            f"{quality['matched_samples']} | {number(quality['psnr_rgb_db_mean'])} | "
            f"{number(run['local_resources']['cpu_percent_one_core_median'])} | "
            f"{number(run['remote_resources']['cpu_percent_one_core_median'])} | "
            f"{number(max(uncertainty) if uncertainty else None)} |")
    lines += ["", "CPU is the sum of matched processes, as percent of one core. See JSON for RSS/GPU/host-network traces.",
        "", *result["caveats"], ""]
    (args.root / "COMPARISON.md").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__ == "__main__":
    main()
