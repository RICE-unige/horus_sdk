"""Sample the benchmark processes without changing host/network settings."""

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time


def processes(patterns):
    result = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            if "real_camera_resources.py" in command or not any(p in command for p in patterns):
                continue
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            result[entry.name] = dict(start=fields[19], cpu=(int(fields[11])+int(fields[12]))/os.sysconf("SC_CLK_TCK"),
                rss_bytes=int(fields[21])*os.sysconf("SC_PAGE_SIZE"),
                name=(entry / "comm").read_text().strip())
        except (OSError, ValueError, IndexError):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=600)
    parser.add_argument("--match", required=True, action="append")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("duration must be positive")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    previous, last = {}, time.monotonic()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    end = last+args.duration
    with args.output.open("x", buffering=1) as output:
        while time.monotonic() < end and not stop.is_set():
            now = time.monotonic()
            current = processes(args.match)
            for pid, value in current.items():
                prior = previous.get(pid)
                value["cpu_percent_one_core"] = ((value["cpu"]-prior["cpu"])/(now-last)*100
                    if prior and value["start"] == prior["start"] and now > last else None)
            net = {}
            for name in ("tailscale0",):
                try:
                    base = Path("/sys/class/net") / name / "statistics"
                    net[name] = {key: int((base/key).read_text()) for key in ("rx_bytes", "tx_bytes")}
                except (OSError, ValueError):
                    pass
            gpu = []
            if shutil.which("nvidia-smi"):
                try:
                    value = subprocess.run(["nvidia-smi", "--query-gpu=utilization.gpu,utilization.memory,memory.used",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=2)
                    if value.returncode == 0:
                        gpu = value.stdout.strip().splitlines()
                except subprocess.TimeoutExpired:
                    pass
            output.write(json.dumps(dict(wall_ns=time.time_ns(), monotonic_s=now,
                processes=current, network_host_totals=net, gpu_percent_memory_percent_mib=gpu))+"\n")
            previous, last = current, now
            stop.wait(max(0, 1-(time.monotonic()-now)))


if __name__ == "__main__":
    main()
