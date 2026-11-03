"""Manage only the process group created by this installation's horus-start."""

import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from manage import atomic_write


def identity(pid):
    try:
        # comm may contain spaces/parentheses; the fields after its last ')' do not.
        fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
        if fields[0] == "Z":
            return None
        return [Path("/proc/sys/kernel/random/boot_id").read_text().strip(), fields[19]]
    except (OSError, IndexError):
        return None


def read_running(path):
    if not path.exists():
        return None
    try:
        state = json.loads(path.read_text())
        current = identity(int(state["pid"]))
        if current is not None and current == state["identity"]:
            return state
    except (ValueError, KeyError, TypeError, OSError):
        pass
    return None


def run(root, command):
    state_path = root / "state/launch.json"
    with (root / "state/runtime.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("HORUS is already running or being updated in this installation")
        if read_running(state_path):
            raise RuntimeError("Recorded HORUS launcher is already running")
        stop = False

        def request_stop(_signum, _frame):
            nonlocal stop
            stop = True

        previous = {sig: signal.signal(sig, request_stop)
                    for sig in (signal.SIGINT, signal.SIGTERM)}
        child = None
        try:
            child = subprocess.Popen(command, start_new_session=True)
            atomic_write(state_path, json.dumps({"pid": os.getpid(),
                         "identity": identity(os.getpid()), "child_pid": child.pid}))
            while child.poll() is None and not stop:
                time.sleep(0.1)
            result = child.poll()
            return 0 if stop else result
        finally:
            if child is not None:
                # Signal the complete owned group, including remaining child nodes.
                for sig, seconds in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
                    try:
                        os.killpg(child.pid, sig)
                    except ProcessLookupError:
                        break
                    deadline = time.monotonic() + seconds
                    while time.monotonic() < deadline:
                        child.poll()
                        try:
                            os.killpg(child.pid, 0)
                        except ProcessLookupError:
                            break
                        time.sleep(0.1)
                    else:
                        continue
                    break
                child.wait(timeout=5)
            if state_path.exists():
                state_path.unlink()
            for sig, handler in previous.items():
                signal.signal(sig, handler)


def stop(root):
    state = read_running(root / "state/launch.json")
    if not state:
        print("No running process owned by horus-start. SDK/external launches are not stopped.")
        return 0
    try:
        os.kill(state["pid"], signal.SIGTERM)
    except ProcessLookupError:
        return 0
    deadline = time.monotonic() + 20
    while identity(state["pid"]) == state["identity"] and time.monotonic() < deadline:
        time.sleep(0.1)
    if identity(state["pid"]) == state["identity"]:
        raise RuntimeError("Launcher did not exit within 20 seconds; no unrelated process was killed")
    print("HORUS launch group stopped.")
    return 0


def main():
    root = Path(sys.argv[1]).resolve()
    action = sys.argv[2]
    if action == "start":
        return run(root, sys.argv[3:])
    if action == "stop":
        return stop(root)
    raise ValueError("Unsupported launcher action")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as error:
        print(f"[horus] {error}", file=sys.stderr)
        sys.exit(1)
