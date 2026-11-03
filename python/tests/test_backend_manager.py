from horus.utils.backend_manager import BackendManager
import sys
import shlex
import subprocess
import socket
import time

import pytest


def test_backend_running_rejects_unidentified_port_owner(monkeypatch):
    manager = BackendManager("ros2")

    monkeypatch.setattr(manager, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(manager, "_list_listening_pids", lambda _port: {1234})
    monkeypatch.setattr(manager, "_read_process_cmdline", lambda _pid: "python -m http.server 8080")

    assert manager._is_backend_running() is False


def test_backend_running_accepts_horus_port_owner(monkeypatch):
    manager = BackendManager("ros2")

    monkeypatch.setattr(manager, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(manager, "_list_listening_pids", lambda _port: {1234})
    monkeypatch.setattr(
        manager,
        "_read_process_cmdline",
        lambda _pid: "ros2 launch horus_backend horus_complete_backend.launch.py",
    )

    assert manager._is_backend_running() is True


def test_backend_running_accepts_sdk_owned_process(monkeypatch):
    manager = BackendManager("ros2")

    class Process:
        pid = 4321

        def poll(self):
            return None

    manager.backend_process = Process()
    monkeypatch.setattr(manager, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(manager, "_list_listening_pids", lambda _port: {4322})
    monkeypatch.setattr("horus.utils.backend_manager.os.getpgid", lambda _pid: 4321)

    assert manager._is_backend_running() is True


def test_owned_backend_does_not_claim_unrelated_existing_listener(monkeypatch):
    manager = BackendManager("ros2")

    class Process:
        pid = 4321

        def poll(self):
            return None

    manager.backend_process = Process()
    monkeypatch.setattr(manager, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(manager, "_list_listening_pids", lambda _port: {9999})
    monkeypatch.setattr("horus.utils.backend_manager.os.getpgid", lambda _pid: 9999)
    assert not manager._is_backend_running()


def test_exited_owned_backend_does_not_claim_external_horus_listener(monkeypatch):
    manager = BackendManager("ros2")

    class Process:
        def poll(self):
            return 2

    manager.backend_process = Process()
    monkeypatch.setattr(manager, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(manager, "_port_owner_looks_like_horus", lambda _port, _config: True)
    assert not manager._is_backend_running()


def test_backend_output_is_drained_and_bounded():
    manager = BackendManager("ros2")
    manager.backend_configs["ros2"]["launch_command"] = (
        f'{shlex.quote(sys.executable)} -c "import sys; sys.stdout.buffer.write(b\'x\' * 2000000); print(\'finished\')"'
    )
    manager._start_backend_process()
    process = manager.backend_process
    try:
        assert process.wait(timeout=5) == 0
        manager._output_reader.join(timeout=2)
        assert not manager._output_reader.is_alive()
        assert len(manager._output_tail) <= 128
        assert manager._backend_output_tail().endswith("finished")
    finally:
        manager.stop_backend()


def test_backend_start_failure_stops_spinner_and_owned_process(monkeypatch):
    from horus.utils import spinner

    events = []

    class Spinner:
        def __init__(self, _text):
            pass

        def start(self):
            events.append("start")

        def stop(self):
            events.append("stop")

    manager = BackendManager("ros2")
    monkeypatch.setattr(spinner, "Spinner", Spinner)
    monkeypatch.setattr(manager, "_is_backend_running", lambda: False)
    monkeypatch.setattr(manager, "_start_backend_process", lambda: None)
    monkeypatch.setattr(manager, "stop_backend", lambda: events.append("cleanup"))
    monkeypatch.setattr(manager, "_backend_output_tail", lambda: "launch error")

    def fail():
        raise RuntimeError("not ready")

    monkeypatch.setattr(manager, "_wait_for_backend_ready_silent", fail)
    with pytest.raises(RuntimeError, match="launch error"):
        manager.launch_backend()
    assert events == ["start", "cleanup", "stop"]


def test_backend_force_stop_reaps_process(monkeypatch):
    manager = BackendManager("ros2")
    waits = []
    signals = []

    class Process:
        pid = 4321

        def wait(self, timeout):
            waits.append(timeout)
            if len(waits) == 1:
                raise subprocess.TimeoutExpired("backend", timeout)
            return -9

    manager.backend_process = Process()
    monkeypatch.setattr("horus.utils.backend_manager.os.killpg", lambda pid, sig: signals.append((pid, sig)))
    manager.stop_backend()
    assert waits == [8, 3]
    assert signals == [(4321, 15), (4321, 9)]
    assert manager.backend_process is None


def test_backend_early_exit_does_not_wait_for_startup_timeout(monkeypatch):
    manager = BackendManager("ros2")

    class Process:
        returncode = 2

        def poll(self):
            return self.returncode

    manager.backend_process = Process()
    monkeypatch.setattr(manager, "_is_backend_running", lambda: False)
    with pytest.raises(RuntimeError, match="exited with code 2"):
        manager._wait_for_backend_ready_silent()


def test_backend_rejects_unknown_type():
    with pytest.raises(ValueError, match="Unsupported backend type"):
        BackendManager("unknown")


def test_live_owned_listener_and_short_startup_log_are_detected():
    manager = BackendManager("ros2")
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    manager.backend_configs["ros2"]["tcp_port"] = port
    manager.backend_configs["ros2"]["launch_command"] = (
        f'{shlex.quote(sys.executable)} -u -c "import socket,time; s=socket.socket(); '
        f's.bind((\'127.0.0.1\',{port})); s.listen(); print(\'listener ready\',flush=True); time.sleep(30)"'
    )
    manager._start_backend_process()
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if manager._is_backend_running() and "listener ready" in manager._backend_output_tail():
                break
            time.sleep(0.02)
        assert manager._is_backend_running()
        assert "listener ready" in manager._backend_output_tail()
    finally:
        manager.stop_backend()
