from horus.utils.backend_manager import BackendManager


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
        def poll(self):
            return None

    manager.backend_process = Process()
    monkeypatch.setattr(manager, "_is_port_open", lambda _port: True)

    assert manager._is_backend_running() is True
