"""Tests for bridge auto-start strategy selection and diagnostics."""

from horus.bridge.robot_registry import RobotRegistryClient
from horus.utils import cli


def _build_client():
    client = RobotRegistryClient.__new__(RobotRegistryClient)
    client.ros_initialized = False
    client.node = None
    client.bridge_process = None
    client._bridge_log_file = None
    return client


def _capture_cli(monkeypatch):
    messages = []
    monkeypatch.setattr(cli, "print_info", lambda msg: messages.append(("info", str(msg))))
    monkeypatch.setattr(cli, "print_error", lambda msg: messages.append(("error", str(msg))))
    return messages


def test_ensure_bridge_running_prefers_ros2_launch_in_auto_mode(monkeypatch):
    client = _build_client()
    messages = _capture_cli(monkeypatch)
    calls = []

    monkeypatch.setenv("HORUS_SDK_BRIDGE_AUTOSTART_MODE", "auto")
    monkeypatch.setattr(client, "_is_port_open", lambda port: False)
    monkeypatch.setattr(client, "_get_ros2_pkg_prefix_current_shell", lambda pkg: "/home/omotoye/horus_ws/install/horus_unity_bridge")
    monkeypatch.setattr(client, "_get_first_helper_bridge_prefix", lambda: "/home/omotoye/horus/ros2/install/horus_unity_bridge")

    def ros2_start():
        calls.append("ros2")
        return True

    def helper_start():
        calls.append("helper")
        return True

    monkeypatch.setattr(client, "_auto_start_bridge_with_ros2_launch", ros2_start)
    monkeypatch.setattr(client, "_auto_start_bridge_with_horus_helper", helper_start)

    assert client._ensure_bridge_running() is True
    assert calls == ["ros2"]
    assert any("Bridge auto-start mode: auto" in msg for level, msg in messages if level == "info")


def test_ensure_bridge_running_falls_back_to_helper_when_ros2_launch_fails(monkeypatch):
    client = _build_client()
    _capture_cli(monkeypatch)
    calls = []

    monkeypatch.setenv("HORUS_SDK_BRIDGE_AUTOSTART_MODE", "auto")
    monkeypatch.setattr(client, "_is_port_open", lambda port: False)
    monkeypatch.setattr(client, "_get_ros2_pkg_prefix_current_shell", lambda pkg: None)
    monkeypatch.setattr(client, "_get_first_helper_bridge_prefix", lambda: None)

    monkeypatch.setattr(client, "_auto_start_bridge_with_ros2_launch", lambda: calls.append("ros2") or False)
    monkeypatch.setattr(client, "_auto_start_bridge_with_horus_helper", lambda: calls.append("helper") or True)

    assert client._ensure_bridge_running() is True
    assert calls == ["ros2", "helper"]


def test_ensure_bridge_running_helper_mode_skips_ros2_launch(monkeypatch):
    client = _build_client()
    _capture_cli(monkeypatch)
    calls = []

    monkeypatch.setenv("HORUS_SDK_BRIDGE_AUTOSTART_MODE", "helper")
    monkeypatch.setattr(client, "_is_port_open", lambda port: False)
    monkeypatch.setattr(client, "_auto_start_bridge_with_ros2_launch", lambda: calls.append("ros2") or True)
    monkeypatch.setattr(client, "_auto_start_bridge_with_horus_helper", lambda: calls.append("helper") or True)

    assert client._ensure_bridge_running() is True
    assert calls == ["helper"]


def test_ensure_bridge_running_no_autostart_when_mode_off(monkeypatch):
    client = _build_client()
    messages = _capture_cli(monkeypatch)
    calls = []

    monkeypatch.setenv("HORUS_SDK_BRIDGE_AUTOSTART_MODE", "off")
    monkeypatch.setattr(client, "_is_port_open", lambda port: False)
    monkeypatch.setattr(client, "_auto_start_bridge_with_ros2_launch", lambda: calls.append("ros2") or True)
    monkeypatch.setattr(client, "_auto_start_bridge_with_horus_helper", lambda: calls.append("helper") or True)

    assert client._ensure_bridge_running() is False
    assert calls == []
    assert any("auto-start disabled" in msg.lower() for level, msg in messages if level == "error")


def test_ensure_bridge_running_logs_prefix_mismatch_warning(monkeypatch):
    client = _build_client()
    messages = _capture_cli(monkeypatch)

    monkeypatch.setenv("HORUS_SDK_BRIDGE_AUTOSTART_MODE", "auto")
    monkeypatch.setattr(client, "_is_port_open", lambda port: False)
    monkeypatch.setattr(client, "_get_ros2_pkg_prefix_current_shell", lambda pkg: "/home/omotoye/horus_ws/install/horus_unity_bridge")
    monkeypatch.setattr(client, "_get_first_helper_bridge_prefix", lambda: "/home/omotoye/horus/ros2/install/horus_unity_bridge")
    monkeypatch.setattr(client, "_auto_start_bridge_with_ros2_launch", lambda: True)
    monkeypatch.setattr(client, "_auto_start_bridge_with_horus_helper", lambda: False)

    assert client._ensure_bridge_running() is True

    info_messages = [msg for level, msg in messages if level == "info"]
    assert any("differs from helper workspace" in msg for msg in info_messages)
    assert any("Current shell prefix:" in msg for msg in info_messages)
    assert any("Helper workspace prefix:" in msg for msg in info_messages)
