from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest


def _load_launcher():
    path = (
        Path(__file__).resolve().parents[1]
        / "examples"
        / "remote_map_portal_registration.py"
    )
    spec = importlib.util.spec_from_file_location("remote_map_v3_launcher", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_v3_launcher_builds_bounded_default_route_fastdds_profile(
    monkeypatch,
    tmp_path,
):
    launcher = _load_launcher()
    monkeypatch.delenv("FASTRTPS_DEFAULT_PROFILES_FILE", raising=False)
    monkeypatch.delenv("FASTDDS_DEFAULT_PROFILES_FILE", raising=False)
    monkeypatch.delenv("RMW_IMPLEMENTATION", raising=False)
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    monkeypatch.delenv("ROS_AUTOMATIC_DISCOVERY_RANGE", raising=False)
    monkeypatch.setattr(launcher, "_default_route_interface", lambda: "eth1")

    path = launcher._configure_v3_fastdds_default_route(tmp_path)

    assert path == tmp_path / "fastdds_default_route_eth1.xml"
    content = path.read_text(encoding="utf-8")
    assert "<interface>eth1</interface>" in content
    assert "<type>SHM</type>" in content
    assert "<useBuiltinTransports>false</useBuiltinTransports>" in content
    assert launcher.os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] == str(path)
    assert launcher.os.environ["FASTDDS_DEFAULT_PROFILES_FILE"] == str(path)


def test_v3_launcher_preserves_explicit_middleware_profile(monkeypatch, tmp_path):
    launcher = _load_launcher()
    monkeypatch.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", "/custom/profile.xml")
    monkeypatch.delenv("FASTDDS_DEFAULT_PROFILES_FILE", raising=False)
    monkeypatch.setattr(
        launcher,
        "_default_route_interface",
        lambda: pytest.fail("explicit profile must bypass route detection"),
    )

    assert launcher._configure_v3_fastdds_default_route(tmp_path) is None


@pytest.mark.parametrize("variable,value", [
    ("ROS_LOCALHOST_ONLY", "1"), ("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")])
def test_local_remote_renderer_keeps_loopback_discovery(monkeypatch, tmp_path, variable, value):
    launcher = _load_launcher()
    for name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
                 "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE"):
        monkeypatch.delenv(name, raising=False)
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setenv(variable, value)
    monkeypatch.setattr(launcher, "_default_route_interface", lambda: pytest.fail("local DDS must not use LAN"))
    path = launcher._configure_v3_fastdds_default_route(tmp_path)
    assert "<interface>lo</interface>" in path.read_text()


@pytest.mark.parametrize("interface", ("../eth1", "eth1<bad", "eth1 bad"))
def test_v3_launcher_rejects_unsafe_interface_names(interface):
    launcher = _load_launcher()
    with pytest.raises(ValueError, match="interface"):
        launcher._fastdds_default_route_profile(interface)


def test_launcher_defaults_to_full_source_v3_webrtc(monkeypatch):
    launcher = _load_launcher()
    monkeypatch.setattr(sys, "argv", ["remote_map_portal_registration.py"])

    args = launcher.parse_args()

    assert args.protocol == "v3"
    assert args.transport == "webrtc"
    assert args.encoder == "nvenc"
    assert args.source_voxel_size == 0.0
    assert args.maximum_frame_age_ms == 250


@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
def test_renderer_is_reaped_when_startup_is_interrupted(monkeypatch, tmp_path, failure):
    launcher = _load_launcher()
    monkeypatch.setattr(sys, "argv", ["remote_map_portal_registration.py"])
    args = launcher.parse_args()
    monkeypatch.setattr(launcher, "CACHE_ROOT", tmp_path)
    process = Mock(pid=1234)
    process.poll.return_value = None
    monkeypatch.setattr(launcher.subprocess, "Popen", lambda *a, **kw: process)
    monkeypatch.setattr(launcher.time, "sleep", Mock(side_effect=failure("interrupted")))
    with pytest.raises(failure):
        launcher.start_render_agent(args)
    process.terminate.assert_called_once()
    process.wait.assert_called_once_with(timeout=5.0)
    assert len(list(tmp_path.glob("*/agent.log"))) == 1
    assert not list(tmp_path.glob("*/ready.json"))
