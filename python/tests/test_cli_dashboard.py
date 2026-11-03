"""Dashboard rendering and status evidence, without ROS or a running bridge."""

import io
import json
import shutil
import socket
from types import SimpleNamespace

import pytest
from rich.console import Console

from horus.bridge import robot_registry
from horus.utils import branding, cli, topic_monitor, topic_status, unity_monitor
from horus.utils.dashboard_status import topic_graph


class Graph:
    def __init__(self, pubs=(), subs=()):
        self.pubs = [SimpleNamespace(node_name=n) for n in pubs]
        self.subs = [SimpleNamespace(node_name=n) for n in subs]
        self.calls = 0

    def get_name(self):
        return "registry"

    def get_publishers_info_by_topic(self, topic):
        self.calls += 1
        return self.pubs

    def get_subscriptions_info_by_topic(self, topic):
        self.calls += 1
        return self.subs

    def get_node_names_and_namespaces(self):
        return [(info.node_name, "/") for info in self.pubs + self.subs]


@pytest.fixture
def client(monkeypatch):
    from horus.description import robot_description_resolver
    monkeypatch.setattr(robot_registry, "ROS2_AVAILABLE", False)
    monkeypatch.setattr(robot_description_resolver, "RobotDescriptionResolver", lambda: None)
    obj = robot_registry.RobotRegistryClient()
    obj.node = Graph()
    return obj


def message(data):
    return SimpleNamespace(data=json.dumps(data))


@pytest.mark.parametrize("pubs,subs,role,expected", [
    (["sensor"], ["rviz"], "backend_sub", "NO_BRIDGE"),
    (["sensor"], ["horus_unity_bridge"], "backend_sub", "OK"),
    ([], ["horus_unity_bridge"], "backend_sub", "NO_SOURCE"),
    (["joystick"], ["robot"], "backend_pub", "NO_BRIDGE"),
    (["horus_unity_bridge"], [], "backend_pub", "NO_CONSUMER"),
    (["horus_unity_bridge"], ["robot"], "backend_pub", "OK"),
    (["horus_unity_bridge"], ["registry"], "sdk_sub", "OK"),
    (["registry"], ["horus_unity_bridge"], "sdk_pub", "OK"),
    (["registry"], ["rviz"], "sdk_pub", "NO_BRIDGE"),
    (["other_sdk"], ["horus_unity_bridge"], "sdk_pub", "NO_SDK"),
])
def test_graph_evidence(pubs, subs, role, expected):
    node = Graph(pubs, subs)
    result = topic_graph(node, "/scan", role)
    assert result == {"pubs": len(pubs), "subs": len(subs), "link": expected}
    assert node.calls == 2


def test_unknown_graph_is_not_zero():
    assert topic_graph(None, "/scan") == {"pubs": None, "subs": None, "link": "UNKNOWN"}


def test_namespace_collision_does_not_match_sdk():
    node = Graph(["registry"], ["horus_unity_bridge"])
    node.get_namespace = lambda: "/local"
    node.pubs[0].node_namespace = "/another"
    assert topic_graph(node, "/horus/registration", "sdk_pub")["link"] == "NO_SDK"


def test_publishers_do_not_prove_streaming_or_webrtc(client):
    client.node = Graph(["camera"], ["horus_unity_bridge"])
    rows = client._build_dashboard_topic_rows(["/camera"], {}, [], True, True)
    assert rows[0]["link"] == "OK"
    assert rows[0]["data"] == "Not measured"
    profiles = {"/camera": {"minimap": "webrtc", "teleop": "webrtc", "startup": "minimap"}}
    rows = client._build_dashboard_topic_rows(["/camera"], {}, [], True, True, profiles)
    assert rows[0]["link"] == "MEDIA"
    assert rows[0]["data"] == "Not measured"
    assert cli._render_camera_profile(rows[0]["camera_profile"]) == "WEBRTC configured"
    assert "active_transport" not in profiles["/camera"]


def test_direct_webrtc_does_not_require_a_ros_camera_publisher(client):
    client.node = Graph()
    profiles = {"/camera": {"minimap": "webrtc", "startup": "minimap"}}
    rows = client._build_dashboard_topic_rows(["/camera"], {}, [], True, True, profiles)
    assert rows[0]["link"] == "MEDIA"
    assert rows[0]["data"] == "Not measured"
    assert "configured" in cli._render_transport_badge(rows[0]["camera_profile"]).plain


@pytest.mark.parametrize("profile,label,colour,qualifier", [
    (None, "ROS", "dark_green", ""),
    ({"minimap": "ros"}, "ROS", "dark_green", "configured"),
    ({"active_transport": "webrtc"}, "WebRTC", "blue", ""),
    ({"active_transport": "ros"}, "ROS", "dark_green", ""),
    ({"teleop": "webrtc", "startup": "teleop"}, "WebRTC", "blue", "configured"),
])
def test_transport_badges_preserve_evidence(profile, label, colour, qualifier):
    badge = cli._render_transport_badge(profile)
    assert badge.plain == f" {label} " + (f" {qualifier}" if qualifier else "")
    assert badge.spans[0].style == f"bold white on {colour}"
    assert badge.spans[0].end == len(label) + 2
    assert "Connected" not in badge.plain


@pytest.mark.parametrize("width", [60, 80, 120, 160])
def test_dashboard_has_badges_but_no_pub_sub_clutter(width):
    output = Console(file=io.StringIO(), width=width, height=30, color_system=None)
    dashboard = cli.ConnectionDashboard("localhost", 10000, show_counts=True, output=output)
    dashboard.update_topics([
        {"topic": "/camera", "link": "MEDIA", "pubs": 10, "subs": 20,
         "camera_profile": {"teleop": "webrtc", "startup": "teleop"}},
        {"topic": "/tf", "link": "OK", "pubs": 10, "subs": 20},
    ])
    with output.capture() as capture:
        output.print(dashboard.render())
    rendered = capture.get()
    assert "WebRTC" in rendered and "configured" in rendered and "ROS" in rendered
    assert "Pub" not in rendered and "Sub" not in rendered
    assert "SDK receipt" not in rendered and "Not measured" not in rendered
    assert all(len(line) <= width for line in rendered.splitlines())


def test_full_wordmark_and_redirected_output():
    output = Console(file=io.StringIO(), width=60, color_system=None)
    with output.capture() as capture:
        branding.show_ascii_art(output=output)
    rendered = capture.get()
    assert all(row in rendered for row in branding._WORDMARK)
    assert "HORUS SDK v" in rendered and "Holistic Operational Reality" in rendered
    assert "\x1b" not in rendered


def test_wordmark_on_ascii_only_stream():
    raw = io.BytesIO()
    stream = io.TextIOWrapper(raw, encoding="ascii")
    branding.show_ascii_art(output=Console(file=stream, width=60, color_system=None))
    stream.flush()
    rendered = raw.getvalue().decode("ascii")
    assert all(row.rstrip() in rendered for row in branding._ASCII_WORDMARK)
    assert "HORUS SDK v" in rendered


def test_transport_report_expires_and_false_string_is_not_active(client, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(robot_registry.time, "monotonic", lambda: now[0])
    client._teleop_runtime_state_callback(message({"camera_topic": "/cam", "active": True, "transport": "webrtc"}))
    assert client._get_runtime_transport_override("/cam") == "webrtc"
    now[0] += 4
    assert client._get_runtime_transport_override("/cam") is None
    client._teleop_runtime_state_callback(message({"camera_topic": "/cam", "active": "false", "transport": "webrtc"}))
    assert client._get_runtime_transport_override("/cam") is None
    client._teleop_runtime_state_callback(message([]))


def test_receipts_only_come_from_callbacks_and_use_monotonic_time(client, monkeypatch):
    now = [100.0]
    monkeypatch.setattr(robot_registry.time, "monotonic", lambda: now[0])
    monkeypatch.setattr(robot_registry.time, "time", lambda: 5000)
    client._heartbeat_callback(SimpleNamespace(data="Heartbeat: 42 | IP: 192.0.2.1"))
    assert client._get_active_app_count(3.0) == 1
    assert client._get_dashboard_bridge_status(True).startswith("Responding")
    monkeypatch.setattr(robot_registry.time, "time", lambda: 1)
    assert client._get_active_app_count(3.0) == 1
    now[0] += 4
    assert client._get_active_app_count(3.0) == 0
    assert client._format_app_link_status(False, True).startswith("Disconnected")
    rows = client._build_dashboard_topic_rows(["/horus/heartbeat"], {"/horus/heartbeat": "sdk_sub"}, [], False, False)
    assert rows[0]["data"] == "Stale 4.0s ago"
    assert not client._get_dashboard_bridge_status(False).startswith("Responding")


@pytest.mark.parametrize("payload", ["", "not a heartbeat", "Heartbeat: nan", "Heartbeat: inf", "Heartbeat: -1"])
def test_malformed_heartbeat_does_not_connect(client, payload):
    client._heartbeat_callback(SimpleNamespace(data=payload))
    assert client._get_active_app_count(3.0) == 0
    assert client._last_heartbeat_time == 0


def test_acks_are_validated_and_consumed_once(client):
    for payload in ([], None, {"success": "false", "robot_name": "r"}, {"success": True}):
        client._ack_callback(message(payload))
    assert not client._ack_received.is_set()
    ack = {"robot_name": "r", "success": True, "robot_id": "r-1"}
    client._ack_callback(message(ack))
    assert client._take_registration_ack("r") == ack
    assert client._take_registration_ack("r") is None
    assert not client._ack_received.is_set()


def test_private_selection_is_not_workspace_ready(client):
    client._multi_operator_presence_callback(message({"app_id": "a", "role": "private", "state": "private_selected"}))
    summary = client._get_app_presence_summary()
    assert summary["private_ready_count"] == summary["workspace_ready_count"] == 0
    client._multi_operator_presence_callback(message({"app_id": "a", "role": "private", "state": "private_workspace_ready", "workspace_active": True}))
    summary = client._get_app_presence_summary()
    assert summary["private_ready_count"] == summary["workspace_ready_count"] == 1
    assert client._resolve_dashboard_queued_status({"r": "Waiting for Workspace"}) == "Waiting for Workspace"


@pytest.mark.parametrize("width", [60, 80, 120, 160])
def test_render_is_bounded_literal_and_readable(width):
    buf = io.StringIO()
    output = Console(file=buf, width=width, height=30, color_system=None)
    dashboard = cli.ConnectionDashboard("192.0.2.1", 10000, output=output, clock=lambda: 100)
    dashboard.update_app_link("Disconnected (heartbeat expired)")
    dashboard.update_registration("Queued")
    dashboard.update_status("[link=https://example.com]literal[/link]")
    dashboard.update_topics([
        {"topic": f"/robot_{i}/camera/image_raw", "robot": f"robot_{i}", "link": "OK", "data": "Not measured"}
        for i in range(50)
    ])
    output.print(dashboard.render())
    rendered = buf.getvalue()
    assert "Disconnected" in rendered and "Queued" in rendered
    assert "literal" in rendered and "Page 1/" in rendered
    assert len(rendered.splitlines()) <= 30
    assert all(len(line) <= width for line in rendered.splitlines())
    assert "\x1b" not in rendered


def test_plain_output_deduplicates_and_flushes_final_state():
    buf = io.StringIO()
    output = Console(file=buf, width=160, color_system=None)
    now = [0.0]
    with cli.ConnectionDashboard("192.0.2.1", 10000, output=output, clock=lambda: now[0]) as dashboard:
        for _ in range(100):
            now[0] += 1
            dashboard.update_app_link("Connected")
            dashboard.update_registration("Registered")
            dashboard.tick()
        dashboard.update_registration("Failed")
        dashboard.update_status("Explicit rejection")
    text = buf.getvalue()
    assert text.count("Registration: Registered") == 1
    assert "Registration: Failed" in text and "Explicit rejection" in text
    assert "\x1b" not in text


def test_refresh_rate_is_bounded():
    now = [0.0]
    dashboard = cli.ConnectionDashboard("localhost", 10000, clock=lambda: now[0])
    dashboard._interactive = dashboard._started = True
    renders = []
    dashboard.live = SimpleNamespace(update=lambda *args, **kwargs: renders.append(now[0]))
    for i in range(1000):
        now[0] = i / 1000
        dashboard.update_status(str(i))
        dashboard.tick()
    assert len(renders) == 4


def test_pages_rotate_without_dropping_topics():
    now = [0.0]
    output = Console(file=io.StringIO(), width=80, height=24, color_system=None)
    dashboard = cli.ConnectionDashboard("localhost", 10000, output=output, clock=lambda: now[0])
    dashboard.update_topics([{"topic": f"/scan_{i}", "link": "OK"} for i in range(16)])
    pages = []
    for instant in (0, 8):
        now[0] = instant
        with output.capture() as capture:
            output.print(dashboard.render())
        pages.append(capture.get())
    assert "/scan_0" in pages[0] and "/scan_15" not in pages[0]
    assert "/scan_15" in pages[1] and "/scan_0" not in pages[1]


def test_spinner_redirected_output_does_not_animate(monkeypatch):
    from horus.utils.spinner import Spinner
    buf = io.StringIO()
    monkeypatch.setattr(cli, "console", Console(file=buf, color_system=None))
    spinner = Spinner("Checking listener")
    spinner.start()
    spinner.start()
    spinner.stop()
    assert buf.getvalue().count("Checking listener") == 1
    assert "\x1b" not in buf.getvalue() and "\r" not in buf.getvalue()


def test_false_environment_value_is_false(monkeypatch):
    monkeypatch.setenv("HORUS_DASHBOARD_SHOW_CORE", "0")
    assert not cli.env_enabled("HORUS_DASHBOARD_SHOW_CORE")


def test_topic_monitor_errors_and_control_direction(monkeypatch):
    board = topic_status.TopicStatusBoard()
    board.set_silent(True)
    monkeypatch.setattr(topic_monitor, "get_topic_status_board", lambda: board)
    monitor = topic_monitor.TopicSubscriptionMonitor()
    monitor.watch_topics(["/cmd_vel"], {"/cmd_vel": "backend_pub"})
    assert board.snapshot()["/cmd_vel"] == "UNKNOWN"
    monitor._node = Graph(["horus_unity_bridge"], ["robot"])
    monitor._check_topic("/cmd_vel", 1)
    assert board.snapshot()["/cmd_vel"] == "SUBSCRIBED"
    monitor._node = object()
    monitor._check_topic("/cmd_vel", 2)
    assert board.snapshot()["/cmd_vel"] == "UNKNOWN"


def test_peer_parser_ipv6_and_exact_local_port():
    data = "\n".join([
        "ESTAB 0 0 0.0.0.0:10000 192.0.2.1:51000",
        "ESTAB 0 0 [::]:10000 [2001:db8::1]:51000",
        "ESTAB 0 0 0.0.0.0:100001 192.0.2.2:51000",
        "ESTAB 0 0 192.0.2.3:51000 192.0.2.4:10000",
    ])
    assert unity_monitor._peers(data, 10000) == {"192.0.2.1", "2001:db8::1"}
    assert unity_monitor._peers("tcp 0 0 192.0.2.3:5555 192.0.2.4:10000 ESTABLISHED", 10000, netstat=True) == set()


def test_failed_probe_cannot_leave_connected_peers(monkeypatch):
    now = [10]
    monkeypatch.setattr(unity_monitor.time, "monotonic", lambda: now[0])
    monitor = unity_monitor.UnityConnectionMonitor()
    monitor._update_observation({"192.0.2.1"})
    assert monitor.is_unity_connected()
    monitor._update_observation(None)
    assert not monitor.observation_available and not monitor.is_unity_connected()
    monitor._update_observation({"192.0.2.1"})
    now[0] += 6
    assert not monitor.is_unity_connected()


@pytest.mark.skipif(shutil.which("ss") is None, reason="ss is not installed")
def test_real_loopback_tcp_connection_and_close():
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        port = listener.getsockname()[1]
        monitor = unity_monitor.UnityConnectionMonitor(port, disconnect_grace_s=0)
        with socket.create_connection(("127.0.0.1", port)) as peer:
            accepted, _ = listener.accept()
            with accepted:
                monitor._monitor_network_connections()
                assert monitor.get_connected_clients() == {"127.0.0.1"}
        monitor._monitor_network_connections()
        assert not monitor.is_unity_connected()
    monitor.stop_monitoring()
    monitor._update_observation({"192.0.2.1"})
    assert not monitor.is_unity_connected()
