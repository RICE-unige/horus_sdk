"""Run the real registration loops against deterministic callback schedules."""

import json
from types import SimpleNamespace

import pytest

from horus.bridge import robot_registry
from horus.utils import cli, topic_monitor, topic_status


class Dashboard:
    instances = []

    def __init__(self, *args, **kwargs):
        self.values = {}
        self.history = []
        self.__class__.instances.append(self)

    def __getattr__(self, name):
        if name.startswith("update_"):
            return lambda value: self.values.update({name[7:]: value})
        raise AttributeError(name)

    def tick(self, **kwargs):
        self.history.append(dict(self.values))

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.tick()


@pytest.fixture
def runtime(monkeypatch):
    from horus.description import robot_description_resolver
    monkeypatch.setattr(robot_description_resolver, "RobotDescriptionResolver", lambda: None)
    monkeypatch.setattr(robot_registry, "ROS2_AVAILABLE", False)
    monkeypatch.setattr(robot_registry, "String", lambda: SimpleNamespace(data=""), raising=False)
    monkeypatch.setattr(cli, "ConnectionDashboard", Dashboard)
    Dashboard.instances = []
    board = topic_status.TopicStatusBoard()
    monkeypatch.setattr(topic_status, "get_topic_status_board", lambda: board)
    monitor = SimpleNamespace(watch_topics=lambda *args: None, start=lambda: None)
    monkeypatch.setattr(topic_monitor, "get_topic_monitor", lambda: monitor)
    client = robot_registry.RobotRegistryClient()
    client.ros_initialized = True
    client._ensure_bridge_running = lambda: True
    client._get_local_ip = lambda: "192.0.2.1"
    client._build_global_visualizations_payload = lambda *args: []
    client._build_robot_config_dict = lambda robot, *args, **kwargs: {"robot_name": robot.name}
    client._apply_registration_metadata = lambda *args, **kwargs: None
    client._get_dashboard_bridge_status = lambda connected: "Responding" if connected else "Not discovered"
    now, step, sent = [100.0], [0], []
    monkeypatch.setattr(robot_registry.time, "monotonic", lambda: now[0])
    client.publisher = SimpleNamespace(publish=lambda msg: sent.append((step[0], json.loads(msg.data))))

    def heartbeat(uptime=None):
        client._heartbeat_callback(SimpleNamespace(data=f"Heartbeat: {uptime if uptime is not None else now[0]} | IP: 192.0.2.2"))

    def ack(name="alpha", success=True, robot_id="R1", **extra):
        client._ack_callback(SimpleNamespace(data=json.dumps({
            "robot_name": name, "success": success, "robot_id": robot_id, **extra,
        })))

    def schedule(events, stop=12):
        def spin(**kwargs):
            step[0] += 1
            now[0] += 1.0
            if step[0] > stop:
                raise KeyboardInterrupt
            if step[0] in events:
                events[step[0]]()
        client._spin_once_registration = spin

    robot = SimpleNamespace(name="alpha")
    return SimpleNamespace(client=client, heartbeat=heartbeat, ack=ack, schedule=schedule,
                           robot=robot, board=board, sent=sent, step=step)


@pytest.mark.parametrize("fleet", [False, True])
def test_disconnect_and_reconnect_wait_for_new_ack(runtime, fleet):
    r = runtime
    r.schedule({1: r.heartbeat, 2: r.ack, 7: r.heartbeat, 8: r.ack}, stop=8)
    if fleet:
        r.client.register_robots([r.robot], [None], keep_alive=True)
    else:
        r.client.register_robot(r.robot, None, keep_alive=True)
    history = Dashboard.instances[-1].history
    assert history[1]["registration"].startswith("Registered")
    assert history[4]["app_link"].startswith("Disconnected")
    assert history[4]["registration"] == "Waiting for App"
    assert history[6]["app_link"] == "Connected"
    assert not history[6]["registration"].startswith("Registered")
    assert history[7]["registration"].startswith("Registered")
    assert not r.board.is_silent


@pytest.mark.parametrize("fleet", [False, True])
def test_restarting_without_disconnect_invalidates_old_ack(runtime, fleet):
    r = runtime
    r.schedule({1: lambda: r.heartbeat(100), 2: r.ack, 3: lambda: r.heartbeat(1)}, stop=3)
    if fleet:
        r.client.register_robots([r.robot], [None])
    else:
        r.client.register_robot(r.robot, None, keep_alive=True)
    history = Dashboard.instances[-1].history
    assert history[1]["registration"].startswith("Registered")
    assert not history[2]["registration"].startswith("Registered")


@pytest.mark.parametrize("fleet", [False, True])
def test_queued_ack_is_not_final_success(runtime, fleet):
    r = runtime
    r.schedule({1: r.heartbeat, 2: lambda: r.ack(robot_id="Queued (Waiting for Workspace)"), 3: r.heartbeat}, stop=3)
    result = (r.client.register_robots([r.robot], [None]) if fleet else
              r.client.register_robot(r.robot, None, keep_alive=True))
    assert result[0] is False
    history = Dashboard.instances[-1].history
    assert history[1]["registration"].startswith("Queued")
    assert history[2]["registration"].startswith("Queued")


def test_fleet_progress_is_per_robot_and_ignores_other_session_ack(runtime):
    r = runtime
    r.schedule({1: r.heartbeat, 2: r.ack, 3: lambda: r.ack("unrelated"),
                4: lambda: (r.heartbeat(), r.ack("beta"))}, stop=4)
    ok, _ = r.client.register_robots([r.robot, SimpleNamespace(name="beta")], [None, None], keep_alive=False)
    assert ok
    history = Dashboard.instances[-1].history
    assert history[1]["registration"] == "Registering (1/2 acknowledged)"
    assert history[2]["registration"] == "Registering (1/2 acknowledged)"
    assert history[-1]["registration"] == "Registered (2/2)"


def test_fleet_failed_ack_is_not_overwritten_next_iteration(runtime):
    r = runtime
    r.schedule({1: r.heartbeat, 2: lambda: r.ack(success=False, error="Binding rejected"), 3: r.heartbeat}, stop=3)
    r.client.register_robots([r.robot], [None])
    history = Dashboard.instances[-1].history
    assert all(h["registration"].startswith("Failed") for h in history[1:])
    assert "Binding rejected" in history[-1]["status"]


@pytest.mark.parametrize("fleet", [False, True])
def test_waiting_for_app_does_not_spend_registration_budget(runtime, fleet):
    r = runtime
    r.schedule({6: r.heartbeat, 7: r.ack}, stop=7)
    if fleet:
        ok, _ = r.client.register_robots([r.robot], [None], timeout_sec=2, keep_alive=False)
    else:
        ok, _ = r.client.register_robot(r.robot, None, timeout_sec=2, keep_alive=False)
    assert ok


def test_single_quiet_retry_is_rate_limited(runtime):
    r = runtime
    r.schedule({}, stop=20)
    ok, _ = r.client.register_robot(r.robot, None, show_dashboard=False, timeout_sec=5)
    assert not ok
    assert len(r.sent) == 3


def test_empty_or_duplicate_fleet_is_rejected(runtime):
    assert not runtime.client.register_robots([], [])[0]
    assert not runtime.client.register_robots([runtime.robot, runtime.robot], [None, None])[0]
    assert not runtime.board.is_silent


def test_large_fleet_retries_cannot_starve_later_robots(runtime):
    r = runtime
    robots = [SimpleNamespace(name=f"robot_{i}") for i in range(20)]
    r.schedule({i: r.heartbeat for i in range(1, 21)}, stop=20)
    r.client.register_robots(robots, [None] * len(robots))
    assert [payload["robot_name"] for _, payload in r.sent] == [robot.name for robot in robots]


@pytest.mark.parametrize("fleet", [False, True])
def test_explicit_timeout_is_reported_and_board_mute_is_restored(runtime, fleet):
    r = runtime
    r.schedule({i: r.heartbeat for i in range(1, 5)}, stop=4)
    r.board.set_silent(True)
    if fleet:
        ok, result = r.client.register_robots([r.robot], [None], timeout_sec=2, keep_alive=False)
    else:
        ok, result = r.client.register_robot(r.robot, None, timeout_sec=2, keep_alive=False)
    assert not ok and result["error"] == "Registration timeout"
    assert Dashboard.instances[-1].history[-1]["registration"] == "Failed"
    assert r.board.is_silent
