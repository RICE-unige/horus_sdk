from horus.bridge.robot_registry import RobotRegistryClient


def make_client():
    return RobotRegistryClient.__new__(RobotRegistryClient)


def test_bridge_listener_rejects_unrelated_ros2_process(monkeypatch):
    client = make_client()

    monkeypatch.setattr(client, "_listener_pids_for_port", lambda _port: [1234])
    monkeypatch.setattr(
        client,
        "_read_process_cmdline",
        lambda _pid: "ros2 run unrelated_package unrelated_server --port 10000",
    )

    assert client._is_likely_horus_bridge_listener(10000) is False


def test_bridge_listener_accepts_horus_unity_bridge_process(monkeypatch):
    client = make_client()

    monkeypatch.setattr(client, "_listener_pids_for_port", lambda _port: [1234])
    monkeypatch.setattr(
        client,
        "_read_process_cmdline",
        lambda _pid: "ros2 run horus_unity_bridge horus_unity_bridge_node --ros-args",
    )

    assert client._is_likely_horus_bridge_listener(10000) is True


def test_bridge_port_open_fails_for_identified_non_horus_listener(monkeypatch):
    client = make_client()

    monkeypatch.setattr(client, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(client, "_is_likely_horus_bridge_listener", lambda _port: False)

    assert client._is_horus_bridge_port_open(10000) is False


def test_bridge_port_open_allows_unknown_listener_identity(monkeypatch):
    client = make_client()

    monkeypatch.setattr(client, "_is_port_open", lambda _port: True)
    monkeypatch.setattr(client, "_is_likely_horus_bridge_listener", lambda _port: None)

    assert client._is_horus_bridge_port_open(10000) is True
