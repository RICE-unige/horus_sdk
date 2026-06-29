import horus.utils.network as network


def test_resolve_advertise_ip_uses_explicit_override(monkeypatch):
    monkeypatch.setenv("HORUS_SDK_ADVERTISE_IP", "192.0.2.55")

    assert network.resolve_advertise_ip() == "192.0.2.55"


def test_iter_local_ipv4_candidates_dedupes_sources(monkeypatch):
    monkeypatch.setattr(network, "_route_probe_ipv4", lambda _target, _port: "10.0.0.5")
    monkeypatch.setattr(network, "_ip_command_ipv4_candidates", lambda: iter(["10.0.0.5", "10.0.0.6"]))
    monkeypatch.setattr(network, "_hostname_ipv4_candidates", lambda: iter(["127.0.0.1", "10.0.0.6", "10.0.0.7"]))

    assert list(network.iter_local_ipv4_candidates(route_targets=(("example.invalid", 1),))) == [
        "10.0.0.5",
        "10.0.0.6",
        "10.0.0.7",
    ]


def test_resolve_advertise_ip_falls_back_when_no_candidate(monkeypatch):
    monkeypatch.delenv("HORUS_SDK_ADVERTISE_IP", raising=False)
    monkeypatch.delenv("HORUS_ADVERTISE_IP", raising=False)
    monkeypatch.setattr(network, "iter_local_ipv4_candidates", lambda: iter(()))

    assert network.resolve_advertise_ip(fallback="127.0.0.9") == "127.0.0.9"
