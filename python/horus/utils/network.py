"""Network helpers for SDK/user-facing connection hints."""

from __future__ import annotations

import ipaddress
import os
import socket
import subprocess
from typing import Iterable, Iterator, Optional


def _usable_ipv4(value: str) -> Optional[str]:
    candidate = str(value or "").strip()
    if not candidate:
        return None
    try:
        address = ipaddress.ip_address(candidate)
    except ValueError:
        return None
    if address.version != 4 or address.is_loopback or address.is_unspecified:
        return None
    return candidate


def _configured_advertise_ip() -> Optional[str]:
    configured = os.environ.get("HORUS_SDK_ADVERTISE_IP") or os.environ.get("HORUS_ADVERTISE_IP")
    configured = str(configured or "").strip()
    return configured or None


def _route_probe_ipv4(target: str, port: int) -> Optional[str]:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((target, port))
            return _usable_ipv4(sock.getsockname()[0])
    except OSError:
        return None


def _ip_command_ipv4_candidates() -> Iterator[str]:
    try:
        result = subprocess.run(
            ["ip", "-o", "-4", "addr", "show", "scope", "global"],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=1.0,
        )
    except (OSError, subprocess.SubprocessError):
        return

    for line in result.stdout.splitlines():
        parts = line.split()
        for index, part in enumerate(parts):
            if part == "inet" and index + 1 < len(parts):
                address = parts[index + 1].split("/", 1)[0]
                usable = _usable_ipv4(address)
                if usable:
                    yield usable


def _hostname_ipv4_candidates() -> Iterator[str]:
    hostnames = {socket.gethostname(), socket.getfqdn()}
    for hostname in hostnames:
        if not hostname:
            continue
        try:
            infos = socket.getaddrinfo(hostname, None, socket.AF_INET, socket.SOCK_STREAM)
        except OSError:
            continue
        for info in infos:
            usable = _usable_ipv4(info[4][0])
            if usable:
                yield usable


def iter_local_ipv4_candidates(
    route_targets: Iterable[tuple[str, int]] = (("10.255.255.255", 1), ("8.8.8.8", 80)),
) -> Iterator[str]:
    """Yield reachable-looking local IPv4 candidates without returning loopback first."""

    seen: set[str] = set()

    for target, port in route_targets:
        candidate = _route_probe_ipv4(target, port)
        if candidate and candidate not in seen:
            seen.add(candidate)
            yield candidate

    for source in (_ip_command_ipv4_candidates, _hostname_ipv4_candidates):
        for raw_candidate in source():
            candidate = _usable_ipv4(raw_candidate)
            if candidate and candidate not in seen:
                seen.add(candidate)
                yield candidate


def resolve_advertise_ip(fallback: str = "127.0.0.1") -> str:
    """Resolve the IP address HORUS should show to the MR app/operator."""

    configured = _configured_advertise_ip()
    if configured:
        return configured

    for candidate in iter_local_ipv4_candidates():
        return candidate

    return fallback
