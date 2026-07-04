"""Multi-operator registration replay helpers."""

from __future__ import annotations

import json
import time
import uuid
from typing import Any, Dict, List, Optional, Tuple

try:
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - tests monkeypatch this when ROS is absent.
    String = None  # type: ignore


def _new_string_message():
    message_cls = String
    if message_cls is None:
        from . import robot_registry as robot_registry_module

        message_cls = getattr(robot_registry_module, "String", None)
    if message_cls is None:
        raise RuntimeError("std_msgs/String is not available")
    return message_cls()


def _trace(client, message: str) -> None:
    trace = getattr(client, "_trace_robot_description", None)
    if callable(trace):
        trace(message)


def publish_sdk_registry_replay_once(client, entries, replay_request: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
    if not client.ros_initialized or client.node is None:
        return False, {"error": "ROS2 not initialized"}
    if client.sdk_replay_begin_publisher is None or client.sdk_replay_item_publisher is None or client.sdk_replay_end_publisher is None:
        return False, {"error": "SDK replay publishers unavailable"}

    replay_request = dict(replay_request or {})
    request_id = str(replay_request.get("request_id") or uuid.uuid4().hex).strip() or uuid.uuid4().hex
    join_attempt_id = str(replay_request.get("join_attempt_id") or "").strip()
    requester_app_id = str(replay_request.get("requester_app_id") or replay_request.get("app_id") or "unknown").strip() or "unknown"
    now_ms = int(time.time() * 1000)

    payloads = []
    for _, _, msg in (entries or []):
        json_payload = str(getattr(msg, "data", "") or "")
        if json_payload:
            payloads.append(json_payload)

    expected_count = len(payloads)
    _trace(
        client,
        f"[SDK replay] publish_once_begin request_id='{request_id}' join_attempt_id='{join_attempt_id}' "
        f"requester_app_id='{requester_app_id}' entries={len(entries or [])} payloads={expected_count}",
    )

    begin_msg = _new_string_message()
    begin_msg.data = json.dumps(
        {
            "request_id": request_id,
            "join_attempt_id": join_attempt_id,
            "expected_count": expected_count,
            "ts_unix_ms": now_ms,
        }
    )
    client.sdk_replay_begin_publisher.publish(begin_msg)

    published_count = 0
    for idx, registration_json in enumerate(payloads):
        item_msg = _new_string_message()
        item_msg.data = json.dumps(
            {
                "request_id": request_id,
                "join_attempt_id": join_attempt_id,
                "index": idx,
                "registration_json": registration_json,
            }
        )
        client.sdk_replay_item_publisher.publish(item_msg)
        published_count += 1

    end_msg = _new_string_message()
    end_msg.data = json.dumps(
        {
            "request_id": request_id,
            "join_attempt_id": join_attempt_id,
            "expected_count": expected_count,
            "published_count": published_count,
            "ts_unix_ms": int(time.time() * 1000),
        }
    )
    client.sdk_replay_end_publisher.publish(end_msg)
    _trace(
        client,
        f"[SDK replay] publish_once_end request_id='{request_id}' join_attempt_id='{join_attempt_id}' "
        f"published_count={published_count}/{expected_count}",
    )

    return True, {
        "request_id": request_id,
        "join_attempt_id": join_attempt_id,
        "requester_app_id": requester_app_id,
        "expected_count": expected_count,
        "published_count": published_count,
    }


def publish_sdk_registry_replay(client, entries, replay_request: Optional[Dict[str, Any]] = None) -> Tuple[bool, Dict[str, Any]]:
    if not client.ros_initialized or client.node is None:
        return False, {"error": "ROS2 not initialized"}
    if client.sdk_replay_begin_publisher is None or client.sdk_replay_item_publisher is None or client.sdk_replay_end_publisher is None:
        return False, {"error": "SDK replay publishers unavailable"}

    base_request = dict(replay_request or {})
    request_id = str(base_request.get("request_id") or uuid.uuid4().hex).strip() or uuid.uuid4().hex
    base_request["request_id"] = request_id

    # Replay settings are configurable via internal attributes. The Unity
    # coordinator also retries requests, so keep this path simple/synchronous.
    attempt_count = max(1, int(getattr(client, "_sdk_replay_burst_attempts", 1) or 1))
    initial_delay_s = max(0.0, float(getattr(client, "_sdk_replay_burst_initial_delay_s", 0.0) or 0.0))
    inter_attempt_delay_s = max(0.0, float(getattr(client, "_sdk_replay_burst_inter_attempt_delay_s", 0.0) or 0.0))

    if initial_delay_s > 0.0:
        time.sleep(initial_delay_s)

    per_attempt_published_count: List[int] = []
    last_result: Dict[str, Any] = {}
    _trace(
        client,
        f"[SDK replay] burst_begin request_id='{request_id}' attempts={attempt_count} "
        f"initial_delay_s={initial_delay_s:.3f} inter_attempt_delay_s={inter_attempt_delay_s:.3f}",
    )

    for attempt_idx in range(attempt_count):
        _trace(client, f"[SDK replay] burst_attempt request_id='{request_id}' attempt={attempt_idx + 1}/{attempt_count}")
        ok, result = client._publish_sdk_registry_replay_once(entries, base_request)
        if not ok:
            _trace(client, f"[SDK replay] burst_failed request_id='{request_id}' attempt={attempt_idx + 1} error='{result}'")
            return False, result
        last_result = dict(result or {})
        per_attempt_published_count.append(int(last_result.get("published_count") or 0))
        if attempt_idx < (attempt_count - 1) and inter_attempt_delay_s > 0.0:
            time.sleep(inter_attempt_delay_s)

    last_result["attempt_count"] = attempt_count
    last_result["per_attempt_published_count"] = per_attempt_published_count
    last_result["total_published_count"] = int(sum(per_attempt_published_count))
    _trace(
        client,
        f"[SDK replay] burst_end request_id='{request_id}' attempts={attempt_count} "
        f"total_published_count={last_result['total_published_count']}",
    )
    return True, last_result
