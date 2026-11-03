"""Read-only graph evidence for terminal status; never subscribes to sensor payloads."""

import re


def is_bridge_node(name):
    name = str(name or "").strip().lstrip("/")
    return bool(
        re.fullmatch(r"horus_(?:unity_bridge|backend(?:_node)?)(?:_[a-zA-Z0-9]+)*", name)
        or name.endswith(("_RosSubscriber", "_RosPublisher"))
    )


def topic_graph(node, topic, role="backend_sub"):
    """Report endpoint discovery only. Missing graph queries remain unknown, not zero."""
    try:
        publishers = node.get_publishers_info_by_topic(topic)
        subscribers = node.get_subscriptions_info_by_topic(topic)
    except Exception:
        return {"pubs": None, "subs": None, "link": "UNKNOWN"}

    result = {"pubs": len(publishers), "subs": len(subscribers)}
    backend_pub = any(is_bridge_node(info.node_name) for info in publishers)
    backend_sub = any(is_bridge_node(info.node_name) for info in subscribers)
    if role.startswith("sdk_"):
        try:
            local_name = node.get_name()
            namespace = node.get_namespace() if hasattr(node, "get_namespace") else None
        except Exception:
            return dict(result, link="UNKNOWN")

        def local(info):
            return info.node_name == local_name and (
                namespace is None or getattr(info, "node_namespace", None) == namespace
            )

        local_endpoint = any(local(info) for info in (publishers if role == "sdk_pub" else subscribers))
        bridge_endpoint = backend_sub if role == "sdk_pub" else backend_pub
        result["link"] = "NO_SDK" if not local_endpoint else "OK" if bridge_endpoint else "NO_BRIDGE"
    elif role == "backend_pub":
        result["link"] = "NO_BRIDGE" if not backend_pub else "OK" if subscribers else "NO_CONSUMER"
    else:
        result["link"] = "NO_SOURCE" if not publishers else "OK" if backend_sub else "NO_BRIDGE"
    return result
