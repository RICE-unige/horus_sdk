---
title: Topic Monitoring
sidebar_position: 5
---

# Topic Monitoring

## When to use it

The registration dashboard separates four observations: app heartbeat, registration acknowledgement, ROS graph discovery, and messages received by the SDK. None of these alone proves that a sensor is being rendered in the headset.

| Indicator | Evidence |
| --- | --- |
| Bridge: Responding | An app heartbeat was received within the last 3 seconds. |
| Bridge: Discovered | A known HORUS bridge/backend node is in the ROS graph. This is not a TCP health check. |
| App: Connected | A valid app heartbeat is fresh. Workspace readiness is shown separately. |
| App: Disconnected | Previously observed heartbeats have expired. Closing the app is not a registration rejection. |
| Registered | A successful, non-queued registration ACK was received for each requested robot. This does not certify every operator's rendered scene. |
| Queued | The application acknowledged the request but has not completed registration. |
| Retrying | The initial acknowledgement budget elapsed in keep-alive mode; monitoring and retries continue. |
| Graph: Ready | The expected publisher and subscriber endpoints were discovered. |
| Graph: No source / No bridge / No consumer | The corresponding expected endpoint was not discovered. |
| Graph: Unknown | The graph query failed. The dashboard does not report missing endpoints without evidence. |
| Graph: Media path | The camera uses or is configured for WebRTC; a ROS image subscriber is not required. WebRTC delivery is not verified here. |
| SDK receipt: Last ... ago | The SDK actually received this core message in a callback. |
| SDK receipt: Not measured | No payload receipt instrumentation is available for this stream. No FPS or delivery claim is made. |

The dashboard does **not** subscribe to camera, map or point-cloud payloads just to measure activity. It queries endpoint metadata at approximately 1 Hz and redraws at most 4 Hz. This keeps monitoring out of the heavy-data path. DDS discovery and polling introduce a delay; graph-ready does not verify QoS compatibility, source rate, bridge forwarding rate or headset display rate.

Camera labels show **configured** when only the registration profile is available; a fresh runtime transport report uses just the transport badge, without a "reported" suffix. WebRTC has a blue badge and ROS has a green badge; these colours identify the transport, not its health. Reports expire after 3.5 seconds. A transport report is not a video frame or a WebRTC connection acknowledgement. Operator presence expires after 5 seconds; selecting a private workspace does not mean that workspace is ready.

## Minimal example

Registration examples use the dashboard automatically. For standalone graph monitoring:

```python
from horus.utils.topic_monitor import get_topic_monitor

monitor = get_topic_monitor()
monitor.watch_topics(["/tf", "/scan"])
monitor.start()
# Keep your application running; call monitor.stop() when it shuts down.
```

The monitor observes the expected bridge subscription and source publisher. It starts in UNKNOWN until the graph has been queried. It does not receive these topics' sensor payloads.

## Realistic example

Monitor robot-to-app sensor endpoints and the opposite direction for a command topic:

```python
import time
from horus.utils.topic_monitor import get_topic_monitor
from horus.utils.topic_status import get_topic_status_board

board = get_topic_status_board()
monitor = get_topic_monitor()
previous_silent = board.is_silent
board.set_silent(True)
monitor.watch_topics(
    ["/rover/scan", "/rover/cmd_vel"],
    modes={"/rover/scan": "backend_sub", "/rover/cmd_vel": "backend_pub"},
)
monitor.start()
try:
    previous = None
    while True:
        current = board.snapshot()
        if current != previous:
            print(dict(current))
            previous = current
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    monitor.stop()
    board.set_silent(previous_silent)
```

In this monitor, SUBSCRIBED is an endpoint observation, not proof of data flow. On command topics it means the bridge publisher and a consumer exist; it does not mean the robot has been commanded. Registration alone no longer marks sensor topics subscribed.

## Terminal behavior

- The full HORUS wordmark is retained above one dashboard with connection, operator, registration and topic sections.
- Large topic lists rotate pages every 8 seconds; the summary counts cover the whole displayed topic set.
- Robot/data topics are shown by default. Set `HORUS_DASHBOARD_SHOW_CORE=1` to include registration, heartbeat and model-transfer topics.
- Publisher/subscriber counts are not displayed: endpoint counts alone do not establish stream health. The SDK receipt column appears only when the page includes receipt observations; sensor traffic is not measured by this dashboard.
- Narrow terminals omit secondary columns; redirected output records state transitions and a final topic snapshot without ANSI animation.
- `NO_COLOR=1` disables colour. `HORUS_SDK_NO_BANNER=1` suppresses import-time branding.
- `HORUS_SDK_DEBUG=1` includes verbose robot-description transport traces. Normal progress and failures remain visible without it.
- Ctrl+C stops monitoring. An intentional cancellation is not evidence that an earlier registration failed.

During a reconnect or app restart, old local acknowledgement state is invalidated and the dashboard waits for a new ACK. Keep-alive mode continues retrying; non-keep-alive dashboard mode applies its acknowledgement timeout after the app first appears by default. Existing examples retain their public registration arguments.

## Common failure and fix

- **No bridge, but RViz works:** an RViz subscriber is not a HORUS bridge subscriber. Check that the intended HORUS node is present in the same ROS domain. Custom-renamed bridge nodes may be shown as unrecognized.
- **Ready, but no image in MR:** endpoint discovery is not a frame-rate measurement. Check publisher output, QoS, camera decoding and headset visualization separately.
- **Queued:** use the reason reported by the application. The SDK does not guess that TF is missing by inspecting a possibly unrelated global `/tf`.
- **Unknown:** verify that the SDK's ROS node/context and graph queries are available.
- **The legacy client reports a TCP peer:** this is only an established socket on the bridge port, not an app handshake, workspace or robot registration.
