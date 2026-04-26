---
title: Topic Monitoring
sidebar_position: 5
---

# Topic Monitoring

## When to use it

Use topic monitoring when you want the SDK to distinguish transport health from actual data flow. This is what keeps the HORUS dashboard from claiming a robot is healthy just because a topic exists.

## Minimal example

```python
from horus.utils.topic_monitor import get_topic_monitor

monitor = get_topic_monitor()
monitor.track("/tf")
monitor.track("/horus/registration_ack")
```

## Realistic example

```python
from horus.utils.topic_status import get_topic_status_board

board = get_topic_status_board()
board.on_app_connected(True)
board.on_subscribe("/tf")
board.on_publish_activity("/tf")
board.on_subscribe("/atlas/camera/image_raw/compressed")
board.on_publish_activity("/atlas/camera/image_raw/compressed")

print(board.render_text())
```

Use this path when validating:

- app connection versus workspace activation
- topic link state versus actual publish activity
- multi-operator presence and replay visibility

## Common failure and fix

- **Failure:** dashboard rows look active even when a publisher or app session is missing.
- **Fix:** confirm that the monitored topics are the real runtime topics and that app connectivity, ACK state, and publish activity are all being fed into the same board instance.
