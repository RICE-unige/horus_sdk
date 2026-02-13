---
title: Topic Monitoring
sidebar_position: 5
---

# Topic Monitoring

## When to use it

Use topic monitoring to distinguish link health from data activity and avoid false positives during disconnected or pre-workspace phases.

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
board.on_subscribe("/robot_1/camera/image_compressed")
print(board.render_text())
```

## Common failure and fix

- **Failure:** dashboard shows ACTIVE when publishers are absent.
- **Fix:** ensure monitor normalization and state source reconciliation are enabled; validate disconnected states map to `Link=OFF, Data=IDLE`.
