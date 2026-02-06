import sys
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional


@dataclass
class _TopicState:
    state: str  # "SUBSCRIBED" | "UNSUBSCRIBED"
    last_changed: float


class TopicStatusBoard:
    """
    Live, single-line-per-topic status board that updates in-place (TTY only).
    Falls back to concise, de-duplicated one-liners when not a TTY.

    Usage:
      board = get_topic_status_board()
      board.on_subscribe(topic)
      board.on_unsubscribe(topic)
      board.stop()  # optional cleanup
    """

    def __init__(self, max_hz: float = 10.0):
        self._topics: "OrderedDict[str, _TopicState]" = OrderedDict()
        self._lock = threading.RLock()
        self._running = False
        self._render_thread: Optional[threading.Thread] = None
        self._isatty = sys.stdout.isatty()
        self._max_hz = max_hz
        self._last_render = 0.0
        self._pending_render = False
        self._rendered_lines = 0
        self._stopped = False
        self._silent = False

    def start(self):
        if self._running or self._stopped or self._silent:
            return
        self._running = True
        if self._isatty:
            self._render_thread = threading.Thread(
                target=self._render_loop, daemon=True
            )
            self._render_thread.start()

    def stop(self):
        with self._lock:
            self._running = False
            self._stopped = True
        if self._render_thread:
            self._render_thread.join(timeout=0.5)
        # On TTY, clear the board area (leave final concise summary?)
        if self._isatty and self._rendered_lines > 0:
            # Move cursor up and clear lines
            sys.stdout.write("\r")
            for _ in range(self._rendered_lines):
                sys.stdout.write("\x1b[2K\n")  # clear line and move down
            # Move cursor back up to where we were
            for _ in range(self._rendered_lines):
                sys.stdout.write("\x1b[1A")
            sys.stdout.flush()

    def on_subscribe(self, topic: str):
        self._update(topic, "SUBSCRIBED")

    def on_unsubscribe(self, topic: str):
        self._update(topic, "UNSUBSCRIBED")

    def set_silent(self, silent: bool):
        with self._lock:
            self._silent = silent

    def snapshot(self):
        with self._lock:
            return OrderedDict(
                (topic, state.state) for topic, state in self._topics.items()
            )

    def _update(self, topic: str, state: str):
        # Auto-start on first use (safe no-op if non-TTY)
        if not self._running and not self._stopped and not self._silent:
            self.start()
        ts = time.time()
        with self._lock:
            prev = self._topics.get(topic)
            if prev and prev.state == state:
                # No change → skip duplicate non-TTY logs
                pass
            else:
                self._topics[topic] = _TopicState(state=state, last_changed=ts)
                # Maintain stable ordering by reinserting
                self._topics.move_to_end(topic)
                if self._silent:
                    return
                if self._isatty:
                    self._pending_render = True
                else:
                    # Concise, de-duplicated one-liner
                    if state == "SUBSCRIBED":
                        icon = "\033[92m✓\033[0m"
                        status = "\033[90mSubscribed\033[0m"
                    else:
                        icon = "\033[93m⧖\033[0m"
                        status = "\033[90mUnsubscribed\033[0m"
                    sys.stdout.write(f"  {icon} Topic: {topic} — {status}\n")
                    sys.stdout.flush()

    def _render_loop(self):
        # Throttle to max_hz
        min_interval = 1.0 / max(self._max_hz, 1.0)
        while self._running:
            now = time.time()
            do_render = False
            with self._lock:
                topics_snapshot: Optional["OrderedDict[str, _TopicState]"]
                if self._pending_render and (now - self._last_render) >= min_interval:
                    self._pending_render = False
                    self._last_render = now
                    do_render = True
                    topics_snapshot = OrderedDict(self._topics)
                else:
                    topics_snapshot = None
            if do_render and topics_snapshot is not None:
                self._render_board(topics_snapshot)
            time.sleep(0.01)

    def _render_board(self, topics: "OrderedDict[str, _TopicState]"):
        # Move cursor up to the start of previous board
        if self._rendered_lines > 0:
            sys.stdout.write(
                f"\x1b[{self._rendered_lines}F"
            )  # cursor up N lines to column 0
        lines = []
        for topic, st in topics.items():
            if st.state == "SUBSCRIBED":
                icon = "\033[92m✓\033[0m"
                status = "\033[92mSUBSCRIBED\033[0m"
            else:
                icon = "\033[93m⧖\033[0m"
                status = "\033[93mUNSUBSCRIBED\033[0m"
            # Keep existing visual style: two-space indent, icon, label colored gray
            line = f"  {icon} Topic: \033[90m{topic}\033[0m — {status}"
            lines.append(line)
        output = "\n".join(lines) + ("\n" if lines else "")
        # Clear as many lines as previously rendered before writing new content
        if self._rendered_lines > 0:
            for _ in range(self._rendered_lines):
                sys.stdout.write("\x1b[2K\n")
            # Move cursor back up by new lines count to start writing
            sys.stdout.write(f"\x1b[{len(lines)}F")
        # Write new lines
        sys.stdout.write(output)
        sys.stdout.flush()
        self._rendered_lines = len(lines)


_singleton_board: Optional[TopicStatusBoard] = None


def get_topic_status_board() -> TopicStatusBoard:
    global _singleton_board
    if _singleton_board is None:
        _singleton_board = TopicStatusBoard()
    return _singleton_board
