"""Terminal presentation. Status values come from evidence, not animation."""

from collections import Counter
from contextlib import contextmanager
import os
import time

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
from rich.theme import Theme

from .branding import __version__

custom_theme = Theme({
    "info": "cyan", "warning": "yellow", "error": "bold red",
    "success": "green", "horus": "bold cyan", "label": "bold",
})
console = Console(theme=custom_theme)


def env_enabled(name: str) -> bool:
    return os.getenv(name, "").strip().lower() in {"1", "true", "yes", "on"}


def _text(value, style="") -> Text:
    # Remote names/errors are literal text, never Rich markup or terminal controls.
    value = str(value or "").replace("\n", " ").replace("\r", " ")
    return Text("".join(c for c in value if c.isprintable()), style=style)


def _state(value: str) -> Text:
    word = str(value or "Unknown").split(" ", 1)[0].lower()
    style = {
        "connected": "green", "registered": "green", "responding": "green",
        "failed": "bold red", "error": "bold red", "disconnected": "yellow",
        "stopped": "yellow", "queued": "yellow", "retrying": "yellow",
        "registering": "cyan", "re-registering": "cyan", "waiting": "yellow",
    }.get(word, "")
    return _text(value or "Unknown", style)


def _render_link_badge(value: str) -> Text:
    label, style = {
        "OK": ("Ready", "green"), "NO": ("Not found", "yellow"),
        "NO_SOURCE": ("No source", "yellow"), "NO_BRIDGE": ("No bridge", "yellow"),
        "NO_CONSUMER": ("No consumer", "yellow"), "NO_SDK": ("No SDK", "yellow"),
        "UNKNOWN": ("Unknown", "yellow"), "MEDIA": ("Media path", "cyan"),
    }.get(value, (value or "Unknown", ""))
    return _text(label, style)


def _render_camera_profile(profile: dict) -> str:
    if not profile:
        return ""
    transport = profile.get("active_transport")
    if transport:
        return f"{str(transport).upper()} reported"
    mode = profile.get("source_mode")
    if mode not in {"minimap", "teleop"}:
        mode = profile.get("startup", "minimap")
    transport = profile.get(mode)
    return f"{str(transport).upper()} configured" if transport else "Not reported"


def _render_transport_badge(profile=None) -> Text:
    description = _render_camera_profile(profile) if profile else "ROS"
    transport, _, qualifier = description.partition(" ")
    label, style = {
        "WEBRTC": ("WebRTC", "bold white on blue"),
        "ROS": ("ROS", "bold white on dark_green"),
    }.get(transport, (None, None))
    if label is None:
        return _text(description)
    badge = Text()
    badge.append(f" {label} ", style=style)
    if qualifier and qualifier != "reported":
        badge.append(f" {qualifier}", style="dim")
    return badge


def _build_topic_subtable(rows: list, *, core=False, show_counts=False, width=100) -> Table:
    table = Table(box=None, expand=True, pad_edge=False, header_style="bold", padding=(0, 1))
    show_transport = width >= 80
    show_receipts = any(row.get("data", "Not measured") != "Not measured" for row in rows)
    if width >= 115:
        table.add_column("Robot / group", ratio=1, overflow="ellipsis", no_wrap=True)
    table.add_column("Topic", ratio=3, overflow="ellipsis", no_wrap=True)
    if show_transport:
        table.add_column("Transport", width=20, no_wrap=True)
    table.add_column("Graph", width=11, no_wrap=True)
    if show_receipts:
        table.add_column("SDK receipt", width=16, no_wrap=True)
    # show_counts remains accepted for older callers; endpoint counts are not
    # a stream-health indicator and are intentionally absent from this UI.
    for row in rows:
        cells = []
        if width >= 115:
            cells.append(_text(row.get("robot") or "Core", "cyan"))
        cells.append(_text(row.get("topic")))
        if show_transport:
            cells.append(_render_transport_badge(row.get("camera_profile")))
        cells.append(_render_link_badge(row.get("link")))
        if show_receipts:
            cells.append(_text(row.get("data", "Not measured"), "yellow" if row.get("stale") else ""))
        table.add_row(*cells)
    return table


def create_dashboard_table(
    ip, port, bridge_status, app_link_state, operators_state, registration_state,
    sub_status, spinner_frame=None, topic_rows=None, show_counts=False, *,
    width=100, page=0, page_size=None, show_core=False,
):
    """Render a bounded single panel; retain the public call arguments."""
    summary = Table.grid(padding=(0, 2), expand=True)
    summary.add_column(width=13)
    summary.add_column(ratio=1)
    summary.add_row(_text("Endpoint", "bold"), _text(f"{ip}:{port}"))
    summary.add_row(_text("Bridge", "bold"), _state(bridge_status))
    summary.add_row(_text("App heartbeat", "bold"), _state(app_link_state))
    summary.add_row(_text("Operators", "bold"), _text(operators_state or "Not reported"))
    summary.add_row(_text("Registration", "bold"), _state(registration_state))
    if sub_status:
        summary.add_row(_text("Detail", "bold"), _text(sub_status))
    content = [summary]
    rows = topic_rows or []
    data_rows = [r for r in rows if r.get("topic_kind") != "core"]
    visible = rows if show_core or not data_rows else data_rows
    if visible:
        counts = Counter(row.get("link", "UNKNOWN") for row in visible)
        ready = counts.get("OK", 0)
        content.append(Text(f"\nTopics  {len(visible)} total | {ready} graph-ready | {len(visible) - ready} other", style="bold"))
        page_size = max(1, page_size or len(visible))
        pages = (len(visible) + page_size - 1) // page_size
        page %= pages
        shown = visible[page * page_size:(page + 1) * page_size]
        content.append(_build_topic_subtable(shown, show_counts=show_counts, width=width))
        if pages > 1:
            content.append(_text(f"Page {page + 1}/{pages} | rotating every 8s"))
        if width < 80:
            for row in shown:
                if row.get("camera_profile"):
                    detail = _text(f"Camera: {row['topic']} | ")
                    detail.append_text(_render_transport_badge(row['camera_profile']))
                    content.append(detail)
            content.append(Text.assemble(_render_transport_badge(), " topics unless labelled otherwise."))
        content.append(_text("Graph-ready is not proof of traffic or headset rendering.", "dim"))
    return Panel(
        Group(*content), title=_text(f"HORUS SDK  {__version__}", "bold cyan"),
        subtitle=_text("Ctrl+C to stop monitoring", "dim"),
        border_style="cyan", padding=(0, 1), expand=True,
    )


class ConnectionDashboard:
    """Coalesce setters; redraw at most 4 Hz. Pipes receive state changes only."""

    def __init__(self, ip, port, bridge_status="Checking", show_counts=False, *, output=None, clock=None):
        self.ip, self.port = ip, port
        self.bridge_status = bridge_status
        self.app_link_state = "Waiting for Horus App"
        self.operators_state = "Not reported"
        self.registration_state = "Idle"
        self.sub_status = ""
        self.topic_rows = []
        self.show_counts = show_counts
        self.output = output or console
        self.clock = clock or time.monotonic
        self.show_core = env_enabled("HORUS_DASHBOARD_SHOW_CORE")
        self.live = Live("", auto_refresh=False, console=self.output, vertical_overflow="ellipsis")
        self._interactive = self.output.is_terminal and not self.output.is_dumb_terminal
        self._started = False
        self._last_refresh = float("-inf")
        self._start_time = self.clock()
        self._last_state = None

    def render(self):
        width = self.output.size.width
        # Reserve room for wrapped summaries, legend and camera transport details.
        page_size = max(1, (self.output.size.height - 16) // (3 if width < 80 else 1))
        return create_dashboard_table(
            self.ip, self.port, self.bridge_status, self.app_link_state, self.operators_state,
            self.registration_state, self.sub_status, topic_rows=self.topic_rows,
            show_counts=self.show_counts, width=width, page_size=page_size,
            page=int((self.clock() - self._start_time) // 8), show_core=self.show_core,
        )

    def update_status(self, status):
        self.sub_status = status or ""

    def update_app_link(self, status):
        self.app_link_state = status

    def update_multi_operator(self, status):
        self.operators_state = status or "Not reported"

    def update_registration(self, status):
        self.registration_state = status

    def update_bridge(self, status):
        self.bridge_status = status

    def update_topics(self, rows, show_counts=None):
        self.topic_rows = list(rows or [])
        if show_counts is not None:
            self.show_counts = show_counts

    def tick(self, *, force=False):
        if not self._started:
            return
        now = self.clock()
        if not force and now - self._last_refresh < 0.25:
            return
        self._last_refresh = now
        if self._interactive:
            self.live.update(self.render(), refresh=True)
        else:
            state = (self.bridge_status, self.app_link_state, self.operators_state, self.registration_state)
            if state != self._last_state:
                self.output.print(_text(
                    f"HORUS | Bridge: {state[0]} | App: {state[1]} | Operators: {state[2]} | Registration: {state[3]}"
                    + (f" | {self.sub_status}" if self.sub_status else "")
                ))
                self._last_state = state

    def __enter__(self):
        self._started = True
        self._start_time = self.clock()
        if self._interactive:
            self.live.start()
        else:
            self.output.print(_text(f"HORUS SDK {__version__} | {self.ip}:{self.port}"))
        self.tick(force=True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is KeyboardInterrupt:
            self.update_status("Monitoring stopped by user; no registration failure implied.")
        self.tick(force=True)
        if self._interactive:
            self.live.stop()
        else:
            if self.sub_status:
                self.output.print(_text(self.sub_status))
            for row in self.topic_rows:
                self.output.print(_text(
                    f"{row.get('topic')} | Graph: {_render_link_badge(row.get('link')).plain}"
                    f" | Transport: {_render_transport_badge(row.get('camera_profile')).plain.strip()}"
                    f" | SDK receipt: {row.get('data', 'Not measured')}"
                ))
        self._started = False


def print_step(message):
    console.print(_text(f"> {message}", "horus"))


def print_success(message):
    console.print(_text(f"OK  {message}", "success"))


def print_error(message):
    console.print(_text(f"ERROR  {message}", "error"))


def print_info(message):
    console.print(_text(message, "info"))


def print_warning(message):
    console.print(_text(f"WARNING  {message}", "warning"))


@contextmanager
def status(message, spinner="dots"):
    with console.status(_text(message), spinner=spinner) as spinner_status:
        yield spinner_status
