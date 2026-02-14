from contextlib import contextmanager
import time

from rich.align import Align
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.spinner import Spinner
from rich.table import Table
from rich.text import Text
from rich.theme import Theme

# Horus brand colors (softened for dashboard readability)
custom_theme = Theme(
    {
        "info": "dim cyan",
        "warning": "yellow",
        "error": "bold red",
        "success": "bold green",
        "horus": "bold cyan",
        "label": "bold white",
    }
)

console = Console(theme=custom_theme)


def _chip(label: str, enabled: bool, color: str) -> str:
    if enabled:
        return f"[bold {color}][ {label.upper()} ][/bold {color}]"
    return f"[dim][ {label.lower()} ][/dim]"


def _render_link_badge(link_value: str) -> str:
    normalized = str(link_value or "").strip().upper()
    if normalized == "OK":
        return _chip("on", True, "green")
    if normalized == "NO":
        return _chip("off", False, "white")
    if normalized:
        return f"[dim][ {normalized.lower()} ][/dim]"
    return _chip("off", False, "white")


def _render_data_badge(data_value: str) -> str:
    normalized = str(data_value or "").strip().upper()
    if normalized == "ACTIVE":
        return _chip("active", True, "green")
    if normalized == "STALE":
        return _chip("stale", True, "yellow")
    if normalized == "IDLE":
        return _chip("idle", False, "white")
    if normalized:
        return f"[dim][ {normalized.lower()} ][/dim]"
    return _chip("idle", False, "white")


def _render_camera_profile(profile: dict) -> str:
    if not isinstance(profile, dict) or not profile:
        return "-"

    minimap = str(profile.get("minimap", "ros")).strip().lower()
    teleop = str(profile.get("teleop", "webrtc")).strip().lower()
    startup = str(profile.get("startup", "minimap")).strip().lower()
    active_transport = str(profile.get("active_transport", "")).strip().lower()

    if minimap not in ("ros", "webrtc"):
        minimap = "ros"
    if teleop not in ("ros", "webrtc"):
        teleop = "webrtc"
    if startup not in ("minimap", "teleop"):
        startup = "minimap"

    if active_transport not in ("ros", "webrtc"):
        active_transport = teleop if startup == "teleop" else minimap

    if active_transport == "ros":
        ros_chip = "[bold black on green] ROS [/]"
        webrtc_chip = "[dim][ WEBRTC ][/dim]"
    else:
        ros_chip = "[dim][ ROS ][/dim]"
        webrtc_chip = "[bold white on blue] WEBRTC [/]"

    return f"{ros_chip} {webrtc_chip}"


def _build_topic_subtable(rows: list, *, core: bool, show_counts: bool) -> Table:
    show_camera_profile = any(bool(row.get("camera_profile")) for row in rows)

    topic_table = Table(show_edge=False, show_header=True, expand=True, box=None, pad_edge=False)

    if core:
        topic_table.add_column("Topic", style="dim yellow")
        topic_table.add_column("Flow", style="dim magenta")
        topic_table.add_column("Link", justify="right")
        topic_table.add_column("Data", justify="right")
        if show_counts:
            topic_table.add_column("P", justify="right", style="dim cyan")
            topic_table.add_column("S", justify="right", style="dim magenta")

        for row in rows:
            if show_counts:
                topic_table.add_row(
                    row.get("topic", ""),
                    row.get("role", ""),
                    _render_link_badge(row.get("link", "")),
                    _render_data_badge(row.get("data", "")),
                    str(row.get("pubs", 0)),
                    str(row.get("subs", 0)),
                )
            else:
                topic_table.add_row(
                    row.get("topic", ""),
                    row.get("role", ""),
                    _render_link_badge(row.get("link", "")),
                    _render_data_badge(row.get("data", "")),
                )

        return topic_table

    topic_table.add_column("Robot", style="dim cyan", no_wrap=True)
    topic_table.add_column("Topic", style="dim yellow")
    if show_camera_profile:
        topic_table.add_column("Transport", style="white")
    topic_table.add_column("Link", justify="right")
    topic_table.add_column("Data", justify="right")
    if show_counts:
        topic_table.add_column("P", justify="right", style="dim cyan")
        topic_table.add_column("S", justify="right", style="dim magenta")

    last_robot = None
    for row in rows:
        robot = row.get("robot", "")
        display_robot = robot if robot != last_robot else ""
        last_robot = robot

        cells = [display_robot, row.get("topic", "")]
        if show_camera_profile:
            cells.append(_render_camera_profile(row.get("camera_profile", {})))
        cells.append(_render_link_badge(row.get("link", "")))
        cells.append(_render_data_badge(row.get("data", "")))
        if show_counts:
            cells.append(str(row.get("pubs", 0)))
            cells.append(str(row.get("subs", 0)))

        topic_table.add_row(*cells)

    return topic_table


def create_dashboard_table(
    ip: str,
    port: int,
    bridge_status: str,
    app_link_state: str,
    registration_state: str,
    sub_status: str,
    spinner_frame,
    topic_rows: list = None,
    show_counts: bool = False,
):
    """Create the layout for the connection dashboard."""
    table = Table(show_edge=False, show_header=False, expand=True)
    table.add_column("Key", style="dim white", width=20)
    table.add_column("Value", style="white")

    bridge_style = "bold green" if "Active" in bridge_status else "bold yellow"
    table.add_row("Horus Bridge", f"[{bridge_style}]* {bridge_status}[/{bridge_style}]")
    table.add_row("Local IP Address", f"{ip}")
    table.add_row("Connection Port", f"{port}")
    table.add_row("", "")

    app_state = str(app_link_state or "").strip()
    if app_state.startswith("Connected"):
        table.add_row("App Link", f"{_chip('connected', True, 'green')} [dim]{app_state}[/dim]")
    elif app_state.startswith("Disconnected"):
        table.add_row("App Link", f"{_chip('disconnected', True, 'red')} [dim]{app_state}[/dim]")
    elif app_state.startswith("Waiting"):
        table.add_row("App Link", spinner_frame if spinner_frame is not None else f"[dim]{app_state}[/dim]")
    else:
        table.add_row("App Link", f"[dim]{app_state}[/dim]")

    reg_text = str(registration_state or "").strip()
    if reg_text.startswith("Registered"):
        reg_chip = _chip("registered", True, "green")
    elif reg_text.startswith("Failed"):
        reg_chip = _chip("failed", True, "red")
    elif reg_text.startswith("Registering") or reg_text.startswith("Re-registering"):
        reg_chip = _chip("busy", True, "yellow")
    elif reg_text.startswith("Queued"):
        reg_chip = _chip("queued", True, "yellow")
    else:
        reg_chip = _chip("idle", False, "white")

    detail = f" [dim]{sub_status}[/dim]" if sub_status else ""
    table.add_row("Registration", f"{reg_chip} [dim]{reg_text}[/dim]{detail}")

    if topic_rows:
        table.add_row("", "")

        core_rows = [row for row in topic_rows if row.get("topic_kind") == "core"]
        data_rows = [row for row in topic_rows if row.get("topic_kind") != "core"]

        if core_rows:
            table.add_row(
                "Core Topics",
                _build_topic_subtable(core_rows, core=True, show_counts=show_counts),
            )
        if data_rows:
            table.add_row(
                "Data Topics",
                _build_topic_subtable(data_rows, core=False, show_counts=show_counts),
            )

    return Panel(
        Align.center(table),
        title="[bold horus]Horus Connection Manager[/bold horus]",
        subtitle="[dim]Ctrl+C to Cancel[/dim]",
        border_style="cyan",
        padding=(1, 2),
    )


class ConnectionDashboard:
    def __init__(self, ip, port, bridge_status="Checking...", show_counts: bool = False):
        self.ip = ip
        self.port = port
        self.bridge_status = bridge_status
        self.app_link_state = "Waiting"
        self.registration_state = "Idle"
        self.sub_status = ""
        self.topic_rows = []
        self.show_counts = show_counts
        self.spinner = Spinner("dots", style="bold white")
        # Disable auto-refresh to avoid fighting with manual updates
        self.live = Live("", auto_refresh=False, console=console)

    def render(self):
        spinner_render = self.spinner.render(time.time())
        app_state = str(self.app_link_state or "").strip()

        if app_state.startswith("Waiting"):
            status_line = Text()
            status_line.append(spinner_render)
            status_line.append(f" {app_state}", style="bold white")
            return create_dashboard_table(
                self.ip,
                self.port,
                self.bridge_status,
                self.app_link_state,
                self.registration_state,
                self.sub_status,
                status_line,
                self.topic_rows,
                self.show_counts,
            )

        return create_dashboard_table(
            self.ip,
            self.port,
            self.bridge_status,
            self.app_link_state,
            self.registration_state,
            self.sub_status,
            None,
            self.topic_rows,
            self.show_counts,
        )

    def update_status(self, status):
        self.sub_status = status or ""
        self.live.update(self.render(), refresh=True)

    def update_app_link(self, status):
        self.app_link_state = status
        self.live.update(self.render(), refresh=True)

    def update_registration(self, status):
        self.registration_state = status
        self.live.update(self.render(), refresh=True)

    def update_bridge(self, status):
        self.bridge_status = status
        self.live.update(self.render(), refresh=True)

    def update_topics(self, rows: list, show_counts: bool = None):
        """Update the topic activity rows."""
        self.topic_rows = rows
        if show_counts is not None:
            self.show_counts = show_counts
        self.live.update(self.render(), refresh=True)

    def tick(self):
        """Update the dashboard animation (spinner)."""
        self.live.update(self.render(), refresh=True)

    def __enter__(self):
        self.live.start()
        # Set initial content after Live starts
        self.live.update(self.render(), refresh=True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.live.stop()
        if exc_type is not None:
            console.print("")


def print_step(message):
    """Print a step description."""
    console.print(f"[horus]*[/horus] {message}")


def print_success(message):
    """Print a success message."""
    console.print(f"[success]+[/success] {message}")


def print_error(message):
    """Print an error message."""
    console.print(f"[error]- {message}[/error]")


def print_info(message):
    """Print an info message."""
    console.print(f"[info]{message}[/info]")


@contextmanager
def status(message, spinner="dots"):
    """Show a loading spinner."""
    with console.status(f"[bold white]{message}[/bold white]", spinner=spinner) as spinner_status:
        yield spinner_status
