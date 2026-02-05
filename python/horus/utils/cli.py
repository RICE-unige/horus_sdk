from rich.console import Console
from rich.theme import Theme
from rich.spinner import Spinner
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.align import Align
from rich.layout import Layout
from rich.control import Control
from contextlib import contextmanager
import time

# Horus Brand Colors
custom_theme = Theme({
    "info": "dim cyan",
    "warning": "yellow",
    "error": "bold red",
    "success": "bold green",
    "horus": "bold cyan",
    "label": "bold white"
})

console = Console(theme=custom_theme)

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
    """Create the layout for the connection dashboard"""
    table = Table(show_edge=False, show_header=False, expand=True)
    table.add_column("Key", style="dim white", width=20)
    table.add_column("Value", style="bold cyan")
    
    # Bridge Status Row
    bridge_style = "bold green" if "Active" in bridge_status else "bold yellow"
    table.add_row("Horus Bridge", f"[{bridge_style}]● {bridge_status}[/{bridge_style}]")
    
    # Network Info
    table.add_row("Local IP Address", f"{ip}")
    table.add_row("Connection Port", f"{port}")
    
    table.add_row("", "") # Spacer
    
    # App Link Status
    if "Waiting" in app_link_state:
        table.add_row("App Link", spinner_frame)
    elif "Connected" in app_link_state:
        state_style = "bold green"
        icon = "OK"
        display_text = f"{icon} [{state_style}]{app_link_state}[/{state_style}]"
        table.add_row("App Link", display_text)
    elif "Disconnected" in app_link_state:
        state_style = "bold red"
        icon = "X"
        display_text = f"{icon} [{state_style}]{app_link_state}[/{state_style}]"
        table.add_row("App Link", display_text)
    else:
        icon = "!"
        state_style = "bold white"
        display_text = f"{icon} [{state_style}]{app_link_state}[/{state_style}]"
        table.add_row("App Link", display_text)

    # Registration Status
    if "Registered" in registration_state and "Re" not in registration_state:
        state_style = "bold green"
        icon = "✓"
    elif "Failed" in registration_state:
        state_style = "bold red"
        icon = "✗"
    elif "Registering" in registration_state or "Re-registering" in registration_state:
        state_style = "bold yellow"
        icon = "✓"
    else:
        state_style = "bold white"
        icon = "!"
    detail = f" [dim][{sub_status}][/dim]" if sub_status else ""
    display_text = f"{icon} [{state_style}]{registration_state}[/{state_style}]{detail}"
    table.add_row("Registration", display_text)

    # Topic Activity Section
    if topic_rows:
        table.add_row("", "") # Spacer
        
        # Nested Table for Topics
        topic_table = Table(show_edge=False, show_header=True, expand=True, box=None)
        topic_table.add_column("Robot", style="dim cyan")
        topic_table.add_column("Topic", style="dim yellow")
        topic_table.add_column("Role", style="magenta")
        topic_table.add_column("Link", justify="right", style="cyan")
        topic_table.add_column("Data", justify="right", style="green")
        if show_counts:
            topic_table.add_column("Pubs", justify="right", style="cyan")
            topic_table.add_column("Subs", justify="right", style="magenta")

        last_robot = None
        for row in topic_rows:
            robot = row.get("robot", "")
            topic = row.get("topic", "")
            role = row.get("role", "")
            link = row.get("link", "")
            data = row.get("data", "")
            pubs = str(row.get("pubs", 0))
            subs = str(row.get("subs", 0))

            display_robot = robot if robot != last_robot else ""
            last_robot = robot

            if show_counts:
                topic_table.add_row(display_robot, topic, role, link, data, pubs, subs)
            else:
                topic_table.add_row(display_robot, topic, role, link, data)
        
        table.add_row("Topic Activity", topic_table)

    # Instructions Panel content
    panel = Panel(
        Align.center(table),
        title="[bold horus]Horus Connection Manager[/bold horus]",
        subtitle="[dim]Ctrl+C to Cancel[/dim]",
        border_style="cyan",
        padding=(1, 2)
    )
    return panel

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
        # We compose the status line here to pass a single renderable or string
        spinner_render = self.spinner.render(time.time())
        
        if "Waiting" in self.app_link_state:
             # Create a composite renderable: Spinner + Text
             # We can't pass a tuple to creating dashboard easily unless we change signature.
             # Let's pass a Text object
             from rich.text import Text
             status_line = Text()
             status_line.append(spinner_render)
             style = "bold red" if "Disconnected" in self.app_link_state else "bold white"
             status_line.append(f" {self.app_link_state}", style=style)
             return create_dashboard_table(
                self.ip, self.port, self.bridge_status,
                self.app_link_state, self.registration_state, self.sub_status,
                status_line, self.topic_rows, self.show_counts
            )
        else:
            return create_dashboard_table(
                self.ip, self.port, self.bridge_status,
                self.app_link_state, self.registration_state, self.sub_status,
                None, self.topic_rows, self.show_counts
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
        """Update the topic activity rows"""
        self.topic_rows = rows
        if show_counts is not None:
            self.show_counts = show_counts
        self.live.update(self.render(), refresh=True)
        
    def tick(self):
        """Update the dashboard animation (spinner)"""
        self.live.update(self.render(), refresh=True)

    def __enter__(self):
        self.live.start()
        # Set initial content after Live starts
        self.live.update(self.render(), refresh=True)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.live.stop()
        if exc_type is not None:
             # Add a spacer for clear separation on exception
             console.print("")

def print_step(message):
    """Print a step description"""
    console.print(f"[horus]•[/horus] {message}")

def print_success(message):
    """Print a success message"""
    console.print(f"[success]✓[/success] {message}")

def print_error(message):
    """Print an error message"""
    console.print(f"[error]✗ {message}[/error]")

def print_info(message):
    """Print an info message"""
    console.print(f"[info]{message}[/info]")

@contextmanager
def status(message, spinner="dots"):
    """Show a loading spinner"""
    with console.status(f"[bold white]{message}[/bold white]", spinner=spinner) as status:
        yield status
