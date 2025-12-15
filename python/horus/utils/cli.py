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

def create_dashboard_table(ip: str, port: int, bridge_status: str, connection_state: str, spinner_frame, topic_stats: dict = None):
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
    
    # Connection State with Spinner
    if "Success" in connection_state or "Connected" in connection_state:
        state_style = "bold green"
        icon = "✓"
        display_text = f"{icon} [{state_style}]{connection_state}[/{state_style}]"
        table.add_row("Horus App Status", display_text)
    elif "Disconnected" in connection_state:
        state_style = "bold red"
        icon = "✗"
        display_text = f"{icon} [{state_style}]{connection_state}[/{state_style}]"
        table.add_row("Horus App Status", display_text)
    elif "Waiting" in connection_state:
        table.add_row("Horus App Status", spinner_frame) 
    else:
        icon = "!"
        state_style = "bold white"
        display_text = f"{icon} [{state_style}]{connection_state}[/{state_style}]"
        table.add_row("Horus App Status", display_text)

    # Topic Activity Section
    if topic_stats:
        table.add_row("", "") # Spacer
        
        # Nested Table for Topics
        topic_table = Table(show_edge=False, show_header=True, expand=True, box=None)
        topic_table.add_column("Topic", style="dim yellow")
        topic_table.add_column("Pubs", justify="right", style="cyan")
        topic_table.add_column("Subs", justify="right", style="magenta")
        
        for topic, counts in topic_stats.items():
            topic_table.add_row(
                topic, 
                str(counts.get('pubs', 0)), 
                str(counts.get('subs', 0))
            )
        
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
    def __init__(self, ip, port, bridge_status="Checking..."):
        self.ip = ip
        self.port = port
        self.bridge_status = bridge_status
        self.connection_state = "Waiting for App..."
        self.topic_stats = {}
        self.spinner = Spinner("dots", style="bold white")
        # Disable auto-refresh to avoid fighting with manual updates
        self.live = Live("", auto_refresh=False, console=console)
        
    def render(self):
        # We compose the status line here to pass a single renderable or string
        spinner_render = self.spinner.render(time.time())
        
        if "Waiting" in self.connection_state:
             # Create a composite renderable: Spinner + Text
             # We can't pass a tuple to creating dashboard easily unless we change signature.
             # Let's pass a Text object
             from rich.text import Text
             status_line = Text()
             status_line.append(spinner_render)
             status_line.append(f" {self.connection_state}", style="bold white")
             return create_dashboard_table(
                self.ip, self.port, self.bridge_status, 
                self.connection_state, status_line, self.topic_stats
            )
        else:
            return create_dashboard_table(
                self.ip, self.port, self.bridge_status, 
                self.connection_state, None, self.topic_stats
            )

    def update_status(self, status):
        self.connection_state = status
        self.live.update(self.render(), refresh=True)
        
    def update_bridge(self, status):
        self.bridge_status = status
        self.live.update(self.render(), refresh=True)

    def update_topics(self, stats: dict):
        """Update the topic statistics dictionary"""
        self.topic_stats = stats
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
