"""HORUS wordmark shared by the SDK and its command-line tools."""

__version__ = "0.1.0-alpha"


_WORDMARK = (
    "██╗  ██╗ ██████╗ ██████╗ ██╗   ██╗███████╗",
    "██║  ██║██╔═══██╗██╔══██╗██║   ██║██╔════╝",
    "███████║██║   ██║██████╔╝██║   ██║███████╗",
    "██╔══██║██║   ██║██╔══██╗██║   ██║╚════██║",
    "██║  ██║╚██████╔╝██║  ██║╚██████╔╝███████║",
    "╚═╝  ╚═╝ ╚═════╝ ╚═╝  ╚═╝ ╚═════╝ ╚══════╝",
)
_ASCII_WORDMARK = (
    "HH   HH  OOOOO  RRRRRR  UU   UU SSSSSSS",
    "HH   HH OO   OO RR   RR UU   UU SS     ",
    "HHHHHHH OO   OO RRRRRR  UU   UU SSSSSSS",
    "HH   HH OO   OO RR  RR  UU   UU      SS",
    "HH   HH  OOOOO  RR   RR  UUUUU  SSSSSSS",
)


def show_ascii_art(*, output=None):
    """Keep the HORUS wordmark, without raw ANSI escapes in logs."""
    from rich.console import Console
    from rich.text import Text

    output = output or Console()
    rows = _WORDMARK
    try:
        "".join(rows).encode(output.encoding)
    except (UnicodeEncodeError, LookupError):
        rows = _ASCII_WORDMARK
    output.print()
    for row in rows:
        output.print(Text(row, style="bold cyan"), no_wrap=True, overflow="crop")
    output.print()
    output.print(Text(f"HORUS SDK v{__version__}", style="bold"))
    output.print("Holistic Operational Reality for Unified Systems", markup=False)
    output.print("RICE Lab | University of Genoa", style="dim", markup=False)
    output.print()
