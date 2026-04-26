__version__ = "0.1.0-alpha"

import sys


def _safe_print(text: str = "") -> None:
    """Print branding text without failing on narrow terminal encodings."""
    encoding = getattr(sys.stdout, "encoding", None) or "utf-8"
    try:
        sys.stdout.write(str(text) + "\n")
    except UnicodeEncodeError:
        safe_text = str(text).encode(encoding, errors="replace").decode(encoding, errors="replace")
        sys.stdout.write(safe_text + "\n")


def show_ascii_art():
    """Display HORUS ASCII art header."""
    reset = "\033[0m"
    colors = ("\033[96m", "\033[94m", "\033[95m", "\033[35m", "\033[91m")

    rows = (
        (
            "\u2588\u2588\u2557  \u2588\u2588\u2557 ",
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2557 ",
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2557 ",
            "\u2588\u2588\u2557   \u2588\u2588\u2557",
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2557",
        ),
        (
            "\u2588\u2588\u2551  \u2588\u2588\u2551",
            "\u2588\u2588\u2554\u2550\u2550\u2550\u2588\u2588\u2557",
            "\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2557",
            "\u2588\u2588\u2551   \u2588\u2588\u2551",
            "\u2588\u2588\u2554\u2550\u2550\u2550\u2550\u255d",
        ),
        (
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2551",
            "\u2588\u2588\u2551   \u2588\u2588\u2551",
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d",
            "\u2588\u2588\u2551   \u2588\u2588\u2551",
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2557",
        ),
        (
            "\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2551",
            "\u2588\u2588\u2551   \u2588\u2588\u2551",
            "\u2588\u2588\u2554\u2550\u2550\u2588\u2588\u2557",
            "\u2588\u2588\u2551   \u2588\u2588\u2551",
            "\u255a\u2550\u2550\u2550\u2550\u2588\u2588\u2551",
        ),
        (
            "\u2588\u2588\u2551  \u2588\u2588\u2551",
            "\u255a\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d",
            "\u2588\u2588\u2551  \u2588\u2588\u2551",
            "\u255a\u2588\u2588\u2588\u2588\u2588\u2588\u2554\u255d",
            "\u2588\u2588\u2588\u2588\u2588\u2588\u2588\u2551",
        ),
        (
            "\u255a\u2550\u255d  \u255a\u2550\u255d ",
            "\u255a\u2550\u2550\u2550\u2550\u2550\u255d ",
            "\u255a\u2550\u255d  \u255a\u2550\u255d ",
            "\u255a\u2550\u2550\u2550\u2550\u2550\u255d ",
            "\u255a\u2550\u2550\u2550\u2550\u2550\u2550\u255d",
        ),
    )

    _safe_print()
    for row in rows:
        line = "".join(f"{colors[idx]}{segment}" for idx, segment in enumerate(row))
        _safe_print(f"{line}{reset}")

    _safe_print()
    _safe_print(f"\033[94m        Holistic Operational Reality{reset}")
    _safe_print(f"\033[95m            for Unified Systems{reset}")
    _safe_print()
    _safe_print(f"\033[2m\U0001F916 Mixed Reality Robot Management Platform{reset}")
    _safe_print(f"\033[2m\U0001F3D7\ufe0f Developed at RICE Lab, University of Genoa{reset}")
    _safe_print(f"\033[2mHORUS SDK v{__version__}{reset}")
    _safe_print("=" * 60)
