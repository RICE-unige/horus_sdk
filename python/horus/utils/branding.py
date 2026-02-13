__version__ = "0.1.0"


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

    print()
    for row in rows:
        line = "".join(f"{colors[idx]}{segment}" for idx, segment in enumerate(row))
        print(f"{line}{reset}")

    print()
    print(f"\033[94m        Holistic Operational Reality{reset}")
    print(f"\033[95m            for Unified Systems{reset}")
    print()
    print(f"\033[2m\U0001F916 Mixed Reality Robot Management Platform{reset}")
    print(f"\033[2m\U0001F3D7\ufe0f Developed at RICE Lab, University of Genoa{reset}")
    print(f"\033[2mHORUS SDK v{__version__}{reset}")
    print("=" * 60)
