#!/usr/bin/env python3
"""Host-authority SDK demo for multi-operator (Host + Join) headset testing."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import sdk_typical_ops_demo as typical_ops_demo  # noqa: E402


def main():
    print(
        "[HORUS multi-operator v1]\n"
        "Run this SDK demo once as the host-authority backend.\n"
        "Headset A: Host -> Connect -> Create Workspace (host can draw before alignment completes)\n"
        "Headset B+: Join -> Connect -> Join Workspace (waits for alignment anchor, then host workspace sync)\n"
        "If a joiner is stuck for >45s, inspect Colocation / SharedSpatialAnchor logs."
    )
    typical_ops_demo.main()


if __name__ == "__main__":
    main()
