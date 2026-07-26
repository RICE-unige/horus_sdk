#!/usr/bin/env python3
"""Stream the PC-rendered ETH3D Courtyard map to HORUS MR."""

from __future__ import annotations

import sys

from remote_map_portal_registration import main


if __name__ == "__main__":
    if "--scene" not in sys.argv:
        sys.argv.extend(("--scene", "eth3d_courtyard"))
    raise SystemExit(main())
