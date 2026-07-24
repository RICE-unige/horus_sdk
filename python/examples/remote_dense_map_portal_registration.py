#!/usr/bin/env python3
"""Stream the PC-rendered ETH Cow and Lady map to HORUS MR."""

from __future__ import annotations

import sys

from remote_map_portal_registration import main


if __name__ == "__main__":
    if "--scene" not in sys.argv:
        sys.argv.extend(("--scene", "cow_lady"))
    raise SystemExit(main())
