#!/usr/bin/env python3
"""Stream a textured triangle-mesh scene; Sponza is the default."""

from __future__ import annotations

import sys

from remote_map_portal_registration import main


if __name__ == "__main__":
    if "--scene" not in sys.argv:
        sys.argv.extend(("--scene", "sponza_mesh"))
    raise SystemExit(main())
