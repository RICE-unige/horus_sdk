#!/usr/bin/env python3
"""Register legged robots for the Robot Manager Stand Up / Sit Down flow."""

import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(SCRIPT_DIR, "..")
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

import sdk_typical_ops_demo


def _has_option(argv, name):
    return any(arg == name or arg.startswith(name + "=") for arg in argv)


def main():
    user_args = sys.argv[1:]
    defaults = []

    if not _has_option(user_args, "--teleop-profile"):
        defaults.extend(["--teleop-profile", "legged"])
    if not _has_option(user_args, "--robot-names") and not _has_option(user_args, "--robot-count"):
        defaults.extend(["--robot-names", "legged_1,legged_2,legged_3"])
    if not _has_option(user_args, "--go-to-min-altitude"):
        defaults.extend(["--go-to-min-altitude", "0.05"])
    if not _has_option(user_args, "--go-to-max-altitude"):
        defaults.extend(["--go-to-max-altitude", "0.45"])

    sys.argv = [sys.argv[0]] + defaults + user_args
    sdk_typical_ops_demo.main()


if __name__ == "__main__":
    main()
