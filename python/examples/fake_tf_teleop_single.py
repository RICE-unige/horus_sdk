#!/usr/bin/env python3
"""Single-robot teleop-driven fake TF publisher."""

from fake_tf_teleop_common import build_teleop_parser, run_from_args


def main():
    parser = build_teleop_parser(default_robot_count=1)
    parser.set_defaults(robot_name="test_bot")
    args = parser.parse_args()
    run_from_args(args)


if __name__ == "__main__":
    main()
