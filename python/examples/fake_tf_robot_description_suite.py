#!/usr/bin/env python3
"""Run fake TF ops suite with one wheeled + one legged robot defaults."""

from fake_tf_ops_suite import build_ops_suite_parser, run_from_args


def _remove_option(parser, option_name: str) -> None:
    action = parser._option_string_actions.get(option_name)  # pylint: disable=protected-access
    if action is None:
        return
    for option in list(action.option_strings):
        parser._option_string_actions.pop(option, None)  # pylint: disable=protected-access
    if action in parser._actions:  # pylint: disable=protected-access
        parser._actions.remove(action)  # pylint: disable=protected-access
    for group in parser._action_groups:  # pylint: disable=protected-access
        if action in group._group_actions:  # pylint: disable=protected-access
            group._group_actions.remove(action)  # pylint: disable=protected-access


def build_parser():
    parser = build_ops_suite_parser()
    parser.description = (
        "Robot-description demo fake runtime (wheeled + legged) with teleop/go-to/waypoint topics."
    )
    # Keep workspace scaling authority in MR only for this demo and avoid exposing a TF scale knob.
    _remove_option(parser, "--scale")
    parser.set_defaults(
        robot_count=2,
        robot_names="jackal,go1",
        scale=1.0,
        robot_base_frames="jackal:base_link,go1:base",
        no_static_frames=True,
        static_camera=False,
        publish_compressed_images=True,
        publish_occupancy_grid=False,
        strict_single_tf_publisher=True,
    )
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.scale = 1.0
    # Robot-description suite should not publish occupancy grid data.
    args.publish_occupancy_grid = False
    run_from_args(args)


if __name__ == "__main__":
    main()
