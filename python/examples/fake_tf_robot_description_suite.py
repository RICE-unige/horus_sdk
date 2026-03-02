#!/usr/bin/env python3
"""Run fake TF ops suite with profile-based robot-description demo defaults."""

import argparse

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
        "Robot-description demo fake runtime with profile-based robot sets "
        "(classic=jackal+go1, real_models=anymal_c+jackal+go1+h1)."
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
    parser.add_argument(
        "--robot-profile",
        choices=["classic", "real_models"],
        default="classic",
        help="Robot profile preset for names/base frames (default: classic).",
    )
    return parser


def _apply_profile_defaults(args: argparse.Namespace) -> None:
    profile = str(getattr(args, "robot_profile", "classic") or "classic").strip().lower()
    if profile == "real_models":
        args.robot_count = 4
        args.robot_names = "anymal_c,jackal,go1,h1"
        args.robot_base_frames = "anymal_c:base,jackal:base_link,go1:base,h1:pelvis"
    else:
        args.robot_count = 2
        args.robot_names = "jackal,go1"
        args.robot_base_frames = "jackal:base_link,go1:base"


def main():
    parser = build_parser()
    args = parser.parse_args()
    _apply_profile_defaults(args)
    args.scale = 1.0
    # Robot-description suite should not publish occupancy grid data.
    args.publish_occupancy_grid = False
    run_from_args(args)


if __name__ == "__main__":
    main()
