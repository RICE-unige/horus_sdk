"""Native teaching data commands: python -m horus.teaching."""

import argparse
import json


def main():
    parser = argparse.ArgumentParser(description="HORUS Teach data tools")
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser("validate", help="Verify saved native artifacts")
    validate.add_argument("episode")
    recovery = commands.add_parser("recover", help="Recover an interrupted capture without modifying its raw evidence")
    recovery.add_argument("episode")
    fixture = commands.add_parser("fixture", help="Create a labeled offline capture fixture; no robot commands")
    fixture.add_argument("root")
    args = parser.parse_args()
    if args.command == "validate":
        from .dataset import Episode
        episode = Episode(args.episode)
        issues = episode.validate()
        print(json.dumps({"integrity_issues": issues, "capture": episode.manifest["capture"],
                          "quality": episode.manifest["quality"], "quality_issues": episode.manifest.get("quality_issues", [])}, indent=2))
        return 1 if issues else 0
    elif args.command == "recover":
        from .recovery import recover
        print(recover(args.episode))
    elif args.command == "fixture":
        from .fixture import create_fixture
        print(create_fixture(args.root))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
