"""Local installer file management. No ROS, sudo or third-party dependencies."""

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import tempfile
import time

START = "# >>> HORUS installer >>>"
END = "# <<< HORUS installer <<<"


def atomic_write(path, text, mode=0o644):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=path.name + ".tmp-")
    try:
        with os.fdopen(fd, "w") as output:
            output.write(text)
        os.chmod(temporary, path.stat().st_mode & 0o777 if path.exists() else mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def configure_shell(root, enabled, check_only=False):
    path = (Path.home() / ".bashrc").resolve()
    original = path.read_text() if path.exists() else ""
    lines = original.splitlines(keepends=True)
    starts = [i for i, line in enumerate(lines) if line.strip() == START]
    ends = [i for i, line in enumerate(lines) if line.strip() == END]
    if starts or ends:
        if len(starts) != 1 or len(ends) != 1 or starts[0] >= ends[0]:
            raise ValueError("Malformed HORUS block in .bashrc; left unchanged")
        block = "".join(lines[starts[0] : ends[0] + 1])
        # Do not remove a different installation's integration on uninstall.
        if not enabled and not any(
            line.strip() in (f"export HORUS_HOME={shlex.quote(str(root))}",
                             f'export HORUS_HOME="{root}"')
            for line in block.splitlines()
        ):
            return
        lines[starts[0] : ends[0] + 1] = []
    text = "".join(lines)
    if enabled:
        text = text.rstrip("\n") + "\n\n" + START + "\n"
        text += f"export HORUS_HOME={shlex.quote(str(root))}\n"
        text += 'if [[ ":$PATH:" != *":$HORUS_HOME/bin:"* ]]; then\n'
        text += '  export PATH="$HORUS_HOME/bin:$PATH"\nfi\n' + END + "\n"
    if check_only:
        return
    if text != original:
        if path.exists():
            backup = path.with_name(path.name + f".horus-backup-{time.time_ns()}")
            shutil.copy2(path, backup)
            print(f"Shell backup: {backup}")
        atomic_write(path, text)


def state(root, channel, distro, webrtc, shell, sdk_ref, ros_ref, sdk_sha, ros_sha):
    values = dict(HORUS_INSTALL_ROOT=str(root), HORUS_CHANNEL=channel,
                  HORUS_ROS_DISTRO=distro, HORUS_WEBRTC=webrtc,
                  HORUS_SHELL_CONFIG=shell, HORUS_SDK_REF=sdk_ref,
                  HORUS_ROS2_REF=ros_ref,
                  HORUS_INSTALLED_AT=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()))
    atomic_write(root / "state/install-config.env",
                 "".join(f"{k}={shlex.quote(v)}\n" for k, v in values.items()))
    payload = dict(values, schema_version=2, status="complete", runtime_mode="full",
                   resolved_commits={"horus_sdk": sdk_sha, "horus_ros2": ros_sha})
    atomic_write(root / "state/install-summary.json", json.dumps(payload, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("command", choices=("shell", "shell-check", "state"))
    parser.add_argument("root", type=Path)
    parser.add_argument("values", nargs="*")
    args = parser.parse_args()
    if args.command in ("shell", "shell-check"):
        configure_shell(args.root, args.values == ["auto"], args.command == "shell-check")
    else:
        state(args.root, *args.values)


if __name__ == "__main__":
    main()
