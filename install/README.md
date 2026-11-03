# HORUS installer maintenance

User instructions: [Installer guide](../docs/getting-started/installer.md).

## Responsibilities

- `../install.sh`: local entry point or bounded Git bootstrap; removes its temporary checkout.
- `install.sh`: preflight, install locking, step ordering and validation.
- `lib/`: platform/dependency setup, repository handling, runtime checks and local file/process helpers.
- `templates/`: installed user-facing commands; no global Python shim.
- `manifest/releases.json`: source URLs and channel revisions.
- `tests/`: offline tests using disposable homes, repositories and child processes.
- `prepare_release.py`: version checks and pinned bootstrap/source artifacts.
- `Dockerfile`: SDK/ROS client image. It does not contain a compiled bridge.

## Local validation

Run from the repository root:

```bash
/usr/bin/python3 -m unittest discover -s install/tests -v
shellcheck -S warning install.sh install/install.sh install/lib/*.sh install/templates/*
bash install.sh --check --install-root "$HOME/horus-release-check"
```

The unit suite does not invoke apt, sudo, real ROS services or robot commands.
The preflight command does not install anything or validate a build.

## Release checklist

1. Commit reviewed source changes and set matching versions in `pyproject.toml`
   and `python/horus/utils/branding.py`. Create a matching release tag.
2. Review the **ROS bridge commit pin** in the manifest. It must contain the
   bridge behavior expected by this SDK and be available on its public remote.
   Do not replace it with an uncommitted/local-only revision.
3. Check out the release tag and run
   `python3 install/prepare_release.py vX.Y.Z --check` (requires `packaging`).
4. Run the release workflow. It builds Python distributions from the repository
   root, checks them with Twine, packages only tracked source and pins the release
   bootstrap/SDK to the tag's actual commit.
5. Both WebRTC-on and WebRTC-off clean Ubuntu installations must pass before
   publication. The workflow records their logs and checks the helper runtime.
6. Complete real Quest/robot acceptance separately. Installer tests cannot prove
   map, teleoperation or multi-operator runtime correctness.

The checked-in stable manifest currently represents a historical baseline, not
a claim that all `dev-plus` work has been published. Release packaging substitutes
the SDK release commit without inventing or silently advancing the ROS pin.
The workflow rejects mismatched tags/versions instead of rewriting source during
publication. A prerelease image never replaces the container's `latest` tag.

For a custom installer source (testing only), `HORUS_INSTALLER_REPO_URL` and
`HORUS_INSTALLER_REF` select the bootstrap checkout. `horus-update` normally
fetches installer tooling from main; these variables can override that choice.
The update helper requires installer API version 2 and refuses older tooling
that could reset local work. API version is separate from the SDK release version.
Use `--check` or a disposable installation root before applying a new manifest.

## Limits

Git source pins are recorded, but apt/PyPI dependencies are not fully locked.
An update is not a two-repository/system-package transaction; failed installs
remain explicitly incomplete. Backups, logs, retired shims and uninstall
directories are retained for recovery and are not automatically deleted.
