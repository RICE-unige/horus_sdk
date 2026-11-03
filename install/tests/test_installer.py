"""Offline regression tests. All writes/processes are confined to temporary fixtures."""

import json
import fcntl
import os
from pathlib import Path
import pty
import select
import shutil
import signal
import subprocess
import sys
import tarfile
import tempfile
import time
import unittest

REPO = Path(__file__).resolve().parents[2]
INSTALL = REPO / "install"
sys.path.insert(0, str(INSTALL / "lib"))
from manage import configure_shell, state
from launch_manager import identity, read_running


class InstallerTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="horus-installer-test-")
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.home = self.base / "home"
        self.home.mkdir()
        self.root = self.home / "robot team's $lab"
        self.env = dict(os.environ, HOME=str(self.home), NO_COLOR="1", TERM="dumb",
                        GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL="/dev/null")
        self.log = self.base / "install.log"

    def command(self, args, ok=True, **kwargs):
        result = subprocess.run(args, cwd=self.base, env=self.env, text=True,
                                capture_output=True, timeout=30, **kwargs)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def shell(self, code, args=(), ok=True):
        prefix = 'set -euo pipefail\n'
        for lib in ("common", "ui", "args", "repos", "runtime", "helpers", "system"):
            prefix += f'source "{INSTALL}/lib/{lib}.sh"\n'
        prefix += f'LOG_FILE="{self.log}"\nSCRIPT_DIR="{INSTALL}"\n'
        return self.command(["bash", "-c", prefix + code, "_", *map(str, args)], ok=ok,
                            start_new_session=True)

    def git(self, root, *args):
        return self.command(["git", "-C", str(root), *args]).stdout.strip()

    def repo(self, name="origin"):
        root = self.base / name
        root.mkdir()
        self.git(root, "init", "-b", "main")
        self.git(root, "config", "user.email", "test@example.invalid")
        self.git(root, "config", "user.name", "Installer Test")
        (root / "tracked").write_text("first\n")
        (root / ".gitignore").write_text(".venv/\ninstall/\n")
        self.git(root, "add", ".")
        self.git(root, "commit", "-m", "test: added fixture")
        return root

    def helpers(self):
        (self.root / "state").mkdir(parents=True)
        (self.root / "state/install-root").write_text(str(self.root) + "\n")
        (self.root / "state/install.status").write_text("complete\n")
        state(self.root, "stable", "jazzy", "on", "manual", "a" * 40,
              "b" * 40, "a" * 40, "b" * 40)
        self.shell('write_helper_scripts "$1" jazzy', [self.root])

    def test_all_shell_files_parse(self):
        paths = [REPO / "install.sh", INSTALL / "install.sh"]
        paths += list((INSTALL / "lib").glob("*.sh"))
        paths += list((INSTALL / "templates").iterdir())
        for path in paths:
            with self.subTest(path=path):
                self.command(["bash", "-n", str(path)])

    def test_read_only_check_does_not_create_target(self):
        self.command(["bash", str(REPO / "install.sh"), "--check", "--install-root", str(self.root), "--log-file", str(self.log)])
        self.assertFalse(self.root.exists())
        self.assertFalse(self.log.exists())

    def test_reject_humble_and_unsafe_roots(self):
        for args in (["--ros-distro", "humble"], ["--install-root", "/"],
                     ["--install-root", str(self.home)], ["--install-root", ""]):
            result = self.command(["bash", str(REPO / "install.sh"), "--check", *args], ok=False)
            self.assertNotEqual(result.returncode, 0)

    def test_noninteractive_prompt_requires_yes(self):
        result = self.shell('read_prompt_value "Choose: " response', ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--yes", result.stderr)

    def test_interactive_answer_reaches_caller_even_with_piped_stdin(self):
        pid, master = pty.fork()
        if pid == 0:
            code = f'source "{INSTALL}/lib/common.sh"; response=old; read_prompt_value "Choose: " response </dev/null; printf "ANSWER=%s" "$response"'
            os.execve("/bin/bash", ["bash", "-c", code], self.env)
        output = b""
        try:
            os.write(master, b"custom value\n")
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                if select.select([master], [], [], 0.2)[0]:
                    try:
                        part = os.read(master, 4096)
                    except OSError:
                        break
                    if not part:
                        break
                    output += part
            self.assertIn(b"ANSWER=custom value", output)
        finally:
            os.close(master)
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            os.waitpid(pid, 0)

    def test_manifest_rejects_floating_stable_and_bad_schema(self):
        manifest = json.loads((INSTALL / "manifest/releases.json").read_text())
        path = self.base / "releases.json"
        for mutate in (lambda d: d.update(schema_version=2),
                       lambda d: d["channels"]["stable"].update(sdk_ref="main")):
            candidate = json.loads(json.dumps(manifest))
            mutate(candidate)
            path.write_text(json.dumps(candidate))
            self.assertNotEqual(self.shell('parse_manifest_channel "$1" stable', [path], ok=False).returncode, 0)

    def test_optional_manifest_notes_and_release_pin(self):
        manifest = json.loads((INSTALL / "manifest/releases.json").read_text())
        del manifest["channels"]["stable"]["notes"]
        path = self.base / "manifest.json"
        path.write_text(json.dumps(manifest))
        result = self.shell('HORUS_RELEASE_SDK_REF="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"; parse_manifest_channel "$1" stable; echo "$MANIFEST_SDK_REF"', [path])
        self.assertIn("a" * 40, result.stdout)

    def test_root_rerun_and_unmanaged_refusal(self):
        self.shell('prepare_install_root "$1"', [self.root])
        self.shell('prepare_install_root "$1"', [self.root])
        other = self.base / "unmanaged"
        other.mkdir()
        (other / "valuable").write_text("keep")
        self.assertNotEqual(self.shell('prepare_install_root "$1"', [other], ok=False).returncode, 0)
        self.assertEqual(sorted(p.name for p in other.iterdir()), ["valuable"])

    def test_repository_fast_forward_and_local_commit_protection(self):
        origin = self.repo()
        target = self.base / "installed"
        code = 'sync_repo_to_ref "$1" refs/heads/main "$2" sdk'
        self.shell(code, [origin, target])
        (origin / "tracked").write_text("second")
        self.git(origin, "commit", "-am", "test: updated fixture")
        self.shell(code, [origin, target])
        self.assertEqual(self.git(origin, "rev-parse", "HEAD"), self.git(target, "rev-parse", "HEAD"))
        self.git(target, "config", "user.name", "Installer Test")
        self.git(target, "config", "user.email", "test@example.invalid")
        (target / "tracked").write_text("local work")
        self.git(target, "commit", "-am", "test: kept local work")
        before = self.git(target, "rev-parse", "HEAD")
        self.assertNotEqual(self.shell(code, [origin, target], ok=False).returncode, 0)
        self.assertEqual(before, self.git(target, "rev-parse", "HEAD"))
        self.assertEqual((target / "tracked").read_text(), "local work")

    def test_dirty_checkout_and_remote_mismatch_are_not_changed(self):
        origin = self.repo()
        target = self.base / "installed"
        code = 'sync_repo_to_ref "$1" refs/heads/main "$2" sdk'
        self.shell(code, [origin, target])
        (target / "untracked").write_text("keep")
        self.assertNotEqual(self.shell(code, [origin, target], ok=False).returncode, 0)
        self.assertEqual((target / "untracked").read_text(), "keep")
        (target / "untracked").unlink()
        self.git(target, "remote", "set-url", "origin", "https://example.invalid/user.git")
        self.assertNotEqual(self.shell(code, [origin, target], ok=False).returncode, 0)
        self.assertEqual(self.git(target, "remote", "get-url", "origin"), "https://example.invalid/user.git")

    def test_failed_clone_cleans_partial_directory(self):
        origin = self.repo()
        target = self.base / "installed"
        self.assertNotEqual(self.shell('sync_repo_to_ref "$1" refs/heads/missing "$2" sdk', [origin, target], ok=False).returncode, 0)
        self.assertFalse(target.exists())
        self.assertEqual(list(self.base.glob("installed.clone.*")), [])

    def test_ignored_local_assets_are_not_overwritten_by_updates(self):
        origin = self.repo()
        target = self.base / "installed"
        code = 'sync_repo_to_ref "$1" refs/heads/main "$2" sdk'
        self.shell(code, [origin, target])
        (target / "install").mkdir()
        asset = target / "install/valuable"
        asset.write_text("local asset")
        (origin / "install").mkdir()
        (origin / "install/valuable").write_text("incoming tracked file")
        self.git(origin, "add", "-f", "install/valuable")
        self.git(origin, "commit", "-m", "test: tracked a previously ignored file")
        before = self.git(target, "rev-parse", "HEAD")
        self.assertNotEqual(self.shell(code, [origin, target], ok=False).returncode, 0)
        self.assertEqual(asset.read_text(), "local asset")
        self.assertEqual(before, self.git(target, "rev-parse", "HEAD"))

    def test_install_lock_rejects_second_installer(self):
        self.shell('prepare_install_root "$1"', [self.root])
        with (self.root / "state/installer.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            failed = self.shell('prepare_install_root "$1"', [self.root], ok=False)
        self.assertNotEqual(failed.returncode, 0)
        self.assertIn("Another installer", failed.stderr)

    def test_uninstall_refuses_a_running_helper(self):
        self.helpers()
        with (self.root / "state/runtime.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            failed = self.command([str(self.root / "bin/horus-uninstall"), "--yes"], ok=False)
        self.assertNotEqual(failed.returncode, 0)
        self.assertTrue(self.root.exists())

    def test_setup_failure_is_not_hidden(self):
        setup = self.base / "broken.bash"
        setup.write_text("return 7\n")
        result = self.shell('source_setup_file "$1"', [setup], ok=False)
        self.assertEqual(result.returncode, 7)

    def test_incomplete_venv_is_preserved(self):
        venv = self.root / ".venv"
        venv.mkdir(parents=True)
        (self.root / "pyproject.toml").write_text("[project]\n")
        (venv / "valuable").write_text("keep")
        self.assertNotEqual(self.shell('setup_python_sdk_runtime "$1"', [self.root], ok=False).returncode, 0)
        self.assertEqual((venv / "valuable").read_text(), "keep")

    def test_managed_sdk_install_is_not_editable(self):
        venv = self.root / ".venv"
        (venv / "bin").mkdir(parents=True)
        (self.root / "pyproject.toml").write_text("[project]\n")
        (venv / "pyvenv.cfg").write_text("include-system-site-packages = true\n")
        python = venv / "bin/python"
        python.write_text("#!/bin/sh\nexit 0\n")
        python.chmod(0o755)
        result = self.shell('''
ROS_DISTRO=jazzy
run_silent() { printf '<%s>\\n' "$@"; }
setup_python_sdk_runtime "$1"
''', [self.root])
        self.assertIn("<pip>\n<install>\n<--retries>\n<2>\n<" + str(self.root) + ">", result.stdout)
        pip_call = result.stdout.split("<Verifying SDK and ROS Python imports>", 1)[0]
        self.assertNotIn("<-e>", pip_call)
        self.assertNotIn("<--editable>", pip_call)

    def test_helpers_do_not_install_python_shim_and_state_is_quoted(self):
        self.helpers()
        self.assertFalse((self.root / "bin/python3").exists())
        result = self.shell('source "$1/state/install-config.env"; printf "%s" "$HORUS_INSTALL_ROOT"', [self.root])
        self.assertEqual(result.stdout, str(self.root))
        for path in (self.root / "bin").iterdir():
            self.command(["bash", "-n", str(path)])

    def test_unknown_python_shim_is_preserved(self):
        (self.root / "bin").mkdir(parents=True)
        shim = self.root / "bin/python3"
        shim.write_text("user-owned\n")
        self.assertNotEqual(self.shell('write_helper_scripts "$1" jazzy', [self.root], ok=False).returncode, 0)
        self.assertEqual(shim.read_text(), "user-owned\n")

    def test_shell_integration_idempotent_and_malformed_block_preserved(self):
        bashrc = self.home / ".bashrc"
        bashrc.write_text("export USER_SETTING=yes\n")
        self.shell('configure_shell_integration "$1" auto', [self.root])
        first = bashrc.read_text()
        self.shell('configure_shell_integration "$1" auto', [self.root])
        self.assertEqual(first, bashrc.read_text())
        self.assertEqual(len(list(self.home.glob(".bashrc.horus-backup-*"))), 1)
        bad = "hello\n# >>> HORUS installer >>>\nkeep this\n"
        bashrc.write_text(bad)
        self.assertNotEqual(self.shell('configure_shell_integration "$1" auto', [self.root], ok=False).returncode, 0)
        self.assertEqual(bashrc.read_text(), bad)

    def test_uninstall_preserves_all_files_and_never_purges_ros(self):
        self.helpers()
        (self.root / "user-dataset").write_text("valuable")
        self.command([str(self.root / "bin/horus-uninstall"), "--yes"])
        self.assertFalse(self.root.exists())
        backups = list(self.home.glob("*.removed-*"))
        self.assertEqual(len(backups), 1)
        self.assertEqual((backups[0] / "user-dataset").read_text(), "valuable")

    def test_bootstrap_pins_and_cleans_success_and_failure(self):
        origin = self.repo()
        installer = origin / "install/install.sh"
        installer.parent.mkdir()
        installer.write_text('#!/bin/bash\nprintf "ARG=%s\\n" "$1"\nexit "${TEST_EXIT:-0}"\n')
        self.git(origin, "add", "-f", "install/install.sh")
        self.git(origin, "commit", "-m", "test: added entry point")
        bootstrap = self.base / "bootstrap.sh"
        shutil.copy(REPO / "install.sh", bootstrap)
        temp = self.base / "bootstrap-temp"
        temp.mkdir()
        self.env.update(TMPDIR=str(temp), HORUS_INSTALLER_REPO_URL=str(origin),
                        HORUS_INSTALLER_REF=self.git(origin, "rev-parse", "HEAD"))
        for status in (0, 7):
            self.env["TEST_EXIT"] = str(status)
            result = self.command(["bash", str(bootstrap), "space arg"], ok=status == 0)
            self.assertEqual(result.returncode, status)
            self.assertIn("ARG=space arg", result.stdout)
            self.assertEqual(list(temp.iterdir()), [])

    def test_update_refuses_legacy_installer_before_running_it(self):
        self.helpers()
        origin = self.repo()
        manifest = origin / "install/manifest/releases.json"
        manifest.parent.mkdir(parents=True)
        manifest.write_text('{"schema_version": 1}')
        (origin / "install/install.sh").write_text('exit 99\n')
        self.git(origin, "add", "-f", "install")
        self.git(origin, "commit", "-m", "test: added legacy installer")
        self.env.update(HORUS_INSTALLER_REPO_URL=str(origin), HORUS_INSTALLER_REF="main")
        result = self.command([str(self.root / "bin/horus-update")], ok=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertNotEqual(result.returncode, 99)
        self.assertIn("incompatible with guarded updates", result.stderr)
        self.assertEqual((self.root / "state/install.status").read_text(), "complete\n")

    def test_start_stop_owns_child_group_and_rejects_duplicate(self):
        self.helpers()
        grandchild_file = self.base / "grandchild.pid"
        script = 'import subprocess,time,sys,pathlib; p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(120)"]); pathlib.Path(sys.argv[1]).write_text(str(p.pid)); time.sleep(120)'
        command = [sys.executable, str(INSTALL / "lib/launch_manager.py"), str(self.root), "start",
                   sys.executable, "-c", script, str(grandchild_file)]
        launch = subprocess.Popen(command, env=self.env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(lambda: launch.poll() is None and launch.kill())
        deadline = time.monotonic() + 5
        record = None
        while time.monotonic() < deadline:
            record = read_running(self.root / "state/launch.json")
            if record and grandchild_file.exists():
                break
            time.sleep(0.05)
        self.assertIsNotNone(record)
        grandchild = int(grandchild_file.read_text())
        self.assertNotEqual(self.command(command, ok=False).returncode, 0)
        self.command([str(self.root / "bin/horus-stop")])
        stdout, stderr = launch.communicate(timeout=20)
        self.assertEqual(launch.returncode, 0, stdout + stderr)
        self.assertIsNone(identity(record["child_pid"]))
        self.assertIsNone(identity(grandchild))
        self.assertFalse((self.root / "state/launch.json").exists())

    def test_stale_pid_does_not_stop_an_unrelated_process(self):
        self.helpers()
        unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(120)"])
        self.addCleanup(unrelated.wait)
        self.addCleanup(unrelated.terminate)
        (self.root / "state/launch.json").write_text(json.dumps({"pid": unrelated.pid, "identity": ["old-boot", "old-start"]}))
        self.command([str(self.root / "bin/horus-stop")])
        self.assertIsNone(unrelated.poll())

    def test_install_end_to_end_rerun_and_failure_state(self):
        """Real orchestration/repos/helpers; privileged installs/build are explicit doubles."""
        tooling = self.base / "tooling"
        shutil.copytree(INSTALL, tooling / "install")
        sdk = self.repo("sdk-origin")
        ros = self.repo("ros-origin")
        manifest = json.loads((INSTALL / "manifest/releases.json").read_text())
        manifest["repos"] = {"horus_sdk": str(sdk), "horus_ros2": str(ros)}
        manifest["channels"]["stable"].update(sdk_ref=self.git(sdk, "rev-parse", "HEAD"),
                                                  ros2_ref=self.git(ros, "rev-parse", "HEAD"))
        (tooling / "install/manifest/releases.json").write_text(json.dumps(manifest))
        with (tooling / "install/lib/system.sh").open("a") as stream:
            stream.write('''
ensure_sudo() { :; }
setup_ros_apt_repository() { :; }
install_base_dependencies() { :; }
install_ros_dependencies() { :; }
install_webrtc_dependencies() { :; }
init_rosdep() { :; }
''')
        with (tooling / "install/lib/runtime.sh").open("a") as stream:
            stream.write('''
setup_python_sdk_runtime() { mkdir -p "$1/.venv"; }
build_ros2_backend_workspace() { mkdir -p "$1/install"; }
validate_runtime_install() { [ "${TEST_FAIL_VALIDATION:-0}" = 0 ]; }
''')
        command = ["bash", str(tooling / "install/install.sh"), "--yes", "--install-root", str(self.root), "--shell-config", "manual"]
        self.command(command)
        summary = json.loads((self.root / "state/install-summary.json").read_text())
        self.assertEqual(summary["resolved_commits"]["horus_sdk"], self.git(sdk, "rev-parse", "HEAD"))
        valuable = self.root / "sdk/.venv/user-package"
        valuable.write_text("keep on rerun")
        self.command(command)
        self.assertEqual(valuable.read_text(), "keep on rerun")
        self.env["TEST_FAIL_VALIDATION"] = "1"
        failed = self.command(command, ok=False)
        self.assertNotEqual(failed.returncode, 0)
        self.assertEqual((self.root / "state/install.status").read_text().strip(), "failed")
        self.assertNotIn("installation validated", failed.stdout)
        self.assertNotEqual(self.command([str(self.root / "bin/horus-python"), "-c", "print(1)"], ok=False).returncode, 0)
        del self.env["TEST_FAIL_VALIDATION"]
        self.command(command)
        self.assertEqual((self.root / "state/install.status").read_text().strip(), "complete")

    def test_release_archive_pins_commit_and_excludes_untracked_data(self):
        origin = self.repo()
        (origin / "pyproject.toml").write_text('[project]\nname="horus-sdk"\nversion="1.2.3"\n')
        brand = origin / "python/horus/utils/branding.py"
        brand.parent.mkdir(parents=True)
        brand.write_text('__version__ = "1.2.3"\n')
        shutil.copy(REPO / "install.sh", origin)
        (origin / "install/manifest").mkdir(parents=True)
        shutil.copy(INSTALL / "manifest/releases.json", origin / "install/manifest")
        self.git(origin, "add", ".")
        self.git(origin, "add", "-f", "install/manifest/releases.json")
        self.git(origin, "commit", "-m", "test: added release fixture")
        self.git(origin, "tag", "v1.2.3")
        (origin / "private-data").write_text("not for release")
        output = self.base / "artifacts"
        result = subprocess.run([sys.executable, str(INSTALL / "prepare_release.py"), "v1.2.3", "--output", str(output)], cwd=origin, env=self.env, text=True, capture_output=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stderr)
        commit = self.git(origin, "rev-parse", "HEAD")
        self.assertIn(f'BOOTSTRAP_SDK_REF="{commit}"', (output / "installer-install.sh").read_text())
        with tarfile.open(next(output.glob("*-source.tar.gz"))) as archive:
            self.assertFalse(any("private-data" in name for name in archive.getnames()))
            member = next(member for member in archive if member.name.endswith("manifest/releases.json"))
            manifest = json.load(archive.extractfile(member))
            self.assertEqual(manifest["channels"]["stable"]["sdk_ref"], commit)
        self.git(origin, "tag", "v2.0.0")
        mismatch = subprocess.run([sys.executable, str(INSTALL / "prepare_release.py"), "v2.0.0", "--check"], cwd=origin, env=self.env, text=True, capture_output=True, timeout=10)
        self.assertNotEqual(mismatch.returncode, 0)
        self.assertIn("must agree", mismatch.stderr)


if __name__ == "__main__":
    unittest.main()
