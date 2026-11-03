"""CPU-only guards for the shared runtime consumer; never start Sim or CUDA."""

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest


SDK = Path(__file__).resolve().parents[2]
TOOLS = SDK / "python/examples/tools"
PROFILE = TOOLS / "mapping_runtime_shared.sh"


def load_tool(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def profile_environment(tmp_path, *, repeat=False, **extra):
    environment = {"HOME": str(tmp_path), "PATH": "/usr/bin:/bin", **extra}
    result = subprocess.run(
        ["bash", "--noprofile", "--norc", "-c",
         'set -eu; source "$1"; ' + ('source "$1"; ' if repeat else '') + 'env -0',
         "profile-test", str(PROFILE)],
        env=environment, capture_output=True, check=True,
    )
    return dict(entry.decode().split("=", 1) for entry in result.stdout.split(b"\0") if entry)


def test_profile_selects_canonical_python_and_fresh_caches_without_creating_them(tmp_path):
    environment = profile_environment(tmp_path)
    root = tmp_path / ".cache/horus/remote_render/isaacsim-6.2-py312-torch2.11-cu130"
    assert environment["HORUS_REMOTE_PYTHON"] == str(tmp_path / "isaac/env_isaacsim/bin/python")
    assert environment["HORUS_REMOTE_CACHE_ROOT"] == str(root)
    assert environment["TORCH_EXTENSIONS_DIR"] == str(root / "torch-extensions")
    assert environment["HORUS_NVENC_ROOT"] == str(root / "nvenc-color-vui-v1")
    assert environment["PYTHONNOUSERSITE"] == "1"
    assert "OMNI_KIT_ACCEPT_EULA" not in environment
    assert not root.exists()
    assert not (tmp_path / "isaac").exists()


def test_profile_removes_only_old_horus_overlay_and_toolkit(tmp_path):
    legacy = "/media/omotoye/New Volume/HORUS/diagnostics/mapping-fixes-2026-10-08"
    toolkit = str(tmp_path / ".cache/horus/mapping-demo/cuda-12.8")
    environment = profile_environment(
        tmp_path, PATH=toolkit + "/bin:/usr/bin:/bin",
        PYTHONPATH=f"/opt/ros/jazzy/lib/python3.12/site-packages:{legacy}/gpu-python:/custom/packages",
        CUDA_HOME=toolkit, CUDA_PATH=legacy + "/cuda-12.8",
        LD_LIBRARY_PATH=f"{tmp_path}/isaac/env_isaacsim_61/lib:/opt/ros/jazzy/lib",
    )
    assert environment["PATH"] == "/usr/bin:/bin"
    assert environment["PYTHONPATH"].endswith(
        "/opt/ros/jazzy/lib/python3.12/site-packages:/custom/packages")
    assert "mapping-fixes-2026-10-08" not in environment["PYTHONPATH"]
    assert "CUDA_HOME" not in environment and "CUDA_PATH" not in environment
    assert environment["LD_LIBRARY_PATH"] == "/opt/ros/jazzy/lib"


def test_profile_removes_old_isaac_interpreter_and_modules_without_changing_ros(tmp_path):
    old = tmp_path / "isaac/env_isaacsim_61"
    environment = profile_environment(
        tmp_path, PATH=f"{old}/bin:/usr/bin:/bin",
        PYTHONPATH=f"{old}/lib/python3.12/site-packages:/ros/packages",
        LD_LIBRARY_PATH=str(old / "lib"),
    )
    assert environment["PATH"] == "/usr/bin:/bin"
    assert environment["PYTHONPATH"].endswith(":/ros/packages")
    assert str(old) not in environment["PYTHONPATH"]
    assert "LD_LIBRARY_PATH" not in environment


def test_profile_preserves_explicit_python_cache_and_new_compiler(tmp_path):
    compiler_root = tmp_path / "cuda 13"
    (compiler_root / "bin").mkdir(parents=True)
    compiler = compiler_root / "bin/nvcc"
    compiler.write_text("#!/bin/sh\nexit 99\n")
    compiler.chmod(0o755)
    environment = profile_environment(
        tmp_path, HORUS_REMOTE_PYTHON="/custom/horus/bin/python",
        HORUS_REMOTE_CACHE_ROOT=str(tmp_path / "fresh ABI cache"),
        HORUS_REMOTE_CUDA_HOME=str(compiler_root),
    )
    assert environment["HORUS_REMOTE_PYTHON"] == "/custom/horus/bin/python"
    assert environment["HORUS_REMOTE_CACHE_ROOT"] == str(tmp_path / "fresh ABI cache")
    assert environment["CUDA_HOME"] == str(compiler_root)
    assert environment["PATH"].split(":")[0] == str(compiler_root / "bin")


def test_profile_rejects_missing_explicit_compiler_instead_of_falling_back(tmp_path):
    with pytest.raises(subprocess.CalledProcessError):
        profile_environment(tmp_path, HORUS_REMOTE_CUDA_HOME=str(tmp_path / "unprovisioned"))


def test_profile_rejects_an_inherited_legacy_python_selector(tmp_path):
    with pytest.raises(subprocess.CalledProcessError) as error:
        profile_environment(tmp_path, HORUS_REMOTE_PYTHON=str(tmp_path / "isaac/env_isaacsim_61/bin/python"))
    assert b"legacy Isaac 6.1 Python is not qualified" in error.value.stderr


def test_profile_can_be_sourced_twice_without_duplicating_the_overlay(tmp_path):
    once = profile_environment(tmp_path, PYTHONPATH="/ros/packages")
    twice = profile_environment(tmp_path, repeat=True, PYTHONPATH="/ros/packages")
    assert twice["PYTHONPATH"] == once["PYTHONPATH"]
    assert twice["PATH"] == once["PATH"]


@pytest.mark.parametrize("arguments", [[], ["--plan"]])
def test_dependency_plan_does_not_install_import_or_create_files(tmp_path, monkeypatch, capsys, arguments):
    helper = load_tool("remote_dependency_plan_test", TOOLS / "install_remote_render_dependencies.py")
    monkeypatch.setenv("HORUS_REMOTE_CACHE_ROOT", str(tmp_path / "new runtime"))
    monkeypatch.delenv("HORUS_NVENC_ROOT", raising=False)

    def forbidden(*args, **kwargs):
        pytest.fail("Plan attempted package discovery, installation or subprocess execution")

    monkeypatch.setattr(helper.subprocess, "run", forbidden)
    monkeypatch.setattr(helper.importlib.util, "find_spec", forbidden)
    assert helper.main(arguments) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan["shared_stack"]["torch"] == "2.11.0+cu130"
    assert plan["mode"] == "plan_only"
    assert plan["nvenc"]["source_version"] == "2.2.2"
    assert plan["nvenc"]["stock_package_is_not_a_substitute"]
    assert plan["nvenc"]["root"] == str(tmp_path / "new runtime/nvenc-color-vui-v1")
    assert plan["nvenc"]["requires"] == ["extra_output_delay", "horus_h264_profile", "horus_h264_color_range"]
    assert str(tmp_path / "new runtime") in plan["native_renderer_cache"]
    assert not (tmp_path / "new runtime").exists()


@pytest.mark.parametrize("folder", ["env_isaacsim", "env_isaacsim_61"])
def test_installer_refuses_shared_isaac_environment_even_with_force(tmp_path, monkeypatch, folder):
    helper = load_tool("remote_shared_install_test", TOOLS / "install_remote_render_dependencies.py")
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    monkeypatch.setattr(sys, "prefix", str(tmp_path / "isaac" / folder))
    monkeypatch.setattr(sys, "base_prefix", "/usr")
    monkeypatch.setattr(helper, "_pip_install", lambda *args: pytest.fail("Shared environment mutated"))
    with pytest.raises(SystemExit) as error:
        helper.main(["--install", "--force"])
    assert error.value.code == 2


def test_installer_refuses_base_python_and_inherited_lab_activation(tmp_path, monkeypatch):
    helper = load_tool("remote_private_install_test", TOOLS / "install_remote_render_dependencies.py")
    monkeypatch.setattr(sys, "prefix", "/usr")
    monkeypatch.setattr(sys, "base_prefix", "/usr")
    monkeypatch.delenv("ISAACLAB_PATH", raising=False)
    with pytest.raises(RuntimeError, match="private HORUS"):
        helper.require_private_environment()
    monkeypatch.setattr(sys, "prefix", str(tmp_path / "private"))
    monkeypatch.setenv("ISAACLAB_PATH", "/shared/IsaacLab")
    with pytest.raises(RuntimeError, match="coordinator-managed"):
        helper.require_private_environment()


def test_installer_refuses_shared_environment_relocated_through_symlink(tmp_path, monkeypatch):
    helper = load_tool("remote_shared_symlink_test", TOOLS / "install_remote_render_dependencies.py")
    home = tmp_path / "home"
    (home / "isaac").mkdir(parents=True)
    relocated = tmp_path / "large volume/runtime-env"
    relocated.mkdir(parents=True)
    (home / "isaac/env_isaacsim").symlink_to(relocated, target_is_directory=True)
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: home))
    monkeypatch.setattr(sys, "prefix", str(relocated))
    monkeypatch.setattr(sys, "base_prefix", "/usr")
    monkeypatch.delenv("ISAACLAB_PATH", raising=False)
    with pytest.raises(RuntimeError, match="coordinator-managed"):
        helper.require_private_environment()


def test_force_cannot_turn_a_plan_into_an_install():
    helper = load_tool("remote_force_test", TOOLS / "install_remote_render_dependencies.py")
    with pytest.raises(SystemExit) as error:
        helper.main(["--plan", "--force"])
    assert error.value.code == 2


def test_private_install_command_has_no_user_or_system_override_flags(monkeypatch):
    helper = load_tool("remote_pip_command_test", TOOLS / "install_remote_render_dependencies.py")
    calls = []
    monkeypatch.setattr(helper.subprocess, "run", lambda command, **kwargs: calls.append(command))
    helper._pip_install(["--no-deps", "gsplat==1.5.3"])
    assert len(calls) == 1
    assert "--user" not in calls[0] and "--break-system-packages" not in calls[0]
    assert calls[0][-2:] == ["--no-deps", "gsplat==1.5.3"]


@pytest.mark.parametrize("name,function,filename", [
    ("cuda_renderer", "build_cuda_renderer_library", "cuda_point_renderer.cu"),
    ("cuda_mesh_renderer", "build_cuda_mesh_renderer_library", "cuda_mesh_renderer.cu"),
])
def test_native_build_uses_new_cache_without_loading_or_overwriting_old_binary(
        tmp_path, monkeypatch, name, function, filename):
    module = __import__("horus.remote_rendering." + name, fromlist=[name])
    source = Path(module.__file__).with_name(filename)
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:16]
    kind = "points" if name == "cuda_renderer" else "mesh"
    library_name = f"libhorus_cuda_{kind}_{digest}.so"
    old = tmp_path / ".cache/horus/remote_renderer" / library_name
    old.parent.mkdir(parents=True)
    old.write_bytes(b"preserved old ABI")
    monkeypatch.setattr(Path, "home", classmethod(lambda cls: tmp_path))
    monkeypatch.setenv("HORUS_REMOTE_CACHE_ROOT", str(tmp_path / "new ABI"))
    monkeypatch.setattr(module, "_resolve_nvcc", lambda: "/central/cuda13/bin/nvcc")

    def fake_compile(command, **kwargs):
        assert command[0] == "/central/cuda13/bin/nvcc"
        Path(command[-1]).write_bytes(b"mock compiler output, not a native binary")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(module.subprocess, "run", fake_compile)
    built = getattr(module, function)()
    assert built == tmp_path / "new ABI/native" / library_name
    assert old.read_bytes() == b"preserved old ABI"


def test_nvenc_consumer_and_builder_share_explicit_overlay(tmp_path, monkeypatch):
    from horus.remote_rendering.nvenc_encoder import _load_low_latency_codec
    root = tmp_path / "new NVENC ABI"
    package = root / "site-packages/PyNvVideoCodec"
    package.mkdir(parents=True)
    monkeypatch.setenv("HORUS_NVENC_ROOT", str(root))
    codec = SimpleNamespace(HORUS_OUTPUT_DELAY_OPTION=1, HORUS_H264_PROFILE_OPTION=1,
                            HORUS_H264_COLOR_VUI_OPTION=1,
                            __file__=str(package / "__init__.py"))
    monkeypatch.setitem(sys.modules, "PyNvVideoCodec", codec)
    before = list(sys.path)
    assert _load_low_latency_codec() is codec
    assert sys.path == before
    builder = load_tool("nvenc_overlay_builder_test", TOOLS / "install_remote_nvenc.py")
    assert builder.ROOT == root


def test_nvenc_does_not_fall_back_to_old_codec_when_selected_overlay_is_missing(tmp_path, monkeypatch):
    from horus.remote_rendering.nvenc_encoder import _load_low_latency_codec
    monkeypatch.setenv("HORUS_NVENC_ROOT", str(tmp_path / "missing ABI"))
    monkeypatch.setitem(sys.modules, "PyNvVideoCodec", SimpleNamespace(
        HORUS_OUTPUT_DELAY_OPTION=1, HORUS_H264_PROFILE_OPTION=1))
    with pytest.raises(RuntimeError, match="not provisioned"):
        _load_low_latency_codec()


def test_nvenc_rejects_empty_new_overlay_and_preloaded_old_module(tmp_path, monkeypatch):
    from horus.remote_rendering.nvenc_encoder import _load_low_latency_codec
    root = tmp_path / "new NVENC ABI"
    (root / "site-packages").mkdir(parents=True)
    monkeypatch.setenv("HORUS_NVENC_ROOT", str(root))
    monkeypatch.setitem(sys.modules, "PyNvVideoCodec", SimpleNamespace(
        HORUS_OUTPUT_DELAY_OPTION=1, HORUS_H264_PROFILE_OPTION=1,
        __file__="/old/abi/PyNvVideoCodec/__init__.py"))
    with pytest.raises(RuntimeError, match="not provisioned"):
        _load_low_latency_codec()
    (root / "site-packages/PyNvVideoCodec").mkdir()
    with pytest.raises(RuntimeError, match="already loaded"):
        _load_low_latency_codec()


def test_mapping_launcher_reports_missing_remote_python_before_running_demo(tmp_path):
    if not Path("/opt/ros/jazzy/setup.bash").is_file():
        pytest.skip("Launcher integration requires the existing Jazzy setup script")
    environment = {"HOME": str(tmp_path), "PATH": "/usr/bin:/bin",
                   "HORUS_REMOTE_PYTHON": str(tmp_path / "not provisioned/bin/python")}
    result = subprocess.run(["bash", str(SDK / "mapping-demo"), "remote-mesh", "--check"],
                            env=environment, capture_output=True, text=True)
    assert result.returncode == 2
    assert "HORUS remote Python is unavailable" in result.stderr
    assert "Complete the coordinated runtime provisioning" in result.stderr
    assert not (tmp_path / ".cache").exists()
