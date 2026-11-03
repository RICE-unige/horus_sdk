"""Cache selection must not rewrite missing external-drive links or start ROS."""

import pytest

from horus.description import RobotMeshBaker
from horus.description.robot_description_resolver import RobotDescriptionResolver


@pytest.fixture
def home(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.delenv("HORUS_ROBOT_MESH_CACHE", raising=False)
    return tmp_path


@pytest.mark.parametrize("env", [None, ""])
def test_unset_or_empty_override_keeps_default(home, monkeypatch, env):
    if env is not None:
        monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", env)
    baker = RobotMeshBaker()
    assert baker.cache_root == home / ".cache/horus/robot_description_meshes"
    assert baker.cache_root.is_dir()


def test_environment_expands_home_and_reaches_resolver(home, monkeypatch):
    monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", "~/local cache/meshes")
    resolver = RobotDescriptionResolver()
    assert resolver._mesh_baker.cache_root == home / "local cache/meshes"
    assert resolver._mesh_baker.cache_root.is_dir()
    assert not (home / ".cache/horus").exists()


def test_explicit_constructor_overrides_environment(home, monkeypatch):
    monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", str(home / "unused"))
    explicit = home / "explicit"
    assert RobotMeshBaker(cache_root=explicit).cache_root == explicit
    assert explicit.is_dir() and not (home / "unused").exists()
    assert RobotMeshBaker(cache_root="~/explicit").cache_root == explicit


def test_registration_uses_override_without_touching_dangling_symlink(home, monkeypatch):
    from horus.bridge.robot_registry import RobotRegistryClient

    link = home / ".cache/horus"
    link.parent.mkdir()
    missing = home / "unmounted-volume/cache"
    link.symlink_to(missing, target_is_directory=True)
    monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", str(home / "local-meshes"))
    initialized = []
    monkeypatch.setattr(RobotRegistryClient, "_initialize_ros2", lambda self: initialized.append(True))
    monkeypatch.setattr("horus.bridge.robot_registry.ROS2_AVAILABLE", True)
    client = RobotRegistryClient()
    assert initialized == [True]
    assert client._robot_description_resolver._mesh_baker.cache_root == home / "local-meshes"
    assert link.is_symlink() and link.readlink() == missing and not missing.exists()


def test_missing_default_link_produces_actionable_error_without_repair(home):
    link = home / ".cache/horus"
    link.parent.mkdir()
    target = home / "missing/volume/cache"
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(OSError, match="HORUS_ROBOT_MESH_CACHE") as caught:
        RobotMeshBaker()
    assert caught.value.filename == str(link / "robot_description_meshes")
    assert link.is_symlink() and not target.exists()


def test_invalid_override_does_not_silently_fall_back(home, monkeypatch):
    occupied = home / "not-a-directory"
    occupied.write_text("keep me", encoding="utf-8")
    monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", str(occupied))
    with pytest.raises(OSError, match="HORUS_ROBOT_MESH_CACHE"):
        RobotMeshBaker()
    assert occupied.read_text(encoding="utf-8") == "keep me"
    assert not (home / ".cache/horus").exists()


def test_blank_override_is_rejected(home, monkeypatch):
    monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", "   ")
    with pytest.raises(ValueError, match="must not be blank"):
        RobotMeshBaker()


def test_existing_cache_is_preserved(home, monkeypatch):
    cache = home / "cache"
    cache.mkdir()
    artifact = cache / "existing.obj"
    artifact.write_bytes(b"existing mesh")
    monkeypatch.setenv("HORUS_ROBOT_MESH_CACHE", str(cache))
    assert RobotMeshBaker().cache_root == cache
    assert artifact.read_bytes() == b"existing mesh"
