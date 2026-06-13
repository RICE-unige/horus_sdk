"""Run manifest definitions for HORUS experiment characterization."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
import subprocess
from typing import Any, Dict, Optional


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def default_run_id(
    experiment: str,
    condition: str,
    repetition: int = 1,
    timestamp: Optional[datetime] = None,
) -> str:
    stamp = (timestamp or datetime.now(timezone.utc)).strftime("%Y%m%d_%H%M%S")
    clean_experiment = _slug(experiment)
    clean_condition = _slug(condition)
    return f"{clean_experiment}_{clean_condition}_r{repetition:02d}_{stamp}"


def _slug(value: str) -> str:
    value = (value or "run").strip().lower()
    chars = [ch if ch.isalnum() else "_" for ch in value]
    slug = "".join(chars).strip("_")
    while "__" in slug:
        slug = slug.replace("__", "_")
    return slug or "run"


def _git_commit(cwd: Optional[Path]) -> str:
    if cwd is None:
        return ""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=str(cwd),
            text=True,
            capture_output=True,
            check=True,
            timeout=2,
        )
        return result.stdout.strip()
    except Exception:
        return ""


@dataclass(frozen=True)
class RunIdentity:
    run_id: str
    experiment: str
    condition: str
    repetition: int = 1


@dataclass
class ExperimentManifest:
    run_id: str
    experiment: str
    condition: str
    date: str = field(default_factory=utc_now_iso)
    duration_s: float = 120.0
    warmup_s: float = 10.0
    repetition: int = 1
    robot_count: int = 1
    operator_count: int = 1
    stream_count: int = 0
    transport: str = "ros2"
    topology: str = "lan"
    resolution: str = ""
    target_fps: float = 0.0
    robot_profile: str = "minimal"
    map_profile: str = "none"
    notes: str = ""
    source_git_commit: str = ""
    host: str = field(default_factory=platform.node)
    platform: str = field(default_factory=platform.platform)
    extra: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_identity(
        cls,
        identity: RunIdentity,
        *,
        duration_s: float = 120.0,
        warmup_s: float = 10.0,
        robot_count: int = 1,
        operator_count: int = 1,
        stream_count: int = 0,
        transport: str = "ros2",
        topology: str = "lan",
        resolution: str = "",
        target_fps: float = 0.0,
        robot_profile: str = "minimal",
        map_profile: str = "none",
        notes: str = "",
        repo_root: Optional[Path] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> "ExperimentManifest":
        return cls(
            run_id=identity.run_id,
            experiment=identity.experiment,
            condition=identity.condition,
            duration_s=duration_s,
            warmup_s=warmup_s,
            repetition=identity.repetition,
            robot_count=robot_count,
            operator_count=operator_count,
            stream_count=stream_count,
            transport=transport,
            topology=topology,
            resolution=resolution,
            target_fps=target_fps,
            robot_profile=robot_profile,
            map_profile=map_profile,
            notes=notes,
            source_git_commit=_git_commit(repo_root),
            extra=extra or {},
        )

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def save(self, run_dir: Path) -> Path:
        run_dir.mkdir(parents=True, exist_ok=True)
        path = run_dir / "run_manifest.json"
        path.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return path
