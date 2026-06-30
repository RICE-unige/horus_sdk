#!/usr/bin/env python3
"""Docusaurus docs quality checks for HORUS SDK."""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]

REQUIRED_DOCS = [
    "docs/intro.md",
    "docs/getting-started/installation.md",
    "docs/getting-started/quickstart.md",
    "docs/getting-started/installer.md",
    "docs/architecture/system-boundary.md",
    "docs/architecture/runtime-flow.md",
    "docs/python-sdk/robot-model.md",
    "docs/python-sdk/sensors.md",
    "docs/python-sdk/dataviz.md",
    "docs/python-sdk/registration.md",
    "docs/python-sdk/topic-monitoring.md",
    "docs/examples/registration-flows.md",
    "docs/examples/camera-transport-profiles.md",
    "docs/examples/topic-monitoring-dashboard.md",
    "docs/examples/occupancy-grid-workflow.md",
    "docs/examples/gaussian-splat-fixture.md",
    "docs/integration/horus-ros2.md",
    "docs/integration/horus-mr-app.md",
    "docs/reference/implementation-status.md",
    "docs/reference/known-limitations.md",
    "docs/operations/troubleshooting.md",
    "docs/contributing/docs-maintenance.md",
]

IMPLEMENTED_API_PAGES = [
    ROOT / "docs" / "python-sdk" / "robot-model.md",
    ROOT / "docs" / "python-sdk" / "sensors.md",
    ROOT / "docs" / "python-sdk" / "dataviz.md",
    ROOT / "docs" / "python-sdk" / "registration.md",
    ROOT / "docs" / "python-sdk" / "topic-monitoring.md",
]

REQUIRED_SECTION_HEADERS = [
    "## When to use it",
    "## Minimal example",
    "## Realistic example",
    "## Common failure and fix",
]

INTEGRATION_PAGES = [
    ROOT / "docs" / "architecture" / "runtime-flow.md",
    ROOT / "docs" / "integration" / "horus-ros2.md",
    ROOT / "docs" / "integration" / "horus-mr-app.md",
]

STATUS_PAGE = ROOT / "docs" / "reference" / "implementation-status.md"
EXPECTED_STUB_PATHS = [
    "python/horus/bridge/ros2.py",
    "python/horus/bridge/unity_tcp.py",
    "python/horus/topics.py",
    "python/horus/robot/status.py",
    "python/horus/robot/teleop.py",
    "python/horus/robot/task.py",
    "python/horus/robot/dataviz.py",
    "python/horus/plugins/rosbot.py",
    "python/horus/core/exceptions.py",
]


def fail(message: str) -> None:
    print(f"ERROR: {message}")
    sys.exit(1)


def read_text(path: Path) -> str:
    if not path.exists():
        fail(f"Missing required docs file: {path.relative_to(ROOT)}")
    return path.read_text(encoding="utf-8")


def has_fenced_block(text: str, lang: str) -> bool:
    return f"```{lang}" in text


def check_required_docs() -> None:
    for rel in REQUIRED_DOCS:
        path = ROOT / rel
        if not path.exists():
            fail(f"Missing required docs file: {rel}")


def check_implemented_api_pages() -> None:
    for page in IMPLEMENTED_API_PAGES:
        content = read_text(page)
        if not has_fenced_block(content, "python"):
            fail(f"{page.relative_to(ROOT)} must include at least one ```python example block.")
        for header in REQUIRED_SECTION_HEADERS:
            if header not in content:
                fail(f"{page.relative_to(ROOT)} is missing required section header: {header}")


def check_integration_pages() -> None:
    for page in INTEGRATION_PAGES:
        content = read_text(page)
        if not has_fenced_block(content, "bash"):
            fail(f"{page.relative_to(ROOT)} must include at least one runnable ```bash flow.")


def check_stub_status_page() -> None:
    content = read_text(STATUS_PAGE)
    if "Status: Stub (not usable yet)" not in content:
        fail(f"{STATUS_PAGE.relative_to(ROOT)} must include explicit stub status text.")
    for path in EXPECTED_STUB_PATHS:
        if path not in content:
            fail(f"{STATUS_PAGE.relative_to(ROOT)} is missing stub module path: {path}")


def check_installer_command() -> None:
    content = read_text(ROOT / "docs" / "getting-started" / "installation.md")
    if "curl -fsSL https://raw.githubusercontent.com/RICE-unige/horus_sdk/main/install.sh | bash" not in content:
        fail("Installation docs must include canonical one-command installer.")


_LINK_RE = re.compile(r"\[[^\]]*\]\(([^)]+)\)")


def check_internal_links() -> None:
    """Validate doc-to-doc markdown links and /docs/ routes resolve to a file.

    Compensates for Docusaurus' build-time `onBrokenLinks: throw` when the site
    cannot be built locally. Only `.md` links and `/docs/...` routes are checked;
    external URLs, anchors, and static assets are skipped.
    """
    docs_dir = ROOT / "docs"
    errors = []
    for md in sorted(docs_dir.rglob("*.md")):
        for match in _LINK_RE.finditer(md.read_text(encoding="utf-8")):
            raw = match.group(1).strip().split()[0]
            target = raw.split("#")[0]
            if not target or target.startswith(("http://", "https://", "mailto:")):
                continue
            if target.startswith("/docs/"):
                candidate = docs_dir / target[len("/docs/"):]
                if candidate.suffix == "":
                    candidate = candidate.with_suffix(".md")
                if not candidate.exists():
                    errors.append(f"{md.relative_to(ROOT)} -> {raw}")
            elif target.endswith(".md") and not (md.parent / target).exists():
                errors.append(f"{md.relative_to(ROOT)} -> {raw}")
    if errors:
        fail("Broken internal doc links:\n  " + "\n  ".join(errors))


def check_sidebar_refs() -> None:
    sidebars = (ROOT / "sidebars.js").read_text(encoding="utf-8")
    errors = [
        sid
        for sid in re.findall(r'"([^"]+)"', sidebars)
        if "/" in sid and not (ROOT / "docs" / f"{sid}.md").exists()
    ]
    if not (ROOT / "docs" / "intro.md").exists():
        errors.append("intro")
    if errors:
        fail("sidebars.js references missing docs:\n  " + "\n  ".join(errors))


def check_config_doc_links() -> None:
    cfg = (ROOT / "docusaurus.config.js").read_text(encoding="utf-8")
    errors = [
        route
        for route in re.findall(r'to:\s*"(/docs/[^"]+)"', cfg)
        if not (ROOT / "docs" / f"{route[len('/docs/'):]}.md").exists()
    ]
    if errors:
        fail("docusaurus.config.js links to missing docs:\n  " + "\n  ".join(errors))


def main() -> None:
    check_required_docs()
    check_implemented_api_pages()
    check_integration_pages()
    check_stub_status_page()
    check_installer_command()
    check_internal_links()
    check_sidebar_refs()
    check_config_doc_links()
    print("Docs quality checks passed.")


if __name__ == "__main__":
    main()
