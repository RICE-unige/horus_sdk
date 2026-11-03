"""Build pinned installer/source artifacts from an existing, version-matched tag."""

import argparse
import ast
import hashlib
import io
import json
from pathlib import Path
import re
import subprocess
import tarfile
import tomllib

from packaging.version import Version


def git(*args):
    return subprocess.check_output(["git", *args], text=True).strip()


def validate(tag):
    if not re.fullmatch(r"v[0-9]+\.[0-9]+\.[0-9]+(?:[-.][A-Za-z0-9.]+)?", tag):
        raise ValueError("Release tag must be vMAJOR.MINOR.PATCH with an optional prerelease suffix")
    commit = git("rev-parse", "--verify", f"refs/tags/{tag}^{{commit}}")
    if git("rev-parse", "HEAD") != commit:
        raise ValueError("Check out the requested release tag before packaging")
    metadata = tomllib.loads(git("show", f"{commit}:pyproject.toml"))["project"]
    brand = ast.parse(git("show", f"{commit}:python/horus/utils/branding.py"))
    branding_version = next(ast.literal_eval(node.value) for node in brand.body
                            if isinstance(node, ast.Assign)
                            and any(isinstance(t, ast.Name) and t.id == "__version__" for t in node.targets))
    version = Version(tag[1:])
    if version != Version(metadata["version"]) or version != Version(branding_version):
        raise ValueError("Tag, pyproject.toml and branding version must agree; no release-time source rewriting")
    return commit, version


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tag")
    parser.add_argument("--output", type=Path, default=Path("release-artifacts"))
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    commit, version = validate(args.tag)
    if args.check:
        print(commit)
        return
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    bootstrap = git("show", f"{commit}:install.sh") + "\n"
    bootstrap = bootstrap.replace('BOOTSTRAP_DEFAULT_REF="main"', f'BOOTSTRAP_DEFAULT_REF="{commit}"')
    bootstrap = bootstrap.replace('BOOTSTRAP_SDK_REF=""', f'BOOTSTRAP_SDK_REF="{commit}"')
    if f'BOOTSTRAP_SDK_REF="{commit}"' not in bootstrap:
        raise ValueError("Bootstrap pin placeholders not found")
    manifest = json.loads(git("show", f"{commit}:install/manifest/releases.json"))
    manifest["channels"]["stable"]["sdk_ref"] = commit
    manifest["channels"]["stable"]["notes"] = f"SDK release {args.tag}; ROS bridge revision pinned independently."
    replacements = {"install.sh": bootstrap.encode(),
                    "install/manifest/releases.json": (json.dumps(manifest, indent=2) + "\n").encode()}
    (output / "installer-install.sh").write_text(bootstrap)
    source = subprocess.check_output(["git", "archive", "--format=tar", commit])
    archive = output / f"horus-sdk-{version}-source.tar.gz"
    with tarfile.open(fileobj=io.BytesIO(source)) as original, tarfile.open(archive, "w:gz") as dest:
        for member in original:
            data = replacements.get(member.name)
            stream = original.extractfile(member) if member.isfile() else None
            if data is not None:
                stream = io.BytesIO(data)
                member.size = len(data)
            member.name = f"horus-sdk-{version}/" + member.name
            dest.addfile(member, stream)
    files = [output / "installer-install.sh", archive]
    sums = "".join(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}\n" for path in files)
    (output / "SHA256SUMS").write_text(sums)
    (output / "installer-install.sh.sha256").write_text(sums.splitlines()[0] + "\n")
    print(f"Packaged {args.tag} ({commit}) in {output}")


if __name__ == "__main__":
    main()
