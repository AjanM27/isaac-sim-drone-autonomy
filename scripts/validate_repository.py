#!/usr/bin/env python3
"""Fast CI checks for the portable public repository."""

from __future__ import annotations

import hashlib
import json
import re
import stat
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "simulation" / "room_quadcopter" / "asset_manifest.json"
FORBIDDEN = (
    "/home/" + "ajanm27",
    "BEGIN OPENSSH " + "PRIVATE KEY",
    "gh" + "p_",
    "github_" + "pat_",
    "@" + "example.com",
)
SKIP_PARTS = {".git", ".venv", ".claude", ".omc", "results", "assets", "__pycache__"}
IRIS_SHA256 = "be6b897304a7743e76ee0575e0f8c5286492b98464cadb846851113609ff3b36"
REQUIRED_FILES = (
    ".dockerignore",
    "README.md",
    "LICENSE",
    "CONTRIBUTING.md",
    "CODE_OF_CONDUCT.md",
    "SECURITY.md",
    "THIRD_PARTY_NOTICES.md",
    "CITATION.cff",
)


def main() -> None:
    for relative in REQUIRED_FILES:
        assert (ROOT / relative).is_file(), f"required repository file missing: {relative}"

    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    assert data["schema"] == 1
    entries = data["entries"]
    assert entries and {entry["profile"] for entry in entries} == {
        "core",
        "warehouse",
        "hospital",
    }
    destinations: set[str] = set()
    for entry in entries:
        destination = entry["destination"]
        assert destination not in destinations, destination
        destinations.add(destination)
        assert not Path(destination).is_absolute()
        assert ".." not in Path(destination).parts
        assert entry["url"].startswith("https://")
        assert int(entry["bytes"]) > 0
        assert len(entry["sha256"]) == 64
        int(entry["sha256"], 16)

    for path in ROOT.rglob("*"):
        if not path.is_file() or SKIP_PARTS.intersection(path.parts):
            continue
        if path.stat().st_size > 2 * 1024 * 1024:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            continue
        for forbidden in FORBIDDEN:
            assert forbidden not in text, f"forbidden private value in {path}: {forbidden}"

        if path.suffix == ".md":
            for link in re.findall(r"(?<!!)\[[^]]+\]\(([^)]+)\)", text):
                target = link.split("#", 1)[0]
                if not target or "://" in target or target.startswith("mailto:"):
                    continue
                assert (path.parent / target).resolve().exists(), (
                    f"broken local Markdown link in {path}: {target}"
                )

        if text.startswith("#!"):
            assert path.stat().st_mode & stat.S_IXUSR, f"script is not executable: {path}"

    iris = ROOT / "simulation" / "room_quadcopter" / "assets" / "iris" / "iris.usd"
    assert iris.is_file(), "project Iris asset is missing"
    iris_digest = hashlib.sha256(iris.read_bytes()).hexdigest()
    assert iris_digest == IRIS_SHA256, f"Iris asset checksum changed: {iris_digest}"
    print(f"Iris SHA-256: {iris_digest}")
    print(f"Validated {len(entries)} checksummed asset records.")


if __name__ == "__main__":
    main()
