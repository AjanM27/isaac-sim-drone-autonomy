#!/usr/bin/env python3
"""Fast CI checks for the portable public repository."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "simulation" / "room_quadcopter" / "asset_manifest.json"
FORBIDDEN = (
    "/home/" + "ajanm27",
    "BEGIN OPENSSH " + "PRIVATE KEY",
    "gh" + "p_",
    "github_" + "pat_",
)
SKIP_PARTS = {".git", ".venv", ".claude", ".omc", "results", "assets", "__pycache__"}


def main() -> None:
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

    iris = ROOT / "simulation" / "room_quadcopter" / "assets" / "iris" / "iris.usd"
    if iris.is_file():
        print(f"Iris SHA-256: {hashlib.sha256(iris.read_bytes()).hexdigest()}")
    print(f"Validated {len(entries)} checksummed asset records.")


if __name__ == "__main__":
    main()
