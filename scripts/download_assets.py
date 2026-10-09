#!/usr/bin/env python3
"""Download and verify the project-local Isaac Sim asset bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ROOM_ROOT = ROOT / "simulation" / "room_quadcopter"
MANIFEST = ROOM_ROOT / "asset_manifest.json"


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Install checksummed Isaac Sim assets for offline use."
    )
    parser.add_argument(
        "--profile",
        action="append",
        choices=("core", "warehouse", "hospital", "all"),
        help="asset profile; repeat as needed (default: all)",
    )
    parser.add_argument("--verify-only", action="store_true")
    parser.add_argument("--jobs", type=int, default=6)
    return parser.parse_args()


def digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            checksum.update(block)
    return checksum.hexdigest()


def destination(entry: dict[str, object]) -> Path:
    target = (ROOM_ROOT / str(entry["destination"])).resolve()
    if not target.is_relative_to(ROOM_ROOT.resolve()):
        raise ValueError(f"asset path escapes project directory: {target}")
    return target


def valid(entry: dict[str, object]) -> bool:
    target = destination(entry)
    return (
        target.is_file()
        and target.stat().st_size == int(entry["bytes"])
        and digest(target) == entry["sha256"]
    )


def install(entry: dict[str, object]) -> tuple[str, int]:
    target = destination(entry)
    if valid(entry):
        return "cached", int(entry["bytes"])
    target.parent.mkdir(parents=True, exist_ok=True)
    partial = target.with_name(f"{target.name}.part")
    subprocess.run(
        [
            "curl",
            "--fail",
            "--location",
            "--retry",
            "5",
            "--retry-delay",
            "2",
            "--retry-all-errors",
            "--continue-at",
            "-",
            "--silent",
            "--show-error",
            "--output",
            str(partial),
            str(entry["url"]),
        ],
        check=True,
    )
    if partial.stat().st_size != int(entry["bytes"]) or digest(partial) != entry["sha256"]:
        partial.unlink(missing_ok=True)
        raise RuntimeError(f"checksum verification failed: {target}")
    os.replace(partial, target)
    return "downloaded", int(entry["bytes"])


def main() -> int:
    args = arguments()
    if not 1 <= args.jobs <= 16:
        raise SystemExit("--jobs must be between 1 and 16")
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    profiles = set(args.profile or ("all",))
    if "all" in profiles:
        profiles = {"core", "warehouse", "hospital"}
    entries = [entry for entry in manifest["entries"] if entry["profile"] in profiles]
    total_bytes = sum(int(entry["bytes"]) for entry in entries)
    print(
        f"Asset profiles: {', '.join(sorted(profiles))}; "
        f"{len(entries)} files, {total_bytes / 1024**3:.2f} GiB"
    )

    if args.verify_only:
        invalid = [str(entry["destination"]) for entry in entries if not valid(entry)]
        if invalid:
            print(f"Asset verification failed: {len(invalid)} missing or changed file(s)", file=sys.stderr)
            for path in invalid[:20]:
                print(f"  - {path}", file=sys.stderr)
            return 1
        print("Asset verification passed.")
        return 0

    counts = {"cached": 0, "downloaded": 0}
    completed_bytes = 0
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(install, entry): entry for entry in entries}
        for number, future in enumerate(as_completed(futures), 1):
            status, size = future.result()
            counts[status] += 1
            completed_bytes += size
            if status == "downloaded" or number == len(entries) or number % 25 == 0:
                print(
                    f"[{number:>3}/{len(entries)}] {completed_bytes / 1024**3:.2f} GiB checked; "
                    f"downloaded={counts['downloaded']} cached={counts['cached']}",
                    flush=True,
                )
    print("Assets installed and verified for offline use.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
