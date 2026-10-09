#!/usr/bin/env python3
"""Print a compact, reproducible comparison of PID trial JSON files."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def value(data: dict, *path: str) -> float | None:
    current = data
    for key in path:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return float(current) if current is not None else None


def shown(number: float | None, scale: float = 1.0) -> str:
    return "n/a" if number is None else f"{number * scale:.3f}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+", type=Path)
    args = parser.parse_args()
    headers = ("trial", "XY RMS cm", "XY max cm", "Z RMS cm", "Z P-P cm", "yaw P-P deg", "stop max cm", "stop Z rise cm")
    rows = []
    for path in args.paths:
        data = json.loads(path.read_text())
        rows.append((
            data.get("name", path.stem),
            shown(value(data, "hover", "xy_rms_m"), 100),
            shown(value(data, "hover", "xy_max_m"), 100),
            shown(value(data, "hover", "z_rms_m"), 100),
            shown(value(data, "hover", "z_peak_to_peak_m"), 100),
            shown(value(data, "hover", "yaw_peak_to_peak_deg")),
            shown(value(data, "aggregate", "worst_stop_displacement_m"), 100),
            shown(value(data, "aggregate", "worst_z_rise_after_stop_m"), 100),
        ))
    widths = [max(len(headers[i]), *(len(row[i]) for row in rows)) for i in range(len(headers))]
    print(" | ".join(headers[i].ljust(widths[i]) for i in range(len(headers))))
    print("-+-".join("-" * width for width in widths))
    for row in rows:
        print(" | ".join(row[i].ljust(widths[i]) for i in range(len(row))))


if __name__ == "__main__":
    main()
