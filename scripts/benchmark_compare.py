#!/usr/bin/env python3
"""Combine two benchmark CSVs into a Markdown comparison table.

Usage:
    python3 scripts/benchmark_compare.py this.csv upstream.csv

Each CSV is the output of `kalman_benchmark --csv` (a header row
``filter,ns_per_cycle`` followed by one row per filter). The table compares the
per-cycle time of this fork against the original upstream library and is written
to stdout (ready to pipe into update_readme_benchmark.py).
"""

import csv
import sys


def read(path):
    with open(path, newline="") as fh:
        return {row["filter"]: float(row["ns_per_cycle"])
                for row in csv.DictReader(fh)}


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: benchmark_compare.py this.csv upstream.csv",
              file=sys.stderr)
        return 2

    this = read(sys.argv[1])
    upstream = read(sys.argv[2])

    lines = [
        "| Filter | This fork | Upstream (mherb/kalman) | Speedup |",
        "| :----- | --------: | ----------------------: | ------: |",
    ]
    for name in this:
        t = this[name]
        u = upstream.get(name)
        if u is None:
            lines.append(f"| {name} | {t / 1000:.3f} µs | n/a | n/a |")
            continue
        speedup = u / t if t > 0 else float("nan")
        lines.append(
            f"| {name} | {t / 1000:.3f} µs | {u / 1000:.3f} µs "
            f"| {speedup:.2f}× |"
        )

    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
