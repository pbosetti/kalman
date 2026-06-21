#!/usr/bin/env python3
"""Inject a benchmark results table into README.md.

Usage:
    kalman_benchmark --markdown | python3 scripts/update_readme_benchmark.py
    python3 scripts/update_readme_benchmark.py results.md

The Markdown table (read from a file argument or from stdin) replaces whatever
sits between the two marker comments in README.md:

    <!-- BENCHMARK:START -->
    ... generated table ...
    <!-- BENCHMARK:END -->

The script is idempotent and exits non-zero if the markers are missing.
"""

import datetime
import pathlib
import sys

START = "<!-- BENCHMARK:START -->"
END = "<!-- BENCHMARK:END -->"

README = pathlib.Path(__file__).resolve().parent.parent / "README.md"


def main() -> int:
    table = (
        pathlib.Path(sys.argv[1]).read_text()
        if len(sys.argv) > 1
        else sys.stdin.read()
    ).strip()

    text = README.read_text()
    if START not in text or END not in text:
        print(f"error: markers {START} / {END} not found in {README}",
              file=sys.stderr)
        return 1

    stamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%d %H:%M UTC")
    block = (
        f"{START}\n"
        f"_Last updated: {stamp}. Indicative numbers from the CI runner; "
        f"absolute values vary with hardware._\n\n"
        f"{table}\n"
        f"{END}"
    )

    before, rest = text.split(START, 1)
    _, after = rest.split(END, 1)
    README.write_text(before + block + after)
    print(f"Updated benchmark table in {README}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
