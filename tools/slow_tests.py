"""Writes the slow tests into .config/nextest.toml: those that took longer than THRESHOLD seconds in a full run's JUnit
report (`cargo nextest run --workspace --cargo-profile quick --exclude sim-py --profile full` writes target/nextest/full/junit.xml).
The default profile leaves them out (`cargo nextest run` takes a few minutes); the profiles `full` and `ci-full` (CI on
a push to main) run them.

    python tools/slow_tests.py [target/nextest/full/junit.xml]
"""
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

THRESHOLD = 20.0
CONFIG = Path(__file__).resolve().parent.parent / ".config" / "nextest.toml"
BEGIN, END = "# The default filter, written by tools/slow_tests.py:", "# End of the default filter."
# What no profile but CI's last step runs (see the comment above the profile `full`).
SWEEPS = (
    "(package(sim-machines) & binary(control) & test(/^across_the_bounds::/))"
    " | test(=the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds)"
)


def slow_tests(junit: Path) -> list[tuple[str, str]]:
    out = set()
    for case in ET.parse(junit).getroot().iter("testcase"):
        if float(case.get("time", "0")) > THRESHOLD:
            out.add((case.get("classname", ""), case.get("name", "")))
    return sorted(out)


def filterset(tests: list[tuple[str, str]]) -> str:
    return "\n| ".join(f"(binary_id(={binary}) & test(={name}))" for binary, name in tests)


def main() -> None:
    junit = Path(sys.argv[1] if len(sys.argv) > 1 else "target/nextest/full/junit.xml")
    tests = slow_tests(junit)
    text = CONFIG.read_text(encoding="utf-8")
    block = f'{BEGIN}\ndefault-filter = """\nnot ({SWEEPS})\n& not (\n{filterset(tests)}\n)\n"""\n{END}'
    new, n = re.subn(re.escape(BEGIN) + r".*?" + re.escape(END), lambda _: block, text, flags=re.S)
    if n != 1:
        sys.exit(f"{CONFIG}: no block between the markers")
    CONFIG.write_text(new, encoding="utf-8", newline="\n")
    print(f"{len(tests)} slow tests (over {THRESHOLD} s)")


if __name__ == "__main__":
    main()
