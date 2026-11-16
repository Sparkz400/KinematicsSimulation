"""Compares the Criterion benchmarks of the last run (`<group>/<bench>/new/estimates.json` under target/criterion)
with a stored baseline (`<group>/<bench>/estimates.json` under bench-baseline, the last run on main, which the
workflow bench.yml keeps in the Actions cache) and warns about each bench that got slower. With --save the run's
estimates then become the baseline.

A bench counts as slower when its median is more than LIMIT above the baseline's median AND the two medians' 95 %
confidence intervals do not overlap: shared runners differ by 10 to 20 % from run to run, and the per-step benches
vary with the phase of the cycle, so the size of the change alone warns too often. It only warns (a GitHub
annotation and a table in the job summary), it never fails: a regression on a noisy runner is a hint to measure on a
quiet machine, not proof.

    python tools/bench_compare.py [--criterion target/criterion] [--baseline bench-baseline] [--limit 0.15] [--save]
    python tools/bench_compare.py --self-test
"""

import argparse
import json
import os
import shutil
import sys
from pathlib import Path


def slower(base, new, limit):
    """The relative change of the median when the bench got slower by more than `limit`, else None."""
    b, n = base["median"], new["median"]
    change = n["point_estimate"] / b["point_estimate"] - 1.0
    apart = n["confidence_interval"]["lower_bound"] > b["confidence_interval"]["upper_bound"]
    return change if change > limit and apart else None


def self_test():
    def est(median, half):
        return {"median": {"point_estimate": median, "confidence_interval": {"lower_bound": median - half,
                                                                              "upper_bound": median + half}}}
    # 20 % slower, intervals 100+-2 and 120+-2 apart: warns.
    assert abs(slower(est(100, 2), est(120, 2), 0.15) - 0.2) < 1e-12
    # 20 % slower but the intervals overlap (100+-15 up to 115, 120+-10 down to 110): noise, no warning.
    assert slower(est(100, 15), est(120, 10), 0.15) is None
    # 10 % slower, intervals apart: under the limit.
    assert slower(est(100, 1), est(110, 1), 0.15) is None
    # Faster: never.
    assert slower(est(100, 1), est(50, 1), 0.15) is None
    print("self-test passed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--criterion", default="target/criterion")
    parser.add_argument("--baseline", default="bench-baseline")
    parser.add_argument("--save", action="store_true", help="store this run's estimates as the baseline afterwards")
    parser.add_argument("--limit", type=float, default=0.15)
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return

    rows, warnings, missing = [], 0, 0
    news = sorted(Path(args.criterion).glob("*/*/new/estimates.json"))
    for new_path in news:
        bench = new_path.parent.parent
        name = f"{bench.parent.name}/{bench.name}"
        base_path = Path(args.baseline) / name / "estimates.json"
        new = json.loads(new_path.read_text())
        if not base_path.exists():
            missing += 1
            continue
        base = json.loads(base_path.read_text())
        change = new["median"]["point_estimate"] / base["median"]["point_estimate"] - 1.0
        flag = slower(base, new, args.limit)
        if flag is not None:
            warnings += 1
            print(f"::warning title=Bench slower::{name}: median {flag:+.0%} against the baseline from main")
        rows.append((name, base["median"]["point_estimate"], new["median"]["point_estimate"], change, flag))

    lines = [f"Benches: {len(rows)} compared, {warnings} slower by more than {args.limit:.0%} (intervals apart), "
             f"{missing} without a baseline.", "", "| bench | baseline median | median | change | |",
             "|---|---:|---:|---:|---|"]
    for name, b, n, change, flag in rows:
        lines.append(f"| {name} | {b / 1e3:.1f} µs | {n / 1e3:.1f} µs | {change:+.1%} | {'slower' if flag else ''} |")
    text = "\n".join(lines)
    print(text)
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a", encoding="utf-8") as summary:
            summary.write(text + "\n")
    if args.save:
        for new_path in news:
            bench = new_path.parent.parent
            target = Path(args.baseline) / bench.parent.name / bench.name
            target.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(new_path, target / "estimates.json")


if __name__ == "__main__":
    sys.exit(main())
