# Before and right after the 1.0.0 release

What is not built is in [ROADMAP.md](../../ROADMAP.md). This file lists only what is left to check or finish around the
release (state of 16 November 2026).

## Before the tag

- The Python tests (pytest of `crates/sim-py`) were not run locally on the final main; CI's job `python` runs them.
  The final local run of 16 November 2026: 1920 of 1920 tests, the 29 OPC UA tests against the wall clock one at a
  time, the 30 long sweeps, the doctests, clippy and fmt.

## Right after the push and the tag

- Watch the workflows restructured since the cut, none of which has run on GitHub: `ci.yml` (lint, build, partitions,
  sweeps), `pages.yml` (docs site with lychee), `fuzz.yml` (libFuzzer has not run anywhere yet), `release.yml` (MSI,
  .deb, FMUs) and `publish-python.yml` (its macOS arm64 wheel was never built; trusted publishing needs the `pypi`
  environment on GitHub and the publisher entered on PyPI).
- Install the MSI and the .deb once each on a clean machine; neither has been installed.
- Until the package is on PyPI, `pip install kinematics-sim` in `crates/sim-py/README.md` does not work.

## Carried over from the cut, not rechecked

- Orientation rates on a turning slerp in the planner come from difference quotients (`Geo::at`); the 0.6 jerk room
  hides the noise.
- The FMU state's format is still "state 3" although the snapshot layout changed on 14 November and since; only states
  saved by development builds are affected. Decide whether 1.0.0 starts a new number.
- Check the D3/D4 arm-angle cancellation and the app's budget-row counts (`line_module` row with any DC link).
