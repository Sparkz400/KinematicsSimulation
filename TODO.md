# Before and right after the 1.0.0 release

What is not built is in [ROADMAP.md](ROADMAP.md). This file lists only what is left to check or finish around the
release (state of 15 November 2026, main at 27544b4).

## Before the tag

- Full run on the final main: `cargo nextest run --workspace --cargo-profile quick --exclude sim-py --profile full`, the
  long sweeps and the doctests (see CONTRIBUTING.md), clippy, fmt, pytest. The last full run (15 November, at 54a2fca)
  had 1533 of 1538 tests passing, before the third wave; then `python tools/slow_tests.py` to refresh the slow list.
- Fill the two placeholders: the release date in CHANGELOG.md (`RELEASE-DATE`) and the test count and date of that run
  in docs/validation.md (`TEST-COUNT`).
- The path example of the user guide (both languages) still prints the figures of before 15 November (`analyze
  t-gantry --path corner.path`: 0.557940 N m and 1.821e-4 m where the program now gives 0.552014 and 1.268e-4; the
  time-optimal 1.695 s is now 1.658 s); README.md has the current ones.
- Update the GitHub repository's description and topics to the machines it has now.

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
