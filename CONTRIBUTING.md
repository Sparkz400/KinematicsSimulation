# Contributing

Thank you for helping. The simulator is only worth something if its numbers can be trusted, so the house rules
below are about evidence first and code second.

## House rules

1. **Every physics change has a hand calculation as its test.** Work the expected value out by hand (or with an
   independent tool such as the SciPy reference in `crates/machine-t-gantry/reference/`), write the numbers and the
   formula into the test's comment, and compare the simulation against them with a stated tolerance. A test that
   only checks "it runs" or compares the simulator with itself is not enough.
2. **Every fix is reviewed by experiment.** Before a fix counts, the test that covers it must be seen to fail with
   the fix taken out (take the change out by hand, run the test, put it back). Say in the pull request that you did.
   Reviews work the same way: find a defect by running something, not by reading alone.
3. **No text for the user in code.** Every label, signal name and message has a key with a German *and* an English
   entry in the crate's `i18n/en.ftl` and `i18n/de.ftl`. The tests fail when one is missing.
4. **Values belong in configs.** Machine parameters live in the machine's TOML config, with units in the key names,
   each marked `[catalogue]` or `[estimate]`; an estimate gets a low and a high value in `[bounds]`. No magic
   numbers for a machine in Rust code.
5. **No machine name outside its own crate**, except the registration list in `sim-machines`. Front ends and
   components work from what the machine supplies (signal list, axes, scene). `sim-machines/tests/architecture.rs`
   checks this and the direction of dependencies (a crate depends only on crates below it).
6. **Dependencies are pinned with `=`** in the root `Cargo.toml` and upgraded deliberately. After adding or upgrading
   one, run `python tools/third_party_licenses.py` and commit the new `THIRD-PARTY-LICENSES.md`; it fails on GPL,
   LGPL, AGPL or an unknown licence.
7. **Each effect has its own switch** (`effects.<name>`), so its contribution can be seen alone.
8. **Docs move with the numbers.** When a shipped figure changes, update the user guide (both languages), the
   config or model reference, `docs/validation.md` and `docs/development/PROGRESS.md` in the same change.

## Before a pull request

```
cargo nextest run --workspace --cargo-profile quick --exclude sim-py
cargo test --doc --workspace --release --exclude sim-py
cargo clippy --workspace --release --all-targets --exclude sim-py -- -D warnings
cargo fmt --all --check
```

Code is formatted with rustfmt (`rustfmt.toml`: 120 characters a line); CI refuses unformatted code and any clippy
warning.

The physics tests integrate millions of steps, so they run optimised. The build profile `quick` (root `Cargo.toml`) is
release built incrementally: after a change only what changed is compiled again. [cargo-nextest](https://nexte.st)
(`cargo install cargo-nextest --locked`) runs the tests of all binaries at once; its settings are in
`.config/nextest.toml`. It does not run doctests, hence the second line.

Its default profile runs the fast tests: it leaves out every test that takes over 20 s, written into its filter by
`tools/slow_tests.py` from the timings of a full run. A pull request runs those. A push to main runs all of them (the
profile `full` locally, `ci-full` in CI). The long sweeps run weekly and by hand (Actions, CI, "Run workflow"), not on
a push, as they take hours: the velocity loops' margins across the bounds (`across_the_bounds::*` in the control tests
of sim-machines) and the EXCT-30's search for the worst corner of its bounds, which no profile runs by itself. The
filter names each slow test exactly, so a new or renamed test (or a check split into one test per machine,
`<check>::machine_06`) runs in the default profile until the list is written again.

```
cargo nextest run --workspace --cargo-profile quick --exclude sim-py --profile full
python tools/slow_tests.py
cargo nextest run --workspace --cargo-profile quick --exclude sim-py --ignore-default-filter -E "(package(sim-machines) & binary(control) & test(/^across_the_bounds::/)) | test(=the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds)" --test-threads 1
```

The first runs every test and writes their times to `target/nextest/full/junit.xml`; the second puts the ones over
20 s into the default filter again; the third runs the sweeps. One test:
`cargo nextest run --cargo-profile quick -p sim-machines -E 'test(=every_effect_switch_changes_the_traces)'`. sim-py
is built and tested with maturin (see the CI workflow).

### CI

`.github/workflows/ci.yml` runs on every pull request and every push to main, each check a job of its own, so that a
red one hides none of the others; Windows and Linux each:

- `lint`: `cargo fmt --check` and clippy, without waiting for any test; red in the first minutes.
- `build`: every test binary, built once and packed with `cargo nextest archive`.
- `test`: the archive run in partitions, each on a runner of its own (`--partition count:k/n`, n = 2 on a pull request,
  6 on main), with the profile `ci` (fast tests) on a pull request and `ci-full` (all) on main. On main every partition
  keeps its `junit.xml` as an artifact `junit-<os>-<k>`; `python tools/slow_tests.py junit-*/junit.xml` writes the slow
  list from them as from a local full run (CI's runners have 4 cores, so the times are longer than on 16 threads).
- `plc`: the OPC UA tests that run against the wall clock, one at a time.
- `sweeps` (weekly and by hand, not on a push): the long sweeps in fifteen groups (the slowest machines each alone), the
  last taking every sweep the others do not name; a group may take up to GitHub's 6 hours.
- `doctests`, `fmu` (the FMU check with fmpy), `python` (wheel, clippy of sim-py, pytest with one worker per core,
  CAD mass tests), `deny`, `wasm`.

The jobs that run the archive wait for `build`, but run even when the other platform's build failed.

`.github/workflows/bench.yml` runs the Criterion benchmarks of `crates/sim-machines/benches` (one step and one pass of
the default cycle, per machine; not the analysis, minutes a machine) on every push to main and by hand, some 20 minutes.
It compares each median with the last run on main (`tools/bench_compare.py`, the baseline kept in the Actions cache) and
warns, in the job summary, about every bench more than 15 % slower whose 95 % interval lies wholly above the baseline's.
It never fails: shared runners differ by 10 to 20 % between runs. Since each run on main becomes the next baseline, a
slowdown in steps of under 15 % passes unwarned; compare against an older commit by hand on a quiet machine (`cargo
bench -p sim-machines --bench simulate -- --save-baseline before`, then `-- --baseline before` after the change).

### Not on crates.io

The product ships as binaries, an MSI, a .deb, a macOS tarball, FMUs and Python wheels (`release.yml`,
`publish-python.yml`), not as Rust libraries: their interfaces change with every phase and are not kept stable for
outside users. So every crate has `publish = false` in its `Cargo.toml`, and `cargo publish` refuses. A new crate needs
the same line; the job `lint` fails on a crate without it.

## Licence

By contributing you agree that your contribution is licensed under MIT OR Apache-2.0, as the rest of the project,
without additional terms or conditions.
