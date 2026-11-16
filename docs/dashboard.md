# Validation dashboard and API docs

Two pages beside this book on the project site, both made by the Pages workflow (`.github/workflows/pages.yml`) on every
push to main:

- **[Validation dashboard](https://sparkz400.github.io/KinematicsSimulation/validation/)**: every check of
  [Validation](validation.md) in one table, with the outcome of the test that makes it in that run, a link to the test's
  source line, and, where the test records them, the reference value, the simulated value, the difference allowed and
  the margin left (the share of the tolerance not used, of the worst sample).
- **[API docs](https://sparkz400.github.io/KinematicsSimulation/api/)**: the rustdoc of every crate of the workspace
  (`cargo doc --no-deps --workspace`).

## How the dashboard is made

1. `python tools/validation_dashboard.py --nextest-filter` reads the tables of `docs/validation.md` and prints a nextest
   filter of every Rust test they name.
2. Those tests run with `KS_VALIDATION_OUT` set to a file. A test that calls `sim_core::testkit::record_check(what,
   reference, simulated, allowed)` appends one JSON line to it:
   `{"test", "file", "line", "what", "reference", "simulated", "allowed"}`. The call asserts nothing; the test's own
   assert still decides. Without the variable it does nothing.
3. `python tools/validation_dashboard.py --junit target/nextest/full/junit.xml --records <file> --out <folder>` writes
   `index.html`: a row per table row of `validation.md`, its status from the JUnit file (pass when every test the row
   names passed), the recorded values of those tests under it (the worst sample of each value).

To build it locally from the tests of one crate:

```bash
KS_VALIDATION_OUT=$PWD/target/validation.jsonl cargo nextest run --cargo-profile quick --profile full -p machine-delta-d3
python tools/validation_dashboard.py --junit target/nextest/full/junit.xml --records target/validation.jsonl
```

The page is then `target/validation/index.html`; checks of tests that did not run show as "not run".

## What it does not show

- Values for most checks: only the tests that call `record_check` write them (the delta pickers' and the wafer handler's
  hand calculations, the references against SymPy, SciPy, MuJoCo and the Python drive, the ISW bench's inertia and
  mass). The other rows show the test's outcome and the reference and tolerance as `validation.md` states them.
- The Python tests (pytest): named in the tables and linked, not run by the Pages workflow ("not run").
- Rows without a test (the tables of figures: the shipped configs' feedforward, the ISW recordings) are shown as figures,
  with no status.
- A test name defined in two crates gets the outcomes of both together.
