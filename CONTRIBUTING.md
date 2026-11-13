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
   config or model reference, `docs/validation.md` and `PROGRESS.md` in the same change.

## Before a pull request

```
cargo build --workspace --release --exclude sim-py
cargo nextest run --workspace --release --exclude sim-py
cargo test --doc --workspace --release --exclude sim-py
cargo clippy --workspace --release --all-targets --exclude sim-py
```

The physics tests integrate millions of steps; run them in release. [cargo-nextest](https://nexte.st) (`cargo install
cargo-nextest --locked`) runs the tests of all binaries at once, which takes the suite from about 10 to under 5 minutes
on 16 cores; its settings are in `.config/nextest.toml`. It does not run doctests, hence the third line.
`cargo test --workspace --release --exclude sim-py` still runs everything, one test binary after the other. One
test: `cargo nextest run --release -p sim-machines -E 'test(=every_effect_switch_changes_the_traces)'`. sim-py is
built and tested with maturin (see the CI workflow).

## Licence

By contributing you agree that your contribution is licensed under MIT OR Apache-2.0, as the rest of the project,
without additional terms or conditions.
