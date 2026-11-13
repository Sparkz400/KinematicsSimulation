## What and why

<!-- What this changes and the reason. Link the issue if there is one. -->

## Checklist (see CONTRIBUTING.md)

- [ ] Physics changes have a hand calculation as their test, with the numbers and formula in the test's comment
- [ ] I took the fix out, saw its test fail, and put it back
- [ ] New user-facing text has keys in both `i18n/en.ftl` and `i18n/de.ftl`
- [ ] Machine values are in the config, marked `[catalogue]` or `[estimate]` (estimates with `[bounds]`)
- [ ] Docs updated in both languages if a shipped figure changed
- [ ] `cargo test --workspace --release` and `cargo clippy --workspace --release --all-targets` pass
