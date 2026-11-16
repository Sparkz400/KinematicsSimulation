# Results per dataset

Generated from each dataset's report, `sim-cli fit --dataset datasets/<id> --report <dir>`, by the test `crates/sim-cli/tests/results.rs`; do not edit by hand. To write it anew after a model change:

```text
UPDATE_RESULTS=1 cargo test --release -p sim-cli --test results
```

Torque: the measured motor torque against the simulated one when the recording's position setpoints are replayed, as RMS difference and worst single sample, in N m and as a share of the recording's peak measured torque (on several motors, the motor with the largest RMS share). The fit recording is the dataset's first trace, the one the free values are fitted on; the checks are the others, from which nothing was set. First mode: the resonance of the two-mass model fitted to each measured frequency response against the model's first mode. What each figure means: [docs/validation.md](../docs/validation.md#against-measured-data); how to add a dataset: [docs/measurement-procedure.md](../docs/measurement-procedure.md).

| Dataset | Machine | Data | Licence | Fit recording: RMS / worst | Checks: RMS / worst | First mode, measured → model |
| --- | --- | --- | --- | --- | --- | --- |
| [example-synthetic-belt-axis](example-synthetic-belt-axis/) | `single-axis` | synthetic | CC0-1.0 | trace: 0.00348 N m (0.8 %) / 0.0121 N m (2.9 %) | - | - |
| [isw-ball-screw-axis](isw-ball-screw-axis/) | `screw-axis` | measured | CC-BY-4.0 | s-profile-v300: 0.430 N m (5.3 %) / 2.27 N m (28 %) | sine-a100-v100: 0.786 N m (2.3 %) / 30.0 N m (88 %)<br>constant-speed-v100: 0.895 N m (6.1 %) / 3.60 N m (25 %) | 134 ± 13 → 137 Hz<br>123 ± 10 → 137 Hz<br>126 ± 12 → 137 Hz |
