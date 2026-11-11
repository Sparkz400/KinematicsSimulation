# Progress

Status per phase of "T-Gantry Physics Simulator Project Plan.md". A phase is done when its "Done when"
criterion and its Validation-table checks exist as automated tests and `cargo test --workspace` passes.

| Phase | Status | Tests that prove it |
| --- | --- | --- |
| 1. Base and rigid reference | done | `sim-machines/tests/suite.rs` (round trip, static hold, energy, step-size convergence, determinism, for every registered machine), `sim-machines/tests/architecture.rs` (dependency direction, pinned versions, no machine name outside its crate), `sim-machines/tests/i18n.rs`, constant-torque and Z static-hold tests in the two machine crates |

## Assumptions made for the plan's open questions

- **Reference machine:** a made-up mid-size catalogue-style T-gantry, not a real product. Stroke X 1.0 m,
  Z 0.5 m, pulley radius 20 mm, carriage 4 kg, Z axis 5 kg. All values are placeholders in
  `crates/machine-t-gantry/default.toml`.
- **Payload and cycle time:** payload 2 kg, switchable. No target cycle time; axis limits are X 0.8 m/s,
  8 m/s², Z 0.4 m/s, 6 m/s², jerk 100 m/s³ on both.
- **Motors, gearboxes, drives:** a generic 400 W class servo (rotor 0.3e-4 kg m²) with a 5:1 gearbox.
- **Desktop or browser:** desktop only.
- **PLC link:** the simulator is the OPC UA server; the PLC (or a test client) connects as client.
- **Access to a real axis:** none assumed. Everything that needs measured traces is out of reach.

## Design decisions that differ from or refine the plan

- **Integrator:** classical RK4 with 4 sub-steps per 62.5 µs base step, not a semi-implicit scheme. The plan
  leaves the final choice to the convergence test; RK4 passes it and the energy check, and keeps the core
  free of any knowledge about which states are positions.
- **Registration list:** lives in its own crate `sim-machines`, between the machine crates and the front
  ends. It is the only place outside a machine crate where a machine is named.
- **Motor angles** in the machine interface are motor shaft angles (before the gearbox), because that is
  where the drive's encoder sits. The plan's kinematic formula uses pulley angles; the gear ratio is the
  only difference.
- **Translation files** use the simple-message subset of Fluent (`key = text`, one `{ $arg }` placeholder)
  with a 30-line loader in `sim-core`, not the `fluent` crates. Each crate ships the entries for the keys
  it introduces, so a machine's names stay inside its crate.
- **Energy check:** drift is measured against the energy a torque pulse puts in, not against the
  gravitational potential. That is roughly a thousand times stricter than the plan's wording requires.
- **Phase 0 leftovers:** the phase 0 commit had no translation files, no dependency check and no window.
  The first two are added in phase 1; the window comes with the app in phase 4.
