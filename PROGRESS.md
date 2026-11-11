# Progress

Status per phase of "T-Gantry Physics Simulator Project Plan.md". A phase is done when its "Done when"
criterion and its Validation-table checks exist as automated tests and `cargo test --workspace` passes.

| Phase | Status | Tests that prove it |
| --- | --- | --- |
| 1. Base and rigid reference | done | `sim-machines/tests/suite.rs` (round trip, static hold, energy, step-size convergence, determinism, for every registered machine), `sim-machines/tests/architecture.rs` (dependency direction, pinned versions, no machine name outside its crate), `sim-machines/tests/i18n.rs`, constant-torque and Z static-hold tests in the two machine crates |
| 2. Full mechanics | done | Component unit tests in `sim-components/src/mech.rs` (belt, LuGre friction, shaft friction, ripple, tooth meshing, gearbox); friction levels, backlash, belt natural frequency and rigid limit in `machine-single-axis/src/tests.rs`; rigid limit, static hold and the SciPy comparison in `machine-t-gantry/src/tests.rs`; energy and step-size convergence of the elastic models and `every_effect_switch_changes_the_response` in `sim-machines/tests/suite.rs` |
| 3. Control and profiles | done | `sim-machines/tests/control.rs` (following error at constant speed = v / Kv on every machine, feedforward, pick-and-place cycle with payload switching, external setpoint interpolation, axis-against-motor limit check, determinism, step-size convergence with control and all effects on, every effect switch changes the traces); profile limits and synchronised moves in `sim-components/src/profile.rs`; drive loops, torque-speed limit, encoder and notch in `sim-components/src/drive.rs`; CSV runs in `sim-cli/tests/cli.rs` |
| 4. App | done (interactive handling not checked by a person, see below) | `sim-app/tests/app.rs`: every machine runs in the same app model with scopes, scene, effect switches and parameters generated from what the machine supplies; every label exists in German and English and the window code holds no text; switching any effect changes the traces within one cycle without a restart, friction within 1 ms; pause, slow motion and real-time pacing of the simulation thread |

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
- **Belt sections, not spans:** idlers are free, so the tension is uniform between two points that hold
  the belt (a motor pulley or a clamp). The T-gantry has three such sections, each one spring with
  k = EA / L where L changes with x and z. The single spans appear only in the 3D scene.
- **Energy-consistent belt:** the pulleys feel the tension times (1 + strain/2), because stretched belt
  carries its strain energy along. The factor is about 1.0003 and makes the energy check exact. The
  elastic holding torque therefore differs from m_z g r / 2 by less than 0.1 %; the rigid one is exact.
- **Pulley radius holds at pretension,** so x = r/2 (a + b) is exact for an evenly tensioned belt.
- **Guide load for load-dependent friction** uses the X acceleration that the commanded motor torques
  would give a loss-free rigid machine. The true acceleration would make friction depend on itself, and
  its absolute value turned belt vibration into extra friction (found by the rigid-limit check).
- **`effects.belt_stretch = false` selects the rigid reference model.** Belt damping, slack, backlash,
  gear stiffness and the frame mode need the elastic model and have no effect in the rigid one.
- **Belt mass** is lumped on the pulleys (half per pulley on the T-gantry), not spread over the load.
- **Gearbox efficiency** fades out below 0.5 W of transmitted power, so a gearbox at rest has no torque step.
  Motor shaft Coulomb friction is smoothed with tanh for the same reason.
- **SciPy reference:** `crates/machine-t-gantry/reference/scipy_reference.py` re-implements the elastic
  T-gantry and integrates it with LSODA; the trace is committed as `scipy_trace.csv`, so `cargo test`
  needs no Python. Regenerate it when the model or the default config changes (venv in `.venv`, pinned
  in `reference/requirements.txt`). Agreement is within 0.2 % of the travel.
- **Placeholder mechanics:** belt EA 5e5 N, pretension 300 N, gearbox backlash 3 arcmin, friction and
  damping values are estimates. See the two `default.toml` files.
- **Drive placeholders:** Kv 50 1/s, velocity loop Kp 0.08 N m s/rad with Tn 10 ms, current loop as a
  0.25 ms lag, peak torque 3.8 N m, full torque up to 314 rad/s falling to zero at 420 rad/s, 20-bit
  encoder. Both motors use the same drive settings. Untuned for any real machine.
- **Feedforward model:** torque feedforward is the rigid inverse dynamics the machine supplies (inertia
  and gravity). Friction is not in it, so some following error stays while accelerating.
- **Stop** finishes the segment under way and then stays; there is no fast stop ramp.
- **Cycle** is a list of segments in the machine's config (target, dwell, gripper state on arrival);
  the parameter panel and `--set` cannot edit it, a config file can.
- **External setpoints** (for the PLC link) are interpolated linearly over `profile.setpoint_interval_s`.
- **App versions:** Bevy 0.20.1, bevy_egui 0.43.1 (egui 0.36) and egui_plot 0.37.0, the plot release
  that shares egui 0.36. Desktop only.
- **App structure:** `sim-app` is a library (simulation thread, interface model; no drawing, fully
  tested) plus the window (`main.rs`, `ui.rs`). The simulation thread sends samples through a channel;
  the interface keeps the last 10 s as the scope history.
- **"Within one cycle"** is read as one pass of the pick-and-place cycle. Effects that act all the time
  are also checked within one position-loop cycle (1 ms).
- **Scopes:** one plot per quantity (position, following error, torque, ...) with one line per axis or
  motor. Belt stretch is shown as a colour shift of the belt spans (red above pretension, blue below).
- **The controller's setpoint starts at the motor position,** a few micrometres from the configured
  start pose (belt stretch and gear twist under load), so the first step has no jolt.
- **Phase 0 leftovers:** the phase 0 commit had no translation files, no dependency check and no window.
  The first two are added in phase 1; the window comes with the app in phase 4.

## Blocked or not verifiable here

- **Phase 4, a person at the 3D view:** the window was started twice with the built-in screenshot hook
  (`SIM_APP_SCREENSHOT=file.png`, optional `SIM_APP_LANG=de`), and both screenshots were inspected: the
  machine, the panels in English and German and the live scopes render correctly. Not checked by anyone:
  dragging the orbit camera, the view presets, the parameter and effect panels by mouse, and how the
  motion looks. Those need a person at the screen. The screenshot run is not part of `cargo test`,
  because it opens a window.

## Not built (named in the plan, outside the phase criteria)

- Low and high bounds on friction and damping parameters, and the sensitivity sweep that would use them.
- Parts given by name from a component catalogue instead of raw numbers in the config.
