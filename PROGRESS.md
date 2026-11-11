# Progress

Status per phase of "T-Gantry Physics Simulator Project Plan.md". A phase is done when its "Done when"
criterion and its Validation-table checks exist as automated tests and `cargo test --workspace` passes.

| Phase | Status | Tests that prove it |
| --- | --- | --- |
| 1. Base and rigid reference | done | `sim-machines/tests/suite.rs` (round trip, static hold, energy, step-size convergence, determinism, for every registered machine), `sim-machines/tests/architecture.rs` (dependency direction, pinned versions, no machine name outside its crate), `sim-machines/tests/i18n.rs`, constant-torque and Z static-hold tests in the two machine crates |
| 2. Full mechanics | done | Component unit tests in `sim-components/src/mech.rs` (belt, LuGre friction, shaft friction, ripple, tooth meshing, gearbox); friction levels, backlash, belt natural frequency and rigid limit in `machine-single-axis/src/tests.rs`; rigid limit, static hold and the SciPy comparison in `machine-t-gantry/src/tests.rs`; energy and step-size convergence of the elastic models in `sim-machines/tests/suite.rs` |
| 3. Control and profiles | done | `sim-machines/tests/control.rs` (following error at constant speed = v / Kv on every machine, feedforward, pick-and-place cycle with payload switching, external setpoint interpolation, axis-against-motor limit check, determinism, step-size convergence with control and all effects on, every effect switch changes the traces); profile limits and synchronised moves in `sim-components/src/profile.rs`; drive loops, torque-speed limit, encoder and notch in `sim-components/src/drive.rs`; CSV runs in `sim-cli/tests/cli.rs` |
| 4. App | done (interactive handling not checked by a person, see below) | `sim-app/tests/app.rs`: every machine runs in the same app model with scopes, scene, effect switches and parameters generated from what the machine supplies; every label exists in German and English and the window code holds no text; switching any effect changes the traces within one cycle without a restart, friction within 1 ms; pause, slow motion and real-time pacing of the simulation thread |
| 5. Analysis | done | `reference_cycle_torque_matches_hand_calculation` (peak and RMS torque, cycle time, peak speed and use of the torque-speed curve against a hand calculation), the over-curve check and the two frequency-sweep checks (pure inertia; belt dip at the hand-calculated frequency) in `machine-single-axis/src/tests.rs`; `analyze` and `sweep` CSV export in `sim-cli/tests/cli.rs` |
| 6. PLC link | done against an OPC UA test client; blocked for a real PLC (see below) | `sim-opcua/tests/plc.rs`: a client-commanded move runs in the simulator in real time and reports back position and following error, for every machine; every signal of the signal list is a readable variable; streamed setpoints are interpolated and followed; refused commands are reported |

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
- **Drive placeholders:** Kv 50 1/s, velocity loop Kp 0.08 N m s/rad with Tn 10 ms, current loop tuned
  to 0.25 ms (see below), peak torque 3.8 N m, full torque up to 314 rad/s falling to zero at 420 rad/s, 20-bit
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
- **Speed setpoints are interpolated inside the drive** between profile cycles. Holding them for 1 ms
  made the torque a sawtooth and the peak torque 14 % too high (found by the hand calculation of phase 5).
- **Analysis** measures the second pass of the cycle. Utilisation is torque over the torque the
  torque-speed curve leaves at that speed. The frequency sweep is the open-loop mechanical response
  from one motor's torque to its own speed, on top of the holding torques. All of it is CLI and CSV;
  the app does not show analysis results yet.
- **PLC link:** `sim-opcua` is an OPC UA server (async-opcua 0.19.0 on tokio 1.54.1), endpoint without
  security, anonymous access, default port 4840. Variables are string node ids in the namespace
  `urn:kinematics-simulation`: `signal.<id>` for every signal, `command.target.<axis>`, `command.move`,
  `command.setpoint.<axis>`, `command.cycle`, `status.busy`, `status.refused`. Signals are published
  100 times a second. No security profiles, no user accounts, no methods, no config changes over OPC UA.
- **Phase 0 leftovers:** the phase 0 commit had no translation files, no dependency check and no window.
  The first two are added in phase 1; the window comes with the app in phase 4.

## Added after phase 6 (on request)

| Addition | Tests |
| --- | --- |
| R-L winding with PI current controller, back-EMF and bus voltage limit, replacing the first-order lag | `current_follows_its_setpoint_with_the_tuned_time_constant`, `back_emf_costs_voltage_and_the_bus_limits_torque_at_speed` in `sim-components/src/drive.rs` |
| Corner blending in the cycle, as an option (`effects.corner_blending`, off by default) | `corner_blending_shortens_the_cycle_and_still_reaches_every_dwell`, `blended_default_cycles_stay_within_the_axis_limits` in `sim-machines/tests/control.rs` |
| Tuning help: step response and closed-loop frequency response, in the CLI (`step`, `bode`, `tuning`) and in the app | `step_and_frequency_response_show_the_position_loop_gain`, `too_much_gain_shows_as_overshoot_in_the_step_response` in `machine-single-axis/src/tests.rs`; `tuning_measures_the_axis_with_the_current_gains` in `sim-app/tests/app.rs` |
| 3D view: pulleys, motor shafts and idlers turn, belt teeth travel around the pulleys, live labels with motor speed in rpm (U/min) and axis position, floor and shadows | `the_scene_shows_pulleys_turning_teeth_travelling_and_motor_speeds` in `sim-machines/tests/suite.rs`; `belt_teeth_keep_their_place_on_the_belt_when_the_path_changes` in `sim-core/src/scene.rs` |
| Scopes: setpoints dashed in the same plot as the actual value, one colour per axis or motor, shared time axis and cursor, time window up to 60 s, freeze, min/max/span/RMS per trace, automatic mm and µm, CSV export | `scopes_have_a_window_freeze_statistics_and_export` in `sim-app/tests/app.rs` |

- **Winding placeholders:** torque constant 0.45 N m/A, 4 Ohm, 10 mH, bus limit 190 V at the winding. With
  Kp = L / T = 40 V/A and Tn = L / R = 2.5 ms the current follows its setpoint with T = 0.25 ms, as the
  old lag did. The drive adds the expected back-EMF to its voltage (as real drives do); the
  torque-speed curve of the drive settings is kept as a separate limit.
- **Corner blending** starts the next cycle segment while the current one is still braking and adds the
  two. It applies only where the machine need not stop (no dwell, no gripper action), where no axis
  reverses, and only on the ramps. On lift-traverse-lower cycles every axis stays within its limits;
  for other shapes of cycle the sum of two segments on one axis is not checked against the limits.
- **Tuning measurements** put the test signal on the position setpoint only, without feedforward, so
  they show the feedback loops. The step is 1 mm, so with friction on it also shows stiction.
- **Belt teeth in the 3D view** are drawn every 40 mm for visibility, not at the 5 mm tooth pitch.

## Blocked or not verifiable here

- **Phase 4, a person at the 3D view:** the window was started twice with the built-in screenshot hook
  (`SIM_APP_SCREENSHOT=file.png`, optional `SIM_APP_LANG=de`), and both screenshots were inspected: the
  machine, the panels in English and German and the live scopes render correctly. After the rework of
  the 3D view and the scopes three more screenshots were inspected the same way. Not checked by anyone:
  dragging the orbit camera, the view presets, the parameter and effect panels by mouse, and how the
  motion looks. Those need a person at the screen. The screenshot run is not part of `cargo test`,
  because it opens a window.
- **Phase 6, a real PLC:** no controller is available here. The "PLC" in the test is an OPC UA client
  written with the same library as the server, so interoperability with a Siemens, Beckhoff or CODESYS
  client, their data types, security settings and cycle jitter are untested.
- **Real traces (validation table, phase 7):** needs measured drive traces; not attempted.

## Not built (named in the plan, outside the phase criteria)

- Low and high bounds on friction and damping parameters, and the sensitivity sweep that would use them.
- Parts given by name from a component catalogue instead of raw numbers in the config.
