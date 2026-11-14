# Open items at the 1.0.0 cut (14 November 2026)

Everything from the fix sessions is merged on main. The last two commits of the app and core-physics fixes
("WIP at the release cut") were merged unverified: run the suites below before trusting them.

## Verify first

- `cargo nextest run --workspace --cargo-profile quick --exclude sim-py` (fast set, about 7 minutes) and
  `--profile full` (all 917+ tests); then the long sweeps (see CONTRIBUTING.md). Before the WIP merges the fast set
  had 6 failures, listed below; the WIP commits aimed at all of them.
- `cargo clippy --workspace --release --all-targets --exclude sim-py -- -D warnings`, `cargo fmt --all --check`.
- Then `python tools/slow_tests.py` to refresh the default filter from the full run's times, and add the app's two new
  tests `the_app_optimises_the_cycle_of_the_machines_slow_to_simulate` and
  `the_app_ranks_the_uncertain_parameters_of_the_machines_slow_to_simulate` to the slow list if they are over 20 s.

## Failing before the WIP merges (fast set)

- `sim-machines::control a_move_arrives_and_comes_to_rest`: parallel SCARA still moves 21.7 µm at rest (limit 20 µm);
  suspected hunting in the backlash of its unloaded arms.
- `sim-machines::control switching_the_model_at_speed_carries_the_motion_over`: slider-crank 8.13 mm off the setpoint
  after the switch (allowed 7.85 + 0.2); check first whether the lag grows without a switch (tool speed varies at
  constant crank speed).
- `sim-opcua::plc a_plc_commanded_move_runs_and_reports_position_and_following_error`: delta-h4 axis 3 following error
  30 mm; its tool turn rings ±34 mrad for 0.5 s after a move.
- `sim-opcua::plc the_plc_disables_and_enables_the_drives`: delta-h4 makes no move after enabling.
- `sim-app::app every_machine_runs_in_the_same_app_model`: slider-crank `quantity-position` counted 2, expected 1.
- `sim-app::app window_code_holds_no_text_for_the_user`: the text "rad/s" in window code.

## Failing in the slow set (last seen before today's fixes; recheck with `--profile full`)

- `sim-machines::suite every_machine_passes_the_standard_suite` and the crank-cam crate's
  `slider_crank_passes_the_standard_suite`: slider-crank energy drift up to 9494 % of 0.0005 J.
- `sim-machines::review2 a_changed_ratio_or_pulley_radius_does_not_tear_the_machine`: parallel SCARA 0.121 mm off
  (limit 0.1 mm).
- `sim-machines::control the_cycle_visits_every_segment_and_switches_the_payload`: following error of turning axes
  against a limit in m (palletiser rz 0.531 mrad vs 0.5, delta-h4 c 11.7 mrad vs 9, spatial cable robot 0.81 mrad vs
  0.5): retune or give rotary axes a limit in rad.
- `sim-cli step_bode_and_tuning_write_csv_for_every_machine` and the long sweep
  `the_shipped_velocity_loops_keep_their_margins_across_the_bounds`: parallel SCARA motor 2 phase margin −298°.
- delta-h4's own crate tests (turn feedforward off by a factor of 94, energy not conserved), reported by the front-end
  fixes; recheck.
- sim-cli `analyze_writes_the_torque_breakdown…`: unaccounted energy 0.2–12 J per machine, seen on a base before the
  core fixes (line module loss); recheck.
- Rotary knife PLC move: once 6.7 mrad peak following error against the 2.4 mrad bound (passed on rerun); its own
  cycle shows 9 mrad: decide the bound or the tuning.

## Follow-ups

- Mount delta-h4, parallel SCARA, crank and cam, hexapod and cable robots on the compliant base (`base::mount`, with
  `frame_momentum`), as the other machines are.
- A machine whose trace does not turn every motor both ways is now accepted with no fit
  (`identify::turns_both_ways`); the delta-h4's 60 ms lifts are shorter than the fit's 80 ms window.
- Orientation rates on a turning slerp in the planner still come from difference quotients (`Geo::at`); the 0.6 jerk
  room hides the noise.
- The FMU state magic stays "state 3" although the snapshot layout changed on 14 November (gearbox hysteresis anchors,
  the arm's last inverse solution); only states of earlier development builds are affected.
- The slowest tests themselves: step_bode, sensitivity and compare in sim-cli loop over the machines one at a time.
- Check the D3/D4 arm-angle cancellation, the PROGRESS lines on today's merges and the app's budget-row counts
  (`line_module` row now with any DC link).
- `docs/validation.md` still gives 417 tests and 13 November: update with the full run's count.
