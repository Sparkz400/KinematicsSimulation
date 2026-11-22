# What is estimated, chosen, left out or unverified

State: 17 November 2026, after the physics audit (about 95 findings), its fixes, and the stage that
modelled the effects the code had only named in `shortcut:` comments (133 comments, 24 left). This file
lists everything that is **not taken from reality or not derived**: values without a source, rules
somebody chose, what the models still leave out and how large that is, and what no run has checked.
It replaces the list of 16 November.

Words used: **estimate** (a value with no source, chosen to be plausible), **chosen** (a rule that is
defensible but not derived from physics or a standard), **left out** (the model knowingly omits it;
the size is given where it was measured or worked out by hand), **unverified** (written, not run or
not checked against the source).

Vendor facts with document and page are in [vendor-facts.md](vendor-facts.md).

## 1. What code cannot close

These need data that no vendor publishes, or a measurement on hardware.

| What | Where | Why it stays |
| --- | --- | --- |
| No measured trace of one of this project's own machines | `docs/validation.md` | The only measured data is the external ISW ball-screw set under `datasets/`. Everything else in "validation" compares the program with a hand calculation, an independent script or itself. |
| S3 duty line in the torque-speed chart | `charts.rs` | No catalogue the simulator reads prints S3 data. |
| Standstill values of Yaskawa and Mitsubishi motors, rated torque of SEW CMP | `catalog.rs` | The makers print only one of the two; the other stands in (overstates what the motor carries at speed by a few per cent). |
| Winding resistance of motors whose maker prints none; base-load current of drives | `catalog.rs` | Not published; an estimate from the braking table, and the S210's share of the rated current, stand in. |
| Winding resistance and inductance of the Siemens 1FK2 / 1FT2 in the shipped configs (10 mH) | shipped configs | Siemens does not print them in catalogue D 32; naming the motor gives another estimate (28.9 mH for the 1FK2105-4AF, from the braking table). Closing it: the per-motor data sheet from Siemens' data-sheet service. |
| The split of a motor's iron loss into eddy and hysteresis parts (one exponent 1.5 for both) | `drive.rs` | No maker prints iron loss over speed. |
| The magnets' temperature (eddy heating of the rotor) | `drive.rs` | No maker gives the rotor's loss. |
| The shape of a gear's hysteresis loop at a reversal, its presliding stiffness | `mech.rs` | Not published; both default to the gear's stiffness K1 (keys `gearbox.hysteresis_stiffness_factor`, `gearbox.mesh_stiffness_factor`). |
| Motor-specific adapter plates and shaft lengths for a named axis | `catalog/mechanics.rs` | Needs the makers' configurators. |
| Bearing life factor below κ = 0.1 | `analysis.rs` | ISO 281 gives no factor there; 0.1 is taken. |
| The Beckhoff XTS mover's centre of mass | `machine-mover-track` (`mass.centre_offset_m` = 0) | Not published; needs the vendor's CAD or load diagrams. Mass 651 g is sourced. |
| What a drive does with its setpoint on a late fieldbus frame | `control.rs` | No primary text found (SINAMICS documents only the tolerated number of missed sign-of-life counts). The model extrapolates the old frame. |
| Fanuc's feed rule for a block with rotary axes; a stepper's current convention | `program.rs`, `stepper.rs` | No primary source found; the catalogue has no stepper vendor. |
| The 1FT7085-7WF7's data sheet (the measured dataset's motor) | `datasets/isw-ball-screw-axis` | Siemens' service did not answer on 17 November; the values rest on the repository's earlier fetch and an independent data sheet copy of the same motor size. |
| RAPID's `v_ori` in speeddata and KRL's `$VEL.ORI1` | `program.rs` | Not read; the config key `program.orientation_speed_rad_s` (500 °/s, ABB's default) is used for every move. |

## 2. Estimates in the shipped files

Every shipped machine is a generic machine: its proportions are made up unless a part is named from the
catalogue. The configs mark each such value with `# [estimate]` (about 2,400 marks). The ones that
matter most, and the ones added in November 2026:

| What | Value | From |
| --- | --- | --- |
| Placeholder motor in the single axis, the T-gantry default and conveyor configs and 12 machine files | 0.45 N m/A | No maker's motor; marked in the config comment. |
| Spatial cable robot's settling tolerance for turns | 2 mrad (0.28 mm at the platform's corners) | A requirement, not physics. The backlash floor is about 0.25 to 0.6 mrad; the default rule's 0.2 mrad cannot be held with this gearbox. |
| Cable mass per metre | 3 g/m (planar), 10 g/m (spatial) | Rope diameters 2 mm and 4 mm. |
| Hexapod and tripod leg parts (mass, centre, inertia) | rod 0.35 m bar, body 0.5 m bar, m l²/12 | Bar model of the actuator and tube sizes in the config. |
| H4 upper-arm tube | carbon 60 × 2.5 mm, 100 GPa; bending damping 2 % of critical; elbow twist inertia 2.9e-4 kg m² | The D3's tube scaled; the H4 config had no tube. |
| D4 tool gearbox output inertia | 1e-4 kg m² | Steel disc 50 × 15 mm (7.2e-5) plus the carrier's share. |
| Five-bar arm tube | 1.2 kg, centre 0.2 m | The uniform tube with the config's inertia. |
| Crane's gains: velocity gain 0.15 N m s/rad for bridge and trolley, integral time 0.1 s, position gain 10 1/s on all three drives | — | Chosen from measured sweeps so the loops are stable with the elastic drives on and off: the bridge's rotor drives 38 times its own inertia through the gearbox, and the old gain of 2.0 crossed at about 2.4 kHz on the rotor alone and ran away. Lowest phase margin across the bounds without friction 45.7° (40.3° at a position gain of 15, 28° at 30). The rigid bridge loop now crosses at 4.5 Hz; settling is 0.57 mm (was 0.036) against a 50 mm limit. |
| Five-bar tool turn's velocity gain | 0.033 N m s/rad | Chosen for the loop's margin with the rotor inside the gear's play and no friction: 6.95 dB and 66.6° (18.8 dB with friction). At 0.0443 the margin without friction was 4.0 dB; 0.08 hammers across the play (`sim-cli margins` says so since 21 November 2026: 650 Hz, 0.77 N m). |
| Linear delta belt | 1500 J/(kg K), 15 W/(m² K), 60 K allowed rise | Polyurethane with steel cords; a moving belt in still air. |
| Linear delta rod-joint offsets from the slide | 0 | Festo's moment ratings are not shipped; the guide moments are therefore not judged. |
| T-gantry beam | 3.5 kg/m, damping 1 % | A light 80 × 80 mm aluminium profile, a bolted structure. |
| Cam indexer's cam radius | 0.10 m | Gives a pressure angle of 35° at the fastest point (peak contact force +2.1 %). |
| Disc cam follower: roller width, contact damping, guide friction | 14 mm, 0.5, 5 N | A KR 30 track roller. |
| Crane: hook block, sling, gauge, wheelbase, wheel lateral stiffness, skew damping, floor friction, floor height | 15 kg, 1 m, 5.6 m, 2 m, 2e6 N/m, 5 %, 0.3, 0.5 m | A crane of this load class. Rope mass 0.126 kg/m is sourced (EN 12385-4 tables: 0.125 to 0.129 at 6 mm). |
| Gantries' yaw inertia including the belt's share | 2.657 and 1.339 kg m² | The belt loop taken symmetric about the bridge centre, as the files' spans have it since 21 November (strands of 1.0 and 0.7 m). |
| Rotary hexapod's clearance of rods to platform and base | 0.02 m | Rod radius plus the plate's half thickness. |
| Joint kit: the worm rotor's axis on the rotary table | along x | No layout given. |
| Pre-existing: tooth-jump factor, brake deflection and slip times, the efficiency fade's 0.5 W | `mech.rs`, configs | Never judged against a source; measure or mark. |

## 3. Rules somebody chose

| Rule | Where | What it costs or assumes |
| --- | --- | --- |
| Retuned velocity gain keeps the config's crossover over the rotor-side inertia; a linear motor by the whole moving mass | `catalog.rs` (`bounded_gain`, `retuned`) | Fitted on a sweep of 12 cases; knows inertias only. Motors with an inertia ratio above about 25 (EXCT-30, 360 candidates swept) get no stable gain and are rejected by the verdicts. The integral time and position gain did not follow the gain until the fix of 17 November (see section 8). |
| Iron loss: the shaft's drag is the printed gap k_t·I_N − M_N less the motor's friction; the winding's heat is the thermal split R(I0² − I_N²); both scale with (n/n_N)^1.5 | `drive.rs` | Backed by the printed efficiencies of 144 1FK7/1FT7 rows (the gap is a median 2.9 times the thermal split). The friction subtracted is the config's estimate. |
| The field-weakening d current uses the unsaturated back-EMF | `drive.rs` | With the saturated one braking beyond the curve ran away; costs 3.6 % torque at 380 rad/s on the test motor. |
| The current controller's back-EMF feedforward uses the cold magnets' flux; the winding's own back-EMF follows the magnet temperature | `drive.rs` (`current_control`) | A drive does not measure its rotor's temperature. With `effects.magnet_temperature` on, the current integrator makes up the difference (the flux change, about 0.1 % per kelvin, of the back-EMF). The field-weakening d current does use the warm flux: an idealised controller there. |
| Without `motor.stall_current_a` the torque-speed chart draws the rated torque as a flat S1 line and no `motor-s1` verdict is given | `drive.rs` (`s1_torque`), `analysis.rs` | The line is on the safe side below the rated speed; the motor is then judged by its thermal model alone. |
| The loop measurement steps below 10 Hz, at the same spacing, until the loop gain exceeds 1, to 1 Hz at the lowest (`LOOP_FLOOR_HZ`) | `analysis.rs` | The floor is chosen. The old 10 Hz floor hid a hoist loop crossing at about 8 Hz with 29° to 31° at the bounds. A test that asked every machine for 4 Hz of position-loop band now asks for the smaller of 4 Hz and 0.8·Kv/2π: the crane has 1.6 to 1.9 Hz. |
| Overmodulation carries the fundamental only (2√3/π three-phase, 4/π stepper bridges) | `drive.rs` | No harmonic loss. |
| S1 curve of a Siemens S210 motor: the vendor's graph, scaled to the printed table when within 4 %; beyond its last point the last torque | `catalog/siemens/s1_curves_s210.csv`, `drive.rs` | 74 curves as read, 14 scaled by 0.985 to 1.036, 8 motors keep the straight line. |
| A duty mix runs each duty in the order given, repeated; S1 and the thermal figure go through the motor's one node | `search.rs` | In the middle regime the order matters. Without `ratings.drive_overload_time_constant_s` the drive current of a mix is the time-weighted RMS. |
| Planner shares: 0.3 of the jerk for the binormal; ½ of the acceleration limit for a slerp's bend; 0.8 torque and tension reserve; 1 % of the speed kept back at least (`speed_kept_of`); 32 grid steps for the tops; interpolation allowance 0.0851·J·h (C2) or 2/3·J·h (Hermite) | `path.rs` | A torsioned corner runs 9 % slower than a planar one; the tension reserve costs 1.9 % (planar) and 5.6 % (spatial) of the cable robots' time-optimal cycle; a fitting range narrower than 1/32 of the span can be missed. |
| Friction is not counted as helping while braking | `path.rs` | Conservative. |
| Fit statistics: `PER_AXIS_SPREAD` 0.05, `REPEATABILITY` 1e-3, the larger of two error estimates, a non-central window only when 10 times smoother | `identify.rs`, `trace.rs` | The row-wise error is conservative (about 8 times the seed scatter on the elastic arm). |
| Sweep fit: the stated error's exponent is measured over one whole probe step (a factor 1.1); S1 curve tolerance 2 % or half the last printed digit | `identify.rs`, `catalog/tools` | Half a step measured the sweep's resolution (0.23 where the T-gantry's belt mode moves as k^0.37). |
| Input shaper on a machine that changes (a payload gripped or let go): the delays go from the old to the new period along a quintic smoothstep, none changing faster than 0.1 of elapsed time (`SHAPER_RAMP_RATE`) | `control.rs` | The rate is chosen, not derived. During the ramp the shaper is tuned between the two resonances; a higher rate shortens that but lets the delayed copies run further off the plan's speed (now 0.9 to 1.1 of it). A period that follows the pose alone does not ramp. |
| An arm's rest-sag allowance (`rest_sag`): the largest sag over 5 places per axis in every combination, times 1.1 | `chain.rs` | Corners and centre alone missed the largest sag (3.75 mrad on the URDF arm against an allowance of 3.05; now 4.08). |
| Self-locking worm: Coulomb friction on both flanks; in the rigid model the overhauling flank takes the statically held torque (a Painlevé regularisation); stick below 1e-3 rad/s over 1 ms | `sim-netlist/src/worm.rs` | Rigid and elastic models agree at steady speed and differ in a transient: the elastic worm jams within milliseconds when a lowering load is left to stop. |
| Mesh friction of a gear with no back efficiency of its own gives 1/(2 − η) driven back, the loss factor η | `mech.rs` | About (1 − η)², 1 % at 0.9. |
| Behind a converter Dahl's law acts on the combined twist | `transmission.rs` | Only the small-amplitude stiffness and the sliding level are exact. |
| The linear delta's guide friction scales its bristles' stiffness and damping with the load, as its levels | `machine-delta-linear` | Needed for the friction and the acceleration to have a fixed point. |
| Guide load law `preload + |tilt|` | gantries | Has a corner at zero acceleration; preloaded carriages stay flat until the preload is exceeded. |
| Joints that turn on (crank, knife, cam disc, indexer driver) have no end stops unless the config gives them | `machine-crank-cam` | A limit switch that trips the drive is not modelled. |
| Cam follower: Palmgren's line contact, a constant Coulomb guide friction | `machine-crank-cam/src/disc.rs` | The contact ignores the curvatures (logarithmic). |
| Crane: the free rope's speed varies linearly along it; the shaper uses the double pendulum's lower root | `machine-overhead-crane` | The rope's mass in the shaper's frequency is ignored (about 0.05 %, an estimate). |
| Cable robot: the winch's spin axis horizontal and square to the winch direction | `machine-cable-robot` | No layout given. |
| A strain wave gear's rotor turns against its output; a rack's pinion rides above the fixed rack; an idler's positive speed follows its section's first pulley | joint kit, `sim-netlist` | Conventions for the frame's reaction. |
| The cylinder check refuses every rotary axis that carries the cylinder | `pneumatic.rs` | Conservative. |
| Modes shown at 5 evenly spaced poses of a run; a refused start push keeps the last start | `sim-app` | The tool is then up to one sag off, on the envelope only. |
| A machine-file table's reach box | `sim-netlist` | Conservative: a pose outside it may be reachable. |
| The DC-link account switches to 257 passes after 131,072 stretches | `analysis.rs` | Error at most 1/512 of the link's room. |
| The torque-band monitor expects the model's own friction state | `control.rs` | An idealised observer; a real controller would drive the same model from measured speeds. |
| OPC UA tests read the end of a path 1 s after it; timing tests assert the pacing's books, not wall time | `sim-opcua/tests` | — |

## 4. What the models still leave out

Sizes are measured by a run where it says so, otherwise worked out by hand.

| Left out | Machine | Size |
| --- | --- | --- |
| The cables' own inertia; the lift of cable from the drum to the pulley; the sag's weight changing with the pose | cable robots | 0.5 % of the spatial platform's mass; 0.05 N; 5e-5 N. The sag model is a shallow parabola, 8.7e-5 to 1.7e-4 N from the exact catenary (measured). |
| The rotors' gyroscopic coupling with the legs' swing (it is in the frame's reaction only); the rods' faces against platform and base | hexapod, tripod, rotary hexapod | About 0.015 N m (hand estimate). |
| The hook block against the payload's sides (it lands on its top only, since 21 November); a hook block already lower than the payload's top (a pick with it under 0.28 m over the payload's point) stays inside it; one resting on the payload drops through once the payload is let go; the landing on the rigid rope is untested | crane | None in the shipped cycle: the sling holds the hook block over the payload, and its picks hang the hook block 1 m over it. |
| Link bending together with parallelogram bars of their own (refused); the palletiser's rod and crank (shipped bar arrays empty) | joint kit | The bend-mode method overestimates by 1.2 to 2.6 % against the exact beam (measured in the test case). |
| The screw's stiffness along the stroke keeps the far end's for the wafer handler's and the parallel SCARA's pose tolerance (their own `rest_sag`, `machine-wafer-handler` and `machine-parallel-scara`: the column's m g / K with the far end's K, the stroke's `static_twist`), on the safe side; `chain.rs`'s takes the nut, and since 21 November the fit's twist (`Machine::twist_at`) and the mesh bristle (σ_m r, its store still not booked: at most (μ |held|)² / (2σ_m) per gear) follow the nut too | wafer handler, parallel SCARA | The tolerance up to the far end's sag over the pose's. |
| The belt's angular momentum about y of the T-gantry file against its crate: the file takes each run where it is drawn (carriage idlers 0.032 m above and below the carriage and 0.042 m to its sides, bottom idler 0.08 m under the Z axis's point, the clamp at that point), the crate its belt path (horizontal runs at the carriage's height, the clamp s + z above it, vertical runs at its x, the tilting runs meeting the carriage at its x). The file's bodies are drawn at their axes' points (an `offset_m` moves only their momentum), so its clamp cannot stand where the crate's is; the crate has no idler spacing | T-gantry | Over 1 s of the default cycle up to 1.29e-3 N m s about y at up to 0.25 m/s, against 0.42 kg of belt (measured; held under 0.42 kg x 0.05 m times the speed, a bound). About x and z none since 22 November (the file's wheels in the bodies' plane). On the beam mode alone the difference is the idlers' spacing d, (2/3) mu d (sx - 2 x) w' (2.8e-3 N m s per m/s at an end), to rounding (`the_t_gantry_files_tilting_runs_turn_with_the_beam_as_the_crates`). |
| Belt-to-pulley heat conduction | linear delta | Not sized. |
| The tube's single-mode shape for inertia with the static stiffness 3EI/L | D3, D4, H4 arm bending | Not sized. |
| An obstacle's push is a spring on the motors' angles where the machine does not take it at its tool: the overhead crane (its tool is bridge, trolley and hoist, not the swinging load: the push would need the contact point's own Jacobian) and the winder. Where it is taken at the tool it acts there, at the tool's coordinates and not at the touching part (a payload under a leaning Z axis is held through the tool point); with `effects.base_compliance` not on the base's coordinates | obstacle contact | A load on a soft belt swings on by the belt's stretch (D3: 6.7 mm deep at the impact against 0.11 mm F / k at rest). The crates' budgets of a move into a slab close to 3.5e-4 (T-gantry) and 2.8e-4 (hexapod) of the supply, against about 2e-5 with the push at the motors (measured, `collision.rs`). Contact needs a load above c·h/2.79 (about 56 g) to stay stable. |
| The stepper's detent while disabled | stepper | Up to T_d/(4N) appears or disappears from the budget at an enable. |
| `mode_shape` of the five-bar | five-bar | Display only. |

Fixed since (22 November), each was a row of the table above:
- Run files recorded before the signal `setpoint_age` (`sim-components/src/charts.rs`): the age is worked out from
  the run's own timing (`charts::setpoint_age_of`: the profile cycle from the setpoints' changes, the ticks at its
  multiples of the simulation's time), within 1e-13 s of the recorded column on the T-gantry sampled every 1 and every
  0.25 ms; the tool error without the column is the one with it (peak 0.082 mm over the cycle's first second, 0.446 mm
  as held). Left: a file sampled at a multiple of its profile cycle above one does not hold the cycle, and reads it as
  its sample period. No code in the repository reads such files back; the function serves `charts::tool_error` on
  any columns.

## 5. The 26 `shortcut:` comments still in the code

`git grep "shortcut:"` lists them (23 in the Rust crates, 3 in the Python tools). None is an effect that could be modelled with data at hand.

- **Need data (section 1):** standstill values, winding resistance, base-load current, adapter plates
  (`catalog.rs`, `catalog/mechanics.rs`); the S3 line (`charts.rs`); the rotor's temperature
  (`drive.rs`); the bearing factor below κ 0.1 (`analysis.rs`).
- **Limits of the fixed-step solver:** bristles faster than the step (`mech.rs`); the last 0.5 % of
  the pneumatic flow law (`pneumatic.rs`); the speed curve's sweeps to 1e-7 (`path.rs`); a chain's
  drag on an axis without a guide below 1 cm/s (`sim-netlist`).
- **Conservative or without effect on a figure:** the thermal image after it tripped (the verdict
  fails anyway); an unstable open loop's crossing (`analysis.rs`); the swing joints' unit inertia
  without a payload (`machine-joint-kit`); the shared cycle times and DC link of per-motor parts
  (`catalog.rs`).
- **App and tooling:** the CAD mesh's size after an edit, three display heuristics (`sim-app`); bold
  text widths in the report; the PDF reader's limits; a hand-kept summary and the GIL in the Python
  package; loss-free configs for machine files in the test suite.

## 6. New effects and their switches

Effects that add states are behind a switch, off in the shipped configs. "Fit" means the reviewer
verified the model by hand values and runs and found it ready to switch on.

| Switch | Machine | State |
| --- | --- | --- |
| `effects.tool_gearbox_body` | D4 | Fit (tool mode 41.07 Hz, a second overdamped motion). |
| `effects.arm_bending`, `effects.arm_tube_inertia` | H4 (and the corrected tube model on the D3) | Fit; gearbox modes about 945 Hz with the tube on. |
| `effects.table_twist` | cam and Geneva indexers | Fit. |
| `effects.follower_lift_off` | disc cam | Fit; adds 5 N of guide friction that the model without it does not have. |
| `effects.beam_vertical_mode` | T-gantry, its machine file | Fit; the beam's mass and stiffness are estimates. Since 21 November the belt rides on it in the crate and the file (58.776 Hz in the middle). |
| `effects.belt_run_offset` | T-gantry and its machine file (`machine.lean`'s `belt_offset_m`) | Verified by its hand test, energy and budget (21 November); off: it lowers the lean's mode by some 6 % (18.75 to 17.55 Hz at x = 0.9, z = 0.05 m), so the shipped shaper tunings and the tests with recorded lean modes would move. The offset `tool_flex.belt_offset_m` is an estimate. |
| `effects.rope_mass`, `effects.hook_block`, `effects.joint_elasticity`, `effects.bridge_skew`, `effects.floor_contact` | crane | Verified by their own tests; see section 4 for what was not run. `effects.joint_elasticity` is fit to switch on since the gains of section 2 (bridge 177 Hz, 51.7°, 9.7 dB; trolley 183 Hz, 51.0°, 9.9 dB; hoist 49.5 Hz, 78.2°, 20.7 dB), and its cycle verdicts match the rigid ones. |
| `drive.overmodulation` | every drive | Off; verified. |

Always on since November 2026 (they changed shipped figures): the legs' bodies on the hexapod and
tripod; cable weight and sag; the elastic Z screw and tool turn of the five-bar and the wafer handler's
lift; the linear delta's friction load with the acceleration; the pressure angle in the cam indexer; the
indexer's ring on the table's own inertia; the rectifier's loss in the supply energy (about +2 % energy
per cycle with a DC link); the shaft drag from the iron loss on three machines; printed S1 curves for
named S210 motors; back-drive efficiency of screws and of the table's worm; gravity in the guides'
commanded acceleration.

## 7. The catalogue against the vendors' documents

- **Checked row by row against the PDF, none wrong:** Bahr 23 rows, Festo DSBC 7, Harmonic Drive SHG
  44, Rexroth CKR 11, Siemens S210 8 (shaft height 40). The Rexroth extraction script reproduces the
  CSV from the vendor's PDF.
- **Never checked against a PDF (none on this machine):** Apex, Beckhoff, B&R, Gates, HIWIN,
  Kollmorgen, Lenze, Megadyne, Mitsubishi, Neugart, SEW, Wittenstein, Yaskawa; Festo EGC / ELGA;
  Harmonic Drive CSF / CSG; Rexroth MS2N / ctrlX.
- **Known disagreement inside Siemens' documents:** catalogue D 21.4 prints 5 pole pairs for all
  sixteen 1FT7 High Dynamic motors; the per-motor data sheets say 8 poles (6 for the smaller sizes).
  The catalogue code takes the data sheet's.
- **Harmonic Drive efficiency:** read off the vendor's graphs (0.699 to 0.800), about 1 point of
  reading error.
- **43 shipped motor blocks** are tested against their catalogue rows.

## 8. Verification state

- The audit's fixes: one full run of the suite on 17 November (2,265 tests; 2,255 passed at once, the
  other ten fixed or passing alone), plus format, clippy, doc tests, the web view's and the Python
  package's tests and the catalogue checks.
- The modelled effects: written by implementers who ran nothing, then reviewed item by item by
  reviewers who redid the derivations and ran each group's tests. The reviews found and corrected,
  among others: the iron loss's definition, the cable's sag law, the belt runs' geometric stiffness,
  the rope's kinetic energy, the parallelogram bars, the duty mix's thermal model, a transposed matrix
  in the machine files' lowest mode (existing code: the five-axis file's resonance was 71.3 Hz, is
  43.6 Hz), the D3's tube inertia (existing code: gearbox modes about 40 % low), a config helper that
  silently dropped all but its first line.
- The final full run after all of this (17 November): 2,493 tests; 2,472 passed at first, 18 failed
  and 3 timed out. After the fixes a second full run: 2,488 passed, 2 failed, 3 timed out (the app's
  optimiser test on three machines, which passes alone in under five minutes). The two failures were
  the input shaper's and were closed afterwards by reading the plan itself between cycles and by the
  ramp of section 3; their fix ran the shaper's tests of every machine (40 of 40) and the shared
  components' tests (394 of 394), not the whole suite again. Format, clippy, doc tests, the web
  view's and the Python package's tests and the catalogue checks pass.
- What the final run corrected in the models: thermal values not passed on through the base mount
  (the linear delta on a base lost its belts' thermal resistance); the tool turn's gain of the
  five-bar (section 2); the arm's sag allowance and the sweep fit's error (section 3). The measured
  ball screw comparison moved in one figure: the constant-speed run's worst sample is 3.62 N m (was
  3.65) since the screw is driven back at its own efficiency (0.98963 against 0.98972 forward); the
  S-profile RMS stays 0.368 N m. Since the damping in series it is 3.65 N m again (section 10).
- The synthetic belt dataset was recorded by an older model (its stiffness fitted as 1.027 ± 0.006
  of the config's) and was regenerated; a test now checks a fresh trace fits its own config.
- Verdicts of every shipped machine: all machine files pass; of the default configs only the crane
  fails (residual sway 0.300 against 0.05 on 21 November 2026, 0.323 on 17 November, with the gains of
  section 2; its warm start converges now, 0.019 K), which is what it ships to show
  without input shaping.

## 9. Documents: what was regenerated and what was not

Regenerated on 17 November from runs on the merged build, or checked against the code:

- `docs/config-reference.md`: every key the program reads (about 105 keys added; the defaults of
  field weakening, iron loss and resistor heating corrected).
- `docs/model-reference.md`: the whole file; every machine has a section; the "Known shortcuts" table
  lists the 26 `shortcut:` comments; five verdicts that were missing are in the verdict table.
- `docs/fmi.md`: output counts and a fresh check log of all five FMUs.
- `docs/user-guide.md` and `docs/de/benutzerhandbuch.md`: every section except the blocks below.

Re-run on 21 November 2026 on the build of main 59a2aff7 (after the screw and rack damping in series and
sourced, the screw stiffness that follows the nut, the T-gantry's belt runs' weight, the XYZ portal's Z belt loop,
the planner's train allowance and exact run join, the retuned tripod and wafer Z gains, the crane's slack sling):

- `docs/user-guide.md` and `docs/de/benutzerhandbuch.md`: every `$ sim-cli` block and the figures
  quoted from them, among them those the list of 17 November left out: the clearance (2.344 s, 2.228 s
  nominal) and box (3.052 s) optimiser runs, the EXCT-30 optimiser and its pareto front, the worst
  corner (3101 cycles), Morris, Sobol', the duty mix (on a reconstructed `exct-30-empty.toml`:
  `exct-30.toml` with `effects.payload = false`), the swing fit (`swing.csv` rebuilt from the guide's
  description reproduces the printed fit to the last digit), the band count (139 cycles, 69 bounded
  values), the tripod's and hexapod's margins and gains, the thermal-feedback figures, the EXCT-30's
  worst-corner optimiser (8 steps back to 2.048 s, 5371 corner runs) and its result at the worst
  corner (settling 0.24 mm, belt 33 N), the D3's and D2's worst corners (they pass now: 0.37 and
  0.496 mm), the linear delta's and the EXCT-30's margins across the bounds.
- `docs/overview.md`, `docs/mcp.md`: every block.
- `docs/validation.md`: the friction feedforward table, all rows and machine files; the EXCT-30 worst
  corner; the wafer, tool-space and tripod rows.
- `docs/model-reference.md`: the base table's T-gantry, screw, rack, parallel SCARA and hexapod rows;
  the robust optimiser; the warm-start rises; the energy budget's tolerance; the wafer column's and the
  parallel SCARA stroke's modes (at the nut); the corner path's motor speeds.
- `docs/config-reference.md`: the tripod's, hexapod's and EXCT-30's margins (the T-gantry's unchanged);
  the linear delta's integral time (0.012 s, as the config has it, not 0.02 s).
- `docs/machine-files.md`, `docs/mechanisms.md` and their German twins: the H-bot block, the rack
  gantry's and dual gantry's figures, the yaw path errors, the wafer lift's screw, the crane's cycle,
  the arm's rest offset; and the claims the program contradicted (a screw's stiffness at the softest
  place, the parallel SCARA's rigid stroke and turn, the crane's failing warm start).

Not re-run on 21 November 2026 (the figures stand as dated in the text):

- The T-gantry's `optimise --worst` (some 5000 corner runs, over an hour on 4 threads) and `--robust`
  in the guides; the two-box height profile (4.72 s against 3.87 s, a test's figures).
- The D3's Morris figures (36 values, 370 cycles; the D3 is unchanged by the changes above).
- The base table's rows other than the five above (17 November).
- Done (21 November): `docs/machine-files.md`'s dual gantry yaw mode (155.0 and 149.5 Hz with J raised to 20 kg m²).
  The file machine had no `Machine::modes`: the trait's default gave `resonance_hz`, the lowest mode alone, with
  damping 0, so `sim-cli modes` showed the Y belt (96.4 Hz in the middle, 124.5 at y = 0.8) and, with J = 20, the yaw
  only where it was lowest (151.7 Hz at y = 0, coupled with X). The 155.0 Hz was the test's, which also stiffened the
  belt (EA 5e8) to make the yaw the lowest; the 149.5 Hz was not reproduced. Now every mode with its damping: as
  shipped the belt 96.40 Hz (0.033), X 262.95 Hz (0.088) and the yaw 425.37 Hz (0.144) in the middle, at y = 0.8 the
  belt 124.45 Hz and X with the yaw 256.12 and 395.01 Hz (since its Y belt is a loop, below: 82.21 Hz (0.039), 262.34 and
  425.37 Hz; 71.56, 255.60 and 394.83 Hz); with J = 20 the yaw 155.04 Hz in the middle, 151.65 Hz at an
  end (`the_dual_gantry_reports_its_yaw_among_its_modes_by_hand`, a hand 2x2 of x and the yaw; the T-gantry file's modes
  and damping its crate's to 1e-9).
- `docs/web.md`: the CoreXY path's sizing check (1.661 s, 14 November) names no path, and none of twenty lines and arcs
  tried on the build of that day gives it. The page now names one, the page's default path moved into the CoreXY's
  range, with its figures of 21 November 2026 (1.748 s within the axis limits, 1.641 s time-optimal, every check
  passing; `sim-cli analyze`, which the page's sizing check equals); the 1.661 s stays as the session's, its path unknown.
- The T-gantry budget named in the list of 17 November: no such table is in `docs/validation.md`.

## 10. Reported by the document agents

Fixed (second audit, 21 November):
- The lumped forearms (planar delta, D3/D4 with `forearm_inertia` off, parallel SCARA) had their momentum and
  first moment at the inertia's split, their weight at the middle: V + g · c drifted (D2 0.060 N s, 2.6 N of
  vertical reaction). The frame momentum now takes them as rods (a sixth at each end, two thirds at the middle);
  after the fix 2e-11 N s and 1e-6 N on the audit's probe. The kinetic energy stays the split's (the model's).
- The T-gantry's belt runs moved in the frame momentum but had no weight: the vertical runs' 0.16 kg now weigh on Z
  (`z_weight`; hold 2T₂ − T₁ − T₃ = 70.216 N, not 68.647), in the crate and, from its spans, in the machine file
  (`runs_height`, which weighs every file's runs where its momentum has them).
  V + g · c drift over the default cycle 0.4708 J before, 1.3e-7 J after.
- `config-reference.md`'s beam mode "59 Hz in the middle" is the stiff belt's; on the default belt it couples
  with Z's mode to 51.5 and 140.1 Hz.

Fixed (after the second audit, 21 November):
- The crane's shaping optimiser took its second shaper from the second mode of `Machine::modes`, and a shaper following
  the pose its second mode there: since the bounce along the rope is in the crane's modes, without the hook block that
  was the bounce (8.30 Hz at the start pose) where it should be none. Both take `Machine::sway_modes` now (by default
  `modes`; the crane's swings alone): with the hook block the upper swing (2.55 Hz), without it no second shaper
  (`the_cranes_second_shaper_is_its_upper_swing_not_the_bounce`).
- A margin figure for a loop across a backlash (was in section 4): `sim-cli margins` prints the describing function's
  margin across the play (`play_margin_db`, model reference, hammering across a backlash), with the rotor's friction
  faded. Five-bar's tool turn: -4.18 dB at 0.08, +3.56 dB at the shipped 0.033 (onset 0.0497); a time run with the
  friction off hammers from 0.048 (within 4 %), with it on from 0.078: the friction still damps the swing, which the
  figure leaves out on the safe side. Told, not judged (the time run stays the verdict); each run is measured once more
  with the play closed (margins of the five-bar 20.5 to 23.8 s). Still open: the kicks and the margins are at the start
  pose only; the margins across the bounds vary config values, not poses, so there was no pose list to reuse.
- The hexapod's lowest held mode (183.2 Hz) lies below its planner trains (289.6 Hz, each leg's M_kk with the others
  held), and the trains are right for the allowance's jerk term: with one motor driven and the others held the twist
  rate is [K^-1 M]_kk j = M_kk j / K_k, the modal sum of its column over the held modes (model reference, the planner's
  speed ratings). Only the cap after a step of the acceleration would see the lower modes; on the tripod's, hexapod's,
  wafer handler's and joint-kit SCARA's default cycles the jerk term is taken by a factor 130 to 3600, so no figure
  moves and the train is left as it is.
- A cam changed by a reconfigure while it runs (was in section 4 as an unmerged fix): the fix is on main since
  dc454002 (17 November), each stored plan keeping its cam's curve (`Plan::cam`, `Machine::cam_curve`). Checked on 21
  November: `a_cam_changed_under_way_leaves_the_shaper_the_plan_as_it_ran` fails with the delayed copies read off the
  machine's cam (2.16e-3 rad off the copies of the plan as it ran) and passes with the fix; the shaper tests of every
  crate and the control tests pass (41 tests, 30 s).
- The XYZ portal's Z belt is a loop: its section z2 is 0.9 m (0.5 m from the motor to the far idler and 0.4 m
  back to the clamp), its span z_2 0.5 m, so the falling run is as long as the two rising with the slide. Its Z
  holds 3.5 kg again, 0.0686466 N m (before 3.54 kg, 0.0694311 N m); its belts weigh 2T / mu = 3.1 vx² + 1.8 vy² +
  1.0 vz² (2.7, 1.4, 0.6 before). Its X and Y belts are loops the same way since (they shared their lengths out, 0.1 m
  from the motor to the far idler): sections x2 2.3 m and y2 1.3 m, spans x_2 1.2 m and y_2 0.7 m (0.1 + stroke + 0.1),
  so 2T / mu = 4.8 vx² + 2.4 vy² + 1.0 vz² (the belts 2.4, 1.4 and 1.0 m; on X 0.32 kg beyond the pulleys' lumps, was
  0.207). The return sections are longer, so the belts are softer: X's mode with the pulleys held 39.23 Hz at its far
  end (73.91 before), 45.1 Hz in the middle (53.5), 97.8 Hz at its start (100.1); Y's 69.83 Hz in the middle with X at
  its start (`the_portals_x_and_y_belts_ring_on_their_loops_by_hand`). The cycle passes every verdict as before:
  settling 27.5 um (22.7), belt tension 376.0 N (330.9), least 170.0 N (179.6), X motor RMS 0.3691 A (0.3669);
  following error 159.4 um (107.3), with the friction feedforward off 189.0 (142.7).
- The drive's current circle holds the q reference with the d reference, i_q,ref ≤ sqrt(I_max² − i_d,ref²), as a
  drive limits its current reference vector, not with the d current that is there. 1FK2205-4AF single axis at
  200 m/s²: 10.944 A against 10.9 A (0.41 %, above it for 8.75 ms), was 10.980 A (0.74 %). The rest is the q
  loop's lag behind its falling reference: 10.926 A (0.24 %) with twice the current gain. No shipped config's
  cycle reaches field weakening (the d current stays 0 on every registered machine's, machine file's and the
  conveyor, EXCT-30, flying, cam and joint-kit configs' cycles), so their figures do not move.
- The H4's lumped model (`forearm_inertia` off; on as shipped) put the forearms' weight and momentum at the
  inertia's split (2/3 at the elbow); now the weight at their middles and the frame momentum as rods, as on the D3:
  hold at the centre 850 mm down −18.58980 N m per gearbox (−18.60312 before), the vertical mode under constant
  torques −4.71 N m/rad at 975 mm down (−4.30), +5.45 at 1175 mm (+5.66).
- The modes' damping behind a screw whose stiffness follows the nut took the far end's dampers with the stiffness at
  the nut (`chain.rs` held and coupled modes, the parallel SCARA's stroke and turn): now both at the nut
  (`Gearbox::at`). Against the linearised machine with its rotors held (effects
  `the_modes_are_damped_as_the_linearised_machine_at_the_nut`): tripod 0.01946, 0.02134, 0.02444 (was 0.01574,
  0.01726, 0.01977), hexapod 0.01047 to 0.0480 (was 0.00872 to 0.0401), parallel SCARA's quill at z = 0.18 m 0.0399
  (was 0.0216), joint-kit SCARA's stroke 0.0271 (was 0.0228), also with its links bending two-way. Only the printed
  damping moves: the shaper takes the frequencies. With the links bending two-way the SCARA's pair of link modes at
  680 and 737 Hz was damped 0.0756 and 0.0844 against 0.0741 and 0.0860 linearised (the sum kept), each mode's damping
  taken on its undamped shape: since 21 November it is the damped problem's, -Re lambda / |lambda| of the eigenvalue of
  M lambda^2 + C lambda + K at the mode, C iterated to |lambda| (`chain::matrix_modes_at`), 0.074087 and 0.085952 against
  0.074087 and 0.085952; the test checks every mode now (it stopped below 650 Hz). `Mode::hz` stays the undamped
  frequency (the shaper's); the pair's |lambda| is 687.4 and 729.7 Hz against 680.1 and 737.5 undamped.
- The hysteresis behind a screw whose stiffness follows the nut (b253b9b5) picked its Dahl branch at the anchor by
  rounding: `at` rebuilt the anchor's twist as (x r) / r, an ulp to either side of x, and at the anchor the branch also
  went by x' where the state lives on y = r x. The torques were right, the slope and so the loss power wrong exactly
  where `Sim::step` takes it (87 of 400 anchors off by more than 0.1 %, worst sign flipped; a nut travelling at a
  standing twist got a negative loss). The anchor's twist now stays y0 with r beside it (`Gearbox::twist_scale`), the
  branch compares x r with y0, and at the anchor goes by y' / r (`the_hysteresis_loss_at_the_anchor_goes_by_the_move`).
  Nut held at 0.3 m under 7 Hz load cycles: 9.1e-5 J of 7.4e-4 J lost left open before, rounding (1e-13 J) after, and
  as little with the nut running from 0.8 to 0 m (9.3e-5 J before)
  (`the_hysteresis_behind_a_screw_closes_its_budget_over_load_cycles`; the review's ~1e-6 J "level without the
  stroke" and its 1.8e-5 J on a running nut were its probe's: it committed at t + dt and started the next step at
  (i + 1) dt, an ulp apart). On the gear itself (r = 1) every number is the same to the bit.
- With effects.beam_vertical_mode the frame momentum's first moment has the beam's own mass moving with the mode
  (−p₀(x) w) but the potential did not weigh it: off the middle, where p₀ changes with x, V + g · c drifted (T-gantry,
  1 s of free pulses: 2.5e-4 J while V moved 0.16 J; 1.3e-8 J with the mode off), and `weight_is_at_the_first_moment`
  checked the middle only. The beam's weight on it, −g p₀ w, is now in the potential and the cross term of its own sag
  with the mode, +g p₀ w, in the beam's elastic energy: they cancel, so the total energy and the dynamics are as
  before. In the crate and the machine file; checked off the middle
  (`testkit::weight_is_at_the_first_moment_at`, `the_beams_own_weight_is_at_its_first_moment_off_the_middle` in both)
  and over the 1 s run (below 1e-7 J with the mode on and off).
- The app's path test (`a_path_is_planned_run_and_analysed_in_the_app`) failed on the crane under load (the bridge
  1.37 mm from home) and passed alone; a 2 mm tolerance for the crane was put in. Cause, measured on the library: the
  load's swing, which nothing but the drive damps, pulls the bridge to and fro by the drive's give, and the app starts
  the path back at the phase the analysis's wall time leaves: over the phase, the bridge stands 0.06 to 1.49 mm (half
  its span over a swing) off and up to 1.55 mm off 0.2 s after the path back; the middle of the span within 0.033 mm.
  In the app with the path back started 1.4 s later: 1.364 mm off at the check after the unreadable line, the middle
  0.001 mm. Not a regression of the crane's commits of 21 November (e4a90405, 074796e0, e9a65662: the library's figures
  are the same to nine digits on 8754d55d before them); the drives' give is the retune of 17 November (389bbac7: 0.3 mm
  either way after the long move where the stiff drives gave 5 um). The test now judges where an axis stands by the
  middle of the span it covers over one period of the machine's lowest mode (`standing`), at the 1 mm it had; the
  swing's amplitude itself is not judged there. (A probe's reversed point list was refused with `error-trajectory`:
  its first time was the plan's end, not 0; the app's file pairs the forward times with the reversed poses.)
- The mover track's modes: the shipped six movers report the held mode everywhere (modes, `resonance_hz` and so the
  shaper, the plan's trains: 694.28 Hz, the plate held), as the convention has it; a track of one mover reported the
  free two-mass mode (1736.39 Hz, the chain's single joint) to the shaper and the modes printout while its train was
  the held one. One mover now reports the held mode too (`the_movers_modes_are_the_held_two_mass_mode`). The 1736 Hz of
  the brief is the plate-free figure.

- The app's mode list (`shapes::mode_views`, also `sim-cli modes --shapes`' linear_hz) paired each reported mode with
  the eigenvalue of the motor-free linearised model nearest it, whatever its kind. For the mover track's held 694.28 Hz
  that was no mode but a real pole, lambda = -639.7 1/s shown as 101.8 Hz with damping 1: the rollers' Coulomb friction
  smoothed over 2 mm/s, (0.1 + 0.005 * 300) / 0.002 + 0.5 = 800.5 N s/m on the 1.251 kg mover (-639.9 by hand); the
  motor-free model has no held mode, only the plate-free one at 1735.8 Hz. Now a real eigenvalue is no partner, and the
  partner is the nearer of the motor-free model's and the model with the rotor states held (`shapes::linearised_held`:
  the states that move a motor's angle or speed held, as `Machine::modes` has its modes): the mover's 694.28 Hz gets
  the held model's 694.28 Hz to 1e-6, one mover and six
  (`the_mode_list_pairs_the_held_mode_with_the_held_model_not_the_friction_s_pole`).
- The mover track and the indexers (Geneva, cam) had no frame momentum (`Machine::frame_momentum` None), so no
  compliant base: they have it, as the joint kit's slide and the knife (`the_frame_momentum_is_the_hand_calculated_one`
  in both crates; the standard suite's first-moment check now judges them). Both are level, so
  `testkit::weight_is_at_the_first_moment` (which asks the first moment to move vertically) has nothing to judge there.
- The crane's hook block lowered by more than the sling's length onto a standing payload passed through it (section 4):
  0.035 m under the payload's point after a lowering 1.2 m past its touchdown. It now lands on the payload's top (the
  scene's boxes, its point 0.28 m over the payload's), a one-sided contact on the sling's span held at rest only and a
  plastic landing booked as the floor's (114.37 J of the fall, no step making energy), the floor then carrying both,
  2598.762 N (`a_hook_block_lowered_beyond_the_sling_lands_on_the_payload`). No new switch or key: it acts with
  `effects.hook_block`; no shipped cycle lowers the hook block that far.

Fixed:
- `fit --sweep` compared the sweep's lowest dip with the belt mode even where that dip is another
  mode (T-gantry × 0.103, EXCT-30 × 0.400 against their own sweeps): each belt mode is now paired with
  the dip that matches; both give 1.000.
- `sim-cli modes` printed a damping of 0 for the wafer handler's modes: the damping now reaches the
  printout. The crane's 0 is right (the pendulum has no sway damping in the model).
- Stale texts: the H4's comment on the D3's tube; `--machine-file` "not for fmu" in the usage text.
- With a jerk limit of 2000 m/s³ the time-optimal corner path's motors reached 420.7 and 423.5 rad/s
  past the drive's 420, though the plan held them to 415.8: the belts' twist (with them rigid 416.5).
  The plan now keeps back the speed an elastic train adds, s j / w², at most the swing after a step of
  the acceleration (`Machine::trains`, `path::train_speed`): 414.4 and 416.7 rad/s. Taken at the start
  pose only, and the step's acceleration is the motor's own torque on its rigid inertia, so at a very
  high jerk limit more is kept back than needed (T-gantry at 1e5 m/s³: 28 % kept, 6 % overshot).
- The time-optimal plan ran the single axis past its torque while braking (gearbox efficiency 0.9,
  peak 0.5 N m, jerk 1e5 m/s³, rotor 3e-4 kg m², incline -30°, `drive.max_speed_rad_s = 399.1596`;
  "out and back" 1.00459 of 0.4 N m at -387 rad/s on the current main). Not the torque limits
  (`motor_limits`, the curve, friction, efficiency): where the run from the start met the run from
  the end at their peak of speed (`path::curve_join`), the plan switched at the rise's first knot
  past the meeting and laid a cubic from the rise's acceleration (2.02 m/s²) to the fall's next knot
  (-5.14 m/s²), which ended 1.05e-3 m/s off that knot's speed; the 1 ms differences read that step
  of the speed as braking beyond the ceiling. The acceleration gate (4 × jerk × step) is 100 m/s² at
  that jerk and let it pass; the speed gate (2e-4 of the speed) caught most such joins by chance,
  which is why only some top speeds failed. The switch now lies at the runs' exact crossing (halved
  on the rise's cubic, the fall's state on its own cubic), with the rise's and the fall's state as
  two knots at the same time: same place and speed, only the acceleration steps. Swept over 380 to
  420 rad/s in planned_limits; `path::tests::the_runs_meet_at_the_same_place_and_speed`.
- A screw or rack behind a converter: the coupling's damping acted across the whole series spring (the
  wafer handler's column 1.74): since the gear damper in series (`Converter::apply_back`, a Zener
  solid) it bridges the gear's spring alone, 0.0401 with the screw's own damping. The screw's damper
  held at the far end's value let the column's ratio fall to 0.029 with the nut 0.1 m from the
  bearing: it now keeps `screw.axial_damping_ratio` along the stroke (0.042 there with the coupling's
  share).
- The planner's train (`RotaryJoint::train`, the machine files' `trains`) put a screw's or rack's
  compliance in series twice: K1 holds it already (`Converter::apply_back`) and `band_compliance` is
  the same spring. The joint kit's screw axis's train was 0.73 of its two-mass mode; now equal
  (`the_screw_axis_train_is_its_two_mass_mode`). The wafer handler had no train (`Machine::trains` empty): it has
  one per motor since 21 November, the joints' with the other joint held and the bands rigid, the lift's at the
  screw's stiffness at its height (`the_trains_are_the_held_modes_one_motor_at_a_time`: the column's 438.4518 and
  349.0392 Hz at z = 0.1 and 0.3, the joints' the arm's mode with the other gearbox rigid). The tripod and the hexapod
  (said here to have none) had theirs from `chain.rs` (`JointMachine`: each leg's M_kk with the others held, the screw
  at the leg's length), now tested against the hand's sqrt(K / M_kk) at two poses. Kept back on the default cycles:
  wafer handler 1.0000 to 1.0094 % (trains 63.3, 34.4, 432.2 Hz at the start), tripod 1.0010 %, hexapod 1.0025 %; the
  wafer handler's cycle (axis limits) 7.762 s and its verdicts unchanged, timed time-optimal 6.135 s both ways, its
  motors' peaks 0.01 % lower (188.138 to 188.120 rad/s) and its wafer-slip row failing as before (3.9464 against 2.4517
  m/s^2, a check the plan does not keep).
- The hysteresis behind a screw whose stiffness follows the nut kept the far end's values on the twist through gear and
  screw: a load carried along the stroke wound it up (the twist changes by F dc at a force). It now acts on the gear's
  own twist (y = r x, r = K1 here / K1 at the far end; the friction follows K1, the loop keeps its width), its store's
  change with the place booked as stored (`the_hysteresis_behind_a_screw_acts_on_the_gears_own_twist`). No shipped
  config has a hysteresis there: no shipped figure moves.
- The ISW ball screw's constant-speed worst sample is 3.65 N m (3.6487), not 3.62: the gear damper in
  series (60e59797: 3.6156, the fit's S-profile RMS 0.3716) and the screw's own damping ratio 0.04
  (e78913f3: 3.6487, S-profile 0.3681). The worst sample sits at 14.5 s, the simulated torque -1.1 N m
  against the measured +2.5: a gap the damping moves by 0.03 N m, not one it makes.
- Program reader: an arc through points near f64's range (`MoveL,2145413,-8.4e301` then `MoveC`) had a
  NaN centre and was taken; now refused (`program-error-arc`).
- The crane's sling was a rod (section 4): it is a rope of the main rope's EA and damping that goes slack. With it the
  floor contact under the hook block closes its energy budget: the floor lifted the hook block on the standing payload
  step by step and booked that as made energy (an audit's set-down cycle: `power.other` -1.30 J, 1.30 J of 42.1 J
  unaccounted; now -2.3e-10 J and -9.3e-10 J with the torque ripple, whose work is in that row, off), and the floor's
  push on a body still falling did work (0.21 J in a lift-off test): it now holds a body that stands still only.
- The crane's mode list had the swing alone: it has the bounce along the rope (and the sling) and, with elastic drives,
  the drives' modes, all with the motors held (6.77 Hz bounce at z = 2 m; bridge 10.49, trolley 41.09, hoist 108.2 Hz).
  The brief's 12.14, 66, 88 and 111 Hz are the free-motor figures of the linearised model.
- The crane's warm start with `effects.hook_block` did not converge (3.88 K; 0.019 K without it). Not a defect: the
  unshaped cycle does not repeat itself. The double pendulum carries the swing a pass leaves (0.5 m at the dwells)
  into the next; over 30 warm passes the bridge's winding settled anywhere from 1.3 to 5.1 K, bounded and irregular,
  passes started at about the same rises (3.1 to 3.2 K) settling 1.4 to 4.9 K apart; shaped, the same cycle converges
  (0.015 K). Where the warm start converges neither per pass nor over a period it now measures one more pass from the
  rises the last one started at (on this cycle 0.69 K apart, seven times the tolerance): the verdict row
  `thermal-cycle-repeat` and the CLI's message name the cause.

- The yaw path errors of `docs/machine-files.md` (H-bot 0.80 → 1.12 mm, rack gantry 0.20 → 0.23, CoreXY 0.06 → 0.02
  between the text of 14 November and 21 November): measured with `sim-cli analyze --machine-file FILE --set
  effects.gantry_yaw=true` on builds of main (and each commit's own files). The 0.80 and 0.20 mm were never the program's:
  b954254b, the commit that wrote them, gives 1.070 and 0.249 mm. The H-bot's 1.070 → 1.120 mm (yaw off 0.060 → 0.063,
  rack gantry 0.2237 → 0.2250) is 024b876a (merged as 124d9120, 16 November), the belt runs' own mass in the files:
  that build's program on the file before it gives 1.070, the program before it on the file after it 1.120, and with
  effects.belt_mass off both give 1.070. The runs put mu (L + 8 (0.65 - y)) of belt on the H-bot's X (0.86 kg at
  y = 0.1 m where the lumps had 0.42, the hand value of `the_belts_runs_keep_the_energy_and_give_the_moved_mass_by_hand`)
  and 2 mu (2 x - 1) between X and Y: the belt's own kinetic energy, so the more correct figure, kept (which of the two
  terms racks the bridge more was not separated). Left out still: the runs do not
  follow the yaw (`machine.span`'s speeds are per axis speed). The CoreXY's 0.063 → 0.020 mm came with d360b7ed (14
  November, the gantry branch merged with that day's other branches), before the 17 November texts. The bisect logs in
  the earlier scratchpad (`bisect-*.log`) were the ISW screw's (worst sample 3.6487 N m), not these.

- The idler signs of the dual and rack gantries' Y belts (section 4: not checked, the drawing not faithful). Neither file
  had an idler: the sections' and spans' signs were right by hand (a positive turn of pulley_y feeds y1, +1, and takes
  from y2, -1; the run from the motor to the far idler moves at -vy; the pulley turns at -vy / r about +z seen from
  above), but y2's route ran straight back from the pulley to the clamp and its spans shared their lengths out (0.1 m
  from the motor to the far idler) where the yaw inertia took the loop's strands. The wheels' check against the belt's
  kinematics cannot check these belts at all: the files draw them in the x-z plane, where a belt along y has no length
  (with the far idler the route's length is NaN, which the check passed as matching before; it now skips non-finite
  rates, so dual and rack report 0 sections checked, the other files as many as before or fewer). Now each file names its far idler on the bridge (`far_y`,
  0.1 + stroke + 0.1 from the pulley, `speed = { y = 1 }`, anticlockwise as the pulley) in y2's route, and its Y belt is
  a loop: y2 1.0 -> 1.9 m and 0.7 -> 1.3 m, y_2 0.1 -> 1.0 and 0.7 m, 2T / mu 1.1 -> 2.0 (vx² + vy²) and 0.8 -> 1.4
  (`the_gantries_y_belts_turn_their_far_idlers_as_they_run`: the far idler's sign from the spans' own speeds relative to
  the bridge and the pulley's rim speed, not from the drawing; fails on the old files). Before/after:
  dual gantry Y belt mode with the pulley held 96.40 -> 82.21 Hz in the middle, 124.45 -> 71.56 Hz at y = 0.8,
  166.97 -> 163.34 at y = 0; X 262.95 -> 262.34 Hz; settling 10.2 -> 16.1 um, gantry-sync 14.15 -> 14.08 um, feed
  forces 110.4 / 95.8 -> 110.8 / 96.2 N, yaw path error 0.143 -> 0.149 mm; rack gantry Y belt 113.46 -> 98.26 Hz in
  the middle, settling 14.6 -> 16.7 um, radial load 207.4 -> 208.0 N, yaw path error 0.2251 -> 0.2262 mm. Every
  verdict passes on both.
  Found and left open: these files' wheels spin about y in the frame momentum (`wheel_axis`: a file draws its belts in
  the x-z plane unless a section changes with both x and y), which a belt running along y cannot turn about; a flat Y
  belt's pulley turns about z. Size: the Y motor's rotor and pulley, J_rotor w + (J_pulley - lump r²) w / i, about
  1.6e-3 N m s at the dual gantry's Y peak of 47 rad/s, put about y instead of z. Which way a belt along y lies (flat
  about z, standing about x) a file does not say; a per-belt plane is needed (also for the XYZ and C portals' Y belts).
  The same drawing places y2's runs for the frame momentum (`run_midpoints`): before, without the idler, none (left out);
  now y_2 midway between the pulley (drawn standing at x = 1.5 m) and the far idler at the bridge's x, y_3 none (its
  wheels meet in the plane). Size: 0.1 kg of belt up to 0.75 m off along x at the Y speed, about 0.014 N m s about z
  at 0.19 m/s; only with effects.base_compliance, off in both files.
  Fixed since (22 November): each section names its belt's plane (`plane`: "x-z" standing along x, "x-y" flat,
  "y-z" standing along y; default as before, so the H-bot, CoreXY and T-gantry do not change; sections sharing a wheel
  share it, else refused), and a pulley may ride on a body (`body`, its `at_m` from it). The wheels' check, the wraps,
  the frame momentum's wheel spin (about the plane's normal, -(along x up) for clockwise) and the runs' places
  (`run_midpoints`) take each section's plane. The dual, rack, XYZ, screw and C portals' and both five-axis files' Y belts
  lie flat (`plane = "x-y"`) with the Y pulley on the bridge, 0.07 m before its y = 0. Dual gantry at x = 1.0, y = 0.3,
  vy = 0.2 m/s: angular momentum [4.0e-4, 2.5e-3, 0.98500] N m s before, [0, 0, 0.997028] after, the hand value to
  1e-16 (`the_dual_gantrys_frame_momentum_with_its_y_belt_flat_by_hand`: the Y carriage's 5 x vy, the pulley's and
  rotor's -(J_p + i J_r) vy / r about z, the runs' -4.72e-4 from their middles' offsets off the wheels' line); the wheels'
  check covers y1 and y2 along y again (dual and rack 4 sections checked, before 0;
  `the_gantries_y_belts_turn_their_far_idlers_as_they_run`). The screw portal's Y wrap with effects.belt_wrap is the flat
  one, pi + 2 asin(r / (y + 0.07)) (3.3774330 rad at y = 0.1, was 3.3419275 from the standing drawing).

- The screw portal's, C portal's and both five-axis files' X and Y belts (section 4: they shared their section lengths
  out, 0.1 m from the motor to the far idler, and drew no far idler) are loops as the XYZ portal's, with their far
  idlers in the routes (`far_x` on the frame at x = 1.1 m, `far_y` on the bridge 0.7 m from the Y pulley, `speed` +1,
  anticlockwise as the pulleys): sections x2 1.2 -> 2.3 m, y2 0.7 -> 1.3 m; the C portal's spans x_2 0.1 -> 1.2,
  y_2 0.1 -> 0.7 m, 2T / mu 2.1 vx² + 0.8 vy² -> 3.8 vx² + 1.4 vy² (on X 0.19 kg beyond the lumps, was 0.105; on Y
  -0.05, was -0.025). Modes with the pulleys held (`the_screw_portals_belts_ring_on_their_loops_by_hand`, fails on the
  old files): screw portal X at its far end 74.48 -> 39.69 Hz, Y in the middle 92.64 -> 80.23 Hz; C portal X 37.42 Hz,
  five-axis X 32.21 Hz (Y couples with the rotaries there). Wraps with effects.belt_wrap at x = 0.3, y = 0.1:
  3.1982802 and 3.2709416 rad (pi + asin(r / D) + asin((r - r_f) / d), was the clamp-and-back drawing's). Cycle
  verdicts all pass; settling, largest and least belt tension before -> after: screw portal 22.1 -> 27.4 um,
  330.5 -> 374.3 N, 181.1 -> 172.9 N; C portal 34.0 -> 34.9 um, 365.7 -> 422.0 N, 168.1 -> 146.8 N; five-axis
  33.6 -> 35.3 um, 351.5 -> 362.7 N, 234.2 -> 210.8 N; five-axis table 30.3 -> 32.2 um, 343.3 -> 360.4 N,
  261.0 -> 248.5 N. The softer return sections raise the following error (validation.md's friction feedforward table,
  re-run: screw portal on 99.5 -> 152.3 um, C portal off 177.4 -> 253.1 um), within every verdict. The screw portal
  and the five-axis files have no spans (their belt mass stays lumped).
- The XYZ portal draws its loops so too (section 4: its X and Y belts loops by length without a far idler, its Z pulley
  at its `at_m`, not on the Y carriage): far_x on the frame, far_y on the bridge, far_z on the Y carriage 0.5 m above
  the Z pulley, which rides on the Y carriage (`body`). The wheels' check covers x2/x, y2/y, z1/z and z2/z (18 sections,
  none of the second sections before; `the_xyz_portals_far_idlers_turn_as_their_belts_run`). Frame angular momentum at
  x 0.5, y 0.25, z 0.1, vz 0.2 m/s: [0.17606, -0.34946, 0] -> [0.175, -0.347264, 0] N m s, the hand value to 1e-12
  (`the_xyz_portals_frame_momentum_with_its_z_belt_by_hand`: the slide's y m vz and -x m vz, the Z pulley's and
  rotor's (J_p + i J_r) vz / r about +y, the runs' mu vz (L1 e1 + L2 e2 + L3 e3)). The section lengths, modes and cycle
  verdicts do not change (verdicts identical, base compliance off).

- An obstacle's push at the tool for the linkage, spatial delta and scissor machines (section 4's row: pushed at the
  motors), and with them the D2, D3 and D4 crates, whose arms they are (22 November): `set_tool_force` puts J^T F on
  the arms in their equations of motion only (`push`: the rigid solve and the elastic derivatives; not the feedforward
  or the state at rest, as the machine files' process force and push; the spatial delta P^T F in `joint_load`, with
  bending arms in `distal` on the elbows' angles and moves beyond the tubes; the scissor's h' F), and the push into the
  frame momentum's outside force; the D4 forwards x, y, z to its arms and puts c on its turn. Files
  (`the_linkage_delta_and_scissor_files_are_pushed_at_their_tool`, fails without: planar delta 0.364 mm deep against
  F / k 0.123 mm): depth F / k to 1e-5 (planar delta 0.1219 mm at 1219 N, D3 0.1500 mm at 1500 N, scissor 2.827 mm at
  28.27 kN), the push's power F . v to 1e-7 as shipped, rigid without the gearbox loss and with the D3's arms bending;
  budgets 2.2e-6 (planar delta), 1.1e-6 (D3) and 2.3e-4 (scissor) of the supply, the same moves without contact
  9.1e-6, 3.5e-5, 2.8e-7: on the deltas the brief's 1e-6 is the integration's floor, not the push's. The scissor's
  2.3e-4 is the integration's too: holding 28 kN against the ceiling its gearbox's efficiency changes between driving
  and driven back within 0.5 W (`Gearbox::loss_factor`'s fade), a few mm/s of the platform at that force, which 4
  substeps of 15.6 us do not follow: 1.2e-4 with 8 substeps, 6.9e-6 with 16, 4.2e-8 with 32 and 64, 4.6e-8 with 4 and
  the gearbox efficiency off. The test runs it with 32 and holds every file's budget to its no-contact move's (or
  1e-6); the shipped file keeps 4 (32 makes its verdicts 5.6 times slower: 3.9 s against 21.5 s). Crates (`collision.rs`, no longer ignored): D2 98.07 um at 980.7 N, budget 6.9e-9, D3
  111.5 um at 1115 N, 9.1e-6, D4 111.8 um at 1118 N, 8.1e-6 of the supply and potential energy.

- The T-gantry file's tilting runs on the beam mode (section 4: their angular momentum at the run's middle): a point at
  the share xi of the way from the post's pulley f to the carriage end c goes down by xi w, so the run's angular
  momentum from w' is its weight times w' / 2 at f + 2/3 (c - f), the crate's weighted point; the file now takes it
  there (`run_ends`, the end that rides on the beam's axis is c). Beam mode alone, w' = 1 m/s, x = 0: about y 2.8949 ->
  2.8739 N m s against the crate's 2.8711; what is left is the idlers' spacing d = 0.042 m, which the crate does not
  have, (2/3) mu d (sx - 2 x) w' to rounding at x = 0, 0.5 and 1
  (`the_t_gantry_files_tilting_runs_turn_with_the_beam_as_the_crates`, fails without: 2.3e-2 off). The belt's angular
  momentum about x and z: the file drew its pulleys and idlers 0.02 m in front of the bodies (clamp at y = 0, wheels at
  -0.02: the belt bent out of its plane), now in the bodies' plane: 1.1e-3 -> 7e-19 N m s about x over the cycle,
  tested to 1e-12. About y 1.29e-3 stays (section 4's row): the drawing's heights against the crate's path; a patch to
  the crate would add the idlers' spacing (the tilting runs' carriage end at xc -+ d) and their heights (the horizontal
  runs at h -+ (r + r_idler)), a patch to the file would need bodies drawn at their offsets. Neither is done.

- The angular momentum of the T-gantry crate against its file (section 4: not compared, different origins and belt-run
  geometry). Compared now over 1 s of the default cycle, every 50 steps, both about the place of the Z axis's centre of
  mass at z = 0 under the carriage at x = 0 (the crate's tool frame, which the file's bodies share). One difference was
  a body's place: the file put the carriage at the Z axis's height, the crate z_com_drop (0.3 m) above it, so the file's
  angular momentum about y lacked 0.3 m x 4 kg x vx and its first moment 1.2 kg m. A body's `offset_m`, before refused
  off rotary axes, may now set a constant offset on straight axes (only the frame momentum takes it: the mass matrix,
  gravity and the drawing do not change), and the file's carriage has [0, 0, 0.3]. With effects.belt_mass off the two
  agree to rounding (1e-16 N m s), the linear momentum and the first moment with the belt too. The belt's angular
  momentum stays counted differently (section 4's row: the file at the runs' drawn places, the crate by its path):
  1.3e-3 N m s about y and 1.1e-3 about x at up to 0.25 m/s. Test `the_t_gantry_file_has_its_crates_angular_momentum`
  (fails without the carriage's offset). Since 22 November the offset names its one source,
  `offset_m = [0.0, 0.0, "geometry.z_com_drop_m"]` (a point's entries may each be the path of a number, as a number
  field's; the crate reads the same key): with z_com_drop moved to 0.45 m crate and file still agree to 1e-16; with the
  literal 0.3 the test fails (first moment along z 3.65 against the crate's 4.25 kg m).

Fixed after the list of 17 November (in main, 20 November):
- The warm start for a motor near its limit (1FK2104-4AF on the EXCT-30) converges; a cycle that repeats
  only every second pass is named by the `thermal-cycle-repeat` verdict.
- A target the machine cannot reach is refused with the segment and the machine's own reason
  (732420e1; the linear delta at +20°: a rod would stand steeper than its joints allow).
- The least belt tension prints as `belt-least-tension`; the optimiser example reads 40 + 834.
- Stale comments: the five-bar's SP100 gearboxes, the palletiser's efficiency torque, the line module's
  loss label (c5fafd68).
- Machine files' `[screw_*]` tables carry `friction_coefficient` (3c0c73b6).
