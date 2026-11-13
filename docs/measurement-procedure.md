# Measuring a drive train for the simulator

How to record traces on a real machine so that `sim-cli identify` and `sim-cli fit` can fit inertia,
friction, stiffness, damping and the drive's delays to them. This covers what to record, which test moves,
how fast to sample, how to export the trace, and how to read the result. Publishing the result is
described in [datasets/README.md](../datasets/README.md).

Everything here was tested on synthetic traces (the simulator's, and a motor model with LuGre friction,
encoder quantisation, noise and a current-loop lag) and on files built after the vendors' documented or
observed export layouts. No trace of a real drive has been read yet. Treat the first real machine as a
test of this procedure, and report what did not fit.

## 1. Safety first

The test moves run the axis without the machine's usual sequence and some of them with loops detuned.

- Run them only with the safety functions active (STO / SS1, limit switches, software limits set inside
  the free travel). Keep a hand on the stop.
- Start every move at a tenth of its speed and torque and work up.
- A vertical axis: never lower the speed-loop gain or step the torque on it without a counterweight or the
  brake's holding margin checked; the friction and step tests are for horizontal axes or with the load
  removed.
- The torque step and the noise excitation can excite the belt. Watch the first ones and stop at any
  growing oscillation.

## 2. What to record

Per motor, at the motor shaft. The names are the usual ones; the mapping file (step 5) tells the importer
which is which and what unit each has.

| Signal | Needed for | SINAMICS S120 / S210 (Startdrive trace) | TwinCAT 3 (Scope) | CODESYS SoftMotion (trace) |
| --- | --- | --- | --- | --- |
| Position setpoint | replay, position-loop lag | r2665 (LU) | `NcToPlc.SetPos` (user unit) | `fSetPosition` (u) |
| Position actual | all | r2521 (LU), or r482 (raw encoder count, wraps) | `NcToPlc.ActPos` | `fActPosition` |
| Speed actual | all (optional: the slope of the position) | r61 (1/min, unsmoothed) | `NcToPlc.ActVelo` | `fActVelocity` |
| Torque actual | all | r80 (N m) | the drive's process data, e.g. CiA 402 0x6077 (‰ of 0x6076) | `fActTorque` (N m at the gearbox output) |
| Torque setpoint | current-loop lag | r79 (N m) | e.g. 0x6071 target torque, or the drive's internal setpoint | `fSetTorque` |

Notes:

- The torque actual is what the drive estimates from the current (k_T × I_q); that is what the fit takes as
  the torque at the shaft. Torque ripple and the k_T's temperature drift show up as friction noise.
- The TwinCAT NC axis has no torque in `NcToPlc`; add the drive's own torque value to the Scope.
- CODESYS SoftMotion gives the torque at the gearbox output; set `scale = 1 / ratio` in the mapping to refer it
  to the motor.
- Positions in LU, increments or mm need `per_rev` in the mapping: units per motor revolution. For SINAMICS,
  LU per motor turn = p2506 × p2505 / p2504.
- A raw encoder count (r482, a 32-bit counter) wraps; give `wrap = 4294967296`.

## 3. How fast to sample

| What | Sample time | Why |
| --- | --- | --- |
| Inertia, friction, cycle replay | 1 ms (the position cycle) | The fit averages over ±5 ms. |
| Position-loop lag | the position cycle | |
| Current-loop lag | the current or speed cycle (62.5 to 250 µs) | The lag is a few cycles long; at 1 ms only its dead time is resolved, and a shorter time constant is reported as 0 ± one row. |
| Torque step (ringing) | ≤ 1/20 of the ringing period: 125 µs for 400 Hz | The damped sine is fitted per row. |
| Frequency response from a trace | the current-loop cycle, 125 µs or faster | At 1 ms the hold of the torque moves the dip by 10 % at 125 Hz (tested). |
| Slow reversal (presliding) | 1 ms or faster, slow enough to give four rows in the swing (below) | |

A trace buffer is limited: record the long, slow tests at 1 ms and the short, fast ones at the fast rate,
as separate recordings.

## 4. The test moves

Each test is one recording. The names are the `test` values of a dataset.

### inertia-sines: inertia and viscous friction

A speed setpoint that is a sum of two sines, e.g. 50 % of rated speed at 1 Hz plus 15 % at 3.7 Hz, for 4 s,
within the travel. Accelerations of 30 to 60 % of the axis's limit. It has to turn the motor both ways:
one direction alone cannot tell Coulomb friction from a constant torque. Position control with the
machine's normal gains is fine.

### friction-map: Coulomb, viscous, Stribeck

Constant speeds both ways, each held at least 0.5 s after the speed has settled: e.g. ±0.25, ±0.4,
±0.6, ±1, ±1.6 and ±2.5 % of rated speed, then ±5, ±10, ±25 and ±50 %. The plateaus at low speed are where
the Stribeck drop is; the fit keeps it only where the trace shows it clearly above its noise, and reports
the breakaway torque (`stiction_nm`) and its speed (`stribeck_speed_rad_s`) then. Moving fast through low
speeds does not do: a passage of a few ms shows a lower peak than the steady curve (LuGre's frictional lag;
in the tests the sines alone gave 0.067 N m for a true 0.080 N m, with the plateaus 0.077 N m).

Record it warm (after 20 minutes of the cycle) and, if friction matters, cold as well: the viscous part of
grease changes by tens of percent.

### slow-reversal: presliding (LuGre σ0)

A position sine of small speed through zero: e.g. 0.3 rad/s amplitude at 0.5 Hz at the motor, for 6 s.
After each reversal the friction swings from one sliding level to the other over the presliding angle
Tc / σ0 (µrad to mrad at the motor). The fit uses a reversal only if the swing ends while the speed stays
below 1 % of the top speed of the trace and spans at least four rows, so the reversal has to be slow:
an acceleration through zero speed below about 2 Tc / (σ0 (10 h)²) for the sample time h, which lets
the presliding angle take ten rows. A 20-bit encoder or better. Expect
an error of ±30 %; σ0 is a rough number from any drive trace.

### torque-step: stiffness and damping from the ringing

The speed loop's gain lowered as far as is safe (or torque control), a torque step of 10 to 30 % of rated
torque on a free axis, recorded at the fast rate for 0.1 to 0.3 s. The motor speed rings at the resonance
of motor against load. `identify --step-from T --step-to T` fits a damped sine on a quadratic trend
(the rigid acceleration) and reports its frequency and damping ratio; with `--motor-inertia J` (the
rotor's inertia from the data sheet plus what sits on the motor side of the belt or coupling) and the total
inertia from the same trace, also stiffness and damping of the two-mass model. A step with the speed loop
at full gain shows the closed loop, not the mechanics.

### noise-excitation or frequency-response: the two-mass model

Best: the drive's own frequency-response measurement of the mechanics (motor torque to motor speed),
exported as a table of frequency and gain. Converted to the columns `frequency_hz` and
`gain_rad_s_per_nm`, `identify --sweep FILE` fits it. Otherwise record a trace with a noise (PRBS) torque
added to the setpoint, at the current-loop rate, for 10 to 30 s; `identify --from HZ --to HZ` computes the
response torque to speed from it (Welch, Hann windows; the trace has to hold eight windows of four periods
of the lowest frequency, so 30 s for 1 Hz) and fits the model. The `response_lowest_coherence`
row says how clean the response is: below about 0.8 the band holds mostly noise, or a loop is
correlating the torque with the speed (lower the speed-loop gain).

The model: motor and load inertia joined by a spring with damping, and a viscous friction at the motor. The
response must hold the dip (anti-resonance), a peak above it, and at least three points below a third of
the dip. The total inertia comes from the part below the dip and should agree with the inertia of the
inertia-sines test; if it is clearly higher, friction is still bending the response at low frequencies
(start the response higher, or excite harder).

### Drive delays and filters (stage 3)

Recorded with any of the moves above. `identify` fits gain × first-order lag × dead time from the torque
setpoint to the torque (the current loop and its filters: needs the torque setpoint in the mapping, and the
fast rate) and from the position setpoint to the position (the position loop as a whole). The setpoint is
taken as held from row to row, as a drive holds it. A dead time below zero means the output leads the
setpoint: the two were recorded on different clocks or one is logged a cycle late. Correct the clock with
`time_offset_s` in the mapping, or note the offset. With velocity and torque feedforward the position follows
its setpoint so closely that only a dead time is left; that is the expected result.

### cycle: replay

The machine's real cycle without payload changes, for `sim-cli replay` and `fit`, which compare the trace
with the simulator's config of the machine. The config's start pose has to be where the trace starts.

### swing: a delta, a SCARA or an arm

Where the motors do not turn in proportion to the axes (the delta's arms), or the inertia a motor turns changes
with the pose (a SCARA's or an arm's elbow), `fit` regresses the motor torques on the machine's own rigid model
(τ = Y(q, q̇, q̈) θ, in the motors' coordinates) and fits the payload's mass (a factor), and per motor an added
inertia, viscous and Coulomb friction and a constant torque. The same cycle as above does, but better is a
swing: every axis about the start pose with its own amplitude and frequency (on the delta 0.2 m across at 0.6
and 0.9 Hz and 0.06 m up and down at 1.1 Hz; on the SCARA 0.8, 0.35 and 0.8 rad and 40 mm at 0.5 to 1.1 Hz),
faded in and out over a second, 8 s, with the payload held. Over much of the range, so that gravity changes
with the pose; at several frequencies, so that inertia and friction are told apart. The values the trace cannot
tell apart (their estimates correlate by more than 0.95) are named on the error stream: only their sum is
known. A rotor and its arm's own inertia about the joint are always one value.

### moves: belt stiffness without a sweep

`fit --trace FILE --stiffness` fits the belts' stiffness (`belt.stiffness_ea_n`, or the spring named with
`--stiffness-key`) to how the trace's own moves ring: the config is replayed with the stiffness times a factor,
and the factor is the one whose motor torques, less their mean over ±10 ms, match the trace's. The config's
loops, filters and friction damp the ringing in the replay as they do on the machine, so a cycle or a few
ordinary moves at the machine's limits do; the masses have to be right first. What rings the belt is the jerk
at the ramps and, at speed, the motor's ripple and cogging sweeping through the mode. The fit is refused
(`fit-move-not-excited`) where the factor's standard error is above a tenth of it, or the best factor lies at an
end of 0.25 to 4: on the single axis a move of 2 mm at 1 m/s³ and 0.2 m/s² is refused, two moves of 0.8 m at the
config's limits give 0.708 ± 0.015 for a belt of 0.7. A sweep stays the better measurement where the drive
offers it.

## 5. Exporting and mapping

| Format | Export | Read as | What is confirmed |
| --- | --- | --- | --- |
| `sinamics` | Startdrive / TIA Portal trace: "Export measurement" as CSV | A table: the line before the first line of numbers holds the names, with units in `[..]` or `(..)`, or a line of units under it; `;`, `,` or tab; decimal comma where the separator is not a comma. | Siemens documents only that the export exists, not its layout. The fixture is an assumption. The native `.ttrecx` (a ZIP of XML and binary, known only from a reverse-engineered reader) is not read. |
| `twincat` | TwinCAT 3 Scope View, export to CSV, full header | The header block of key/value pairs per channel (`Name`, `SymbolName`, `SampleTime[ms]`, `Unit`, ...), then a time and a value column per channel, in ms from the start (or FILETIME stamps). Channels of different sample times, padded cells and the `EOF` line are handled. | The layout of real exports of Scope 3.4 (the test data of the pytcs project). The values are taken as exported; whether Scope applies `ScaleFactor` and `Offset` before export is not confirmed, so check one value against the drive. The `.svdx` is not documented and not read. |
| `codesys` | CODESYS trace: "Save trace" as `.trace`, `.txt` or `.trace.csv` | `.trace`: UTF-16 XML, `Values` and `Timestamps` per variable, µs where `MicroSeconds` is true, else ms. `.txt`: one time column with its unit in the header. `.trace.csv`: a data block per variable. | Real exports of a CODESYS V3.5 SP20 based IDE (MOVIRUN). In the `.trace.csv`, µs is taken from `Flags` having bit 32 set; that was seen in one export and is not documented. |
| `csv` | anything else | as `sinamics`, with `time` naming the time column | |

Copy [datasets/template/map.toml](../datasets/template/map.toml) and keep the block of your format. The
mapping's motors are in the order of the machine's motors where it is replayed or fitted against a config.
All signals are resampled (linearly) onto one grid, the step of the first motor's position or
`sample_time_s`, over the time all of them cover. `time_unit` overrides the unit of a `csv` or `sinamics` time column (TwinCAT and CODESYS
exports state theirs, and the mapping may not). A misspelt key is refused, and so is a unit the importer
does not know unless `scale` says how to make N m of it.

Then:

```text
sim-cli identify --trace trace.csv --map map.toml --out report.csv --overlay overlay.csv
sim-cli identify --trace step.csv --map map.toml --step-from 0.05 --step-to 0.25 --motor-inertia 3.6e-5
sim-cli identify --sweep response.csv
sim-cli fit MACHINE --trace cycle.csv --map map.toml
sim-cli fit delta-d3 --trace swing.csv --map map.toml
sim-cli fit MACHINE --trace cycle.csv --map map.toml --stiffness
```

`--motor N` picks the motor of a multi-motor trace for `identify`.

## 6. Reading the report

One line per quantity: value, standard error, and the 95 % interval (value ± 1.96 standard errors):

```text
quantity,of,value,standard_error,ci95_low,ci95_high
inertia_kg_m2,m,1.049089e-4,1.165e-7,1.046806e-4,1.051373e-4
coulomb_nm,m,8.483956e-2,3.080e-4,8.423579e-2,8.544332e-2
friction_model_fit_percent,m,9.839305e1,,,
```

- The standard errors come from the least-squares covariance, widened for residuals that are correlated
  from row to row. They hold if the model is right. Where it is not (a friction that is not Stribeck, a
  second resonance), the error is larger than stated: compare the fits of two recordings of the same test.
  On synthetic traces of the static friction model, over 12 noise seeds, the scatter of viscous and
  Coulomb friction and the constant torque matched their standard errors (RMS of the z-scores below 2);
  on traces with LuGre friction and loops the values lay within 5 standard errors, or a few percent, of the
  true ones.
- `*_fit_percent` is how well the model reproduces the trace: 100 % × (1 − |measured − model| /
  |measured − mean|). Above 90 % for the friction model on the smoothed torque is good. Lower: look at the
  overlay; a periodic leftover is cogging or ripple, a leftover that follows the position is a load that
  changes along the travel.
- `*_following_error_fit_percent` is the same for setpoint − output, which says more where the output
  follows closely. It goes negative where the following error is mostly noise.
- A lag time constant of 0 ± one row means the trace cannot resolve it: record faster.
- No line for stiction, Stribeck speed or presliding stiffness: the trace did not show them clearly (see
  the tests above).

What the values are in the simulator's config: the total inertia is the rotor plus everything reflected to
the motor (`mass.*`, pulley and gearbox inertia); the friction values are the sum of the guide, the gearbox
and the motor bearings at the motor shaft. `sim-cli fit MACHINE` splits them by axis against a config.

`fit` writes the config's value, the fitted one and its standard error side by side, the config's from the
same estimate on its own replay of the trace, so that what the estimate gets wrong on both sides drops out. On a
delta, SCARA or arm the rows are `payload_factor` (1 for the config) and per motor `added_inertia_kg_m2`
(near 0 for the config), `viscous_friction_nm_s_rad`, `coulomb_friction_nm` and `constant_torque_nm`, at the
motor shaft with the gearbox's losses in them; the advice says the change. Strain wave gears lose a quarter of
what passes them, and more on a heavier payload: that shows as more Coulomb friction on the joint that carries
it, and cannot be told from it. `--stiffness` adds `stiffness_factor` with its standard error and the RMS of
the ringing in the trace and of what the fitted replay leaves of it (the noise, where the fit is good).

## 7. What it does not do

- It does not tell where friction or inertia sit (guide, gearbox, motor): one motor's torque sees the sum.
- Backlash, a second resonance, or a stiffness that changes along the travel (a belt's free length) are not
  in the two-mass model; fit near the position the machine works at most.
- The friction model is static plus presliding; frictional lag at fast reversals is not fitted.
- The importers have been tested on files built after the vendors' layouts, not on files from real drives
  of this project. Expect the first real export to need a fix; please send it (anonymised) with the issue.
