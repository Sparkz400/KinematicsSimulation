# Examples

One small example per task of the [user guide](../docs/user-guide.md), each with the command that runs it and what to
look at in its output. Run them from the repository's root, with `sim-cli` built (`cargo build --release -p sim-cli`,
then `target/release/sim-cli`, or `cargo run -q --release -p sim-cli -- ...` in place of `sim-cli`). Every `sim-cli`
command in a `bash` block below is run by the test `crates/sim-cli/tests/examples.rs`, and the Python script by
`crates/sim-py/tests/test_examples.py`, so they keep working as the program changes.

Where a task needs a whole config, the example takes a shipped one and changes it with `--set`: a config file has to be
complete (some 700 lines, [config reference](../docs/config-reference.md#rules)), and a copy here would fall behind the
shipped one.

The first part is a gallery: one recipe per question users ask, with the output it printed on 16 November 2026 (wide
CSV cut to its first columns, marked `...`). The second part, from [A path on two axes](#a-path-on-two-axes), shows
each input format.

- [Will this motor do it?](#will-this-motor-do-it)
- [Which is the cheapest motor that passes?](#which-is-the-cheapest-motor-that-passes)
- [How fast can this path go?](#how-fast-can-this-path-go)
- [What happens in an E-stop?](#what-happens-in-an-e-stop)
- [Where does the energy go?](#where-does-the-energy-go)
- [Does input shaping help here?](#does-input-shaping-help-here)
- [Fit my machine from a drive trace](#fit-my-machine-from-a-drive-trace)
- [Drive the sim from my own controller (Python / FMU)](#drive-the-sim-from-my-own-controller-python--fmu)

## Will this motor do it?

The T-gantry's default cycle with a Siemens 1FK2104-5AK in place of the config's motor, and the converter the catalogue
pairs with it (16 s):

```bash
sim-cli verdicts t-gantry --motor 1FK2104-5AK
```

```text
Thermal start: warm, at the rises the cycle settles at; largest change of a rise over the measured pass 0.000 K
check,motor,value,limit,result
motor-peak,a,0.07979,1.000,ok
motor-thermal,a,4.528,100.0,ok
motor-s1,a,0.1720,2.272,ok
motor-speed,a,200.3,837.8,ok
drive-current,a,0.3153,3.000,ok
...
settling,,0.00004352,0.0001000,ok
belt-tension,,333.2,1500,ok
belt-slack,,205.6,0,ok
guide-load,,173.0,2000,ok
guide-life,,5168670,20000,ok
thermal-warm-start,,0.000005704,0.1000,ok
```

One row per check (and per motor where it has one): `value` against `limit`, `result` ok or FAIL. Every row is ok,
so the motor does it on the nominal parameters. `motor-peak` is the share of the peak torque used (0.08), `motor-thermal`
the winding's rise in K against its rated rise, `settling` the tool's error in the dwell in metres against the
tolerance. The nominal parameters are not the whole answer: `--bands --worst` searches the worst corner of every
`[estimate]`'s error band (minutes, thousands of cycles; [user guide](../docs/user-guide.md#see-which-unknown-parameters-matter-and-judge-with-error-bands)).

## Which is the cheapest motor that passes?

Every Siemens 1FK2 motor on the EXCT-30's cycle, each with its converter (9 s on 16 threads):

```bash
sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --series 1FK2
```

```text
Not simulated (a part the config cannot take, or a cycle that does not run; the reason is in the table): 0 / 28 candidates
Search time on 16: 7.8 s
axis,motor,gearbox,drive,vendor,supply,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k,...
,1FK2104-4AF,,6SL3210-5HE10-4UF0,siemens,400v,2.05,4.10,ok,drive-current,0.9843,3.2653,97.95,...
,1FK2104-4AK,,6SL3210-5HE11-0UF0,siemens,400v,2.05,4.10,ok,motor-thermal,0.9956,3.2633,99.56,...
,1FK2104-5AF,,6SL3210-5HE10-8UF0,siemens,400v,2.85,5.70,ok,motor-s1,0.5087,3.3377,20.42,...
,1FK2104-5AK,,6SL3210-5HE11-5UF0,siemens,400v,2.85,5.70,ok,motor-s1,0.5249,3.3374,22.60,...
...
```

Passing candidates first, cheapest first. No catalogue has prices, so `cost_kg` (the installed mass of motors and
gearboxes, both motors here) stands in. The first row is the answer on paper, but its `tightest_utilisation` is 0.98
(the drive's RMS current): 2 % from failing, with nothing for an estimate that is off. The 1FK2104-5AF, 0.8 kg more,
leaves half of its tightest rating free. Leave out `--series` for the whole catalogue, add
`--robust N` to judge each candidate on N samples of the error bands, `--rank-by margin` to sort by reserve.

## How fast can this path go?

The corner of [`corner.path`](corner.path) on the T-gantry, first within the configured axis limits, then as fast as
the motors and belts allow (5 s each):

```bash
sim-cli analyze t-gantry --path examples/corner.path
sim-cli analyze t-gantry --path examples/corner.path --time-optimal
```

```text
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,...,path_error_peak_m,...
a,2.128000,0.552014,0.189860,200.4058,0.1459,0.1495,...,1.268e-4,...
b,2.128000,0.298401,0.090958,200.4554,0.0787,0.0716,...,1.268e-4,...

motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,...,path_error_peak_m,...
a,1.658000,0.719198,0.239475,394.3955,0.1896,0.1886,...,1.702e-4,...
b,1.658000,0.391303,0.153789,394.6258,0.1032,0.1211,...,1.702e-4,...
```

`cycle_s` is the path plus 0.3 s of rest: 1.83 s of motion become 1.36 s. The motors use only 0.19 of their peak
torque in the faster plan, because the axes' jerk limit (100 m/s³ by default) decides nearly everywhere; with
`--set axis.x.max_jerk_m_s3=...` (and `axis.z`) raised the path gets faster still, until a motor or belt rating binds. The
path error grows from 0.13 to 0.17 mm: check it against your tolerance. A time-optimal plan keeps every rating that
holds at a point of the path; it does not know the winding's warming over the cycle, so run `verdicts` on it as well.

## What happens in an E-stop?

A category 0 stop (the torque off at once) of the T-gantry's cycle at three moments of a grid and where
it is worst for the energy, the X speed and the Z speed (10 s):

```bash
sim-cli stop t-gantry --points 3
```

```text
moment,trigger[s],tool_distance[m],stop_time[s],peak_deceleration[m/s2],hard_stop,peak_dc_link[V],braking_resistor_energy[J],tripped,x_distance[m],z_distance[m]
grid,0.0000,0.000001,0.0000,0.218,false,155.14,0.0000,false,0.000000,0.000001
grid,2.0263,0.009207,0.9760,166.304,false,155.24,0.0000,false,0.009091,0.001556
energy,2.0800,0.009321,0.9762,164.064,false,159.78,0.0000,false,0.009208,0.001565
grid,4.0527,0.000119,0.0148,34.901,false,224.06,0.0000,false,0.000070,0.000118
fastest_z,4.3240,0.001384,0.1861,102.106,false,175.70,0.0000,false,0.000412,0.001384
fastest_x,5.0750,0.008226,0.9896,154.088,false,155.15,0.0000,false,0.008181,0.000934
worst,2.0800,0.009321,0.9762,164.064,false,159.78,0.0000,false,0.009208,0.001565
Worst stop: triggered at 2.0800 s (energy)
Stop verdict: stop-dc-link: 224.1 / 257.6 ok
```

`tool_distance` is how far the tool travels after the trigger: 9.3 mm at worst, in 0.98 s, with no end stop hit
(`hard_stop` false) and the DC link at most 224 V against its 258 V trip level. `z_distance` is the hanging axis's
fall. `--category 1` brakes on the drives' quick-stop ramp first, `--category 2` brakes and holds; `--set
effects.brake_delay=true` adds the brakes' engage time (the worst stop then takes 30.7 mm, [user
guide](../docs/user-guide.md)).

## Where does the energy go?

The T-gantry's default cycle with its energy budget written to a file (22 s; `target/` keeps it out of the
repository):

```bash
sim-cli analyze t-gantry --budget target/budget.csv
```

`target/budget.csv`:

```text
item,energy_j,share
supply,1.425558e2,1.000000
copper.a,3.333986e0,0.023387
copper.b,2.858302e0,0.020050
resistor,5.482817e0,0.038461
discharge,0.000000e0,0.000000
line_module,2.851117e0,0.020000
power.guide_x,5.101350e1,0.357849
power.guide_z,1.915052e1,0.134337
...
power.shaft_a,2.454118e1,0.172151
power.shaft_b,2.454122e1,0.172152
power.gearbox_a,4.436623e0,0.031122
power.gearbox_b,4.347728e0,0.030498
power.belt,2.745834e-3,0.000019
...
unaccounted,-4.268458e-4,-0.000003
```

143 J a cycle from the supply: half goes into guide friction (36 % X, 13 % Z), a third into the motor shafts' friction,
6 % into the gearboxes, 4 % into the windings' copper, 4 % into the braking resistor and 2 % into the line module. The
rows add up to the supply; `unaccounted` (0.0003 %) is the integration's error. The [user
guide](../docs/user-guide.md) names every row. `--breakdown FILE` splits the motor torque instead (inertia, gravity,
friction, rest).

## Does input shaping help here?

The EXCT-30 ships with an input shaper at 25 Hz (and a second at 17.5 Hz). Its cycle with the shaper and without
(5 s each):

```bash
sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml --set effects.input_shaping=true
sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml --set effects.input_shaping=false
```

```text
motor,cycle_s,peak_torque_nm,...,utilisation,...,path_error_peak_m,...,tracking_error_peak_m,dwell_error_peak_m,...
a,2.048000,3.655658,...,0.2927,...,4.393e-4,...,5.120e-4,3.477e-5,...
b,2.048000,3.030811,...,0.2427,...,4.393e-4,...,5.120e-4,3.477e-5,...

motor,cycle_s,peak_torque_nm,...,utilisation,...,path_error_peak_m,...,tracking_error_peak_m,dwell_error_peak_m,...
a,1.852000,4.287499,...,0.3435,...,5.273e-4,...,5.870e-4,5.866e-5,...
b,1.852000,3.534336,...,0.2830,...,5.273e-4,...,5.870e-4,5.866e-5,...
```

`dwell_error_peak_m`, the tool's swing while it should stand still, falls from 0.059 to 0.035 mm with the shaper, and
the peak torque by 15 %; the price is 0.196 s of cycle, since every stop waits for the shaper. On nominal parameters
both pass the 0.1 mm tolerance, so the shaper is not needed there: it was switched on, with a higher belt pretension and a
lower jerk limit, so that `settling` passes at the worst corner of the error bands. On the default T-gantry the same switch helps nothing (0.0434 against
0.0429 mm) and costs 0.076 s. To tune one: `sim-cli modes MACHINE` lists the resonances over the stroke, and
`profile.shaper_frequency_hz` goes to the one that rings ([user
guide](../docs/user-guide.md#tune-gains-and-use-input-shaping)).

## Fit my machine from a drive trace

A trace of the single belt axis (positions as encoder counts, speed in 1/min; the map file says which column is which)
fitted for moved mass, friction and a constant force (5 s):

```bash
sim-cli fit single-axis --trace datasets/example-synthetic-belt-axis/trace.csv --map datasets/example-synthetic-belt-axis/map.toml --set effects.payload=false
```

```text
quantity,of,config,fitted,standard_error
moved_mass_kg,x,6.548784e0,6.548736e0,8.629e-3
viscous_friction_n_s_m,x,2.674104e1,2.678076e1,1.161e-1
coulomb_friction_n,x,2.138412e1,2.134337e1,1.042e-1
constant_force_n,x,5.175676e-5,3.359007e-3,3.232e-2
presliding_stiffness_n_m,x,4.693015e5,4.880754e5,8.456e4
Moved mass, axis x: -0.000 kg ± 0.009. Add this to the masses that move with the axis. The trace cannot tell load mass from rotor and pulley inertia.
Viscous friction, axis x: × 1.001 ± 0.004. Scale the viscous friction of the axis's guide and of the motor shafts by this factor; the trace cannot tell the two apart.
Coulomb friction, axis x: × 0.998 ± 0.005. Scale the Coulomb friction and stiction of the axis's guide and of the motor shafts by this factor; the trace cannot tell the two apart.
Constant force, axis x: +0.003 N ± 0.032. The measured axis needs this much more than the config gives: a weight, an incline or a cable pull the config lacks.
```

The trace was made by the simulator, so the fit finds the config again: each value within about one standard error of
it. On your machine's trace, the sentences say what to change in the config and by how much; a correction several
standard errors from zero is real, one within them is not. Record a trace by
[docs/measurement-procedure.md](../docs/measurement-procedure.md), write its `map.toml` after
[`datasets/template`](../datasets/template/), and check it first with `sim-cli replay` (below: [A drive trace with
its map file](#a-drive-trace-with-its-map-file)). `identify` reads resonances and inertia from a trace as well.

## Drive the sim from my own controller (Python / FMU)

**Python.** [`crates/sim-py/examples/own_controller.py`](../crates/sim-py/examples/own_controller.py) measures the
inertia at the motor of the single axis, tunes a PD for 20 Hz on it, runs it at 1 kHz through the drive's torque mode
on the move the built-in cascade runs, and judges both runs with the same checks. Needs the module `kinematics_sim`
([docs/python.md](../docs/python.md)):

```sh
python crates/sim-py/examples/own_controller.py
```

```text
single-axis: inertia at the motor 0.0001344 kg m^2
built-in cascade: peak torque 0.389 N m, RMS 0.174 N m, RMS current 0.386 A, failing checks: none
own PD:           peak torque 0.407 N m, RMS 0.168 N m, RMS current 0.373 A, failing checks: none; 1.75e-02 rad from the target at the end
```

Your controller against the drive's: the PD needs about the cascade's RMS torque (0.168 against 0.174 N m) and passes
every check, but, having no integrator, stops 0.0175 rad short of the target against the guides' friction: add an I
part and run it again. The verdicts judge your run as they judge the built-in drive's.

**FMU.** The same in any FMI 2.0 importer: [`fmu_controller.py`](fmu_controller.py) drives the single axis's FMU with
fmpy, the motor in torque mode (`command.mode.m = "torque"`, the torque in `command.torque.m`), a PD at 1 ms on a
20 rad move:

```sh
sim-cli fmu single-axis --out target/single-axis.fmu
python examples/fmu_controller.py target/single-axis.fmu
```

```text
inertia at the motor 0.0001344 kg m^2, kp 2.123 N m/rad, kd 0.03379 N m s/rad
time_s,setpoint_rad,angle_rad,error_rad,current_a
0.100,1.8169,1.7177,9.92e-02,0.477
0.200,10.0000,9.9263,7.37e-02,0.309
0.300,18.1831,18.1775,5.59e-03,0.026
0.400,20.0000,19.9672,3.28e-02,0.163
0.500,20.0000,19.9663,3.37e-02,0.159
0.600,20.0000,19.9663,3.37e-02,0.159
peak following error 1.048e-01 rad
```

The script measures the inertia at the motor with two torques (the same 1.344e-4 kg m² as the Python module), tunes
the PD for 20 Hz and runs the move: 0.1 rad (0.4 mm at the carriage) behind while it accelerates, and 0.034 rad short
at rest, where 0.159 A still push against the friction a PD without integrator cannot overcome. The current is the
winding's, through the drive's current loop. In Simulink, Dymola or OpenModelica the same two
inputs do it ([docs/fmi.md](../docs/fmi.md#your-own-controller-torque-and-velocity-mode)). These two run in
`crates/sim-py/tests/test_examples.py`, not in the `sim-cli` test (an `sh` block).

## A path on two axes

[`corner.path`](corner.path): line, arc, line on the T-gantry, the first path printed in the [overview](../docs/overview.md).

```bash
sim-cli analyze t-gantry --path examples/corner.path
```

Look at `cycle_s` (the path plus 0.3 s of rest) and `path_error_peak_m`, the tool tip's largest distance from the path.

[`curve.path`](curve.path): a line, a spline through two via points, a line.

```bash
sim-cli analyze t-gantry --path examples/curve.path
```

The same columns; the curve is slow where it joins the lines, because the curvature steps there.

## A path on three axes

[`delta-3d.path`](delta-3d.path): the delta picker D3 goes down, round a quarter arc in the vertical plane, across and
through a curve in space.

```bash
sim-cli analyze delta-d3 --path examples/delta-3d.path
```

One row per arm motor: `peak_torque_nm` and `utilisation` show which arm the path loads most.

## A G-code program

[`pocket.nc`](pocket.nc): the corner of `corner.path` in G-code (G18 plane, mm, feed per minute), read by its extension.

```bash
sim-cli analyze t-gantry --path examples/pocket.nc
```

`cycle_s` follows from the feeds: F is the speed along the path, with a stop between moves at different feeds.

## A trajectory file

[`move.traj`](move.traj): poses at times on the single belt axis; the controller runs a cubic curve through them.

```bash
sim-cli analyze single-axis --trajectory examples/move.traj
```

`tracking_error_peak_m` is how far the tool lags the poses at their times.

## A cam table

[`press-cam.csv`](press-cam.csv): a servo press cam for the slider-crank, slave = master + 0.6 sin(master), in place of
the config's cam segments.

```bash
sim-cli analyze slider-crank --set cam.run=true --set 'cam.table_file="examples/press-cam.csv"'
```

`peak_speed_rad_s` is about 100.5: the slope's peak 1.6 times the master's pi rad/s (30 strokes a minute) times the
gearbox's 20.

## Obstacles and a crash

A wall in the single axis's way, given with `--set obstacles=...` (in a config file it is the key `obstacles`), with the
crash and the drives' monitoring on.

```bash
sim-cli run single-axis --move 0.9 --time 1.5 --every 100 --set effects.crash=true --set effects.drive_monitoring=true --set 'obstacles=[{name="wall",centre_m=[0.75,-0.03,0.06],size_m=[0.1,0.2,0.2]}]'
```

The lines on the error stream: the planned clearance, the first contact (0.47 s), the contact force and how the drives
tripped and stopped. The signals go to the standard output as CSV (`--out FILE` writes them to a file).

## Pick from a moving conveyor

The shipped conveyor config of the planar delta D2: a part a second on a belt at 0.3 m/s.

```bash
sim-cli analyze delta-d2 --config crates/machine-delta-d2/conveyor.toml
```

The line `Conveyor:` says the picks a minute (60, all the belt brings), the tracking error during the grip and the parts
missed (0). Add `--set conveyor.pitch_s=0.5` and the delta misses parts.

## A duty mix for the motor search

The D2's own cycle three times to one conveyor shift, every 1FK2105 motor with its drive.

```bash
sim-cli search delta-d2 --config crates/machine-delta-d2/default.toml --config crates/machine-delta-d2/conveyor.toml --weights 3,1 --series 1FK2105
```

One row per candidate, passing and cheapest first: `thermal_utilisation` is the heat over the mix, the peaks the worst
of the two cycles. Leave out `--series` for every motor of the catalogue (some 40 s).

## A drive trace with its map file

The synthetic trace of [`datasets/example-synthetic-belt-axis`](../datasets/example-synthetic-belt-axis/): positions as
encoder counts, speed in 1/min; its `map.toml` says which column is which and how to convert it.

```bash
sim-cli replay single-axis --trace datasets/example-synthetic-belt-axis/trace.csv --map datasets/example-synthetic-belt-axis/map.toml --set effects.payload=false
```

Per motor the RMS and peak difference between trace and simulation, in torque and following error; small here, since
the trace was made by the simulator. `fit` and `identify` take the same two options.

## A `--set` sweep in Python

[`payload_sweep.py`](payload_sweep.py): the single axis at three payloads with the module `kinematics_sim`
([docs/python.md](../docs/python.md)).

```bash
python examples/payload_sweep.py
```

One CSV row per payload: peak and RMS torque rise with it, and `all_checks_pass` says whether every check passes.

## A task on several kinematics

[`pick-place-task.toml`](pick-place-task.toml) is the task file of `sim-cli compare-kinematics` (user guide, "Which
kinematics for a task"). It runs a motor search per machine, minutes in all, so the test leaves it out.
