# Gantry physics simulator

A simulator of belt-driven gantries in which torque, speed and following error come out of a physical model: belt sections as springs, LuGre guide friction, a gearbox with backlash, a motor winding and a cascaded servo drive.
It has two machines, a two-motor T-gantry (X and Z on one belt) and a single belt axis, behind four front ends: a desktop app with a 3D view and scopes, a command-line tool that writes CSV, an OPC UA server for a PLC, and an FMI 2.0 Co-Simulation export for Simulink, OpenModelica, Dymola and other tools.
It is written in Rust; vendor data for motors, gearboxes, drives and belt axes is in [`catalog/`](catalog/).
Deutsche Fassung: [docs/de/README.md](docs/de/README.md) und [Benutzerhandbuch](docs/de/benutzerhandbuch.md).

## What it can and cannot tell you

It can tell you, for a machine described in a config file:

- peak, RMS and cubic-mean torque, speed, current, winding temperature rise and energy per motor over a cycle or a path
- how much of each motor's torque goes to inertia, gravity, friction and the belt, where the energy of a cycle is lost (each guide, shaft, gearbox, damper, winding, the braking resistor), and what each guide's friction does at its operating point
- pass or fail of that cycle against the ratings in the config (torque-speed curve, winding temperature, gearbox, belt tension, slack and tooth load, DC link and braking resistor, guide and shaft load, settling at the pick)
- how many hours the pulley shaft bearings, the gearbox's planet bearings and the guides last on that cycle, against a required life (ISO 281, Siemens' gearbox formula, the guide makers' formula; the belt has a tooth load rating instead, because no belt maker publishes a life)
- how those results move when an uncertain value is at its low or high bound, or when all uncertain values are off at once
- how far the tool tip is from its path, and what gains, corner blending and input shaping do to that
- which catalogue motor with its drive (and gearbox), on which catalogue axis, is the cheapest that passes, with the installed weight standing in for the price

What stands behind the numbers:

| Checked against | What | Where |
| --- | --- | --- |
| Hand calculations | Every component and every analysis figure has a test that compares it with a number worked out by hand: belt frequency, friction levels, backlash, cycle torque, winding temperature, DC link voltage, guide and shaft loads, path error, and more | [docs/validation.md](docs/validation.md) |
| A SciPy reference | One open-loop move of the elastic T-gantry against a second implementation of the same equations in Python, integrated with LSODA. Agreement within 0.2 % of the travel. The reference has no leaning Z axis and predates the later effects | [docs/validation.md](docs/validation.md#independent-reference) |
| Vendor formulas | Festo's own torque formula for the EXCT-30 at 45 degrees travel, within 2 % | `exct_30_config_agrees_with_the_catalogue` |
| Cross-checks between vendor documents | The same Siemens motors read from two documents by two scripts must agree; catalogue values must fit their own physics | [docs/catalogue.md](docs/catalogue.md) |

These checks prove that the equations are solved correctly and that the catalogue data was read correctly. They do not prove that the parameters describe a real machine.

**No real machine has been measured.** Friction, damping, guide and frame stiffness, belt pretension, winding resistance and inductance, DC link and braking resistor data are estimates. The configs say so: [`exct-30.toml`](crates/machine-t-gantry/exct-30.toml) marks every value `[catalogue]` or `[estimate]`, and each config has a `[bounds]` table with a low and a high value for its estimates. Trends and comparisons (this motor against that one, with and without shaping) are sound. Absolute numbers are as good as the estimates. Use `verdicts --bands` to see how far a result moves with them.

Other limits that matter:

- The default configs of both machines are made-up placeholder machines, not products.
- The motor is one equivalent winding (R, L, back-EMF) that carries the RMS phase current and gets at most sqrt(3/2) times the DC link voltage. There is no three-phase model, no PWM, no field weakening.
- The life figures are the makers' formulas and nothing more: no life factors, one bearing per pulley shaft, load ratings that are partly estimates (Festo publishes none for the EXCT). They are not validated.
- The OPC UA link was tested against a client written with the same library, not against a PLC.
- The FMU was tested by calling its functions from Rust and Python, not imported into Simulink, OpenModelica or Dymola, and not checked with an independent FMI tool.
- Mouse handling of the app (orbit camera, panels) was not checked by a person; screenshots were.
- Import of drive traces and the parameter fit were tested on traces the simulator made itself.

## Build and run

Needs a stable Rust toolchain with edition 2024 support. All dependency versions are pinned in [`Cargo.toml`](Cargo.toml). The catalogue data is compiled in, so the binaries need no data files.

```sh
cargo run --release -p sim-app                    # the desktop app
cargo run -q --release -p sim-cli -- list         # the command-line tool
cargo run --release -p sim-opcua -- t-gantry      # the OPC UA server on opc.tcp://127.0.0.1:4840/
cargo build --release -p sim-cli -p sim-fmi       # then: target/release/sim-cli fmu t-gantry --out t-gantry.fmu
cargo test --workspace                            # all tests
```

`cargo test --workspace` also builds the app with its graphics libraries. To test only the physics, the analysis and the command line:

```sh
cargo test -p sim-core -p sim-components -p machine-t-gantry -p machine-single-axis -p sim-machines -p sim-cli
```

## Five-minute tour

All commands print CSV to standard output, or to a file with `--out FILE`. Messages and errors go to the error stream. The output below is real, with columns left out where marked.

**The machines.**

```text
$ cargo run -q --release -p sim-cli -- list
t-gantry	T-gantry
single-axis	Single axis
```

**Cycle analysis of the default T-gantry.** One row per motor, measured over the second pass of the pick-and-place cycle. The first seven of 28 columns:

```text
$ cargo run -q --release -p sim-cli -- analyze t-gantry
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,...
a,6.080000,0.559623,0.165360,200.4514,0.1476,0.1302,...
b,6.080000,0.460106,0.153385,200.4378,0.1211,0.1208,...
```

**Verdicts.** The same cycle judged against the motor data and the `[ratings]` of the config:

```text
$ cargo run -q --release -p sim-cli -- verdicts t-gantry
check,motor,value,limit,result
motor-peak,a,0.1476,1.000,ok
motor-thermal,a,1.227,100.0,ok
gearbox-continuous,a,0.7955,40.00,ok
gearbox-input-speed,a,1914,13000,ok
gearbox-peak,a,2.175,64.00,ok
pulley-bearing-life,a,212521,20000,ok
gearbox-life,a,6735878139,20000,ok
belt-tooth-load,a,0.1910,1.000,ok
dc-link-energy,a,5.865,6.205,ok
dc-link-overvoltage,a,221.4,257.6,ok
motor-peak,b,0.1211,1.000,ok
...
dc-link-energy,b,6.205,6.205,ok
braking-resistor-continuous,b,0.8789,20.00,ok
braking-resistor-peak,b,32.64,500.0,ok
dc-link-overvoltage,b,224.7,257.6,ok
settling,,0.00004734,0.0001000,ok
belt-tension,,334.4,1500,ok
belt-slack,,204.6,0,ok
gearbox-radial-load,,644.8,1500,ok
guide-load,,177.8,2000,ok
guide-life,,4994954,20000,ok
```

The three lives are in hours: the bearing that takes the belt's pull on each pulley shaft (ISO 281), the gearbox's planet bearings (Siemens' formula) and the guide, each against the 20000 h of `ratings.required_life_h`. `belt-tooth-load` is the belt's tooth load over what its teeth allow at the pulley's speed (Megadyne's method): no belt maker publishes a life in hours.

The DC link is simulated (`effects.dc_link`). Motor B's braking fills the capacitors: 6.2 J of room between the DC link voltage of 155 V and the chopper's threshold of 224.65 V. That is `ok` because the config has a braking resistor, which burns the rest and is judged in the next two rows. Values are printed with four significant digits.

**A catalogue machine.** [`exct-30.toml`](crates/machine-t-gantry/exct-30.toml) is a Festo EXCT-30 with two Siemens 1FK2205-4AF motors:

```text
$ cargo run -q --release -p sim-cli -- verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml
check,motor,value,limit,result
motor-peak,a,0.2930,1.000,ok
motor-thermal,a,6.882,100.0,ok
drive-current,a,1.145,5.000,ok
drive-peak-current,a,2.874,19.00,ok
pulley-torque,a,3.126,12.50,ok
pulley-bearing-life,a,31788,20000,ok
belt-tooth-load,a,0.3529,1.000,ok
dc-link-energy,a,9.943,9.943,ok
braking-resistor-continuous,a,8.303,100.0,ok
braking-resistor-peak,a,200.5,3000,ok
dc-link-overvoltage,a,770.0,820.0,ok
...
settling,,0.00003947,0.0001000,ok
belt-tension,,555.0,2140,ok
belt-slack,,257.8,0,ok
gearbox-radial-load,,1000,2000,ok
guide-load,,185.0,2000,ok
guide-life,,3024172,20000,ok
```

The lowest belt tension in the cycle is 258 N. With `--bands` the same command runs the cycle with each bounded value at its low and at its high bound, one at a time: the slackest section keeps 158 N and the tool is at most 0.044 mm off in the dwell. With `--bands --samples 200` all bounded values lie between their bounds at once in 200 runs: 140 N and 0.051 mm at worst. With `--bands --worst` (a search for the worst corner of the bounds) it is 87 N and 0.095 mm against 0.1 mm. Every check passes in all three, but one: the pulley shaft bearings last 31800 h as written and fail 20000 h in all three modes (17300 h with the belt set 100 N high, 10200 h at the worst corner); Festo publishes no data on them, and the load rating is an estimate. An earlier state of this config failed `belt-slack` and `settling` in the worst case. For that the belt pretension was raised from 300 N to 450 N, the X jerk limit lowered from 500 to 150 m/s³, input shaping and corner blending switched on, and a second input shaper at 17.5 Hz added; the comments in the file give the reasons.

**A motor by name.** `--motor ARTICLE` puts a motor of the Siemens S210 catalogue into any config:

```text
$ cargo run -q --release -p sim-cli -- verdicts t-gantry --motor 1FK2104-5AK
check,motor,value,limit,result
motor-peak,a,0.07988,1.000,ok
motor-thermal,a,0.6971,100.0,ok
...
dc-link-energy,a,6.325,75.18,ok
dc-link-overvoltage,a,564.4,896.7,ok
...
```

The motor brings the 540 V DC link of its supply. The config's braking threshold and trip level are scaled with the DC link voltage (224.65 V and 257.6 V at 155.13 V become 782 V and 897 V), and its velocity gain is retuned for the motor's rotor. See [user guide](docs/user-guide.md#check-a-real-catalogue-machine).

**The search.** Every 1FK2 motor on the EXCT-30's cycle, each with the converter the catalogue pairs with it, passing ones first, cheapest first (no catalogue has prices; the installed weight of motors and gearboxes stands in):

```text
$ cargo run -q --release -p sim-cli -- search t-gantry --config crates/machine-t-gantry/exct-30.toml --series 1FK2
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 13 / 28 candidates
Search time on 16: 0.3 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,5.70,ok,pulley-bearing-life,0.6309,3.2886,20.44
,1FK2104-5AK,,6SL3210-5HE11-5UF0,2.85,5.70,ok,pulley-bearing-life,0.6309,3.2977,41.56
,1FK2204-5AF,,6SL3210-5HE10-8UF0,2.90,5.80,ok,pulley-bearing-life,0.6310,3.4296,21.55
...
,1FK2205-4AF,,6SL3210-5HE11-5UF0,5.20,10.40,ok,pulley-bearing-life,0.6310,3.6598,6.88
...
,1FK2204-5AK,,6SL3210-5HE11-5UF0,2.90,5.80,FAIL,motor-thermal,1.6796,3.4378,167.96
,1FK2203-2AK,,6SL3210-5HE10-4UF0,1.16,2.32,FAIL,limit-motor-torque,NaN,NaN,NaN
...
,1FK2208-3AC,,6SL3210-5HE13-5UF0,12.60,25.20,FAIL,limit-motor-speed,NaN,NaN,NaN
```

The tightest check of every passing motor is now the pulley shaft bearings' life (20000 h of 31700 h), which the motor hardly changes; `winding_rise_k` and `peak_torque_nm` tell the motors apart. `--supply s120` searches the S120 motors with their Motor Modules, `--gearboxes NRK` adds gearboxes on a machine that has one, and `--axes` tries every motor on every axis of the catalogue of the machine's kind as well (Festo EXCT and Bahr ELZI for this one: 338 candidates in 4.6 s on 16 cores; the Bahr ELZI 40 comes out lighter than the EXCT-30). A drive, a belt or an axis can be named like a motor (`--drive`, `--belt AT5-25`, `--axis ELZI-40`).

**The optimiser.** A shorter cycle that still passes every verdict:

```text
$ cargo run -q --release -p sim-cli -- optimise t-gantry --config crates/machine-t-gantry/exct-30.toml
setting,before,after
cycle_s,2.0480,1.4920
dwell_error_peak_m,3.947e-5,9.462e-5
axis.x.max_acceleration_m_s2,25.0,53.99999999999999
axis.x.max_jerk_m_s3,150.0,337.5
axis.x.max_velocity_m_s,2.5,3.0
axis.z.max_acceleration_m_s2,15.0,32.4
axis.z.max_jerk_m_s3,500.0,1125.0
axis.z.max_velocity_m_s,1.5,1.7999999999999998
effects.input_shaping,true,false
```

The optimiser judges the config as written, not its bounds. This result takes back the margins the config was given: `verdicts --bands` with these settings fails `settling` (0.29 mm) and `belt-slack` (0 N) in the worst case.

**A path file.** Lines, arcs and splines, from where the machine stands (the default T-gantry starts at 0.1, 0.35):

```text
$ cat corner.path
# line, arc, line from the start pose (0.1, 0.35)
line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.05

$ cargo run -q --release -p sim-cli -- analyze t-gantry --path corner.path
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,...
a,2.128000,0.547780,...,1.579e-4,5.575e-5,1.679e-4,...
b,2.128000,0.296779,...,1.579e-4,5.575e-5,1.679e-4,...
```

With `--time-optimal` the same path takes 1.729 s instead of 2.128 s (both include 0.3 s of rest at the end), and the path error rises from 0.16 mm to 0.19 mm. That plan takes speed and acceleration from what the motors can give instead of from the axis limits, and keeps the jerk limits of the axes. It knows the motors' limits, not the belt's; the analysis shows what it costs.

Within the axis limits the path crawls into and out of the arc at 0.07 m/s, because the acceleration towards the centre steps there. `--set profile.path_tolerance_m=0.0001` lets the planner put transition curves at both ends of the arc: the path is then off the file's by up to 0.1 mm, enters the arc at 0.40 m/s and takes 1.890 s.

Lines of `spline,x,z` that follow one another form one smooth curve through their points:

```text
$ cat curve.path
# line, a curve through two via points, line; from the start pose (0.1, 0.35)
line,0.3,0.35
spline,0.45,0.3
spline,0.55,0.15
line,0.55,0.05

$ cargo run -q --release -p sim-cli -- analyze t-gantry --path curve.path
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,...
a,2.101000,0.547145,...,1.646e-4,5.616e-5,1.679e-4,...
b,2.101000,0.296009,...,1.646e-4,5.616e-5,1.679e-4,...
```

The curve leaves the first line in its direction and ends in the direction of the last one. Where it joins a line the curvature steps, so the path is slow there, like a line into an arc.

**The app** does the same from its right panel: the cycle analysis with its checks and their error bands (one value at a time or sampled), the sensitivity ranking, the optimiser, the motor search with a button that puts a motor into the running machine, paths and traces from files (a planned path is also drawn in the 3D view; a replayed trace is plotted against its simulation), the operating points over the torque-speed curve, the frequency response of the mechanics with the fit of the belt stiffness, and the tuning help.

## Where to go next

| Document | For |
| --- | --- |
| [docs/user-guide.md](docs/user-guide.md) | One section per task: size a motor, check a catalogue machine, search, error bands and sampled runs, shorten a cycle, run a path, tune, replay and fit a trace, connect a PLC, use the app |
| [docs/config-reference.md](docs/config-reference.md) | Every config table and key, with unit, meaning, source and effect switch |
| [docs/model-reference.md](docs/model-reference.md) | The equations, per component and per machine, with assumptions and known shortcuts |
| [docs/mechanisms.md](docs/mechanisms.md) | Ball screw, rack and pinion, linear motor, rotary table and arms from URDF files: the machines, their formulas, checks and limits ([German](docs/de/mechanismen.md)) |
| [docs/catalogue.md](docs/catalogue.md) | What is in `catalog/`, where it came from, misprints found, what is missing |
| [docs/validation.md](docs/validation.md) | Every check against something independent, and what is not validated |
| [PROGRESS.md](PROGRESS.md) | Status per phase, design decisions, what is blocked |
| [Project plan](<T-Gantry Physics Simulator Project Plan.md>) | Goal, architecture, stages |

## Layout

| Crate | Layer | Content |
| --- | --- | --- |
| [`sim-core`](crates/sim-core/src) | Core | Fixed-step integrator, machine interface, config loading, translations, the standard test suite. Knows no physics |
| [`sim-components`](crates/sim-components/src) | Components | Belt, friction, gearbox, drive, controller, profiles, paths, analysis, catalogue lookup, trace replay and fit. Knows no machine |
| [`machine-t-gantry`](crates/machine-t-gantry), [`machine-single-axis`](crates/machine-single-axis) | Machines | Transformation, assembly, signals, 3D scene description, configs |
| [`sim-machines`](crates/sim-machines) | Registration | The one list that names the machines; cross-machine tests |
| [`sim-fmi`](crates/sim-fmi) | Front ends | FMI 2.0 Co-Simulation: the FMU's binary (C functions), its model description and archive. The command line writes the FMU with it |
| [`sim-cli`](crates/sim-cli), [`sim-app`](crates/sim-app), [`sim-opcua`](crates/sim-opcua) | Front ends | Command line, desktop app, OPC UA server |

A crate depends only on crates below it; a test checks that.

## Licence

Copyright (c) 2026 Nico Schuermann. Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at
your option. Contributions are accepted under the same terms ([CONTRIBUTING.md](CONTRIBUTING.md)).

Third-party crates and their licences: [THIRD-PARTY-LICENSES.md](THIRD-PARTY-LICENSES.md), generated by
`tools/third_party_licenses.py`. None is GPL or LGPL. The OPC UA server (`sim-opcua`) uses `async-opcua`, which is
**MPL-2.0**: a file-level copyleft. Binaries that contain it may be distributed under MIT or Apache-2.0, as long as
the source of the MPL-covered files stays available to recipients (they are the unmodified crates.io releases listed
in that file) and any change to one of those files is published under the MPL. The project's own code is not
affected. `sim-app`, `sim-cli` and the FMU's binary do not contain MPL code.

Release notes: [CHANGELOG.md](CHANGELOG.md). What is planned: [ROADMAP.md](ROADMAP.md).
