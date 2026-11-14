# FMI export

`sim-cli fmu` writes a machine with its config as an FMU (Functional Mock-up Unit) for FMI 2.0 Co-Simulation (the default) or FMI 3.0 Co-Simulation. Both come from one binary, `sim_fmi` (`crates/sim-fmi`), which exports the `fmi2*` and the `fmi3*` C functions side by side. The FMU runs the same simulation as `sim-cli run`: drives, controller, every switched-on effect. The [user guide](user-guide.md#use-the-model-in-simulink--modelica-via-fmi) explains the stepping, the setpoint inputs and the parameters; this page adds the two versions, the FMU state, the derivatives, the missing Model Exchange, the validation and the import into other tools.

## Writing an FMU

```sh
cargo build --release -p sim-fmi -p sim-cli
sim-cli fmu t-gantry --out t-gantry.fmu              # FMI 2.0 Co-Simulation
sim-cli fmu t-gantry --fmi 3 --out t-gantry-3.fmu    # FMI 3.0 Co-Simulation
```

`--fmi` takes `2` or `3`; anything else is refused. `--binary FILE` names the binaries as before. The binaries go to these folders:

| Binary | FMI 2.0 | FMI 3.0 |
| --- | --- | --- |
| `sim_fmi.dll` | `binaries/win64/sim_fmi.dll` | `binaries/x86_64-windows/sim_fmi.dll` (or `aarch64-windows`) |
| `libsim_fmi.so` | `binaries/linux64/sim_fmi.so` | `binaries/x86_64-linux/sim_fmi.so` (or `aarch64-linux`) |
| `libsim_fmi.dylib` | `binaries/darwin64/sim_fmi.dylib` | `binaries/x86_64-darwin/sim_fmi.dylib` or `binaries/aarch64-darwin/sim_fmi.dylib` |

For FMI 3.0 the architecture is read from the binary's header (the machine field of a PE file, `e_machine` of an ELF file, `cputype` of a 64-bit Mach-O file). A file without such a header, a universal (fat) Mach-O file among them, is refused: FMI 3.0 wants one folder per architecture, so build one `.dylib` per architecture and pass both.

## Variables

The variables are the same in both versions; only the type names differ. The value reference of a variable is its place in the list.

| Variable | FMI 2.0 type | FMI 3.0 type | Causality | Variability |
| --- | --- | --- | --- | --- |
| `time` | Real | Float64 | independent | continuous |
| every other signal (`axis.x.position`, `motor.a.torque`, ...) | Real | Float64 | output | continuous |
| `command.setpoint.<axis id>` | Real | Float64 | input | continuous |
| `command.enable` | Boolean | Boolean | input | discrete |
| `command.cycle` | Boolean | Boolean | input | discrete |
| `command.target.<axis id>` | Real | Float64 | input | continuous |
| `command.move` | Boolean | Boolean | input | discrete |
| `command.path`, `command.path_file`, `command.trajectory_file` | String | String | input | discrete |
| `command.run_path` | Boolean | Boolean | input | discrete |
| `time_optimal` | Boolean | Boolean | parameter | fixed |
| `machine`, `config_file` | String | String | parameter | fixed |
| `effects.<name>` | Boolean | Boolean | parameter | fixed |

An FMU of a machine file (`sim-cli fmu --machine-file FILE`) carries the file as `resources/machine.toml` and builds its machine from it when it is instantiated, in both versions; its `machine` parameter is the file's machine id.

The outputs are every signal of the simulation (`Sim::signal_infos`), generated, never listed per machine: a signal the simulator gains (the drives' overload image or a motor's housing temperature, once they are signals) is an output of every FMU exported after that, with no change here.

### Cycle, moves and paths

Besides the setpoints, three trigger inputs command the machine as `sim-cli run --cycle`, `--move` and `--path` do. A trigger acts at the communication point where it has changed since the last one; the command it gives is logged under `logEvents`.

| Input | When it acts | Command |
| --- | --- | --- |
| `command.cycle` | turns true / turns false | starts the configured cycle / stops it at the end of its segment |
| `command.move` | turns true | moves to `command.target.<axis id>` (start: the start pose) |
| `command.run_path` | turns true | plans the path from where the machine stands, within the axis limits (with `time_optimal` as fast as the motors allow), and runs it; or runs the poses of `command.trajectory_file` |

The path is the text of a path file ([user guide](user-guide.md#run-a-path-and-read-path-error)) in `command.path`, or the file named in `command.path_file`, which wins when it is not empty; a file is read by its extension as `sim-cli run --path` reads it (G-code, KRL, RAPID, robot moves as CSV, else a path file), a program with feeds planned run by run. The parameter `time_optimal` (false by default) times the path as `--time-optimal` does: as fast as the motors' torque and the belts allow instead of within the axis limits. `command.trajectory_file` names a trajectory file (per line the time and one position per axis, as `sim-cli run --trajectory` reads it); where it is not empty, `command.run_path` runs its poses instead of a path. Both match the command line bit for bit (`crates/sim-fmi/tests/fmi.rs`, and against `sim-cli run --path FILE --time-optimal` through fmpy in `tools/check_fmu.py`). A path that cannot be read or planned, and a command the controller refuses (the drives off, a target out of range), gives `fmi2Warning` / `fmi3Warning` with the reason, and the machine goes on as before. A setpoint input, once set, takes the axes over whenever it differs from the setpoint the controller follows, and so replaces a cycle, move or path at the next step: drive the machine either by setpoints or by triggers.

In FMI 3.0 the start value of a String is a `<Start value="..."/>` element, as the FMI 3.0 schema demands. The model structure lists every output as an output and as an initial unknown. The outputs carry `dependencies=""`: no output depends directly on an input, because a setpoint reaches the outputs only through the next communication step. The initial unknowns carry no dependency list, which means they may depend on every parameter.

The model description of FMI 3.0 has `instantiationToken` where FMI 2.0 has `guid`; both are a hash of the model description, so the FMI 2.0 and the FMI 3.0 FMU of one machine and config have different tokens and refuse each other's instantiation and FMU states.

## Capabilities

| Attribute in the model description | FMI 2.0 | FMI 3.0 |
| --- | --- | --- |
| Co-Simulation | yes | yes |
| Model Exchange, Scheduled Execution | no | no |
| `canHandleVariableCommunicationStepSize` | true | true |
| `fixedInternalStepSize` | (not in FMI 2.0) | `sim.dt_s`, for example 6.25e-05 |
| `canGetAndSetFMUstate` / `canGetAndSetFMUState` | true | true |
| `canSerializeFMUstate` / `canSerializeFMUState` | true | true |
| `providesDirectionalDerivative` / `providesDirectionalDerivatives` | true | true |
| `providesAdjointDerivatives` | (not in FMI 2.0) | true |
| `hasEventMode` | (not in FMI 2.0) | false |
| `canInterpolateInputs`, `maxOutputDerivativeOrder` | false, 0 | (not used), 0 |

FMI 3.0 functions that need what this FMU does not have return `fmi3Error` with a logged message: Event Mode (`fmi3EnterEventMode`, `fmi3EnterStepMode`, `fmi3UpdateDiscreteStates`), Configuration Mode, clocks and their intervals and shifts, output derivatives, `fmi3GetVariableDependencies` (the model description has the dependencies), and every Model Exchange function. `fmi3InstantiateModelExchange` and `fmi3InstantiateScheduledExecution` log why and return null. Get and set functions of types the FMU has no variables of (Float32, the integer types, Binary, Clock) refuse every value reference and accept an empty list. `fmi3DoStep` always reports `eventHandlingNeeded`, `terminateSimulation` and `earlyReturn` as false, and `lastSuccessfulTime` as the time the simulation stands at (after a refused step, the time before it).

## FMU state

`fmi2GetFMUstate` / `fmi3GetFMUState` copy the state of an instance; `fmi2SetFMUstate` / `fmi3SetFMUState` restore it; the free, size, serialize and deserialize functions do what the standard says.

What a state holds: the instance's mode, the start time, every value the importer has set (inputs and parameters), the triggers as the last step saw them, and in step mode a snapshot of the simulation with its outputs (`Sim::snapshot`: the integrated state, the controller with its drives, loops, filters, windings, thermal and overload images, the parts' warmth and the obstacle record; everything but the machine, which the parameters build again).

Setting a state builds the machine from the state's parameters and restores the snapshot into it: as long as building the machine takes, however far the state is from the start. A serialized state carries the snapshot as bytes, so a deserialized one is set as fast, on the same instance or another one. Both give bit for bit the signals of the simulation that was saved: on every registered machine and machine file of `machines/` with its cycle running (`crates/sim-fmi/tests/fmi.rs`, `tests/fmi3.rs`, and through fmpy in `tools/check_fmu.py`).

Everything is built before the instance is changed: a state that cannot be set (its `config_file` gone, a config that is refused now) is refused with `fmi2Error` / `fmi3Error`, and the instance goes on where it was.

Limits:

- A serialized state does not grow with the steps since initialisation; its size is that of the snapshot (the machine's state, the controller with its plan and drives), a few kilobytes, more while a path or cycle with many points is under way.
- With `config_file` set, setting a state reads that file again. A file changed on disk since the state was taken gives another simulation, or a refusal if its signals differ.
- A serialized state is checked against the FMU's GUID or instantiation token and its number of variables; a state of another FMU, a cut-off state or one with bytes left over is refused with `fmi2Error` / `fmi3Error`. The snapshot's numbers are written with all their bits, so the next steps are bit for bit those of the saved simulation on the same binary; on a binary built by another compiler or for another platform they follow that binary's floating-point results from the state on.
- The serialized format: the text `sim-fmi state 3` and a zero byte, the FMU's version (`1.0.0`), the GUID, the mode, the start time, the three triggers and the set values, little-endian, then the outputs at the state and the snapshot as `postcard` 1 writes it (serde's data model, field by field in declaration order). The snapshot's layout follows the simulator's types, so a state is set only by the version of the FMU that saved it: one of another version, and the replayed states of format 2 (14 November 2026) and 1 (before), are refused with `fmi2Error` / `fmi3Error` and a message naming the version or format.

## Log categories

The model description declares `logEvents` (the commands the triggers gave, FMU states set), `logStatusWarning` (refused commands, paths that cannot be planned), `logStatusError` (refused calls) and `logAll`. An instance made with `loggingOn` false logs warnings and errors; with it true, events too. `fmi2SetDebugLogging` / `fmi3SetDebugLogging` switch the categories they name on or off, all of them when they name none or `logAll`; an unknown category is refused with an error and changes nothing. FMI 2.0 messages go to the logger as its format string with every `%` doubled, so a logger that formats them and one that prints them as they are both show the text.

## Directional and adjoint derivatives

`fmi2GetDirectionalDerivative`, `fmi3GetDirectionalDerivative` and `fmi3GetAdjointDerivative` accept outputs (Real / Float64, causality output) as unknowns and setpoint inputs as knowns, in Initialization Mode and in Step Mode, and return zero for every entry. That is the exact answer, not a placeholder: in Co-Simulation the derivative is the partial derivative of the outputs at the current communication point with respect to the inputs at that point. The outputs are the signals at the end of the last step; a new setpoint goes to the controller at the start of the next step. So setting an input changes no output until `DoStep` runs, and every partial derivative is zero. The tests check this against a finite difference by hand: they set a setpoint 1e-4 m higher, read every output again, and find the difference zero.

What these functions do not give is the sensitivity of the outputs after the next step to the inputs (how far the axis moves within one step per metre of setpoint). The FMI standard does not define that quantity for Co-Simulation, so it is not offered under these function names. An importer that wants it can take an FMU state, step with two input values, and restore the state.

Any other reference (an input as unknown, an output or a parameter as known, `time`) is refused with `fmi2Error` / `fmi3Error`.

## Why there is no Model Exchange

Model Exchange hands the FMU's continuous states and their derivatives to the importer's integrator. For this simulator that would be correct only with these parts:

1. Continuous states: the machine's mechanical state, and per drive the winding current and the three energy integrals of the step (`Sim::advance_with_windings` integrates these together as one vector).
2. A time event at every base step `sim.dt_s` (62.5 µs in the shipped configs): there the controller samples the motors, runs its cascaded loops and the setpoint interpolation, and sets the winding voltages, which are then held over the step. The controller's integrators, filters, interpolator, drive states, DC-link and braking-resistor logic are discrete states that change only at these events.
3. Further jumps at the same events: `Machine::after_step` changes the state after each step (a belt tooth jump changes a pulley's speed at once), the part temperatures are updated by the exact answer of a first-order lag held over the step, and the payload switch changes the machine.

Point 2 alone means 16 000 events per simulated second. The importer's integrator restarts after every event, so a variable-step solver never takes a step longer than 62.5 µs and adds event handling to each one; Model Exchange would be slower than Co-Simulation and give nothing in return. The results would also differ from `sim-cli run` and from the Co-Simulation FMU, because the importer's integrator replaces the fixed-step RK4 with substeps that the simulator uses and that the tests compare bit for bit. Finally, points 1 to 3 need the controller's and the simulation's internal states, which `sim-components` keeps private; opening them would change the `Sim` and `Machine` interfaces that the machine crates are written against. For these reasons this export offers Co-Simulation only, in both FMI versions.

## Validation

Done on Windows 11 x86_64 on 13 November 2026 with the binary from `cargo build --release -p sim-fmi -p sim-cli`. The FMUs were the default configs of `t-gantry` and `single-axis` and `t-gantry` with `crates/machine-t-gantry/exct-30.toml`, each as FMI 2.0 and FMI 3.0.

Rust tests (`cargo test -p sim-fmi -p sim-cli --release`): the FMI 2.0 and FMI 3.0 functions called as an importer calls them; every machine's signals bit for bit equal to a plain `Sim` with the same setpoints, and with the same move and path through the trigger inputs; FMU state restored bit for bit on the same instance (from its snapshot, on every machine with its cycle running) and, serialized with its snapshot, on a second instance; a time-optimal path through `command.path_file` and the poses of `command.trajectory_file` as `Sim::plan_path` and `Command::Trajectory` run them; states of another format or version refused as such; a state that cannot be set leaves the instance going on; the log categories switched; derivatives against the finite difference by hand; refusals.

fmpy 0.3.32 with numpy 2.3.4 on Python 3.13.17, installed from PyPI into a virtual environment:

```sh
python -m venv .venv
.venv/Scripts/python -m pip install fmpy==0.3.32 numpy==2.3.4
.venv/Scripts/python tools/check_fmu.py target/release/sim-cli.exe t-gantry-3.fmu t-gantry
.venv/Scripts/fmpy validate t-gantry-3.fmu
.venv/Scripts/fmpy simulate t-gantry-3.fmu --stop-time 0.1 --output-file sim3.csv
```

`tools/check_fmu.py` validates the FMU with fmpy's validator (the XSD of the FMU's FMI version, variable names, model structure, start values, causality and variability, units), simulates it with fmpy for 1 s at its default step, compares every output with `sim-cli run`, and then, through fmpy's bindings of the C functions, takes an FMU state after 5 steps, checks that the 7 steps after it come back bit for bit on the same instance and serialized on a second one, and checks that the directional derivative and the finite difference are zero; then a move and a time-optimal path through the inputs against the command line (parts 4 and 5, below). Result for all six FMUs:

```
FMU state: restored bit for bit, serialized 360 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry  (FMI 2.0): 0 validator problems, 96 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 306 bytes and restored bit for bit; directional derivative and finite difference zero
single-axis  (FMI 2.0): 0 validator problems, 51 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 358 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry crates/machine-t-gantry/exct-30.toml (FMI 2.0): 0 validator problems, 94 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 360 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry  (FMI 3.0): 0 validator problems, 96 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 306 bytes and restored bit for bit; directional derivative and finite difference zero
single-axis  (FMI 3.0): 0 validator problems, 51 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 358 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry crates/machine-t-gantry/exct-30.toml (FMI 3.0): 0 validator problems, 94 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
```

The worst difference of 5e-10 is the rounding of the 9 digits `sim-cli run` prints. `fmpy validate` printed "No problems found." and `fmpy simulate` ran without an error.

OMSimulator 3.0.0.post198 (OpenModelica's co-simulation tool), installed from PyPI into a second virtual environment:

```sh
python -m pip install OMSimulator==3.0.0.post198
python -m OMSimulator --validate t-gantry-3.fmu
python -m OMSimulator t-gantry-3.fmu --stopTime 1.0 --resultFile oms-3.csv
```

It reported "t-gantry-2.fmu conforms to the FMI-2.0 schema" and "t-gantry-3.fmu conforms to the FMI-3.0 schema", and simulated both for 1 s at the default step of 0.01 s. Its 101 rows of all 96 outputs differ from `sim-cli run` by at most 5.0e-10 of each signal's peak, for both versions.

On 14 November 2026 `tools/check_fmu.py` gained a fourth part: the first axis moved 10 mm through `command.target.*` and `command.move` against `sim-cli run --move`, with `logEvents` switched on (one message) and then every category off (no message). Same setup, fmpy 0.3.32, numpy 2.2.6, Python 3.14.3; the FMI 2.0 FMUs of `t-gantry`, `single-axis` and `t-gantry` with `exct-30.toml`, and the FMI 3.0 FMU of `machines/h-bot.toml` (as in CI):

```
FMU state: restored bit for bit, serialized 417 bytes and restored bit for bit; directional derivative and finite difference zero
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
t-gantry  (FMI 2.0): 0 validator problems, 96 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
single-axis  (FMI 2.0): 0 validator problems, 51 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
t-gantry crates/machine-t-gantry/exct-30.toml (FMI 2.0): 0 validator problems, 94 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 407 bytes and restored bit for bit; directional derivative and finite difference zero
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
machines/h-bot.toml  (FMI 3.0): 0 validator problems, 90 outputs, 101 rows, worst 4.9e-10 (motor.b.torque_rest): ok
```

Later on 14 November 2026 it gained a fifth part: the same target as a path file through `command.path_file` and `command.run_path` with the parameter `time_optimal`, against `sim-cli run --path FILE --time-optimal`; and the serialized state now carries the snapshot (format 3). Same setup:

```
FMU state: restored bit for bit, serialized 3840 bytes and restored bit for bit; directional derivative and finite difference zero
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
time-optimal path input: worst 4.5e-10 against sim-cli run --path --time-optimal
t-gantry  (FMI 2.0): 0 validator problems, 102 outputs, 101 rows, worst 4.9e-10 (axis.z.friction_load_scale): ok
FMU state: restored bit for bit, serialized 2244 bytes and restored bit for bit; directional derivative and finite difference zero
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
time-optimal path input: worst 4.5e-10 against sim-cli run --path --time-optimal
single-axis  (FMI 2.0): 0 validator problems, 54 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 3750 bytes and restored bit for bit; directional derivative and finite difference zero
move input: worst 4.5e-10 against sim-cli run --move; log categories: one logEvents message, none with logging off
time-optimal path input: worst 4.5e-10 against sim-cli run --path --time-optimal
machines/h-bot.toml  (FMI 3.0): 0 validator problems, 96 outputs, 101 rows, worst 4.9e-10 (motor.b.torque): ok
```

Not run: the Modelica Association's `fmusim` (Reference-FMUs) and the FMU Compliance Checker. Neither is published on a package registry: `pip index versions` finds no distribution named `fmusim`, `reference-fmus`, `fmi-compliance-checker` or `fmucheck`, and conda is not installed here. Both are distributed as binaries on GitHub releases, which were not downloaded.

## Import into other tools

Import in Simulink and Dymola was **not tested** here: neither was available. OpenModelica 1.27.1 was (see its section). The steps below follow the tools' documented menus. In every tool the communication step must be a whole multiple of `sim.dt_s` (62.5 µs in the shipped configs); use the FMU's default step, `profile.setpoint_interval_s` (10 ms), unless you change that interval in the config.

### Simulink

Simulink imports FMI 2.0 Co-Simulation FMUs with the FMU block; MathWorks lists FMI 3.0 Co-Simulation import from R2023b on (check the release notes of your release).

1. Write the FMU (`sim-cli fmu t-gantry --out t-gantry.fmu`, or with `--fmi 3`).
2. In the Library Browser open Simulink Extras > FMU Import and drag the FMU block into the model (or click the canvas, type `FMU` and pick the block).
3. Open the block and set its FMU name to the `.fmu` file. The block shows `command.setpoint.<axis>` and `command.enable` as input ports and the selected outputs as output ports.
4. Set the block's communication step size to 0.01 s (or another whole multiple of `sim.dt_s`) and use a fixed-step solver whose step divides it.
5. On the parameters tab set `config_file` or the `effects.<name>` switches if needed; they are fixed after initialisation.
6. Connect the setpoint inputs to your controller or PLC model and run.

### OpenModelica

Tested with OpenModelica 1.27.1 (64-bit, Windows), 14 November 2026. OpenModelica runs these FMUs through OMSimulator, its co-simulation tool, not as Modelica blocks:

- **OMSimulator 2.1.3, which ships with OpenModelica 1.27.1**, ran the FMI 2.0 FMUs of the T-gantry, the delta D3 and the H-bot machine file for 1 s at the default step of 0.01 s (`OMSimulator t-gantry.fmu --stopTime=1.0 --stepSize=0.01 --resultFile=result.csv`). Every output agrees with `sim-cli run` to within 5.0e-10 of its peak, the rounding of the CLI's 9 digits. That version refuses FMI 3.0 ("unsupported FMI version"); OMSimulator 3.0 (from PyPI, above) runs both.
- **OMEdit's File > Import > FMU** (the compiler's `importFMU`) turns only FMI 2.0 **Model Exchange** FMUs into Modelica blocks; it refuses these Co-Simulation FMUs ("Only FMI 2.0 ModelExchange is supported"), and this export has no Model Exchange (see [why](#why-there-is-no-model-exchange)).
- FMIL, the FMI library of OpenModelica and other importers, reported an error on the FMI 2.0 description while it was read: it wants a start value on the independent variable `time`, which the standard forbids there. Since 14 November the FMI 2.0 description leaves `time` out (the standard then has it implicitly; its value reference still answers), and FMIL reads it without a message. The FMI 3.0 description declares it.

In OMEdit, File > New SSP Model opens an OMSimulator model to which the FMU is added as a component and connected to other FMUs.

### Dymola

Dymola imports FMI 2.0 FMUs; check the documentation of your Dymola version for FMI 3.0 import.

1. File > Import > FMU..., choose the `.fmu` file.
2. In the import dialog choose co-simulation (this FMU has no Model Exchange part) and confirm. Dymola generates a Modelica block that calls the FMU.
3. Drag the block into a model, connect the setpoint inputs, and simulate with an output interval that is a whole multiple of the communication step.
