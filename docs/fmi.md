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
| `machine`, `config_file` | String | String | parameter | fixed |
| `effects.<name>` | Boolean | Boolean | parameter | fixed |

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

What a state holds: the instance's mode, the start time, every value the importer has set (inputs and parameters), and for each communication step since initialisation the number of base steps, the enable input and the setpoint inputs it applied. Restoring a state builds a fresh simulation from the parameters and replays those steps. The simulation is deterministic, so the replay gives bit for bit the signals of the simulation that was saved (tested in `crates/sim-fmi/tests/fmi.rs` and `tests/fmi3.rs`, and through fmpy in `tools/check_fmu.py`).

Limits:

- Restoring a state costs as long as simulating from the start up to the saved point, and a state grows by 13 bytes plus, per axis, 9 bytes with each communication step (1 byte for an axis whose setpoint has not been set). The reason is that the simulation holds the machine as a `Box<dyn Machine>`, which cannot be copied; a direct snapshot needs `Machine` and `Sim` to be cloneable. The code marks this with `shortcut:` in `crates/sim-fmi/src/api.rs`.
- With `config_file` set, the replay reads that file again. A file changed on disk since the state was taken gives another simulation, or a refusal if its signals differ.
- A serialized state is checked against the FMU's GUID or instantiation token and its number of variables; a state of another FMU, a cut-off state or one with bytes left over is refused with `fmi2Error` / `fmi3Error`. Bit-identical replay was tested within one build of the binary; a state replayed by a binary built by another compiler or for another platform is replayed with that binary's floating-point results.
- The serialized format is little-endian: the text `sim-fmi state 1` and a zero byte, the GUID, the mode, the start time, the set values and the steps.

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

Rust tests (`cargo test -p sim-fmi -p sim-cli --release`): the FMI 2.0 and FMI 3.0 functions called as an importer calls them; every machine's signals bit for bit equal to a plain `Sim` with the same setpoints; FMU state restored bit for bit on the same instance and, serialized, on a second instance; derivatives against the finite difference by hand; refusals.

fmpy 0.3.32 with numpy 2.3.4 on Python 3.13.17, installed from PyPI into a virtual environment:

```sh
python -m venv .venv
.venv/Scripts/python -m pip install fmpy==0.3.32 numpy==2.3.4
.venv/Scripts/python tools/check_fmu.py target/release/sim-cli.exe t-gantry-3.fmu t-gantry
.venv/Scripts/fmpy validate t-gantry-3.fmu
.venv/Scripts/fmpy simulate t-gantry-3.fmu --stop-time 0.1 --output-file sim3.csv
```

`tools/check_fmu.py` validates the FMU with fmpy's validator (the XSD of the FMU's FMI version, variable names, model structure, start values, causality and variability, units), simulates it with fmpy for 1 s at its default step, compares every output with `sim-cli run`, and then, through fmpy's bindings of the C functions, takes an FMU state after 5 steps, checks that the 7 steps after it come back bit for bit on the same instance and serialized on a second one, and checks that the directional derivative and the finite difference are zero. Result for all six FMUs:

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

Not run: the Modelica Association's `fmusim` (Reference-FMUs) and the FMU Compliance Checker. Neither is published on a package registry: `pip index versions` finds no distribution named `fmusim`, `reference-fmus`, `fmi-compliance-checker` or `fmucheck`, and conda is not installed here. Both are distributed as binaries on GitHub releases, which were not downloaded.

## Import into other tools

Import in Simulink, OpenModelica (OMEdit) and Dymola was **not tested** here: none of them was available. The steps below follow the tools' documented menus. In every tool the communication step must be a whole multiple of `sim.dt_s` (62.5 µs in the shipped configs); use the FMU's default step, `profile.setpoint_interval_s` (10 ms), unless you change that interval in the config.

### Simulink

Simulink imports FMI 2.0 Co-Simulation FMUs with the FMU block; MathWorks lists FMI 3.0 Co-Simulation import from R2023b on (check the release notes of your release).

1. Write the FMU (`sim-cli fmu t-gantry --out t-gantry.fmu`, or with `--fmi 3`).
2. In the Library Browser open Simulink Extras > FMU Import and drag the FMU block into the model (or click the canvas, type `FMU` and pick the block).
3. Open the block and set its FMU name to the `.fmu` file. The block shows `command.setpoint.<axis>` and `command.enable` as input ports and the selected outputs as output ports.
4. Set the block's communication step size to 0.01 s (or another whole multiple of `sim.dt_s`) and use a fixed-step solver whose step divides it.
5. On the parameters tab set `config_file` or the `effects.<name>` switches if needed; they are fixed after initialisation.
6. Connect the setpoint inputs to your controller or PLC model and run.

### OpenModelica

OpenModelica's model editor OMEdit imports FMI 2.0 FMUs as Modelica blocks; for FMI 3.0 and for running co-simulation directly use OMSimulator, which was tested here from the command line (see above).

1. OMEdit: File > Import > FMU..., choose the `.fmu` file and an output directory, and confirm. OMEdit generates a Modelica block with the FMU's inputs and outputs and loads it into the library tree.
2. Drag the block into a model, connect the setpoint inputs, and set the simulation's interval to the communication step.
3. OMSimulator: `OMSimulator t-gantry-3.fmu --stopTime 1.0 --resultFile result.csv` runs the FMU on its own; in OMEdit, File > New SSP Model opens an OMSimulator model to which the FMU is added as a component and connected to other FMUs.

### Dymola

Dymola imports FMI 2.0 FMUs; check the documentation of your Dymola version for FMI 3.0 import.

1. File > Import > FMU..., choose the `.fmu` file.
2. In the import dialog choose co-simulation (this FMU has no Model Exchange part) and confirm. Dymola generates a Modelica block that calls the FMU.
3. Drag the block into a model, connect the setpoint inputs, and simulate with an output interval that is a whole multiple of the communication step.
