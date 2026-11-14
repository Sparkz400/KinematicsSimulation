# Python module

`kinematics_sim` runs the simulator from Python: the machines, their configs, runs, the cycle analysis, the verdicts with
their error bands, the catalogue sizing, the cycle optimiser, the frequency sweep and the fitting and identification
from measured traces, the loop analyses (step, Bode, tuning, margins, modes), sensitivity, replay, effect comparison,
the sizing report, the dataset check and the FMU export. Every command of `sim-cli` is a function (`list` is
`machines`; what the functions leave out is listed at the end), and they give the same numbers (the tests in
`crates/sim-py/tests` compare them). Results come as NumPy arrays and as dicts of equal-length columns;
`pandas.DataFrame(result)` takes every table as it is.

Source: `crates/sim-py` (PyO3 0.30.1, built with maturin 1.16.0). The wheel is abi3: one wheel per platform serves
every CPython from 3.10 on. Type stubs (`kinematics_sim.pyi`) and the `py.typed` marker are in the wheel.

## Install

The module is not on PyPI. Build the wheel from the repository and install it:

```bash
pip install maturin==1.16.0
cd crates/sim-py
maturin build --release --out ../../target/wheels
pip install ../../target/wheels/kinematics_sim-*.whl
```

`maturin develop --release` installs it into the active virtual environment instead. A Rust toolchain (1.98.1) is
needed for either. CI builds the wheel for Windows and Linux and keeps it as an artifact (`wheel-windows-latest`,
`wheel-ubuntu-latest`).

## Use

```python
import kinematics_sim as ks
import pandas as pd

ks.machines()                       # [{"id": "t-gantry", "name": "T-gantry"}, ...]; ks.machines("de") in German

cfg = ks.Config("t-gantry")         # the default config; Config("t-gantry", path="my.toml") or text="..."
cfg["mass.payload_kg"]              # 2.0
cfg["mass.payload_kg"] = 3.5        # the value at a dotted path
cfg.set('parts.motor = "1FK2104-5AK"')   # exactly as sim-cli --set: the value in TOML syntax
cfg.to_toml()                       # the config as TOML

run = pd.DataFrame(ks.run(cfg, cycle=True, time=2.0))   # every signal, one column each, "time" first
pd.DataFrame(ks.analyze(cfg))                           # one row per motor, the columns of sim-cli analyze
pd.DataFrame(ks.verdicts(cfg, bands=True))              # one row per check and motor, with low, high, worst_ok
pd.DataFrame(ks.sizing(cfg, series="1FK2"))             # every catalogue motor on this cycle, passing first
```

| function | what it does | as on the command line |
| --- | --- | --- |
| `machines(lang="en")` | the machines of the registry, with their names in English or German | `sim-cli list` |
| `Config(machine, text=None, path=None)` | a machine's default config, a config in TOML text, or a config file | `--config FILE` |
| `Config(machine_file=FILE, text=None, path=None)` | the same for a machine from a machine file ([machine files](machine-files.md)); an error in the file names the file and line | `--machine-file FILE` |
| `cfg[path] = value`, `cfg.set("path = value")` | a value set on top. A part named under `[parts]` (`cfg["parts.motor"] = "1FK2104-5AK"`) acts as `--motor` and the other part options: all named parts come in from the catalogue together, and every other value set stays on top of theirs, in whatever order the two were set | `--set`, `--motor` and the other part options |
| `signals(cfg)` | `{"id": [...], "unit": [...]}`, the signals in the order `run` gives them | the header of `sim-cli run` |
| `run(cfg, cycle=, move_to=, path=, trajectory=, time_optimal=, time=2.0, every=16, disable_at=)` | `{signal id: array}` over `time` seconds, sampled every `every` base steps | `sim-cli run` |
| `analyze(cfg, path=, trajectory=, time_optimal=, points=False)` | the load per motor over one pass of the cycle, or over a path; `points=True` adds the operating points per motor as an (n, 2) array of speed (rad/s) and torque (N m) | `sim-cli analyze` |
| `verdicts(cfg, bands=False, samples=None, seed=1, worst=False)` | the checks; with `bands` their lowest and highest value over the `[bounds]` and whether the worst case passes; with `samples` N runs drawn between the bounds and the share that passed; with `bands=True, worst=True` the worst corner per check: `worst`, `limit`, `worst_ok`, `at_high_bound` and `at_low_bound` (lists of parameters), and `runs` | `sim-cli verdicts [--bands] [--samples N --seed S] [--worst]` |
| `sizing(cfg, supply=None, vendor=None, series="", gearboxes=None, axes=False, motor_id=None)` | every catalogue motor on the config's cycle, passing and cheapest first; `supply` one of the catalogue's (the config's own `parts.motor_supply` when left out, else `"400v"`), `vendor` a maker's supplies or `"all"` makers', with the columns `vendor` and `supply`; `motor_id` sizes one motor of several (a machine file's), its peak torque and winding rise alone (`winding_rise_k`) | `sim-cli search [--motor-id ID]` |
| `sweep(cfg, motor=0, from_hz=5.0, to_hz=500.0, count=120, amplitude=None)` | the frequency response from a motor's torque to its speed: `frequency_hz`, `gain_rad_s_per_nm`, `phase_deg` | `sim-cli sweep` |
| `optimise(cfg, runs=40, nominal=False, worst=False, pareto=False)` | a shorter cycle that passes every verdict: `cycle_s` and `dwell_error_peak_m` as (before, after), `runs`, `changes` (columns `setting`, `before`, `after`), `robust` (what the bands cost, or None), with `pareto` the front as columns | `sim-cli optimise` |
| `fit(cfg, trace_file=, map_file=, stiffness=False, stiffness_key=, sweep_file=, sweep_motor=0, amplitude=)` | what a measured trace or sweep says the config should be: `quantity`, `of`, `config`, `fitted`, `standard_error` (NaN where the command line leaves the cell empty), `advice`, what to change, and `errors` (see below) | `sim-cli fit` |
| `identify(trace_file=, map_file=, config=, motor=0, from_hz=, to_hz=, step_from=, step_to=, motor_inertia=, sweep_file=)` | one motor's drive train from its own trace: `quantity`, `of`, `value`, `standard_error`, `ci95_low`, `ci95_high`, `notes` (what the trace could not tell), `overlay` (measured and modelled signals with `time`) and `errors` | `sim-cli identify [--overlay]` |
| `step(cfg, axis_index=0, size=0.001, time=0.5)` | an axis's answer to a setpoint step: `time_s`, `setpoint`, `response`, and `rise_time_s`, `overshoot`, `settling_time_s` | `sim-cli step` |
| `bode(cfg, axis_index=0, from_hz=1.0, to_hz=200.0, count=40, amplitude=0.0005)` | the closed-loop response of an axis's position loop: `frequency_hz`, `gain`, `gain_db`, `phase_deg` | `sim-cli bode` |
| `tuning(cfg, size=0.001, amplitude=0.0005)` | per axis `rise_time_s`, `overshoot`, `settling_time_s`, `bandwidth_hz` (NaN where the response stays above -3 dB to 200 Hz) | `sim-cli tuning` |
| `margins(cfg, bands=False)` | per run and motor the velocity loop's `crossover_hz`, `phase_margin_deg`, `gain_margin_db`, `peak_sensitivity`, `peak_frequency_hz`, `ok` | `sim-cli margins [--bands]` |
| `modes(cfg, axis_index=0, points=21)` | the modes over an axis's stroke: `position`, `mode<i>_hz`, `mode<i>_damping` (NaN where a place lacks one) | `sim-cli modes` |
| `sensitivity(cfg, delta=0.2, groups=None)` | each parameter at its low and high value, strongest first: `parameter`, `value`, `low`, `high`, the relative changes of peak and RMS torque and cycle time, the least belt tension | `sim-cli sensitivity [--group G]` |
| `replay(cfg, trace_file=, map_file=None)` | a trace's setpoints through the simulator: per motor torque and following error apart (RMS, peak), and `overlay`, measured and simulated side by side | `sim-cli replay [--out FILE]` |
| `compare(cfg, effect=, signals=None)` | the cycle with `effects.<effect>` on and off: `time`, `<id>.on`, `<id>.off`, and `differences` (`signal`, `unit`, `peak_difference`, `rms_difference`) | `sim-cli compare [--differences FILE]` |
| `report(cfg, path, bands=False, lang="en")` | the sizing report as PDF (a path ending in .pdf) or HTML; returns whether every check passed | `sim-cli report` |
| `check_dataset(path)` | what keeps a dataset folder from being published, one text per problem; empty: ready | `sim-cli check-dataset` |
| `fmu(cfg, path, binaries=[...], fmi=2, lang="en")` | the machine as an FMI 2.0 or 3.0 FMU ([FMI](fmi.md)) carrying the sim-fmi binaries named (`cargo build -p sim-fmi --release` builds the one of this platform) | `sim-cli fmu [--binary FILE]` |

`path` and `trajectory` take the text of a path file or a trajectory file (formats: `sim_components::path`), not a file
name: `ks.analyze(cfg, path=open("pick.path").read())`. `trace_file`, `map_file` and `sweep_file` take file names, as the
command line does (a vendor export is decoded from its bytes). A refused config, command, path or trace raises
`ValueError` with the simulator's message in English; an unknown machine or config key read with `cfg[path]` raises
`KeyError`; an int beyond 64 bits set with `cfg[path] = value` raises `OverflowError` (TOML's integers are 64-bit; before
14 November 2026 it became a float silently). `run` takes at most 1e8 base steps and samples at most 2.5e8 values (2 GB
of arrays); a longer run raises `ValueError` at once instead of seeming to hang or exhausting the memory: raise `every`,
or run in pieces.

A config change is checked as a config file is: a key the machine's default config does not have is refused by name.

`fit` and `identify` run several fits, and one may find nothing (a measured sweep with no resonance, a trace with no
ringing after `step_from`, no two-mass model). The command line writes what the others found and then fails; these two
functions return what the others found, and list each fit that found nothing in `result["errors"]` (its rows are not in
the table). Check it: an empty list means every fit asked for is there. What keeps every fit from starting (a refused
config, an unreadable trace or sweep file) still raises `ValueError`.

## Parameter sweeps

Every long call releases Python's global interpreter lock, so a thread pool runs several on all processors:

```python
from concurrent.futures import ThreadPoolExecutor

def peak(mass):
    c = cfg.copy()
    c["mass.payload_kg"] = mass
    return ks.analyze(c)["peak_torque_nm"].max()

with ThreadPoolExecutor() as pool:
    peaks = list(pool.map(peak, [1.0, 2.0, 4.0, 8.0]))
```

`crates/sim-py/examples/parameter_sweep.ipynb` is a worked example: the T-gantry's payload from 0.5 kg to 8 kg, peak and
RMS torque, thermal utilisation and the verdicts per mass, and the signals of one run as a DataFrame. The test suite
runs it from top to bottom.

## Tests

```bash
cargo build -p sim-cli -p sim-fmi --release   # the tests compare with them
pip install "pytest==9.2.0" pandas matplotlib nbformat nbclient ipykernel
pytest crates/sim-py/tests
```

The suite checks the machine list, config reading, setting and refusals, the TOML round trip, `run` against
`sim-cli run` (same samples to 1e-8), `analyze` and `verdicts` against the command line, that a heavier payload needs
more torque, the order of the sizing table, that two threads get the same result, and the notebook. 11 tests pass on
Windows with CPython 3.14 (13 November 2026). Added on 14 November 2026: a machine file run against
`sim-cli run --machine-file`, its refusals, `sweep`, `optimise`, `fit` and `identify` against the command line on one
pass of the cycle, the overflow of 2^63 (one more than the largest 64-bit integer) and the guard on `run(time=1e12)`;
those 10 pass on Windows with CPython 3.14. Added later that day, each against the command line on a shipped config:
`step`, `bode`, `tuning`, `margins`, `modes`, `sensitivity`, `replay` (with its overlay), `compare` (with its
differences), `report` (the same HTML), `check_dataset`, `fmu` (the same FMU bytes, FMI 2.0 and 3.0), `verdicts` at the
worst corner, `sizing` of one motor of a machine file and by vendor, and `fit` and `identify` with a sweep that has no
resonance (the same rows as the command line writes before it fails, and the failure in `errors`); those 13 pass on
Windows with CPython 3.14 in 19 minutes, most of it the worst-corner search and the sizing.

## Not covered

What the command line writes to the error stream beside its table (the clearance of a planned path to obstacles, the
thermal start of the measured pass, the conveyor's picks, the search's timing and screened count, the deviation of a
planned path) has no place in the results. `analyze`'s `--breakdown` and `--budget` tables, `optimise`'s TOML of the new
cycle, and `--catalog DIR` (another catalogue folder) have no argument. `path=` takes the text of a path file only, not
a G-code, KRL, RAPID or robot CSV program as `--path FILE` does. `fmu` needs the sim-fmi binaries named. Messages are in
English only.
