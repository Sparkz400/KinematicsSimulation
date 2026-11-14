# Python module

`kinematics_sim` runs the simulator from Python: the machines, their configs, runs, the cycle analysis, the verdicts with
their error bands, the catalogue sizing, the cycle optimiser, the frequency sweep and the fitting and identification
from measured traces. It has the main commands of `sim-cli` as functions (what it lacks is listed at the end) and gives
the same numbers (the tests in `crates/sim-py/tests` compare them). Results come as NumPy arrays and as dicts of equal-length columns;
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
| `verdicts(cfg, bands=False, samples=None, seed=1)` | the checks; with `bands` their lowest and highest value over the `[bounds]` and whether the worst case passes; with `samples` N runs drawn between the bounds and the share that passed | `sim-cli verdicts [--bands] [--samples N --seed S]` |
| `sizing(cfg, supply=None, vendor=None, series="", gearboxes=None, axes=False)` | every catalogue motor on the config's cycle, passing and cheapest first; `supply` one of the catalogue's (`"400v"` when left out), `vendor` a maker's supplies or `"all"` makers', with the columns `vendor` and `supply` | `sim-cli search` |
| `sweep(cfg, motor=0, from_hz=5.0, to_hz=500.0, count=120, amplitude=None)` | the frequency response from a motor's torque to its speed: `frequency_hz`, `gain_rad_s_per_nm`, `phase_deg` | `sim-cli sweep` |
| `optimise(cfg, runs=40, nominal=False, worst=False, pareto=False)` | a shorter cycle that passes every verdict: `cycle_s` and `dwell_error_peak_m` as (before, after), `runs`, `changes` (columns `setting`, `before`, `after`), `robust` (what the bands cost, or None), with `pareto` the front as columns | `sim-cli optimise` |
| `fit(cfg, trace_file=, map_file=, stiffness=False, stiffness_key=, sweep_file=, sweep_motor=0, amplitude=)` | what a measured trace or sweep says the config should be: `quantity`, `of`, `config`, `fitted`, `standard_error` (NaN where the command line leaves the cell empty), and `advice`, what to change | `sim-cli fit` |
| `identify(trace_file=, map_file=, config=, motor=0, from_hz=, to_hz=, step_from=, step_to=, motor_inertia=, sweep_file=)` | one motor's drive train from its own trace: `quantity`, `of`, `value`, `standard_error`, `ci95_low`, `ci95_high`, `notes` (what the trace could not tell) and `overlay` (measured and modelled signals with `time`) | `sim-cli identify [--overlay]` |

`path` and `trajectory` take the text of a path file or a trajectory file (formats: `sim_components::path`), not a file
name: `ks.analyze(cfg, path=open("pick.path").read())`. `trace_file`, `map_file` and `sweep_file` take file names, as the
command line does (a vendor export is decoded from its bytes). A refused config, command, path or trace raises
`ValueError` with the simulator's message in English; an unknown machine or config key read with `cfg[path]` raises
`KeyError`; an int beyond 64 bits set with `cfg[path] = value` raises `OverflowError` (TOML's integers are 64-bit; before
14 November 2026 it became a float silently). `run` takes at most 1e8 base steps and samples at most 2.5e8 values (2 GB
of arrays); a longer run raises `ValueError` at once instead of seeming to hang or exhausting the memory: raise `every`,
or run in pieces.

A config change is checked as a config file is: a key the machine's default config does not have is refused by name.

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
cargo build -p sim-cli --release       # the tests compare with it
pip install "pytest==9.2.0" pandas matplotlib nbformat nbclient ipykernel
pytest crates/sim-py/tests
```

The suite checks the machine list, config reading, setting and refusals, the TOML round trip, `run` against
`sim-cli run` (same samples to 1e-8), `analyze` and `verdicts` against the command line, that a heavier payload needs
more torque, the order of the sizing table, that two threads get the same result, and the notebook. 11 tests pass on
Windows with CPython 3.14 (13 November 2026). Added on 14 November 2026: a machine file run against
`sim-cli run --machine-file`, its refusals, `sweep`, `optimise`, `fit` and `identify` against the command line on one
pass of the cycle, the overflow of 2^63 (one more than the largest 64-bit integer) and the guard on `run(time=1e12)`;
those 10 pass on Windows with CPython 3.14.

## Not covered

The command line's `step`, `bode`, `tuning`, `margins`, `modes`, `sensitivity`, `replay`, `compare`, `report`,
`check-dataset` and `fmu` have no function yet; the `--worst` search of `verdicts` and `search --motor-id` neither.
`fit` and `identify` raise at the first failure, where the command line writes what it got and then fails. Messages are
in English only.
