# Python module

`kinematics_sim` runs the simulator from Python: the machines, their configs, runs, the cycle analysis, the verdicts with
their error bands and the catalogue sizing. It is the command line `sim-cli` as functions, and gives the same numbers
(the tests in `crates/sim-py/tests` compare them). Results come as NumPy arrays and as dicts of equal-length columns;
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
| `cfg[path] = value`, `cfg.set("path = value")` | a value set on top. A part named under `[parts]` (`cfg["parts.motor"] = "1FK2104-5AK"`) acts as `--motor` and the other part options: all named parts come in from the catalogue together, and every other value set stays on top of theirs, in whatever order the two were set | `--set`, `--motor` and the other part options |
| `signals(cfg)` | `{"id": [...], "unit": [...]}`, the signals in the order `run` gives them | the header of `sim-cli run` |
| `run(cfg, cycle=, move_to=, path=, trajectory=, time_optimal=, time=2.0, every=16, disable_at=)` | `{signal id: array}` over `time` seconds, sampled every `every` base steps | `sim-cli run` |
| `analyze(cfg, path=, trajectory=, time_optimal=, points=False)` | the load per motor over one pass of the cycle, or over a path; `points=True` adds the operating points per motor as an (n, 2) array of speed (rad/s) and torque (N m) | `sim-cli analyze` |
| `verdicts(cfg, bands=False, samples=None, seed=1)` | the checks; with `bands` their lowest and highest value over the `[bounds]` and whether the worst case passes; with `samples` N runs drawn between the bounds and the share that passed | `sim-cli verdicts [--bands] [--samples N --seed S]` |
| `sizing(cfg, supply="400v", series="", gearboxes=None, axes=False)` | every catalogue motor on the config's cycle, passing and cheapest first | `sim-cli search` |

`path` and `trajectory` take the text of a path file or a trajectory file (formats: `sim_components::path`), not a file
name: `ks.analyze(cfg, path=open("pick.path").read())`. A refused config, command or path raises `ValueError` with the
simulator's message in English; an unknown machine or config key read with `cfg[path]` raises `KeyError`.

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
Windows with CPython 3.14 (13 November 2026).

## Not covered

The command line's `sweep`, `step`, `bode`, `tuning`, `margins`, `sensitivity`, `optimise`, `replay`, `fit`, `compare` and
`fmu` have no function yet; the `--worst` search of `verdicts` neither. Messages are in English only.
