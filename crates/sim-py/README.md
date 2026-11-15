# kinematics-sim

`kinematics_sim` runs a physics simulator of motion machines from Python: gantries, delta pickers, arms, a hexapod,
cable robots, crank and cam machines, with a servo drive model. Torque, speed and following error come out of a
physical model (belt sections as springs, LuGre friction, gearbox backlash, motor winding, cascaded drive), and the
module gives runs, the cycle analysis, sizing verdicts with error bands and a catalogue motor search as NumPy arrays and
pandas-ready tables. Every command of the command-line tool `sim-cli` is a function and gives the same numbers.

```bash
pip install kinematics-sim
```

```python
import kinematics_sim as ks
import pandas as pd

ks.machines()                                   # [{"id": "t-gantry", "name": "T-gantry"}, ...]
cfg = ks.Config("t-gantry")                     # a machine's default config
cfg["mass.payload_kg"] = 3.5                    # a value at a dotted path, as sim-cli --set
pd.DataFrame(ks.analyze(cfg))                   # the load per motor over one cycle
pd.DataFrame(ks.verdicts(cfg, bands=True))      # every check, with its band over the uncertain values
pd.DataFrame(ks.sizing(cfg, series="1FK2"))     # every catalogue motor on this cycle, passing first
```

One abi3 wheel per platform serves every CPython from 3.10 on; type stubs are included.

- Documentation: https://sparkz400.github.io/KinematicsSimulation/docs/python.html
- Source, issues and the other front ends (desktop app, CLI, OPC UA, FMI, browser): https://github.com/Sparkz400/KinematicsSimulation

No real machine has been measured: friction, damping and stiffness are estimates with stated bounds, so trends and
comparisons hold, absolute numbers are as good as the estimates (see the validation page of the documentation).

Licensed under either of MIT or Apache-2.0, at your option.
