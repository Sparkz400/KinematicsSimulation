"""Independent check of an FMU written by `sim-cli fmu`, with fmpy (CI runs it on Windows and Linux).

usage: python tools/check_fmu.py SIM_CLI FMU MACHINE [--config TOML] [--stop S]

1. fmpy's validator (FMI 2.0 XSD, names, model structure, start values, causality, units) finds no problem.
2. fmpy runs the FMU as co-simulation at its default step with no input; every output must equal
   `sim-cli run` of the same machine and config at the same instants (the CLI prints 9 digits).
Exits 1 on any failure.
"""
import argparse, os, subprocess, sys, tempfile
import numpy as np
from fmpy import read_model_description, simulate_fmu
from fmpy.validation import validate_fmu

REL_TOL = 1e-8  # the CLI's 9 printed digits; the hand check found 5e-10

ap = argparse.ArgumentParser()
ap.add_argument("cli"); ap.add_argument("fmu"); ap.add_argument("machine")
ap.add_argument("--config"); ap.add_argument("--stop", type=float, default=1.0)
a = ap.parse_args()

failed = False
problems = validate_fmu(a.fmu)
for p in problems:
    print("validator:", p)
failed |= bool(problems)

md = read_model_description(a.fmu)
step = float(md.defaultExperiment.stepSize)
outputs = [v.name for v in md.modelVariables if v.causality == "output"]
logs = []
fmu = simulate_fmu(a.fmu, fmi_type="CoSimulation", stop_time=a.stop, step_size=step, output_interval=step,
                   output=outputs, logger=lambda *m: logs.append(m))
for m in logs:
    print("fmu log:", m)

with tempfile.TemporaryDirectory() as d:
    out = os.path.join(d, "cli.csv")
    cmd = [os.path.abspath(a.cli), "run", a.machine, "--time", str(a.stop), "--every", "1", "--out", out]
    if a.config:
        cmd[3:3] = ["--config", a.config]
    subprocess.run(cmd, check=True)
    cli = np.genfromtxt(out, delimiter=",", names=True, deletechars="")
cols = {n.split("[")[0]: n for n in cli.dtype.names}
rows = np.searchsorted(cli[cols["time"]], fmu["time"] - 1e-9)
if not np.allclose(cli[cols["time"]][rows], fmu["time"], atol=1e-9):
    print("time grids differ"); sys.exit(1)

worst = (0.0, "")
for name in outputs:
    if name not in cols:
        print(f"output {name} not in sim-cli run"); failed = True; continue
    f, c = fmu[name].astype(float), cli[cols[name]][rows]
    scale = max(np.max(np.abs(c)), 1e-12)
    rel = np.max(np.abs(f - c)) / scale
    worst = max(worst, (rel, name))
    if rel > REL_TOL:
        print(f"{name}: FMU differs from sim-cli by {rel:.2e} of its peak"); failed = True

print(f"{a.machine} {a.config or ''}: {len(problems)} validator problems, {len(outputs)} outputs, "
      f"{len(fmu)} rows, worst {worst[0]:.1e} ({worst[1]}): {'FAIL' if failed else 'ok'}")
sys.exit(1 if failed else 0)
