"""Turn a recording of the ISW ball screw test bench (DaRUS, doi:10.18419/darus-3003, CC BY 4.0) into the CSV of
datasets/isw-ball-screw-axis/.

The files are MATLAB v7.3 (HDF5) structs as dSPACE ControlDesk writes them: per signal its name and data, and per
raster (1 ms, 4 kHz "HostService") its time vector. This takes the motor's signals on the 1 ms grid, every fourth
sample of the 4 kHz ones (the grids coincide there; checked below), and drops everything else, the operator's user
name among it.

    python -I tools/darus_ball_screw_to_csv.py SOURCE.mat OUT.csv

Needs h5py and numpy (not dependencies of the project).
"""

import sys

import h5py
import numpy as np


def text(f, ref):
    return "".join(map(chr, f[ref][()].flatten())) if f[ref].dtype == np.uint16 else ""


def main(source, out):
    f = h5py.File(source, "r")
    g = f[next(k for k in f if k != "#refs#")]
    times = [np.array(f[g["X/Data"][i, 0]][()]).flatten() for i in range(g["X/Data"].shape[0])]
    signals = {text(f, g["Y/Name"][i, 0]): np.array(f[g["Y/Data"][i, 0]][()]).flatten() for i in range(g["Y/Name"].shape[0])}
    setpoint = signals["traj_KGT[0]"]
    n = len(setpoint)
    t1 = next(t for t in times if len(t) == n)
    fast = len(signals["M_KGT_act"])
    t4 = next(t for t in times if len(t) == fast)
    assert np.array_equal(t4[::4][:n], t1[: len(t4[::4])]), "the 1 ms grid is not every fourth 4 kHz sample"
    n = min(n, len(t4[::4]))
    pick = lambda name: signals[name][::4][:n]
    columns = [
        ("time [s]", t1[:n], "%.6f"),
        ("position setpoint [m]", setpoint[:n], "%.8f"),
        ("position actual [m]", pick("x_meas_motor"), "%.8f"),
        ("speed actual [m/s]", pick("v_meas_motor"), "%.6f"),
        ("torque actual [Nm]", pick("M_KGT_act"), "%.6f"),
    ]
    with open(out, "w", newline="\n") as w:
        w.write(",".join(c[0] for c in columns) + "\n")
        for row in zip(*(c[1] for c in columns)):
            w.write(",".join(fmt % v for v, (_, _, fmt) in zip(row, columns)) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
