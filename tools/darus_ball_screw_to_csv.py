"""Turn a recording of the ISW ball screw test bench (DaRUS, CC BY 4.0: doi:10.18419/darus-3003, darus-3252) into the
CSV of datasets/isw-ball-screw-axis/.

The files are MATLAB v7.3 (HDF5) structs as dSPACE ControlDesk writes them: per signal its name and data, and per
raster (1 ms, 4 kHz "HostService") its time vector. This takes the motor's signals on the 1 ms grid, every fourth
sample of the 4 kHz ones (the grids coincide there; checked below), and drops everything else, the operator's user
name among it. Where the recording has them, two more columns: the force the linear direct drive under the table was
told to push with (`F_des_LDD`, the compensation of its magnets' pull) and the force the nut's sensors measured
(`F_nut_sum`, four KISTLER sensors in the nut, not referenced: their zero drifts).

    python -I tools/darus_ball_screw_to_csv.py SOURCE.mat OUT.csv

With --sweep, a velocity sweep (the speed loop closed, the position loop open: Identification/VelocitySweeps) becomes a
frequency response from the motor's torque to its speed, as `sim-cli sweep` writes one (frequency_hz,
gain_rad_s_per_nm, phase_deg), from the 4 kHz signals: Welch's estimate H = S_ty / S_tt over Hann windows of 8192
samples (2.05 s, 0.49 Hz apart) overlapping by half, the speed turned into motor rad/s through the lead (0.04 m, the
estimate of machine.toml), averaged over five neighbouring frequencies, between 15 and 250 Hz. In the closed loop the estimate is that of the plant where the
torque's noise is small against its excitation, as here (the sweep's power).

    python -I tools/darus_ball_screw_to_csv.py --sweep vel_sweep_s_350_dir_pos.mat OUT.csv

Needs h5py and numpy (not dependencies of the project).
"""

import sys

import h5py
import numpy as np

LEAD_M = 0.04


def text(f, ref):
    return "".join(map(chr, f[ref][()].flatten())) if f[ref].dtype == np.uint16 else ""


def read(source):
    f = h5py.File(source, "r")
    g = f[next(k for k in f if k != "#refs#")]
    times = [np.array(f[g["X/Data"][i, 0]][()]).flatten() for i in range(g["X/Data"].shape[0])]
    signals = {}
    for i in range(g["Y/Name"].shape[0]):
        # The first of a name: the recordings name two parameters "Gain".
        signals.setdefault(text(f, g["Y/Name"][i, 0]), np.array(f[g["Y/Data"][i, 0]][()]).flatten())
    return times, signals


def main(source, out):
    times, signals = read(source)
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
    for name, column in [("F_des_LDD", "LDD force setpoint [N]"), ("F_nut_sum", "nut force [N]")]:
        if name in signals and len(signals[name]) >= n:
            columns.append((column, signals[name][:n], "%.3f"))
    with open(out, "w", newline="\n") as w:
        w.write(",".join(c[0] for c in columns) + "\n")
        for row in zip(*(c[1] for c in columns)):
            w.write(",".join(fmt % v for v, (_, _, fmt) in zip(row, columns)) + "\n")


def sweep(source, out, rate=4000.0, length=8192):
    times, signals = read(source)
    torque, speed = signals["M_KGT_act"], signals["v_meas_motor"] * 2.0 * np.pi / LEAD_M
    t4 = next(t for t in times if len(t) == len(torque))
    assert abs(np.median(np.diff(t4)) - 1.0 / rate) < 1e-7, "the fast raster is not 4 kHz"
    window = np.hanning(length)
    stt, sty = 0.0, 0.0
    for k in range(0, len(torque) - length, length // 2):
        t, y = torque[k : k + length], speed[k : k + length]
        tf, yf = np.fft.rfft((t - t.mean()) * window), np.fft.rfft((y - y.mean()) * window)
        stt, sty = stt + np.abs(tf) ** 2, sty + np.conj(tf) * yf
    # Averaged over five neighbouring frequencies (2.4 Hz): the noise left a dozen dips of a few bins below the first mode.
    smooth = lambda s: np.convolve(s, np.ones(5) / 5, "same")
    frequency = np.fft.rfftfreq(length, 1.0 / rate)
    response = smooth(sty) / smooth(stt)
    keep = (frequency >= 15.0) & (frequency <= 250.0)
    with open(out, "w", newline="\n") as w:
        w.write("frequency_hz,gain_rad_s_per_nm,phase_deg\n")
        for f, h in zip(frequency[keep], response[keep]):
            w.write("%.4f,%.6e,%.3f\n" % (f, abs(h), np.degrees(np.angle(h))))


if __name__ == "__main__":
    if sys.argv[1] == "--sweep":
        sweep(sys.argv[2], sys.argv[3])
    else:
        main(sys.argv[1], sys.argv[2])
