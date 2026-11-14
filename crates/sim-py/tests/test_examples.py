"""The Python example of examples/README.md runs and prints its table (the sim-cli commands there are run by
crates/sim-cli/tests/examples.rs)."""

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]


def test_the_set_sweep_example_prints_one_row_per_value():
    out = subprocess.run([sys.executable, str(ROOT / "examples" / "payload_sweep.py")], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout.splitlines()
    assert out[0] == "payload_kg,peak_torque_nm,rms_torque_nm,thermal_utilisation,all_checks_pass"
    assert len(out) == 4, out
    # A heavier payload needs more peak torque.
    peaks = [float(row.split(",")[1]) for row in out[1:]]
    assert peaks == sorted(peaks), peaks
