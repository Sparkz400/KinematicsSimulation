"""The Python examples of examples/README.md run and print their tables (the sim-cli commands there are run by
crates/sim-cli/tests/examples.rs)."""

import os
import pathlib
import re
import subprocess
import sys

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]


def test_the_set_sweep_example_prints_one_row_per_value():
    out = subprocess.run([sys.executable, str(ROOT / "examples" / "payload_sweep.py")], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout.splitlines()
    assert out[0] == "payload_kg,peak_torque_nm,rms_torque_nm,thermal_utilisation,all_checks_pass"
    assert len(out) == 4, out
    # A heavier payload needs more peak torque.
    peaks = [float(row.split(",")[1]) for row in out[1:]]
    assert peaks == sorted(peaks), peaks


def test_the_fmu_controller_example_follows_its_move(tmp_path):
    # The FMU example of examples/README.md: a PD in torque mode through fmpy. Needs sim-cli and sim_fmi built.
    pytest.importorskip("fmpy")
    cli = ROOT / "target" / "release" / ("sim-cli.exe" if os.name == "nt" else "sim-cli")
    if not cli.exists():
        pytest.skip("build sim-cli and sim-fmi first: cargo build -p sim-cli -p sim-fmi --release")
    # The machine the example's own docstring exports (no machine name outside its crate: the architecture test).
    example = (ROOT / "examples" / "fmu_controller.py").read_text(encoding="utf-8")
    machine = re.search(r"sim-cli fmu (\S+) --out", example).group(1)
    fmu = tmp_path / f"{machine}.fmu"
    subprocess.run([str(cli), "fmu", machine, "--out", str(fmu)], cwd=ROOT, check=True)
    out = subprocess.run([sys.executable, str(ROOT / "examples" / "fmu_controller.py"), str(fmu)], cwd=ROOT,
                         capture_output=True, text=True, check=True).stdout.splitlines()
    # The inertia at the motor as docs/python.md works it out by hand: 6 kg (0.02/5)^2 + 0.26 kg (0.02/5)^2
    # + 1e-4/25 + 0.3e-4 = 1.344e-4 kg m^2.
    assert out[0].startswith("inertia at the motor 0.0001344 "), out[0]
    # The 20 rad move is followed within 0.2 rad (the README shows 0.105 at the peak).
    assert float(out[-1].split()[-2]) < 0.2, out[-1]
