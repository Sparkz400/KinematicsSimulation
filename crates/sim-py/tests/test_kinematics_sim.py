"""The Python module against the command line it mirrors: same machines, same numbers."""

import csv
import io
import os
import subprocess
import threading
from pathlib import Path

import numpy as np
import pytest

import kinematics_sim as ks

# The tests name no machine (a rule of the architecture test): the first and the last of the registry.
FIRST, LAST = ks.machines()[0]["id"], ks.machines()[-1]["id"]
ROOT = Path(__file__).resolve().parents[3]
CLI = ROOT / "target" / "release" / ("sim-cli.exe" if os.name == "nt" else "sim-cli")


def cli(*args: str) -> list[dict[str, str]]:
    if not CLI.exists():
        pytest.skip("build sim-cli first: cargo build -p sim-cli --release")
    out = subprocess.run([str(CLI), *args], capture_output=True, text=True, check=True).stdout
    return list(csv.DictReader(io.StringIO(out)))


def test_machines_lists_the_registry():
    assert len(ks.machines()) >= 2 and len({m["id"] for m in ks.machines()}) == len(ks.machines())
    assert all(m["name"] for m in ks.machines("de"))
    with pytest.raises(ValueError):
        ks.machines("fr")


def test_config_get_set_and_refusals():
    cfg = ks.Config(FIRST)
    assert cfg.machine == FIRST
    dt = cfg["sim.dt_s"]
    assert isinstance(dt, float) and dt > 0
    cfg["sim.dt_s"] = dt
    cfg.set(f"sim.dt_s = {dt!r}")
    effect = next(k for k, v in cfg.leaves().items() if k.startswith("effects.") and isinstance(v, bool))
    other = cfg.copy()
    other[effect] = not cfg[effect]
    assert other[effect] != cfg[effect]
    # A misspelt key is refused by name, as in a config file.
    with pytest.raises(ValueError, match="no_such"):
        cfg.set("sim.no_such_key = 1")
    with pytest.raises(KeyError):
        cfg["sim.no_such_key_either"]
    with pytest.raises(KeyError):
        ks.Config("no-such-machine")
    # TOML round trip.
    again = ks.Config(FIRST, text=cfg.to_toml())
    assert again.leaves() == cfg.leaves()


def test_parts_set_one_after_the_other_come_in_as_with_the_cli():
    sets = ['parts.motor = "1FK2104-5AK"', 'parts.motor_supply = "230v"']
    rows = cli("analyze", FIRST, *[a for s in sets for a in ("--set", s)])
    for order in (sets, sets[::-1]):
        cfg = ks.Config(FIRST)
        for assignment in order:
            cfg.set(assignment)
        result = ks.analyze(cfg)
        for column in ("peak_torque_nm", "rms_torque_nm", "dc_link_peak_voltage_v"):
            np.testing.assert_allclose(result[column], [float(r[column]) for r in rows], rtol=1e-4, err_msg=f"{order}: {column}")


def test_a_value_set_with_a_part_stays_on_top_in_either_order_as_with_the_cli():
    # sim-cli brings the named parts in first and applies every --set on top, whatever the order on the line.
    part, values = 'parts.motor = "1FK2104-5AK"', ["motor.rotor_inertia_kgm2 = 5e-4", "drive.velocity_kp_nm_s_rad = 0.5"]
    rows = cli("analyze", FIRST, "--motor", "1FK2104-5AK", *[a for v in values for a in ("--set", v)])
    for order in ([part, *values], [*values, part]):
        cfg = ks.Config(FIRST)
        for assignment in order:
            cfg.set(assignment)
        assert cfg["motor.rotor_inertia_kgm2"] == 5e-4
        result = ks.analyze(cfg)
        np.testing.assert_allclose(result["peak_torque_nm"], [float(r["peak_torque_nm"]) for r in rows], rtol=1e-4, err_msg=str(order))


def test_numpy_scalars_keep_their_type():
    cfg = ks.Config(FIRST)
    leaves = cfg.leaves()
    integer = next(k for k, v in leaves.items() if type(v) is int)
    flag = next(k for k, v in leaves.items() if type(v) is bool)
    cfg[integer] = np.int64(2)
    assert cfg[integer] == 2 and type(cfg[integer]) is int
    cfg[flag] = np.False_
    assert cfg[flag] is False
    cfg["sim.dt_s"] = np.float64(cfg["sim.dt_s"])
    # A bool is not a number: refused on a float key, not stored as 1.0.
    with pytest.raises(ValueError):
        cfg["sim.dt_s"] = np.True_


def test_config_from_file(tmp_path):
    path = tmp_path / "c.toml"
    path.write_text(ks.Config(LAST).to_toml(), encoding="utf-8")
    assert ks.Config(LAST, path=path)["sim.dt_s"] == ks.Config(LAST)["sim.dt_s"]


def test_run_returns_numpy_arrays_like_the_cli():
    cfg = ks.Config(FIRST)
    data = ks.run(cfg, cycle=True, time=0.5, every=16)
    names = ks.signals(cfg)["id"]
    assert list(data) == names and names[0] == "time"
    assert all(isinstance(v, np.ndarray) and v.dtype == np.float64 for v in data.values())
    rows = cli("run", FIRST, "--cycle", "--time", "0.5", "--every", "16")
    assert len(rows) == len(data["time"])
    first = list(rows[0])[5]  # e.g. an axis position column "axis.x.position[m]"
    signal = first.split("[")[0]
    np.testing.assert_allclose(data[signal], [float(r[first]) for r in rows], rtol=1e-8, atol=1e-12)


def test_run_motion_arguments():
    cfg = ks.Config(FIRST)
    with pytest.raises(ValueError):
        ks.run(cfg, cycle=True, move_to=[0.1, 0.1])
    with pytest.raises(ValueError):
        ks.run(cfg, path="this is not a path")
    data = ks.run(cfg, disable_at=0.1, time=0.2)
    assert len(data["time"]) > 1
    # Refusals are ValueError, as the CLI refuses them.
    for bad in ({"disable_at": float("nan")}, {"disable_at": -1.0}, {"disable_at": float("inf")}, {"every": -1}, {"every": 0}):
        with pytest.raises(ValueError):
            ks.run(cfg, time=0.1, **bad)


def test_analyze_matches_the_cli():
    cfg = ks.Config(FIRST)
    result = ks.analyze(cfg, points=True)
    rows = cli("analyze", FIRST)
    assert result["motor"] == [r["motor"] for r in rows]
    np.testing.assert_allclose(result["peak_torque_nm"], [float(r["peak_torque_nm"]) for r in rows], rtol=1e-5)
    np.testing.assert_allclose(result["rms_torque_nm"], [float(r["rms_torque_nm"]) for r in rows], rtol=1e-5)
    assert all(p.ndim == 2 and p.shape[1] == 2 for p in result["points"])
    pd = pytest.importorskip("pandas")
    frame = pd.DataFrame({k: v for k, v in result.items() if k != "points"})
    assert len(frame) == len(rows)


def test_verdicts_and_bands_match_the_cli():
    cfg = ks.Config(FIRST)
    plain = ks.verdicts(cfg)
    rows = cli("verdicts", FIRST)
    assert plain["check"] == [r["check"] for r in rows]
    assert [("ok" if ok else "FAIL") for ok in plain["ok"]] == [r["result"] for r in rows]
    banded = ks.verdicts(cfg, bands=True)
    assert np.all(banded["low"] <= banded["value"] + 1e-12) and np.all(banded["value"] <= banded["high"] + 1e-12)
    with pytest.raises(ValueError):
        ks.verdicts(cfg, samples=3)


def test_a_heavier_payload_raises_the_torque():
    light, heavy = ks.Config(FIRST), ks.Config(FIRST)
    key = next(k for k in light.leaves() if k.endswith("payload_kg"))
    heavy[key] = light[key] * 3 + 1
    assert ks.analyze(heavy)["rms_torque_nm"].max() > ks.analyze(light)["rms_torque_nm"].max()


def test_sizing_ranks_passing_motors_first():
    found = ks.sizing(ks.Config(LAST))
    assert len(found["motor"]) > 0
    passes = list(found["passes"])
    assert passes == sorted(passes, reverse=True)


def test_calls_release_the_gil():
    cfg = ks.Config(LAST)
    results = []
    threads = [threading.Thread(target=lambda: results.append(ks.analyze(cfg)["peak_torque_nm"])) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    np.testing.assert_array_equal(results[0], results[1])


def test_the_example_notebook_runs():
    nbformat = pytest.importorskip("nbformat")
    nbclient = pytest.importorskip("nbclient")
    pytest.importorskip("matplotlib")
    nb = nbformat.read(ROOT / "crates" / "sim-py" / "examples" / "parameter_sweep.ipynb", as_version=4)
    nbclient.NotebookClient(nb, timeout=900, kernel_name="python3").execute()
    assert not [o for c in nb.cells if c.cell_type == "code" for o in c.outputs if o.output_type == "error"]
