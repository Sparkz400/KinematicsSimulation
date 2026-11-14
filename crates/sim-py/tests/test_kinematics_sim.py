"""The Python module against the command line it mirrors: same machines, same numbers."""

import csv
import io
import os
import subprocess
import sys
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


def test_a_machine_file_runs_as_with_the_cli():
    # The first machine file of machines/, as `sim-cli --machine-file` takes it.
    path = sorted((ROOT / "machines").glob("*.toml"))[0]
    cfg = ks.Config(machine_file=path)
    data = ks.run(cfg, cycle=True, time=0.2, every=16)
    rows = cli("run", "--machine-file", str(path), "--cycle", "--time", "0.2", "--every", "16")
    column = list(rows[0])[5]
    np.testing.assert_allclose(data[column.split("[")[0]], [float(r[column]) for r in rows], rtol=1e-8, atol=1e-12)
    with pytest.raises(ValueError):
        ks.Config(FIRST, machine_file=path)
    with pytest.raises(ValueError):
        ks.Config()


def test_a_machine_file_with_an_error_names_its_line(tmp_path):
    bad = tmp_path / "bad.toml"
    bad.write_text("this is = not a machine\n", encoding="utf-8")
    with pytest.raises(ValueError, match="bad.toml"):
        ks.Config(machine_file=bad)


def test_an_integer_beyond_64_bits_is_an_overflow_not_a_float():
    cfg = ks.Config(FIRST)
    integer = next(k for k, v in cfg.leaves().items() if type(v) is int)
    before = cfg[integer]
    # 2**63 is one more than the largest 64-bit integer, 9223372036854775807; before, it became 9.223372036854776e18.
    for big in (2**63, -(2**63) - 1, np.uint64(2**63)):
        with pytest.raises(OverflowError):
            cfg[integer] = big
    assert cfg[integer] == before and type(cfg[integer]) is int


def test_a_run_too_long_to_finish_or_to_hold_is_refused():
    cfg = ks.Config(FIRST)
    dt = cfg["sim.dt_s"]
    # 1e12 s are 1e12 / dt base steps, far beyond the 1e8 allowed: refused at once instead of running for ever.
    with pytest.raises(ValueError, match="base steps"):
        ks.run(cfg, time=1e12)
    # 1e8 * dt seconds sampled every step: 1e8 rows of every signal, far beyond 2.5e8 values.
    with pytest.raises(ValueError, match="values"):
        ks.run(cfg, time=1e8 * dt, every=1)


def test_sweep_matches_the_cli():
    cfg = ks.Config(FIRST)
    result = ks.sweep(cfg, from_hz=10.0, to_hz=200.0, count=6)
    rows = cli("sweep", FIRST, "--from", "10", "--to", "200", "--count", "6")
    for column in ("frequency_hz", "gain_rad_s_per_nm", "phase_deg"):
        np.testing.assert_allclose(result[column], [float(r[column]) for r in rows], rtol=1e-3, atol=1e-3, err_msg=column)
    with pytest.raises(ValueError):
        ks.sweep(cfg, from_hz=0.0)


def test_optimise_matches_the_cli():
    cfg = ks.Config(FIRST)
    result = ks.optimise(cfg, runs=3, nominal=True)
    rows = {r["setting"]: r for r in cli("optimise", FIRST, "--runs", "3", "--nominal")}
    before, after = result["cycle_s"]
    assert (round(before, 4), round(after, 4)) == (float(rows["cycle_s"]["before"]), float(rows["cycle_s"]["after"]))
    assert after <= before and result["robust"] is None
    assert [s for s in result["changes"]["setting"]] == [s for s in rows if s not in ("cycle_s", "dwell_error_peak_m")]


@pytest.fixture(scope="module")
def trace(tmp_path_factory):
    """One pass of FIRST's cycle as the CLI writes it, with no payload and exact encoders (as the CLI's own fit test)."""
    path = tmp_path_factory.mktemp("trace") / "trace.csv"
    cli("run", FIRST, "--cycle", "--time", "6.2", "--out", str(path), *SETS)
    return path


SETS = ("--set", "effects.payload=false", "--set", "effects.encoder_resolution=false")


def test_fit_matches_the_cli(trace):
    cfg = ks.Config(FIRST)
    cfg["effects.payload"] = False
    cfg["effects.encoder_resolution"] = False
    result = ks.fit(cfg, trace_file=trace)
    rows = cli("fit", FIRST, "--trace", str(trace), *SETS)
    assert result["quantity"] == [r["quantity"] for r in rows] and result["of"] == [r["of"] for r in rows]
    np.testing.assert_allclose(result["fitted"], [float(r["fitted"]) for r in rows], rtol=1e-5)
    np.testing.assert_allclose(result["standard_error"], [float(r["standard_error"]) for r in rows], rtol=1e-2)
    assert result["advice"] and all(isinstance(a, str) for a in result["advice"])
    with pytest.raises(ValueError):
        ks.fit(cfg)
    with pytest.raises(ValueError):
        ks.fit(cfg, trace_file=trace.with_name("missing.csv"))


def test_identify_matches_the_cli(trace):
    cfg = ks.Config(FIRST)
    result = ks.identify(trace_file=trace, config=cfg)
    rows = cli("identify", FIRST, "--trace", str(trace))
    assert result["quantity"] == [r["quantity"] for r in rows]
    np.testing.assert_allclose(result["value"], [float(r["value"]) for r in rows], rtol=1e-5)
    assert len(result["overlay"]["time"]) > 0 and all(len(v) == len(result["overlay"]["time"]) for v in result["overlay"].values())
    with pytest.raises(ValueError):
        ks.identify(trace_file=trace)


def test_the_example_notebook_runs():
    nbformat = pytest.importorskip("nbformat")
    nbclient = pytest.importorskip("nbclient")
    pytest.importorskip("matplotlib")
    nb = nbformat.read(ROOT / "crates" / "sim-py" / "examples" / "parameter_sweep.ipynb", as_version=4)
    nbclient.NotebookClient(nb, timeout=900, kernel_name="python3").execute()
    assert not [o for c in nb.cells if c.cell_type == "code" for o in c.outputs if o.output_type == "error"]


def cli_run(*args: str) -> subprocess.CompletedProcess:
    """sim-cli as it is, for commands that may fail or write no CSV."""
    if not CLI.exists():
        pytest.skip("build sim-cli first: cargo build -p sim-cli --release")
    return subprocess.run([str(CLI), *args], capture_output=True, text=True)


def floats(rows: list[dict[str, str]], column: str) -> list[float]:
    """A column of the CLI's CSV; an empty cell is NaN, as the module gives it."""
    return [float(r[column]) if r[column] != "" else float("nan") for r in rows]


def configured() -> "ks.Config":
    """FIRST as the trace fixture ran it: no payload, exact encoders."""
    cfg = ks.Config(FIRST)
    cfg["effects.payload"] = False
    cfg["effects.encoder_resolution"] = False
    return cfg


def test_step_matches_the_cli():
    cfg = ks.Config(FIRST)
    result = ks.step(cfg, size=0.002, time=0.2)
    rows = cli("step", FIRST, "--size", "0.002", "--time", "0.2")
    # The CLI prints the time with 6 decimals and the response with 10 digits.
    np.testing.assert_allclose(result["time_s"], floats(rows, "time_s"), atol=1e-6)
    np.testing.assert_allclose(result["response"], floats(rows, "response"), rtol=1e-8, atol=1e-15)
    assert np.all(result["setpoint"] == 0.002) and 0 < result["rise_time_s"] < 0.2
    with pytest.raises(ValueError):
        ks.step(cfg, axis_index=99)
    with pytest.raises(ValueError):
        ks.step(cfg, size=0.0)


def test_bode_matches_the_cli():
    result = ks.bode(ks.Config(FIRST), from_hz=5.0, to_hz=50.0, count=3)
    rows = cli("bode", FIRST, "--from", "5", "--to", "50", "--count", "3")
    np.testing.assert_allclose(result["frequency_hz"], floats(rows, "frequency_hz"), atol=1e-4)
    np.testing.assert_allclose(result["gain"], floats(rows, "gain"), rtol=1e-6)
    np.testing.assert_allclose(result["gain_db"], floats(rows, "gain_db"), atol=1e-3)
    np.testing.assert_allclose(result["phase_deg"], floats(rows, "phase_deg"), atol=1e-3)


def test_tuning_matches_the_cli():
    result = ks.tuning(ks.Config(FIRST))
    rows = cli("tuning", FIRST)
    assert result["axis"] == [r["axis"] for r in rows]
    # As many decimals as the CLI prints: half a unit of the last one apart at most.
    for column, places in (("rise_time_s", 5), ("overshoot", 4), ("settling_time_s", 5), ("bandwidth_hz", 3)):
        np.testing.assert_allclose(result[column], floats(rows, column), atol=0.51 * 10**-places, err_msg=column)


def test_margins_match_the_cli():
    result = ks.margins(ks.Config(FIRST))
    rows = cli("margins", FIRST)
    assert result["run"] == [r["run"] for r in rows] and result["motor"] == [r["motor"] for r in rows]
    assert [("ok" if ok else "FAIL") for ok in result["ok"]] == [r["result"] for r in rows]
    for column, places in (("crossover_hz", 1), ("phase_margin_deg", 1), ("gain_margin_db", 2), ("peak_sensitivity", 3)):
        np.testing.assert_allclose(result[column], floats(rows, column), atol=0.51 * 10**-places, err_msg=column)


def test_modes_match_the_cli():
    result = ks.modes(ks.Config(FIRST), points=4)
    rows = cli("modes", FIRST, "--points", "4")
    assert list(result) == list(rows[0])
    for column in rows[0]:
        np.testing.assert_allclose(result[column], floats(rows, column), atol=1e-4, err_msg=column)
    with pytest.raises(ValueError):
        ks.modes(ks.Config(FIRST), points=1)


def test_sensitivity_matches_the_cli():
    cfg = ks.Config(FIRST)
    group = next(k for k in cfg.leaves() if k.endswith("payload_kg")).split(".")[0]
    result = ks.sensitivity(cfg, groups=[group])
    rows = cli("sensitivity", FIRST, "--group", group)
    assert result["parameter"] == [r["parameter"] for r in rows] and len(rows) > 0
    np.testing.assert_allclose(result["low"], floats(rows, "low"))
    for side in ("low", "high"):
        for name in ("peak_torque", "rms_torque", "cycle_time"):
            column = f"{name}_{side}"
            np.testing.assert_allclose(result[column], floats(rows, column), atol=0.51e-4, err_msg=column)
        column = f"belt_tension_min_{side}_n"
        np.testing.assert_allclose(result[column], floats(rows, column), atol=0.0051, err_msg=column)


def test_replay_matches_the_cli(trace, tmp_path):
    result = ks.replay(configured(), trace_file=trace)
    overlay = tmp_path / "overlay.csv"
    rows = cli("replay", FIRST, "--trace", str(trace), "--out", str(overlay), *SETS)
    assert result["motor"] == [r["motor"] for r in rows]
    for column in ("torque_rms_nm", "torque_peak_nm", "following_error_rms_rad", "following_error_peak_rad"):
        np.testing.assert_allclose(result[column], floats(rows, column), rtol=1e-6, atol=1e-15, err_msg=column)
    written = list(csv.DictReader(overlay.open(encoding="utf-8")))
    assert list(result["overlay"]) == list(written[0])
    for column in written[0]:
        np.testing.assert_allclose(result["overlay"][column], floats(written, column), rtol=1e-8, atol=1e-15, err_msg=column)


def test_compare_matches_the_cli(tmp_path):
    cfg = ks.Config(FIRST)
    signals = ks.signals(cfg)
    signal, unit = signals["id"][5], signals["unit"][5]
    result = ks.compare(cfg, effect="payload", signals=[signal])
    differences = tmp_path / "differences.csv"
    rows = cli("compare", FIRST, "--effect", "payload", "--signal", signal, "--differences", str(differences))
    for side in ("on", "off"):
        np.testing.assert_allclose(result[f"{signal}.{side}"], floats(rows, f"{signal}.{side}[{unit}]"), rtol=1e-8, atol=1e-15)
    written = list(csv.DictReader(differences.open(encoding="utf-8")))
    assert result["differences"]["signal"] == [r["signal"] for r in written]
    for column in ("peak_difference", "rms_difference"):
        np.testing.assert_allclose(result["differences"][column], floats(written, column), rtol=1e-6, atol=1e-15)
    with pytest.raises(KeyError):
        ks.compare(cfg, effect="payload", signals=["no.such.signal"])
    with pytest.raises(ValueError):
        ks.compare(cfg, effect="no_such_effect")


def test_report_matches_the_cli(tmp_path):
    mine, theirs = tmp_path / "python.html", tmp_path / "cli.html"
    passed = ks.report(ks.Config(FIRST), mine)
    done = cli_run("report", FIRST, "--out", str(theirs))
    assert done.returncode == 0, done.stderr
    # The same page, dated the same day.
    assert mine.read_text(encoding="utf-8") == theirs.read_text(encoding="utf-8")
    assert passed == ("every check passes" in done.stderr)
    pdf = tmp_path / "report.pdf"
    ks.report(ks.Config(FIRST), pdf, lang="de")
    assert pdf.read_bytes().startswith(b"%PDF")


def test_check_dataset_matches_the_cli(tmp_path):
    for folder in (ROOT / "datasets" / "example-synthetic-belt-axis", tmp_path):
        problems = ks.check_dataset(folder)
        done = cli_run("check-dataset", str(folder))
        # The CLI prints one line per problem, or one saying the folder is ready, and fails with problems.
        assert (done.returncode == 0) == (problems == []), done.stdout
        assert len(done.stdout.splitlines()) == max(len(problems), 1)
    assert ks.check_dataset(tmp_path), "an empty folder is no dataset"


def test_fmu_matches_the_cli(tmp_path):
    name = "sim_fmi.dll" if os.name == "nt" else "libsim_fmi.dylib" if sys.platform == "darwin" else "libsim_fmi.so"
    binary = CLI.with_name(name)
    if not binary.exists():
        pytest.skip("build the FMU binary first: cargo build -p sim-fmi --release")
    for fmi in (2, 3):
        mine, theirs = tmp_path / f"python-{fmi}.fmu", tmp_path / f"cli-{fmi}.fmu"
        ks.fmu(ks.Config(FIRST), mine, binaries=[binary], fmi=fmi)
        done = cli_run("fmu", FIRST, "--fmi", str(fmi), "--out", str(theirs))
        assert done.returncode == 0, done.stderr
        assert mine.read_bytes() == theirs.read_bytes(), f"FMI {fmi}"
    with pytest.raises(ValueError):
        ks.fmu(ks.Config(FIRST), tmp_path / "none.fmu", binaries=[])
    with pytest.raises(ValueError):
        ks.fmu(ks.Config(FIRST), tmp_path / "four.fmu", binaries=[binary], fmi=4)


def test_verdicts_at_the_worst_corner_match_the_cli():
    cfg = ks.Config(FIRST)
    result = ks.verdicts(cfg, bands=True, worst=True)
    rows = cli("verdicts", FIRST, "--bands", "--worst")
    assert result["check"] == [r["check"] for r in rows] and result["runs"] >= 1
    assert [("ok" if ok else "FAIL") for ok in result["worst_ok"]] == [r["worst_case"] for r in rows]
    assert [";".join(b) for b in result["at_high_bound"]] == [r["at_high_bound"] for r in rows]
    assert [";".join(b) for b in result["at_low_bound"]] == [r["at_low_bound"] for r in rows]
    for bad in ({"worst": True}, {"worst": True, "bands": True, "samples": 3}):
        with pytest.raises(ValueError):
            ks.verdicts(cfg, **bad)


def test_sizing_of_one_motor_and_by_vendor_match_the_cli():
    # The first machine file of machines/ with two motors or more; its last motor sized on its own.
    files = sorted((ROOT / "machines").glob("*.toml"))
    path = next(p for p in files if len(ks.analyze(ks.Config(machine_file=p))["motor"]) > 1)
    cfg = ks.Config(machine_file=path)
    motor = ks.analyze(cfg)["motor"][-1]
    result = ks.sizing(cfg, motor_id=motor, series="1FK2104")
    rows = cli("search", "--machine-file", str(path), "--motor-id", motor, "--series", "1FK2104")
    assert result["motor"] == [r["motor"] for r in rows] and len(rows) > 0
    assert [("ok" if p else "FAIL") for p in result["passes"]] == [r["result"] for r in rows]
    np.testing.assert_allclose(result["peak_torque_nm"], floats(rows, "peak_torque_nm"), atol=0.51e-4)
    np.testing.assert_allclose(result["winding_rise_k"], floats(rows, "winding_rise_k"), atol=0.0051)
    with pytest.raises(ValueError):
        ks.sizing(cfg, motor_id=motor, axes=True)
    by_vendor = ks.sizing(ks.Config(LAST), vendor="siemens", series="1FK2104")
    rows = cli("search", LAST, "--vendor", "siemens", "--series", "1FK2104")
    assert by_vendor["supply"] == [r["supply"] for r in rows] and by_vendor["motor"] == [r["motor"] for r in rows]
    assert set(by_vendor["vendor"]) == {"siemens"}


def test_fit_and_identify_return_what_they_found_beside_the_fits_that_found_nothing(trace, tmp_path):
    # A pure inertia of 0.01 kg m2: gain 1 / (J 2 pi f), no dip and no peak, so no resonance and no two-mass model.
    flat = tmp_path / "inertia.csv"
    lines = "".join(f"{f!r},{1 / (0.01 * 2 * np.pi * f)!r},-90\n" for f in np.geomspace(5.0, 500.0, 8).tolist())
    flat.write_text("frequency_hz,gain_rad_s_per_nm,phase_deg\n" + lines, encoding="utf-8")
    result = ks.fit(configured(), trace_file=trace, sweep_file=flat)
    done = cli_run("fit", FIRST, "--trace", str(trace), "--sweep", str(flat), *SETS)
    # The CLI writes what the trace gave, then fails; the module returns the same rows and the failure.
    assert done.returncode != 0
    rows = list(csv.DictReader(io.StringIO(done.stdout)))
    assert len(rows) > 0 and result["quantity"] == [r["quantity"] for r in rows]
    assert len(result["errors"]) == 1 and result["errors"][0] in done.stderr
    found = ks.identify(trace_file=trace, config=ks.Config(FIRST), sweep_file=flat)
    done = cli_run("identify", FIRST, "--trace", str(trace), "--sweep", str(flat))
    assert done.returncode != 0
    assert found["quantity"] == [r["quantity"] for r in csv.DictReader(io.StringIO(done.stdout))]
    assert len(found["errors"]) == 1 and found["errors"][0] in done.stderr
    assert ks.fit(configured(), trace_file=trace)["errors"] == []
