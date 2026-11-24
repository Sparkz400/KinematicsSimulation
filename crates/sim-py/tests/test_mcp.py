"""The MCP server (kinematics_sim.mcp) through the SDK's own client: in process for every tool, and once over stdio."""

import json
import os
import re
import sys
import time
from pathlib import Path

import anyio
import numpy as np
import pytest

mcp_sdk = pytest.importorskip("mcp", reason="the MCP tests need the extra: pip install 'kinematics-sim[mcp]'")

import kinematics_sim as ks  # noqa: E402
import kinematics_sim.mcp as server  # noqa: E402

# The tests name no machine (a rule of the architecture test): the first two of the registry, the two-axis belt
# gantry and the single belt axis.
GANTRY, AXIS = ks.machines()[0]["id"], ks.machines()[1]["id"]
ROOT = Path(__file__).resolve().parents[3]
CORNER = "line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw\nline,0.6,0.05"


def calls(*steps):
    """Each (tool, arguments) through one client session; the results as (is_error, JSON or the error text)."""

    async def go():
        out = []
        async with mcp_sdk.Client(server.mcp) as client:
            for name, args in steps:
                r = await client.call_tool(name, args)
                out.append((r.is_error, r.content[0].text if r.is_error else r.structured_content))
        return out

    return anyio.run(go)


def call(tool, **args):
    error, result = calls((tool, args))[0]
    assert not error, result
    return result


@pytest.fixture(autouse=True)
def no_sessions():
    server._sessions.clear()


def test_the_tools_are_listed_with_descriptions():
    async def go():
        async with mcp_sdk.Client(server.mcp) as client:
            return await client.list_tools()

    tools = {t.name: t.description for t in anyio.run(go).tools}
    assert set(tools) == {
        "list_machines", "describe_machine", "create_session", "list_sessions", "get_config", "set_config_values",
        "config_diff", "check_sizing", "analyze_cycle", "run_path", "find_motor", "optimise_cycle", "loop_margins",
        "positioning_accuracy", "sensitivity", "write_report", "export_fmu",
    }
    assert all(len(d) > 40 for d in tools.values())


def test_sessions_config_and_a_misspelt_key():
    toml = ks.Config(GANTRY).to_toml().replace("payload_kg = 2.0", "payload_kg = 2.5")
    (create, set_ok, misspelt, wrong_type, diff, read, toml_made, copied, listed, diff_between, unknown) = calls(
        ("create_session", {"name": "base", "machine": GANTRY}),
        ("set_config_values", {"session": "base", "values": {"mass.payload_kg": 3.5}}),
        ("set_config_values", {"session": "base", "values": {"mass.paylod_kg": 4.0, "mass.carriage_kg": 9.0}}),
        ("set_config_values", {"session": "base", "values": {"sim.dt_s": "fast"}}),
        ("config_diff", {"session": "base"}),
        ("get_config", {"session": "base", "keys": ["mass.payload_kg", "mass.carriage_kg"], "prefix": "mass."}),
        ("create_session", {"name": "from-toml", "machine": GANTRY, "toml": toml}),
        ("create_session", {"name": "copy", "copy_of": "base"}),
        ("list_sessions", {}),
        ("config_diff", {"session": "from-toml", "against": "base"}),
        ("get_config", {"session": "no-such-session", "prefix": "mass."}),
    )
    assert not create[0] and create[1]["machine"] == GANTRY
    assert set_ok == (False, {"session": "base", "set": {"mass.payload_kg": 3.5}})
    # A misspelt key: refused by name with the key meant, and all or nothing (the carriage mass stays).
    assert misspelt[0]
    assert "mass.paylod_kg" in misspelt[1] and "did you mean mass.payload_kg" in misspelt[1]
    assert "nothing was changed" in misspelt[1]
    assert wrong_type[0] and "sim.dt_s" in wrong_type[1] and "a float" in wrong_type[1]
    assert diff[1]["differences"] == {"mass.payload_kg": {"base": 2.0, "now": 3.5}}
    assert read[1]["values"]["mass.payload_kg"] == 3.5 and read[1]["values"]["mass.carriage_kg"] == 4.0
    assert not toml_made[0] and not copied[0]
    assert {s["session"]: s["values_changed"] for s in listed[1]["sessions"]} == {"base": 1, "from-toml": 1, "copy": 1}
    assert diff_between[1]["differences"] == {"mass.payload_kg": {"base": 3.5, "now": 2.5}}
    assert unknown[0] and "no session or machine named 'no-such-session'" in unknown[1]
    # The whole config as TOML reads back as the same config.
    text = call("get_config", session=GANTRY, as_toml=True)["toml"]
    assert ks.Config(GANTRY, text=text).leaves() == ks.Config(GANTRY).leaves()


@pytest.mark.parametrize("machine", [GANTRY, AXIS])
def test_describe_check_analyze_margins_accuracy(machine):
    cfg = ks.Config(machine)
    about = call("describe_machine", session=machine)
    signals = ks.signals(cfg)["id"]
    assert [m["id"] for m in about["motors"]] and all(f"motor.{m['id']}.torque" in signals for m in about["motors"])
    assert about["axes"] and all(f"axis.{a['id']}.position" in signals for a in about["axes"])
    assert about["settings"]["mass.payload_kg"]["value"] == cfg["mass.payload_kg"]
    # Every estimate with bounds is listed with its value and its [low, high].
    bounds = {f"{table}.{key}": v for table, keys in cfg["bounds"].items() for key, v in keys.items()}
    assert bounds and set(about["estimates_with_bounds"]) == set(bounds)
    for key, e in about["estimates_with_bounds"].items():
        assert e["low_high"] == pytest.approx(bounds[key], rel=1e-3) and e["value"] == pytest.approx(cfg[key], rel=1e-3)
    assert "estimates" in about["notes"]

    # The verdicts as ks.verdicts gives them, tightest first, with a meaning and a margin.
    sizing = call("check_sizing", session=machine, top=500)
    v = ks.verdicts(cfg)
    assert sizing["row_count"] == len(v["check"]) and sizing["passed"] == bool(np.all(v["ok"]))
    rows = sizing["rows"]
    assert all(r["meaning"] and not r["meaning"].startswith("check-") for r in rows)
    margins = [r["margin_share"] for r in rows if r["ok"] and r["margin_share"] is not None]
    assert margins == sorted(margins)
    for r in rows:
        if r["margin_share"] is not None and r["limit"]:
            # The share of the limit left: |limit - value| / |limit|, to the 4 digits given.
            assert abs(r["margin_share"]) == pytest.approx(abs(r["limit"] - r["value"]) / abs(r["limit"]), rel=2e-3)
    top = call("check_sizing", session=machine, top=3)
    assert len(top["rows"]) == 3 and top["rows_left_out"] == sizing["row_count"] - 3

    load = call("analyze_cycle", session=machine)
    a = ks.analyze(cfg)
    assert [m["motor"] for m in load["motors"]] == list(a["motor"])
    for m, peak, rms in zip(load["motors"], a["peak_torque_nm"], a["rms_torque_nm"]):
        assert m["peak_torque_nm"] == pytest.approx(peak, rel=1e-3) and m["rms_torque_nm"] == pytest.approx(rms, rel=1e-3)
        assert {"winding_rise_k", "energy_in_j", "peak_speed_rad_s", "thermal_utilisation"} <= set(m)

    loop = call("loop_margins", session=machine)
    # One row per motor at each run: the config as it is ("") and each other pose the cycle stops at.
    runs: dict[str, list[str]] = {}
    for r in loop["rows"]:
        runs.setdefault(r["run"], []).append(r["motor"])
    motors = sorted(m["id"] for m in about["motors"])
    assert "" in runs and all(sorted(m) == motors for m in runs.values())
    assert loop["row_count"] == len(runs) * len(motors) and all("phase_margin_deg" in r for r in loop["rows"])

    acc = call("positioning_accuracy", session=machine, points=3, repeats=2)
    assert [s["axis"] for s in acc["summary"]] == [x["id"] for x in about["axes"]]
    assert acc["tool_accuracy"] > 0


def test_run_path_gives_the_path_error_and_a_refusal_in_words():
    out = call("run_path", session=GANTRY, path=CORNER)
    a = ks.analyze(ks.Config(GANTRY), path=CORNER)
    assert out["path_error_peak_m"] == pytest.approx(a["path_error_peak_m"][0], rel=1e-3)
    assert out["duration_s"] == pytest.approx(a["cycle_s"][0], rel=1e-3)
    (error, text), = calls(("run_path", {"session": GANTRY, "path": "line,9,9"}))
    assert error and "travel range" in text


def test_sizing_search_optimiser_and_sensitivity_on_the_axis():
    found = call("find_motor", session=AXIS, series="1FK2", top=2)
    s = ks.sizing(ks.Config(AXIS), series="1FK2")
    assert found["candidates_searched"] == len(s["motor"]) and found["rows_left_out"] == len(s["motor"]) - 2
    assert [r["motor"] for r in found["rows"]] == list(s["motor"][:2])

    opt = call("optimise_cycle", session=AXIS, runs=4)
    before, after = opt["cycle_s"]
    assert after < before
    # The changes come as values that set_config_values takes as they are.
    call("create_session", name="fast", machine=AXIS)
    call("set_config_values", session="fast", values={c["setting"]: c["after"] for c in opt["changes"]})

    sens = call("sensitivity", session=AXIS, top=3)
    assert len(sens["rows"]) == 3 and sens["row_count"] > 3 and "parameter" in sens["rows"][0]
    bounded = {f"{t}.{k}" for t, keys in ks.Config(AXIS)["bounds"].items() for k in keys}
    assert all(r["is_estimate"] == (r["parameter"] in bounded) for r in sens["rows"])
    (error, text), = calls(("sensitivity", {"session": AXIS, "method": "guess"}))
    assert error and "morris" in text


def test_report_and_fmu_only_inside_the_output_folder(tmp_path, monkeypatch):
    out_dir = tmp_path / "out"
    monkeypatch.setenv("KINEMATICS_SIM_MCP_OUT_DIR", str(out_dir))
    monkeypatch.delenv("KINEMATICS_SIM_MCP_OVERWRITE", raising=False)
    out = call("write_report", session=AXIS, file_name="axis.html")
    assert Path(out["written"]) == (out_dir / "axis.html").resolve() and Path(out["written"]).stat().st_size > 1000
    assert call("write_report", session=AXIS, file_name="variants/axis.html")["written"].endswith("axis.html")
    refused = calls(
        ("write_report", {"session": AXIS, "file_name": "../escape.html"}),
        ("write_report", {"session": AXIS, "file_name": "variants/../../escape.html"}),
        ("write_report", {"session": AXIS, "file_name": str(tmp_path / "absolute.html")}),
        ("write_report", {"session": AXIS, "file_name": "axis.exe"}),
        ("write_report", {"session": AXIS, "file_name": "axis.html"}),
        ("export_fmu", {"session": AXIS, "file_name": "axis.html"}),
    )
    assert all(error for error, _ in refused), refused
    assert all("output folder" in t for _, t in refused[:3])
    assert "must end in .html" in refused[3][1] and "exists" in refused[4][1] and "must end in .fmu" in refused[5][1]
    assert not (tmp_path / "escape.html").exists() and not (tmp_path / "absolute.html").exists()
    # The operator may allow overwriting.
    monkeypatch.setenv("KINEMATICS_SIM_MCP_OVERWRITE", "1")
    call("write_report", session=AXIS, file_name="axis.html")
    # A link inside the folder that points out of it is refused (where this system lets a test make one).
    outside = tmp_path / "elsewhere"
    outside.mkdir()
    try:
        os.symlink(outside, out_dir / "link", target_is_directory=True)
    except OSError:
        pass
    else:
        (error, text), = calls(("write_report", {"session": AXIS, "file_name": "link/x.html"}))
        assert error and "outside the output folder" in text and not list(outside.iterdir())
    # Without the variable: kinematics-sim-output in the working directory.
    monkeypatch.delenv("KINEMATICS_SIM_MCP_OUT_DIR")
    monkeypatch.chdir(tmp_path)
    assert Path(call("write_report", session=AXIS, file_name="d.html")["written"]).parent.name == "kinematics-sim-output"

    # The FMU's binaries come from the operator, not from the call.
    monkeypatch.setenv("PATH", "")
    monkeypatch.delenv("KINEMATICS_SIM_FMI_BINARIES", raising=False)
    (error, text), = calls(("export_fmu", {"session": AXIS, "file_name": "axis.fmu"}))
    assert error and "KINEMATICS_SIM_FMI_BINARIES" in text
    name = "sim_fmi.dll" if os.name == "nt" else "libsim_fmi.dylib" if sys.platform == "darwin" else "libsim_fmi.so"
    binary = ROOT / "target" / "release" / name
    if not binary.exists():
        pytest.skip("build the FMU binary first: cargo build -p sim-fmi --release")
    monkeypatch.setenv("KINEMATICS_SIM_FMI_BINARIES", str(binary))
    out = call("export_fmu", session=AXIS, file_name="axis.fmu")
    assert Path(out["written"]).stat().st_size > 1000


def test_a_call_past_the_timeout_is_given_up(monkeypatch):
    monkeypatch.setattr(server, "TIMEOUT_S", 0.2)
    start = time.monotonic()
    with pytest.raises(server.ToolError, match="did not finish within"):
        anyio.run(server._call, None, "sleep", time.sleep, 2.0)
    assert time.monotonic() - start < 1.5


def test_resource_and_prompt():
    async def go():
        async with mcp_sdk.Client(server.mcp) as client:
            res = await client.read_resource("kinematics-sim://limits")
            prompt = await client.get_prompt("size_a_motor", {"machine_description": "a 2 kg pick and place"})
            return res, prompt

    res, prompt = anyio.run(go)
    assert "estimates" in res.contents[0].text and "[bounds]" in res.contents[0].text
    assert "2 kg pick and place" in prompt.messages[0].content.text and "check_sizing" in prompt.messages[0].content.text


def test_over_stdio_as_a_client_starts_it():
    params = mcp_sdk.StdioServerParameters(command=sys.executable, args=["-m", "kinematics_sim.mcp"])

    async def go():
        async with mcp_sdk.Client(params) as client:
            tools = await client.list_tools()
            r = await client.call_tool("list_machines", {})
            return tools, r

    tools, r = anyio.run(go)
    assert len(tools.tools) == 17
    assert [m["id"] for m in r.structured_content["machines"]] == [m["id"] for m in ks.machines()]
    json.dumps(r.structured_content)


# --- file settings: read only inside the operator's input folder ------------------------------------------------------


def _machine_with(key):
    """A machine whose default config has `key` (no machine is named in the tests)."""
    return next(m["id"] for m in ks.machines() if key in ks.Config(m["id"]).leaves())


def _toml_with(machine, key, value):
    """The default config of `machine` as TOML with the file setting `key` set to `value`."""
    leaf = key.split(".")[1]
    line = f"{leaf} = {json.dumps(value)}"  # a function as the replacement: a string's backslashes would be escapes
    return re.sub(rf"^{leaf} = .*$", lambda _: line, ks.Config(machine).to_toml(), flags=re.M)


def _refusals(*steps):
    """The error text of steps that must all be refused with one and the same text."""
    out = calls(*steps)
    assert all(error for error, _ in out), out
    texts = {text.split(": ", 1)[1] for _, text in out}  # without the "Error executing tool <name>" lead
    assert len(texts) == 1, out
    return texts.pop()


def test_file_settings_are_refused_outside_the_input_folder(tmp_path, monkeypatch):
    # Shows: a model cannot make the core open a path of its choosing (urdf_file, table_file), nor tell whether it
    # exists: the same fixed text for a file that exists, one that does not and one reached by `..`, on both entry
    # points (also as a table); built-in and empty values and a file inside the folder work.
    arm, cam = _machine_with("arm.urdf_file"), _machine_with("cam.table_file")
    inside = tmp_path / "in"
    inside.mkdir()
    # A machine crate's own URDF, found without naming the machine (architecture.rs forbids that outside its crate).
    (inside / "arm.urdf").write_text(next(ROOT.glob("crates/machine-*/urdf/six-axis.urdf")).read_text())
    (inside / "cam.csv").write_text((ROOT / "examples/press-cam.csv").read_text())
    (tmp_path / "secret.csv").write_text("a,b\n")
    monkeypatch.setenv("KINEMATICS_SIM_MCP_IN_DIR", str(inside))
    paths = [str(tmp_path / "secret.csv"), str(tmp_path / "missing.csv"), str(inside / ".." / "secret.csv"),
             str(inside / ".." / "missing.csv"), "no/such/file"]

    for machine, key in ((arm, "arm.urdf_file"), (cam, "cam.table_file")):
        section, leaf = key.split(".")
        call("create_session", name="s", machine=machine)
        before = server._sessions["s"][key]
        steps = [("create_session", {"name": "t", "machine": machine, "toml": _toml_with(machine, key, p)}) for p in paths]
        steps += [("set_config_values", {"session": "s", "values": {key: p}}) for p in paths]
        steps += [("set_config_values", {"session": "s", "values": {section: {leaf: paths[0]}}})]
        text = _refusals(*steps)
        assert "KINEMATICS_SIM_MCP_IN_DIR" in text and not any(p in text for p in paths)
        assert server._sessions["s"][key] == before and "t" not in server._sessions  # nothing was changed or made

    # Text the server's own TOML parser cannot read is refused before the core sees it (the core's parser might read
    # it and open what it names): no session is made, and the path is not in the answer.
    if sys.version_info >= (3, 11):
        unread = _toml_with(arm, "arm.urdf_file", paths[0]) + "\nbroken = [1, 2\n"
        text = _refusals(("create_session", {"name": "u", "machine": arm, "toml": unread}))
        assert "the TOML text could not be read" in text and paths[0] not in text and "u" not in server._sessions

    # Allowed: the built-in name and the empty value (the defaults), and a file inside the folder, on both paths.
    for machine, key, name in ((arm, "arm.urdf_file", "arm.urdf"), (cam, "cam.table_file", "cam.csv")):
        call("create_session", name="d", machine=machine, toml=ks.Config(machine).to_toml())
        call("set_config_values", session="d", values={key: str(inside / name)})
        call("create_session", name="e", machine=machine, toml=_toml_with(machine, key, str(inside / name)))
        assert server._sessions["e"][key] == str(inside / name)

    # Without the variable nothing but the defaults is allowed.
    monkeypatch.delenv("KINEMATICS_SIM_MCP_IN_DIR")
    _refusals(("set_config_values", {"session": "d", "values": {"cam.table_file": str(inside / "cam.csv")}}))


def test_a_link_out_of_the_input_folder_is_refused(tmp_path, monkeypatch):
    # Shows: the path is compared after resolving it, so a symlink inside the folder to a file outside is refused.
    inside = tmp_path / "in"
    inside.mkdir()
    (tmp_path / "out.csv").write_text("a,b\n")
    try:
        (inside / "link.csv").symlink_to(tmp_path / "out.csv")
    except OSError:
        pytest.skip("no symlinks here")
    monkeypatch.setenv("KINEMATICS_SIM_MCP_IN_DIR", str(inside))
    call("create_session", name="c", machine=_machine_with("cam.table_file"))
    _refusals(("set_config_values", {"session": "c", "values": {"cam.table_file": str(inside / "link.csv")}}))


def test_a_file_over_the_size_limit_is_refused(tmp_path, monkeypatch):
    monkeypatch.setenv("KINEMATICS_SIM_MCP_IN_DIR", str(tmp_path))
    monkeypatch.setattr(server, "MAX_INPUT_BYTES", 3)
    (tmp_path / "big.csv").write_text("a,b\n1,2\n")
    call("create_session", name="c", machine=_machine_with("cam.table_file"))
    _refusals(("set_config_values", {"session": "c", "values": {"cam.table_file": str(tmp_path / "big.csv")}}))


def test_loop_margins_keeps_the_worst_rows_when_it_caps(monkeypatch):
    # Shows: the rows are sorted (failing first, then the smallest phase margin, an unmeasurable one after the
    # measured ones) before the cap, so the worst row survives `top`, and rows_left_out counts the rest.
    fake = {
        "run": [""] * 6, "motor": list("abcdef"), "ok": np.array([True, True, True, True, False, True]),
        "phase_margin_deg": np.array([60.0, 45.0, float("nan"), 30.0, 10.0, 50.0]),
    }
    monkeypatch.setattr(ks, "margins", lambda cfg, bands=False: fake)
    out = call("loop_margins", session=GANTRY, top=2)
    assert [r["motor"] for r in out["rows"]] == ["e", "d"]
    assert out["row_count"] == 6 and out["rows_left_out"] == 4
    assert [r["motor"] for r in call("loop_margins", session=GANTRY, top=6)["rows"]] == ["e", "d", "b", "f", "a", "c"]


def test_a_call_given_up_at_the_timeout_is_cancelled_and_its_thread_ends(monkeypatch):
    # Uncancelled this 3000-run band calculation takes minutes on all processors (it used to be left running to its
    # end after the timeout); with the call's job cancelled it ends with the runs under way, within seconds.
    import threading

    finished = threading.Event()

    def long_call():
        try:
            return ks.verdicts(ks.Config(GANTRY), bands=True, samples=3000)
        finally:
            finished.set()

    monkeypatch.setattr(server, "TIMEOUT_S", 1.0)
    with pytest.raises(server.ToolError, match="did not finish within"):
        anyio.run(server._call, None, "verdicts", long_call)
    assert finished.wait(15), "the thread of the given-up call still computes"


def test_a_panic_of_the_core_is_an_error_of_the_tool_not_the_end_of_the_server():
    class PanicException(BaseException):  # pyo3_runtime.PanicException, which cannot be imported by name
        pass

    def broken():
        raise PanicException("the model panicked")

    with pytest.raises(server.ToolError, match="failed internally"):
        anyio.run(server._call, None, "margins", broken)


def test_the_sessions_are_bounded_and_the_least_recently_used_is_dropped():
    names = [f"s{i}" for i in range(server.MAX_SESSIONS)]
    steps = [("create_session", {"name": n, "machine": AXIS}) for n in names]
    steps += [("get_config", {"session": names[0], "prefix": "mass."})]
    steps += [("create_session", {"name": "one-more", "machine": AXIS})]
    results = calls(*steps)
    assert not any(error for error, _ in results)
    last = results[-1][1]
    assert last["dropped_session"] == names[1] and len(last["sessions"]) == server.MAX_SESSIONS
    assert names[0] in server._sessions and names[1] not in server._sessions
    # A session that is replaced is not a new one.
    ((_, replaced),) = calls(("create_session", {"name": "one-more", "machine": AXIS}))
    assert "dropped_session" not in replaced


def test_the_counts_that_multiply_the_work_are_bounded():
    runs, points, repeats = calls(
        ("optimise_cycle", {"session": AXIS, "runs": server.MAX_RUNS + 1}),
        ("positioning_accuracy", {"session": AXIS, "points": server.MAX_POINTS + 1}),
        ("positioning_accuracy", {"session": AXIS, "repeats": server.MAX_REPEATS + 1}),
    )
    assert runs[0] and f"limit of {server.MAX_RUNS}" in runs[1]
    assert points[0] and f"limit of {server.MAX_POINTS}" in points[1]
    assert repeats[0] and f"limit of {server.MAX_REPEATS}" in repeats[1]
