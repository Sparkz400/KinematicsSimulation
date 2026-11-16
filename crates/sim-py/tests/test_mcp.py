"""The MCP server (kinematics_sim.mcp) through the SDK's own client: in process for every tool, and once over stdio."""

import json
import os
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
    assert loop["row_count"] == len(about["motors"]) and all("phase_margin_deg" in r for r in loop["rows"])

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


def test_report_and_fmu_written_where_asked(tmp_path):
    out = call("write_report", session=AXIS, path=str(tmp_path / "axis.html"))
    assert out["every_check_passed"] in (True, False) and (tmp_path / "axis.html").stat().st_size > 1000
    (error, text), = calls(("write_report", {"session": AXIS, "path": "relative.html"}))
    assert error and "absolute path" in text
    name = "sim_fmi.dll" if os.name == "nt" else "libsim_fmi.dylib" if sys.platform == "darwin" else "libsim_fmi.so"
    binary = ROOT / "target" / "release" / name
    if not binary.exists():
        pytest.skip("build the FMU binary first: cargo build -p sim-fmi --release")
    out = call("export_fmu", session=AXIS, path=str(tmp_path / "axis.fmu"), binaries=[str(binary)])
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
