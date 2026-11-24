"""An MCP server on top of kinematics_sim, for LLM clients (Claude Desktop and Code, Cursor, any MCP client); docs/mcp.md.

Run it with `kinematics-sim-mcp` (stdio) after `pip install "kinematics-sim[mcp]"`, or without installing:
`uvx --from "kinematics-sim[mcp]" kinematics-sim-mcp`.

Configs live in named sessions in this process's memory, so a conversation can keep and compare variants. Every result
is compact JSON with the units in the keys, tables capped to their top rows, and a "notes" field with what the numbers
cannot claim.
"""

from __future__ import annotations

import difflib
import json
import math
import os
import shutil
import sys
from functools import partial
from pathlib import Path
from typing import Any

import anyio
import anyio.to_thread
import numpy as np
from mcp.server.mcpserver import Context, MCPServer
from mcp.server.mcpserver.exceptions import ToolError

import kinematics_sim as ks

# A call that runs longer is given up and its job cancelled: no new run starts, the thread ends once the runs under way do.
TIMEOUT_S = float(os.environ.get("KINEMATICS_SIM_MCP_TIMEOUT_S", "900"))
MAX_ROWS = 25
# Bounds that keep one call (or a client's retries) from using the machine for hours, and the sessions, which live in
# this process, from growing without end. The counts multiply the runs of a tool; all are far above the defaults.
MAX_SESSIONS = 32  # a 33rd drops the least recently used (there is no tool to delete one)
MAX_RUNS = 200  # optimise_cycle `runs` (default 40)
MAX_POINTS = 20  # positioning_accuracy `points` (default 5)
MAX_REPEATS = 20  # positioning_accuracy `repeats` (default 5)

ESTIMATES = (
    "Only one machine (a ball screw bench) was measured; friction, damping, stiffness and similar values of the other "
    "configs are estimates (bounds in [bounds]), so comparisons are sounder than absolute numbers; resource "
    "kinematics-sim://limits has the rest."
)

mcp = MCPServer(
    "kinematics-sim",
    instructions=(
        "A physics simulator of motion machines (gantries, parallel pickers, arms, parallel kinematics on cylinders "
        "and cables, cranes, presses, web handling) with servo drive models: cycle loads, sizing verdicts, catalogue "
        "motor search, loop margins, accuracy. Start with "
        "list_machines and describe_machine, keep configs in sessions (create_session, set_config_values), then "
        "check_sizing / analyze_cycle / find_motor. Every result has 'notes': pass them on, do not overclaim. "
        + ESTIMATES
    ),
    version=ks.__version__,
)

_sessions: dict[str, ks.Config] = {}


# --- plain JSON ------------------------------------------------------------------------------------------------------


def _plain(v: Any) -> Any:
    """NumPy and floats to compact JSON: 4 significant digits, NaN and infinity as None."""
    if isinstance(v, np.ndarray):
        return [_plain(x) for x in v.tolist()]
    if isinstance(v, (np.bool_, bool)):
        return bool(v)
    if isinstance(v, (np.integer, int)):
        return int(v)
    if isinstance(v, (np.floating, float)):
        f = float(v)
        return None if not math.isfinite(f) else float(f"{f:.4g}")
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    return v


def _rows(table: dict[str, Any], columns: list[str] | None = None) -> list[dict[str, Any]]:
    """A dict of equal-length columns as rows, only the columns named (those the table has)."""
    cols = [c for c in (columns or list(table)) if c in table and not isinstance(table[c], dict)]
    n = len(table[cols[0]]) if cols else 0
    return [{c: _plain(table[c][i]) for c in cols} for i in range(n)]


def _capped(rows: list[dict[str, Any]], top: int = MAX_ROWS) -> dict[str, Any]:
    top = max(1, min(top, 200))
    out: dict[str, Any] = {"rows": rows[:top], "row_count": len(rows)}
    if len(rows) > top:
        out["rows_left_out"] = len(rows) - top
    return out


# --- sessions --------------------------------------------------------------------------------------------------------


def _machine_ids() -> list[str]:
    return [m["id"] for m in ks.machines()]


def _at_most(name: str, value: int, limit: int) -> None:
    if value > limit:
        raise ToolError(f"{name} is {value}, more than the limit of {limit} this server allows; give {limit} or fewer")


def _store(name: str, cfg: ks.Config) -> str | None:
    """Keeps `cfg` as session `name`, the most recently used; the session dropped to stay within MAX_SESSIONS, or None."""
    _sessions.pop(name, None)
    dropped = next(iter(_sessions)) if len(_sessions) >= MAX_SESSIONS else None
    if dropped is not None:
        del _sessions[dropped]
    _sessions[name] = cfg
    return dropped


def _config(session: str) -> ks.Config:
    """A session's config, or a machine id's default config (not stored)."""
    if session in _sessions:
        _sessions[session] = _sessions.pop(session)  # now the most recently used
        return _sessions[session]
    if session in _machine_ids():
        return ks.Config(session)
    known = list(_sessions) + _machine_ids()
    near = difflib.get_close_matches(session, known, n=3)
    raise ToolError(
        f"no session or machine named {session!r}"
        + (f"; did you mean {', '.join(near)}?" if near else "")
        + f" Sessions: {', '.join(_sessions) or 'none'}. Machines: call list_machines."
    )


def _default_leaves(cfg: ks.Config) -> dict[str, Any] | None:
    try:
        return ks.Config(cfg.machine).leaves()
    except (KeyError, ValueError):  # a machine file's machine has no registered default
        return None


def _key_error(cfg: ks.Config, key: str, err: Exception) -> str:
    leaves = cfg.leaves()
    msg = f"{key}: " + str(err).strip("'\"")
    if key in leaves:
        return msg + f" (it is now {leaves[key]!r}, a {type(leaves[key]).__name__})"
    near = difflib.get_close_matches(key, list(leaves), n=3, cutoff=0.6)
    return msg + (f" (did you mean {', '.join(near)}?)" if near else "")


def _param_label(cfg: ks.Config, key: str) -> str | None:
    text = ks.label(cfg, f"param-{key.rsplit('.', 1)[-1]}")
    return None if text.startswith("param-") else text


# --- long calls ------------------------------------------------------------------------------------------------------


async def _call(ctx: Context | None, what: str, fn: Any, *args: Any, **kwargs: Any) -> Any:
    """`fn` in a worker thread (the simulator releases the GIL) as a `ks.Job`, with progress every 5 s and the timeout;
    where the call is given up or cancelled by the client, the job is cancelled and the thread ends soon."""
    result: Any = None
    error: BaseException | None = None
    job = ks.Job()

    async def heartbeat() -> None:
        elapsed = 0
        while True:
            await anyio.sleep(5)
            elapsed += 5
            if ctx is not None:
                done, total = job.done, job.total
                if total:  # runs finished of runs planned so far (later batches add to it)
                    await ctx.report_progress(done, total, f"{what}: {done} of {total} runs, {elapsed} s")
                else:
                    await ctx.report_progress(elapsed, TIMEOUT_S, f"{what}: running for {elapsed} s")

    try:
        with anyio.fail_after(TIMEOUT_S):
            async with anyio.create_task_group() as tg:
                tg.start_soon(heartbeat)
                try:
                    result = await anyio.to_thread.run_sync(
                        partial(job.run, partial(fn, *args, **kwargs)), abandon_on_cancel=True
                    )
                except Exception as e:  # kept out of the task group, which would wrap it
                    error = e
                except BaseException as e:
                    # A panic of the core arrives as pyo3_runtime.PanicException, a BaseException that is not
                    # importable by name. The server and its sessions stay.
                    if type(e).__name__ != "PanicException":
                        raise
                    error = ToolError(
                        f"{what}: the model failed internally (a bug in the simulator, not in the input); the "
                        "sessions are kept. Report it with the config (get_config as_toml)."
                    )
                finally:
                    job.cancel()  # a no-op once the call has returned
                    tg.cancel_scope.cancel()
    except TimeoutError:
        raise ToolError(
            f"{what} did not finish within {TIMEOUT_S:.0f} s and was given up (set KINEMATICS_SIM_MCP_TIMEOUT_S to "
            "allow longer). Narrow it: fewer runs or samples, a motor series, no bands."
        ) from None
    if isinstance(error, ToolError):
        raise error
    if error is not None:
        raise ToolError(f"{what}: {str(error).strip(chr(39))}") from None
    return result


# Files are written only where the operator says, never where a (possibly prompt-injected) model asks.
def _output_dir() -> Path:
    out = Path(os.environ.get("KINEMATICS_SIM_MCP_OUT_DIR") or Path.cwd() / "kinematics-sim-output")
    out.mkdir(parents=True, exist_ok=True)
    return out.resolve()


def _output_file(name: str, extensions: tuple[str, ...]) -> Path:
    """`name` (a file name or a relative path) inside the output folder, refused if it leads outside it (.., an
    absolute path or drive, a link or junction), has another extension, or exists (unless the operator allows)."""
    out = _output_dir()
    given = Path(name)
    if not name or given.anchor or given.drive or ".." in given.parts:
        raise ToolError(f"give a file name inside the output folder {out}, not a path out of it: {name!r}")
    if given.suffix.lower() not in extensions:
        raise ToolError(f"the file name must end in {' or '.join(extensions)}: {name!r}")
    target = (out / given).resolve()  # follows links and junctions, so one pointing out of the folder is caught
    if not target.is_relative_to(out) or target == out:
        raise ToolError(f"{name!r} leads outside the output folder {out}")
    target.parent.mkdir(parents=True, exist_ok=True)
    if (out / given).resolve() != target:  # a link that appeared meanwhile
        raise ToolError(f"{name!r} leads outside the output folder {out}")
    if target.exists() and os.environ.get("KINEMATICS_SIM_MCP_OVERWRITE") != "1":
        raise ToolError(f"{target} exists; give another name (the operator can allow overwriting with "
                        "KINEMATICS_SIM_MCP_OVERWRITE=1)")
    return target


# Files are read only where the operator says, too. The Rust core opens a file named by a config value when a machine is
# built: `arm.urdf_file` (machine-joint-kit/src/arm.rs `chain`: a built-in name, else the path) and `<prefix>.table_file`
# (sim-components/src/cam.rs `Cam::from_config`: a CSV path). Both end in `_file`, and no other config key is opened
# (the CLI, app, FMI and OPC UA front ends read files of their own, not through a config key), so a key whose last
# segment ends in `_file` is a file setting. A model's value for one is refused unless it is empty, the machine's own
# default (the built-in names), or a file inside KINEMATICS_SIM_MCP_IN_DIR: else the error text would tell whether any
# path exists, and a huge file or a device would be read without bound.
MAX_INPUT_BYTES = 5_000_000
FILE_REFUSED = (
    "a file setting (a key ending in _file) may be empty, keep the machine's built-in value, or name a file of at most "
    f"{MAX_INPUT_BYTES // 1_000_000} MB inside the folder the operator set in KINEMATICS_SIM_MCP_IN_DIR; this value was "
    "refused"
)


def _file_allowed(value: str) -> bool:
    folder = os.environ.get("KINEMATICS_SIM_MCP_IN_DIR")
    if not folder:
        return False
    try:  # a relative path is the server's working directory's, as the core opens it
        target = Path(value).resolve()  # follows links, so one pointing out of the folder is caught
        inside = target.is_relative_to(Path(folder).resolve())
        return inside and target.is_file() and target.stat().st_size <= MAX_INPUT_BYTES
    except (OSError, ValueError):
        return False


def _check_files(files: dict[str, Any], machine: str) -> None:
    """Refuses (one fixed message) any file setting of `files` (key: value) that is not allowed."""
    try:
        default = ks.Config(machine).leaves()
    except (KeyError, ValueError):  # a machine file's machine has no registered default
        default = {}
    for key, value in files.items():
        if isinstance(value, str) and value and value != default.get(key) and not _file_allowed(value):
            raise ToolError(FILE_REFUSED)


def _file_settings(leaves: dict[str, Any]) -> dict[str, Any]:
    return {k: v for k, v in leaves.items() if k.rsplit(".", 1)[-1].endswith("_file")}


def _toml_file_settings(text: str) -> dict[str, Any]:
    """The file settings of TOML text, found before the core would open them while it reads the text."""
    try:
        import tomllib
    except ImportError:  # Python 3.10: no parser at hand, so any text that could spell a file key is refused
        lower = text.lower()
        if "_file" in lower or "\\u" in lower or "\\x" in lower:
            raise ToolError(FILE_REFUSED) from None
        return {}
    try:
        tree = tomllib.loads(text)
    except tomllib.TOMLDecodeError as e:
        # Refused here: the core's parser may read what this one does not, and would then open what it names.
        raise ToolError(f"the TOML text could not be read: {e}") from None

    def walk(node: Any, path: str) -> dict[str, Any]:
        found: dict[str, Any] = {}
        for k, v in (node.items() if isinstance(node, dict) else []):
            here = f"{path}.{k}" if path else k
            if isinstance(v, list):
                found.update({a: b for item in v for a, b in walk(item, here).items()})
            elif isinstance(v, dict):
                found.update(walk(v, here))
            elif k.endswith("_file"):
                found[here] = v
        return found

    return walk(tree, "")


def _fmi_binaries() -> list[str]:
    """The sim-fmi libraries an FMU carries: those the operator names in KINEMATICS_SIM_FMI_BINARIES (separated by
    the path separator), or else, as sim-cli does, this platform's library beside a sim-cli on PATH."""
    named = [b for b in os.environ.get("KINEMATICS_SIM_FMI_BINARIES", "").split(os.pathsep) if b]
    if named:
        return named
    lib = {"win32": "sim_fmi.dll", "darwin": "libsim_fmi.dylib"}.get(sys.platform, "libsim_fmi.so")
    cli = shutil.which("sim-cli")
    if cli and (Path(cli).with_name(lib)).is_file():
        return [str(Path(cli).with_name(lib))]
    raise ToolError(
        f"no sim-fmi library found: the operator sets KINEMATICS_SIM_FMI_BINARIES to {lib} (a release ships it beside "
        "sim-cli; `cargo build -p sim-fmi --release` builds it), or puts a sim-cli with it beside on PATH"
    )


# --- tools: machines and configs ------------------------------------------------------------------------------------


@mcp.tool()
def list_machines() -> dict[str, Any]:
    """Every machine the simulator has: id and name. Use the id in create_session or as `session` directly."""
    return {"machines": ks.machines(), "notes": ESTIMATES}


def _bounds(cfg: ks.Config) -> dict[str, list[float]]:
    """The [bounds] table (not among the leaves) as {dotted path: [low, high]}."""
    try:
        table = cfg["bounds"]
    except KeyError:
        return {}

    def flat(prefix: str, t: dict[str, Any]) -> dict[str, list[float]]:
        out: dict[str, list[float]] = {}
        for k, v in t.items():
            out.update(flat(f"{prefix}{k}.", v) if isinstance(v, dict) else {f"{prefix}{k}": v})
        return out

    return flat("", table)


@mcp.tool()
def describe_machine(session: str) -> dict[str, Any]:
    """What a machine (id) or a session's config is: name, motors, axes, parts, the settings that matter with units,
    and which values are estimates with their low and high bounds. Call it before changing a config."""
    cfg = _config(session)
    leaves = cfg.leaves()
    sig = ks.signals(cfg)["id"]
    motors = sorted({s.split(".")[1] for s in sig if s.startswith("motor.") and s.count(".") >= 2})
    axes = sorted({s.split(".")[1] for s in sig if s.startswith("axis.") and s.count(".") >= 2})
    name = next((m["name"] for m in ks.machines() if m["id"] == cfg.machine), cfg.machine)
    main = ("parts.", "mass.", "profile.", "geometry.", "start.")
    settings = {
        k: {"value": _plain(v), **({"label": lb} if (lb := _param_label(cfg, k)) else {})}
        for k, v in leaves.items()
        if k.startswith(main) and not isinstance(v, (list, dict))
    }
    estimates = {
        k: {"value": _plain(leaves.get(k)), "low_high": _plain(v)} for k, v in _bounds(cfg).items()
    }
    effects = {k.removeprefix("effects."): v for k, v in leaves.items() if k.startswith("effects.") and isinstance(v, bool)}
    return {
        "machine": cfg.machine,
        "name": name,
        "motors": [{"id": m, "label": ks.label(cfg, f"motor-{m}")} for m in motors],
        "axes": [{"id": a, "label": ks.label(cfg, f"axis-{a}")} for a in axes],
        "settings": settings,
        "estimates_with_bounds": estimates,
        "effects_on": sorted(k for k, v in effects.items() if v),
        "effects_off": sorted(k for k, v in effects.items() if not v),
        "config_key_count": len(leaves),
        "notes": (
            "Units are in the key names (_kg, _m, _s, _nm, _rad_s...). The settings listed are a selection; get_config "
            "with a prefix reads any other section (motor., drive., ratings., friction_x., belt....). Values under "
            "estimates_with_bounds are estimates: check_sizing with bands=true shows what they do. " + ESTIMATES
        ),
    }


@mcp.tool()
def create_session(
    name: str, machine: str | None = None, toml: str | None = None, copy_of: str | None = None
) -> dict[str, Any]:
    """A named config kept in memory for this conversation: a machine's default (`machine`), TOML text of a config
    for `machine`, or a copy of another session (`copy_of`). An existing session of that name is replaced. At most
    32 sessions are kept: one more drops the least recently used."""
    if copy_of is not None:
        cfg = _config(copy_of).copy()
    elif machine is not None:
        if machine not in _machine_ids():
            raise ToolError(f"unknown machine {machine!r}; call list_machines")
        if toml:  # before the core reads the text, which opens the files it names
            _check_files(_toml_file_settings(toml), machine)
        try:
            cfg = ks.Config(machine, text=toml) if toml else ks.Config(machine)
        except (ValueError, KeyError) as e:
            raise ToolError(f"the TOML was refused: {e}") from None
    else:
        raise ToolError("give `machine` (with optional `toml`) or `copy_of`")
    dropped = _store(name, cfg)
    out: dict[str, Any] = {"session": name, "machine": cfg.machine, "sessions": list(_sessions)}
    if dropped:
        out["dropped_session"] = dropped
        out["notes"] = f"At most {MAX_SESSIONS} sessions are kept: {dropped!r}, the least recently used, is gone."
    return out


@mcp.tool()
def list_sessions() -> dict[str, Any]:
    """The sessions of this conversation, each with its machine and how many values differ from its default."""
    out = []
    for name, cfg in _sessions.items():
        d = _diff(cfg, _default_leaves(cfg))
        out.append({"session": name, "machine": cfg.machine, "values_changed": None if d is None else len(d)})
    return {"sessions": out}


@mcp.tool()
def get_config(
    session: str, keys: list[str] | None = None, prefix: str | None = None, as_toml: bool = False
) -> dict[str, Any]:
    """Values of a config by dotted path (`keys`, e.g. ["mass.payload_kg"]) or every value under a `prefix`
    (e.g. "motor."); `as_toml` gives the whole config as TOML text to save or edit."""
    cfg = _config(session)
    if as_toml:
        return {"toml": cfg.to_toml()}
    leaves = cfg.leaves()
    values: dict[str, Any] = {}
    errors = []
    for k in keys or []:
        try:
            values[k] = _plain(cfg[k])
        except KeyError as e:
            errors.append(_key_error(cfg, k, e))
    if prefix is not None:
        values.update({k: _plain(v) for k, v in leaves.items() if k.startswith(prefix)})
    if not keys and prefix is None:
        sections = sorted({k.split(".")[0] for k in leaves})
        return {"sections": sections, "notes": "give keys or a prefix (a section name and a dot) to read values"}
    if len(values) > 400:
        values = dict(list(values.items())[:400])
        errors.append("more than 400 values: give a longer prefix")
    return {"values": values, **({"errors": errors} if errors else {})}


@mcp.tool()
def set_config_values(session: str, values: dict[str, Any]) -> dict[str, Any]:
    """Set values in a session by dotted path, e.g. {"mass.payload_kg": 3.5, "parts.motor": "1FK2104-5AK"}. All or
    nothing: if any key or value is refused, nothing changes and each refusal is listed with a suggestion."""
    if session not in _sessions:
        raise ToolError(f"no session {session!r}: create_session first (sessions: {', '.join(_sessions) or 'none'})")
    trial = _sessions[session].copy()
    errors = []
    for k, v in values.items():
        try:
            trial[k] = v
        except (KeyError, ValueError, OverflowError, TypeError) as e:
            errors.append(_key_error(trial, k, e))
    if errors:
        raise ToolError("nothing was changed. " + "; ".join(errors))
    # After the values are in: a value may be a table with a file setting inside, which only the leaves show.
    _check_files(_file_settings(trial.leaves()), trial.machine)
    _store(session, trial)
    return {"session": session, "set": {k: _plain(trial[k]) for k in values}}


def _diff(cfg: ks.Config, base: dict[str, Any] | None) -> dict[str, Any] | None:
    if base is None:
        return None
    now = cfg.leaves()
    return {
        k: {"base": _plain(base.get(k)), "now": _plain(now.get(k))}
        for k in sorted(set(now) | set(base))
        if now.get(k) != base.get(k)
    }


@mcp.tool()
def config_diff(session: str, against: str | None = None) -> dict[str, Any]:
    """What differs in a session from its machine's default config, or from another session (`against`)."""
    cfg = _config(session)
    base = _config(against).leaves() if against else _default_leaves(cfg)
    d = _diff(cfg, base)
    if d is None:
        raise ToolError("this config's machine has no default to compare with; give `against`")
    return {"session": session, "against": against or "default", "differences": d}


# --- tools: results --------------------------------------------------------------------------------------------------


def _margin(value: Any, limit: Any, ok: Any) -> float | None:
    """The share of the limit left (positive) or exceeded (negative), whichever way the check points."""
    v, lim = _plain(value), _plain(limit)
    if v is None or lim is None or lim == 0:
        return None
    return _plain(abs(lim - v) / abs(lim) * (1 if ok else -1))


@mcp.tool()
async def check_sizing(
    session: str, bands: bool = False, path: str | None = None, top: int = MAX_ROWS, ctx: Context | None = None
) -> dict[str, Any]:
    """Pass or fail of the configured cycle (or a `path`, text of a path file) against every rating: per check and
    motor the value, the limit, the margin and a one-line meaning; tightest first. `bands` adds each check's range over
    the estimates' bounds and whether the worst case passes (slower)."""
    cfg = _config(session)
    r = await _call(ctx, "verdicts", ks.verdicts, cfg, bands=bands, path=path)
    rows = []
    for i, check in enumerate(r["check"]):
        row = {"check": check, "motor": r["motor"][i], "meaning": ks.label(cfg, f"check-{check}")}
        row.update({c: _plain(r[c][i]) for c in r if c not in ("check", "motor")})
        row["margin_share"] = _margin(r["value"][i], r["limit"][i], r["ok"][i])
        rows.append(row)
    rows.sort(key=lambda x: (x["ok"], x.get("worst_ok", True), x["margin_share"] if x["margin_share"] is not None else 9))
    failed = [f"{x['check']}" + (f" ({x['motor']})" if x["motor"] else "") for x in rows if not x["ok"]]
    out = {
        "passed": not failed,
        "failed": failed,
        **_capped(rows, top),
        "notes": "margin_share: share of the limit left (negative: exceeded by). value and limit are in the unit in "
        "brackets at the end of meaning. "
        + ("worst_ok judges the worst case within the bounds. " if bands else "Without bands: at the nominal values only. ")
        + "Settling, stops' verdicts and the time-optimal plan's whole-cycle limits are not all covered here. "
        + ESTIMATES,
    }
    if bands:
        out["worst_case_failed"] = [x["check"] for x in rows if x.get("worst_ok") is False]
    return out


ANALYZE_COLUMNS = [
    "motor", "cycle_s", "peak_torque_nm", "rms_torque_nm", "peak_speed_rad_s", "utilisation", "thermal_utilisation",
    "winding_rise_k", "rms_current_a", "energy_in_j", "energy_regenerated_j", "mains_energy_j",
]
PATH_COLUMNS = ["path_error_peak_m", "path_error_rms_m", "tracking_error_peak_m", "dwell_error_peak_m"]


@mcp.tool()
async def analyze_cycle(
    session: str, path: str | None = None, extra_columns: list[str] | None = None, ctx: Context | None = None
) -> dict[str, Any]:
    """The load per motor over one pass of the cycle (or a `path`): peak and RMS torque, peak speed, utilisation of
    the torque-speed curve, thermal utilisation, winding temperature rise, RMS current, energy in and regenerated.
    `extra_columns` adds any other column of the analysis (their names are listed in the result)."""
    cfg = _config(session)
    r = await _call(ctx, "analyze", ks.analyze, cfg, path=path)
    cols = ANALYZE_COLUMNS + (PATH_COLUMNS if path else []) + list(extra_columns or [])
    return {
        "motors": _rows(r, cols),
        "other_columns": [c for c in r if c not in cols],
        "notes": "utilisation and thermal_utilisation above 1 fail. winding_rise_k is the settled rise. " + ESTIMATES,
    }


@mcp.tool()
async def run_path(session: str, path: str, time_optimal: bool = False, ctx: Context | None = None) -> dict[str, Any]:
    """Run a path given inline and give the path error. One piece per line, positions in m per axis, from where the
    machine stands: `line,a,b`; `arc,a,b,ca,cb,cw|ccw` (end, centre); `spline,a,b` (via point); three numbers per
    point on a 3-axis machine. `time_optimal` runs it as fast as the limits judged at a point allow."""
    cfg = _config(session)
    r = await _call(ctx, "path", ks.analyze, cfg, path=path, time_optimal=time_optimal)
    rows = _rows(r, ["motor", "peak_torque_nm", "rms_torque_nm", "peak_speed_rad_s", "utilisation"])
    first = _rows(r, ["cycle_s", *PATH_COLUMNS])[0] if rows else {}
    return {
        "duration_s": first.get("cycle_s"),
        **{k: v for k, v in first.items() if k != "cycle_s"},
        "motors": rows,
        "notes": "path_error is the tool's distance from the line its setpoint drew (contour error); tracking_error "
        "is the lag along it. Frame and belt compliance are in; their stiffness values are estimates. " + ESTIMATES,
    }


@mcp.tool()
async def find_motor(
    session: str,
    top: int = 5,
    series: str = "",
    vendor: str | None = None,
    supply: str | None = None,
    rank_by: str = "cost",
    ctx: Context | None = None,
) -> dict[str, Any]:
    """Catalogue sizing: every catalogue motor (with drive and gearbox) on the session's cycle, the passing ones first,
    cheapest (or `rank_by`, e.g. "energy") first; the top N. `series` (e.g. "1FK2") narrows and speeds it up."""
    cfg = _config(session)
    r = await _call(ctx, "sizing", ks.sizing, cfg, series=series, vendor=vendor, supply=supply, rank_by=rank_by)
    rows = _rows(r, [c for c in r if not c.startswith("u_")])
    passing = sum(1 for x in rows if not x.get("failing"))
    return {
        "candidates_searched": len(rows),
        "candidates_passing": passing,
        **_capped(rows, top),
        "notes": "Ranked by installed weight standing in for price (no prices in the catalogue). failing lists the "
        "checks a candidate fails; estimated the values that are estimates for it. Apply one with set_config_values "
        "on parts.motor (and parts.gearbox, parts.drive). " + ESTIMATES,
    }


def _toml_value(text: Any) -> Any:
    """A TOML number, bool or string as its value (their TOML and JSON spellings agree; tomllib needs 3.11)."""
    try:
        return json.loads(text)
    except (ValueError, TypeError):
        return text


@mcp.tool()
async def optimise_cycle(
    session: str, runs: int = 40, robust: bool = False, ctx: Context | None = None
) -> dict[str, Any]:
    """A shorter cycle that still passes every check (higher limits, blending, shaping, lower lifts): the cycle time
    before and after and the settings changed. `robust` keeps it passing within the error bands (slower). At most 200 `runs`."""
    _at_most("runs", runs, MAX_RUNS)
    cfg = _config(session)
    r = await _call(ctx, "optimise", ks.optimise, cfg, runs=runs, worst=robust)
    out = {k: _plain(v) for k, v in r.items() if k not in ("changes", "pareto", "robust")}
    # The settings come as TOML text ("0.8", "true"): as JSON values, ready for set_config_values.
    out["changes"] = [
        {k: (_toml_value(v) if k in ("before", "after") else v) for k, v in row.items()}
        for row in (_rows(r["changes"]) if r.get("changes") else [])
    ]
    out["robust"] = _plain(r.get("robust"))
    out["notes"] = "Not applied: set_config_values with the 'after' values applies them to a session. " + ESTIMATES
    return out


@mcp.tool()
async def loop_margins(
    session: str, bands: bool = False, top: int = MAX_ROWS, ctx: Context | None = None
) -> dict[str, Any]:
    """Per motor the velocity loop's crossover, phase margin, gain margin and peak sensitivity, and whether they pass,
    at the config as it is and at each other pose the cycle stops at; `bands` adds runs at the estimates' bounds. Rows
    that fail (`ok` false) come first, then by the smallest phase margin; a row whose phase margin is not measurable
    (null: no gain crossover) follows the measured ones of its group. `top` rows are given, the rest counted."""
    cfg = _config(session)
    r = await _call(ctx, "margins", ks.margins, cfg, bands=bands)
    rows = _rows(r)
    rows.sort(key=lambda x: (x["ok"], x["phase_margin_deg"] is None, x["phase_margin_deg"] or 0.0))
    return {
        **_capped(rows, top),
        "notes": "Linearised about the operating point; the drive's filters and delays as configured. " + ESTIMATES,
    }


@mcp.tool()
async def positioning_accuracy(
    session: str, points: int = 5, repeats: int = 5, ctx: Context | None = None
) -> dict[str, Any]:
    """Static positioning accuracy and repeatability per axis (ISO 230-2: reversal, repeatability, systematic error,
    accuracy) and at the tool (ISO 9283), with their verdicts where the config rates them. At most 20 `points` and
    20 `repeats` (the runs multiply)."""
    _at_most("points", points, MAX_POINTS)
    _at_most("repeats", repeats, MAX_REPEATS)
    cfg = _config(session)
    r = await _call(ctx, "accuracy", ks.accuracy, cfg, points=points, repeats=repeats)
    out = {k: _plain(v) for k, v in r.items() if not isinstance(v, (dict, np.ndarray, list))}
    for k in ("summary", "verdicts"):
        if isinstance(r.get(k), dict):
            out[k] = _rows(r[k])
    out["notes"] = (
        "Lengths in m (angles in rad on a rotary axis). Simulated: backlash, friction, compliance, and thermal growth "
        "and lead error where switched on; encoder "
        "quantisation as configured. Not compared with a measured axis. " + ESTIMATES
    )
    return out


@mcp.tool()
async def sensitivity(
    session: str, top: int = 10, method: str = "one-at-a-time", ctx: Context | None = None
) -> dict[str, Any]:
    """Which values move the results most, so you know what to measure first. "one-at-a-time" (every parameter at
    its bounds, or +-20 % where it has none, design values too) or "morris" / "sobol" (all [bounds] at once, ranked per result)."""
    cfg = _config(session)
    if method == "one-at-a-time":
        r = await _call(ctx, "sensitivity", ks.sensitivity, cfg)
    elif method in ("morris", "sobol"):
        r = await _call(ctx, "global sensitivity", ks.global_sensitivity, cfg, method=method)
    else:
        raise ToolError('method: "one-at-a-time", "morris" or "sobol"')
    rows = _rows(r)
    bounded = _bounds(cfg)
    for row in rows:
        row["is_estimate"] = row.get("parameter") in bounded
    return {
        "method": method,
        **_capped(rows, top),
        "notes": "Strongest first. *_low and *_high: the result's relative change (0.2 = +20 %) with the parameter at "
        "its low or high value (one-at-a-time; *_n columns are absolute, in N). Design values (a gear ratio, a "
        "pulley radius) rank too: what to measure first are the top rows with is_estimate true "
        "(docs/measurement-procedure.md). " + ESTIMATES,
    }


@mcp.tool()
async def write_report(
    session: str, file_name: str, bands: bool = False, lang: str = "en", ctx: Context | None = None
) -> dict[str, Any]:
    """Write the sizing report into the server's output folder: `file_name` ending in .html (or .pdf), e.g.
    "gantry-3kg.html". The result gives the full path to show the user."""
    cfg = _config(session)
    p = _output_file(file_name, (".html", ".htm", ".pdf"))
    passed = await _call(ctx, "report", ks.report, cfg, p, bands=bands, lang=lang)
    return {"written": str(p), "every_check_passed": passed, "notes": ESTIMATES}


@mcp.tool()
async def export_fmu(session: str, file_name: str, fmi: int = 2, ctx: Context | None = None) -> dict[str, Any]:
    """Export the session's machine as an FMI 2.0 or 3.0 Co-Simulation FMU into the server's output folder:
    `file_name` ending in .fmu. The result gives the full path to show the user."""
    cfg = _config(session)
    p = _output_file(file_name, (".fmu",))
    await _call(ctx, "fmu", ks.fmu, cfg, p, binaries=_fmi_binaries(), fmi=fmi)
    return {
        "written": str(p),
        "notes": "Checked with fmpy and OpenModelica/OMSimulator; not with Simulink or Dymola. " + ESTIMATES,
    }


# --- resources and prompts -------------------------------------------------------------------------------------------


# shortcut: a summary of docs/overview.md's section kept by hand (the wheel cannot carry docs/, and files under
# crates/sim-py may not name machines, which that section does); update it with that section.
LIMITS = """\
# What the simulator can and cannot tell you

Full text: https://sparkz400.github.io/KinematicsSimulation/docs/overview.html#what-it-can-and-cannot-tell-you

It can tell you, for a machine described in a config: peak, RMS and cubic-mean torque, speed, current, winding
temperature rise and energy per motor over a cycle or a path; pass or fail against the ratings in the config (torque-
speed curve, S1, winding temperature, gearbox, belt, DC link and braking resistor, guide and shaft loads, settling, and
the machine's own checks); bearing, guide and gearbox lives from the makers' formulas; how results move with the
uncertain values at their bounds; path error; how fast a path can be run within the limits judged at a point; the
resonances over the working range; which catalogue motor with drive and gearbox is the cheapest that passes (installed
weight stands in for price); a shorter cycle that still passes; what to measure first.

What stands behind the numbers: every component and analysis figure has a test against a hand calculation; some
machines' dynamics were checked against independent implementations (SciPy, MuJoCo, SymPy Lagrangians); catalogue data
against vendor formulas. These show the equations are solved as documented, not that the parameters describe a real
machine.

Limits that matter:
- One machine has been measured, by others: a ball screw bench (ISW Stuttgart). Replayed with fitted friction the motor
  torque is off by 4.6 % of the peak in RMS on the fit recording, 6 to 13 % on the checks; the first mode 137 Hz in the
  model against 123 to 134 Hz measured.
- For every other config friction, damping, guide and frame stiffness, belt pretension, winding resistance and
  inductance, DC link and braking resistor data are estimates. Each config has a [bounds] table with a low and a high
  value for its estimates: judge with bands (check_sizing bands=true). Trends and comparisons are sound; absolute
  numbers are as good as the estimates.
- The belt machines' default configs are made-up placeholder machines, not products; the other machines are sized
  examples whose mechanics are estimates of a typical part of that size. Steppers, induction motors, vacuum cups and
  valves have no catalogue data.
- A servo motor is one equivalent winding (with a d-q model when field weakening is on); no three-phase model, no PWM.
- The time-optimal plan keeps the limits judged at a point of the path, not those over the whole cycle (warming, S1,
  RMS current, lives): a time-optimal cycle can overheat a motor.
- Life figures are the makers' formulas with partly estimated ratings; not validated.
- The FMU was checked with fmpy and OpenModelica/OMSimulator, not in Simulink or Dymola.
- Trace import and fitting were tested on simulated traces and the one bench's recordings; no drive's own export.
"""


@mcp.resource("kinematics-sim://limits", name="model-limits", mime_type="text/markdown")
def model_limits() -> str:
    """What the simulator can and cannot tell you: read it before stating results as facts about a real machine."""
    return LIMITS


@mcp.prompt(name="size_a_motor")
def size_a_motor(machine_description: str) -> str:
    """Size a motor for my machine: from a description to a checked motor choice."""
    return (
        f"Size the motors for this machine: {machine_description}\n\n"
        "1. list_machines, pick the closest machine and describe_machine it. Say which parts of my machine it does "
        "not model.\n"
        "2. create_session from it and set_config_values for what I told you (payload, strokes, cycle, motor...). "
        "Ask me for the values that matter and that I did not give; say which ones you assumed.\n"
        "3. analyze_cycle and check_sizing; if a check fails, find_motor (top 5) and apply the best candidate in a "
        "copy of the session; compare with config_diff.\n"
        "4. check_sizing with bands=true on the choice: report the worst case, not only the nominal one.\n"
        "5. sensitivity: name the two or three values I should measure first.\n"
        "Pass on every result's notes; the numbers are estimates unless the config was measured."
    )


def main() -> None:
    """The console script `kinematics-sim-mcp`: the server on stdio."""
    mcp.run("stdio")


if __name__ == "__main__":
    main()
