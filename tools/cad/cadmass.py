"""Mass properties of CAD parts -> the [mass] values of a machine config. Pure Python, no CAD kernel.

SI throughout: kg, m, kg m2. Inertia tensors are full 3x3 tensors (off-diagonals are -products of inertia),
as OpenCascade's GProp MatrixOfInertia and FreeCAD's Shape.MatrixOfInertia give them.
"""

import csv
import io
import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# role -> [mass] key; a tuple is (mass key, COM distance key, inertia key) about the shoulder axis.
ROLE_KEYS = {
    "t-gantry": {"carriage": "carriage_kg", "arm": "z_axis_kg", "payload": "payload_kg"},
    "single-axis": {"carriage": "carriage_kg", "payload": "payload_kg"},
    "delta-d3": {
        "carriage": "platform_kg",
        "arm": ("upper_arm_kg", "upper_arm_cg_m", "upper_arm_inertia_kgm2"),
        "payload": "payload_kg",
    },
}

# kg/m3; a map file or a part's own density can add to or override these.
MATERIALS = {"steel": 7850.0, "aluminium": 2700.0, "stainless": 7900.0, "pom": 1410.0, "carbon": 1550.0}

# Delta arm convention: the shoulder axis is the CAD x-axis through the origin.
SHOULDER_POINT = (0.0, 0.0, 0.0)
SHOULDER_DIR = (1.0, 0.0, 0.0)


@dataclass
class Body:
    name: str
    role: str
    mass: float
    com: tuple
    inertia: list  # 3x3 about the COM


def _shift(inertia, mass, d):
    """Tensor moved from the COM to a point at -d from it (parallel-axis theorem)."""
    d2 = sum(x * x for x in d)
    return [[inertia[i][j] + mass * ((d2 if i == j else 0.0) - d[i] * d[j]) for j in range(3)] for i in range(3)]


def combine(bodies, name="", role=""):
    if not bodies:
        raise ValueError("combine: no bodies")
    m = sum(b.mass for b in bodies)
    com = tuple(sum(b.mass * b.com[i] for b in bodies) / m for i in range(3))
    total = [[0.0] * 3 for _ in range(3)]
    for b in bodies:
        moved = _shift(b.inertia, b.mass, [b.com[i] - com[i] for i in range(3)])
        total = [[total[i][j] + moved[i][j] for j in range(3)] for i in range(3)]
    return Body(name or "+".join(b.name for b in bodies), role or bodies[0].role, m, com, total)


def about_axis(body, point=SHOULDER_POINT, direction=SHOULDER_DIR):
    """(distance of the COM from the axis, inertia about the axis)."""
    n = sum(x * x for x in direction) ** 0.5
    u = [x / n for x in direction]
    r = [body.com[i] - point[i] for i in range(3)]
    along = sum(r[i] * u[i] for i in range(3))
    perp2 = max(sum(x * x for x in r) - along * along, 0.0)
    i_com = sum(u[i] * body.inertia[i][j] * u[j] for i in range(3) for j in range(3))
    return perp2**0.5, i_com + body.mass * perp2


def by_role(bodies):
    roles = {}
    for b in bodies:
        roles.setdefault(b.role, []).append(b)
    return {role: combine(bs, name=role, role=role) for role, bs in roles.items()}


def values(machine, roles):
    """{key: value} for the [mass] table from {role: Body}."""
    table = ROLE_KEYS.get(machine)
    if table is None:
        raise ValueError(f"unknown machine {machine!r}; known: {', '.join(ROLE_KEYS)}")
    out = {}
    for role, body in roles.items():
        keys = table.get(role)
        if keys is None:
            raise ValueError(f"machine {machine} has no role {role!r}; it has: {', '.join(table)}")
        if isinstance(keys, tuple):
            cg, inertia = about_axis(body)
            out.update(zip(keys, (body.mass, cg, inertia)))
        else:
            out[keys] = body.mass
    return out


def _g(x):
    return f"{x:.6g}"


def cad_comment(source, body):
    com = ", ".join(_g(c) for c in body.com)
    tensor = ", ".join("[" + ", ".join(_g(v) for v in row) + "]" for row in body.inertia)
    return f"# [cad] {source}: m={_g(body.mass)} kg, COM=({com}) m, I_com=[{tensor}] kg m2"


TAG = re.compile(r"\[(catalogue|estimate|cad)\]", re.IGNORECASE)
_TABLE = re.compile(r"^\s*\[([^\[\]]+)\]\s*(#.*)?$")


def apply(config_text, machine, roles, source):
    """config_text with the mapped [mass] values replaced and a [cad] line directly above each.

    Every other line is kept byte for byte, line endings included.
    """
    new = values(machine, roles)
    comment_of = {}
    for role, body in roles.items():
        keys = ROLE_KEYS[machine][role]
        for k in keys if isinstance(keys, tuple) else (keys,):
            comment_of[k] = cad_comment(source, body)
    lines = config_text.splitlines(keepends=True)
    # A last line without its newline gets the file's own kind.
    newline = "\r\n" if "\r\n" in config_text else "\n"
    out, table, done = [], None, set()
    for line in lines:
        body, eol = line.rstrip("\r\n"), line[len(line.rstrip("\r\n")) :]
        head = _TABLE.match(body)
        if head:
            table = head.group(1).strip()
        m = re.match(r"^(\s*)([A-Za-z0-9_]+)(\s*=\s*)([^#\s]+)(.*)$", body) if table == "mass" else None
        if m and m.group(2) in new:
            key = m.group(2)
            if out and out[-1].lstrip().startswith("# [cad]"):
                out.pop()
            # A float literal even for whole numbers, so the TOML value stays a float.
            out.append(m.group(1) + comment_of[key] + (eol or newline))
            # A source tag at the end of the line described the old value, and would outrank the [cad] line.
            rest = "" if TAG.search(m.group(5)) else m.group(5)
            line = m.group(1) + key + m.group(3) + repr(float(_g(new[key]))) + rest + eol
            done.add(key)
        out.append(line)
    missing = set(new) - done
    if missing:
        raise ValueError(f"config has no [mass] key {', '.join(sorted(missing))}: pass a complete config")
    return "".join(out)


def default_config(machine):
    return ROOT / "crates" / f"machine-{machine}" / "default.toml"


def find_sim_cli(given=None):
    for c in (given, os.environ.get("SIM_CLI"), ROOT / "target/release/sim-cli.exe", ROOT / "target/debug/sim-cli.exe",
              ROOT / "target/release/sim-cli", ROOT / "target/debug/sim-cli"):
        if c and Path(c).is_file():
            return str(Path(c).resolve())
    return None


def run_verdicts(sim_cli, machine, config):
    """CSV text of `sim-cli verdicts`; raises RuntimeError with the CLI's message on failure."""
    r = subprocess.run([sim_cli, "verdicts", machine, "--config", str(config)], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"sim-cli verdicts failed ({r.returncode}): {r.stderr.strip() or r.stdout.strip()}")
    return r.stdout


def parse_verdicts(csv_text):
    """Rows of check, motor, value, limit, result (ok/FAIL), ok, margin.

    The CSV has no direction, so it is inferred: if (value <= limit) agrees with ok, the check is an at-most
    one and margin = (limit - value) / limit, else at-least and margin = (value - limit) / value (one minus
    limit over value, as the sizing report has it, so a long life reads as near 100 %, not millions). At
    value == limit it reads as at-most with margin 0. No margin where the divisor is 0.
    """
    rows = []
    for r in csv.DictReader(io.StringIO(csv_text)):
        value, limit, ok = float(r["value"]), float(r["limit"]), r["result"].strip() == "ok"
        at_most = (value <= limit) == ok
        base = abs(limit) if at_most else abs(value)
        margin = None if base == 0 else ((limit - value) if at_most else (value - limit)) / base
        rows.append({"check": r["check"], "motor": r["motor"], "value": value, "limit": limit,
                     "result": r["result"].strip(), "ok": ok, "margin": margin})
    return rows
