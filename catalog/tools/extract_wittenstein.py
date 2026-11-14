"""Extracts the WITTENSTEIN alpha SP+ MF and TP+ MF precision planetary gearboxes (1- and 2-stage, every size the
data sheets list) into catalog/wittenstein/gearbox_sp.csv and gearbox_tp.csv, in schema columns and SI units.

The documents are not part of this repository. Get them from WITTENSTEIN's download centre
(https://www.wittenstein-group.com/en-us/services/downloads, search "SP+" and "TP+"):
"Technical data / Dimension sheets SP+" (ALPHA-ADVANCED-LINE-SP.PDF) and "... TP+" (ALPHA-ADVANCED-LINE-TP.PDF), then:

    pdftotext -table -enc UTF-8 ALPHA-ADVANCED-LINE-SP.PDF sp.txt      (xpdf 4.x)
    pdftotext -table -enc UTF-8 ALPHA-ADVANCED-LINE-TP.PDF tp.txt
    python tools/extract_wittenstein.py sp.txt tp.txt wittenstein

Each table "<series> <size> MF <n>-stage" has one column per ratio; a value printed once holds for every ratio. The
mass moment of inertia J1 at the end of the table has one row per clamping hub (code letter and bore in mm), each
printed in kg cm2 and then in 10^-3 in lb s2; the script takes the kg cm2 rows and checks the in lb s2 row against
them, which also proves the rows were paired with the right hub. Rows: one per size, ratio and clamping hub. The MC,
MA and HIGH SPEED tables are skipped. A table without a row it needs, or a row with a number of values that is neither
one nor the number of ratios, stops the script. Where the in.lb row of a torque disagrees with its Nm row by more
than rounding, the script prints a warning and keeps the Nm value.
"""
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["series", "size", "motor_frame", "clamping_hub_diameter_m", "stages", "ratio", "rated_output_torque_nm",
           "max_output_torque_nm", "emergency_stop_output_torque_nm", "rated_input_speed_rad_s",
           "max_input_speed_rad_s", "no_load_torque_nm", "no_load_torque_speed_rad_s", "backlash_rad",
           "backlash_reduced_rad", "torsional_stiffness_nm_rad", "max_axial_force_n", "max_radial_force_n",
           "max_tilting_moment_nm", "efficiency_full_load", "mass_kg", "inertia_at_input_kgm2"]
FILES = {"SP+": "gearbox_sp.csv", "TP+": "gearbox_tp.csv"}

HEAD = re.compile(r"^(SP\+|TP\+) (\d{3,4}) (MF) ([12])-stage\s*$")
OTHER_HEAD = re.compile(r"^(SP\+|TP\+) \d{3,4} (MC|MA|MF)\b.*stage")
NUMBER = re.compile(r"^\d+(\.\d+)?$")
# label at the start of a line, its unit token, key, conversion to SI; the in.lb row follows the Nm rows.
ROWS = [
    ("Max. acceleration torque", "Nm", "t2b", "same"),
    ("Nominal torque", "Nm", "t2n", "same"),
    ("Emergency stop torque", "Nm", "t2not", "same"),
    ("Permitted average input speed", "rpm", "n1n", "rpm"),
    ("Max. input speed", "rpm", "n1max", "rpm"),
    ("Mean no load running torque", "Nm", "t012", "same"),
    ("Torsional rigidity", "Nm/arcmin", "ct21", "per_arcmin"),
    ("Max. axial force", "N", "f2a", "same"),
    ("Max. lateral force", "N", "f2q", "same"),
    ("Max. tilting moment", "Nm", "m2k", "same"),
    ("Efficiency at full load", "%", "eta", "percent"),
    ("Weight", "kg", "m", "same"),
]
REQUIRED = {"t2b", "t2n", "t2not", "n1n", "n1max", "t012", "ct21", "f2a", "m2k", "eta", "m"}
IN_LB_PER_NM = 8.850745793  # 1 N m in lb-force inch
KGCM2_PER_MILLI_IN_LB_S2 = 1.129848290  # 1e-3 lbf in s2 in kg cm2


def trailing_values(line):
    out = []
    for token in reversed(line.split()):
        if not NUMBER.match(token):
            break
        out.append(token)
    return out[::-1]


def spread(values, n, where):
    if len(values) == n:
        return values
    if len(values) == 1:
        return values * n
    raise ValueError(f"{where}: {len(values)} values {values} for {n} ratios")


def blocks(lines):
    """(series, size digits, stages, lines of the table) per MF table."""
    out, cur = [], None
    for line in lines:
        h = HEAD.match(line)
        if h or OTHER_HEAD.match(line) or line.startswith("Please use our sizing software"):
            if cur:
                out.append(cur)
            cur = None
        if h:
            cur = (h[1], h[2], int(h[4]), [])
        if cur is not None:
            cur[3].append(line)
    if cur:
        out.append(cur)
    return out


def half_unit(text):
    """Half a unit of the last printed digit of `text`."""
    return 0.5 * 10 ** -(len(text.split(".")[1]) if "." in text else 0)


def check_in_lb(name, label, nm, lines, i, n, tolerance):
    """The in.lb row printed under a Nm row agrees with it within rounding; a warning if not."""
    for j in range(i + 1, min(i + 3, len(lines))):
        if "in.lb" in lines[j].split():
            ib = trailing_values(lines[j])
            if len(ib) not in (1, n):
                # The text layer runs the digits of some in.lb rows together; the Nm row is the data.
                print(f"warning: {name} {label}: in.lb row not readable, not compared", file=sys.stderr)
                return
            for a, b in zip(nm, spread(ib, n, label)):
                # each printed to its own last digit: allow half a unit of that digit on both, and 1 %
                slack = half_unit(a) * IN_LB_PER_NM + half_unit(b) + tolerance * float(b)
                if abs(float(a) * IN_LB_PER_NM - float(b)) > slack:
                    print(f"warning: {name} {label}: {a} Nm but {b} in.lb", file=sys.stderr)
            return
    raise ValueError(f"{name} {label}: no in.lb row")


def parse_block(series, digits, stages, lines):
    name = f"{series} {digits} MF {stages}-stage"
    ratios = None
    for line in lines:
        if line.startswith("Ratio"):
            ratios = trailing_values(line)
            break
    if not ratios:
        raise ValueError(f"{name}: no ratio row")
    n = len(ratios)
    v, extra = {}, {}
    for i, line in enumerate(lines):
        for label, unit, key, _ in ROWS:
            if line.startswith(label) and key not in v:
                tokens = line.split()
                if unit not in tokens:
                    raise ValueError(f"{name} {label}: unit {unit} not on the line")
                vals = spread(trailing_values(line), n, f"{name} {label}")
                v[key] = vals
                if unit == "Nm" and key in ("t2b", "t2n", "t2not", "t012", "m2k"):
                    check_in_lb(name, label, vals, lines, i, n, 0.01)
                if key == "t012":
                    below = next((l for l in lines[i + 1:] if l.strip()), "")
                    m = re.search(r"at n1 = (\d+) rpm and (\d+) °C", below)
                    if not m:
                        raise ValueError(f"{name}: no reference speed under the no-load torque")
                    extra["t012_speed"], extra["t012_temp"] = m[1], m[2]
                break
        if line.startswith("Max. backlash"):
            m = re.search(r"Standard\s+≤\s*(\d+(?:\.\d+)?)\s*/\s*Reduced\s+≤\s*(\d+(?:\.\d+)?)", line)
            if not m:
                raise ValueError(f"{name}: backlash row not read: {line.strip()}")
            extra["jt"], extra["jt_red"] = m[1], m[2]
    missing = REQUIRED - v.keys()
    if missing or "jt" not in extra:
        raise ValueError(f"{name}: rows missing: {sorted(missing) or 'backlash'}")
    if extra["t012_temp"] != "20":
        raise ValueError(f"{name}: no-load torque at {extra['t012_temp']} °C, not 20 °C")
    hubs = inertia(name, lines, n)
    rows = []
    for k, ratio in enumerate(ratios):
        for code, bore, j1 in hubs:
            if j1[k] is None:
                continue
            rows.append({
                "series": series, "size": f"{series}{digits}MF", "motor_frame": bore,
                "clamping_hub_diameter_m": to_si(bore, "milli"), "stages": str(stages), "ratio": ratio,
                "rated_output_torque_nm": v["t2n"][k], "max_output_torque_nm": v["t2b"][k],
                "emergency_stop_output_torque_nm": v["t2not"][k],
                "rated_input_speed_rad_s": to_si(v["n1n"][k], "rpm"),
                "max_input_speed_rad_s": to_si(v["n1max"][k], "rpm"),
                "no_load_torque_nm": v["t012"][k], "no_load_torque_speed_rad_s": to_si(extra["t012_speed"], "rpm"),
                "backlash_rad": to_si(extra["jt"], "arcmin"), "backlash_reduced_rad": to_si(extra["jt_red"], "arcmin"),
                "torsional_stiffness_nm_rad": to_si(v["ct21"][k], "per_arcmin"),
                "max_axial_force_n": v["f2a"][k], "max_radial_force_n": v["f2q"][k] if "f2q" in v else "",
                "max_tilting_moment_nm": v["m2k"][k], "efficiency_full_load": to_si(v["eta"][k], "percent"),
                "mass_kg": v["m"][k], "inertia_at_input_kgm2": to_si(j1[k], "kgcm2"),
            })
    return rows


def inertia(name, lines, n):
    """[(hub code, bore mm, J1 kg cm2 per ratio)]: the value rows alternate kg cm2 and 10^-3 in lb s2, hub by hub."""
    start = next((i for i, l in enumerate(lines) if l.strip().startswith("Protection class")), None)
    if start is None:
        raise ValueError(f"{name}: no protection class row before the inertia")
    hubs, values = [], []
    for line in lines[start:]:
        hubs += re.findall(r"\b([A-Z])\s+(\d{2,3})\s+J1\b", line)
        vals = trailing_values(line)
        if len(vals) == n and "X" not in line.split():
            values.append(vals)
    if not hubs or len(values) != 2 * len(hubs):
        raise ValueError(f"{name}: {len(hubs)} clamping hubs but {len(values)} inertia rows")
    out = []
    for h, (code, bore) in enumerate(hubs):
        kg, us = values[2 * h], values[2 * h + 1]
        for a, b in zip(kg, us):
            # Both are printed to two or three digits: allow 6 % or one unit of the last digit.
            if abs(float(a) - float(b) * KGCM2_PER_MILLI_IN_LB_S2) > 0.06 * float(a) + 0.011:
                raise ValueError(f"{name} hub {code} {bore}: J1 {a} kg cm2 does not match {b} 1e-3 in lb s2")
        out.append((code, bore, kg))
    return out


def main(argv):
    if len(argv) < 3:
        sys.exit("usage: extract_wittenstein.py sp.txt [tp.txt ...] out_folder")
    out = Path(argv[-1])
    out.mkdir(parents=True, exist_ok=True)
    tables = {}
    for path in argv[1:-1]:
        lines = Path(path).read_text(encoding="utf-8").splitlines()
        found = blocks(lines)
        if not found:
            sys.exit(f"{path}: no SP+ or TP+ MF table")
        for series, digits, stages, block in found:
            try:
                tables.setdefault(series, []).extend(parse_block(series, digits, stages, block))
            except ValueError as e:
                sys.exit(str(e))
    for series, rows in tables.items():
        keys = [(r["size"], r["motor_frame"], r["ratio"]) for r in rows]
        dup = {k for k in keys if keys.count(k) > 1}
        if dup:
            sys.exit(f"{series}: rows twice: {sorted(dup)}")
        with open(out / FILES[series], "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, COLUMNS, lineterminator="\n")
            w.writeheader()
            w.writerows(rows)
        print(f"{FILES[series]}: {len(rows)} rows")


if __name__ == "__main__":
    main(sys.argv)
