"""Extracts the AM8000 and AM8500 servo motors from Beckhoff's operating instructions
"AM8000 and AM8500 Synchronous Servomotors", version 4.9.1 (2023-09-19), chapter 4 "Technical data",
into catalog/beckhoff/motors_am8000.csv.

The document is not part of this repository. Get the PDF from Beckhoff
(https://download.beckhoff.com/download/document/motion/am8000_am8500_ba_en.pdf), then:

    pdftotext -table -enc UTF-8 -f 30 -l 77 am8000_am8500_ba_en.pdf am8000.txt      (xpdf 4.x)
    python tools/extract_beckhoff.py am8000.txt beckhoff

Each "Electrical data" table has one column per winding (e.g. 31C); a value printed once across the
table holds for every column. The "Mechanical data" table after the electrical tables of a size has
one column per motor (AM8031, AM8531, ...). The sections "... with fan cover [+]" repeat the tables for
the motors with fan; their part numbers get "-fan", as some windings are in both. Rows: one per motor, winding and line voltage (400 V and 480 V, the voltages the
document gives rated values for and the AX5000/AX8000 drives take). A row whose number of values is
neither one nor the number of columns stops the script.
"""
import csv
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["part_number", "vendor", "series", "line_voltage_v", "static_torque_nm", "stall_current_a",
           "rated_torque_nm", "rated_current_a", "rated_speed_rad_s", "rated_power_w", "max_torque_nm",
           "max_current_a", "max_speed_rad_s", "torque_constant_nm_a", "voltage_constant_v_s_rad",
           "line_resistance_20c_ohm", "line_inductance_h", "pole_pairs", "rotor_inertia_kgm2",
           "rotor_inertia_with_brake_kgm2", "mass_kg", "mass_with_brake_kg", "thermal_time_constant_s",
           "paired_drive", "source"]

# label at the start of a line -> key; the first match wins, so longer labels come first.
ELECTRICAL = [
    ("Standstill torque", "m0"), ("Standstill current", "i0"), ("Maximum mechanical speed", "nmax"),
    ("Peak current", "imax"), ("Peak torque", "mmax"), ("Torque constant", "kt"), ("Voltage constant", "ke"),
    ("Winding resistance", "r"), ("Winding inductance", "l"), ("Nominal speed", "nn"),
    ("Nominal torque", "mn"), ("Nominal output", "pn"), ("Nominal current", "in"),
]
MECHANICAL = [
    ("Rotor moment of inertia with brake", "jb"), ("Rotor moment of inertia", "j"), ("Number of poles", "poles"),
    ("Thermal time constant", "tth"), ("Weight with brake", "mb"), ("Weight", "m"),
]
RATED = {"nn", "mn", "pn", "in"}
NUMBER = re.compile(r"^(\d[\d,]*(\.\d+)?|---)$")


def trailing_values(line):
    """The run of numbers (or ---) at the end of a line, thousands commas removed; --- is None."""
    out = []
    for token in reversed(line.split()):
        if not NUMBER.match(token):
            break
        out.append(None if token == "---" else token.replace(",", ""))
    return out[::-1]


def spread(values, n, where):
    if len(values) == n:
        return values
    if len(values) == 1:
        return values * n
    raise ValueError(f"{where}: {len(values)} values {values} for {n} columns")


def winding(token):
    """'31C' or the misordered 'J52' -> '52J'."""
    m = re.fullmatch(r"(\d\d)([A-Z])|([A-Z])(\d\d)", token)
    if not m:
        raise ValueError(f"not a winding: {token}")
    return (m[1] + m[2]) if m[1] else (m[4] + m[3])


def motor_columns(size, tokens, with_am85):
    """The motors of a mechanical table, by position: some headers are cut off ('AM806')."""
    if with_am85:
        names = []
        for length in range(1, 5):
            names += [f"AM80{size}{length}", f"AM85{size}{length}"]
        names = names[:len(tokens)]
    else:
        names = [f"AM80{size}{length}" for length in range(1, len(tokens) + 1)]
    for name, token in zip(names, tokens):
        if not (name.startswith(token) or token == name[-2:]):
            raise ValueError(f"mechanical header {tokens} does not match {names}")
    return names


def parse(lines):
    rows, pending, size, fan, i = [], [], None, False, 0
    while i < len(lines):
        line = lines[i]
        m = re.match(r"^4\.\d+AM80(\d)x", line)
        if m:
            size, fan = m[1], False
        if re.match(r"^4\.\d+\.\d+AM80\dx.*with fan", line):
            fan = True
        if line.startswith("Electrical data"):
            with_am85 = "AM85xx" in line
            i += 1
            while not lines[i].strip():
                i += 1
            windings = [winding(t) for t in lines[i].split()]
            block = {"windings": windings, "am85": with_am85, "fan": fan, "size": size, "v": {}}
            supply = None
            i += 1
            while not lines[i].startswith("Connection technology"):
                text = lines[i]
                s = re.match(r"^Power supply UN= (\d+) V", text)
                if s:
                    supply = int(s[1])
                for label, key in ELECTRICAL:
                    if text.startswith(label):
                        vals = spread(trailing_values(text), len(windings), f"{windings} {label}")
                        if key in RATED:
                            block["v"].setdefault(supply, {})[key] = vals
                        else:
                            block[key] = vals
                        break
                i += 1
            pending.append(block)
        elif line.startswith("Mechanical data"):
            tokens = line.split()[2:]
            with_am85 = "AM85xx" in line or any(t.startswith("AM85") for t in tokens)
            if tokens and tokens[0].startswith("AM80xx"):
                i += 1
                while not lines[i].strip():
                    i += 1
                tokens = lines[i].split()
            names = motor_columns(size, tokens, with_am85)
            mech = {}
            i += 1
            while not lines[i].startswith(("Flange", "Optional holding brake")):
                for label, key in MECHANICAL:
                    if lines[i].startswith(label):
                        mech[key] = dict(zip(names, spread(trailing_values(lines[i]), len(names), f"{names} {label}")))
                        break
                i += 1
            for block in pending:
                rows += motor_rows(block, mech)
            pending = []
        i += 1
    if pending:
        raise ValueError("electrical tables without a mechanical table")
    return rows


def motor_rows(block, mech):
    out = []
    for c, w in enumerate(block["windings"]):
        frames = [f"AM80{w[:2]}"] + ([f"AM85{w[:2]}"] if block["am85"] else [])
        for frame in frames:
            if frame not in mech["j"]:
                continue
            for volt in (400, 480):
                rated = block["v"].get(volt)
                if rated is None:
                    continue
                get = lambda key: rated[key][c] if key in rated else ""
                poles = mech["poles"][frame]
                series = frame[:4] + "00" + (" with fan cover" if block["fan"] else "")
                row = {
                    "part_number": f"{frame}-0{w[2]}" + ("-fan" if block["fan"] else ""), "vendor": "beckhoff", "series": series,
                    "line_voltage_v": str(volt),
                    "static_torque_nm": block["m0"][c], "stall_current_a": block["i0"][c],
                    "rated_torque_nm": get("mn"), "rated_current_a": get("in"),
                    "rated_speed_rad_s": to_si(get("nn"), "rpm"), "rated_power_w": to_si(get("pn"), "kilo"),
                    "max_torque_nm": block["mmax"][c], "max_current_a": block["imax"][c],
                    "max_speed_rad_s": to_si(block["nmax"][c], "rpm"), "torque_constant_nm_a": block["kt"][c],
                    "voltage_constant_v_s_rad": to_si(block["ke"][c], "v_per_krpm"),
                    "line_resistance_20c_ohm": block["r"][c], "line_inductance_h": to_si(block["l"][c], "milli"),
                    "pole_pairs": str(int(poles) // 2), "rotor_inertia_kgm2": to_si(mech["j"][frame], "kgcm2"),
                    "rotor_inertia_with_brake_kgm2": to_si(mech["jb"][frame] or "", "kgcm2"),
                    "mass_kg": mech["m"][frame], "mass_with_brake_kg": mech["mb"][frame] or "",
                    "thermal_time_constant_s": to_si(mech["tth"][frame], "minute"),
                    "paired_drive": "", "source": "",
                }
                out.append(row)
    return out


def check(rows):
    """The spec's plausibility checks; prints what fails."""
    f = lambda r, k: float(r[k]) if r[k] else None
    bad = []
    for r in rows:
        name = f"{r['part_number']}@{r['line_voltage_v']}"
        m0, i0, kt, ke = f(r, "static_torque_nm"), f(r, "stall_current_a"), f(r, "torque_constant_nm_a"), f(r, "voltage_constant_v_s_rad")
        if f(r, "max_torque_nm") <= m0:
            bad.append(f"{name}: max torque {r['max_torque_nm']} <= static {m0}")
        if abs(m0 / i0 / kt - 1) > 0.15:
            bad.append(f"{name}: M0/I0 {m0 / i0:.3f} vs Kt {kt}")
        if abs(kt / (math.sqrt(3) * ke) - 1) > 0.20:
            bad.append(f"{name}: Kt {kt} vs sqrt3*Ke {math.sqrt(3) * ke:.3f}")
        mn, nn, pn = f(r, "rated_torque_nm"), f(r, "rated_speed_rad_s"), f(r, "rated_power_w")
        if abs(pn / (mn * nn) - 1) > 0.12:
            bad.append(f"{name}: Pn {pn} vs Mn*nn {mn * nn:.0f}")
        if mn > m0:
            bad.append(f"{name}: rated torque {mn} > static {m0}")
        if nn > f(r, "max_speed_rad_s"):
            bad.append(f"{name}: rated speed > max speed")
        if r["rotor_inertia_with_brake_kgm2"] and f(r, "rotor_inertia_with_brake_kgm2") < f(r, "rotor_inertia_kgm2"):
            bad.append(f"{name}: inertia with brake < without")
        if r["mass_with_brake_kg"] and f(r, "mass_with_brake_kg") < f(r, "mass_kg"):
            bad.append(f"{name}: mass with brake < without")
    return bad


def main():
    text, out = Path(sys.argv[1]), Path(sys.argv[2])
    rows = parse(text.read_text(encoding="utf-8").splitlines())
    keys = [(r["part_number"], r["line_voltage_v"]) for r in rows]
    dupes = {k for k in keys if keys.count(k) > 1}
    if dupes:
        raise ValueError(f"duplicate rows: {sorted(dupes)}")
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "motors_am8000.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    for message in check(rows):
        print("check:", message)
    print(f"{len(rows)} rows")


if __name__ == "__main__":
    main()
