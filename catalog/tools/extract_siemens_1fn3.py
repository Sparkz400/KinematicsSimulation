"""Extracts the SIMOTICS L-1FN3 linear motors (peak load and continuous load) from Siemens' Configuration Manual
"Linear motors SIMOTICS L-1FN3", edition 06/2023, A5E50685682B AB, into catalog/siemens/linear_motors_1fn3.csv.

The document is not part of this repository. Get the PDF from Siemens
(https://cache.industry.siemens.com/dl/files/768/109475768/att_1151738/v1/1FN3_config_man_0623_en-US.pdf), then:

    pdftotext -table -enc UTF-8 1FN3_config_man_0623_en-US.pdf 1fn3.txt      (xpdf 4.x)
    python -I tools/extract_siemens_1fn3.py 1fn3.txt siemens

Electrical, thermal and mass data come from the data sheets of chapter 7.2 ("Data sheet of 1FN3...", one per
primary section, label / designation / unit / value lines). The primary section length lP and the secondary section
length lS come from Tables 3-14, 3-16 and 3-18 ("Overview of the most important data ... / Part 2"); the rated and
maximum values of Tables 3-13, 3-15 and 3-17 (Part 1) are compared with the data sheets. Every data sheet value
must have the unit printed below; a missing value, a second value, an other unit or a table row with the wrong
number of values stops the script.
"""
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["part_number", "series", "peak_force_n", "continuous_force_n", "force_constant_n_a",
           "back_emf_constant_v_s_m", "continuous_current_a", "peak_current_a", "phase_resistance_20c_ohm",
           "phase_inductance_h", "pole_pitch_m", "coil_mass_kg", "magnet_track_mass_kg_m",
           "magnet_track_unit_length_m", "attraction_force_n", "max_speed_m_s", "max_speed_at_peak_force_m_s",
           "motor_constant_n_sqrt_w", "continuous_power_loss_w", "thermal_time_constant_s", "line_voltage_v",
           "cooling", "coil_length_m", "paired_drive", "source"]

# designation -> (unit as printed, or the spellings of it, key)
SHEET = {
    "UDC": ("V", "udc"), "FN": ("N", "fn"), "IN": ("A", "in"), "vMAX,FN": ("m/min", "vfn"), "PV,N": ("kW", "pv"),
    "FMAX": ("N", "fmax"), "IMAX": ("A", "imax"), "vMAX,FMAX": ("m/min", "vfmax"), "kF,20": ("N/A", "kf"),
    "kE": ("Vs/m", "ke"), "kM,20": (("N/W0.5", "N/(W)0.5"), "km"), "RSTR,20": ("Ω", "r"), "LSTR": ("mH", "l"),
    "FA": ("N", "fa"), "tTH": ("s", "tth"), "τM": ("mm", "tau"), "mP": ("kg", "mp"), "mS": ("kg", "ms"),
}
# misprint kept out: this data sheet prints its powers in "W" where every other prints kW (1.39 "W" rated power loss;
# Table 3-17 has 1.39 kW), so its "W" is read as kW
UNIT_MISPRINTS = {("1FN3450-2NB80-0xAx", "PV,N"): "W"}
VALUE_LINE = re.compile(r"^(\S.*?)\s{2,}(\S+)\s+(\S+)\s+(\d[\d.]*)\s*$")
PART = r"1FN3\d{3}-\d[WN][A-Z]\d\d-0xAx"


def m_per_min(text):
    return repr(float(text) / 60)


def data_sheets(lines):
    sheets, current, part = {}, None, None
    for line in lines:
        m = re.match(rf"^Data sheet of ({PART})\s*$", line)
        if m:
            if m[1] in sheets:
                raise ValueError(f"second data sheet of {m[1]}")
            part, current = m[1], {}
            sheets[part] = current
            continue
        if current is None:
            continue
        if line.startswith(("Characteristics of", "7.2.")):
            current = None
            continue
        v = VALUE_LINE.match(line)
        if v and v[2] in SHEET:
            unit, key = SHEET[v[2]]
            if v[3] not in ((unit,) if isinstance(unit, str) else unit) and UNIT_MISPRINTS.get((part, v[2])) != v[3]:
                raise ValueError(f"{v[2]}: unit {v[3]}, expected {unit}: {line}")
            if key in current:
                raise ValueError(f"{key} twice in a data sheet: {line}")
            current[key] = v[4]
    for part, sheet in sheets.items():
        missing = {k for _, k in SHEET.values()} - sheet.keys()
        if missing:
            raise ValueError(f"{part}: data sheet lacks {sorted(missing)}")
    return sheets


def overview(lines):
    """Part 1 rows (FN FMAX IN IMAX vFN vFMAX PV) by (table, article); Part 2 (lP, lS, mS) by article, repeated rows
    must agree."""
    part1, part2 = {}, {}
    table, number = None, None
    for line in lines:
        if re.match(r"^3\.5\s+Rating plate data", line):
            table = None
        t = re.match(r"^Table 3-1([3-8])\s", line)
        if t:
            table, number = (1 if t[1] in "357" else 2), f"3-1{t[1]}"
            continue
        m = re.match(rf"^({PART})\s+(.*)$", line)
        if not m or table is None:
            continue
        rest = re.sub(r"PG\d+\s*/\s*M[\dx.]+", " ", m[2])  # thread column of Table 3-16
        nums = [x for x in rest.split() if x != "/"]
        if table == 1:
            if len(nums) != 7:
                raise ValueError(f"Part 1 row with {len(nums)} values: {line}")
            row = dict(zip(["fn", "fmax", "in", "imax", "vfn", "vfmax", "pv"], nums))
            part1[(number, m[1])] = row  # Tables 3-13 and 3-15 repeat each other; compare() reports differences
            continue
        else:
            # hM3 [hM1] bP [bPK1] lP lP,AKT mP [mP,P] lS mS mS,P: 11 values, 8 without precision cooler
            if len(nums) == 11:
                row = {"lp": nums[4], "ls": nums[8], "ms": nums[9]}
            elif len(nums) == 8:
                row = {"lp": nums[2], "ls": nums[5], "ms": nums[6]}
            else:
                raise ValueError(f"Part 2 row with {len(nums)} values: {line}")
            target = part2
        if m[1] in target and target[m[1]] != row:
            raise ValueError(f"{m[1]}: overview rows disagree: {target[m[1]]} vs {row}")
        target[m[1]] = row
    return part1, part2


def rows_of(sheets, part2):
    rows = []
    for part, s in sheets.items():
        if part not in part2:
            raise ValueError(f"{part}: not in Tables 3-14/3-16/3-18")
        o = part2[part]
        if o["ms"] != s["ms"]:
            raise ValueError(f"{part}: mS {o['ms']} in the overview, {s['ms']} in the data sheet")
        peak = part[9] == "W"
        rows.append({
            "part_number": part, "series": "1FN3 peak load" if peak else "1FN3 continuous load",
            "peak_force_n": s["fmax"], "continuous_force_n": s["fn"], "force_constant_n_a": s["kf"],
            "back_emf_constant_v_s_m": s["ke"], "continuous_current_a": s["in"], "peak_current_a": s["imax"],
            "phase_resistance_20c_ohm": s["r"], "phase_inductance_h": to_si(s["l"], "milli"),
            # tau_M is north centre to south centre; the schema's pole pitch is north to north
            "pole_pitch_m": to_si(str(2 * float(s["tau"])).removesuffix(".0"), "milli"),
            "coil_mass_kg": s["mp"],
            "magnet_track_mass_kg_m": f"{float(s['ms']) / (float(o['ls']) / 1000):.6g}",
            "magnet_track_unit_length_m": to_si(o["ls"], "milli"), "attraction_force_n": s["fa"],
            "max_speed_m_s": m_per_min(s["vfn"]), "max_speed_at_peak_force_m_s": m_per_min(s["vfmax"]),
            "motor_constant_n_sqrt_w": s["km"], "continuous_power_loss_w": to_si(s["pv"], "kilo"),
            "thermal_time_constant_s": s["tth"], "line_voltage_v": s["udc"], "cooling": "water",
            "coil_length_m": to_si(o["lp"], "milli"), "paired_drive": "", "source": "",
        })
    return rows


def compare(sheets, part1):
    """Part 1 overview values that differ from the data sheet (printed, not fatal)."""
    out = []
    for (table, part), o in sorted(part1.items()):
        s = sheets.get(part)
        if s is None:
            out.append(f"Table {table} {part}: no data sheet")
            continue
        for k, v in o.items():
            if float(v) != float(s[k]):
                out.append(f"Table {table} {part}: {k} {v}, data sheet {s[k]}")
    listed = {p for _, p in part1}
    out += [f"{p}: data sheet, not in a Part 1 table" for p in sorted(sheets.keys() - listed)]
    return out


def main():
    text, out = Path(sys.argv[1]), Path(sys.argv[2])
    lines = text.read_text(encoding="utf-8").splitlines()
    sheets = data_sheets(lines)
    part1, part2 = overview(lines)
    if len(sheets) != 98:
        raise ValueError(f"{len(sheets)} data sheets, the 06/2023 edition has 98")
    rows = rows_of(sheets, part2)
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "linear_motors_1fn3.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    for message in compare(sheets, part1):
        print("overview:", message)
    print(f"{len(rows)} rows")


if __name__ == "__main__":
    main()
