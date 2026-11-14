"""Extracts B&R 8LSA...-3 servo motors and ACOPOS P3 servo drives (3x 200-480 VAC modules) into
catalog/br/motors_8ls.csv and catalog/br/drives_acopos_p3.csv.

The manuals are not part of this repository. Get from br-automation.com the "8LSA/C/O/P...-3 Users manual
(Motor version: 3)" V2.51 (MAMOT2-ENG) and the "ACOPOS P3 User's manual" V1.40 (MAACPP3-ENG), then:

    pdftotext -table -fixed 2 -f 52 -l 167 8LS-A-C-O-P...-3_Users_manual_Motor-V3_Manual-V2.51.pdf 8ls.txt   (xpdf 4.x)
    pdftotext -table -fixed 2 -f 24 -l 134 MAACPP-ENG_V1.40.pdf p3.txt
    python extract_br.py 8ls.txt p3.txt ../br

-fixed 2 puts one character per 2 pt, so a value's position on the line is where it is on the page. The motor
tables print a value that several windings share once, centred over their columns: each row is split into
spans that cover every column once, left to right, by where each value's centre lies. A row whose values do
not cover the columns that way, or a table that is missing, stops the script.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

MOTOR_COLUMNS = (
    "part_number,vendor,series,line_voltage_v,static_torque_nm,stall_current_a,rated_torque_nm,rated_current_a,"
    "rated_speed_rad_s,rated_power_w,max_torque_nm,max_current_a,max_speed_rad_s,torque_constant_nm_a,"
    "voltage_constant_v_s_rad,line_resistance_20c_ohm,line_inductance_h,pole_pairs,rotor_inertia_kgm2,"
    "rotor_inertia_with_brake_kgm2,mass_kg,mass_with_brake_kg,thermal_time_constant_s,paired_drive,source"
).split(",")
DRIVE_COLUMNS = (
    "part_number,vendor,series,supply,line_voltage_v,axes,rated_output_current_a,max_output_current_a,"
    "max_current_duration_s,dc_link_voltage_v,dc_link_capacitance_f,braking_resistor_internal_ohm,"
    "braking_resistor_continuous_w,braking_resistor_peak_w,min_braking_resistance_ohm,braking_threshold_v,"
    "dc_link_overvoltage_v,source"
).split(",")

# Row label of the motor tables -> (key, conversion to SI)
MOTOR_ROWS = {
    "Nominal speed nN [rpm]": ("nN", "rpm"),
    "Number of pole pairs": ("pp", "same"),
    "Nominal torque Mn [Nm]": ("MN", "same"),
    "Nominal power PN [W]": ("PN", "same"),
    "Nominal current IN [A]": ("IN", "same"),
    "Stall torque M0 [Nm]": ("M0", "same"),
    "Stall current I0 [A]": ("I0", "same"),
    "Maximum torque Mmax [Nm]": ("Mmax", "same"),
    "Maximum current Imax [A]": ("Imax", "same"),
    "Maximum speed nmax [rpm]": ("nmax", "rpm"),
    "Torque constant KT [Nm/A]": ("KT", "same"),
    "Voltage constant KE [V/1000 rpm]": ("KE", "v_per_krpm"),
    "Stator resistance R2ph": ("R", "same"),
    "Stator inductance L2ph [mH]": ("L", "milli"),
    "Thermal time constant ttherm [min]": ("tth", "minute"),
    "Moment of inertia J [kgcm2]": ("J", "kgcm2"),
    "Weight without brake m [kg]": ("m", "same"),
    "Mass of brake [kg]": ("mBr", "same"),
    "Moment of inertia of brake JBr [kgcm2]": ("JBr", "kgcm2"),
    "ACOPOS P3 8EIxxxx...": ("P3", None),
}
NUMBER = re.compile(r"^\d+(\.\d+)?$")


def fail(message):
    sys.exit(f"extract_br: {message}")


def tokens(line, start=0):
    """(text, centre) of each blank-separated word from column `start` on."""
    return [(m.group(), (m.start() + m.end()) / 2) for m in re.finditer(r"\S+", line) if m.start() >= start]


def spans(values, anchors, where):
    """Split a row into spans covering every column once, left to right: [(value, first, last)]."""
    out, first = [], 0
    for text, centre in values:
        if first >= len(anchors):
            fail(f"{where}: more values than columns")
        last = min(range(first, len(anchors)), key=lambda b: abs((anchors[first] + anchors[b]) / 2 - centre))
        if abs((anchors[first] + anchors[last]) / 2 - centre) > 4:
            fail(f"{where}: {text} at {centre} fits no span from column {first + 1}")
        out.append((text, first, last))
        first = last + 1
    if first != len(anchors):
        fail(f"{where}: values cover {first} of {len(anchors)} columns")
    return out


def motor_tables(text):
    """Every 8LSA technical data table: [(model numbers, {key: [value per column]})]."""
    lines = text.replace("\f", "\n").splitlines()
    tables = []
    for i, line in enumerate(lines):
        if not (line.strip().startswith("Model number") and "8LSA" in line):
            continue
        heads = [t for t in tokens(line) if t[0].startswith("8LSA")]
        start = line.index(heads[0][0]) - 12
        j = i + 1
        if all(t[0].endswith(".") for t in heads):
            while not lines[j].strip():
                j += 1
            tails = tokens(lines[j])
            if len(tails) != len(heads) or not all(re.fullmatch(r"ee\d{3}ffgg-3", t[0]) for t in tails):
                fail(f"line {j + 1}: second header line does not match {heads}")
            names = [h[0] + t[0] for h, t in zip(heads, tails)]
            j += 1
        else:
            names = [h[0] for h in heads]
        if not all(re.fullmatch(r"8LSA\w\w\.ee\d{3}ffgg-3", n) for n in names):
            fail(f"line {i + 1}: model numbers {names}")
        rows = {}
        while not lines[j].strip().startswith("NOTE"):
            for label, (key, _) in MOTOR_ROWS.items():
                if lines[j].strip().startswith(label):
                    rows[key] = (j, tokens(lines[j], start))
            j += 1
        if set(rows) != {k for k, _ in MOTOR_ROWS.values()}:
            fail(f"line {i + 1}: rows missing: {set(k for k, _ in MOTOR_ROWS.values()) - set(rows)}")
        # Column positions from the rows with one value per column (values sit a little right of the header).
        full = [[c for _, c in vals] for _, vals in rows.values() if len(vals) == len(names)]
        if not full:
            fail(f"line {i + 1}: no row with a value per column")
        anchors = [sorted(col)[len(col) // 2] for col in zip(*full)]
        data = {}
        for key, (at, vals) in rows.items():
            cells = [None] * len(names)
            for value, first, last in spans(vals, anchors, f"line {at + 1} ({key})"):
                for c in range(first, last + 1):
                    cells[c] = value
            data[key] = cells
        tables.append((names, data))
    return tables


def p3_drive(code):
    """The 1-axis ACOPOS P3 for 3x 200-480 VAC with the manual's code: "2X2X" -> 8EI2X2HWS10.XXXX-1; "-": none."""
    if code == "-":
        return ""
    if not re.fullmatch(r"[0-9X]{3}X", code):
        fail(f"ACOPOS P3 recommendation {code}")
    return f"8EI{code[:3]}HWS10.XXXX-1"


def motors(text):
    rows = []
    for names, data in motor_tables(text):
        for c, name in enumerate(names):
            v = {k: cells[c] for k, cells in data.items()}
            for k, x in v.items():
                if k != "P3" and not NUMBER.match(x):
                    fail(f"{name}: {k} = {x}")
            si = {key: to_si(v[key], conv) for key, conv in MOTOR_ROWS.values() if conv}
            rows.append({
                "part_number": name.removesuffix("ffgg-3"),
                "vendor": "br",
                "series": "8LSA...-3",
                "line_voltage_v": "400",
                "static_torque_nm": si["M0"],
                "stall_current_a": si["I0"],
                "rated_torque_nm": si["MN"],
                "rated_current_a": si["IN"],
                "rated_speed_rad_s": si["nN"],
                "rated_power_w": si["PN"],
                "max_torque_nm": si["Mmax"],
                "max_current_a": si["Imax"],
                "max_speed_rad_s": si["nmax"],
                "torque_constant_nm_a": si["KT"],
                "voltage_constant_v_s_rad": si["KE"],
                "line_resistance_20c_ohm": si["R"],
                "line_inductance_h": si["L"],
                "pole_pairs": si["pp"],
                "rotor_inertia_kgm2": si["J"],
                "rotor_inertia_with_brake_kgm2": format((Decimal(si["J"]) + Decimal(si["JBr"])).normalize(), "f"),
                "mass_kg": si["m"],
                "mass_with_brake_kg": format((Decimal(v["m"]) + Decimal(v["mBr"])).normalize(), "f"),
                "thermal_time_constant_s": si["tth"],
                "paired_drive": p3_drive(v["P3"]),
                "source": "",
            })
    names = [r["part_number"] for r in rows]
    if len(set(names)) != len(names):
        fail("a motor appears twice")
    return rows


def unit(text, pattern, where):
    m = re.fullmatch(pattern, text)
    if not m:
        fail(f"{where}: '{text}' is not {pattern}")
    return m


def drives(text):
    """Per module of the 3x 200-480 VAC ("H") tables: currents per axis, DC bus and braking resistor."""
    found = {}
    for page in text.split("\f"):
        lines = page.splitlines()
        head = next((l for l in lines if re.match(r"\s*Order number\s+8EI\w+\.XXXX-1", l)), None)
        if head is None:
            continue
        parts = [t for t in tokens(head) if t[0].startswith("8EI")]
        if not all(p[0][6] == "H" for p in parts):
            continue  # 1x 110-230 VAC / 3x 200-230 VAC modules
        start = head.index(parts[0][0]) - 6
        for line in lines:
            label = line.strip()
            for key, prefix in (("rated", "Continuous current per motor con"), ("peak", "Peak current per motor connection"),
                                ("cap", "DC bus capacitance"), ("brk_peak", "Peak power int./ext."),
                                ("brk_cont", "Continuous power int./ext."), ("rmin", "Minimum braking resistance (ext.)")):
                if not label.startswith(prefix):
                    continue
                cells = [c.strip() for c in re.split(r"\s{3,}", line[start:].strip())]
                if len(cells) == 1:
                    cells = cells * len(parts)
                if len(cells) != len(parts):
                    fail(f"{parts[0][0]}: {key} has {len(cells)} values for {len(parts)} modules")
                for (part, _), cell in zip(parts, cells):
                    entry = found.setdefault(part, {})
                    if entry.get(key, cell) != cell:
                        fail(f"{part}: {key} {entry[key]} and {cell}")
                    entry[key] = cell
    rows = []
    for part in sorted(found):
        e = found[part]
        if not {"rated", "peak", "cap", "rmin"} <= set(e):
            fail(f"{part}: missing {[k for k in ('rated', 'peak', 'cap', 'rmin') if k not in e]}")
        axes = {"S": 1, "D": 2, "T": 3}[part[8]]
        rated = unit(e["rated"].replace(" ", ""), r"(\d+(?:\.\d+)?)(?:/\1)*Aeff", part).group(1)
        peak = unit(e["peak"].replace(" ", ""), r"(\d+(?:\.\d+)?)(?:/\1)*Aeff", part).group(1)
        for key, what in (("rated", e["rated"]), ("peak", e["peak"])):
            if what.replace(" ", "").count("/") != axes - 1:
                fail(f"{part}: {key} '{what}' is not one value per axis")
        cap = unit(e["cap"], r"(\d+) \S?F", part).group(1)  # the micro sign comes out in the PDF's own encoding
        rmin = unit(e["rmin"], r"(\d+)( \S)?", part).group(1)
        brk_peak = brk_cont = ""
        if "brk_peak" in e:  # modules without an internal braking resistor print only external values
            brk_peak = to_si(unit(e["brk_peak"], r"(\d+(?:\.\d+)?) kW / \d+ kW", part).group(1), "kilo")
            m = unit(e["brk_cont"], r"(\d+(?:\.\d+)?) (k?W) / \d+(?:\.\d+)? k?W", part)
            brk_cont = to_si(m.group(1), "kilo" if m.group(2) == "kW" else "same")
        rows.append({
            "part_number": part,
            "vendor": "br",
            "series": "ACOPOS P3",
            "supply": "3x 200 ... 480 VAC",
            "line_voltage_v": "400",
            "axes": str(axes),
            "rated_output_current_a": rated,
            "max_output_current_a": peak,
            "max_current_duration_s": "",
            "dc_link_voltage_v": "560",
            "dc_link_capacitance_f": to_si(cap, "micro"),
            "braking_resistor_internal_ohm": "",
            "braking_resistor_continuous_w": brk_cont,
            "braking_resistor_peak_w": brk_peak,
            "min_braking_resistance_ohm": rmin,
            "braking_threshold_v": "",
            "dc_link_overvoltage_v": "",
            "source": "",
        })
    return rows


def write(path, columns, rows):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, columns, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def main():
    motor_text, drive_text, out = sys.argv[1:4]
    m = motors(Path(motor_text).read_text(encoding="latin-1"))
    d = drives(Path(drive_text).read_text(encoding="latin-1"))
    known = {r["part_number"] for r in d}
    for r in m:
        if r["paired_drive"] and r["paired_drive"] not in known:
            fail(f"{r['part_number']}: paired drive {r['paired_drive']} is not in the drive table")
    write(Path(out) / "motors_8ls.csv", MOTOR_COLUMNS, m)
    write(Path(out) / "drives_acopos_p3.csv", DRIVE_COLUMNS, d)
    print(f"{len(m)} motors, {len(d)} drives")


if __name__ == "__main__":
    main()
