"""Extracts the AL8000 linear servomotors (coil units) from Beckhoff's operating instructions "AL8000",
version 1.10.2 (2026-06-10), into catalog/beckhoff/linear_motors_al8000.csv.

The document is not part of this repository. Get the PDF from Beckhoff
(https://download.beckhoff.com/download/document/motion/al8000_ba_en.pdf), then:

    pdftotext -table -enc UTF-8 al8000_ba_en.pdf al8000.txt      (xpdf 4.x)
    python -I tools/extract_beckhoff_al8000.py al8000.txt beckhoff

Chapter 4 has one "Performance data" table per coil family (4.3 AL802x, 4.5 AL804x on two pages, 4.7 AL806x-0,
4.8 AL806x-1): one column per coil unit and winding (e.g. 21-0E), a value printed once across the table holds for
every column. Chapter 7.1 has the housing lengths D per coil size. A row whose number of values is neither one nor
the number of columns, a missing row, or a coil without a housing length stops the script.
"""
import csv
import math
import re
import sys
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["part_number", "series", "peak_force_n", "continuous_force_n", "force_constant_n_a",
           "back_emf_constant_v_s_m", "continuous_current_a", "peak_current_a", "phase_resistance_20c_ohm",
           "phase_inductance_h", "pole_pitch_m", "coil_mass_kg", "magnet_track_mass_kg_m",
           "magnet_track_unit_length_m", "attraction_force_n", "max_speed_m_s", "max_speed_at_peak_force_m_s",
           "motor_constant_n_sqrt_w", "continuous_power_loss_w", "thermal_time_constant_s", "line_voltage_v",
           "cooling", "coil_length_m", "paired_drive", "source"]

# label at the start of a line -> key
LABELS = [
    ("Peak force Fmax [N]", "fmax"), ("Peak current Imax [A]", "imax"), ("Continuous force Fc", "fc"),
    ("Continuous current Ic [A]", "ic"), ("Maximum velocity", "vmax"), ("Force constant Kf [N/A]", "kf"),
    ("Magnetic force of attraction Fa [N]", "fa"), ("Winding resistance ph-ph", "r"),
    ("Winding inductance ph-ph [mH]", "l"), ("Counter EMF ph-ph [V/m/s]", "ke"), ("Power loss PL [W]", "pl"),
    ("Motor constant [N²/W]", "km2"), ("Pole pair distance [mm]", "pole"), ("Coil unit [kg]", "mcoil"),
    ("Magnetic plate [kg/m]", "mplate"), ("AX8000", "ax8000"),
]
NUMBER = re.compile(r"^\d+(\.\d+)?$")


def trailing(line):
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
    raise ValueError(f"{where}: {len(values)} values {values} for {n} columns")


def performance_tables(lines):
    tables, i = [], 0
    while i < len(lines):
        if lines[i].startswith("Performance data"):
            i += 1
            while not lines[i].strip():
                i += 1
            cols = lines[i].split()
            if not all(re.fullmatch(r"[0-9][0-9A-F]-[01][A-Z]", c) for c in cols):
                raise ValueError(f"not a column header: {lines[i]}")
            t = {"cols": cols}
            while not lines[i].startswith("The values are valid"):
                for label, key in LABELS:
                    if lines[i].startswith(label):
                        vals = lines[i].split()[1:] if key == "ax8000" else trailing(lines[i])
                        t[key] = spread(vals, len(cols), f"{cols} {label}")
                        break
                i += 1
            missing = {k for _, k in LABELS} - t.keys()
            if missing:
                raise ValueError(f"{cols}: no {sorted(missing)}")
            tables.append(t)
        i += 1
    return tables


def housing_lengths(lines):
    """Chapter 7.1: 'Designation' tables, header line of sizes (21, 64-0 ...), then 'Housing length D [mm]'."""
    out, i = {}, 0
    while i < len(lines):
        if lines[i].startswith("Designation"):
            i += 1
            while not lines[i].strip():
                i += 1
            sizes = lines[i].split()
            while not lines[i].startswith("Housing length"):
                i += 1
            vals = spread(trailing(lines[i]), len(sizes), f"{sizes} housing length")
            out.update({f"AL80{s}": v for s, v in zip(sizes, vals)})
        i += 1
    return out


def coil_length(part, lengths):
    """'AL8064-0F' -> D of 'AL8064-0' (AL806x) or of 'AL8064' (AL802x, AL804x)."""
    for key in (part[:-1], part[:6]):
        if key in lengths:
            return lengths[key]
    raise ValueError(f"{part}: no housing length")


def rows_of(tables, lengths):
    rows = []
    for t in tables:
        for c, col in enumerate(t["cols"]):
            v = {k: t[k][c] for k in t if k != "cols"}
            part = f"AL80{col}"
            water = col[3] == "1"
            rows.append({
                "part_number": part, "series": "AL8000", "peak_force_n": v["fmax"], "continuous_force_n": v["fc"],
                "force_constant_n_a": v["kf"], "back_emf_constant_v_s_m": v["ke"],
                "continuous_current_a": v["ic"], "peak_current_a": v["imax"],
                # printed phase to phase; the schema's column is per phase (star connection: half)
                "phase_resistance_20c_ohm": format((Decimal(v["r"]) / 2).normalize(), "f"),
                "phase_inductance_h": to_si(format((Decimal(v["l"]) / 2).normalize(), "f"), "milli"),
                "pole_pitch_m": to_si(v["pole"], "milli"), "coil_mass_kg": v["mcoil"],
                "magnet_track_mass_kg_m": v["mplate"], "magnet_track_unit_length_m": "",
                "attraction_force_n": v["fa"], "max_speed_m_s": v["vmax"], "max_speed_at_peak_force_m_s": "",
                # printed as its square, in N^2/W
                "motor_constant_n_sqrt_w": f"{math.sqrt(float(v['km2'])):.4g}",
                "continuous_power_loss_w": v["pl"], "thermal_time_constant_s": "", "line_voltage_v": "400",
                "cooling": "water" if water else "convection",
                "coil_length_m": to_si(coil_length(part, lengths), "milli"), "paired_drive": v["ax8000"],
                "source": "",
            })
    return rows


def main():
    text, out = Path(sys.argv[1]), Path(sys.argv[2])
    lines = text.read_text(encoding="utf-8").splitlines()
    rows = rows_of(performance_tables(lines), housing_lengths(lines))
    parts = [r["part_number"] for r in rows]
    if len(set(parts)) != len(parts):
        raise ValueError(f"duplicate coil units: {sorted({p for p in parts if parts.count(p) > 1})}")
    if len(rows) != 30:
        raise ValueError(f"{len(rows)} coil units, version 1.10.2 has 30")
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "linear_motors_al8000.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} rows")


if __name__ == "__main__":
    main()
