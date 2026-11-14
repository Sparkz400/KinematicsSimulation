"""Extracts the Bosch Rexroth MS2N servo motors (frame sizes 03 ... 10) from the MS2N project planning manual
into catalog/rexroth/motors_ms2n.csv.

The manual is not part of this repository. Get it from Bosch Rexroth (R911347583, edition 11,
https://www.boschrexroth.com/media/df1540c6-849e-4d7f-a609-6916c08eb78c), then:

    pdftotext -raw MS2N_R911347583_11.pdf ms2n.txt        (xpdf 4.x; -raw keeps each table row's values together)
    python extract_rexroth.py ms2n.txt ../rexroth/motors_ms2n.csv

Each motor has one table ("Designation Symbol Unit" + one column per type code with and without holding brake).
A row gives either one value for all columns or one value per column; anything else stops the script.
"""
import math
import re
import sys
from pathlib import Path

HEADER = ["part_number", "vendor", "series", "line_voltage_v", "static_torque_nm", "stall_current_a",
          "rated_torque_nm", "rated_current_a", "rated_speed_rad_s", "rated_power_w", "max_torque_nm", "max_current_a",
          "max_speed_rad_s", "torque_constant_nm_a", "voltage_constant_v_s_rad", "line_resistance_20c_ohm",
          "line_inductance_h", "pole_pairs", "rotor_inertia_kgm2", "rotor_inertia_with_brake_kgm2", "mass_kg",
          "mass_with_brake_kg", "thermal_time_constant_s", "paired_drive", "source"]
# label at the start of the line -> unit printed in front of the values
FIELDS = {
    "m0": (r"Standstill torque - 100K", "Nm"),
    "i0": (r"Standstill current - 100K", "A"),
    "j": (r"Moment of inertia of rotor", r"kg\*m2"),
    "nn": (r"Rated speed - 100K", "1/min"),
    "mn": (r"Rated torque - 100K", "Nm"),
    "in": (r"Rated current - 100K", "A"),
    "pn": (r"Rated power - 100K", "kW"),
    "mmax": (r"Maximum torque 100K", "Nm"),
    "imax": (r"Maximum current", "A"),
    "nel": (r"Maximum speed \(electrical\)", "1/min"),
    "nmech": (r"Maximum speed \(mechanical\)", "1/min"),
    "km": (r"Torque constant", "Nm/A"),
    "ke": (r"Voltage constant", "min-1"),
    "r": (r"Winding resistance", "Ohm"),
    "l": (r"Winding inductance", "mH"),
    "tth": (r"Thermal time constant of motor", "min"),
    "mass": (r"Mass mmot", "kg"),
}
COLUMN = re.compile(r"(MS2N\d\d-[A-Z0-9]{5})-(\d)-N")
FRAMES = ("03", "04", "05", "06", "07", "10")


def values(lines, label, unit, cols_pn, what):
    """The values of one row, one per column (a single printed value holds for every column)."""
    ncols, groups = len(cols_pn), list(dict.fromkeys(cols_pn))
    start = next((i for i, l in enumerate(lines) if re.match(label, l)), None)
    if start is None:
        return [""] * ncols
    for i in range(start, min(start + 5, len(lines) - 1)):
        line = lines[i]
        m = re.match(rf"(?:.*\s)?{unit}\s*(.*)$", line)
        if m:
            rest = m.group(1).strip() or lines[i + 1].strip()
            tokens = [t.replace(",", "") for t in rest.split()]
            if not all(re.fullmatch(r"\d+(\.\d+)?|-", t) for t in tokens):
                raise ValueError(f"{what} {label}: not numbers: {tokens}")
            tokens = ["" if t == "-" else t for t in tokens]
            if len(tokens) == 1:
                return tokens * ncols
            if len(tokens) == ncols:
                return tokens
            if len(tokens) == len(groups):  # one value per type code, spanning its brake columns
                return [tokens[groups.index(g)] for g in cols_pn]
            raise ValueError(f"{what} {label}: {len(tokens)} values for {ncols} columns: {tokens}")
    raise ValueError(f"{what} {label}: no row with unit {unit}")


def scale(v, f):
    """v * f, without the float noise of the multiplication."""
    return "" if v == "" else repr(float(f"{float(v) * f:.12g}"))


def tables(text):
    """(page, columns, lines) of each motor table."""
    parts = text.split("Designation Symbol Unit")
    page = None
    for before, part in zip(parts, parts[1:]):
        pages = re.findall(r"(\d+) / 444", before)
        page = int(pages[-1]) if pages else page
        body = part.split("For tolerance details")[0]
        head = re.sub(r"[\s_]+", "", body.split("Standstill")[0])  # one header prints "_0- N"
        cols = COLUMN.findall(head)
        if cols:
            yield page, cols, body.splitlines()


def main():
    text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
    rows = {}
    for page, cols, lines in tables(text):
        if cols[0][0][4:6] not in FRAMES:
            continue
        what = f"page {page} {cols[0][0]}"
        v = {k: values(lines, label, unit, [c[0] for c in cols], what) for k, (label, unit) in FIELDS.items()}
        pole = re.search(r"Number of pole pairs p (?:- )?(\d+)", "\n".join(lines))
        for pn in dict.fromkeys(c[0] for c in cols):
            plain = next(i for i, c in enumerate(cols) if c == (pn, "0"))
            brake = next((i for i, c in enumerate(cols) if c[0] == pn and c[1] != "0"), None)
            g = {k: x[plain] for k, x in v.items()}
            speeds = [float(s) for s in (g["nel"], g["nmech"]) if s]
            rows[pn] = [
                pn, "rexroth", "MS2N", "400", g["m0"], g["i0"], g["mn"], g["in"], "" if g["nn"] == "" else repr(float(g["nn"]) * math.pi / 30),
                scale(g["pn"], 1000), g["mmax"], g["imax"], repr(min(speeds) * math.pi / 30) if speeds else "",
                g["km"], scale(g["ke"], 30 / (1000 * math.pi)), g["r"], scale(g["l"], 1e-3),
                pole.group(1) if pole else "", g["j"], v["j"][brake] if brake is not None else "", g["mass"],
                v["mass"][brake] if brake is not None else "", scale(g["tth"], 60), "", ""]
    out = Path(sys.argv[2])
    with open(out, "w", encoding="utf-8", newline="\n") as f:
        f.write(",".join(HEADER) + "\n")
        for pn in sorted(rows):
            f.write(",".join(rows[pn]) + "\n")
    print(f"{len(rows)} motors -> {out}")


if __name__ == "__main__":
    main()
