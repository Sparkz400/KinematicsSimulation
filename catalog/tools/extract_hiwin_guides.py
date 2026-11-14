"""Extracts HIWIN's HG (heavy load, ball) and EG (low profile, ball) profile rail guides from HIWIN's "Linear Guideway
Technical Information", FORM G99TE24-2410 (2024-10), into catalog/hiwin/linear_guides.csv.

The document is not part of this repository. Get the PDF from HIWIN
(https://hiwin.com/wp-content/uploads/2026/08/HIWIN-Linear-Guideway-Catalog-1.pdf), then:

    pdftotext -table -enc UTF-8 HIWIN-Linear-Guideway-Catalog-1.pdf guideway.txt      (xpdf 4.06)
    python tools/extract_hiwin_guides.py guideway.txt hiwin

Tables: 2-1-13 (1) HGH-CA (page 39), (5) HGW-CC (page 43), 2-2-13 (1) EGH-CA (page 58), (2) EGW-CA (page 59); the
seal resistance per end seal, Tables 2-1-21 (page 36) and 2-2-17 (page 55).
A dimension table prints the columns one size shares (height, rail ...) once, on a line of their own next to the
carriage rows of that size; the smallest size has all of them on its carriage row. Per table, HEADER lists the
columns in printed order and ROW the ones on the carriage row. A carriage row, or the one shared line next to it,
with another number of values stops the script, as does a model found other than once.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

COLUMNS = ["part_number", "series", "size", "carriage_type", "rail_width_m", "height_m", "carriage_length_m",
           "dynamic_load_rating_n", "dynamic_load_rating_distance_m", "static_load_rating_n", "static_moment_x_nm",
           "static_moment_y_nm", "static_moment_z_nm", "carriage_mass_kg", "rail_mass_kg_m", "seal_drag_n"]
TAIL = ["Cdyn", "C0", "MR", "MP", "MY", "mblock", "mrail"]
HG = ["H", "H1", "N", "W", "B", "B1", "C", "L1", "L", "K1", "K2", "G", "M", "T"]
RAIL = ["WR", "HR", "D", "h", "d", "P", "E", "bolt"]
TABLES = {  # prefix, carriage suffix, sizes, header, carriage row columns, carriage type
    "HGH": ("CA", [15, 20, 25, 30, 35, 45], HG + ["H2", "H3"] + RAIL + TAIL,
            ["C", "L1", "L", "K1"], "HGH-CA: square; tap hole; assembly height high; heavy load"),
    "HGW": ("CC", [15, 20, 25, 30, 35, 45], HG + ["T1", "T2", "H2", "H3"] + RAIL + TAIL,
            ["L1", "L", "K1"], "HGW-CC: flange; combination (tap and drilled hole); assembly height low; heavy load"),
    "EGH": ("CA", [15, 20, 25, 30, 35], HG + ["H2", "H3"] + RAIL + TAIL,
            ["C", "L1", "L", "K1"], "EGH-CA: square; tap hole; low profile; heavy load"),
    "EGW": ("CA", [15, 20, 25, 30, 35], HG + ["T1", "H2", "H3"] + RAIL + TAIL,
            ["C", "L1", "L", "K1"], "EGW-CA: flange; tap hole; low profile; heavy load"),
}
ROW_TAIL = ["Cdyn", "C0", "MR", "MP", "MY", "mblock"]
SEAL = re.compile(r"\b(HG|EG)(\d\d)\s+([\d.]+)\s*\(([\d.]+)\)")
SEAL_TABLES = {"HG": [15, 20, 25, 30, 35, 45], "EG": [15, 20, 25, 30, 35]}


def si(text, factor="1"):
    return format((Decimal(text) * Decimal(factor)).normalize(), "f")


def shared_line(lines, i, n):
    """The one line of n values that starts with blanks right before or after carriage row i (blank lines skipped)."""
    found = []
    for step in (-1, 1):
        j = i + step
        while 0 <= j < len(lines) and not lines[j].strip():
            j += step
        if 0 <= j < len(lines) and lines[j][:1].isspace() and len(lines[j].split()) == n:
            found.append(lines[j].split())
    if len(found) != 1:
        raise SystemExit(f"line {i + 1}: {len(found)} shared lines of {n} values next to {lines[i].split()[0]}")
    return found[0]


def carriage(lines, model, header, row_cols):
    hits = [i for i, line in enumerate(lines) if re.match(rf"{model}\s", line)]
    if len(hits) != 1:
        raise SystemExit(f"{model}: found on {len(hits)} lines, expected one")
    i = hits[0]
    t = lines[i].split()[1:]
    if len(t) == len(header):
        return dict(zip(header, t))
    row = [c for c in header if c in row_cols] + ROW_TAIL
    if len(t) != len(row):
        raise SystemExit(f"{model}: {len(t)} values, expected {len(row)} or {len(header)}")
    rest = [c for c in header if c not in row]
    return dict(zip(row, t)) | dict(zip(rest, shared_line(lines, i, len(rest))))


def seals(text):
    """Per end seal, N as printed, from the tables titled 'Seal Resistance' (up to their note)."""
    out, lines = {}, text.splitlines()
    block = []
    for i, line in enumerate(lines):
        if "Seal Resistance" in line:
            j = i + 1
            while j < len(lines) and not lines[j].strip().startswith("Note"):
                block.append(lines[j])
                j += 1
    for series, size, newton, _kgf in SEAL.findall(chr(10).join(block)):
        key = f"{series}{size}"
        if key in out and out[key] != newton:
            raise SystemExit(f"seal resistance of {key} printed twice: {out[key]} and {newton}")
        out[key] = newton
    for series, sizes in SEAL_TABLES.items():
        for size in sizes:
            if f"{series}{size}" not in out:
                raise SystemExit(f"seal resistance of {series}{size} missing")
    return out


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: extract_hiwin_guides.py guideway.txt out_dir")
    text = Path(sys.argv[1]).read_text(encoding="utf-8")
    if "50km of operation for a ball type" not in text:
        raise SystemExit("definition of C (50 km for ball type, page 4) not found")
    lines, seal, rows = text.splitlines(), seals(text), []
    for prefix, (suffix, sizes, header, row_cols, kind) in TABLES.items():
        for size in sizes:
            model = f"{prefix}{size}{suffix}"
            v = carriage(lines, model, header, row_cols)
            for c in ["H", "L", "WR", "mrail"] + ROW_TAIL:
                Decimal(v[c])  # stops on a value that is not a number
            rows.append({
                "part_number": model, "series": prefix[:2], "size": str(size), "carriage_type": kind,
                "rail_width_m": si(v["WR"], "0.001"), "height_m": si(v["H"], "0.001"),
                "carriage_length_m": si(v["L"], "0.001"), "dynamic_load_rating_n": si(v["Cdyn"], "1000"),
                "dynamic_load_rating_distance_m": "50000", "static_load_rating_n": si(v["C0"], "1000"),
                "static_moment_x_nm": si(v["MR"], "1000"), "static_moment_y_nm": si(v["MP"], "1000"),
                "static_moment_z_nm": si(v["MY"], "1000"), "carriage_mass_kg": si(v["mblock"]),
                "rail_mass_kg_m": si(v["mrail"]), "seal_drag_n": si(seal[f"{prefix[:2]}{size}"], "2"),
            })
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "linear_guides.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} carriages")


if __name__ == "__main__":
    main()
