"""Extracts HIWIN's preloaded precision ground ballscrews FDW (flange, double nut), FDC (Super S, flange, double nut)
and FSH (high lead, end cap, flange, single nut), and the mounting factors of the critical speed and buckling
formulas, from HIWIN's "Ballscrew Technical Information", FORM S99TE22-2009 (2020-09), into
catalog/hiwin/ball_screws.csv and catalog/hiwin/ball_screw_mountings.csv.

The document is not part of this repository. Get the PDF from HIWIN
(https://hiwin.com/wp-content/uploads/2026/08/HIWIN-Ballscrew-Catalog-1.pdf), then:

    pdftotext -table -enc UTF-8 HIWIN-Ballscrew-Catalog-1.pdf ballscrew.txt      (xpdf 4.06)
    python tools/extract_hiwin_screws.py ballscrew.txt hiwin

FDW rows: the dimension tables "FDW Type Standard Product", pages 65 to 67; every row is
Model, Nominal Dia., Lead, P.C.D., RD, Ball Dia., Circuits, Stiffness K (kgf/um), C (kgf), Co (kgf), D, L, F, T, BCD-E,
X, Y, Z, S. FDC rows: "FDC Type Standard Product", pages 46 and 47: Model ... Co, D, L1, L2 (nut length), TYPE,
Form A, Form B, Form C, L7, D4, D5, M, L10, L11 and up to two dots (double start, incomplete thread). FSH rows:
"FSH Type High Lead", page 87: Model ... Co, D, L (nut length), F, T, BCD-E, H, X, S, M.
Only nominal diameters 12 to 50 mm are taken. A row with another number of values, or a model code that does not
match its diameter and lead, stops the script. A model code a table prints more than once (another ball size, or
another nut flange) gets "-<ball diameter>" and, if still not unique, "-D<nut diameter D>" after the nut type.
Mountings: chapter 4.5 page 32, formulas M29 (Fk = 40720 Nf dr^4 / Lt^2, kgf) and M31 (Nc = 2.71e8 Mf dr / Lt^2,
r/min), with the factors Nf and Mf per mounting. They are written in the schema's form: critical_speed_factor
f = 27.1 Mf (n = f dr / L^2 1e7) and buckling_factor m = 4.072 Nf x 9.80665 (F = m dr^4 / L^2 1e4 in N).
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

KGF = Decimal("9.80665")
MM = Decimal("0.001")
COLUMNS = ["part_number", "series", "nominal_diameter_m", "lead_m", "root_diameter_m", "ball_diameter_m",
           "dynamic_load_rating_n", "static_load_rating_n", "axial_stiffness_n_m", "nut_length_m", "nut_type",
           "max_dn"]
MOUNT_COLUMNS = ["mounting", "critical_speed_factor", "buckling_factor"]
MOUNTINGS = ["fixed-fixed", "fixed-supported", "supported-supported", "fixed-free"]
# nut type -> (fewest and most values of a row, index of the nut length, nut_type, max_dn)
TYPES = {
    "FDW": (19, 19, 11, "FDW: flange end; double nut; return tube within the nut diameter; preloaded",
            "70000"),
    "FDC": (23, 25, 12, "FDC: flange end; double nut; Super S return; preloaded", "220000"),
    "FSH": (19, 19, 11, "FSH: high lead; flange end; single nut; end cap return; preloaded", "70000"),
}
ROW = re.compile(r"^\s*(\d+)-([\d.]+)([A-Z])(\d)\s")
HEADER = re.compile(r"^\s*([A-Z]{3,5})\s+Type\s+(Standard|High)")


def si(text, factor):
    return format((Decimal(text) * factor).normalize(), "f")


def screws(lines):
    rows, section = [], None
    for n, line in enumerate(lines, 1):
        h = HEADER.match(line)
        if h:
            section = h[1]
            continue
        if section not in TYPES or not ROW.match(line):
            continue
        fewest, most, at_length, nut_type, max_dn = TYPES[section]
        t = line.split()
        # FDC rows end in up to two dots (double start / incomplete thread options)
        if not fewest <= len(t) <= most or any(x != "\u25cf" for x in t[fewest:]):
            raise SystemExit(f"line {n}: {len(t)} values, expected {fewest} to {most}: {line.strip()}")
        model, dia, lead, _pcd, rd, ball, _circ, k, c, c0, d = t[:11]
        length = t[at_length]
        m = ROW.match(line)
        if (m[1], m[2]) != (dia, lead):
            raise SystemExit(f"line {n}: model {model} does not match diameter {dia} and lead {lead}")
        for v in t[1:6] + t[7:11] + [length]:
            Decimal(v)  # stops on a value that is not a number
        if not 12 <= int(dia) <= 50:
            continue
        rows.append({"part_number": f"{model}-{section}", "series": section, "nominal_diameter_m": si(dia, MM),
                     "lead_m": si(lead, MM), "root_diameter_m": si(rd, MM), "ball_diameter_m": si(ball, MM),
                     "dynamic_load_rating_n": si(c, KGF), "static_load_rating_n": si(c0, KGF),
                     "axial_stiffness_n_m": si(k, KGF * Decimal("1e6")), "nut_length_m": si(length, MM),
                     "nut_type": nut_type, "max_dn": max_dn, "_ball": ball, "_d": d})
    for series in TYPES:
        if not any(r["series"] == series for r in rows):
            raise SystemExit(f"no {series} table found")
    # A model code printed more than once (other ball size or nut flange) gets the ball diameter, then the nut's D.
    for suffix, mark in (("_ball", "-"), ("_d", "-D")):
        group = {}
        for r in rows:
            group.setdefault(r["part_number"], []).append(r)
        for same in group.values():
            if len({r[suffix] for r in same}) > 1:
                for r in same:
                    r["part_number"] += mark + r[suffix]
    names = [r["part_number"] for r in rows]
    dupes = {x for x in names if names.count(x) > 1}
    if dupes:
        raise SystemExit(f"models printed twice with the same ball and nut diameter: {sorted(dupes)}")
    for r in rows:
        del r["_ball"], r["_d"]
    return rows


def factor(text, name, symbol):
    """The value printed as '<mounting> ... <symbol> = <value>' in the formula block, e.g. 'fixed - free  Nf = 0.0625'."""
    words = name.split("-")
    pat = re.compile(rf"\b{words[0]}\s*-\s*{words[1]}\s+{symbol}\s*=\s*([\d.]+)")
    found = pat.findall(text)
    if len(found) != 1:
        raise SystemExit(f"{symbol} of {name}: found {found}, expected one value")
    return Decimal(found[0])


def mountings(text):
    out = []
    for name in MOUNTINGS:
        mf, nf = factor(text, name, "Mf"), factor(text, name, "Nf")
        out.append({"mounting": name, "critical_speed_factor": format((Decimal("27.1") * mf).normalize(), "f"),
                    "buckling_factor": format((Decimal("4.072") * KGF * nf).normalize(), "f")})
    if "40720" not in text or "2.71" not in text:
        raise SystemExit("formulas M29 / M31 (40720, 2.71e8) not found")
    return out


def write(path, columns, rows):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, columns, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: extract_hiwin_screws.py ballscrew.txt out_dir")
    text = Path(sys.argv[1]).read_text(encoding="utf-8")
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    rows = screws(text.splitlines())
    write(out / "ball_screws.csv", COLUMNS, rows)
    write(out / "ball_screw_mountings.csv", MOUNT_COLUMNS, mountings(text))
    print(f"{len(rows)} screws, {len(MOUNTINGS)} mountings")


if __name__ == "__main__":
    main()
