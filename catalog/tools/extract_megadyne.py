"""Extracts open-ended steel-cord timing belts from the Megadyne MEGALINEAR technical handbook
(02.2026) into catalog/megadyne/belts.csv and catalog/megadyne/belt_tooth_load.csv.

The handbook is not part of this repository. Get the PDF from Megadyne
(https://megadynegroup.com/files/resources/attachments/md_manu_en_megalinear_web.pdf), then:

    pdftotext -table -enc UTF-8 md_manu_en_megalinear_web.pdf megalinear.txt      (xpdf 4.x)
    python tools/extract_megadyne.py megalinear.txt megadyne

It writes the schema-1 tables (SI units) directly. Each profile's first page has a row of standard widths, a row of
weights, the standard thickness, a tooth resistance table over r/min and a traction table per cord material; its
second page has the "FLEXION RESISTANCE" table. Only the standard cords are read (the first traction block and the
first flexion row: "Standard steel cords", for MTD8 "HP = Standard"). A row that does not have one value per width
or per speed stops the script.
"""
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

# Profile, pitch in mm. MTD is Megadyne's HTD-compatible profile (Bahr's 3M, 5M, 8M).
PROFILES = [("T5", 5), ("AT3", 3), ("AT5", 5), ("AT10", 10), ("MTD3", 3), ("MTD5", 5), ("MTD8", 8)]
SPEEDS = [0, 20, 40, 60, 80, 100, 200, 300, 400, 500, 750, 1000, 1500, 2000, 3000, 4000, 5000, 8000]


def values(line, label):
    """The numbers after `label` in a line, with a decimal point."""
    rest = line.split(label, 1)[1].split()
    return [t.replace(",", ".") for t in rest]


def section(lines, profile):
    """The lines of a profile's first page, from its heading to the next "Average values"."""
    heading = re.compile(rf"^\s*{profile} OPEN-ENDED\s*$")
    start = next(i for i, line in enumerate(lines) if heading.match(line))
    end = next(i for i in range(start, len(lines)) if "Average values" in lines[i])
    return lines[start:end]


def flexion(lines, profile):
    """Z min of the standard cords in a linear drive (first column of the flexion table on the profile's second page)."""
    heading = re.compile(rf"^\s*{profile} OPEN-ENDED\s*$")
    pages = [i for i, line in enumerate(lines) if heading.match(line)]
    if len(pages) < 2:
        raise ValueError(f"{profile}: no flexion page")
    head = next(i for i in range(pages[1], len(lines)) if "ZMIN" in lines[i])
    row = next(line for line in lines[head + 1:] if line.strip())
    m = re.match(r"^\s*(Standard\s+steel cords|HP = Standard)\s+(\d+)\s+\d+\s+\d+\s+\d+\s+\d+\s*$", row)
    if m is None:
        raise ValueError(f"{profile}: flexion row {row.strip()!r}")
    return m.group(2)


def profile_rows(lines, profile, pitch):
    part = section(lines, profile)
    thickness = re.search(r"Standard\s+thickness\s+([\d,]+) \+", find_in(part, "thickness")).group(1).replace(",", ".")
    z_min = flexion(lines, profile)
    find = lambda label: find_in(part, label)
    # "STANDARD 6 10 ..." with "WIDTHS (mm)" on the next line, or "STANDARD WIDTHS (mm) 10 20 ...".
    widths = values(find("STANDARD").replace("WIDTHS (mm)", ""), "STANDARD")
    weights = values(find("Weight (gr/m)"), "Weight (gr/m)")
    rpm = values(find("RPM (1/min)"), "RPM (1/min)")
    tooth = values(find("Fp spec (N/cm)"), "Fp spec (N/cm)")
    cords = values(find("BELT WIDTH (mm)"), "BELT WIDTH (mm)")
    traction = values(find("Max Traction Load (N)"), "Max Traction Load (N)")
    breaking = values(find("Breaking Strength (N)"), "Breaking Strength (N)")
    elongation = values(find("Elongation at MTL (mm/m)"), "Elongation at MTL (mm/m)")
    if [float(r) for r in rpm] != SPEEDS or len(tooth) != len(SPEEDS):
        raise ValueError(f"{profile}: tooth resistance table {rpm} {tooth}")
    for name, row in [("weights", weights), ("traction widths", cords), ("traction", traction), ("breaking", breaking), ("elongation", elongation)]:
        if len(row) != len(widths):
            raise ValueError(f"{profile}: {name} {row} for widths {widths}")
    if cords != widths:
        raise ValueError(f"{profile}: traction table widths {cords} differ from {widths}")
    belts = [{"profile": profile, "pitch_m": to_si(str(pitch), "milli"), "width_m": to_si(w, "milli"),
              "mass_kg_m": to_si(g, "milli"), "max_pull_n": t, "breaking_strength_n": b,
              "elongation_at_max_pull": to_si(e, "milli"), "min_pulley_teeth": z_min,
              "thickness_m": to_si(thickness, "milli")}
             for w, g, t, b, e in zip(widths, weights, traction, breaking, elongation)]
    teeth = [{"profile": profile, "speed_rad_s": to_si(str(s), "rpm"), "specific_tooth_load_n_m": to_si(f, "centi_inverse")}
             for s, f in zip(SPEEDS, tooth)]
    return belts, teeth


def find_in(part, label):
    line = next((line for line in part if label in line), None)
    if line is None:
        raise ValueError(f"no line {label!r}")
    return line


def write(path, rows):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def main(text_file, out_dir):
    lines = Path(text_file).read_text(encoding="utf-8").splitlines()
    belts, teeth = [], []
    for profile, pitch in PROFILES:
        b, t = profile_rows(lines, profile, pitch)
        belts += b
        teeth += t
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    write(out / "belts.csv", belts)
    write(out / "belt_tooth_load.csv", teeth)
    print(f"{len(belts)} belts, {len(teeth)} tooth load points")


if __name__ == "__main__":
    main(*sys.argv[1:3])
