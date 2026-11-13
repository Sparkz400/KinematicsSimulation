"""Extracts open-ended steel-cord timing belts from the Megadyne MEGALINEAR technical handbook
(02.2026) into catalog/megadyne/belts.csv and catalog/megadyne/belt_tooth_load.csv.

The handbook is not part of this repository. Get the PDF from Megadyne
(https://megadynegroup.com/files/resources/attachments/md_manu_en_megalinear_web.pdf), then:

    pdftotext -table -enc UTF-8 md_manu_en_megalinear_web.pdf megalinear.txt      (xpdf 4.x)
    python extract_megadyne.py megalinear.txt ../megadyne

Each profile's page has a row of standard widths, a row of weights, a tooth resistance table over
r/min and a traction table per cord material. Only the standard steel cords are read (the first
traction block). A row that does not have one value per width or per speed stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Megadyne (Ammega), MEGALINEAR technical handbook, 02.2026"
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


def profile_rows(lines, profile, pitch):
    part = section(lines, profile)
    find = lambda label: next(line for line in part if label in line)
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
    belts = [[profile, str(pitch), w, g, t, b, e] for w, g, t, b, e in zip(widths, weights, traction, breaking, elongation)]
    return belts, [[profile, str(s), f] for s, f in zip(SPEEDS, tooth)]


def main(text_file, out_dir):
    lines = Path(text_file).read_text(encoding="utf-8").splitlines()
    belts, teeth = [], []
    for profile, pitch in PROFILES:
        b, t = profile_rows(lines, profile, pitch)
        belts += b
        teeth += t
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    head = f"""# Source: {SOURCE} (megadynegroup.com, md_manu_en_megalinear_web.pdf), the page of each profile.
# Extracted by catalog/tools/extract_megadyne.py; the numbers are the handbook's, the layout is ours.
# Open-ended polyurethane belts with standard steel cords (S and Z torsion zinked steel). MTD is Megadyne's
# HTD-compatible profile: MTD3, MTD5, MTD8 fit Bahr's 3M, 5M, 8M belts.
"""
    (out / "belts.csv").write_text(head + """# One row per profile and standard width. Average values, as the handbook says.
# mass: "Weight (gr/m)". max_traction_load: the allowable pull of the cords, a quarter of the breaking strength
#   (the handbook's figures); elongation_at_mtl: the stretch at that pull, so EA = max_traction_load / (elongation / 1000).
# Not extracted: the other cord materials (Kevlar, HP, HF, HPF, stainless), the flexion table (minimum pulley teeth),
#   pulley diameters, thickness and tolerances.
profile,pitch_mm,width_mm,mass_g_m,max_traction_load_n,breaking_strength_n,elongation_at_mtl_mm_m
""" + "".join(",".join(r) + "\n" for r in belts), encoding="utf-8", newline="\n")
    (out / "belt_tooth_load.csv").write_text(head + """# The "tooth resistance" table: specific tooth load Fp spec in N per cm of width and per tooth in mesh, over the
#   pulley's r/min. The handbook sizes by effective pull * service factor <= Fp spec * width in cm * teeth in mesh
#   (at most 12 for open-ended belts), page 9.
profile,rpm,specific_tooth_load_n_cm
""" + "".join(",".join(r) + "\n" for r in teeth), encoding="utf-8", newline="\n")
    print(f"{len(belts)} belts, {len(teeth)} tooth load points")


if __name__ == "__main__":
    main(*sys.argv[1:3])
