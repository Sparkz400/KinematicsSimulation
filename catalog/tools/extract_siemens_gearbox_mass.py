"""Extracts the mass of the coaxial planetary gearboxes NRB, NRK and NLC from the Siemens configuration
manual "Planetary gearboxes for S-1FK2 and S-1FT2 servomotors" (01/2023, A5E51318133B AC), Tables 7-12,
7-33 and 7-54, into catalog/siemens/gearbox_mass.csv.

The manual is not part of this repository. Get the PDF from Siemens
(https://cache.industry.siemens.com/dl/files/596/109802596/att_1132680/v2/Plan_gear_for_servo_motors_config_man_0123_en-US.pdf),
then:

    pdftotext -table -enc UTF-8 manual.pdf manual.txt      (xpdf 4.x)
    python extract_siemens_gearbox_mass.py manual.txt ../siemens

A table row is: the number of stages where a stage begins, the ratio, one mass per column. Only the last
column may be left out (the NRB160 is not offered with every ratio); any other count stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, Planetary gearboxes for S-1FK2 and S-1FT2 servomotors, Configuration Manual, 01/2023, A5E51318133B AC"
# The table, and its columns: gearbox size and the motor frame sizes that share the column.
TABLES = {
    "NRB": ("Table 7-12", [("NRB040", "02"), ("NRB040", "03"), ("NRB060", "02 03 04"), ("NRB060", "05"), ("NRB080", "03 04 05"),
                           ("NRB080", "06"), ("NRB120", "03 04 05 06"), ("NRB120", "08"), ("NRB160", "05 06 08")]),
    "NRK": ("Table 7-33", [("NRK050", "02"), ("NRK050", "03"), ("NRK070", "02 03 04"), ("NRK070", "05"), ("NRK090", "03 04 05"),
                           ("NRK090", "06"), ("NRK120", "03 04 05 06"), ("NRK120", "08"), ("NRK155", "05 06 08"), ("NRK155", "10")]),
    "NLC": ("Table 7-54", [("NLC060", "02 03 04"), ("NLC060", "05"), ("NLC080", "03 04 05"), ("NLC080", "06"),
                           ("NLC120", "03 04 05 06"), ("NLC120", "08")]),
}
VALUE = re.compile(r"^(-|\d+(\.\d+)?)$")


def table_rows(lines, series):
    title, columns = TABLES[series]
    start = next(i for i, line in enumerate(lines) if line.strip().startswith(title))
    end = next(i for i in range(start, len(lines)) if lines[i].strip().startswith("z = number") or lines[i].strip().startswith("z  =  number"))
    rows, stage = [], 0
    for line in lines[start + 1:end]:
        tokens = line.split()
        if not tokens or not all(VALUE.match(t) for t in tokens):
            continue
        # A row that begins a stage starts with its number (1, 2, 3), which no ratio is.
        if tokens[0] == str(stage + 1) and len(tokens) >= len(columns):
            stage, tokens = stage + 1, tokens[1:]
        ratio, masses = tokens[0], tokens[1:]
        if len(masses) == len(columns) - 1:
            masses.append("-")
        if len(masses) != len(columns):
            raise ValueError(f"{series}, ratio {ratio}: {len(masses)} masses for {len(columns)} columns: {line.strip()}")
        for (size, frames), mass in zip(columns, masses):
            if mass != "-":
                rows.append([series, size, frames, str(stage), ratio, mass])
    if stage not in (2, 3):
        raise ValueError(f"{series}: {stage} stages read")
    return rows


def main(text_file, out_dir):
    lines = Path(text_file).read_text(encoding="utf-8").splitlines()
    rows = [row for series in TABLES for row in table_rows(lines, series)]
    head = f"""# Source: {SOURCE}, Tables 7-12 (NRB), 7-33 (NRK) and 7-54 (NLC), "Mass mG of ... coaxial planetary gearboxes".
# Extracted by catalog/tools/extract_siemens_gearbox_mass.py; the numbers are the manual's, the layout is ours.
# One row per gearbox size, group of motor frame sizes and ratio. motor_frames: the last two digits of the motor's
#   frame size (1F.2.xx), separated by blanks, that share the column. mass: of the gearbox alone, in kg.
# This edition is older than the equipment manual 04/2025 the other gearbox files come from; the sizes and ratios
#   agree, and catalog tests check that every gearbox of those files has a mass here.
# A ratio the manual marks "-" for a size is left out.
series,size,motor_frames,stages,ratio,mass_kg
"""
    out = Path(out_dir) / "gearbox_mass.csv"
    out.write_text(head + "".join(",".join(r) + "\n" for r in rows), encoding="utf-8", newline="\n")
    print(f"{len(rows)} rows")


if __name__ == "__main__":
    main(*sys.argv[1:3])
