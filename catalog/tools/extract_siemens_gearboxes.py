"""Extracts the coaxial planetary gearboxes NRB, NRK and NLC from the Siemens equipment manual
"Planetary gearboxes for 1FK2 and 1FT2 servomotors" (04/2025, A5E54515894B AA) into one CSV per series.

The manual is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 manual.pdf manual.txt      (xpdf 4.x)
    python extract_siemens_gearboxes.py manual.txt ../siemens

Every row of a table must have one value per column; a row that does not is reported and the
script stops, so a changed edition of the manual cannot produce shifted values silently.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, Planetary gearboxes for 1FK2 and 1FT2 servomotors, Equipment Manual, 04/2025, A5E54515894B AA"

# Columns of the tables that depend on the motor: (gearbox size, motor frame sizes sharing the column).
FRAMES = {
    "NRB": [("NRB040", ["02"]), ("NRB040", ["03"]), ("NRB060", ["02", "03"]), ("NRB060", ["04"]), ("NRB060", ["05"]),
            ("NRB080", ["03", "04", "05"]), ("NRB080", ["06"]), ("NRB120", ["03", "04", "05", "06"]), ("NRB120", ["08"]),
            ("NRB160", ["05", "06", "08"])],
    "NRK": [("NRK050", ["02"]), ("NRK050", ["03"]), ("NRK070", ["02", "03"]), ("NRK070", ["04"]), ("NRK070", ["05"]),
            ("NRK090", ["03", "04", "05"]), ("NRK090", ["06"]), ("NRK120", ["03", "04", "05", "06"]), ("NRK120", ["08"]),
            ("NRK155", ["05", "06", "08"]), ("NRK155", ["10"])],
    "NLC": [("NLC060", ["02", "03"]), ("NLC060", ["04"]), ("NLC060", ["05"]), ("NLC080", ["03", "04", "05"]),
            ("NLC080", ["06"]), ("NLC120", ["03", "04", "05", "06"]), ("NLC120", ["08"])],
}
# Table numbers in chapter 6: rated, maximum and EMERGENCY OFF torque, backlash (per size);
# efficiency, torsional stiffness, moment of inertia (per size and motor frame).
TABLES = {
    "NRB": {"rated": 1, "max": 2, "emergency": 3, "backlash": 8, "efficiency": 4, "stiffness": 6, "inertia": 10},
    "NRK": {"rated": 19, "max": 21, "emergency": 23, "backlash": 29, "efficiency": 25, "stiffness": 27, "inertia": 31},
    "NLC": {"rated": 40, "max": 42, "emergency": 44, "backlash": 50, "efficiency": 46, "stiffness": 48, "inertia": 52},
}
# Table 6-4 (NRB efficiency) prints one value across several motor frame columns where they
# share it. How many values each gearbox size has per ratio, read from the page layout
# (NRB040, NRB060, NRB080, NRB120, NRB160). Two values for the three NRB060 columns mean: the
# first covers frames 02 to 04, the second frame 05.
SPANS = {4: {3: (1, 1, 1, 1, 1), 4: (1, 1, 1, 1, 1), 5: (2, 1, 1, 1, 1), 7: (2, 2, 1, 1, 1), 8: (2, 2, 2, 1, 1),
             **{i: (2, 2, 2, 2, 1) for i in (10, 9, 20, 32, 40, 60, 512)}, **{i: (2, 2, 2, 1, 1) for i in (12, 15, 16, 25)},
             **{i: (2, 3, 2, 2, 1) for i in (64, 80, 100, 120, 160, 200, 256, 320)}}}

VALUE = re.compile(r"^(-|\d+(\.\d+)?(/\d+(\.\d+)?)?)$")


def spread(values, counts, widths):
    """One value per column from the values a row prints for each gearbox size."""
    out = []
    for count, width in zip(counts, widths):
        part, values = values[:count], values[count:]
        if count == width:
            out += part
        elif count == 1:
            out += part * width
        elif (count, width) == (2, 3):
            out += [part[0], part[0], part[1]]
        else:
            raise ValueError(f"{count} values for {width} columns")
    return out


def table_rows(lines, number, columns, widths):
    """{ratio: (stages, [value per column])} of table 6-<number>; where a cell reads
    "coaxial / angled", the coaxial value."""
    spans = SPANS.get(number, {})
    start = next(i for i, l in enumerate(lines) if re.match(rf"\s*Table\s+6-{number}\s", l))
    rows, stage, bad = {}, None, []
    for line in lines[start + 1:]:
        if re.match(r"\s*Table\s+\d+-\d+\s", line) or re.match(r"\s*6\.\d+\.\d+\s", line):
            break
        tokens = re.sub(r"(\d)\s*/\s*(\d)", r"\1/\2", line.replace("–", "-").replace("‑", "-")).split()
        if len(tokens) < 2 or not all(VALUE.match(t) for t in tokens):
            continue
        # A row starts with the ratio, or with the number of stages and the ratio.
        for lead in (1, 2):
            ratio = int(tokens[lead - 1]) if tokens[lead - 1].isdigit() else None
            counts = spans.get(ratio)
            if spans and not counts:
                continue  # in a table with spans, every ratio has one
            if ratio is not None and len(tokens) - lead == (sum(counts) if counts else columns):
                stage = int(tokens[0]) if lead == 2 else stage
                values = [t.split("/")[0] for t in tokens[lead:]]
                rows[ratio] = (stage, spread(values, counts, widths) if counts else values)
                break
        else:
            bad.append(line.rstrip())
    return rows, bad


def extract(series, lines):
    frames, tables = FRAMES[series], TABLES[series]
    sizes = list(dict.fromkeys(size for size, _ in frames))
    widths = [sum(1 for size, _ in frames if size == s) for s in sizes]
    data, problems = {}, []
    for name, number in tables.items():
        per_frame = name in ("efficiency", "stiffness", "inertia")
        data[name], bad = table_rows(lines, number, len(frames) if per_frame else len(sizes), widths)
        problems += [f"table 6-{number} ({series} {name}): {b}" for b in bad]
    if problems:
        return [], problems
    out = []
    for ratio, (stage, stiffness) in data["stiffness"].items():
        # The backlash table has one row per number of stages, keyed by the stage's lowest ratio.
        backlash = next(v for st, v in data["backlash"].values() if st == stage)
        for column, (size, motor_frames) in enumerate(frames):
            at_size = sizes.index(size)
            row = [data["rated"][ratio][1][at_size], data["max"][ratio][1][at_size], data["emergency"][ratio][1][at_size],
                   data["efficiency"][ratio][1][column], stiffness[column], backlash[at_size], data["inertia"][ratio][1][column]]
            if stiffness[column] == "-":
                # This motor does not exist with this gearbox and ratio; the other tables must agree.
                if data["efficiency"][ratio][1][column] != "-" or data["inertia"][ratio][1][column] != "-":
                    problems.append(f"{size} frames {motor_frames} i={ratio}: no stiffness, but other values")
                continue
            if "-" in row[:5]:
                problems.append(f"{size} frames {motor_frames} i={ratio}: a value is missing: {row}")
            # The manual leaves two gaps: no backlash for the two-stage NRB160, no inertia for NRK155 with i = 50.
            row[5:] = ["" if v == "-" else v for v in row[5:]]
            out += [[series, size, frame, stage, ratio] + row for frame in motor_frames]
    return sorted(out, key=lambda r: (r[1], r[2], r[3], r[4])), problems


HEADER = ["series", "size", "motor_frame", "stages", "ratio", "rated_torque_nm", "max_torque_nm", "emergency_off_torque_nm",
          "efficiency_full_load", "torsional_stiffness_nm_arcmin", "backlash_arcmin", "inertia_at_input_kgcm2"]

NOTES = """# Source: {source}, chapter 6.
# Extracted by catalog/tools/extract_siemens_gearboxes.py; the numbers are the manual's, the layout is ours.
# One row per gearbox size, motor frame size and ratio that the manual lists as available.
# motor_frame: the last two digits of the motor's frame size in the article number 1F.2.xx
#   (frame 10 is printed in the manual as 1F.2210).
# Torques are at the output shaft. rated: continuous; max: for at most 30 000 output revolutions;
#   emergency_off: at most 1000 times in the gearbox's life.
# torsional_stiffness and backlash are referred to the output; inertia is referred to the input (motor side).
# An empty field: the manual gives no value there.
# Not extracted: mass, radial and axial forces, reference speeds, bending moments, input speed limits,
#   the angled versions (NRBW, NRKW, NLCW) and the precision gearboxes SP+ and TP+.
"""


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    out_dir = Path(sys.argv[2])
    results, problems = {}, []
    for series in FRAMES:
        results[series], p = extract(series, lines)
        problems += p
    if problems:
        sys.exit("rows without one value per column:\n" + "\n".join(problems))
    out_dir.mkdir(parents=True, exist_ok=True)
    for series, rows in results.items():
        text = NOTES.format(source=SOURCE) + ",".join(HEADER) + "\n" + "".join(",".join(map(str, r)) + "\n" for r in rows)
        (out_dir / f"gearbox_{series.lower()}.csv").write_text(text, encoding="utf-8", newline="\n")
        print(f"{series}: {len(rows)} rows")


if __name__ == "__main__":
    main()
