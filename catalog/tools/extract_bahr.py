"""Extracts the Bahr Modultechnik toothed-belt axes and gantries from the data sheets
"Linearsystem <series>" into catalog/bahr/belt_axes.csv.

The data sheets are not part of this repository. Get the PDFs from Norgren
(https://cdn.norgren.com/pdf/de_21_30_100_ELZI.pdf and so on, one per name in SHEETS), then:

    pdftotext -q -table -enc UTF-8 de_21_30_100_ELZI.pdf        (xpdf 4.x, once per sheet)
    python extract_bahr.py folder_with_the_txt_files ../bahr

Every sheet has the same tables with one column per size. A table that does not have one value
per size stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = 'Bahr Modultechnik GmbH (IMI Norgren), data sheets "Linearsystem <series>", German editions'
# The sheets this script reads without special cases. The other toothed-belt sheets are listed
# with the reason in the notes of the CSV.
SHEETS = ["de_21_30_100_ELZI", "de_21_20_100_ELZU", "de_21_1_100_ELZ", "de_21_1_103_ELZ-W", "de_21_1_106_ELHZ",
          "de_21_1_107_ELSD", "de_21_1_109_ELSZ", "de_21_1_111_ELVZ", "de_21_1_112_ELZD-W", "de_21_1_114_ELZT",
          "de_21_1_115_ELZZ", "de_21_1_131_DLSZ", "de_21_1_132_DLVZ", "de_21_1_133_DLZPVI", "de_21_1_152_LLZS"]
KINDS = {"21.1": "axis", "21.20": "xy_gantry", "21.30": "xz_gantry"}
LOADS = [f"{load}_{state}_{'n' if load[0] == 'f' else 'nm'}" for load in ("fx", "fy", "fz", "mx", "my", "mz")
         for state in ("static", "dynamic")]
HEADER = (["series", "size", "kind", "belt", "belt_pitch_mm", "belt_width_mm", "feed_per_rev_mm", "pulley_teeth",
           "no_load_torque_nm", "no_load_torque_times", "max_speed_m_s", "feed_force_continuous_n", "feed_force_peak_n"]
          + LOADS + ["base_weight_kg", "weight_per_100_mm_kg", "weight_per_100_mm_z_kg", "moved_mass_x_kg",
                     "moved_mass_y_kg", "moved_mass_z_kg", "max_length_one_piece_mm", "max_length_x_mm",
                     "max_length_y_mm", "max_length_z_mm", "sheet", "edition", "source_file"])
NUMBER = re.compile(r"^\d+(,\d+)?$")


def numbers(text, count, what):
    """The `count` values of a table row, with a decimal point."""
    tokens = text.split()
    if len(tokens) != count or not all(NUMBER.match(t) for t in tokens):
        raise ValueError(f"{what}: expected {count} values, found {tokens}")
    return [t.replace(",", ".") for t in tokens]


def sheet(path):
    lines = path.read_text(encoding="utf-8").replace("\f", "\n").splitlines()
    filled = [l for l in lines if l.strip()]
    # "21.30.100   © 2023 Bahr Modultechnik GmbH   11/25" closes every page; the first page counts.
    foot = next((m for l in filled if (m := re.search(r"(\d+\.\d+)\.\d+\s+©.*Bahr.*\s(\d\d/\d\d)\s*$", l))), None)
    if not foot or foot.group(1) not in KINDS:
        raise ValueError("no page footer with a known sheet number")
    number, kind = foot.group(0).split()[0], KINDS[foot.group(1)]

    # The load table names the sizes: "Baugröße   ELZ 30   ELZ 40   ELZ 60 S".
    top = next((i for i, l in enumerate(filled) if "Baugröße" in l and "Fx (N)" in " ".join(filled[i + 1:i + 3])), None)
    if top is None:
        raise ValueError("no load table")
    sizes = [n + ("S" if s else "") for n, s in re.findall(r"\b(\d+)(\s+S\b)?", filled[top].split("Baugröße")[1])]
    # The series is the name the sheet is published under: ELZ and ELZex have the same sizes.
    series = path.stem.split("_", 4)[4]
    rows = {s: dict.fromkeys(HEADER, "") for s in sizes}
    for s, row in rows.items():
        row.update(series=series, size=s, kind=kind, sheet=number, edition=foot.group(2), source_file=path.stem + ".pdf")

    def table_row(name, text, columns):
        for s, *values in zip(sizes, *(iter(numbers(text, len(sizes) * len(columns), name)),) * len(columns)):
            rows[s].update(zip(columns, values))

    # Row label -> columns. The first line with the label counts; the formulas further down repeat some words.
    labels = {r"\(m/s\) max": ["max_speed_m_s"], r"Dauer \(N\)": ["feed_force_continuous_n"],
              r"0,2 s \(N\)": ["feed_force_peak_n"]}
    labels.update({rf"{load[:2].title()} \(Nm?\)": [load, load.replace("static", "dynamic")] for load in LOADS[::2]})
    for label, columns in labels.items():
        text = next((m.group(1) for l in filled[top:] if (m := re.search(label + r"\s(.*)", l))), None)
        if text is not None:
            table_row(label, text, columns)
        elif len(columns) == 1:
            raise ValueError(f"no row {label}")  # a load the sheet does not limit stays empty
    # The no-load torque stands under its heading in a row labelled "Nm". The X/Z gantries print "2 x 0,4".
    at = next(i for i, l in enumerate(filled) if "Leerlaufdrehmoment" in l)
    torque = re.match(r"\s*Nm\s(.*)", filled[at + 1])
    if not torque:
        raise ValueError("no row with the no-load torque")
    times = re.findall(r"(\d) x ", torque.group(1))
    table_row("no-load torque", re.sub(r"\d x ", "", torque.group(1)), ["no_load_torque_nm"])
    if times and (len(times) != len(sizes) or len(set(times)) != 1):
        raise ValueError(f"no-load torque: 'n x' not on every size")
    for row in rows.values():
        row["no_load_torque_times"] = times[0] if times else ""

    # Belt table: "0  4   60 (S)   5M25   130   26", some sheets print "5M 25"; "(S)" means the row is for both 60 and 60S.
    # A second belt table on a later page is for another drive version and is not read.
    at = next((i for i, l in enumerate(filled) if "Zahnriementabelle" in l), None)
    if at is None:
        raise ValueError("no belt table")
    for line in filled[at + 1:]:
        m = re.match(r"\s*\d\s+\d\s+(\d+)( \(S\))?\s+(\d+)M ?(\d+)\s+(\d+)\s+(\d+)\s", line)
        if not m and "Code" not in line and "Nr." not in line:
            break
        for s in ([m.group(1)] + ([m.group(1) + "S"] if m.group(2) else []) if m else []):
            if s not in rows or rows[s]["belt"]:
                raise ValueError(f"belt table: size {s} unknown or twice")
            rows[s].update(zip(HEADER[3:8], [f"{m.group(3)}M{m.group(4)}", m.group(3), str(int(m.group(4))), m.group(5), m.group(6)]))

    axis = []
    for line in filled:
        # Dimension table: "ELZI 30   290  280 ...   5,20 kg   0,32/0,18 kg", the last pair is X/Z.
        m = re.match(r"\s*[A-Za-z]+\s+(\d+(?:\s?S)?)\s.*\s([\d,]+) [kK]g\s+([\d,]+)(?:/([\d,]+))? [kK]g\s*$", line)
        if m and m.group(1).replace(" ", "") in rows:
            # A later page repeats the table for another drive version: the first one counts.
            table = rows[m.group(1).replace(" ", "")]
            for column, value in zip(["base_weight_kg", "weight_per_100_mm_kg", "weight_per_100_mm_z_kg"], m.groups()[1:]):
                table[column] = table[column] or (value or "").replace(",", ".")
            continue
        # Moved masses of the gantries: "30   1,5 kg   0,2 kg" under "Masse X   Masse Y", or one
        # table per axis, "30   2 x UL40   4,5 kg" under "X-Achse". The profile name is not kept.
        axis = re.findall(r"Masse ([XYZ])\b", line) or re.findall(r"\b([XYZ])-Achse\s*$", line) or axis
        m = re.search(r"\s(\d+S?)\s+(?:\S+ [xX] \S+|EL\d+)?\s*([\d,]+) kg(?:\s+([\d,]+) kg)?\s*$", line)
        if m and kind != "axis":
            masses = [g for g in m.groups()[1:] if g]
            if m.group(1) not in rows or len(masses) != len(axis):
                raise ValueError(f"moved mass not understood: {line.strip()}")
            for a, mass in zip(axis, masses):
                column = f"moved_mass_{a.lower()}_kg"
                if rows[m.group(1)][column]:
                    raise ValueError(f"moved mass twice: {line.strip()}")
                rows[m.group(1)][column] = mass.replace(",", ".")

        # "ELZU 60(S): max. Länge und Breite 2000mm"; without a size in front it is for all sizes.
        if "max. Länge" in line:
            m = re.search(r"[A-Z]+ (\d+)(\(S\))?:", line)
            targets = [m.group(1)] + ([m.group(1) + "S"] if m.group(2) else []) if m else sizes
            for pattern, columns in ((r"aus einem Stück ([\d.]+) ?mm", ["max_length_one_piece_mm"]),
                                     (r"in X-Richtung (\d+) ?mm, in Z-Richtung (\d+) ?mm", ["max_length_x_mm", "max_length_z_mm"]),
                                     (r"Länge und Breite ([\d.]+) ?mm()", ["max_length_x_mm", "max_length_y_mm"])):
                if m := re.search(pattern, line):
                    for s in targets:
                        values = [g or m.group(1) for g in m.groups()]  # length and width are one number
                        rows[s].update(zip(columns, (v.replace(".", "") for v in values)))

    required = HEADER[3:9] + ["base_weight_kg", "weight_per_100_mm_kg"] + {
        "axis": [], "xy_gantry": ["moved_mass_x_kg", "moved_mass_y_kg"],
        "xz_gantry": ["moved_mass_x_kg", "moved_mass_z_kg", "weight_per_100_mm_z_kg"]}[kind]
    missing = [f"{series} {s}: {c}" for s, row in rows.items() for c in required if not row[c]]
    if missing:
        raise ValueError("not found: " + ", ".join(missing))
    return [[row[c] for c in HEADER] for row in rows.values()]


def main():
    rows, problems = [], []
    for name in SHEETS:
        try:
            rows += sheet(Path(sys.argv[1]) / (name + ".txt"))
        except (ValueError, OSError) as e:
            problems.append(f"{name}: {e}")
    if problems:
        sys.exit("Bahr sheets not read:\n" + "\n".join(problems))
    notes = f"""# Source: {SOURCE},
#   https://cdn.norgren.com/pdf/<source_file>.
# Extracted by catalog/tools/extract_bahr.py; the numbers are the data sheets', the layout is ours.
# One row per series and size of a linear axis driven by an HTD toothed belt with steel cords.
# series: the name the sheet is published under (ELZ-W is the sheet "ELZ 60 (S) W"); size: as printed, without the blank ("60 S" is 60S).
# kind: axis; xy_gantry (two X axes and a Y axis); xz_gantry (T-gantry). Both gantries have one circulating belt and
#   two motors.
# belt: HTD code, pitch in mm, "M", width in mm; a blank inside ("5M 25") is dropped. feed_per_rev: travel per turn of the pulley.
# no_load_torque: "Leerlaufdrehmoment", added to the drive torque at the pulley in the sheets' formula. no_load_torque_times: 2 where the sheet prints "2 x 0,4", the X/Z gantry's value
#   for horizontal travel; whether that is per motor is not said. The XY gantry prints one number.
# max_speed: "max"; the sheets give no conditions.
# feed_force: belt pull, continuous and for 0.2 s.
# fx ... mz: permissible loads on the slide, static and dynamic. The sheets want the sum of
#   Fy/Fy_dyn + Fz/Fz_dyn + Mx/Mx_dyn + My/My_dyn + Mz/Mz_dyn to stay at or below 1. The X/Z gantry has no Fy.
# base_weight: at the base length, without travel. weight_per_100_mm: per 100 mm of travel; the X/Z gantry prints
#   two values, X then Z (weight_per_100_mm_z), the XY gantry only one.
# moved_mass: the "bewegte Masse" the gantry sheets give per axis.
# max_length_one_piece: longer axes are joined from pieces. max_length_x/y/z: of the gantries; the XY gantry
#   prints one number for "length and width". Empty where the sheet does not state it in one of these forms
#   (ELSD: "max. Länge 2.000 mm"; DLZPVI: one length per size).
# sheet and edition: from the footer of the first page, "21.30.100 ... 11/25".
# As printed, and not smoothed over:
#   The continuous feed force is the static Fx, except ELZ, ELZ-W, ELSD, ELSZ, ELZD-W, ELZU 60 and 60S and
#   DLVZ and DLZPVI 120 (900 N, Fx 894 N), ELZU 100 (3600 N, Fx 4000 N) and LLZS 60 (1050 N, Fx 1073 N).
#   ELZT 80S has a lower no-load torque (1.2) than ELZT 80 (1.3).
#   ELZZ prints the belt width with a leading zero ("5M09").
#   The footer of the DLZPVI sheet (file de_21_1_133) carries the number of the DSZPVI sheet, 21.1.142; page 2 of
#   the ELZU sheet says 21.20.200.
# DLZPVI has a planetary gear unit built in; its ratio is not extracted and the values are the sheet's as they stand.
# Not extracted: second moments of area, dimensions, base lengths, slide lengths, journal and coupling sizes, order
#   codes, profile names of the gantries, and the second drive version of ELZ and ELVZ on a later page
#   (slide movable to the right and left, with its own weights and belt table).
# Sheets not read, because their tables need special cases: no-load torque with and without cover band (DLZ, DLZS,
#   DLZZ, DSZ, DSZPVI, DSZS, DSZZ, DLZU, DSZU); no feed force rows (ELFZ, ELFZex, DLZS-P, DSZS-P, LLZ, LLN, LSZ,
#   LSN); belt table with a second, effective belt width or none (MLZ, MLN, MLZ-W, MLZD-W, QLZ); belt table names
#   size 60 for the 60S (ELZI-W, ELZU-W); load or torque table laid out differently (ELZex, ELZG, DSSZ, QSZ, QSZT).
#   The rack-and-pinion axes (21.3.x) are no belt axes.
"""
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    (out / "belt_axes.csv").write_text(notes + ",".join(HEADER) + "\n" + "".join(",".join(r) + "\n" for r in rows),
                                       encoding="utf-8", newline="\n")
    print(f"{len(rows)} sizes from {len(SHEETS)} sheets")


if __name__ == "__main__":
    main()
