"""Extracts the SIMOTICS S-1FK7 and S-1FT7 servomotors for SINAMICS S120 from the Siemens catalogue
D 21.4, 2017 (the part "SIMOTICS servomotors") into catalog/siemens/motors_s120.csv.

The catalogue is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 catalogue.pdf catalogue.txt      (xpdf 4.x)
    python extract_siemens_d21_4.py catalogue.txt ../siemens

Each motor has a row in a selection table (ratings, pole pairs, inertia, weight) and a row on
the facing page (efficiency, stall current, Motor Module). A row that cannot be read, a motor
that is printed twice with different values, or a motor with only one of its two rows stops the
script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, catalogue D 21.4, SINAMICS S120 and SIMOTICS, 2017"
HEADER = ["article", "series", "version", "cooling", "shaft_height_mm", "rated_speed_rpm", "rated_values_at_rpm",
          "rated_power_kw", "static_torque_nm", "rated_torque_nm", "rated_current_a", "pole_pairs", "rotor_inertia_kgcm2",
          "mass_kg", "efficiency_percent", "stall_current_a", "calculated_power_kw", "motor_module_rated_current_a", "motor_module"]

# The article number up to the winding; the DC link code 7 after it is sometimes printed apart.
STEM = r"(1F[KT]7\d{3}-\d[A-Z]{2})"
SECTION = re.compile(r"SIMOTICS S-(1F[KT]7) (Compact|High Dynamic|High Inertia)\b.*?(Natural\s+cooling|Forced\s+ventilation|Water\s+cooling)")
NUMBER = re.compile(r"^\d+(\.\d+)?$")
IMPERIAL = re.compile(r"^(?=.*[()])[()\d.,]*$")


def reduced_speed(lines, at, marks):
    """The speed a footnote gives for the rated values of the row in line `at`, or ""."""
    for line in lines[at:at + 200]:
        found = re.match(r"\s*(\d\))\s+These values refer to n = (\d+) rpm", line)
        if found and found.group(1) in marks:
            return found.group(2)
    return ""


def motors(lines):
    selection, facing, problems, section = {}, {}, [], None
    speed = height = None
    for number, line in enumerate(lines):
        found = SECTION.search(line)
        if found:
            section = (found.group(1), found.group(2), " ".join(found.group(3).lower().split()))
        # A few article numbers are printed with a gap before the speed letter: "1FT7065-7S F 77".
        line = re.sub(r"(1F[KT]7\d{3}-\d[A-Z])\s+([A-Z])(?=\s|7)", r"\1\2", line)
        raw = line.split()
        tokens = [t for t in raw if not IMPERIAL.match(t)]
        at = next((i for i, t in enumerate(tokens) if re.match("^" + STEM + "(7|$)", t)), None)
        if at is None or not section:
            continue
        stem = re.match(STEM, tokens[at]).group(1) + "7"
        if at == 0 and re.match("^" + STEM + r"7(\d-)?\.\.\.$", tokens[0]):
            # Facing page: efficiency, stall current, calculated power, Motor Module current and article.
            # The module's article number is sometimes printed in parts.
            module = ""
            for i, t in enumerate(tokens):
                if re.match(r"^6SL31\d\d-", t):
                    module = t
                    while not module.endswith(".") and i + 1 < len(tokens):
                        i += 1
                        module += tokens[i]
                    break
            values = []
            for t in tokens[1:]:
                if not NUMBER.match(t):
                    break
                values.append(t)
            if len(values) != 4 or not re.match(r"^6SL31\d\d-\dTE\d\d-\d[A-Z]{2}\.$", module):
                problems.append(" ".join(tokens))
                continue
            row = values + [module]
            if facing.setdefault(stem, row) != row:
                problems.append(f"{stem} twice on a facing page: {facing[stem]} and {row}")
            continue
        before, after = tokens[:at], [t for t in tokens[at + 1:] if NUMBER.match(t)]
        if not before or not all(NUMBER.match(t) for t in before) or len(after) < 3:
            continue  # the article number in running text or in a table of options
        # The rated speed and the shaft height are printed once for the rows they cover.
        if len(before) == 6:
            speed, height = before[0], before[1]
        elif len(before) == 5:
            height = before[0]
        elif len(before) != 4:
            problems.append(" ".join(tokens))
            continue
        # A footnote mark behind the rated values: they hold at a lower speed than the rated one.
        marks = {t for t in raw[:raw.index(tokens[at])] if re.match(r"^\d\)$", t)}
        power, static, rated, current = before[-4:]
        row = list(section) + [height, speed, reduced_speed(lines, number, marks), power, static, rated, current] + after[-3:]
        if selection.setdefault(stem, row) != row:
            problems.append(f"{stem} twice in the selection tables: {selection[stem]} and {row}")
    out = []
    for stem, row in sorted(selection.items()):
        if stem not in facing:
            problems.append(f"{stem}: no row on a facing page")
            continue
        out.append([stem] + row + facing[stem])
    problems += [f"{stem}: on a facing page, but in no selection table" for stem in sorted(set(facing) - set(selection))]
    return out, problems


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    rows, problems = motors(lines)
    if problems or not rows:
        sys.exit("motor tables not read:\n" + "\n".join(problems))
    notes = f"""# Source: {SOURCE}, selection and ordering data of the SIMOTICS S synchronous motors.
# Extracted by catalog/tools/extract_siemens_d21_4.py; the numbers are the catalogue's, the layout is ours.
# One row per motor winding: the article number up to the DC link code, e.g. 1FK7042-2AF7. All for a
# DC link of 510 ... 720 V (line 380 ... 480 V 3 AC).
# Ratings are for a winding temperature rise of 100 K. Currents are RMS. static and stall: at standstill.
# rated_values_at_rpm: where the catalogue marks rated power, torque and current with a footnote
#   "These values refer to n = ... rpm", that speed; empty where they hold at the rated speed.
# calculated_power: static torque times rated speed, the catalogue's figure for choosing the Motor Module.
# motor_module: the booksize Motor Module the catalogue pairs with the motor; the "7" and the "." in its
#   article number are placeholders (7: 1 for a Single, 2 for a Double Motor Module).
# Not in these tables, and so not here: maximum torque, current and speed (given as characteristic curves),
#   torque and voltage constant, winding resistance and inductance, thermal time constant.
"""
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    (out / "motors_s120.csv").write_text(notes + ",".join(HEADER) + "\n" + "".join(",".join(r) + "\n" for r in rows),
                                        encoding="utf-8", newline="\n")
    print(f"{len(rows)} motors")


if __name__ == "__main__":
    main()
