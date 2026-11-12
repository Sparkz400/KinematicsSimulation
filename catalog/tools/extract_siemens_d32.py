"""Extracts the SIMOTICS S-1FK2 and S-1FT2 servomotors and their SINAMICS S210 converters from the
Siemens catalogue D 32, January 2024, into catalog/siemens/motors_s210.csv.

The catalogue is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 catalogue.pdf catalogue.txt      (xpdf 4.x)
    python extract_siemens_d32.py catalogue.txt ../siemens

The motor data sits in the "Specific technical specifications" tables, one column per motor.
A row that does not have one value per motor stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, catalogue D 32, SINAMICS S210 servo drive system, January 2024"

# Row label -> column name. Longer labels first where one starts like another.
ROWS = [("Static torque", "static_torque_nm"), ("Stall current", "stall_current_a"), ("Maximum torque", "max_torque_nm"),
        ("Maximum current", "max_current_a"), ("Maximum speed", "max_speed_rpm"),
        ("Rotor moment of inertia", "rotor_inertia_kgcm2"), ("Moment of inertia (with brake)", "rotor_inertia_with_brake_kgcm2"),
        ("Weight (with brake)", "mass_with_brake_kg"), ("Weight", "mass_kg")]
RATED = [("Rated speed", "rated_speed_{}_rpm"), ("Rated torque", "rated_torque_{}_nm"), ("Rated current", "rated_current_{}_a"),
         ("Rated power", "rated_power_{}_w")]
SUPPLIES = {"200 ... 240 V 1 AC": "230v_1ac", "200 ... 240 V 3 AC": "230v_3ac", "380 ... 480 V 3 AC": "400v_3ac"}
HEADER = (["article", "series", "version", "shaft_height_mm", "cooling"] + [c for _, c in ROWS]
          + [c.format(v) for v in ("230v", "400v") for _, c in RATED]
          + [f"converter_{s}" for s in SUPPLIES.values()] + [f"converter_new_{s}" for s in SUPPLIES.values()])

NUMBER = re.compile(r"^(–|\d+(\.\d+)?)$")
MOTOR = re.compile(r"^1F[KT]2\d{3}-(\d[A-Z]{2})?$")
CONVERTER = re.compile(r"^6SL\d{4}-(\w{5}-\w{4})?$")


def joined(tokens, more, pattern):
    """Article numbers of a table row; where the row prints only their first part, the parts in
    the next rows complete them in order."""
    out = ["" if t == "–" else t for t in tokens if t == "–" or pattern.match(t)]
    for row in more:
        if not any(a.endswith("-") for a in out):
            break
        parts = iter(row)
        out = [a + next(parts) if a.endswith("-") else a for a in out]
    return out


def blocks(lines):
    """The lines of each "Specific technical specifications" table."""
    starts = [i for i, l in enumerate(lines) if "Specific technical specifications" in l]
    for start in starts:
        end = next(i for i in range(start + 1, len(lines)) if "Siemens D 32" in lines[i])
        # Drop the imperial values in brackets (the catalogue has a few stray brackets there too)
        # and read the decimal commas that some tables use.
        imperial = re.compile(r"^(?=.*[()])[()\d.,]*$")
        yield [[re.sub(r"^(\d+),(\d+)$", r"\1.\2", t) for t in l.replace("‑", "-").split() if not imperial.match(t)]
               for l in lines[start + 1:end] if l.strip()]


def motors_of(block):
    problems, motors, supply, new = [], [], None, False
    for n, tokens in enumerate(block):
        text, after = " ".join(tokens), block[n + 1] if n + 1 < len(block) else []
        if text.startswith("Shaft height") and not motors:
            articles = joined(tokens[3:], block[n + 1:n + 3], MOTOR)
            if not articles or not all(re.match(r"^1F[KT]2\d{3}-\d[A-Z]{2}$", a) for a in articles):
                return [], []  # not a 1FK2 or 1FT2 table
            cooling = "forced ventilation" if "forced ventilation" in text else "natural"
            motors = [{"article": a, "shaft_height_mm": tokens[2], "cooling": cooling} for a in articles]
            continue
        if not motors:
            continue
        if text.startswith("Suitable for"):
            new = "(New)" in text or "(New)" in " ".join(after)
            continue
        if text.startswith("•"):
            name = next((s for label, s in SUPPLIES.items() if label in text), None)
            articles = joined(tokens, block[n + 1:n + 3], CONVERTER)
            if name and len(articles) == len(motors):
                for m, a in zip(motors, articles):
                    m[f"converter_{'new_' if new else ''}{name}"] = a
            elif name and articles:  # a row without any article: no converter at this supply
                problems.append(text)
            continue
        if re.match(r"^\d+ \.\.\. \d+ V", text):
            supply = "230v" if text.startswith("200") else "400v"
            continue
        label = next(((l, c) for l, c in ROWS + RATED if text.startswith(l)), None)
        # "with brake" can stand on the label's own line or on the next one.
        brake = text + " " + (" ".join(after) if after and after[0].startswith("(with") else "")
        if "reinforced" in brake:
            continue
        if text.startswith("Moment of inertia") and "with brake" in brake:
            label = ROWS[6]
        elif text.startswith("Weight"):
            label = ROWS[7] if "with brake" in brake else ROWS[8]
        if not label:
            continue
        values = tokens[-len(motors):]
        if label[0] == "Rated power" and sum(1 for t in tokens if NUMBER.match(t)) < len(motors):
            continue  # one table prints a value too few; which motor lacks it cannot be told, so none gets one
        if len(tokens) <= len(motors) or not all(NUMBER.match(v) for v in values):
            problems.append(text)
            continue
        column = label[1].format(supply)
        if column.startswith("rated_power") and "kW" in tokens:
            values = [v if v == "–" else f"{float(v) * 1000:g}" for v in values]
        for m, v in zip(motors, values):
            m[column] = "" if v == "–" else v
    return motors, problems


CONVERTER_ROWS = [("Rated current", "line_current_a"), ("Power loss", "power_loss_w"),
                  ("• Rated power 230 V", "rated_power_1ac_230v_kw"), ("- 400 V", "rated_power_3ac_400v_kw"),
                  ("- 240 V", "rated_power_3ac_240v_kw"), ("• Rated current", "rated_output_current_a"),
                  ("• Output current, max.", "max_output_current_a")]
CONVERTER_HEADER = ["article", "series", "supply"] + [c for _, c in CONVERTER_ROWS]


def converters(lines):
    """The converters of the four "Technical specifications" tables, one column per converter."""
    out, problems = [], []
    for start, line in enumerate(lines):
        head = re.match(r"\s*Line voltage (200 \.\.\. \d+ V \d AC)\s+SINAMICS S210 servo converters( \(New\))?, \d AC series", line)
        if not head:
            continue
        rows = [l.replace("‑", "-").split() for l in lines[start + 1:] if l.strip()]
        found = [{"article": a, "series": "S210 (New)" if head.group(2) else "S210", "supply": head.group(1)}
                 for a in joined(rows[0], rows[1:3], CONVERTER)]
        for tokens in rows[1:]:
            text = " ".join(tokens)
            if text.startswith("Pulse frequency"):
                break
            column = next((c for label, c in CONVERTER_ROWS if text.startswith(label)), None)
            values = tokens[-len(found):]
            if not column or len(tokens) <= len(found):
                continue
            if not all(NUMBER.match(v) for v in values):
                problems.append(text)
            for converter, v in zip(found, values):
                converter[column] = v
        out += found
    return out, problems


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    motors, problems = {}, []
    for block in blocks(lines):
        found, p = motors_of(block)
        problems += p
        for m in found:
            motors.setdefault(m["article"], {}).update(m)
    if problems:
        sys.exit("rows without one value per motor:\n" + "\n".join(problems))
    rows = []
    for article, m in sorted(motors.items()):
        m["series"] = article[:4]
        m["version"] = {"1": "High Dynamic", "2": "Compact"}[article[4]]
        missing = [c for _, c in ROWS if not m.get(c)]
        if missing:
            sys.exit(f"{article}: no {missing}")
        rows.append([m.get(c, "") for c in HEADER])
    notes = f"""# Source: {SOURCE}, "Specific technical specifications" of each shaft height.
# Extracted by catalog/tools/extract_siemens_d32.py; the numbers are the catalogue's, the layout is ours.
# One row per motor winding: the article number up to the speed letter, e.g. 1FK2104-5AK.
# Currents are RMS. static: at standstill; max_speed: mechanical limit. Inertias are the rotor's.
# The rated values depend on the supply: 230v is 200 ... 240 V (1 AC or 3 AC), 400v is 380 ... 480 V 3 AC.
# converter_*: the SINAMICS S210 converter the catalogue pairs with the motor at that supply;
#   converter_new_*: the same for the "SINAMICS S210 (New)" converters (6SL5310).
# An empty field: the catalogue lists no value, mostly because the motor is not offered at that supply.
#   The 230 V rated power of the 1FT2 shaft height 100 motors is empty because the catalogue's row is one value short.
# Kept as printed although it looks like a misprint: rated_current_230v_a of the 1FT2206-2AC is 3.75 A, above
#   its stall current of 2.8 A and above its converter's 3 A.
# Not in this catalogue, and so not here: torque and voltage constant, winding resistance and
#   inductance, pole pairs, thermal time constant. The configuration manuals have the thermal time constant
#   of most of these motors (motors_1fk2_s120.csv, motors_1ft2_s120.csv). Winding resistance and inductance
#   are published nowhere for these motors.
"""
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    (out / "motors_s210.csv").write_text(notes + ",".join(HEADER) + "\n" + "".join(",".join(r) + "\n" for r in rows),
                                        encoding="utf-8", newline="\n")
    print(f"{len(rows)} motors")

    found, problems = converters(lines)
    if problems or not found or any(not c.get("max_output_current_a") for c in found):
        sys.exit("converter tables not read:\n" + "\n".join(problems))
    notes = f"""# Source: {SOURCE}, "Technical specifications" of the servo converters.
# Extracted by catalog/tools/extract_siemens_d32.py; the numbers are the catalogue's, the layout is ours.
# One row per converter. line_current: rated input current. Output currents are RMS; the power unit
# switches at 8 kHz. rated_power: at the output, for the supply voltage in the column name.
# Not extracted: DC link and braking resistor data, frame sizes, dimensions, inputs and outputs.
"""
    text = notes + ",".join(CONVERTER_HEADER) + "\n" + "".join(",".join(c.get(k, "") for k in CONVERTER_HEADER) + "\n" for c in found)
    (out / "converters_s210.csv").write_text(text, encoding="utf-8", newline="\n")
    print(f"{len(found)} converters")


if __name__ == "__main__":
    main()
