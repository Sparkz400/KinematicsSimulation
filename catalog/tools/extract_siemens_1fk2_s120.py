"""Extracts the data sheets of the SIMOTICS S-1FK2 motors on SINAMICS S120 and the table of armature
short-circuit braking from the Siemens configuration manual "1FK2 Synchronous Motors for SINAMICS S120"
(02/2022, A5E46927724B AD) into catalog/siemens/motors_1fk2_s120.csv and dynamic_braking_1fk2.csv.

The manual is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 manual.pdf manual.txt      (xpdf 4.x)
    python extract_siemens_1fk2_s120.py manual.txt ../siemens

Every data sheet must give every value and every braking row six; one that does not stops the script.
The 1FT2 manual has the same layout; extract_siemens_1ft2_s120.py is this script's sibling.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, 1FK2 Synchronous Motors for SINAMICS S120, Configuration Manual, 02/2022, A5E46927724B AD"
ROWS = [("Static torque", "static_torque_nm"), ("Stall current", "stall_current_a"), ("Max. permissible speed", "max_speed_inverter_rpm"),
        ("Maximum torque", "max_torque_nm"), ("Maximum current", "max_current_a"), ("Thermal time constant", "thermal_time_constant_min"),
        ("Moment of inertia (with brake)", "rotor_inertia_with_brake_kgcm2"), ("Moment of inertia", "rotor_inertia_kgcm2"),
        ("Weight (with brake)", "mass_with_brake_kg"), ("Weight", "mass_kg"), ("Rated speed", "rated_speed_rpm"),
        ("Rated torque", "rated_torque_nm"), ("Rated current", "rated_current_a"), ("Rated power", "rated_power_kw")]
HEADER = ["article"] + [c for _, c in ROWS]
BRAKING = ["article", "optimum_resistance_ohm", "braking_torque_rms_nm", "braking_torque_rms_with_resistor_nm", "max_braking_torque_nm",
           "braking_current_rms_a", "braking_current_rms_with_resistor_a"]
NUMBER = r"^\d+(\.\d+)?$"


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    motors, article, sheet = {}, None, None
    braking, in_table = {}, False
    for line in lines:
        text = " ".join(line.split())
        if text.startswith("Table 5-2 Dynamic braking"):
            in_table = True
        elif in_table and text.startswith("Note"):
            in_table = False  # the note under the table; the table runs over a page break
        elif in_table and text.startswith("1FK2"):
            fields = text.split()
            if len(fields) != len(BRAKING) or not all(re.match(NUMBER, f) for f in fields[1:]) or fields[0] in braking:
                sys.exit("braking table not read: " + text)
            braking[fields[0]] = fields
        name = re.search(r"\b(1FK2\d{3}-\d[A-Z]{2})\b", text)
        if name and not re.search(r"\d$", text.replace(name.group(1), "")):
            article = name.group(1)  # a heading, not a table row
        if "Technical specifications in S120 system" in text:
            sheet = motors.setdefault(article, {})
            continue
        if sheet is None:
            continue
        label = next((c for l, c in ROWS if text.startswith(l)), None)
        value = text.split()[-1] if text else ""
        if label and re.match(NUMBER, value):
            sheet.setdefault(label, value)
            if label == "rated_power_kw":
                sheet = None  # the last row of a data sheet
    # The one value the manual prints unreadably ("1,60,1"); it stays empty rather than guessed.
    unreadable = ("1FK2308-3AB", "rotor_inertia_kgcm2")
    problems = [f"{a}: no {c}" for a, m in motors.items() for _, c in ROWS if c not in m and (a, c) != unreadable]
    if problems or not motors or not braking:
        sys.exit("data sheets not read:\n" + "\n".join(problems))
    notes = f"""# Source: {SOURCE}, chapter "Data sheets and characteristics".
# Extracted by catalog/tools/extract_siemens_1fk2_s120.py; the numbers are the manual's, the layout is ours.
# One row per 1FK2 motor winding as operated on SINAMICS S120 at a line voltage of 380 ... 480 V 3 AC.
# Most of these motors are also in motors_s210.csv, there as operated on SINAMICS S210.
# Currents are RMS. max_speed_inverter: what the converter allows. thermal_time_constant: of the winding.
# The data sheets of the 1FK2306-4AC, 1FK2308-3AB and 1FK2308-4AB head their rated data "3AC 400 ... 480 V".
# An empty field: rotor_inertia_kgcm2 of the 1FK2308-3AB, which the manual prints as "1,60,1".
"""
    braking_notes = f"""# Source: {SOURCE}, section 5.4.3, Table 5-2 "Dynamic braking for 1FK2".
# Extracted by catalog/tools/extract_siemens_1fk2_s120.py; the numbers are the manual's, the layout is ours.
# Armature short-circuit braking: the windings are shorted directly or through external resistors.
# optimum_resistance: the external resistor the manual dimensions for an optimum braking time. *_with_resistor: with it.
# braking_torque_rms: the manual's "average braking torque". Currents are RMS.
# The manual's conditions: braking from the rated speed with an external moment of inertia equal to the
#   motor's own; calculated from the rated values of the data sheet, without the variance of production and
#   without iron saturation, which can give higher currents and torques. From a lower speed the braking time
#   is shorter or the same, but not in proportion. The resistor must be intrinsically safe and temperature-monitored,
#   and braking from the rated speed is permitted at most every 2 minutes.
# Kept as printed although it looks like a misprint: braking_current_rms_with_resistor_a of the 1FK2306-2AC (6.5)
#   is above the current without the resistor (6.1); for every other motor it is about 10 % below.
"""
    out = Path(sys.argv[2])
    for file, text, header, rows in [
        ("motors_1fk2_s120.csv", notes, HEADER, [[a] + [m.get(c, "") for _, c in ROWS] for a, m in sorted(motors.items())]),
        ("dynamic_braking_1fk2.csv", braking_notes, BRAKING, [r for _, r in sorted(braking.items())]),
    ]:
        (out / file).write_text(text + ",".join(header) + "\n" + "".join(",".join(r) + "\n" for r in rows), encoding="utf-8", newline="\n")
    print(f"{len(motors)} motors, {len(braking)} braking rows")


if __name__ == "__main__":
    main()
