"""Extracts the data sheets of the SIMOTICS S-1FT2 motors on SINAMICS S120 from the Siemens
configuration manual "1FT2 synchronous motors for SINAMICS S120" (02/2022, A5E50645584B AC)
into catalog/siemens/motors_1ft2_s120.csv.

The manual is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -table -enc UTF-8 manual.pdf manual.txt      (xpdf 4.x)
    python extract_siemens_1ft2_s120.py manual.txt ../siemens

Every data sheet must give every value; one that does not stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, 1FT2 synchronous motors for SINAMICS S120, Configuration Manual, 02/2022, A5E50645584B AC"
ROWS = [("Static torque", "static_torque_nm"), ("Stall current", "stall_current_a"), ("Max. permissible speed", "max_speed_inverter_rpm"),
        ("Maximum torque", "max_torque_nm"), ("Maximum current", "max_current_a"), ("Thermal time constant", "thermal_time_constant_min"),
        ("Moment of inertia (with brake)", "rotor_inertia_with_brake_kgcm2"), ("Moment of inertia", "rotor_inertia_kgcm2"),
        ("Weight (with brake)", "mass_with_brake_kg"), ("Weight", "mass_kg"), ("Rated speed", "rated_speed_rpm"),
        ("Rated torque", "rated_torque_nm"), ("Rated current", "rated_current_a"), ("Rated power", "rated_power_kw")]
HEADER = ["article"] + [c for _, c in ROWS]


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()
    motors, article, sheet = {}, None, None
    for line in lines:
        text = " ".join(line.split())
        name = re.search(r"\b(1FT2\d{3}-\d[A-Z]{2})\b", text)
        if name and not re.search(r"\d$", text.replace(name.group(1), "")):
            article = name.group(1)  # a heading, not a table row
        if "Technical specifications in S120 system" in text:
            sheet = motors.setdefault(article, {})
            continue
        if sheet is None:
            continue
        label = next((c for l, c in ROWS if text.startswith(l)), None)
        value = text.split()[-1] if text else ""
        if label and re.match(r"^\d+(\.\d+)?$", value):
            sheet.setdefault(label, value)
            if label == "rated_power_kw":
                sheet = None  # the last row of a data sheet
    problems = [f"{a}: no {c}" for a, m in motors.items() for _, c in ROWS if c not in m and c != "max_speed_inverter_rpm"]
    if problems or not motors:
        sys.exit("data sheets not read:\n" + "\n".join(problems))
    notes = f"""# Source: {SOURCE}, chapter "Data sheets and characteristics".
# Extracted by catalog/tools/extract_siemens_1ft2_s120.py; the numbers are the manual's, the layout is ours.
# One row per 1FT2 motor winding as operated on SINAMICS S120 at a line voltage of 380 ... 480 V 3 AC.
# The same motors on SINAMICS S210 are in motors_s210.csv; the manual of 02/2022 covers 36 of them.
# Currents are RMS. max_speed_inverter: what the converter allows. thermal_time_constant: of the winding.
# An empty field: the manual does not state the value.
# Kept as printed although they are misprints: rated_speed_rpm of the 1FT2105-4AH and 1FT2206-4AH (3000) and of
#   the 1FT2208-4AF (2000); the manual's own rated torque and power, and the S210 catalogue, give 4500 and 3000.
"""
    rows = [[a] + [m.get(c, "") for _, c in ROWS] for a, m in sorted(motors.items())]
    out = Path(sys.argv[2])
    (out / "motors_1ft2_s120.csv").write_text(notes + ",".join(HEADER) + "\n" + "".join(",".join(r) + "\n" for r in rows),
                                             encoding="utf-8", newline="\n")
    print(f"{len(rows)} motors")


if __name__ == "__main__":
    main()
