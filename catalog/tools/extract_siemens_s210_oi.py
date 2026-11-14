"""Extracts the thermal time constants of the SIMOTICS S-1FK2 and S-1FT2 motors from the motor data sheets in the Siemens
operating instructions "SINAMICS S210 servo drive system with S-1FK2 and S-1FT2" (09/2022, A5E41702836B AL, section 12.1
"Technical data and properties of the motor"; the 06/2020 edition AE, without 1FT2, has them in section 11.1) into
catalog/siemens/motors_s210_oi.csv. These include the 230 V windings (1FK2...-.AG) that the S120 configuration manual
does not have, and the 1FT2 that the S120 manual lacks.

The manual is not part of this repository. Get the PDF from Siemens, then:

    pdftotext -layout manual.pdf manual.txt      (xpdf 4.x or poppler)
    python extract_siemens_s210_oi.py manual.txt ../siemens

Every data sheet gives fourteen values in this order: static torque, stall current, maximum speed, maximum torque,
maximum current, thermal time constant (min), inertia, inertia with brake, weight, weight with brake, rated speed,
rated torque, rated current, rated power. The layout puts labels and values in separate columns, so the script takes
the last number of each line in a section; static torque and maximum current are kept so that the validator can match
the rows against motors_s210.csv. A motor whose data sheets (one per supply) disagree stops the script.
"""
import re
import sys
from pathlib import Path

SOURCE = "Siemens, SINAMICS S210 servo drive system with S-1FK2 and S-1FT2, Operating Instructions, 09/2022, A5E41702836B AL"
HEADER = ["article", "static_torque_nm", "max_current_a", "thermal_time_constant_min"]
HEADING = re.compile(r"\s*1[12]\.1\.\d+\.\d+\s+(1F[KT]2\d{3}-\d[A-Z]{2})\s+connected to\s+(\S.*)")


def main():
    lines = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace").splitlines()
    # Headings in the table of contents end in dots and a page number; the data sheets' headings do not.
    heads = [(i, m.group(1)) for i, l in enumerate(lines) if (m := HEADING.match(l)) and "...." not in l]
    motors = {}
    for n, (start, article) in enumerate(heads):
        end = heads[n + 1][0] if n + 1 < len(heads) else start + 120
        values = []
        for line in lines[start + 1:end]:
            # Page headers and footers carry a page number at the end of the line.
            if not line.strip() or "servo drive system" in line or "Operating Instructions" in line:
                continue
            last = line.split()[-1].replace(",", "")
            if re.fullmatch(r"\d+(\.\d+)?", last):
                values.append(last)
        if len(values) < 14:
            sys.exit(f"{article}: only {len(values)} values")
        row = [values[0], values[4], values[5]]
        if motors.setdefault(article, row) != row:
            sys.exit(f"{article}: the data sheets disagree: {motors[article]} against {row}")
    if not motors:
        sys.exit("no data sheets read")
    notes = f"""# Source: {SOURCE}, section 12.1 "Technical data and properties of the motor".
# Extracted by catalog/tools/extract_siemens_s210_oi.py; the numbers are the manual's, the layout is ours.
# One row per 1FK2 and 1FT2 motor winding with a data sheet there. thermal_time_constant: of the winding.
# static_torque and max_current only to match the rows against motors_s210.csv.
"""
    rows = "".join(",".join([a] + r) + "\n" for a, r in sorted(motors.items()))
    out = Path(sys.argv[2]) / "motors_s210_oi.csv"
    out.write_text(notes + ",".join(HEADER) + "\n" + rows, encoding="utf-8", newline="\n")
    print(f"{len(motors)} motors")


if __name__ == "__main__":
    main()
