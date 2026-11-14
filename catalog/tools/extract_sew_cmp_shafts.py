"""Extracts the shaft end and flange of the SEW-EURODRIVE CMP servomotors from the catalogue "Synchronous
Servomotors CMP40 - CMP112 High Dynamics", 27799468/EN, 11/2023, section 4.2 "Overview of motor dimensions" (page 32),
into catalog/sew/motor_shafts.csv.

The document is not part of this repository. Get the PDF from SEW-EURODRIVE
(https://download.sew-eurodrive.com/download/pdf/27799468.pdf), then:

    pdftotext -table -enc UTF-8 -f 32 -l 32 27799468.pdf cmp_p32.txt      (xpdf 4.x)
    python tools/extract_sew_cmp_shafts.py cmp_p32.txt sew

The table has one row per motor size: d1 shaft, l shaft length ("Wave length" as printed), b5 centering, e5 hole
circle, s5 bore, g5 flange square, all in mm. A size row without exactly six numbers, or a size missing, stops the
script.
"""
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

SIZES = ["CMP40", "CMP50", "CMP63", "CMP71", "CMP80", "CMP100", "CMP112"]
COLUMNS = ["part_number_prefix", "series", "shaft_diameter_m", "shaft_length_m", "centering_diameter_m",
           "flange_square_m", "bolt_circle_diameter_m"]


def parse(text):
    start = text.find("Overview of motor dimensions")
    end = text.find("Key", text.find("Flange square", start))
    if start < 0 or end < 0:
        sys.exit("table 'Overview of motor dimensions' (d1 ... g5) not found")
    rows = {}
    for line in text[start:end].splitlines():
        tokens = line.split()
        if tokens and tokens[0] in SIZES:
            if len(tokens) != 7 or not all(re.fullmatch(r"\d+(\.\d+)?", t) for t in tokens[1:]):
                sys.exit(f"{tokens[0]}: expected 6 numbers (d1 l b5 e5 s5 g5), got: {line.strip()}")
            if tokens[0] in rows:
                sys.exit(f"{tokens[0]} twice")
            rows[tokens[0]] = tokens[1:]
    missing = [s for s in SIZES if s not in rows]
    if missing:
        sys.exit(f"sizes missing: {missing}")
    return rows


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    rows = parse(Path(sys.argv[1]).read_text(encoding="utf-8"))
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "motor_shafts.csv", "w", encoding="utf-8", newline="\n") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(COLUMNS)
        for size in SIZES:
            d1, l, b5, e5, _s5, g5 = rows[size]
            w.writerow([size, "CMP"] + [to_si(v, "milli") for v in (d1, l, b5, g5, e5)])
    print(f"{len(SIZES)} rows -> {out / 'motor_shafts.csv'}")


if __name__ == "__main__":
    main()
