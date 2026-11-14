"""Extracts the open-ended Synchro-Power LINEAR polyurethane timing belts with standard steel cords (SL) from Gates'
catalogue "Thermoplastic Polyurethane Belt Program, Product and Technical Information", catalog no. 20166,
edition 2025, into catalog/gates/belts.csv and catalog/gates/belt_tooth_load.csv.

The catalogue is not part of this repository. Get the PDF from Gates
(https://www.gates.com/content/dam/documents-library/catalogs/thermoplastic-polyurethane-belt-catalogue-en.pdf), then:

    pdftotext -table -enc UTF-8 thermoplastic-polyurethane-belt-catalogue-en.pdf gates_tpu.txt      (xpdf 4.x)
    python tools/extract_gates.py gates_tpu.txt gates

Each profile's page ("T5 / PITCH: 5MM", pages 11 to 23) has a standard thickness, a "MIN PULLEY TOOTH COUNT AND
DIAMETER" table (first column steel; first Z MIN / D MIN pair: drive without back idler) and a "TECHNICAL DATA"
table with one column per standard width. Only the Steel (SL) rows are read. The numbers use "." for thousands
(1.250 = 1250 N) and "," as decimal mark (0,02 kg/m). A row without one value per width stops the script.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

PROFILES = ["T5", "T10", "AT5", "AT10", "HTD5", "HTD8"]
BELT_COLUMNS = ["profile", "pitch_m", "width_m", "mass_kg_m", "max_pull_n", "breaking_strength_n",
                "elongation_at_max_pull", "min_pulley_teeth", "min_pulley_diameter_m",
                "stiffness_ea_per_width_n_m", "thickness_m"]
TOOTH_COLUMNS = ["profile", "speed_rad_s", "specific_tooth_load_n_m"]
TEETH_IN_MESH = 12  # the catalogue's allowable effective force holds for at least 12 teeth in mesh (tm = Ze/12, table 2)


def section(lines, profile):
    """The lines of the profile's first page in the Synchro-Power LINEAR part (the sleeve and flex parts come later)."""
    head = re.compile(rf"^\s*{profile} / PITCH:")
    start = next((i for i, line in enumerate(lines) if head.match(line)), None)
    if start is None:
        raise ValueError(f"{profile}: no page")
    end = next(i for i in range(start + 1, len(lines)) if "/ PITCH:" in lines[i])
    return lines[start:end]


def numbers(line, after):
    """The numbers after the token `after`, as decimal text: "." is a thousands point, "," the decimal mark."""
    rest = line.split(after, 1)[1].split()
    return [t.replace(".", "").replace(",", ".") for t in rest]


def block(part, title, label, unit, count):
    """The values of the row `label` (None: the unlabelled row) in the block headed `title`."""
    start = next((i for i, line in enumerate(part) if title in line), None)
    if start is None:
        raise ValueError(f"no block {title}")
    for line in part[start + 1:]:
        if line.strip() and line.strip().isupper() and "[" not in line:
            break  # the next block's title
        if unit in line and (line.split(unit)[0].strip() == "" if label is None else label in line):
            row = numbers(line, unit)
            if len(row) != count:
                raise ValueError(f"{title} {label}: {row} for {count} widths")
            return row
    raise ValueError(f"no row {label} in {title}")


def first_number(part, label, unit):
    """The first value of the first row "Zmin  Z MIN  10 ..." (the steel column, drive without back idler)."""
    m = next((re.match(rf"^\s*{label}\s+{unit}\s+(\d+)", line) for line in part if re.match(rf"^\s*{label}\s", line)), None)
    if m is None:
        raise ValueError(f"no {label}")
    return m.group(1)


def thickness(part, profile):
    i = next(i for i, line in enumerate(part) if "STANDARD THICKNESS" in line)
    m = re.search(r"([\d,]+) mm", part[i])
    if m is None:  # printed per variant on the next lines: "T10   4,5 mm"
        following = next(line for line in part[i + 1:] if line.strip())
        m = re.match(rf"^\s*{profile}\s+([\d,]+) mm", following)
    if m is None:
        raise ValueError("no standard thickness")
    return m.group(1).replace(",", ".")


def profile_rows(lines, profile):
    part = section(lines, profile)
    try:
        pitch = re.search(r"PITCH:\s*(\d+)MM", part[0]).group(1)
        header = next(line for line in part if line.lstrip().startswith("STANDARD") and "UNIT" in line)
        widths = numbers(header, "UNIT")
        n = len(widths)
        breaking = block(part, "BREAKING FORCE / AVERAGE VALUE", "Steel (SL)", "[N]", n)
        pull = block(part, "ALLOWABLE BELT FORCE / OPEN ENDED", "Steel (SL)", "[N]", n)
        effective = block(part, "ALLOWABLE EFFECTIVE FORCE / MINIMUM 12 TEETH IN MESH", None, "[N]", n)
        mass = block(part, "BELT WEIGHT", "Steel (SL)", "[kg/m]", n)
        stiffness = block(part, "SPECIFIC BELT STIFFNESS", "Steel (SL)", "[N]", n)
        z_min = first_number(part, "Zmin", "Z MIN")
        d_min = first_number(part, "dmin", "D MIN")
        thick = thickness(part, profile)
    except (ValueError, StopIteration, AttributeError) as e:
        raise ValueError(f"{profile}: {e}") from None
    belts = []
    for w, b, p, m, c in zip(widths, breaking, pull, mass, stiffness):
        width_m = to_si(w, "milli")
        belts.append({
            "profile": profile, "pitch_m": to_si(pitch, "milli"), "width_m": width_m, "mass_kg_m": format(Decimal(m).normalize(), "f"),
            "max_pull_n": p, "breaking_strength_n": b, "elongation_at_max_pull": "",
            "min_pulley_teeth": z_min, "min_pulley_diameter_m": to_si(d_min, "milli"),
            # "Specific belt stiffness" [N] is the force for 100 % strain of that width (EA); per metre of width:
            "stiffness_ea_per_width_n_m": format((Decimal(c) / Decimal(width_m)).quantize(Decimal(1)), "f"),
            "thickness_m": to_si(thick, "milli"),
        })
    # Allowable effective force at standstill per tooth and metre of width, from the widest belt (least rounding).
    per_tooth = Decimal(effective[-1]) / TEETH_IN_MESH / Decimal(belts[-1]["width_m"])
    tooth = [{"profile": profile, "speed_rad_s": "0.0", "specific_tooth_load_n_m": format(per_tooth.quantize(Decimal(1)), "f")}]
    return belts, tooth


def write(path, columns, rows):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=columns, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def main(text_file, out_dir):
    lines = Path(text_file).read_text(encoding="utf-8").splitlines()
    belts, teeth = [], []
    for profile in PROFILES:
        b, t = profile_rows(lines, profile)
        belts += b
        teeth += t
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    write(out / "belts.csv", BELT_COLUMNS, belts)
    write(out / "belt_tooth_load.csv", TOOTH_COLUMNS, teeth)
    print(f"{len(belts)} belts, {len(teeth)} tooth load points")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: extract_gates.py <pdftotext output> <output folder>")
    main(*sys.argv[1:3])
