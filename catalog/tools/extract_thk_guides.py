"""Extracts THK's caged-ball LM Guide SHS (blocks C, LC, V, LV, R, LR) and full-ball LM Guide HSR (blocks C, LC, XC,
XLC) from THK's "General Catalog" into catalog/thk/linear_guides.csv.

The catalogue is not part of this repository. THK publishes it in chunks named by their first page; get

    https://tech.thk.com/en/products/pdf/en_a01_094.pdf     (SHS, pages A1-94 to A1-105, edition mark 515-2E)
    https://tech.thk.com/en/products/pdfs/en_a01_192.pdf    (HSR, pages A1-192 and A1-193, edition mark 515-1E)

(note the directory pdf against pdfs), then

    pdftotext -table -enc UTF-8 en_a01_094.pdf shs.txt      (xpdf 4.06)
    pdftotext -table -enc UTF-8 en_a01_192.pdf hsr.txt
    python -I tools/extract_thk_guides.py shs.txt hsr.txt thk

SHS: dimension tables A1-98 (C, LC), A1-100 (V, LV), A1-102 (R, LR); rail dimensions, ratings, static moments and
masses on the next page of each (A1-99, A1-101, A1-103). HSR: A1-192 (dimensions of C, LC, XC, XLC) and A1-193 (the
rest); the chunk's text layer prints its page numbers without digits, so these two numbers are the chunk's own
(en_a01_192 starts at A1-192). A dimension table lists a model per line (the models with an M suffix, stainless steel,
have no values of their own and are not rows); the table after it lists C, C0, the moments and masses in the same
order, without a model name. A full line has the rail dimensions in it; a line without them (the long block of
sizes 30 and up) shares the rail of the line before. The model lines and value lines must be the same number and
every line the same count of values, else the script stops.
Columns of the table after the dimensions: H3, W1, W2, M1, F, d1 x d2 x h, Max length, C [kN], C0 [kN], MA 1 block,
MA 2 blocks, MB 1 block, MB 2 blocks, MC 1 block [kN m], block mass [kg], rail mass [kg/m]. MA is the moment in the
pitching, MB in the yawing and MC in the rolling direction ("A Technical Descriptions", page A-8 of the General
Catalog); the table takes the 1 block values.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

COLUMNS = ["part_number", "series", "size", "carriage_type", "rail_width_m", "height_m", "carriage_length_m",
           "dynamic_load_rating_n", "dynamic_load_rating_distance_m", "static_load_rating_n", "static_moment_x_nm",
           "static_moment_y_nm", "static_moment_z_nm", "carriage_mass_kg", "rail_mass_kg_m", "source"]
MODEL = re.compile(r"^\s*(SHS|HSR)\s+(\d+)\s*([A-Z]+)\b(.*)$")
TYPES = {"SHS": {"C", "LC", "V", "LV", "R", "LR"}, "HSR": {"C", "LC", "XC", "XLC"}}
STAINLESS = {"CM", "LCM", "VM", "LVM", "RM", "LRM", "XCM", "XLCM"}
D = re.compile(r"\d+(?:\.\d+)?[×ʷ]\d+(?:\.\d+)?[×ʷ]\d+(?:\.\d+)?$")
NUM = re.compile(r"\d+(?:\.\d+)?$")


def scale(text, factor):
    return format((Decimal(text) * Decimal(factor)).normalize(), "f")


def sections(lines, series):
    """The lines of every "Models ..." section of one chunk, from its heading up to the footnote "1 Length L ..." of
    its second table (what follows, the table of standard rail lengths, is numbers too)."""
    heads = [i for i, l in enumerate(lines) if re.match(rf"\s*Models\s+{series}-", l)]
    if not heads:
        raise SystemExit(f"{series}: no 'Models {series}...' heading")
    out = []
    for k, i in enumerate(heads):
        block = lines[i: heads[k + 1] if k + 1 < len(heads) else len(lines)]
        end = next((j for j, l in enumerate(block) if re.match(r"\s*1\s+Length", l)), None)
        if end is None:
            raise SystemExit(f"{block[0].strip()}: footnote 1 not found")
        out.append((re.sub(r"\s+", " ", block[0]).strip(), block[:end]))
    return out


def read_section(series, block):
    """(models, values) of one section: per model (size, type, L, M or None); per value line (W1, the eight values
    from C to the block mass, the rail mass)."""
    models, values, rail = [], [], None
    for line in block:
        m = MODEL.match(line)
        if m and m.group(1) == series:
            size, kind, rest = m.group(2), m.group(3), m.group(4).split()
            if kind in STAINLESS:
                continue
            if kind not in TYPES[series]:
                raise SystemExit(f"{series} {size}{kind}: block type not known")
            if len(rest) >= 10:  # full line: M, W, L, B, C, mounting hole ...
                models.append((size, kind, rest[2], rest[0]))
            elif rest and all(NUM.match(t) for t in rest):  # the long block of a size: L, then L1 (and C)
                models.append((size, kind, rest[0], None))
            else:
                raise SystemExit(f"{series} {size}{kind}: line not read: {line.strip()}")
            continue
        tokens = line.split()
        if not tokens or tokens[0].startswith("("):
            continue
        d = [i for i, t in enumerate(tokens) if D.match(t)]
        if d:
            i = d[0]
            if len(tokens) - i - 1 != 10 or not all(NUM.match(t) for t in tokens[i + 1:]) or i < 4:
                raise SystemExit(f"{series}: value line not read: {line.strip()}")
            rail = (tokens[i - 4], tokens[-1])
            values.append((rail[0], tokens[i + 2:i + 10], rail[1]))
        elif len(tokens) in (8, 9) and all(NUM.match(t) for t in tokens):
            # the long block of a size shares the rail; HSR prints its mass again, SHS does not
            if rail is None or (len(tokens) == 9 and tokens[8] != rail[1]):
                raise SystemExit(f"{series}: value line without its rail: {line.strip()}")
            values.append((rail[0], tokens[:8], rail[1]))
    return models, values


def main():
    if len(sys.argv) != 4:
        raise SystemExit("usage: extract_thk_guides.py shs.txt hsr.txt out_dir")
    rows = []
    for path, series, source, stamp in ((sys.argv[1], "SHS", "thk-shs", "515-2E"), (sys.argv[2], "HSR", "thk-hsr", "515-1E")):
        text = Path(path).read_text(encoding="utf-8")
        if stamp not in text or "Static permissible moment" not in text:
            raise SystemExit(f"{path}: edition mark {stamp} or the table of {series} not found")
        for heading, block in sections(text.splitlines(), series):
            models, values = read_section(series, block)
            if not models or len(models) != len(values):
                raise SystemExit(f"{heading}: {len(models)} models and {len(values)} value lines")
            height = None
            for (size, kind, length, m_height), (w1, v, rail_mass) in zip(models, values):
                height = m_height or height
                c, c0, ma, _ma2, mb, _mb2, mc, mass = v
                rows.append({
                    "part_number": f"{series}{size}{kind}", "series": series, "size": size,
                    "carriage_type": f"{series}-{kind}", "rail_width_m": scale(w1, "0.001"),
                    "height_m": scale(height, "0.001"), "carriage_length_m": scale(length, "0.001"),
                    "dynamic_load_rating_n": scale(c, "1000"), "dynamic_load_rating_distance_m": "50000",
                    "static_load_rating_n": scale(c0, "1000"), "static_moment_x_nm": scale(mc, "1000"),
                    "static_moment_y_nm": scale(ma, "1000"), "static_moment_z_nm": scale(mb, "1000"),
                    "carriage_mass_kg": mass, "rail_mass_kg_m": rail_mass, "source": source,
                })
    names = [r["part_number"] for r in rows]
    if len(set(names)) != len(names):
        raise SystemExit("a part number twice")
    out = Path(sys.argv[3])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "linear_guides.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} carriages")


if __name__ == "__main__":
    main()
