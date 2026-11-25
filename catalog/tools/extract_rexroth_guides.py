"""Extracts Bosch Rexroth's ball runner blocks BSHP made of steel (the standard blocks and the heavy-duty blocks of
sizes 55 and 65, ball rail systems "High Precision BSHP") into catalog/rexroth/linear_guides.csv.

The document is not part of this repository. Get the PDF from Bosch Rexroth, "Ball rail systems, High Precision BSHP",
R999000485/2025-11 (https://www.boschrexroth.com/media/ec1fd87f-8f69-4a5e-9596-e3f0089bc0f9), then:

    pdftotext -table -enc UTF-8 rex_ec1f.pdf rex_ec1f.txt      (xpdf 4.06)
    python -I tools/extract_rexroth_guides.py rex_ec1f.txt rexroth

Pages (printed numbers): the dimension and rating tables of the standard blocks on pages 51 to 73 (one block type per
double page: FNS 51, FLS 53, FKS 55, SNS 57, SLS 59, SKS 61, SNH 63, SLH 65, FNN 67, FKN 69, SNN 71, SKN 73), the
heavy-duty blocks of sizes 55 and 65 on pages 75 to 85 (FNS, FLS, SNS, SLS, SNH, SLH); the rail mass of the standard
rails on page 113; the definition of the moments on page 20. The text file holds one form feed per PDF page, and a PDF
page holds a double page, so the table of the printed page n is on form-feed page n - 1.
A block type's first table prints the dimensions per size (the columns named in its header line), the second the mass,
C, C0, Mt, Mt0, ML and ML0. The numbers use a space as thousands separator ("9 860"); a row's values are read in the
one way in which the static value of each pair is 1 to 3 times the dynamic one, and the script stops where there
is not exactly one way. Every row, header or count that does not fit stops the script.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

COLUMNS = ["part_number", "series", "size", "carriage_type", "rail_width_m", "height_m", "carriage_length_m",
           "dynamic_load_rating_n", "dynamic_load_rating_distance_m", "static_load_rating_n", "static_moment_x_nm",
           "static_moment_y_nm", "static_moment_z_nm", "dynamic_moment_x_nm", "dynamic_moment_y_nm",
           "dynamic_moment_z_nm", "carriage_mass_kg", "rail_mass_kg_m"]
# Form-feed page of the first (left) PDF page of each block type's double page.
STANDARD = list(range(50, 74, 2))
HEAVY = list(range(74, 86, 2))
SIZES = {"15", "20", "25", "30", "35", "45", "55", "65"}


def mm(text):
    """Millimetres as printed to metres, exactly: 24 -> 0.024, 58.2 -> 0.0582."""
    return format((Decimal(text) / 1000).normalize(), "f")


def splits(tokens, want=6):
    """Every way to read the integer tokens as `want` numbers with a space as thousands separator, that satisfy the
    pairs C < C0, Mt <= Mt0, ML <= ML0 within a factor of 3 (the static value of a pair is never far above)."""
    out = []

    def go(i, acc):
        if len(acc) > want:
            return
        if i == len(tokens):
            if len(acc) == want:
                out.append(acc)
            return
        go(i + 1, acc + [int(tokens[i])])
        if i + 1 < len(tokens) and len(tokens[i + 1]) == 3 and len(tokens[i]) <= 3:
            go(i + 2, acc + [int(tokens[i] + tokens[i + 1])])

    go(0, [])
    return [v for v in out if all(1 <= v[i + 1] / v[i] <= 3 for i in (0, 2, 4)) and v[0] != v[1]]


def table_rows(lines, start, what):
    """The data rows (first token a size) after line `start` up to the first line that is not one."""
    rows = []
    for line in lines[start:]:
        tokens = line.split()
        if not tokens:
            continue
        if tokens[0] in SIZES and len(tokens) > 3:
            rows.append(tokens)
        elif rows:
            break
    if not rows:
        raise SystemExit(f"{what}: no rows")
    return rows


def block_type(text):
    """The block type code and its title from the heading line "FNS – Flange, normal, standard height"."""
    found = re.findall(r"^\s*([A-Z]{3}) – ([A-Za-z, ]+?)\s*$", text, re.M)
    if len(found) != 1:
        raise SystemExit(f"{len(found)} block type headings: {found}")
    code, title = found[0]
    return code, "; ".join(p.strip().lower() for p in title.split(","))


def one_page(pages, n):
    left, right = pages[n - 1], pages[n]  # the title is on the left PDF page, the tables on the right one
    code, title = block_type(left + "\n" + right)
    lines = right.splitlines()
    head = [i for i, l in enumerate(lines) if re.match(r"\s*Size\s+Dimensions\s+\(mm\)\s*$", l)]
    mass = [i for i, l in enumerate(lines) if re.match(r"\s*Size\s+Dimensions\s+\(mm\)\s+Mass\b", l)]
    if len(head) != 1 or len(mass) != 1:
        raise SystemExit(f"page {n + 1}: {len(head)} dimension and {len(mass)} rating tables, expected one each")
    names = next(l for l in lines[head[0] + 1:] if l.strip()).split()
    dims = {}
    for t in table_rows(lines, head[0] + 2, f"page {n + 1} dimensions"):
        if len(t) != len(names) + 1:
            raise SystemExit(f"page {n + 1}: size {t[0]} has {len(t) - 1} values for {len(names)} columns")
        dims[t[0]] = dict(zip(names, t[1:]))
    # the rating table's header may run over one or two lines before the first row
    ratings = {}
    for t in table_rows(lines, mass[0] + 1, f"page {n + 1} ratings"):
        decimals = [i for i, v in enumerate(t) if "." in v]
        tail = t[decimals[-1] + 1:]
        if not decimals or not re.fullmatch(r"\d+\.\d{1,3}", t[decimals[-1]]) or not all(x.isdigit() for x in tail):
            raise SystemExit(f"page {n + 1}: size {t[0]}: rating row not read: {t}")
        got = splits(tail)
        if len(got) != 1:
            raise SystemExit(f"page {n + 1}: size {t[0]}: {len(got)} readings of {tail}")
        ratings[t[0]] = (t[decimals[-1]], got[0])
    if dims.keys() != ratings.keys():
        raise SystemExit(f"page {n + 1}: sizes differ between the tables: {sorted(dims)} and {sorted(ratings)}")
    return code, title, dims, ratings


def rail_masses(text):
    # the first rail table, on printed page 113 (standard rails SNS/SNO with cover strip and strip clamps)
    page = next((p for p in text.split("\f") if "Mass m" in p and "(kg/m)" in p and "Lmax" in p), None)
    if page is None:
        raise SystemExit("rail mass table (page 113) not found")
    lines = page.splitlines()
    at = next(i for i, l in enumerate(lines) if "Mass m" in l)
    out = {}
    for t in table_rows(lines, at + 2, "rail masses"):
        out[t[0]] = t[-1]
    if set(out) != {"15", "20", "25", "30", "35", "45", "55", "65"}:
        raise SystemExit(f"rail mass sizes {sorted(out)}")
    return out


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: extract_rexroth_guides.py rex_ec1f.txt out_dir")
    text = Path(sys.argv[1]).read_text(encoding="utf-8")
    pages = text.split("\f")
    if "based on a 100,000 m" not in text and "based on a travel of 100,000 m" not in text \
            and "based on a stroke travel of 100,000 m" not in re.sub(r"\s+", " ", text):
        raise SystemExit("the 100 km basis of C (every table's footnote) not found")
    if "R999000485/2025-11" not in text:
        raise SystemExit("edition stamp R999000485/2025-11 not found")
    if not re.search(r"around the transverse axis y or the vertical\s+axis z", text):
        raise SystemExit("definition of ML (about the transverse axis y or the vertical axis z) not found")
    rail = rail_masses(text)
    rows = []
    for firsts in (STANDARD, HEAVY):
        for n in firsts:
            code, title, dims, ratings = one_page(pages, n)
            kind = f"{code}: {title}; steel; without ball chain"
            for size, d in sorted(dims.items(), key=lambda kv: int(kv[0])):
                m, (c, c0, mt, mt0, ml, ml0) = ratings[size]
                rows.append({
                    "part_number": f"{code}{size}", "series": "BSHP", "size": size, "carriage_type": kind,
                    "rail_width_m": mm(d["A2"]), "height_m": mm(d["H"]), "carriage_length_m": mm(d["B+0.5"]),
                    "dynamic_load_rating_n": str(c), "dynamic_load_rating_distance_m": "100000",
                    "static_load_rating_n": str(c0), "static_moment_x_nm": str(mt0), "static_moment_y_nm": str(ml0),
                    "static_moment_z_nm": str(ml0), "dynamic_moment_x_nm": str(mt), "dynamic_moment_y_nm": str(ml),
                    "dynamic_moment_z_nm": str(ml), "carriage_mass_kg": m, "rail_mass_kg_m": rail[size],
                })
    names = [r["part_number"] for r in rows]
    if len(set(names)) != len(names):
        raise SystemExit("a part number twice")
    out = Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with open(out / "linear_guides.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} carriages")


if __name__ == "__main__":
    main()
