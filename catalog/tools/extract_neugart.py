"""Extracts Neugart's planetary gearboxes PLE, PLN and PLFE from the series chapters of Neugart's product catalogue,
edition 05.2026 (English), into catalog/neugart/gearbox_<series>.csv.

The documents are not part of this repository. Get the PDFs from Neugart:

    https://cdn.neugart.com/fileadmin/user_upload/Downloads/Catalog_Chapters/05_2026/PLE/PLE_2026_05_EN.pdf
    https://cdn.neugart.com/fileadmin/user_upload/Downloads/Catalog_Chapters/05_2026/PLN/PLN_2026_05_EN.pdf
    https://cdn.neugart.com/fileadmin/user_upload/Downloads/Catalog_Chapters/05_2026/PLFE/PLFE_2026_05_EN.pdf

then, per chapter (xpdf 4.x):

    pdftotext -table -enc UTF-8 PLE_2026_05_EN.pdf PLE.txt
    python tools/extract_neugart.py PLE.txt PLN.txt PLFE.txt neugart

Read: the "Technical data" pages. Per size (columns) and stage: efficiency, standard and reduced backlash, gearbox
weight and average idle torque (a single value only where the range collapses to one number; the chapters print the
rest as ranges, the per-ratio values are only in Neugart's online Tec Data Finder), max radial and axial force and
tilting moment, the clamping system bores and which one is the reference bore, n1max. Per size and ratio: T2D, T2max,
n1D, T2Stop (and T2Z, checked for presence only: the schema has no column for it). One row per size and ratio with
the reference clamping system bore, the bore the per-ratio values are "based on". The stage of each ratio comes from
the stage label printed on the middle row of each stage group. A row or table that does not fit stops the script.
"""
import csv
import re
import sys
from itertools import product
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["series", "size", "motor_frame", "clamping_hub_diameter_m", "stages", "ratio", "rated_output_torque_nm",
           "max_output_torque_nm", "emergency_stop_output_torque_nm", "rated_input_speed_rad_s",
           "max_input_speed_rad_s", "no_load_torque_nm", "no_load_torque_speed_rad_s", "backlash_rad",
           "backlash_reduced_rad", "torsional_stiffness_nm_rad", "inertia_at_input_kgm2", "max_radial_force_n", "max_axial_force_n", "max_tilting_moment_nm",
           "efficiency_full_load", "mass_kg"]
SYMBOLS = ["T2Z", "T2max", "T2D", "n1D", "T2Stop"]
IDLE_SPEED_RPM = "3000"  # glossary of the complete catalogue 05.2026, p. 206: T0 at n1 = 3,000 rpm and 20 degC
VALUE = re.compile(r"^(<\s*)?\d[\d.,]*( - \d[\d.,]*)?$|^-$")


def die(msg):
    sys.exit(f"extract_neugart: {msg}")


def dec(text):
    """'0,7' -> '0.7' (the chapters mix decimal commas into dotted tables)."""
    return text.replace(",", ".")


def trailing_cells(line):
    """The run of value cells (columns are 2+ blanks apart) at the end of a line."""
    out = []
    for cell in reversed(re.split(r"\s{2,}", line.strip())):
        if not VALUE.match(cell):
            break
        out.append(cell)
    return out[::-1]


def section(lines, start, end):
    i = next((k for k, l in enumerate(lines) if re.search(start, l)), None)
    if i is None:
        die(f"no line matching {start!r}")
    j = next((k for k in range(i + 1, len(lines)) if re.search(end, lines[k])), None)
    if j is None:
        die(f"no line matching {end!r} after {start!r}")
    return i, j


def per_stage(lines, lo, hi, n):
    """Groups of (stage, values) lines between lo and hi, keyed by the label inside each group's line span."""
    labels = {"efficiency": r"Efficiency", "stiffness": r"Torsional stiffness", "mass": r"Gearbox weight",
              "inertia": r"Mass moment of inertia", "idle": r"Average idle torque"}
    groups, cur = [], None
    for k in range(lo, hi):
        cells = trailing_cells(lines[k])
        if cells and cells[0].startswith("<"):
            continue
        if len(cells) == n and " - " in " ".join(cells):
            # PLN inertia: the stage number sits on the next (lbf.in) line, alone after its unclosed "(x -" cells
            nxt = next((trailing_cells(lines[j]) for j in range(k + 1, min(k + 3, hi)) if lines[j].strip()), [])
            if nxt in (["1"], ["2"], ["3"]):
                cells = cells + nxt
        if len(cells) < 2 or cells[-1] not in ("1", "2", "3"):
            continue
        values = cells[:-1]
        if len(values) not in (1, n):
            continue
        stage = int(cells[-1])
        if stage == 1:
            cur = {"first": k, "rows": {}}
            groups.append(cur)
        elif cur is None or stage != max(cur["rows"]) + 1:
            die(f"line {k + 1}: stage {stage} out of order")
        cur["rows"][stage] = values * n if len(values) == 1 else values
        cur["last"] = k
    out = {}
    for g in groups:
        found = [name for name, rx in labels.items()
                 if any(re.search(rx, lines[k]) for k in range(g["first"], g["last"] + 1))]
        if len(found) != 1:
            die(f"lines {g['first'] + 1}-{g['last'] + 1}: labels {found}")
        out[found[0]] = g["rows"]
    for need in labels:
        if need not in out:
            die(f"no per-stage table '{need}'")
    return out


def single(cell):
    """A per-stage cell as one number, or '' for a range ('2.1 - 2.2') or '-'."""
    return "" if " - " in cell or cell == "-" else dec(cell)


def end_of(cell, upper):
    """(value, from_range): a single number as printed, or the lower / upper end of a printed range."""
    if cell == "-":
        die("'-' where a value is needed")
    if " - " not in cell:
        return dec(cell), False
    lo, hi = (dec(x.strip()) for x in cell.split(" - "))
    if not float(lo) <= float(hi):
        die(f"range {cell}")
    return (hi if upper else lo), True


def parse(text):
    lines = text.splitlines()
    head = next((l for l in lines if "Gearbox characteristics" in l), None) or die("no 'Gearbox characteristics'")
    sizes = re.findall(r"\b((PLE|PLN|PLFE)\d{3})\b", head)
    series = {s[1] for s in sizes}
    if len(series) != 1 or len(sizes) < 3:
        die(f"sizes {sizes}")
    series, sizes, n = series.pop(), [s[0] for s in sizes], len(sizes)

    lo, hi = section(lines, r"Gearbox characteristics", r"Maximum bending moment")
    stage_tables = per_stage(lines, lo, hi, n)

    backlash, reduced = {}, None
    for k in range(lo, hi):
        if "<" not in lines[k]:
            continue
        cells = trailing_cells(lines[k])
        if "Reduced backlash" in lines[k]:
            if len(cells) != n:
                die(f"line {k + 1}: reduced backlash {cells}")
            reduced = [c.lstrip("<").strip() for c in cells]
            continue
        stage, values = int(cells[-1]), cells[:-1]
        if len(values) not in (1, n) or stage in backlash:
            die(f"line {k + 1}: backlash {cells}")
        backlash[stage] = [c.lstrip("<").strip() for c in (values * n if len(values) == 1 else values)]

    loads = {}
    for label, key in (("Maximum radial force", "fr"), ("Maximum axial force", "fa"), ("Maximum tilting moment", "mk")):
        line = next((l for l in lines[lo:hi] if label in l), None) or die(f"no '{label}'")
        vals = re.findall(r"(\d[\d.,]*)\s*\([\d.,]+\)", line)
        if len(vals) != n:
            die(f"{label}: {vals}")
        loads[key] = [dec(v) for v in vals]

    ref_note = next((re.match(r"\s*\((\d)\)\s+Reference clamping system diameter", l) for l in lines
                     if re.match(r"\s*\(\d\)\s+Reference clamping system diameter", l)), None)
    if not ref_note:
        die("no footnote 'Reference clamping system diameter'")
    mark = f"({ref_note[1]})"
    c0, c1 = section(lines, r"Input characteristics", r"Mass moment of inertia input")
    ref = [None] * n
    entry = re.compile(r"(\d+)\s*\([A-Z]\)(\(\d\))?|(?<=\s)-(?=\s|$)")
    for k in range(c0 + 1, c1):
        found = list(entry.finditer(lines[k]))
        if not found:
            continue
        if len(found) != n:
            die(f"line {k + 1}: clamping bores {[m[0] for m in found]}")
        for c, m in enumerate(found):
            if m[2] == mark:
                if ref[c]:
                    die(f"{sizes[c]}: two reference bores")
                ref[c] = m[1]
    if None in ref:
        die(f"reference bores {ref}")

    nmax_line = next((l for l in lines if re.search(r"\bn1max\b", l)), None) or die("no n1max")
    n1max = trailing_cells(nmax_line)
    if len(n1max) != n:
        die(f"n1max {n1max}")

    # Per-ratio tables, from the first "Output torques" header to the dimensions.
    t0, t1 = section(lines, r"^\s*Output torques", r"Dimensions")
    rows = []
    for k in range(t0, t1):
        tokens = [t for t in re.sub(r"\([^)]*\)", " ", lines[k]).split() if re.fullmatch(r"\d[\d.,]*|-", t)]
        if len(tokens) in (n + 1, n + 2):
            rows.append((k, tokens))
    first_ratio = rows[0][1][n] if rows else die("no per-ratio rows")
    tables, cur = [], None
    for k, tokens in rows:
        if tokens[n] == first_ratio:
            cur = []
            tables.append(cur)
        cur.append((k, tokens))
    by_symbol = {}
    for t in tables:
        span = " ".join(lines[t[0][0]:t[-1][0] + 1])
        sym = [s for s in SYMBOLS if re.search(rf"\b{s}\b", span)]
        if len(sym) != 1 or sym[0] in by_symbol:
            die(f"table at line {t[0][0] + 1}: symbols {sym}")
        by_symbol[sym[0]] = t
    if set(by_symbol) != set(SYMBOLS):
        die(f"tables {sorted(by_symbol)}")
    ratios = [tok[n] for _, tok in by_symbol["T2D"]]
    for sym, t in by_symbol.items():
        if [tok[n] for _, tok in t] != ratios:
            die(f"{sym}: ratios differ from T2D")
    stages = stage_of(ratios, [(i, tok[n + 1]) for i, (_, tok) in enumerate(by_symbol["T2D"]) if len(tok) == n + 2])
    for sym, t in by_symbol.items():
        labels = [(i, tok[n + 1]) for i, (_, tok) in enumerate(t) if len(tok) == n + 2]
        if stage_of(ratios, labels) != stages:
            die(f"{sym}: stage labels differ from T2D")

    out = []
    for c, size in enumerate(sizes):
        for i, ratio in enumerate(ratios):
            vals = {sym: by_symbol[sym][i][1][c] for sym in SYMBOLS}
            if all(v == "-" for v in vals.values()):
                continue
            if "-" in vals.values():
                die(f"{size} i={ratio}: '-' in some tables only: {vals}")
            p = stages[i]
            if p not in backlash or backlash[p][c] == "-":
                die(f"{size} i={ratio}: no backlash for {p} stages")
            est, picked = [], {}
            # the conservative end of a printed range: lower stiffness, higher inertia, mass and idle torque
            for col, table, upper in (("torsional_stiffness_nm_rad", "stiffness", False),
                                      ("inertia_at_input_kgm2", "inertia", True), ("mass_kg", "mass", True),
                                      ("no_load_torque_nm", "idle", True)):
                if p not in stage_tables[table]:
                    die(f"{size}: no {table} for {p} stages")
                picked[col], from_range = end_of(stage_tables[table][p][c], upper)
                if from_range:
                    est.append(col)
            out.append({
                "series": series, "size": size, "motor_frame": ref[c],
                "clamping_hub_diameter_m": to_si(ref[c], "milli"), "stages": str(p), "ratio": ratio,
                "rated_output_torque_nm": dec(vals["T2D"]), "max_output_torque_nm": dec(vals["T2max"]),
                "emergency_stop_output_torque_nm": dec(vals["T2Stop"]),
                "rated_input_speed_rad_s": to_si(vals["n1D"], "rpm"), "max_input_speed_rad_s": to_si(n1max[c], "rpm"),
                "no_load_torque_nm": picked["no_load_torque_nm"],
                "no_load_torque_speed_rad_s": to_si(IDLE_SPEED_RPM, "rpm"),
                "torsional_stiffness_nm_rad": to_si(picked["torsional_stiffness_nm_rad"], "per_arcmin"),
                "inertia_at_input_kgm2": to_si(picked["inertia_at_input_kgm2"], "kgcm2"),
                "backlash_rad": to_si(backlash[p][c], "arcmin"),
                "backlash_reduced_rad": to_si(reduced[c], "arcmin") if reduced else "",
                "max_radial_force_n": loads["fr"][c], "max_axial_force_n": loads["fa"][c],
                "max_tilting_moment_nm": loads["mk"][c],
                "efficiency_full_load": to_si(single(stage_tables["efficiency"][p][c]), "percent"),
                "mass_kg": picked["mass_kg"], "_estimated": est,
            })
    return series, out


def stage_of(ratios, labels):
    """Stage per ratio row: contiguous groups, each with its label on row (group length - 1) // 2."""
    if [int(s) for _, s in labels] != list(range(1, len(labels) + 1)):
        die(f"stage labels {labels}")
    fits = []
    for sizes in product(range(1, len(ratios) + 1), repeat=len(labels)):
        if sum(sizes) != len(ratios):
            continue
        start, ok = 0, True
        for (row, _), m in zip(labels, sizes):
            ok &= row == start + (m - 1) // 2
            # inside a group the ratios rise
            ok &= all(float(ratios[r]) < float(ratios[r + 1]) for r in range(start, start + m - 1))
            start += m
        if ok:
            fits.append(sizes)
    if len(fits) != 1:
        die(f"stage groups for ratios {ratios} and labels {labels}: {fits}")
    return [p + 1 for p, m in enumerate(fits[0]) for _ in range(m)]


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out_dir = Path(sys.argv[-1])
    out_dir.mkdir(parents=True, exist_ok=True)
    for path in sys.argv[1:-1]:
        series, rows = parse(Path(path).read_text(encoding="utf-8"))
        keys = [(r["size"], r["motor_frame"], r["ratio"]) for r in rows]
        if len(set(keys)) != len(keys):
            die(f"{series}: duplicate keys")
        target = out_dir / f"gearbox_{series.lower()}.csv"
        with open(target, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=COLUMNS, lineterminator="\n", extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        print(f"{target}: {len(rows)} rows")
        write_provenance(target.with_suffix(".meta.toml"), rows)


MARK = "# Provenance of the range estimates: written by tools/extract_neugart.py, do not edit below.\n"


def write_provenance(meta, rows):
    """Replaces the part of an existing meta file below MARK with one [[provenance]] per estimated column."""
    if not meta.exists():
        return
    text = meta.read_text(encoding="utf-8").split(MARK)[0].rstrip("\n") + "\n\n" + MARK
    for col in ("torsional_stiffness_nm_rad", "inertia_at_input_kgm2", "mass_kg", "no_load_torque_nm"):
        keys = [f"{r['size']}|{r['motor_frame']}|{r['ratio']}" for r in rows if col in r["_estimated"]]
        if keys:
            listed = '["*"]' if len(keys) == len(rows) else "[" + ", ".join(f'"{k}"' for k in keys) + "]"
            text += f'\n[[provenance]]\nrows = {listed}\nfields = ["{col}"]\nsource = "neugart-range-estimate"\n'
    meta.write_text(text, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()
