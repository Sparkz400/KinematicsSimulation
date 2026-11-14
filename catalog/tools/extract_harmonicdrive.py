"""Extracts Harmonic Drive's CSF, CSG and SHG component sets (strain wave gears) into
catalog/harmonicdrive/gearbox_csf.csv, gearbox_csg.csv and gearbox_shg.csv, in SI.

The documents are not part of this repository. Get them from Harmonic Drive LLC:

    https://www.harmonicdrive.net/_hd/content/documents1/CSG-CSF_Component.pdf   (TI-26108 REV 1, 08/2026)
    https://www.harmonicdrive.net/_hd/content/documents1/SHG-SHF_Component.pdf   (TI-26121 REV 1, 08/2026)

then, with both text files in one folder (xpdf 4.x):

    pdftotext -table -enc UTF-8 CSG-CSF_Component.pdf txt/CSG-CSF_Component.txt
    pdftotext -table -enc UTF-8 SHG-SHF_Component.pdf txt/SHG-SHF_Component.txt
    python tools/extract_harmonicdrive.py txt harmonicdrive

Tables read (page numbers as printed): CSG-CSF rating tables 038-1 (CSG) and 038-2/039-1 (CSF), dimensions 041-1
(wave generator bore and mass, common to CSG and CSF), hysteresis loss 042-2, torsional stiffness 042-4 (ratio 30 and 50)
and 043-1 (ratio 80 and more); SHG-SHF rating table 082-1, dimensions 085-1, hysteresis 086-2, stiffness 086-4 and 087-1.
Rows: one per size and ratio of the rating table. A rating row with the wrong number of values, a size missing from a
per-size table, or a table not found stops the script.
"""
import csv
import re
import sys
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

COLUMNS = ["series", "size", "motor_frame", "clamping_hub_diameter_m", "stages", "ratio", "rated_output_torque_nm",
           "max_output_torque_nm", "emergency_stop_output_torque_nm", "rated_input_speed_rad_s",
           "max_input_speed_rad_s", "inertia_at_input_kgm2", "mass_kg", "torsional_stiffness_nm_rad",
           "stiffness_t1_nm", "stiffness_t2_nm", "stiffness_k1_nm_rad", "stiffness_k2_nm_rad", "stiffness_k3_nm_rad",
           "hysteresis_rad"]
NUMBER = re.compile(r"^\d+(\.\d+)?$")
DASH = "—"

# series: (text file, rating table ids (a table may run over two pages), dimensions, hysteresis, stiffness 30/50, 80+)
DOCS = {
    "CSG": ("CSG-CSF_Component.txt", ["Table 038-1"], "Table 041-1", "Table 042-2", "Table 042-4", "Table 043-1"),
    "CSF": ("CSG-CSF_Component.txt", ["Table 038-2", "Table 039-1"], "Table 041-1", "Table 042-2", "Table 042-4",
            "Table 043-1"),
    "SHG": ("SHG-SHF_Component.txt", ["Table 082-1"], "Table 085-1", "Table 086-2", "Table 086-4", "Table 087-1"),
}


def scaled(text, power):
    """`text` times 10**power, as decimal text (as cataloglib.units.to_si writes it)."""
    return "" if text is None else format((Decimal(text) * Decimal(10) ** power).normalize(), "f")


def find(lines, marker, start=0):
    for i in range(start, len(lines)):
        if marker in " ".join(lines[i].split()):  # pdftotext sets "Table  086-2" with two blanks
            return i
    raise ValueError(f"{marker} not found")


def numbers(line):
    return [t for t in line.split() if NUMBER.match(t)]


def rating_rows(lines, tables):
    """Rating table rows: (size, ratio, rated, repeated peak, momentary peak, speeds and inertia of the size)."""
    raw = []
    for n, table in enumerate(tables):
        i = find(lines, table) + 1
        follow = tables[n + 1] if n + 1 < len(tables) else None  # the table runs on over the next page
        while not lines[i].lstrip().startswith("(Note)") and not (follow and follow in lines[i]):
            if "Table 0" in lines[i]:
                raise ValueError(f"{table}: ran into {lines[i].strip()}")
            vals = numbers(lines[i])
            if len(vals) in (9, 16):
                raw.append(vals)
            elif not (len(vals) == 0 or (len(vals) == 1 and lines[i].strip() == vals[0])):  # empty or page number
                raise ValueError(f"{table}: {len(vals)} values in '{lines[i].strip()}'")
            i += 1
    # A size's ratios rise; the middle row carries the size, the speeds and the inertia.
    groups, last = [], None
    for vals in raw:
        ratio = float(vals[1] if len(vals) == 16 else vals[0])
        if last is None or ratio <= last:
            groups.append([])
        groups[-1].append(vals)
        last = ratio
    rows = []
    for group in groups:
        heads = [v for v in group if len(v) == 16]
        if len(heads) != 1:
            raise ValueError(f"a size with {len(heads)} size rows: {group}")
        head = heads[0]
        size, speeds = head[0], head[10:16]  # oil max, grease max, oil average, grease average, I, J
        for vals in group:
            v = vals[1:10] if len(vals) == 16 else vals
            # ratio, rated Nm, kgfm, repeated peak Nm, kgfm, average Nm, kgfm, momentary peak Nm, kgfm
            rows.append({"size": size, "ratio": v[0], "rated": v[1], "repeated": v[3], "average": v[5],
                         "momentary": v[7], "speeds": speeds})
    return rows


def size_header(lines, i, table):
    """The sizes of the 'Size ...' line at or after line i (within 4 lines); '40 or more' covers all larger sizes."""
    for j in range(i, i + 5):
        m = re.search(r"\bSize\b(.*)", lines[j])
        if m and numbers(m[1]):
            return numbers(m[1]), "or more" in m[1], j
    raise ValueError(f"{table}: no size header")


def values_after(line, label, n, where):
    tail = line.split(label, 1)[1]
    vals = [None if t == DASH else t for t in tail.split() if NUMBER.match(t) or t == DASH][:n]
    if len(vals) != n:
        raise ValueError(f"{where}: {len(vals)} values for {n} sizes in '{line.strip()}'")
    return vals


def per_size(lines, table, labels, until):
    """{label: {size: value}} of a table with one column per size; a label matching several lines gives a list."""
    i = find(lines, table)
    sizes, _, i = size_header(lines, i, table)
    end = find(lines, until, i + 1)
    out = {}
    for label in labels:
        hits = [values_after(lines[j], label, len(sizes), f"{table} {label}") for j in range(i + 1, end)
                if label in lines[j]]
        if not hits:
            raise ValueError(f"{table}: no line '{label}'")
        out[label] = [dict(zip(sizes, h)) for h in hits]
    return out


def hysteresis(lines, table):
    """{(size, ratio class 30/50/80): value in 1e-4 rad}, rows in the order 30, 50, 80 or more."""
    i = find(lines, table)
    sizes, or_more, i = size_header(lines, i, table)
    end = find(lines, "Backlash", i)
    rows = [values_after(lines[j], "×10-4rad", len(sizes), table) for j in range(i, end)
            if "×10-4rad" in lines[j]]
    if len(rows) != 3 or not or_more:
        raise ValueError(f"{table}: {len(rows)} rows, expected 30, 50, 80 or more")
    return sizes, {cls: dict(zip(sizes, r)) for cls, r in zip((30, 50, 80), rows)}


def stiffness(lines, table, n_blocks):
    """T1, T2 per size and n_blocks of (K1, K2, K3) per size, in printed units."""
    i = find(lines, table)
    sizes, _, i = size_header(lines, i, table)
    t1 = t2 = None
    ks = []
    for j in range(i + 1, len(lines)):
        if "reference values" in lines[j]:
            break
        if re.search(r"\bT1\s+Nm\b", lines[j]):
            t1 = dict(zip(sizes, values_after(lines[j], "Nm", len(sizes), table)))
        elif re.search(r"\bT2\s+Nm\b", lines[j]):
            t2 = dict(zip(sizes, values_after(lines[j], "Nm", len(sizes), table)))
        elif "×104Nm/rad" in lines[j]:  # the K3 line of SHG's ratio 50 has its label on the next line
            ks.append(dict(zip(sizes, values_after(lines[j], "×104Nm/rad", len(sizes), table))))
    if t1 is None or t2 is None or len(ks) != 3 * n_blocks:
        raise ValueError(f"{table}: T1/T2 missing or {len(ks)} K lines for {n_blocks} ratio blocks")
    return t1, t2, [ks[3 * b:3 * b + 3] for b in range(n_blocks)]


def need(table, size, where):
    if size not in table:
        raise ValueError(f"{where}: no size {size}")
    return table[size]


def extract(series, folder):
    name, ratings, dims, hyst, stiff_low, stiff_high = DOCS[series]
    lines = (folder / name).read_text(encoding="utf-8").splitlines()
    rows = rating_rows(lines, ratings)
    dim = per_size(lines, dims, ["Standard (H7)", "Max. size", "Mass (kg)"], "Positional")
    bore, bore_max, mass = dim["Standard (H7)"][0], dim["Max. size"][0], dim["Mass (kg)"][0]
    h_sizes, hyst_by = hysteresis(lines, hyst)
    t1, t2, (k30, k50) = stiffness(lines, stiff_low, 2)
    _, _, (k80,) = stiffness(lines, stiff_high, 1)
    out = []
    for r in rows:
        size, ratio = r["size"], r["ratio"]
        where = f"{series}{size} ratio {ratio}"
        cls = 30 if float(ratio) < 50 else 50 if float(ratio) < 80 else 80
        h_size = size if size in h_sizes else h_sizes[-1]  # the last column is "40 or more"
        k = {30: k30, 50: k50, 80: k80}[cls]
        k1, k2, k3 = (need(kk, size, where) for kk in k)
        if None in (k1, k2, k3):
            raise ValueError(f"{where}: no stiffness printed")
        oil_max, grease_max, oil_avg, grease_avg, inertia, _ = r["speeds"]
        standard, largest = need(bore, size, where), need(bore_max, size, where)
        out.append({
            "series": series, "size": f"{series}{size}", "motor_frame": standard,
            "clamping_hub_diameter_m": to_si(largest or "", "milli"), "stages": "1", "ratio": ratio,
            "rated_output_torque_nm": r["rated"], "max_output_torque_nm": r["repeated"],
            "emergency_stop_output_torque_nm": r["momentary"],
            "rated_input_speed_rad_s": to_si(grease_avg, "rpm"), "max_input_speed_rad_s": to_si(grease_max, "rpm"),
            "inertia_at_input_kgm2": scaled(inertia, -4), "mass_kg": need(mass, size, where),
            "torsional_stiffness_nm_rad": scaled(k3, 4), "stiffness_t1_nm": need(t1, size, where),
            "stiffness_t2_nm": need(t2, size, where), "stiffness_k1_nm_rad": scaled(k1, 4),
            "stiffness_k2_nm_rad": scaled(k2, 4), "stiffness_k3_nm_rad": scaled(k3, 4),
            "hysteresis_rad": scaled(need(hyst_by[cls], h_size, where), -4),
        })
    keys = [(o["size"], o["ratio"]) for o in out]
    if len(set(keys)) != len(keys):
        raise ValueError(f"{series}: a size and ratio twice")
    return out


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: extract_harmonicdrive.py <folder with the pdftotext files> <output folder>")
    folder, target = Path(sys.argv[1]), Path(sys.argv[2])
    target.mkdir(parents=True, exist_ok=True)
    for series in DOCS:
        rows = extract(series, folder)
        with open(target / f"gearbox_{series.lower()}.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=COLUMNS, lineterminator="\n")
            w.writeheader()
            w.writerows(rows)
        print(f"{series}: {len(rows)} rows")


if __name__ == "__main__":
    main()
