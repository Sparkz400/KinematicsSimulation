"""Extracts the Bosch Rexroth compact modules with toothed belt drive CKR-070 to CKR-280 (single axes) from the
catalogue "Compact modules CKK/CKR" (R999000499/2025-05) into catalog/rexroth/belt_axes_ckr.csv.

The catalogue is not part of this repository. Get it from Bosch Rexroth:

    https://www.boschrexroth.com/media/e8278bcd-f4c8-4191-814e-9e557f6c5f27

then (pages 64 and 65, "Technical data" of the CKR; the raw text order keeps each table's rows together)

    pdftotext -raw -enc UTF-8 -f 64 -l 65 ck.pdf ckr_raw.txt         (xpdf 4.06)
    python -I extract_rexroth_ckr.py ckr_raw.txt ../rexroth

Every size but 280 is printed with two carriage lengths: the short one is size "070", the long one "070L". Each table
is read as the sequence of its numbers; a table with a different count of numbers stops the script.
"""
import re
import sys
from decimal import Decimal
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

SIZES = ["070", "090", "110", "145", "200", "280"]
HEADER = ["series", "size", "kind", "belt", "belt_pitch_m", "belt_width_m", "pulley_effective_diameter_m",
          "feed_per_rev_m", "pulley_teeth", "no_load_torque_nm", "max_pulley_torque_nm", "feed_force_peak_n",
          "max_speed_m_s", "max_acceleration_m_s2", "fy_dynamic_n", "fz_dynamic_n", "mx_dynamic_nm", "my_dynamic_nm",
          "mz_dynamic_nm", "mass_at_zero_stroke_kg", "mass_per_stroke_kg_m", "moved_mass_x_kg", "stroke_x_min_m",
          "max_length_one_piece_m"]
# "5, 500" (Lmax of the 280) has a blank after the thousands comma.
TOKEN = re.compile(r"AT\d+|\d{1,3}(?:,\s?\d{3})+(?:\.\d+)?|\d+(?:\.\d+)?")


def tokens(lines):
    return [t.replace(",", "").replace(" ", "") for l in lines for t in TOKEN.findall(l)]


def between(lines, start, stop):
    """The lines after the first line that is `start` up to the next line starting with `stop`."""
    try:
        i = lines.index(start)
    except ValueError:
        raise ValueError(f"no line {start!r}") from None
    j = next((k for k in range(i + 1, len(lines)) if lines[k].startswith(stop)), None)
    if j is None:
        raise ValueError(f"no line {stop!r} after {start!r}")
    return lines[i + 1:j]


def by_size(lines):
    """The lines of a table split at the size marks "-070" ... "-280" that open a row group."""
    groups, size = {}, None
    for l in lines:
        m = re.match(r"-(\d{3})\b ?(.*)", l)
        if m:
            size, l = m.group(1), m.group(2)
            groups[size] = []
        if size is not None:
            groups[size].append(l)
    if list(groups) != SIZES:
        raise ValueError(f"sizes {list(groups)}, expected {SIZES}")
    return groups


def expect(values, count, what):
    if len(values) != count:
        raise ValueError(f"{what}: {len(values)} numbers, expected {count}: {values}")
    return values


def extract(lines):
    lines = [l.strip() for l in lines]
    keys = [k for s in SIZES for k in ([s, s + "L"] if s != "280" else [s])]
    rows = {k: dict.fromkeys(HEADER, "") | {"series": "CKR", "size": k, "kind": "axis"} for k in keys}
    carriage = {}  # key -> (Lca without, Lca with, Lad without)

    # General technical data: Lca Lca Lad Lad smin Lmax Cgw Mt ML (short), Lca Lca Lad Lad Cgw Mt ML (long).
    general = by_size(between(lines, "(mm) (mm) (mm) (mm) (mm) (mm) (N) (Nm) (Nm)", "Technical data"))
    for s, group in general.items():
        t = expect(tokens(group), 9 if s == "280" else 16, f"general technical data {s}")
        carriage[s] = (t[0], t[1], t[2])
        rows[s].update(stroke_x_min_m=to_si(t[4], "milli"), max_length_one_piece_m=to_si(t[5], "milli"))
        if s != "280":
            carriage[s + "L"] = (t[9], t[10], t[11])
            rows[s + "L"].update(stroke_x_min_m=to_si(t[4], "milli"), max_length_one_piece_m=to_si(t[5], "milli"))

    # Drive data: the row with gear ratio 1 (no gear unit), and the moved mass mca per carriage. Misprint kept: the
    # drive data give the 110's carriage with connection plate as 60 mm long, the general data as 110 mm.
    drive = by_size(between(lines, "(–) (Nm) (mm/rev) (m/s) (mm) (mm) (kg) (kg)", "Observe"))
    for s, group in drive.items():
        m = re.search(r"(?:–|ohne) 1 (\S+) (\S+) (\S+)", " ".join(group))  # 280 prints German "ohne" for "–"
        if not m:
            raise ValueError(f"drive data {s}: no row with ratio 1")
        mp, u, v = m.groups()
        for k in ([s, s + "L"] if s != "280" else [s]):
            lca, lca_plate, _ = carriage[k]
            hit = [x for l in group if (x := re.search(rf"(?:^|\s){lca} (\d+) (\d+\.\d+) (\d+\.\d+)$", l))]
            if len(hit) != 1:
                raise ValueError(f"drive data {k}: {len(hit)} rows with carriage {lca}")
            if hit[0].group(1) != lca_plate and (k, hit[0].group(1)) != ("110", "60"):
                raise ValueError(f"drive data {k}: carriage with plate {hit[0].group(1)}, general data {lca_plate}")
            rows[k].update(max_pulley_torque_nm=mp, feed_per_rev_m=to_si(u, "milli"), max_speed_m_s=v,
                           moved_mass_x_kg=hit[0].group(2))

    # Maximum permissible loads: Mx My Mz Fy Fz1 Fz2 per carriage, in the order of `keys`; the planar moments of
    # inertia (4 numbers) stand on their own lines, except for the 280.
    loads = [tokens([l]) for l in between(lines, "(Nm) (Nm) (Nm) (N) (N) (N) (cm4) (cm4) (mm) (mm)", "Constant mass")]
    loads = [t[:6] for t in loads if len(t) in (6, 10)]
    expect(loads, len(keys), "maximum permissible loads (rows)")
    for k, (mx, my, mz, fy, fz1, fz2) in zip(keys, loads):
        # One Fz column: the lower of Fz1 and Fz2 (they differ only for the 280).
        rows[k].update(mx_dynamic_nm=mx, my_dynamic_nm=my, mz_dynamic_nm=mz, fy_dynamic_n=fy,
                       fz_dynamic_n=min(fz1, fz2, key=Decimal))

    # Mass, inertia, friction and belt: kg fix, kg var, kJ fix (2), kJ var, kJ m, MRs (short), d3, Bt, belt, Fbp,
    # Ft perm, then kJ fix (2), MRs (long); amax (one merged cell for all sizes) follows the first belt row.
    t = tokens(between(lines, "(kg) (kg/mm) (kg/mm2) (kg/mm2) (kg/mm) (mm2) (Nm) (mm) (N) (N) (m/s2)", "5)"))
    expect(t, 5 * 15 + 1 + 12, "mass and belt table")
    amax = t.pop(12)
    if amax != "50":
        raise ValueError(f"max. acceleration {amax}, expected the merged cell 50")
    for i, s in enumerate(SIZES):
        g = t[15 * i:15 * i + 15] if s != "280" else t[75:]
        if not re.fullmatch(r"AT\d+", g[9]):
            raise ValueError(f"belt table {s}: {g}")
        for k, mrs in ([(s, g[6]), (s + "L", g[14])] if s != "280" else [(s, g[6])]):
            lca, _, lad = carriage[k]
            # ms = kg fix + kg var x L + mca, L = smax + Lca + Lad (stroke reserve 0), at smax = 0.
            zero = Decimal(g[0]) + Decimal(g[1]) * (Decimal(lca) + Decimal(lad)) + Decimal(rows[k]["moved_mass_x_kg"])
            pitch = g[9][2:]
            teeth = Decimal(rows[k]["feed_per_rev_m"]) * 1000 / Decimal(pitch)
            if teeth != teeth.to_integral_value():
                raise ValueError(f"{k}: feed constant over belt pitch {teeth} is no whole number")
            rows[k].update(no_load_torque_nm=mrs, pulley_effective_diameter_m=to_si(g[7], "milli"),
                           belt_width_m=to_si(g[8], "milli"), belt=g[9], belt_pitch_m=to_si(pitch, "milli"),
                           feed_force_peak_n=g[10], max_acceleration_m_s2=amax,
                           mass_per_stroke_kg_m=to_si(g[1], "kilo"), mass_at_zero_stroke_kg=format(zero.normalize(), "f"),
                           pulley_teeth=str(int(teeth)))
    return list(rows.values())


def main(raw_txt, out):
    rows = extract(Path(raw_txt).read_text(encoding="utf-8").replace("\f", "\n").splitlines())
    text = ",".join(HEADER) + "\n" + "".join(",".join(r[c] for c in HEADER) + "\n" for r in rows)
    (Path(out) / "belt_axes_ckr.csv").write_bytes(text.encode())


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(*sys.argv[1:])
