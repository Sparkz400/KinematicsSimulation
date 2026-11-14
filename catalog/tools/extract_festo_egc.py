"""Extracts the Festo toothed-belt axes EGC-TB-KF, ELGA-TB-KF and ELGA-TB-G (single axes) from Festo's catalogues
into catalog/festo/belt_axes_egc.csv and catalog/festo/belt_axes_elga.csv.

The catalogues are not part of this repository. Get them from Festo:

    https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2026/EN/EGC-TB_EN.PDF    (2026/06)
    https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2026/EN/ELGA-TB_EN.PDF   (2026/02)

then

    pdftotext -table -enc UTF-8 EGC-TB_EN.PDF egc.txt         (xpdf 4.06)
    pdftotext -table -enc UTF-8 ELGA-TB_EN.PDF elga.txt
    python -I extract_festo_egc.py egc.txt elga.txt ../festo

Each data sheet has tables with one column per size. A row whose values do not fit the sizes stops the script. Three
rows are merged cells (one value printed across several sizes): a value holds from its own column up to the next
printed value, found by the text column it starts in.
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib.units import to_si  # noqa: E402

HEADER = ["series", "size", "kind", "belt_material", "belt_pitch_m", "belt_width_m", "belt_elongation",
          "belt_elongation_force_n", "pulley_effective_diameter_m", "feed_per_rev_m", "pulley_teeth",
          "max_no_load_torque_nm", "max_pulley_torque_nm", "feed_force_peak_n", "max_speed_m_s",
          "max_acceleration_m_s2", "repeatability_m", "fy_dynamic_n", "fz_dynamic_n", "mx_dynamic_nm", "my_dynamic_nm",
          "mz_dynamic_nm", "mass_at_zero_stroke_kg", "mass_per_stroke_kg_m", "moved_mass_x_kg", "stroke_x_min_m",
          "stroke_x_max_m"]
# The standard belt; the -PU2 option (polyurethane with steel cord) is not a row.
BELT = "polychloroprene or NBR with glass cord and nylon coating"
VALUE = re.compile(r"±?\d+(?:\.\d+)?|–")
RANGE = re.compile(r"(\d+)\s*…\s*(\d+)")


def norm(line):
    return re.sub(r"\s+", " ", line).strip()


class Section:
    """The lines of one series' data sheet, from its "General technical data" up to the next one."""

    def __init__(self, lines, series):
        tops = [i for i, l in enumerate(lines) if norm(l) == "General technical data"]
        mine = [i for i in tops if any(f"Toothed belt axes {series}, with" in norm(l) for l in lines[max(0, i - 40):i])]
        if len(mine) != 1:
            raise ValueError(f"{series}: found {len(mine)} data sheets")
        end = next((i for i in tops if i > mine[0]), len(lines))
        self.series, self.lines = series, lines[mine[0]:end]

    def table(self, title):
        """The line index of the table whose title line matches the pattern `title` and its sizes with the column each starts at."""
        at = next((i for i, l in enumerate(self.lines) if re.fullmatch(title, norm(l))), None)
        if at is None:
            raise ValueError(f"{self.series}: no table {title!r}")
        size_line = next(l for l in self.lines[at:] if norm(l).startswith("Size "))
        cols = [(m.group(), m.start()) for m in re.finditer(r"\b\d+\b", size_line)]
        return at, cols

    def row(self, title, label, after=None, merged=False, ranges=False):
        """The values of the row `label` (below the line `after`, when given) in table `title`, one per size."""
        at, cols = self.table(title)
        if after is not None:
            at = next((i for i in range(at, len(self.lines)) if norm(self.lines[i]).startswith(after)), None)
            if at is None:
                raise ValueError(f"{self.series}: {title}: no line {after!r}")
        line = next((l for l in self.lines[at:] if norm(l).startswith(label)), None)
        if line is None:
            raise ValueError(f"{self.series}: {title}: no row {label!r}")
        start = line.index(label.split()[-1]) + len(label.split()[-1])
        text = line[start:]
        unit = re.match(r"\S*\s*(\[[^\]]*\])?", text)  # a footnote mark and the unit after the label
        offset = start + unit.end()
        text = line[offset:]
        if ranges:
            values = RANGE.findall(text)
            if len(values) != len(cols):
                raise ValueError(f"{self.series}: {label}: {len(values)} ranges for {len(cols)} sizes: {text!r}")
            return values
        found = [(m.group(), offset + m.start()) for m in VALUE.finditer(text)]
        if not merged:
            if len(found) != len(cols):
                raise ValueError(f"{self.series}: {label}: {len(found)} values for {len(cols)} sizes: {text!r}")
            return [v for v, _ in found]
        # Merged cells: each value belongs to the size column it starts nearest to and holds up to the next value.
        values = [None] * len(cols)
        for v, x in found:
            nearest = min(range(len(cols)), key=lambda k: abs(cols[k][1] - x))
            if abs(cols[nearest][1] - x) > 4 or values[nearest] is not None:
                raise ValueError(f"{self.series}: {label}: value {v} at column {x} fits no size")
            values[nearest] = v
        if values[0] is None:
            raise ValueError(f"{self.series}: {label}: no value for the first size")
        for k in range(1, len(values)):
            values[k] = values[k] if values[k] is not None else values[k - 1]
        return values


def si(value, conversion):
    return "" if value == "–" else to_si(value.lstrip("±"), conversion)


def teeth(feed_mm, pitch_mm):
    """The pulley's tooth count: the feed constant over the pitch, both printed; it must be a whole number."""
    q = float(feed_mm) / float(pitch_mm)
    if abs(q - round(q)) > 1e-9:
        raise ValueError(f"feed constant {feed_mm} over pitch {pitch_mm} is no whole number")
    return str(round(q))


def egc(lines):
    s = Section(lines, "EGC-TB-KF")
    g, w, b, ld = "General technical data", r"Weight \[g\]", "Toothed belt", r"Max\. permissible forces and torques.*"
    sizes = [c for c, _ in s.table(g)[1]]
    get = {
        "stroke": s.row(g, "EGC-…-GK/-GP", after="Working stroke", ranges=True),
        "feed_force_peak_n": s.row(g, "Max. feed force Fx"),
        "max_no_load_torque_nm": s.row(g, "Max. no-load torque"),
        "max_pulley_torque_nm": s.row(g, "Max. driving torque"),
        "max_speed_m_s": s.row(g, "Max. speed", merged=True),
        "max_acceleration_m_s2": s.row(g, "Max. acceleration", merged=True),
        "repeatability_m": s.row(g, "Repetition accuracy", merged=True),
        "mass_at_zero_stroke_kg": s.row(w, "EGC-…-GK/-GP", after="Basic weight"),
        "mass_per_stroke_kg_m": s.row(w, "Additional weight per 10 mm stroke"),
        "moved_mass_x_kg": s.row(w, "EGC-…-GK/-GP", after="Moving mass"),
        "belt_pitch_m": s.row(b, "Pitch"),
        "belt_width_m": s.row(b, "Width"),
        "pulley_effective_diameter_m": s.row(b, "Effective diameter"),
        "feed_per_rev_m": s.row(b, "Feed constant"),
        "fy_dynamic_n": s.row(ld, "Fymax."),
        "fz_dynamic_n": s.row(ld, "Fzmax"),
        "mx_dynamic_nm": s.row(ld, "Mxmax."),
        "my_dynamic_nm": s.row(ld, "EGC-…-GK/-GP", after="Mymax./Mzmax."),
    }
    get["mz_dynamic_nm"] = get["my_dynamic_nm"]  # printed once as "Mymax./Mzmax."
    conv = {"mass_at_zero_stroke_kg": "milli", "moved_mass_x_kg": "milli", "repeatability_m": "milli",
            "belt_pitch_m": "milli", "belt_width_m": "milli", "pulley_effective_diameter_m": "milli",
            "feed_per_rev_m": "milli"}
    rows = []
    for k, size in enumerate(sizes):
        r = dict.fromkeys(HEADER, "")
        r.update(series="EGC-TB-KF", size=size, kind="axis", belt_material=BELT)
        for col, values in get.items():
            if col != "stroke":
                r[col] = si(values[k], conv.get(col, "same"))
        # g per 10 mm -> kg per m: g -> kg, then per 10 mm -> per m.
        r["mass_per_stroke_kg_m"] = to_si(to_si(get["mass_per_stroke_kg_m"][k], "milli"), "centi_inverse")
        r["stroke_x_min_m"], r["stroke_x_max_m"] = (to_si(v, "milli") for v in get["stroke"][k])
        r["pulley_teeth"] = teeth(get["feed_per_rev_m"][k], get["belt_pitch_m"][k])
        rows.append(r)
    return rows


def elga(lines, series):
    s = Section(lines, series)
    g, w, b = "General technical data", r"Weight \[kg\]", "Toothed belt"
    ld = r"(Max\. p|P)ermissible forces and torques.*"
    moving = dict(label="ELGA-…", after="Moving mass") if series == "ELGA-TB-KF" else dict(label="Moving mass")
    sizes = [c for c, _ in s.table(g)[1]]
    get = {
        "stroke": s.row(g, "Working stroke", ranges=True),
        "feed_force_peak_n": s.row(g, "Max. feed force Fx"),
        "max_no_load_torque_nm": s.row(g, "Max. no-load torque"),
        "max_pulley_torque_nm": s.row(g, "Max. driving torque"),
        "max_speed_m_s": s.row(g, "Max. speed", merged=True),
        "max_acceleration_m_s2": s.row(g, "Max. acceleration", merged=True),
        "repeatability_m": s.row(g, "Repetition accuracy", merged=True),
        "mass_at_zero_stroke_kg": s.row(w, "Basic weight with 0 mm stroke" + (" (including slide)" if series == "ELGA-TB-G" else "")),
        "mass_per_stroke_kg_m": s.row(w, "Additional weight per 1000 mm stroke"),
        "moved_mass_x_kg": s.row(w, **moving),
        "belt_pitch_m": s.row(b, "Pitch"),
        "belt_elongation": s.row(b, "ELGA-…", after="Elongation"),
        "pulley_effective_diameter_m": s.row(b, "Effective diameter"),
        "feed_per_rev_m": s.row(b, "Feed constant"),
        "fy_dynamic_n": s.row(ld, "Fymax."),
        "fz_dynamic_n": s.row(ld, "Fzmax"),
        "mx_dynamic_nm": s.row(ld, "Mxmax."),
        "my_dynamic_nm": s.row(ld, "Mymax."),
        "mz_dynamic_nm": s.row(ld, "Mzmax."),
    }
    conv = {"repeatability_m": "milli", "belt_pitch_m": "milli", "pulley_effective_diameter_m": "milli",
            "feed_per_rev_m": "milli", "belt_elongation": "percent"}
    rows = []
    for k, size in enumerate(sizes):
        r = dict.fromkeys(HEADER, "")
        r.update(series=series, size=size, kind="axis", belt_material=BELT)
        for col, values in get.items():
            if col != "stroke":
                r[col] = si(values[k], conv.get(col, "same"))
        # Per 1000 mm of stroke is per m.
        r["belt_elongation_force_n"] = r["feed_force_peak_n"]  # "1) At max. feed force"
        r["stroke_x_min_m"], r["stroke_x_max_m"] = (to_si(v, "milli") for v in get["stroke"][k])
        r["pulley_teeth"] = teeth(get["feed_per_rev_m"][k], get["belt_pitch_m"][k])
        rows.append(r)
    return rows


def write(path, rows):
    keys = [(r["series"], r["size"]) for r in rows]
    if len(set(keys)) != len(keys):
        raise ValueError(f"{path.name}: duplicate keys")
    cols = [c for c in HEADER if any(r[c] for r in rows)]  # no column the catalogue leaves empty throughout
    path.write_bytes((",".join(cols) + "\n" + "".join(",".join(r[c] for c in cols) + "\n" for r in rows)).encode())


def main(egc_txt, elga_txt, out):
    read = lambda p: Path(p).read_text(encoding="utf-8").replace("\f", "\n").splitlines()
    out = Path(out)
    write(out / "belt_axes_egc.csv", egc(read(egc_txt)))
    lines = read(elga_txt)
    write(out / "belt_axes_elga.csv", elga(lines, "ELGA-TB-KF") + elga(lines, "ELGA-TB-G"))


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
