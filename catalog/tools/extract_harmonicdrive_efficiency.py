"""Reads the efficiency graphs of Harmonic Drive's CSG/CSF and SHG component sets off the PDFs' vector paths and
writes tools/harmonicdrive_efficiency_readings.csv; with `--apply` it also fills the column efficiency_full_load of
catalog/harmonicdrive/gearbox_{csf,csg,shg}.csv.

    python -I tools/extract_harmonicdrive_efficiency.py CSG-CSF_Component.pdf SHG-SHF_Component.pdf [--apply]

Run it from catalog/ with the two PDFs (TI-26108 REV 1 and TI-26121 REV 1, URLs in extract_harmonicdrive.py) kept in a
folder of their own.

The graphs ("Efficiency at rated torque", Graphs 047-1 to 047-8 on printed page 47 and 090-2 to 091-5 on pages 90 and 91)
are vector art: a frame, grid lines, four curves (500, 1000, 2000, 3500 r/min input speed) of efficiency against
ambient temperature (-10 to 40 degC). Method: the frame and its grid lines give the axes (checked against the printed
tick labels, which must agree within one unit), the curves are the path pieces of the curves' colour inside the frame,
chained into four curves; they are told apart by their order at every temperature (they must not cross) and checked
against the legend labels; the value is read at 2000 r/min and 25 degC (40 degC where the curve reaches it). The two
anchors printed in the documents' calculation examples must come out within 2 points: CSF-20-80 at 1000 r/min and
20 degC 78 %, SHF-20-80 82 %. Else the script stops.
"""
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib import pdfvec  # noqa: E402

READINGS = Path(__file__).resolve().parent / "harmonicdrive_efficiency_readings.csv"
SPEEDS = (500, 1000, 2000, 3500)
CURVE_RGB = (0.218, 0.258, 0.354)

# document: (pdf page index (0-based), graph numbers in reading order (rows top to bottom, left to right), y axis start,
#            frame line width, series, (graph number -> (size group, ratios)))
DOCS = {
    "CSG-CSF": {
        "pages": [(16, [f"047-{n}" for n in range(1, 9)])], "y0": 20, "frame_w": 0.23,
        "graphs": {"047-1": ("8-14", [30]), "047-2": ("8-14", [50, 80]), "047-3": ("8-14", [100]),
                   "047-4": ("17-100", [30]), "047-5": ("17-100", [50]), "047-6": ("17-100", [80, 100]),
                   "047-7": ("17-100", [120]), "047-8": ("17-100", [160])},
        "series": ("CSF", "CSG"), "anchor": ("047-6", 0.78, "CSF-20-80"),
    },
    "SHG-SHF": {
        "pages": [(15, ["090-2", "090-3", "090-4"]), (16, [f"091-{n}" for n in range(1, 6)])], "y0": 10,
        "frame_w": 0.48,
        "graphs": {"090-2": ("14", [30]), "090-3": ("14", [50, 80]), "090-4": ("14", [100]),
                   "091-1": ("17-65", [30]), "091-2": ("17-65", [50]), "091-3": ("17-65", [80, 100]),
                   "091-4": ("17-65", [120]), "091-5": ("17-65", [160])},
        "series": ("SHG",), "anchor": ("091-3", 0.82, "SHF-20-80"),
    },
}
HEADER = ["document", "graph", "series", "size_group", "ratio", "efficiency_2000rpm_25C", "efficiency_2000rpm_40C",
          "efficiency_1000rpm_20C", "reading_error_points"]
READING_ERROR = 1.0  # percentage points: the larger of the two anchor residuals (CSF 1.1, SHF 0.4) rounded; see the meta files


def frames(paths, width):
    """Plot frames (xmin, xmax, ymin, ymax) of the page, rows top to bottom, left to right."""
    out = []
    for p in paths:
        if p["kind"] == "S" and abs(p["w"] - width) < 0.02 and len(p["subs"]) == 1 and len(p["subs"][0]) == 5:
            xs, ys = [q[0] for q in p["subs"][0]], [q[1] for q in p["subs"][0]]
            if 80 < max(xs) - min(xs) < 120 and 100 < max(ys) - min(ys) < 180:
                out.append((min(xs), max(xs), min(ys), max(ys)))
    return sorted(out, key=lambda f: (-round(f[3]), f[0]))


def axes(fr, paths, texts, y0, name):
    """(temperature of x, efficiency of y) in the frame; the grid lines and the tick labels must agree with them."""
    x0, x1, ya, yb = fr
    temp = lambda x: -10 + (x - x0) / (x1 - x0) * 50  # noqa: E731
    eff = lambda y: y0 + (y - ya) / (yb - ya) * (100 - y0)  # noqa: E731
    grid_x, grid_y = [], []
    for p in paths:
        pts = p["subs"][0]
        if p["kind"] == "S" and len(p["subs"]) == 1 and len(pts) == 2 and abs(p["rgb"][0]) < 0.01:
            (xa, ya_), (xb, yb_) = pts
            if abs(ya_ - yb_) < 0.01 and x0 - 0.3 < min(xa, xb) and max(xa, xb) < x1 + 0.3 and ya < ya_ < yb:  # (a line may be cut around a label)
                grid_y.append(ya_)
            elif abs(xa - xb) < 0.01 and x0 < xa < x1 and abs(min(ya_, yb_) - ya) < 0.2 and abs(max(ya_, yb_) - yb) < 0.2:
                grid_x.append(xa)
    if sorted(round(temp(x)) for x in grid_x) != [0, 10, 20, 30]:
        raise ValueError(f"{name}: x grid lines do not sit at 0, 10, 20, 30 degC")
    if any(abs(temp(x) - round(temp(x))) > 0.1 for x in grid_x):
        raise ValueError(f"{name}: x grid lines off the 10 degC steps")
    want = list(range(y0 + 10, 100, 10))
    grid_y = sorted({round(eff(y), 1) for y in grid_y})
    if [round(y) for y in grid_y] != want or any(abs(y - round(y)) > 0.1 for y in grid_y):
        raise ValueError(f"{name}: y grid lines are not {want}")
    # the tick labels left of the frame: value against baseline must lie on one straight line parallel to the axis
    labels = [(int(t[0]), t[2]) for t in texts if re.fullmatch(r"\d+", t[0]) and x0 - 25 < t[1] < x0
              and ya - 4 < t[2] < yb + 1 and y0 <= int(t[0]) <= 100]
    # (the lowest label is missing from a few of the printed graphs; the grid lines above cover it)
    if sorted(v for v, _ in labels)[-9:][-(len(labels)):] not in (list(range(y0, 101, 10)), list(range(y0 + 10, 101, 10))):
        raise ValueError(f"{name}: y tick labels {sorted(v for v, _ in labels)}")
    offsets = [y - (ya + (v - y0) / (100 - y0) * (yb - ya)) for v, y in labels]
    if max(offsets) - min(offsets) > 1.0 * (yb - ya) / (100 - y0):  # one unit: the labels are placed by hand
        raise ValueError(f"{name}: y tick labels do not follow the frame's scale (offsets {offsets})")
    return temp, eff


def curves(paths, fr):
    """The four curves of a frame as lists of (x, y) sorted by x: path pieces in the curves' colour, chained."""
    x0, x1, ya, yb = fr
    segs = []
    for p in paths:
        if p["kind"] == "S" and all(abs(a - b) < 0.01 for a, b in zip(p["rgb"], CURVE_RGB)) and 0.4 < p["w"] < 1:
            for sub in p["subs"]:
                if all(x0 - 0.5 <= q[0] <= x1 + 0.5 and ya - 0.5 <= q[1] <= yb + 0.5 for q in sub):
                    segs += list(zip(sub, sub[1:]))
    parent = list(range(len(segs)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    ends = [(i, q) for i, s in enumerate(segs) for q in s]
    for n, (i, a) in enumerate(ends):
        for j, b in ends[n + 1:]:
            if abs(a[0] - b[0]) < 0.02 and abs(a[1] - b[1]) < 0.02:
                parent[root(i)] = root(j)
    groups = {}
    for i, s in enumerate(segs):
        groups.setdefault(root(i), []).extend(s)
    return [sorted(set(pts)) for pts in groups.values()]


def at(curve, x):
    for (xa, ya), (xb, yb) in zip(curve, curve[1:]):
        if xa <= x <= xb and xb > xa:
            return ya + (yb - ya) * (x - xa) / (xb - xa)
    return None


def read_graph(paths, texts, fr, y0, name):
    """{speed: {(temperature): efficiency in %}} for the four curves of one graph."""
    temp, eff = axes(fr, paths, texts, y0, name)
    cs = curves(paths, fr)
    if len(cs) != 4:
        raise ValueError(f"{name}: {len(cs)} curves found, expected 4")
    xs = lambda t: fr[0] + (t + 10) / 50 * (fr[1] - fr[0])  # noqa: E731
    # the order of the curves must be the same at every temperature they share (no crossing)
    shared = [t for t in range(-10, 41) if all(at(c, xs(t) - 1e-6 if t == 40 else xs(t)) is not None for c in cs)]
    orders = {tuple(sorted(range(4), key=lambda i: -at(cs[i], xs(t) - 1e-6 if t == 40 else xs(t)))) for t in shared}
    if len(orders) != 1 or len(shared) < 20:
        raise ValueError(f"{name}: the curves cross or share too little range ({len(shared)} degC, {orders})")
    ranked = [cs[i] for i in orders.pop()]  # highest efficiency first = 500 r/min first (checked below)
    # (one label has a stray letter after "rpm"; the CSG/CSF legends split the number from "rpm")
    labs = sorted((t for t in texts if re.fullmatch(r"(500|1000|2000|3500)(rpm.?)?", t[0]) and fr[0] - 5 < t[1] < fr[1] + 25
                   and fr[2] < t[2] < fr[3]), key=lambda t: -t[2])
    if [int(re.match(r"\d+", t[0])[0]) for t in labs] != list(SPEEDS):
        raise ValueError(f"{name}: legend labels {[t[0] for t in labs]}")
    for (label, lx, ly, _), c in zip(labs, ranked):  # the labels are stacked in speed order beside the curves' ends (CSG/CSF) or on them (SHG/SHF): rank, within 8 points
        near = min(c, key=lambda q: abs(q[0] - lx) + abs(q[1] - ly))
        if abs(eff(near[1]) - eff(ly)) > 8:
            raise ValueError(f"{name}: label {label} is not next to the curve of its rank")
    out = {}
    for speed, c in zip(SPEEDS, ranked):
        out[speed] = {t: (eff(at(c, xs(t) - 1e-6 if t == 40 else xs(t)))
                          if at(c, xs(t) - 1e-6 if t == 40 else xs(t)) is not None else None) for t in (20, 25, 40)}
    return out


def readings(pdfs):
    rows = []
    for doc, cfg in DOCS.items():
        data, objs = pdfvec.load(pdfs[doc])
        page_list = pdfvec.pages(data, objs)
        found = {}
        for index, names in cfg["pages"]:
            paths, texts = pdfvec.interpret(pdfvec.content(objs, page_list[index]))
            frs = frames(paths, cfg["frame_w"])
            if len(frs) != len(names):
                raise ValueError(f"{doc} page {index + 1}: {len(frs)} graph frames, expected {len(names)}")
            for name, fr in zip(names, frs):
                found[name] = read_graph(paths, texts, fr, cfg["y0"], f"Graph {name}")
        graph, printed, label = cfg["anchor"]
        got = found[graph][1000][20] / 100
        print(f"anchor {label}: Graph {graph}, 1000 r/min, 20 degC: read {got * 100:.1f} %, printed {printed * 100:.0f} %")
        if abs(got - printed) > 0.02:
            raise ValueError(f"anchor {label} off by {abs(got - printed) * 100:.1f} points: the axis mapping is wrong")
        for name, (group, ratios) in cfg["graphs"].items():
            g = found[name]
            for ratio in ratios:
                for series in cfg["series"]:
                    e40 = g[2000][40]
                    rows.append({"document": doc, "graph": name, "series": series, "size_group": group, "ratio": ratio,
                                 "efficiency_2000rpm_25C": f"{g[2000][25] / 100:.3f}",
                                 "efficiency_2000rpm_40C": "" if e40 is None else f"{e40 / 100:.3f}",
                                 "efficiency_1000rpm_20C": f"{g[1000][20] / 100:.3f}",
                                 "reading_error_points": READING_ERROR})
    return rows


def size_group(series, size):
    n = int(size[len(series):])
    return ("14" if n <= 14 else "17-65") if series == "SHG" else ("8-14" if n <= 14 else "17-100")


def efficiency_for(series, size, ratio):
    """Efficiency at rated torque, 2000 r/min, 25 degC, grease (0 to 1) as text, or "" where no graph covers the row."""
    if not READINGS.exists():
        return ""
    for r in csv.DictReader(open(READINGS, encoding="utf-8")):
        if r["series"] == series and r["size_group"] == size_group(series, size) and r["ratio"] == str(ratio):
            return r["efficiency_2000rpm_25C"]
    return ""


def apply(folder):
    for series in ("CSF", "CSG", "SHG"):
        path = folder / f"gearbox_{series.lower()}.csv"
        rows = list(csv.DictReader(open(path, encoding="utf-8")))
        names = list(rows[0].keys())
        if "efficiency_full_load" not in names:
            names.append("efficiency_full_load")
        for r in rows:
            r["efficiency_full_load"] = efficiency_for(series, r["size"], r["ratio"])
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=names, lineterminator="\n")
            w.writeheader()
            w.writerows(rows)
        print(f"{series}: {sum(1 for r in rows if r['efficiency_full_load'])} of {len(rows)} rows with an efficiency")


def main():
    args = [a for a in sys.argv[1:] if a != "--apply"]
    if len(args) != 2:
        sys.exit("usage: extract_harmonicdrive_efficiency.py <CSG-CSF_Component.pdf> <SHG-SHF_Component.pdf> [--apply]")
    rows = readings({"CSG-CSF": args[0], "SHG-SHF": args[1]})
    with open(READINGS, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=HEADER, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} readings -> {READINGS.name}")
    if "--apply" in sys.argv:
        apply(READINGS.parent.parent / "harmonicdrive")


if __name__ == "__main__":
    main()
