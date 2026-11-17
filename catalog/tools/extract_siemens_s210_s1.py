"""Reads the S1 curves of the SIMOTICS S-1FK2 and S-1FT2 servomotors off the speed/torque graphs of the Siemens
catalogue D 32 (SINAMICS S210, January 2024) and writes catalog/siemens/s1_curves_s210.csv.

    python -I tools/extract_siemens_s210_s1.py d32.pdf [../siemens/s1_curves_s210.csv]

Run it from catalog/ with the catalogue PDF (URL in siemens/motors_s210.meta.toml) kept in a folder of its own.

The graphs ("Speed/torque characteristics", printed pages 3.1/20 ff., PDF pages 99 to 172) are vector art: a frame with
a grid, and per supply voltage two thick lines, Mmax above and S1 below. Each motor has one graph for the 200 ... 240 V
supply (legend "230 V 1 AC" and "240 V 3 AC", or one of them) and one for 380 ... 480 V ("400 V 3 AC", "480 V 3 AC").
Method, per graph: the frame (a 0.37 wide rectangle) and its grid lines give the axes, checked against the printed tick
labels (y: one label per horizontal grid line, 0 and equal steps; x: the labels are one string "0 d 2d ... xmax" whose
last number is the frame's right edge); the thick lines inside the frame are the curves; the S1 curves are the lower
half by their torque at the speed 0 (Mmax is above M0 by a factor of 2 or more), told apart by supply as the
legend names them in rising voltage (the line with the larger area under it is the higher voltage). A curve ends where
it first reaches its largest speed (the vertical drop to the axis after it is the speed limit, not part of S1). The
polyline is reduced to at most 12 points by the Douglas-Peucker rule on the curve in units of the speed range and M0
(least tolerance that fits), at least 5 (then points of the same polyline are added); no point is interpolated between
what the graph shows. The curve of a motor is the one for 400 V 3 AC where the catalogue row has a rated point at 400 V
(motors_s210.csv, rated_*_400v), else the one for 240 V 3 AC (230 V 3 AC where the graph names that). The script
stops where a check fails: the Mmax level of the graph against the printed maximum torque (2 %), the S1 start against
the static torque (2 %, but at least half of the last printed digit), the reduced curve against the printed
rated point of its supply (same tolerance), the speed where the Mmax lines of the 400 V graph stop against the maximum speed (between 0.8 and 1.03 times).
A graph the checks refuse, or a motor with no graph, gets no rows: the script lists them.
"""
import csv
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cataloglib import pdfvec  # noqa: E402

HERE = Path(__file__).resolve().parent
MOTORS = HERE.parent / "siemens" / "motors_s210.csv"
PAGES = range(95, 176)  # 1-based PDF pages that hold the motors' graphs (shaft heights 20 to 100, 1FK2 then 1FT2)
TITLE = re.compile(r"(1F[KT]2\d{3}-\d[A-Z]{2})")
SUPPLY = re.compile(r"(\d{3}) ?V ?(\d) ?AC")
RPM = 2 * math.pi / 60
POINTS = (5, 12)


def tolerance(text):
    """What a reading may differ from a printed value: 2 % of it, but at least half of its last printed digit."""
    quantum = 10 ** -len(text.split(".")[1]) if "." in text else 1
    return max(0.02 * float(text), 0.5 * quantum)


def at_speed(points, w):
    """Torque of the polyline [(speed, torque)] at speed w (linear between the points), None beyond its end."""
    for (xa, ya), (xb, yb) in zip(points, points[1:]):
        if xa <= w <= xb and xb > xa:
            return ya + (yb - ya) * (w - xa) / (xb - xa)
    return None


def number(text):
    try:
        return float(text)
    except ValueError:
        return None


def frames(paths):
    """Plot frames (x0, x1, ybottom, ytop) of the page: the closed rectangles of a graph's size (about 192 wide), drawn
    stroked (0.37) or filled; the same frame drawn twice counts once."""
    out = []
    for p in paths:
        if (p["kind"] == "f" or abs(p["w"] - 0.37) < 0.02) and len(p["subs"]) == 1 and len(p["subs"][0]) == 5:
            xs, ys = [q[0] for q in p["subs"][0]], [q[1] for q in p["subs"][0]]
            f = (min(xs), max(xs), min(ys), max(ys))
            if 170 < f[1] - f[0] < 215 and 80 < f[3] - f[2] < 200 and not any(
                    all(abs(a - b) < 1.5 for a, b in zip(f, g)) for g in out):
                out.append(f)
    return sorted(out, key=lambda f: (-round(f[3]), f[0]))


def labels_x(string):
    """The x tick labels, printed as strings that may be split, shuffled and lack a label: (d, k) such that the characters
    of the string are those of 0, d, 2d, ... k d but for one label that may be missing (and one stray character), the
    last label being in it; of several the one with the largest d."""
    from collections import Counter
    have, best = Counter(string), None
    for d in range(100, 2501, 50):
        for k in range(3, 21):
            full = Counter("".join(str(i * d) for i in range(k + 1)))
            extra, missing = have - full, full - have
            if sum(extra.values()) <= 1 and not Counter(str(k * d)) - have and (
                    not missing or any(missing == Counter(str(j * d)) for j in range(1, k))):
                key = (sum(extra.values()), sum(missing.values()), -d)
                if best is None or key < best[0]:
                    best = (key, d, k)
    if best is None:
        raise ValueError(f"x tick labels '{string}' are not 0, d, 2d, ... with at most one label missing")
    return best[1], best[2]


def axes(fr, paths, texts, name):
    """(speed of x in r/min, torque of y in N m, speed limit of the axis) of the frame. The tick labels give the scales:
    y: 0 at the bottom axis line, the top label at the top one (else at the grid line the labels' own line points to);
    x: 0 at the left edge, the last label at the right edge (checked later against the catalogue's rated points)."""
    x0, x1, yb, yt = fr
    grid = set()
    for p in paths:
        if p["kind"] == "S" and p["w"] < 0.4 and len(p["subs"]) == 1 and len(p["subs"][0]) == 2:
            (xa, ya), (xb, yb_) = p["subs"][0]
            if abs(ya - yb_) < 0.01 and yb - 1.5 <= ya <= yt + 1.5 and abs(xa - xb) > 0.8 * (x1 - x0) \
                    and min(xa, xb) < x0 + 5 and max(xa, xb) > x1 - 5:
                grid.add(round(ya, 1))
    if grid:  # the drawn axis lines are the grid lines at the frame's edges
        near = lambda y: min(grid, key=lambda g: abs(g - y))  # noqa: E731
        yb, yt = (near(yb) if abs(near(yb) - yb) < 1.5 else yb), (near(yt) if abs(near(yt) - yt) < 1.5 else yt)
    ylab = sorted(((t[2], number(t[0])) for t in texts if len(t[0]) <= 6 and number(t[0]) is not None
                   and x0 - 45 < t[1] < x0 - 3 and yb - 8 < t[2] < yt + 8), key=lambda t: t[0])
    values = [v for _, v in ylab]
    step = min(b - a for a, b in zip(values, values[1:])) if len(values) > 1 else 0
    if len(values) < 3 or values[0] != 0 or step <= 0 or any(abs(v / step - round(v / step)) > 1e-6 for v in values):
        raise ValueError(f"{name}: y tick labels {values}")  # (a label may be missing from a printed graph: a gap is allowed)
    # the labels are placed by hand (a few units off the lines): the scale is the axis' own, the bottom edge being 0 and
    # the top edge the top label (it must sit within 6 units of it); the other labels must lie within 6 of their value
    per = (yt - yb) / values[-1]
    if abs(ylab[-1][0] - yt) > 6 or any(abs(y - (yb + per * v)) > 6 for y, v in ylab):
        # the frame is taller than the top label: take the scale from the grid lines the labels belong to
        n = len(values)
        my, mv = sum(y for y, _ in ylab) / n, sum(values) / n
        slope = sum((y - my) * (v - mv) for y, v in ylab) / sum((v - mv) ** 2 for v in values)
        snapped = [min(grid | {yb}, key=lambda g: abs(g - (my + slope * (v - mv)))) for v in values]
        per = (snapped[-1] - snapped[0]) / values[-1]
        if len(set(snapped)) != n or abs(snapped[0] - yb) > 1.5 or any(abs(g - (snapped[0] + per * v)) > 0.3 for g, v in zip(snapped, values)):
            raise ValueError(f"{name}: the y tick labels fit neither the frame's height nor the grid lines")
        yb = snapped[0]
    xs = {round(x0, 1), round(x1, 1)}
    for p in paths:
        if p["kind"] == "S" and p["w"] < 0.4 and len(p["subs"]) == 1 and len(p["subs"][0]) == 2:
            (xa, ya), (xb, yb_) = p["subs"][0]
            if abs(xa - xb) < 0.01 and x0 - 0.5 <= xa <= x1 + 0.5 and abs(ya - yb_) > 0.8 * (yt - yb) and min(ya, yb_) > yb - 1.5 and max(ya, yb_) < yt + 1.5:
                xs.add(round(xa, 1))
    xs = sorted(xs)
    gaps = sorted(b - a for a, b in zip(xs, xs[1:]) if b - a > 3)
    lines = round((x1 - x0) / gaps[len(gaps) // 2]) if gaps else 0  # intervals of the x grid
    xstring = "".join(t[0] for t in texts if re.fullmatch(r"\d+", t[0]) and x0 - 5 < t[1] < x1 + 5 and yb - 30 < t[2] < yb - 5)
    d, k = labels_x(xstring)
    xmax = d * k
    return (lambda x: (x - x0) / (x1 - x0) * xmax), (lambda y: (y - yb) / per), xmax, lines


def supplies(texts, fr):
    """The supplies named in the legend below the frame, in the order printed: [(volts, phases)]."""
    x0, x1, yb, _ = fr
    line = "".join(t[0] for t in sorted((t for t in texts if x0 - 5 < t[1] < x1 + 5 and yb - 60 < t[2] < yb - 5),
                                        key=lambda t: (-round(t[2]), t[1])))
    return [(int(v), int(p)) for v, p in SUPPLY.findall(line)]


def curves(paths, fr, name):
    """The thick lines inside the frame as polylines of (x, y) in page units, one per line starting at the left axis.
    Where the drawing breaks a line into pieces (short gaps), a piece that does not start at the axis continues the
    piece whose right end is nearest to its left end (within 8 page units)."""
    x0, x1, yb, yt = fr
    pieces = []
    for p in paths:
        if p["kind"] == "S" and 2.0 < p["w"] < 2.5:
            for sub in p["subs"]:
                if len(sub) >= 2 and all(x0 - 1 <= q[0] <= x1 + 1 and yb - 1 <= q[1] <= yt + 1 for q in sub):
                    pieces.append(sub)
    heads = [s for s in pieces if min(q[0] for q in s) < x0 + 1.5]
    rest = sorted((s for s in pieces if s not in heads), key=lambda s: min(q[0] for q in s))
    heads = [list(s) for s in heads]
    for piece in rest:
        left = min(piece, key=lambda q: q[0])
        host = min(heads, key=lambda h: math.dist(max(h, key=lambda q: q[0]), left))
        if math.dist(max(host, key=lambda q: q[0]), left) > 8:
            raise ValueError(f"{name}: a piece of a line at x {left[0]:.1f} has no line to continue")
        host.extend(sorted(piece, key=lambda q: q[0]))
    return heads


def trimmed(sub):
    """The polyline up to the first point at the line's largest x, and from the left axis on."""
    xmax = max(q[0] for q in sub)
    for i, q in enumerate(sub):
        if q[0] >= xmax - 0.05:
            sub = sub[:i + 1]
            break
    if any(b[0] < a[0] - 0.3 for a, b in zip(sub, sub[1:])):
        raise ValueError("a curve turns back in speed")
    return sub


def reduce(points, lo, hi, scale_x, scale_y):
    """At most `hi` points of the polyline [(x, y)]: Douglas-Peucker with the least tolerance that fits; at least `lo`."""
    def dp(pts, tol):
        (xa, ya), (xb, yb) = pts[0], pts[-1]
        far, at = 0.0, 0
        for i in range(1, len(pts) - 1):
            x, y = pts[i]
            d = abs((xb - xa) * (ya - y) - (xa - x) * (yb - ya)) / (math.hypot(xb - xa, yb - ya) or 1)
            if d > far:
                far, at = d, i
        if far <= tol:
            return [pts[0], pts[-1]]
        return dp(pts[:at + 1], tol)[:-1] + dp(pts[at:], tol)

    norm = [(x / scale_x, y / scale_y) for x, y in points]
    tol = 0.001
    while True:
        keep = dp(norm, tol)
        if len(keep) <= hi:
            break
        tol *= 1.25
    out = [(x * scale_x, y * scale_y) for x, y in keep]
    while len(out) < lo:  # add the polyline's own point in the middle of the longest gap
        i = max(range(len(out) - 1), key=lambda j: out[j + 1][0] - out[j][0])
        xm = (out[i][0] + out[i + 1][0]) / 2
        out.insert(i + 1, (xm, at_x(points, xm)))
    return out


def at_x(points, x):
    for (xa, ya), (xb, yb) in zip(points, points[1:]):
        if xa <= x <= xb and xb > xa:
            return ya + (yb - ya) * (x - xa) / (xb - xa)
    return points[-1][1]


def area(points):
    return sum((xb - xa) * (ya + yb) / 2 for (xa, ya), (xb, yb) in zip(points, points[1:]))


def read_graph(paths, texts, fr, title):
    """{(volts, phases): [(speed r/min, torque N m), ...]} of the S1 lines, and the Mmax level, of one graph."""
    speed, torque, xmax, n_grid = axes(fr, paths, texts, title)
    sup = supplies(texts, fr)
    if not sup or sup != sorted(sup, key=lambda s: s[0]):
        raise ValueError(f"{title}: legend supplies {sup}")
    cs = curves(paths, fr, title)
    if len(cs) != 2 * len(sup):
        raise ValueError(f"{title}: {len(cs)} curves for the supplies {sup}")
    start = lambda c: torque(min(c, key=lambda q: q[0])[1])  # noqa: E731
    cs.sort(key=start)
    s1, mmax = cs[:len(sup)], cs[len(sup):]
    if max(start(c) for c in s1) - min(start(c) for c in s1) > 0.03 * start(s1[0]) or min(start(c) for c in mmax) < 1.5 * max(start(c) for c in s1):
        raise ValueError(f"{title}: the S1 and Mmax lines are not told apart by their start")
    out = {}
    lines = []
    for c in s1:
        pts = [(speed(x), torque(y)) for x, y in trimmed(c)]
        lines.append(pts)
    lines.sort(key=area)
    for s, pts in zip(sup, lines):
        out[s] = pts
    return out, max(torque(q[1]) for c in mmax for q in c), (max(speed(q[0]) for c in mmax for q in c), xmax, n_grid)


def readings(pdf, rows):
    """{article: {(volts, phases): [(rpm, Nm)]}} and the graphs' names that failed their checks."""
    data, objs = pdfvec.load(pdf)
    page_list = pdfvec.pages(data, objs)
    found, failed = {}, []
    for number_ in PAGES:
        paths, texts = pdfvec.interpret(pdfvec.content(objs, page_list[number_ - 1]))
        titles = [(m[1], t[1], t[2]) for t in texts if (m := TITLE.fullmatch(t[0].strip()) or re.match(TITLE.pattern + " servomotor", t[0]))]
        for fr in frames(paths):
            above = [t for t in titles if fr[0] - 5 < t[1] < fr[1] and 0 < t[2] - fr[3] < 40]
            if not above:
                continue
            article = min(above, key=lambda t: t[2] - fr[3])[0]
            row = rows.get(article)
            try:
                curves_, mmax, scale = read_graph(paths, texts, fr, f"{article} (page {number_})")
                if row is None:
                    raise ValueError(f"{article} (page {number_}): not in motors_s210.csv")
                if abs(mmax - float(row["max_torque_nm"])) > tolerance(row["max_torque_nm"]):
                    raise ValueError(f"{article} (page {number_}): Mmax level {mmax:.3f} against {row['max_torque_nm']}")
            except ValueError as e:
                failed.append(str(e))
                continue
            found.setdefault(article, {}).update({s: (pts, number_, *scale) for s, pts in curves_.items()})
    return found, failed


def rescaled(row, curves_):
    """The curves with the speed axis corrected where the last tick label is not the frame's right edge: the Mmax lines
    of a 380 ... 480 V graph stop at the motor's maximum speed, which gives the right edge's speed; that is taken when it
    is a whole number of grid lines of a multiple of 50 r/min each (else the labels' reading stays)."""
    ref = curves_.get((480, 3)) or curves_.get((400, 3))
    if ref is None:
        return curves_
    pts, page, last, xmax, lines = ref
    nmax = float(row["max_speed_rad_s"]) / RPM
    if 0.97 <= last / nmax <= 1.03 or not lines:
        return curves_
    raw = xmax * nmax / last
    nice = round(raw / lines / 50) * 50 * lines
    if nice <= 0 or abs(nice - raw) > 0.03 * raw:
        return curves_
    f = nice / xmax
    return {s: ([(x * f, y) for x, y in c[0]], c[1], c[2] * f, nice, c[4]) if c[3] == xmax else c for s, c in curves_.items()}


def chosen(article, row, curves_):
    """(supply, points, page) the motor's curve is taken from, or None."""
    wanted = (400, 3) if row["rated_speed_400v_rad_s"] else None
    if wanted in curves_:
        return (wanted, *curves_[wanted])
    if wanted is None:
        three = [s for s in curves_ if s[1] == 3 and s[0] < 300]
        if three:
            return (three[-1], *curves_[three[-1]])
    return None


def build(pdf):
    with open(MOTORS, encoding="utf-8", newline="") as f:
        rows = {r["part_number"]: r for r in csv.DictReader(f)}
    found, failed = readings(pdf, rows)
    out, none = [], []
    for article, row in rows.items():
        pick = chosen(article, row, rescaled(row, found.get(article, {})))
        if pick is None:
            none.append(article)
            continue
        supply, pts, page, last, _, _ = pick
        m0 = float(row["static_torque_nm"])
        if abs(pts[0][1] - m0) > tolerance(row["static_torque_nm"]):
            failed.append(f"{article} (page {page}): S1 starts at {pts[0][1]:.3f} N m, static torque {m0}")
            none.append(article)
            continue
        if supply == (400, 3) and not 0.8 * float(row["max_speed_rad_s"]) <= last * RPM <= 1.03 * float(row["max_speed_rad_s"]):
            failed.append(f"{article} (page {page}): the 480 V Mmax line ends at {last * RPM:.1f} rad/s, maximum speed {row['max_speed_rad_s']}")
            none.append(article)
            continue
        if abs(pts[0][0]) > 16:
            failed.append(f"{article} (page {page}): the curve starts at {pts[0][0]} r/min, not at 0")
            none.append(article)
            continue
        pts = [(0.0, pts[0][1])] + pts[1:]
        if min(y for _, y in pts) < -0.01 * m0:
            failed.append(f"{article} (page {page}): the curve dips below zero torque")
            none.append(article)
            continue
        pts = [(x, max(0.0, y)) for x, y in pts]  # (the line ends on the axis: a reading of -0.5 % of M0 is the axis)
        red = reduce(pts, POINTS[0], POINTS[1], max(p[0] for p in pts), m0)
        sup = "400v" if supply == (400, 3) else "230v"
        w, q = float(row[f"rated_speed_{sup}_rad_s"]), row[f"rated_torque_{sup}_nm"]
        t = at_speed([(x * RPM, y) for x, y in red], w)
        if t is None or abs(t - float(q)) > tolerance(q):
            failed.append(f"{article} (page {page}): the {supply[0]} V curve gives {t} N m at the rated speed {w:.2f} rad/s, printed {q}")
            none.append(article)
            continue
        out += [(article, f"{x * RPM:.4f}", f"{y:.4f}") for x, y in red]
        print(f"{article}: {supply[0]} V {supply[1]} AC, page {page}, {len(red)} points")
    return out, none, failed


def main(argv):
    if not argv:
        raise SystemExit(__doc__)
    out, none, failed = build(argv[0])
    target = Path(argv[1]) if len(argv) > 1 else HERE.parent / "siemens" / "s1_curves_s210.csv"
    with open(target, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["article", "speed_rad_s", "torque_nm"])
        w.writerows(out)
    print(f"{len({r[0] for r in out})} motors with a curve, {len(set(none))} without: {sorted(set(none))}")
    for line in failed:
        print("refused:", line)


if __name__ == "__main__":
    main(sys.argv[1:])
