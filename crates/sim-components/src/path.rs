//! Paths of straight lines, circular arcs and smooth curves through via points in tool coordinates, and the
//! planning of the timing along them.
//! Both planners give poses at times, one per profile cycle, which `Command::Trajectory` runs.
//!
//! A path file has one piece per line. The path starts where the machine stands.
//!
//! ```text
//! line,0.5,0.35              straight to this point: one position per axis
//! arc,0.6,0.25,0.5,0.25,cw   on a circle to the point (0.6, 0.25) around the centre (0.5, 0.25)
//! spline,0.7,0.2             on a smooth curve through this point
//! ```
//!
//! `cw` and `ccw` are seen with the first axis to the right and the second upwards. An arc that ends on the end
//! of the piece before it is a full turn. As the first piece an arc has to end at least 1 mm from where the
//! machine stands (see `FIRST_ARC`). Spline pieces that follow one another form one curve through all their points (see
//! `spline`); it leaves the line or arc before it in that piece's direction and ends in the direction the
//! line or arc after it starts with. Arcs and splines need a machine with two axes. Empty lines and lines
//! starting with `#` are skipped.

use crate::control::CommandError;
use crate::drive::DriveParams;
use sim_core::{AxisInfo, Machine};

#[derive(Debug, Clone, PartialEq)]
pub enum Piece {
    /// Straight to this point.
    Line(Vec<f64>),
    /// On a circle around `centre` to the point `to`.
    Arc { to: Vec<f64>, centre: Vec<f64>, clockwise: bool },
    /// On a smooth curve through this point; consecutive ones form one curve.
    Spline(Vec<f64>),
}

/// Reads a path file (format: see the top of this file). The error is the line that cannot be read.
pub fn parse(text: &str) -> Result<Vec<Piece>, String> {
    let mut pieces = Vec::new();
    for line in text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        let numbers = |fields: &[&str]| fields.iter().map(|v| v.parse().ok().filter(|n: &f64| n.is_finite())).collect::<Option<Vec<f64>>>();
        let turn = fields[fields.len() - 1];
        let piece = match fields[0] {
            "line" if fields.len() > 1 => numbers(&fields[1..]).map(Piece::Line),
            "spline" if fields.len() > 1 => numbers(&fields[1..]).map(Piece::Spline),
            "arc" if fields.len() == 6 && (turn == "cw" || turn == "ccw") => {
                numbers(&fields[1..5]).map(|n| Piece::Arc { to: n[..2].to_vec(), centre: n[2..].to_vec(), clockwise: turn == "cw" })
            }
            _ => None,
        };
        pieces.push(piece.ok_or(line.to_string())?);
    }
    Ok(pieces)
}

/// Reads a path given as poses at times: per line the time in seconds and one position per axis. Lines that
/// do not start with a number (a header, comments) are skipped. The error is the line that cannot be read.
pub fn parse_trajectory(text: &str) -> Result<Vec<(f64, Vec<f64>)>, String> {
    let mut points = Vec::new();
    for line in text.lines() {
        let mut fields = line.split(',').map(|f| f.trim().parse::<f64>());
        let Some(Ok(time)) = fields.next() else { continue };
        points.push((time, fields.collect::<Result<_, _>>().map_err(|_| line.to_string())?));
    }
    Ok(points)
}

/// A piece with its start and its length worked out.
#[derive(Clone)]
enum Geo {
    Line { from: Vec<f64>, dir: Vec<f64>, length: f64 },
    /// `turn` is 1 counter-clockwise and -1 clockwise.
    Arc { centre: [f64; 2], radius: f64, from_angle: f64, turn: f64, length: f64 },
    /// A transition curve (clothoid): from the point `from` in the direction `heading`, the curvature goes
    /// evenly with the distance from `k0` to `k1`. Curvature is 1 / radius, positive counter-clockwise.
    Clothoid { from: [f64; 2], heading: f64, k0: f64, k1: f64, length: f64 },
    /// One span of a spline, from one via point to the next, or a part of one: the polynomial c0 + c1 u + ... +
    /// c5 u^5 for u from 0 to `h` (a cubic, c4 = c5 = 0, but where a spline is eased into a line or an arc,
    /// `ease`). `travelled` is the distance along it at `SPLINE_PARTS` + 1 evenly spaced u, the last being the
    /// length; `tightest` and `rate` are the most its curvature and the change of curvature per distance get.
    Spline { c: Poly, h: f64, travelled: Vec<f64>, tightest: f64, rate: f64 },
}

/// The coefficients of a spline span's polynomial in u, per power, for both axes.
type Poly = [[f64; 2]; 6];

/// A spline span's table of distance travelled has this many parts.
const SPLINE_PARTS: usize = 64;
/// The curvature of a spline span and its rate of change are looked at at 4 * `SPLINE_PARTS` + 1 places; what
/// they reach in between is covered by raising the largest found by this factor.
const SPLINE_MARGIN: f64 = 1.05;

/// Derivatives of a spline span's polynomial by u: first, second, third.
fn poly_rates(c: &Poly, u: f64) -> [[f64; 2]; 3] {
    // The k-th derivative: sum over m of m! / (m - k)! c_m u^(m - k).
    let rate = |k: usize, i: usize| (k..6).rev().fold(0.0, |sum, m| sum * u + c[m][i] * ((m - k + 1)..=m).map(|f| f as f64).product::<f64>());
    [1, 2, 3].map(|k| [rate(k, 0), rate(k, 1)])
}

/// The same polynomial with u counted from `from`: its coefficients around that point.
fn poly_shift(c: &Poly, from: f64) -> Poly {
    let mut out = [[0.0; 2]; 6];
    for (m, cm) in c.iter().enumerate() {
        // (u + from)^m = sum over k of m! / (k! (m - k)!) u^k from^(m - k)
        let mut binomial = 1.0;
        for (k, o) in out.iter_mut().enumerate().take(m + 1) {
            for i in 0..2 {
                o[i] += cm[i] * binomial * from.powi((m - k) as i32);
            }
            binomial *= (m - k) as f64 / (k + 1) as f64;
        }
    }
    out
}

/// Distance travelled on a spline span between two values of u: five-point Gauss-Legendre, exact to rounding
/// for a part of a 64th of a span.
fn poly_distance(c: &Poly, from: f64, to: f64) -> f64 {
    const NODES: [(f64, f64); 5] =
        [(0.0, 0.568_888_888_888_888_9), (0.538_469_310_105_683_1, 0.478_628_670_499_366_5), (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5), (0.906_179_845_938_664, 0.236_926_885_056_189_1), (-0.906_179_845_938_664, 0.236_926_885_056_189_1)];
    let (mid, half) = (0.5 * (from + to), 0.5 * (to - from));
    half * NODES.iter().map(|(x, w)| { let [d, ..] = poly_rates(c, mid + half * x); w * d[0].hypot(d[1]) }).sum::<f64>()
}

/// A spline span of the polynomial `c` for u from 0 to `h`, with its table of distance travelled and the most its
/// curvature and the change of curvature per distance get. `None` for a curve that nearly stops and turns on the
/// spot somewhere (its speed by u, 1 on average, falls below 0.05).
fn span(c: Poly, h: f64) -> Option<Geo> {
    let part = h / SPLINE_PARTS as f64;
    let mut travelled = vec![0.0];
    for k in 0..SPLINE_PARTS {
        travelled.push(travelled[k] + poly_distance(&c, k as f64 * part, (k + 1) as f64 * part));
    }
    let (mut tightest, mut rate) = (0f64, 0f64);
    for k in 0..=4 * SPLINE_PARTS {
        let [d, d2, d3] = poly_rates(&c, k as f64 * part / 4.0);
        let (speed, cross) = (d[0].hypot(d[1]), |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0]);
        if speed < 0.05 {
            return None;
        }
        // Curvature k = d x d2 / speed^3, and its derivative by u over the speed.
        let curvature = cross(d, d2) / speed.powi(3);
        let by_u = cross(d, d3) / speed.powi(3) - 3.0 * curvature * (d[0] * d2[0] + d[1] * d2[1]) / (speed * speed);
        (tightest, rate) = (tightest.max(curvature.abs()), rate.max((by_u / speed).abs()));
    }
    Some(Geo::Spline { c, h, travelled, tightest: SPLINE_MARGIN * tightest, rate: SPLINE_MARGIN * rate })
}

impl Geo {
    fn length(&self) -> f64 {
        match self {
            Geo::Line { length, .. } | Geo::Arc { length, .. } | Geo::Clothoid { length, .. } => *length,
            Geo::Spline { travelled, .. } => travelled[SPLINE_PARTS],
        }
    }

    /// Pose, direction and curvature at the distance s from the start of the piece: the pose and its first
    /// and second derivative by the distance travelled.
    fn at(&self, s: f64) -> [Vec<f64>; 3] {
        match self {
            Geo::Line { from, dir, .. } => [from.iter().zip(dir).map(|(p, d)| p + d * s).collect(), dir.clone(), vec![0.0; dir.len()]],
            Geo::Arc { centre, radius, from_angle, turn, .. } => {
                let (sin, cos) = (from_angle + turn * s / radius).sin_cos();
                [vec![centre[0] + radius * cos, centre[1] + radius * sin], vec![-turn * sin, turn * cos], vec![-cos / radius, -sin / radius]]
            }
            Geo::Clothoid { from, heading, k0, k1, length } => {
                let angle = |u: f64| heading + k0 * u + 0.5 * (k1 - k0) * u * u / length;
                // The pose is the direction added up along the curve, which has no closed form: Simpson's
                // rule over 64 intervals, exact to far below a nanometre for the turns a blend makes.
                let (n, h) = (64, s / 64.0);
                let (mut x, mut y) = (0.0, 0.0);
                for i in 0..=n {
                    let weight = if i == 0 || i == n { 1.0 } else if i % 2 == 1 { 4.0 } else { 2.0 };
                    let (sin, cos) = angle(i as f64 * h).sin_cos();
                    (x, y) = (x + weight * cos, y + weight * sin);
                }
                let ((sin, cos), k) = (angle(s).sin_cos(), k0 + (k1 - k0) * s / length);
                [vec![from[0] + x * h / 3.0, from[1] + y * h / 3.0], vec![cos, sin], vec![-k * sin, k * cos]]
            }
            Geo::Spline { c, h, travelled, .. } => {
                // The u at which the span has run the distance s: from the table, then Newton's method on the
                // distance itself, which converges to rounding in two or three steps.
                let part = travelled.partition_point(|t| *t <= s).clamp(1, SPLINE_PARTS) - 1;
                let (u0, du) = (part as f64 * h / SPLINE_PARTS as f64, h / SPLINE_PARTS as f64);
                let mut u = u0 + du * (s - travelled[part]) / (travelled[part + 1] - travelled[part]);
                for _ in 0..4 {
                    let [d, ..] = poly_rates(c, u);
                    u -= (travelled[part] + poly_distance(c, u0, u) - s) / d[0].hypot(d[1]);
                }
                let [d, d2, _] = poly_rates(c, u);
                let speed = d[0].hypot(d[1]);
                let (dir, along) = ([d[0] / speed, d[1] / speed], (d[0] * d2[0] + d[1] * d2[1]) / speed);
                let pose = (0..2).map(|i| c.iter().rev().fold(0.0, |sum, cm| sum * u + cm[i])).collect();
                // The second derivative by distance: what of the second derivative by u is across the direction.
                [pose, dir.to_vec(), (0..2).map(|i| (d2[i] - along * dir[i]) / (speed * speed)).collect()]
            }
        }
    }
}

/// A smooth curve from `points[0]` through all of `points`: a cubic spline, one `Geo::Spline` per span.
///
/// The parameter u runs with the straight distance between the via points, and on each span the curve is a
/// cubic in u. The cubics are chosen so that the curve has no step in position, in direction or in curvature
/// at a via point (the first and second derivative by u agree there). At an end the curve takes the given
/// direction (`before` at the start, `after` at the end: that of the line or arc it joins); where there is
/// none it ends without curvature. That leaves one curve, found by solving for the derivatives at the
/// points (a system with three entries per row).
///
/// What it does not promise: the curvature at an end with a given direction is whatever the points ask for,
/// so where the spline joins a line or an arc the curvature steps, and the path passes that junction as
/// slowly as a line into an arc (`junction_speed`), unless a tolerance lets `ease` bend the spline's end to the
/// curvature of its neighbour.
///
/// `None` for a curve that nearly stops and turns on the spot somewhere (its speed by u, 1 on average, falls
/// below 0.05): via points and end directions that fight each other.
fn spline(points: &[Vec<f64>], before: Option<[f64; 2]>, after: Option<[f64; 2]>) -> Option<Vec<Geo>> {
    let n = points.len() - 1;
    let h: Vec<f64> = points.windows(2).map(|w| distance(&w[0], &w[1])).collect();
    let chord = |i: usize, axis: usize| (points[i + 1][axis] - points[i][axis]) / h[i];
    // Rows (below, diagonal, above, right side) of the system for the derivatives at the points.
    let mut rows: Vec<[f64; 3]> = Vec::new();
    let mut right: Vec<[f64; 2]> = Vec::new();
    for i in 0..=n {
        let (row, side) = match (i, before, after) {
            (0, Some(dir), _) => ([0.0, 1.0, 0.0], dir),
            (0, None, _) => ([0.0, 2.0, 1.0], [3.0 * chord(0, 0), 3.0 * chord(0, 1)]),
            (i, _, Some(dir)) if i == n => ([0.0, 1.0, 0.0], dir),
            (i, _, None) if i == n => ([1.0, 2.0, 0.0], [3.0 * chord(n - 1, 0), 3.0 * chord(n - 1, 1)]),
            _ => ([h[i], 2.0 * (h[i - 1] + h[i]), h[i - 1]], [0, 1].map(|axis| 3.0 * (h[i] * chord(i - 1, axis) + h[i - 1] * chord(i, axis)))),
        };
        rows.push(row);
        right.push(side);
    }
    // Elimination downwards, then substitution upwards.
    for i in 1..=n {
        let factor = rows[i][0] / rows[i - 1][1];
        rows[i][1] -= factor * rows[i - 1][2];
        right[i] = [right[i][0] - factor * right[i - 1][0], right[i][1] - factor * right[i - 1][1]];
    }
    let mut d = vec![[0.0; 2]; n + 1];
    for i in (0..=n).rev() {
        let above = if i < n { d[i + 1] } else { [0.0; 2] };
        d[i] = [0, 1].map(|axis| (right[i][axis] - rows[i][2] * above[axis]) / rows[i][1]);
    }
    (0..n)
        .map(|i| {
            let c = [
                [points[i][0], points[i][1]],
                d[i],
                [0, 1].map(|axis| (3.0 * chord(i, axis) - 2.0 * d[i][axis] - d[i + 1][axis]) / h[i]),
                [0, 1].map(|axis| (d[i][axis] + d[i + 1][axis] - 2.0 * chord(i, axis)) / (h[i] * h[i])),
                [0.0; 2],
                [0.0; 2],
            ];
            span(c, h[i])
        })
        .collect()
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| (a - b) * (a - b)).sum::<f64>().sqrt()
}

/// The path the planners time: the pieces laid end to end from `start`, with transition curves where the
/// tolerance allows them (`blend` between lines and arcs, `ease` where a spline meets a line or an arc), and how
/// far that path is from the given one at most. `speed` is the highest speed along a piece that the planner will
/// allow, which sets how long a transition has to be.
fn geometry(axes: &[AxisInfo], start: &[f64], pieces: &[Piece], tolerance: f64, speed: &dyn Fn(&Geo) -> f64) -> Result<(Vec<Geo>, f64), CommandError> {
    let (geo, off) = given(start, pieces)?;
    if tolerance <= 0.0 {
        return Ok((geo, off));
    }
    let (geo, moved) = blend(geo, axes, tolerance, speed);
    let (geo, eased) = ease(geo, axes, tolerance, speed);
    // The two change different places of the path, so the larger of them counts.
    Ok((geo, off + moved.max(eased)))
}

/// The highest speed along a piece within the axis limits, as `plan` times it.
fn axis_speed(axes: &[AxisInfo]) -> impl Fn(&Geo) -> f64 {
    let bounds = axis_bounds(axes);
    move |geo| limits(geo, &bounds)[0]
}

/// How far the path that is planned for these pieces is from the pieces as given, at most: the deviation
/// the transition curves make, plus how far an arc ends off its stated end (`given`). Zero with a tolerance of
/// zero and arcs that end on their circles. For the plan within the axis limits.
pub fn deviation(axes: &[AxisInfo], start: &[f64], pieces: &[Piece], tolerance: f64) -> Result<f64, CommandError> {
    Ok(geometry(axes, start, pieces, tolerance, &axis_speed(axes))?.1)
}

/// For an arc that is the first piece: how far its end may be off the circle through its start. A path starts
/// at the setpoint of the machine at rest, which is where the motors stand: under belt stretch and gear twist up
/// to 0.1 mm from the pose the file was written for (the controller's `SETPOINT_SLACK`; 12 and 46 um on a gantry
/// holding its Z axis against gravity).
const ARC_SLACK: f64 = 1e-4;
/// A first arc has to end at least this far from where the machine stands. Nearer, a full turn (whose end is
/// up to `ARC_SLACK` from there) cannot be told from a short arc (a 130 um arc ran as a full turn), so neither
/// is guessed: the path is refused, and the file states the start with a `line` to it first, after which the
/// arc is no longer the first piece. With a margin over `ARC_SLACK`, so that a first arc that is run is at
/// most a tenth longer or shorter than the one the file means.
const FIRST_ARC: f64 = 1e-3;
/// For every later arc: how far the stated end may be off the circle through the stated end before it.
const ARC_TOLERANCE: f64 = 1e-5;

fn piece_end(piece: &Piece) -> &Vec<f64> {
    let (Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to)) = piece;
    to
}

/// The pieces laid end to end from `start`, and how far a piece ends off its stated end at most: an arc is
/// laid on the circle through where the path stands, which its stated end may miss by the tolerances above.
/// Pieces without length are left out.
fn given<'a>(start: &[f64], pieces: &'a [Piece]) -> Result<(Vec<Geo>, f64), CommandError> {
    let bad = Err(CommandError { key: "error-path" });
    let (mut here, mut out): (Vec<f64>, Vec<Geo>) = (start.to_vec(), Vec::new());
    // The end the file stated last; none before the first piece, where the path stands on the machine's setpoint.
    let (mut stated, mut off): (Option<&'a Vec<f64>>, f64) = (None, 0.0);
    let mut rest = pieces;
    while let Some((piece, after)) = rest.split_first() {
        rest = after;
        let geo = match piece {
            Piece::Spline(_) if here.len() == 2 => {
                // The whole run of spline pieces is one curve; a via point given twice counts once.
                let run = 1 + rest.iter().take_while(|p| matches!(p, Piece::Spline(_))).count();
                let mut points = vec![here.clone()];
                for to in std::iter::once(piece).chain(&rest[..run - 1]).filter_map(|p| if let Piece::Spline(to) = p { Some(to) } else { None }) {
                    if to.len() != 2 {
                        return bad;
                    }
                    if distance(to, &points[points.len() - 1]) > 1e-9 {
                        points.push(to.clone());
                    }
                }
                rest = &rest[run - 1..];
                stated = Some(piece_end(&pieces[pieces.len() - rest.len() - 1]));
                if points.len() < 2 {
                    continue;
                }
                let end = &points[points.len() - 1];
                // The direction the piece before ends with, and the one the line or arc after starts with.
                let before = out.last().map(|g| { let [_, dir, _] = g.at(g.length()); [dir[0], dir[1]] });
                let after = match rest.first() {
                    Some(Piece::Line(to)) if to.len() == 2 && distance(to, end) > 1e-9 => Some([(to[0] - end[0]) / distance(to, end), (to[1] - end[1]) / distance(to, end)]),
                    Some(Piece::Arc { centre, clockwise, .. }) if centre.len() == 2 && distance(end, centre) > 1e-6 => {
                        let (turn, radius) = (if *clockwise { -1.0 } else { 1.0 }, distance(end, centre));
                        Some([-turn * (end[1] - centre[1]) / radius, turn * (end[0] - centre[0]) / radius])
                    }
                    _ => None,
                };
                let Some(spans) = spline(&points, before, after) else { return bad };
                here = end.clone();
                out.extend(spans);
                continue;
            }
            Piece::Line(to) if to.len() == here.len() => {
                let length = distance(to, &here);
                Geo::Line { dir: to.iter().zip(&here).map(|(t, h)| (t - h) / length).collect(), from: here.clone(), length }
            }
            Piece::Arc { to, centre, clockwise } if here.len() == 2 && to.len() == 2 && centre.len() == 2 => {
                let radius = distance(&here, centre);
                // The arc starts where the path stands and takes its radius from there. What it is, a full turn
                // or by which angle it turns, is told from the points the file states: the end stated before
                // it and its own. Only a first piece has none before it and is told from where the machine
                // stands, with the slack that needs; too near to there it is not told at all (`FIRST_ARC`).
                let (from, slack) = match stated { Some(p) => (&p[..], ARC_TOLERANCE), None => (&here[..], ARC_SLACK) };
                if !(radius > 1e-6 && (distance(to, centre) - distance(from, centre)).abs() < slack) {
                    return bad;
                }
                if stated.is_none() && distance(to, from) < FIRST_ARC {
                    return Err(CommandError { key: "error-path-first-arc" });
                }
                let angle = |p: &[f64]| (p[1] - centre[1]).atan2(p[0] - centre[0]);
                let turn = if *clockwise { -1.0 } else { 1.0 };
                let full = distance(to, from) < 1e-9;
                let sweep = if full { std::f64::consts::TAU } else { (turn * (angle(to) - angle(from))).rem_euclid(std::f64::consts::TAU) };
                Geo::Arc { centre: [centre[0], centre[1]], radius, from_angle: angle(&here), turn, length: sweep * radius }
            }
            _ => return bad,
        };
        if geo.length() > 1e-9 {
            // The next piece starts where this one really ends, so the path has no jump.
            here = geo.at(geo.length())[0].clone();
            out.push(geo);
        }
        stated = Some(piece_end(piece));
        off = off.max(distance(&here, piece_end(piece)));
    }
    // Below a nanometre it is rounding.
    if out.is_empty() { bad } else { Ok((out, if off > 1e-9 { off } else { 0.0 })) }
}

/// A planned pose with what rounding has put outside an axis's range back on its end. A path that ends on the
/// end of a stroke, or runs along it, comes out of the sines and sums a few 1e-17 m beyond it, and would be
/// refused as leaving the travel range.
fn within(axes: &[AxisInfo], mut pose: Vec<f64>) -> Vec<f64> {
    for (p, axis) in pose.iter_mut().zip(axes) {
        let inside = p.clamp(axis.min, axis.max);
        if (*p - inside).abs() < 1e-9 {
            *p = inside;
        }
    }
    pose
}

/// A corner: the direction turns by more than 1 mrad from one piece to the next. The path stops there.
fn corner(a: &Geo, b: &Geo) -> bool {
    distance(&a.at(a.length())[1], &b.at(0.0)[1]) > 1e-3
}

// ---------------------------------------------------------------------------------------------
// Transition curves.

/// What a transition curve joins: a straight line through a point, or a circle with its curvature.
#[derive(Clone, Copy)]
enum Side {
    Line { point: [f64; 2], dir: [f64; 2] },
    Circle { centre: [f64; 2], curvature: f64 },
}

/// The transition curve from `a` to `b`, which are apart by `gap` where they are nearest: a line and a
/// circle that the line misses by the gap, a circle inside a wider one, or two circles side by side that
/// turn opposite ways. A curve whose curvature goes evenly from that of `a` to that of `b` joins two such
/// with no step in position, direction or curvature for exactly one length, found here by halving.
fn transition(a: Side, b: Side, gap: f64) -> Option<Geo> {
    // The curve laid from the origin along +x: the centre of the circle that fits it at its end.
    let centre_at_end = |k0: f64, k1: f64, length: f64| {
        let [end, dir, _] = Geo::Clothoid { from: [0.0, 0.0], heading: 0.0, k0, k1, length }.at(length);
        [end[0] - dir[1] / k1, end[1] + dir[0] / k1]
    };
    // To first order the gap is the change of curvature times length squared / 24.
    let longest = |change: f64| 2.0 * (24.0 * gap / change.abs()).sqrt();
    match (a, b) {
        (Side::Line { point, dir }, Side::Circle { centre, curvature: k }) => {
            let to_centre = [centre[0] - point[0], centre[1] - point[1]];
            // The centre is this far beside the line; the curve's own circle has to end up as far beside +x.
            let beside = (dir[0] * to_centre[1] - dir[1] * to_centre[0]).abs();
            let length = largest(0.0, longest(k), |length| length == 0.0 || centre_at_end(0.0, k, length)[1].abs() <= beside);
            let along = dir[0] * to_centre[0] + dir[1] * to_centre[1] - centre_at_end(0.0, k, length)[0];
            let from = [point[0] + along * dir[0], point[1] + along * dir[1]];
            (length > 0.0 && length < longest(k)).then(|| Geo::Clothoid { from, heading: dir[1].atan2(dir[0]), k0: 0.0, k1: k, length })
        }
        // The same curve run backwards: every direction and every turn is the other way round.
        (Side::Circle { centre, curvature }, Side::Line { point, dir }) => {
            let back = transition(Side::Line { point, dir: [-dir[0], -dir[1]] }, Side::Circle { centre, curvature: -curvature }, gap)?;
            let [end, dir, _] = back.at(back.length());
            Some(Geo::Clothoid { from: [end[0], end[1]], heading: (-dir[1]).atan2(-dir[0]), k0: curvature, k1: 0.0, length: back.length() })
        }
        (Side::Circle { centre: ca, curvature: k0 }, Side::Circle { centre: cb, curvature: k1 }) => {
            // The two circles of the curve start out touching and move apart (or into each other) as it
            // gets longer; it fits when their centres are as far apart as the real ones.
            let apart = |length: f64| if length == 0.0 { (1.0 / k1 - 1.0 / k0).abs() } else { distance(&centre_at_end(k0, k1, length), &[0.0, 1.0 / k0]) };
            let moved = (distance(&ca, &cb) - apart(0.0)).abs();
            let length = largest(0.0, longest(k1 - k0), |length| (apart(length) - apart(0.0)).abs() <= moved);
            // Turned so that the line between the curve's centres lies on the line between the real ones.
            let end = centre_at_end(k0, k1, length);
            let heading = (cb[1] - ca[1]).atan2(cb[0] - ca[0]) - (end[1] - 1.0 / k0).atan2(end[0]);
            let from = [ca[0] + heading.sin() / k0, ca[1] - heading.cos() / k0];
            (length > 0.0 && length < longest(k1 - k0)).then(|| Geo::Clothoid { from, heading, k0, k1, length })
        }
        _ => None,
    }
}

/// Puts transition curves where a line and an arc, or two arcs, meet in the same direction. There the
/// curvature steps, and with it the acceleration towards the centre, which is why `junction_speed` lets the
/// path pass such a place only at a crawl. A transition curve changes the curvature evenly over its length
/// L instead, so at the speed v the acceleration towards the centre changes at the rate v^3 dk / L (dk: the
/// step of the curvature, 1 / r for a line into an arc of radius r).
///
/// Such a curve does not fit between a line and a circle that touch: it needs the circle moved in by
///
/// ```text
/// e = L^2 / (24 r)        (first term; the code solves the exact length)
/// ```
///
//// So every blended arc keeps its centre and gets a radius smaller by its e, and the transition curves
/// replace about L / 2 of the pieces on either side of each junction. The planned path is then nowhere
/// further from the given one than the largest e, which is what is reported. One L serves the whole path:
/// the length the fastest junction needs to pass at the speed of its slower piece with 0.4 of the jerk
/// limit, L = v^3 dk / (0.4 j), but no longer than the tolerance allows on the tightest arc,
/// L = sqrt(24 e_max r_min). The speed v is that of the plan: the axis limits for `plan`, the motors for
/// `plan_time_optimal`. Where L is shorter than needed, `limits` lowers the speed on the curve.
///
/// With every arc moved in by e = L^2 / (24 r), all junctions come out with curves of about this one L:
/// two arcs turning the same way are apart by the difference of their e, two turning opposite ways by the
/// sum. An arc whose one end has to stay where it is given (at a corner, at an end of the path, or where a
/// spline joins it, see `ease`) and whose other end meets a line in its direction is moved another way: its
/// circle shrinks by s and moves towards the fixed end by s, so that it still passes through that end in the
/// same direction, and is then apart from the line by s (1 - cos phi), phi being the angle the arc turns
/// through. That gap is made e, so s = e / (1 - cos phi), for arcs of up to half a turn whose radius stays above
/// half of what it was; the new circle is nowhere on the arc further from the old one than about e either (the
/// exact figure is reported). An arc with both ends fixed, or with one end fixed and an arc at the other, stays as
/// it is; and where a curve would not fit (a wider arc moved in beside a tighter one that was not) both arcs stay
/// as given. Two arcs of the same curvature (to 1e-9 relative: one circle) get no curve between them. Pieces too
/// short for their curves halve L for the whole path, up to eight times; then the path stays as given. So does a
/// blend in which a curve would take more of an arc than it can reach, or whose length differs from the given one's
/// by more than the curves' lengths and the moved arcs' shrink times their angles (a whole turn gained or lost).
fn blend(geo: Vec<Geo>, axes: &[AxisInfo], tolerance: f64, speed: &dyn Fn(&Geo) -> f64) -> (Vec<Geo>, f64) {
    let n = geo.len();
    let k: Vec<f64> = geo.iter().map(|g| if let Geo::Arc { radius, turn, .. } = g { turn / radius } else { 0.0 }).collect();
    // Junction j is where piece j - 1 hands over to piece j; it gets a curve where both are lines or arcs that
    // meet in the same direction.
    let spline = |i: usize| matches!(geo[i], Geo::Spline { .. });
    let tangent: Vec<bool> = (0..=n).map(|j| j > 0 && j < n && !corner(&geo[j - 1], &geo[j]) && !spline(j - 1) && !spline(j)).collect();
    let free: Vec<bool> = (0..n).map(|i| k[i] != 0.0 && tangent[i] && tangent[i + 1]).collect();
    let line = |i: usize| matches!(geo[i], Geo::Line { .. });
    let half_turn = |i: usize| geo[i].length() * k[i].abs() <= std::f64::consts::PI;
    // An arc fixed at one end: Some(true) at its start, Some(false) at its end.
    let fixed: Vec<Option<bool>> = (0..n)
        .map(|i| match (k[i] != 0.0 && half_turn(i), tangent[i], tangent[i + 1]) {
            (true, true, false) if line(i - 1) => Some(false),
            (true, false, true) if line(i + 1) => Some(true),
            _ => None,
        })
        .collect();
    let tightest = (0..n).filter(|&i| free[i] || fixed[i].is_some()).map(|i| k[i].abs()).fold(0.0, f64::max);
    if tightest == 0.0 {
        return (geo, 0.0);
    }
    let jerk = axes.iter().map(|a| a.max_jerk).fold(f64::INFINITY, f64::min);
    let speed: Vec<f64> = geo.iter().map(speed).collect();
    let needed = (1..n).filter(|&j| tangent[j]).map(|j| speed[j - 1].min(speed[j]).powi(3) * (k[j] - k[j - 1]).abs() / (0.4 * jerk)).fold(0.0, f64::max);
    let mut length = (24.0 * tolerance / tightest).sqrt().min(needed);
    for _ in 0..8 {
        if let Some(blended) = blend_with(&geo, &k, &free, &fixed, &tangent, length) {
            return blended;
        }
        length /= 2.0;
    }
    (geo, 0.0)
}

/// `blend` for one length of the transition curves; `None` if a piece is too short for its curves.
fn blend_with(geo: &[Geo], k: &[f64], free: &[bool], fixed: &[Option<bool>], tangent: &[bool], length: f64) -> Option<(Vec<Geo>, f64)> {
    let n = geo.len();
    // The gap each arc leaves to a line beside it: moved in by it, or (fixed at one end) shrunk and moved.
    let mut inset: Vec<f64> = (0..n).map(|i| if free[i] || fixed[i].is_some() { k[i].abs() * length * length / 24.0 } else { 0.0 }).collect();
    // The circle each arc is laid on: centre and radius. A fixed arc's circle moves towards its fixed end.
    let circle = |inset: &[f64], i: usize| match &geo[i] {
        Geo::Arc { centre, radius, from_angle, turn, length } => Some(match fixed[i] {
            None => (*centre, radius - inset[i]),
            Some(at_start) => {
                let sweep = length / radius;
                let shrink = inset[i] / (1.0 - sweep.cos());
                let (sin, cos) = if at_start { from_angle.sin_cos() } else { (from_angle + turn * sweep).sin_cos() };
                ([centre[0] + shrink * cos, centre[1] + shrink * sin], radius - shrink)
            }
        }),
        _ => None,
    };
    for i in 0..n {
        if let (Some(_), Some((_, moved)), Geo::Arc { radius, .. }) = (fixed[i], circle(&inset, i), &geo[i]) {
            if moved < 0.5 * radius {
                inset[i] = 0.0;
            }
        }
    }
    // Curvatures this close are one (two arcs on one circle come out with radii apart by rounding), and pieces this
    // close touch: a curve between them would be built from rounding, and may take any angle of the arcs.
    let same = |j: usize| (k[j] - k[j - 1]).abs() <= 1e-9 * k[j].abs().max(k[j - 1].abs());
    // How far apart the two pieces of junction j are once the arcs are moved in.
    let gap = |inset: &[f64], j: usize| {
        let (tighter, wider) = if k[j].abs() > k[j - 1].abs() { (j, j - 1) } else { (j - 1, j) };
        let gap = if k[j] * k[j - 1] < 0.0 { inset[j] + inset[j - 1] } else { inset[tighter] - inset[wider] };
        if gap.abs() < 1e-12 { 0.0 } else { gap }
    };
    let fits = |inset: &[f64], j: usize| gap(inset, j) >= 0.0 && (gap(inset, j) == 0.0 || !same(j));
    while let Some(j) = (1..n).find(|&j| tangent[j] && !fits(&inset, j)) {
        (inset[j - 1], inset[j]) = (0.0, 0.0);
    }
    let side = |i: usize| match &geo[i] {
        Geo::Arc { turn, .. } => circle(&inset, i).map(|(centre, radius)| Side::Circle { centre, curvature: turn / radius }),
        Geo::Line { from, dir, .. } => Some(Side::Line { point: [from[0], from[1]], dir: [dir[0], dir[1]] }),
        Geo::Clothoid { .. } | Geo::Spline { .. } => None,
    };
    let mut curves: Vec<Option<Geo>> = vec![None];
    for j in 1..n {
        let gap = gap(&inset, j);
        curves.push(if tangent[j] && gap > 0.0 { Some(transition(side(j - 1)?, side(j)?, gap)?) } else { None });
        // A curve that turns half a circle is no blend, and what it takes of an arc could no longer be told
        // from a whole turn more.
        if matches!(curves[j], Some(Geo::Clothoid { k0, k1, length, .. }) if 0.5 * (k0.abs() + k1.abs()) * length >= std::f64::consts::PI) {
            return None;
        }
    }
    curves.push(None);
    // The most a curve can take of an arc beside it: it replaces about half its length there, so less than its length
    // times its larger curvature, as an angle.
    let reach = |c: &Option<Geo>| match c {
        Some(Geo::Clothoid { k0, k1, length, .. }) => length * k0.abs().max(k1.abs()) + 1e-9,
        _ => 0.0,
    };
    let (mut out, mut off, mut cut) = (Vec::new(), 0f64, 0f64);
    for (i, piece) in geo.iter().enumerate() {
        // Where the curves before and after the piece leave of it.
        let start = curves[i].as_ref().map(|c| c.at(c.length())[0].clone());
        let end = curves[i + 1].as_ref().map(|c| c.at(0.0)[0].clone());
        let reaches = (reach(&curves[i]), reach(&curves[i + 1]));
        cut += curves[i].as_ref().map_or(0.0, Geo::length);
        out.extend(curves[i].take());
        out.push(match piece {
            Geo::Line { from, dir, length } => {
                let from = start.unwrap_or(from.clone());
                let to = end.unwrap_or_else(|| piece.at(*length)[0].clone());
                let length = from.iter().zip(&to).zip(dir).map(|((f, t), d)| (t - f) * d).sum::<f64>();
                Geo::Line { from, dir: dir.clone(), length }
            }
            Geo::Arc { radius, from_angle, turn, length, .. } => {
                let (centre, moved) = circle(&inset, i)?;
                if moved <= 0.0 {
                    return None;
                }
                // The ends' angles seen from the new centre are those seen from the old one: a fixed end's centre
                // moves along the line to that end, and a free end is where the circle is nearest the line, in the
                // direction the old end had from the old centre.
                let angle = |p: &[f64]| (p[1] - centre[1]).atan2(p[0] - centre[0]);
                let sweep = length / radius;
                // What each curve takes of the arc, as the angle from the arc's own end to where the curve
                // meets it. (The angle between the two curves alone does not tell curves that overlap from
                // an arc: overlapping by more than the rest of the circle, they read as a short arc again,
                // and the path went round once more than the given one.)
                let taken = |by: &Option<Vec<f64>>, arc_end: f64, sign: f64| by.as_ref().map_or(0.0, |p| (sign * turn * (angle(p) - arc_end)).rem_euclid(std::f64::consts::TAU));
                let (before, after) = (taken(&start, *from_angle, 1.0), taken(&end, from_angle + turn * sweep, -1.0));
                // More than a curve can take is a wrapped angle, a whole turn off: refused.
                if before > reaches.0 || after > reaches.1 {
                    return None;
                }
                let left = sweep - before - after;
                // How far the new circle is from the old one on the arc: the gap where it meets the line, and for a
                // fixed arc at its free end r - |s u + (r - s) v|, with u and v at the angle phi apart.
                let shrink = radius - moved;
                // On its smaller circle the arc is shorter by up to the shrink times its angle.
                cut += shrink.abs() * sweep;
                let swing = if fixed[i].is_some() { radius - (radius * radius - 2.0 * shrink * moved * (1.0 - sweep.cos())).sqrt() } else { 0.0 };
                off = off.max(inset[i]).max(swing);
                // Curves that overlap leave less than nothing: a length below zero, refused below.
                Geo::Arc { centre, radius: moved, from_angle: start.as_deref().map_or(*from_angle, angle), turn: *turn, length: left * moved }
            }
            Geo::Spline { .. } => piece.clone(),
            Geo::Clothoid { .. } => return None,
        });
        if out[out.len() - 1].length() <= 0.0 {
            return None;
        }
    }
    // The last check: the blended path is as long as the given one but for what the curves and the moved arcs may
    // change. A whole turn gained or lost (2 pi r) is far more.
    let length = |path: &[Geo]| path.iter().map(Geo::length).sum::<f64>();
    if (length(&out) - length(geo)).abs() > cut + 1e-9 {
        return None;
    }
    Some((out, off))
}

/// How far the easing of a spline's end (`ease`) moves it at most, per change of curvature and square of the
/// length eased: the largest value of w^2 (1 - w)^3 / 2 for w from 0 to 1, at w = 0.4.
const EASE_PEAK: f64 = 0.01728;
/// The change of curvature per distance at the eased end is this many times the change of curvature over the
/// length eased: the third derivative of w^2 (1 - w)^3 / 2 at w = 0 is -9, and nowhere else is it larger.
const EASE_RATE: f64 = 9.0;

/// Eases a spline into the line or arc it meets: where the curvature steps at such a junction, the spline's end
/// is bent so that its curvature there is that of its neighbour, and the junction is passed at speed instead of
/// at the crawl `junction_speed` allows for the step.
///
/// The spline leaves the junction in the neighbour's direction at a speed by u of 1 (`spline`), so the step of its
/// second derivative by u across the direction is the step of the curvature, dk. Over the length l of the span
/// next to the junction (u from 0 to l, counted from the junction) the polynomial gets
///
/// ```text
/// delta(u) = D l^2 w^2 (1 - w)^3 / 2,   w = u / l
/// ```
///
/// added, D the change of the second derivative that makes the curvature that of the neighbour (across the
/// direction, of size dk). It has no position, direction or second derivative at w = 1, and none but D at w = 0:
/// the junction keeps its place and direction, the rest of the spline stays as it was, and the eased piece is a
/// polynomial of the fifth degree. It moves the spline by at most `EASE_PEAK` dk l^2 (at w = 0.4), which the
/// tolerance bounds:
///
/// ```text
/// l = sqrt(e / (0.01728 dk))
/// ```
///
/// but no longer than half the span, so that a via point never moves, and no longer than needed: at the
/// junction the curvature changes by `EASE_RATE` dk / l per distance, so the speed v of the slower of the two
/// pieces asks for l = 9 dk v^3 / (0.4 j) (with the margin of `SPLINE_MARGIN`). Each junction gets its own l.
/// Only junctions where the direction does not turn (no corner) are eased. The spline's shape between its via
/// points away from the junctions, and the via points themselves, stay as they are.
fn ease(mut geo: Vec<Geo>, axes: &[AxisInfo], tolerance: f64, speed: &dyn Fn(&Geo) -> f64) -> (Vec<Geo>, f64) {
    let jerk = axes.iter().map(|a| a.max_jerk).fold(f64::INFINITY, f64::min);
    let mut moved = 0f64;
    let mut j = 1;
    while j < geo.len() {
        let spline = |g: &Geo| matches!(g, Geo::Spline { .. });
        let (into, out_of) = (spline(&geo[j]) && !spline(&geo[j - 1]), spline(&geo[j - 1]) && !spline(&geo[j]));
        if (into || out_of) && !corner(&geo[j - 1], &geo[j]) {
            let (at, other) = if into { (j, j - 1) } else { (j - 1, j) };
            let Geo::Spline { c, h, .. } = &geo[at] else { unreachable!() };
            let (c, h) = (*c, *h);
            // The neighbour's curvature where it meets the spline, and the spline's own there.
            let target = &geo[other].at(if into { geo[other].length() } else { 0.0 })[2];
            let [d, d2, _] = poly_rates(&c, if into { 0.0 } else { h });
            let along = d[0] * d2[0] + d[1] * d2[1];
            let change = [0, 1].map(|i| target[i] - (d2[i] - along * d[i]));
            let dk = change[0].hypot(change[1]);
            let v = speed(&geo[other]).min(speed(&geo[at]));
            let needed = EASE_RATE * SPLINE_MARGIN * dk * v.powi(3) / (0.4 * jerk);
            let l = (tolerance / (EASE_PEAK * dk)).sqrt().min(needed).min(0.5 * h);
            if dk > 1e-9 && l > 0.0 {
                // The eased part and the rest, each a polynomial of its own with u from 0.
                let (mut eased, rest) = if into { (c, poly_shift(&c, l)) } else { (poly_shift(&c, h - l), c) };
                // delta expanded in powers of u: from the junction at u = 0, or towards it at u = l.
                let powers = if into { [0.5, -1.5 / l, 1.5 / (l * l), -0.5 / (l * l * l)] } else { [0.0, 0.5 / l, -1.0 / (l * l), 0.5 / (l * l * l)] };
                for (m, p) in powers.iter().enumerate() {
                    for i in 0..2 {
                        eased[m + 2][i] += p * change[i];
                    }
                }
                if let (Some(eased), Some(rest)) = (span(eased, l), span(rest, h - l)) {
                    let pieces = if into { [eased, rest] } else { [rest, eased] };
                    geo.splice(at..=at, pieces);
                    moved = moved.max(EASE_PEAK * dk * l * l);
                    j += 1;
                }
            }
        }
        j += 1;
    }
    (geo, moved)
}

// ---------------------------------------------------------------------------------------------
// Timing within the axis limits.

/// A change of speed by `dv` under an acceleration and a jerk limit, with no acceleration before and after:
/// the time of each of the two jerk phases and the time at constant acceleration between them.
fn ramp(dv: f64, a: f64, j: f64) -> (f64, f64) {
    let dv = dv.abs();
    if dv * j < a * a { ((dv / j).sqrt(), 0.0) } else { (a / j, dv / a - a / j) }
}

/// The distance such a ramp covers: its time at the mean of the two speeds, because it is symmetric.
fn ramp_distance(v0: f64, v1: f64, a: f64, j: f64) -> f64 {
    let (tj, ta) = ramp(v1 - v0, a, j);
    0.5 * (v0 + v1) * (2.0 * tj + ta)
}

/// The largest value from `low`, which passes, to `high` that passes `ok`, found by halving.
fn largest(mut low: f64, mut high: f64, ok: impl Fn(f64) -> bool) -> f64 {
    if ok(high) {
        return high;
    }
    for _ in 0..60 {
        let mid = 0.5 * (low + high);
        if ok(mid) { low = mid } else { high = mid }
    }
    low
}

/// What bounds the tool's speed, its acceleration or its jerk: per row a direction in axis space and the most
/// the rate may be along it. An axis limit is a row along its axis; a motor's rated speed is a row along the
/// tool movement that turns the motor.
type Bounds = Vec<(Vec<f64>, f64)>;

/// The velocity, acceleration and jerk limits of the axes as bounds.
fn axis_bounds(axes: &[AxisInfo]) -> [Bounds; 3] {
    let rows = |limit: fn(&AxisInfo) -> f64| axes.iter().enumerate().map(|(i, a)| ((0..axes.len()).map(|k| (k == i) as u8 as f64).collect(), limit(a))).collect();
    [rows(|a| a.max_vel), rows(|a| a.max_acc), rows(|a| a.max_jerk)]
}

/// Speed, acceleration and jerk along a piece that keep the tool within its bounds wherever on the piece.
fn limits(geo: &Geo, bounds: &[Bounds; 3]) -> [f64; 3] {
    let tightest = |per_row: &dyn Fn(&[f64]) -> f64| bounds.each_ref().map(|rows| rows.iter().map(|(row, most)| most / per_row(row)).fold(f64::INFINITY, f64::min));
    // The direction turns through the axes on a curve, so there a bound holds for the length of the vector.
    let any = || tightest(&|row| row.iter().map(|w| w * w).sum::<f64>().sqrt());
    match geo {
        Geo::Line { dir, .. } => tightest(&|row| row.iter().zip(dir).map(|(w, d)| w * d).sum::<f64>().abs()),
        Geo::Arc { radius, .. } => curved(*radius, 0.0, any()),
        // A transition curve counts as an arc of its tightest radius.
        Geo::Clothoid { k0, k1, length, .. } => curved(1.0 / k0.abs().max(k1.abs()), (k1 - k0).abs() / length, any()),
        // So does a spline span, with the fastest its curvature changes anywhere on it.
        Geo::Spline { tightest, rate, .. } => curved(1.0 / tightest, *rate, any()),
    }
}

/// `limits` on a curve of radius `r` whose curvature changes by `dk` per distance (0 on an arc), from the
/// speed, acceleration and jerk the tool may have in any direction.
///
/// Along the path (t) and towards the centre (n), at the speed v:
///
/// ```text
/// acceleration = t dv/dt + n v^2 / r
/// jerk         = t (d2v/dt2 - v^3 / r^2) + n (3 v dv/dt / r + v^3 dk)
/// ```
///
/// The limits are shared out 3 : 4 : 5. Acceleration: 0.6 along, 0.8 towards the centre. Jerk: 0.4 for the
/// ramps and 0.4 for v^3 / r^2, together 0.8 along, and 0.6 towards the centre: all of it for 3 v dv/dt / r
/// on an arc, and on a transition curve 0.4 for v^3 dk and 0.2 for 3 v dv/dt / r.
fn curved(r: f64, dk: f64, [v, a, j]: [f64; 3]) -> [f64; 3] {
    let v = v.min((0.8 * a * r).sqrt()).min((0.4 * j * r * r).cbrt());
    let (v, share) = if dk > 0.0 { (v.min((0.4 * j / dk).cbrt()), 0.2 / 3.0) } else { (v, 0.2) };
    [v, (0.6 * a).min(share * j * r / v), 0.4 * j]
}

/// How fast the path may pass from one piece into the next. At a corner not at all. Elsewhere the direction
/// may turn a little and the curvature may jump (line into arc), which steps the axis speeds and
/// accelerations. Taken over one profile cycle, each of the two steps may use half an axis's jerk limit.
fn junction_speed(a: &Geo, b: &Geo, axes: &[AxisInfo], cycle: f64) -> f64 {
    if corner(a, b) {
        return 0.0;
    }
    let ([_, dir0, curve0], [_, dir1, curve1]) = (a.at(a.length()), b.at(0.0));
    let per_axis = |i: usize, axis: &AxisInfo| {
        let half = 0.5 * axis.max_jerk;
        (half * cycle * cycle / (dir1[i] - dir0[i]).abs()).min((half * cycle / (curve1[i] - curve0[i]).abs()).sqrt())
    };
    axes.iter().enumerate().map(|(i, axis)| per_axis(i, axis)).fold(f64::INFINITY, f64::min)
}

/// Plans the timing of a path within the velocity, acceleration and jerk limits of every axis, from rest to rest.
///
/// Each piece gets one speed, acceleration and jerk limit along the path that hold on all of it (`limits`),
/// and each junction a highest speed (`junction_speed`). Look-ahead then lowers the junction speeds until
/// every piece is long enough to change from the speed at its start to the speed at its end; a tight arc or
/// a corner far ahead so slows the pieces before it. On each piece the speed goes by a jerk-limited ramp to
/// the highest speed the piece has room for, stays there, and goes by a second ramp to the speed at its end.
///
/// With a `tolerance` above zero the path gets transition curves first (`blend`), which let it pass from a
/// line into an arc at speed, and is off the given pieces by up to the tolerance (`deviation` says by how much).
pub fn plan(axes: &[AxisInfo], start: &[f64], pieces: &[Piece], cycle: f64, tolerance: f64) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
    let (geo, bounds) = (geometry(axes, start, pieces, tolerance, &axis_speed(axes))?.0, axis_bounds(axes));
    let limits: Vec<[f64; 3]> = geo.iter().map(|g| limits(g, &bounds)).collect();
    Ok(timed(axes, &geo, &limits, cycle))
}

/// The timing of `plan` for pieces with their speed, acceleration and jerk limits along the path.
fn timed(axes: &[AxisInfo], geo: &[Geo], limits: &[[f64; 3]], cycle: f64) -> Vec<(f64, Vec<f64>)> {
    let n = geo.len();
    // Two cycles at steady speed on either side of a junction, so that the jerk of a ramp and the steps of
    // the junction never fall into the same three cycles.
    let hold = 2.0 * cycle;
    // The length piece i needs to change between two speeds.
    let need = |i: usize, v0: f64, v1: f64| hold * (v0 + v1) + ramp_distance(v0, v1, limits[i][1], limits[i][2]);

    // Speed at the start of each piece and at the end of the last; at rest at both ends of the path.
    let mut speed = vec![0.0; n + 1];
    for i in 1..n {
        // A piece too short for the steady cycles at both of its ends is passed slowly enough to have them.
        let short = geo[i - 1].length().min(geo[i].length()) / (2.0 * hold);
        speed[i] = junction_speed(&geo[i - 1], &geo[i], axes, cycle).min(limits[i - 1][0]).min(limits[i][0]).min(short);
    }
    // Look-ahead: no junction faster than the piece after it can brake from, or than the piece before it can reach.
    for i in (0..n).rev() {
        let after = speed[i + 1];
        speed[i] = largest(after.min(speed[i]), speed[i], |v| need(i, v, after) <= geo[i].length());
    }
    for i in 0..n {
        let before = speed[i];
        speed[i + 1] = largest(before.min(speed[i + 1]), speed[i + 1], |v| need(i, before, v) <= geo[i].length());
    }

    // Per piece the phases of constant jerk as (duration, jerk): steady, up to the top speed, cruise, down, steady.
    let mut phases = Vec::new();
    for i in 0..n {
        let ([v_max, a, j], v0, v1, length) = (limits[i], speed[i], speed[i + 1], geo[i].length());
        let covered = |top: f64| hold * (v0 + v1) + ramp_distance(v0, top, a, j) + ramp_distance(top, v1, a, j);
        let top = largest(v0.max(v1), v_max, |top| covered(top) <= length);
        let (up, down, cruise) = (ramp(top - v0, a, j), ramp(top - v1, a, j), ((length - covered(top)) / top).max(0.0));
        let steady = |v: f64| if v > 0.0 { hold } else { 0.0 };
        phases.push([(steady(v0), 0.0), (up.0, j), (up.1, 0.0), (up.0, -j), (cruise, 0.0), (down.0, -j), (down.1, 0.0), (down.0, j), (steady(v1), 0.0)]);
    }

    let duration = |i: usize| phases[i].iter().map(|p| p.0).sum::<f64>();
    let total: f64 = (0..n).map(duration).sum();
    let mut points = Vec::new();
    for k in 0..=(total / cycle - 1e-9).ceil() as usize {
        // The piece this moment falls into, and the time since its start.
        let (mut i, mut t) = (0, k as f64 * cycle);
        while i + 1 < n && t >= duration(i) {
            t -= duration(i);
            i += 1;
        }
        let (mut s, mut v, mut a) = (0.0, speed[i], 0.0);
        for (time, jerk) in phases[i] {
            let h = t.min(time);
            s += v * h + a * h * h / 2.0 + jerk * h * h * h / 6.0;
            v += a * h + jerk * h * h / 2.0;
            a += jerk * h;
            t -= h;
        }
        points.push((k as f64 * cycle, within(axes, geo[i].at(s.min(geo[i].length()))[0].clone())));
    }
    points
}

// ---------------------------------------------------------------------------------------------
// Timing as fast as the motors allow.

/// Plans the timing of a path as fast as the motors allow, within the jerk limits of the axes.
///
/// The timing is the one of `plan`: one speed and one acceleration limit per piece, look-ahead over the
/// junctions and jerk-limited ramps, with the jerk limits of the axes. Only the speed and the acceleration
/// limits come from the motors instead of the axes. The machine's rigid model gives, at nine poses along
/// each piece, how fast each motor turns per speed of the tool and how much torque it needs per acceleration
/// of the tool, plus the torque that holds the tool there. A motor may turn at its rated speed, and may give
/// `reserve` times its torque-speed curve (which is level up to that speed) less the holding torque, whichever
/// way it has to push. The rest of the curve is for friction and elasticity, which the rigid model leaves out.
///
/// So on a line the plan uses the motors' torque and speed in full, apart from the holding torque being
/// counted against both directions. On an arc it shares them out as `curved` does, which leaves a part
/// unused. The velocity and acceleration limits of the axes play no part.
pub fn plan_time_optimal(machine: &dyn Machine, drive: &DriveParams, reserve: f64, start: &[f64], pieces: &[Piece], cycle: f64, tolerance: f64) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
    let ((geo, _), limited) = time_optimal_geometry(machine, drive, reserve, start, pieces, tolerance)?;
    Ok(timed(machine.axes(), &geo, &limited, cycle))
}

/// The same path as `plan` times, transition curves included, their lengths for the speeds the motors allow, with
/// the deviation and the motors' limits per piece; refused where a piece has no torque left beyond holding the tool
/// or turns no motor.
fn time_optimal_geometry(machine: &dyn Machine, drive: &DriveParams, reserve: f64, start: &[f64], pieces: &[Piece], tolerance: f64) -> Result<((Vec<Geo>, f64), Vec<[f64; 3]>), CommandError> {
    let piece_limits = motor_limits(machine, drive, reserve);
    let path = geometry(machine.axes(), start, pieces, tolerance, &|piece| piece_limits(piece)[0])?;
    let mut limited = Vec::new();
    for piece in &path.0 {
        let limits = piece_limits(piece);
        if !limits.iter().all(|l| l.is_finite() && *l > 0.0) {
            return Err(CommandError { key: "error-path-torque" });
        }
        limited.push(limits);
    }
    Ok((path, limited))
}

/// `deviation` for the path `plan_time_optimal` plans: its transition curves are as long as the speeds the
/// motors allow need them to be, not the speeds of the axis limits. Refused as that plan is.
pub fn deviation_time_optimal(machine: &dyn Machine, drive: &DriveParams, reserve: f64, start: &[f64], pieces: &[Piece], tolerance: f64) -> Result<f64, CommandError> {
    Ok(time_optimal_geometry(machine, drive, reserve, start, pieces, tolerance)?.0.1)
}

/// The speed, acceleration and jerk limits along a piece that `plan_time_optimal` uses: speed and acceleration
/// from the motors, jerk from the axes.
fn motor_limits<'a>(machine: &'a dyn Machine, drive: &'a DriveParams, reserve: f64) -> impl Fn(&Geo) -> [f64; 3] + 'a {
    let axes = machine.axes();
    let [_, _, jerk] = axis_bounds(axes);
    let (torque, h) = (reserve * drive.torque_curve(drive.rated_speed).1, 1e-3);
    move |piece| {
        let per_motor = |f: &dyn Fn(&mut [f64])| {
            let mut out = vec![0.0; machine.motors().len()];
            f(&mut out);
            out
        };
        let (mut vel, mut acc) = (Vec::new(), Vec::new());
        for i in 0..=8 {
            let pose = &piece.at(piece.length() * i as f64 / 8.0)[0];
            let unit = |axis: usize, size: f64| -> Vec<f64> { (0..axes.len()).map(|k| if k == axis { size } else { 0.0 }).collect() };
            let hold = per_motor(&|out| machine.feedforward_torque(pose, &unit(0, 0.0), out));
            // Per axis and motor: the torque for an acceleration of 1 m/s^2, and the angle per metre.
            let per_acc: Vec<Vec<f64>> = (0..axes.len()).map(|a| per_motor(&|out| machine.feedforward_torque(pose, &unit(a, 1.0), out))).collect();
            let angle = |a: usize, sign: f64| per_motor(&|out| machine.inverse(&pose.iter().zip(unit(a, sign * h)).map(|(p, d)| p + d).collect::<Vec<_>>(), out));
            let turn: Vec<Vec<f64>> = (0..axes.len()).map(|a| angle(a, 1.0).iter().zip(angle(a, -1.0)).map(|(ahead, behind)| (ahead - behind) / (2.0 * h)).collect()).collect();
            for m in 0..hold.len() {
                vel.push((turn.iter().map(|per_axis| per_axis[m]).collect(), drive.rated_speed));
                acc.push((per_acc.iter().map(|per_axis| per_axis[m] - hold[m]).collect(), torque - hold[m].abs()));
            }
        }
        limits(piece, &[vel, acc, jerk.clone()])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axes() -> [AxisInfo; 2] {
        let axis = |max_vel: f64, max_acc: f64| AxisInfo { id: "", key: "", unit: "m", min: 0.0, max: 1.0, max_vel, max_acc, max_jerk: 100.0 };
        [axis(0.8, 8.0), axis(0.4, 6.0)]
    }

    #[test]
    fn a_path_file_is_read_line_by_line() {
        let pieces = parse("# a corner rounded off\nline, 0.5, 0.35\n\narc,0.6,0.25,0.5,0.25,cw\nline,0.6,0.05\n").unwrap();
        assert_eq!(pieces.len(), 3);
        assert_eq!(pieces[0], Piece::Line(vec![0.5, 0.35]));
        assert_eq!(pieces[1], Piece::Arc { to: vec![0.6, 0.25], centre: vec![0.5, 0.25], clockwise: true });
        assert_eq!(parse("spline, 0.7,0.2").unwrap(), [Piece::Spline(vec![0.7, 0.2])]);
        for bad in ["circle,1,2", "line", "line,a", "arc,0.6,0.25,0.5,0.25", "arc,0.6,0.25,0.5,0.25,left", "line,inf", "spline", "spline,0.5,nan"] {
            assert_eq!(parse(&format!("line,0.5\n{bad}")), Err(bad.to_string()));
        }
        // An arc whose end is not on the circle through the start, an arc on one axis and a point with the
        // wrong number of positions cannot be laid out.
        let refused = |start: &[f64], text: &str| plan(&axes()[..start.len()], start, &parse(text).unwrap(), 1e-3, 0.0).unwrap_err().key;
        assert_eq!(refused(&[0.5, 0.35], "arc,0.6,0.26,0.5,0.25,cw"), "error-path");
        // A start up to 0.1 mm off the circle (the setpoint of a machine at rest) is run; one a millimetre off
        // is not.
        let geo = given(&[0.0999876, 0.3500458], &parse("arc,0.3,0.35,0.2,0.35,cw").unwrap()).unwrap().0;
        assert!((geo[0].length() - std::f64::consts::PI * 0.1).abs() < 1e-4, "{} m", geo[0].length());
        assert_eq!(refused(&[0.101, 0.35], "arc,0.3,0.35,0.2,0.35,cw"), "error-path");
        assert_eq!(refused(&[0.5], "arc,0.6,0.25,0.5,0.25,cw"), "error-path");
        assert_eq!(refused(&[0.5, 0.35], "line,0.6"), "error-path");
        assert_eq!(refused(&[0.5, 0.35], ""), "error-path");
        // A spline needs two axes too, and a point with two positions.
        assert_eq!(refused(&[0.5], "spline,0.6"), "error-path");
        assert_eq!(refused(&[0.5, 0.35], "spline,0.6,0.3,0.1"), "error-path");
    }

    /// An arc with a chord below 0.1 mm ran as a full circle: "ends where it starts" was judged with the slack
    /// that only the machine's rest setpoint needs. Around (0.3, 0.25) with r = 0.1 m, from the top:
    /// (a) after a line, an arc of 0.8 mrad (chord 80 um) is 80 um long and the path stays above z = 0.3499
    /// (it went down to 0.15); an arc back to the line's end is still a full turn, 0.6283 m.
    /// (b) As the first piece an arc that ends within 1 mm of where the machine stands is refused, because a
    /// full turn cannot be told from a short arc there: a 130 um arc written from the start pose (0.1, 0.35)
    /// ran as a full turn from the rest setpoint 47 um off it. So are the full turn and an arc of 0.5 mrad; a
    /// first arc of 2 mm is run as that. With its start stated by a line first, each is what the file says:
    /// the arc of 130 um or 50 um, also from a setpoint 47 um off, and the full turn.
    /// (c) A later arc whose end is 90 um off its circle is refused (it ended 90 um short, unsaid); one 8 um
    /// off is run and the 8 um are reported, on their own and on top of what transition curves take. Four
    /// quarter turns around one centre, each stated 9 um further out than the one before, end 36 um off: every
    /// arc keeps the radius the path has, so the offsets add up, and the sum is what is reported.
    #[test]
    fn a_short_arc_is_not_a_full_turn_and_an_arc_off_its_end_is_reported() {
        let (axes, top) = (axes(), vec![0.3, 0.35]);
        let arc = |by: f64, r: f64| Piece::Arc { to: vec![0.3 + r * by.sin(), 0.25 + r * by.cos()], centre: vec![0.3, 0.25], clockwise: true };
        let lengths = |start: &[f64], pieces: &[Piece]| given(start, pieces).unwrap().0.iter().map(Geo::length).collect::<Vec<f64>>();
        let short = [Piece::Line(top.clone()), arc(8e-4, 0.1), Piece::Line(vec![0.5, 0.35])];
        assert!((lengths(&[0.1, 0.35], &short)[1] - 8e-5).abs() < 1e-12, "{:?}", lengths(&[0.1, 0.35], &short));
        let lowest = plan(&axes, &[0.1, 0.35], &short, 1e-3, 0.0).unwrap().iter().map(|p| p.1[1]).fold(f64::INFINITY, f64::min);
        assert!(lowest > 0.3499, "the path goes down to z = {lowest}");
        assert_eq!(deviation(&axes, &[0.1, 0.35], &short, 0.0).unwrap(), 0.0);
        assert!((lengths(&[0.1, 0.35], &[Piece::Line(top.clone()), arc(0.0, 0.1)])[1] - std::f64::consts::TAU * 0.1).abs() < 1e-12);

        let (pose, rest) = (vec![0.1, 0.35], [0.0999876, 0.3500458]);
        let up = |by: f64| Piece::Arc { to: vec![0.2 - 0.1 * by.cos(), 0.35 + 0.1 * by.sin()], centre: vec![0.2, 0.35], clockwise: true };
        for (start, first) in [(&rest[..], up(1.3e-3)), (&rest[..], up(0.0)), (&pose[..], up(0.0)), (&top[..], arc(5e-4, 0.1)), (&top[..], arc(9.9e-3, 0.1))] {
            assert_eq!(deviation(&axes, start, &[first.clone()], 0.0).unwrap_err().key, "error-path-first-arc", "{first:?} from {start:?}");
        }
        assert!((lengths(&rest, &[up(0.02)])[0] - 2e-3).abs() < 1e-4);
        for (by, length) in [(1.3e-3, 1.3e-4), (0.0, std::f64::consts::TAU * 0.1)] {
            let stated = lengths(&rest, &[Piece::Line(pose.clone()), up(by)]);
            assert!(stated.len() == 2 && (stated[1] - length).abs() < 1e-7, "{stated:?}");
        }
        assert!((lengths(&top, &[Piece::Line(top.clone()), arc(5e-4, 0.1)])[0] - 5e-5).abs() < 1e-12);
        let sagged = lengths(&[0.300012, 0.350046], &[Piece::Line(top.clone()), arc(5e-4, 0.1)]);
        assert!(sagged.len() == 2 && (sagged[1] - 5e-5).abs() < 1e-12, "{sagged:?}");

        let off_by = |by: f64| [Piece::Line(top.clone()), arc(std::f64::consts::FRAC_PI_2, 0.1 + by), Piece::Line(vec![0.4, 0.05])];
        assert_eq!(deviation(&axes, &[0.1, 0.35], &off_by(-9e-5), 0.0).unwrap_err().key, "error-path");
        let reported = deviation(&axes, &[0.1, 0.35], &off_by(8e-6), 0.0).unwrap();
        assert!((reported - 8e-6).abs() < 1e-12, "{reported} m reported");
        let spiral: Vec<Piece> = std::iter::once(Piece::Line(top.clone())).chain((1..=4).map(|k| arc(k as f64 * std::f64::consts::FRAC_PI_2, 0.1 + 9e-6 * k as f64))).collect();
        let added = deviation(&axes, &[0.1, 0.35], &spiral, 0.0).unwrap();
        assert!((added - 3.6e-5).abs() < 1e-12, "{added} m reported");
        let blended = deviation(&axes, &[0.1, 0.35], &off_by(8e-6), 1e-4).unwrap();
        assert!((blended - 1.08e-4).abs() < 1e-6,"{blended} m reported with transition curves");
    }

    /// Splines: smooth curves through via points.
    /// (a) By hand: one spline piece between two points at rest has no direction to keep and no curvature at
    /// its ends, so it is the straight line between them, and u is the distance along it.
    /// (b) Geometry of four paths (between two lines; alone, from rest to rest; out of an arc and into an arc;
    /// a tight S of via points 20 mm apart): every via point is where one span hands over to the next, the
    /// path has no step in position or direction anywhere, none in curvature from span to span, and the curve
    /// leaves and meets its neighbours in their direction.
    /// (c) Timing by both the limits: speed, acceleration and jerk of both axes, by differences over one cycle
    /// as for arcs, stay within the axis limits (margin 0.1 %), from the start to the end of the path, and the
    /// planned poses pass every via point within half of what the tool travels in a cycle.
    #[test]
    fn a_spline_passes_through_its_via_points_within_the_axis_limits() {
        let (axes, cycle) = (axes(), 1e-3);
        let straight = plan(&axes, &[0.1, 0.1], &parse("spline,0.5,0.4").unwrap(), cycle, 0.0).unwrap();
        // On the line through (0.1, 0.1) and (0.5, 0.4): 3 (x - 0.1) = 4 (z - 0.1).
        assert!(straight.iter().all(|(_, p)| (3.0 * (p[0] - 0.1) - 4.0 * (p[1] - 0.1)).abs() < 1e-12) && distance(&straight[straight.len() - 1].1, &[0.5, 0.4]) < 1e-12);
        let [Geo::Spline { travelled, .. }] = &given(&[0.1, 0.1], &parse("spline,0.5,0.4").unwrap()).unwrap().0[..] else { panic!("one span") };
        assert!((travelled[SPLINE_PARTS] - 0.5).abs() < 1e-14 && (travelled[16] - 0.125).abs() < 1e-14);

        let paths = [
            ([0.1, 0.35], "line,0.3,0.35\nspline,0.4,0.3\nspline,0.5,0.32\nspline,0.6,0.2\nline,0.6,0.05"),
            ([0.2, 0.1], "spline,0.3,0.2\nspline,0.5,0.25\nspline,0.7,0.1"),
            ([0.1, 0.5], "line,0.3,0.5\narc,0.4,0.4,0.3,0.4,cw\nspline,0.45,0.3\nspline,0.55,0.25\narc,0.65,0.15,0.55,0.15,cw\nline,0.65,0.05"),
            ([0.3, 0.2], "line,0.4,0.2\nspline,0.42,0.21\nspline,0.44,0.19\nspline,0.46,0.21\nspline,0.48,0.2\nline,0.6,0.2"),
        ];
        for (start, text) in paths {
            let pieces = parse(text).unwrap();
            let geo = given(&start, &pieces).unwrap().0;
            let via: Vec<&Vec<f64>> = pieces.iter().filter_map(|p| if let Piece::Spline(to) = p { Some(to) } else { None }).collect();
            let ends: Vec<Vec<f64>> = geo.iter().filter(|g| matches!(g, Geo::Spline { .. })).map(|g| g.at(g.length())[0].clone()).collect();
            assert_eq!(ends.len(), via.len(), "{text}");
            for (end, via) in ends.iter().zip(&via) {
                assert!(distance(end, via) < 1e-12, "{text}: span ends at {end:?}, via point {via:?}");
            }
            for pair in geo.windows(2) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps = [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                let both = matches!(pair, [Geo::Spline { .. }, Geo::Spline { .. }]);
                assert!(steps[0] < 1e-12 && steps[1] < 1e-9 && (!both || steps[2] < 1e-6), "{text}: steps {steps:?}");
            }
            // With a tolerance of 0.1 mm the spline is eased into the lines and arcs it meets (`ease`), and an arc beside
            // it gets its transition curve towards the line at its other end (`blend`): no step in position, direction
            // or curvature at any junction, every planned pose within the reported deviation of the path as given, the
            // reported deviation within the tolerance, the axis limits kept, the via points still passed, and faster.
            let (eased, reported) = geometry(&axes, &start, &pieces, 1e-4, &axis_speed(&axes)).unwrap();
            assert!(reported <= 1e-4 + 1e-12 && (reported > 0.0) == (eased.len() > geo.len()), "{text}: {reported} m reported, {} pieces from {}", eased.len(), geo.len());
            for pair in eased.windows(2) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps = [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-6, "{text}, eased: steps {steps:?}");
            }
            let (smooth, given_plan) = (plan(&axes, &start, &pieces, cycle, 1e-4).unwrap(), plan(&axes, &start, &pieces, cycle, 0.0).unwrap());
            let peak = peak_rates(&smooth, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "{text}, eased, axis {i}: {peak:?}");
            }
            let line = polyline(&geo);
            let furthest = smooth.iter().map(|p| off_line(&line, &p.1)).fold(0.0, f64::max);
            assert!(furthest <= reported + 1e-7, "{text}: {furthest} m off the given path, {reported} m reported");
            for via in &via {
                let nearest = smooth.iter().map(|p| distance(&p.1, via)).fold(f64::INFINITY, f64::min);
                assert!(nearest <= 0.5 * 0.8 * cycle * 1.01, "{text}, eased: {nearest} m past the via point {via:?}");
            }
            let (eased_time, given_time) = (smooth[smooth.len() - 1].0, given_plan[given_plan.len() - 1].0);
            assert!(reported == 0.0 || eased_time < given_time, "{text}: {eased_time} s eased against {given_time} s");
            println!("eased: {eased_time} s against {given_time} s, off by {furthest} m of {reported} m reported");

            let points = plan(&axes, &start, &pieces, cycle, 0.0).unwrap();
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "{text}, axis {i}: {peak:?}");
            }
            let last = points.len() - 1;
            let end = geo[geo.len() - 1].at(geo[geo.len() - 1].length())[0].clone();
            assert!(points[0].1 == start && distance(&points[last].1, &end) < 1e-9 && distance(&points[last].1, &points[last - 1].1) < 1e-6, "{text}");
            for via in &via {
                let nearest = points.iter().map(|p| distance(&p.1, via)).fold(f64::INFINITY, f64::min);
                assert!(nearest <= 0.5 * 0.8 * cycle * 1.01, "{text}: {nearest} m past the via point {via:?}");
            }
            println!("{} s, peak rates {peak:?}: {}", points[last].0, text.replace('\n', " | "));
        }

        // A via point straight back along the line before it: the curve would have to turn on the spot.
        assert_eq!(plan(&axes, &[0.1, 0.35], &parse("line,0.5,0.35\nspline,0.3,0.35").unwrap(), cycle, 0.0).unwrap_err().key, "error-path");
        // A via point given twice counts once, and a spline of nothing but where the path stands is no piece.
        let twice = given(&[0.2, 0.1], &parse("spline,0.3,0.2\nspline,0.3,0.2\nspline,0.5,0.25").unwrap()).unwrap().0;
        assert_eq!(twice.len(), 2);
        assert_eq!(plan(&axes, &[0.2, 0.1], &parse("spline,0.2,0.1").unwrap(), cycle, 0.0).unwrap_err().key, "error-path");
    }

    /// Rates of the planned poses by differences over one cycle: the largest speed, acceleration and jerk
    /// of each axis.
    fn peak_rates(points: &[(f64, Vec<f64>)], cycle: f64) -> [[f64; 2]; 3] {
        let mut peak = [[0f64; 2]; 3];
        for w in points.windows(4) {
            for i in 0..2 {
                let p = [w[0].1[i], w[1].1[i], w[2].1[i], w[3].1[i]];
                let rates = [(p[1] - p[0]) / cycle, (p[2] - 2.0 * p[1] + p[0]) / cycle.powi(2), (p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0]) / cycle.powi(3)];
                for (peak, rate) in peak.iter_mut().zip(rates) {
                    peak[i] = peak[i].max(rate.abs());
                }
            }
        }
        peak
    }

    /// A path as points 0.05 mm apart along it, whose straight pieces cut a curve of 5 mm radius by 0.06 um at most.
    fn polyline(geo: &[Geo]) -> Vec<Vec<f64>> {
        let mut out = vec![geo[0].at(0.0)[0].clone()];
        for g in geo {
            let parts = (g.length() / 5e-5).ceil().max(1.0) as usize;
            out.extend((1..=parts).map(|k| g.at(g.length() * k as f64 / parts as f64)[0].clone()));
        }
        out
    }

    /// How far a point is from a path given as `polyline`: from the nearest of its straight pieces.
    fn off_line(line: &[Vec<f64>], p: &[f64]) -> f64 {
        line.windows(2)
            .map(|w| {
                let (ab, ap) = ([w[1][0] - w[0][0], w[1][1] - w[0][1]], [p[0] - w[0][0], p[1] - w[0][1]]);
                let t = ((ab[0] * ap[0] + ab[1] * ap[1]) / (ab[0] * ab[0] + ab[1] * ab[1]).max(1e-300)).clamp(0.0, 1.0);
                (ap[0] - t * ab[0]).hypot(ap[1] - t * ab[1])
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// A line, an arc that leaves it tangentially, and a line that leaves the arc tangentially.
    /// (a) Speed, acceleration and jerk of both axes, taken from the planned poses by differences over one
    /// cycle, stay within the axis limits. Margin 0.1 %, for taking differences instead of derivatives. (b) Every planned pose lies on the path. And the
    /// hand calculations: on the arc of radius 0.1 m the speed is at most sqrt(a r) with the smaller
    /// acceleration limit, 6 m/s^2, so 0.775 m/s; the planner keeps 0.8 of that limit for the centripetal
    /// acceleration and the axis speed limit of 0.4 m/s is lower still, so the arc runs at 0.4 m/s. Into
    /// and out of the arc the acceleration towards the centre steps by v^2 / r, and that step within one
    /// cycle may use half the jerk limit: v = sqrt(0.5 * 100 * 0.001 * 0.1) = 0.0707 m/s.
    #[test]
    fn a_planned_path_stays_within_the_axis_limits_and_on_its_pieces() {
        let (axes, cycle) = (axes(), 1e-3);
        let pieces = parse("line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw\nline,0.6,0.05").unwrap();
        let points = plan(&axes, &[0.1, 0.35], &pieces, cycle, 0.0).unwrap();
        let peak = peak_rates(&points, cycle);
        for (i, axis) in axes.iter().enumerate() {
            assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "axis {i}: {peak:?}");
        }
        // The limits are used: full speed and acceleration on the first line, most of the jerk limit.
        assert!(peak[0][0] > 0.999 * 0.8 && peak[1][0] > 0.99 * 8.0 && peak[2][0] > 0.95 * 100.0, "{peak:?}");

        let on_arc = |p: &[f64]| p[0] > 0.5 + 1e-9 && p[1] > 0.25 + 1e-9;
        let (mut off_path, mut arc_speed, mut junction_speed) = (0f64, 0f64, f64::INFINITY);
        for w in points.windows(2) {
            let p = &w[1].1;
            let off = if on_arc(p) { (distance(p, &[0.5, 0.25]) - 0.1).abs() } else { (p[1] - 0.35).abs().min((p[0] - 0.6).abs()) };
            off_path = off_path.max(off);
            let speed = distance(&w[0].1, p) / cycle;
            if on_arc(p) && on_arc(&w[0].1) {
                arc_speed = arc_speed.max(speed);
            } else if on_arc(p) != on_arc(&w[0].1) {
                junction_speed = junction_speed.min(speed);
            }
        }
        assert!(off_path < 1e-12, "{off_path} m off the path");
        assert!(arc_speed <= (6.0f64 * 0.1).sqrt() && (arc_speed - 0.4).abs() < 1e-3, "{arc_speed} m/s on the arc");
        assert!((junction_speed - 0.0707).abs() < 1e-3, "{junction_speed} m/s into the arc");
        // Rest to rest, from the start to the end of the path.
        let last = points.len() - 1;
        assert!(points[0].1 == [0.1, 0.35] && distance(&points[last].1, &[0.6, 0.05]) < 1e-12);
        assert!(distance(&points[1].1, &points[0].1) < 1e-6 && distance(&points[last].1, &points[last - 1].1) < 1e-6);
        println!("duration {} s, peak rates {peak:?}, arc {arc_speed} m/s, junction {junction_speed} m/s", points[last].0);
    }

    /// Transition curves on the same line, arc, line. Without them the path crawls into and out of the arc at
    /// 0.0707 m/s (the test above). With a tolerance of 0.1 mm:
    /// (a) The curve may be L = sqrt(24 e r) = sqrt(24 * 0.0001 * 0.1) = 15.49 mm long. To enter the arc at its
    /// 0.4 m/s with 0.4 of the jerk limit it would need v^3 / (0.4 j r) = 0.064 / (0.4 * 100 * 0.1) = 16 mm, so
    /// the tolerance decides: the arc moves in by the full 0.1 mm, and the speed on the curve is
    /// (0.4 j r L)^(1/3) = (40 * 0.1 * 0.01549)^(1/3) = 0.3957 m/s, five and a half times the crawl.
    /// (b) With a tolerance of 1 mm the curve is the 16 mm it needs and no longer, and the path is off by
    /// e = L^2 / (24 r) = 0.016^2 / 2.4 = 0.1067 mm.
    /// (c) Speed, acceleration and jerk of both axes stay within the limits, by differences as above; every
    /// planned pose is within the reported deviation of the given path, and reaches it on the arc; the path
    /// still ends on its target; and it takes clearly less time.
    #[test]
    fn transition_curves_let_the_path_enter_an_arc_at_speed() {
        let (axes, cycle, start) = (axes(), 1e-3, [0.1, 0.35]);
        let pieces = parse("line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.05").unwrap();
        let given = plan(&axes, &start, &pieces, cycle, 0.0).unwrap();
        assert_eq!(deviation(&axes, &start, &pieces, 0.0).unwrap(), 0.0);
        // How far a pose is from the given path: from the first line, the arc or the second line.
        let off = |p: &[f64]| if p[0] <= 0.5 { (p[1] - 0.35).abs() } else if p[1] <= 0.25 { (p[0] - 0.6).abs() } else { (distance(p, &[0.5, 0.25]) - 0.1).abs() };
        let curves = |tolerance: f64| -> Vec<f64> {
            let geo = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes)).unwrap().0;
            geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect()
        };
        for (tolerance, by_hand, length, speed) in [(1e-4, 1e-4, 0.01549, 0.3957), (1e-3, 0.1067e-3, 0.016, 0.4)] {
            let reported = deviation(&axes, &start, &pieces, tolerance).unwrap();
            assert!((reported / by_hand - 1.0).abs() < 2e-3 && reported <= tolerance, "{reported} m reported, {by_hand} m by hand");
            let lengths = curves(tolerance);
            assert!(lengths.len() == 2 && lengths.iter().all(|l| (l / length - 1.0).abs() < 5e-3), "{lengths:?} against {length}");

            let points = plan(&axes, &start, &pieces, cycle, tolerance).unwrap();
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "axis {i}: {peak:?}");
            }
            let furthest = points.iter().map(|p| off(&p.1)).fold(0.0, f64::max);
            assert!(furthest <= reported + 1e-9 && furthest > 0.999 * reported, "{furthest} m off the path, {reported} m reported");
            // The slowest the path gets between leaving the first line and being back on the second.
            let blended = |p: &[f64]| off(p) > 1e-9;
            let slowest = points.windows(2).filter(|w| blended(&w[0].1) && blended(&w[1].1)).map(|w| distance(&w[0].1, &w[1].1) / cycle).fold(f64::INFINITY, f64::min);
            assert!((slowest - speed).abs() < 2e-3, "{slowest} m/s through the blend, {speed} m/s by hand");
            let (last, before) = (points.len() - 1, given[given.len() - 1].0);
            assert!(points[0].1 == start && distance(&points[last].1, &[0.6, 0.05]) < 1e-9 && distance(&points[last].1, &points[last - 1].1) < 1e-6);
            assert!(points[last].0 < 0.9 * before, "{} s with transition curves, {before} s without", points[last].0);
            println!("tolerance {tolerance} m: {} s against {before} s, off by {furthest} m, curves {lengths:?} m, slowest {slowest} m/s, peak rates {peak:?}", points[last].0);
        }
    }

    /// Two arcs: an S of a 0.1 m and a 0.05 m arc turning opposite ways, and the same two turning the same way,
    /// each between two lines. Every junction gets its curve, and the blended path has no step in position,
    /// direction or curvature anywhere; it ends on its target, stays within the axis limits and is faster.
    /// (An arc at an end of the path: `an_arc_at_an_end_or_a_corner_gets_its_transition_curve`.)
    #[test]
    fn transition_curves_join_two_arcs() {
        let (axes, cycle, start, tolerance) = (axes(), 1e-3, [0.1, 0.5], 1e-4);
        let s_curve = "line,0.3,0.5
arc,0.4,0.4,0.3,0.4,cw
arc,0.45,0.35,0.45,0.4,ccw
line,0.7,0.35";
        let same_way = "line,0.3,0.5
arc,0.4,0.4,0.3,0.4,cw
arc,0.35,0.35,0.35,0.4,cw
line,0.1,0.35";
        for (text, end) in [(s_curve, [0.7, 0.35]), (same_way, [0.1, 0.35])] {
            let pieces = parse(text).unwrap();
            let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes)).unwrap();
            assert_eq!(geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).count(), 3, "{text}");
            // The tighter arc is the one that moves in by the whole tolerance.
            assert!((reported - tolerance).abs() < 1e-12, "{reported}");
            for pair in geo.windows(2) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps = [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-7, "{text}: steps {steps:?}");
            }
            let (points, given) = (plan(&axes, &start, &pieces, cycle, tolerance).unwrap(), plan(&axes, &start, &pieces, cycle, 0.0).unwrap());
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "{text}, axis {i}: {peak:?}");
            }
            let (last, before) = (points.len() - 1, given[given.len() - 1].0);
            assert!(distance(&points[last].1, &end) < 1e-9 && points[last].0 < 0.9 * before, "{text}: {} s against {before} s", points[last].0);
            println!("{} s against {before} s", points[last].0);
        }
    }

    /// A wide arc (0.25 m, 0.35 rad of it) that hands over to a tight one (1.72 mm, 1 rad) turning the same
    /// way, between two lines, with a tolerance of 1 mm. The curves on the tight arc turned by more than the
    /// whole circle between them; what they left of the arc was taken as the angle between
    /// them, which read as a short arc again, and the planned path went round once more than the given one
    /// (turning by 7.63 rad for 1.35 rad). Now what each curve takes is counted from the arc's own end, such
    /// curves are refused and shorter ones tried: the planned path turns as far as the given one.
    #[test]
    fn curves_too_long_for_a_tight_arc_do_not_add_a_turn() {
        let (axes, start, tolerance) = (axes(), [0.2, 0.02], 1e-3);
        let (wide, tight, a, b) = (0.25, 0.00172, 0.35, 1.0);
        // Clockwise from heading +z: after turning by t, the point is at the angle t above the centre's left.
        let on = |centre: [f64; 2], r: f64, t: f64| [centre[0] - r * t.cos(), centre[1] + r * t.sin()];
        let c1 = [0.2 + wide, 0.2];
        let p1 = on(c1, wide, a);
        let c2 = [p1[0] + (c1[0] - p1[0]) * tight / wide, p1[1] + (c1[1] - p1[1]) * tight / wide];
        let p2 = on(c2, tight, a + b);
        let end = [p2[0] + 0.02 * (a + b).sin(), p2[1] + 0.02 * (a + b).cos()];
        let arc = |to: [f64; 2], centre: [f64; 2]| Piece::Arc { to: to.to_vec(), centre: centre.to_vec(), clockwise: true };
        let pieces = [Piece::Line(vec![0.2, 0.2]), arc(p1, c1), arc(p2, c2), Piece::Line(end.to_vec())];
        let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes)).unwrap();
        let turned: f64 = geo
            .iter()
            .map(|g| match g {
                Geo::Arc { radius, turn, length, .. } => turn * length / radius,
                Geo::Clothoid { k0, k1, length, .. } => 0.5 * (k0 + k1) * length,
                Geo::Line { .. } | Geo::Spline { .. } => 0.0,
            })
            .sum();
        assert!((turned + a + b).abs() < 1e-9, "the planned path turns by {turned} rad, the given one by {}", -a - b);
        // It is blended all the same, with shorter curves, and the tight arc moves in by less than its radius.
        assert!(geo.iter().any(|g| matches!(g, Geo::Clothoid { .. })) && reported > 0.0 && reported < tight, "{reported} m reported");
        let points = plan(&axes, &start, &pieces, 1e-3, tolerance).unwrap();
        assert!(distance(&points[points.len() - 1].1, &end) < 1e-9);
        println!("turned {turned} rad, off by {reported} m, {} pieces", geo.len());
    }

    /// A line that ends on the end of a stroke, at z = 0: its last pose came out at z = -2.8e-17, outside the
    /// range, and the controller refused the whole path. Planned poses stay within the ranges of the axes.
    #[test]
    fn a_path_onto_the_end_of_a_stroke_stays_within_the_range() {
        let axes = axes();
        let points = plan(&axes, &[0.278, 0.231], &[Piece::Line(vec![0.525, 0.0])], 1e-3, 0.0).unwrap();
        assert!(points.iter().all(|p| p.1[1] >= 0.0), "ends at z = {:e}", points[points.len() - 1].1[1]);
        assert_eq!(points[points.len() - 1].1, [0.525, 0.0]);
        // A path that leaves the range for good is left as it is, for the controller to refuse.
        let beyond = plan(&axes, &[0.278, 0.231], &[Piece::Line(vec![0.525, -0.001])], 1e-3, 0.0).unwrap();
        assert!(beyond[beyond.len() - 1].1[1] < -0.000_999);
    }

    /// A corner is stopped at: the speed at the corner of two lines at right angles is zero, and each line
    /// is the S-curve of a single move, whose time the profile test calculates by hand.
    #[test]
    fn a_corner_is_stopped_at() {
        let (axes, cycle) = (axes(), 1e-3);
        let points = plan(&axes, &[0.1, 0.1], &[Piece::Line(vec![0.5, 0.1]), Piece::Line(vec![0.5, 0.3])], cycle, 0.0).unwrap();
        let moves = [crate::profile::Move::new(&[0.1, 0.1], &[0.5, 0.1], &axes), crate::profile::Move::new(&[0.5, 0.1], &[0.5, 0.3], &axes)];
        let duration = points[points.len() - 1].0;
        assert!((duration - moves[0].duration() - moves[1].duration()).abs() <= cycle, "{duration} s");
        let corner = (moves[0].duration() / cycle).round() as usize;
        assert!(distance(&points[corner].1, &[0.5, 0.1]) < 1e-6 && distance(&points[corner + 1].1, &points[corner].1) < 1e-6);
        let peak = peak_rates(&points, cycle);
        assert!(peak[1][0] <= 8.0 * 1.001 && peak[1][1] <= 6.0 * 1.001 && peak[2][0] <= 100.1 && peak[2][1] <= 100.1, "{peak:?}");
    }

    /// An arc that touches an end of the path or a corner: a line into a quarter arc of 0.1 m that ends the path,
    /// and the same with a corner after the arc (a line at right angles). The arc's end has to stay where it is
    /// given, so its circle shrinks by s and moves towards that end by s; after a quarter turn it is then apart
    /// from the line by s (1 - cos 90 degrees) = s, which is made e = L^2 / (24 r). With 0.1 mm these are the
    /// numbers of `transition_curves_let_the_path_enter_an_arc_at_speed`: L = sqrt(24 e r) = 15.49 mm, and the
    /// path enters the arc at (0.4 j r L)^(1/3) = 0.3957 m/s instead of 0.0707 m/s. The arc still ends exactly
    /// where it is given (the end of the path, or the corner, where the path stops), every planned pose is within
    /// the reported 0.1 mm of the given path, and the axis limits hold.
    #[test]
    fn an_arc_at_an_end_or_a_corner_gets_its_transition_curve() {
        let (axes, cycle, start, tolerance) = (axes(), 1e-3, [0.1, 0.35], 1e-4);
        // How far a pose is from the given path: the first line, the arc, or the line after the corner.
        let off = |p: &[f64]| if p[0] <= 0.5 { (p[1] - 0.35).abs() } else if p[0] <= 0.6 { (distance(p, &[0.5, 0.25]) - 0.1).abs() } else { (p[1] - 0.25).abs() };
        for (text, end) in [("line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw", [0.6, 0.25]), ("line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw\nline,0.7,0.25", [0.7, 0.25])] {
            let pieces = parse(text).unwrap();
            let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes)).unwrap();
            let curves: Vec<f64> = geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect();
            assert!(curves.len() == 1 && (curves[0] / 0.01549 - 1.0).abs() < 5e-3, "{text}: curves {curves:?}");
            assert!((reported / tolerance - 1.0).abs() < 2e-3 && reported <= tolerance, "{text}: {reported} m reported");
            let arc = geo.iter().find(|g| matches!(g, Geo::Arc { .. })).unwrap();
            assert!(distance(&arc.at(arc.length())[0], &[0.6, 0.25]) < 1e-12 && distance(&arc.at(arc.length())[1], &[0.0, -1.0]) < 1e-12, "{text}: the arc ends elsewhere");
            for pair in geo.windows(2).filter(|pair| !corner(&pair[0], &pair[1])) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps = [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-7, "{text}: steps {steps:?}");
            }
            for tolerance in [0.0, tolerance] {
                let points = plan(&axes, &start, &pieces, cycle, tolerance).unwrap();
                let peak = peak_rates(&points, cycle);
                for (i, axis) in axes.iter().enumerate() {
                    assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "{text}, axis {i}: {peak:?}");
                }
                let furthest = points.iter().map(|p| off(&p.1)).fold(0.0, f64::max);
                assert!(furthest <= reported + 1e-9 && (tolerance == 0.0 || furthest > 0.5 * reported), "{text}: {furthest} m off the path");
                // The arc's end is passed exactly (the path stops there), and the path ends on its target.
                assert!(points.iter().any(|p| distance(&p.1, &[0.6, 0.25]) < 1e-6) && distance(&points[points.len() - 1].1, &end) < 1e-9, "{text}");
                // The speed where the path leaves the first line.
                let leaves = points.iter().position(|p| off(&p.1) > 1e-9 || p.1[0] > 0.5).unwrap();
                let speed = distance(&points[leaves].1, &points[leaves - 1].1) / cycle;
                let by_hand = if tolerance > 0.0 { 0.3957 } else { 0.0707 };
                assert!((speed - by_hand).abs() < 2e-3, "{text}, tolerance {tolerance}: {speed} m/s into the arc, {by_hand} by hand");
                println!("{text:?}, tolerance {tolerance}: {} s, {speed} m/s into the arc, off by {furthest} m", points[points.len() - 1].0);
            }
        }
    }

    /// A line into a spline: along x to (0.3, 0.2), then one span to (0.39, 0.23), where the path ends. By hand
    /// (`spline`): the span leaves in the line's direction, d0 = (1, 0), and ends without curvature, so
    /// d1 = (3 chord - d0) / 2 with chord = (0.09, 0.03) / h, h = sqrt(0.009) = 94.87 mm, and its curvature at the
    /// junction is 3 dz / h^2 = 3 * 0.03 / 0.009 = 10 1/m.
    /// (a) Without a tolerance the curvature steps by 10 1/m there and the path passes at
    /// sqrt(0.5 j T / 10) = sqrt(0.5 * 100 * 0.001 / 10) = 0.0707 m/s.
    /// (b) With 0.1 mm the span is eased over l = sqrt(e / (0.01728 dk)) = sqrt(1e-4 / 0.1728) = 24.06 mm (half
    /// the span is 47.4 mm; the length needed at 0.4 m/s, 9 * 1.05 * 10 * 0.4^3 / (0.4 * 100) = 151 mm, is more),
    /// which moves it by 0.01728 * 10 * l^2 = 0.1 mm. At the junction its curvature is 0 and changes per distance
    /// by 9 * 10 / l = 3740.6 1/m^2 from the easing, plus the span's own change there, d0 x 6 c3 with
    /// c3 = (d0 + d1 - 2 chord) / h^2: (0.4743 - 0.6325) / 0.009 * 6 = -105.4 1/m^2, together 3635.2 1/m^2. With
    /// the margin of 1.05 the speed on the eased part is cbrt(0.4 j / (1.05 * 3635.2)) = 0.2189 m/s, three times
    /// the crawl. The planned poses stay within the reported 0.1 mm of the spline as given and within the axis
    /// limits, and the path still ends on the via point.
    #[test]
    fn a_spline_eased_into_a_line_is_entered_at_the_hand_calculated_speed() {
        let (axes, cycle, start) = (axes(), 1e-3, [0.1, 0.2]);
        let pieces = parse("line,0.3,0.2\nspline,0.39,0.23").unwrap();
        let geo = given(&start, &pieces).unwrap().0;
        let Geo::Spline { c, .. } = &geo[1] else { panic!("a spline span") };
        let [d, d2, _] = poly_rates(c, 0.0);
        assert!((d[0] - 1.0).abs() < 1e-12 && d[1].abs() < 1e-12 && (d2[1] - 10.0).abs() < 1e-9, "{d:?} {d2:?}");
        let (eased, reported) = geometry(&axes, &start, &pieces, 1e-4, &axis_speed(&axes)).unwrap();
        let Geo::Spline { h: l, .. } = &eased[1] else { panic!("the eased part") };
        assert!(eased.len() == 3 && (l - 0.024056).abs() < 1e-5 && (reported - 1e-4).abs() < 1e-12, "{l} m eased, {reported} m reported");
        for (tolerance, by_hand) in [(0.0, 0.0707), (1e-4, 0.2189)] {
            let points = plan(&axes, &start, &pieces, cycle, tolerance).unwrap();
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(peak[0][i] <= 1.001 * axis.max_vel && peak[1][i] <= 1.001 * axis.max_acc && peak[2][i] <= 1.001 * axis.max_jerk, "tolerance {tolerance}, axis {i}: {peak:?}");
            }
            let past = points.iter().position(|p| p.1[0] > 0.3).unwrap();
            let speed = distance(&points[past].1, &points[past - 1].1) / cycle;
            assert!((speed / by_hand - 1.0).abs() < 0.01, "tolerance {tolerance}: {speed} m/s at the junction, {by_hand} by hand");
            let line = polyline(&geo);
            let furthest = points.iter().map(|p| off_line(&line, &p.1)).fold(0.0, f64::max);
            assert!(furthest <= reported * (tolerance > 0.0) as u8 as f64 + 1e-7 && (tolerance == 0.0 || furthest > 0.9 * reported), "tolerance {tolerance}: {furthest} m off");
            assert!(distance(&points[points.len() - 1].1, &[0.39, 0.23]) < 1e-9);
            println!("tolerance {tolerance}: {} s, {speed} m/s at the junction, off by {furthest} m", points[points.len() - 1].0);
        }
    }

    /// Total length and total turning of a path.
    fn length_and_turning(geo: &[Geo]) -> (f64, f64) {
        let turned = |g: &Geo| match g {
            Geo::Arc { radius, turn, length, .. } => turn * length / radius,
            Geo::Clothoid { k0, k1, length, .. } => 0.5 * (k0 + k1) * length,
            Geo::Line { .. } | Geo::Spline { .. } => 0.0,
        };
        (geo.iter().map(Geo::length).sum(), geo.iter().map(turned).sum())
    }

    /// The blended path is as long as the given one, but for what the transitions change: each curve replaces about
    /// its own length of the pieces beside it, and each arc moved in by at most the reported deviation is shorter by
    /// at most that times the angle it turns. And it turns by as much (a lost or added turn shows here).
    fn assert_blend_keeps_the_path(start: &[f64], pieces: &[Piece], tolerance: f64, what: &str) {
        let axes = axes();
        let (given, _) = geometry(&axes, start, pieces, 0.0, &axis_speed(&axes)).unwrap();
        let (planned, reported) = geometry(&axes, start, pieces, tolerance, &axis_speed(&axes)).unwrap();
        let ((given_length, given_turn), (planned_length, planned_turn)) = (length_and_turning(&given), length_and_turning(&planned));
        let curves: f64 = planned.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).sum();
        let allowance = curves + reported * given_turn.abs().max(planned_turn.abs()) + 1e-12;
        assert!((planned_length - given_length).abs() <= allowance, "{what}: given {given_length} m, planned {planned_length} m, allowance {allowance} m");
        assert!((planned_turn - given_turn).abs() < 1e-6, "{what}: given turns {given_turn} rad, planned {planned_turn} rad");
    }

    /// Two arcs on the same circle that together turn more than a whole turn (4 rad, then pi + 0.05 rad, radius
    /// 0.05 m), between two lines: their radii differ by rounding, so a transition curve was built between them,
    /// and what it took of the arcs wrapped by a whole turn: the planned path was 0.1953 m long for 0.5096 m given.
    /// Equal curvatures get no curve, and the path keeps its length and its turning.
    #[test]
    fn two_arcs_on_one_circle_keep_their_whole_turn() {
        let (centre, r) = ([0.5, 0.5], 0.05);
        let at = |angle: f64| vec![centre[0] + r * angle.cos(), centre[1] + r * angle.sin()];
        let (a1, a2) = (std::f64::consts::FRAC_PI_2 - 4.0, std::f64::consts::FRAC_PI_2 - 4.0 - std::f64::consts::PI - 0.05);
        let end = at(a2);
        let arc = |to: Vec<f64>| Piece::Arc { to, centre: centre.to_vec(), clockwise: true };
        let pieces = [Piece::Line(vec![0.5, 0.55]), arc(at(a1)), arc(end.clone()), Piece::Line(vec![end[0] + 0.1 * a2.sin(), end[1] - 0.1 * a2.cos()])];
        let axes = axes();
        let given = length_and_turning(&geometry(&axes, &[0.45, 0.55], &pieces, 0.0, &axis_speed(&axes)).unwrap().0).0;
        assert!((given - (0.05 + 0.05 * (4.0 + std::f64::consts::PI + 0.05) + 0.1)).abs() < 1e-9, "{given} m");
        assert_blend_keeps_the_path(&[0.45, 0.55], &pieces, 1e-4, "two arcs on one circle");
    }

    /// Random paths of lines and arcs that join in the same direction (arcs of 2 mm to 0.2 m, turning 0.05 to 6 rad
    /// either way, some continuing on the circle before them), blended with tolerances of 0.01 to 1 mm: every one
    /// keeps its length within what its transitions may change, and its turning.
    #[test]
    fn blended_random_paths_keep_their_length_and_turning() {
        let mut seed = 0x2545_F491_4F6C_DD1Du64;
        let mut random = move || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for case in 0..400 {
            let (mut here, start_heading) = ([0.5, 0.5], random() * std::f64::consts::TAU);
            let mut heading = start_heading;
            let mut pieces = Vec::new();
            let mut last_arc: Option<([f64; 2], bool)> = None;
            for _ in 0..2 + (random() * 6.0) as usize {
                if random() < 0.35 {
                    let length = 0.001 + 0.1 * random();
                    here = [here[0] + length * heading.cos(), here[1] + length * heading.sin()];
                    pieces.push(Piece::Line(here.to_vec()));
                    last_arc = None;
                } else {
                    // On the circle before it half the time, else a new one turning either way.
                    let (centre, clockwise) = match last_arc {
                        Some(same) if random() < 0.5 => same,
                        _ => {
                            let (r, clockwise) = (0.002 + 0.2 * random(), random() < 0.5);
                            let side = if clockwise { -1.0 } else { 1.0 };
                            ([here[0] - side * r * heading.sin(), here[1] + side * r * heading.cos()], clockwise)
                        }
                    };
                    let (r, side) = (distance(&here, &centre), if clockwise { -1.0 } else { 1.0 });
                    let sweep = 0.05 + 5.95 * random();
                    let angle = (here[1] - centre[1]).atan2(here[0] - centre[0]) + side * sweep;
                    here = [centre[0] + r * angle.cos(), centre[1] + r * angle.sin()];
                    heading += side * sweep;
                    pieces.push(Piece::Arc { to: here.to_vec(), centre: centre.to_vec(), clockwise });
                    last_arc = Some((centre, clockwise));
                }
            }
            pieces.push(Piece::Line(vec![here[0] + 0.05 * heading.cos(), here[1] + 0.05 * heading.sin()]));
            // A lead-in line onto the first piece, in its direction.
            let lead = [0.5 - 0.01 * start_heading.cos(), 0.5 - 0.01 * start_heading.sin()];
            let pieces = [&[Piece::Line(vec![0.5, 0.5])][..], &pieces].concat();
            for tolerance in [1e-5, 1e-4, 1e-3] {
                assert_blend_keeps_the_path(&lead, &pieces, tolerance, &format!("case {case}, tolerance {tolerance}: {pieces:?}"));
            }
        }
    }
}
