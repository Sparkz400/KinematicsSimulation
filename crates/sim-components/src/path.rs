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
//! line or arc after it starts with. Lines work on any number of axes; arcs and splines on two and on three (see
//! below). Empty lines and lines starting with `#` are skipped.
//!
//! On a machine with three axes (x, y, z) a line has three positions (`line,0.1,0.0,-0.8`), and so does a spline
//! point. An arc gives its end and its centre with three positions each, `arc,x,y,z,cx,cy,cz,cw`: it lies in the
//! plane through where it starts, its end and its centre, and `cw`/`ccw` is seen from the side of that plane its
//! normal points to when the normal is taken upwards (+z); for a vertical plane, towards the viewer (-y), as the
//! scene looks at the machine; for the y-z plane, from +x. So an arc in the x-z plane is seen as on a two-axis
//! machine with x and z. What three axes do not have: a full turn or a half turn (start, end and centre on one line
//! leave the plane open, and such an arc is refused). Transition curves on three axes are laid in the plane of the arc
//! they meet (`blend3`); where two arcs in different planes meet, the path passes as given.
//!
//! Where the axes are x, y, z and an orientation as a rotation vector rx, ry, rz (an arm in tool space), a line moves
//! the place straight and turns the orientation by slerp (`Geo::Slerp`): about one fixed axis at a steady rate, not
//! along the straight line between the two rotation vectors, which is not the same turn where they are not parallel.

use crate::control::CommandError;
use crate::drive::DriveParams;
use sim_core::{AxisInfo, Machine};
use std::sync::Arc;

#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
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
        let numbers = |fields: &[&str]| {
            fields.iter().map(|v| v.parse().ok().filter(|n: &f64| n.is_finite())).collect::<Option<Vec<f64>>>()
        };
        let turn = fields[fields.len() - 1];
        let piece = match fields[0] {
            "line" if fields.len() > 1 => numbers(&fields[1..]).map(Piece::Line),
            "spline" if fields.len() > 1 => numbers(&fields[1..]).map(Piece::Spline),
            // The end and the centre: two positions each, three each, or on more axes the end's and three.
            "arc" if fields.len() >= 6 && fields.len() != 7 && (turn == "cw" || turn == "ccw") => {
                let count = fields.len() - 2;
                let split = match count {
                    4 => 2,
                    6 => 3,
                    _ => count - 3,
                };
                numbers(&fields[1..fields.len() - 1]).map(|n| Piece::Arc {
                    to: n[..split].to_vec(),
                    centre: n[split..].to_vec(),
                    clockwise: turn == "cw",
                })
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
    Line {
        from: Vec<f64>,
        dir: Vec<f64>,
        length: f64,
    },
    /// A line of a tool whose axes are x, y, z and a rotation vector rx, ry, rz: the place on the straight line from
    /// `from` to `to`, the orientation by slerp between theirs, both at the share s / `length` of the way; `length` the
    /// straight distance between the two poses, as a `Line` has it.
    Slerp {
        from: Vec<f64>,
        to: Vec<f64>,
        length: f64,
    },
    /// `turn` is 1 counter-clockwise and -1 clockwise.
    Arc {
        centre: [f64; 2],
        radius: f64,
        from_angle: f64,
        turn: f64,
        length: f64,
    },
    /// An arc in space: at the angle phi = s / radius from its start the point centre + radius (cos phi u + sin phi w),
    /// u from the centre to its start, w at right angles to it in its plane, in the direction it turns.
    Arc3 {
        centre: [f64; 3],
        u: [f64; 3],
        w: [f64; 3],
        radius: f64,
        length: f64,
    },
    /// A transition curve (clothoid): from the point `from` in the direction `heading`, the curvature goes
    /// evenly with the distance from `k0` to `k1`. Curvature is 1 / radius, positive counter-clockwise. On three axes
    /// `plane` is where the curve lies: a point (the origin of `from`) and the two directions of its first and second
    /// coordinate; `None` on two axes.
    Clothoid {
        from: [f64; 2],
        heading: f64,
        k0: f64,
        k1: f64,
        length: f64,
        plane: Option<[[f64; 3]; 3]>,
    },
    /// One span of a spline, from one via point to the next, or a part of one: the polynomial c0 + c1 u + ... +
    /// c5 u^5 for u from 0 to `h` (a cubic, c4 = c5 = 0, but where a spline is eased into a line or an arc,
    /// `ease`). `travelled` is the distance along it at `SPLINE_PARTS` + 1 evenly spaced u, the last being the
    /// length; `tightest` and `rate` are the most its curvature and the change of curvature per distance get.
    /// `dims` is the number of axes it runs in, two to `WIDEST` (the coefficients of the axes beyond are zero).
    Spline {
        c: Box<Poly>,
        h: f64,
        travelled: Vec<f64>,
        tightest: f64,
        rate: f64,
        dims: usize,
    },
    /// An arc on a machine with more than three axes (`arc_on`): `arc` (an `Arc3`) on the first three, and the others
    /// going from `from` by `by`, evenly with the distance along the arc. The distance along this piece counts all
    /// axes, as a line's does: the piece is `length` = sqrt(L^2 + |by|^2) long for the arc's L, and at the distance s it
    /// is at s L / length on the arc and at from + by s / length on the other axes.
    Carried {
        arc: Box<Geo>,
        from: Vec<f64>,
        by: Vec<f64>,
        length: f64,
    },
}

/// The most axes a spline (and a rounded corner, `round_corners`) runs in: the six of an arm in tool space.
const WIDEST: usize = 6;

/// A vector on up to `WIDEST` axes, zeros after the axes it has.
type Wide = [f64; WIDEST];

/// The coefficients of a spline span's polynomial in u, per power, for up to `WIDEST` axes.
type Poly = [Wide; 6];

fn norm3(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `dot3` and `norm3` on any number of axes (the sums of a three-axis vector with zeros after it are the same).
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a * b).sum()
}

fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

/// The first three of a spline's rates, for the cross products of a span on up to three axes.
fn first3(a: &Wide) -> [f64; 3] {
    [a[0], a[1], a[2]]
}

/// A vector as a `Wide` one: zeros after its axes. `None` with more axes than `WIDEST`.
fn wide(v: &[f64]) -> Option<Wide> {
    (v.len() <= WIDEST).then(|| std::array::from_fn(|i| v.get(i).copied().unwrap_or(0.0)))
}

/// A spline span's table of distance travelled has this many parts.
const SPLINE_PARTS: usize = 64;
/// The curvature of a spline span and its rate of change are looked at at 4 * `SPLINE_PARTS` + 1 places; what
/// they reach in between is covered by raising the largest found by this factor.
const SPLINE_MARGIN: f64 = 1.05;

/// Derivatives of a spline span's polynomial by u: first, second, third.
fn poly_rates(c: &Poly, u: f64) -> [Wide; 3] {
    // The k-th derivative: sum over m of m! / (m - k)! c_m u^(m - k).
    let rate = |k: usize, i: usize| {
        (k..6).rev().fold(0.0, |sum, m| sum * u + c[m][i] * ((m - k + 1)..=m).map(|f| f as f64).product::<f64>())
    };
    [1, 2, 3].map(|k| std::array::from_fn(|i| rate(k, i)))
}

/// The same polynomial with u counted from `from`: its coefficients around that point.
fn poly_shift(c: &Poly, from: f64) -> Poly {
    let mut out = [[0.0; WIDEST]; 6];
    for (m, cm) in c.iter().enumerate() {
        // (u + from)^m = sum over k of m! / (k! (m - k)!) u^k from^(m - k)
        let mut binomial = 1.0;
        for (k, o) in out.iter_mut().enumerate().take(m + 1) {
            for i in 0..WIDEST {
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
    const NODES: [(f64, f64); 5] = [
        (0.0, 0.568_888_888_888_888_9),
        (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (0.906_179_845_938_664, 0.236_926_885_056_189_1),
        (-0.906_179_845_938_664, 0.236_926_885_056_189_1),
    ];
    let (mid, half) = (0.5 * (from + to), 0.5 * (to - from));
    half * NODES
        .iter()
        .map(|(x, w)| {
            let [d, ..] = poly_rates(c, mid + half * x);
            w * norm(&d)
        })
        .sum::<f64>()
}

/// A spline span of the polynomial `c` for u from 0 to `h`, with its table of distance travelled and the most its
/// curvature and the change of curvature per distance get. `None` for a curve that nearly stops and turns on the
/// spot somewhere (its speed by u, 1 on average, falls below 0.05).
fn span(c: Poly, h: f64, dims: usize) -> Option<Geo> {
    let part = h / SPLINE_PARTS as f64;
    let mut travelled = vec![0.0];
    for k in 0..SPLINE_PARTS {
        travelled.push(travelled[k] + poly_distance(&c, k as f64 * part, (k + 1) as f64 * part));
    }
    let (mut tightest, mut rate) = (0f64, 0f64);
    for k in 0..=4 * SPLINE_PARTS {
        let [d, d2, d3] = poly_rates(&c, k as f64 * part / 4.0);
        let speed = norm(&d);
        if speed < 0.05 {
            return None;
        }
        // Curvature k = |d x d2| / speed^3, and its derivative by u over the speed: d|n|/du = n . (d x d3) / |n| with
        // n = d x d2 (on two axes n has a z part only, and this is the size of the signed formula). On more than three
        // axes by Lagrange's identity: |n|^2 = |d|^2 |d2|^2 - (d . d2)^2 and n . (d x d3) = |d|^2 (d2 . d3) - (d . d2)
        // (d . d3).
        let (size, along) = if dims <= 3 {
            let (n, turn) = (cross3(first3(&d), first3(&d2)), cross3(first3(&d), first3(&d3)));
            let size = norm3(n);
            (size, if size > 0.0 { dot3(n, turn) / size } else { norm3(turn) })
        } else {
            let (dd2, dd3) = (dot(&d, &d2), dot(&d, &d3));
            let size = (speed * speed * dot(&d2, &d2) - dd2 * dd2).max(0.0).sqrt();
            let turn = (speed * speed * dot(&d3, &d3) - dd3 * dd3).max(0.0).sqrt();
            (size, if size > 0.0 { (speed * speed * dot(&d2, &d3) - dd2 * dd3) / size } else { turn })
        };
        let curvature = size / speed.powi(3);
        let by_u = along / speed.powi(3) - 3.0 * curvature * dot(&d, &d2) / (speed * speed);
        (tightest, rate) = (tightest.max(curvature), rate.max((by_u / speed).abs()));
    }
    Some(Geo::Spline {
        c: Box::new(c),
        h,
        travelled,
        tightest: SPLINE_MARGIN * tightest,
        rate: SPLINE_MARGIN * rate,
        dims,
    })
}

impl Geo {
    fn length(&self) -> f64 {
        match self {
            Geo::Line { length, .. }
            | Geo::Slerp { length, .. }
            | Geo::Arc { length, .. }
            | Geo::Arc3 { length, .. }
            | Geo::Clothoid { length, .. }
            | Geo::Carried { length, .. } => *length,
            Geo::Spline { travelled, .. } => travelled[SPLINE_PARTS],
        }
    }

    /// Pose, direction and curvature at the distance s from the start of the piece: the pose and its first
    /// and second derivative by the distance travelled.
    fn at(&self, s: f64) -> [Vec<f64>; 3] {
        match self {
            Geo::Line { from, dir, .. } => {
                [from.iter().zip(dir).map(|(p, d)| p + d * s).collect(), dir.clone(), vec![0.0; dir.len()]]
            }
            Geo::Slerp { from, to, length } => {
                let pose = |s: f64| -> Vec<f64> {
                    let share = s / length;
                    let turned = crate::rigid::slerp([from[3], from[4], from[5]], [to[3], to[4], to[5]], share);
                    (0..6).map(|i| if i < 3 { from[i] + share * (to[i] - from[i]) } else { turned[i - 3] }).collect()
                };
                // The orientation's rates by central differences over 1e-4 of the piece (the slerp goes on smoothly past
                // its ends): truncation ~1e-8 of them, rounding ~1e-16 / 1e-8 of the rotation vector.
                let h = 1e-4 * length;
                let (ahead, here, behind) = (pose(s + h), pose(s), pose(s - h));
                let first = (0..6).map(|i| (ahead[i] - behind[i]) / (2.0 * h)).collect();
                let second = (0..6).map(|i| (ahead[i] - 2.0 * here[i] + behind[i]) / (h * h)).collect();
                [here, first, second]
            }
            Geo::Arc { centre, radius, from_angle, turn, .. } => {
                let (sin, cos) = (from_angle + turn * s / radius).sin_cos();
                [
                    vec![centre[0] + radius * cos, centre[1] + radius * sin],
                    vec![-turn * sin, turn * cos],
                    vec![-cos / radius, -sin / radius],
                ]
            }
            Geo::Arc3 { centre, u, w, radius, .. } => {
                let (sin, cos) = (s / radius).sin_cos();
                [
                    (0..3).map(|i| centre[i] + radius * (cos * u[i] + sin * w[i])).collect(),
                    (0..3).map(|i| -sin * u[i] + cos * w[i]).collect(),
                    (0..3).map(|i| -(cos * u[i] + sin * w[i]) / radius).collect(),
                ]
            }
            Geo::Clothoid { from, heading, k0, k1, length, plane } => {
                let angle = |u: f64| heading + k0 * u + 0.5 * (k1 - k0) * u * u / length;
                // The pose is the direction added up along the curve, which has no closed form: Simpson's
                // rule over 64 intervals, exact to far below a nanometre for the turns a blend makes.
                let (n, h) = (64, s / 64.0);
                let (mut x, mut y) = (0.0, 0.0);
                for i in 0..=n {
                    let weight = if i == 0 || i == n {
                        1.0
                    } else if i % 2 == 1 {
                        4.0
                    } else {
                        2.0
                    };
                    let (sin, cos) = angle(i as f64 * h).sin_cos();
                    (x, y) = (x + weight * cos, y + weight * sin);
                }
                let ((sin, cos), k) = (angle(s).sin_cos(), k0 + (k1 - k0) * s / length);
                let flat = [[from[0] + x * h / 3.0, from[1] + y * h / 3.0], [cos, sin], [-k * sin, k * cos]];
                match plane {
                    None => flat.map(|v| v.to_vec()),
                    // In space: the pose from the plane's point, direction and curvature along its two directions.
                    Some([o, e1, e2]) => {
                        let lift = |v: [f64; 2], point: f64| {
                            (0..3).map(|i| point * o[i] + v[0] * e1[i] + v[1] * e2[i]).collect()
                        };
                        [lift(flat[0], 1.0), lift(flat[1], 0.0), lift(flat[2], 0.0)]
                    }
                }
            }
            Geo::Spline { c, h, travelled, dims, .. } => {
                let u = spline_u(c, *h, travelled, s);
                let [d, d2, _] = poly_rates(c, u);
                let speed = norm(&d);
                let (dir, along) = (d.map(|x| x / speed), dot(&d, &d2) / speed);
                let pose = (0..*dims).map(|i| c.iter().rev().fold(0.0, |sum, cm| sum * u + cm[i])).collect();
                // The second derivative by distance: what of the second derivative by u is across the direction.
                [pose, dir[..*dims].to_vec(), (0..*dims).map(|i| (d2[i] - along * dir[i]) / (speed * speed)).collect()]
            }
            Geo::Carried { arc, from, by, length } => {
                let share = arc.length() / length;
                let [mut pose, mut dir, mut curve] = arc.at(s * share);
                dir.iter_mut().for_each(|d| *d *= share);
                curve.iter_mut().for_each(|c| *c *= share * share);
                pose.extend(from.iter().zip(by).map(|(f, b)| f + b * s / length));
                dir.extend(by.iter().map(|b| b / length));
                curve.extend(by.iter().map(|_| 0.0));
                [pose, dir, curve]
            }
        }
    }
}

/// The u at which a spline span has run the distance s: from its table of distance travelled, then Newton's method on
/// the distance itself, which converges to rounding in two or three steps.
fn spline_u(c: &Poly, h: f64, travelled: &[f64], s: f64) -> f64 {
    let part = travelled.partition_point(|t| *t <= s).clamp(1, SPLINE_PARTS) - 1;
    let (u0, du) = (part as f64 * h / SPLINE_PARTS as f64, h / SPLINE_PARTS as f64);
    let mut u = u0 + du * (s - travelled[part]) / (travelled[part + 1] - travelled[part]);
    for _ in 0..4 {
        let [d, ..] = poly_rates(c, u);
        u -= (travelled[part] + poly_distance(c, u0, u) - s) / norm(&d);
    }
    u
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
fn spline(points: &[Vec<f64>], before: Option<Wide>, after: Option<Wide>) -> Option<Vec<Geo>> {
    let (n, dims) = (points.len() - 1, points[0].len());
    let h: Vec<f64> = points.windows(2).map(|w| distance(&w[0], &w[1])).collect();
    // A two-axis point has no third position: its chord there is zero.
    let chord = |i: usize, axis: usize| if axis < dims { (points[i + 1][axis] - points[i][axis]) / h[i] } else { 0.0 };
    // Rows (below, diagonal, above, right side) of the system for the derivatives at the points.
    let mut rows: Vec<[f64; 3]> = Vec::new();
    let mut right: Vec<Wide> = Vec::new();
    for i in 0..=n {
        let (row, side) = match (i, before, after) {
            (0, Some(dir), _) => ([0.0, 1.0, 0.0], dir),
            (0, None, _) => ([0.0, 2.0, 1.0], std::array::from_fn(|axis| 3.0 * chord(0, axis))),
            (i, _, Some(dir)) if i == n => ([0.0, 1.0, 0.0], dir),
            (i, _, None) if i == n => ([1.0, 2.0, 0.0], std::array::from_fn(|axis| 3.0 * chord(n - 1, axis))),
            _ => (
                [h[i], 2.0 * (h[i - 1] + h[i]), h[i - 1]],
                std::array::from_fn(|axis| 3.0 * (h[i] * chord(i - 1, axis) + h[i - 1] * chord(i, axis))),
            ),
        };
        rows.push(row);
        right.push(side);
    }
    // Elimination downwards, then substitution upwards.
    for i in 1..=n {
        let factor = rows[i][0] / rows[i - 1][1];
        rows[i][1] -= factor * rows[i - 1][2];
        right[i] = std::array::from_fn(|axis| right[i][axis] - factor * right[i - 1][axis]);
    }
    let mut d = vec![[0.0; WIDEST]; n + 1];
    for i in (0..=n).rev() {
        let above = if i < n { d[i + 1] } else { [0.0; WIDEST] };
        d[i] = std::array::from_fn(|axis| (right[i][axis] - rows[i][2] * above[axis]) / rows[i][1]);
    }
    (0..n)
        .map(|i| {
            let c = [
                std::array::from_fn(|axis| if axis < dims { points[i][axis] } else { 0.0 }),
                d[i],
                std::array::from_fn(|axis| (3.0 * chord(i, axis) - 2.0 * d[i][axis] - d[i + 1][axis]) / h[i]),
                std::array::from_fn(|axis| (d[i][axis] + d[i + 1][axis] - 2.0 * chord(i, axis)) / (h[i] * h[i])),
                [0.0; WIDEST],
                [0.0; WIDEST],
            ];
            span(c, h[i], dims)
        })
        .collect()
}

fn distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(a, b)| (a - b) * (a - b)).sum::<f64>().sqrt()
}

/// The path the planners time: the pieces laid end to end from `start`, with transition curves where the
/// tolerance allows them (`blend` between lines and arcs, `ease` where a spline meets a line or an arc), and how
/// far that path is from the given one at most. `speed` is the highest speed along a piece that the planner will
/// allow, which sets how long a transition has to be. On three axes the curves between lines and arcs are laid in the
/// arcs' planes (`blend3`); paths on other numbers of axes have only lines and run as given. Corners are rounded first,
/// within the corner tolerance (`round_corners`).
fn geometry(
    axes: &[AxisInfo],
    start: &[f64],
    pieces: &[Piece],
    tolerance: impl Into<Tolerance>,
    speed: &dyn Fn(&Geo) -> f64,
    slack: f64,
) -> Result<(Vec<Geo>, f64), CommandError> {
    let Tolerance { path: tolerance, corner } = tolerance.into();
    let (geo, off) = given(start, pieces, slack)?;
    let (mut geo, rounded) = round_corners(geo, corner);
    // An orientation as a rotation vector turns by slerp along a line (what is left of it after its corners are
    // rounded: the rounding curves pass the rotation vector as any other axis, within the corner tolerance).
    if axes.len() == 6 && axes[3..].iter().map(|a| a.id).eq(["rx", "ry", "rz"]) {
        for g in &mut geo {
            if let Geo::Line { from, dir, length } = g {
                let to = from.iter().zip(dir.iter()).map(|(f, d)| f + *length * d).collect();
                *g = Geo::Slerp { from: from.clone(), to, length: *length };
            }
        }
    }
    if tolerance <= 0.0 {
        return Ok((geo, off + rounded));
    }
    // The speeds of the pieces as given, in space: `blend3` hands `blend` the pieces in a plane's coordinates.
    let speeds: Vec<f64> = geo.iter().map(speed).collect();
    let (geo, moved) = match start.len() {
        2 => blend(geo, &speeds, axes, tolerance),
        3 => blend3(geo, &speeds, axes, tolerance),
        _ => (geo, 0.0),
    };
    let (geo, eased) = ease(geo, axes, tolerance, speed);
    // The three change different places of the path, so the largest of them counts.
    Ok((geo, off + moved.max(eased).max(rounded)))
}

/// How far a planned path may leave the pieces as given: `path` for the transition curves where pieces meet in their
/// direction (`profile.path_tolerance_m`), `corner` for rounding the corners (`round_corners`,
/// `profile.corner_tolerance_m`). A tolerance alone is a path tolerance, with corners stopped at.
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Tolerance {
    pub path: f64,
    pub corner: f64,
}

impl From<f64> for Tolerance {
    fn from(path: f64) -> Self {
        Self { path, corner: 0.0 }
    }
}

/// The highest speed along a piece within the axis limits, as `plan` times it.
fn axis_speed(axes: &[AxisInfo]) -> impl Fn(&Geo) -> f64 {
    let bounds = axis_bounds(axes);
    move |geo| limits(geo, &bounds)[0]
}

/// How far the path that is planned for these pieces is from the pieces as given, at most: the deviation
/// the transition curves make, plus how far an arc ends off its stated end (`given`). Zero with a tolerance of
/// zero and arcs that end on their circles. For the plan within the axis limits.
pub fn deviation(
    axes: &[AxisInfo],
    start: &[f64],
    pieces: &[Piece],
    tolerance: impl Into<Tolerance>,
    slack: f64,
) -> Result<f64, CommandError> {
    Ok(geometry(axes, start, pieces, tolerance, &axis_speed(axes), slack)?.1)
}

/// For an arc that is the first piece: how far its end may be off the circle through its start (the planners'
/// `slack`). A path starts at the setpoint of the machine at rest, which is where the motors stand: under belt
/// stretch and gear twist that far from the pose the file was written for. The controller passes
/// `Machine::setpoint_slack` (12 and 46 um on a gantry holding its Z axis against gravity, within this default of
/// 0.1 mm; more on the delta, whose gearboxes sag under the platform); this is for a path from a stated point.
pub const ARC_SLACK: f64 = 1e-4;
/// A first arc has to end at least this far from where the machine stands, and at least ten times the slack.
/// Nearer, a full turn (whose end is up to the slack from there) cannot be told from a short arc (a 130 um arc ran
/// as a full turn), so neither is guessed: the path is refused, and the file states the start with a `line` to it
/// first, after which the arc is no longer the first piece. The margin of ten over the slack keeps a first arc that
/// is run at most a tenth longer or shorter than the one the file means.
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
fn given<'a>(start: &[f64], pieces: &'a [Piece], slack: f64) -> Result<(Vec<Geo>, f64), CommandError> {
    let bad = Err(CommandError { key: "error-path" });
    let (mut here, mut out): (Vec<f64>, Vec<Geo>) = (start.to_vec(), Vec::new());
    // The end the file stated last; none before the first piece, where the path stands on the machine's setpoint.
    let (mut stated, mut off): (Option<&'a Vec<f64>>, f64) = (None, 0.0);
    let mut rest = pieces;
    while let Some((piece, after)) = rest.split_first() {
        rest = after;
        let geo = match piece {
            Piece::Spline(_) if (2..=WIDEST).contains(&here.len()) => {
                // The whole run of spline pieces is one curve; a via point given twice counts once.
                let run = 1 + rest.iter().take_while(|p| matches!(p, Piece::Spline(_))).count();
                let mut points = vec![here.clone()];
                for to in std::iter::once(piece)
                    .chain(&rest[..run - 1])
                    .filter_map(|p| if let Piece::Spline(to) = p { Some(to) } else { None })
                {
                    if to.len() != here.len() {
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
                let before = out.last().and_then(|g| wide(&g.at(g.length())[1]));
                let after = match rest.first() {
                    Some(Piece::Line(to)) if to.len() == end.len() && distance(to, end) > 1e-9 => {
                        wide(&to.iter().zip(end).map(|(t, e)| (t - e) / distance(to, end)).collect::<Vec<_>>())
                    }
                    Some(Piece::Arc { centre, clockwise, .. })
                        if centre.len() == 2 && end.len() == 2 && distance(end, centre) > 1e-6 =>
                    {
                        let (turn, radius) = (if *clockwise { -1.0 } else { 1.0 }, distance(end, centre));
                        wide(&[-turn * (end[1] - centre[1]) / radius, turn * (end[0] - centre[0]) / radius])
                    }
                    Some(Piece::Arc { to, centre, clockwise })
                        if end.len() >= 3 && to.len() == end.len() && centre.len() == 3 =>
                    {
                        arc_on(end, to, centre, *clockwise).and_then(|g| wide(&g.at(0.0)[1]))
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
                Geo::Line {
                    dir: to.iter().zip(&here).map(|(t, h)| (t - h) / length).collect(),
                    from: here.clone(),
                    length,
                }
            }
            Piece::Arc { to, centre, clockwise } if here.len() == 2 && to.len() == 2 && centre.len() == 2 => {
                let radius = distance(&here, centre);
                // The arc starts where the path stands and takes its radius from there. What it is, a full turn
                // or by which angle it turns, is told from the points the file states: the end stated before
                // it and its own. Only a first piece has none before it and is told from where the machine
                // stands, with the slack that needs; too near to there it is not told at all (`FIRST_ARC`).
                let (from, slack) = match stated {
                    Some(p) => (&p[..], ARC_TOLERANCE),
                    None => (&here[..], slack),
                };
                if !(radius > 1e-6 && (distance(to, centre) - distance(from, centre)).abs() < slack) {
                    return bad;
                }
                if stated.is_none() && distance(to, from) < FIRST_ARC.max(10.0 * slack) {
                    return Err(CommandError { key: "error-path-first-arc" });
                }
                let angle = |p: &[f64]| (p[1] - centre[1]).atan2(p[0] - centre[0]);
                let turn = if *clockwise { -1.0 } else { 1.0 };
                let full = distance(to, from) < 1e-9;
                let sweep = if full {
                    std::f64::consts::TAU
                } else {
                    (turn * (angle(to) - angle(from))).rem_euclid(std::f64::consts::TAU)
                };
                Geo::Arc {
                    centre: [centre[0], centre[1]],
                    radius,
                    from_angle: angle(&here),
                    turn,
                    length: sweep * radius,
                }
            }
            Piece::Arc { to, centre, clockwise } if here.len() >= 3 && to.len() == here.len() && centre.len() == 3 => {
                // Its end is told from the end stated before it, as on two axes. An arc in space is never a full turn,
                // so a first arc needs no distance from where the machine stands. On more than three axes the arc is
                // on the first three, and the others go along with it (`arc_on`).
                let (from, slack) = match stated {
                    Some(p) => (&p[..], ARC_TOLERANCE),
                    None => (&here[..], slack),
                };
                if (distance(&to[..3], centre) - distance(&from[..3], centre)).abs() >= slack {
                    return bad;
                }
                let Some(geo) = arc_on(&here, to, centre, *clockwise) else { return bad };
                geo
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

/// An arc in space from `from` around `centre` towards `to`, in the plane of the three points, turning clockwise or
/// counter-clockwise as seen from the side its normal points to: the normal taken upwards (+z), for a vertical plane
/// towards the viewer (-y), for the y-z plane along +x (see the top of the file). `None` where the three points leave
/// the plane open (on one line: a half or a full turn) or the radius is nil.
fn arc3(from: &[f64], to: &[f64], centre: &[f64], clockwise: bool) -> Option<Geo> {
    let at = |p: &[f64]| [p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]];
    let (a, b) = (at(from), at(to));
    let radius = norm3(a);
    let mut normal = cross3(a, b);
    let size = norm3(normal);
    if radius < 1e-6 || size < 1e-9 * radius * norm3(b) {
        return None;
    }
    normal = normal.map(|x| x / size);
    let view = if normal[2].abs() > 1e-9 {
        [0.0, 0.0, 1.0]
    } else if normal[1].abs() > 1e-9 {
        [0.0, -1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    if dot3(normal, view) < 0.0 {
        normal = normal.map(|x| -x);
    }
    // Counter-clockwise seen from the normal's side turns from u towards normal x u.
    let u = a.map(|x| x / radius);
    let w = cross3(normal, u).map(|x| if clockwise { -x } else { x });
    let angle = dot3(b, w).atan2(dot3(b, u)).rem_euclid(std::f64::consts::TAU);
    Some(Geo::Arc3 { centre: [centre[0], centre[1], centre[2]], u, w, radius, length: angle * radius })
}

/// `arc3` on three axes or more: the arc on the first three (the centre has three positions), and on a machine with more
/// axes the others carried along from where the arc starts to its end, each evenly with the distance (`Geo::Carried`):
/// a SCARA's or a delta's turn, an arm's orientation in tool space.
///
/// shortcut: an arm's orientation (rx, ry, rz) goes along as its rotation vector, evenly per component, which is a
/// rotation at an even rate only about a fixed axis; take the arm's quaternion slerp (the arms' tool space) here
/// if tool-space arcs that turn the tool about two axes at once have to keep its rate of turning even.
fn arc_on(from: &[f64], to: &[f64], centre: &[f64], clockwise: bool) -> Option<Geo> {
    let arc = arc3(&from[..3], &to[..3], centre, clockwise)?;
    if from.len() == 3 {
        return Some(arc);
    }
    let by: Vec<f64> = to[3..].iter().zip(&from[3..]).map(|(t, f)| t - f).collect();
    let length = (arc.length().powi(2) + by.iter().map(|b| b * b).sum::<f64>()).sqrt();
    Some(Geo::Carried { arc: Box::new(arc), from: from[3..].to_vec(), by, length })
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

/// A corner: the direction turns by more than 1 mrad from one piece to the next. The path stops there, unless
/// `round_corners` has rounded it.
fn corner(a: &Geo, b: &Geo) -> bool {
    distance(&a.at(a.length())[1], &b.at(0.0)[1]) > 1e-3
}

/// The distance from the point `p` to the straight piece from `a` to `b`.
fn to_segment(p: &[f64], a: &[f64], b: &[f64]) -> f64 {
    let ab: Vec<f64> = b.iter().zip(a).map(|(b, a)| b - a).collect();
    let ap: Vec<f64> = p.iter().zip(a).map(|(p, a)| p - a).collect();
    let along = (dot(&ap, &ab) / dot(&ab, &ab)).clamp(0.0, 1.0);
    ap.iter().zip(&ab).map(|(p, d)| (p - along * d).powi(2)).sum::<f64>().sqrt()
}

/// The curve that rounds a corner between two lines (`round_corners`): from the point `d` before the corner `at` on the
/// line coming in (direction `ta`) to the point `d` after it on the line going out (`tb`), a polynomial in u from 0 to
/// 2 d that leaves and arrives in the lines' directions at a speed by u of 1, with no curvature at either end:
///
/// ```text
/// p(u) = at - d ta + ta u + (tb - ta) (u^3 / (4 d^2) - u^4 / (16 d^3))
/// ```
///
/// (the quintic through both ends with those first and zero second derivatives, whose fifth power drops out). So the
/// path keeps its direction and its curvature (zero) at both ends: no step in the axes' accelerations. Its middle, u = d,
/// is 3 d / 16 (tb - ta) from the corner, which is 3 d sin(theta) / 16 off each line for the turn theta, and there it
/// moves at (ta + tb) / 2 by u and turns at 3 (tb - ta) / (4 d), at right angles: a curvature of
/// 3 |tb - ta| / (d |ta + tb|^2), for a right angle 3 sqrt(2) / (2 d), a radius of sqrt(2) d / 3.
/// `None` where it nearly stops (`span`): a turn of nearly half a circle.
fn corner_curve(at: &[f64], ta: &[f64], tb: &[f64], d: f64) -> Option<Geo> {
    let (dims, change) = (at.len(), wide(&tb.iter().zip(ta).map(|(b, a)| b - a).collect::<Vec<_>>())?);
    let (ta, start) = (wide(ta)?, wide(&at.iter().zip(ta).map(|(p, t)| p - d * t).collect::<Vec<_>>())?);
    let zero = [0.0; WIDEST];
    let c = [start, ta, zero, change.map(|x| x / (4.0 * d * d)), change.map(|x| -x / (16.0 * d * d * d)), zero];
    span(c, 2.0 * d, dims)
}

/// The curve that rounds a corner next to an arc or a spline (`round_corners`): the quintic in u from 0 to `h` that leaves
/// `p0` in the unit direction `t0` with the curvature vector `k0` and arrives at `p1` in `t1` with `k1`, at a speed by u of
/// 1 at both ends (quintic Hermite interpolation, in tau = u / h with the derivatives h t and h^2 k):
///
/// ```text
/// p = p0 + h t0 tau + h^2 k0 / 2 tau^2
///   + (10 (p1 - p0) - h (6 t0 + 4 t1) - h^2 (3 k0 - k1) / 2) tau^3
///   + (15 (p0 - p1) + h (8 t0 + 7 t1) + h^2 (3 k0 / 2 - k1)) tau^4
///   + (6 (p1 - p0) - 3 h (t0 + t1) - h^2 (k0 - k1) / 2) tau^5
/// ```
///
/// So the path keeps its place, direction and curvature at both ends (G2). Between two lines (k = 0, h = 2 d, p0 and p1
/// `d` either side of the corner) it is `corner_curve`'s curve. `None` where it nearly stops (`span`).
fn hermite_curve(start: [&[f64]; 3], end: [&[f64]; 3], h: f64) -> Option<Geo> {
    let dims = start[0].len();
    let ([p0, t0, k0], [p1, t1, k1]) = (start.map(wide), end.map(wide));
    let (p0, t0, k0, p1, t1, k1) = (p0?, t0?, k0?, p1?, t1?, k1?);
    let tau: [Wide; 6] = [
        p0,
        std::array::from_fn(|i| h * t0[i]),
        std::array::from_fn(|i| h * h * k0[i] / 2.0),
        std::array::from_fn(|i| {
            10.0 * (p1[i] - p0[i]) - h * (6.0 * t0[i] + 4.0 * t1[i]) - h * h * (3.0 * k0[i] - k1[i]) / 2.0
        }),
        std::array::from_fn(|i| {
            15.0 * (p0[i] - p1[i]) + h * (8.0 * t0[i] + 7.0 * t1[i]) + h * h * (1.5 * k0[i] - k1[i])
        }),
        std::array::from_fn(|i| 6.0 * (p1[i] - p0[i]) - 3.0 * h * (t0[i] + t1[i]) - h * h * (k0[i] - k1[i]) / 2.0),
    ];
    let c: Poly = std::array::from_fn(|m| tau[m].map(|x| x / h.powi(m as i32)));
    span(c, h, dims)
}

/// The piece with `from` cut off its start and `to` off its end (both along it), for a corner rounded beside it. Lines,
/// arcs and spline spans; `None` for the others, and where nothing is left.
fn trimmed(geo: &Geo, from: f64, to: f64) -> Option<Geo> {
    let length = geo.length() - from - to;
    if length <= 1e-12 {
        return None;
    }
    Some(match geo {
        Geo::Line { from: start, dir, .. } => {
            Geo::Line { from: start.iter().zip(dir).map(|(p, d)| p + from * d).collect(), dir: dir.clone(), length }
        }
        Geo::Arc { centre, radius, from_angle, turn, .. } => Geo::Arc {
            centre: *centre,
            radius: *radius,
            from_angle: from_angle + turn * from / radius,
            turn: *turn,
            length,
        },
        Geo::Arc3 { centre, u, w, radius, .. } => {
            let (sin, cos) = (from / radius).sin_cos();
            Geo::Arc3 {
                centre: *centre,
                u: std::array::from_fn(|i| cos * u[i] + sin * w[i]),
                w: std::array::from_fn(|i| -sin * u[i] + cos * w[i]),
                radius: *radius,
                length,
            }
        }
        Geo::Carried { arc, from: start, by, length: whole } => {
            let share = arc.length() / whole;
            let arc = trimmed(arc, from * share, to * share)?;
            let start = start.iter().zip(by).map(|(f, b)| f + b * from / whole).collect();
            Geo::Carried {
                arc: Box::new(arc),
                from: start,
                by: by.iter().map(|b| b * length / whole).collect(),
                length,
            }
        }
        Geo::Spline { c, h, travelled, dims, .. } => {
            let (u0, u1) = (spline_u(c, *h, travelled, from), spline_u(c, *h, travelled, geo.length() - to));
            span(poly_shift(c, u0), u1 - u0, *dims)?
        }
        Geo::Slerp { .. } | Geo::Clothoid { .. } => return None,
    })
}

/// How far the points of `curve` are from the path it replaces, at most: the last `back` of `a` and the first `ahead` of
/// `b`, each looked at at 65 places (as two broken lines, which lie within (d / 64)^2 / (8 r) of an arc of radius r).
fn off_corner(curve: &Geo, a: &Geo, back: f64, b: &Geo, ahead: f64) -> f64 {
    let la = a.length();
    let line: Vec<Vec<f64>> = (0..=64)
        .map(|k| a.at(la - back + back * k as f64 / 64.0)[0].clone())
        .chain((1..=64).map(|k| b.at(ahead * k as f64 / 64.0)[0].clone()))
        .collect();
    (0..=64)
        .map(|k| {
            let p = &curve.at(curve.length() * k as f64 / 64.0)[0];
            line.windows(2).map(|w| to_segment(p, &w[0], &w[1])).fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max)
}

/// The curve that rounds the corner between `a` and `b`, either of them an arc or a spline (`hermite_curve`), from `d`
/// before the corner on `a` to `d` after it on `b`, with `d` as large as keeps it within `tolerance` of them, at most
/// `most`; with d and its deviation. The deviation grows about as d (3 d sin(theta) / 16 for the turn theta where the
/// curvature is small against 1 / d, as between lines), so d is scaled by the tolerance over the deviation until the
/// deviation is within 2 % below the tolerance, at most 30 times.
fn round_corner(a: &Geo, b: &Geo, tolerance: f64, most: f64) -> Option<(f64, Geo, f64)> {
    let curve_at = |d: f64| {
        let ([p0, t0, k0], [p1, t1, k1]) = (a.at(a.length() - d), b.at(d));
        let curve = hermite_curve([&p0, &t0, &k0], [&p1, &t1, &k1], 2.0 * d)?;
        let off = off_corner(&curve, a, d, b, d);
        Some((d, curve, off))
    };
    let mut best: Option<(f64, Geo, f64)> = None;
    let mut d = most;
    for _ in 0..30 {
        let Some(found) = curve_at(d) else {
            d *= 0.5;
            continue;
        };
        let off = found.2;
        if off <= tolerance {
            let done = off >= 0.98 * tolerance || d >= most;
            if best.as_ref().is_none_or(|b| found.0 > b.0) {
                best = Some(found);
            }
            if done {
                break;
            }
        }
        d = (d * 0.99 * tolerance / off.max(1e-300)).min(most);
    }
    best
}

/// Rounds each corner (`corner`) by a curve as far from the corner as `tolerance` allows, as a CNC rounds a corner within
/// a tolerance (Siemens' G642) or a robot passes a point in a zone: the path no longer stops there. Between two lines the
/// curve is `corner_curve`'s: it starts and ends `d` from the corner on the lines, d such that the curve is nowhere
/// further from the two lines than the tolerance (for a right angle d = 16 tolerance / 3); the deviation is measured on
/// the curve at 65 places against the two pieces of line it replaces, and it scales with d. Where an arc or a spline
/// meets the corner the curve is `hermite_curve`'s, from `d` back along the piece before to `d` on along the one after,
/// with their directions and curvatures there (G2), d found by `round_corner`. Either way d is no more than half of either
/// piece, so that two rounded corners never overlap. A turn of nearly half a circle (where the curve would nearly stop,
/// `span`) and a corner on more than `WIDEST` axes stay corners.
fn round_corners(geo: Vec<Geo>, tolerance: f64) -> (Vec<Geo>, f64) {
    let n = geo.len();
    if tolerance <= 0.0 || n < 2 {
        return (geo, 0.0);
    }
    // Per piece how much of it its corners take at its start and its end, and the curve at each junction.
    let (mut cut, mut curves, mut off) = (vec![[0.0f64; 2]; n], vec![None; n + 1], 0f64);
    let roundable = |g: &Geo| !matches!(g, Geo::Slerp { .. } | Geo::Clothoid { .. });
    for j in 1..n {
        if !corner(&geo[j - 1], &geo[j]) || !roundable(&geo[j - 1]) || !roundable(&geo[j]) {
            continue;
        }
        let most = 0.5 * geo[j - 1].length().min(geo[j].length());
        let (d, curve, by) = match (&geo[j - 1], &geo[j]) {
            (Geo::Line { dir: ta, .. }, Geo::Line { from: at, dir: tb, .. }) => {
                // The deviation for d = 1, against the corner's two lines as far as the curve reaches.
                let Some(unit) = corner_curve(at, ta, tb, 1.0) else { continue };
                let (before, after): (Vec<f64>, Vec<f64>) =
                    (at.iter().zip(ta).map(|(p, t)| p - t).collect(), at.iter().zip(tb).map(|(p, t)| p + t).collect());
                let per_d = (0..=64)
                    .map(|k| {
                        let p = &unit.at(unit.length() * k as f64 / 64.0)[0];
                        to_segment(p, &before, at).min(to_segment(p, at, &after))
                    })
                    .fold(0.0, f64::max);
                let d = (tolerance / per_d).min(most);
                let Some(curve) = corner_curve(at, ta, tb, d) else { continue };
                (d, curve, per_d * d)
            }
            (a, b) => {
                let Some(found) = round_corner(a, b, tolerance, most) else { continue };
                found
            }
        };
        (cut[j - 1][1], cut[j][0], curves[j]) = (d, d, Some(curve));
        off = off.max(by);
    }
    let mut out = Vec::new();
    for (i, piece) in geo.into_iter().enumerate() {
        out.extend(curves[i].take());
        if cut[i] == [0.0; 2] {
            out.push(piece);
        } else {
            // Two corners that each take half of a piece leave nothing of it.
            out.extend(trimmed(&piece, cut[i][0], cut[i][1]));
        }
    }
    (out, off)
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
        let [end, dir, _] = Geo::Clothoid { from: [0.0, 0.0], heading: 0.0, k0, k1, length, plane: None }.at(length);
        [end[0] - dir[1] / k1, end[1] + dir[0] / k1]
    };
    // To first order the gap is the change of curvature times length squared / 24.
    let longest = |change: f64| 2.0 * (24.0 * gap / change.abs()).sqrt();
    match (a, b) {
        (Side::Line { point, dir }, Side::Circle { centre, curvature: k }) => {
            let to_centre = [centre[0] - point[0], centre[1] - point[1]];
            // The centre is this far beside the line; the curve's own circle has to end up as far beside +x.
            let beside = (dir[0] * to_centre[1] - dir[1] * to_centre[0]).abs();
            let length =
                largest(0.0, longest(k), |length| length == 0.0 || centre_at_end(0.0, k, length)[1].abs() <= beside);
            let along = dir[0] * to_centre[0] + dir[1] * to_centre[1] - centre_at_end(0.0, k, length)[0];
            let from = [point[0] + along * dir[0], point[1] + along * dir[1]];
            (length > 0.0 && length < longest(k)).then(|| Geo::Clothoid {
                from,
                heading: dir[1].atan2(dir[0]),
                k0: 0.0,
                k1: k,
                length,
                plane: None,
            })
        }
        // The same curve run backwards: every direction and every turn is the other way round.
        (Side::Circle { centre, curvature }, Side::Line { point, dir }) => {
            let back = transition(
                Side::Line { point, dir: [-dir[0], -dir[1]] },
                Side::Circle { centre, curvature: -curvature },
                gap,
            )?;
            let [end, dir, _] = back.at(back.length());
            Some(Geo::Clothoid {
                from: [end[0], end[1]],
                heading: (-dir[1]).atan2(-dir[0]),
                k0: curvature,
                k1: 0.0,
                length: back.length(),
                plane: None,
            })
        }
        (Side::Circle { centre: ca, curvature: k0 }, Side::Circle { centre: cb, curvature: k1 }) => {
            // The two circles of the curve start out touching and move apart (or into each other) as it
            // gets longer; it fits when their centres are as far apart as the real ones.
            let apart = |length: f64| {
                if length == 0.0 {
                    (1.0 / k1 - 1.0 / k0).abs()
                } else {
                    distance(&centre_at_end(k0, k1, length), &[0.0, 1.0 / k0])
                }
            };
            let moved = (distance(&ca, &cb) - apart(0.0)).abs();
            let length = largest(0.0, longest(k1 - k0), |length| (apart(length) - apart(0.0)).abs() <= moved);
            // Turned so that the line between the curve's centres lies on the line between the real ones.
            let end = centre_at_end(k0, k1, length);
            let heading = (cb[1] - ca[1]).atan2(cb[0] - ca[0]) - (end[1] - 1.0 / k0).atan2(end[0]);
            let from = [ca[0] + heading.sin() / k0, ca[1] - heading.cos() / k0];
            (length > 0.0 && length < longest(k1 - k0)).then_some(Geo::Clothoid {
                from,
                heading,
                k0,
                k1,
                length,
                plane: None,
            })
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
/// So every blended arc keeps its centre and gets a radius smaller by its e, and the transition curves
/// replace about L / 2 of the pieces on either side of each junction. The planned path is then nowhere
/// further from the given one than the largest e, which is what is reported. Each arc gets its own L: the
/// length the faster of its junctions needs to be passed at the speed of its slower piece with 0.4 of the jerk
/// limit, L = v^3 dk / (0.4 j), but no longer than the tolerance allows on it, L = sqrt(24 e_max r). The speed v
/// is that of the plan: the axis limits for `plan`, the motors for `plan_time_optimal`. Where L is shorter than
/// needed, `limits` lowers the speed on the curve. A junction of a line and an arc so gets the curve its own speed
/// asks for, not the one the fastest junction of the path asks for.
///
/// With an arc moved in by e = L^2 / (24 r), a line beside it is apart from it by e and the curve there comes out
/// L long. Two arcs turning opposite ways are apart by the sum of their e, which fits whatever their L. Two arcs
/// turning the same way are apart by the difference of their e, which is positive for one L only: arcs that hand
/// over to one another turning the same way share one L (the longest any of them needs, the shortest any allows).
/// An arc whose one end has to stay where it is given (at a corner, at an end of the path, or where a
/// spline joins it, see `ease`) and whose other end meets a line or an arc in its direction is moved another way:
/// its circle shrinks by s and moves towards the fixed end by s, so that it still passes through that end in the
/// same direction, and is then inside its old circle by s (1 - cos phi) at its other end, phi being the angle the arc
/// turns through. That gap is made e, so s = e / (1 - cos phi), for arcs whose radius stays above half of what it
/// was; next to an arc the gap is taken as e as well, and `transition` fits the curve to where the circles really
/// are. Halfway round, phi = pi, the new circle is 2 s inside the old one; so an arc that turns more than half a
/// circle gets s no larger than half the tolerance, e = (tolerance / 2) (1 - cos phi) at its free end and a shorter
/// L. The exact distance of the two circles on the arc is reported. A whole turn has no free end to move and stays
/// as it is. An arc with both ends fixed meets nothing in its direction and needs no curve. Where a curve would not
/// fit (a wider arc moved in beside a tighter one that was not) both arcs stay as given. Two arcs of the same
/// curvature (to 1e-9 relative: one circle) get no curve between them. Pieces too short for their curves halve
/// every L of the path, up to eight times; then the path stays as given. So does a blend in which a curve would
/// take more of an arc than it can reach, or whose length differs from the given one's by more than the curves'
/// lengths and the moved arcs' shrink times their angles (a whole turn gained or lost).
fn blend(geo: Vec<Geo>, speed: &[f64], axes: &[AxisInfo], tolerance: f64) -> (Vec<Geo>, f64) {
    let n = geo.len();
    let k: Vec<f64> =
        geo.iter().map(|g| if let Geo::Arc { radius, turn, .. } = g { turn / radius } else { 0.0 }).collect();
    // Junction j is where piece j - 1 hands over to piece j; it gets a curve where both are lines or arcs that
    // meet in the same direction.
    let spline = |i: usize| matches!(geo[i], Geo::Spline { .. });
    let tangent: Vec<bool> =
        (0..=n).map(|j| j > 0 && j < n && !corner(&geo[j - 1], &geo[j]) && !spline(j - 1) && !spline(j)).collect();
    let free: Vec<bool> = (0..n).map(|i| k[i] != 0.0 && tangent[i] && tangent[i + 1]).collect();
    // The angle an arc turns through.
    let sweep = |i: usize| geo[i].length() * k[i].abs();
    // An arc fixed at one end: Some(true) at its start, Some(false) at its end. Not a whole turn, whose far end is
    // its fixed one (1 - cos phi = 0).
    let fixed: Vec<Option<bool>> = (0..n)
        .map(|i| match (k[i] != 0.0 && 1.0 - sweep(i).cos() > 1e-9, tangent[i], tangent[i + 1]) {
            (true, true, false) => Some(false),
            (true, false, true) => Some(true),
            _ => None,
        })
        .collect();
    let blended = |i: usize| free[i] || fixed[i].is_some();
    if !(0..n).any(blended) {
        return (geo, 0.0);
    }
    let jerk = axes.iter().map(|a| a.max_jerk).fold(f64::INFINITY, f64::min);
    let needed = |j: usize| speed[j - 1].min(speed[j]).powi(3) * (k[j] - k[j - 1]).abs() / (0.4 * jerk);
    // Per arc: the longest L its junctions need, and the longest the tolerance allows on it (for an arc fixed at one
    // end that turns more than half a circle, s = e / (1 - cos phi) up to half the tolerance).
    let mut want: Vec<[f64; 2]> = (0..n)
        .map(|i| {
            if !blended(i) {
                return [0.0; 2];
            }
            let room = if fixed[i].is_some() && sweep(i) > std::f64::consts::PI {
                0.5 * tolerance * (1.0 - sweep(i).cos())
            } else {
                tolerance
            };
            [
                [i, i + 1].into_iter().filter(|&j| tangent[j]).map(needed).fold(0.0, f64::max),
                (24.0 * room / k[i].abs()).sqrt(),
            ]
        })
        .collect();
    // Arcs that hand over to one another turning the same way share one L, along the whole chain of them.
    for _ in 0..n {
        for j in (1..n).filter(|&j| tangent[j] && k[j] * k[j - 1] > 0.0 && blended(j) && blended(j - 1)) {
            let shared = [want[j][0].max(want[j - 1][0]), want[j][1].min(want[j - 1][1])];
            (want[j - 1], want[j]) = (shared, shared);
        }
    }
    let mut length: Vec<f64> = want.iter().map(|[need, most]| need.min(*most)).collect();
    for _ in 0..8 {
        if let Some(blended) = blend_with(&geo, &k, &free, &fixed, &tangent, &length) {
            return blended;
        }
        length.iter_mut().for_each(|l| *l /= 2.0);
    }
    (geo, 0.0)
}

/// `blend` for one length of the transition curves per arc; `None` if a piece is too short for its curves.
fn blend_with(
    geo: &[Geo],
    k: &[f64],
    free: &[bool],
    fixed: &[Option<bool>],
    tangent: &[bool],
    length: &[f64],
) -> Option<(Vec<Geo>, f64)> {
    let n = geo.len();
    // The gap each arc leaves to a line beside it: moved in by it, or (fixed at one end) shrunk and moved.
    let mut inset: Vec<f64> = (0..n)
        .map(|i| if free[i] || fixed[i].is_some() { k[i].abs() * length[i] * length[i] / 24.0 } else { 0.0 })
        .collect();
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
        if let (Some(_), Some((_, moved)), Geo::Arc { radius, .. }) = (fixed[i], circle(&inset, i), &geo[i])
            && moved < 0.5 * radius
        {
            inset[i] = 0.0;
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
        Geo::Arc { turn, .. } => {
            circle(&inset, i).map(|(centre, radius)| Side::Circle { centre, curvature: turn / radius })
        }
        Geo::Line { from, dir, .. } => Some(Side::Line { point: [from[0], from[1]], dir: [dir[0], dir[1]] }),
        Geo::Clothoid { .. } | Geo::Spline { .. } | Geo::Arc3 { .. } | Geo::Slerp { .. } | Geo::Carried { .. } => None,
    };
    let mut curves: Vec<Option<Geo>> = vec![None];
    for j in 1..n {
        let gap = gap(&inset, j);
        curves.push(if tangent[j] && gap > 0.0 { Some(transition(side(j - 1)?, side(j)?, gap)?) } else { None });
        // A curve that turns half a circle is no blend, and what it takes of an arc could no longer be told
        // from a whole turn more.
        if matches!(curves[j], Some(Geo::Clothoid { k0, k1, length, .. }) if 0.5 * (k0.abs() + k1.abs()) * length >= std::f64::consts::PI)
        {
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
            // On two axes only: a slerp is a line of six.
            Geo::Slerp { .. } => piece.clone(),
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
                let taken = |by: &Option<Vec<f64>>, arc_end: f64, sign: f64| {
                    by.as_ref().map_or(0.0, |p| (sign * turn * (angle(p) - arc_end)).rem_euclid(std::f64::consts::TAU))
                };
                let (before, after) = (taken(&start, *from_angle, 1.0), taken(&end, from_angle + turn * sweep, -1.0));
                // More than a curve can take is a wrapped angle, a whole turn off: refused.
                if before > reaches.0 || after > reaches.1 {
                    return None;
                }
                let left = sweep - before - after;
                // How far the new circle is from the old one on the arc: the gap where it meets the line, and for a
                // fixed arc r - |s u + (r - s) v|, with u and v at the angle theta apart, which grows with theta up to
                // half a turn (2 s there): at its free end, or halfway round for an arc that turns further.
                let shrink = radius - moved;
                // On its smaller circle the arc is shorter by up to the shrink times its angle.
                cut += shrink.abs() * sweep;
                let swing = if fixed[i].is_some() {
                    radius
                        - (radius * radius - 2.0 * shrink * moved * (1.0 - sweep.min(std::f64::consts::PI).cos()))
                            .sqrt()
                } else {
                    0.0
                };
                off = off.max(inset[i]).max(swing);
                // Curves that overlap leave less than nothing: a length below zero, refused below.
                Geo::Arc {
                    centre,
                    radius: moved,
                    from_angle: start.as_deref().map_or(*from_angle, angle),
                    turn: *turn,
                    length: left * moved,
                }
            }
            Geo::Spline { .. } | Geo::Arc3 { .. } | Geo::Carried { .. } => piece.clone(),
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

/// A plane: a point on it, two directions in it at right angles, and its normal.
type Plane = [[f64; 3]; 4];

/// `blend` on three axes. A line that meets an arc in its direction lies in the arc's plane, and so does a line or an
/// arc that meets that line in its direction and lies in the plane (to 1 nm). Such a run of pieces is blended in its
/// plane, on the coordinates along the first arc's start and its direction there (`blend` as on two axes, with the
/// speeds of the pieces in space), and laid back into space: lines and arcs as such, the curves with their plane.
/// The deviation is the one in the plane, as the plane's coordinates keep distances. A line between two arcs in
/// different planes lies in both; it is cut in half, and each half goes with its arc.
///
/// Two arcs that meet in their direction in different planes keep their junction, which the path passes at the
/// speed `junction_speed` allows: a curve between them has to turn from one plane into the other, with torsion, and
/// the planar construction (each arc moved in within its plane, the curve fitted exactly between the two circles)
/// has no counterpart there. A piece that lies off its run's plane by more than 1 nm (a line written with too few
/// digits) ends the run, and its junctions stay as given.
fn blend3(geo: Vec<Geo>, speed: &[f64], axes: &[AxisInfo], tolerance: f64) -> (Vec<Geo>, f64) {
    // shortcut: a piece more than 1 nm off its run's plane ends the run, and two arcs in different planes keep their
    // junction; project such pieces (adding the projection to the deviation) or build a space curve with torsion there
    // if hand-written tilted paths or helix-like tool paths have to pass those junctions at speed.
    let plane_of =
        |g: &Geo| if let Geo::Arc3 { centre, u, w, .. } = g { Some([*centre, *u, *w, cross3(*u, *w)]) } else { None };
    let near =
        |p: &[f64], [o, _, _, normal]: &Plane| dot3([p[0] - o[0], p[1] - o[1], p[2] - o[2]], *normal).abs() < 1e-9;
    let lies_in = |g: &Geo, plane: &Plane| match g {
        Geo::Line { .. } => near(&g.at(0.0)[0], plane) && near(&g.at(g.length())[0], plane),
        Geo::Arc3 { centre, u, w, .. } => near(centre, plane) && norm3(cross3(cross3(*u, *w), plane[3])) < 1e-9,
        _ => false,
    };
    let (mut out, mut off) = (Vec::new(), 0f64);
    // The run being gathered, with the speeds of its pieces, and its plane once an arc has set it.
    let (mut run, mut plane): (Vec<(Geo, f64)>, Option<Plane>) = (Vec::new(), None);
    let mut flush = |run: Vec<(Geo, f64)>, plane: Option<Plane>| {
        let (blended, moved) = blend_in_plane(run, plane, axes, tolerance);
        out.extend(blended);
        off = off.max(moved);
    };
    for (g, v) in geo.into_iter().zip(speed.iter().copied()) {
        let joins = run.last().is_some_and(|(last, _)| {
            matches!(last, Geo::Line { .. } | Geo::Arc3 { .. })
                && matches!(g, Geo::Line { .. } | Geo::Arc3 { .. })
                && !corner(last, &g)
        });
        let own = plane_of(&g);
        match (joins, plane, own) {
            (true, Some(p), _) if lies_in(&g, &p) => run.push((g, v)),
            // An arc in another plane: a line before it that lies in both is shared out between the two runs.
            (true, Some(_), Some(q)) => {
                let shared = match run.last() {
                    Some((line @ Geo::Line { from, dir, length }, _)) if lies_in(line, &q) => {
                        let middle: Vec<f64> = from.iter().zip(dir).map(|(f, d)| f + 0.5 * length * d).collect();
                        Some((
                            Geo::Line { from: from.clone(), dir: dir.clone(), length: 0.5 * length },
                            Geo::Line { from: middle, dir: dir.clone(), length: 0.5 * length },
                        ))
                    }
                    _ => None,
                };
                let mut next = Vec::new();
                if let Some((first, second)) = shared {
                    let (_, speed) = run.pop().unwrap();
                    run.push((first, speed));
                    next.push((second, speed));
                }
                flush(std::mem::replace(&mut run, next), plane);
                (run, plane) = ([run, vec![(g, v)]].concat(), own);
            }
            // The first arc of a run of lines sets its plane; lines before it that do not lie in it stay as given.
            (true, None, Some(q)) => {
                let keep = run.len() - run.iter().rev().take_while(|(line, _)| lies_in(line, &q)).count();
                let rest = run.split_off(keep);
                flush(std::mem::replace(&mut run, rest), None);
                run.push((g, v));
                plane = own;
            }
            (true, None, None) => run.push((g, v)),
            _ => {
                flush(std::mem::replace(&mut run, vec![(g, v)]), plane);
                plane = own;
            }
        }
    }
    flush(run, plane);
    (out, off)
}

/// A run of lines and arcs in one plane (`blend3`) blended there and laid back into space.
fn blend_in_plane(run: Vec<(Geo, f64)>, plane: Option<Plane>, axes: &[AxisInfo], tolerance: f64) -> (Vec<Geo>, f64) {
    let Some([o, e1, e2, normal]) = plane.filter(|_| run.len() > 1) else {
        return (run.into_iter().map(|(g, _)| g).collect(), 0.0);
    };
    let along = |d: &[f64]| vec![dot3([d[0], d[1], d[2]], e1), dot3([d[0], d[1], d[2]], e2)];
    let flat = |p: &[f64]| along(&[p[0] - o[0], p[1] - o[1], p[2] - o[2]]);
    let (pieces, speeds): (Vec<Geo>, Vec<f64>) = run
        .into_iter()
        .map(|(g, v)| {
            let g = match g {
                Geo::Line { from, dir, length } => Geo::Line { from: flat(&from), dir: along(&dir), length },
                Geo::Arc3 { centre, u, w, radius, length } => {
                    let c = flat(&centre);
                    Geo::Arc {
                        centre: [c[0], c[1]],
                        radius,
                        from_angle: dot3(u, e2).atan2(dot3(u, e1)),
                        turn: dot3(cross3(u, w), normal).signum(),
                        length,
                    }
                }
                other => other,
            };
            (g, v)
        })
        .unzip();
    let (blended, moved) = blend(pieces, &speeds, axes, tolerance);
    let up = |v: &[f64], point: f64| [0, 1, 2].map(|i| point * o[i] + v[0] * e1[i] + v[1] * e2[i]);
    let lifted = blended
        .into_iter()
        .map(|g| match g {
            Geo::Line { from, dir, length } => {
                Geo::Line { from: up(&from, 1.0).to_vec(), dir: up(&dir, 0.0).to_vec(), length }
            }
            Geo::Arc { centre, radius, from_angle, turn, length } => {
                let (sin, cos) = from_angle.sin_cos();
                Geo::Arc3 {
                    centre: up(&centre, 1.0),
                    u: up(&[cos, sin], 0.0),
                    w: up(&[-turn * sin, turn * cos], 0.0),
                    radius,
                    length,
                }
            }
            Geo::Clothoid { from, heading, k0, k1, length, .. } => {
                Geo::Clothoid { from, heading, k0, k1, length, plane: Some([o, e1, e2]) }
            }
            other => other,
        })
        .collect();
    (lifted, moved)
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
            let Geo::Spline { c, h, dims, .. } = &geo[at] else { unreachable!() };
            let (c, h, dims) = (**c, *h, *dims);
            // The neighbour's curvature where it meets the spline, and the spline's own there.
            let target = &geo[other].at(if into { geo[other].length() } else { 0.0 })[2];
            let [d, d2, _] = poly_rates(&c, if into { 0.0 } else { h });
            let along = dot(&d, &d2);
            let change: Wide = std::array::from_fn(|i| target.get(i).copied().unwrap_or(0.0) - (d2[i] - along * d[i]));
            let dk = norm(&change);
            let v = speed(&geo[other]).min(speed(&geo[at]));
            let needed = EASE_RATE * SPLINE_MARGIN * dk * v.powi(3) / (0.4 * jerk);
            let l = (tolerance / (EASE_PEAK * dk)).sqrt().min(needed).min(0.5 * h);
            if dk > 1e-9 && l > 0.0 {
                // The eased part and the rest, each a polynomial of its own with u from 0.
                let (mut eased, rest) = if into { (c, poly_shift(&c, l)) } else { (poly_shift(&c, h - l), c) };
                // delta expanded in powers of u: from the junction at u = 0, or towards it at u = l.
                let powers = if into {
                    [0.5, -1.5 / l, 1.5 / (l * l), -0.5 / (l * l * l)]
                } else {
                    [0.0, 0.5 / l, -1.0 / (l * l), 0.5 / (l * l * l)]
                };
                for (m, p) in powers.iter().enumerate() {
                    for i in 0..WIDEST {
                        eased[m + 2][i] += p * change[i];
                    }
                }
                if let (Some(eased), Some(rest)) = (span(eased, l, dims), span(rest, h - l, dims)) {
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

/// `largest` to a share of 1e-9 of `high`: for the time-optimal plan's top speeds, where each try runs a piece.
fn highest_fitting(mut low: f64, mut high: f64, ok: impl Fn(f64) -> bool) -> f64 {
    if ok(high) {
        return high;
    }
    while high - low > 1e-9 * high {
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
    let rows = |limit: fn(&AxisInfo) -> f64| {
        axes.iter()
            .enumerate()
            .map(|(i, a)| ((0..axes.len()).map(|k| (k == i) as u8 as f64).collect(), limit(a)))
            .collect()
    };
    [rows(|a| a.max_vel), rows(|a| a.max_acc), rows(|a| a.max_jerk)]
}

/// Speed, acceleration and jerk along a piece that keep the tool within its bounds wherever on the piece.
fn limits(geo: &Geo, bounds: &[Bounds; 3]) -> [f64; 3] {
    let tightest = |per_row: &dyn Fn(&[f64]) -> f64| {
        bounds.each_ref().map(|rows| rows.iter().map(|(row, most)| most / per_row(row)).fold(f64::INFINITY, f64::min))
    };
    // The direction turns through the axes on a curve, so there a bound holds for the length of the vector.
    let any = || tightest(&|row| row.iter().map(|w| w * w).sum::<f64>().sqrt());
    match geo {
        Geo::Line { dir, .. } => tightest(&|row| row.iter().zip(dir).map(|(w, d)| w * d).sum::<f64>().abs()),
        // shortcut: as the straight line between the two poses; the slerp's orientation rates differ from the chord's by
        // the turn's curvature in rotation-vector coordinates (a few percent for a quarter turn about a tilted axis).
        // Take the most of `at`'s first derivative along it where a cycle runs its rotation axes at their limits.
        Geo::Slerp { from, to, length } => tightest(&|row| {
            row.iter().zip(from.iter().zip(to)).map(|(w, (f, t))| w * (t - f) / length).sum::<f64>().abs()
        }),
        Geo::Arc { radius, .. } | Geo::Arc3 { radius, .. } => curved(*radius, 0.0, any()),
        // A transition curve counts as an arc of its tightest radius.
        Geo::Clothoid { k0, k1, length, .. } => curved(1.0 / k0.abs().max(k1.abs()), (k1 - k0).abs() / length, any()),
        // So does a spline span, with the fastest its curvature changes anywhere on it.
        Geo::Spline { tightest, rate, .. } => curved(1.0 / tightest, *rate, any()),
        // An arc with other axes carried along is a helix: its curvature is the arc's times the share of the distance
        // that is on the arc, squared.
        Geo::Carried { arc, length, .. } => {
            let Geo::Arc3 { radius, length: on_arc, .. } = **arc else { unreachable!("a carried piece is an arc") };
            curved(radius / (on_arc / length).powi(2), 0.0, any())
        }
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
pub fn plan(
    axes: &[AxisInfo],
    start: &[f64],
    pieces: &[Piece],
    cycle: f64,
    tolerance: impl Into<Tolerance>,
    slack: f64,
) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
    plan_at(axes, start, pieces, cycle, tolerance, slack, f64::INFINITY)
}

/// `plan` with the speed along the path no higher than `feed` (in the axes' units per second, the length of the speed
/// vector over all axes): a program's feed (G-code F, a robot's v or $VEL.CP), which is the speed along the path, not
/// along each axis. Infinite: the axes' limits alone.
pub fn plan_at(
    axes: &[AxisInfo],
    start: &[f64],
    pieces: &[Piece],
    cycle: f64,
    tolerance: impl Into<Tolerance>,
    slack: f64,
    feed: f64,
) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
    let speed = axis_speed(axes);
    let (geo, bounds) =
        (geometry(axes, start, pieces, tolerance, &|g| speed(g).min(feed), slack)?.0, axis_bounds(axes));
    let limits: Vec<[f64; 3]> = geo
        .iter()
        .map(|g| {
            let [v, a, j] = limits(g, &bounds);
            [v.min(feed), a, j]
        })
        .collect();
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
        phases.push([
            (steady(v0), 0.0),
            (up.0, j),
            (up.1, 0.0),
            (up.0, -j),
            (cruise, 0.0),
            (down.0, -j),
            (down.1, 0.0),
            (down.0, j),
            (steady(v1), 0.0),
        ]);
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

/// What a time-optimal plan keeps within beside each motor's torque-speed curve, its rated speed and the axes' jerk
/// limits (see `plan_time_optimal`).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PlanLimits {
    /// The share of each torque limit the plan may use (`profile.torque_reserve`): of the motor's torque-speed curve, of
    /// the torque of the drive's maximum current, and of the gearbox's and the pulley's torque rating.
    pub reserve: f64,
    /// The lowest and the highest tension a belt section may have and the share of the room to them the plan may use.
    pub tension: [f64; 3],
    /// Per motor (in the order of `Machine::motors`) its gearbox: the ratio, the inertia that turns at its input (rotor
    /// and gearbox, kg m^2), the most torque at its output (N m), `ratings.gearbox_max_torque_nm` (M2max) or
    /// `pulley_max_torque_nm` where that is lower (infinite where the config rates neither), and its efficiency
    /// (`gearbox.efficiency` with effects.gearbox_efficiency, else 1). `None` (or no entry) where the machine reports no
    /// gearbox output torque, or the gearbox is neither rated nor lossy.
    pub gearbox: Vec<Option<[f64; 4]>>,
    /// The most load a guide may carry, `ratings.guide_max_load_n` (N); infinite without. The plan keeps `reserve` of it.
    pub guide: f64,
    /// Per motor (in the order of `Machine::motors`) the friction on its shaft as the motor and its gearbox have it:
    /// [Coulomb level, viscous, smoothing speed, gearbox drag at its reference speed, that speed] in N m and rad/s
    /// (`mech::ShaftFriction`, `mech::GearDrag`), all 0 with effects.motor_friction and gearbox_drag off. No entry: none.
    #[serde(default)]
    pub friction: Vec<[f64; 5]>,
}

impl PlanLimits {
    /// The torque share and the belt only, with no gearbox or guide rating and no friction.
    pub fn new(reserve: f64, tension: [f64; 3]) -> Self {
        Self { reserve, tension, gearbox: Vec::new(), guide: f64::INFINITY, friction: Vec::new() }
    }

    /// The steady friction on motor `m`'s shaft at the speed `speed` (rad/s, at least 0): c tanh(w / smoothing) + b w +
    /// T0 (w / n0)^(2/3), as `mech::ShaftFriction::torque` has it at the reference temperature, without the breakaway's
    /// Stribeck rise near a standstill.
    pub fn shaft_friction(&self, m: usize, speed: f64) -> f64 {
        let Some(&[coulomb, viscous, smoothing, drag, drag_speed]) = self.friction.get(m) else { return 0.0 };
        let drag = if drag > 0.0 { drag * (speed / drag_speed).powf(crate::mech::DRAG_EXPONENT) } else { 0.0 };
        coulomb * (speed / smoothing).tanh() + viscous * speed + drag
    }
}

/// The most torque a motor gives the time-optimal plan, before the reserve: its torque-speed curve at the rated speed
/// (level up to it) or, with effects.drive_current_limit, the torque of the drive's maximum current where that is lower.
fn motor_torque(drive: &DriveParams) -> f64 {
    let curve = drive.torque_curve(drive.rated_speed).1;
    if drive.drive_max_current.is_finite() { curve.min(drive.torque_of(drive.drive_max_current)) } else { curve }
}

/// Speeds at which a motor's torque-speed curve is read for the time-optimal plan (`MotorCurve`), evenly from zero to
/// its maximum speed.
const CURVE_SAMPLES: usize = 256;

/// What one motor gives the time-optimal plan by its speed: its torque-speed curve (`DriveParams::torque_curve`, which
/// falls above the rated speed: linearly to the no-load speed, or with effects.field_weakening as the d-q model has it),
/// or the torque of the drive's maximum current where that is lower, as it drives (`motoring`, torque in the direction
/// it turns) and as it brakes, less what the reserve keeps at a standstill ((1 - reserve) times the torque there: for
/// friction and elasticity, which do not shrink with the speed), read at `CURVE_SAMPLES` + 1 speeds `step` apart from
/// zero to its maximum speed (`DriveParams::max_speed`) and lowered so that neither rises with the speed. Up to the
/// rated speed that is `reserve` times the curve, as before; above it the room falls faster than the curve. The motor's
/// own shaft friction and its gearbox's drag at each speed (`PlanLimits::shaft_friction`) come off as it drives; as it
/// brakes they would help, which is not counted (at a standstill there is none).
#[derive(Debug)]
struct MotorCurve {
    step: f64,
    motoring: Vec<f64>,
    braking: Vec<f64>,
}

impl MotorCurve {
    fn new(drive: &DriveParams, reserve: f64, friction: impl Fn(f64) -> f64) -> Self {
        let step = drive.max_speed / CURVE_SAMPLES as f64;
        let most =
            if drive.drive_max_current.is_finite() { drive.torque_of(drive.drive_max_current) } else { f64::INFINITY };
        let at = |speed: f64| {
            let (low, high) = drive.torque_curve(speed);
            [high.min(most), (-low).min(most)]
        };
        // What friction and elasticity keep of the torque at a standstill, they keep at every speed: where the curve has
        // less than that left, the rigid torque has to stay as far on the other side (below zero here).
        let kept = at(0.0).map(|t| (1.0 - reserve) * t);
        let (mut motoring, mut braking) = (Vec::new(), Vec::new());
        let (mut drives, mut brakes) = (f64::INFINITY, f64::INFINITY);
        for i in 0..=CURVE_SAMPLES {
            let [high, low] = at(i as f64 * step);
            drives = drives.min((1.0 - KEPT_BACK) * (high - kept[0] - friction(i as f64 * step)));
            brakes = brakes.min((1.0 - KEPT_BACK) * (low - kept[1]));
            motoring.push(drives);
            braking.push(brakes);
        }
        Self { step, motoring, braking }
    }

    /// The highest speed the motor may turn at.
    fn max_speed(&self) -> f64 {
        self.step * CURVE_SAMPLES as f64
    }

    /// The lowest and the highest torque at the (signed) motor speed `speed` and at every speed of its sign below it: the
    /// curve as read at the next speed up, which is no higher, as the curve does not rise. Beyond the maximum speed an
    /// empty range.
    fn range(&self, speed: f64) -> (f64, f64) {
        // The rounding of a speed at the maximum is not beyond it.
        let i = (speed.abs() / self.step * (1.0 - 1e-12)).ceil() as usize;
        let (Some(drives), Some(brakes)) = (self.motoring.get(i), self.braking.get(i)) else {
            return (f64::INFINITY, f64::NEG_INFINITY);
        };
        if speed >= 0.0 { (-brakes, *drives) } else { (-drives, *brakes) }
    }
}

/// Plans the timing of a path as fast as the motors allow, within the jerk limits of the axes, point by point.
///
/// The path is the one `plan` times (transition curves included, their lengths for the speeds the motors allow per
/// piece, `motor_limits`), with its look-ahead over the junctions, the speed at a junction and the stop at a corner.
/// Within a piece the limits are read at places about 2 mm apart (`piece_limits`): at each the rigid model gives how
/// fast each motor turns per speed along the path, and its torque as c a + d v^2 + e for the acceleration a and the
/// speed v along the path (with the centripetal and Coriolis torque of the speed, `Machine::feedforward_torque_moving`,
/// and e the torque that holds the tool there). Each motor is judged by its own drive and motor (`drives`, in the order
/// of `Machine::motors`): it may turn up to its maximum speed (`DriveParams::max_speed`) and give `reserve` times its
/// torque-speed curve at the speed it turns (level up to the rated speed, falling above it: `MotorCurve`), or of the
/// torque of the drive's maximum current where that is lower (effects.drive_current_limit), each way as gravity pushes
/// and as it drives or brakes. The rest is for friction and elasticity, which the
/// rigid model leaves out. Each axis keeps its jerk limit: on a line the ramps use it in full, on a curve 0.4 of it,
/// and the rest is for the jerk that speed and acceleration make on the curve.
///
/// A gearbox (`PlanLimits::gearbox`) keeps `reserve` times its output torque rating: the output torque is the ratio
/// times the motor's torque less what turns the inertia at the input, again of the form c a + d v^2 + e. A guide keeps
/// its load rating (`PlanLimits::guide`): the machine gives each guide's load as a load at rest plus the size of a part
/// linear in the acceleration (`Machine::rigid_guide_loads`).
///
/// Each piece then follows the highest speed its places allow (`smoothed`: the places' top speeds, lowered where the
/// speed could not come down in time or would bend too sharply for the jerk) from below, as fast as the jerk and the
/// acceleration there allow, from its start and from its end (`follow`), and the two runs meet at a top speed at which
/// both have no acceleration (`piece_timing`): the highest that fits. The velocity and acceleration limits of the axes
/// play no part.
///
/// The belt is a limit of the plan too: `tension` is the lowest and the highest tension a belt section may have
/// (`ratings.belt_min_tension_n` and `belt_max_tension_n`, each less `profile.tension_margin_n`; -inf and inf where the
/// config has none) and the share of the room to them the plan may use (`profile.tension_reserve`). The rigid model gives
/// each section's tension at each place as c + g . acceleration (`Machine::rigid_tensions`), linear in the acceleration,
/// so each section and place is one more bound: |g . acceleration| <= share * min(highest - c, c - lowest), the same
/// room both ways. The rest of the room is for what the rigid model leaves out: the margin for the guide friction the
/// belt pulls, the share (0.5 in the shipped configs) for the belt's vibration, which after a quick ramp of acceleration
/// overshoots by up to the rigid model's change of tension, either way.
pub fn plan_time_optimal(
    machine: &dyn Machine,
    drives: &[&DriveParams],
    limits: &PlanLimits,
    start: &[f64],
    pieces: &[Piece],
    cycle: f64,
    tolerance: impl Into<Tolerance>,
    slack: f64,
) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
    let ((geo, _), _) = time_optimal_geometry(machine, drives, limits, start, pieces, tolerance, slack)?;
    let curves: Arc<[MotorCurve]> = drives
        .iter()
        .enumerate()
        .map(|(m, drive)| MotorCurve::new(drive, limits.reserve, |w| limits.shaft_friction(m, w)))
        .collect();
    let places =
        geo.iter().map(|piece| piece_limits(machine, &curves, limits, piece)).collect::<Result<Vec<_>, _>>()?;
    timed_by_motors(machine.axes(), &geo, &places, cycle)
}

/// The same path as `plan` times, transition curves included, their lengths for the speeds the motors allow, with
/// the deviation and the motors' limits per piece; refused where a piece has no torque left beyond holding the tool
/// or turns no motor, or where the belt holding the tool there is already beyond its tension limits.
fn time_optimal_geometry(
    machine: &dyn Machine,
    drives: &[&DriveParams],
    limits: &PlanLimits,
    start: &[f64],
    pieces: &[Piece],
    tolerance: impl Into<Tolerance>,
    slack: f64,
) -> Result<((Vec<Geo>, f64), Vec<[f64; 3]>), CommandError> {
    let (plan, piece_limits) = (limits, motor_limits(machine, drives, limits, limits.tension));
    let path = geometry(machine.axes(), start, pieces, tolerance, &|piece| piece_limits(piece)[0], slack)?;
    let mut limited = Vec::new();
    let usable = |limits: &[f64; 3]| limits.iter().all(|l| l.is_finite() && *l > 0.0);
    for piece in &path.0 {
        let limits = piece_limits(piece);
        if !usable(&limits) {
            // Said by what is short: the torque, or the belt alone.
            let torque_alone =
                usable(&motor_limits(machine, drives, plan, [f64::NEG_INFINITY, f64::INFINITY, 1.0])(piece));
            return Err(CommandError { key: if torque_alone { "error-path-tension" } else { "error-path-torque" } });
        }
        limited.push(limits);
    }
    Ok((path, limited))
}

/// `deviation` for the path `plan_time_optimal` plans: its transition curves are as long as the speeds the
/// motors allow need them to be, not the speeds of the axis limits. Refused as that plan is.
pub fn deviation_time_optimal(
    machine: &dyn Machine,
    drives: &[&DriveParams],
    limits: &PlanLimits,
    start: &[f64],
    pieces: &[Piece],
    tolerance: impl Into<Tolerance>,
    slack: f64,
) -> Result<f64, CommandError> {
    Ok(time_optimal_geometry(machine, drives, limits, start, pieces, tolerance, slack)?.0.1)
}

/// The speed, acceleration and jerk limits along a piece that `plan_time_optimal` uses: speed and acceleration
/// from the motors and the belt's tension limits, jerk from the axes.
fn motor_limits<'a>(
    machine: &'a dyn Machine,
    drives: &'a [&'a DriveParams],
    plan: &'a PlanLimits,
    [lowest, highest, share]: [f64; 3],
) -> impl Fn(&Geo) -> [f64; 3] + 'a {
    let reserve = plan.reserve;
    let axes = machine.axes();
    let [_, _, jerk] = axis_bounds(axes);
    let h = 1e-3;
    // shortcut: the transition curves are sized for the corner of each motor's curve (its rated speed at its full torque);
    // the timing then uses the whole curve. Size them along the curve if a plan above the rated speed needs longer ones.
    // Less the shaft's friction at the rated speed (`MotorCurve`).
    let torque = move |m: usize| reserve * motor_torque(drives[m]) - plan.shaft_friction(m, drives[m].rated_speed);
    move |piece| {
        let per_motor = |f: &dyn Fn(&mut [f64])| {
            let mut out = vec![0.0; machine.motors().len()];
            f(&mut out);
            out
        };
        let (mut vel, mut acc) = (Vec::new(), Vec::new());
        for i in 0..=8 {
            let pose = &piece.at(piece.length() * i as f64 / 8.0)[0];
            let unit = |axis: usize, size: f64| -> Vec<f64> {
                (0..axes.len()).map(|k| if k == axis { size } else { 0.0 }).collect()
            };
            let hold = per_motor(&|out| machine.feedforward_torque(pose, &unit(0, 0.0), out));
            // Per axis and motor: the torque for an acceleration of 1 m/s^2, and the angle per metre.
            let per_acc: Vec<Vec<f64>> = (0..axes.len())
                .map(|a| per_motor(&|out| machine.feedforward_torque(pose, &unit(a, 1.0), out)))
                .collect();
            let angle = |a: usize, sign: f64| {
                per_motor(&|out| {
                    machine.inverse(&pose.iter().zip(unit(a, sign * h)).map(|(p, d)| p + d).collect::<Vec<_>>(), out)
                })
            };
            let turn: Vec<Vec<f64>> = (0..axes.len())
                .map(|a| {
                    angle(a, 1.0)
                        .iter()
                        .zip(angle(a, -1.0))
                        .map(|(ahead, behind)| (ahead - behind) / (2.0 * h))
                        .collect()
                })
                .collect();
            for m in 0..hold.len() {
                vel.push((turn.iter().map(|per_axis| per_axis[m]).collect(), drives[m].rated_speed));
                acc.push((per_acc.iter().map(|per_axis| per_axis[m] - hold[m]).collect(), torque(m) - hold[m].abs()));
            }
            if lowest.is_finite() || highest.is_finite() {
                let tensions = |acc: &[f64]| {
                    let mut out = Vec::new();
                    machine.rigid_tensions(pose, acc, &mut out);
                    out
                };
                let still = tensions(&unit(0, 0.0));
                let per_acc: Vec<Vec<f64>> = (0..axes.len()).map(|a| tensions(&unit(a, 1.0))).collect();
                for (k, c) in still.iter().enumerate() {
                    acc.push((
                        per_acc.iter().map(|per_axis| per_axis[k] - c).collect(),
                        share * (highest - c).min(c - lowest),
                    ));
                }
            }
        }
        limits(piece, &[vel, acc, jerk.clone()])
    }
}

// ---------------------------------------------------------------------------------------------
// Timing as fast as the motors allow, point by point.

/// Whether a piece of `length` under `limits` can be timed from the speed `v0` at its start to `v1` at its end: by
/// `piece_timing` with its top at the higher of the two, or (`joins`) by `curve_join`.
fn piece_fits(limits: &PieceLimits, length: f64, [v0, v1]: [f64; 2], cycle: f64, joins: bool) -> bool {
    let (hold, step) = (2.0 * cycle, cycle / STEPS_PER_CYCLE);
    piece_timing(limits, length, [v0, v1, v0.max(v1)], hold, step, 0.0).is_some()
        || joins && curve_join(limits, length, [v0, v1], hold, step).is_some()
}

/// The quickest timing of a piece from `v0` to `v1` (`piece_fits`): `piece_timing` at the highest top that fits, or
/// (`joins`) `curve_join` where that is quicker or the only one.
fn piece_knots(
    limits: &PieceLimits,
    length: f64,
    [v0, v1]: [f64; 2],
    cycle: f64,
    joins: bool,
) -> Result<Vec<[f64; 4]>, CommandError> {
    let (hold, step) = (2.0 * cycle, cycle / STEPS_PER_CYCLE);
    let timing = |top: f64, slack: f64| piece_timing(limits, length, [v0, v1, top], hold, step, slack);
    // The top speed: the highest that fits, up to the curve's highest. The runs are the same whatever the top until
    // they brake for it, so the higher top is the quicker; it fits while the runs reach it before they meet.
    // shortcut: halving takes the tops that fit as one range from `low`; where the speed curve has two humps with a
    // dip between them, a top between the dip and the lower hump does not fit and the halving may settle below the
    // dip. Search each hump's range if machines with such pieces need it.
    let low = v0.max(v1);
    let peak = timing(low, 0.0).map(|_| {
        let highest = limits.curve[0].iter().fold(low * low, |a: f64, b| a.max(*b)).sqrt();
        let top = highest_fitting(low, highest, |top| timing(top, 0.0).is_some());
        // The look-ahead has made every piece fit; the slack is for a difference in the last bit.
        timing(top, 1e-9 * length)
    });
    let joined = if joins { curve_join(limits, length, [v0, v1], hold, step) } else { None };
    let duration = |knots: &Option<Vec<[f64; 4]>>| knots.as_ref().map_or(f64::INFINITY, |k| k[k.len() - 1][0]);
    let peak = peak.flatten();
    let quicker = if duration(&joined) < duration(&peak) { joined } else { peak };
    quicker.ok_or(CommandError { key: "error-path" })
}

/// The speed at the start of each piece and at the end of the last, at rest at both ends of the path, as fast as each
/// junction and the pieces beside it allow: `junction_speed` (the step of the axes' rates), what the speed curve of each
/// piece allows at the junction (its first or its last place, from where it can still slow down along the curve), and
/// the look-ahead of `timed` with `piece_fits`. A run from the junction into either piece has to keep below that piece's
/// speed curve (`follow` with no top), which caps the junction before the look-ahead. The look-ahead takes each speed as
/// fitting from the lower of a junction's speed and the next one's on, which holds where both are no faster than the
/// slowest place of their pieces; where the speeds that come out leave a piece that does not fit, the two junctions of
/// that piece are lowered to what they are with `joins` off and the look-ahead runs again, piece by piece, as long as
/// pieces are left that do not fit (at most once per piece; then the whole path as with `joins` off). With `joins` off,
/// `curve_join` is left out and a junction is passed no faster than the slowest place of either piece, as before 14
/// November (also for the comparison in the tests).
fn junction_speeds(axes: &[AxisInfo], geo: &[Geo], limits: &[PieceLimits], cycle: f64, joins: bool) -> Vec<f64> {
    let n = geo.len();
    let (hold, step) = (2.0 * cycle, cycle / STEPS_PER_CYCLE);
    let fits = |i: usize, v0: f64, v1: f64| piece_fits(&limits[i], geo[i].length(), [v0, v1], cycle, joins);
    let curve_end = |l: &PieceLimits, end: bool| l.curve[0][if end { l.curve[0].len() - 1 } else { 0 }].sqrt();
    // A run from the speed `v` at piece i's start (`backward`: its end) that follows its curve all along.
    let enters = |i: usize, v: f64, backward: bool| {
        follow(&limits[i], v, f64::INFINITY, [hold * v, geo[i].length()], backward, step)
    };
    let mut speed = vec![0.0; n + 1];
    for i in 1..n {
        // A piece too short for the steady cycles at both of its ends is passed slowly enough to have them.
        let short = geo[i - 1].length().min(geo[i].length()) / (2.0 * hold);
        let (before, after) = if joins {
            (curve_end(&limits[i - 1], true), curve_end(&limits[i], false))
        } else {
            (limits[i - 1].lowest, limits[i].lowest)
        };
        speed[i] = junction_speed(&geo[i - 1], &geo[i], axes, cycle).min(before).min(after).min(short);
        let slowest = limits[i - 1].lowest.min(limits[i].lowest);
        if joins && speed[i] > slowest {
            let ok = |v: f64| enters(i, v, false).is_some() && enters(i - 1, v, true).is_some();
            speed[i] = highest_fitting(slowest.min(speed[i]), speed[i], ok);
        }
    }
    let look_ahead = |speed: &mut Vec<f64>| {
        for i in (0..n).rev() {
            let after = speed[i + 1];
            speed[i] = largest(after.min(speed[i]), speed[i], |v| fits(i, v, after));
        }
        for i in 0..n {
            let before = speed[i];
            speed[i + 1] = largest(before.min(speed[i + 1]), speed[i + 1], |v| fits(i, before, v));
        }
    };
    look_ahead(&mut speed);
    if !joins {
        return speed;
    }
    // Per piece that does not fit, its two junctions as without the joins, where that is slower.
    let mut without: Option<Vec<f64>> = None;
    for _ in 0..n {
        let misfits: Vec<usize> = (0..n).filter(|&i| !fits(i, speed[i], speed[i + 1])).collect();
        if misfits.is_empty() {
            return speed;
        }
        let without = without.get_or_insert_with(|| junction_speeds(axes, geo, limits, cycle, false));
        for i in misfits {
            (speed[i], speed[i + 1]) = (speed[i].min(without[i]), speed[i + 1].min(without[i + 1]));
        }
        look_ahead(&mut speed);
    }
    junction_speeds(axes, geo, limits, cycle, false)
}

/// The places at which the time-optimal plan reads the rigid model are about this far apart along a piece (m), and on
/// a curve no more than `PLACE_TURN` of its turning (rad), at least `PLACES_LEAST` and at most `PLACES_MOST` of them
/// per piece. Between two places the bounds are taken as straight; where the direction turns by 0.02 rad they bend
/// away from that by about 0.02^2 / 8 = 5e-5 of themselves, within what the plan keeps back (`KEPT_BACK`).
const PLACE_SPACING: f64 = 2e-3;
const PLACE_TURN: f64 = 0.02;
const PLACES_LEAST: usize = 32;
const PLACES_MOST: usize = 1000;
/// The share of each torque and tension limit that the time-optimal plan keeps back, for what falls between its
/// places and the rounding of its steps.
const KEPT_BACK: f64 = 2e-4;
/// Steps of the time-optimal plan's ramps per profile cycle.
const STEPS_PER_CYCLE: f64 = 4.0;
/// The step of the central differences by which the plan reads the motors' speed per speed along the path (m).
const TURN_STEP: f64 = 1e-6;
/// A motor that turns more than this per distance along the path (rad per m, or per rad in joint space) is at a
/// singularity: a gearbox of 100 on an arm of 0.1 m turns 1e3, so this is far beyond any machine that moves.
/// shortcut: one fixed bound; derive it from the motors' rated speed and the slowest speed a plan may crawl at if a
/// machine ever needs more.
const SINGULAR_TURN: f64 = 1e6;
/// The longest a time-optimal plan may take (s): a path that crawls this slowly passes a singularity, or a pose where
/// the motors nearly cannot move the tool, and its poses (one per profile cycle, a million at 1 ms) would be unbounded.
const PLAN_LONGEST: f64 = 1000.0;
/// The step of the second differences by which the plan reads how the motors' speed per speed along the path changes
/// along it, for the torque that turns a gearbox's input (m, or rad on a path in joint space).
const BEND_STEP: f64 = 1e-4;
/// The share of the ceilings of acceleration that the speed curve a piece follows may use (`smoothed`), and the share of
/// the ramps' jerk its own jerk may use: the rest lets a run catch up with the curve and follow its bends.
const CURVE_ACCELERATION: f64 = 0.9;
const CURVE_JERK: f64 = 0.5;
/// How far a run with no top may get above the speed curve it follows (`follow`, for `curve_join`), as a share of the
/// curve's speed: a run that comes onto the curve at its ceiling of acceleration passes it by some 1e-4 before it
/// settles on it; one that starts faster than the curve lets it slow down passes it by far more. The plans' tests allow
/// the motors 0.1 % over their rated speed for the same rounding.
const CURVE_OVERSHOOT: f64 = 1e-3;
/// The most sweeps that lower the speed curve until its acceleration and jerk hold.
const CURVE_SWEEPS: usize = 300;
/// The speeds, evenly from zero to the curve's highest, at which the ceilings of acceleration are worked out
/// (`PieceLimits::most`): a run takes the ceilings of the lowest level at or above the speed it can reach.
const LEVELS: usize = 32;
/// The most steps of a run along a piece (`follow`): one that takes longer is held back by a ceiling near zero and
/// refused. 2^18 steps are 65 s at a profile cycle of 1 ms.
const RUN_STEPS: usize = 1 << 18;
/// The most points a time-optimal plan may have: ten million, 2.8 hours at a planning cycle of 1 ms (160 MB of
/// six-axis poses).
const PLAN_POINTS_MOST: f64 = 1e7;

/// What bounds the speed v and the acceleration a along the path at one place of a piece (p', p'', p''': the pose's
/// derivatives by the distance along the path).
struct Place {
    /// low <= c a + d v^2 + e <= high, as [c, d, e, low, high]: per motor its torque (c per acceleration, d per speed
    /// squared: the centripetal and Coriolis part, e the torque that holds the tool), per belt section its change of
    /// tension (e = 0).
    rows: Vec<[f64; 5]>,
    /// Per motor (the first rows), how fast it turns per speed along the path (rad/m, signed): its row's low and high
    /// are then those of its curve (`curves`) at that speed, not the row's own (which are those at a standstill).
    turn: Vec<f64>,
    curves: Arc<[MotorCurve]>,
    /// Per motor the friction of the guides it drives (`Machine::rigid_friction`), as [c, b] for the torque c + b v it
    /// gives against them at the speed v along the path; empty where the machine tells none.
    guide: Vec<[f64; 2]>,
    /// Per axis, 3 |p''| v |a| + |p'''| v^3 <= room as [3 |p''|, |p'''|, room]: the part of the axis jerk that speed and
    /// acceleration take on a curve. The rest of the jerk limit is for the ramps.
    bends: Vec<[f64; 3]>,
    /// The highest speed at which the place holds every row and bend without acceleration.
    top: f64,
}

impl Place {
    /// The low and high of row `i` at the speed `v` along the path, and at every speed below it.
    fn bounds(&self, i: usize, v: f64) -> (f64, f64) {
        match self.turn.get(i) {
            Some(turn) => {
                let (low, high) = self.curves[i].range(turn * v);
                // The guides' friction takes its torque off the room on its side, at its most over the speeds up to v
                // (from none at a standstill).
                let [c, b] = self.guide.get(i).copied().unwrap_or([0.0; 2]);
                let (least, most) = (c.min(c + b * v).min(0.0), c.max(c + b * v).max(0.0));
                (low - least, high - most)
            }
            None => (self.rows[i][3], self.rows[i][4]),
        }
    }

    /// The most acceleration (`sign` 1) or deceleration (-1) along the path that holds every row and bend here at
    /// every speed from zero to `top`. Rows are linear in v^2, so the two ends of that range are the worst, against
    /// bounds that do not widen with the speed (a motor's curve), so those at `top`; the bends get tighter with the
    /// speed, so `top` is.
    fn most(&self, top: f64, sign: f64) -> f64 {
        let x = top * top;
        let rows = self.rows.iter().enumerate().filter(|(_, r)| r[0] != 0.0).map(|(i, [c, d, e, ..])| {
            let (low, high) = self.bounds(i, top);
            let room = |at: f64| if c * sign > 0.0 { high - at } else { at - low };
            room(*e).min(room(d * x + e)) / c.abs()
        });
        let bends = self
            .bends
            .iter()
            .filter(|b| b[0] > 0.0 && top > 0.0)
            .map(|[p2, p3, room]| (room - p3 * top.powi(3)) / (p2 * top));
        rows.chain(bends).fold(f64::INFINITY, f64::min).max(0.0)
    }
}

/// A piece as the time-optimal plan sees it: its places, evenly spaced by `spacing` from its start to its end, the
/// jerk along the path its ramps may use, the lowest of the places' speeds without acceleration, and for a run from
/// the start ([0]) and one from the end ([1]), each by the distance from where it starts: the speed curve it follows
/// (`smoothed`, as squared speeds at the places) and, per speed level (`level` apart, `LEVELS` of them), the most
/// acceleration and deceleration of the run at each place at speeds up to that level (`ceiling`), as
/// most[run][acceleration or deceleration][level][place].
struct PieceLimits {
    places: Vec<Place>,
    spacing: f64,
    jerk: f64,
    lowest: f64,
    curve: [Vec<f64>; 2],
    /// Per run, the lowest of the curve over 2^i places from each place on, as lowest_ahead[run][i][place].
    lowest_ahead: [Vec<Vec<f64>>; 2],
    level: f64,
    most: [[Vec<Vec<f64>>; 2]; 2],
}

impl PieceLimits {
    /// The speed level whose ceilings hold up to the speed `v`.
    fn level_of(&self, v: f64) -> usize {
        ((v / self.level).ceil() as usize).clamp(1, LEVELS) - 1
    }

    /// The lowest squared speed of a run's curve between the distances `from` and `to` (by the places around them).
    fn lowest_between(&self, backward: bool, from: f64, to: f64) -> f64 {
        let table = &self.lowest_ahead[backward as usize];
        let last = table[0].len() - 1;
        let (i, j) = (((from / self.spacing) as usize).min(last), ((to / self.spacing).ceil() as usize).min(last));
        let (i, j) = (i.min(j), i.max(j));
        let size = (usize::BITS - 1 - (j - i + 1).leading_zeros()) as usize;
        table[size][i].min(table[size][j + 1 - (1 << size)])
    }
}

/// The lowest of `values` over 2^i entries from each entry on, for i from 0 while 2^i fits: a range's lowest is then the
/// lower of two overlapping such spans.
fn lowest_ahead(values: &[f64]) -> Vec<Vec<f64>> {
    let mut table = vec![values.to_vec()];
    while 2 << (table.len() - 1) <= values.len() {
        let (half, last) = (1 << (table.len() - 1), &table[table.len() - 1]);
        let next = (0..=values.len() - 2 * half).map(|k| last[k].min(last[k + half])).collect();
        table.push(next);
    }
    table
}

/// The speed curve a piece follows from below, as squared speeds x at its places: no place's top exceeded, the
/// acceleration along it (x' / 2) within `CURVE_ACCELERATION` of the places' ceilings (`Place::most`, at speeds up to
/// the curve's), and its jerk (v times the change of that acceleration per distance, so |x''| <= 2 j / v) within
/// `CURVE_JERK` of the ramps' jerk. It starts at the places' tops (infinite ones at the highest finite) and is only ever
/// lowered, in sweeps forwards and backwards: a rise or a fall too steep lowers its upper end; a peak too sharp is
/// lowered onto the bound; at a dip too sharp the higher of its two sides is lowered, then both. So it never falls below
/// its lowest place's top, which is also why the end speeds a piece is planned with (no more than that,
/// `timed_by_motors`) are always below it.
///
/// shortcut: the sweeps stop once one changes the curve by no more than 1e-7 of its highest, or after `CURVE_SWEEPS`
/// (a sharp dip converges slowly, by about 1 % a sweep); what is left of a bend too sharp is far within the jerk the runs
/// keep beside the curve's. Solve the bends directly (a parabola in x around each dip) if a machine's dips need more.
fn smoothed(places: &[Place], spacing: f64, jerk: f64, highest: f64) -> Vec<f64> {
    let n = places.len();
    let mut x: Vec<f64> = places.iter().map(|p| p.top.min(highest).powi(2)).collect();
    let most = |k: usize, x: f64, sign: f64| CURVE_ACCELERATION * places[k].most(x.sqrt(), sign);
    let bend = 2.0 * CURVE_JERK * jerk * spacing * spacing;
    let bend_at = |x: &mut Vec<f64>, k: usize| {
        let (c, d2) = (bend / x[k].sqrt(), x[k - 1] + x[k + 1] - 2.0 * x[k]);
        if d2 < -c {
            x[k] = 0.5 * (x[k - 1] + x[k + 1] + c);
        } else if d2 > c {
            let (high, low) = if x[k - 1] > x[k + 1] { (k - 1, k + 1) } else { (k + 1, k - 1) };
            let mut excess = d2 - c;
            let first = excess.min(x[high] - x[low]);
            (x[high], excess) = (x[high] - first, excess - first);
            x[k - 1] -= 0.5 * excess;
            x[k + 1] -= 0.5 * excess;
        }
    };
    for _ in 0..CURVE_SWEEPS {
        let before = x.clone();
        for k in 0..n - 1 {
            let rise = most(k, x[k + 1], 1.0).min(most(k + 1, x[k + 1], 1.0));
            x[k + 1] = x[k + 1].min(x[k] + 2.0 * spacing * rise);
            if k > 0 {
                bend_at(&mut x, k);
            }
        }
        for k in (0..n - 1).rev() {
            let fall = most(k, x[k], -1.0).min(most(k + 1, x[k], -1.0));
            x[k] = x[k].min(x[k + 1] + 2.0 * spacing * fall);
            if k + 2 < n {
                bend_at(&mut x, k + 1);
            }
        }
        let top = x.iter().fold(0.0, |a: f64, b| a.max(*b));
        if x.iter().zip(&before).all(|(now, then)| then - now <= 1e-7 * top) {
            break;
        }
    }
    x
}

/// The most acceleration of a run (`sign` 1 for a run from the start: forwards; -1 from the end: the deceleration
/// forwards) or the most it slows down (the other sign) at each place, by the distance from where the run starts
/// (`backward` from the end), at speeds up to the run's `curve` there and up to `cap`, lowered ahead of where it falls by
/// the jerk times the least time between two places (their spacing over that speed), so that the acceleration can
/// follow it. Between places it is taken as straight (`ceiling_at`), within what the plan keeps back (`KEPT_BACK`).
fn ceiling(places: &[Place], curve: &[f64], spacing: f64, jerk: f64, sign: f64, backward: bool, cap: f64) -> Vec<f64> {
    let n = places.len();
    let at = |k: usize| if backward { n - 1 - k } else { k };
    let speed: Vec<f64> = curve.iter().map(|x| x.sqrt().min(cap)).collect();
    let mut most: Vec<f64> = (0..n).map(|k| places[at(k)].most(speed[k], sign)).collect();
    for k in (0..n - 1).rev() {
        most[k] = most[k].min(most[k + 1] + jerk * spacing / speed[k].max(speed[k + 1]));
    }
    most
}

/// The places of a piece and what bounds the time-optimal plan at each (see `plan_time_optimal`). Refused where the
/// torque that holds the tool (at a motor or a gearbox), the belt's holding tension or a guide's load at rest leaves no
/// room, or where the piece turns no motor.
fn piece_limits(
    machine: &dyn Machine,
    curves: &Arc<[MotorCurve]>,
    limits: &PlanLimits,
    geo: &Geo,
) -> Result<PieceLimits, CommandError> {
    let (reserve, [lowest, highest, share]) = (limits.reserve, limits.tension);
    let gearbox = |m: usize| limits.gearbox.get(m).copied().flatten();
    let (axes, motors, length) = (machine.axes(), machine.motors().len(), geo.length());
    // The tightest curvature, so that on a tight curve the places are no more than `PLACE_TURN` of its turning apart.
    let tightest = match geo {
        Geo::Line { .. } | Geo::Slerp { .. } => 0.0,
        Geo::Arc { radius, .. } | Geo::Arc3 { radius, .. } => 1.0 / radius,
        Geo::Clothoid { k0, k1, .. } => k0.abs().max(k1.abs()),
        Geo::Spline { tightest, .. } => *tightest,
        Geo::Carried { arc, length, .. } => {
            let Geo::Arc3 { radius, length: on_arc, .. } = **arc else { unreachable!("a carried piece is an arc") };
            (on_arc / length).powi(2) / radius
        }
    };
    let count =
        ((length / PLACE_SPACING).max(tightest * length / PLACE_TURN).ceil() as usize).clamp(PLACES_LEAST, PLACES_MOST);
    let spacing = length / count as f64;
    // The pose and its derivatives by distance; the third from the second by central differences within the piece.
    let h = spacing / 4.0;
    let derivatives: Vec<[Vec<f64>; 4]> = (0..=count)
        .map(|k| {
            let s = if k == count { length } else { k as f64 * spacing };
            let [pose, d1, d2] = geo.at(s);
            let (behind, ahead) = ((s - h).max(0.0), (s + h).min(length));
            let d3 = geo.at(ahead)[2].iter().zip(&geo.at(behind)[2]).map(|(a, b)| (a - b) / (ahead - behind)).collect();
            [pose, d1, d2, d3]
        })
        .collect();
    // The ramps' jerk along the path: on a line all of the axes' jerk limits; on a curve 0.4 of it, as `curved` shares
    // it out, and the bends get the rest.
    let along = derivatives
        .iter()
        .flat_map(|[_, d1, ..]| axes.iter().zip(d1).map(|(a, d)| a.max_jerk / d.abs()))
        .fold(f64::INFINITY, f64::min);
    let jerk = if matches!(geo, Geo::Line { .. } | Geo::Slerp { .. }) { 1.0 } else { 0.4 } * along;
    // What each motor gives at a standstill, either way: the bounds of its row, and what holds the tool.
    let torque: Vec<f64> = curves.iter().map(|c| c.motoring[0].min(c.braking[0])).collect();
    let zero = vec![0.0; axes.len()];
    let mut places = Vec::new();
    for [pose, d1, d2, d3] in &derivatives {
        let torques = |vel: &[f64], acc: &[f64]| {
            let mut out = vec![0.0; motors];
            machine.feedforward_torque_moving(pose, vel, acc, &mut out);
            out
        };
        // At the speed v and the acceleration a along the path the tool moves at v p' and accelerates at a p' + v^2 p''.
        let (hold, per_acc, per_speed) = (torques(&zero, &zero), torques(&zero, d1), torques(d1, d2));
        // The motors' angles a distance h along the path, to second order: pose + h p' + h^2 / 2 p''.
        let angles = |h: f64| {
            let mut out = vec![0.0; motors];
            let moved: Vec<f64> =
                pose.iter().zip(d1).zip(d2).map(|((p, d1), d2)| p + h * d1 + 0.5 * h * h * d2).collect();
            machine.inverse(&moved, &mut out);
            out
        };
        let (ahead, behind) = (angles(TURN_STEP), angles(-TURN_STEP));
        // Near a singularity (or where the inverse finds no pose) a motor turns without bound per distance along the
        // path, and the plan would crawl or, with the rows not finite, run unbounded: refused.
        let bounded = |v: &[f64]| v.iter().all(|x| x.is_finite());
        if ![&hold, &per_acc, &per_speed, &ahead, &behind].iter().all(|v| bounded(v))
            || ahead.iter().zip(&behind).any(|(a, b)| (a - b).abs() > 2.0 * TURN_STEP * SINGULAR_TURN)
        {
            return Err(CommandError { key: "error-path-singular" });
        }
        if hold.iter().zip(&torque).any(|(t, most)| t.abs() >= *most) {
            return Err(CommandError { key: "error-path-torque" });
        }
        let mut rows: Vec<[f64; 5]> = (0..motors)
            .map(|m| [per_acc[m] - hold[m], per_speed[m] - hold[m], hold[m], -torque[m], torque[m]])
            .collect();
        if lowest.is_finite() || highest.is_finite() {
            let tensions = |acc: &[f64]| {
                let mut out = Vec::new();
                machine.rigid_tensions(pose, acc, &mut out);
                out
            };
            let (still, per_acc, per_speed) = (tensions(&zero), tensions(d1), tensions(d2));
            for k in 0..still.len() {
                let room = (1.0 - KEPT_BACK) * share * (highest - still[k]).min(still[k] - lowest);
                if !(room > 0.0) {
                    return Err(CommandError { key: "error-path-tension" });
                }
                // The same room on both sides: the belt's vibration after a change of tension swings as far the
                // other way (see `plan_time_optimal`).
                rows.push([per_acc[k] - still[k], per_speed[k] - still[k], 0.0, -room, room]);
            }
        }
        if (0..motors).any(|m| gearbox(m).is_some()) {
            // The output torque ratio (T - I alpha), alpha = theta' a + theta'' v^2 the input's angular acceleration: the
            // torque the load takes, whatever the gearbox loses. What it loses the motor gives on top: the rigid model's
            // motor torque is I alpha + (T - I alpha), and with the efficiency eta the motor gives I alpha + (T - I alpha)
            // / eta while it drives the load, I alpha + (T - I alpha) eta while the load drives it. The motor's row takes
            // the first, which is the larger either way: at most (1 / eta - eta) of the load's torque more than it needs
            // while braking.
            let (far, here, back) = (angles(BEND_STEP), angles(0.0), angles(-BEND_STEP));
            for m in (0..motors).filter_map(|m| Some((m, gearbox(m)?))).collect::<Vec<_>>() {
                let (m, [ratio, inertia, rated, efficiency]) = m;
                let turn = (ahead[m] - behind[m]) / (2.0 * TURN_STEP);
                let bend = (far[m] - 2.0 * here[m] + back[m]) / (BEND_STEP * BEND_STEP);
                let [c, d, e, ..] = rows[m];
                let (c_load, d_load) = (c - inertia * turn, d - inertia * bend);
                (rows[m][0], rows[m][1], rows[m][2]) =
                    (inertia * turn + c_load / efficiency, inertia * bend + d_load / efficiency, e / efficiency);
                if (e / efficiency).abs() >= torque[m] {
                    return Err(CommandError { key: "error-path-torque" });
                }
                if rated.is_finite() {
                    let room = (1.0 - KEPT_BACK) * reserve * rated;
                    if (ratio * e).abs() >= room {
                        return Err(CommandError { key: "error-path-torque" });
                    }
                    rows.push([ratio * c_load, ratio * d_load, ratio * e, -room, room]);
                }
            }
        }
        if limits.guide.is_finite() {
            // Each guide's load is its load at rest plus the size of a part linear in the tool's acceleration, within
            // `reserve` of the rating, as the motors' torques are within `reserve` of theirs: the rest is for what the
            // rigid model leaves out (the frame's and the belt's vibration, the friction the rows do not see).
            let loads = |acc: &[f64]| {
                let mut out = Vec::new();
                machine.rigid_guide_loads(pose, acc, &mut out);
                out
            };
            let (still, per_acc, per_speed) = (loads(&zero), loads(d1), loads(d2));
            for k in 0..still.len() {
                let room = (1.0 - KEPT_BACK) * reserve * limits.guide - still[k][0];
                if !(room > 0.0) {
                    return Err(CommandError { key: "error-path-guide" });
                }
                rows.push([per_acc[k][1] - still[k][1], per_speed[k][1] - still[k][1], 0.0, -room, room]);
            }
        }
        let bends: Vec<[f64; 3]> = axes
            .iter()
            .enumerate()
            .map(|(i, a)| [3.0 * d2[i].abs(), d3[i].abs(), a.max_jerk - jerk * d1[i].abs()])
            .collect();
        // The highest speed squared: each motor at its maximum speed, each row and bend without acceleration; the rows
        // of fixed bounds and the bends directly, then the motors' rows against their curves, which do not widen with
        // the speed, by halving.
        let turn: Vec<f64> = (0..motors).map(|m| (ahead[m] - behind[m]) / (2.0 * TURN_STEP)).collect();
        let mut x = (0..motors).map(|m| (curves[m].max_speed() / turn[m].abs()).powi(2)).fold(f64::INFINITY, f64::min);
        for [_, d, e, low, high] in rows[motors..].iter().filter(|r| r[1] != 0.0) {
            x = x.min(if *d > 0.0 { (high - e) / d } else { (e - low) / -d });
        }
        for [_, p3, room] in bends.iter().filter(|b| b[1] > 0.0) {
            x = x.min((room / p3).max(0.0).powf(2.0 / 3.0));
        }
        let mut guide = Vec::new();
        machine.rigid_friction(pose, d1, &mut guide);
        // Behind a lossy gearbox the motor gives the friction's torque over the efficiency (as the load's, above).
        for (m, g) in guide.iter_mut().enumerate() {
            if let Some([.., efficiency]) = gearbox(m) {
                *g = g.map(|t| t / efficiency);
            }
        }
        let mut place = Place { rows, turn, curves: curves.clone(), guide, bends, top: x.sqrt() };
        // A motor that turns here and has to hold the tool against more than its curve less its friction leaves no room.
        if (0..motors).any(|m| {
            let (low, high) = place.bounds(m, 0.0);
            place.turn[m] != 0.0 && !(low..=high).contains(&place.rows[m][2])
        }) {
            return Err(CommandError { key: "error-path-torque" });
        }
        let holds = |place: &Place, v: f64| {
            (0..motors).all(|m| {
                let ([_, d, e, ..], (low, high)) = (place.rows[m], place.bounds(m, v));
                (low..=high).contains(&(d * v * v + e))
            })
        };
        if place.top.is_finite() && !holds(&place, place.top) {
            let (mut slow, mut fast) = (0.0, place.top);
            for _ in 0..60 {
                let v = 0.5 * (slow + fast);
                if holds(&place, v) { slow = v } else { fast = v }
            }
            place.top = slow;
        }
        places.push(place);
    }
    piece_limits_of(places, spacing, jerk)
}

/// A piece's `PieceLimits` from its places, `spacing` apart, and the jerk along the path its ramps may use.
fn piece_limits_of(places: Vec<Place>, spacing: f64, jerk: f64) -> Result<PieceLimits, CommandError> {
    let lowest = places.iter().map(|p| p.top).fold(f64::INFINITY, f64::min);
    // A piece that turns no motor has no speed limit: refused as the per-piece limits refuse it.
    if !(lowest.is_finite() && lowest > 0.0 && jerk.is_finite() && jerk > 0.0) {
        return Err(CommandError { key: "error-path-torque" });
    }
    let highest = places.iter().map(|p| p.top).filter(|t| t.is_finite()).fold(lowest, f64::max);
    let forward = smoothed(&places, spacing, jerk, highest);
    let backward: Vec<f64> = forward.iter().rev().copied().collect();
    let level = forward.iter().fold(0.0, |a: f64, b| a.max(*b)).sqrt() / LEVELS as f64;
    let tables = |curve: &[f64], sign: f64, backward: bool| -> Vec<Vec<f64>> {
        (1..=LEVELS).map(|l| ceiling(&places, curve, spacing, jerk, sign, backward, l as f64 * level)).collect()
    };
    let most = [
        [tables(&forward, 1.0, false), tables(&forward, -1.0, false)],
        [tables(&backward, -1.0, true), tables(&backward, 1.0, true)],
    ];
    let lowest_ahead = [lowest_ahead(&forward), lowest_ahead(&backward)];
    Ok(PieceLimits { places, spacing, jerk, lowest, curve: [forward, backward], lowest_ahead, level, most })
}

/// A table of `ceiling` at the distance `s`, between its places by straight lines.
fn ceiling_at(most: &[f64], spacing: f64, s: f64) -> f64 {
    let x = (s / spacing).clamp(0.0, (most.len() - 1) as f64);
    let k = (x as usize).min(most.len() - 2);
    most[k] + (x - k as f64) * (most[k + 1] - most[k])
}

/// The speed curve of a run (`PieceLimits::curve`) at the distance `s`: its speed, its acceleration v dv/ds (straight
/// between the middles of the places' gaps) and its jerk, v times the change of that acceleration per distance.
fn along_curve(curve: &[f64], spacing: f64, s: f64) -> [f64; 3] {
    let n = curve.len();
    let x = (s / spacing).clamp(0.0, (n - 1) as f64);
    let k = (x as usize).min(n - 2);
    let speed = (curve[k] + (x - k as f64) * (curve[k + 1] - curve[k])).max(0.0).sqrt();
    let acc = |i: usize| (curve[i + 1] - curve[i]) / (2.0 * spacing);
    let m = (x - 0.5).clamp(0.0, (n - 2) as f64);
    let i = (m as usize).min(n - 3);
    let (a0, a1) = (acc(i), acc(i + 1));
    [speed, a0 + (m - i as f64) * (a1 - a0), speed * (a1 - a0) / spacing]
}

/// A jerk-limited run along a piece from the speed `from`, with no acceleration, at the distance `start` from where it
/// starts (the piece's start, or its end with `backward`, by `PieceLimits::curve` and `most` [1]), until the speed
/// reaches `top` with no acceleration, no further than `end`. Its knots (time, distance, speed, acceleration) with the
/// jerk constant between them, at steps of `step`. `None` where it does not reach top by `end`, or stops. With no top
/// (infinite) the run follows the curve as far as `end` and its knots end there; `None` where it gets above the curve
/// (by more than `CURVE_OVERSHOOT` of it): it started faster than the curve lets it slow down (`curve_join`).
///
/// Each step the acceleration rises by as much as the jerk allows, but no more than three things allow: the ceiling of
/// acceleration ahead; that it can still fall to zero at the jerk limit by the time the speed reaches top; and that it
/// can still come down onto the speed curve the run follows, at the jerk left beside the curve's own (at least
/// `1 - CURVE_JERK` of it), by the time the speed reaches the curve. The last is left out while the acceleration can
/// fall to zero at the jerk limit with the speed below the curve all the way, and the curve keeps room there for its own
/// slowing down. It falls at most to the ceiling of slowing down. So the run rises along the ceiling, comes onto the
/// speed curve and follows it, up and down, and on to top; and it is the same run, whatever the top, until it brakes
/// for it, so that a higher top that fits is the quicker.
fn follow(
    limits: &PieceLimits,
    from: f64,
    top: f64,
    [start, end]: [f64; 2],
    backward: bool,
    step: f64,
) -> Option<Vec<[f64; 4]>> {
    let (curve, [up, down]) = (&limits.curve[backward as usize], &limits.most[backward as usize]);
    let (jerk, spacing) = (limits.jerk, limits.spacing);
    let rise = jerk * step;
    let mut knots = vec![[0.0, start, from, 0.0]];
    while knots.len() < RUN_STEPS {
        let [t, s, v, a] = knots[knots.len() - 1];
        if v >= top {
            return Some(knots);
        }
        // The largest change du of the acceleration in this step after which it can still fall to zero at the jerk
        // limit by the time the speed reaches top: (a + du)^2 <= 2 jerk (top - v - a step - du step / 2).
        let b = 2.0 * a + rise;
        let disc = b * b - 4.0 * (a * a + 2.0 * jerk * (v + a * step - top));
        let brake = if disc >= 0.0 { 0.5 * (disc.sqrt() - b) } else { f64::NEG_INFINITY };
        if brake <= -a && a > 0.0 {
            // Within this step the acceleration has to reach zero: at the constant jerk -a / h that takes it there
            // over the time h in which the speed reaches top, h = 2 (top - v) / a. On the brake curve that is the
            // jerk limit; below it, less.
            let h = (2.0 * (top - v) / a).max(0.0);
            let s = s + v * h + a * h * h / 3.0;
            knots.push([t + h, s, top, 0.0]);
            return (s <= end).then_some(knots);
        }
        let ahead = s + v * step + (a / 2.0 + rise / 6.0) * step * step;
        // Onto the curve: with y the acceleration above the curve's after this step and g the gap to the curve's speed
        // then, at the jerk k beside the curve's own the gap closes as y falls to zero where y^2 <= 2 k g, and
        // g = gap - y step / 2, gap = curve - v - (a + curve's a) step / 2 at an acceleration rising evenly to it.
        let [speed, acc, own] = along_curve(curve, spacing, ahead);
        let k = (jerk - own.abs()).max((1.0 - CURVE_JERK) * jerk);
        let gap = speed - v - 0.5 * (a + acc) * step;
        let y =
            if gap >= 0.0 { 0.5 * ((k * k * step * step + 8.0 * k * gap).sqrt() - k * step) } else { 2.0 * gap / step };
        // The ceilings for the most speed the run can reach if its acceleration falls to zero from after this step.
        let high = (a + rise).max(0.0);
        let level = limits.level_of(v + (a + rise / 2.0) * step + high * high / (2.0 * jerk));
        let floor = (-ceiling_at(&down[level], spacing, ahead) - a).clamp(-rise, rise);
        let du = brake.min(ceiling_at(&up[level], spacing, ahead) - a).min(rise).max(floor);
        // Below the curve all the way if the acceleration falls to zero from after this step: the speed rises to
        // `reach` over the time a / jerk; there the curve's own slowing down has to have room (y >= 0 at no gap).
        let after = a + du;
        let reach = v + (a + du / 2.0) * step + after.max(0.0).powi(2) / (2.0 * jerk);
        let there = ahead + reach * after.max(0.0) / jerk;
        let [end_speed, end_acc, end_own] = along_curve(curve, spacing, there);
        let room = (jerk - end_own.abs()).max((1.0 - CURVE_JERK) * jerk);
        // (To the rounding of a run that ends on the curve's speed.)
        let reach = reach * (1.0 - 1e-9);
        let clear = reach * reach <= limits.lowest_between(backward, ahead, there)
            && end_acc.min(0.0).powi(2) <= 2.0 * room * (end_speed - reach).max(0.0);
        let du = if clear { du } else { du.min(y + acc - a).max(floor) };
        let next = [t + step, s + v * step + (a / 2.0 + du / 6.0) * step * step, v + (a + du / 2.0) * step, a + du];
        if top.is_infinite() {
            if next[1] > end {
                return Some(knots);
            }
            if next[2] > (1.0 + CURVE_OVERSHOOT) * along_curve(curve, spacing, next[1])[0] {
                return None;
            }
        }
        // Past the end, or held at rest by a ceiling of zero.
        if next[1] > end || (next[2] <= 0.0 && next[3] <= 0.0) {
            return None;
        }
        knots.push(next);
    }
    None
}

/// The timing of one piece of the length `length` from the speed `v0` at its start to `v1` at its end, with `top` the
/// highest speed on it: steady for `hold` at each end that is not at rest, a run from v0 that follows the speed curve
/// up to top, cruise at top, and a run down to v1 (planned from the end backwards the same way). As knots (time from
/// the start, distance, speed, acceleration) with the jerk constant between them. `None` where that does not fit into
/// the piece or the cruise passes a place faster than it holds; `slack` is how much more than the piece's length it
/// may take, for rounding. Where the runs would meet while the speed curve still rises or falls, `curve_join` joins them
/// on the curve instead, with no cruise.
fn piece_timing(
    limits: &PieceLimits,
    length: f64,
    [v0, v1, top]: [f64; 3],
    hold: f64,
    step: f64,
    slack: f64,
) -> Option<Vec<[f64; 4]>> {
    let (start, end) = (hold * v0, length + slack - hold * v1);
    let rise = follow(limits, v0, top, [start, end], false, step)?;
    let reached = rise[rise.len() - 1][1];
    let fall = follow(limits, v1, top, [hold * v1, length + slack - reached], true, step)?;
    let (braking, took) = ((length - fall[fall.len() - 1][1]).max(reached), fall[fall.len() - 1][0]);
    // The cruise between the climbs holds at top at every place it passes.
    let (first, last) = (
        (reached / limits.spacing).floor() as usize,
        ((braking / limits.spacing).ceil() as usize).min(limits.places.len() - 1),
    );
    // (To the rounding of the places' tops, read by differences: a top on the curve is on the places' tops.)
    if limits.places[first.min(last)..=last].iter().any(|p| p.top < top * (1.0 - 1e-8)) {
        return None;
    }
    let mut knots = vec![[0.0, 0.0, v0, 0.0]];
    if v0 > 0.0 {
        knots.push([hold, start, v0, 0.0]);
    }
    let t0 = knots[knots.len() - 1][0];
    knots.extend(rise[1..].iter().map(|[t, s, v, a]| [t0 + t, *s, *v, *a]));
    let [t, s, ..] = knots[knots.len() - 1];
    if braking > s {
        knots.push([t + (braking - s) / top, braking, top, 0.0]);
    }
    let t1 = knots[knots.len() - 1][0];
    knots.extend(fall.iter().rev().skip(1).map(|[t, s, v, a]| [t1 + took - t, length - s, *v, -a]));
    if v1 > 0.0 {
        let t = knots[knots.len() - 1][0];
        knots.push([t + hold, length, v1, 0.0]);
    }
    knots.last_mut().unwrap()[1] = length;
    Some(knots)
}

/// The two runs of a piece (`follow`, from `v0` at its start and from `v1` at its end, each after its steady cycles),
/// with no top: each follows the speed curve as far as the piece goes, and the plan takes the run from the start up to
/// where it meets the run from the end, and that one from there. Where they meet both are on the speed curve, at its
/// speed and its acceleration, so the plan switches from one to the other directly, without the cruise at no
/// acceleration that `piece_timing` joins them on: the plan when the runs meet while the curve rises or falls, and the
/// only one where the piece has to slow down below both its end speeds somewhere. Knots as `piece_timing`'s; `None`
/// where a run starts faster than the curve lets it slow down, or the runs meet off the curve (at a peak of speed,
/// where `piece_timing` joins them).
fn curve_join(limits: &PieceLimits, length: f64, [v0, v1]: [f64; 2], hold: f64, step: f64) -> Option<Vec<[f64; 4]>> {
    let rise = follow(limits, v0, f64::INFINITY, [hold * v0, length - hold * v1], false, step)?;
    let fall = follow(limits, v1, f64::INFINITY, [hold * v1, length - hold * v0], true, step)?;
    // A fall of its start knot alone has no span to meet the rise on: the cruise join (`piece_timing`) times the piece.
    if fall.len() < 2 {
        return None;
    }
    // The fall's state at the distance `s` from the start, by straight lines between its knots: its time, speed and
    // acceleration forwards (its own acceleration the other way round).
    let fall_at = |s: f64| {
        let back = length - s;
        let k = fall.partition_point(|knot| knot[1] <= back).clamp(1, fall.len() - 1);
        let ([t0, s0, v0, a0], [t1, s1, v1, a1]) = (fall[k - 1], fall[k]);
        let w = if s1 > s0 { ((back - s0) / (s1 - s0)).clamp(0.0, 1.0) } else { 0.0 };
        [t0 + w * (t1 - t0), v0 + w * (v1 - v0), -(a0 + w * (a1 - a0))]
    };
    // The first knot of the rise at which it is as fast as the fall: there they meet, and both are on the curve if their
    // accelerations agree to the jerk of a few steps.
    let meet = rise.iter().position(|&[_, s, v, _]| s >= length - fall[fall.len() - 1][1] && v >= fall_at(s)[1])?;
    let [t_meet, s_meet, v_meet, a_meet] = rise[meet];
    let [took, v_fall, a_fall] = fall_at(s_meet);
    if (v_meet - v_fall).abs() > KEPT_BACK * v_meet.max(1e-9) || (a_meet - a_fall).abs() > 4.0 * limits.jerk * step {
        return None;
    }
    // The switch keeps the jerk limit over the profile cycle up to it: from the rise's acceleration a cycle before the meet
    // to the fall's at its first knot after it. (A rise still ramping at the full jerk met a fall a cycle's jerk further
    // on, 4 steps' worth, and the plan stepped its acceleration on the same way: 1.23 times the jerk over that cycle.)
    let after = fall.iter().rev().find(|knot| knot[1] < length - s_meet);
    let [t_back, _, _, a_back] = rise[meet.saturating_sub(STEPS_PER_CYCLE as usize)];
    if after.is_some_and(|&[t, _, _, a]| (a + a_back).abs() > limits.jerk * (t_meet - t_back + took - t) * (1.0 + 1e-9))
    {
        return None;
    }
    let mut knots = vec![[0.0, 0.0, v0, 0.0]];
    if v0 > 0.0 {
        knots.push([hold, hold * v0, v0, 0.0]);
    }
    let t0 = knots[knots.len() - 1][0];
    knots.extend(rise[1..=meet].iter().map(|[t, s, v, a]| [t0 + t, *s, *v, *a]));
    let t1 = t0 + t_meet + took;
    knots.extend(
        fall.iter().rev().filter(|knot| knot[1] < length - s_meet).map(|[t, s, v, a]| [t1 - t, length - s, *v, -a]),
    );
    if v1 > 0.0 {
        let t = knots[knots.len() - 1][0];
        knots.push([t + hold, length, v1, 0.0]);
    }
    knots.last_mut().unwrap()[1] = length;
    Some(knots)
}

/// The timing of `plan_time_optimal`: the look-ahead of `timed` over the junction speeds (`junction_speeds`), with each
/// piece timed point by point under its `PieceLimits`: by `piece_timing` at the highest top that fits, or by
/// `curve_join`, whichever is the quicker.
fn timed_by_motors(
    axes: &[AxisInfo],
    geo: &[Geo],
    limits: &[PieceLimits],
    cycle: f64,
) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
    let n = geo.len();
    let speed = junction_speeds(axes, geo, limits, cycle, true);
    let mut knots = Vec::new();
    for i in 0..n {
        knots.push(piece_knots(&limits[i], geo[i].length(), [speed[i], speed[i + 1]], cycle, true)?);
    }

    let duration = |i: usize| knots[i][knots[i].len() - 1][0];
    let total: f64 = (0..n).map(duration).sum();
    // A crawl near a singularity, or where the motors' limits slow the path to one, would ask for a pose per cycle
    // without bound and fill the memory: refused beyond `PLAN_LONGEST`, and beyond `PLAN_POINTS_MOST` poses where a
    // cycle below 0.1 ms makes that the tighter bound.
    if !(total <= PLAN_LONGEST && total / cycle <= PLAN_POINTS_MOST) {
        return Err(CommandError { key: "error-path-singular" });
    }
    let mut points = Vec::new();
    for k in 0..=(total / cycle - 1e-9).ceil() as usize {
        let (mut i, mut t) = (0, k as f64 * cycle);
        while i + 1 < n && t >= duration(i) {
            t -= duration(i);
            i += 1;
        }
        let piece = &knots[i];
        let next = piece.partition_point(|knot| knot[0] <= t);
        let s = if next >= piece.len() {
            piece[piece.len() - 1][1]
        } else {
            let ([t0, s, v, a], [t1, .., a1]) = (piece[next - 1], piece[next]);
            let (h, tau) = (t1 - t0, t - t0);
            let jerk = if h > 0.0 { (a1 - a) / h } else { 0.0 };
            s + v * tau + a * tau * tau / 2.0 + jerk * tau.powi(3) / 6.0
        };
        points.push((k as f64 * cycle, within(axes, geo[i].at(s.clamp(0.0, geo[i].length()))[0].clone())));
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axes() -> [AxisInfo; 2] {
        let axis = |max_vel: f64, max_acc: f64| AxisInfo {
            id: "",
            key: "",
            unit: "m",
            min: 0.0,
            max: 1.0,
            max_vel,
            max_acc,
            max_jerk: 100.0,
        };
        [axis(0.8, 8.0), axis(0.4, 6.0)]
    }

    fn axes3() -> [AxisInfo; 3] {
        let axis = |min: f64, max: f64| AxisInfo {
            id: "",
            key: "",
            unit: "m",
            min,
            max,
            max_vel: 3.0,
            max_acc: 50.0,
            max_jerk: 2500.0,
        };
        [axis(-0.6, 0.6), axis(-0.6, 0.6), axis(-1.1, -0.6)]
    }

    /// On three axes: a line and a spline point have three positions, an arc its end and its centre with three each.
    /// A quarter circle of 0.1 m in the x-z plane, from +x to +z around its centre, is counter-clockwise seen from the
    /// viewer (-y, x to the right and z up, as on a two-axis machine with x and z): 0.1 pi / 2 long, through
    /// (0.1 cos 45, 0, 0.1 sin 45) about the centre halfway; clockwise it goes the other way round, three quarters.
    /// In the x-y plane, seen from +z, from +x to +y is counter-clockwise too. An arc whose start, end and centre lie on
    /// one line (a half or a full turn) leaves its plane open and is refused.
    #[test]
    fn arcs_on_three_axes_lie_in_the_plane_of_their_points() {
        let pieces = parse(
            "line,0.1,0.0,-0.8
arc,0.0,0.0,-0.7,0.0,0.0,-0.8,ccw
arc,-0.1,0.0,-0.8,0.0,0.0,-0.8,ccw",
        )
        .unwrap();
        assert_eq!(pieces[1], Piece::Arc { to: vec![0.0, 0.0, -0.7], centre: vec![0.0, 0.0, -0.8], clockwise: false });
        let geo = given(&[0.1, 0.0, -0.9], &pieces, ARC_SLACK).unwrap().0;
        assert_eq!(geo.len(), 3);
        let quarter = std::f64::consts::FRAC_PI_2 * 0.1;
        assert!((geo[1].length() - quarter).abs() < 1e-12 && (geo[2].length() - quarter).abs() < 1e-12);
        let half = 0.1 * std::f64::consts::FRAC_1_SQRT_2;
        let [mid, dir, curve] = geo[1].at(0.5 * quarter);
        assert!(
            distance(&mid, &[half, 0.0, -0.8 + half]) < 1e-12
                && distance(&dir, &[-std::f64::consts::FRAC_1_SQRT_2, 0.0, std::f64::consts::FRAC_1_SQRT_2]) < 1e-12
        );
        // Its curvature points at the centre, 1 / r long.
        assert!(
            distance(&curve, &[-10.0 * std::f64::consts::FRAC_1_SQRT_2, 0.0, -10.0 * std::f64::consts::FRAC_1_SQRT_2])
                < 1e-9
        );
        let cw = given(&[0.1, 0.0, -0.8], &parse("arc,0.0,0.0,-0.7,0.0,0.0,-0.8,cw").unwrap(), ARC_SLACK).unwrap().0;
        assert!((cw[0].length() - 3.0 * quarter).abs() < 1e-12 && cw[0].at(cw[0].length() / 3.0)[0][2] < -0.8);
        let flat = given(&[0.1, 0.0, -0.8], &parse("arc,0.0,0.1,-0.8,0.0,0.0,-0.8,ccw").unwrap(), ARC_SLACK).unwrap().0;
        assert!((flat[0].length() - quarter).abs() < 1e-12 && flat[0].at(0.5 * quarter)[0][1] > 0.0);
        assert_eq!(
            given(&[0.1, 0.0, -0.8], &parse("arc,-0.1,0.0,-0.8,0.0,0.0,-0.8,ccw").unwrap(), ARC_SLACK)
                .err()
                .unwrap()
                .key,
            "error-path"
        );
        // Planned within the axis limits, the path ends where it was told.
        let points = plan(&axes3(), &[0.1, 0.0, -0.9], &pieces, 1e-3, 0.0, ARC_SLACK).unwrap();
        assert!(distance(&points[points.len() - 1].1, &[-0.1, 0.0, -0.8]) < 1e-9);
    }

    /// A spline on three axes passes through its via points, leaves the line before it in that line's direction, and its
    /// plan keeps every axis within its speed, acceleration and jerk (to the 0.1 % of differencing the planned poses).
    #[test]
    fn a_spline_on_three_axes_passes_through_its_points_within_the_limits() {
        let pieces = parse(
            "line,0.0,0.0,-0.8
spline,0.1,0.1,-0.75
spline,0.2,0.0,-0.8
line,0.2,0.0,-0.9",
        )
        .unwrap();
        let geo = given(&[0.0, 0.0, -0.9], &pieces, ARC_SLACK).unwrap().0;
        let ends: Vec<Vec<f64>> = geo.iter().map(|g| g.at(g.length())[0].clone()).collect();
        assert!(distance(&ends[1], &[0.1, 0.1, -0.75]) < 1e-9 && distance(&ends[2], &[0.2, 0.0, -0.8]) < 1e-9);
        // Out of the vertical line upwards, into the last line downwards: no corner at either junction.
        assert!(
            distance(&geo[1].at(0.0)[1], &[0.0, 0.0, 1.0]) < 1e-9
                && distance(&geo[2].at(geo[2].length())[1], &[0.0, 0.0, -1.0]) < 1e-9
        );
        let (axes, cycle) = (axes3(), 1e-3);
        let points = plan(&axes, &[0.0, 0.0, -0.9], &pieces, cycle, 0.0, ARC_SLACK).unwrap();
        for w in points.windows(4) {
            for (i, axis) in axes.iter().enumerate() {
                let p = [w[0].1[i], w[1].1[i], w[2].1[i], w[3].1[i]];
                assert!(
                    ((p[1] - p[0]) / cycle).abs() <= 1.001 * axis.max_vel
                        && ((p[2] - 2.0 * p[1] + p[0]) / cycle.powi(2)).abs() <= 1.001 * axis.max_acc
                );
                assert!(((p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0]) / cycle.powi(3)).abs() <= 1.001 * axis.max_jerk);
            }
        }
        assert!(distance(&points[points.len() - 1].1, &[0.2, 0.0, -0.9]) < 1e-9);
        // With a tolerance its junctions with the lines are eased as on two axes, within the tolerance.
        let eased = deviation(&axes, &[0.0, 0.0, -0.9], &pieces, 1e-3, ARC_SLACK).unwrap();
        assert!(eased > 0.0 && eased <= 1e-3, "{eased} m");
    }

    /// A corner between a line and an arc is rounded within the corner tolerance, no longer stopped at. The line runs
    /// along +x to (0.1, 0); the arc of radius 1 m around (-0.9, 0) leaves there along +y, counter-clockwise: a right
    /// angle. Between two lines a right angle is rounded from d = 16 tolerance / 3 before the corner to d after it, its
    /// deviation 3 d / 16 at the middle; the arc bends away from its tangent by d^2 / (2 R) = 1.4e-7 m over d, 0.1 % of
    /// the tolerance, so with 0.1 mm the curve starts 16e-4 / 3 = 0.533 mm before the corner within the 2 % the search
    /// leaves (`round_corner`), and its deviation is the tolerance within those 2 %. At its ends the curve has the
    /// pieces' curvature: none on the line, 1 / R = 1 / m on the arc (G2). Before, the corner stayed a stop.
    #[test]
    fn a_line_arc_corner_is_rounded_within_the_corner_tolerance() {
        let axes = axes();
        let pieces = parse("line,0.1,0.0\narc,0.0950041652780258,0.0998334166468282,-0.9,0.0,ccw").unwrap();
        let tolerance = Tolerance { path: 0.0, corner: 1e-4 };
        let (geo, off) = geometry(&axes, &[0.0, 0.0], &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        assert_eq!(geo.len(), 3, "line, curve, arc");
        let d = 0.1 - geo[0].length();
        assert!((d / (16e-4 / 3.0) - 1.0).abs() < 0.025, "{d} m");
        assert!((0.97e-4..=1e-4).contains(&off), "{off} m");
        let size = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let curve = &geo[1];
        assert!(size(&curve.at(0.0)[2]) < 1e-6 && (size(&curve.at(curve.length())[2]) - 1.0).abs() < 1e-3);
        assert!(!corner(&geo[0], &geo[1]) && !corner(&geo[1], &geo[2]));
        // The plan passes the junctions at speed and ends on the arc's end.
        let points = plan(&axes, &[0.0, 0.0], &pieces, 1e-3, tolerance, ARC_SLACK).unwrap();
        let at = |p: &(f64, Vec<f64>)| (p.1[0] - 0.1).abs() + p.1[1].abs();
        let near = points.windows(2).min_by(|a, b| at(&a[0]).total_cmp(&at(&b[0]))).unwrap();
        assert!(distance(&near[0].1, &near[1].1) > 1e-5, "stops at the corner");
        assert!(distance(&points[points.len() - 1].1, &[0.0950041652780258, 0.0998334166468282]) < 1e-9);
        // Without a corner tolerance it stays a corner.
        let (stopped, _) = geometry(&axes, &[0.0, 0.0], &pieces, 0.0, &axis_speed(&axes), ARC_SLACK).unwrap();
        assert!(stopped.len() == 2 && corner(&stopped[0], &stopped[1]));
    }

    #[test]
    fn a_path_file_is_read_line_by_line() {
        let pieces =
            parse("# a corner rounded off\nline, 0.5, 0.35\n\narc,0.6,0.25,0.5,0.25,cw\nline,0.6,0.05\n").unwrap();
        assert_eq!(pieces.len(), 3);
        assert_eq!(pieces[0], Piece::Line(vec![0.5, 0.35]));
        assert_eq!(pieces[1], Piece::Arc { to: vec![0.6, 0.25], centre: vec![0.5, 0.25], clockwise: true });
        assert_eq!(parse("spline, 0.7,0.2").unwrap(), [Piece::Spline(vec![0.7, 0.2])]);
        for bad in [
            "circle,1,2",
            "line",
            "line,a",
            "arc,0.6,0.25,0.5,0.25",
            "arc,0.6,0.25,0.5,0.25,left",
            "line,inf",
            "spline",
            "spline,0.5,nan",
        ] {
            assert_eq!(parse(&format!("line,0.5\n{bad}")), Err(bad.to_string()));
        }
        // An arc whose end is not on the circle through the start, an arc on one axis and a point with the
        // wrong number of positions cannot be laid out.
        let refused = |start: &[f64], text: &str| {
            plan(&axes()[..start.len()], start, &parse(text).unwrap(), 1e-3, 0.0, ARC_SLACK).unwrap_err().key
        };
        assert_eq!(refused(&[0.5, 0.35], "arc,0.6,0.26,0.5,0.25,cw"), "error-path");
        // A start up to 0.1 mm off the circle (the setpoint of a machine at rest) is run; one a millimetre off
        // is not.
        let geo = given(&[0.0999876, 0.3500458], &parse("arc,0.3,0.35,0.2,0.35,cw").unwrap(), ARC_SLACK).unwrap().0;
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
        let arc = |by: f64, r: f64| Piece::Arc {
            to: vec![0.3 + r * by.sin(), 0.25 + r * by.cos()],
            centre: vec![0.3, 0.25],
            clockwise: true,
        };
        let lengths = |start: &[f64], pieces: &[Piece]| {
            given(start, pieces, ARC_SLACK).unwrap().0.iter().map(Geo::length).collect::<Vec<f64>>()
        };
        let short = [Piece::Line(top.clone()), arc(8e-4, 0.1), Piece::Line(vec![0.5, 0.35])];
        assert!((lengths(&[0.1, 0.35], &short)[1] - 8e-5).abs() < 1e-12, "{:?}", lengths(&[0.1, 0.35], &short));
        let lowest = plan(&axes, &[0.1, 0.35], &short, 1e-3, 0.0, ARC_SLACK)
            .unwrap()
            .iter()
            .map(|p| p.1[1])
            .fold(f64::INFINITY, f64::min);
        assert!(lowest > 0.3499, "the path goes down to z = {lowest}");
        assert_eq!(deviation(&axes, &[0.1, 0.35], &short, 0.0, ARC_SLACK).unwrap(), 0.0);
        assert!(
            (lengths(&[0.1, 0.35], &[Piece::Line(top.clone()), arc(0.0, 0.1)])[1] - std::f64::consts::TAU * 0.1).abs()
                < 1e-12
        );

        let (pose, rest) = (vec![0.1, 0.35], [0.0999876, 0.3500458]);
        let up = |by: f64| Piece::Arc {
            to: vec![0.2 - 0.1 * by.cos(), 0.35 + 0.1 * by.sin()],
            centre: vec![0.2, 0.35],
            clockwise: true,
        };
        for (start, first) in [
            (&rest[..], up(1.3e-3)),
            (&rest[..], up(0.0)),
            (&pose[..], up(0.0)),
            (&top[..], arc(5e-4, 0.1)),
            (&top[..], arc(9.9e-3, 0.1)),
        ] {
            assert_eq!(
                deviation(&axes, start, std::slice::from_ref(&first), 0.0, ARC_SLACK).unwrap_err().key,
                "error-path-first-arc",
                "{first:?} from {start:?}"
            );
        }
        assert!((lengths(&rest, &[up(0.02)])[0] - 2e-3).abs() < 1e-4);
        for (by, length) in [(1.3e-3, 1.3e-4), (0.0, std::f64::consts::TAU * 0.1)] {
            let stated = lengths(&rest, &[Piece::Line(pose.clone()), up(by)]);
            assert!(stated.len() == 2 && (stated[1] - length).abs() < 1e-7, "{stated:?}");
        }
        assert!((lengths(&top, &[Piece::Line(top.clone()), arc(5e-4, 0.1)])[0] - 5e-5).abs() < 1e-12);
        let sagged = lengths(&[0.300012, 0.350046], &[Piece::Line(top.clone()), arc(5e-4, 0.1)]);
        assert!(sagged.len() == 2 && (sagged[1] - 5e-5).abs() < 1e-12, "{sagged:?}");

        let off_by = |by: f64| {
            [Piece::Line(top.clone()), arc(std::f64::consts::FRAC_PI_2, 0.1 + by), Piece::Line(vec![0.4, 0.05])]
        };
        assert_eq!(deviation(&axes, &[0.1, 0.35], &off_by(-9e-5), 0.0, ARC_SLACK).unwrap_err().key, "error-path");
        let reported = deviation(&axes, &[0.1, 0.35], &off_by(8e-6), 0.0, ARC_SLACK).unwrap();
        assert!((reported - 8e-6).abs() < 1e-12, "{reported} m reported");
        let spiral: Vec<Piece> = std::iter::once(Piece::Line(top.clone()))
            .chain((1..=4).map(|k| arc(k as f64 * std::f64::consts::FRAC_PI_2, 0.1 + 9e-6 * k as f64)))
            .collect();
        let added = deviation(&axes, &[0.1, 0.35], &spiral, 0.0, ARC_SLACK).unwrap();
        assert!((added - 3.6e-5).abs() < 1e-12, "{added} m reported");
        let blended = deviation(&axes, &[0.1, 0.35], &off_by(8e-6), 1e-4, ARC_SLACK).unwrap();
        assert!((blended - 1.08e-4).abs() < 1e-6, "{blended} m reported with transition curves");
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
        let straight = plan(&axes, &[0.1, 0.1], &parse("spline,0.5,0.4").unwrap(), cycle, 0.0, ARC_SLACK).unwrap();
        // On the line through (0.1, 0.1) and (0.5, 0.4): 3 (x - 0.1) = 4 (z - 0.1).
        assert!(
            straight.iter().all(|(_, p)| (3.0 * (p[0] - 0.1) - 4.0 * (p[1] - 0.1)).abs() < 1e-12)
                && distance(&straight[straight.len() - 1].1, &[0.5, 0.4]) < 1e-12
        );
        let [Geo::Spline { travelled, .. }] =
            &given(&[0.1, 0.1], &parse("spline,0.5,0.4").unwrap(), ARC_SLACK).unwrap().0[..]
        else {
            panic!("one span")
        };
        assert!((travelled[SPLINE_PARTS] - 0.5).abs() < 1e-14 && (travelled[16] - 0.125).abs() < 1e-14);

        let paths = [
            ([0.1, 0.35], "line,0.3,0.35\nspline,0.4,0.3\nspline,0.5,0.32\nspline,0.6,0.2\nline,0.6,0.05"),
            ([0.2, 0.1], "spline,0.3,0.2\nspline,0.5,0.25\nspline,0.7,0.1"),
            (
                [0.1, 0.5],
                "line,0.3,0.5\narc,0.4,0.4,0.3,0.4,cw\nspline,0.45,0.3\nspline,0.55,0.25\narc,0.65,0.15,0.55,0.15,cw\nline,0.65,0.05",
            ),
            (
                [0.3, 0.2],
                "line,0.4,0.2\nspline,0.42,0.21\nspline,0.44,0.19\nspline,0.46,0.21\nspline,0.48,0.2\nline,0.6,0.2",
            ),
        ];
        for (start, text) in paths {
            let pieces = parse(text).unwrap();
            let geo = given(&start, &pieces, ARC_SLACK).unwrap().0;
            let via: Vec<&Vec<f64>> =
                pieces.iter().filter_map(|p| if let Piece::Spline(to) = p { Some(to) } else { None }).collect();
            let ends: Vec<Vec<f64>> =
                geo.iter().filter(|g| matches!(g, Geo::Spline { .. })).map(|g| g.at(g.length())[0].clone()).collect();
            assert_eq!(ends.len(), via.len(), "{text}");
            for (end, via) in ends.iter().zip(&via) {
                assert!(distance(end, via) < 1e-12, "{text}: span ends at {end:?}, via point {via:?}");
            }
            for pair in geo.windows(2) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps =
                    [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                let both = matches!(pair, [Geo::Spline { .. }, Geo::Spline { .. }]);
                assert!(steps[0] < 1e-12 && steps[1] < 1e-9 && (!both || steps[2] < 1e-6), "{text}: steps {steps:?}");
            }
            // With a tolerance of 0.1 mm the spline is eased into the lines and arcs it meets (`ease`), and an arc beside
            // it gets its transition curve towards the line at its other end (`blend`): no step in position, direction
            // or curvature at any junction, every planned pose within the reported deviation of the path as given, the
            // reported deviation within the tolerance, the axis limits kept, the via points still passed, and faster.
            let (eased, reported) = geometry(&axes, &start, &pieces, 1e-4, &axis_speed(&axes), ARC_SLACK).unwrap();
            assert!(
                reported <= 1e-4 + 1e-12 && (reported > 0.0) == (eased.len() > geo.len()),
                "{text}: {reported} m reported, {} pieces from {}",
                eased.len(),
                geo.len()
            );
            for pair in eased.windows(2) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps =
                    [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-6, "{text}, eased: steps {steps:?}");
            }
            let (smooth, given_plan) = (
                plan(&axes, &start, &pieces, cycle, 1e-4, ARC_SLACK).unwrap(),
                plan(&axes, &start, &pieces, cycle, 0.0, ARC_SLACK).unwrap(),
            );
            let peak = peak_rates(&smooth, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(
                    peak[0][i] <= 1.001 * axis.max_vel
                        && peak[1][i] <= 1.001 * axis.max_acc
                        && peak[2][i] <= 1.001 * axis.max_jerk,
                    "{text}, eased, axis {i}: {peak:?}"
                );
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

            let points = plan(&axes, &start, &pieces, cycle, 0.0, ARC_SLACK).unwrap();
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(
                    peak[0][i] <= 1.001 * axis.max_vel
                        && peak[1][i] <= 1.001 * axis.max_acc
                        && peak[2][i] <= 1.001 * axis.max_jerk,
                    "{text}, axis {i}: {peak:?}"
                );
            }
            let last = points.len() - 1;
            let end = geo[geo.len() - 1].at(geo[geo.len() - 1].length())[0].clone();
            assert!(
                points[0].1 == start
                    && distance(&points[last].1, &end) < 1e-9
                    && distance(&points[last].1, &points[last - 1].1) < 1e-6,
                "{text}"
            );
            for via in &via {
                let nearest = points.iter().map(|p| distance(&p.1, via)).fold(f64::INFINITY, f64::min);
                assert!(nearest <= 0.5 * 0.8 * cycle * 1.01, "{text}: {nearest} m past the via point {via:?}");
            }
            println!("{} s, peak rates {peak:?}: {}", points[last].0, text.replace('\n', " | "));
        }

        // A via point straight back along the line before it: the curve would have to turn on the spot.
        assert_eq!(
            plan(&axes, &[0.1, 0.35], &parse("line,0.5,0.35\nspline,0.3,0.35").unwrap(), cycle, 0.0, ARC_SLACK)
                .unwrap_err()
                .key,
            "error-path"
        );
        // A via point given twice counts once, and a spline of nothing but where the path stands is no piece.
        let twice = given(&[0.2, 0.1], &parse("spline,0.3,0.2\nspline,0.3,0.2\nspline,0.5,0.25").unwrap(), ARC_SLACK)
            .unwrap()
            .0;
        assert_eq!(twice.len(), 2);
        assert_eq!(
            plan(&axes, &[0.2, 0.1], &parse("spline,0.2,0.1").unwrap(), cycle, 0.0, ARC_SLACK).unwrap_err().key,
            "error-path"
        );
    }

    /// Rates of the planned poses by differences over one cycle: the largest speed, acceleration and jerk
    /// of each axis.
    fn peak_rates(points: &[(f64, Vec<f64>)], cycle: f64) -> [[f64; 2]; 3] {
        let mut peak = [[0f64; 2]; 3];
        for w in points.windows(4) {
            for i in 0..2 {
                let p = [w[0].1[i], w[1].1[i], w[2].1[i], w[3].1[i]];
                let rates = [
                    (p[1] - p[0]) / cycle,
                    (p[2] - 2.0 * p[1] + p[0]) / cycle.powi(2),
                    (p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0]) / cycle.powi(3),
                ];
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

    /// How far a point is from a path given as `polyline`: from the nearest of its straight pieces. Any number of axes.
    fn off_line(line: &[Vec<f64>], p: &[f64]) -> f64 {
        let dot = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
        line.windows(2)
            .map(|w| {
                let ab: Vec<f64> = w[1].iter().zip(&w[0]).map(|(b, a)| b - a).collect();
                let ap: Vec<f64> = p.iter().zip(&w[0]).map(|(p, a)| p - a).collect();
                let t = (dot(&ab, &ap) / dot(&ab, &ab).max(1e-300)).clamp(0.0, 1.0);
                ap.iter().zip(&ab).map(|(p, b)| (p - t * b) * (p - t * b)).sum::<f64>().sqrt()
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
        let points = plan(&axes, &[0.1, 0.35], &pieces, cycle, 0.0, ARC_SLACK).unwrap();
        let peak = peak_rates(&points, cycle);
        for (i, axis) in axes.iter().enumerate() {
            assert!(
                peak[0][i] <= 1.001 * axis.max_vel
                    && peak[1][i] <= 1.001 * axis.max_acc
                    && peak[2][i] <= 1.001 * axis.max_jerk,
                "axis {i}: {peak:?}"
            );
        }
        // The limits are used: full speed and acceleration on the first line, most of the jerk limit.
        assert!(peak[0][0] > 0.999 * 0.8 && peak[1][0] > 0.99 * 8.0 && peak[2][0] > 0.95 * 100.0, "{peak:?}");

        let on_arc = |p: &[f64]| p[0] > 0.5 + 1e-9 && p[1] > 0.25 + 1e-9;
        let (mut off_path, mut arc_speed, mut junction_speed) = (0f64, 0f64, f64::INFINITY);
        for w in points.windows(2) {
            let p = &w[1].1;
            let off = if on_arc(p) {
                (distance(p, &[0.5, 0.25]) - 0.1).abs()
            } else {
                (p[1] - 0.35).abs().min((p[0] - 0.6).abs())
            };
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
        println!(
            "duration {} s, peak rates {peak:?}, arc {arc_speed} m/s, junction {junction_speed} m/s",
            points[last].0
        );
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
        let pieces = parse(
            "line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.05",
        )
        .unwrap();
        let given = plan(&axes, &start, &pieces, cycle, 0.0, ARC_SLACK).unwrap();
        assert_eq!(deviation(&axes, &start, &pieces, 0.0, ARC_SLACK).unwrap(), 0.0);
        // How far a pose is from the given path: from the first line, the arc or the second line.
        let off = |p: &[f64]| {
            if p[0] <= 0.5 {
                (p[1] - 0.35).abs()
            } else if p[1] <= 0.25 {
                (p[0] - 0.6).abs()
            } else {
                (distance(p, &[0.5, 0.25]) - 0.1).abs()
            }
        };
        let curves = |tolerance: f64| -> Vec<f64> {
            let geo = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap().0;
            geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect()
        };
        for (tolerance, by_hand, length, speed) in [(1e-4, 1e-4, 0.01549, 0.3957), (1e-3, 0.1067e-3, 0.016, 0.4)] {
            let reported = deviation(&axes, &start, &pieces, tolerance, ARC_SLACK).unwrap();
            assert!(
                (reported / by_hand - 1.0).abs() < 2e-3 && reported <= tolerance,
                "{reported} m reported, {by_hand} m by hand"
            );
            let lengths = curves(tolerance);
            assert!(
                lengths.len() == 2 && lengths.iter().all(|l| (l / length - 1.0).abs() < 5e-3),
                "{lengths:?} against {length}"
            );

            let points = plan(&axes, &start, &pieces, cycle, tolerance, ARC_SLACK).unwrap();
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(
                    peak[0][i] <= 1.001 * axis.max_vel
                        && peak[1][i] <= 1.001 * axis.max_acc
                        && peak[2][i] <= 1.001 * axis.max_jerk,
                    "axis {i}: {peak:?}"
                );
            }
            let furthest = points.iter().map(|p| off(&p.1)).fold(0.0, f64::max);
            assert!(
                furthest <= reported + 1e-9 && furthest > 0.999 * reported,
                "{furthest} m off the path, {reported} m reported"
            );
            // The slowest the path gets between leaving the first line and being back on the second.
            let blended = |p: &[f64]| off(p) > 1e-9;
            let slowest = points
                .windows(2)
                .filter(|w| blended(&w[0].1) && blended(&w[1].1))
                .map(|w| distance(&w[0].1, &w[1].1) / cycle)
                .fold(f64::INFINITY, f64::min);
            assert!((slowest - speed).abs() < 2e-3, "{slowest} m/s through the blend, {speed} m/s by hand");
            let (last, before) = (points.len() - 1, given[given.len() - 1].0);
            assert!(
                points[0].1 == start
                    && distance(&points[last].1, &[0.6, 0.05]) < 1e-9
                    && distance(&points[last].1, &points[last - 1].1) < 1e-6
            );
            assert!(points[last].0 < 0.9 * before, "{} s with transition curves, {before} s without", points[last].0);
            println!(
                "tolerance {tolerance} m: {} s against {before} s, off by {furthest} m, curves {lengths:?} m, slowest {slowest} m/s, peak rates {peak:?}",
                points[last].0
            );
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
            let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
            assert_eq!(geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).count(), 3, "{text}");
            // The tighter arc is the one that moves in by the whole tolerance.
            assert!((reported - tolerance).abs() < 1e-12, "{reported}");
            for pair in geo.windows(2) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps =
                    [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-7, "{text}: steps {steps:?}");
            }
            let (points, given) = (
                plan(&axes, &start, &pieces, cycle, tolerance, ARC_SLACK).unwrap(),
                plan(&axes, &start, &pieces, cycle, 0.0, ARC_SLACK).unwrap(),
            );
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(
                    peak[0][i] <= 1.001 * axis.max_vel
                        && peak[1][i] <= 1.001 * axis.max_acc
                        && peak[2][i] <= 1.001 * axis.max_jerk,
                    "{text}, axis {i}: {peak:?}"
                );
            }
            let (last, before) = (points.len() - 1, given[given.len() - 1].0);
            assert!(
                distance(&points[last].1, &end) < 1e-9 && points[last].0 < 0.9 * before,
                "{text}: {} s against {before} s",
                points[last].0
            );
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
        let arc =
            |to: [f64; 2], centre: [f64; 2]| Piece::Arc { to: to.to_vec(), centre: centre.to_vec(), clockwise: true };
        let pieces = [Piece::Line(vec![0.2, 0.2]), arc(p1, c1), arc(p2, c2), Piece::Line(end.to_vec())];
        let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        let turned: f64 = geo
            .iter()
            .map(|g| match g {
                Geo::Arc { radius, turn, length, .. } => turn * length / radius,
                Geo::Clothoid { k0, k1, length, .. } => 0.5 * (k0 + k1) * length,
                Geo::Line { .. } | Geo::Slerp { .. } | Geo::Spline { .. } | Geo::Arc3 { .. } | Geo::Carried { .. } => {
                    0.0
                }
            })
            .sum();
        assert!((turned + a + b).abs() < 1e-9, "the planned path turns by {turned} rad, the given one by {}", -a - b);
        // It is blended all the same, with shorter curves, and the tight arc moves in by less than its radius.
        assert!(
            geo.iter().any(|g| matches!(g, Geo::Clothoid { .. })) && reported > 0.0 && reported < tight,
            "{reported} m reported"
        );
        let points = plan(&axes, &start, &pieces, 1e-3, tolerance, ARC_SLACK).unwrap();
        assert!(distance(&points[points.len() - 1].1, &end) < 1e-9);
        println!("turned {turned} rad, off by {reported} m, {} pieces", geo.len());
    }

    /// A line that ends on the end of a stroke, at z = 0: its last pose came out at z = -2.8e-17, outside the
    /// range, and the controller refused the whole path. Planned poses stay within the ranges of the axes.
    #[test]
    fn a_path_onto_the_end_of_a_stroke_stays_within_the_range() {
        let axes = axes();
        let points = plan(&axes, &[0.278, 0.231], &[Piece::Line(vec![0.525, 0.0])], 1e-3, 0.0, ARC_SLACK).unwrap();
        assert!(points.iter().all(|p| p.1[1] >= 0.0), "ends at z = {:e}", points[points.len() - 1].1[1]);
        assert_eq!(points[points.len() - 1].1, [0.525, 0.0]);
        // A path that leaves the range for good is left as it is, for the controller to refuse.
        let beyond = plan(&axes, &[0.278, 0.231], &[Piece::Line(vec![0.525, -0.001])], 1e-3, 0.0, ARC_SLACK).unwrap();
        assert!(beyond[beyond.len() - 1].1[1] < -0.000_999);
    }

    /// A corner is stopped at: the speed at the corner of two lines at right angles is zero, and each line
    /// is the S-curve of a single move, whose time the profile test calculates by hand.
    #[test]
    fn a_corner_is_stopped_at() {
        let (axes, cycle) = (axes(), 1e-3);
        let points = plan(
            &axes,
            &[0.1, 0.1],
            &[Piece::Line(vec![0.5, 0.1]), Piece::Line(vec![0.5, 0.3])],
            cycle,
            0.0,
            ARC_SLACK,
        )
        .unwrap();
        let moves = [
            crate::profile::Move::new(&[0.1, 0.1], &[0.5, 0.1], &axes),
            crate::profile::Move::new(&[0.5, 0.1], &[0.5, 0.3], &axes),
        ];
        let duration = points[points.len() - 1].0;
        assert!((duration - moves[0].duration() - moves[1].duration()).abs() <= cycle, "{duration} s");
        let corner = (moves[0].duration() / cycle).round() as usize;
        assert!(
            distance(&points[corner].1, &[0.5, 0.1]) < 1e-6
                && distance(&points[corner + 1].1, &points[corner].1) < 1e-6
        );
        let peak = peak_rates(&points, cycle);
        assert!(
            peak[1][0] <= 8.0 * 1.001 && peak[1][1] <= 6.0 * 1.001 && peak[2][0] <= 100.1 && peak[2][1] <= 100.1,
            "{peak:?}"
        );
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
        let off = |p: &[f64]| {
            if p[0] <= 0.5 {
                (p[1] - 0.35).abs()
            } else if p[0] <= 0.6 {
                (distance(p, &[0.5, 0.25]) - 0.1).abs()
            } else {
                (p[1] - 0.25).abs()
            }
        };
        for (text, end) in [
            ("line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw", [0.6, 0.25]),
            ("line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw\nline,0.7,0.25", [0.7, 0.25]),
        ] {
            let pieces = parse(text).unwrap();
            let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
            let curves: Vec<f64> = geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect();
            assert!(curves.len() == 1 && (curves[0] / 0.01549 - 1.0).abs() < 5e-3, "{text}: curves {curves:?}");
            assert!(
                (reported / tolerance - 1.0).abs() < 2e-3 && reported <= tolerance,
                "{text}: {reported} m reported"
            );
            let arc = geo.iter().find(|g| matches!(g, Geo::Arc { .. })).unwrap();
            assert!(
                distance(&arc.at(arc.length())[0], &[0.6, 0.25]) < 1e-12
                    && distance(&arc.at(arc.length())[1], &[0.0, -1.0]) < 1e-12,
                "{text}: the arc ends elsewhere"
            );
            for pair in geo.windows(2).filter(|pair| !corner(&pair[0], &pair[1])) {
                let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
                let steps =
                    [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
                assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-7, "{text}: steps {steps:?}");
            }
            for tolerance in [0.0, tolerance] {
                let points = plan(&axes, &start, &pieces, cycle, tolerance, ARC_SLACK).unwrap();
                let peak = peak_rates(&points, cycle);
                for (i, axis) in axes.iter().enumerate() {
                    assert!(
                        peak[0][i] <= 1.001 * axis.max_vel
                            && peak[1][i] <= 1.001 * axis.max_acc
                            && peak[2][i] <= 1.001 * axis.max_jerk,
                        "{text}, axis {i}: {peak:?}"
                    );
                }
                let furthest = points.iter().map(|p| off(&p.1)).fold(0.0, f64::max);
                assert!(
                    furthest <= reported + 1e-9 && (tolerance == 0.0 || furthest > 0.5 * reported),
                    "{text}: {furthest} m off the path"
                );
                // The arc's end is passed exactly (the path stops there), and the path ends on its target.
                assert!(
                    points.iter().any(|p| distance(&p.1, &[0.6, 0.25]) < 1e-6)
                        && distance(&points[points.len() - 1].1, &end) < 1e-9,
                    "{text}"
                );
                // The speed where the path leaves the first line.
                let leaves = points.iter().position(|p| off(&p.1) > 1e-9 || p.1[0] > 0.5).unwrap();
                let speed = distance(&points[leaves].1, &points[leaves - 1].1) / cycle;
                let by_hand = if tolerance > 0.0 { 0.3957 } else { 0.0707 };
                assert!(
                    (speed - by_hand).abs() < 2e-3,
                    "{text}, tolerance {tolerance}: {speed} m/s into the arc, {by_hand} by hand"
                );
                println!(
                    "{text:?}, tolerance {tolerance}: {} s, {speed} m/s into the arc, off by {furthest} m",
                    points[points.len() - 1].0
                );
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
        let geo = given(&start, &pieces, ARC_SLACK).unwrap().0;
        let Geo::Spline { c, .. } = &geo[1] else { panic!("a spline span") };
        let [d, d2, _] = poly_rates(c, 0.0);
        assert!((d[0] - 1.0).abs() < 1e-12 && d[1].abs() < 1e-12 && (d2[1] - 10.0).abs() < 1e-9, "{d:?} {d2:?}");
        let (eased, reported) = geometry(&axes, &start, &pieces, 1e-4, &axis_speed(&axes), ARC_SLACK).unwrap();
        let Geo::Spline { h: l, .. } = &eased[1] else { panic!("the eased part") };
        assert!(
            eased.len() == 3 && (l - 0.024056).abs() < 1e-5 && (reported - 1e-4).abs() < 1e-12,
            "{l} m eased, {reported} m reported"
        );
        for (tolerance, by_hand) in [(0.0, 0.0707), (1e-4, 0.2189)] {
            let points = plan(&axes, &start, &pieces, cycle, tolerance, ARC_SLACK).unwrap();
            let peak = peak_rates(&points, cycle);
            for (i, axis) in axes.iter().enumerate() {
                assert!(
                    peak[0][i] <= 1.001 * axis.max_vel
                        && peak[1][i] <= 1.001 * axis.max_acc
                        && peak[2][i] <= 1.001 * axis.max_jerk,
                    "tolerance {tolerance}, axis {i}: {peak:?}"
                );
            }
            let past = points.iter().position(|p| p.1[0] > 0.3).unwrap();
            let speed = distance(&points[past].1, &points[past - 1].1) / cycle;
            assert!(
                (speed / by_hand - 1.0).abs() < 0.01,
                "tolerance {tolerance}: {speed} m/s at the junction, {by_hand} by hand"
            );
            let line = polyline(&geo);
            let furthest = points.iter().map(|p| off_line(&line, &p.1)).fold(0.0, f64::max);
            assert!(
                furthest <= reported * (tolerance > 0.0) as u8 as f64 + 1e-7
                    && (tolerance == 0.0 || furthest > 0.9 * reported),
                "tolerance {tolerance}: {furthest} m off"
            );
            assert!(distance(&points[points.len() - 1].1, &[0.39, 0.23]) < 1e-9);
            println!(
                "tolerance {tolerance}: {} s, {speed} m/s at the junction, off by {furthest} m",
                points[points.len() - 1].0
            );
        }
    }

    /// Total length and total turning of a path.
    fn length_and_turning(geo: &[Geo]) -> (f64, f64) {
        let turned = |g: &Geo| match g {
            Geo::Arc { radius, turn, length, .. } => turn * length / radius,
            Geo::Clothoid { k0, k1, length, .. } => 0.5 * (k0 + k1) * length,
            Geo::Line { .. } | Geo::Slerp { .. } | Geo::Spline { .. } | Geo::Arc3 { .. } | Geo::Carried { .. } => 0.0,
        };
        (geo.iter().map(Geo::length).sum(), geo.iter().map(turned).sum())
    }

    /// The blended path is as long as the given one, but for what the transitions change: each curve replaces about
    /// its own length of the pieces beside it, and each arc moved in by at most the reported deviation is shorter by
    /// at most that times the angle it turns. And it turns by as much (a lost or added turn shows here).
    fn assert_blend_keeps_the_path(start: &[f64], pieces: &[Piece], tolerance: f64, what: &str) {
        let axes = axes();
        let (given, _) = geometry(&axes, start, pieces, 0.0, &axis_speed(&axes), ARC_SLACK).unwrap();
        let (planned, reported) = geometry(&axes, start, pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        let ((given_length, given_turn), (planned_length, planned_turn)) =
            (length_and_turning(&given), length_and_turning(&planned));
        let curves: f64 = planned.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).sum();
        let allowance = curves + reported * given_turn.abs().max(planned_turn.abs()) + 1e-12;
        assert!(
            (planned_length - given_length).abs() <= allowance,
            "{what}: given {given_length} m, planned {planned_length} m, allowance {allowance} m"
        );
        assert!(
            (planned_turn - given_turn).abs() < 1e-6,
            "{what}: given turns {given_turn} rad, planned {planned_turn} rad"
        );
    }

    /// Two arcs on the same circle that together turn more than a whole turn (4 rad, then pi + 0.05 rad, radius
    /// 0.05 m), between two lines: their radii differ by rounding, so a transition curve was built between them,
    /// and what it took of the arcs wrapped by a whole turn: the planned path was 0.1953 m long for 0.5096 m given.
    /// Equal curvatures get no curve, and the path keeps its length and its turning.
    #[test]
    fn two_arcs_on_one_circle_keep_their_whole_turn() {
        let (centre, r) = ([0.5, 0.5], 0.05);
        let at = |angle: f64| vec![centre[0] + r * angle.cos(), centre[1] + r * angle.sin()];
        let (a1, a2) =
            (std::f64::consts::FRAC_PI_2 - 4.0, std::f64::consts::FRAC_PI_2 - 4.0 - std::f64::consts::PI - 0.05);
        let end = at(a2);
        let arc = |to: Vec<f64>| Piece::Arc { to, centre: centre.to_vec(), clockwise: true };
        let pieces = [
            Piece::Line(vec![0.5, 0.55]),
            arc(at(a1)),
            arc(end.clone()),
            Piece::Line(vec![end[0] + 0.1 * a2.sin(), end[1] - 0.1 * a2.cos()]),
        ];
        let axes = axes();
        let given =
            length_and_turning(&geometry(&axes, &[0.45, 0.55], &pieces, 0.0, &axis_speed(&axes), ARC_SLACK).unwrap().0)
                .0;
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
                assert_blend_keeps_the_path(
                    &lead,
                    &pieces,
                    tolerance,
                    &format!("case {case}, tolerance {tolerance}: {pieces:?}"),
                );
            }
        }
    }

    /// No step in position, direction or curvature between two pieces that meet in their direction.
    fn assert_no_steps(geo: &[Geo], what: &str) {
        for pair in geo.windows(2).filter(|pair| !corner(&pair[0], &pair[1])) {
            let (before, after) = (pair[0].at(pair[0].length()), pair[1].at(0.0));
            let steps =
                [distance(&before[0], &after[0]), distance(&before[1], &after[1]), distance(&before[2], &after[2])];
            assert!(steps[0] < 1e-11 && steps[1] < 1e-9 && steps[2] < 1e-7, "{what}: steps {steps:?}");
        }
    }

    /// Every planned pose is within the reported deviation of the given path (and a part of it gets there), the axis
    /// limits hold (by differences, as above), the path ends on `end`, and it takes less time than as given (the gain
    /// is that of the junctions only: 5 % on a path that is mostly one arc at full speed). The planned poses.
    fn assert_blend_runs_within(
        axes: &[AxisInfo],
        start: &[f64],
        pieces: &[Piece],
        tolerance: f64,
        end: &[f64],
        what: &str,
    ) -> Vec<(f64, Vec<f64>)> {
        let (cycle, given) = (1e-3, geometry(axes, start, pieces, 0.0, &axis_speed(axes), ARC_SLACK).unwrap().0);
        let reported = deviation(axes, start, pieces, tolerance, ARC_SLACK).unwrap();
        assert!(reported > 0.0 && reported <= tolerance * (1.0 + 1e-9), "{what}: {reported} m reported");
        let (points, before) = (
            plan(axes, start, pieces, cycle, tolerance, ARC_SLACK).unwrap(),
            plan(axes, start, pieces, cycle, 0.0, ARC_SLACK).unwrap(),
        );
        let line = polyline(&given);
        let furthest = points.iter().map(|p| off_line(&line, &p.1)).fold(0.0, f64::max);
        assert!(
            furthest <= reported + 1e-7 && furthest > 0.5 * reported,
            "{what}: {furthest} m off, {reported} m reported"
        );
        for w in points.windows(4) {
            for (i, axis) in axes.iter().enumerate() {
                let p = [w[0].1[i], w[1].1[i], w[2].1[i], w[3].1[i]];
                assert!(
                    ((p[1] - p[0]) / cycle).abs() <= 1.001 * axis.max_vel
                        && ((p[2] - 2.0 * p[1] + p[0]) / cycle.powi(2)).abs() <= 1.001 * axis.max_acc,
                    "{what}, axis {i}"
                );
                assert!(
                    ((p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0]) / cycle.powi(3)).abs() <= 1.001 * axis.max_jerk,
                    "{what}, axis {i}"
                );
            }
        }
        let (last, slower) = (points.len() - 1, before[before.len() - 1].0);
        assert!(
            distance(&points[last].1, end) < 1e-9 && points[last].0 < slower - 0.01,
            "{what}: {} s against {slower} s",
            points[last].0
        );
        println!("{what}: {} s against {slower} s, off by {furthest} m, {reported} m reported", points[last].0);
        points
    }

    /// The pieces of a line, an arc and a line on three axes, in the plane through (0, 0, -0.85) along x and along
    /// (0, cos 30, sin 30) (tilted by 30 degrees about x, in no plane of two axes): along x onto the circle of 0.1 m
    /// around that point, a quarter of it counter-clockwise, then 0.15 m on in its direction. The start, the plane's
    /// point and its two directions.
    fn tilted_path() -> (Vec<f64>, Vec<Piece>, [[f64; 3]; 3]) {
        let (c, e1, e2, r) =
            ([0.0, 0.0, -0.85], [1.0, 0.0, 0.0], [0.0, 30f64.to_radians().cos(), 30f64.to_radians().sin()], 0.1);
        let at = |a: f64, b: f64| (0..3).map(|i| c[i] + a * e1[i] + b * e2[i]).collect::<Vec<f64>>();
        let pieces = vec![
            Piece::Line(at(0.0, -r)),
            Piece::Arc { to: at(r, 0.0), centre: c.to_vec(), clockwise: false },
            Piece::Line(at(r, 0.15)),
        ];
        (at(-0.2, -r), pieces, [c, e1, e2])
    }

    /// Transition curves on three axes, on `tilted_path` with a tolerance of 0.1 mm (the delta's limits: 3 m/s,
    /// 50 m/s^2, 2500 m/s^3). By hand: on the arc the speed is sqrt(0.8 a r) = sqrt(0.8 * 50 * 0.1) = 2 m/s, whose
    /// entry would need a curve of v^3 dk / (0.4 j) = 8 * 10 / 1000 = 80 mm; the tolerance allows
    /// L = sqrt(24 e r) = sqrt(24 * 1e-4 * 0.1) = 15.49 mm, so the arc moves in by e = L^2 / (24 r) = 0.1 mm to a radius
    /// of 99.9 mm. Both curves are 15.49 mm, every piece and every planned pose lies in the tilted plane, there is no
    /// step in position, direction or curvature, every pose is within the reported 0.1 mm of the given path and the
    /// path is faster than as given. Then a second arc of 0.05 m in another plane (x = 0.1, through the last line)
    /// and a line out of it: the line between the arcs lies in both planes and is shared out, so all four junctions get
    /// their curve (the second arc's in its own plane), with the same checks.
    #[test]
    fn transition_curves_on_three_axes_lie_in_the_plane_of_the_arc() {
        let (axes, tolerance) = (axes3(), 1e-4);
        let (start, pieces, [c, e1, e2]) = tilted_path();
        let normal = cross3(e1, e2);
        let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        assert!((reported - 1e-4).abs() < 1e-12, "{reported} m reported, 0.1 mm by hand");
        let curves: Vec<f64> = geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect();
        assert!(
            curves.len() == 2 && curves.iter().all(|l| (l / 0.01549 - 1.0).abs() < 5e-3),
            "curves {curves:?}, 15.49 mm by hand"
        );
        assert!(
            geo.iter().any(|g| matches!(g, Geo::Arc3 { radius, .. } if (radius - 0.0999).abs() < 1e-12)),
            "the arc moved in by 0.1 mm"
        );
        assert_no_steps(&geo, "tilted");
        let in_plane = |p: &[f64]| dot3([p[0] - c[0], p[1] - c[1], p[2] - c[2]], normal).abs();
        let points = assert_blend_runs_within(
            &axes,
            &start,
            &pieces,
            tolerance,
            &[0.1, 0.15 * e2[1], -0.85 + 0.15 * e2[2]],
            "tilted",
        );
        let off_plane = points.iter().map(|p| in_plane(&p.1)).fold(0.0, f64::max);
        assert!(off_plane < 1e-12, "{off_plane} m off the plane");

        // On out of the last line into an arc in the plane x = 0.1: its centre 0.05 m along the first plane's normal
        // (0, -sin 30, cos 30), a quarter turn towards e2, and 0.1 m on along that normal.
        let q = [0.1, 0.15 * e2[1], -0.85 + 0.15 * e2[2]];
        let centre: Vec<f64> = (0..3).map(|i| q[i] + 0.05 * normal[i]).collect();
        let end: Vec<f64> = (0..3).map(|i| centre[i] + 0.05 * e2[i] + 0.1 * normal[i]).collect();
        let more = [
            pieces,
            vec![
                Piece::Arc { to: (0..3).map(|i| centre[i] + 0.05 * e2[i]).collect(), centre, clockwise: false },
                Piece::Line(end.clone()),
            ],
        ]
        .concat();
        let (geo, _) = geometry(&axes, &start, &more, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        assert_eq!(geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).count(), 4);
        assert_no_steps(&geo, "two planes");
        assert_blend_runs_within(&axes, &start, &more, tolerance, &end, "two planes");
    }

    /// One curve length per arc: a line, a quarter arc of 0.1 m, a line, a quarter arc of 20 mm, a line, with 1 mm.
    /// By hand (axes as above, 100 m/s^3): on the wide arc 0.4 m/s (the axis limit), its curves
    /// L = v^3 dk / (0.4 j) = 0.064 * 10 / 40 = 16 mm (the tolerance allows sqrt(24 * 1e-3 * 0.1) = 49 mm); on the
    /// tight arc the jerk decides, v = (0.4 j r^2)^(1/3) = (40 * 0.0004)^(1/3) = 0.252 m/s, and
    /// L = 0.016 * 50 / 40 = 20 mm (allowed: sqrt(24 * 1e-3 * 0.02) = 21.9 mm). One length for the whole path made
    /// both 20 mm. The tight arc moves in by e = L^2 / (24 r) = 0.0004 / 0.48 = 0.833 mm, the wide one by
    /// 0.000256 / 2.4 = 0.107 mm; the larger is reported.
    #[test]
    fn each_arc_gets_the_curve_its_own_junctions_need() {
        let (axes, start, tolerance) = (axes(), [0.1, 0.35], 1e-3);
        let pieces = parse(
            "line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.15
arc,0.58,0.13,0.58,0.15,cw
line,0.3,0.13",
        )
        .unwrap();
        let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        let curves: Vec<f64> = geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect();
        assert_eq!(curves.len(), 4);
        println!("curves {curves:?} m");
        assert!(curves[..2].iter().all(|l| (l / 0.016 - 1.0).abs() < 5e-3), "{curves:?}: the wide arc's 16 mm by hand");
        // Fitted exactly, 19.67 mm: e = L^2 / (24 r) is the first term, and a curve as long as the radius turns 0.5 rad.
        assert!(
            curves[2..].iter().all(|l| (l / 0.020 - 1.0).abs() < 0.02),
            "{curves:?}: the tight arc's 20 mm by hand"
        );
        assert!((reported / 0.833e-3 - 1.0).abs() < 2e-3, "{reported} m reported, 0.833 mm by hand");
        assert_no_steps(&geo, "two arcs");
        assert_blend_runs_within(&axes, &start, &pieces, tolerance, &[0.3, 0.13], "two arcs");
    }

    /// An arc fixed at one end with an arc at its other end: the path starts on a quarter arc of 0.1 m (its start, where
    /// the machine stands, stays), which hands over to a quarter arc of 50 mm turning the other way (an S), or the same
    /// way, and then to a line. The first arc shrinks by s and moves towards its start, s (1 - cos 90 degrees) = e:
    /// both junctions get their curve, the first arc still leaves the start along +x, no step anywhere, every pose
    /// within the reported deviation (at most 0.1 mm), faster than as given.
    #[test]
    fn an_arc_fixed_at_one_end_gets_its_curve_towards_an_arc() {
        let (axes, start, tolerance) = (axes(), [0.3, 0.5], 1e-4);
        for (text, end) in [
            ("arc,0.4,0.4,0.3,0.4,cw\narc,0.45,0.35,0.45,0.4,ccw\nline,0.7,0.35", [0.7, 0.35]),
            ("arc,0.4,0.4,0.3,0.4,cw\narc,0.35,0.35,0.35,0.4,cw\nline,0.1,0.35", [0.1, 0.35]),
        ] {
            let pieces = parse(text).unwrap();
            let (geo, _) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
            assert_eq!(geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).count(), 2, "{text}");
            let [from, dir, _] = geo[0].at(0.0);
            assert!(distance(&from, &start) < 1e-12 && distance(&dir, &[1.0, 0.0]) < 1e-12, "{text}: {from:?} {dir:?}");
            assert_no_steps(&geo, text);
            assert_blend_runs_within(&axes, &start, &pieces, tolerance, &end, text);
        }
    }

    /// An arc fixed at one end that turns three quarters: a line into a 0.1 m arc that ends the path after 270 degrees.
    /// Halfway round its smaller circle is 2 s inside the old one, so s is at most half the tolerance: with 0.1 mm,
    /// s = 0.05 mm, the gap at the free end e = s (1 - cos 270 degrees) = 0.05 mm, the curve
    /// L = sqrt(24 e r) = sqrt(24 * 5e-5 * 0.1) = 10.95 mm (shorter than the 16 mm the speed asks for), and the reported
    /// deviation is 2 s = 0.1 mm. The arc still ends on (0.4, 0.25) heading +y, and the path runs as above.
    #[test]
    fn an_arc_fixed_at_one_end_that_turns_three_quarters_gets_its_curve() {
        let (axes, start, tolerance) = (axes(), [0.1, 0.35], 1e-4);
        let pieces = parse("line,0.5,0.35\narc,0.4,0.25,0.5,0.25,cw").unwrap();
        let (geo, reported) = geometry(&axes, &start, &pieces, tolerance, &axis_speed(&axes), ARC_SLACK).unwrap();
        let curves: Vec<f64> = geo.iter().filter(|g| matches!(g, Geo::Clothoid { .. })).map(Geo::length).collect();
        assert!(curves.len() == 1 && (curves[0] / 0.010954 - 1.0).abs() < 5e-3, "{curves:?}, 10.95 mm by hand");
        assert!((reported / 1e-4 - 1.0).abs() < 1e-3 && reported <= tolerance, "{reported} m reported, 0.1 mm by hand");
        let [end, dir, _] = geo[geo.len() - 1].at(geo[geo.len() - 1].length());
        assert!(distance(&end, &[0.4, 0.25]) < 1e-12 && distance(&dir, &[0.0, 1.0]) < 1e-12, "{end:?} {dir:?}");
        assert_no_steps(&geo, "three quarters");
        assert_blend_runs_within(&axes, &start, &pieces, tolerance, &[0.4, 0.25], "three quarters");
    }

    /// Places with the same acceleration bound A = 10 m/s^2 both ways at any speed and a top of 1 m/s, except one of
    /// 0.5 m/s in the middle, 10 mm apart.
    fn dip(jerk: f64) -> (Vec<Place>, Vec<f64>) {
        let places: Vec<Place> = (0..41)
            .map(|k| Place {
                rows: vec![[1.0, 0.0, 0.0, -10.0, 10.0]],
                turn: Vec::new(),
                curves: Arc::from([]),
                guide: Vec::new(),
                bends: Vec::new(),
                top: if k == 20 { 0.5 } else { 1.0 },
            })
            .collect();
        let x = smoothed(&places, 0.01, jerk, 1.0);
        (places, x)
    }

    /// The speed curve a piece follows around a slow place. With a jerk too high to matter, the squared speed may change
    /// by 2 * 0.01 m * 0.9 * 10 m/s^2 = 0.18 m^2/s^2 from place to place, so from the dip's 0.25 it rises 0.43, 0.61,
    /// 0.79, 0.97 to the tops' 1 on either side. With 50 m/s^3 the bends hold too: |x'' spacing^2| <= 2 * 0.5 * 50 *
    /// 0.01^2 / v, never above a top, never below the dip.
    #[test]
    fn the_speed_curve_comes_down_to_a_slow_place_within_its_acceleration_and_jerk() {
        let (_, x) = dip(1e9);
        for (i, want) in [0.25, 0.43, 0.61, 0.79, 0.97, 1.0, 1.0].iter().enumerate() {
            assert!((x[20 + i] - want).abs() < 1e-12 && (x[20 - i] - want).abs() < 1e-12, "{i}: {x:?}");
        }
        let (places, x) = dip(50.0);
        let lowest = x.iter().fold(f64::INFINITY, |a: f64, b| a.min(*b));
        assert!((lowest - 0.25).abs() < 1e-12 && x.iter().zip(&places).all(|(x, p)| *x <= p.top * p.top), "{x:?}");
        for k in 1..40 {
            let bend = (x[k - 1] + x[k + 1] - 2.0 * x[k]).abs() * x[k].sqrt() / (2.0 * 0.5 * 50.0 * 1e-4);
            let slope = (x[k + 1] - x[k]).abs() / (2.0 * 0.01 * 0.9 * 10.0);
            assert!(bend <= 1.0 + 1e-4 && slope <= 1.0 + 1e-9, "at {k}: bend {bend}, slope {slope}, {x:?}");
        }
        // The jerk makes the dip wider than the acceleration alone: the curve is lower beside it.
        assert!(x[21] < 0.43 - 0.01 && x[24] < 0.97, "{x:?}");
    }

    /// A right-angled corner rounded within a tolerance e of 1 mm (`round_corners`): the curve's middle is 3 d sin(90°) /
    /// 16 = 3 d / 16 off each line, so d = 16 e / 3 = 5.333 mm, and there it turns at a radius of sqrt(2) d / 3 = 16
    /// sqrt(2) e / 9 = 2.514 mm (`corner_curve`: moving at (ta + tb) / 2, |.| = 1 / sqrt(2), turning at 3 (tb - ta) /
    /// (4 d), |.| = 3 sqrt(2) / (4 d), at right angles: k = 3 sqrt(2) / (2 d)). The lines are cut back by d, the planned
    /// path keeps within the tolerance of the two lines and no longer stops at the corner.
    #[test]
    fn a_right_angled_corner_is_rounded_within_the_tolerance_at_the_hand_calculated_radius() {
        let (axes, start, e) = (axes(), [0.1, 0.1], 1e-3);
        let pieces = parse("line,0.5,0.1\nline,0.5,0.5").unwrap();
        let rounded = Tolerance { path: 0.0, corner: e };
        let (geo, reported) = geometry(&axes, &start, &pieces, rounded, &axis_speed(&axes), ARC_SLACK).unwrap();
        let (d, radius) = (16.0 * e / 3.0, 16.0 * 2f64.sqrt() * e / 9.0);
        assert!((d - 5.3333e-3).abs() < 1e-7 && (radius - 2.5142e-3).abs() < 1e-7);
        assert!((reported - e).abs() < 1e-12, "{reported} m reported");
        let [Geo::Line { length: first, .. }, curve @ Geo::Spline { .. }, Geo::Line { from, length: last, .. }] =
            &geo[..]
        else {
            panic!("a line, the corner's curve and a line");
        };
        assert!((first - (0.4 - d)).abs() < 1e-12 && (last - (0.4 - d)).abs() < 1e-12, "{first} m, {last} m");
        let ([begin, ..], [end, ..]) = (curve.at(0.0), curve.at(curve.length()));
        assert!(distance(&begin, &[0.5 - d, 0.1]) < 1e-12 && distance(&end, from) < 1e-12, "{begin:?} {end:?}");
        let [middle, _, turning] = curve.at(0.5 * curve.length());
        let off = (0.5 - middle[0]).min(middle[1] - 0.1);
        assert!((off - e).abs() < 1e-12 && (1.0 / norm(&turning) - radius).abs() < 1e-9, "{off} m, {turning:?}");
        // Planned within the axis limits: never off the two lines by more than the tolerance, and moving at every
        // cycle between its start and its end; stopped at the corner, one cycle moves the tool by a few 1e-8 m only.
        let step = |points: &[(f64, Vec<f64>)]| {
            points[5..points.len() - 5].windows(2).map(|w| distance(&w[0].1, &w[1].1)).fold(f64::INFINITY, f64::min)
        };
        let points = plan(&axes, &start, &pieces, 1e-3, rounded, ARC_SLACK).unwrap();
        let furthest = points.iter().map(|(_, p)| (0.5 - p[0]).min(p[1] - 0.1).max(0.0)).fold(0.0, f64::max);
        let stopped = plan(&axes, &start, &pieces, 1e-3, 0.0, ARC_SLACK).unwrap();
        assert!(furthest <= e * (1.0 + 1e-9) && step(&points) > 1e-6 && step(&stopped) < 1e-7, "{furthest} m off");
    }

    /// An arc on four axes: on the first three a quarter circle of 0.1 m from +x to +y (L = 0.05 pi = 0.15708 m), and the
    /// fourth (a turn) carried from 0 to 1 rad evenly with the distance along it. The piece counts the distance on all
    /// four, sqrt(L^2 + 1^2) = 1.01226, so at a share w of it the arc is at the angle w pi / 2 and the fourth axis at w:
    /// the fourth axis is linear in the arc's length. The direction has length 1, as on a line.
    #[test]
    fn a_four_axis_arc_keeps_its_fourth_axis_linear_in_the_arc_length() {
        let pieces = parse("arc,0.0,0.1,0.0,1.0,0.0,0.0,0.0,ccw").unwrap();
        assert_eq!(pieces[0], Piece::Arc { to: vec![0.0, 0.1, 0.0, 1.0], centre: vec![0.0; 3], clockwise: false });
        let (geo, off) = given(&[0.1, 0.0, 0.0, 0.0], &pieces, ARC_SLACK).unwrap();
        let length = (0.05f64 * std::f64::consts::PI).hypot(1.0);
        assert!((length - 1.01226).abs() < 1e-5 && (geo[0].length() - length).abs() < 1e-12 && off < 1e-12);
        for k in 0..=8 {
            let w = k as f64 / 8.0;
            let [pose, dir, _] = geo[0].at(w * length);
            let angle = pose[1].atan2(pose[0]);
            assert!(
                (angle - w * std::f64::consts::FRAC_PI_2).abs() < 1e-12
                    && (pose[0].hypot(pose[1]) - 0.1).abs() < 1e-12
                    && pose[2].abs() < 1e-12
                    && (pose[3] - w).abs() < 1e-12
                    && (norm(&dir) - 1.0).abs() < 1e-12,
                "at {w}: {pose:?} {dir:?}"
            );
        }
        // Planned within the limits of four axes, it ends where it was given.
        let axis =
            AxisInfo { id: "", key: "", unit: "m", min: -1.0, max: 2.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 100.0 };
        let points = plan(
            &[axis.clone(), axis.clone(), axis.clone(), axis],
            &[0.1, 0.0, 0.0, 0.0],
            &pieces,
            1e-3,
            0.0,
            ARC_SLACK,
        )
        .unwrap();
        assert!(distance(&points[points.len() - 1].1, &[0.0, 0.1, 0.0, 1.0]) < 1e-12);
    }

    /// Places of a made-up piece of `count` + 1 places: an acceleration limit of 10 m/s^2 and the top speed `top` per
    /// share of the piece; with a jerk along the path of 1000 m/s^3.
    fn made_up(length: f64, count: usize, top: impl Fn(f64) -> f64) -> PieceLimits {
        let places = (0..=count)
            .map(|k| Place {
                rows: vec![[1.0, 0.0, 0.0, -10.0, 10.0]],
                turn: Vec::new(),
                curves: Arc::from([]),
                guide: Vec::new(),
                bends: Vec::new(),
                top: top(k as f64 / count as f64),
            })
            .collect();
        piece_limits_of(places, length / count as f64, 1000.0).unwrap()
    }

    /// Two arcs that meet in their direction, of 0.2 m and 0.1 m radius turning the same way: at the junction the
    /// curvature steps by 1 / 0.1 - 1 / 0.2 = 5 1/m across the path (along x), and over a profile cycle of 1 ms half the
    /// x axis's jerk limit of 100 m/s^3 allows v^2 * 5 = 0.5 * 100 * 1e-3, v = 0.1 m/s (`junction_speed`). The arcs' places
    /// allow 2 m/s, but for the far end of the second arc, which allows 0.05 m/s: the junction is passed at the 0.1 m/s
    /// the junction itself allows, no longer at the 0.05 m/s of the slowest place of either piece.
    #[test]
    fn the_junction_of_two_arcs_is_passed_at_the_hand_calculated_speed() {
        use std::f64::consts::{FRAC_PI_2, PI};
        let wide = Geo::Arc { centre: [0.0, 0.0], radius: 0.2, from_angle: -FRAC_PI_2, turn: 1.0, length: 0.1 * PI };
        let tight = Geo::Arc { centre: [0.1, 0.0], radius: 0.1, from_angle: 0.0, turn: 1.0, length: 0.05 * PI };
        assert!(distance(&wide.at(wide.length())[0], &tight.at(0.0)[0]) < 1e-12);
        let geo = [wide, tight];
        let limits = [made_up(0.1 * PI, 40, |_| 2.0), made_up(0.05 * PI, 40, |w| if w < 0.5 { 2.0 } else { 0.05 })];
        let by_hand = (0.5f64 * 100.0 * 1e-3 / 5.0).sqrt();
        assert!((junction_speed(&geo[0], &geo[1], &axes(), 1e-3) - by_hand).abs() < 1e-15);
        let speed = junction_speeds(&axes(), &geo, &limits, 1e-3, true);
        let before = junction_speeds(&axes(), &geo, &limits, 1e-3, false);
        assert!((speed[1] - 0.1).abs() < 1e-12 && (before[1] - 0.05).abs() < 1e-12, "{speed:?}, before {before:?}");
        // And the plan runs: each piece is timed from that speed to the next.
        for i in 0..2 {
            piece_knots(&limits[i], geo[i].length(), [speed[i], speed[i + 1]], 1e-3, true).unwrap();
        }
    }

    /// A line at up to 1 m/s into a second line whose places allow 1 m/s at its start, falling evenly to 0.3 m/s at its
    /// end, from rest to rest. Joined on a cruise at no acceleration (and the junction no faster than the slowest place
    /// of either piece, 0.3 m/s) the second line is crossed at 0.3 m/s; joined on the speed curve the plan slows down
    /// along the curve and is quicker.
    #[test]
    fn a_path_without_the_forced_cruise_is_quicker_than_with_it() {
        let line = |x0: f64| Geo::Line { from: vec![x0, 0.1], dir: vec![1.0, 0.0], length: 0.5 };
        let geo = [line(0.0), line(0.5)];
        let limits = [made_up(0.5, 50, |_| 1.0), made_up(0.5, 50, |w| 1.0 - 0.7 * w)];
        let duration = |joins: bool| {
            let speed = junction_speeds(&axes(), &geo, &limits, 1e-3, joins);
            let pieces = (0..2).map(|i| piece_knots(&limits[i], 0.5, [speed[i], speed[i + 1]], 1e-3, joins).unwrap());
            (speed[1], pieces.map(|k| k[k.len() - 1][0]).sum::<f64>())
        };
        let ((joined, quick), (cruised, slow)) = (duration(true), duration(false));
        assert!(
            (cruised - 0.3).abs() < 1e-9 && joined > 0.9 && quick < slow - 0.05,
            "{joined} m/s {quick} s against {cruised} m/s {slow} s"
        );
    }

    /// The lowest of a range of values from the table of lowest values over powers of two, against looking at each.
    #[test]
    fn the_lowest_over_a_range_is_the_lowest_of_its_values() {
        let values: Vec<f64> = (0..37).map(|k| ((k * 7919) % 101) as f64).collect();
        let table = lowest_ahead(&values);
        for i in 0..values.len() {
            for j in i..values.len() {
                let size = (usize::BITS - 1 - (j - i + 1).leading_zeros()) as usize;
                let lowest = table[size][i].min(table[size][j + 1 - (1 << size)]);
                assert_eq!(lowest, values[i..=j].iter().fold(f64::INFINITY, |a, b| a.min(*b)), "{i}..={j}");
            }
        }
    }
}
