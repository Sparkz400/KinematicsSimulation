//! Programs read into path pieces (`path::Piece`) for the path planner: a subset of G-code (DIN 66025 / ISO 6983), robot
//! moves as CSV, and subsets of KUKA KRL and ABB RAPID. `read` picks the reader by the file's extension.
//!
//! G-code (`.nc`, `.ngc`, `.gcode`, `.g`, `.cnc`, `.tap`), one block per line:
//! - G0 rapid, G1 line, G2 clockwise and G3 counterclockwise arc (seen from the plus side of the plane's normal),
//!   centre by I, J, K (from the start, always incremental) or radius R (negative: the longer of the two arcs);
//! - G17 (XY plane), G18 (ZX), G19 (YZ); G20 inch, G21 mm (the default); G90 absolute (the default), G91 incremental;
//! - F the feed in units per minute, needed before the first G1, G2 or G3: the speed along the path. Each run of moves
//!   at one feed is planned on its own, its speed along the path capped at the feed (`path::plan_at`), with a stop
//!   between runs; G0 runs at the axes' limits;
//! - X, Y, Z (and A, B, C in degrees) move the axes of the same name (ids x, y, z, a, b, c); the start is where the
//!   machine stands, an axis a block does not name stays;
//! - taken and without effect: N, O, M, S, T words, G40, G49, G54, G80, G94; comments in ( ) and after ;; % lines;
//!   M2 and M30 end the program. Every other word or G code is refused with its line, and so is an arc that changes the
//!   plane's third axis (a helix), whose start and end lie at different distances from its centre, or a word for an axis
//!   the machine does not have.
//!
//! Robot moves as CSV (any other extension, when a line starts with Move): `MoveL,x,y,z` and `MoveJ,x,y,z` straight to
//! the pose, `MoveC,via...,end...` on the circle through the via point to the end; one value per axis in the machine's
//! units (m, rad), as a path file has them; at the axes' limits.
//!
//! KRL (`.src`): `PTP {X 100, Y 0, Z 500}`, `LIN {...}`, `CIRC {aux}, {end}` with mm and degrees, components left out
//! kept from the point before; A, B, C, S, T, E1... move axes of those names where the machine has them and are left out
//! otherwise. Where the machine's axes are x, y, z and a rotation vector rx, ry, rz (an arm in tool space), A, B, C are
//! the orientation (R = Rz(A) Ry(B) Rx(C), KUKA's) and set rx, ry, rz. `$VEL.CP = v` (m/s) is the speed along the path
//! of the LIN and CIRC moves after it; a PTP runs at the axes' limits.
//!
//! RAPID (`.mod`, `.prg`): `MoveL`, `MoveJ` and `MoveC` to robtargets given in place (`[[x, y, z], [q1, q2, q3, q4],
//! ...]`) or declared (`CONST robtarget p10 := [[...]];`), positions in mm; where the machine has rx, ry, rz the
//! orientation quaternion (q1 the scalar part) sets them. The speed: `v100` (the predefined speeddata, mm/s), `vmax`
//! (the axes' limits), a speeddata given in place (`[150, 500, 5000, 1000]`, its TCP speed in mm/s) or declared (`CONST
//! speeddata vfast := [...];`); the zone is left out. In both every other line or statement without a move is skipped;
//! other move instructions (PTP_REL, SPLINE, MoveAbsJ, named KRL points) are refused. A PTP or MoveJ is a straight line
//! in the machine's axes, as a MoveL, at the move's speed along it.
//! shortcut: a speed is along the path over all axes, as a path's length is (on an arm in tool space the rotation
//! vector's radians count with the metres); a robot's TCP speed counts the position alone. Split the speed into the
//! TCP's and the orientation's (RAPID's v_ori) where reorienting moves have to run at the robot's own speed.

use crate::path::Piece;
use sim_core::AxisInfo;

/// Moves at one feed: the pieces, and the speed along the path they are capped at (axis units per second, over all
/// axes, as `path::plan_at` takes it), `None` for the axes' limits.
#[derive(Debug, Clone, PartialEq)]
pub struct Run {
    pub feed: Option<f64>,
    pub pieces: Vec<Piece>,
}

/// A line of a program that cannot be read: its number (from 1), a translation key and the word or detail.
#[derive(Debug, Clone, PartialEq)]
pub struct ProgramError {
    pub line: usize,
    pub key: &'static str,
    pub detail: String,
}

type Result<T> = std::result::Result<T, ProgramError>;

fn fail<T>(line: usize, key: &'static str, detail: impl Into<String>) -> Result<T> {
    Err(ProgramError { line, key, detail: detail.into() })
}

/// The program in `text`, read by the file's extension (see the module): G-code, KRL, RAPID, robot moves as CSV, else a
/// path file (`path::parse`, one run at the axes' limits). Planned from `start`, on a machine with `axes`.
pub fn read(file_name: &str, text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    let extension = file_name.rsplit_once('.').map_or("", |(_, e)| e).to_ascii_lowercase();
    match extension.as_str() {
        "nc" | "ngc" | "gcode" | "g" | "cnc" | "tap" => gcode(text, axes, start),
        "src" => krl(text, axes, start),
        "mod" | "prg" => rapid(text, axes, start),
        _ if text.lines().any(|l| l.trim().to_ascii_lowercase().starts_with("move")) => robot_csv(text, axes, start),
        _ => crate::path::parse(text).map(|pieces| vec![Run { feed: None, pieces }]).map_err(|line| ProgramError {
            line: 0,
            key: "program-error-word",
            detail: line,
        }),
    }
}

/// The index of the axis a program letter names (X: id "x"), if the machine has it.
fn axis_of(axes: &[AxisInfo], letter: char) -> Option<usize> {
    axes.iter().position(|a| a.id.len() == 1 && a.id.eq_ignore_ascii_case(&letter.to_string()))
}

/// Whether the arc whose plane is spanned by axes `a` then `b` (a to the right, b up, seen from the plus side of the
/// normal a x b) is clockwise as `path` reads it: on two axes seen with the first to the right and the second up, on
/// three from +z, for the x-z plane from -y, for the y-z plane from +x (by position: the first axis is x). `turning` is
/// -1 for clockwise seen from a x b, +1 for counterclockwise.
fn path_clockwise(a: usize, b: usize, turning: f64) -> bool {
    // a x b = sign e_k; the path's viewing normal is +e_2, -e_1 or +e_0 for k = 2, 1, 0 (two axes: k = 2).
    let cyclic = matches!((a, b), (0, 1) | (1, 2) | (2, 0));
    let k = 3 - a - b;
    let viewed = if k == 1 { -1.0 } else { 1.0 };
    turning * if cyclic { 1.0 } else { -1.0 } * viewed < 0.0
}

fn cross(u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

/// The arc from `from` through `via` to `to` (two or three axes, by position): its centre and whether `path` reads it
/// clockwise. `None` where the three points are on one line.
fn arc_through(from: &[f64], via: &[f64], to: &[f64]) -> Option<(Vec<f64>, bool)> {
    let n = from.len();
    if !(2..=3).contains(&n) {
        return None;
    }
    let p = |x: &[f64]| [x[0], x[1], if n == 3 { x[2] } else { 0.0 }];
    let (s, v, e) = (p(from), p(via), p(to));
    let (u, w) = ([v[0] - s[0], v[1] - s[1], v[2] - s[2]], [e[0] - s[0], e[1] - s[1], e[2] - s[2]]);
    let normal = cross(u, w);
    let nn = normal.iter().map(|x| x * x).sum::<f64>();
    let scale = u.iter().chain(&w).map(|x| x * x).sum::<f64>();
    if nn <= 1e-18 * scale * scale {
        return None;
    }
    // Circumcentre: s + ((|u|^2 w - |w|^2 u) x n) / (2 |n|^2).
    let (uu, ww) = (u.iter().map(|x| x * x).sum::<f64>(), w.iter().map(|x| x * x).sum::<f64>());
    let m = cross([uu * w[0] - ww * u[0], uu * w[1] - ww * u[1], uu * w[2] - ww * u[2]], normal);
    let centre: Vec<f64> = (0..n).map(|i| s[i] + m[i] / (2.0 * nn)).collect();
    // The travel turns about `normal`; the path's viewing normal is +z, else -y, else +x.
    let viewing = if normal[2].abs() > 1e-12 * nn.sqrt() {
        [0.0, 0.0, 1.0]
    } else if normal[1].abs() > 1e-12 * nn.sqrt() {
        [0.0, -1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let along: f64 = normal.iter().zip(viewing).map(|(a, b)| a * b).sum();
    Some((centre, along < 0.0))
}

/// Adds `piece` to the last run where that moves at `feed`, else to a new run.
fn push(runs: &mut Vec<Run>, feed: Option<f64>, piece: Piece) {
    match runs.last_mut() {
        Some(run) if run.feed == feed => run.pieces.push(piece),
        _ => runs.push(Run { feed, pieces: vec![piece] }),
    }
}

/// G-code (see the module).
pub fn gcode(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    let mut pos = start.to_vec();
    let (mut absolute, mut inch, mut plane, mut motion, mut feed) = (true, false, 17, None::<u32>, None::<f64>);
    let mut runs = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = n + 1;
        // Comments in brackets and after a semicolon.
        let mut clean = String::new();
        let mut depth = 0;
        for c in raw.chars() {
            match c {
                '(' => depth += 1,
                ')' if depth > 0 => depth -= 1,
                ';' if depth == 0 => break,
                c if depth == 0 && !c.is_whitespace() => clean.push(c.to_ascii_uppercase()),
                _ => {}
            }
        }
        if clean.is_empty() || clean.starts_with('%') {
            continue;
        }
        // Words: a letter and a number.
        let mut words: Vec<(char, f64, &str)> = Vec::new();
        let mut rest = clean.as_str();
        while let Some(letter) = rest.chars().next() {
            let end = rest[1..].find(|c: char| c.is_ascii_alphabetic()).map_or(rest.len(), |i| i + 1);
            let (word, number) = (&rest[..end], &rest[1..end]);
            match number.parse::<f64>() {
                Ok(v) if letter.is_ascii_alphabetic() && v.is_finite() => words.push((letter, v, word)),
                _ => return fail(line, "program-error-word", word),
            }
            rest = &rest[end..];
        }
        let mut moving = false;
        let mut ends = false;
        let (mut target, mut centre_offset, mut radius) = (pos.clone(), [None::<f64>; 3], None::<f64>);
        // Modal words first: units and distances apply to the whole block.
        for &(letter, v, word) in &words {
            if letter == 'G' {
                match v {
                    20.0 => inch = true,
                    21.0 => inch = false,
                    90.0 => absolute = true,
                    91.0 => absolute = false,
                    17.0 | 18.0 | 19.0 => plane = v as u32,
                    0.0 | 1.0 | 2.0 | 3.0 => motion = Some(v as u32),
                    40.0 | 49.0 | 54.0 | 80.0 | 94.0 => {}
                    _ => return fail(line, "program-error-word", word),
                }
            }
        }
        let length = if inch { 0.0254 } else { 0.001 };
        for &(letter, v, word) in &words {
            match letter {
                'G' | 'N' | 'O' | 'S' | 'T' => {}
                'M' => ends |= v == 2.0 || v == 30.0,
                'F' if v > 0.0 => feed = Some(v * length / 60.0),
                'X' | 'Y' | 'Z' | 'A' | 'B' | 'C' => {
                    let Some(i) = axis_of(axes, letter) else { return fail(line, "program-error-axis", word) };
                    let scale = if axes[i].unit == "rad" { std::f64::consts::PI / 180.0 } else { length };
                    target[i] = if absolute { v * scale } else { target[i] + v * scale };
                    moving = true;
                }
                'I' | 'J' | 'K' => centre_offset[(letter as u8 - b'I') as usize] = Some(v * length),
                'R' => radius = Some(v * length),
                _ => return fail(line, "program-error-word", word),
            }
        }
        if moving {
            let Some(kind) = motion else { return fail(line, "program-error-motion", clean) };
            let rate = if kind == 0 {
                None
            } else {
                Some(feed.ok_or_else(|| ProgramError { line, key: "program-error-feed", detail: clean.clone() })?)
            };
            let piece = if kind < 2 {
                Piece::Line(target.clone())
            } else {
                arc_block(line, axes, &pos, &target, plane, kind == 2, centre_offset, radius)?
            };
            push(&mut runs, rate, piece);
            pos = target;
        }
        if ends {
            break;
        }
    }
    Ok(runs)
}

/// The arc of a G2 (`clockwise`) or G3 block in `plane` from `from` to `to`, its centre from the offsets I, J, K or
/// the radius R.
#[allow(clippy::too_many_arguments)]
fn arc_block(
    line: usize,
    axes: &[AxisInfo],
    from: &[f64],
    to: &[f64],
    plane: u32,
    clockwise: bool,
    offset: [Option<f64>; 3],
    radius: Option<f64>,
) -> Result<Piece> {
    // The plane's axes, the first to the right and the second up seen from the plus side of the normal, and their
    // offset letters.
    let (a, b) = match plane {
        17 => ('X', 'Y'),
        18 => ('Z', 'X'),
        _ => ('Y', 'Z'),
    };
    let letters = format!("G{plane}");
    let (Some(ia), Some(ib)) = (axis_of(axes, a), axis_of(axes, b)) else {
        return fail(line, "program-error-axis", letters);
    };
    // Every other axis stays: a helix, or a turn of A, B, C on the way, is not an arc of the path.
    if (0..from.len()).any(|i| i != ia && i != ib && (to[i] - from[i]).abs() > 1e-12) {
        return fail(line, "program-error-helix", letters);
    }
    let (sa, sb, ea, eb) = (from[ia], from[ib], to[ia], to[ib]);
    let (ca, cb) = match radius {
        Some(r) => {
            let (da, db) = (ea - sa, eb - sb);
            let d = (da * da + db * db).sqrt();
            if d == 0.0 || r.abs() < d / 2.0 * (1.0 - 1e-9) {
                return fail(line, "program-error-arc", format!("R{}", r * 1e3));
            }
            let h = (r * r - d * d / 4.0).max(0.0).sqrt();
            // Clockwise with R > 0 (the shorter arc) has its centre to the right of the chord.
            let side = if clockwise == (r > 0.0) { 1.0 } else { -1.0 };
            ((sa + ea) / 2.0 + side * h * db / d, (sb + eb) / 2.0 - side * h * da / d)
        }
        None => {
            let letter = |c: char| offset[(c as u8 - b'X') as usize].unwrap_or(0.0);
            (sa + letter(a), sb + letter(b))
        }
    };
    let (r0, r1) = ((sa - ca).hypot(sb - cb), (ea - ca).hypot(eb - cb));
    if r0 == 0.0 || (r0 - r1).abs() > 1e-4 * r0 + 1e-6 {
        return fail(line, "program-error-arc", format!("{:.4} / {:.4} mm", r0 * 1e3, r1 * 1e3));
    }
    let mut centre = from.to_vec();
    (centre[ia], centre[ib]) = (ca, cb);
    Ok(Piece::Arc { to: to.to_vec(), centre, clockwise: path_clockwise(ia, ib, if clockwise { -1.0 } else { 1.0 }) })
}

/// Robot moves as CSV (see the module).
pub fn robot_csv(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    let mut pos = start.to_vec();
    let mut runs = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = n + 1;
        let raw = raw.trim();
        if raw.is_empty() || raw.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = raw.split(',').map(str::trim).collect();
        let numbers: Option<Vec<f64>> =
            fields[1..].iter().map(|v| v.parse().ok().filter(|x: &f64| x.is_finite())).collect();
        let Some(numbers) = numbers else { return fail(line, "program-error-word", raw) };
        let k = axes.len();
        let piece = match fields[0].to_ascii_lowercase().as_str() {
            "movel" | "movej" if numbers.len() == k => Piece::Line(numbers),
            "movec" if numbers.len() == 2 * k => circle(line, &pos, &numbers[..k], &numbers[k..])?,
            _ => return fail(line, "program-error-word", raw),
        };
        pos = match &piece {
            Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to) => to.clone(),
        };
        push(&mut runs, None, piece);
    }
    Ok(runs)
}

/// The arc from `from` through `via` to `to`; on more than three axes on the first three, the others carried along.
fn circle(line: usize, from: &[f64], via: &[f64], to: &[f64]) -> Result<Piece> {
    let n = from.len().min(3);
    match arc_through(&from[..n], &via[..n], &to[..n]) {
        Some((centre, clockwise)) => Ok(Piece::Arc { to: to.to_vec(), centre, clockwise }),
        None => fail(line, "program-error-arc", format!("{via:?}")),
    }
}

/// The pose a robot position gives (`components`: name and value, mm and degrees): `from` with the axes of those
/// names set; names the machine has no axis of are left out.
fn robot_pose(axes: &[AxisInfo], from: &[f64], components: &[(String, f64)]) -> Vec<f64> {
    let mut pose = from.to_vec();
    for (name, v) in components {
        if let Some(i) = axes.iter().position(|a| a.id.eq_ignore_ascii_case(name)) {
            pose[i] = v * if axes[i].unit == "rad" { std::f64::consts::PI / 180.0 } else { 1e-3 };
        }
    }
    pose
}

/// The indices of the axes rx, ry, rz, where the machine's tool has an orientation as a rotation vector.
fn rotation_axes(axes: &[AxisInfo]) -> Option<[usize; 3]> {
    let at = |id: &str| axes.iter().position(|a| a.id == id);
    Some([at("rx")?, at("ry")?, at("rz")?])
}

/// KUKA's A, B, C (degrees) of a rotation vector: R = Rz(A) Ry(B) Rx(C).
fn abc_of(r: [f64; 3]) -> [f64; 3] {
    let m = crate::rigid::exp(r);
    [m[1][0].atan2(m[0][0]), (-m[2][0]).clamp(-1.0, 1.0).asin(), m[2][1].atan2(m[2][2])].map(f64::to_degrees)
}

/// The rotation vector of KUKA's A, B, C (degrees).
fn rotation_of_abc([a, b, c]: [f64; 3]) -> [f64; 3] {
    crate::rigid::log(&crate::rigid::rpy([c.to_radians(), b.to_radians(), a.to_radians()]))
}

/// KUKA KRL (see the module).
pub fn krl(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    let mut pos = start.to_vec();
    let mut runs = Vec::new();
    // The orientation as KUKA's A, B, C, where the machine has rx, ry, rz: components left out are kept.
    let rotation = rotation_axes(axes);
    let mut abc = rotation.map(|r| abc_of(r.map(|i| start[i])));
    let mut speed = None;
    for (n, raw) in text.lines().enumerate() {
        let line = n + 1;
        let code = raw.split(';').next().unwrap_or("").trim();
        if let Some(value) = code.to_ascii_uppercase().strip_prefix("$VEL.CP") {
            let value = value.trim().strip_prefix('=').map(str::trim).and_then(|v| v.parse::<f64>().ok());
            speed = Some(value.filter(|v| *v > 0.0 && v.is_finite()).ok_or_else(|| ProgramError {
                line,
                key: "program-error-word",
                detail: code.to_string(),
            })?);
            continue;
        }
        let keyword: String = code.chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '_').collect();
        let keyword = keyword.to_ascii_uppercase();
        if !matches!(keyword.as_str(), "PTP" | "LIN" | "CIRC") {
            if ["PTP_", "LIN_", "CIRC_", "SPLINE", "SPL", "SLIN", "SCIRC", "SPTP"]
                .iter()
                .any(|k| keyword.starts_with(k))
            {
                return fail(line, "program-error-word", code);
            }
            continue;
        }
        // The aggregates in braces: {X 100, Y 0, Z 500} or {E6POS: X 100, ...}.
        let mut aggregates = Vec::new();
        let mut rest = &code[keyword.len()..];
        while let Some(open) = rest.find('{') {
            let Some(close) = rest[open..].find('}') else { return fail(line, "program-error-word", code) };
            let inner = &rest[open + 1..open + close];
            let inner = inner.split_once(':').map_or(inner, |(_, r)| r);
            let mut components = Vec::new();
            for part in inner.split(',').map(str::trim).filter(|p| !p.is_empty()) {
                let (name, value) = part.split_once(char::is_whitespace).unwrap_or((part, ""));
                let Ok(v) = value.trim().parse::<f64>() else { return fail(line, "program-error-word", part) };
                components.push((name.to_string(), v));
            }
            aggregates.push(components);
            rest = &rest[open + close + 1..];
        }
        // A pose with its orientation from A, B, C where the machine has rx, ry, rz (the aux point's is not used).
        let mut oriented = |from: &[f64], components: &[(String, f64)]| {
            let mut pose = robot_pose(axes, from, components);
            if let (Some(rotation), Some(abc)) = (rotation, abc.as_mut()) {
                for (name, v) in components {
                    if let Some(k) = ["A", "B", "C"].iter().position(|c| name.eq_ignore_ascii_case(c)) {
                        abc[k] = *v;
                    }
                }
                for (i, r) in rotation.into_iter().zip(rotation_of_abc(*abc)) {
                    pose[i] = r;
                }
            }
            pose
        };
        let (piece, feed) = match (keyword.as_str(), aggregates.as_slice()) {
            ("PTP", [end]) => (Piece::Line(oriented(&pos, end)), None),
            ("LIN", [end]) => (Piece::Line(oriented(&pos, end)), speed),
            ("CIRC", [aux, end]) => {
                let via = robot_pose(axes, &pos, aux);
                let to = oriented(&via, end);
                (circle(line, &pos, &via, &to)?, speed)
            }
            _ => return fail(line, "program-error-named", code),
        };
        pos = match &piece {
            Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to) => to.clone(),
        };
        push(&mut runs, feed, piece);
    }
    Ok(runs)
}

/// ABB RAPID (see the module).
pub fn rapid(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    // Statements end at ';'; a '!' starts a comment to the end of its line. Each statement keeps the line it starts on.
    let mut statements: Vec<(usize, String)> = Vec::new();
    let mut current = (0, String::new());
    for (n, raw) in text.lines().enumerate() {
        let code = raw.split('!').next().unwrap_or("");
        for (i, piece) in code.split(';').enumerate() {
            if current.1.trim().is_empty() {
                current.0 = n + 1;
            }
            if i > 0 {
                statements.push(std::mem::take(&mut current));
                current.0 = n + 1;
            }
            current.1.push_str(piece);
            current.1.push(' ');
        }
    }
    // The numbers of a bracket of numbers, `[a, b, ...]`.
    let numbers = |text: &str| -> Option<Vec<f64>> {
        let inner = text.trim().strip_prefix('[')?;
        inner.split(']').next()?.split(',').map(|v| v.trim().parse().ok().filter(|v: &f64| v.is_finite())).collect()
    };
    // The [x, y, z] of a robtarget (mm), and its orientation [q1, q2, q3, q4] as "q1".."q4" where it has one.
    let position = |target: &str| -> Option<Vec<(String, f64)>> {
        let inner = target.trim().strip_prefix('[')?;
        let place = numbers(inner)?;
        if place.len() != 3 {
            return None;
        }
        let mut out: Vec<(String, f64)> = ["x", "y", "z"].iter().map(|s| s.to_string()).zip(place).collect();
        let rest = inner.split_once(']').map_or("", |(_, r)| r).trim_start().trim_start_matches(',');
        if let Some(q) = numbers(rest).filter(|q| q.len() == 4) {
            out.extend(["q1", "q2", "q3", "q4"].iter().map(|s| s.to_string()).zip(q));
        }
        Some(out)
    };
    let rotation = rotation_axes(axes);
    // A robtarget's pose: its position, and its orientation where the machine has rx, ry, rz.
    let oriented = |from: &[f64], components: &[(String, f64)]| {
        let mut pose = robot_pose(axes, from, components);
        let q: Vec<f64> = components.iter().filter(|(n, _)| n.starts_with('q')).map(|(_, v)| *v).collect();
        let size = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        if let (Some(rotation), true) = (rotation, q.len() == 4 && size > 0.0) {
            let r = crate::rigid::from_quaternion([q[0], q[1], q[2], q[3]].map(|v| v / size));
            for (i, r) in rotation.into_iter().zip(r) {
                pose[i] = r;
            }
        }
        pose
    };
    let mut named: std::collections::HashMap<String, Vec<(String, f64)>> = Default::default();
    // Declared speeddata: the TCP speed, m/s.
    let mut speeds: std::collections::HashMap<String, f64> = Default::default();
    let mut pos = start.to_vec();
    let mut runs = Vec::new();
    for (line, statement) in statements {
        let statement = statement.split_whitespace().collect::<Vec<_>>().join(" ");
        let words: Vec<&str> = statement.split(' ').collect();
        if let Some(at) = words.iter().position(|w| w.eq_ignore_ascii_case("speeddata")) {
            let declaration = words[at + 1..].join(" ");
            if let Some((name, value)) = declaration.split_once(":=") {
                let Some(v) = numbers(value).and_then(|v| v.first().copied()).filter(|v| *v > 0.0) else {
                    return fail(line, "program-error-word", statement);
                };
                speeds.insert(name.trim().to_string(), v * 1e-3);
            }
            continue;
        }
        if let Some(at) = words.iter().position(|w| w.eq_ignore_ascii_case("robtarget")) {
            let declaration = words[at + 1..].join(" ");
            if let Some((name, value)) = declaration.split_once(":=") {
                let Some(p) = position(value) else { return fail(line, "program-error-word", statement) };
                named.insert(name.trim().to_string(), p);
            }
            continue;
        }
        // A statement may begin with lines that end without a semicolon (PROC main()): the move is where its word is.
        let Some(at) = words.iter().position(|w| w.to_ascii_lowercase().starts_with("move")) else { continue };
        let (instruction, args_text) = (words[at], words[at + 1..].join(" "));
        // Arguments at the top level, split at commas outside brackets.
        let args_text = args_text.trim();
        let (mut args, mut depth, mut from) = (Vec::new(), 0, 0);
        for (i, c) in args_text.char_indices() {
            match c {
                '[' => depth += 1,
                ']' => depth -= 1,
                ',' if depth == 0 => {
                    args.push(args_text[from..i].trim());
                    from = i + 1;
                }
                _ => {}
            }
        }
        args.push(args_text[from..].trim());
        let target = |arg: &str| -> Result<Vec<(String, f64)>> {
            let arg = arg.split('\\').next().unwrap_or(arg).trim();
            position(arg).or_else(|| named.get(arg).cloned()).ok_or_else(|| ProgramError {
                line,
                key: "program-error-named",
                detail: arg.to_string(),
            })
        };
        // The speed along the path (m/s): `vmax` or none given, the axes' limits.
        let speed = |arg: Option<&&str>| -> Result<Option<f64>> {
            let Some(arg) = arg.map(|a| a.split('\\').next().unwrap_or(a).trim()) else { return Ok(None) };
            if arg.eq_ignore_ascii_case("vmax") {
                return Ok(None);
            }
            let predefined = arg.strip_prefix(['v', 'V']).and_then(|n| n.parse::<f64>().ok());
            let given = numbers(arg).and_then(|v| v.first().copied());
            match predefined.or(given).map(|mm| mm * 1e-3).or_else(|| speeds.get(arg).copied()) {
                Some(v) if v > 0.0 && v.is_finite() => Ok(Some(v)),
                _ => fail(line, "program-error-named", arg),
            }
        };
        let (piece, feed) = match instruction.to_ascii_lowercase().as_str() {
            "movel" | "movej" => (Piece::Line(oriented(&pos, &target(args[0])?)), speed(args.get(1))?),
            "movec" if args.len() >= 2 => {
                let via = robot_pose(axes, &pos, &target(args[0])?);
                let to = oriented(&pos, &target(args[1])?);
                (circle(line, &pos, &via, &to)?, speed(args.get(2))?)
            }
            _ => return fail(line, "program-error-word", instruction.to_string()),
        };
        pos = match &piece {
            Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to) => to.clone(),
        };
        push(&mut runs, feed, piece);
    }
    Ok(runs)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn axis(id: &'static str) -> AxisInfo {
        AxisInfo { id, key: "", unit: "m", min: -1.0, max: 1.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 1000.0 }
    }

    /// Length of a planned path: the sum of its steps.
    fn length(axes: &[AxisInfo], start: &[f64], pieces: &[Piece]) -> f64 {
        let points = crate::path::plan(axes, start, pieces, 1e-3, 0.0, crate::path::ARC_SLACK).unwrap();
        let mut last = start.to_vec();
        let mut total = 0.0;
        for (_, p) in &points {
            total += p.iter().zip(&last).map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
            last = p.clone();
        }
        total
    }

    /// A quarter circle in G-code: from (0, 0) G2 X10 Y10 I10 J0 in mm has its centre at (10, 0) mm, radius 10 mm, and
    /// turns clockwise from 180 to 90 degrees: a quarter, 10 pi / 2 = 15.708 mm long. G3 to the same end goes the other
    /// way round, three quarters, 47.124 mm. R10 gives the same centre (the shorter arc, to the right of the chord for
    /// G2), R-10 the longer arc around (0, 10). Measured on the path the planner runs.
    #[test]
    fn a_gcode_arc_has_its_centre_and_length() {
        let axes = [axis("x"), axis("y")];
        let start = [0.0, 0.0];
        let arc = |text: &str| -> (Vec<f64>, bool) {
            let runs = gcode(text, &axes, &start).unwrap();
            match &runs[0].pieces[0] {
                Piece::Arc { centre, clockwise, .. } => (centre.clone(), *clockwise),
                p => panic!("{p:?}"),
            }
        };
        for (text, centre, turn, long) in [
            ("G21 G17 F600\nG2 X10 Y10 I10 J0", [0.01, 0.0], true, PI / 2.0),
            ("F600\nG3 X10 Y10 I10", [0.01, 0.0], false, 1.5 * PI),
            ("F600\nG2 X10 Y10 R10", [0.01, 0.0], true, PI / 2.0),
            ("F600\nG2 X10 Y10 R-10", [0.0, 0.01], true, 1.5 * PI),
        ] {
            let (c, clockwise) = arc(text);
            assert!(c.iter().zip(centre).all(|(a, b)| (a - b).abs() < 1e-12) && clockwise == turn, "{text}: {c:?}");
            let runs = gcode(text, &axes, &start).unwrap();
            let l = length(&axes, &start, &runs[0].pieces);
            assert!((l - 0.01 * long).abs() < 1e-5, "{text}: {l} m against {}", 0.01 * long);
        }
        // F600 mm/min is 0.01 m/s.
        assert!((gcode("G1 X1 F600", &axes, &start).unwrap()[0].feed.unwrap() - 0.01).abs() < 1e-15);
    }

    /// The other planes and units: G18 on a gantry of x and z: G2 is clockwise seen from +y, so counterclockwise seen
    /// from -y, where x is to the right and z up as the path reads two axes. G20 reads inches: X1 is 25.4 mm. G91 adds.
    #[test]
    fn gcode_planes_units_and_increments() {
        let xz = [axis("x"), axis("z")];
        let runs = gcode("G18 G20 F10\nG2 X1 Z1 I1 K0", &xz, &[0.0, 0.0]).unwrap();
        let Piece::Arc { to, centre, clockwise } = &runs[0].pieces[0] else { panic!() };
        assert!(!clockwise && (to[0] - 0.0254).abs() < 1e-12 && (centre[0] - 0.0254).abs() < 1e-12 && centre[1] == 0.0);
        // Seen from -y, G2 from (0, 0) to (25.4, 25.4) mm around (25.4, 0) is the three-quarter way: 3/4 2 pi 25.4 mm.
        let l = length(&xz, &[0.0, 0.0], &runs[0].pieces);
        assert!((l - 1.5 * PI * 0.0254).abs() < 1e-5, "{l}");
        let runs = gcode("G91 G0 X10\nX10 Z-5", &xz, &[0.1, 0.1]).unwrap();
        assert_eq!(runs[0].feed, None);
        let Piece::Line(end) = &runs[0].pieces[1] else { panic!() };
        assert!((end[0] - 0.12).abs() < 1e-15 && (end[1] - 0.095).abs() < 1e-15, "{end:?}");
    }

    /// What G-code the reader does not take is refused with its line.
    #[test]
    fn unsupported_gcode_is_refused_by_line() {
        let axes = [axis("x"), axis("y"), axis("z")];
        let error = |text: &str| gcode(text, &axes, &[0.0; 3]).unwrap_err();
        assert_eq!(error("G0 X1\nG4 P1"), ProgramError { line: 2, key: "program-error-word", detail: "G4".into() });
        assert_eq!(error("G1 X1").key, "program-error-feed");
        assert_eq!(error("F100\nG2 X10 Y0 Z5 I5").key, "program-error-helix");
        assert_eq!(error("F100\nG2 X10 I3").key, "program-error-arc");
        assert_eq!(gcode("G0 A1", &axes[..2], &[0.0; 2]).unwrap_err().key, "program-error-axis");
        // Comments, line numbers and switching functions pass; M30 ends the program.
        let runs = gcode("%\nN10 G0 X1 (to the side) ; rapid\nM3 S1000\nM30\nG0 X2", &axes, &[0.0; 3]).unwrap();
        assert_eq!(runs, vec![Run { feed: None, pieces: vec![Piece::Line(vec![0.001, 0.0, 0.0])] }]);
    }

    /// Robot programs: MoveC through (0, 1) from (-1, 0) to (1, 0) is the upper half circle around (0, 0), clockwise as
    /// the path sees two axes; KRL and RAPID give the same in mm.
    #[test]
    fn robot_programs_read_lines_and_circles() {
        let axes = [axis("x"), axis("y")];
        let half = Piece::Arc { to: vec![1.0, 0.0], centre: vec![0.0, 0.0], clockwise: true };
        let csv = robot_csv("MoveL,-1,0\nMoveC,0,1,1,0", &axes, &[0.0, 0.0]).unwrap();
        assert_eq!(csv[0].pieces, vec![Piece::Line(vec![-1.0, 0.0]), half.clone()]);
        let close = |a: &Piece, b: &Piece| match (a, b) {
            (Piece::Arc { to, centre, clockwise }, Piece::Arc { to: t2, centre: c2, clockwise: w2 }) => {
                to.iter().chain(centre).zip(t2.iter().chain(c2)).all(|(x, y)| (x - y).abs() < 1e-12) && clockwise == w2
            }
            (Piece::Line(a), Piece::Line(b)) => a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12),
            _ => false,
        };
        let src =
            "DEF demo()\n  PTP {X -1000, Y 0, Z 500, A 0, B 90, C 0} ; start\n  CIRC {X 0, Y 1000}, {X 1000, Y 0}\nEND";
        let kuka = krl(src, &axes, &[0.0, 0.0]).unwrap();
        assert!(
            close(&kuka[0].pieces[0], &Piece::Line(vec![-1.0, 0.0])) && close(&kuka[0].pieces[1], &half),
            "{kuka:?}"
        );
        let module = "MODULE m\n CONST robtarget p1 := [[0,1000,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];\n PROC main()\n  MoveL [[-1000,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v100, z10, tool0;\n  MoveC p1, [[1000,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v100, fine, tool0; ! half\n ENDPROC\nENDMODULE";
        let abb = rapid(module, &axes, &[0.0, 0.0]).unwrap();
        assert!(close(&abb[0].pieces[0], &Piece::Line(vec![-1.0, 0.0])) && close(&abb[0].pieces[1], &half), "{abb:?}");
        assert_eq!(rapid("MoveAbsJ jpos, v100, fine, tool0;", &axes, &[0.0; 2]).unwrap_err().key, "program-error-word");
        assert_eq!(krl("PTP HOME", &axes, &[0.0; 2]).unwrap_err().key, "program-error-named");
        // The circle is the half turn: pi m long on the planned path.
        assert!((length(&axes, &[-1.0, 0.0], &[half]) - PI).abs() < 1e-4);
    }

    /// Robot programs carry their speeds and, on an arm in tool space (axes x, y, z, rx, ry, rz), their orientations.
    /// RAPID: v100 is 100 mm/s, 0.1 m/s; a speeddata declared as [250, 500, 5000, 1000] 0.25 m/s; vmax the axes' limits.
    /// The quaternion [cos 45°, 0, 0, sin 45°] is a quarter turn about z, the rotation vector (0, 0, pi / 2). KRL:
    /// `$VEL.CP = 0.2` is 0.2 m/s for the LIN after it, a PTP runs at the axes' limits; A 90 (B 0, C 0) is Rz(90°), the
    /// same quarter turn; positions in mm.
    #[test]
    fn robot_programs_carry_their_speeds_and_orientations() {
        let axes = ["x", "y", "z", "rx", "ry", "rz"].map(axis);
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let module = format!(
            "MODULE m\n VAR speeddata vfast := [250, 500, 5000, 1000];\n PROC main()\n  MoveL [[100,0,0],[{half},0,0,{half}],\
             [0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v100, z10, tool0;\n  MoveL [[200,0,0],[1,0,0,0],[0,0,0,0],\
             [9E9,9E9,9E9,9E9,9E9,9E9]], vfast, fine, tool0;\n  MoveJ [[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], \
             vmax, fine, tool0;\n ENDPROC\nENDMODULE"
        );
        let runs = rapid(&module, &axes, &[0.0; 6]).unwrap();
        assert_eq!(runs.iter().map(|r| r.feed).collect::<Vec<_>>(), [Some(0.1), Some(0.25), None]);
        let Piece::Line(turned) = &runs[0].pieces[0] else { panic!() };
        assert!(
            turned.iter().zip([0.1, 0.0, 0.0, 0.0, 0.0, PI / 2.0]).all(|(a, b)| (a - b).abs() < 1e-12),
            "{turned:?}"
        );
        let src = "DEF demo()\n  PTP {X 0, Y 0, Z 0}\n  $VEL.CP = 0.2\n  LIN {X 100, A 90}\nEND";
        let runs = krl(src, &axes, &[0.0; 6]).unwrap();
        assert_eq!(runs.iter().map(|r| r.feed).collect::<Vec<_>>(), [None, Some(0.2)]);
        let Piece::Line(turned) = &runs[1].pieces[0] else { panic!() };
        assert!(
            turned.iter().zip([0.1, 0.0, 0.0, 0.0, 0.0, PI / 2.0]).all(|(a, b)| (a - b).abs() < 1e-12),
            "{turned:?}"
        );
        let unknown = rapid("MoveL [[0,0,0],[1,0,0,0]], vslow, fine, tool0;", &axes, &[0.0; 6]).unwrap_err();
        assert_eq!((unknown.key, unknown.detail.as_str()), ("program-error-named", "vslow"));
    }

    /// The extension picks the reader; a path file stays a path file.
    #[test]
    fn the_extension_picks_the_reader() {
        let axes = [axis("x"), axis("y")];
        assert!((read("part.NC", "F60\nG1 X10", &axes, &[0.0; 2]).unwrap()[0].feed.unwrap() - 0.001).abs() < 1e-15);
        assert_eq!(
            read("p.csv", "line,0.1,0.2", &axes, &[0.0; 2]).unwrap()[0].pieces,
            vec![Piece::Line(vec![0.1, 0.2])]
        );
        assert_eq!(
            read("p.csv", "MoveJ,0.1,0.2", &axes, &[0.0; 2]).unwrap()[0].pieces,
            vec![Piece::Line(vec![0.1, 0.2])]
        );
    }
}
