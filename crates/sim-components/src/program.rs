//! Programs read into path pieces (`path::Piece`) for the path planner: a subset of G-code (DIN 66025 / ISO 6983), robot
//! moves as CSV, and subsets of KUKA KRL and ABB RAPID. `read` picks the reader by the file's extension.
//!
//! G-code (`.nc`, `.ngc`, `.gcode`, `.g`, `.cnc`, `.tap`), one block per line:
//! - G0 rapid, G1 line, G2 clockwise and G3 counterclockwise arc (seen from the plus side of the plane's normal),
//!   centre by I, J, K (from the start, always incremental) or radius R (negative: the longer of the two arcs);
//! - G17 (XY plane), G18 (ZX), G19 (YZ); G20 inch, G21 mm (the default); G90 absolute (the default), G91 incremental;
//! - F the feed per minute, needed before the first G1, G2 or G3: the speed along the path, in length units for a block
//!   with a linear axis and in degrees for a block that moves only rotary axes (A, B, C); where both move, the linear
//!   axes' distance at F sets the time and the rotary axes follow in it. This is Siemens SINUMERIK's reading with its
//!   default FGROUP (the geometry axes; a rotary axis in a mixed block is synchronised and has no F of its own;
//!   Fanuc's, which counts degrees like mm in a mixed block, is not taken: no primary source). G2 or G3 with I, J, K and
//!   no end is a full circle. F is modal and caps each piece at its own block's feed (`path::plan_feeds`, `Run::caps`):
//!   the moves between two G0 are one run planned as one path, with no stop where the feed changes (a corner still
//!   stops it, as it does without a feed); G0 runs at the axes' limits;
//! - X, Y, Z (and A, B, C in degrees) move the axes of the same name (ids x, y, z, a, b, c); the start is where the
//!   machine stands, an axis a block does not name stays;
//! - G4 P<seconds> a dwell: the path stops at the end of the move before, stands P seconds (0 to `MAX_DWELL_S`) and goes
//!   on (RS-274NGC's G4, as LinuxCNC reads it; Siemens' G4 F and Fanuc's P in milliseconds are not read). G61 exact stop
//!   (corner tolerance 0 from here on), G64 continuous mode (the config's `profile.corner_tolerance_m`) and G64 P<length>
//!   (that tolerance in the program's unit, G20/G21 apply), as RS-274NGC and LinuxCNC have them (Siemens' G64 takes no
//!   P). The mode that governs a corner is the one in force for the block after it; a change of mode, like a dwell,
//!   ends the run, so the corner at the change is stopped at (`Run::corner`, `Run::dwell_s`);
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
//! kept from the point before (for a CIRC too, the auxiliary point's are not used: KUKA System Software 8.3, section
//! 11.5.3); A, B, C, S, T, E1... move axes of those names where the machine has them and are left out
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
//!
//! A robot's speed (KRL's `$VEL.CP`, RAPID's TCP speed) is the tool centre's, the position alone, and the reorientation
//! is limited by its own: `program.orientation_speed_rad_s` (default 500 degrees per second, the default of RAPID's
//! `v_ori`), on a machine whose tool space has rx, ry, rz. A move takes the longer of the tool centre's distance over
//! its speed and the angle between the two orientations over that rate, so a move that only reorients runs at the
//! rate; the planner's cap along the path (whose length counts the rotation vector's radians with the metres) is the
//! path's length over that time. A move with no speed (PTP, MoveJ, `vmax`) stays at the axes' limits. G-code keeps its
//! feed rule above.

use crate::path::Piece;
use sim_core::AxisInfo;

/// Moves planned as one path with no stop between them: the pieces, and the speed along the path each is capped at (axis
/// units per second, over all axes, as `path::plan_feeds` takes it): `caps`, one per piece. `feed` is the highest of them,
/// `None` where the run is at the axes' limits (and `caps` is empty).
#[derive(Debug, Clone, PartialEq)]
pub struct Run {
    pub feed: Option<f64>,
    pub caps: Vec<f64>,
    pub pieces: Vec<Piece>,
    /// The corner tolerance (m) of the G61 or G64 P in force for the run, `None` for the config's
    /// (`profile.corner_tolerance_m`).
    pub corner: Option<f64>,
    /// A run with no pieces is a dwell (G4): the path stands this long (s) where the run before ended.
    pub dwell_s: f64,
}

impl Run {
    /// The cap of each piece, `None` where the run is at the axes' limits; a run built with a feed and no caps has it for all.
    pub fn caps(&self) -> Option<Vec<f64>> {
        self.feed
            .map(|f| if self.caps.len() == self.pieces.len() { self.caps.clone() } else { vec![f; self.pieces.len()] })
    }
}

/// The pieces of a program that plans as a path file does (`Sim::plan_path`): one run at the axes' limits in the config's
/// corner tolerance. `None` for one to plan run by run (`Sim::plan_program`): with a feed, more runs, or a G61 or G64 P
/// whose corner tolerance only `plan_program` takes.
pub fn as_path(runs: &[Run]) -> Option<&[Piece]> {
    match runs {
        [run] if run.feed.is_none() && run.corner.is_none() => Some(&run.pieces),
        _ => None,
    }
}

/// The longest dwell (s) a G4 takes: the plan has a pose per profile cycle for it. This simulator's limit.
pub const MAX_DWELL_S: f64 = 3600.0;

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
    read_with(file_name, text, axes, start, ORIENTATION_SPEED)
}

/// `read` with the tool's reorientation speed `orientation_speed` (rad/s) for the robot programs on a machine whose tool
/// has an orientation (`program.orientation_speed_rad_s`).
pub fn read_with(
    file_name: &str,
    text: &str,
    axes: &[AxisInfo],
    start: &[f64],
    orientation_speed: f64,
) -> Result<Vec<Run>> {
    let extension = file_name.rsplit_once('.').map_or("", |(_, e)| e).to_ascii_lowercase();
    match extension.as_str() {
        "nc" | "ngc" | "gcode" | "g" | "cnc" | "tap" => gcode(text, axes, start),
        "src" => krl_with(text, axes, start, orientation_speed),
        "mod" | "prg" => rapid_with(text, axes, start, orientation_speed),
        _ if text.lines().any(|l| l.trim().to_ascii_lowercase().starts_with("move")) => robot_csv(text, axes, start),
        _ => crate::path::parse(text)
            .map(|pieces| vec![Run { feed: None, caps: Vec::new(), pieces, corner: None, dwell_s: 0.0 }])
            .map_err(|line| ProgramError { line: 0, key: "program-error-word", detail: line }),
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
/// clockwise. `None` where the three points are on one line or so far out that the centre overflows.
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
    // Points near f64's range overflow the squares above: no centre to compute with.
    if !centre.iter().all(|c| c.is_finite()) {
        return None;
    }
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
    push_by(runs, feed, piece, false);
}

/// `push`; with `joined` a piece with a feed also joins a run at another feed, keeping its own cap (G-code: the feed is
/// modal and changes between blocks without a stop).
fn push_by(runs: &mut Vec<Run>, feed: Option<f64>, piece: Piece, joined: bool) {
    match runs.last_mut() {
        Some(run) if run.feed == feed || (joined && run.feed.is_some() && feed.is_some()) => {
            run.pieces.push(piece);
            if let Some(f) = feed {
                run.caps.push(f);
                run.feed = run.feed.map(|g| g.max(f));
            }
        }
        _ => runs.push(Run { feed, caps: feed.into_iter().collect(), pieces: vec![piece], corner: None, dwell_s: 0.0 }),
    }
}

/// G-code (see the module).
pub fn gcode(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    let mut pos = start.to_vec();
    let (mut absolute, mut inch, mut plane, mut motion, mut feed) = (true, false, 17, None::<u32>, None::<(f64, f64)>);
    let mut runs = Vec::new();
    // G61 (exact stop: tolerance 0) and G64 (the config's, or its P) as the modal corner tolerance; `split` where the next
    // move starts a new run, after a change of it or a dwell.
    let (mut corner, mut split) = (None::<f64>, false);
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
            let after = letter.len_utf8();
            let end = rest[after..].find(|c: char| c.is_ascii_alphabetic()).map_or(rest.len(), |i| i + after);
            let (word, number) = (&rest[..end], &rest[after..end]);
            match number.parse::<f64>() {
                Ok(v) if letter.is_ascii_alphabetic() && v.is_finite() => words.push((letter, v, word)),
                _ => return fail(line, "program-error-word", word),
            }
            rest = &rest[end..];
        }
        let mut moving = false;
        let mut ends = false;
        let (mut target, mut centre_offset, mut radius) = (pos.clone(), [None::<f64>; 3], None::<f64>);
        let (mut dwell, mut mode, mut p) = (false, None::<Option<f64>>, None::<(f64, &str)>);
        // Modal words first: units and distances apply to the whole block.
        for &(letter, v, word) in &words {
            if letter == 'G' {
                match v {
                    4.0 => dwell = true,
                    61.0 => mode = Some(Some(0.0)),
                    64.0 => mode = Some(None),
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
                // The feed as per second: length units, and degrees as radians for a block that only turns rotary axes.
                'F' if v > 0.0 => feed = Some((v * length / 60.0, v.to_radians() / 60.0)),
                'X' | 'Y' | 'Z' | 'A' | 'B' | 'C' => {
                    let Some(i) = axis_of(axes, letter) else { return fail(line, "program-error-axis", word) };
                    let scale = if axes[i].unit == "rad" { std::f64::consts::PI / 180.0 } else { length };
                    target[i] = if absolute { v * scale } else { target[i] + v * scale };
                    moving = true;
                }
                'I' | 'J' | 'K' => centre_offset[(letter as u8 - b'I') as usize] = Some(v * length),
                'R' => radius = Some(v * length),
                'P' => p = Some((v, word)),
                _ => return fail(line, "program-error-word", word),
            }
        }
        // A full circle (G2/G3 with I, J, K and no end) moves too.
        moving |= matches!(motion, Some(2 | 3)) && centre_offset.iter().any(Option::is_some);
        // P is the dwell's seconds (G4) or the corner tolerance in the program's length unit (G64): RS-274NGC's (LinuxCNC);
        // neither Siemens' G4 F nor Fanuc's G4 P in milliseconds is read.
        let g64 = words.iter().any(|&(letter, v, _)| letter == 'G' && v == 64.0);
        match p {
            None if dwell => return fail(line, "program-error-p", clean),
            Some((_, word)) if !dwell && !g64 => return fail(line, "program-error-word", word),
            Some(_) if dwell && (g64 || moving) => return fail(line, "program-error-p", clean),
            Some((v, _)) if g64 && v >= 0.0 => mode = Some(Some(v * length)),
            Some((v, _)) if dwell && (0.0..=MAX_DWELL_S).contains(&v) => {}
            Some(_) => return fail(line, "program-error-p", clean),
            None => {}
        }
        // The mode in force for the block's moves, as RS-274 has it: a corner is governed by the mode of the block that
        // follows it. A change starts a new run, so the corner at the change is stopped at (a path planned as one has one
        // tolerance); from G61 to G64 that is where RS-274 would round it.
        if let Some(m) = mode.filter(|m| *m != corner) {
            (corner, split) = (m, true);
        }
        if let (true, Some((seconds, _))) = (dwell, p) {
            runs.push(Run { feed: None, caps: Vec::new(), pieces: Vec::new(), corner, dwell_s: seconds });
            split = true;
        }
        if moving {
            let Some(kind) = motion else { return fail(line, "program-error-motion", clean) };
            let rate = if kind == 0 {
                None
            } else {
                let (linear, rotary) =
                    feed.ok_or_else(|| ProgramError { line, key: "program-error-feed", detail: clean.clone() })?;
                Some(path_feed(axes, &pos, &target, linear, rotary))
            };
            let pieces = if kind < 2 {
                vec![Piece::Line(target.clone())]
            } else {
                arc_block(line, axes, &pos, &target, plane, kind == 2, centre_offset, radius)?
            };
            for piece in pieces {
                if std::mem::take(&mut split) {
                    let caps = rate.into_iter().collect();
                    runs.push(Run { feed: rate, caps, pieces: vec![piece], corner, dwell_s: 0.0 });
                } else {
                    push_by(&mut runs, rate, piece, true);
                    if let Some(run) = runs.last_mut() {
                        run.corner = corner;
                    }
                }
            }
            pos = target;
        }
        if ends {
            break;
        }
    }
    Ok(runs)
}

/// The speed along the path (over all axes, as `path::plan_at` takes it) that moves `from` to `to` at the F of the
/// block: `linear` m/s for the linear axes' distance, else `rotary` rad/s for the rotary axes' (a block that only
/// turns), and where both move the linear axes set the time and the rotary ones follow in it.
fn path_feed(axes: &[AxisInfo], from: &[f64], to: &[f64], linear: f64, rotary: f64) -> f64 {
    let (mut lin, mut rot) = (0.0, 0.0);
    for (i, a) in axes.iter().enumerate() {
        let d = (to[i] - from[i]).powi(2);
        if a.unit == "rad" { rot += d } else { lin += d }
    }
    if lin > 0.0 {
        linear * (lin + rot).sqrt() / lin.sqrt()
    } else if rot > 0.0 {
        rotary
    } else {
        linear
    }
}

/// The arc of a G2 (`clockwise`) or G3 block in `plane` from `from` to `to`, its centre from the offsets I, J, K or
/// the radius R, in the plane G17, G18 or G19 states (`Piece::Arc::plane`); a full circle (`to` at `from`, centre by I,
/// J, K) as three arcs of 120 degrees, which a full turn's end cannot tell from no turn at all.
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
) -> Result<Vec<Piece>> {
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
    let turning = if clockwise { -1.0 } else { 1.0 };
    let clockwise = path_clockwise(ia, ib, turning);
    // The plane's normal is the third of x, y, z by position: it is what a half turn, whose end lies opposite its start,
    // has no other way to tell the planner (G18 and G19 half turns lie in the x-z and y-z planes).
    let plane = (ia < 3 && ib < 3).then(|| std::array::from_fn(|k| if k == 3 - ia - ib { 1.0 } else { 0.0 }));
    if (ea - sa).hypot(eb - sb) > 1e-12 * r0 {
        return Ok(vec![Piece::Arc { to: to.to_vec(), centre, clockwise, plane }]);
    }
    let start = (sb - cb).atan2(sa - ca);
    Ok((1..=3)
        .map(|k| {
            let mut end = to.to_vec();
            if k < 3 {
                let angle = start + turning * k as f64 * 2.0 * std::f64::consts::PI / 3.0;
                (end[ia], end[ib]) = (ca + r0 * angle.cos(), cb + r0 * angle.sin());
            }
            Piece::Arc { to: end, centre: centre.clone(), clockwise, plane }
        })
        .collect())
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
        Some((centre, clockwise)) => {
            // The plane of the three points: a half turn through them (end opposite start) is in no other plane.
            let at = |p: &[f64]| [p[0], p[1], p.get(2).copied().unwrap_or(0.0)];
            let (s, v, e) = (at(from), at(via), at(to));
            let normal = cross([v[0] - s[0], v[1] - s[1], v[2] - s[2]], [e[0] - s[0], e[1] - s[1], e[2] - s[2]]);
            let size = normal.iter().map(|x| x * x).sum::<f64>().sqrt();
            Ok(Piece::Arc { to: to.to_vec(), centre, clockwise, plane: (n == 3).then(|| normal.map(|x| x / size)) })
        }
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

/// The tool's reorientation speed a robot program runs at by default (rad/s): 500 degrees per second, the default of
/// ABB's `v_ori` (RAPID speeddata); `program.orientation_speed_rad_s` sets it.
pub const ORIENTATION_SPEED: f64 = 500.0 * std::f64::consts::PI / 180.0;

/// The config's `program.orientation_speed_rad_s` (above 0), `ORIENTATION_SPEED` where it has none: the value the front
/// ends pass to `read_with`, as the simulator's own path run does.
pub fn orientation_speed_of(cfg: &sim_core::Config) -> std::result::Result<f64, sim_core::ConfigError> {
    let key = "program.orientation_speed_rad_s";
    if cfg.get(key).is_ok() { cfg.positive(key) } else { std::result::Result::Ok(ORIENTATION_SPEED) }
}

/// The angle (rad) of the arc through `via` from `from` to `to` about `centre`, on the first three axes: the two
/// parts from the start to `via` and from `via` to the end, each taken as at most a half turn.
fn arc_angle(from: &[f64], via: &[f64], to: &[f64], centre: &[f64]) -> f64 {
    let n = from.len().min(3);
    let at = |p: &[f64]| (0..n).map(|i| p[i] - centre[i]).collect::<Vec<f64>>();
    let angle = |a: &[f64], b: &[f64]| {
        let (dot, size) = (a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>(), (norm(a) * norm(b)).max(1e-300));
        (dot / size).clamp(-1.0, 1.0).acos()
    };
    let (a, v, b) = (at(from), at(via), at(to));
    angle(&a, &v) + angle(&v, &b)
}

fn norm(a: &[f64]) -> f64 {
    a.iter().map(|x| x * x).sum::<f64>().sqrt()
}

/// The cap along the path (the planner's units: the length counts the rotation vector's radians with the metres) of a
/// piece that a robot program runs at the tool centre's speed `v` (`None`: the axes' limits) while the orientation
/// (the axes `rotation`, a rotation vector) turns at no more than `omega` rad/s (RAPID's `v_ori`): the piece takes
/// the longer of d / v for the tool centre's distance d (the axes that are not the rotation's, along the arc through
/// `via` on an arc) and theta / omega for the angle theta between the two orientations, and its length over that time
/// is the cap. A move that only reorients (d = 0) runs at `omega`. Without a rotation, `v` itself.
fn tcp_cap(
    rotation: Option<[usize; 3]>,
    from: &[f64],
    piece: &Piece,
    via: Option<&[f64]>,
    v: Option<f64>,
    omega: f64,
) -> Option<f64> {
    let v = v?;
    let Some(rotation) = rotation else { return Some(v) };
    let (Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to)) = piece;
    let others = |from_axis: usize| -> f64 {
        (from_axis..from.len()).filter(|i| !rotation.contains(i)).map(|i| (to[i] - from[i]).powi(2)).sum::<f64>()
    };
    let position = match (piece, via) {
        (Piece::Arc { centre, .. }, Some(via)) => {
            let r = norm(&from.iter().zip(centre).take(3).map(|(f, c)| f - c).collect::<Vec<f64>>());
            (r * arc_angle(from, via, to, centre)).hypot(others(3).sqrt())
        }
        _ => others(0).sqrt(),
    };
    use crate::rigid::{exp, log, matmul, transpose};
    let (a, b) = (rotation.map(|i| from[i]), rotation.map(|i| to[i]));
    let turn = log(&matmul(&exp(b), &transpose(&exp(a))));
    let theta = (turn[0] * turn[0] + turn[1] * turn[1] + turn[2] * turn[2]).sqrt();
    let chord = (0..3).map(|k| (b[k] - a[k]).powi(2)).sum::<f64>().sqrt();
    let time = (position / v).max(theta / omega);
    if time > 0.0 && time.is_finite() { Some(position.hypot(chord) / time) } else { Some(v) }
}

/// KUKA KRL (see the module).
pub fn krl(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    krl_with(text, axes, start, ORIENTATION_SPEED)
}

/// `krl` with the tool's reorientation speed `orientation_speed` (rad/s).
pub fn krl_with(text: &str, axes: &[AxisInfo], start: &[f64], orientation_speed: f64) -> Result<Vec<Run>> {
    let mut pos = start.to_vec();
    let mut runs = Vec::new();
    // The orientation as KUKA's A, B, C, where the machine has rx, ry, rz: components left out are kept.
    let rotation = rotation_axes(axes);
    let mut abc = rotation.map(|r| abc_of(r.map(|i| start[i])));
    let mut speed = None;
    let mut last: Option<Option<f64>> = None;
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
                let Some(v) = value.trim().parse::<f64>().ok().filter(|v| v.is_finite()) else {
                    return fail(line, "program-error-word", part);
                };
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
        let (piece, via, feed) = match (keyword.as_str(), aggregates.as_slice()) {
            ("PTP", [end]) => (Piece::Line(oriented(&pos, end)), None, None),
            ("LIN", [end]) => (Piece::Line(oriented(&pos, end)), None, speed),
            ("CIRC", [aux, end]) => {
                let via = robot_pose(axes, &pos, aux);
                let to = oriented(&pos, end);
                (circle(line, &pos, &via, &to)?, Some(via), speed)
            }
            _ => return fail(line, "program-error-named", code),
        };
        let capped = tcp_cap(rotation, &pos, &piece, via.as_deref(), feed, orientation_speed);
        pos = match &piece {
            Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to) => to.clone(),
        };
        // Moves at the same programmed speed are one run, whatever each one's cap along the path is.
        push_by(&mut runs, capped, piece, last == Some(feed));
        last = Some(feed);
    }
    Ok(runs)
}

/// ABB RAPID (see the module).
pub fn rapid(text: &str, axes: &[AxisInfo], start: &[f64]) -> Result<Vec<Run>> {
    rapid_with(text, axes, start, ORIENTATION_SPEED)
}

/// `rapid` with the tool's reorientation speed `orientation_speed` (rad/s).
pub fn rapid_with(text: &str, axes: &[AxisInfo], start: &[f64], orientation_speed: f64) -> Result<Vec<Run>> {
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
    let mut last: Option<Option<f64>> = None;
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
        let (piece, via, feed) = match instruction.to_ascii_lowercase().as_str() {
            "movel" | "movej" => (Piece::Line(oriented(&pos, &target(args[0])?)), None, speed(args.get(1))?),
            "movec" if args.len() >= 2 => {
                let via = robot_pose(axes, &pos, &target(args[0])?);
                let to = oriented(&pos, &target(args[1])?);
                (circle(line, &pos, &via, &to)?, Some(via), speed(args.get(2))?)
            }
            _ => return fail(line, "program-error-word", instruction.to_string()),
        };
        let capped = tcp_cap(rotation, &pos, &piece, via.as_deref(), feed, orientation_speed);
        pos = match &piece {
            Piece::Line(to) | Piece::Arc { to, .. } | Piece::Spline(to) => to.clone(),
        };
        push_by(&mut runs, capped, piece, last == Some(feed));
        last = Some(feed);
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

    /// F per minute is mm for linear axes and degrees for a block with rotary axes only. With x (m) and c (rad):
    /// `G1 X10 F600` is 10 mm at 10 mm/s, 1 s: path speed 0.01. `G1 C90 F3600` is pi/2 rad at 3600 deg/min = pi/3
    /// rad/s, 1.5 s. `G1 X10 C90 F600` takes 1 s for the 10 mm, so the path of length sqrt(0.01^2 + (pi/2)^2) runs at
    /// that length per second.
    #[test]
    fn feed_is_per_minute_in_mm_or_degrees() {
        let mut c = axis("c");
        c.unit = "rad";
        let axes = [axis("x"), c];
        let feed = |text: &str| gcode(text, &axes, &[0.0, 0.0]).unwrap()[0].feed.unwrap();
        assert!((feed("G1 X10 F600") - 0.01).abs() < 1e-15);
        assert!((feed("G1 C90 F3600") - PI / 3.0).abs() < 1e-12);
        assert!((feed("G1 X10 C90 F600") - (0.01f64.powi(2) + (PI / 2.0).powi(2)).sqrt()).abs() < 1e-12);
        // Planned (axes quick enough that the ramps take a few ms: v / a + a / j at most 1.05 + 1 ms), the three take 1,
        // 1.5 and 1 s.
        let quick = axes.map(|a| AxisInfo { max_vel: 10.0, max_acc: 1e3, max_jerk: 1e6, ..a });
        for (text, time) in [("G1 X10 F600", 1.0), ("G1 C90 F3600", 1.5), ("G1 X10 C90 F600", 1.0)] {
            let runs = gcode(text, &quick, &[0.0, 0.0]).unwrap();
            let caps = runs[0].caps().unwrap();
            let points =
                crate::path::plan_feeds(&quick, &[0.0, 0.0], &runs[0].pieces, &caps, 1e-3, 0.0, crate::path::ARC_SLACK)
                    .unwrap();
            let planned = points[points.len() - 1].0;
            assert!(planned >= time && planned <= time + 4e-3, "{text}: {planned} s");
        }
    }

    /// A feed that changes between blocks is a cap per piece inside one run, with no stop between: `F600 G1 X10 / F1200 G1
    /// X20 / F600 G1 X30` is 10 mm at 10 mm/s, 10 mm at 20 mm/s and 10 mm at 10 mm/s: 1 + 0.5 + 1 = 2.5 s (the ramps between
    /// the speeds add a few ms: 0.01 m/s over a jerk of 1000 m/s^3 takes 2 sqrt(0.01 / 1000) = 6.3 ms), and the path
    /// passes x = 10 mm at the lower feed, 10 mm/s, not at rest.
    #[test]
    fn a_feed_that_changes_between_blocks_is_a_cap_per_piece_in_one_run() {
        let axes = [axis("x"), axis("y")];
        let runs = gcode(
            "F600
G1 X10
F1200
G1 X20
F600
G1 X30",
            &axes,
            &[0.0; 2],
        )
        .unwrap();
        assert_eq!(runs.len(), 1);
        let caps = runs[0].caps().unwrap();
        assert!(caps.iter().zip([0.01, 0.02, 0.01]).all(|(a, b)| (a - b).abs() < 1e-15) && runs[0].feed == Some(0.02));
        let points =
            crate::path::plan_feeds(&axes, &[0.0; 2], &runs[0].pieces, &caps, 1e-3, 0.0, crate::path::ARC_SLACK)
                .unwrap();
        let (time, k) = (points[points.len() - 1].0, points.iter().position(|(_, p)| p[0] >= 0.01).unwrap());
        let speed = (points[k + 1].1[0] - points[k - 1].1[0]) / 2e-3;
        assert!((time - 2.5).abs() < 0.03 && (speed - 0.01).abs() < 1e-4, "{time} s, {speed} m/s at 10 mm");
    }

    /// `F600 / G1 X1 / G2 I10` is 1 mm, then a full circle of radius 10 mm: 0.001 + 2 pi 0.01 m on the planned path, which
    /// closes where the circle starts, (1, 0) mm, and reaches x = 21 mm and y = 10 and -10 mm on the way.
    #[test]
    fn a_full_circle_is_planned() {
        let axes = [axis("x"), axis("y")];
        let runs = gcode(
            "F600
G1 X1
G2 I10",
            &axes,
            &[0.0; 2],
        )
        .unwrap();
        let pieces = &runs[0].pieces;
        assert_eq!(pieces.len(), 4);
        let l = length(&axes, &[0.0; 2], pieces);
        assert!((l - (0.001 + 2.0 * PI * 0.01)).abs() < 1e-5, "{l}");
        let points = crate::path::plan(&axes, &[0.0; 2], pieces, 1e-3, 0.0, crate::path::ARC_SLACK).unwrap();
        let end = &points[points.len() - 1].1;
        assert!((end[0] - 0.001).abs() < 1e-12 && end[1].abs() < 1e-12, "ends at {end:?}");
        // The poses (some 1 mm apart) reach the circle's far side and both of its extremes in y.
        let most = |f: &dyn Fn(&[f64]) -> f64| points.iter().map(|(_, p)| f(p)).fold(f64::NEG_INFINITY, f64::max);
        let (x, up, down) = (most(&|p| p[0]), most(&|p| p[1]), most(&|p| -p[1]));
        assert!([x - 0.021, up - 0.01, down - 0.01].iter().all(|d| d.abs() < 2e-4), "{x} {up} {down}");
    }

    /// A KRL CIRC whose end leaves out a component keeps it from the point before, not from the auxiliary point: from
    /// (0, 0, 100) mm through {X 100, Y 100, Z 200} to {X 200, Y 0} ends at z = 100 mm, on the circle through the three
    /// points, and the planned path ends there.
    #[test]
    fn a_krl_circ_keeps_a_component_left_out_of_its_end_from_the_point_before() {
        let axes = [axis("x"), axis("y"), axis("z")];
        let runs = krl("LIN {X 0, Y 0, Z 100}\nCIRC {X 100, Y 100, Z 200}, {X 200, Y 0}", &axes, &[0.0; 3]).unwrap();
        let pieces: Vec<Piece> = runs.iter().flat_map(|r| r.pieces.clone()).collect();
        let Piece::Arc { to, .. } = &pieces[1] else { panic!("{pieces:?}") };
        assert!(to.iter().zip([0.2, 0.0, 0.1]).all(|(a, b)| (a - b).abs() < 1e-12), "{to:?}");
        let points = crate::path::plan(&axes, &[0.0; 3], &pieces, 1e-3, 0.0, crate::path::ARC_SLACK).unwrap();
        let end = &points[points.len() - 1].1;
        assert!(end.iter().zip([0.2, 0.0, 0.1]).all(|(a, b)| (a - b).abs() < 1e-9), "{end:?}");
    }

    /// The other planes and units: G18 on a gantry of x and z: G2 is clockwise seen from +y, so counterclockwise seen
    /// from -y, where x is to the right and z up as the path reads two axes. G20 reads inches: X1 is 25.4 mm. G91 adds.
    #[test]
    fn gcode_planes_units_and_increments() {
        let xz = [axis("x"), axis("z")];
        let runs = gcode("G18 G20 F10\nG2 X1 Z1 I1 K0", &xz, &[0.0, 0.0]).unwrap();
        let Piece::Arc { to, centre, clockwise, .. } = &runs[0].pieces[0] else { panic!() };
        assert!(!clockwise && (to[0] - 0.0254).abs() < 1e-12 && (centre[0] - 0.0254).abs() < 1e-12 && centre[1] == 0.0);
        // Seen from -y, G2 from (0, 0) to (25.4, 25.4) mm around (25.4, 0) is the three-quarter way: 3/4 2 pi 25.4 mm.
        let l = length(&xz, &[0.0, 0.0], &runs[0].pieces);
        assert!((l - 1.5 * PI * 0.0254).abs() < 1e-5, "{l}");
        let runs = gcode("G91 G0 X10\nX10 Z-5", &xz, &[0.1, 0.1]).unwrap();
        assert_eq!(runs[0].feed, None);
        let Piece::Line(end) = &runs[0].pieces[1] else { panic!() };
        assert!((end[0] - 0.12).abs() < 1e-15 && (end[1] - 0.095).abs() < 1e-15, "{end:?}");
    }

    /// Half turns on three axes lie in the plane the program states: G18 from (10, 0, 0) mm to (-10, 0, 0) around the origin
    /// is in the x-z plane (y stays 0, z reaches the radius 10 mm), where the first plane holding the start's radius would
    /// have been x-y. The three points of a MoveC (-0.5, 0, 0), (0, 0, 0.5), (0.5, 0, 0) say x-z too (their normal is
    /// (1, 0, 1) x (2, 0, 0) = (0, 2, 0)), and the circle through them reaches z = 0.5.
    #[test]
    fn a_half_turn_on_three_axes_lies_in_the_plane_the_program_states() {
        let axes = [axis("x"), axis("y"), axis("z")];
        let reach = |start: &[f64], pieces: &[Piece]| {
            let points = crate::path::plan(&axes, start, pieces, 1e-3, 0.0, crate::path::ARC_SLACK).unwrap();
            points.iter().fold((0f64, 0f64), |(y, z), (_, p)| (y.max(p[1].abs()), z.max(p[2].abs())))
        };
        let start = [0.01, 0.0, 0.0];
        let runs = gcode(
            "G18 F600
G2 X-10 Z0 I-10 K0",
            &axes,
            &start,
        )
        .unwrap();
        assert!(matches!(&runs[0].pieces[0], Piece::Arc { plane: Some(n), .. } if *n == [0.0, 1.0, 0.0]));
        let (y, z) = reach(&start, &runs[0].pieces);
        assert!(y < 1e-12 && (z - 0.01).abs() < 1e-4, "y {y} m, z {z} m");
        let runs = robot_csv(
            "MoveL,-0.5,0,0
MoveC,0,0,0.5,0.5,0,0",
            &axes,
            &[0.0; 3],
        )
        .unwrap();
        let (y, z) = reach(&[0.0; 3], &runs[0].pieces);
        assert!(y < 1e-12 && (z - 0.5).abs() < 1e-3, "y {y} m, z {z} m");
    }

    /// What G-code the reader does not take is refused with its line.
    #[test]
    fn unsupported_gcode_is_refused_by_line() {
        let axes = [axis("x"), axis("y"), axis("z")];
        let error = |text: &str| gcode(text, &axes, &[0.0; 3]).unwrap_err();
        assert_eq!(error("G0 X1\nG41"), ProgramError { line: 2, key: "program-error-word", detail: "G41".into() });
        assert_eq!(error("G1 X1").key, "program-error-feed");
        assert_eq!(error("F100\nG2 X10 Y0 Z5 I5").key, "program-error-helix");
        assert_eq!(error("F100\nG2 X10 I3").key, "program-error-arc");
        assert_eq!(gcode("G0 A1", &axes[..2], &[0.0; 2]).unwrap_err().key, "program-error-axis");
        // Comments, line numbers and switching functions pass; M30 ends the program.
        let runs = gcode("%\nN10 G0 X1 (to the side) ; rapid\nM3 S1000\nM30\nG0 X2", &axes, &[0.0; 3]).unwrap();
        let line = vec![Piece::Line(vec![0.001, 0.0, 0.0])];
        assert_eq!(runs, vec![Run { feed: None, caps: vec![], pieces: line, corner: None, dwell_s: 0.0 }]);
    }

    /// G61, G64 and G64 P are modal and each change starts a run, with the tolerance in force for the block that follows
    /// the corner: G61 is 0, G64 the config's (`None`), G64 P0.5 in mm 0.0005 m and in inch (G20) P0.02 is 0.02 x 0.0254 =
    /// 0.000508 m. Without a change the moves stay one run.
    #[test]
    fn g61_and_g64_set_the_corner_tolerance_by_run() {
        let axes = [axis("x"), axis("y"), axis("z")];
        let read = |text: &str| gcode(text, &axes, &[0.0; 3]).unwrap();
        let corners = |text: &str| read(text).iter().map(|r| (r.corner, r.pieces.len())).collect::<Vec<_>>();
        assert_eq!(corners("F6000\nG1 X10\nG1 Y10\nG1 X0"), [(None, 3)]);
        assert_eq!(
            corners("F6000\nG1 X10\nG1 Y10\nG61\nG1 X0\nG64 P0.5\nG1 Y0\nG1 X5\nG64\nG1 Y5"),
            [(None, 2), (Some(0.0), 1), (Some(0.0005), 2), (None, 1)]
        );
        // The same mode again is no change; the unit of P is the program's.
        assert_eq!(corners("F6000\nG61\nG1 X10\nG61\nG1 Y10"), [(Some(0.0), 2)]);
        let inch = read("F100\nG20 G64 P0.02\nG1 X1")[0].corner.unwrap();
        assert!((inch - 0.000508).abs() < 1e-15, "{inch}");
        // P with G64 in the block of the move: the mode governs that move.
        assert_eq!(corners("F6000\nG1 X10\nG64 P1 G1 Y10")[1], (Some(0.001), 1));
    }

    /// G4 P<s> is a run of its own with no pieces and the time; the moves after it are a run of their own, so the path
    /// stops before and after it. G4 without P, with a negative P or one beyond an hour, with a move in its block, and
    /// a P with no G4 or G64 are refused with their line.
    #[test]
    fn g4_is_a_dwell_run_and_wrong_p_is_refused() {
        let axes = [axis("x"), axis("y"), axis("z")];
        let runs = gcode("F6000\nG1 X10\nG4 P0.5\nG1 X20", &axes, &[0.0; 3]).unwrap();
        assert_eq!(runs.len(), 3, "{runs:?}");
        assert_eq!((runs[0].pieces.len(), runs[0].dwell_s), (1, 0.0));
        assert_eq!((runs[1].pieces.len(), runs[1].dwell_s, runs[1].feed), (0, 0.5, None));
        assert_eq!((runs[2].pieces.len(), runs[2].dwell_s), (1, 0.0));
        assert!((runs[2].feed.unwrap() - 0.1).abs() < 1e-12);
        for text in ["G4", "G4 P-1", "G4 P7200", "G0 X1\nG4 P1 X5", "G4 P1 G64", "G64 P-1"] {
            assert_eq!(gcode(text, &axes, &[0.0; 3]).unwrap_err().key, "program-error-p", "{text}");
        }
        assert_eq!(gcode("G0 X1\nP5", &axes, &[0.0; 3]).unwrap_err().key, "program-error-word");
    }

    /// Robot programs: MoveC through (0, 1) from (-1, 0) to (1, 0) is the upper half circle around (0, 0), clockwise as
    /// the path sees two axes; KRL and RAPID give the same in mm.
    #[test]
    fn robot_programs_read_lines_and_circles() {
        let axes = [axis("x"), axis("y")];
        let half = Piece::Arc { to: vec![1.0, 0.0], centre: vec![0.0, 0.0], clockwise: true, plane: None };
        let csv = robot_csv("MoveL,-1,0\nMoveC,0,1,1,0", &axes, &[0.0, 0.0]).unwrap();
        assert_eq!(csv[0].pieces, vec![Piece::Line(vec![-1.0, 0.0]), half.clone()]);
        let close = |a: &Piece, b: &Piece| match (a, b) {
            (Piece::Arc { to, centre, clockwise, .. }, Piece::Arc { to: t2, centre: c2, clockwise: w2, .. }) => {
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
        // The cap along the path is the path's length (the 0.1 m and the quarter turn's pi / 2 rad, counted together) over
        // the move's time: 0.1 m at 0.1 m/s takes 1 s, 0.1 m at 0.25 m/s 0.4 s, and the quarter turn at 500 degrees per
        // second only 0.18 s, so the tool centre's speed sets the time.
        let length = 0.1_f64.hypot(PI / 2.0);
        let feeds: Vec<_> = runs.iter().map(|r| r.feed).collect();
        assert!(
            feeds[0].is_some_and(|f| (f - length / 1.0).abs() < 1e-12)
                && feeds[1].is_some_and(|f| (f - length / 0.4).abs() < 1e-12)
                && feeds[2].is_none(),
            "{feeds:?}"
        );
        let Piece::Line(turned) = &runs[0].pieces[0] else { panic!() };
        assert!(
            turned.iter().zip([0.1, 0.0, 0.0, 0.0, 0.0, PI / 2.0]).all(|(a, b)| (a - b).abs() < 1e-12),
            "{turned:?}"
        );
        let src = "DEF demo()\n  PTP {X 0, Y 0, Z 0}\n  $VEL.CP = 0.2\n  LIN {X 100, A 90}\nEND";
        let runs = krl(src, &axes, &[0.0; 6]).unwrap();
        // 0.1 m at 0.2 m/s takes 0.5 s, longer than the quarter turn's 0.18 s.
        assert!(runs[0].feed.is_none() && runs[1].feed.is_some_and(|f| (f - length / 0.5).abs() < 1e-12));
        let Piece::Line(turned) = &runs[1].pieces[0] else { panic!() };
        assert!(
            turned.iter().zip([0.1, 0.0, 0.0, 0.0, 0.0, PI / 2.0]).all(|(a, b)| (a - b).abs() < 1e-12),
            "{turned:?}"
        );
        let unknown = rapid("MoveL [[0,0,0],[1,0,0,0]], vslow, fine, tool0;", &axes, &[0.0; 6]).unwrap_err();
        assert_eq!((unknown.key, unknown.detail.as_str()), ("program-error-named", "vslow"));
    }

    /// The tool centre's speed and the reorientation's are limited on their own (`tcp_cap`): a move takes the longer of
    /// its two times. 0.1 m at 1 m/s takes 0.1 s, the quarter turn (pi / 2 rad) at 500 degrees per second (8.7266 rad/s)
    /// 0.18 s: the move takes 0.18 s, and the cap along the path (0.1 m and pi / 2 rad counted together) is its length
    /// over that. A move that only reorients (v100, the position kept) runs at the 500 degrees per second: the cap is the
    /// rate itself, as the rotation vector's length is the angle. With 90 degrees per second the same move takes 1 s.
    #[test]
    fn a_robot_move_with_a_turn_takes_the_longer_of_the_two_times() {
        let axes = ["x", "y", "z", "rx", "ry", "rz"].map(axis);
        let half = std::f64::consts::FRAC_1_SQRT_2;
        let module = |x: u32, speed: &str| {
            format!("MoveL [[{x},0,0],[{half},0,0,{half}],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], {speed}, fine, tool0;")
        };
        let cap = |text: &str, omega: f64| rapid_with(text, &axes, &[0.0; 6], omega).unwrap()[0].feed.unwrap();
        let omega = ORIENTATION_SPEED;
        // To the rounding of the two ways of writing it (an ulp of 8.7 is 1.8e-15).
        assert!((omega - 500.0_f64.to_radians()).abs() < 1e-14);
        let time = (PI / 2.0) / omega;
        assert!((time - 0.18).abs() < 1e-12);
        assert!((cap(&module(100, "v1000"), omega) * 0.18 - 0.1_f64.hypot(PI / 2.0)).abs() < 1e-12);
        assert!((cap(&module(0, "v100"), omega) - omega).abs() < 1e-12);
        let slow = 90.0_f64.to_radians();
        assert!((cap(&module(0, "v100"), slow) - slow).abs() < 1e-12);
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
