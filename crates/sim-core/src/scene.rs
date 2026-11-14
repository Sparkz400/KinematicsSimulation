//! What a machine looks like, as data. A machine never renders; a front end draws these parts.
//! Coordinates in metres: x to the right, y into the depth, z up.

#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// Box centred on `pos`.
    Box { size: [f64; 3] },
    /// Cylinder centred on `pos`, axis along y (pulleys and motors seen from the front).
    Cylinder { radius: f64, length: f64 },
    /// Straight strip from `pos` to `to` (a belt span).
    Segment { to: [f64; 3], width: f64 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Part {
    pub id: &'static str,
    pub shape: Shape,
    pub pos: [f64; 3],
    /// Rotation about the depth axis in radians, from +x towards +z: what turns a pulley's spoke.
    pub angle: f64,
    pub color: [f32; 3],
    /// -1..1, drawn as a colour shift: belt tension below or above pretension.
    pub highlight: f32,
    /// Temperature rise as a share of the part's rated limit, where it is computed: drawn from cold to hot in place
    /// of the part's colour. Set by the simulation, not by the machine.
    pub heat: Option<f32>,
}

impl Part {
    pub fn new(id: &'static str, shape: Shape, pos: [f64; 3], color: [f32; 3]) -> Self {
        Self { id, shape, pos, angle: 0.0, color, highlight: 0.0, heat: None }
    }
    pub fn turned(mut self, angle: f64) -> Self {
        self.angle = angle;
        self
    }
    pub fn highlighted(mut self, highlight: f32) -> Self {
        self.highlight = highlight;
        self
    }
}

/// A live value shown in the 3D view next to the part it belongs to, e.g. a motor's speed.
/// Both keys are translation keys; the front end formats "owner value unit".
#[derive(Debug, Clone, PartialEq)]
pub struct Label {
    pub pos: [f64; 3],
    pub owner_key: &'static str,
    pub value: f64,
    pub unit_key: &'static str,
    pub decimals: usize,
}

/// A force that acts at a place in the machine, drawn as an arrow: on what (`owner_key`, a translation key)
/// and how much, in newtons, along the scene's axes.
#[derive(Debug, Clone, PartialEq)]
pub struct Arrow {
    pub pos: [f64; 3],
    pub force: [f64; 3],
    pub owner_key: &'static str,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Scene {
    pub parts: Vec<Part>,
    pub labels: Vec<Label>,
    /// Forces worth seeing, e.g. the belt's pull on the carriage. Filled in by the machine.
    pub arrows: Vec<Arrow>,
    /// Where the tool is in the scene right now (`Machine::scene_point` of its pose), for drawing the path
    /// it travels. Filled in by whoever steps the machine, not by the machine; `None` where there is no mapping.
    pub tool: Option<[f64; 3]>,
    /// Where the plan has the tool now (`Machine::scene_point` of the planned pose), filled in like `tool`.
    pub planned: Option<[f64; 3]>,
    /// Asked of the machine, not filled in by it: how many times larger to draw the parts' deflections (frame
    /// sway, the Z axis leaning), which are micrometres. 1 draws them as they are.
    pub exaggeration: f64,
    /// The obstacles of the config, each marked while a part of the machine touches it. Filled in like `tool`.
    pub obstacles: Vec<Obstacle>,
    /// Where there are obstacles: the nearest points of the machine's parts and of an obstacle (`Aabb::gap`), and the
    /// clearance between them, m. Filled in like `tool`.
    pub clearance: Option<([f64; 3], [f64; 3], f64)>,
    /// The simulated time of the first contact with an obstacle so far, s. Filled in like `tool`.
    pub first_contact: Option<f64>,
}

impl Default for Scene {
    fn default() -> Self {
        Self {
            parts: Vec::new(),
            labels: Vec::new(),
            arrows: Vec::new(),
            tool: None,
            planned: None,
            exaggeration: 1.0,
            obstacles: Vec::new(),
            clearance: None,
            first_contact: None,
        }
    }
}

/// A box with its edges along the scene's axes: an obstacle, or a part of the machine that can hit one
/// (`Machine::collision_bodies`). Metres, in scene coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Aabb {
    pub centre: [f64; 3],
    pub size: [f64; 3],
}

impl Aabb {
    /// The nearest points of the two boxes, one on each, and the clearance between them: the distance of the two
    /// points; where the boxes overlap, the depth of the overlap, negative: how far one box would have to move to part
    /// from the other, along the axis where that is least (both points then lie in the middle of the overlap). A box
    /// inside the other along an axis has to leave by the nearer face; the intersection's width (the depth until 14
    /// November) is the depth only where neither box holds the other, and was 0 for a point inside a box.
    pub fn gap(&self, other: &Aabb) -> ([f64; 3], [f64; 3], f64) {
        let (mut p, mut q, mut apart, mut overlap) = ([0.0; 3], [0.0; 3], 0.0, f64::INFINITY);
        for i in 0..3 {
            let (lo, hi) = (self.centre[i] - self.size[i] / 2.0, self.centre[i] + self.size[i] / 2.0);
            let (other_lo, other_hi) = (other.centre[i] - other.size[i] / 2.0, other.centre[i] + other.size[i] / 2.0);
            (p[i], q[i]) = if hi < other_lo {
                (hi, other_lo)
            } else if other_hi < lo {
                (lo, other_hi)
            } else {
                overlap = f64::min(overlap, f64::min(hi - other_lo, other_hi - lo));
                let middle = (lo.max(other_lo) + hi.min(other_hi)) / 2.0;
                (middle, middle)
            };
            apart += (q[i] - p[i]).powi(2);
        }
        (p, q, if apart > 0.0 { apart.sqrt() } else { -overlap })
    }
}

/// A capsule: every point within `radius` of the segment from `from` to `to`. A slanted rod, arm or shaft as a part
/// that can hit an obstacle (`Machine::collision_capsules`), where a box along the scene's axes around it would be up to
/// its length wider than it. Metres, in scene coordinates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Capsule {
    pub from: [f64; 3],
    pub to: [f64; 3],
    pub radius: f64,
}

impl Capsule {
    /// The nearest points of the capsule and the box, one on each, and the clearance between them, as `Aabb::gap` has
    /// them: the least signed distance of the segment's points to the box, less the radius. Negative in an overlap,
    /// where it is the depth to the box face nearest to the segment's deepest point (both points then on the
    /// segment). The signed distance of a point to a box is convex along a line, so its least along the segment is
    /// found by a golden-section search, to the last bit of the segment's parameter: exact, as is the box's.
    pub fn gap(&self, other: &Aabb) -> ([f64; 3], [f64; 3], f64) {
        let at = |t: f64| -> [f64; 3] { std::array::from_fn(|i| self.from[i] + t * (self.to[i] - self.from[i])) };
        let signed = |t: f64| {
            let q: [f64; 3] = std::array::from_fn(|i| (at(t)[i] - other.centre[i]).abs() - other.size[i] / 2.0);
            q.iter().map(|v| v.max(0.0).powi(2)).sum::<f64>().sqrt() + q[0].max(q[1]).max(q[2]).min(0.0)
        };
        let golden = (5f64.sqrt() - 1.0) / 2.0;
        let (mut a, mut b) = (0.0, 1.0);
        let (mut c, mut d) = (b - golden * (b - a), a + golden * (b - a));
        let (mut fc, mut fd) = (signed(c), signed(d));
        // 0.618^80 is 2e-17: the interval is down to the last bit of the parameter.
        for _ in 0..80 {
            if fc < fd {
                (b, d, fd) = (d, c, fc);
                c = b - golden * (b - a);
                fc = signed(c);
            } else {
                (a, c, fc) = (c, d, fd);
                d = a + golden * (b - a);
                fd = signed(d);
            }
        }
        let t = [0.0, 1.0, 0.5 * (a + b)].into_iter().min_by(|x, y| signed(*x).total_cmp(&signed(*y))).unwrap_or(0.0);
        let (p, distance) = (at(t), signed(t));
        if distance <= 0.0 {
            return (p, p, distance - self.radius);
        }
        let q: [f64; 3] = std::array::from_fn(|i| {
            p[i].clamp(other.centre[i] - other.size[i] / 2.0, other.centre[i] + other.size[i] / 2.0)
        });
        let surface = std::array::from_fn(|i| p[i] + (q[i] - p[i]) * (self.radius / distance).min(1.0));
        (surface, q, distance - self.radius)
    }
}

/// An obstacle of the config (`[[obstacles]]`), and whether a part of the machine touches it now. With effects.crash
/// it pushes back on a part that goes into it with `stiffness` (N/m) times the depth and `damping` (N s/m) times the
/// speed into it.
#[derive(Debug, Clone, PartialEq)]
pub struct Obstacle {
    pub name: String,
    pub aabb: Aabb,
    pub contact: bool,
    pub stiffness: f64,
    pub damping: f64,
}

/// The boxes among `parts` (as a machine draws them in its scene) whose id is one of `ids`, turned or not: what
/// `Machine::collision_bodies` takes for the parts that move with the tool. A machine passes only the parts it needs:
/// drawing the whole scene every base step made the live check about three times as slow as the run without it.
pub fn scene_boxes(parts: &[Part], ids: &[&str]) -> Vec<Aabb> {
    parts
        .iter()
        .filter(|part| ids.contains(&part.id))
        .filter_map(|part| match part.shape {
            Shape::Box { size } => Some(Aabb { centre: part.pos, size: size.map(f64::abs) }),
            _ => None,
        })
        .collect()
}

/// The colour of the heat scale at a temperature rise of `heat` times the rated one: cold blue, yellow at half, red at
/// the rated rise, and magenta beyond it.
pub fn heat_color(heat: f32) -> [f32; 3] {
    const SCALE: [[f32; 3]; 3] = [[0.15, 0.3, 0.75], [0.95, 0.8, 0.1], [0.95, 0.1, 0.05]];
    if heat > 1.0 {
        return [0.9, 0.2, 0.9];
    }
    let at = 2.0 * heat.max(0.0);
    let (i, f) = if at < 1.0 { (0, at) } else { (1, at - 1.0) };
    std::array::from_fn(|c| SCALE[i][c] + (SCALE[i + 1][c] - SCALE[i][c]) * f)
}

/// The colour a 3D view draws a part in: where its warmth is computed (`Part::heat`) the heat scale in its place;
/// otherwise its own colour, shifted towards red by belt tension above pretension (`highlight` up to 1, at twice the
/// pretension) and towards blue below it (-1 at no tension).
pub fn part_color(color: [f32; 3], highlight: f32, heat: Option<f32>) -> [f32; 3] {
    if let Some(heat) = heat {
        return heat_color(heat);
    }
    let h = highlight.clamp(-1.0, 1.0);
    let (toward, amount) = if h >= 0.0 { ([1.0, 0.1, 0.1], h) } else { ([0.1, 0.3, 1.0], -h) };
    std::array::from_fn(|c| color[c] + (toward[c] - color[c]) * amount)
}

/// A point on a circle, for laying a belt path around a pulley. `turn` is in radians from +x.
pub fn on_circle(centre: [f64; 2], radius: f64, turn: f64) -> [f64; 2] {
    [centre[0] + radius * turn.cos(), centre[1] + radius * turn.sin()]
}

/// The colour of a belt strip at pretension; its highlight shifts it.
pub const BELT_COLOR: [f32; 3] = [0.08, 0.08, 0.09];

/// Draws a belt along a path in the x-z plane at depth `y`: one strip per path segment, and teeth
/// at a fixed spacing of belt length counted from the start of the path.
///
/// Start the path where the belt is clamped. Then the teeth sit at fixed places on the belt, and
/// when the path changes with the pose they travel along it and around the pulleys by themselves.
/// `shade(i)` is the highlight of the strip from point i to point i + 1.
pub fn belt(
    out: &mut Vec<Part>,
    path: &[[f64; 2]],
    y: f64,
    width: f64,
    tooth_spacing: f64,
    shade: impl Fn(usize) -> f32,
) {
    let along: Vec<f64> = std::iter::once(0.0)
        .chain(path.windows(2).scan(0.0, |sum, p| {
            *sum += (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]);
            Some(*sum)
        }))
        .collect();
    belt_along(out, path, &along, y, width, tooth_spacing, shade);
}

/// `belt` with the belt length of each path point given (`along`, rising) rather than taken from the drawn path:
/// for a drawn path that is not the belt's length, e.g. a span drawn sagging larger than it is. Each tooth keeps its
/// place in belt length and sits on the drawn segment whose ends' belt lengths hold it, at the same share between them.
pub fn belt_along(
    out: &mut Vec<Part>,
    path: &[[f64; 2]],
    along: &[f64],
    y: f64,
    width: f64,
    tooth_spacing: f64,
    shade: impl Fn(usize) -> f32,
) {
    let (strip, tooth) = (BELT_COLOR, [0.75, 0.75, 0.7]);
    let mut next_tooth = tooth_spacing / 2.0;
    for (i, pair) in path.windows(2).enumerate() {
        let (a, b, (c0, c1)) = (pair[0], pair[1], (along[i], along[i + 1]));
        let (dx, dz) = (b[0] - a[0], b[1] - a[1]);
        if dx.hypot(dz) < 1e-9 || c1 - c0 < 1e-9 {
            continue;
        }
        out.push(
            Part::new("belt", Shape::Segment { to: [b[0], y, b[1]], width }, [a[0], y, a[1]], strip)
                .highlighted(shade(i)),
        );
        while next_tooth < c1 {
            let f = (next_tooth - c0) / (c1 - c0);
            let pos = [a[0] + f * dx, y - 0.001, a[1] + f * dz];
            out.push(
                Part::new("belt-tooth", Shape::Box { size: [0.004, width, 0.007] }, pos, tooth).turned(dz.atan2(dx)),
            );
            next_tooth += tooth_spacing;
        }
    }
}

/// The id of a motor's housing in the scene (`drive_unit`): the k-th such part is motor k's, and the simulation
/// colours it by the warmth of that motor's winding.
pub const MOTOR_PART: &str = "motor";

/// Draws a motor with gearbox and belt pulley at `centre` (x, z), the pulley in the belt plane
/// at depth `y` and the motor behind it. A cross on the pulley turns with the pulley, a bar on
/// the hub turns with the motor shaft, so both speeds can be seen.
pub fn drive_unit(
    out: &mut Vec<Part>,
    centre: [f64; 2],
    y: f64,
    radius: f64,
    pulley_angle: f64,
    rotor_angle: f64,
    color: [f32; 3],
) {
    let at = |depth: f64| [centre[0], y + depth, centre[1]];
    out.extend([
        Part::new(MOTOR_PART, Shape::Box { size: [0.075, 0.13, 0.075] }, at(0.15), color),
        Part::new("gearbox", Shape::Cylinder { radius: 0.035, length: 0.07 }, at(0.05), [0.25, 0.25, 0.28]),
        Part::new("pulley", Shape::Cylinder { radius, length: 0.03 }, at(0.0), [0.55, 0.55, 0.6]),
        Part::new(
            "pulley-spoke",
            Shape::Box { size: [1.8 * radius, 0.004, 0.22 * radius] },
            at(-0.017),
            [0.95, 0.95, 0.95],
        )
        .turned(pulley_angle),
        Part::new(
            "pulley-spoke",
            Shape::Box { size: [0.22 * radius, 0.004, 1.8 * radius] },
            at(-0.017),
            [0.2, 0.2, 0.2],
        )
        .turned(pulley_angle),
        Part::new("rotor-hub", Shape::Cylinder { radius: 0.5 * radius, length: 0.012 }, at(-0.024), [0.95, 0.75, 0.1]),
        Part::new("rotor-bar", Shape::Box { size: [0.9 * radius, 0.004, 0.2 * radius] }, at(-0.032), [0.1, 0.1, 0.1])
            .turned(rotor_angle),
    ]);
}

/// Draws a free-running pulley with a bar that shows its rotation.
pub fn idler(out: &mut Vec<Part>, centre: [f64; 2], y: f64, radius: f64, angle: f64) {
    out.extend([
        Part::new("idler", Shape::Cylinder { radius, length: 0.03 }, [centre[0], y, centre[1]], [0.5, 0.5, 0.55]),
        Part::new(
            "idler-bar",
            Shape::Box { size: [1.7 * radius, 0.004, 0.25 * radius] },
            [centre[0], y - 0.017, centre[1]],
            [0.9, 0.9, 0.9],
        )
        .turned(angle),
    ]);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// By hand: a 0.2 m cube at the origin and a 0.1 m cube at (0.4, 0.3, 0): 0.25 m apart in x (0.4 - 0.1 - 0.05),
    /// 0.15 m in y, overlapping in z, so 0.29155 m apart. Moved to (0.12, 0, 0.02) they overlap by 0.03 m in x, the
    /// least of 0.03, 0.1 and 0.1.
    /// By hand: a point at (0.02, 0, 0.01) inside a 0.1 m cube at the origin leaves it by the nearest face, x = 0.05,
    /// 0.03 m away (z = 0.05 is 0.04 away). A 0.02 m cube there leaves by 0.03 + 0.01 = 0.04 m.
    #[test]
    fn a_box_inside_another_is_as_deep_as_its_way_out() {
        let cube = Aabb { centre: [0.0; 3], size: [0.1; 3] };
        let (_, _, d) = cube.gap(&Aabb { centre: [0.02, 0.0, 0.01], size: [0.0; 3] });
        assert!((d + 0.03).abs() < 1e-12, "{d}");
        let (_, _, d) = cube.gap(&Aabb { centre: [0.02, 0.0, 0.01], size: [0.02; 3] });
        assert!((d + 0.04).abs() < 1e-12, "{d}");
    }

    #[test]
    fn gap_between_boxes_by_hand() {
        let big = Aabb { centre: [0.0; 3], size: [0.2; 3] };
        let (p, q, d) = big.gap(&Aabb { centre: [0.4, 0.3, 0.0], size: [0.1; 3] });
        assert!((d - (0.25f64.powi(2) + 0.15f64.powi(2)).sqrt()).abs() < 1e-12, "{d}");
        let near = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12);
        assert!(near(p, [0.1, 0.1, 0.0]) && near(q, [0.35, 0.25, 0.0]), "{p:?} {q:?}");
        let (_, _, d) = big.gap(&Aabb { centre: [0.12, 0.0, 0.02], size: [0.1; 3] });
        assert!((d + 0.03).abs() < 1e-12, "{d}");
    }

    /// By hand: a rod of radius 6 mm slanted at 45 degrees in the x-z plane, from (0, 0, 0) to (0.3, 0, 0.3), and a
    /// 0.1 m cube centred at (0.25, 0, 0.05). The cube's top edge nearest to the rod runs along y through
    /// (0.2, y, 0.1); the rod's line x = z is (0.2 - 0.1) / sqrt(2) = 70.711 mm from it, its nearest point (0.15, 0,
    /// 0.15) within the segment, so the clearance is 70.711 - 6 = 64.711 mm. The old box around the rod (0.312 m wide)
    /// overlapped the cube. Shifted to centre (0.15, 0, 0.1), the cube holds the rod's point (0.125, 0, 0.125), 25 mm
    /// inside its top face and its left face: the overlap is 25 + 6 = 31 mm deep.
    #[test]
    fn a_slanted_capsule_keeps_the_hand_calculated_clearance_to_a_box() {
        let rod = Capsule { from: [0.0; 3], to: [0.3, 0.0, 0.3], radius: 0.006 };
        let (p, q, d) = rod.gap(&Aabb { centre: [0.25, 0.0, 0.05], size: [0.1; 3] });
        assert!((d - (0.1 / 2f64.sqrt() - 0.006)).abs() < 1e-12, "{d}");
        let near = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9);
        let s = 0.006 / 2f64.sqrt();
        assert!(near(q, [0.2, 0.0, 0.1]) && near(p, [0.15 + s, 0.0, 0.15 - s]), "{p:?} {q:?}");
        let boxed = Aabb { centre: [0.15, 0.0, 0.15], size: [0.312, 0.012, 0.312] };
        assert!(boxed.gap(&Aabb { centre: [0.25, 0.0, 0.05], size: [0.1; 3] }).2 < 0.0);
        let (_, _, d) = rod.gap(&Aabb { centre: [0.15, 0.0, 0.1], size: [0.1; 3] });
        assert!((d + 0.031).abs() < 1e-12, "{d}");
    }

    #[test]
    fn belt_teeth_keep_their_place_on_the_belt_when_the_path_changes() {
        let teeth = |path: &[[f64; 2]]| {
            let mut parts = Vec::new();
            belt(&mut parts, path, 0.0, 0.02, 0.1, |_| 0.0);
            parts.into_iter().filter(|p| p.id == "belt-tooth").map(|p| p.pos).collect::<Vec<_>>()
        };
        // An L-shaped path of 1 m: 0.6 m right, then 0.4 m up. Ten teeth, 0.1 m apart.
        let before = teeth(&[[0.0, 0.0], [0.6, 0.0], [0.6, 0.4]]);
        assert_eq!(before.len(), 10);
        assert!((before[0][0] - 0.05).abs() < 1e-12 && (before[9][2] - 0.35).abs() < 1e-12);
        // The corner moves 0.1 m to the left: the same belt, one tooth more on the vertical part.
        let after = teeth(&[[0.0, 0.0], [0.5, 0.0], [0.5, 0.5]]);
        assert_eq!(after.len(), 10);
        assert_eq!(before.iter().filter(|p| p[2] > 0.0).count() + 1, after.iter().filter(|p| p[2] > 0.0).count());
    }

    #[test]
    fn belt_strips_carry_the_shade_of_their_section() {
        let mut parts = Vec::new();
        belt(&mut parts, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], 0.0, 0.02, 10.0, |i| i as f32 * 0.5);
        let shades: Vec<f32> = parts.iter().filter(|p| p.id == "belt").map(|p| p.highlight).collect();
        assert_eq!(shades, vec![0.0, 0.5]);
    }
}
