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
}

impl Part {
    pub fn new(id: &'static str, shape: Shape, pos: [f64; 3], color: [f32; 3]) -> Self {
        Self { id, shape, pos, angle: 0.0, color, highlight: 0.0 }
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

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scene {
    pub parts: Vec<Part>,
    pub labels: Vec<Label>,
    /// Where the tool is in the scene right now (`Machine::scene_point` of its pose), for drawing the path
    /// it travels. Filled in by whoever steps the machine, not by the machine; `None` where there is no mapping.
    pub tool: Option<[f64; 3]>,
}

/// A point on a circle, for laying a belt path around a pulley. `turn` is in radians from +x.
pub fn on_circle(centre: [f64; 2], radius: f64, turn: f64) -> [f64; 2] {
    [centre[0] + radius * turn.cos(), centre[1] + radius * turn.sin()]
}

/// Draws a belt along a path in the x-z plane at depth `y`: one strip per path segment, and teeth
/// at a fixed spacing of belt length counted from the start of the path.
///
/// Start the path where the belt is clamped. Then the teeth sit at fixed places on the belt, and
/// when the path changes with the pose they travel along it and around the pulleys by themselves.
/// `shade(i)` is the highlight of the strip from point i to point i + 1.
pub fn belt(out: &mut Vec<Part>, path: &[[f64; 2]], y: f64, width: f64, tooth_spacing: f64, shade: impl Fn(usize) -> f32) {
    let (strip, tooth) = ([0.08, 0.08, 0.09], [0.75, 0.75, 0.7]);
    let mut along = 0.0;
    let mut next_tooth = tooth_spacing / 2.0;
    for (i, pair) in path.windows(2).enumerate() {
        let (a, b) = (pair[0], pair[1]);
        let (dx, dz) = (b[0] - a[0], b[1] - a[1]);
        let length = dx.hypot(dz);
        if length < 1e-9 {
            continue;
        }
        out.push(Part::new("belt", Shape::Segment { to: [b[0], y, b[1]], width }, [a[0], y, a[1]], strip).highlighted(shade(i)));
        while next_tooth < along + length {
            let f = (next_tooth - along) / length;
            let pos = [a[0] + f * dx, y - 0.001, a[1] + f * dz];
            out.push(Part::new("belt-tooth", Shape::Box { size: [0.004, width, 0.007] }, pos, tooth).turned(dz.atan2(dx)));
            next_tooth += tooth_spacing;
        }
        along += length;
    }
}

/// Draws a motor with gearbox and belt pulley at `centre` (x, z), the pulley in the belt plane
/// at depth `y` and the motor behind it. A cross on the pulley turns with the pulley, a bar on
/// the hub turns with the motor shaft, so both speeds can be seen.
pub fn drive_unit(out: &mut Vec<Part>, centre: [f64; 2], y: f64, radius: f64, pulley_angle: f64, rotor_angle: f64, color: [f32; 3]) {
    let at = |depth: f64| [centre[0], y + depth, centre[1]];
    out.extend([
        Part::new("motor", Shape::Box { size: [0.075, 0.13, 0.075] }, at(0.15), color),
        Part::new("gearbox", Shape::Cylinder { radius: 0.035, length: 0.07 }, at(0.05), [0.25, 0.25, 0.28]),
        Part::new("pulley", Shape::Cylinder { radius, length: 0.03 }, at(0.0), [0.55, 0.55, 0.6]),
        Part::new("pulley-spoke", Shape::Box { size: [1.8 * radius, 0.004, 0.22 * radius] }, at(-0.017), [0.95, 0.95, 0.95]).turned(pulley_angle),
        Part::new("pulley-spoke", Shape::Box { size: [0.22 * radius, 0.004, 1.8 * radius] }, at(-0.017), [0.2, 0.2, 0.2]).turned(pulley_angle),
        Part::new("rotor-hub", Shape::Cylinder { radius: 0.5 * radius, length: 0.012 }, at(-0.024), [0.95, 0.75, 0.1]),
        Part::new("rotor-bar", Shape::Box { size: [0.9 * radius, 0.004, 0.2 * radius] }, at(-0.032), [0.1, 0.1, 0.1]).turned(rotor_angle),
    ]);
}

/// Draws a free-running pulley with a bar that shows its rotation.
pub fn idler(out: &mut Vec<Part>, centre: [f64; 2], y: f64, radius: f64, angle: f64) {
    out.extend([
        Part::new("idler", Shape::Cylinder { radius, length: 0.03 }, [centre[0], y, centre[1]], [0.5, 0.5, 0.55]),
        Part::new("idler-bar", Shape::Box { size: [1.7 * radius, 0.004, 0.25 * radius] }, [centre[0], y - 0.017, centre[1]], [0.9, 0.9, 0.9]).turned(angle),
    ]);
}

#[cfg(test)]
mod tests {
    use super::*;

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
