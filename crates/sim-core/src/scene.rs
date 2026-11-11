//! What a machine looks like, as data. A machine never renders; a front end draws these parts.
//! Coordinates in metres: x to the right, y into the depth, z up.

#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// Axis-aligned box centred on `pos`.
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
    pub color: [f32; 3],
    /// -1..1, drawn as a colour shift: belt tension below or above pretension.
    pub highlight: f32,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scene {
    pub parts: Vec<Part>,
}
