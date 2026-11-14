//! The `[machine]` part of a machine file: the node lists, what each node may hold, and how they are read and
//! checked. `NODES` is the schema: the parser refuses a field it does not list, and `json_schema` is written from it.

use sim_core::{Config, ConfigError};
use toml::{Table, Value};

type Result<T> = std::result::Result<T, ConfigError>;

/// The version of the `[machine]` part. A file states it as `machine.version`; a later format that reads files
/// differently gets a new number, and this one refuses it.
pub const VERSION: i64 = 1;

/// What a field holds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Kind {
    /// The node's own id: unique among all nodes of the file, letters, digits, `_` and `-`.
    Id,
    /// A translation key (shown as is where no translation has it).
    Key,
    /// A number, or the dotted path of a number elsewhere in the file (`"geometry.stroke_m"`), which is then the
    /// value the parameter panel, the catalogue and the sensitivity analysis change.
    Number,
    Bool,
    /// The name of a table of the file.
    Table,
    /// The id of a node of the named list.
    Ref(&'static str),
    /// Ids of nodes of the named list.
    Refs(&'static str),
    /// A table from ids of nodes of the named list to numbers.
    Coefficients(&'static str),
    /// Where an editor draws the node, [x, y]; the simulation does not read it.
    Position,
    /// Where the 3D view draws the node, [x, y, z] in metres.
    Point,
    /// Ids of nodes of any of the named lists, in an order that means something (a belt's way past them).
    Route(&'static [&'static str]),
}

pub struct Field {
    pub name: &'static str,
    pub kind: Kind,
    pub required: bool,
    pub doc: &'static str,
}

const fn field(name: &'static str, kind: Kind, required: bool, doc: &'static str) -> Field {
    Field { name, kind, required, doc }
}

const ID: Field = field("id", Kind::Id, true, "Stable id of the node; signals and other nodes refer to it.");
const POS: Field =
    field("pos", Kind::Position, false, "Where a node editor draws the node, [x, y]. Not read by the simulation.");
const CLOCKWISE: Field = field(
    "clockwise",
    Kind::Bool,
    false,
    "With effects.belt_wrap: the belt runs round it clockwise, seen from the front (x to the right, z up) and followed along the routes in their order. Default false (anticlockwise).",
);

/// The node lists of `[machine]`, in the order they are read, each with its fields.
pub const NODES: &[(&str, &str, &[Field])] = &[
    (
        "axis",
        "A tool axis: a straight coordinate of the tool, from 0 to its stroke. Its limits are in [axis.<id>].",
        &[
            ID,
            field("key", Kind::Key, false, "Translation key of its name; default axis-<id>."),
            field("stroke_m", Kind::Number, false, "Stroke, m; every axis but a rotary one has it."),
            field(
                "stroke_rad",
                Kind::Number,
                false,
                "Range of a rotary axis (one a rotary node turns), from 0, rad; in place of stroke_m.",
            ),
            field(
                "incline_deg",
                Kind::Number,
                false,
                "Angle of the axis above the horizontal, deg; 90 is vertical, rising towards +. Default 0.",
            ),
            field(
                "cable_chain",
                Kind::Bool,
                false,
                "The axis drags a cable chain fixed at its 0 end (effects.cable_chain, values in [cable_chain]). Default false.",
            ),
            POS,
        ],
    ),
    (
        "motor",
        "A servo motor with its drive; its data is in [motor] and [drive], shared by all motors; [motor.<id>], [drive.<id>], [gearbox.<id>], [ratings.<id>] and [parts.<id>] give one motor its own.",
        &[ID, field("key", Kind::Key, false, "Translation key of its name; default motor-<id>."), POS],
    ),
    (
        "gearbox",
        "The gearbox on a motor's shaft; its data is in [gearbox], shared by all gearboxes. A ratio of 1 is a coupling.",
        &[ID, field("motor", Kind::Ref("motor"), true, "The motor that drives it."), POS],
    ),
    (
        "pulley",
        "The toothed pulley on a gearbox's output, which feeds belt into the sections it ends.",
        &[
            ID,
            field("gearbox", Kind::Ref("gearbox"), true, "The gearbox that turns it."),
            field("radius_m", Kind::Number, true, "Pitch radius, m."),
            field("inertia_kgm2", Kind::Number, true, "Inertia without the belt, kg m^2."),
            field(
                "at_m",
                Kind::Point,
                false,
                "Where the 3D view draws it, [x, y, z], m; default at an end of the first axis. With effects.belt_wrap its place in the belt's plane (x, z), which the wrap is worked out from.",
            ),
            CLOCKWISE,
            POS,
        ],
    ),
    (
        "section",
        "A belt section: the belt between two driven pulleys, or between a driven pulley and a clamp. Its tension is the same all along; idlers it runs over count as belt.",
        &[
            ID,
            field("key", Kind::Key, false, "Translation key of its name; default belt-section-<id>."),
            field("length_m", Kind::Number, true, "Length with every axis at 0, m."),
            field("axes", Kind::Coefficients("axis"), true, "How much longer the section gets per metre of each axis."),
            field(
                "pulleys",
                Kind::Coefficients("pulley"),
                true,
                "+1 for a pulley whose forward turn feeds belt into the section, -1 for one that takes belt out of it.",
            ),
            field(
                "route",
                Kind::Route(&["pulley", "idler", "body"]),
                false,
                "The pulleys, idlers and bodies (a clamp on it) the section runs past, in order, for the 3D view; default its pulleys and the body it is clamped to.",
            ),
            POS,
        ],
    ),
    (
        "body",
        "A rigid body the axes move: a carriage, a vertical axis.",
        &[
            ID,
            field("mass_kg", Kind::Number, true, "Mass, kg."),
            field(
                "payload_kg",
                Kind::Number,
                false,
                "Payload it carries while effects.payload is on and the payload is attached, kg.",
            ),
            field("axes", Kind::Refs("axis"), true, "The axes it moves along: it moves with each of them."),
            POS,
        ],
    ),
    (
        "guide",
        "The linear guide of an axis, with its friction (LuGre with Stribeck, see mech::Lugre). Its id names its loss power, power.<id>, and its warmth.",
        &[
            ID,
            field("axis", Kind::Ref("axis"), true, "The axis it guides; one guide per axis at most."),
            field(
                "friction",
                Kind::Table,
                true,
                "The table of its friction values, laid out as [friction_x] of the shipped files.",
            ),
            field("carries", Kind::Refs("body"), false, "Bodies whose weight across the axis it carries."),
            field(
                "overhang",
                Kind::Ref("body"),
                false,
                "A body that overhangs the guide: accelerated along overhang_axis it tilts the guide's carriages.",
            ),
            field(
                "overhang_axis",
                Kind::Ref("axis"),
                false,
                "The axis along which the overhang's acceleration tilts the guide.",
            ),
            field(
                "lever_m",
                Kind::Number,
                false,
                "Distance of the overhang's centre of mass from the guide with lever_axis at 0, m.",
            ),
            field(
                "lever_axis",
                Kind::Ref("axis"),
                false,
                "An axis whose travel shortens the lever (the overhang rises towards the guide).",
            ),
            field(
                "spacing_m",
                Kind::Number,
                false,
                "Spacing of the guide's two carriages, m; the tilt puts 2 m a lever / spacing on them.",
            ),
            field(
                "side_load",
                Kind::Bool,
                false,
                "The guide also carries the overhang's inertia force m a sideways. Default false.",
            ),
            POS,
        ],
    ),
    (
        "idler",
        "A free pulley the belt runs over (effects.idlers): its inertia and bearing friction, at the belt speed past it.",
        &[
            ID,
            field(
                "speed",
                Kind::Coefficients("axis"),
                true,
                "Belt speed past it per speed of each axis (m/s per m/s).",
            ),
            field("radius_m", Kind::Number, true, "Radius, m."),
            field("inertia_kgm2", Kind::Number, true, "Inertia, kg m^2."),
            field("friction_nm", Kind::Number, true, "Bearing friction torque, N m."),
            field(
                "body",
                Kind::Ref("body"),
                false,
                "A body it rides on, for the 3D view: at_m is then from that body.",
            ),
            field(
                "at_m",
                Kind::Point,
                false,
                "Where the 3D view draws it, [x, y, z], m (from its body where it has one); with effects.belt_wrap its place in the belt's plane (x, z).",
            ),
            CLOCKWISE,
            POS,
        ],
    ),
    (
        "frame",
        "The machine frame swaying along a horizontal axis on its own spring (effects.frame_compliance, elastic model only). At most one.",
        &[
            ID,
            field("axis", Kind::Ref("axis"), true, "The axis the frame sways along; it has to be horizontal."),
            field(
                "mass_kg",
                Kind::Number,
                true,
                "The frame's mass in its sway, kg; bodies that do not move along the axis ride on it too.",
            ),
            field("stiffness_n_m", Kind::Number, true, "Its stiffness, N/m."),
            field("damping_n_s_m", Kind::Number, true, "Its damping, N s/m."),
            POS,
        ],
    ),
    (
        "span",
        "A free belt span between two points that hold it (pulley, idler, clamp), for its sideways mode (effects.belt_span_vibration).",
        &[
            ID,
            field("section", Kind::Ref("section"), true, "The section it is part of: its tension."),
            field("length_m", Kind::Number, true, "Length with every axis at 0, m."),
            field("axes", Kind::Coefficients("axis"), false, "How much longer it gets per metre of each axis."),
            POS,
        ],
    ),
    (
        "screw",
        "A ball screw on a gearbox's output (a gearbox of ratio 1 is the coupling) that drives an axis through its nut (transmission::BallScrew; checks screw-speed, screw-dn, screw-buckling, screw-static, screw-life). Its signals and checks are named by its motor: screw.<motor>.speed, joint.<motor>.force.",
        &[
            ID,
            field("gearbox", Kind::Ref("gearbox"), true, "The gearbox that turns it."),
            field(
                "axis",
                Kind::Ref("axis"),
                true,
                "The axis its nut drives; one screw, rack, linear motor or rotary per axis.",
            ),
            field(
                "values",
                Kind::Table,
                true,
                "The table of its values, laid out as [screw] of the registered screw axis (lead, diameters, length, mounting, efficiency, nut and bearing stiffness, preload, load ratings).",
            ),
            POS,
        ],
    ),
    (
        "rack",
        "A pinion on a gearbox's output that drives an axis along a rack (transmission::RackPinion; check rack-feed-force on rack.<motor>.feed_force).",
        &[
            ID,
            field("gearbox", Kind::Ref("gearbox"), true, "The gearbox that turns the pinion."),
            field(
                "axis",
                Kind::Ref("axis"),
                true,
                "The axis it drives; one screw, rack, linear motor or rotary per axis.",
            ),
            field(
                "values",
                Kind::Table,
                true,
                "The table of its values, laid out as [rack] of the registered rack axis (module, teeth, helix angle, efficiency, stiffness, backlash, pinion inertia, rated feed force).",
            ),
            POS,
        ],
    ),
    (
        "linear_motor",
        "An ironcore linear motor that drives an axis with no transmission (transmission::LinearMotor). Its motor's values read in N for N m and m for rad ([motor.<id>]: force constant in N/A, the forcer's mass as rotor_inertia_kgm2); its gearbox values ([gearbox.<id>], ratio 1) are the forcer's mounting.",
        &[
            ID,
            field("motor", Kind::Ref("motor"), true, "The motor it is; that motor has no gearbox node."),
            field(
                "axis",
                Kind::Ref("axis"),
                true,
                "The axis it drives; one screw, rack, linear motor or rotary per axis.",
            ),
            field(
                "values",
                Kind::Table,
                true,
                "The table of its values, laid out as [linear_motor] of the registered linear motor axis (attraction force, pole pitch, guide friction coefficient).",
            ),
            POS,
        ],
    ),
    (
        "rotary",
        "A rotary table or joint on a gearbox's output: its axis turns, in rad. With strain_wave the gearbox is a strain wave gear (check strain-wave-life on strain_wave.<motor>.damage_rate).",
        &[
            ID,
            field("gearbox", Kind::Ref("gearbox"), true, "The gearbox that turns it."),
            field("axis", Kind::Ref("axis"), true, "The axis it turns, which has stroke_rad; no body moves with it."),
            field("inertia_kgm2", Kind::Number, true, "Inertia it turns, kg m^2."),
            field(
                "payload_inertia_kgm2",
                Kind::Number,
                false,
                "Inertia of its payload while effects.payload is on and the payload is attached, kg m^2.",
            ),
            field(
                "strain_wave",
                Kind::Table,
                false,
                "The table of the strain wave gear's life values, laid out as [strain_wave] of the registered rotary table.",
            ),
            POS,
        ],
    ),
    (
        "linkage",
        "A parallel linkage of driven arms, each with a pair of forearms (a parallelogram) down to a plate that keeps its orientation. Two arms (crate sim-netlist, linkage.rs): in the vertical x-z plane, one at each side of the base (the planar delta); the tool is the plate's centre, x and z. Three arms (spatial.rs): at 0, 120 and 240 degrees round the base (the spatial delta); the tool is x, y and z. A file with a linkage has as many axes as arms (no stroke: the working range below sets their range), one motor per arm, a gearbox on each and the linkage; nothing else, and no belt effect on. The arms' joints read [joint] (bearing friction, stops), each motor its drive train as everywhere.",
        &[
            ID,
            field(
                "arms",
                Kind::Refs("gearbox"),
                true,
                "The gearboxes that turn the arms: two (the first the arm at +x, the second the one at -x) or three (at 0, 120 and 240 degrees from +x).",
            ),
            field(
                "axes",
                Kind::Refs("axis"),
                true,
                "The tool's axes, as many as the arms: across (x), with three arms y, then up (z), from the shoulders' plane.",
            ),
            field("base_radius_m", Kind::Number, true, "Shoulder axis from the centre, m."),
            field("platform_radius_m", Kind::Number, true, "Forearms' joints on the plate from its centre, m."),
            field("upper_arm_m", Kind::Number, true, "Upper arm, shoulder to elbow, m."),
            field("forearm_m", Kind::Number, true, "Forearm, elbow to plate, m; longer than the upper arm."),
            field(
                "forearm_spacing_m",
                Kind::Number,
                false,
                "Distance between the two rods of a forearm pair, m; the 3D view only. Default 0.",
            ),
            field("upper_arm_kg", Kind::Number, true, "Upper arm's tube, kg."),
            field("upper_arm_cg_m", Kind::Number, true, "Its centre of mass from the shoulder axis, m."),
            field("upper_arm_inertia_kgm2", Kind::Number, true, "Its inertia about the shoulder axis, kg m^2."),
            field("elbow_kg", Kind::Number, true, "Elbow block at the arm's end, kg."),
            field("forearm_pair_kg", Kind::Number, true, "One forearm pair, kg."),
            field(
                "forearm_elbow_share",
                Kind::Number,
                true,
                "Share of a forearm pair's mass counted at the elbow, the rest at the plate, 0 to 1.",
            ),
            field("platform_kg", Kind::Number, true, "Plate, kg."),
            field(
                "payload_kg",
                Kind::Number,
                false,
                "Payload at the plate while effects.payload is on and the payload is attached, kg. Default 0.",
            ),
            field("half_width_m", Kind::Number, false, "Two arms: the working range's half width, m. Required there."),
            field(
                "radius_m",
                Kind::Number,
                false,
                "Three arms: the working range's radius (a cylinder about z), m. Required there.",
            ),
            field(
                "forearm_rod_spin_inertia_kgm2",
                Kind::Number,
                false,
                "Three arms, effects.forearm_inertia: each rod's inertia about its own axis, kg m^2. Default 0.",
            ),
            field(
                "bending_stiffness_nm_rad",
                Kind::Number,
                false,
                "Three arms, effects.link_elasticity: an upper arm's bending in the plane it turns in, as a turn of the elbow, N m/rad. Required with three arms.",
            ),
            field("bending_damping_nm_s_rad", Kind::Number, false, "Its damping, N m s/rad. Default 0."),
            field(
                "lateral_stiffness_n_m",
                Kind::Number,
                false,
                "Three arms, effects.link_elasticity: the elbow's move along its axis, out of the arm's plane, N/m. Required with three arms.",
            ),
            field("lateral_damping_n_s_m", Kind::Number, false, "Its damping, N s/m. Default 0."),
            field("top_z_m", Kind::Number, true, "Working range: its top, z, m (below 0: under the shoulders)."),
            field("height_edge_m", Kind::Number, true, "Working range: its height at the edges, m."),
            field(
                "height_centre_m",
                Kind::Number,
                true,
                "Working range: its height at the centre, m; the bottom is an arc (two arms) or a sphere's cap (three) through the edges and the centre.",
            ),
            field(
                "max_condition_number",
                Kind::Number,
                true,
                "Targets where the Jacobian's condition number is above this are refused (near a singularity).",
            ),
            POS,
        ],
    ),
    (
        "lean",
        "A body leaning on its carriage along a horizontal axis (effects.tool_flex, elastic model only): its centre of mass moves sideways by u against the guide, which gives way sideways and in tilt, and the tool tip below it moves with it. At most one.",
        &[
            ID,
            field("body", Kind::Ref("body"), true, "The body that leans; it moves with axis."),
            field("axis", Kind::Ref("axis"), true, "The horizontal axis it leans along."),
            field("com_drop_m", Kind::Number, true, "Its centre of mass below the guide with lever_axis at 0, m."),
            field("tool_drop_m", Kind::Number, true, "The tool tip below the guide with lever_axis at 0, m."),
            field(
                "lever_axis",
                Kind::Ref("axis"),
                false,
                "An axis whose travel raises both towards the guide (shortens the lever).",
            ),
            field("lateral_stiffness_n_m", Kind::Number, true, "The guide's stiffness sideways, N/m."),
            field("tilt_stiffness_nm_rad", Kind::Number, true, "The guide's stiffness in tilt, N m/rad."),
            field("damping_n_s_m", Kind::Number, true, "The damping of the lean at the centre of mass, N s/m."),
            POS,
        ],
    ),
    (
        "worm",
        "A worm gear: the gearbox it names is a worm stage of this ratio, which loses by its forward efficiency while the motor drives and by its back efficiency while the load drives it back; with a back efficiency of 0 or less it locks itself. The efficiencies count with effects.gearbox_efficiency on.",
        &[
            ID,
            field("gearbox", Kind::Ref("gearbox"), true, "The gearbox that is the worm stage; one worm per gearbox."),
            field("ratio", Kind::Number, true, "Its ratio (wheel teeth per worm start), in place of the gearbox's."),
            field("efficiency", Kind::Number, true, "Driven by the worm, above 0 and at most 1."),
            field(
                "back_efficiency",
                Kind::Number,
                true,
                "Driven back by the wheel, at most 1; 0 or less locks itself.",
            ),
            POS,
        ],
    ),
];

pub struct AxisNode {
    pub id: String,
    pub key: String,
    pub stroke: f64,
    pub incline_deg: f64,
    pub cable_chain: bool,
    /// A rotary node turns it: its unit is rad.
    pub rotary: bool,
}

/// What a motor drives: a belt pulley, or an axis through a screw, rack, linear motor or rotary (each with the dotted
/// path of its values' table).
pub enum Transmission {
    Belt,
    Screw { values: String },
    Rack { values: String },
    LinearMotor { values: String },
    Rotary { inertia: f64, payload: f64, strain_wave: Option<String> },
}

pub struct MotorNode {
    pub id: String,
    pub key: String,
    /// The pulley it turns, through its gearbox: radius and inertia (1 and 0 where it turns none).
    pub radius: f64,
    pub inertia: f64,
    pub pulley: String,
    pub at: Option<[f64; 3]>,
    /// The belt runs round its pulley clockwise (`machine.pulley.clockwise`).
    pub clockwise: bool,
    pub transmission: Transmission,
    /// The axis its screw, rack, linear motor or rotary drives; `None` for a belt.
    pub axis: Option<usize>,
    /// Its gearbox is a worm stage.
    pub worm: Option<WormNode>,
}

/// A worm stage (`machine.worm`): its ratio and its efficiencies driven forward and back.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WormNode {
    pub ratio: f64,
    pub efficiency: f64,
    pub back_efficiency: f64,
}

/// A body leaning on its carriage along a horizontal axis (`machine.lean`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeanNode {
    pub body: usize,
    pub axis: usize,
    pub com_drop: f64,
    pub tool_drop: f64,
    pub lever_axis: Option<usize>,
    pub lateral: f64,
    pub tilt: f64,
    pub damping: f64,
}

pub struct SectionNode {
    pub id: String,
    pub key: String,
    pub length: f64,
    /// Per axis, and per motor (the pulley that motor turns).
    pub axes: Vec<f64>,
    pub pulleys: Vec<f64>,
    /// The 3D view's way of the section, empty for the default.
    pub route: Vec<Waypoint>,
}

/// A point a section's belt runs past in the 3D view.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Waypoint {
    /// The pulley of the motor with this index.
    Pulley(usize),
    Idler(usize),
    Body(usize),
}

pub struct IdlerNode {
    pub id: String,
    pub speed: Vec<f64>,
    pub radius: f64,
    pub inertia: f64,
    pub friction: f64,
    pub body: Option<usize>,
    pub at: Option<[f64; 3]>,
    pub clockwise: bool,
}

pub struct FrameNode {
    pub axis: usize,
    pub mass: f64,
    pub stiffness: f64,
    pub damping: f64,
}

pub struct SpanNode {
    pub id: String,
    pub section: usize,
    pub length: f64,
    pub axes: Vec<f64>,
}

pub struct BodyNode {
    pub id: String,
    pub mass: f64,
    pub payload: f64,
    pub axes: Vec<bool>,
}

pub struct Overhang {
    pub body: usize,
    pub axis: usize,
    pub lever: f64,
    pub lever_axis: Option<usize>,
    pub spacing: f64,
    pub side_load: bool,
}

pub struct GuideNode {
    pub id: String,
    pub axis: usize,
    pub friction: String,
    pub carries: Vec<usize>,
    pub overhang: Option<Overhang>,
}

/// The `[machine]` part, read and checked; indices point into the node lists, motors in their order of the file.
pub struct Netlist {
    pub axes: Vec<AxisNode>,
    pub motors: Vec<MotorNode>,
    pub sections: Vec<SectionNode>,
    pub bodies: Vec<BodyNode>,
    pub guides: Vec<GuideNode>,
    pub idlers: Vec<IdlerNode>,
    pub frame: Option<FrameNode>,
    pub spans: Vec<SpanNode>,
    pub lean: Option<LeanNode>,
    /// Every dotted path a number field named: these keys of the file are in use.
    pub refs: Vec<String>,
    /// The value tables of screws, racks, linear motors and strain wave gears: the registered one-axis machines' table each is laid out
    /// as ("screw", ...), and its path in the file.
    pub values: Vec<(&'static str, String)>,
    /// The linkage of a file that has one, planar (`crate::linkage`) or spatial (`crate::spatial`); its netlist has no
    /// belts, bodies or guides.
    pub linkage: Option<crate::linkage::LinkageNode>,
}

/// One node of a list, with the path its errors name: `machine.<list>[<id>]`, or `machine.<list>[#<n>]` (from 1)
/// before its id is known.
pub(crate) struct Node<'a> {
    pub(crate) table: &'a Table,
    pub(crate) path: String,
}

impl Node<'_> {
    pub(crate) fn at(&self, name: &str) -> String {
        format!("{}.{name}", self.path)
    }
    pub(crate) fn get(&self, name: &str) -> Option<&Value> {
        self.table.get(name)
    }
    fn required(&self, name: &str) -> Result<&Value> {
        self.get(name).ok_or_else(|| ConfigError::Missing(self.at(name)))
    }
    pub(crate) fn text(&self, name: &str) -> Result<Option<&str>> {
        self.get(name).map(|v| v.as_str().ok_or_else(|| ConfigError::Type(self.at(name)))).transpose()
    }
    fn bool(&self, name: &str) -> Result<bool> {
        self.get(name).map_or(Ok(false), |v| v.as_bool().ok_or_else(|| ConfigError::Type(self.at(name))))
    }
    /// An optional [x, y, z] of numbers.
    fn point(&self, name: &str) -> Result<Option<[f64; 3]>> {
        let Some(value) = self.get(name) else { return Ok(None) };
        let numbers: Option<Vec<f64>> = value.as_array().and_then(|p| {
            p.iter()
                .map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).filter(|f| f.is_finite()))
                .collect()
        });
        match numbers.as_deref() {
            Some(&[x, y, z]) => Ok(Some([x, y, z])),
            _ => Err(ConfigError::Type(self.at(name))),
        }
    }
    pub(crate) fn id_list(&self, name: &str) -> Result<Vec<&str>> {
        let Some(value) = self.get(name) else { return Ok(Vec::new()) };
        let err = || ConfigError::Type(self.at(name));
        value.as_array().ok_or_else(err)?.iter().map(|v| v.as_str().ok_or_else(err)).collect()
    }
}

/// A number field: a number, or the path of one. NaN and infinity are refused, as everywhere in a config.
pub(crate) fn number(cfg: &Config, value: &Value, at: &str, refs: &mut Vec<String>) -> Result<f64> {
    match value {
        Value::Float(f) if f.is_finite() => Ok(*f),
        Value::Float(_) => Err(ConfigError::Invalid(at.into())),
        Value::Integer(i) => Ok(*i as f64),
        Value::String(path) => {
            let v = cfg.f64(path).map_err(|e| match e {
                ConfigError::Missing(_) => ConfigError::Missing(format!("{at} = \"{path}\"")),
                other => other,
            })?;
            refs.push(path.clone());
            Ok(v)
        }
        _ => Err(ConfigError::Type(at.into())),
    }
}

fn valid_id(id: &str) -> bool {
    !id.is_empty() && id.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}

impl Netlist {
    pub fn parse(cfg: &Config) -> Result<Self> {
        let machine = cfg.get("machine")?.as_table().ok_or_else(|| ConfigError::Type("machine".into()))?;
        match machine.get("version").map(Value::as_integer) {
            Some(Some(VERSION)) => {}
            Some(_) => return Err(ConfigError::Version("machine.version".into())),
            None => return Err(ConfigError::Missing("machine.version".into())),
        }
        for (key, _) in machine {
            if !["version", "id", "name"].contains(&key.as_str()) && !NODES.iter().any(|(list, ..)| list == key) {
                return Err(ConfigError::Unknown(format!("machine.{key}")));
            }
        }
        // Every node with its path, its fields checked against the schema and its id against all others.
        let mut lists: Vec<Vec<Node>> = Vec::new();
        let mut ids: Vec<String> = Vec::new();
        let none = Vec::new();
        for (list, _, fields) in NODES {
            let mut nodes = Vec::new();
            let items = match machine.get(*list) {
                None => &none,
                Some(v) => v.as_array().ok_or_else(|| ConfigError::Type(format!("machine.{list}")))?,
            };
            for (n, item) in items.iter().enumerate() {
                let table = item.as_table().ok_or_else(|| ConfigError::Type(format!("machine.{list}[#{}]", n + 1)))?;
                let mut node = Node { table, path: format!("machine.{list}[#{}]", n + 1) };
                let id = node.required("id")?.as_str().ok_or_else(|| ConfigError::Type(node.at("id")))?;
                if !valid_id(id) {
                    return Err(ConfigError::Invalid(format!(
                        "{} = \"{id}\" (letters, digits, _ and - only)",
                        node.at("id")
                    )));
                }
                if ids.iter().any(|known| known == id) {
                    return Err(ConfigError::Invalid(format!(
                        "{} = \"{id}\" (another node has this id)",
                        node.at("id")
                    )));
                }
                ids.push(id.into());
                node.path = format!("machine.{list}[{id}]");
                for key in table.keys() {
                    if !fields.iter().any(|f| f.name == key) {
                        return Err(ConfigError::Unknown(node.at(key)));
                    }
                }
                for f in fields.iter().filter(|f| f.required) {
                    node.required(f.name)?;
                }
                if let Some(pos) = node.get("pos") {
                    let ok = pos
                        .as_array()
                        .is_some_and(|p| p.len() == 2 && p.iter().all(|v| v.is_float() || v.is_integer()));
                    if !ok {
                        return Err(ConfigError::Type(node.at("pos")));
                    }
                }
                // A field of the schema with the wrong type, which the reading below may pass over.
                for f in fields.iter() {
                    let Some(value) = node.get(f.name) else { continue };
                    let ok = match f.kind {
                        Kind::Id | Kind::Key | Kind::Table | Kind::Ref(_) => value.is_str(),
                        Kind::Bool => value.is_bool(),
                        Kind::Refs(_) | Kind::Route(_) => value.is_array(),
                        _ => true,
                    };
                    if !ok {
                        return Err(ConfigError::Type(node.at(f.name)));
                    }
                }
                nodes.push(node);
            }
            lists.push(nodes);
        }
        // The nodes of a list by its name in `NODES`.
        let mut take = |list: &str| {
            let at = NODES.iter().position(|(name, ..)| *name == list).expect("a list of NODES");
            std::mem::take(&mut lists[at])
        };
        let (axes, motors, gearboxes, pulleys, sections) =
            (take("axis"), take("motor"), take("gearbox"), take("pulley"), take("section"));
        let (bodies, guides, idlers, frames, spans) =
            (take("body"), take("guide"), take("idler"), take("frame"), take("span"));
        let (screws, racks, linears, rotaries) = (take("screw"), take("rack"), take("linear_motor"), take("rotary"));
        let (leans, worms, linkages) = (take("lean"), take("worm"), take("linkage"));
        // Pulleys, sections and bodies only where the motors drive belts and the axes move bodies: checked below.
        for (list, nodes) in [("axis", &axes), ("motor", &motors)] {
            if nodes.is_empty() {
                return Err(ConfigError::Missing(format!("machine.{list}")));
            }
        }
        for (key, check) in [("id", true), ("name", false)] {
            match machine.get(key) {
                None => {}
                Some(Value::String(text)) if !check || valid_id(text) => {}
                Some(Value::String(_)) => {
                    return Err(ConfigError::Invalid(format!("machine.{key} (letters, digits, _ and - only)")));
                }
                Some(_) => return Err(ConfigError::Type(format!("machine.{key}"))),
            }
        }
        if !linkages.is_empty() {
            let others = [
                &pulleys, &sections, &bodies, &guides, &idlers, &frames, &spans, &screws, &racks, &linears, &rotaries,
                &leans, &worms,
            ];
            return crate::linkage::read(cfg, &axes, &motors, &gearboxes, &linkages, &others.map(|n| &n[..]));
        }
        let id_of = |node: &Node| node.table["id"].as_str().unwrap_or_default().to_string();
        // A reference to a node of a list, as its index there.
        let find = |nodes: &[Node], node: &Node, name: &str, id: &str| -> Result<usize> {
            nodes
                .iter()
                .position(|n| n.table["id"].as_str() == Some(id))
                .ok_or_else(|| ConfigError::Invalid(format!("{}.{name} = \"{id}\"", node.path)))
        };
        let mut refs = Vec::new();

        // The axes a rotary node turns are in rad.
        let turned: Vec<&str> = rotaries.iter().filter_map(|r| r.table.get("axis").and_then(Value::as_str)).collect();
        let mut axis_nodes = Vec::new();
        for node in &axes {
            let id = id_of(node);
            let rotary = turned.contains(&id.as_str());
            let incline = node
                .get("incline_deg")
                .map(|v| number(cfg, v, &node.at("incline_deg"), &mut refs))
                .transpose()?
                .unwrap_or(0.0);
            let (stroke_field, other) = if rotary { ("stroke_rad", "stroke_m") } else { ("stroke_m", "stroke_rad") };
            if node.get(other).is_some() {
                return Err(ConfigError::Invalid(format!(
                    "{} (a rotary axis has stroke_rad, any other stroke_m)",
                    node.at(other)
                )));
            }
            let stroke = number(cfg, node.required(stroke_field)?, &node.at(stroke_field), &mut refs)?;
            if stroke <= 0.0 {
                return Err(ConfigError::Invalid(node.at(stroke_field)));
            }
            // A table turns about a vertical axis and drags no chain.
            if rotary && incline != 0.0 {
                return Err(ConfigError::Invalid(format!(
                    "{} (a rotary axis turns about a vertical axis)",
                    node.at("incline_deg")
                )));
            }
            if rotary && node.bool("cable_chain")? {
                return Err(ConfigError::Invalid(format!("{} (not on a rotary axis)", node.at("cable_chain"))));
            }
            let key = node.text("key")?.map_or_else(|| format!("axis-{id}"), str::to_string);
            axis_nodes.push(AxisNode {
                id,
                key,
                stroke,
                incline_deg: incline,
                cable_chain: node.bool("cable_chain")?,
                rotary,
            });
        }

        // Each motor turns one gearbox, and each gearbox one pulley, screw, rack or rotary; or the motor is a linear
        // motor, with no gearbox node. The drive trains, in the order of the motors.
        let refers = |nodes: &[Node], field: &str, to: &str| -> Vec<usize> {
            (0..nodes.len()).filter(|&i| nodes[i].table.get(field).and_then(Value::as_str) == Some(to)).collect()
        };
        // A node's field that names a table of the file.
        let table_path = |node: &Node, name: &str| -> Result<String> {
            let path = node.required(name)?.as_str().ok_or_else(|| ConfigError::Type(node.at(name)))?;
            if !cfg.get(path).is_ok_and(Value::is_table) {
                return Err(ConfigError::Missing(format!("{} = \"{path}\"", node.at(name))));
            }
            Ok(path.to_string())
        };
        let mut motor_nodes = Vec::new();
        let mut values = Vec::new();
        // Per axis, the screw, rack, linear motor or rotary that drives it.
        let mut driven: Vec<Option<String>> = vec![None; axis_nodes.len()];
        for motor in &motors {
            let id = id_of(motor);
            let key = motor.text("key")?.map_or_else(|| format!("motor-{id}"), str::to_string);
            let mut worm = None;
            let (node, transmission) =
                match (&refers(&gearboxes, "motor", &id)[..], &refers(&linears, "motor", &id)[..]) {
                    ([], [l]) => {
                        (&linears[*l], Transmission::LinearMotor { values: table_path(&linears[*l], "values")? })
                    }
                    ([g], []) => {
                        let gearbox = &gearboxes[*g];
                        let gearbox_id = id_of(gearbox);
                        worm = worm_node(cfg, &worms, &gearbox_id, &mut refs)?;
                        let outputs: Vec<(usize, usize)> = [&pulleys, &screws, &racks, &rotaries]
                            .into_iter()
                            .enumerate()
                            .flat_map(|(list, nodes)| {
                                refers(nodes, "gearbox", &gearbox_id).into_iter().map(move |i| (list, i))
                            })
                            .collect();
                        let [(list, i)] = outputs[..] else {
                            return Err(ConfigError::Invalid(format!(
                                "{} (one pulley, screw, rack or rotary per gearbox)",
                                gearbox.path
                            )));
                        };
                        match list {
                            0 => {
                                let pulley = &pulleys[i];
                                let radius =
                                    number(cfg, pulley.required("radius_m")?, &pulley.at("radius_m"), &mut refs)?;
                                let inertia = number(
                                    cfg,
                                    pulley.required("inertia_kgm2")?,
                                    &pulley.at("inertia_kgm2"),
                                    &mut refs,
                                )?;
                                if radius <= 0.0 || inertia <= 0.0 {
                                    return Err(ConfigError::Invalid(pulley.at(if radius <= 0.0 {
                                        "radius_m"
                                    } else {
                                        "inertia_kgm2"
                                    })));
                                }
                                motor_nodes.push(MotorNode {
                                    id,
                                    key,
                                    radius,
                                    inertia,
                                    pulley: id_of(pulley),
                                    at: pulley.point("at_m")?,
                                    clockwise: pulley.bool("clockwise")?,
                                    transmission: Transmission::Belt,
                                    axis: None,
                                    worm,
                                });
                                continue;
                            }
                            1 => (&screws[i], Transmission::Screw { values: table_path(&screws[i], "values")? }),
                            2 => (&racks[i], Transmission::Rack { values: table_path(&racks[i], "values")? }),
                            _ => {
                                let node = &rotaries[i];
                                let inertia =
                                    number(cfg, node.required("inertia_kgm2")?, &node.at("inertia_kgm2"), &mut refs)?;
                                let payload = node
                                    .get("payload_inertia_kgm2")
                                    .map(|v| number(cfg, v, &node.at("payload_inertia_kgm2"), &mut refs))
                                    .transpose()?
                                    .unwrap_or(0.0);
                                if inertia <= 0.0 || payload < 0.0 {
                                    return Err(ConfigError::Invalid(node.at(if inertia <= 0.0 {
                                        "inertia_kgm2"
                                    } else {
                                        "payload_inertia_kgm2"
                                    })));
                                }
                                let strain_wave =
                                    node.get("strain_wave").map(|_| table_path(node, "strain_wave")).transpose()?;
                                (node, Transmission::Rotary { inertia, payload, strain_wave })
                            }
                        }
                    }
                    _ => {
                        return Err(ConfigError::Invalid(format!(
                            "{} (one gearbox per motor, or one linear motor)",
                            motor.path
                        )));
                    }
                };
            let axis = find(&axes, node, "axis", node.text("axis")?.unwrap_or_default())?;
            if let Some(other) = &driven[axis] {
                return Err(ConfigError::Invalid(format!(
                    "{} (one screw, rack, linear motor or rotary per axis; {other} drives it too)",
                    node.at("axis")
                )));
            }
            driven[axis] = Some(id_of(node));
            match &transmission {
                Transmission::Screw { values: v } => values.push(("screw", v.clone())),
                Transmission::Rack { values: v } => values.push(("rack", v.clone())),
                Transmission::LinearMotor { values: v } => values.push(("linear_motor", v.clone())),
                Transmission::Rotary { strain_wave: Some(v), .. } => values.push(("strain_wave", v.clone())),
                _ => {}
            }
            motor_nodes.push(MotorNode {
                id,
                key,
                radius: 1.0,
                inertia: 0.0,
                pulley: String::new(),
                at: None,
                clockwise: false,
                transmission,
                axis: Some(axis),
                worm,
            });
        }
        for gearbox in &gearboxes {
            find(&motors, gearbox, "motor", gearbox.text("motor")?.unwrap_or_default())?;
        }
        for (nodes, field, targets) in [
            (&pulleys, "gearbox", &gearboxes),
            (&screws, "gearbox", &gearboxes),
            (&racks, "gearbox", &gearboxes),
            (&rotaries, "gearbox", &gearboxes),
            (&linears, "motor", &motors),
            (&worms, "gearbox", &gearboxes),
        ] {
            for node in nodes {
                find(targets, node, field, node.text(field)?.unwrap_or_default())?;
            }
        }

        let mut section_nodes = Vec::new();
        for node in &sections {
            let id = id_of(node);
            let length = number(cfg, node.required("length_m")?, &node.at("length_m"), &mut refs)?;
            if length <= 0.0 {
                return Err(ConfigError::Invalid(node.at("length_m")));
            }
            let mut along = vec![0.0; axis_nodes.len()];
            let mut fed = vec![0.0; motor_nodes.len()];
            for (name, out) in [("axes", &mut along), ("pulleys", &mut fed)] {
                let table = node.required(name)?.as_table().ok_or_else(|| ConfigError::Type(node.at(name)))?;
                for (target, value) in table {
                    let at = format!("{}.{target}", node.at(name));
                    let i = if name == "axes" {
                        axis_nodes.iter().position(|a| &a.id == target)
                    } else {
                        motor_nodes.iter().position(|m| &m.pulley == target)
                    };
                    let i = i.ok_or_else(|| ConfigError::Invalid(at.clone()))?;
                    out[i] = number(cfg, value, &at, &mut refs)?;
                    // A section ends at a pulley once at most: it takes or gives the belt there, with a whole turn.
                    if name == "pulleys" && out[i].abs() != 1.0 {
                        return Err(ConfigError::Invalid(at));
                    }
                }
            }
            let key = node.text("key")?.map_or_else(|| format!("belt-section-{id}"), str::to_string);
            let mut route = Vec::new();
            for target in node.id_list("route")? {
                let found = motor_nodes.iter().position(|m| m.pulley == target).map(Waypoint::Pulley);
                let found = found.or_else(|| {
                    idlers.iter().position(|n| n.table["id"].as_str() == Some(target)).map(Waypoint::Idler)
                });
                let found = found
                    .or_else(|| bodies.iter().position(|n| n.table["id"].as_str() == Some(target)).map(Waypoint::Body));
                route.push(found.ok_or_else(|| ConfigError::Invalid(format!("{} = \"{target}\"", node.at("route"))))?);
            }
            section_nodes.push(SectionNode { id, key, length, axes: along, pulleys: fed, route });
        }
        // Per axis, the coefficients of a node's table field (`speed = { x = 2 }`).
        let coefficients = |node: &Node, name: &str, refs: &mut Vec<String>| -> Result<Vec<f64>> {
            let mut out = vec![0.0; axis_nodes.len()];
            let Some(value) = node.get(name) else { return Ok(out) };
            for (target, value) in value.as_table().ok_or_else(|| ConfigError::Type(node.at(name)))? {
                let at = format!("{}.{target}", node.at(name));
                let i =
                    axis_nodes.iter().position(|a| &a.id == target).ok_or_else(|| ConfigError::Invalid(at.clone()))?;
                out[i] = number(cfg, value, &at, refs)?;
            }
            Ok(out)
        };

        let mut body_nodes = Vec::new();
        for node in &bodies {
            let mass = number(cfg, node.required("mass_kg")?, &node.at("mass_kg"), &mut refs)?;
            let payload = node
                .get("payload_kg")
                .map(|v| number(cfg, v, &node.at("payload_kg"), &mut refs))
                .transpose()?
                .unwrap_or(0.0);
            if mass <= 0.0 || payload < 0.0 {
                return Err(ConfigError::Invalid(node.at(if mass <= 0.0 { "mass_kg" } else { "payload_kg" })));
            }
            let mut moves = vec![false; axis_nodes.len()];
            for id in node.id_list("axes")? {
                let i = find(&axes, node, "axes", id)?;
                // A rotary axis turns its rotary's inertia; a body on it would move on a circle, not along the axes.
                if axis_nodes[i].rotary {
                    return Err(ConfigError::Invalid(format!(
                        "{} (no body moves with the rotary axis {id})",
                        node.at("axes")
                    )));
                }
                moves[i] = true;
            }
            if !moves.contains(&true) {
                return Err(ConfigError::Missing(node.at("axes")));
            }
            body_nodes.push(BodyNode { id: id_of(node), mass, payload, axes: moves });
        }

        let mut guide_nodes: Vec<GuideNode> = Vec::new();
        for node in &guides {
            let axis = find(&axes, node, "axis", node.text("axis")?.unwrap_or_default())?;
            if guide_nodes.iter().any(|g| g.axis == axis) {
                return Err(ConfigError::Invalid(node.at("axis")));
            }
            // shortcut: a rotary axis has no bearing friction of its own (only its motor's and gearbox's); give the
            // rotary node a friction table if a table's bearing drag matters.
            if axis_nodes[axis].rotary {
                return Err(ConfigError::Invalid(format!("{} (a rotary axis has no guide)", node.at("axis"))));
            }
            let friction = node.text("friction")?.unwrap_or_default().to_string();
            if !cfg.get(&friction).is_ok_and(Value::is_table) {
                return Err(ConfigError::Missing(format!("{} = \"{friction}\"", node.at("friction"))));
            }
            let carries = node
                .id_list("carries")?
                .into_iter()
                .map(|id| find(&bodies, node, "carries", id))
                .collect::<Result<Vec<_>>>()?;
            let overhang = match node.text("overhang")? {
                None => {
                    if let Some(extra) = ["overhang_axis", "lever_m", "lever_axis", "spacing_m", "side_load"]
                        .into_iter()
                        .find(|f| node.get(f).is_some())
                    {
                        return Err(ConfigError::Missing(format!("{} (with {extra})", node.at("overhang"))));
                    }
                    None
                }
                Some(body) => {
                    let body = find(&bodies, node, "overhang", body)?;
                    let axis_id = node
                        .required("overhang_axis")?
                        .as_str()
                        .ok_or_else(|| ConfigError::Type(node.at("overhang_axis")))?;
                    let lever_axis =
                        node.text("lever_axis")?.map(|id| find(&axes, node, "lever_axis", id)).transpose()?;
                    let spacing = number(cfg, node.required("spacing_m")?, &node.at("spacing_m"), &mut refs)?;
                    if spacing <= 0.0 {
                        return Err(ConfigError::Invalid(node.at("spacing_m")));
                    }
                    Some(Overhang {
                        body,
                        axis: find(&axes, node, "overhang_axis", axis_id)?,
                        lever: number(cfg, node.required("lever_m")?, &node.at("lever_m"), &mut refs)?,
                        lever_axis,
                        spacing,
                        side_load: node.bool("side_load")?,
                    })
                }
            };
            guide_nodes.push(GuideNode { id: id_of(node), axis, friction, carries, overhang });
        }

        let mut idler_nodes = Vec::new();
        for node in &idlers {
            let value = |name: &str, refs: &mut Vec<String>| number(cfg, node.required(name)?, &node.at(name), refs);
            let (radius, inertia, friction) =
                (value("radius_m", &mut refs)?, value("inertia_kgm2", &mut refs)?, value("friction_nm", &mut refs)?);
            if radius <= 0.0 || inertia < 0.0 || friction < 0.0 {
                return Err(ConfigError::Invalid(node.at(if radius <= 0.0 {
                    "radius_m"
                } else if inertia < 0.0 {
                    "inertia_kgm2"
                } else {
                    "friction_nm"
                })));
            }
            let speed = coefficients(node, "speed", &mut refs)?;
            let body = node.text("body")?.map(|id| find(&bodies, node, "body", id)).transpose()?;
            idler_nodes.push(IdlerNode {
                id: id_of(node),
                speed,
                radius,
                inertia,
                friction,
                body,
                at: node.point("at_m")?,
                clockwise: node.bool("clockwise")?,
            });
        }

        let frame = match &frames[..] {
            [] => None,
            [node] => {
                let axis = find(&axes, node, "axis", node.text("axis")?.unwrap_or_default())?;
                // A frame swaying along a sloped axis would move the bodies' weight with it.
                if axis_nodes[axis].incline_deg != 0.0 || axis_nodes[axis].rotary {
                    return Err(ConfigError::Invalid(format!(
                        "{} (the frame sways along a horizontal axis)",
                        node.at("axis")
                    )));
                }
                let value =
                    |name: &str, refs: &mut Vec<String>| number(cfg, node.required(name)?, &node.at(name), refs);
                let (mass, stiffness, damping) = (
                    value("mass_kg", &mut refs)?,
                    value("stiffness_n_m", &mut refs)?,
                    value("damping_n_s_m", &mut refs)?,
                );
                if mass <= 0.0 || stiffness <= 0.0 || damping < 0.0 {
                    return Err(ConfigError::Invalid(node.at(if mass <= 0.0 {
                        "mass_kg"
                    } else if stiffness <= 0.0 {
                        "stiffness_n_m"
                    } else {
                        "damping_n_s_m"
                    })));
                }
                Some(FrameNode { axis, mass, stiffness, damping })
            }
            [_, second, ..] => return Err(ConfigError::Invalid(format!("{} (one frame at most)", second.path))),
        };

        let mut span_nodes = Vec::new();
        for node in &spans {
            let section = find(&sections, node, "section", node.text("section")?.unwrap_or_default())?;
            let length = number(cfg, node.required("length_m")?, &node.at("length_m"), &mut refs)?;
            if length <= 0.0 {
                return Err(ConfigError::Invalid(node.at("length_m")));
            }
            span_nodes.push(SpanNode {
                id: id_of(node),
                section,
                length,
                axes: coefficients(node, "axes", &mut refs)?,
            });
        }
        let lean = lean_node(cfg, &leans, &axes, &bodies, &axis_nodes, &body_nodes, &mut refs)?;
        if let Some(i) = (0..axis_nodes.len()).find(|&i| !axis_nodes[i].rotary && !body_nodes.iter().any(|b| b.axes[i]))
        {
            return Err(ConfigError::Invalid(format!("machine.axis[{}] (no body moves with it)", axis_nodes[i].id)));
        }
        if motor_nodes.len() != axis_nodes.len() {
            return Err(ConfigError::Invalid("machine.motor (as many motors as axes)".into()));
        }
        Ok(Self {
            axes: axis_nodes,
            motors: motor_nodes,
            sections: section_nodes,
            bodies: body_nodes,
            guides: guide_nodes,
            idlers: idler_nodes,
            frame,
            spans: span_nodes,
            lean,
            refs,
            values,
            linkage: None,
        })
    }
}

/// The worm stage on the gearbox `gearbox` (`machine.worm`), where one names it: its ratio above 0, its efficiency
/// above 0 and at most 1, its back efficiency at most 1 (0 or less locks itself).
fn worm_node(cfg: &Config, worms: &[Node], gearbox: &str, refs: &mut Vec<String>) -> Result<Option<WormNode>> {
    let mine: Vec<&Node> =
        worms.iter().filter(|w| w.table.get("gearbox").and_then(Value::as_str) == Some(gearbox)).collect();
    let node = match mine[..] {
        [] => return Ok(None),
        [node] => node,
        [_, second, ..] => {
            return Err(ConfigError::Invalid(format!("{} (one worm per gearbox)", second.at("gearbox"))));
        }
    };
    let value = |name: &str, refs: &mut Vec<String>| number(cfg, node.required(name)?, &node.at(name), refs);
    let worm = WormNode {
        ratio: value("ratio", refs)?,
        efficiency: value("efficiency", refs)?,
        back_efficiency: value("back_efficiency", refs)?,
    };
    if worm.ratio <= 0.0 {
        return Err(ConfigError::Invalid(node.at("ratio")));
    }
    if worm.efficiency <= 0.0 || worm.efficiency > 1.0 {
        return Err(ConfigError::Invalid(format!("{} (above 0, at most 1)", node.at("efficiency"))));
    }
    if worm.back_efficiency > 1.0 {
        return Err(ConfigError::Invalid(format!("{} (at most 1)", node.at("back_efficiency"))));
    }
    Ok(Some(worm))
}

/// The leaning body (`machine.lean`), at most one: on a horizontal straight axis the body moves with, stiffnesses above
/// 0, damping not below.
fn lean_node(
    cfg: &Config,
    leans: &[Node],
    axes: &[Node],
    bodies: &[Node],
    axis_nodes: &[AxisNode],
    body_nodes: &[BodyNode],
    refs: &mut Vec<String>,
) -> Result<Option<LeanNode>> {
    let node = match leans {
        [] => return Ok(None),
        [node] => node,
        [_, second, ..] => return Err(ConfigError::Invalid(format!("{} (one lean at most)", second.path))),
    };
    let find = |nodes: &[Node], name: &str, id: &str| {
        nodes
            .iter()
            .position(|n| n.table["id"].as_str() == Some(id))
            .ok_or_else(|| ConfigError::Invalid(format!("{}.{name} = \"{id}\"", node.path)))
    };
    let body = find(bodies, "body", node.text("body")?.unwrap_or_default())?;
    let axis = find(axes, "axis", node.text("axis")?.unwrap_or_default())?;
    let lever_axis = node.text("lever_axis")?.map(|id| find(axes, "lever_axis", id)).transpose()?;
    // A lean along a sloped axis would move the body's weight with it.
    if axis_nodes[axis].incline_deg != 0.0 || axis_nodes[axis].rotary {
        return Err(ConfigError::Invalid(format!("{} (a body leans along a horizontal axis)", node.at("axis"))));
    }
    if !body_nodes[body].axes[axis] {
        return Err(ConfigError::Invalid(format!("{} (the body does not move with axis)", node.at("body"))));
    }
    let value = |name: &str, refs: &mut Vec<String>| number(cfg, node.required(name)?, &node.at(name), refs);
    let lean = LeanNode {
        body,
        axis,
        com_drop: value("com_drop_m", refs)?,
        tool_drop: value("tool_drop_m", refs)?,
        lever_axis,
        lateral: value("lateral_stiffness_n_m", refs)?,
        tilt: value("tilt_stiffness_nm_rad", refs)?,
        damping: value("damping_n_s_m", refs)?,
    };
    for (name, bad) in [
        ("lateral_stiffness_n_m", lean.lateral <= 0.0),
        ("tilt_stiffness_nm_rad", lean.tilt <= 0.0),
        ("damping_n_s_m", lean.damping < 0.0),
    ] {
        if bad {
            return Err(ConfigError::Invalid(node.at(name)));
        }
    }
    Ok(Some(lean))
}

/// The JSON Schema (draft 2020-12) of a machine file, from `NODES`, as a value for a JSON writer (`sim_machines`
/// writes it; this layer has no JSON). It describes `[machine]` in full; the other tables it lists as objects, since
/// they are the tables of a machine config (docs/config-reference.md).
pub fn json_schema() -> Value {
    fn obj<const N: usize>(pairs: [(&str, Value); N]) -> Value {
        Value::Table(pairs.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
    }
    let text = |t: &str| Value::String(t.into());
    let number = obj([(
        "oneOf",
        Value::Array(vec![
            obj([("type", text("number"))]),
            obj([
                ("type", text("string")),
                ("pattern", text(r"^[A-Za-z0-9_]+(\.[A-Za-z0-9_]+)+$")),
                ("description", text("Dotted path of a number in the file.")),
            ]),
        ]),
    )]);
    let id = obj([("type", text("string")), ("pattern", text("^[A-Za-z0-9_-]+$"))]);
    let mut lists = Table::new();
    for (list, doc, fields) in NODES {
        let mut properties = Table::new();
        for f in *fields {
            let (schema, about) = match f.kind {
                Kind::Id => (id.clone(), None),
                Kind::Key | Kind::Table => (obj([("type", text("string"))]), None),
                Kind::Ref(target) => (obj([("type", text("string"))]), Some(format!("Id of a machine.{target} node."))),
                Kind::Refs(target) => (
                    obj([("type", text("array")), ("items", obj([("type", text("string"))]))]),
                    Some(format!("Ids of machine.{target} nodes.")),
                ),
                Kind::Coefficients(target) => (
                    obj([("type", text("object")), ("additionalProperties", number.clone())]),
                    Some(format!("Keys: ids of machine.{target} nodes.")),
                ),
                Kind::Number => (number.clone(), None),
                Kind::Bool => (obj([("type", text("boolean"))]), None),
                Kind::Position => (
                    obj([
                        ("type", text("array")),
                        ("items", obj([("type", text("number"))])),
                        ("minItems", Value::Integer(2)),
                        ("maxItems", Value::Integer(2)),
                    ]),
                    None,
                ),
                Kind::Point => (
                    obj([
                        ("type", text("array")),
                        ("items", obj([("type", text("number"))])),
                        ("minItems", Value::Integer(3)),
                        ("maxItems", Value::Integer(3)),
                    ]),
                    None,
                ),
                Kind::Route(targets) => (
                    obj([("type", text("array")), ("items", obj([("type", text("string"))]))]),
                    Some(format!(
                        "Ids of {} nodes.",
                        targets.iter().map(|t| format!("machine.{t}")).collect::<Vec<_>>().join(", ")
                    )),
                ),
            };
            let mut schema = schema.as_table().cloned().unwrap_or_default();
            schema.insert(
                "description".into(),
                text(&about.map_or_else(|| f.doc.to_string(), |a| format!("{} {a}", f.doc))),
            );
            properties.insert(f.name.into(), Value::Table(schema));
        }
        let required = fields.iter().filter(|f| f.required).map(|f| text(f.name)).collect();
        let items = obj([
            ("type", text("object")),
            ("properties", Value::Table(properties)),
            ("required", Value::Array(required)),
            ("additionalProperties", Value::Boolean(false)),
        ]);
        lists.insert((*list).into(), obj([("type", text("array")), ("description", text(doc)), ("items", items)]));
    }
    lists.insert(
        "version".into(),
        obj([("const", Value::Integer(VERSION)), ("description", text("Version of the [machine] part."))]),
    );
    lists.insert(
        "id".into(),
        obj([
            ("type", text("string")),
            ("pattern", text("^[A-Za-z0-9_-]+$")),
            (
                "description",
                text("Id of the machine, as the front ends list it; default the file name without its extension."),
            ),
        ]),
    );
    lists.insert(
        "name".into(),
        obj([
            ("type", text("string")),
            ("description", text("Translation key or plain text of the machine's name; default its id.")),
        ]),
    );
    let required = ["version", "axis", "motor"].map(text).to_vec();
    obj([
        ("$schema", text("https://json-schema.org/draft/2020-12/schema")),
        ("$id", text("https://github.com/kinematics-simulation/machine-file.schema.json")),
        ("title", text("Machine file")),
        (
            "description",
            text(
                "A machine as a netlist of simulation components: [machine] says what is connected to what, the other tables hold the values (see docs/machine-files.md). Generated by sim_netlist::json_schema; do not edit.",
            ),
        ),
        ("type", text("object")),
        ("required", Value::Array(vec![text("format_version"), text("machine")])),
        (
            "properties",
            obj([
                ("format_version", obj([("const", Value::Integer(sim_core::config::FORMAT_VERSION))])),
                (
                    "machine",
                    obj([
                        ("type", text("object")),
                        ("required", Value::Array(required)),
                        ("properties", Value::Table(lists)),
                        ("additionalProperties", Value::Boolean(false)),
                    ]),
                ),
            ]),
        ),
        ("additionalProperties", obj([("type", Value::Array(vec![text("object"), text("array")]))])),
    ])
}
