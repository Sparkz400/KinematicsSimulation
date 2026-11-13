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
const POS: Field = field("pos", Kind::Position, false, "Where a node editor draws the node, [x, y]. Not read by the simulation.");

/// The node lists of `[machine]`, in the order they are read, each with its fields.
pub const NODES: &[(&str, &str, &[Field])] = &[
    (
        "axis",
        "A tool axis: a straight coordinate of the tool, from 0 to its stroke. Its limits are in [axis.<id>].",
        &[
            ID,
            field("key", Kind::Key, false, "Translation key of its name; default axis-<id>."),
            field("stroke_m", Kind::Number, true, "Stroke, m."),
            field("incline_deg", Kind::Number, false, "Angle of the axis above the horizontal, deg; 90 is vertical, rising towards +. Default 0."),
            POS,
        ],
    ),
    (
        "motor",
        "A servo motor with its drive; its data is in [motor] and [drive], shared by all motors.",
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
            field("pulleys", Kind::Coefficients("pulley"), true, "+1 for a pulley whose forward turn feeds belt into the section, -1 for one that takes belt out of it."),
            POS,
        ],
    ),
    (
        "body",
        "A rigid body the axes move: a carriage, a vertical axis.",
        &[
            ID,
            field("mass_kg", Kind::Number, true, "Mass, kg."),
            field("payload_kg", Kind::Number, false, "Payload it carries while effects.payload is on and the payload is attached, kg."),
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
            field("friction", Kind::Table, true, "The table of its friction values, laid out as [friction_x] of the shipped files."),
            field("carries", Kind::Refs("body"), false, "Bodies whose weight across the axis it carries."),
            field("overhang", Kind::Ref("body"), false, "A body that overhangs the guide: accelerated along overhang_axis it tilts the guide's carriages."),
            field("overhang_axis", Kind::Ref("axis"), false, "The axis along which the overhang's acceleration tilts the guide."),
            field("lever_m", Kind::Number, false, "Distance of the overhang's centre of mass from the guide with lever_axis at 0, m."),
            field("lever_axis", Kind::Ref("axis"), false, "An axis whose travel shortens the lever (the overhang rises towards the guide)."),
            field("spacing_m", Kind::Number, false, "Spacing of the guide's two carriages, m; the tilt puts 2 m a lever / spacing on them."),
            field("side_load", Kind::Bool, false, "The guide also carries the overhang's inertia force m a sideways. Default false."),
            POS,
        ],
    ),
];

pub struct AxisNode {
    pub id: String,
    pub key: String,
    pub stroke: f64,
    pub incline_deg: f64,
}

pub struct MotorNode {
    pub id: String,
    pub key: String,
    /// The pulley it turns, through its gearbox: radius and inertia.
    pub radius: f64,
    pub inertia: f64,
    pub pulley: String,
}

pub struct SectionNode {
    pub id: String,
    pub key: String,
    pub length: f64,
    /// Per axis, and per motor (the pulley that motor turns).
    pub axes: Vec<f64>,
    pub pulleys: Vec<f64>,
}

pub struct BodyNode {
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
    /// Every dotted path a number field named: these keys of the file are in use.
    pub refs: Vec<String>,
}

/// One node of a list, with the path its errors name: `machine.<list>[<id>]`, or `machine.<list>[#<n>]` (from 1)
/// before its id is known.
struct Node<'a> {
    table: &'a Table,
    path: String,
    fields: &'static [Field],
}

impl Node<'_> {
    fn at(&self, name: &str) -> String {
        format!("{}.{name}", self.path)
    }
    fn get(&self, name: &str) -> Option<&Value> {
        self.table.get(name)
    }
    fn required(&self, name: &str) -> Result<&Value> {
        self.get(name).ok_or_else(|| ConfigError::Missing(self.at(name)))
    }
    fn text(&self, name: &str) -> Result<Option<&str>> {
        self.get(name).map(|v| v.as_str().ok_or_else(|| ConfigError::Type(self.at(name)))).transpose()
    }
    fn bool(&self, name: &str) -> Result<bool> {
        self.get(name).map_or(Ok(false), |v| v.as_bool().ok_or_else(|| ConfigError::Type(self.at(name))))
    }
    fn id_list(&self, name: &str) -> Result<Vec<&str>> {
        let Some(value) = self.get(name) else { return Ok(Vec::new()) };
        let err = || ConfigError::Type(self.at(name));
        value.as_array().ok_or_else(err)?.iter().map(|v| v.as_str().ok_or_else(err)).collect()
    }
}

/// A number field: a number, or the path of one. NaN and infinity are refused, as everywhere in a config.
fn number(cfg: &Config, value: &Value, at: &str, refs: &mut Vec<String>) -> Result<f64> {
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
                let mut node = Node { table, path: format!("machine.{list}[#{}]", n + 1), fields };
                let id = node.required("id")?.as_str().ok_or_else(|| ConfigError::Type(node.at("id")))?;
                if !valid_id(id) {
                    return Err(ConfigError::Invalid(format!("{} = \"{id}\" (letters, digits, _ and - only)", node.at("id"))));
                }
                if ids.iter().any(|known| known == id) {
                    return Err(ConfigError::Invalid(format!("{} = \"{id}\" (another node has this id)", node.at("id"))));
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
                    let ok = pos.as_array().is_some_and(|p| p.len() == 2 && p.iter().all(|v| v.is_float() || v.is_integer()));
                    if !ok {
                        return Err(ConfigError::Type(node.at("pos")));
                    }
                }
                nodes.push(node);
            }
            lists.push(nodes);
        }
        let [axes, motors, gearboxes, pulleys, sections, bodies, guides] = <[Vec<Node>; 7]>::try_from(lists).ok().expect("one list per entry of NODES");
        for (list, nodes) in [("axis", &axes), ("motor", &motors), ("pulley", &pulleys), ("section", &sections), ("body", &bodies)] {
            if nodes.is_empty() {
                return Err(ConfigError::Missing(format!("machine.{list}")));
            }
        }
        let id_of = |node: &Node| node.table["id"].as_str().unwrap_or_default().to_string();
        // A reference to a node of a list, as its index there.
        let find = |nodes: &[Node], node: &Node, name: &str, id: &str| -> Result<usize> {
            nodes.iter().position(|n| n.table["id"].as_str() == Some(id)).ok_or_else(|| ConfigError::Invalid(format!("{}.{name} = \"{id}\"", node.path)))
        };
        let mut refs = Vec::new();

        let mut axis_nodes = Vec::new();
        for node in &axes {
            let id = id_of(node);
            let incline = node.get("incline_deg").map(|v| number(cfg, v, &node.at("incline_deg"), &mut refs)).transpose()?.unwrap_or(0.0);
            let stroke = number(cfg, node.required("stroke_m")?, &node.at("stroke_m"), &mut refs)?;
            if stroke <= 0.0 {
                return Err(ConfigError::Invalid(node.at("stroke_m")));
            }
            let key = node.text("key")?.map_or_else(|| format!("axis-{id}"), str::to_string);
            axis_nodes.push(AxisNode { id, key, stroke, incline_deg: incline });
        }

        // Each motor turns one gearbox, and each gearbox one pulley: the drive trains, in the order of the motors.
        let mut motor_nodes = Vec::new();
        for motor in &motors {
            let id = id_of(motor);
            let geared: Vec<&Node> = gearboxes.iter().filter(|g| g.table["motor"].as_str() == Some(&id)).collect();
            let [gearbox] = geared[..] else { return Err(ConfigError::Invalid(format!("{} (one gearbox per motor)", motor.path))) };
            let gearbox_id = id_of(gearbox);
            let turned: Vec<&Node> = pulleys.iter().filter(|p| p.table["gearbox"].as_str() == Some(&gearbox_id)).collect();
            let [pulley] = turned[..] else { return Err(ConfigError::Invalid(format!("{} (one pulley per gearbox)", gearbox.path))) };
            let radius = number(cfg, pulley.required("radius_m")?, &pulley.at("radius_m"), &mut refs)?;
            let inertia = number(cfg, pulley.required("inertia_kgm2")?, &pulley.at("inertia_kgm2"), &mut refs)?;
            if radius <= 0.0 || inertia <= 0.0 {
                return Err(ConfigError::Invalid(pulley.at(if radius <= 0.0 { "radius_m" } else { "inertia_kgm2" })));
            }
            let key = motor.text("key")?.map_or_else(|| format!("motor-{id}"), str::to_string);
            motor_nodes.push(MotorNode { id, key, radius, inertia, pulley: id_of(pulley) });
        }
        for gearbox in &gearboxes {
            find(&motors, gearbox, "motor", gearbox.text("motor")?.unwrap_or_default())?;
        }
        for pulley in &pulleys {
            find(&gearboxes, pulley, "gearbox", pulley.text("gearbox")?.unwrap_or_default())?;
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
            section_nodes.push(SectionNode { id, key, length, axes: along, pulleys: fed });
        }

        let mut body_nodes = Vec::new();
        for node in &bodies {
            let mass = number(cfg, node.required("mass_kg")?, &node.at("mass_kg"), &mut refs)?;
            let payload = node.get("payload_kg").map(|v| number(cfg, v, &node.at("payload_kg"), &mut refs)).transpose()?.unwrap_or(0.0);
            if mass <= 0.0 || payload < 0.0 {
                return Err(ConfigError::Invalid(node.at(if mass <= 0.0 { "mass_kg" } else { "payload_kg" })));
            }
            let mut moves = vec![false; axis_nodes.len()];
            for id in node.id_list("axes")? {
                moves[find(&axes, node, "axes", id)?] = true;
            }
            if !moves.contains(&true) {
                return Err(ConfigError::Missing(node.at("axes")));
            }
            body_nodes.push(BodyNode { mass, payload, axes: moves });
        }

        let mut guide_nodes: Vec<GuideNode> = Vec::new();
        for node in &guides {
            let axis = find(&axes, node, "axis", node.text("axis")?.unwrap_or_default())?;
            if guide_nodes.iter().any(|g| g.axis == axis) {
                return Err(ConfigError::Invalid(node.at("axis")));
            }
            let friction = node.text("friction")?.unwrap_or_default().to_string();
            if !cfg.get(&friction).is_ok_and(Value::is_table) {
                return Err(ConfigError::Missing(format!("{} = \"{friction}\"", node.at("friction"))));
            }
            let carries = node.id_list("carries")?.into_iter().map(|id| find(&bodies, node, "carries", id)).collect::<Result<Vec<_>>>()?;
            let overhang = match node.text("overhang")? {
                None => {
                    if let Some(extra) = ["overhang_axis", "lever_m", "lever_axis", "spacing_m", "side_load"].into_iter().find(|f| node.get(f).is_some()) {
                        return Err(ConfigError::Missing(format!("{} (with {extra})", node.at("overhang"))));
                    }
                    None
                }
                Some(body) => {
                    let body = find(&bodies, node, "overhang", body)?;
                    let axis_id = node.required("overhang_axis")?.as_str().ok_or_else(|| ConfigError::Type(node.at("overhang_axis")))?;
                    let lever_axis = node.text("lever_axis")?.map(|id| find(&axes, node, "lever_axis", id)).transpose()?;
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
        // Checked here: a field of the schema with the wrong type that the reading above passed over.
        for node in axes.iter().chain(&motors).chain(&gearboxes).chain(&pulleys).chain(&sections).chain(&bodies).chain(&guides) {
            for f in node.fields {
                let Some(value) = node.get(f.name) else { continue };
                let ok = match f.kind {
                    Kind::Id | Kind::Key | Kind::Table | Kind::Ref(_) => value.is_str(),
                    Kind::Bool => value.is_bool(),
                    _ => true,
                };
                if !ok {
                    return Err(ConfigError::Type(node.at(f.name)));
                }
            }
        }
        if let Some(i) = (0..axis_nodes.len()).find(|&i| !body_nodes.iter().any(|b| b.axes[i])) {
            return Err(ConfigError::Invalid(format!("machine.axis[{}] (no body moves with it)", axis_nodes[i].id)));
        }
        for (key, check) in [("id", true), ("name", false)] {
            match machine.get(key) {
                None => {}
                Some(Value::String(text)) if !check || valid_id(text) => {}
                Some(Value::String(_)) => return Err(ConfigError::Invalid(format!("machine.{key} (letters, digits, _ and - only)"))),
                Some(_) => return Err(ConfigError::Type(format!("machine.{key}"))),
            }
        }
        if motor_nodes.len() != axis_nodes.len() {
            return Err(ConfigError::Invalid("machine.motor (as many motors as axes)".into()));
        }
        Ok(Self { axes: axis_nodes, motors: motor_nodes, sections: section_nodes, bodies: body_nodes, guides: guide_nodes, refs })
    }
}

/// The JSON Schema (draft 2020-12) of a machine file, from `NODES`, as a value for a JSON writer (`sim_machines`
/// writes it; this layer has no JSON). It describes `[machine]` in full; the other tables it lists as objects, since
/// they are the tables of a machine config (docs/config-reference.md).
pub fn json_schema() -> Value {
    fn obj<const N: usize>(pairs: [(&str, Value); N]) -> Value {
        Value::Table(pairs.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
    }
    let text = |t: &str| Value::String(t.into());
    let number = obj([("oneOf", Value::Array(vec![obj([("type", text("number"))]), obj([("type", text("string")), ("pattern", text(r"^[A-Za-z0-9_]+(\.[A-Za-z0-9_]+)+$")), ("description", text("Dotted path of a number in the file."))])]))]);
    let id = obj([("type", text("string")), ("pattern", text("^[A-Za-z0-9_-]+$"))]);
    let mut lists = Table::new();
    for (list, doc, fields) in NODES {
        let mut properties = Table::new();
        for f in *fields {
            let (schema, about) = match f.kind {
                Kind::Id => (id.clone(), None),
                Kind::Key | Kind::Table => (obj([("type", text("string"))]), None),
                Kind::Ref(target) => (obj([("type", text("string"))]), Some(format!("Id of a machine.{target} node."))),
                Kind::Refs(target) => (obj([("type", text("array")), ("items", obj([("type", text("string"))]))]), Some(format!("Ids of machine.{target} nodes."))),
                Kind::Coefficients(target) => (obj([("type", text("object")), ("additionalProperties", number.clone())]), Some(format!("Keys: ids of machine.{target} nodes."))),
                Kind::Number => (number.clone(), None),
                Kind::Bool => (obj([("type", text("boolean"))]), None),
                Kind::Position => (obj([("type", text("array")), ("items", obj([("type", text("number"))])), ("minItems", Value::Integer(2)), ("maxItems", Value::Integer(2))]), None),
            };
            let mut schema = schema.as_table().cloned().unwrap_or_default();
            schema.insert("description".into(), text(&about.map_or_else(|| f.doc.to_string(), |a| format!("{} {a}", f.doc))));
            properties.insert(f.name.into(), Value::Table(schema));
        }
        let required = fields.iter().filter(|f| f.required).map(|f| text(f.name)).collect();
        let items = obj([("type", text("object")), ("properties", Value::Table(properties)), ("required", Value::Array(required)), ("additionalProperties", Value::Boolean(false))]);
        lists.insert((*list).into(), obj([("type", text("array")), ("description", text(doc)), ("items", items)]));
    }
    lists.insert("version".into(), obj([("const", Value::Integer(VERSION)), ("description", text("Version of the [machine] part."))]));
    lists.insert("id".into(), obj([("type", text("string")), ("pattern", text("^[A-Za-z0-9_-]+$")), ("description", text("Id of the machine, as the front ends list it; default the file name without its extension."))]));
    lists.insert("name".into(), obj([("type", text("string")), ("description", text("Translation key or plain text of the machine's name; default its id."))]));
    let required = ["version", "axis", "motor", "gearbox", "pulley", "section", "body"].map(text).to_vec();
    obj([
        ("$schema", text("https://json-schema.org/draft/2020-12/schema")),
        ("$id", text("https://github.com/kinematics-simulation/machine-file.schema.json")),
        ("title", text("Machine file")),
        ("description", text("A machine as a netlist of simulation components: [machine] says what is connected to what, the other tables hold the values (see docs/machine-files.md). Generated by sim_netlist::json_schema; do not edit.")),
        ("type", text("object")),
        ("required", Value::Array(vec![text("format_version"), text("machine")])),
        ("properties", obj([("format_version", obj([("const", Value::Integer(sim_core::config::FORMAT_VERSION))])), ("machine", obj([("type", text("object")), ("required", Value::Array(required)), ("properties", Value::Table(lists)), ("additionalProperties", Value::Boolean(false))]))])),
        ("additionalProperties", obj([("type", Value::Array(vec![text("object"), text("array")]))])),
    ])
}
