//! Machines defined in a file, without Rust code: a machine config (the tables every machine has: `[sim]`,
//! `[effects]`, `[belt]`, `[motor]`, `[drive]`, `[ratings]`, `[cycle]`, ...) plus a `[machine]` table that wires
//! components together as a netlist (see `netlist::NODES` and docs/machine-files.md).
//!
//! `from_text` checks a file's text into a `MachineEntry` that every front end takes like a registered machine;
//! `sim_machines::load_file` reads the file (TOML, or JSON converted).

pub mod geometry;
pub mod linkage;
pub mod machine;
pub mod netlist;
pub mod scissor;
pub mod spatial;
pub mod worm;

pub use machine::ConfigMachine;
pub use netlist::{Netlist, VERSION, json_schema};

use machine::intern;
use sim_core::{Config, ConfigError, Machine, MachineEntry};
use std::fmt;
use std::path::PathBuf;
use toml::{Table, Value};

/// `MachineEntry::build` of every machine file: the netlist is in the config itself.
pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let net = Netlist::parse(cfg)?;
    if let Some(scissor) = &net.scissor {
        return sim_components::base::mount(cfg, Box::new(scissor::ScissorMachine::from_node(cfg, &net, scissor)?));
    }
    match &net.linkage {
        Some(linkage) if linkage.gearboxes.len() == 3 => {
            sim_components::base::mount(cfg, Box::new(spatial::Delta::from_node(cfg, &net, linkage)?))
        }
        Some(linkage) => {
            sim_components::base::mount(cfg, Box::new(linkage::LinkageMachine::from_node(cfg, &net, linkage)?))
        }
        None => sim_components::base::mount(cfg, Box::new(ConfigMachine::new(cfg, &net)?)),
    }
}

/// shortcut: a machine file brings no loss-free configs for the standard suite; the tests build their own.
fn no_suite_configs() -> Vec<Config> {
    Vec::new()
}

/// Why a machine file was refused: the file, the line the error is on where it can be found, and the error (a
/// translation key and the config path it is about).
#[derive(Debug, Clone, PartialEq)]
pub struct FileError {
    pub file: PathBuf,
    pub line: Option<usize>,
    pub error: ConfigError,
}

impl fmt::Display for FileError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.file.display())?;
        if let Some(line) = self.line {
            write!(f, ":{line}")?;
        }
        write!(f, ": {}: {}", self.error.key(), self.error.detail())
    }
}

impl std::error::Error for FileError {}

/// A machine file's TOML text, checked in full: the netlist, every key against `known` (the default configs of the
/// registered machines, whose tables a machine file shares), and every value the machine and the simulation read.
/// `fallback_id` names the machine where `machine.id` does not. `lines` says whether the text is the file as written,
/// so that an error can name its line (a JSON file arrives here converted, and its lines are not these). The entry
/// lives as long as the program. Files are read, and JSON converted, by `sim_machines::load_file`.
pub fn from_text(
    toml_text: &str,
    lines: bool,
    fallback_id: &str,
    known: &[Config],
) -> Result<&'static MachineEntry, (ConfigError, Option<usize>)> {
    let located = |e: ConfigError| {
        let line = if lines { line_of(toml_text, e.detail()) } else { None };
        (e, line)
    };
    let cfg = Config::parse(toml_text).map_err(|e| (e, None))?;
    let net = Netlist::parse(&cfg).map_err(located)?;
    cfg.check_keys(&schema(&cfg, &net, known), &sim_components::analysis::RATINGS_BEYOND_DEFAULT).map_err(located)?;
    let text_of = |key: &str| cfg.get(key).ok().and_then(Value::as_str).map(str::to_string);
    let id = text_of("machine.id").unwrap_or_else(|| fallback_id.to_string());
    let name = text_of("machine.name").unwrap_or_else(|| id.clone());
    let entry: &'static MachineEntry = Box::leak(Box::new(MachineEntry {
        id: intern(&id),
        key: intern(&name),
        default_config: Box::leak(toml_text.to_string().into_boxed_str()),
        i18n: &[],
        build,
        suite_configs: no_suite_configs,
    }));
    // Everything the simulation reads at its start, with the parts the file names from the catalogue.
    sim_components::Sim::new(entry, &cfg).map_err(located)?;
    Ok(entry)
}

/// The keys a machine file may have: every key of a registered machine's default config, `[machine]`, every value
/// the netlist names by its path (`"geometry.stroke_y_m"`), and per axis and guide of the netlist an `[axis.<id>]` and
/// a friction table laid out as theirs.
fn schema(cfg: &Config, net: &Netlist, known: &[Config]) -> Config {
    fn merge(into: &mut Table, from: &Table) {
        for (key, value) in from {
            match (into.get_mut(key), value) {
                (Some(Value::Table(a)), Value::Table(b)) => merge(a, b),
                (None, v) => {
                    into.insert(key.clone(), v.clone());
                }
                _ => {}
            }
        }
    }
    let mut all = Table::new();
    for c in known {
        if let Ok(text) = c.to_text() {
            merge(&mut all, &text.parse().unwrap_or_default());
        }
    }
    let axis = all.get("axis").and_then(|a| a.get("x")).cloned().unwrap_or_else(|| Value::Table(Table::new()));
    let friction = all.get("friction_x").cloned().unwrap_or_else(|| Value::Table(Table::new()));
    // A rotary axis's limits are in rad: max_velocity_rad_s, as the registered rotary table names them.
    let turning = match &axis {
        Value::Table(t) => Value::Table(t.iter().map(|(k, v)| (k.replace("_m_s", "_rad_s"), v.clone())).collect()),
        other => other.clone(),
    };
    let mut axes = Table::new();
    for a in &net.axes {
        axes.insert(a.id.clone(), if a.rotary { turning.clone() } else { axis.clone() });
    }
    all.insert("axis".into(), Value::Table(axes));
    // A motor's own tables (`catalog::motor_config`), laid out as the shared ones; a process force per axis.
    for table in sim_components::catalog::PER_MOTOR_TABLES {
        if let Some(Value::Table(shared)) = all.get(table).cloned() {
            let plain: Table = shared.into_iter().filter(|(_, v)| !v.is_table()).collect();
            if let Some(Value::Table(t)) = all.get_mut(table) {
                for m in &net.motors {
                    t.insert(m.id.clone(), Value::Table(plain.clone()));
                }
            }
        }
    }
    if let Some(Value::Table(process)) = all.get_mut("process") {
        for a in &net.axes {
            process.insert(format!("force_{}_n", a.id), Value::Float(0.0));
        }
    }
    for g in &net.guides {
        // At a dotted path ("friction.x") as nested tables; where a known value is in the way, the file's table there
        // stays unknown and is refused by name.
        let nested = g
            .friction
            .rsplit('.')
            .fold(friction.clone(), |inner, part| Value::Table(Table::from_iter([(part.to_string(), inner)])));
        if let Value::Table(nested) = nested {
            merge(&mut all, &nested);
        }
    }
    // The value tables of screws, racks, linear motors and strain wave gears, laid out as the registered one-axis machines'.
    for (layout, path) in &net.values {
        let Some(known) = all.get(*layout).cloned() else { continue };
        let nested =
            path.rsplit('.').fold(known, |inner, part| Value::Table(Table::from_iter([(part.to_string(), inner)])));
        if let Value::Table(nested) = nested {
            merge(&mut all, &nested);
        }
    }
    for path in &net.refs {
        let nested = path
            .rsplit('.')
            .fold(Value::Float(0.0), |inner, part| Value::Table(Table::from_iter([(part.to_string(), inner)])));
        if let Value::Table(nested) = nested {
            merge(&mut all, &nested);
        }
    }
    // A dual-drive gantry's synchronisation (`gantry.sync`, read by the controller).
    if net.gantry.as_ref().is_some_and(|g| g.drives.iter().flatten().count() >= 2) {
        let sync = Table::from_iter([
            ("sync".to_string(), Value::String("plain".into())),
            ("sync_gain_1_s".to_string(), Value::Float(0.0)),
        ]);
        merge(&mut all, &Table::from_iter([("gantry".to_string(), Value::Table(sync))]));
    }
    if let Ok(machine) = cfg.get("machine") {
        all.insert("machine".into(), machine.clone());
    }
    all.insert("format_version".into(), Value::Integer(sim_core::config::FORMAT_VERSION));
    Config::parse(&toml::to_string(&all).unwrap_or_default()).unwrap_or_else(|_| cfg.clone())
}

/// The line (from 1) of a TOML text that an error's path is about: `machine.<list>[<id>].<field>` (or `[#<n>]`, the
/// n-th node of the list), or a dotted key `table.sub.key`. Falls back to the node or the table; `None` where neither
/// is in the text.
pub fn line_of(text: &str, detail: &str) -> Option<usize> {
    let path = detail.split([' ', '(']).next().unwrap_or_default();
    let lines: Vec<&str> = text.lines().collect();
    let header = |line: &str| line.trim_start().starts_with('[');
    let header_name =
        |line: &str| line.split('#').next().unwrap_or_default().trim().trim_matches(['[', ']']).trim().to_string();
    let key_of = |line: &str| line.split('=').next().unwrap_or_default().trim().trim_matches('"').to_string();
    // The first line from `start` up to the next header that sets `key`.
    let key_below = |start: usize, key: &str| {
        (start..lines.len()).take_while(|&i| i == start || !header(lines[i])).find(|&i| key_of(lines[i]) == key)
    };

    if let Some(rest) = path.strip_prefix("machine.").filter(|r| r.contains('[')) {
        let (list, rest) = rest.split_once('[')?;
        let (node, field) = rest.split_once(']')?;
        let field = field.trim_start_matches('.').split('.').next().unwrap_or_default();
        let start = if let Some(n) = node.strip_prefix('#') {
            let n: usize = n.parse().ok()?;
            lines
                .iter()
                .enumerate()
                .filter(|(_, l)| header(l) && header_name(l) == format!("machine.{list}"))
                .nth(n.checked_sub(1)?)?
                .0
        } else {
            // Ids are unique in a file: the line that gives this one starts its node (or is all of it, inline).
            // The line that gives this id under a header of its list ([[machine.<list>]]), or inline under [machine]:
            // the machine's own id and comments may give the same text elsewhere.
            (0..lines.len()).find(|&i| {
                let compact: String =
                    lines[i].split('#').next().unwrap_or_default().chars().filter(|c| !c.is_whitespace()).collect();
                let under = (0..=i).rev().find(|&h| header(lines[h])).map(|h| header_name(lines[h]));
                compact.contains(&format!("id=\"{node}\""))
                    && (under.as_deref() == Some(&format!("machine.{list}"))
                        || (under.as_deref() == Some("machine") && compact.contains('{')))
            })?
        };
        // The node's header is the nearest one above its id.
        let top = (0..=start).rev().find(|&i| header(lines[i])).unwrap_or(start);
        // A node written inline (`{ id = "a", ... }`) has no key lines of its own: then its id's line.
        let found = if field.is_empty() { None } else { key_below(top, field) };
        return Some(found.unwrap_or(start) + 1);
    }
    // The longest table prefix that has a header, then the key below it.
    let parts: Vec<&str> = path.split('.').collect();
    for split in (0..=parts.len()).rev() {
        let (table, key) = (parts[..split].join("."), parts[split..].join("."));
        let top =
            if table.is_empty() { Some(0) } else { lines.iter().position(|l| header(l) && header_name(l) == table) };
        if let Some(top) = top {
            return Some(if key.is_empty() { top } else { key_below(top, &key).unwrap_or(top) } + 1);
        }
    }
    None
}
