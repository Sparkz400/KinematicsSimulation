//! FMI export: a machine as an FMI 2.0 Co-Simulation FMU, for Simulink, OpenModelica, Dymola and other tools.
//!
//! FMI 2.0 and not 3.0: every tool named above imports 2.0 Co-Simulation, 3.0 support is still patchy, and the
//! simulator needs nothing that only 3.0 has (clocks, arrays, binary variables).
//!
//! The FMU carries one generic binary (`sim_fmi.dll`, `sim_fmi.so`) for every machine; which machine and which
//! config it runs come from its resources (`resources/model.txt`: machine id and language; `resources/config.toml`).
//! The model description is generated from the machine's signal list and config, never written per machine:
//!
//! | variable                    | type    | causality   | meaning                                               |
//! | --------------------------- | ------- | ----------- | ----------------------------------------------------- |
//! | `time`                      | Real    | independent | simulated time, s                                     |
//! | every other signal id       | Real    | output      | every signal of the simulation, with its unit         |
//! | `command.setpoint.<axis>`   | Real    | input       | position setpoint, interpolated as from a PLC         |
//! | `command.enable`            | Boolean | input       | false switches the drives off (brakes on)             |
//! | `machine`                   | String  | parameter   | the machine of the registry                           |
//! | `config_file`               | String  | parameter   | a config file to use instead of the one in the FMU    |
//! | `effects.<name>`            | Boolean | parameter   | the effect switches, overriding the config's          |
//!
//! The value reference of a variable is its place in the list, unique across types. The names are the signal ids
//! as CSV columns and OPC UA nodes have them; the naming convention is "flat", since ids such as `belt.1.tension`
//! are not identifiers of the "structured" one.
//! A position setpoint goes to the controller as `Command::Setpoint` whenever it differs from the one the
//! controller follows; the controller interpolates towards it over `profile.setpoint_interval_s`, so the
//! communication step should be that interval (the model description's default step). `fmi2DoStep` runs whole
//! base steps (`sim.dt_s`) and refuses a communication step that is not a whole multiple of it.

pub mod api;

use sim_components::{Sim, load_config};
use sim_core::{Config, ConfigError, MachineEntry, Translations};
use std::fmt::Write;

/// (language code, translation file contents) for the export's own labels and messages.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

/// The name of the binary in the FMU and of the C functions' library.
pub const MODEL_IDENTIFIER: &str = "sim_fmi";

/// What a variable stands for.
#[derive(Debug, Clone, PartialEq)]
pub enum Kind {
    /// The signal at this place in `Sim::signal_infos`.
    Signal(usize),
    /// The position setpoint of the axis at this place.
    Setpoint(usize),
    Enable,
    Machine,
    ConfigFile,
    /// The switch `effects.<name>`.
    Effect(String),
}

#[derive(Debug, Clone, PartialEq)]
pub struct Variable {
    pub name: String,
    pub kind: Kind,
    pub description: String,
    pub unit: String,
    /// The start value as written in the model description; none for outputs.
    pub start: Option<String>,
}

impl Variable {
    pub fn causality(&self) -> &'static str {
        match self.kind {
            Kind::Signal(0) => "independent",
            Kind::Signal(_) => "output",
            Kind::Setpoint(_) | Kind::Enable => "input",
            _ => "parameter",
        }
    }
    pub fn type_name(&self) -> &'static str {
        match self.kind {
            Kind::Signal(_) | Kind::Setpoint(_) => "Real",
            Kind::Enable | Kind::Effect(_) => "Boolean",
            Kind::Machine | Kind::ConfigFile => "String",
        }
    }
}

/// Every label the FMU shows, in one language: machines, components and the export's own.
pub fn texts(lang: &str, entry: &MachineEntry) -> Translations {
    let mut tr = sim_machines::translations(lang, entry);
    I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    tr
}

/// An FMU's content apart from its binaries: the machine, the config as text, and the model description generated from them.
pub struct Model {
    pub entry: &'static MachineEntry,
    pub config_text: String,
    pub lang: String,
    pub variables: Vec<Variable>,
    /// Changes whenever the model description does: a hash of it.
    pub guid: String,
    pub xml: String,
}

impl Model {
    /// `config_text` is read as a config file (`load_config`); `lang` is the language of the descriptions.
    pub fn new(entry: &'static MachineEntry, config_text: &str, lang: &str) -> Result<Self, ConfigError> {
        let cfg = load_config(entry, config_text)?;
        let sim = Sim::new(entry, &cfg)?;
        let tr = texts(lang, entry);
        let variables = variables(entry.id, &sim, &cfg, &tr);
        let unsigned = description(entry, &cfg, &tr, &variables, "")?;
        let guid = guid(&unsigned);
        let xml = description(entry, &cfg, &tr, &variables, &guid)?;
        Ok(Self { entry, config_text: config_text.into(), lang: lang.into(), variables, guid, xml })
    }

    /// The files of the FMU apart from its binaries, as (path in the archive, content).
    pub fn files(&self) -> Vec<(String, Vec<u8>)> {
        vec![
            ("modelDescription.xml".into(), self.xml.clone().into_bytes()),
            ("resources/model.txt".into(), format!("{}\n{}\n", self.entry.id, self.lang).into_bytes()),
            ("resources/config.toml".into(), self.config_text.clone().into_bytes()),
        ]
    }
}

fn variables(machine: &str, sim: &Sim, cfg: &Config, tr: &Translations) -> Vec<Variable> {
    let mut out: Vec<Variable> = sim
        .signal_infos()
        .into_iter()
        .enumerate()
        .map(|(i, s)| Variable { description: format!("{} · {}", tr.tr(&s.owner_key), tr.tr(&s.quantity_key)), name: s.id, kind: Kind::Signal(i), unit: s.unit, start: None })
        .collect();
    for (i, axis) in sim.machine.axes().iter().enumerate() {
        let description = format!("{} · {}", tr.tr(axis.key), tr.tr("fmi-setpoint"));
        out.push(Variable { name: format!("command.setpoint.{}", axis.id), kind: Kind::Setpoint(i), description, unit: axis.unit.into(), start: Some(format!("{:?}", sim.control.set_pos[i])) });
    }
    let flag = |name: &str, kind: Kind, key: &str, start: bool| Variable { name: name.into(), kind, description: tr.tr(key).into(), unit: String::new(), start: Some(start.to_string()) };
    out.push(flag("command.enable", Kind::Enable, "fmi-enable", true));
    let text = |name: &str, kind: Kind, key: &str, start: &str| Variable { name: name.into(), kind, description: tr.tr(key).into(), unit: String::new(), start: Some(start.into()) };
    out.push(text("machine", Kind::Machine, "fmi-machine", machine));
    out.push(text("config_file", Kind::ConfigFile, "fmi-config-file", ""));
    for (path, value) in cfg.leaves() {
        if let (Some(name), Some(on)) = (path.strip_prefix("effects."), value.as_bool()) {
            out.push(flag(&path, Kind::Effect(name.into()), &format!("effect-{name}"), on));
        }
    }
    out
}

fn escape(text: &str) -> String {
    text.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;").replace('\'', "&apos;").replace('\n', "&#10;")
}

/// The model description; `guid` empty for the text the GUID is the hash of.
fn description(entry: &MachineEntry, cfg: &Config, tr: &Translations, variables: &[Variable], guid: &str) -> Result<String, ConfigError> {
    let mut x = String::new();
    let tool = format!("sim-fmi {}", env!("CARGO_PKG_VERSION"));
    let _ = writeln!(x, r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    let _ = writeln!(
        x,
        r#"<fmiModelDescription fmiVersion="2.0" modelName="{}" guid="{guid}" description="{}" generationTool="{tool}" variableNamingConvention="flat" numberOfEventIndicators="0">"#,
        escape(entry.id),
        escape(tr.tr(entry.key))
    );
    let _ = writeln!(
        x,
        r#"  <CoSimulation modelIdentifier="{MODEL_IDENTIFIER}" canHandleVariableCommunicationStepSize="true" canInterpolateInputs="false" canBeInstantiatedOnlyOncePerProcess="false" canNotUseMemoryManagementFunctions="true" canGetAndSetFMUstate="false" canSerializeFMUstate="false" providesDirectionalDerivative="false"/>"#
    );
    let mut units: Vec<&str> = variables.iter().map(|v| v.unit.as_str()).filter(|u| !u.is_empty()).collect();
    units.sort();
    units.dedup();
    let _ = writeln!(x, "  <UnitDefinitions>");
    for unit in units {
        let _ = writeln!(x, r#"    <Unit name="{}"/>"#, escape(unit));
    }
    let _ = writeln!(x, "  </UnitDefinitions>");
    let _ = writeln!(x, r#"  <DefaultExperiment startTime="0" stepSize="{:?}"/>"#, cfg.positive("profile.setpoint_interval_s")?);
    let _ = writeln!(x, "  <ModelVariables>");
    for (vr, v) in variables.iter().enumerate() {
        let variability = match (v.causality(), v.type_name()) {
            ("parameter", _) => "fixed",
            (_, "Real") => "continuous",
            _ => "discrete",
        };
        let _ = writeln!(x, r#"    <ScalarVariable name="{}" valueReference="{vr}" causality="{}" variability="{variability}" description="{}">"#, escape(&v.name), v.causality(), escape(&v.description));
        let start = v.start.as_ref().map_or(String::new(), |s| format!(r#" start="{}""#, escape(s)));
        let unit = if v.unit.is_empty() { String::new() } else { format!(r#" unit="{}""#, escape(&v.unit)) };
        let _ = writeln!(x, "      <{}{start}{unit}/>", v.type_name());
        let _ = writeln!(x, "    </ScalarVariable>");
    }
    let _ = writeln!(x, "  </ModelVariables>");
    // Outputs are calculated during initialisation, so they are initial unknowns as well.
    let outputs: Vec<usize> = variables.iter().enumerate().filter(|(_, v)| v.causality() == "output").map(|(i, _)| i + 1).collect();
    let _ = writeln!(x, "  <ModelStructure>");
    for list in ["Outputs", "InitialUnknowns"] {
        let _ = writeln!(x, "    <{list}>");
        for index in &outputs {
            let _ = writeln!(x, r#"      <Unknown index="{index}"/>"#);
        }
        let _ = writeln!(x, "    </{list}>");
    }
    let _ = writeln!(x, "  </ModelStructure>");
    let _ = writeln!(x, "</fmiModelDescription>");
    Ok(x)
}

/// A GUID-shaped 128-bit hash (two FNV-1a 64-bit hashes with different offsets): stable across builds and platforms,
/// which `std`'s hasher is not.
fn guid(text: &str) -> String {
    let fnv = |offset: u64| text.bytes().fold(offset, |h, b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3));
    let (a, b) = (fnv(0xcbf2_9ce4_8422_2325), fnv(0x6c62_272e_07bb_0142));
    format!("{{{:08x}-{:04x}-{:04x}-{:04x}-{:012x}}}", a >> 32, (a >> 16) & 0xffff, a & 0xffff, b >> 48, b & 0xffff_ffff_ffff)
}

/// Where a binary goes in the FMU, from its file name: `binaries/<platform>/sim_fmi.<extension>`.
pub fn binary_path(file_name: &str) -> Option<String> {
    let extension = file_name.rsplit_once('.')?.1.to_ascii_lowercase();
    let platform = match extension.as_str() {
        "dll" => "win64",
        "so" => "linux64",
        "dylib" => "darwin64",
        _ => return None,
    };
    Some(format!("binaries/{platform}/{MODEL_IDENTIFIER}.{extension}"))
}

/// The CRC-32 a zip archive carries per file.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc = !0u32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb8_8320 & (crc & 1).wrapping_neg());
        }
    }
    !crc
}

/// A zip archive of the files, stored without compression: an FMU is a zip, and stored entries need no library.
pub fn zip(files: &[(String, Vec<u8>)]) -> Vec<u8> {
    let (mut out, mut central) = (Vec::new(), Vec::new());
    for (name, data) in files {
        let (crc, size, offset) = (crc32(data), data.len() as u32, out.len() as u32);
        // Version 2.0, UTF-8 names (bit 11), stored, no time stamp (1980-01-01).
        let common = |out: &mut Vec<u8>| {
            out.extend(20u16.to_le_bytes());
            out.extend(0x0800u16.to_le_bytes());
            out.extend(0u16.to_le_bytes());
            out.extend([0, 0, 0x21, 0]);
            out.extend(crc.to_le_bytes());
            out.extend(size.to_le_bytes());
            out.extend(size.to_le_bytes());
            out.extend((name.len() as u16).to_le_bytes());
            out.extend(0u16.to_le_bytes());
        };
        out.extend(0x0403_4b50u32.to_le_bytes());
        common(&mut out);
        out.extend(name.as_bytes());
        out.extend(data);
        central.extend(0x0201_4b50u32.to_le_bytes());
        central.extend(20u16.to_le_bytes());
        common(&mut central);
        // Comment length, disk, internal and external attributes.
        central.extend([0u8; 10]);
        central.extend(offset.to_le_bytes());
        central.extend(name.as_bytes());
    }
    let (start, length) = (out.len() as u32, central.len() as u32);
    out.extend(central);
    out.extend(0x0605_4b50u32.to_le_bytes());
    out.extend([0u8; 4]);
    out.extend((files.len() as u16).to_le_bytes());
    out.extend((files.len() as u16).to_le_bytes());
    out.extend(length.to_le_bytes());
    out.extend(start.to_le_bytes());
    out.extend(0u16.to_le_bytes());
    out
}

#[cfg(test)]
mod tests {
    #[test]
    fn crc32_of_the_standard_check_string() {
        assert_eq!(super::crc32(b"123456789"), 0xcbf4_3926);
    }

    /// Windows does not care about case: `SIM_FMI.DLL` is the Windows binary, stored under the standard's name.
    #[test]
    fn a_binary_is_known_by_its_extension_in_any_case() {
        assert_eq!(super::binary_path("SIM_FMI.DLL").as_deref(), Some("binaries/win64/sim_fmi.dll"));
        assert_eq!(super::binary_path("libsim_fmi.So").as_deref(), Some("binaries/linux64/sim_fmi.so"));
        assert_eq!(super::binary_path("sim_fmi.exe"), None);
    }

    /// Every key the export shows has a German and an English entry.
    #[test]
    fn every_export_message_exists_in_every_language() {
        let sources = [include_str!("lib.rs"), include_str!("api.rs")].concat();
        let keys: Vec<&str> = sources.split('"').filter(|s| s.starts_with("fmi-") && s.len() > 4 && s.chars().all(|c| c.is_ascii_lowercase() || c == '-')).collect();
        assert!(keys.len() >= 10, "{keys:?}");
        for (lang, text) in super::I18N {
            let tr = sim_core::Translations::parse(text);
            for key in &keys {
                assert!(tr.has(key), "no '{lang}' entry for '{key}'");
            }
        }
    }
}
