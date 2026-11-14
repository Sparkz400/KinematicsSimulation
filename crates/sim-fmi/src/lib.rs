//! FMI export: a machine as an FMI 2.0 or FMI 3.0 Co-Simulation FMU, for Simulink, OpenModelica, Dymola and other tools.
//!
//! One binary serves both versions: the `fmi2*` functions are in `api`, the `fmi3*` ones in `api3`, and both drive
//! the same `api::Instance`. Model Exchange is not offered (docs/fmi.md says why: the controller is a sampled
//! controller whose internal states the importer's integrator cannot see). FMU state get, set and serialization
//! work by replaying the inputs from a fresh simulation; the directional derivatives of the outputs with respect to
//! the inputs are exactly zero, because an input acts on the outputs only through the next communication step.
//!
//! The FMU carries one generic binary (`sim_fmi.dll`, `sim_fmi.so`) for every machine; which machine and which
//! config it runs come from its resources (`resources/model.txt`: machine id and language; `resources/config.toml`).
//! A machine from a machine file (docs/machine-files.md) is not registered: its file goes along as
//! `resources/machine.toml`, and the FMU builds the machine from it when it is instantiated.
//! The model description is generated from the machine's signal list and config, never written per machine:
//!
//! | variable                    | type    | causality   | meaning                                               |
//! | --------------------------- | ------- | ----------- | ----------------------------------------------------- |
//! | `time`                      | Real    | independent | simulated time, s (Real in 2.0 is Float64 in 3.0)     |
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
//! communication step should be that interval (the model description's default step). `fmi2DoStep` and `fmi3DoStep`
//! run whole base steps (`sim.dt_s`) and refuse a communication step that is not a whole multiple of it.

pub mod api;
pub mod api3;

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
    /// The FMI version of the model description: 2 or 3.
    pub fmi: u8,
}

impl Model {
    /// An FMI 2.0 FMU. `config_text` is read as a config file (`load_config`); `lang` is the language of the descriptions.
    pub fn new(entry: &'static MachineEntry, config_text: &str, lang: &str) -> Result<Self, ConfigError> {
        Self::with_fmi(entry, config_text, lang, 2)
    }

    /// As `new`, for FMI 3.0 if `fmi` is 3 and FMI 2.0 otherwise. In 3.0 the GUID is the instantiation token.
    pub fn with_fmi(entry: &'static MachineEntry, config_text: &str, lang: &str, fmi: u8) -> Result<Self, ConfigError> {
        let fmi = if fmi == 3 { 3 } else { 2 };
        let cfg = load_config(entry, config_text)?;
        let sim = Sim::new(entry, &cfg)?;
        let tr = texts(lang, entry);
        let variables = variables(entry.id, &sim, &cfg, &tr);
        let unsigned = description(entry, &cfg, &tr, &variables, "", fmi, sim.dt)?;
        let guid = guid(&unsigned);
        let xml = description(entry, &cfg, &tr, &variables, &guid, fmi, sim.dt)?;
        Ok(Self { entry, config_text: config_text.into(), lang: lang.into(), variables, guid, xml, fmi })
    }

    /// The files of the FMU apart from its binaries, as (path in the archive, content).
    pub fn files(&self) -> Vec<(String, Vec<u8>)> {
        let mut out = vec![
            ("modelDescription.xml".into(), self.xml.clone().into_bytes()),
            ("resources/model.txt".into(), format!("{}\n{}\n", self.entry.id, self.lang).into_bytes()),
            ("resources/config.toml".into(), self.config_text.clone().into_bytes()),
        ];
        if !is_registered(self.entry) {
            out.push(("resources/machine.toml".into(), self.entry.default_config.as_bytes().to_vec()));
        }
        out
    }
}

/// Whether `entry` is one of `sim_machines::MACHINES`, not a machine from a file.
pub fn is_registered(entry: &MachineEntry) -> bool {
    sim_machines::find(entry.id).is_some_and(|known| std::ptr::eq(known, entry))
}

/// The machine of an FMU's resources: from `machine.toml` where it carries a machine file, else the registered one
/// of that id.
pub fn machine_of(resources: &std::path::Path, id: &str) -> Result<&'static MachineEntry, ConfigError> {
    match std::fs::read_to_string(resources.join("machine.toml")) {
        Ok(text) => sim_machines::load_text(&text, id),
        Err(_) => sim_machines::find(id).ok_or_else(|| ConfigError::Missing(id.into())),
    }
}

fn variables(machine: &str, sim: &Sim, cfg: &Config, tr: &Translations) -> Vec<Variable> {
    let mut out: Vec<Variable> = sim
        .signal_infos()
        .into_iter()
        .enumerate()
        .map(|(i, s)| Variable {
            description: format!("{} · {}", tr.tr(&s.owner_key), tr.tr(&s.quantity_key)),
            name: s.id,
            kind: Kind::Signal(i),
            unit: s.unit,
            start: None,
        })
        .collect();
    for (i, axis) in sim.machine.axes().iter().enumerate() {
        let description = format!("{} · {}", tr.tr(axis.key), tr.tr("fmi-setpoint"));
        out.push(Variable {
            name: format!("command.setpoint.{}", axis.id),
            kind: Kind::Setpoint(i),
            description,
            unit: axis.unit.into(),
            start: Some(format!("{:?}", sim.control.set_pos[i])),
        });
    }
    let flag = |name: &str, kind: Kind, key: &str, start: bool| Variable {
        name: name.into(),
        kind,
        description: tr.tr(key).into(),
        unit: String::new(),
        start: Some(start.to_string()),
    };
    out.push(flag("command.enable", Kind::Enable, "fmi-enable", true));
    let text = |name: &str, kind: Kind, key: &str, start: &str| Variable {
        name: name.into(),
        kind,
        description: tr.tr(key).into(),
        unit: String::new(),
        start: Some(start.into()),
    };
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
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
        .replace('\n', "&#10;")
}

/// The model description of FMI `fmi` (2 or 3); `guid` empty for the text the GUID is the hash of; `dt` the base step.
fn description(
    entry: &MachineEntry,
    cfg: &Config,
    tr: &Translations,
    variables: &[Variable],
    guid: &str,
    fmi: u8,
    dt: f64,
) -> Result<String, ConfigError> {
    let v3 = fmi == 3;
    let mut x = String::new();
    let tool = format!("sim-fmi {}", env!("CARGO_PKG_VERSION"));
    let (name, about) = (escape(entry.id), escape(tr.tr(entry.key)));
    let _ = writeln!(x, r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    if v3 {
        let _ = writeln!(
            x,
            r#"<fmiModelDescription fmiVersion="3.0" modelName="{name}" instantiationToken="{guid}" description="{about}" generationTool="{tool}" variableNamingConvention="flat">"#
        );
        let _ = writeln!(
            x,
            r#"  <CoSimulation modelIdentifier="{MODEL_IDENTIFIER}" canHandleVariableCommunicationStepSize="true" fixedInternalStepSize="{dt:?}" canBeInstantiatedOnlyOncePerProcess="false" canGetAndSetFMUState="true" canSerializeFMUState="true" providesDirectionalDerivatives="true" providesAdjointDerivatives="true" hasEventMode="false"/>"#
        );
    } else {
        let _ = writeln!(
            x,
            r#"<fmiModelDescription fmiVersion="2.0" modelName="{name}" guid="{guid}" description="{about}" generationTool="{tool}" variableNamingConvention="flat" numberOfEventIndicators="0">"#
        );
        let _ = writeln!(
            x,
            r#"  <CoSimulation modelIdentifier="{MODEL_IDENTIFIER}" canHandleVariableCommunicationStepSize="true" canInterpolateInputs="false" canBeInstantiatedOnlyOncePerProcess="false" canNotUseMemoryManagementFunctions="true" canGetAndSetFMUstate="true" canSerializeFMUstate="true" providesDirectionalDerivative="true"/>"#
        );
    }
    let mut units: Vec<&str> = variables.iter().map(|v| v.unit.as_str()).filter(|u| !u.is_empty()).collect();
    units.sort();
    units.dedup();
    let _ = writeln!(x, "  <UnitDefinitions>");
    for unit in units {
        let _ = writeln!(x, r#"    <Unit name="{}"/>"#, escape(unit));
    }
    let _ = writeln!(x, "  </UnitDefinitions>");
    let _ = writeln!(
        x,
        r#"  <DefaultExperiment startTime="0" stepSize="{:?}"/>"#,
        cfg.positive("profile.setpoint_interval_s")?
    );
    let _ = writeln!(x, "  <ModelVariables>");
    // FMI 2.0 (2.2.7) gives an independent variable no start attribute, and FMIL (the FMI library of OpenModelica and
    // other importers) reports one without it as an error; where none is declared, `time` is there implicitly. So 2.0
    // leaves it out of the list (its value reference still answers), and 3.0, whose libraries accept it, declares it.
    let written: Vec<usize> =
        (0..variables.len()).filter(|&i| v3 || variables[i].causality() != "independent").collect();
    for &vr in &written {
        let v = &variables[vr];
        let variability = match (v.causality(), v.type_name()) {
            ("parameter", _) => "fixed",
            (_, "Real") => "continuous",
            _ => "discrete",
        };
        let attributes = format!(
            r#"name="{}" valueReference="{vr}" causality="{}" variability="{variability}" description="{}""#,
            escape(&v.name),
            v.causality(),
            escape(&v.description)
        );
        let start = v.start.as_ref().map_or(String::new(), |s| format!(r#" start="{}""#, escape(s)));
        let unit = if v.unit.is_empty() { String::new() } else { format!(r#" unit="{}""#, escape(&v.unit)) };
        match (v3, v.type_name()) {
            // FMI 3.0 gives a String's start as an element of its own.
            (true, "String") => {
                let _ = writeln!(x, "    <String {attributes}>");
                let _ = writeln!(x, r#"      <Start value="{}"/>"#, escape(v.start.as_deref().unwrap_or("")));
                let _ = writeln!(x, "    </String>");
            }
            (true, type_name) => {
                let _ = writeln!(
                    x,
                    "    <{} {attributes}{start}{unit}/>",
                    if type_name == "Real" { "Float64" } else { type_name }
                );
            }
            (false, type_name) => {
                let _ = writeln!(x, "    <ScalarVariable {attributes}>");
                let _ = writeln!(x, "      <{type_name}{start}{unit}/>");
                let _ = writeln!(x, "    </ScalarVariable>");
            }
        }
    }
    let _ = writeln!(x, "  </ModelVariables>");
    // Outputs are calculated during initialisation, so they are initial unknowns as well. They depend on no input
    // directly (dependencies=""): a setpoint reaches them only through the next communication step.
    let outputs: Vec<usize> =
        variables.iter().enumerate().filter(|(_, v)| v.causality() == "output").map(|(i, _)| i).collect();
    let _ = writeln!(x, "  <ModelStructure>");
    if v3 {
        for vr in &outputs {
            let _ = writeln!(x, r#"    <Output valueReference="{vr}" dependencies=""/>"#);
        }
        for vr in &outputs {
            let _ = writeln!(x, r#"    <InitialUnknown valueReference="{vr}"/>"#);
        }
    } else {
        for (list, dependencies) in [("Outputs", r#" dependencies="""#), ("InitialUnknowns", "")] {
            let _ = writeln!(x, "    <{list}>");
            for vr in &outputs {
                // The index counts the variables as listed (from 1), not their value references.
                let index = written.iter().position(|w| w == vr).expect("outputs are listed") + 1;
                let _ = writeln!(x, r#"      <Unknown index="{index}"{dependencies}/>"#);
            }
            let _ = writeln!(x, "    </{list}>");
        }
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
    format!(
        "{{{:08x}-{:04x}-{:04x}-{:04x}-{:012x}}}",
        a >> 32,
        (a >> 16) & 0xffff,
        a & 0xffff,
        b >> 48,
        b & 0xffff_ffff_ffff
    )
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

/// Where a binary goes in an FMI 3.0 FMU: `binaries/<architecture>-<system>/sim_fmi.<extension>`, the system from
/// the extension and the architecture (x86_64 or aarch64) from the binary's header (PE, ELF or a thin 64-bit Mach-O).
/// None for anything else, a universal Mach-O among it: FMI 3.0 wants one folder per architecture.
pub fn binary_path3(file_name: &str, bytes: &[u8]) -> Option<String> {
    let extension = file_name.rsplit_once('.')?.1.to_ascii_lowercase();
    let u16_at = |i: usize| Some(u16::from_le_bytes(bytes.get(i..i.checked_add(2)?)?.try_into().ok()?));
    let u32_at = |i: usize| Some(u32::from_le_bytes(bytes.get(i..i.checked_add(4)?)?.try_into().ok()?));
    // PE machine, ELF e_machine, Mach-O cputype.
    let (system, machine) = match extension.as_str() {
        "dll" if bytes.starts_with(b"MZ") => {
            let pe = u32_at(0x3c)? as usize;
            if bytes.get(pe..pe.checked_add(4)?)? != b"PE\0\0" {
                return None;
            }
            ("windows", u32::from(u16_at(pe + 4)?))
        }
        "so" if bytes.starts_with(b"\x7fELF") => ("linux", u32::from(u16_at(18)?)),
        "dylib" if bytes.starts_with(&[0xcf, 0xfa, 0xed, 0xfe]) => ("darwin", u32_at(4)?),
        _ => return None,
    };
    let architecture = match (system, machine) {
        ("windows", 0x8664) | ("linux", 62) | ("darwin", 0x0100_0007) => "x86_64",
        ("windows", 0xaa64) | ("linux", 183) | ("darwin", 0x0100_000c) => "aarch64",
        _ => return None,
    };
    Some(format!("binaries/{architecture}-{system}/{MODEL_IDENTIFIER}.{extension}"))
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

    /// FMI 3.0 puts a binary in the folder of its architecture, read from its header.
    #[test]
    fn an_fmi3_binary_goes_to_the_folder_of_its_architecture() {
        let mut pe = vec![0u8; 0x48];
        pe[..2].copy_from_slice(b"MZ");
        pe[0x3c] = 0x40;
        pe[0x40..0x44].copy_from_slice(b"PE\0\0");
        pe[0x44..0x46].copy_from_slice(&0x8664u16.to_le_bytes());
        assert_eq!(super::binary_path3("SIM_FMI.DLL", &pe).as_deref(), Some("binaries/x86_64-windows/sim_fmi.dll"));
        let mut elf = b"\x7fELF".to_vec();
        elf.resize(20, 0);
        elf[18] = 183;
        assert_eq!(super::binary_path3("libsim_fmi.so", &elf).as_deref(), Some("binaries/aarch64-linux/sim_fmi.so"));
        let macho = [[0xcf, 0xfa, 0xed, 0xfe], 0x0100_0007u32.to_le_bytes()].concat();
        assert_eq!(
            super::binary_path3("libsim_fmi.dylib", &macho).as_deref(),
            Some("binaries/x86_64-darwin/sim_fmi.dylib")
        );
        let universal = [0xca, 0xfe, 0xba, 0xbe, 0, 0, 0, 2];
        assert_eq!(super::binary_path3("libsim_fmi.dylib", &universal), None);
        assert_eq!(super::binary_path3("sim_fmi.dll", b"windows binary"), None);
        // The test program itself has the header of the platform it runs on.
        let me = std::fs::read(std::env::current_exe().unwrap()).unwrap();
        let extension = if cfg!(windows) {
            "dll"
        } else if cfg!(target_os = "macos") {
            "dylib"
        } else {
            "so"
        };
        let path = super::binary_path3(&format!("sim_fmi.{extension}"), &me).unwrap();
        assert!(path.contains(std::env::consts::ARCH), "{path}");
    }

    /// Every key the export shows has a German and an English entry.
    #[test]
    fn every_export_message_exists_in_every_language() {
        let sources = [include_str!("lib.rs"), include_str!("api.rs"), include_str!("api3.rs")].concat();
        let keys: Vec<&str> = sources
            .split('"')
            .filter(|s| s.starts_with("fmi-") && s.len() > 4 && s.chars().all(|c| c.is_ascii_lowercase() || c == '-'))
            .collect();
        assert!(keys.len() >= 10, "{keys:?}");
        for (lang, text) in super::I18N {
            let tr = sim_core::Translations::parse(text);
            for key in &keys {
                assert!(tr.has(key), "no '{lang}' entry for '{key}'");
            }
        }
    }
}
