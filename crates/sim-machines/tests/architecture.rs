//! The rules that keep the workspace maintainable, checked on every test run:
//! dependencies point downwards, versions are pinned, no machine name outside its crate.

use std::fs;
use std::path::{Path, PathBuf};
use toml::{Table, Value};

fn crates_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf()
}

fn crate_names() -> Vec<String> {
    let mut names: Vec<String> =
        fs::read_dir(crates_dir()).unwrap().map(|e| e.unwrap().file_name().into_string().unwrap()).collect();
    names.sort();
    names
}

fn manifest(name: &str) -> Table {
    fs::read_to_string(crates_dir().join(name).join("Cargo.toml")).unwrap().parse().unwrap()
}

fn deps(manifest: &Table) -> Vec<(String, Value)> {
    ["dependencies", "dev-dependencies", "build-dependencies"]
        .iter()
        .filter_map(|section| manifest.get(*section)?.as_table())
        .flat_map(|t| t.iter().map(|(k, v)| (k.clone(), v.clone())))
        .collect()
}

/// core 0, components 1, machines 2, registration list 3, front ends 4 and 5. The FMI export is the one front end
/// another may use: the command line writes the FMU from the same code as the FMU's binary. The machine that machine
/// files build (`sim-netlist`) is a machine like the others, but names none.
fn layer(name: &str) -> u32 {
    match name {
        "sim-core" => 0,
        "sim-components" => 1,
        n if n.starts_with("machine-") || n == "sim-netlist" => 2,
        "sim-machines" => 3,
        "sim-fmi" => 4,
        _ => 5,
    }
}

/// The one machine built on another, (machine, base): the four-axis delta is the three-arm delta with a fourth axis
/// behind it and takes the base's arms, kinematics and dynamics as they are, so that a fix to the arms reaches both.
/// The base names nothing of the machine built on it; every other machine stays apart from every other.
const BUILT_ON: &[(&str, &str)] = &[("machine-delta-d4", "machine-delta-d3")];

#[test]
fn dependencies_point_downwards() {
    let names = crate_names();
    for name in &names {
        for (dep, _) in deps(&manifest(name)) {
            if names.contains(&dep) {
                let built_on = BUILT_ON.contains(&(name.as_str(), dep.as_str()));
                assert!(layer(&dep) < layer(name) || built_on, "{name} must not depend on {dep}");
            } else if layer(name) < 3 {
                // Core, components and machines are plain Rust: no graphics, interface or network libraries.
                assert_eq!(dep, "toml", "{name} must not depend on {dep}");
            }
        }
    }
}

#[test]
fn every_external_version_is_pinned() {
    let root: Table = fs::read_to_string(crates_dir().parent().unwrap().join("Cargo.toml")).unwrap().parse().unwrap();
    let pinned = |v: &Value| {
        let version = v.as_str().or_else(|| v.get("version")?.as_str());
        v.get("path").is_some() || version.is_some_and(|s| s.starts_with('='))
    };
    for (name, value) in root["workspace"]["dependencies"].as_table().unwrap() {
        assert!(pinned(value), "workspace dependency {name} is not pinned with '='");
    }
    for name in crate_names() {
        for (dep, value) in deps(&manifest(&name)) {
            let from_workspace = value.get("workspace").and_then(Value::as_bool) == Some(true);
            assert!(from_workspace || pinned(&value), "{name}: dependency {dep} is not pinned");
        }
    }
}

fn text_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.is_dir() {
            text_files(&path, out);
        } else if matches!(path.extension().and_then(|e| e.to_str()), Some("rs" | "toml" | "ftl" | "py" | "md" | "txt"))
        {
            out.push(path);
        }
    }
}

#[test]
fn no_machine_name_outside_its_crate() {
    let names = crate_names();
    let mut files = vec![crates_dir().parent().unwrap().join("Cargo.toml")];
    for name in names.iter().filter(|n| layer(n) != 2 && layer(n) != 3) {
        text_files(&crates_dir().join(name), &mut files);
    }
    // `sim-netlist` is no machine of its own to be named, but is scanned like the machines for the others' names.
    for machine in names.iter().filter(|n| n.starts_with("machine-")) {
        // A machine may not name another machine either, but the one it is built on.
        let mut scanned = files.clone();
        let built_on = |other: &String| BUILT_ON.contains(&(other.as_str(), machine.as_str()));
        for other in names.iter().filter(|n| layer(n) == 2 && *n != machine && !built_on(n)) {
            text_files(&crates_dir().join(other), &mut scanned);
        }
        let name = machine.trim_start_matches("machine-");
        let variants = [name.to_string(), name.replace('-', "_"), name.replace('-', " "), name.replace('-', "")];
        for file in &scanned {
            let text = fs::read_to_string(file).unwrap().to_lowercase();
            for variant in &variants {
                assert!(!text.contains(variant), "{} names the machine '{variant}'", file.display());
            }
        }
    }
}
