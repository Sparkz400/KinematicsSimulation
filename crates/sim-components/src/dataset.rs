//! Published measured datasets (`datasets/<id>/`): a `dataset.toml` with the metadata, a README, and per
//! recording a trace file with its mapping. `check` says what keeps a folder from being published; the
//! format is described in datasets/README.md.

use crate::identify::parse_sweep;
use crate::import::{Mapping, decode, import};
use std::path::Path;
use toml::{Table, Value};

pub const LICENSES: [&str; 5] = ["CC0-1.0", "CC-BY-4.0", "CC-BY-SA-4.0", "ODbL-1.0", "PDDL-1.0"];
/// The test moves of docs/measurement-procedure.md.
pub const TESTS: [&str; 7] = ["inertia-sines", "friction-map", "slow-reversal", "torque-step", "noise-excitation", "frequency-response", "cycle"];

/// (table, key, required, kind): `s` text, `n` number, `b` true/false.
const SCHEMA: &[(&str, &str, bool, char)] = &[
    ("dataset", "id", true, 's'),
    ("dataset", "title", true, 's'),
    ("dataset", "description", true, 's'),
    ("dataset", "license", true, 's'),
    ("dataset", "contributor", true, 's'),
    ("dataset", "recorded", true, 's'),
    ("dataset", "synthetic", true, 'b'),
    ("machine", "kind", true, 's'),
    ("machine", "transmission", true, 's'),
    ("machine", "mounting", true, 's'),
    ("machine", "travel_per_motor_rev_m", false, 'n'),
    ("machine", "payload_kg", false, 'n'),
    ("drive", "vendor", true, 's'),
    ("drive", "family", true, 's'),
    ("drive", "current_cycle_s", false, 'n'),
    ("drive", "speed_cycle_s", false, 'n'),
    ("drive", "position_cycle_s", false, 'n'),
    ("motor", "family", true, 's'),
    ("motor", "rated_torque_nm", true, 'n'),
    ("motor", "rotor_inertia_kg_m2", false, 'n'),
    ("motor", "encoder_counts_per_rev", false, 'n'),
    ("recording", "file", true, 's'),
    ("recording", "map", false, 's'),
    ("recording", "test", true, 's'),
    ("recording", "sample_time_s", true, 'n'),
    ("recording", "notes", false, 's'),
];

fn kind_ok(value: &Value, kind: char) -> bool {
    match kind {
        's' => value.is_str(),
        'b' => value.is_bool(),
        _ => value.as_float().or(value.as_integer().map(|i| i as f64)).is_some_and(|v| v.is_finite() && v >= 0.0),
    }
}

/// Checks a table against the schema of `name`: required keys there, every key known, values of their kind.
fn check_table(name: &str, table: &Table, problems: &mut Vec<String>) {
    let schema: Vec<_> = SCHEMA.iter().filter(|s| s.0 == name).collect();
    for (_, key, required, kind) in &schema {
        match table.get(*key) {
            None if *required => problems.push(format!("{name}.{key} is missing")),
            Some(v) if !kind_ok(v, *kind) => problems.push(format!("{name}.{key} has the wrong kind of value")),
            _ => {}
        }
    }
    for key in table.keys().filter(|k| !schema.iter().any(|s| s.1 == k.as_str())) {
        problems.push(format!("{name}.{key} is not a known key"));
    }
}

/// What in a text gives away a person, a network or a machine: e-mail addresses, IP addresses and TwinCAT
/// AMS Net IDs, absolute paths of a user's or a project's folders, serial numbers. One line per finding.
pub fn personal_data(file: &str, text: &str) -> Vec<String> {
    let mut found = Vec::new();
    for (n, line) in text.lines().enumerate() {
        let at = |what: &str| format!("{file}, line {}: {what}", n + 1);
        let lower = line.to_lowercase();
        if line.split(|c: char| c.is_whitespace() || ",;\t\"'<>()".contains(c)).any(|w| w.contains('@') && w.split('@').nth(1).is_some_and(|d| d.contains('.'))) {
            found.push(at("an e-mail address"));
        }
        let dotted = line.split(|c: char| !(c.is_ascii_digit() || c == '.')).any(|w| {
            // A sentence's full stop is not part of the address.
            let parts: Vec<&str> = w.trim_matches('.').split('.').collect();
            (parts.len() == 4 || parts.len() == 6) && parts.iter().all(|p| !p.is_empty() && p.len() <= 3 && p.parse::<u16>().is_ok_and(|v| v < 256))
        });
        // IPv6: hex groups with a "::" or all eight of them (a time 12:30:00 has neither).
        let ipv6 = line.split(|c: char| c.is_whitespace() || ",;\"'()[]<>".contains(c)).any(|w| {
            let groups = w.split(':').filter(|g| !g.is_empty()).count();
            w.chars().all(|c| c.is_ascii_hexdigit() || c == ':') && groups >= 2 && (w.contains("::") || w.matches(':').count() == 7)
        });
        if dotted || ipv6 {
            found.push(at("an IP address or AMS Net ID"));
        }
        // A drive letter with either slash (not the scheme of a URL), a UNC share, or a user's home.
        let bytes = line.as_bytes();
        let drive_path = (0..bytes.len().saturating_sub(2)).any(|i| {
            bytes[i].is_ascii_alphabetic() && bytes[i + 1] == b':' && (bytes[i + 2] == b'\\' || bytes[i + 2] == b'/') && (i == 0 || !bytes[i - 1].is_ascii_alphanumeric())
        });
        if drive_path || line.contains("\\\\") || lower.contains("/home/") || lower.contains("/users/") || lower.contains("\\users\\") {
            found.push(at("an absolute path"));
        }
        if ["serial", "serien", "s/n", "s/no"].iter().any(|w| lower.contains(w)) {
            found.push(at("a serial number"));
        }
    }
    found
}

/// Everything that keeps the folder `dir` from being published; empty when it is fine.
pub fn check(dir: &Path) -> Vec<String> {
    let mut problems = Vec::new();
    let folder = dir.file_name().and_then(|n| n.to_str()).unwrap_or("").to_string();
    let read = |name: &str| std::fs::read(dir.join(name)).map(|b| decode(&b));
    let Ok(text) = read("dataset.toml") else { return vec!["dataset.toml is missing".into()] };
    if read("README.md").is_err() {
        problems.push("README.md is missing".into());
    }
    let table: Table = match text.parse() {
        Ok(t) => t,
        Err(e) => return vec![format!("dataset.toml: {e}")],
    };
    for key in table.keys().filter(|k| !["dataset", "machine", "drive", "motor", "recording"].contains(&k.as_str())) {
        problems.push(format!("{key} is not a known table"));
    }
    for name in ["dataset", "machine", "drive", "motor"] {
        match table.get(name).and_then(Value::as_table) {
            Some(t) => check_table(name, t, &mut problems),
            None => problems.push(format!("[{name}] is missing")),
        }
    }
    let get = |t: &str, k: &str| table.get(t).and_then(|v| v.get(k)).and_then(Value::as_str).unwrap_or("");
    if table.get("dataset").is_some() {
        if get("dataset", "id") != folder {
            problems.push(format!("dataset.id has to be the folder's name, {folder}"));
        }
        if !LICENSES.contains(&get("dataset", "license")) {
            problems.push(format!("dataset.license has to be one of {}", LICENSES.join(", ")));
        }
        let recorded = get("dataset", "recorded");
        let month = recorded.len() == 7 && recorded.as_bytes()[4] == b'-' && recorded[..4].parse::<u16>().is_ok() && recorded[5..].parse::<u8>().is_ok_and(|m| (1..=12).contains(&m));
        if !month {
            problems.push("dataset.recorded has to be year and month only, YYYY-MM".into());
        }
    }
    let recordings = table.get("recording").and_then(Value::as_array).cloned().unwrap_or_default();
    if recordings.is_empty() {
        problems.push("[[recording]] is missing".into());
    }
    let mut listed = vec!["dataset.toml".to_string(), "README.md".to_string()];
    for (i, r) in recordings.iter().enumerate() {
        let Some(r) = r.as_table() else { continue };
        let before = problems.len();
        check_table("recording", r, &mut problems);
        if problems.len() > before {
            continue;
        }
        let field = |k: &str| r.get(k).and_then(Value::as_str).unwrap_or("").to_string();
        let (file, map, test) = (field("file"), field("map"), field("test"));
        let name = format!("recording {} ({file})", i + 1);
        if !TESTS.contains(&test.as_str()) {
            problems.push(format!("{name}: test has to be one of {}", TESTS.join(", ")));
        }
        // Only files in the folder itself.
        let inside = |f: &str| !f.is_empty() && !f.contains(['/', '\\', ':']) && !f.starts_with('.');
        if !inside(&file) || (!map.is_empty() && !inside(&map)) {
            problems.push(format!("{name}: file and map have to be plain names of files in the folder"));
            continue;
        }
        listed.push(file.clone());
        listed.push(map.clone());
        let Ok(trace) = read(&file) else {
            problems.push(format!("{name}: the file is missing"));
            continue;
        };
        if test == "frequency-response" {
            match parse_sweep(&trace) {
                Ok(points) if points.len() >= 10 => {}
                Ok(_) => problems.push(format!("{name}: fewer than 10 points")),
                Err(e) => problems.push(format!("{name}: {} {}", e.key, e.detail)),
            }
            continue;
        }
        if map.is_empty() {
            problems.push(format!("{name}: a trace needs a map"));
            continue;
        }
        let mapping = match read(&map).map_err(|e| e.to_string()).and_then(|m| Mapping::parse(&m).map_err(|e| format!("{} {}", e.key, e.detail))) {
            Ok(m) => m,
            Err(e) => {
                problems.push(format!("{name}: map: {e}"));
                continue;
            }
        };
        match import(&trace, &mapping) {
            Ok(imported) => {
                let t = &imported.trace.time;
                let step = (t[t.len() - 1] - t[0]) / (t.len() - 1) as f64;
                let stated = r.get("sample_time_s").and_then(|v| v.as_float().or(v.as_integer().map(|i| i as f64))).unwrap_or(0.0);
                if (step - stated).abs() > 0.01 * stated {
                    problems.push(format!("{name}: sample_time_s is {stated}, the trace has {step:e} s"));
                }
                if t.len() < 100 {
                    problems.push(format!("{name}: fewer than 100 rows"));
                }
            }
            Err(e) => problems.push(format!("{name}: {} {}", e.key, e.detail)),
        }
    }
    // Every file is listed (nothing published by accident) and none gives anyone away.
    let entries = std::fs::read_dir(dir).map(|d| d.flatten().collect::<Vec<_>>()).unwrap_or_default();
    for entry in entries {
        let name = entry.file_name().to_string_lossy().into_owned();
        if !listed.contains(&name) {
            problems.push(format!("{name} is not part of any recording; remove it or list it"));
            continue;
        }
        if let Ok(bytes) = std::fs::read(entry.path()) {
            problems.extend(personal_data(&name, &decode(&bytes)));
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn personal_data_is_found() {
        let found = |t: &str| personal_data("f", t).len();
        assert_eq!(found("contact: jane.doe@example.com"), 1);
        assert_eq!(found("NetID\t192.168.178.20.1.1"), 1);
        assert_eq!(found("host 10.0.0.12"), 1);
        assert_eq!(found("at 10.1.2.3."), 1);
        assert_eq!(found("C:\\Projekte\\Machine.project"), 1);
        assert_eq!(found("/home/jane/trace.csv"), 1);
        assert_eq!(found("SerialNumber;ST12345"), 1);
        assert_eq!(found("D:/Kunden/Acme/Maschine.project"), 1);
        assert_eq!(found("\\\\fileserver\\projekte\\trace.csv"), 1);
        assert_eq!(found("Seriennummer;ST12345"), 1);
        assert_eq!(found("S/N: 21345"), 1);
        assert_eq!(found("gateway fe80::1c2:3ff:fe4d:5"), 1);
        // A time, a URL and numbers with decimal points are none of these.
        assert_eq!(found("15:56:24.376 12:30:00 https://example.org/data"), 0);
        assert_eq!(found("0.001,1.25,3.5,4.75\n1.2.3"), 0);
        assert_eq!(found("time [s],speed [1/min]"), 0);
    }
}
