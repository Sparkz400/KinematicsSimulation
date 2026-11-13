//! The catalogue read from a directory (`--catalog DIR`) instead of the compiled-in copy. Its own test binary,
//! because the choice holds for the whole process.

use sim_components::catalog::{motor_mass, use_directory};
use std::path::{Path, PathBuf};

fn copy(from: &Path, to: &Path) {
    std::fs::create_dir_all(to).unwrap();
    for entry in std::fs::read_dir(from).unwrap().flatten() {
        let path = entry.path();
        if path.is_dir() {
            copy(&path, &to.join(entry.file_name()));
        } else {
            std::fs::copy(&path, to.join(entry.file_name())).unwrap();
        }
    }
}

#[test]
fn a_catalogue_directory_replaces_the_compiled_in_one_and_another_schema_version_is_refused() {
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog");
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("catalog-dir-test");
    let _ = std::fs::remove_dir_all(&dir);
    for sub in ["siemens", "megadyne", "festo", "bahr"] {
        copy(&source.join(sub), &dir.join(sub));
    }
    assert_eq!(motor_mass("1FK2205-4AF", "400v"), Some(5.2));

    // No catalog.toml, then another schema version: refused, and the compiled-in catalogue stays.
    assert_eq!(use_directory(&dir).unwrap_err().key(), "error-catalog-read");
    std::fs::write(dir.join("catalog.toml"), "schema_version = 2\n").unwrap();
    assert_eq!(use_directory(&dir).unwrap_err().key(), "error-catalog-version");
    assert_eq!(motor_mass("1FK2205-4AF", "400v"), Some(5.2));

    // A copy with one motor's mass changed: the simulator reads the copy.
    std::fs::copy(source.join("catalog.toml"), dir.join("catalog.toml")).unwrap();
    let motors = dir.join("siemens/motors_s210.csv");
    let text = std::fs::read_to_string(&motors).unwrap();
    let line = text.lines().find(|l| l.starts_with("1FK2205-4AF,")).unwrap();
    let changed = line.replace(",5.2,", ",9.9,");
    assert_ne!(line, changed, "the row has its mass of 5.2 kg");
    std::fs::write(&motors, text.replace(line, &changed)).unwrap();
    use_directory(&dir).unwrap();
    assert_eq!(motor_mass("1FK2205-4AF", "400v"), Some(9.9));
    assert!(use_directory(&dir).is_err(), "the catalogue is chosen once");
}
