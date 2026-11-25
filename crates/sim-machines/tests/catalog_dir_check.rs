//! A catalogue directory with a table the readers would panic on is refused, naming the file and the column or the
//! line. Its own test binary: an accepted directory holds for the whole process, so the refusals come first, in one test.

use sim_components::catalog::use_directory;
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

fn message(dir: &Path) -> String {
    format!("{:?}", use_directory(dir).unwrap_err())
}

#[test]
fn a_table_with_a_missing_column_or_a_short_row_is_refused_and_an_unchanged_copy_is_accepted() {
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog");
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("catalog-dir-check");
    let _ = std::fs::remove_dir_all(&dir);
    for entry in std::fs::read_dir(&source).unwrap().flatten() {
        if entry.path().is_dir() && entry.file_name() != "tools" {
            copy(&entry.path(), &dir.join(entry.file_name()));
        }
    }
    std::fs::copy(source.join("catalog.toml"), dir.join("catalog.toml")).unwrap();
    let motors = dir.join("siemens/motors_s210.csv");
    let original = std::fs::read_to_string(&motors).unwrap();

    // (a) One column renamed: the file and the column are named.
    std::fs::write(&motors, original.replacen("part_number", "part_no", 1)).unwrap();
    let text = message(&dir);
    assert!(text.contains("siemens/motors_s210.csv") && text.contains("part_number"), "{text}");

    // (b) One row cut short: the file and its line are named (the first row after the notes and the header).
    let lines: Vec<&str> = original.lines().collect();
    let header = lines.iter().position(|l| !l.starts_with('#')).unwrap();
    let row = header + 1;
    let mut cut = lines.clone();
    let short: String = lines[row].split(',').take(2).collect::<Vec<_>>().join(",");
    cut[row] = &short;
    std::fs::write(&motors, cut.join("\n")).unwrap();
    let text = message(&dir);
    assert!(text.contains("siemens/motors_s210.csv") && text.contains(&format!("line {}", row + 1)), "{text}");

    // (c) Unchanged but for a blank line at the end of every table, as an editor leaves it: accepted, and a motor search
    // (whose readers panicked on the blank line's row of one empty field) runs through the catalogue.
    std::fs::write(&motors, &original).unwrap();
    fn blank_lines(dir: &Path) {
        for entry in std::fs::read_dir(dir).unwrap().flatten() {
            let path = entry.path();
            if path.is_dir() {
                blank_lines(&path);
            } else if path.extension().is_some_and(|e| e == "csv") {
                let text = std::fs::read_to_string(&path).unwrap();
                std::fs::write(
                    &path,
                    format!(
                        "{text}

"
                    ),
                )
                .unwrap();
            }
        }
    }
    blank_lines(&dir);
    use_directory(&dir).unwrap();
    let entry = &machine_single_axis::ENTRY;
    let found =
        sim_components::catalog::search_motors(entry, &entry.config(), "400v", &|a| a.starts_with("1FK2104"), None);
    assert!(!found.is_empty());
}
