//! Every `sim-cli` command of `examples/README.md` runs and exits cleanly with output. The commands are read from the
//! README itself (the lines starting with `sim-cli ` in its `bash` blocks), so the two cannot drift apart.

use std::path::Path;
use std::process::Command;

/// The words of a command line as a POSIX shell splits them: blanks separate, quotes group and are removed (no
/// escapes or variables: the README has none).
fn words(line: &str) -> Vec<String> {
    let (mut out, mut word, mut quote, mut started) = (Vec::new(), String::new(), None, false);
    for c in line.chars() {
        match (quote, c) {
            (Some(q), c) if c == q => quote = None,
            (Some(_), c) => word.push(c),
            (None, '\'' | '"') => (quote, started) = (Some(c), true),
            (None, c) if c.is_whitespace() => {
                if started {
                    out.push(std::mem::take(&mut word));
                }
                started = false;
            }
            (None, c) => {
                word.push(c);
                started = true;
            }
        }
    }
    assert!(quote.is_none(), "unclosed quote in: {line}");
    if started {
        out.push(word);
    }
    out
}

fn commands(readme: &str) -> Vec<String> {
    let mut in_bash = false;
    let mut out = Vec::new();
    for line in readme.lines() {
        match line.trim() {
            "```bash" => in_bash = true,
            "```" => in_bash = false,
            l if in_bash && l.starts_with("sim-cli ") => out.push(l.to_string()),
            _ => {}
        }
    }
    out
}

#[test]
fn words_split_as_a_shell_does() {
    assert_eq!(words(r#"a  'b c' --set 'k="v w"' x"y z""#), ["a", "b c", "--set", r#"k="v w""#, "xy z"]);
    assert_eq!(words("a ''"), ["a", ""]);
}

#[test]
fn every_command_of_the_examples_readme_runs() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let readme = std::fs::read_to_string(root.join("examples/README.md")).unwrap();
    let commands = commands(&readme);
    // One per example of the README; fewer means a block was lost or no longer parses.
    assert!(commands.len() >= 10, "only {} commands found in examples/README.md", commands.len());
    std::thread::scope(|scope| {
        for line in &commands {
            let root = &root;
            scope.spawn(move || {
                let args = words(line);
                let out =
                    Command::new(env!("CARGO_BIN_EXE_sim-cli")).args(&args[1..]).current_dir(root).output().unwrap();
                let stderr = String::from_utf8_lossy(&out.stderr);
                assert!(out.status.success(), "{line}\nexit {:?}\n{stderr}", out.status.code());
                assert!(!out.stdout.is_empty(), "{line}\nno output\n{stderr}");
            });
        }
    });
}
