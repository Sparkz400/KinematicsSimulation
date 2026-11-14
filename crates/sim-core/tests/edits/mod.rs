//! Arbitrary edits of a valid file for the property tests of the readers of untrusted input. Other crates' tests take
//! this file by `#[path = "../../sim-core/tests/edits/mod.rs"] mod edits;`, so that every reader is attacked alike.

#![allow(dead_code)]

use proptest::prelude::*;
use std::path::{Path, PathBuf};

/// One change to a text, its indices taken modulo what the text has.
#[derive(Debug, Clone)]
pub enum Edit {
    Delete(usize),
    Duplicate(usize),
    Swap(usize, usize),
    /// The k-th number of the text replaced.
    Number(usize, String),
    Insert(usize, String),
    /// The text cut off at this character.
    Truncate(usize),
}

/// A number as a file might have it wrong: huge, negative, not a number, or not one at all.
pub fn number() -> impl Strategy<Value = String> {
    prop_oneof![
        prop::sample::select(vec![
            "1e308",
            "-1e308",
            "1e999",
            "nan",
            "inf",
            "-inf",
            "NaN",
            "0",
            "-0.0",
            "5e-324",
            "1e-300",
            "-1",
            "-1e9",
            "1e9",
            "1e18",
            "9223372036854775807",
            "-9223372036854775808",
            "18446744073709551616",
            "4294967296",
            "1.2.3",
            "--1",
            "1e",
            "",
            "0x1F",
            ".",
            "1,5",
            "\"1\"",
            "[]",
        ])
        .prop_map(String::from),
        any::<f64>().prop_map(|v| format!("{v:?}")),
        any::<i64>().prop_map(|v| v.to_string()),
    ]
}

pub fn edits() -> impl Strategy<Value = Vec<Edit>> {
    let edit = prop_oneof![
        1 => any::<usize>().prop_map(Edit::Delete),
        1 => any::<usize>().prop_map(Edit::Duplicate),
        1 => (any::<usize>(), any::<usize>()).prop_map(|(a, b)| Edit::Swap(a, b)),
        4 => (any::<usize>(), number()).prop_map(|(k, n)| Edit::Number(k, n)),
        1 => (any::<usize>(), ".{0,40}").prop_map(|(l, s)| Edit::Insert(l, s)),
        1 => any::<usize>().prop_map(Edit::Truncate),
    ];
    prop::collection::vec(edit, 1..4)
}

/// The spans of the numbers in a line: a digit with what may belong to a number around it.
fn numbers(line: &str) -> Vec<(usize, usize)> {
    let b = line.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        let starts = b[i].is_ascii_digit() && (i == 0 || !(b[i - 1].is_ascii_alphanumeric() || b[i - 1] == b'_'));
        if !starts {
            i += 1;
            continue;
        }
        let from = if i > 0 && b[i - 1] == b'-' { i - 1 } else { i };
        let mut to = i;
        while to < b.len() && (b[to].is_ascii_digit() || matches!(b[to], b'.' | b'e' | b'E' | b'_' | b'+' | b'-')) {
            to += 1;
        }
        out.push((from, to));
        i = to;
    }
    out
}

pub fn apply(text: &str, edits: &[Edit]) -> String {
    let mut lines: Vec<String> = text.lines().map(String::from).collect();
    for edit in edits {
        let n = lines.len();
        if n == 0 {
            break;
        }
        match edit {
            Edit::Delete(i) => {
                lines.remove(i % n);
            }
            Edit::Duplicate(i) => lines.insert(i % n, lines[i % n].clone()),
            Edit::Swap(a, b) => lines.swap(a % n, b % n),
            Edit::Number(k, new) => {
                let all: Vec<(usize, (usize, usize))> =
                    lines.iter().enumerate().flat_map(|(l, s)| numbers(s).into_iter().map(move |r| (l, r))).collect();
                if let Some(&(l, (a, b))) = all.get(k % all.len().max(1)) {
                    lines[l].replace_range(a..b, new);
                }
            }
            Edit::Insert(i, s) => lines.insert(i % (n + 1), s.clone()),
            Edit::Truncate(i) => {
                let joined = lines.join("\n");
                let mut cut = i % (joined.len() + 1);
                while !joined.is_char_boundary(cut) {
                    cut -= 1;
                }
                lines = joined[..cut].lines().map(String::from).collect();
            }
        }
    }
    lines.join("\n")
}

/// The repository's root.
pub fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// The texts of the files in `dir` (from the root) whose name ends in `suffix`, with their names.
pub fn files(dir: &str, suffix: &str) -> Vec<(String, String)> {
    let mut out: Vec<(String, String)> = std::fs::read_dir(root().join(dir))
        .unwrap_or_else(|e| panic!("{dir}: {e}"))
        .map(|e| e.unwrap().path())
        .filter(|p| p.to_string_lossy().ends_with(suffix))
        .map(|p| (p.file_name().unwrap().to_string_lossy().into_owned(), std::fs::read_to_string(&p).unwrap()))
        .collect();
    out.sort();
    assert!(!out.is_empty(), "no {suffix} files in {dir}");
    out
}

/// A seed of `seeds` with edits applied: the strategy every reader's property test draws its input from.
pub fn edited(seeds: Vec<(String, String)>) -> impl Strategy<Value = (String, String)> {
    (prop::sample::select(seeds), edits()).prop_map(|((name, text), e)| {
        let text = apply(&text, &e);
        (name, text)
    })
}

#[test]
fn edits_change_what_they_say() {
    let text = "a = 1.5\nb = -2\nc = x";
    assert_eq!(apply(text, &[Edit::Number(1, "nan".into())]), "a = 1.5\nb = nan\nc = x");
    assert_eq!(apply(text, &[Edit::Delete(4)]), "a = 1.5\nc = x");
    assert_eq!(apply(text, &[Edit::Swap(0, 2)]), "c = x\nb = -2\na = 1.5");
    assert_eq!(apply(text, &[Edit::Truncate(5)]), "a = 1");
}
