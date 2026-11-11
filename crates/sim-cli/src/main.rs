//! Batch front end: runs the core as fast as the CPU allows and writes CSV.

use sim_machines::{MACHINES, translations};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let lang = args.iter().position(|a| a == "--lang").and_then(|i| args.get(i + 1)).map_or("en", String::as_str);
    // `list`: one line per registered machine, id and name.
    for entry in MACHINES {
        println!("{}\t{}", entry.id, translations(lang, entry).tr(entry.key));
    }
}
