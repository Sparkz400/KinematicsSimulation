//! The link's command line refuses what it cannot read. It used to fall back without a word: an unknown
//! machine became the first one, an unreadable port became 4840.

use std::process::{Command, Output};

fn link(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_sim-opcua")).args(args).output().unwrap()
}

#[test]
fn bad_arguments_end_with_exit_code_2_and_a_message() {
    for (args, named) in [
        (&["no-such-machine"][..], "no-such-machine"),
        (&["--port", "abc"], "--port abc"),
        (&["--port", "70000"], "--port 70000"),
        (&["--port"], "--port"),
        (&["--lang", "fr"], "--lang fr"),
        (&["--prot", "4840"], "--prot"),
        (&["--port", "4840", "stray"], "stray"),
        (&["--cycle-ms", "0"], "--cycle-ms 0"),
    ] {
        let (en, de) = (link(args), link(&[args, &["--lang", "de"]].concat()));
        assert_eq!(en.status.code(), Some(2), "{args:?}: {}", String::from_utf8_lossy(&en.stderr));
        assert!(String::from_utf8_lossy(&en.stderr).contains(named) && en.stdout.is_empty(), "{args:?}: {}", String::from_utf8_lossy(&en.stderr));
        // In German where the language itself was readable.
        assert_eq!(de.status.code(), Some(2), "{args:?}");
        if !args.contains(&"--lang") {
            assert_ne!(en.stderr, de.stderr, "{args:?}");
        }
    }
}
