//! The PLC link as a process: `sim-opcua [MACHINE] [--port N] [--lang en|de]`.
//! Serves the machine's default config on opc.tcp://127.0.0.1:PORT/ until stopped.

use sim_core::MachineEntry;
use sim_machines::{LANGUAGES, MACHINES, find, translations};

/// What the command line asks for: machine, language and port, or the translation key and the argument of
/// what is wrong with it. Nothing is passed over: a server for another machine or on another port than the
/// one asked for looks like the right one until a PLC connects.
fn arguments(args: &[String]) -> Result<(&'static MachineEntry, &'static str, u16), (&'static str, String)> {
    let (mut entry, mut lang, mut port) = (MACHINES[0], LANGUAGES[0], 4840);
    let mut at = 0;
    while at < args.len() {
        let value = args.get(at + 1).map(String::as_str);
        let unreadable = || ("opcua-error-argument", format!("{} {}", args[at], value.unwrap_or("")).trim().to_string());
        match args[at].as_str() {
            "--lang" => lang = LANGUAGES.iter().copied().find(|known| Some(*known) == value).ok_or_else(unreadable)?,
            "--port" => port = value.and_then(|port| port.parse().ok()).ok_or_else(unreadable)?,
            // The machine comes first, if it is named at all.
            id if at == 0 && !id.starts_with("--") => {
                entry = find(id).ok_or(("opcua-error-machine", id.to_string()))?;
                at += 1;
                continue;
            }
            other => return Err(("opcua-error-argument", other.to_string())),
        }
        at += 2;
    }
    Ok((entry, lang, port))
}

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (entry, lang, port) = arguments(&args).unwrap_or_else(|(key, detail)| {
        // In the language asked for, if that much of the command line can be read.
        let lang = args.windows(2).find(|w| w[0] == "--lang").and_then(|w| LANGUAGES.iter().find(|l| **l == w[1])).unwrap_or(&LANGUAGES[0]);
        let tr = sim_opcua::I18N.iter().find(|(l, _)| l == lang).map_or_else(Default::default, |(_, text)| sim_core::Translations::parse(text));
        eprintln!("{}", tr.tr_arg(key, &detail));
        std::process::exit(2);
    });
    let mut tr = translations(lang, entry);
    sim_opcua::I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));

    let started = match tokio::net::TcpListener::bind(("127.0.0.1", port)).await {
        Ok(listener) => sim_opcua::serve(entry, &entry.config(), &tr, listener, std::path::Path::new("pki")).await,
        Err(e) => Err(sim_opcua::LinkError("opcua-error-start", e.to_string())),
    };
    match started {
        Ok(link) => {
            println!("{}", tr.tr_arg("opcua-listening", &format!("opc.tcp://127.0.0.1:{port}/")));
            link.handle.token().cancelled().await;
        }
        Err(sim_opcua::LinkError(key, detail)) => {
            eprintln!("{}", tr.tr_arg(key, &detail));
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parsed(args: &[&str]) -> Result<(&'static str, &'static str, u16), (&'static str, String)> {
        arguments(&args.iter().map(|a| a.to_string()).collect::<Vec<_>>()).map(|(entry, lang, port)| (entry.id, lang, port))
    }

    /// What is asked for is taken, in any order of the options, and the defaults fill in the rest.
    #[test]
    fn arguments_name_machine_language_and_port() {
        assert_eq!(parsed(&[]), Ok((MACHINES[0].id, "en", 4840)));
        for entry in MACHINES {
            assert_eq!(parsed(&[entry.id, "--port", "4841", "--lang", "de"]), Ok((entry.id, "de", 4841)));
            assert_eq!(parsed(&[entry.id, "--lang", "de", "--port", "4841"]), Ok((entry.id, "de", 4841)));
        }
        assert_eq!(parsed(&["--port", "5000"]), Ok((MACHINES[0].id, "en", 5000)));
    }

    /// An unknown machine used to become the first one and an unreadable port 4840, without a word.
    #[test]
    fn arguments_that_cannot_be_read_are_refused() {
        assert_eq!(parsed(&["no-such-machine"]), Err(("opcua-error-machine", "no-such-machine".into())));
        for (args, named) in [(&["--port", "abc"][..], "--port abc"), (&["--port", "70000"], "--port 70000"), (&["--port"], "--port"), (&["--lang", "fr"], "--lang fr")] {
            assert_eq!(parsed(args), Err(("opcua-error-argument", named.into())));
        }
        // A misspelt option, and a machine that does not come first.
        assert_eq!(parsed(&["--prot", "4840"]), Err(("opcua-error-argument", "--prot".into())));
        assert_eq!(parsed(&["--port", "4840", MACHINES[0].id]), Err(("opcua-error-argument", MACHINES[0].id.into())));
    }
}
