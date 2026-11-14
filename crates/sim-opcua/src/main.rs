//! The PLC link as a process: `sim-opcua [MACHINE | --machine-file FILE] [--config FILE] [--set PATH=VALUE ...]
//! [--trajectory FILE] [--port N] [--lang en|de] [--cycle-ms N] [--host IP] [--pki DIR] [--trust-all] [--user NAME]
//! [--user-cert FILE ...] [--renew-certificate]`.
//! Serves the machine's config on opc.tcp://HOST:PORT/ (HOST 127.0.0.1 unless given) until stopped with Ctrl+C, then
//! prints the measured real-time pacing and the clients' read and write intervals. The config is the machine's default
//! or `--config FILE`, with every `--set path=value` on top, as sim-cli reads them; `--trajectory FILE` (time, then one
//! position per axis, a line per pose) runs at the start. `--cycle-ms` sets the simulation loop's tick (default 10 ms).
//! Security (docs/plc.md): the server's certificate and the trusted and rejected client certificates live in `--pki`
//! (default `pki` in the working directory); `--trust-all` accepts every client certificate; `--user NAME` requires
//! that user name, with the password from the environment variable `SIM_OPCUA_PASSWORD` (not on the command line,
//! where every process on the computer could read it); `--user-cert FILE` lets a client in that signs with that user
//! certificate's key; `--renew-certificate` replaces the server's certificate with a new one, valid for 365 days.

use sim_core::{Config, MachineEntry};
use sim_machines::{LANGUAGES, MACHINES, find, translations};
use std::net::IpAddr;
use std::path::PathBuf;
use std::time::Duration;

/// Days before the server's certificate expires from which every start warns.
const RENEW_DAYS: f64 = 30.0;

/// What the command line asks for.
struct Arguments {
    entry: &'static MachineEntry,
    cfg: Config,
    /// The poses at times of `--trajectory`, run at the start.
    trajectory: Option<Vec<(f64, Vec<f64>)>>,
    lang: &'static str,
    port: u16,
    /// The loop's cycle in ms.
    cycle_ms: u64,
    host: IpAddr,
    pki: PathBuf,
    trust_all: bool,
    /// The user name a client must give.
    user: Option<String>,
    user_certs: Vec<PathBuf>,
    renew: bool,
}

/// What the command line asks for, or the translation key and the argument of what is wrong with it. Nothing is
/// passed over: a server for another machine or on another port than the one asked for looks like the right one
/// until a PLC connects.
fn arguments(args: &[String]) -> Result<Arguments, (&'static str, String)> {
    let (mut entry, mut lang, mut port, mut cycle_ms, mut host) =
        (MACHINES[0], LANGUAGES[0], 4840, 10, IpAddr::from([127, 0, 0, 1]));
    let (mut pki, mut trust_all, mut user, mut user_certs, mut renew) =
        (PathBuf::from("pki"), false, None, Vec::new(), false);
    let (mut config, mut sets, mut trajectory) = (None, Vec::new(), None);
    let mut at = 0;
    while at < args.len() {
        let value = args.get(at + 1).map(String::as_str);
        let unreadable =
            || ("opcua-error-argument", format!("{} {}", args[at], value.unwrap_or("")).trim().to_string());
        match args[at].as_str() {
            "--lang" => lang = LANGUAGES.iter().copied().find(|known| Some(*known) == value).ok_or_else(unreadable)?,
            "--port" => port = value.and_then(|port| port.parse().ok()).ok_or_else(unreadable)?,
            "--host" => host = value.and_then(|ip| ip.parse().ok()).ok_or_else(unreadable)?,
            "--pki" => pki = value.filter(|dir| !dir.starts_with("--")).map(PathBuf::from).ok_or_else(unreadable)?,
            "--user" => user = Some(value.filter(|name| !name.starts_with("--")).ok_or_else(unreadable)?.to_string()),
            "--user-cert" => user_certs.push(PathBuf::from(file(value).ok_or_else(unreadable)?)),
            "--config" => config = Some(PathBuf::from(file(value).ok_or_else(unreadable)?)),
            "--set" => sets.push(value.filter(|a| a.contains('=')).ok_or_else(unreadable)?.to_string()),
            "--trajectory" => trajectory = Some(PathBuf::from(file(value).ok_or_else(unreadable)?)),
            // Switches without a value.
            "--trust-all" | "--renew-certificate" => {
                trust_all |= args[at] == "--trust-all";
                renew |= args[at] == "--renew-certificate";
                at += 1;
                continue;
            }
            "--cycle-ms" => {
                cycle_ms =
                    value.and_then(|ms| ms.parse().ok()).filter(|ms| (1..=1000).contains(ms)).ok_or_else(unreadable)?
            }
            // A machine defined in a file (docs/machine-files.md), in place of a registered one: not both.
            "--machine-file" => {
                let path = value.filter(|_| at == 0 || args[0].starts_with("--")).ok_or_else(unreadable)?;
                entry = sim_machines::load_file(std::path::Path::new(path))
                    .map_err(|e| ("opcua-error-machine-file", e.to_string()))?;
            }
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
    // After the loop: `--config` belongs to the machine, which may come after it.
    let (cfg, trajectory) = sim_opcua::configured(entry, config.as_deref(), &sets, trajectory.as_deref())
        .map_err(|sim_opcua::LinkError(key, detail)| (key, detail))?;
    Ok(Arguments { entry, cfg, trajectory, lang, port, cycle_ms, host, pki, trust_all, user, user_certs, renew })
}

/// An option's value naming a file: there, and not the next option.
fn file(value: Option<&str>) -> Option<&str> {
    value.filter(|path| !path.starts_with("--"))
}

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let all = |lang: &str, entry| {
        let mut tr = translations(lang, entry);
        sim_opcua::I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
        tr
    };
    let Arguments { entry, cfg, trajectory, lang, port, cycle_ms, host, pki, trust_all, user, user_certs, renew } =
        arguments(&args).unwrap_or_else(|(key, detail)| {
            // In the language asked for, if that much of the command line can be read; with the config's messages.
            let lang = args
                .windows(2)
                .find(|w| w[0] == "--lang")
                .and_then(|w| LANGUAGES.iter().find(|l| **l == w[1]))
                .unwrap_or(&LANGUAGES[0]);
            eprintln!("{}", all(lang, MACHINES[0]).tr_arg(key, &detail));
            std::process::exit(2);
        });
    let tr = all(lang, entry);
    let fail = |key: &str, detail: &str| -> ! {
        eprintln!("{}", tr.tr_arg(key, detail));
        std::process::exit(2);
    };

    let password = std::env::var("SIM_OPCUA_PASSWORD").ok().filter(|p| !p.is_empty());
    let user = match (user, password) {
        (Some(user), Some(password)) => Some((user, password)),
        (Some(user), None) => fail("opcua-error-password", &user),
        (None, _) => None,
    };
    let security = sim_opcua::Security { trust_all, user, user_certs };
    if renew && let Err(e) = sim_opcua::renew_certificate(&pki) {
        fail("opcua-error-renew", &e.to_string());
    }
    let started = match tokio::net::TcpListener::bind((host, port)).await {
        Ok(listener) => {
            sim_opcua::serve_secured(entry, &cfg, &tr, listener, &pki, Duration::from_millis(cycle_ms), &security).await
        }
        Err(e) => Err(sim_opcua::LinkError("opcua-error-start", e.to_string())),
    };
    match started {
        Ok(link) => {
            if let Some(points) = trajectory {
                link.run_trajectory(points);
            }
            // The certificate is there now: created on the first start, or by `--renew-certificate`.
            if let Some(days) = sim_opcua::certificate_days_left(&pki).filter(|days| *days < RENEW_DAYS) {
                eprintln!("{}", tr.tr_arg("opcua-certificate-expires", &format!("{days:.0}")));
            }
            println!(
                "{}",
                tr.tr_arg("opcua-listening", &format!("opc.tcp://{}/", std::net::SocketAddr::new(host, port)))
            );
            let folder = std::path::absolute(&pki).unwrap_or(pki);
            println!("{}", tr.tr_arg("opcua-pki", &folder.display().to_string()));
            if security.trust_all {
                println!("{}", tr.tr("opcua-trust-all"));
            }
            if let Some((user, _)) = &security.user {
                println!("{}", tr.tr_arg("opcua-user", user));
            }
            for cert in &security.user_certs {
                println!("{}", tr.tr_arg("opcua-user-cert", &cert.display().to_string()));
            }
            tokio::select! {
                _ = link.handle.token().cancelled() => {}
                _ = tokio::signal::ctrl_c() => {}
            }
            let p = link.pacing.lock().unwrap().clone();
            let ms = |s: f64| format!("{:.3}", s * 1e3);
            let line = format!(
                "{} · {} · {} · {} · {:.4}",
                ms(p.period_s()),
                ms(p.jitter_max_s),
                ms(p.jitter_rms_s()),
                p.overruns,
                p.realtime_factor()
            );
            println!("{}", tr.tr_arg("opcua-pacing", &line));
            let clients = link.clients.lock().unwrap().clone();
            let intervals = |i: &sim_opcua::Intervals| {
                format!("{} · {} · {} · {}", i.count, ms(i.mean_s()), ms(i.std_s()), ms(i.max_s))
            };
            println!("{}", tr.tr_arg("opcua-client-timing-writes", &intervals(&clients.writes)));
            println!("{}", tr.tr_arg("opcua-client-timing-reads", &intervals(&clients.reads)));
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

    fn parsed(args: &[&str]) -> Result<(&'static str, &'static str, u16, u64), (&'static str, String)> {
        arguments(&args.iter().map(|a| a.to_string()).collect::<Vec<_>>())
            .map(|a| (a.entry.id, a.lang, a.port, a.cycle_ms))
    }

    /// The server listens on 127.0.0.1 unless `--host` names another address of this computer.
    #[test]
    fn host_is_local_unless_named() {
        let host = |args: &[&str]| {
            arguments(&args.iter().map(|a| a.to_string()).collect::<Vec<_>>()).map(|a| a.host.to_string())
        };
        assert_eq!(host(&[]), Ok("127.0.0.1".into()));
        assert_eq!(host(&["--host", "192.168.0.10"]), Ok("192.168.0.10".into()));
        assert_eq!(host(&["--host", "plc"]), Err(("opcua-error-argument", "--host plc".into())));
    }

    /// Security: certificates in `pki` unless `--pki` names another folder, client certificates trusted only from that
    /// folder unless `--trust-all`, anonymous clients unless `--user` names the user a client must give.
    #[test]
    fn security_options_are_read() {
        let security = |args: &[&str]| {
            arguments(&args.iter().map(|a| a.to_string()).collect::<Vec<_>>()).map(|a| (a.pki, a.trust_all, a.user))
        };
        assert_eq!(security(&[]), Ok((PathBuf::from("pki"), false, None)));
        assert_eq!(
            security(&["--trust-all", "--pki", "certs", "--user", "plc", "--port", "4841"]),
            Ok((PathBuf::from("certs"), true, Some("plc".into())))
        );
        assert_eq!(
            security(&[MACHINES[0].id, "--port", "4841", "--trust-all"]),
            Ok((PathBuf::from("pki"), true, None))
        );
        assert_eq!(security(&["--user"]), Err(("opcua-error-argument", "--user".into())));
        assert_eq!(security(&["--user", "--trust-all"]), Err(("opcua-error-argument", "--user --trust-all".into())));
        assert_eq!(security(&["--pki"]), Err(("opcua-error-argument", "--pki".into())));
    }

    /// What is asked for is taken, in any order of the options, and the defaults fill in the rest.
    #[test]
    fn arguments_name_machine_language_port_and_cycle() {
        assert_eq!(parsed(&[]), Ok((MACHINES[0].id, "en", 4840, 10)));
        for entry in MACHINES {
            assert_eq!(parsed(&[entry.id, "--port", "4841", "--lang", "de"]), Ok((entry.id, "de", 4841, 10)));
            assert_eq!(
                parsed(&[entry.id, "--lang", "de", "--cycle-ms", "4", "--port", "4841"]),
                Ok((entry.id, "de", 4841, 4))
            );
        }
        assert_eq!(parsed(&["--port", "5000"]), Ok((MACHINES[0].id, "en", 5000, 10)));
    }

    /// An unknown machine used to become the first one and an unreadable port 4840, without a word.
    #[test]
    fn arguments_that_cannot_be_read_are_refused() {
        assert_eq!(parsed(&["no-such-machine"]), Err(("opcua-error-machine", "no-such-machine".into())));
        for (args, named) in [
            (&["--port", "abc"][..], "--port abc"),
            (&["--port", "70000"], "--port 70000"),
            (&["--port"], "--port"),
            (&["--lang", "fr"], "--lang fr"),
            (&["--cycle-ms", "0"], "--cycle-ms 0"),
            (&["--cycle-ms", "2.5"], "--cycle-ms 2.5"),
        ] {
            assert_eq!(parsed(args), Err(("opcua-error-argument", named.into())));
        }
        // A misspelt option, and a machine that does not come first.
        assert_eq!(parsed(&["--prot", "4840"]), Err(("opcua-error-argument", "--prot".into())));
        assert_eq!(parsed(&["--port", "4840", MACHINES[0].id]), Err(("opcua-error-argument", MACHINES[0].id.into())));
    }

    /// `--machine-file` names a machine defined in a file, in place of a registered one; a file that cannot be used
    /// is refused with its file, line and key, and so is a machine named twice.
    #[test]
    fn a_machine_file_takes_the_place_of_a_machine() {
        let file = concat!(env!("CARGO_MANIFEST_DIR"), "/../../machines/h-bot.toml");
        assert_eq!(parsed(&["--machine-file", file, "--port", "4842"]), Ok(("h-bot", "en", 4842, 10)));
        assert_eq!(parsed(&["--port", "4842", "--machine-file", file]), Ok(("h-bot", "en", 4842, 10)));
        assert_eq!(
            parsed(&[MACHINES[0].id, "--machine-file", file]),
            Err(("opcua-error-argument", format!("--machine-file {file}")))
        );
        let missing = parsed(&["--machine-file", "no-such-file.toml"]).unwrap_err();
        assert_eq!(missing.0, "opcua-error-machine-file");
        assert!(missing.1.starts_with("no-such-file.toml: error-config-parse"), "{}", missing.1);
    }

    /// `--config` and `--set` give the config served, read as sim-cli reads them (the file first, every `--set` on top,
    /// in order); `--trajectory` gives the poses to run; `--user-cert` and `--renew-certificate` are read.
    #[test]
    fn config_set_trajectory_and_certificates_are_read() {
        let read = |args: &[&str]| arguments(&args.iter().map(|a| a.to_string()).collect::<Vec<_>>());
        let dir = std::env::temp_dir().join(format!("sim-opcua-args-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        // The machine's config as a file of its own.
        let config = dir.join("config.toml");
        std::fs::write(&config, MACHINES[0].default_config).unwrap();
        let config = config.to_str().unwrap();
        let trajectory = dir.join("poses.csv");
        std::fs::write(
            &trajectory,
            "0, 0.1, 0.35
0.5, 0.2, 0.35
",
        )
        .unwrap();
        let trajectory = trajectory.to_str().unwrap();
        let a = read(&[
            MACHINES[0].id,
            "--config",
            config,
            "--set",
            "start.pose = [0.3, 0.3]",
            "--set",
            "start.pose = [0.2, 0.3]",
            "--trajectory",
            trajectory,
            "--user-cert",
            "plc.der",
            "--renew-certificate",
        ])
        .unwrap_or_else(|e| panic!("{e:?}"));
        assert_eq!(a.cfg.f64s("start.pose").unwrap(), [0.2, 0.3]);
        assert_eq!(a.trajectory, Some(vec![(0.0, vec![0.1, 0.35]), (0.5, vec![0.2, 0.35])]));
        assert_eq!((a.user_certs, a.renew), (vec![PathBuf::from("plc.der")], true));
        // Without them: the machine's default config, nothing to run, no renewal.
        let plain = read(&[]).unwrap_or_else(|e| panic!("{e:?}"));
        assert_eq!(plain.cfg.f64s("start.pose").unwrap(), MACHINES[0].config().f64s("start.pose").unwrap());
        assert!(plain.trajectory.is_none() && plain.user_certs.is_empty() && !plain.renew);
        // What cannot be used is refused, naming it.
        let refused = |args: &[&str]| read(args).err().unwrap();
        assert_eq!(refused(&["--set", "start.pose"]), ("opcua-error-argument", "--set start.pose".into()));
        assert_eq!(refused(&["--set", "start.pose = [0.2,"]).0, "error-config-parse");
        assert_eq!(refused(&["--config", "--port"]), ("opcua-error-argument", "--config --port".into()));
        assert_eq!(refused(&["--config", "no-such-config.toml"]).0, "opcua-error-file");
        assert_eq!(refused(&["--trajectory", "no-such-trajectory.csv"]).0, "opcua-error-file");
        std::fs::remove_dir_all(&dir).unwrap();
    }
}
