//! The PLC link as a process: `sim-opcua [MACHINE] [--port N] [--lang en|de]`.
//! Serves the machine's default config on opc.tcp://127.0.0.1:PORT/ until stopped.

use sim_machines::{LANGUAGES, MACHINES, find, translations};

#[tokio::main]
async fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let option = |name: &str| args.windows(2).find(|w| w[0] == name).map(|w| w[1].as_str());
    let lang = option("--lang").filter(|l| LANGUAGES.contains(l)).unwrap_or("en");
    let entry = args.first().and_then(|id| find(id)).unwrap_or(MACHINES[0]);
    let mut tr = translations(lang, entry);
    sim_opcua::I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    let port = option("--port").and_then(|p| p.parse().ok()).unwrap_or(4840u16);

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
