//! The registration list: the one place outside a machine crate that names a machine.
//! App, CLI and PLC link pick every machine up from here.

use sim_core::{MachineEntry, Translations};

pub static MACHINES: &[&MachineEntry] = &[&machine_t_gantry::ENTRY, &machine_single_axis::ENTRY, &machine_delta_d3::ENTRY];

pub const LANGUAGES: &[&str] = &["en", "de"];

pub fn find(id: &str) -> Option<&'static MachineEntry> {
    MACHINES.iter().copied().find(|m| m.id == id)
}

/// Everything core, components and machines name, in one language. Front ends add their own labels.
/// `machine` selects whose axis and parameter names apply; machines may reuse keys such as `axis-x`.
pub fn translations(lang: &str, machine: &MachineEntry) -> Translations {
    let mut out = Translations::default();
    let sources = [sim_core::i18n::I18N, sim_components::I18N].into_iter();
    // The selected machine comes last, so its own wording wins where keys repeat.
    let machines = MACHINES.iter().filter(|m| m.id != machine.id).chain([&machine]).map(|m| m.i18n);
    for (_, text) in sources.chain(machines).flatten().filter(|(l, _)| *l == lang) {
        out.add(text);
    }
    out
}
