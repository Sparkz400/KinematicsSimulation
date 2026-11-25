//! The help text: one translation key per line (`cli-help-<command>-<n>`, see `i18n/en.ftl`), so that a line of the file
//! is a line of the text, and the commands' parts can be printed alone.

use sim_core::Translations;

/// Where a line is wrapped, in characters.
const WIDTH: usize = 100;
/// The column an option's description starts in.
const COLUMN: usize = 30;

/// The commands, in the order of the overview.
pub(super) const COMMANDS: [&str; 26] = [
    "list",
    "run",
    "analyze",
    "sweep",
    "optimise",
    "search",
    "transmission",
    "compare-kinematics",
    "verdicts",
    "sensitivity",
    "step",
    "bode",
    "tuning",
    "margins",
    "tune",
    "modes",
    "workspace",
    "replay",
    "fit",
    "identify",
    "check-dataset",
    "compare",
    "stop",
    "accuracy",
    "report",
    "fmu",
    "parameters",
];

/// The topics beside the commands: the options every command takes, the path formats and the units.
const TOPICS: [&str; 3] = ["options", "paths", "units"];

/// `text` in lines of at most `WIDTH` characters, the first behind `first`, the others behind blanks as wide.
fn wrap(first: &str, text: &str) -> Vec<String> {
    // What fits stays as written: a line may be aligned by hand.
    if first.chars().count() + text.chars().count() <= WIDTH {
        return vec![format!("{first}{text}")];
    }
    let rest = " ".repeat(first.chars().count());
    let mut lines: Vec<String> = Vec::new();
    for word in text.split_whitespace() {
        match lines.last_mut() {
            Some(line) if line.chars().count() + 1 + word.chars().count() <= WIDTH => {
                line.push(' ');
                line.push_str(word);
            }
            Some(_) => lines.push(format!("{rest}{word}")),
            None => lines.push(format!("{first}{word}")),
        }
    }
    lines
}

/// The lines `prefix-1`, `prefix-2`, ... of the translation. An option line (`--option ARG  what it does`) comes in two
/// columns, any other line as it is, indented.
fn lines(tr: &Translations, prefix: &str) -> Vec<String> {
    let texts = (1..).map(|n| format!("{prefix}-{n}")).take_while(|key| tr.has(key));
    let mut out = Vec::new();
    for key in texts {
        let text = tr.tr(&key);
        match text.split_once("  ").filter(|_| text.starts_with("--")) {
            Some((option, what)) if option.chars().count() < COLUMN - 2 => {
                out.extend(wrap(&format!("  {option:<width$}", width = COLUMN - 2), what));
            }
            Some((option, what)) => {
                out.push(format!("  {option}"));
                out.extend(wrap(&" ".repeat(COLUMN), what));
            }
            None => out.extend(wrap("  ", text)),
        }
    }
    out
}

/// The help of one command: its usage line, what it does and its options.
fn command(tr: &Translations, name: &str) -> Vec<String> {
    let mut out = lines(tr, &format!("cli-help-{name}"));
    if !out.is_empty() {
        // The usage line stands at the margin, what the command does under it.
        out[0] = out[0].trim_start().to_string();
        out.insert(1, format!("  {}", tr.tr(&format!("cli-help-brief-{name}"))));
    }
    out
}

/// Every command with a few words on what it does, and where to find more.
fn overview(tr: &Translations) -> Vec<String> {
    let mut out: Vec<String> = lines(tr, "cli-help-overview").iter().map(|l| l.trim_start().to_string()).collect();
    for name in COMMANDS {
        out.extend(wrap(&format!("  {name:<20}"), tr.tr(&format!("cli-help-brief-{name}"))));
    }
    out.push(String::new());
    out.extend(lines(tr, "cli-help-more").iter().map(|l| l.trim_start().to_string()));
    out
}

/// The help for `topic`: a command, one of `TOPICS`, or (none) the overview; `None` for a word that is none of these.
pub(super) fn text(tr: &Translations, topic: Option<&str>) -> Option<String> {
    let body = match topic {
        None => overview(tr),
        Some(name) if COMMANDS.contains(&name) => command(tr, name),
        Some(name) if TOPICS.contains(&name) => lines(tr, &format!("cli-help-{name}")),
        Some(_) => return None,
    };
    Some(body.join("\n"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_machines::{LANGUAGES, MACHINES};

    fn texts(lang: &str) -> Translations {
        super::super::texts(lang, MACHINES[0])
    }

    /// Every command and topic has a help in both languages, no line is wider than 100 characters, and the German has
    /// the lines of the English (a line of one without the other is a gap).
    #[test]
    fn every_command_has_help_in_both_languages_within_100_columns() {
        for lang in LANGUAGES {
            let tr = texts(lang);
            for topic in COMMANDS.iter().chain(&TOPICS) {
                let help = text(&tr, Some(topic)).unwrap_or_default();
                assert!(help.lines().count() > 1, "{lang} {topic}: {help}");
                assert!(!help.contains("cli-help"), "{lang} {topic}: a key without its text");
                assert!(help.lines().all(|l| l.chars().count() <= WIDTH), "{lang} {topic}: {help}");
            }
            let overview = text(&tr, None).unwrap();
            assert!(!overview.contains("cli-help"), "{lang}: {overview}");
            assert!(overview.lines().all(|l| l.chars().count() <= WIDTH), "{lang}: {overview}");
            assert!(COMMANDS.iter().all(|c| overview.contains(c)), "{lang}: every command is in the overview");
        }
        let (en, de) = (texts("en"), texts("de"));
        // Per key, so that wrapping does not matter: as many lines, and the same option first on each option line.
        let keys = |tr: &Translations, name: &str| -> Vec<String> {
            (1..)
                .map(|n| format!("cli-help-{name}-{n}"))
                .take_while(|key| tr.has(key))
                .map(|key| tr.tr(&key).split_whitespace().next().unwrap_or("").to_string())
                .map(|word| if word.starts_with("--") { word } else { String::new() })
                .collect()
        };
        for name in COMMANDS.iter().chain(&TOPICS) {
            assert_eq!(keys(&en, name), keys(&de, name), "{name}: the same lines and options in both languages");
        }
        assert!(text(&en, Some("no-such-command")).is_none());
    }

    /// The help of a command names every option its parser accepts, and the help of the options every option all commands
    /// take (a misspelt option is refused, so an option the help leaves out could not be found): the options of the
    /// parser and the help cannot drift apart.
    #[test]
    fn the_help_names_every_option_the_parser_takes() {
        let tr = texts("en");
        for name in COMMANDS {
            let help = text(&tr, Some(name)).unwrap();
            for option in super::super::command_options(Some(name)) {
                let named = help.split(|c: char| !(c.is_alphanumeric() || c == '-')).any(|word| word == *option);
                assert!(named, "{name}: {option} is not in\n{help}");
            }
        }
        let common = text(&tr, Some("options")).unwrap();
        for option in super::super::COMMON {
            assert!(common.contains(option), "{option} is not in\n{common}");
        }
    }
}
