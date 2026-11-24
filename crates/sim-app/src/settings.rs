//! What the window remembers between starts: the language, the theme, the unit switches, the speed, and the config
//! or machine file last opened with the machine picked. A small TOML file in the user's config folder, or where
//! `SIM_APP_SETTINGS` points. Nothing here reads the environment or the disk of its own accord, so that the tests
//! and the developer hooks can keep clear of the user's file (`path`).

use crate::model::Message;
use std::path::{Path, PathBuf};

/// The file's layout; a file of another one is ignored, not guessed at.
const VERSION: i64 = 1;

/// The values kept; `None` where the file has none (a first start), so that the default shows.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Settings {
    pub lang: Option<String>,
    pub light: Option<bool>,
    pub rpm: Option<bool>,
    pub mm: Option<bool>,
    pub speed: Option<f64>,
    /// The config or machine file last opened or saved, and the id of the machine that was shown.
    pub config_file: Option<String>,
    pub machine: Option<String>,
}

impl Settings {
    /// The settings in `text`; a text that is no TOML, or of another version of the file, is refused.
    pub fn parse(text: &str) -> Result<Self, Message> {
        let table = text.parse::<toml::Table>().map_err(|e| ("ui-settings-ignored", e.message().to_string()))?;
        if table.get("version").and_then(toml::Value::as_integer) != Some(VERSION) {
            return Err(("ui-settings-outdated", String::new()));
        }
        let text = |key: &str| table.get(key).and_then(toml::Value::as_str).map(str::to_string);
        let flag = |key: &str| table.get(key).and_then(toml::Value::as_bool);
        Ok(Self {
            lang: text("language"),
            light: flag("light"),
            rpm: flag("rpm"),
            mm: flag("mm"),
            speed: table.get("speed").and_then(toml::Value::as_float).filter(|s| s.is_finite() && *s > 0.0),
            config_file: text("config_file").filter(|f| !f.is_empty()),
            machine: text("machine"),
        })
    }

    pub fn to_text(&self) -> String {
        let mut table = toml::Table::new();
        table.insert("version".into(), VERSION.into());
        let strings = [("language", &self.lang), ("config_file", &self.config_file), ("machine", &self.machine)];
        for (key, value) in strings {
            if let Some(value) = value {
                table.insert(key.into(), value.clone().into());
            }
        }
        for (key, value) in [("light", self.light), ("rpm", self.rpm), ("mm", self.mm)] {
            if let Some(value) = value {
                table.insert(key.into(), value.into());
            }
        }
        if let Some(speed) = self.speed {
            table.insert("speed".into(), speed.into());
        }
        table.to_string()
    }
}

/// The file the settings are kept in, from the environment `env` (a lookup by name): `SIM_APP_SETTINGS` where set;
/// else none while the screenshot hook runs or in a test, which must not read or write the user's file; else the app's
/// folder in the user's config folder (`APPDATA` on Windows, `XDG_CONFIG_HOME` or `~/.config` elsewhere,
/// `~/Library/Application Support` on macOS). `None` where the environment names no such folder.
pub fn path(env: impl Fn(&str) -> Option<String>) -> Option<PathBuf> {
    if let Some(file) = env("SIM_APP_SETTINGS").filter(|f| !f.is_empty()) {
        return Some(file.into());
    }
    if cfg!(test) || env("SIM_APP_SCREENSHOT").is_some() {
        return None;
    }
    let home = || env("HOME").filter(|h| !h.is_empty()).map(PathBuf::from);
    let folder = if cfg!(windows) {
        env("APPDATA").filter(|a| !a.is_empty()).map(PathBuf::from)
    } else if cfg!(target_os = "macos") {
        home().map(|h| h.join("Library").join("Application Support"))
    } else {
        env("XDG_CONFIG_HOME")
            .filter(|x| !x.is_empty())
            .map(PathBuf::from)
            .or_else(|| home().map(|h| h.join(".config")))
    };
    folder.map(|f| f.join("KinematicsSimulation").join("settings.toml"))
}

/// The settings stored in `path`: none where there is no file yet; where it cannot be read, is no TOML or is of
/// another version, none either, and the message that says so.
pub fn load(path: &Path) -> (Settings, Option<Message>) {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return (Settings::default(), None),
        Err(e) => return (Settings::default(), Some(("ui-settings-ignored", format!("{}: {e}", path.display())))),
    };
    match Settings::parse(&text) {
        Ok(settings) => (settings, None),
        Err((key, detail)) => {
            let detail =
                if detail.is_empty() { path.display().to_string() } else { format!("{}: {detail}", path.display()) };
            (Settings::default(), Some((key, detail)))
        }
    }
}

pub fn save(path: &Path, settings: &Settings) -> Result<(), Message> {
    let written = path
        .parent()
        .filter(|folder| !folder.as_os_str().is_empty())
        .map_or(Ok(()), std::fs::create_dir_all)
        .and_then(|()| std::fs::write(path, settings.to_text()));
    written.map_err(|e| ("ui-settings-unsaved", format!("{}: {e}", path.display())))
}

/// The language to start in: the environment's `SIM_APP_LANG` first, then the stored one, then the system's, then
/// the first of `languages`; one of them that the app does not have is passed over.
pub fn language(
    languages: &'static [&'static str],
    asked: Option<&str>,
    stored: Option<&str>,
    system: Option<&str>,
) -> &'static str {
    [asked, stored, system]
        .into_iter()
        .flatten()
        .find_map(|wanted| languages.iter().copied().find(|l| *l == wanted))
        .unwrap_or(languages[0])
}

/// The system's language code from the locale variables (`LC_ALL`, `LC_MESSAGES`, `LANG`; "de_DE.UTF-8" is "de"),
/// where there are any. Windows has none of them: the app stays English there.
pub fn system_language(env: impl Fn(&str) -> Option<String>) -> Option<String> {
    ["LC_ALL", "LC_MESSAGES", "LANG"].into_iter().find_map(|name| {
        let locale = env(name).filter(|l| !l.is_empty() && l != "C" && l != "POSIX")?;
        Some(locale.split(['_', '.', '@']).next()?.to_ascii_lowercase())
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp(name: &str) -> PathBuf {
        let folder = std::env::temp_dir().join(format!("sim-app-settings-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&folder);
        folder.join("settings.toml")
    }

    fn stored() -> Settings {
        Settings {
            lang: Some("de".into()),
            light: Some(true),
            rpm: Some(true),
            mm: Some(false),
            speed: Some(0.25),
            config_file: Some("my machine.toml".into()),
            machine: Some("some-machine".into()),
        }
    }

    /// What is stored comes back, from a folder that did not exist; the language and the theme with it.
    #[test]
    fn what_is_saved_is_loaded_again() {
        let file = temp("roundtrip");
        save(&file, &stored()).unwrap();
        assert_eq!(load(&file), (stored(), None));
        assert_eq!(load(&temp("none")), (Settings::default(), None), "no file yet is no message");
    }

    /// The environment's language beats the stored one, which beats the system's; a language the app lacks is passed
    /// over; with none of them, English.
    #[test]
    fn the_language_comes_from_the_environment_then_the_file_then_the_system() {
        let langs = &["en", "de"];
        assert_eq!(language(langs, Some("en"), Some("de"), Some("de")), "en");
        assert_eq!(language(langs, None, Some("de"), None), "de");
        assert_eq!(language(langs, None, None, Some("de")), "de");
        assert_eq!(language(langs, Some("xx"), Some("yy"), Some("fr")), "en");
        let env = |name: &str| (name == "LANG").then(|| "de_DE.UTF-8".to_string());
        assert_eq!(system_language(env).as_deref(), Some("de"));
        assert_eq!(system_language(|_| Some("C".into())), None);
        assert_eq!(system_language(|_| None), None);
    }

    /// A file that is no TOML, or is of another version, is ignored with a message, and so is one that cannot be read
    /// as a file (a folder).
    #[test]
    fn a_broken_or_outdated_file_is_ignored_with_a_message() {
        let file = temp("broken");
        std::fs::create_dir_all(file.parent().unwrap()).unwrap();
        std::fs::write(&file, "language = [oops").unwrap();
        let (settings, message) = load(&file);
        assert_eq!((settings, message.map(|m| m.0)), (Settings::default(), Some("ui-settings-ignored")));
        std::fs::write(&file, "version = 99\nlanguage = \"de\"\n").unwrap();
        let (settings, message) = load(&file);
        assert_eq!((settings, message.map(|m| m.0)), (Settings::default(), Some("ui-settings-outdated")));
        std::fs::write(&file, "language = \"de\"\n").unwrap();
        assert_eq!(load(&file).1.map(|m| m.0), Some("ui-settings-outdated"), "a file without a version");
        assert_eq!(load(file.parent().unwrap()).1.map(|m| m.0), Some("ui-settings-ignored"));
    }

    /// Tests and the screenshot hook have no file of their own unless one is named; the user's is never theirs.
    #[test]
    fn the_file_is_where_the_environment_says_and_never_the_users_in_a_test() {
        let with = |pairs: &'static [(&str, &str)]| {
            move |name: &str| pairs.iter().find(|(k, _)| *k == name).map(|(_, v)| v.to_string())
        };
        assert_eq!(path(with(&[("SIM_APP_SETTINGS", "x.toml")])), Some(PathBuf::from("x.toml")));
        assert_eq!(path(with(&[("APPDATA", "a"), ("HOME", "h"), ("XDG_CONFIG_HOME", "x")])), None, "under test");
    }
}
