//! Translation tables in the simple-message subset of the Fluent format: `key = text`, `#` comments,
//! `{ $arg }` as the one placeholder.

use std::collections::BTreeMap;

/// (language code, translation file contents) for the keys the core itself uses.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

#[derive(Debug, Clone, Default)]
pub struct Translations(BTreeMap<String, String>);

impl Translations {
    pub fn parse(ftl: &str) -> Self {
        let mut out = Self::default();
        out.add(ftl);
        out
    }

    pub fn add(&mut self, ftl: &str) {
        for line in ftl.lines() {
            let line = line.trim();
            if line.starts_with('#') {
                continue;
            }
            if let Some((key, text)) = line.split_once('=') {
                self.0.insert(key.trim().into(), text.trim().into());
            }
        }
    }

    pub fn has(&self, key: &str) -> bool {
        self.0.contains_key(key)
    }

    /// A missing key shows as the key itself, so a gap is visible instead of blank.
    pub fn tr<'a>(&'a self, key: &'a str) -> &'a str {
        self.0.get(key).map_or(key, String::as_str)
    }

    pub fn tr_arg(&self, key: &str, arg: &str) -> String {
        self.tr(key).replace("{ $arg }", arg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_messages_and_placeholders() {
        let t = Translations::parse("# comment\nhello = Hallo\nbad-path = Fehlt: { $arg }\n");
        assert_eq!(t.tr("hello"), "Hallo");
        assert_eq!(t.tr("missing"), "missing");
        assert_eq!(t.tr_arg("bad-path", "a.b"), "Fehlt: a.b");
        assert!(t.has("hello") && !t.has("comment"));
    }
}
