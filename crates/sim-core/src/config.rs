//! Machine configuration: one TOML file per machine, with a format version and units in the field names.

use toml::{Table, Value};

pub const FORMAT_VERSION: i64 = 1;

/// Carries a translation key and a detail (usually the config path), never display text.
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigError {
    Parse(String),
    Version(String),
    Missing(String),
    Type(String),
    Invalid(String),
}

impl ConfigError {
    pub fn key(&self) -> &'static str {
        match self {
            Self::Parse(_) => "error-config-parse",
            Self::Version(_) => "error-config-version",
            Self::Missing(_) => "error-config-missing",
            Self::Type(_) => "error-config-type",
            Self::Invalid(_) => "error-config-invalid",
        }
    }
    pub fn detail(&self) -> &str {
        match self {
            Self::Parse(d) | Self::Version(d) | Self::Missing(d) | Self::Type(d) | Self::Invalid(d) => d,
        }
    }
}

type Result<T> = std::result::Result<T, ConfigError>;

#[derive(Debug, Clone, PartialEq)]
pub struct Config(Table);

impl Config {
    pub fn parse(text: &str) -> Result<Self> {
        let table: Table = text.parse().map_err(|e: toml::de::Error| ConfigError::Parse(e.to_string()))?;
        match table.get("format_version").and_then(Value::as_integer) {
            Some(FORMAT_VERSION) => Ok(Self(table)),
            other => Err(ConfigError::Version(format!("{other:?}"))),
        }
    }

    pub fn get(&self, path: &str) -> Result<&Value> {
        let mut parts = path.split('.');
        let mut value = self.0.get(parts.next().unwrap_or(""));
        for part in parts {
            value = value.and_then(|v| v.get(part));
        }
        value.ok_or_else(|| ConfigError::Missing(path.into()))
    }

    pub fn f64(&self, path: &str) -> Result<f64> {
        as_f64(self.get(path)?).ok_or_else(|| ConfigError::Type(path.into()))
    }

    /// A value that must be strictly positive (masses, radii, stiffnesses, cycle times).
    pub fn positive(&self, path: &str) -> Result<f64> {
        let v = self.f64(path)?;
        if v > 0.0 && v.is_finite() { Ok(v) } else { Err(ConfigError::Invalid(path.into())) }
    }

    pub fn bool(&self, path: &str) -> Result<bool> {
        self.get(path)?.as_bool().ok_or_else(|| ConfigError::Type(path.into()))
    }

    pub fn f64s(&self, path: &str) -> Result<Vec<f64>> {
        let err = || ConfigError::Type(path.into());
        self.get(path)?.as_array().ok_or_else(err)?.iter().map(|v| as_f64(v).ok_or_else(err)).collect()
    }

    pub fn array(&self, path: &str) -> Result<&Vec<Value>> {
        self.get(path)?.as_array().ok_or_else(|| ConfigError::Type(path.into()))
    }

    /// Overwrites an existing value. New keys and type changes are rejected, so a typo cannot pass silently.
    pub fn set(&mut self, path: &str, value: Value) -> Result<()> {
        let old = self.get(path)?;
        let value = match (old, value) {
            (Value::Float(_), Value::Integer(i)) => Value::Float(i as f64),
            (old, new) if old.same_type(&new) => new,
            _ => return Err(ConfigError::Type(path.into())),
        };
        let (parents, leaf) = path.rsplit_once('.').unwrap_or(("", path));
        let mut table = &mut self.0;
        for part in parents.split('.').filter(|p| !p.is_empty()) {
            table = table.get_mut(part).and_then(Value::as_table_mut).ok_or_else(|| ConfigError::Missing(path.into()))?;
        }
        table.insert(leaf.into(), value);
        Ok(())
    }

    /// Applies one `path=value` assignment, with the value in TOML syntax.
    pub fn set_text(&mut self, assignment: &str) -> Result<()> {
        let bad = || ConfigError::Parse(assignment.into());
        let (path, value) = assignment.split_once('=').ok_or_else(bad)?;
        let parsed: Table = format!("v = {value}").parse().map_err(|_| bad())?;
        self.set(path.trim(), parsed["v"].clone())
    }

    /// Every scalar value with its dotted path, for the parameter panel and the translation check.
    pub fn leaves(&self) -> Vec<(String, Value)> {
        fn walk(prefix: &str, table: &Table, out: &mut Vec<(String, Value)>) {
            for (key, value) in table {
                let path = if prefix.is_empty() { key.clone() } else { format!("{prefix}.{key}") };
                match value {
                    Value::Table(t) => walk(&path, t, out),
                    Value::Array(_) => {}
                    v => out.push((path, v.clone())),
                }
            }
        }
        let mut out = Vec::new();
        walk("", &self.0, &mut out);
        out
    }
}

fn as_f64(value: &Value) -> Option<f64> {
    match value {
        Value::Float(f) => Some(*f),
        Value::Integer(i) => Some(*i as f64),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TEXT: &str = "format_version = 1\n[a]\nmass_kg = 2.0\non = true\n[a.b]\nlist = [1, 2.5]\n";

    #[test]
    fn reads_values_by_path() {
        let c = Config::parse(TEXT).unwrap();
        assert_eq!(c.f64("a.mass_kg").unwrap(), 2.0);
        assert!(c.bool("a.on").unwrap());
        assert_eq!(c.f64s("a.b.list").unwrap(), vec![1.0, 2.5]);
        assert_eq!(c.f64("a.nope"), Err(ConfigError::Missing("a.nope".into())));
        assert_eq!(c.f64("a.on"), Err(ConfigError::Type("a.on".into())));
    }

    #[test]
    fn rejects_wrong_format_version() {
        assert!(matches!(Config::parse("format_version = 2"), Err(ConfigError::Version(_))));
        assert!(matches!(Config::parse("x = 1"), Err(ConfigError::Version(_))));
    }

    #[test]
    fn set_keeps_types_and_rejects_new_keys() {
        let mut c = Config::parse(TEXT).unwrap();
        c.set_text("a.mass_kg = 3").unwrap();
        assert_eq!(c.f64("a.mass_kg").unwrap(), 3.0);
        c.set_text("a.on=false").unwrap();
        assert!(!c.bool("a.on").unwrap());
        assert!(c.set_text("a.typo = 1").is_err());
        assert!(c.set_text("a.on = 1").is_err());
        assert_eq!(c.leaves().len(), 3);
    }
}
