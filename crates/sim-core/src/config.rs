//! Machine configuration: one TOML file per machine, with a format version and units in the field names.

use toml::{Table, Value};

pub const FORMAT_VERSION: i64 = 1;

/// The limit that the shared readers (`Config::positive`, `non_negative`, `f64`) refused a value for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bound {
    /// Above zero (`positive`).
    Positive,
    /// Zero or above (`non_negative`).
    NonNegative,
    /// Not a finite number of at most 1e15 in size (`number`).
    Number,
}

/// Carries a translation key and a detail (usually the config path), never display text.
#[derive(Debug, Clone)]
pub enum ConfigError {
    Parse(String),
    Version(String),
    Missing(String),
    Type(String),
    Invalid(String),
    /// `Invalid` at a path, with the limit it broke where a shared reader knows it: the same key and detail as
    /// `Invalid` (and equal to it), so that a front end that does not look at the limit shows what it always did.
    Bounded(Bound, String),
    /// A key that the machine's default config does not have.
    Unknown(String),
    /// A value refused for a reason with a message of its own: the translation key, and the value's name (a part name
    /// that the catalogue does not have, under the key for its kind; a pose the machine cannot reach, at its path).
    Part(&'static str, String),
}

impl ConfigError {
    pub fn key(&self) -> &'static str {
        match self {
            Self::Parse(_) => "error-config-parse",
            Self::Version(_) => "error-config-version",
            Self::Missing(_) => "error-config-missing",
            Self::Type(_) => "error-config-type",
            Self::Invalid(_) | Self::Bounded(..) => "error-config-invalid",
            Self::Unknown(_) => "error-config-unknown",
            Self::Part(key, _) => key,
        }
    }
    pub fn detail(&self) -> &str {
        match self {
            Self::Parse(d)
            | Self::Version(d)
            | Self::Missing(d)
            | Self::Type(d)
            | Self::Invalid(d)
            | Self::Bounded(_, d)
            | Self::Unknown(d)
            | Self::Part(_, d) => d,
        }
    }
}

/// Errors are equal when key and detail are: what every test and front end goes by, so `Bounded` equals `Invalid`.
impl PartialEq for ConfigError {
    fn eq(&self, other: &Self) -> bool {
        (self.key(), self.detail()) == (other.key(), other.detail())
    }
}

/// What `Config::hint` finds to say beside an error.
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigHint {
    /// The path of the config nearest to the misspelt one.
    Nearest(String),
    /// The value (in TOML syntax) at the path that was refused.
    Value(String),
    /// The value that was refused, and the limit it broke.
    Bounded(String, Bound),
}

/// The Levenshtein distance of two strings, in characters.
fn edit_distance(a: &str, b: &str) -> usize {
    let b: Vec<char> = b.chars().collect();
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.chars().enumerate() {
        let mut diagonal = std::mem::replace(&mut row[0], i + 1);
        for (j, cb) in b.iter().enumerate() {
            let above = row[j + 1];
            row[j + 1] = (diagonal + usize::from(ca != *cb)).min(above + 1).min(row[j] + 1);
            diagonal = above;
        }
    }
    row[b.len()]
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

    /// A number. NaN and infinity are valid TOML and a value of no quantity, so they never pass; nor does a number beyond
    /// 1e15 or, but for zero, below 1e-15 in size (`number`).
    pub fn f64(&self, path: &str) -> Result<f64> {
        number(as_f64(self.get(path)?).ok_or_else(|| ConfigError::Type(path.into()))?, path)
    }

    /// A value that may be zero but not negative (dampings, frictions, a payload).
    pub fn non_negative(&self, path: &str) -> Result<f64> {
        let v = self.f64(path)?;
        if v >= 0.0 { Ok(v) } else { Err(ConfigError::Bounded(Bound::NonNegative, path.into())) }
    }

    /// A value that must be strictly positive (masses, radii, stiffnesses, cycle times).
    pub fn positive(&self, path: &str) -> Result<f64> {
        let v = self.f64(path)?;
        if v > 0.0 && v.is_finite() { Ok(v) } else { Err(ConfigError::Bounded(Bound::Positive, path.into())) }
    }

    pub fn bool(&self, path: &str) -> Result<bool> {
        self.get(path)?.as_bool().ok_or_else(|| ConfigError::Type(path.into()))
    }

    pub fn f64s(&self, path: &str) -> Result<Vec<f64>> {
        let err = || ConfigError::Type(path.into());
        self.get(path)?.as_array().ok_or_else(err)?.iter().map(|v| number(as_f64(v).ok_or_else(err)?, path)).collect()
    }

    /// Refuses a key that neither `schema` (a machine's default config) nor `optional` has, so that a
    /// misspelt key in a config file cannot pass silently. `[bounds]` is left out: it has its own check, and
    /// so are the entries of arrays, which the code that reads them checks.
    pub fn check_keys(&self, schema: &Config, optional: &[&str]) -> Result<()> {
        fn walk(prefix: &str, table: &Table, schema: &Config, optional: &[&str]) -> Result<()> {
            for (key, value) in table {
                let path = if prefix.is_empty() { key.clone() } else { format!("{prefix}.{key}") };
                if path == "bounds" {
                    continue;
                }
                if schema.get(&path).is_err() && !optional.contains(&path.as_str()) {
                    return Err(ConfigError::Unknown(path));
                }
                if let Value::Table(t) = value {
                    walk(&path, t, schema, optional)?;
                }
            }
            Ok(())
        }
        walk("", &self.0, schema, optional)
    }

    pub fn array(&self, path: &str) -> Result<&Vec<Value>> {
        self.get(path)?.as_array().ok_or_else(|| ConfigError::Type(path.into()))
    }

    /// Overwrites an existing value. New keys and type changes are rejected, so a typo cannot pass silently.
    pub fn set(&mut self, path: &str, value: Value) -> Result<()> {
        // A version `parse` would refuse is refused here too: a config set to it could be written but not read again
        // (found by the property test of `set_text`, 15 November 2026).
        if path == "format_version" && value.as_integer() != Some(FORMAT_VERSION) {
            return Err(ConfigError::Version(format!("{:?}", value.as_integer())));
        }
        let value = of_type(self.get(path)?, value, path)?;
        self.insert(path, value)
    }

    /// `set`, and where the config has not got `path`, adds it where a file may have it (`check_keys` with the same
    /// `schema` and `optional` takes it): in the type `schema` has there, or as given for a key of `optional`. A path of
    /// neither is refused as by `set` (`Missing`), so a misspelt key still cannot pass silently.
    pub fn set_within(&mut self, path: &str, value: Value, schema: &Config, optional: &[&str]) -> Result<()> {
        if path == "format_version" || self.get(path).is_ok() {
            return self.set(path, value);
        }
        let value = match schema.get(path) {
            Ok(Value::Table(_)) => return Err(ConfigError::Type(path.into())),
            Ok(old) => of_type(old, value, path)?,
            Err(_) if optional.contains(&path) && !value.is_table() => value,
            Err(e) => return Err(e),
        };
        self.insert(path, value)
    }

    /// `value` at `path`, with the tables on the way made where they are missing.
    fn insert(&mut self, path: &str, value: Value) -> Result<()> {
        let (parents, leaf) = path.rsplit_once('.').unwrap_or(("", path));
        let mut table = &mut self.0;
        for part in parents.split('.').filter(|p| !p.is_empty()) {
            table = table
                .entry(part)
                .or_insert_with(|| Value::Table(Table::new()))
                .as_table_mut()
                .ok_or_else(|| ConfigError::Type(path.into()))?;
        }
        table.insert(leaf.into(), value);
        Ok(())
    }

    /// Applies one `path=value` assignment, with the value in TOML syntax.
    pub fn set_text(&mut self, assignment: &str) -> Result<()> {
        let (path, value) = assignment_of(assignment)?;
        self.set(path, value)
    }

    /// `set_text` as `set_within` adds a key.
    pub fn set_text_within(&mut self, assignment: &str, schema: &Config, optional: &[&str]) -> Result<()> {
        let (path, value) = assignment_of(assignment)?;
        self.set_within(path, value, schema, optional)
    }

    /// The config as TOML text, which `parse` reads back to the same config. Comments are not kept.
    pub fn to_text(&self) -> Result<String> {
        toml::to_string(&self.0).map_err(|e| ConfigError::Parse(e.to_string()))
    }

    /// Extra help for an error of this config, for a front end to append to its message (the error itself, its key and
    /// its detail stay as they are): for a path that is missing or unknown the nearest path the config has, where one
    /// is close (at most 2 edits, or a third of the path's length); for a value out of range or of the wrong type the
    /// value the path has.
    pub fn hint(&self, error: &ConfigError) -> Option<ConfigHint> {
        self.hint_within(error, &Config(Table::new()), &[])
    }

    /// `hint`, with the paths `set_within` may add (`schema`'s and `optional`) among those a misspelt one is near.
    pub fn hint_within(&self, error: &ConfigError, schema: &Config, optional: &[&str]) -> Option<ConfigHint> {
        // A float as Rust writes it: 1e300, where TOML's text spells out all 301 digits.
        let shown = |value: &Value| match value {
            Value::Float(f) if f.is_finite() => format!("{f:?}"),
            other => other.to_string(),
        };
        match error {
            ConfigError::Missing(path) | ConfigError::Unknown(path) => {
                let near = path.chars().count().div_ceil(3).max(2);
                let paths = self.leaves().into_iter().chain(schema.leaves()).map(|(p, _)| p);
                let paths = paths.chain(optional.iter().map(|p| p.to_string()));
                let (distance, nearest) = paths.map(|p| (edit_distance(path, &p), p)).min_by_key(|(d, _)| *d)?;
                (distance > 0 && distance <= near).then_some(ConfigHint::Nearest(nearest))
            }
            ConfigError::Bounded(bound, path) => {
                self.get(path).ok().map(|value| ConfigHint::Bounded(shown(value), *bound))
            }
            ConfigError::Invalid(path) | ConfigError::Type(path) => {
                self.get(path).ok().map(|value| ConfigHint::Value(shown(value)))
            }
            _ => None,
        }
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

/// The largest and the smallest non-zero size of a number a config may hold, in its SI unit. The shipped configs and the
/// catalogue lie between 1e-6 (an encoder step, m) and 2.1e11 (steel's Young's modulus, Pa); a value beyond is a slip
/// of the exponent, and 1e30 or 1e-30 in a stiffness, damping, inductance or ratio overflowed the integration to NaN.
const LARGEST: f64 = 1e15;
const SMALLEST: f64 = 1e-15;

/// `value` as a config's number at `path`, for a reader that takes it from TOML itself: refused out of range.
pub fn number(value: f64, path: &str) -> Result<f64> {
    if value.abs() < SMALLEST {
        // What is left of a sum in floating point (a pose of 5e-17 m) is zero, not a slip of the exponent; as zero, a
        // key that must be positive still refuses it.
        Ok(0.0)
    } else if value.abs() <= LARGEST {
        Ok(value)
    } else {
        Err(ConfigError::Bounded(Bound::Number, path.into()))
    }
}

/// The path and the value (TOML syntax) of one `path=value` assignment.
fn assignment_of(assignment: &str) -> Result<(&str, Value)> {
    let bad = || ConfigError::Parse(assignment.into());
    let (path, value) = assignment.split_once('=').ok_or_else(bad)?;
    let mut parsed: Table = format!("v = {value}").parse().map_err(|_| bad())?;
    // One assignment: a second line ("a = 1\nb = 2") parsed beside `v` and was dropped without a word.
    if parsed.len() != 1 {
        return Err(bad());
    }
    Ok((path.trim(), parsed.remove("v").ok_or_else(bad)?))
}

/// `new` in the type of `old`: an integer where a float was is the float, any other change of type is refused.
fn of_type(old: &Value, new: Value, path: &str) -> Result<Value> {
    match (old, new) {
        (Value::Float(_), Value::Integer(i)) => Ok(Value::Float(i as f64)),
        (old, new) if old.same_type(&new) => Ok(new),
        _ => Err(ConfigError::Type(path.into())),
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

    /// A typo of one letter finds the path it was meant for (also one missing from the config altogether, `Unknown`), a
    /// name far off finds none, a path of the config itself is no typo of itself, and a refused value is shown.
    #[test]
    fn hints_name_the_nearest_path_and_the_value() {
        let c = Config::parse(TEXT).unwrap();
        let near = |p: &str| c.hint(&ConfigError::Missing(p.into()));
        assert_eq!(near("a.mas_kg"), Some(ConfigHint::Nearest("a.mass_kg".into())));
        assert_eq!(c.hint(&ConfigError::Unknown("a.onn".into())), Some(ConfigHint::Nearest("a.on".into())));
        assert_eq!(near("something.entirely_else"), None);
        assert_eq!(near("a.mass_kg"), None);
        assert_eq!(c.hint(&ConfigError::Invalid("a.mass_kg".into())), Some(ConfigHint::Value("2.0".into())));
        assert_eq!(c.hint(&ConfigError::Invalid("a.nowhere".into())), None);
        assert_eq!(c.hint(&ConfigError::Parse("x".into())), None);
        assert_eq!(edit_distance("kitten", "sitting"), 3);
    }

    /// A value refused by a shared reader is shown with the limit it broke (`positive`: above zero, `non_negative`: not
    /// below zero, `f64`: size), under the same key and detail as any `Invalid`; a site that checks for itself
    /// (`Invalid`) is shown with its value only.
    #[test]
    fn hints_name_the_limit_a_shared_reader_refused() {
        let c = Config::parse("format_version = 1\nzero = 0.0\nneg = -5.5\nbig = 1e20\n").unwrap();
        let hint = |e: Result<f64>| c.hint(&e.unwrap_err());
        assert_eq!(hint(c.positive("neg")), Some(ConfigHint::Bounded("-5.5".into(), Bound::Positive)));
        assert_eq!(hint(c.positive("zero")), Some(ConfigHint::Bounded("0.0".into(), Bound::Positive)));
        assert_eq!(hint(c.non_negative("neg")), Some(ConfigHint::Bounded("-5.5".into(), Bound::NonNegative)));
        assert_eq!(hint(c.f64("big")), Some(ConfigHint::Bounded("1e20".into(), Bound::Number)));
        let refused = c.positive("neg").unwrap_err();
        assert_eq!((refused.key(), refused.detail()), ("error-config-invalid", "neg"));
        assert_eq!(refused, ConfigError::Invalid("neg".into()));
        assert_eq!(c.hint(&ConfigError::Invalid("neg".into())), Some(ConfigHint::Value("-5.5".into())));
    }

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
        let mut cfg = Config::parse("format_version = 1").unwrap();
        assert!(matches!(cfg.set_text("format_version = 2"), Err(ConfigError::Version(_))));
        assert!(cfg.set_text("format_version = 1").is_ok() && Config::parse(&cfg.to_text().unwrap()).is_ok());
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

    /// A key the config has not got is added where the schema has it (in the schema's type: an integer for a float is the
    /// float, a bool for it refused) or the optional list names it, with its tables; a misspelt one is still refused, and
    /// the hint finds the key it was meant for among those it could have added.
    #[test]
    fn set_within_adds_the_keys_a_file_may_have() {
        let schema = Config::parse("format_version = 1\n[a]\nmass_kg = 2.0\n[d.x]\nkp = 0.5\n").unwrap();
        let optional = ["r.altitude_m"];
        let mut c = Config::parse("format_version = 1\n[a]\nmass_kg = 2.0\n").unwrap();
        c.set_text_within("d.x.kp = 1", &schema, &optional).unwrap();
        assert_eq!(c.get("d.x.kp"), Ok(&Value::Float(1.0)));
        c.set_text_within("r.altitude_m = 2000", &schema, &optional).unwrap();
        assert_eq!(c.f64("r.altitude_m"), Ok(2000.0));
        assert_eq!(c.set_text_within("a.mass_kg = 3", &schema, &optional), Ok(()));
        assert_eq!(c.set_text_within("d.x.kp = true", &schema, &optional), Err(ConfigError::Type("d.x.kp".into())));
        assert_eq!(c.set_text_within("d.x = 1", &schema, &optional), Err(ConfigError::Type("d.x".into())));
        let misspelt = c.set_text_within("r.altitud_m = 2000", &schema, &optional).unwrap_err();
        assert_eq!(misspelt, ConfigError::Missing("r.altitud_m".into()));
        let fresh = Config::parse("format_version = 1\n").unwrap();
        assert_eq!(fresh.hint_within(&misspelt, &schema, &optional), Some(ConfigHint::Nearest("r.altitude_m".into())));
        assert_eq!(fresh.hint(&misspelt), None);
        assert_eq!(Config::parse(&c.to_text().unwrap()), Ok(c));
    }

    /// NaN and infinity are numbers to TOML and to no component: no read lets them through.
    #[test]
    fn numbers_that_are_not_finite_or_negative_are_refused_with_their_key() {
        let c = Config::parse("format_version = 1\na = nan\nb = -inf\nc = [1.0, nan]\nd = -1.0\ne = 0.0\n").unwrap();
        for path in ["a", "b"] {
            assert_eq!(c.f64(path), Err(ConfigError::Invalid(path.into())));
            assert_eq!(c.positive(path), Err(ConfigError::Invalid(path.into())));
            assert_eq!(c.non_negative(path), Err(ConfigError::Invalid(path.into())));
        }
        assert_eq!(c.f64s("c"), Err(ConfigError::Invalid("c".into())));
        assert_eq!((c.f64("d"), c.non_negative("d")), (Ok(-1.0), Err(ConfigError::Invalid("d".into()))));
        assert_eq!(c.non_negative("e"), Ok(0.0));
    }

    /// The default config is the schema: a key it does not have is refused by name, except under `[bounds]`
    /// and where it is named as optional.
    #[test]
    fn a_key_the_schema_does_not_have_is_refused() {
        let schema = Config::parse(TEXT).unwrap();
        let file = |text: &str| {
            Config::parse(&format!("format_version = 1\n{text}")).unwrap().check_keys(&schema, &["a.more_kg"])
        };
        assert_eq!(file("[a]\nmass_kg = 3.0\nmore_kg = 1.0\n[bounds]\nanything = [1, 2]\n"), Ok(()));
        assert_eq!(file("[a]\nmas_kg = 3.0\n"), Err(ConfigError::Unknown("a.mas_kg".into())));
        assert_eq!(file("[a.b]\nlist = [1]\nmore = 1\n"), Err(ConfigError::Unknown("a.b.more".into())));
        assert_eq!(file("[typo]\nx = 1\n"), Err(ConfigError::Unknown("typo".into())));
    }
}
