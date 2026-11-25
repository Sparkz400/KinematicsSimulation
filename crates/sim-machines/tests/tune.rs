//! `tune::velocity_loops`: the gains found are checked by an independent `velocity_loop_margins` call on the config
//! returned, never by the tuning's own figures. The single axis (one motor, the quickest machine to measure) carries
//! most of them; the notch needs a loop whose gain margin a resonance limits, the flying saw's (its hand-tuned notch
//! switched off).

use sim_components::analysis::{Cancelled, Progress, velocity_loop_margins, with_progress};
use sim_components::tune::{Targets, velocity_loops};
use sim_core::{Config, MachineEntry};
use sim_machines::MACHINES;
use std::sync::Arc;
use std::sync::atomic::Ordering::Relaxed;

const KP: &str = "drive.velocity_kp_nm_s_rad";

fn entry(id: &str) -> &'static MachineEntry {
    MACHINES.iter().copied().find(|e| e.id == id).unwrap()
}

fn with(cfg: &Config, edits: &[String]) -> Config {
    let mut cfg = cfg.clone();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    cfg
}

/// The least phase margin and gain margin of motor 0 over all poses, and whether it is stable at each: what `tune`
/// promises, measured by the same call `sim-cli margins` makes.
fn least(entry: &MachineEntry, cfg: &Config) -> (f64, f64, bool) {
    let runs = velocity_loop_margins(entry, cfg, &[]).unwrap();
    let phase = runs.iter().map(|(_, m)| m[0].phase_margin_deg).filter(|x| !x.is_nan()).fold(f64::INFINITY, f64::min);
    let gain = runs.iter().map(|(_, m)| m[0].gain_margin_db).fold(f64::INFINITY, f64::min);
    (phase, gain, runs.iter().all(|(_, m)| m[0].stable()))
}

/// From a quarter of the shipped gain the search raises it: the result meets 35 degrees and 6 dB at every pose in a
/// measurement of its own, is higher than the start, and lies within a factor of the shipped gain, which was held to the
/// same margins over the `[bounds]` as well (so the poses alone allow somewhat more): expected between 0.8 and 4 times it.
/// Fails if the search stops at the start, takes a gain that misses a target, or the returned config is not the one that
/// was verified.
#[test]
fn a_gain_divided_by_four_is_raised_to_one_that_meets_the_targets() {
    let entry = entry("single-axis");
    let shipped = entry.config().f64(KP).unwrap();
    let start = with(&entry.config(), &[format!("{KP} = {:?}", shipped / 4.0)]);
    let tuned = velocity_loops(entry, &start, &Targets::default()).unwrap();
    let after = tuned.motors[0].after.kp;
    assert!(tuned.met && tuned.motors[0].met, "{tuned:?}");
    assert!(after > shipped / 4.0 * 1.5, "{after} from {}", shipped / 4.0);
    assert!((0.8..=4.0).contains(&(after / shipped)), "{after} against the shipped {shipped}");
    assert_eq!(tuned.config.f64(KP).unwrap(), after);
    let (phase, gain, stable) = least(entry, &tuned.config);
    assert!(stable && phase >= 35.0 && gain >= 6.0, "{phase} degrees, {gain} dB");
    // The words that make the config say the same.
    assert!(tuned.settings().iter().any(|s| s.starts_with(&format!("{KP}="))), "{:?}", tuned.settings());
}

/// From a gain that is unstable the search comes down: the start misses (checked here), the result is stable and meets
/// the targets, and is lower than the start.
#[test]
fn an_unstable_gain_is_lowered_to_one_that_meets_the_targets() {
    let entry = entry("single-axis");
    let shipped = entry.config().f64(KP).unwrap();
    let start = with(&entry.config(), &[format!("{KP} = {:?}", shipped * 16.0)]);
    let (phase, gain, stable) = least(entry, &start);
    assert!(!(stable && phase >= 35.0 && gain >= 6.0), "the start must miss: {phase} degrees, {gain} dB");
    let tuned = velocity_loops(entry, &start, &Targets::default()).unwrap();
    assert!(tuned.met, "{tuned:?}");
    assert!(tuned.motors[0].after.kp < shipped * 16.0 / 2.0, "{}", tuned.motors[0].after.kp);
    // A lowered gain lowers the position gain with it (the cascade's margin stays).
    assert!(tuned.motors[0].after.kv < tuned.motors[0].before.kv, "{:?}", tuned.motors[0]);
    let (phase, gain, stable) = least(entry, &tuned.config);
    assert!(stable && phase >= 35.0 && gain >= 6.0, "{phase} degrees, {gain} dB");
}

/// Where the gain margin stops the gain at a resonance (the flying saw without its hand-tuned notch: 5.5 dB at Kp 0.25
/// over its bounds, the loop held by the gearbox's mode at 367 Hz) the notch is put at the peak, within 8 % of the
/// mode, and the gain is higher than without a notch (`notch: false`) by at least `NOTCH_MIN_GAIN`; the result with it
/// meets the targets in a measurement of its own. Expected from the hand tuning (0.25 against 0.2); the reviewer
/// adjusts the frequency band to the measured value.
#[test]
fn a_resonance_that_holds_the_gain_back_gets_a_notch_and_a_higher_gain() {
    let entry = entry("flying-saw");
    let start = with(&entry.config(), &["effects.notch_filter = false".into(), format!("{KP} = 0.2")]);
    let without = velocity_loops(entry, &start, &Targets { notch: false, ..Targets::default() }).unwrap();
    assert!(without.notch.is_none());
    let with_notch = velocity_loops(entry, &start, &Targets::default()).unwrap();
    let notch = with_notch.notch.expect("a notch at the resonance");
    assert!((notch.frequency_hz / 367.0 - 1.0).abs() < 0.08, "{notch:?}");
    assert!(with_notch.motors[0].after.kp >= 1.15 * without.motors[0].after.kp, "{with_notch:?} {without:?}");
    assert!(with_notch.met && with_notch.config.bool("effects.notch_filter").unwrap());
    let (phase, gain, stable) = least(entry, &with_notch.config);
    assert!(stable && phase >= 35.0 && gain >= 6.0, "{phase} degrees, {gain} dB");
}

/// Targets no gain can meet: the flag is not set, and the gains are the best found, not an error. (At the start pose
/// alone, the poses are not what is shown here.)
#[test]
fn targets_no_gain_meets_are_reported_not_met() {
    let entry = entry("single-axis");
    let cfg = with(&entry.config(), &["cycle.segment = []".into()]);
    let targets = Targets { phase_margin_deg: 179.0, gain_margin_db: 80.0, notch: true };
    let tuned = velocity_loops(entry, &cfg, &targets).unwrap();
    assert!(!tuned.met && !tuned.motors[0].met, "{tuned:?}");
}

/// A job cancelled before it starts ends with `Cancelled`, as the other analyses do.
#[test]
fn tuning_in_a_cancelled_job_ends_with_cancelled() {
    let entry = entry("single-axis");
    let cfg = entry.config();
    let cancelled = Arc::new(Progress::default());
    cancelled.cancel.store(true, Relaxed);
    let out = with_progress(&cancelled, || velocity_loops(entry, &cfg, &Targets::default()).map(|_| ()));
    assert_eq!(out.err(), Some(Cancelled));
}
