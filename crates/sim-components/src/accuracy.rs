//! Static positioning accuracy and repeatability (`study`), judged along ISO 230-2 (a numerically controlled axis) and
//! ISO 9283 (the pose of a robot's tool): the machine runs to a line of targets along each tool axis, approaching each
//! target several times from below and from above, and the tool's real pose at rest is compared with the target.
//!
//! What puts the tool off its target is what the simulation already has: backlash, gearbox twist, belt stretch under
//! the load it holds, guide and frame compliance, the encoder's step, sensor noise, and friction where the axis comes to
//! rest. Two more effects move the axis's real position against what its motor encoder reads (`AxisErrors`):
//! effects.thermal_growth, the screw or belt grown with its temperature, and effects.lead_error, a lead (pitch) error
//! table. Both are a map from the position the controller holds to the real one, applied to the tool pose the
//! simulation reports; they are not fed back into the dynamics (a change of lead of a few 1e-5 changes the inertia the
//! motor sees by as little), and the run's axis signals do not carry them.

use crate::analysis::Verdict;
use crate::control::Command;
use crate::sim::Sim;
use sim_core::{Config, ConfigError, MachineEntry};

/// Linear expansion of steel (a ball screw), per K: 11.5e-6 for a hardened bearing steel (EN 1.3505 / 100Cr6, DIN EN
/// ISO 683-17; ball screw makers give 11 to 12e-6).
pub const STEEL_EXPANSION_PER_K: f64 = 11.5e-6;
/// Linear expansion of a toothed belt along its length, per K: [estimate] that of its steel tension cords, which carry
/// its tension and fix its pitch (the polyurethane body is some thousand times less stiff); no belt maker's datasheet
/// found gives one (16 November 2026). A glass fibre cord would be about 5e-6, an aramid one about -2e-6.
pub const BELT_EXPANSION_PER_K: f64 = 11.5e-6;

/// How the test is run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Settings {
    /// Targets per axis, evenly over the middle 80 % of its stroke.
    pub points: usize,
    /// Approaches of each target from each side (ISO 230-2 asks for 5).
    pub repeats: usize,
    /// How long the axis rests at a target before it is measured, s; the pose is averaged over the last tenth.
    pub dwell_s: f64,
}

impl Default for Settings {
    fn default() -> Self {
        Self { points: 5, repeats: 5, dwell_s: 0.3 }
    }
}

/// The map from the position an axis's motor encoder holds to its real one, for one tool axis (see the module).
#[derive(Debug, Clone, PartialEq)]
pub struct AxisErrors {
    pub axis: usize,
    /// Linear expansion of the screw or belt, per K; 0 with effects.thermal_growth off.
    pub expansion: f64,
    /// Where the axis does not grow from (a screw's fixed bearing, a belt axis's home), in the axis's unit.
    pub fixed_end: f64,
    /// A temperature rise of the screw or belt the config gives, K, and the heated part (`Sim::heated`, by index) whose
    /// rise it takes on top.
    pub rise: f64,
    pub part: Option<usize>,
    /// The lead error table, (position, error) by rising position, in the axis's unit; empty with effects.lead_error
    /// off.
    pub table: Vec<(f64, f64)>,
}

impl AxisErrors {
    /// The real position less the one the encoder holds, at that position `x` with the heated part at `part_rise` (K).
    /// The error of the table is linear between its rows and that of the nearest row beyond them.
    pub fn offset(&self, x: f64, part_rise: f64) -> f64 {
        let growth = self.expansion * (self.rise + part_rise) * (x - self.fixed_end);
        let lead = match self.table.iter().position(|r| r.0 > x) {
            None => self.table.last().map_or(0.0, |r| r.1),
            Some(0) => self.table[0].1,
            Some(i) => {
                let ((x0, e0), (x1, e1)) = (self.table[i - 1], self.table[i]);
                e0 + (e1 - e0) * (x - x0) / (x1 - x0)
            }
        };
        growth + lead
    }
}

/// The optional switch `effects.<name>`: off where the config does not have it.
fn switch(cfg: &Config, name: &str) -> Result<bool, ConfigError> {
    let path = format!("effects.{name}");
    if cfg.get(&path).is_ok() { cfg.bool(&path) } else { Ok(false) }
}

/// The axes of `[[accuracy.axes]]` that grow or have a lead error, for `sim` (its axes and heated parts), with the
/// effects that are off left out: none at all with both off, or with effects.load_encoder, whose scale reads the
/// carriage itself (its own expansion is not modelled). Each entry has `axis` (a tool axis's id) and may have
/// `drive` ("screw", the default, or "belt": which of `accuracy.screw_expansion_per_k` and
/// `accuracy.belt_expansion_per_k` it takes, else `STEEL_EXPANSION_PER_K` and `BELT_EXPANSION_PER_K`), `fixed_end`
/// (0), `rise_k` (0), `heated_part` (the id of one of the machine's `thermal_parts`) and `lead_error` (rows
/// [position, error], positions rising).
pub fn axis_errors(cfg: &Config, sim: &Sim) -> Result<Vec<AxisErrors>, ConfigError> {
    let (growth, lead) = (switch(cfg, "thermal_growth")?, switch(cfg, "lead_error")?);
    if !(growth || lead) || switch(cfg, "load_encoder")? {
        return Ok(Vec::new());
    }
    let Ok(entries) = cfg.array("accuracy.axes") else { return Ok(Vec::new()) };
    let coefficient = |key: &str, default: f64| if cfg.get(key).is_ok() { cfg.f64(key) } else { Ok(default) };
    let (screw, belt) = (
        coefficient("accuracy.screw_expansion_per_k", STEEL_EXPANSION_PER_K)?,
        coefficient("accuracy.belt_expansion_per_k", BELT_EXPANSION_PER_K)?,
    );
    let bad = |what: &str| ConfigError::Invalid(format!("accuracy.axes.{what}"));
    let number = |v: &toml::Value| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).filter(|x| x.is_finite());
    let axes = sim.machine.axes();
    let mut out = Vec::new();
    for entry in entries {
        let table = entry.as_table().ok_or_else(|| bad("axis"))?;
        if let Some(key) = table
            .keys()
            .find(|k| !["axis", "drive", "fixed_end", "rise_k", "heated_part", "lead_error"].contains(&k.as_str()))
        {
            return Err(ConfigError::Unknown(format!("accuracy.axes.{key}")));
        }
        let id = table.get("axis").and_then(toml::Value::as_str).ok_or_else(|| bad("axis"))?;
        let axis = axes.iter().position(|a| a.id == id).ok_or_else(|| bad("axis"))?;
        let get = |key: &str| table.get(key).map_or(Some(0.0), number).ok_or_else(|| bad(key));
        let expansion = match table.get("drive").map(|d| d.as_str()) {
            None | Some(Some("screw")) => screw,
            Some(Some("belt")) => belt,
            _ => return Err(bad("drive")),
        };
        let part = match table.get("heated_part") {
            None => None,
            Some(p) => {
                let id = p.as_str().ok_or_else(|| bad("heated_part"))?;
                Some(sim.heated.iter().position(|h| h.part.id == id).ok_or_else(|| bad("heated_part"))?)
            }
        };
        let mut rows = Vec::new();
        for row in table.get("lead_error").map_or(Ok(&Vec::new()), |v| v.as_array().ok_or_else(|| bad("lead_error")))? {
            match row.as_array().map(|r| r.iter().map(number).collect::<Option<Vec<f64>>>()) {
                Some(Some(r)) if r.len() == 2 && rows.last().is_none_or(|l: &(f64, f64)| r[0] > l.0) => {
                    rows.push((r[0], r[1]))
                }
                _ => return Err(bad("lead_error")),
            }
        }
        out.push(AxisErrors {
            axis,
            expansion: if growth { expansion } else { 0.0 },
            fixed_end: get("fixed_end")?,
            rise: get("rise_k")?,
            part: if growth { part } else { None },
            table: if lead { rows } else { Vec::new() },
        });
    }
    Ok(out)
}

/// ISO 230-2 on one tool axis, in its unit: the targets and, per target, the mean deviation (real less target) and its
/// standard deviation over the approaches from below (↑) and from above (↓).
#[derive(Debug, Clone, PartialEq)]
pub struct AxisResult {
    pub axis: usize,
    pub targets: Vec<f64>,
    pub mean_up: Vec<f64>,
    pub mean_down: Vec<f64>,
    pub std_up: Vec<f64>,
    pub std_down: Vec<f64>,
    /// B, the largest reversal error |x̄↑ - x̄↓| (backlash and what friction leaves where the axis settles).
    pub reversal: f64,
    /// R, the bidirectional repeatability: the largest of max(2s↑ + 2s↓ + |B_j|, 4s↑, 4s↓).
    pub repeatability: f64,
    /// E, the bidirectional systematic positional deviation: the range of all x̄↑ and x̄↓.
    pub systematic: f64,
    /// M, the range of the mean bidirectional positional deviation (x̄↑ + x̄↓) / 2.
    pub mean_range: f64,
    /// A, the bidirectional accuracy: max(x̄ + 2s) - min(x̄ - 2s) over both directions.
    pub accuracy: f64,
    /// The largest |x̄↑| or |x̄↓|.
    pub mean_deviation: f64,
}

/// The ISO 230-2 figures from the deviations `up[j][i]`, `down[j][i]` (target j, approach i).
pub fn iso_230_2(axis: usize, targets: Vec<f64>, up: &[Vec<f64>], down: &[Vec<f64>]) -> AxisResult {
    let mean = |v: &Vec<f64>| v.iter().sum::<f64>() / v.len() as f64;
    let std = |v: &Vec<f64>| {
        let m = mean(v);
        if v.len() < 2 { 0.0 } else { (v.iter().map(|x| (x - m).powi(2)).sum::<f64>() / (v.len() - 1) as f64).sqrt() }
    };
    let (mean_up, mean_down): (Vec<f64>, Vec<f64>) = (up.iter().map(mean).collect(), down.iter().map(mean).collect());
    let (std_up, std_down): (Vec<f64>, Vec<f64>) = (up.iter().map(std).collect(), down.iter().map(std).collect());
    let max = |it: &mut dyn Iterator<Item = f64>| it.fold(f64::NEG_INFINITY, f64::max);
    let min = |it: &mut dyn Iterator<Item = f64>| it.fold(f64::INFINITY, f64::min);
    let n = targets.len();
    let both = || mean_up.iter().chain(&mean_down).copied();
    let reversal = max(&mut (0..n).map(|j| (mean_up[j] - mean_down[j]).abs()));
    let repeatability = max(&mut (0..n).map(|j| {
        let r = 2.0 * std_up[j] + 2.0 * std_down[j] + (mean_up[j] - mean_down[j]).abs();
        r.max(4.0 * std_up[j]).max(4.0 * std_down[j])
    }));
    let bidirectional: Vec<f64> = (0..n).map(|j| 0.5 * (mean_up[j] + mean_down[j])).collect();
    let high = (0..n).flat_map(|j| [mean_up[j] + 2.0 * std_up[j], mean_down[j] + 2.0 * std_down[j]]);
    let low = (0..n).flat_map(|j| [mean_up[j] - 2.0 * std_up[j], mean_down[j] - 2.0 * std_down[j]]);
    AxisResult {
        axis,
        reversal,
        repeatability,
        systematic: max(&mut both()) - min(&mut both()),
        mean_range: max(&mut bidirectional.iter().copied()) - min(&mut bidirectional.iter().copied()),
        accuracy: max(&mut high.into_iter()) - min(&mut low.into_iter()),
        mean_deviation: max(&mut both().map(f64::abs)),
        targets,
        mean_up,
        mean_down,
        std_up,
        std_down,
    }
}

/// ISO 9283 at the tool, over the axes in metres (all of them on a machine without one), from the attained poses of
/// each target and direction: the largest pose accuracy AP (the distance of their barycentre from the target), the
/// largest pose repeatability RP (l̄ + 3 S_l, l the distance from the barycentre) and the largest distance between the
/// barycentres of the two directions (vAP, the multi-directional pose accuracy variation).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ToolResult {
    pub accuracy: f64,
    pub repeatability: f64,
    pub direction_variation: f64,
}

/// One target's attained deviation vectors (over the axes of `ToolResult`) from below and from above.
fn iso_9283(up: &[Vec<f64>], down: &[Vec<f64>], into: &mut ToolResult) {
    let barycentre = |v: &[Vec<f64>]| -> Vec<f64> {
        (0..v[0].len()).map(|k| v.iter().map(|p| p[k]).sum::<f64>() / v.len() as f64).collect()
    };
    let distance = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| (x - y).powi(2)).sum::<f64>().sqrt();
    let (bu, bd) = (barycentre(up), barycentre(down));
    for (poses, b) in [(up, &bu), (down, &bd)] {
        into.accuracy = into.accuracy.max(b.iter().map(|x| x * x).sum::<f64>().sqrt());
        let l: Vec<f64> = poses.iter().map(|p| distance(p, b)).collect();
        let mean = l.iter().sum::<f64>() / l.len() as f64;
        let s = if l.len() < 2 {
            0.0
        } else {
            (l.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (l.len() - 1) as f64).sqrt()
        };
        into.repeatability = into.repeatability.max(mean + 3.0 * s);
    }
    into.direction_variation = into.direction_variation.max(distance(&bu, &bd));
}

/// What `study` found, and the verdicts: `positioning-accuracy`, the largest A of the axes in metres (all axes on a
/// machine without one) against `ratings.max_positioning_error_m`, and `positioning-repeatability`, their largest R
/// against `ratings.max_repeatability_m`, each where the config has the rating.
#[derive(Debug, Clone, PartialEq)]
pub struct Study {
    pub axes: Vec<AxisResult>,
    pub tool: ToolResult,
    pub verdicts: Vec<Verdict>,
}

/// The longest a move may take before the test gives up on it, s.
const LONGEST_MOVE_S: f64 = 20.0;

/// Moves the tool to `pose`, lets it rest `dwell_s` and returns its real pose, averaged over the last tenth of the
/// rest, with the encoder-to-real map of `errors` applied.
fn attain(sim: &mut Sim, pose: &[f64], dwell_s: f64, errors: &[AxisErrors]) -> Result<Vec<f64>, ConfigError> {
    sim.command(Command::MoveTo(pose.to_vec())).map_err(|e| ConfigError::Invalid(e.key.into()))?;
    let start = sim.time();
    while sim.control.busy() && sim.time() - start < LONGEST_MOVE_S {
        sim.step();
    }
    let steps = (dwell_s / sim.dt).round().max(1.0) as u64;
    let averaged = (steps / 10).max(1);
    let n = pose.len();
    let (mut sum, mut at, mut rate) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
    for i in 0..steps {
        sim.step();
        if i >= steps - averaged {
            sim.machine.tool_state(&sim.state, &mut at, &mut rate);
            for e in errors {
                let rise = e.part.map_or(0.0, |p| sim.heated[p].rise);
                at[e.axis] += e.offset(at[e.axis], rise);
            }
            sum.iter_mut().zip(&at).for_each(|(s, x)| *s += x);
        }
    }
    Ok(sum.into_iter().map(|s| s / averaged as f64).collect())
}

/// The test: for each tool axis with a stroke, `settings.points` targets evenly over the middle 80 % of it (those the
/// machine can reach, the other axes at the start pose), run `settings.repeats` times forth and back as ISO 230-2's
/// linear cycle (each target approached from below on the way up, from above on the way down, with an overrun of 5 %
/// of the stroke beyond the end targets), in one simulation from the start pose, so that the parts warm as they run.
pub fn study(entry: &MachineEntry, cfg: &Config, settings: Settings) -> Result<Study, ConfigError> {
    if settings.points < 2 || settings.repeats < 1 || !(settings.dwell_s > 0.0 && settings.dwell_s.is_finite()) {
        return Err(ConfigError::Invalid("accuracy settings".into()));
    }
    let mut sim = Sim::new(entry, cfg)?;
    let errors = axis_errors(cfg, &sim)?;
    let axes = sim.machine.axes().to_vec();
    let start = cfg.f64s("start.pose")?;
    let metres: Vec<bool> = if axes.iter().any(|a| a.unit == "m") {
        axes.iter().map(|a| a.unit == "m").collect()
    } else {
        vec![true; axes.len()]
    };
    let on_metres = |dev: &[f64]| dev.iter().zip(&metres).filter(|(_, m)| **m).map(|(d, _)| *d).collect::<Vec<_>>();
    let (mut results, mut tool) = (Vec::new(), ToolResult::default());
    for (k, a) in axes.iter().enumerate() {
        let stroke = a.max - a.min;
        if stroke <= 0.0 {
            continue;
        }
        let at = |x: f64| {
            let mut p = start.clone();
            p[k] = x;
            p
        };
        let overrun = 0.05 * stroke;
        let reach = |x: f64| sim.machine.unreachable(&at(x), 0.0).is_none();
        let targets: Vec<f64> = (0..settings.points)
            .map(|j| a.min + stroke * (0.1 + 0.8 * j as f64 / (settings.points - 1) as f64))
            .filter(|&x| reach(x) && reach(x - overrun) && reach(x + overrun))
            .collect();
        if targets.len() < 2 {
            continue;
        }
        let m = targets.len();
        let (mut up, mut down) = (vec![Vec::new(); m], vec![Vec::new(); m]);
        let (mut up_poses, mut down_poses) = (vec![Vec::new(); m], vec![Vec::new(); m]);
        for _ in 0..settings.repeats {
            attain(&mut sim, &at(targets[0] - overrun), settings.dwell_s, &errors)?;
            for j in 0..m {
                let p = at(targets[j]);
                let got = attain(&mut sim, &p, settings.dwell_s, &errors)?;
                up[j].push(got[k] - targets[j]);
                up_poses[j].push(on_metres(&got.iter().zip(&p).map(|(g, t)| g - t).collect::<Vec<_>>()));
            }
            attain(&mut sim, &at(targets[m - 1] + overrun), settings.dwell_s, &errors)?;
            for j in (0..m).rev() {
                let p = at(targets[j]);
                let got = attain(&mut sim, &p, settings.dwell_s, &errors)?;
                down[j].push(got[k] - targets[j]);
                down_poses[j].push(on_metres(&got.iter().zip(&p).map(|(g, t)| g - t).collect::<Vec<_>>()));
            }
        }
        for j in 0..m {
            iso_9283(&up_poses[j], &down_poses[j], &mut tool);
        }
        results.push(iso_230_2(k, targets, &up, &down));
        // Back to the start pose, so that the next axis's line runs through it as the config sets it up.
        attain(&mut sim, &start, settings.dwell_s, &errors)?;
    }
    let judged: Vec<&AxisResult> = results.iter().filter(|r| metres[r.axis]).collect();
    let mut verdicts = Vec::new();
    for (check, key, value) in [
        (
            "positioning-accuracy",
            "ratings.max_positioning_error_m",
            judged.iter().map(|r| r.accuracy).fold(0.0, f64::max),
        ),
        (
            "positioning-repeatability",
            "ratings.max_repeatability_m",
            judged.iter().map(|r| r.repeatability).fold(0.0, f64::max),
        ),
    ] {
        if cfg.get(key).is_ok() {
            let limit = cfg.f64(key)?;
            verdicts.push(Verdict { check, motor: None, value, limit, ok: value <= limit });
        }
    }
    Ok(Study { axes: results, tool, verdicts })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn errors() -> AxisErrors {
        AxisErrors { axis: 0, expansion: 0.0, fixed_end: 0.0, rise: 0.0, part: None, table: Vec::new() }
    }

    /// A ball screw at 5 K above its surroundings, 0.4 m from its fixed bearing: 11.5e-6 / K * 5 K * 0.4 m = 23 um,
    /// half from the config's own rise (2 K) and the rest from the heated part (3 K).
    #[test]
    fn a_warm_screw_grows_from_its_fixed_end() {
        let e = AxisErrors { expansion: STEEL_EXPANSION_PER_K, fixed_end: 0.05, rise: 2.0, ..errors() };
        assert!((e.offset(0.45, 3.0) - 23.0e-6).abs() < 1e-12);
        // At the fixed bearing nothing moves; on its other side the growth turns.
        assert_eq!(e.offset(0.05, 3.0), 0.0);
        assert!((e.offset(0.0, 3.0) + 11.5e-6 * 5.0 * 0.05).abs() < 1e-15);
    }

    /// A pitch table of three rows: at 0.15 m, halfway from (0.1 m, +4 um) to (0.2 m, -2 um), +1 um; beyond the table
    /// the nearest row's error.
    #[test]
    fn the_lead_error_is_interpolated_between_its_rows() {
        let e = AxisErrors { table: vec![(0.1, 4e-6), (0.2, -2e-6), (0.4, 6e-6)], ..errors() };
        assert!((e.offset(0.15, 0.0) - 1e-6).abs() < 1e-15);
        assert!((e.offset(0.3, 0.0) - 2e-6).abs() < 1e-15);
        assert_eq!(e.offset(0.0, 0.0), 4e-6);
        assert_eq!(e.offset(0.5, 0.0), 6e-6);
    }

    /// Two targets, two approaches each way, by hand: target 0 up {+3, +5} um, down {-3, -5} um: x̄↑ = 4, x̄↓ = -4,
    /// s = sqrt(2) um each; B = 8 um; R = 2 s + 2 s + B = 4 sqrt(2) + 8 = 13.657 um. Target 1 up {+1, +1}, down {-1,
    /// -1}: s = 0, B = 2. E = 4 - (-4) = 8 um; M = 0; A = (4 + 2 sqrt(2)) - (-4 - 2 sqrt(2)) = 8 + 4 sqrt(2) um.
    #[test]
    fn iso_230_2_figures_match_a_hand_calculation() {
        let r = iso_230_2(
            0,
            vec![0.1, 0.2],
            &[vec![3e-6, 5e-6], vec![1e-6, 1e-6]],
            &[vec![-3e-6, -5e-6], vec![-1e-6, -1e-6]],
        );
        let s = 2f64.sqrt() * 1e-6;
        assert!((r.reversal - 8e-6).abs() < 1e-15);
        assert!((r.repeatability - (4.0 * s + 8e-6)).abs() < 1e-15);
        assert!((r.systematic - 8e-6).abs() < 1e-15);
        assert!(r.mean_range.abs() < 1e-15);
        assert!((r.accuracy - (8e-6 + 4.0 * s)).abs() < 1e-15);
        assert!((r.mean_deviation - 4e-6).abs() < 1e-15);
    }

    /// ISO 9283 by hand: from below the poses (1, 0) and (3, 0) um about the target, barycentre (2, 0): AP = 2 um, l =
    /// 1, 1, RP = 1 um; from above (0, -1) twice: AP = 1, RP = 0; vAP = |(2, 0) - (0, -1)| = sqrt(5) um.
    #[test]
    fn iso_9283_figures_match_a_hand_calculation() {
        let mut t = ToolResult::default();
        iso_9283(&[vec![1e-6, 0.0], vec![3e-6, 0.0]], &[vec![0.0, -1e-6], vec![0.0, -1e-6]], &mut t);
        assert!((t.accuracy - 2e-6).abs() < 1e-15);
        assert!((t.repeatability - 1e-6).abs() < 1e-15);
        assert!((t.direction_variation - 5f64.sqrt() * 1e-6).abs() < 1e-15);
    }
}
