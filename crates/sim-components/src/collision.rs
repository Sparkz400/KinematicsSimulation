//! Obstacles of the config and the clearance the machine keeps from them: along a path before it runs (`check_path`,
//! `check_cycle`), and in the simulation as it runs (`Sim`). The machine tells which of its parts can hit an obstacle
//! (`Machine::collision_bodies`, `collision_capsules`); obstacles are boxes along the scene's axes, the parts boxes too
//! or, slanted, capsules.

use sim_core::{Aabb, Capsule, Config, ConfigError, Machine, MachineEntry, Obstacle};

/// An obstacle's contact stiffness and damping where its entry gives none (effects.crash): [estimate] a fixture of
/// steel profile on a machine bed, some 10 N/um at the point of contact, damped near critically for a few kilograms
/// (2 sqrt(k m) = 2 sqrt(1e7 * 2.5) = 1e4 N s/m). [bounds] 1e6 to 1e8 N/m: from an aluminium profile on a slender
/// stand to a solid steel block.
pub const CONTACT_STIFFNESS: f64 = 1e7;
pub const CONTACT_DAMPING: f64 = 1e4;

/// The obstacles of the config: `obstacles`, a list of tables each with a `name`, the box's `centre_m` and `size_m`
/// (three numbers each, in scene coordinates; the sizes above 0), and optionally its contact `stiffness_n_m` (above 0)
/// and `damping_n_s_m` (not negative), by default `CONTACT_STIFFNESS` and `CONTACT_DAMPING`. A config without the
/// list has none.
pub fn obstacles(cfg: &Config) -> Result<Vec<Obstacle>, ConfigError> {
    let Ok(list) = cfg.get("obstacles") else { return Ok(Vec::new()) };
    let list = list.as_array().ok_or_else(|| ConfigError::Type("obstacles".into()))?;
    list.iter()
        .enumerate()
        .map(|(i, item)| {
            let bad = || ConfigError::Invalid(format!("obstacles[{i}]"));
            let three = |key: &str| -> Option<[f64; 3]> {
                let values: Vec<f64> = item
                    .get(key)?
                    .as_array()?
                    .iter()
                    .map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)))
                    .collect::<Option<_>>()?;
                values.try_into().ok().filter(|v: &[f64; 3]| v.iter().all(|x| x.is_finite()))
            };
            // A misspelt key in an entry would otherwise pass silently, as `Config::check_keys` leaves arrays out.
            let keys = ["name", "centre_m", "size_m", "stiffness_n_m", "damping_n_s_m"];
            if item.as_table().is_none_or(|t| t.keys().any(|k| !keys.contains(&k.as_str()))) {
                return Err(bad());
            }
            let name = item.get("name").and_then(|n| n.as_str()).ok_or_else(bad)?;
            let centre = three("centre_m").ok_or_else(bad)?;
            let size = three("size_m").filter(|s| s.iter().all(|x| *x > 0.0)).ok_or_else(bad)?;
            let number = |key: &str, default: f64| match item.get(key) {
                None => Some(default),
                Some(v) => v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).filter(|x| x.is_finite()),
            };
            let stiffness = number("stiffness_n_m", CONTACT_STIFFNESS).filter(|k| *k > 0.0).ok_or_else(bad)?;
            let damping = number("damping_n_s_m", CONTACT_DAMPING).filter(|c| *c >= 0.0).ok_or_else(bad)?;
            Ok(Obstacle { name: name.into(), aabb: Aabb { centre, size }, contact: false, stiffness, damping })
        })
        .collect()
}

/// The parts of a machine that can hit an obstacle at a state: its boxes and its capsules.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Bodies {
    pub boxes: Vec<Aabb>,
    pub capsules: Vec<Capsule>,
}

impl Bodies {
    pub fn of(machine: &dyn Machine, state: &[f64]) -> Self {
        Self { boxes: machine.collision_bodies(state), capsules: machine.collision_capsules(state) }
    }

    /// The nearest points and the clearance (`Aabb::gap`, `Capsule::gap`) of the part nearest to the box `other`;
    /// `None` without parts.
    pub fn gap(&self, other: &Aabb) -> Option<([f64; 3], [f64; 3], f64)> {
        let boxes = self.boxes.iter().map(|b| b.gap(other));
        boxes.chain(self.capsules.iter().map(|c| c.gap(other))).min_by(|a, b| a.2.total_cmp(&b.2))
    }
}

/// The least clearance between the parts `bodies` and the obstacles: which obstacle, the nearest points and the
/// clearance (negative in an overlap). `None` without parts or obstacles.
pub fn nearest(bodies: &Bodies, obstacles: &[Obstacle]) -> Option<(usize, ([f64; 3], [f64; 3], f64))> {
    obstacles
        .iter()
        .enumerate()
        .filter_map(|(k, o)| Some((k, bodies.gap(&o.aabb)?)))
        .min_by(|a, b| a.1.2.total_cmp(&b.1.2))
}

/// What a path keeps from the obstacles before it runs: the least clearance along it (m, infinite without obstacles),
/// the first pose at which a part of the machine touches one (clearance 0 or less), and the first pose at the least
/// clearance (`None` without obstacles).
#[derive(Debug, Clone, PartialEq)]
pub struct PlanCheck {
    pub clearance: f64,
    pub contact: Option<Vec<f64>>,
    pub nearest: Option<Vec<f64>>,
}

impl Default for PlanCheck {
    fn default() -> Self {
        Self { clearance: f64::INFINITY, contact: None, nearest: None }
    }
}

/// Adds the path through `poses` to `check`: straight from pose to pose, looked at every millimetre (every
/// thousandth of an axis unit) or closer, the machine at rest at each pose with the payload as it is set.
pub fn check_path(machine: &dyn Machine, obstacles: &[Obstacle], poses: &[Vec<f64>], check: &mut PlanCheck) {
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    let mut at = |pose: &[f64]| {
        machine.equilibrium(pose, &mut state, &mut torque);
        let gap = nearest(&Bodies::of(machine, &state), obstacles).map_or(f64::INFINITY, |(_, (_, _, gap))| gap);
        if gap < check.clearance {
            (check.clearance, check.nearest) = (gap, Some(pose.to_vec()));
        }
        if gap <= 0.0 && check.contact.is_none() {
            check.contact = Some(pose.to_vec());
        }
    };
    if let Some(first) = poses.first() {
        at(first);
    }
    for pair in poses.windows(2) {
        let length = pair[0].iter().zip(&pair[1]).map(|(a, b)| (b - a).powi(2)).sum::<f64>().sqrt();
        let pieces = (length / 1e-3).ceil().max(1.0) as usize;
        for k in 1..=pieces {
            let f = k as f64 / pieces as f64;
            let pose: Vec<f64> = pair[0].iter().zip(&pair[1]).map(|(a, b)| a + f * (b - a)).collect();
            at(&pose);
        }
    }
}

/// The configured cycle checked as `check_path` does, before it runs: from the start pose to the first segment's
/// target with the payload attached (as the controller starts), then each segment from the target before it with the
/// payload as that one left it, and round from the last to the first again. A segment with via points takes the path
/// the planner times through them; the straight moves are taken as lines, which corner blending would cut short (the
/// simulation itself is checked as it runs). `None` without obstacles, and with `[conveyor]`, whose passes are planned
/// as the parts come: the simulation's live check judges them.
pub fn check_cycle(entry: &MachineEntry, cfg: &Config) -> Result<Option<PlanCheck>, ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let obstacles = obstacles(cfg)?;
    if obstacles.is_empty() || cfg.get("conveyor").is_ok() && cfg.bool("conveyor.enabled")? {
        return Ok(None);
    }
    let mut machine = (entry.build)(cfg)?;
    let start = crate::sim::start_pose(cfg, machine.as_ref())?;
    let (cycle, tolerance) = (cfg.positive("profile.cycle_s")?, cfg.non_negative("profile.path_tolerance_m")?);
    let number = |v: &toml::Value| v.as_float().or_else(|| v.as_integer().map(|i| i as f64));
    let mut segments = Vec::new();
    for (i, segment) in cfg.array("cycle.segment")?.iter().enumerate() {
        let bad = || ConfigError::Invalid(format!("cycle.segment[{i}]"));
        let target: Vec<f64> =
            segment.get("target").and_then(|t| t.as_array()).ok_or_else(bad)?.iter().filter_map(number).collect();
        let payload = segment.get("payload").and_then(|p| p.as_bool()).ok_or_else(bad)?;
        let via: Option<Vec<Vec<f64>>> = match segment.get("via") {
            None => None,
            Some(points) => Some(
                points
                    .as_array()
                    .ok_or_else(bad)?
                    .iter()
                    .map(|p| p.as_array().map(|p| p.iter().filter_map(number).collect()))
                    .collect::<Option<_>>()
                    .ok_or_else(bad)?,
            ),
        };
        segments.push((target, payload, via, i));
    }
    let mut check = PlanCheck::default();
    check_path(machine.as_ref(), &obstacles, std::slice::from_ref(&start), &mut check);
    let n = segments.len();
    // Leg k ends at segment k; leg n is the way round from the last segment to the first.
    for k in (0..=n).filter(|_| n > 0) {
        let (from, payload) =
            if k == 0 { (start.clone(), true) } else { (segments[k - 1].0.clone(), segments[k - 1].1) };
        let (target, _, via, i) = &segments[k % n];
        machine.set_payload_attached(payload);
        let poses = match via {
            None => vec![from, target.clone()],
            Some(points) => {
                let pieces = crate::control::via_pieces(points, target);
                let planned =
                    crate::path::plan(machine.axes(), &from, &pieces, cycle, tolerance, crate::path::ARC_SLACK)
                        .map_err(|_| ConfigError::Invalid(format!("cycle.segment[{i}].via")))?;
                std::iter::once(from).chain(planned.into_iter().map(|(_, pose)| pose)).collect()
            }
        };
        check_path(machine.as_ref(), &obstacles, &poses, &mut check);
    }
    Ok(Some(check))
}
