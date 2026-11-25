//! Obstacles of the config and the clearance the machine keeps from them: along a path before it runs (`check_path`,
//! `check_cycle`), and in the simulation as it runs (`Sim`). The machine tells which of its parts can hit an obstacle
//! (`Machine::collision_bodies`, `collision_capsules`); obstacles are boxes along the scene's axes, the parts boxes too
//! or, slanted, capsules.

use crate::conveyor::Conveyor;
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

/// One of the machine's own parts, by its place among `Machine::collision_bodies` (a box) or
/// `Machine::self_collision_capsules` (a capsule), counted from 0.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Body {
    Box(usize),
    Capsule(usize),
}

/// The least clearance between two of the machine's own parts that are not joined along a path: m (negative in an
/// overlap), the two parts, and the first pose at it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SelfClearance {
    pub clearance: f64,
    pub bodies: [Body; 2],
    pub pose: Vec<f64>,
}

/// Which pairs of parts are joined (not checked) for a pose with `counts` boxes and capsules: `joined[i * n + j]`.
#[derive(Debug, Clone, PartialEq)]
struct Joined {
    counts: (usize, usize),
    joined: Vec<bool>,
    /// Whether any pair is left to check.
    open: bool,
}

/// What a path keeps from the obstacles before it runs: the least clearance along it (m, infinite without obstacles),
/// the first pose at which a part of the machine touches one (clearance 0 or less), and the first pose at the least
/// clearance (`None` without obstacles). `own` is the least clearance between the machine's own parts that are not
/// joined (`Machine::connected_bodies`), a figure apart from the obstacles'; `None` where nothing was checked (fewer
/// than two parts, or all of them joined), which is not the same as clear.
#[derive(Debug, Clone, PartialEq)]
pub struct PlanCheck {
    pub clearance: f64,
    pub contact: Option<Vec<f64>>,
    pub nearest: Option<Vec<f64>>,
    pub own: Option<SelfClearance>,
    /// Parts closer than this at the start pose are taken as joined where the machine does not say
    /// (`profile.min_clearance_m`).
    join_below: f64,
    /// The joined pairs for each number of boxes and capsules: those of the start pose (`at_start`), or of the first
    /// pose seen with that number.
    joins: Vec<Joined>,
}

impl Default for PlanCheck {
    fn default() -> Self {
        Self { clearance: f64::INFINITY, contact: None, nearest: None, own: None, join_below: 0.0, joins: Vec::new() }
    }
}

impl PlanCheck {
    /// A check that takes parts of the machine closer than `clearance` at the first pose as joined.
    pub fn keeping(clearance: f64) -> Self {
        Self { join_below: clearance, ..Self::default() }
    }

    /// `keeping`, with the parts joined as they are at the machine's start pose, whatever pose a path starts from:
    /// `state` is `machine`'s there with the payload on (as the controller starts). Where the machine names its joined
    /// pairs (`Machine::connected_bodies`) that is all; where the rule joins them by their distance, and some pair is
    /// left to check, also with the payload off, on a machine of its own that `detached` builds (switching the payload
    /// may leave more in a machine: a crane books a pick), at the start pose `start`.
    pub fn at_start(
        clearance: f64,
        machine: &dyn Machine,
        state: &[f64],
        start: &[f64],
        detached: impl FnOnce() -> Result<Box<dyn Machine>, ConfigError>,
    ) -> Result<Self, ConfigError> {
        let mut check = Self::keeping(clearance);
        if !check.join_at(machine, state) && check.has_pairs() {
            let mut other = detached()?;
            other.set_payload_attached(false);
            let (mut state, mut torque) = (Vec::new(), vec![0.0; other.motors().len()]);
            other.equilibrium(start, &mut state, &mut torque);
            check.join_at(other.as_ref(), &state);
        }
        Ok(check)
    }

    /// The joined pairs of `machine`'s parts at `state`; whether the machine named them.
    fn join_at(&mut self, machine: &dyn Machine, state: &[f64]) -> bool {
        let (boxes, capsules) = (machine.collision_bodies(state), machine.self_collision_capsules(state));
        let named = machine.connected_bodies(state, boxes.len(), capsules.len());
        let is_named = named.is_some();
        self.joins_for(&boxes, &capsules, || named);
        is_named
    }

    /// A check with the same joined pairs and nothing seen yet.
    pub fn anew(&self) -> Self {
        Self { join_below: self.join_below, joins: self.joins.clone(), ..Self::default() }
    }

    /// Whether the machine has two parts not joined to check, as far as the start pose tells (`at_start`).
    pub fn has_pairs(&self) -> bool {
        self.joins.iter().any(|j| j.open)
    }

    /// The joined pairs for parts of these numbers, found now where none are known for them.
    fn joins_for(
        &mut self,
        boxes: &[Aabb],
        capsules: &[Capsule],
        connected: impl FnOnce() -> Option<Vec<(usize, usize)>>,
    ) -> &Joined {
        let counts = (boxes.len(), capsules.len());
        let k = match self.joins.iter().position(|j| j.counts == counts) {
            Some(k) => k,
            None => {
                self.joins.push(joined(connected(), boxes, capsules, self.join_below));
                self.joins.len() - 1
            }
        };
        &self.joins[k]
    }

    /// Whether the machine's own parts came nearer than `limit` (`profile.min_clearance_m`) or touched: as the check
    /// "collision" judges the obstacles, a contact fails whatever the limit. Not checked is not failed.
    pub fn own_fails(&self, limit: f64) -> bool {
        self.own.as_ref().is_some_and(|own| own.clearance < limit || own.clearance <= 0.0)
    }

    /// The machine's own parts at `state` (its boxes `boxes`): the least clearance of the pairs not joined, if it is
    /// below the least so far. Linear in the samples: a lower bound from the parts' extents spares most pairs, and no
    /// pair allocates.
    pub(crate) fn look_at_own(&mut self, machine: &dyn Machine, state: &[f64], boxes: &[Aabb], pose: &[f64]) {
        let capsules = machine.self_collision_capsules(state);
        let connected = || machine.connected_bodies(state, boxes.len(), capsules.len());
        self.look_at_parts(boxes, &capsules, connected, pose);
    }

    /// `look_at_own` for parts given: `connected` tells the joined pairs of these parts where the machine knows them
    /// (`Machine::connected_bodies`), asked once for each number of boxes and capsules that `at_start` did not see.
    pub fn look_at_parts(
        &mut self,
        boxes: &[Aabb],
        capsules: &[Capsule],
        connected: impl FnOnce() -> Option<Vec<(usize, usize)>>,
        pose: &[f64],
    ) {
        let n = boxes.len() + capsules.len();
        if n < 2 {
            return;
        }
        let mut best = self.own.as_ref().map_or(f64::INFINITY, |own| own.clearance);
        let joins = self.joins_for(boxes, capsules, connected);
        if !joins.open {
            return;
        }
        let mut found = None;
        for i in 0..n {
            for j in i + 1..n {
                if joins.joined[i * n + j] {
                    continue;
                }
                let (a, b) = (part(boxes, capsules, i), part(boxes, capsules, j));
                let bound = a.lower_bound(&b);
                // Extents that overlap say nothing about the parts: only apart ones can be spared.
                if bound > 0.0 && bound >= best {
                    continue;
                }
                let gap = a.gap(&b);
                if gap < best {
                    (best, found) = (gap, Some((i, j)));
                }
            }
        }
        if let Some((i, j)) = found {
            let body = |k: usize| if k < boxes.len() { Body::Box(k) } else { Body::Capsule(k - boxes.len()) };
            self.own = Some(SelfClearance { clearance: best, bodies: [body(i), body(j)], pose: pose.to_vec() });
        }
    }
}

/// A part of the machine as a box or a capsule.
#[derive(Clone, Copy)]
enum Part<'a> {
    Box(&'a Aabb),
    Capsule(&'a Capsule),
}

fn part<'a>(boxes: &'a [Aabb], capsules: &'a [Capsule], i: usize) -> Part<'a> {
    if i < boxes.len() { Part::Box(&boxes[i]) } else { Part::Capsule(&capsules[i - boxes.len()]) }
}

impl Part<'_> {
    fn extent(&self) -> ([f64; 3], [f64; 3]) {
        match self {
            Part::Box(b) => b.extent(),
            Part::Capsule(c) => c.extent(),
        }
    }

    /// The distance of the two parts' extents: no more than their clearance where they are apart.
    fn lower_bound(&self, other: &Part) -> f64 {
        let ((lo, hi), (other_lo, other_hi)) = (self.extent(), other.extent());
        (0..3).map(|i| (lo[i] - other_hi[i]).max(other_lo[i] - hi[i]).max(0.0).powi(2)).sum::<f64>().sqrt()
    }

    fn gap(&self, other: &Part) -> f64 {
        match (self, other) {
            (Part::Box(a), Part::Box(b)) => a.gap(b).2,
            (Part::Capsule(c), Part::Box(b)) | (Part::Box(b), Part::Capsule(c)) => c.gap(b).2,
            (Part::Capsule(a), Part::Capsule(b)) => a.gap_capsule(b),
        }
    }
}

/// How near two parts are to count as touching when they are joined (`joined`): faces drawn at the same place miss
/// each other by rounding (a slide's payload on its carriage stands 7e-18 m off it), and a pair so joined by chance
/// would be judged on that noise.
const TOUCHING: f64 = 1e-9;

/// The pairs of parts not to check against each other: those the machine names (`Machine::connected_bodies`), or else
/// those that touch, overlap or lie closer than `below` to each other now.
fn joined(named: Option<Vec<(usize, usize)>>, boxes: &[Aabb], capsules: &[Capsule], below: f64) -> Joined {
    let n = boxes.len() + capsules.len();
    let mut joined = vec![false; n * n];
    match named {
        Some(pairs) => {
            for (i, j) in pairs.into_iter().filter(|(i, j)| *i < n && *j < n) {
                (joined[i * n + j], joined[j * n + i]) = (true, true);
            }
        }
        None => {
            for i in 0..n {
                for j in i + 1..n {
                    let gap = part(boxes, capsules, i).gap(&part(boxes, capsules, j));
                    let together = gap < below || gap <= TOUCHING;
                    (joined[i * n + j], joined[j * n + i]) = (together, together);
                }
            }
        }
    }
    let open = (0..n).any(|i| (i + 1..n).any(|j| !joined[i * n + j]));
    Joined { counts: (boxes.len(), capsules.len()), joined, open }
}

/// Adds the path through `poses` to `check`: straight from pose to pose, looked at every millimetre (every
/// thousandth of an axis unit) or closer, the machine at rest at each pose with the payload as it is set.
pub fn check_path(machine: &dyn Machine, obstacles: &[Obstacle], poses: &[Vec<f64>], check: &mut PlanCheck) {
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    let mut at = |pose: &[f64]| {
        machine.equilibrium(pose, &mut state, &mut torque);
        let bodies = Bodies::of(machine, &state);
        let gap = nearest(&bodies, obstacles).map_or(f64::INFINITY, |(_, (_, _, gap))| gap);
        check.look_at_own(machine, &state, &bodies.boxes, pose);
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

/// The passes of a conveyor cycle checked as `check_path` does, before they run: planned as the controller plans them
/// (`Conveyor::plan`, without input shaping) from `start` at time 0, each from where the one before ends, for two
/// pitches' worth of parts (all parts of a pitch, twice, so that a pass from the start and one from the place are both
/// there), the payload as the pass holds it. The path between samples 5 ms apart is taken as a line.
fn check_passes(
    machine: &mut dyn Machine,
    obstacles: &[Obstacle],
    conveyor: &mut Conveyor,
    start: &[f64],
    mut check: PlanCheck,
) -> PlanCheck {
    let axes = machine.axes().to_vec();
    let reachable = |pose: &[f64]| crate::control::in_range(&*machine, pose);
    let mut passes = Vec::new();
    let (mut now, mut from) = (0.0, start.to_vec());
    let none = (vec![0.0; axes.len()], 0.0);
    while passes.len() < 2 * conveyor.arrivals.len()
        && let Some(pass) = conveyor.plan(&axes, now, &from, &reachable, &none)
    {
        let n = from.len();
        let (mut pos, mut vel, mut acc) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        pass.at(pass.duration(), &mut pos, &mut vel, &mut acc);
        (now, from) = (pass.start + pass.duration(), pos);
        passes.push(pass);
    }
    check_path(machine, obstacles, std::slice::from_ref(&start.to_vec()), &mut check);
    for pass in &passes {
        let samples = (pass.duration() / 0.005).ceil().max(1.0) as usize;
        let n = start.len();
        let (mut pos, mut vel, mut acc) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        let mut last: Option<Vec<f64>> = None;
        for i in 0..=samples {
            let t = pass.duration() * i as f64 / samples as f64;
            pass.at(t, &mut pos, &mut vel, &mut acc);
            machine.set_payload_attached(pass.holds(t));
            if let Some(last) = &last {
                check_path(machine, obstacles, &[last.clone(), pos.clone()], &mut check);
            }
            last = Some(pos.clone());
        }
    }
    check
}

/// The configured cycle checked as `check_path` does, before it runs: from the start pose to the first segment's
/// target with the payload attached (as the controller starts), then each segment from the target before it with the
/// payload as that one left it, and round from the last to the first again. A segment with via points takes the path
/// the planner times through them; the straight moves are taken as lines, which corner blending would cut short (the
/// simulation itself is checked as it runs). The machine's own parts are checked against each other along the way
/// (`PlanCheck::own`, with `profile.min_clearance_m` as the distance under which parts at the start pose are joined,
/// `PlanCheck::at_start`).
/// `None` without obstacles where the machine has no two parts to check (or the cycle cannot be read: then it is
/// not checked, where with obstacles it is an error). With `[conveyor]` the passes are checked instead
/// (`check_passes`).
pub fn check_cycle(entry: &MachineEntry, cfg: &Config) -> Result<Option<PlanCheck>, ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let obstacles = obstacles(cfg)?;
    match cycle_check(entry, cfg, &obstacles) {
        Ok(check) => Ok((!obstacles.is_empty() || check.own.is_some()).then_some(check)),
        Err(_) if obstacles.is_empty() => Ok(None),
        Err(e) => Err(e),
    }
}

fn cycle_check(entry: &MachineEntry, cfg: &Config, obstacles: &[Obstacle]) -> Result<PlanCheck, ConfigError> {
    let mut machine = (entry.build)(cfg)?;
    let start = crate::sim::start_pose(cfg, machine.as_ref())?;
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.set_payload_attached(true);
    machine.equilibrium(&start, &mut state, &mut torque);
    let clearance = cfg.non_negative("profile.min_clearance_m")?;
    let mut check = PlanCheck::at_start(clearance, machine.as_ref(), &state, &start, || (entry.build)(cfg))?;
    if obstacles.is_empty() && !check.has_pairs() {
        return Ok(PlanCheck::default());
    }
    if let Some(mut conveyor) = crate::conveyor::Conveyor::from_config(cfg, machine.axes().len())? {
        return Ok(check_passes(machine.as_mut(), obstacles, &mut conveyor, &start, check));
    }
    let (cycle, tolerance) = (
        cfg.positive("profile.cycle_s")?,
        crate::path::Tolerance {
            path: cfg.non_negative("profile.path_tolerance_m")?,
            corner: cfg.non_negative("profile.corner_tolerance_m")?,
        },
    );
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
        // With `corners_m`: lines through the via points, their corners rounded within it (`control::lines_through`).
        let corner = segment.get("corners_m").and_then(number);
        segments.push((target, payload, via.map(|v| (v, corner)), i));
    }
    check_path(machine.as_ref(), obstacles, std::slice::from_ref(&start), &mut check);
    let n = segments.len();
    // Leg k ends at segment k; leg n is the way round from the last segment to the first.
    for k in (0..=n).filter(|_| n > 0) {
        let (from, payload) =
            if k == 0 { (start.clone(), true) } else { (segments[k - 1].0.clone(), segments[k - 1].1) };
        let (target, _, via, i) = &segments[k % n];
        machine.set_payload_attached(payload);
        let poses = match via {
            None => vec![from, target.clone()],
            Some((points, corner)) => {
                let pieces = match corner {
                    Some(_) => crate::control::lines_through(points, target),
                    None => crate::control::via_pieces(points, target),
                };
                let tolerance = crate::path::Tolerance { corner: corner.unwrap_or(tolerance.corner), ..tolerance };
                let planned =
                    crate::path::plan(machine.axes(), &from, &pieces, cycle, tolerance, crate::path::ARC_SLACK)
                        .map_err(|_| ConfigError::Invalid(format!("cycle.segment[{i}].via")))?;
                std::iter::once(from).chain(planned.into_iter().map(|(_, pose)| pose)).collect()
            }
        };
        check_path(machine.as_ref(), obstacles, &poses, &mut check);
    }
    Ok(check)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(x: f64) -> Aabb {
        Aabb { centre: [x, 0.0, 0.0], size: [0.1; 3] }
    }

    /// Two 0.1 m cubes, the second first 0.2 m clear of the first (x = 0.3: faces at 0.05 and 0.25) and then at
    /// x = 0.103 (faces at 0.05 and 0.053): 3 mm apart. With 5 mm kept the 3 mm fails and names both boxes, the 0.2 m
    /// before it does not; with 2 mm kept the 3 mm passes. Fails if the pair is not looked at, or is judged without the
    /// limit.
    #[test]
    fn two_boxes_3_mm_apart_fail_a_clearance_of_5_mm_and_name_both() {
        let mut check = PlanCheck::keeping(0.005);
        check.look_at_parts(&[cube(0.0), cube(0.3)], &[], || None, &[0.3]);
        let own = check.own.clone().unwrap();
        assert!((own.clearance - 0.2).abs() < 1e-12 && !check.own_fails(0.005), "{own:?}");
        check.look_at_parts(&[cube(0.0), cube(0.103)], &[], || None, &[0.103]);
        let own = check.own.clone().unwrap();
        assert!((own.clearance - 0.003).abs() < 1e-12, "{own:?}");
        assert_eq!((own.bodies, own.pose), ([Body::Box(0), Body::Box(1)], vec![0.103]));
        assert!(check.own_fails(0.005) && !check.own_fails(0.002));
    }

    /// The same two cubes but named as joined: nothing is left to check, which says "not checked", not "clear".
    #[test]
    fn joined_parts_are_skipped_and_leave_the_check_not_made() {
        let mut check = PlanCheck::keeping(0.005);
        check.look_at_parts(&[cube(0.0), cube(0.3)], &[], || Some(vec![(1, 0)]), &[0.3]);
        check.look_at_parts(&[cube(0.0), cube(0.103)], &[], || panic!("asked again for the same parts"), &[0.1]);
        assert_eq!(check.own, None);
        assert!(!check.own_fails(0.005));
    }

    /// One part, none, or a part and nothing else: not checked, whatever the pose.
    #[test]
    fn fewer_than_two_parts_are_not_checked() {
        let mut check = PlanCheck::keeping(0.005);
        check.look_at_parts(&[cube(0.0)], &[], || None, &[0.0]);
        check.look_at_parts(&[], &[], || None, &[0.0]);
        assert_eq!(check.own, None);
    }

    /// Without a say of the machine, parts that touch or lie closer than the clearance at the first pose are joined: two
    /// cubes 3 mm apart from the start are not reported at 3 mm (the rule cannot see them). Fails if the first pose
    /// does not join.
    #[test]
    fn parts_in_contact_at_the_first_pose_are_taken_as_joined() {
        let mut check = PlanCheck::keeping(0.005);
        check.look_at_parts(&[cube(0.0), cube(0.103)], &[], || None, &[0.0]);
        check.look_at_parts(&[cube(0.0), cube(0.101)], &[], || None, &[1.0]);
        assert_eq!(check.own, None);
        // Overlapping ones join with no clearance asked for as well.
        let mut check = PlanCheck::default();
        check.look_at_parts(&[cube(0.0), cube(0.05)], &[], || None, &[0.0]);
        assert_eq!(check.own, None);
    }

    /// A capsule along x from the origin to (1, 0, 0), radius 0.01, and one along y through x = 0.5 at height 0.05,
    /// radius 0.02: the segments cross 0.05 m apart in z, so the clearance is 0.05 - 0.01 - 0.02 = 0.02. A 0.1 m cube at
    /// (0.5, 0.5, 0.5) has its lowest face at z = 0.45: 0.45 - 0.01 = 0.44 m from the first capsule and 0.4 - 0.02 =
    /// 0.38 m from the second, so the capsules are the nearest pair.
    #[test]
    fn capsules_and_boxes_are_judged_by_their_surfaces() {
        let along_x = Capsule { from: [0.0; 3], to: [1.0, 0.0, 0.0], radius: 0.01 };
        let along_y = Capsule { from: [0.5, -1.0, 0.05], to: [0.5, 1.0, 0.05], radius: 0.02 };
        let high = Aabb { centre: [0.5, 0.5, 0.5], size: [0.1; 3] };
        let mut check = PlanCheck::default();
        check.look_at_parts(&[high], &[along_x, along_y], || Some(Vec::new()), &[0.0]);
        let own = check.own.unwrap();
        assert!((own.clearance - 0.02).abs() < 1e-12, "{own:?}");
        assert_eq!(own.bodies, [Body::Capsule(0), Body::Capsule(1)]);
    }
}
