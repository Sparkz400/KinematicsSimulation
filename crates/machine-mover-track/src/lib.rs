//! A linear transport system with independent movers on a closed track, as Beckhoff XTS or B&R ACOPOStrak: motor
//! segments (modules) of linear-motor coils in a loop of two straights and two 180 degree curves, and on them movers,
//! each a mass on rollers carrying the magnet plate, each driven by the coils under it with a position loop of its own.
//!
//! Each mover is a joint of `sim_components::chain::JointMachine`: its coordinate is s, the distance along the track's
//! centre line (m), unwrapped (one loop adds the track's length L); its "motor" the coils under it, read in N for N m
//! and m for rad as on the linear motor axis (force constant in N/A, the magnet plate's mass as the rotor's inertia, the
//! cogging once per pole pitch). The rollers' rolling resistance is the joint's friction, with the attraction between
//! magnet plate and coils pressing on them as the linear motor axis has it. The track lies level: its movers' weight
//! does no work, M = m (body plus payload) per mover, no bias.
//!
//! The track (`Track`): straight, curve, straight, curve, the movers running anticlockwise seen from above, on the
//! outside of the loop with the magnet plate facing the coils. A curve turns by pi either at a constant radius R or with
//! a clothoid at each end (curvature rising linearly over `track.clothoid_m`, an arc of R between). A mover's rollers
//! span its wheelbase w, so the curvature its body follows is the mean over it, k(s) = (theta(s + w/2) - theta(s -
//! w/2)) / w with theta the heading: the lateral acceleration v^2 k rises over the wheelbase where a constant-radius
//! curve begins, over the clothoid where one has it, and its rate, the lateral jerk 2 v a k + v^3 dk/ds, is what the
//! clothoid takes down.
//!
//! The rollers carry the attraction F_a less the centrifugal force m v^2 k across the track (the mover sits outside the
//! curve, so the centrifugal force pulls it off the coils) and the weight m g along the rollers' other axis: guide load
//! sqrt((F_a - m v^2 k)^2 + (m g)^2). Above m v^2 k = F_a the mover lifts off its rollers (`mover.<k>.liftoff_ratio`).
//!
//! The coils (`MoverTrack`): a mover's current is its force over the force constant, I = F / k_t, and it flows in the
//! coils under its magnet plate, shared among the segments by the plate's length on each. A segment's current is the sum
//! of |I| of the movers on it in those shares; its copper loss R I^2 per mover in those shares (R the coil set under one
//! mover, `motor.resistance_ohm`). Each segment warms with its loss through one thermal resistance and time constant
//! (`segment.temperature_rise`); the verdict judges the rise it settles at, the loss averaged over about one pass of the
//! cycle times the thermal resistance: several movers stopping on one segment heat it.
//!
//! The cycle is an electronic cam (`[cam]`, `track.cycle`): "index" moves every mover on by one station pitch L / n
//! (`track.index_law`) and stands it at its station for the rest of the period, at `track.index_rate_hz`; "flying" runs
//! each mover once round the loop per n products of a conveyor (the master, m), synchronous with it over
//! `track.process_length_m` and back to the start of the process along the rest of the loop, the movers a product apart.
//! With `cam.run = false` (default.toml, as the crank and cam machines ship) the config's `[[cycle.segment]]` move the
//! movers instead, as on every machine; `flying.toml` runs its cam, and `--set cam.run=true` the index cam.

use sim_components::cam::{Cam, Law, Segment};
use sim_components::chain::{JointMachine, JointSpec, MOTOR_IDS, Mechanism};
use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_components::transmission::LinearMotor;
use sim_core::{
    AxisInfo, BeltSection, CamMaster, Config, ConfigError, FrameMomentum, FrictionContact, Label, Machine,
    MachineEntry, Mode, MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo, ThermalPart,
};
use std::f64::consts::{PI, TAU};
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "mover-track",
    key: "machine-mover-track",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

/// The flying-process cycle: the default config with `track.cycle = "flying"` and its start poses (a test keeps the two
/// files the same but for those). Load it as a whole config with `--config`.
pub const FLYING_CONFIG: &str = include_str!("../flying.toml");

const G: f64 = 9.80665;
/// Steps of the curve's table of points (drawing only).
const CURVE_POINTS: usize = 2000;
/// Strips a curve segment is drawn with.
const CURVE_STRIPS: usize = 12;

pub const MAX_MOVERS: usize = 7;
/// How far a mover's axis range reaches behind and ahead of its cycle's targets, m: as far as one mover alone at an end
/// of its range keeps from its neighbours at their starts (the shipped 0.503 m pitch less a body and, ahead, the 0.3 m of
/// jog: 0.433 m and 0.133 m), a little less. The 0.8 m range gives one mover alone a stroke of some 0.25 s at its top
/// speed; at the middles of the box's faces the shipped movers keep 0.103 m.
const BEHIND: f64 = 0.37;
const AHEAD: f64 = 0.13;
const AXIS_IDS: [&str; MAX_MOVERS] = ["s1", "s2", "s3", "s4", "s5", "s6", "s7"];
const AXIS_KEYS: [&str; MAX_MOVERS] =
    ["axis-mover-1", "axis-mover-2", "axis-mover-3", "axis-mover-4", "axis-mover-5", "axis-mover-6", "axis-mover-7"];
const MOVER_KEYS: [&str; MAX_MOVERS] = ["mover-1", "mover-2", "mover-3", "mover-4", "mover-5", "mover-6", "mover-7"];
const MOTOR_KEYS: [&str; MAX_MOVERS] = [
    "motor-mover-1",
    "motor-mover-2",
    "motor-mover-3",
    "motor-mover-4",
    "motor-mover-5",
    "motor-mover-6",
    "motor-mover-7",
];

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 7] = [
    "joint_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    sim_components::base::mount(cfg, Box::new(MoverTrack::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models (the magnet plate on its mount), with and without the payload.
fn suite_configs() -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config();
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg
        })
        .collect()
}

/// The closed track's centre line: straight 1 from the origin along +x, curve 1, straight 2 back along -x at the
/// track's width, curve 2; and its motor segments (each straight cut into modules, each curve one).
#[derive(Debug, Clone)]
pub struct Track {
    pub straight: f64,
    pub radius: f64,
    pub clothoid: f64,
    /// One curve's length, pi R + clothoid.
    pub curve: f64,
    /// The loop, 2 straight + 2 curve.
    pub length: f64,
    /// The distance between the straights (the curve's "diameter" in Beckhoff's catalogue).
    pub width: f64,
    /// Where each segment starts along the loop, rising from 0; the last ends at `length`.
    pub segments: Vec<f64>,
    /// The curve's points in its own frame (x along straight 1, y towards straight 2) every curve / CURVE_POINTS.
    points: Vec<[f64; 2]>,
}

impl Track {
    pub fn new(straight: f64, module: f64, radius: f64, clothoid: f64) -> Result<Self, ConfigError> {
        if clothoid < 0.0 || clothoid >= PI * radius {
            return Err(ConfigError::Invalid("track.clothoid_m".into()));
        }
        let modules = (straight / module).round();
        if modules < 1.0 || (modules * module - straight).abs() > 1e-9 {
            return Err(ConfigError::Invalid("track.straight_m".into()));
        }
        let curve = PI * radius + clothoid;
        let mut t = Self {
            straight,
            radius,
            clothoid,
            curve,
            length: 2.0 * (straight + curve),
            width: 0.0,
            segments: Vec::new(),
            points: Vec::with_capacity(CURVE_POINTS + 1),
        };
        // The curve's points by the midpoint rule along its heading.
        let h = curve / CURVE_POINTS as f64;
        let mut p = [0.0, 0.0];
        t.points.push(p);
        for i in 0..CURVE_POINTS {
            let theta = t.curve_heading((i as f64 + 0.5) * h);
            p = [p[0] + h * theta.cos(), p[1] + h * theta.sin()];
            t.points.push(p);
        }
        t.width = p[1];
        let n = modules as usize;
        for start in [0.0, straight + curve] {
            t.segments.extend((0..n).map(|k| start + k as f64 * module));
            t.segments.push(start + straight);
        }
        Ok(t)
    }

    /// The heading within a curve at `u` from its start: 0 to pi.
    fn curve_heading(&self, u: f64) -> f64 {
        let (r, lc, c) = (self.radius, self.clothoid, self.curve);
        let u = u.clamp(0.0, c);
        if lc == 0.0 {
            u / r
        } else if u < lc {
            u * u / (2.0 * r * lc)
        } else if u <= c - lc {
            lc / (2.0 * r) + (u - lc) / r
        } else {
            PI - (c - u).powi(2) / (2.0 * r * lc)
        }
    }
    fn curve_curvature(&self, u: f64) -> f64 {
        let (r, lc, c) = (self.radius, self.clothoid, self.curve);
        if !(0.0..c).contains(&u) {
            0.0
        } else if lc == 0.0 || (lc..=c - lc).contains(&u) {
            1.0 / r
        } else if u < lc {
            u / (r * lc)
        } else {
            (c - u) / (r * lc)
        }
    }

    /// `s` on the loop (0 to L) and whole loops before it.
    fn wrap(&self, s: f64) -> (f64, f64) {
        let turns = (s / self.length).floor();
        ((s - turns * self.length).clamp(0.0, self.length), turns)
    }

    /// The heading at `s`, unwrapped (2 pi more per loop).
    pub fn heading(&self, s: f64) -> f64 {
        let (r, turns) = self.wrap(s);
        let (a, c) = (self.straight, self.curve);
        TAU * turns
            + if r < a {
                0.0
            } else if r < a + c {
                self.curve_heading(r - a)
            } else if r < 2.0 * a + c {
                PI
            } else {
                PI + self.curve_heading(r - 2.0 * a - c)
            }
    }

    /// The curvature at `s`, 1/m, positive turning left.
    pub fn curvature(&self, s: f64) -> f64 {
        let (r, _) = self.wrap(s);
        let (a, c) = (self.straight, self.curve);
        if r < a + c { self.curve_curvature(r - a) } else { self.curve_curvature(r - 2.0 * a - c) }
    }

    /// The curvature a body on rollers `w` apart follows at `s`, its mean over [s - w/2, s + w/2], and its rate along s.
    pub fn mean_curvature(&self, s: f64, w: f64) -> (f64, f64) {
        if w <= 0.0 {
            return (self.curvature(s), 0.0);
        }
        let (a, b) = (s - 0.5 * w, s + 0.5 * w);
        ((self.heading(b) - self.heading(a)) / w, (self.curvature(b) - self.curvature(a)) / w)
    }

    /// The centre line's point at `s` (m, in the track's plane).
    pub fn point(&self, s: f64) -> [f64; 2] {
        let (r, _) = self.wrap(s);
        let (a, c, d) = (self.straight, self.curve, self.width);
        let local = |u: f64| {
            let x = (u / c * CURVE_POINTS as f64).clamp(0.0, CURVE_POINTS as f64);
            let i = (x.floor() as usize).min(CURVE_POINTS - 1);
            let (p, q, f) = (self.points[i], self.points[i + 1], x - i as f64);
            [p[0] + f * (q[0] - p[0]), p[1] + f * (q[1] - p[1])]
        };
        if r < a {
            [r, 0.0]
        } else if r < a + c {
            let p = local(r - a);
            [a + p[0], p[1]]
        } else if r < 2.0 * a + c {
            [a - (r - a - c), d]
        } else {
            let p = local(r - 2.0 * a - c);
            [-p[0], d - p[1]]
        }
    }

    /// Segment `j`'s start and end along the loop.
    pub fn segment(&self, j: usize) -> (f64, f64) {
        (self.segments[j], self.segments.get(j + 1).copied().unwrap_or(self.length))
    }
    /// Whether segment `j` is a curve (one segment each), else a straight module.
    pub fn is_curve(&self, j: usize) -> bool {
        let (start, end) = self.segment(j);
        (end - start - self.curve).abs() < 1e-12
            && (start == self.straight || start == 2.0 * self.straight + self.curve)
    }

    /// The share of a plate of length `plate` centred at `s` that lies over segment `j`.
    pub fn share(&self, s: f64, plate: f64, j: usize) -> f64 {
        let (r, _) = self.wrap(s);
        let (a, b) = (r - 0.5 * plate, r + 0.5 * plate);
        let (start, end) = self.segment(j);
        let on: f64 = [-self.length, 0.0, self.length]
            .iter()
            .map(|shift| (b.min(end + shift) - a.max(start + shift)).max(0.0))
            .sum();
        on / plate
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Kind {
    Index,
    Flying,
}

/// The cycle's cam: every mover's (`Movers::cam_at`), its master and which cycle it is; `pitch` the master's travel
/// between neighbouring movers ("flying"), 0 where all run the same cam ("index").
#[derive(Debug, Clone)]
pub struct Cycle {
    pub kind: Kind,
    pub cam: Cam,
    pub master: CamMaster,
    pub pitch: f64,
}

impl Cycle {
    fn from_config(cfg: &Config, track: &Track, n: usize) -> Result<Self, ConfigError> {
        let law = |key: &str| {
            cfg.get(key)?
                .as_str()
                .and_then(Law::from_name)
                .filter(|l| !matches!(l, Law::Dwell | Law::Linear))
                .ok_or_else(|| ConfigError::Invalid(key.into()))
        };
        let ramp_s = cfg.positive("cam.ramp_s")?;
        let kind = match cfg.get("track.cycle")?.as_str() {
            Some("index") => Kind::Index,
            Some("flying") => Kind::Flying,
            _ => return Err(ConfigError::Invalid("track.cycle".into())),
        };
        // Both cycles' keys are read whichever runs, so that a config is checked whole.
        let (rate, share) = (cfg.positive("track.index_rate_hz")?, cfg.positive("track.index_share")?);
        let index_law = law("track.index_law")?;
        let (speed, pitch) = (cfg.positive("track.conveyor_speed_m_s")?, cfg.positive("track.product_pitch_m")?);
        let process = cfg.positive("track.process_length_m")?;
        let return_law = law("track.return_law")?;
        let period = n as f64 * pitch;
        Ok(match kind {
            Kind::Index => {
                if share >= 1.0 {
                    return Err(ConfigError::Invalid("track.index_share".into()));
                }
                let segments = [
                    Segment { master: 1.0 - share, slave: 0.0, law: Law::Dwell },
                    Segment { master: share, slave: track.length / n as f64, law: index_law },
                ];
                let cam =
                    Cam::from_segments(&segments).map_err(|_| ConfigError::Invalid("track.index_share".into()))?;
                Self { kind, cam, master: CamMaster { speed: rate, period: 1.0, ramp_s }, pitch: 0.0 }
            }
            Kind::Flying => {
                if process >= period || process >= track.length {
                    return Err(ConfigError::Invalid("track.process_length_m".into()));
                }
                let segments = [
                    Segment { master: process, slave: process, law: Law::Linear },
                    Segment { master: period - process, slave: track.length - process, law: return_law },
                ];
                let cam =
                    Cam::from_segments(&segments).map_err(|_| ConfigError::Invalid("track.process_length_m".into()))?;
                // One pass is every mover once round: each takes its neighbour's place after a pitch, but the drives' heat and
                // the verdicts per mover need the whole loop.
                Self { kind, cam, master: CamMaster { speed, period, ramp_s }, pitch }
            }
        })
    }

    /// Mover `k` from `start` at the master `m`: [s, s', s'', s'''] over the master.
    pub fn at(&self, k: usize, start: f64, m: f64) -> [f64; 4] {
        let phase = k as f64 * self.pitch;
        let mut c = self.cam.at(m + phase);
        c[0] += start - self.cam.at(phase)[0];
        c
    }
}

/// The ratings the movers' own checks judge (infinite where the config has none).
#[derive(Debug, Clone, Copy)]
struct Limits {
    guide: f64,
    liftoff: f64,
    gap: f64,
    station: f64,
}

/// The load the `JointMachine` drives: the movers on the track.
pub struct Movers {
    pub track: Track,
    /// A mover's body without its magnet plate (whose mass is the joint's rotor inertia), its payload, the plate.
    pub body: f64,
    pub payload: f64,
    pub plate_mass: f64,
    attached: bool,
    pub attraction: f64,
    pub g: f64,
    pub wheelbase: f64,
    /// The body's length along the track (for the gaps) and the magnet plate's (for the segments' shares).
    pub length: f64,
    pub plate: f64,
    pub start: Vec<f64>,
    pub cycle: Option<Cycle>,
    /// The cam's master as the controller last asked for it (`cam_at`), f64 bits: where the setpoints stand.
    master: AtomicU64,
    limits: Limits,
}

impl Movers {
    fn n(&self) -> usize {
        self.start.len()
    }
    /// What a mover's body moves along the track: body and payload (the plate is the joint's rotor).
    pub fn moving(&self) -> f64 {
        self.body + if self.attached { self.payload } else { 0.0 }
    }
    /// What the rollers hold across the track: the whole mover.
    pub fn total(&self) -> f64 {
        self.moving() + self.plate_mass
    }

    /// Mover `k`'s figures at s, v, a: [lateral acceleration, lateral jerk, guide load, lift-off ratio].
    pub fn curve_loads(&self, s: f64, v: f64, a: f64) -> [f64; 4] {
        let (k, dk) = self.track.mean_curvature(s, self.wheelbase);
        let (lateral, m) = (v * v * k, self.total());
        let radial = self.attraction - m * lateral;
        [lateral, 2.0 * v * a * k + v * v * v * dk, radial.hypot(m * self.g), m * lateral / self.attraction]
    }

    /// The gap from mover `k` to the one ahead of it, m: along the centre line, less a body's length.
    pub fn gap(&self, q: &[f64], k: usize) -> f64 {
        let ahead = if k + 1 < self.n() { q[k + 1] } else { q[0] + self.track.length };
        ahead - q[k] - self.length
    }

    /// While the index cam stands mover `k` at a station: how far it is off the nearest station (its start plus whole
    /// station pitches, so that a pass the controller starts from a station further on counts the same); 0 while it
    /// moves.
    fn station_error(&self, q: &[f64], k: usize) -> f64 {
        match &self.cycle {
            Some(c) if c.kind == Kind::Index => {
                let set = c.at(k, self.start[k], f64::from_bits(self.master.load(Ordering::Relaxed)));
                let (pitch, off) = (c.cam.advance, q[k] - self.start[k]);
                if set[1] == 0.0 { (off - pitch * (off / pitch).round()).abs() } else { 0.0 }
            }
            _ => 0.0,
        }
    }

    /// Mover `k`'s point in the scene and the track's heading there.
    fn place(&self, s: f64) -> ([f64; 3], f64) {
        let (p, theta) = (self.track.point(s), self.track.heading(s));
        // Outside the loop: to the right of the direction of travel.
        let out = 0.035;
        ([p[0] + out * theta.sin(), p[1] - out * theta.cos(), 0.03], theta)
    }

    fn all_checks(&self) -> Vec<RatingCheck> {
        let check = |check, signal: String, motor, limit| RatingCheck { check, signal, motor, limit, life: false };
        let mut out = Vec::new();
        let index = self.cycle.as_ref().is_some_and(|c| c.kind == Kind::Index);
        for k in 0..self.n() {
            let id = k + 1;
            out.push(check("mover-guide-load", format!("mover.{id}.guide_load"), Some(k), self.limits.guide));
            out.push(check("curve-holding", format!("mover.{id}.liftoff_ratio"), Some(k), self.limits.liftoff));
            if index {
                out.push(check("station-settling", format!("mover.{id}.station_error"), Some(k), self.limits.station));
            }
        }
        let gap = if self.limits.gap.is_finite() { 1.0 } else { f64::INFINITY };
        out.push(check("mover-gap", "track.gap_ratio".into(), None, gap));
        out
    }
}

impl Mechanism for Movers {
    fn linear_motor(&self) -> bool {
        true
    }
    fn dynamics(&self, _q: &[f64], _qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let n = self.n();
        mass.fill(0.0);
        bias.fill(0.0);
        for k in 0..n {
            mass[k * n + k] = self.moving();
        }
    }
    fn potential(&self, _q: &[f64]) -> f64 {
        0.0
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }

    fn cam(&self) -> Option<CamMaster> {
        self.cycle.as_ref().map(|c| c.master)
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        let Some(cycle) = &self.cycle else { return };
        self.master.store(master.to_bits(), Ordering::Relaxed);
        for (k, o) in out.iter_mut().enumerate().take(self.n()) {
            *o = cycle.at(k, self.start[k], master);
        }
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for k in 0..self.n() {
            let (id, key) = (k + 1, MOVER_KEYS[k]);
            for (what, quantity, unit) in [
                ("track_position", "quantity-track-position", "m"),
                ("lateral_acceleration", "quantity-lateral-acceleration", "m/s²"),
                ("lateral_jerk", "quantity-lateral-jerk", "m/s³"),
                ("guide_load", "quantity-guide-load", "N"),
                ("liftoff_ratio", "quantity-liftoff-ratio", ""),
                ("gap_ahead", "quantity-gap-ahead", "m"),
                ("station_error", "quantity-station-error", "m"),
            ] {
                out.push(SignalInfo::new(format!("mover.{id}.{what}"), key, quantity, unit));
            }
        }
        out.push(SignalInfo::new("track.min_gap", "group-track", "quantity-min-gap", "m"));
        out.push(SignalInfo::new("track.gap_ratio", "group-track", "quantity-gap-ratio", ""));
        out
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let mut least = f64::INFINITY;
        for k in 0..self.n() {
            let [lateral, jerk, guide, lift] = self.curve_loads(q[k], qd[k], qdd[k]);
            let gap = self.gap(q, k);
            least = least.min(gap);
            out.extend([self.track.wrap(q[k]).0, lateral, jerk, guide, lift, gap, self.station_error(q, k)]);
        }
        let limit = if self.limits.gap.is_finite() { self.limits.gap } else { 0.0 };
        out.push(least);
        out.push(if least > 0.0 { (limit / least).min(1e9) } else { 1e9 });
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.all_checks().into_iter().filter(|c| c.limit.is_finite()).collect()
    }

    /// A pose that puts a mover on the one ahead of it (closer than a body's length along the loop) cannot be stood in.
    fn unreachable(&self, tool: &[f64], _slack: f64) -> Option<&'static str> {
        let n = self.n().min(tool.len());
        let behind = (0..n).any(|k| self.gap(tool, k) < 0.0);
        behind.then_some("error-target-out-of-range")
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &[sim_core::scene::MOTOR_PART, "payload"]
    }

    /// From above: the motor segments along the centre line (`MoverTrack::scene` shades them by their warmth), the
    /// movers outside it as the motor parts (their coils' warmth shows on them), each with its payload.
    fn scene(&self, q: &[f64], _rotors: &[f64], _rotor_speeds: &[f64], out: &mut Scene) {
        let t = &self.track;
        let at = |s: f64| {
            let p = t.point(s);
            [p[0], p[1], 0.0]
        };
        for j in 0..t.segments.len() {
            let (start, end) = t.segment(j);
            let strips = if t.is_curve(j) { CURVE_STRIPS } else { 1 };
            let colour = if j % 2 == 0 { [0.3, 0.32, 0.36] } else { [0.4, 0.42, 0.46] };
            for i in 0..strips {
                let (a, b) = (
                    start + (end - start) * i as f64 / strips as f64,
                    start + (end - start) * (i + 1) as f64 / strips as f64,
                );
                out.parts.push(Part::new("segment", Shape::Segment { to: at(b), width: 0.04 }, at(a), colour));
            }
        }
        let colours = [[0.85, 0.45, 0.1], [0.2, 0.5, 0.85], [0.25, 0.7, 0.3], [0.8, 0.25, 0.3], [0.6, 0.35, 0.75]];
        for (k, &s) in q.iter().enumerate().take(self.n()) {
            let (p, _) = self.place(s);
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.06, 0.06, 0.05] },
                p,
                colours[k % colours.len()],
            ));
            if self.attached && self.payload > 0.0 {
                out.parts.push(Part::new(
                    "payload",
                    Shape::Box { size: [0.04, 0.04, 0.04] },
                    [p[0], p[1], p[2] + 0.045],
                    [0.2, 0.7, 0.3],
                ));
            }
        }
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some(self.place(tool[0]).0)
    }
}

/// The machine: the movers' `JointMachine`, with the coils' currents and the segments' warmth around it.
pub struct MoverTrack {
    pub inner: JointMachine<Movers>,
    inner_base: usize,
    inner_memory: usize,
    kt: f64,
    resistance: f64,
    thermal_resistance: f64,
    time_constant: f64,
    /// The averaging time of the settled rise: one pass of the cycle (1 s without a cam).
    window: f64,
    current_limit: f64,
    rise_limit: f64,
    /// Per segment: its temperature rise, and the rise its loss of about the last pass settles at, K.
    pub rises: Vec<f64>,
    pub settled: Vec<f64>,
    /// The motor forces of the last `derivatives`: those of the step (`after_step` warms the segments with them).
    last_force: Mutex<Vec<f64>>,
}

impl MoverTrack {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let track = Track::new(
            cfg.positive("track.straight_m")?,
            cfg.positive("track.module_m")?,
            cfg.positive("track.curve_radius_m")?,
            cfg.non_negative("track.clothoid_m")?,
        )?;
        let start = cfg.f64s("start.pose")?;
        let n = start.len();
        let length = cfg.positive("mover.length_m")?;
        if n == 0
            || n > MAX_MOVERS
            || start.windows(2).any(|w| w[1] - w[0] < length)
            || start[n - 1] - start[0] > track.length - length
        {
            return Err(ConfigError::Invalid("start.pose".into()));
        }
        let mut joint = RotaryJoint::linear_from_config(cfg)?;
        let motor = LinearMotor::from_config(cfg)?;
        joint.ripple.cogging_order = motor.cogging_order();
        if cfg.bool("effects.joint_friction")? {
            let pressed = cfg.non_negative("linear_motor.guide_friction_coefficient")? * motor.attraction;
            joint.friction.coulomb += pressed;
            if joint.friction.presliding > 0.0 {
                joint.friction.stiction += pressed;
            }
        }
        // A closed track has no ends: no stops (slide.stroke_m is read, not used).
        joint.range = (-1e12, 1e12);
        let cycle = Cycle::from_config(cfg, &track, n)?;
        let rating = |key: &str| cfg.f64(key).unwrap_or(f64::INFINITY);
        let movers = Movers {
            body: cfg.positive("mass.mover_kg")?,
            payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            plate_mass: joint.rotor_inertia,
            attached: true,
            attraction: motor.attraction.max(1e-9),
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            wheelbase: cfg.non_negative("mover.wheelbase_m")?,
            length,
            plate: cfg.positive("mover.magnet_length_m")?,
            start,
            cycle: if cfg.bool("cam.run")? { Some(cycle) } else { None },
            master: AtomicU64::new(0f64.to_bits()),
            limits: Limits {
                guide: rating("ratings.mover_guide_max_load_n"),
                liftoff: rating("ratings.mover_max_liftoff_ratio"),
                gap: rating("ratings.min_gap_m"),
                station: rating("ratings.station_tolerance_m"),
            },
            track,
        };
        // Each mover's axis its own limits, `axis.s<k>.max_*`, as every machine's axes have theirs. Its range spans its
        // cycle's targets, BEHIND m behind and AHEAD m ahead of them (a cam runs on beyond it anyway): each mover's range
        // its own stretch of the loop, so that the middles of the ranges keep the movers in order and apart, and the
        // middles of the box's faces too (the shipped movers 0.103 m apart there); two poses in it are never a whole loop
        // apart, the same place on the track. Poses that put movers on each other are `unreachable`. The start pose
        // widens a range only as far as it lies outside it (a cam's start, a cycle cut short), without the reach: a start
        // pose on a range's end moves no range, as on every machine.
        let mut stretch: Vec<Vec<f64>> = cfg
            .array("cycle.segment")
            .map(|segments| {
                let targets = segments.iter().filter_map(|s| s.get("target")?.as_array());
                targets
                    .map(|t| t.iter().filter_map(|v| v.as_float()).collect())
                    .filter(|t: &Vec<f64>| t.len() == n)
                    .collect()
            })
            .unwrap_or_default();
        if stretch.is_empty() {
            stretch.push(movers.start.clone());
        }
        let reach = |k: usize, pick: fn(f64, f64) -> f64, by: f64| {
            pick(stretch.iter().map(|t| t[k]).reduce(pick).unwrap_or(0.0) + by, movers.start[k])
        };
        let get = |k: usize, what: &str, rate: &str| cfg.positive(&format!("axis.{}.max_{what}_m_{rate}", AXIS_IDS[k]));
        let axes = (0..n)
            .map(|k| -> Result<AxisInfo, ConfigError> {
                Ok(AxisInfo {
                    id: AXIS_IDS[k],
                    key: AXIS_KEYS[k],
                    unit: "m",
                    min: reach(k, f64::min, -BEHIND),
                    max: reach(k, f64::max, AHEAD),
                    max_vel: get(k, "velocity", "s")?,
                    max_acc: get(k, "acceleration", "s2")?,
                    max_jerk: get(k, "jerk", "s3")?,
                })
            })
            .collect::<Result<_, _>>()?;
        let specs = (0..n)
            .map(|k| JointSpec {
                drive: joint.clone(),
                id: MOTOR_IDS[k],
                key: MOVER_KEYS[k],
                gear_lever: None,
                unit: "m",
            })
            .collect();
        let motors = (0..n).map(|k| MotorInfo { id: MOTOR_IDS[k], key: MOTOR_KEYS[k] }).collect();
        let window = movers.cycle.as_ref().map_or(1.0, |c| c.master.period / c.master.speed);
        let inner = JointMachine::new(
            movers,
            specs,
            cfg.bool("effects.joint_elasticity")?,
            ThermalFeedback::from_config(cfg)?,
            axes,
            motors,
        );
        let segments = inner.mechanism.track.segments.len();
        let mut m = Self {
            inner_base: 0,
            inner_memory: inner.memory().len(),
            inner,
            kt: cfg.positive("motor.torque_constant_nm_a")?,
            resistance: cfg.non_negative("motor.resistance_ohm")?,
            thermal_resistance: cfg.positive("segment.thermal_resistance_k_w")?,
            time_constant: cfg.positive("segment.thermal_time_constant_s")?,
            window,
            current_limit: rating("ratings.segment_max_current_a"),
            rise_limit: rating("ratings.segment_max_rise_k"),
            rises: vec![0.0; segments],
            settled: vec![0.0; segments],
            last_force: Mutex::new(vec![0.0; n]),
        };
        let (mut state, mut torque) = (Vec::new(), vec![0.0; n]);
        m.inner.equilibrium(&m.inner.mechanism.start.clone(), &mut state, &mut torque);
        let mut signals = Vec::new();
        m.inner.base_signals(&state, &torque, &mut signals);
        m.inner_base = signals.len();
        Ok(m)
    }

    pub fn movers(&self) -> &Movers {
        &self.inner.mechanism
    }

    /// Per segment its current (A, the movers' |F| / k_t in their plates' shares on it) and its copper loss (W), at the
    /// movers' positions `q` and motor forces `force`.
    pub fn segment_load(&self, q: &[f64], force: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let t = &self.movers().track;
        let (mut current, mut loss) = (vec![0.0; t.segments.len()], vec![0.0; t.segments.len()]);
        for (k, f) in force.iter().enumerate() {
            let i = f / self.kt;
            for j in 0..t.segments.len() {
                let share = t.share(q[k], self.movers().plate, j);
                current[j] += share * i.abs();
                loss[j] += share * self.resistance * i * i;
            }
        }
        (current, loss)
    }

    fn positions(&self, s: &[f64]) -> Vec<f64> {
        let n = self.movers().n();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.inner.tool_state(s, &mut pos, &mut vel);
        pos
    }

    /// The segments judged by the highest of them (`track.max_segment_current`, `track.max_settled_rise`): one verdict
    /// each, as a check without a motor is one row of the verdicts.
    fn own_checks(&self) -> Vec<RatingCheck> {
        let check =
            |check, signal: &str, limit| RatingCheck { check, signal: signal.into(), motor: None, limit, life: false };
        [
            check("segment-current", "track.max_segment_current", self.current_limit),
            check("segment-temperature", "track.max_settled_rise", self.rise_limit),
        ]
        .into_iter()
        .filter(|c| c.limit.is_finite())
        .collect()
    }
}

impl Machine for MoverTrack {
    fn frame_momentum(&self, state: &[f64]) -> Option<FrameMomentum> {
        self.inner.frame_momentum(state)
    }
    fn base_shift(&self, tool_pose: &[f64], pivot: [f64; 3], shift: &[f64; 6], out: &mut [f64]) {
        self.inner.base_shift(tool_pose, pivot, shift, out)
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out = self.inner.rating_checks();
        out.extend(self.own_checks());
        out
    }
    fn axes(&self) -> &[AxisInfo] {
        self.inner.axes()
    }
    fn motors(&self) -> &[MotorInfo] {
        self.inner.motors()
    }
    fn motor_linear(&self, motor: usize) -> bool {
        self.inner.motor_linear(motor)
    }
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        self.inner.inverse(tool, motors)
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        self.inner.forward(motors, tool)
    }
    fn state_len(&self) -> usize {
        self.inner.state_len()
    }
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        self.inner.equilibrium(tool, state, torque)
    }
    fn derivatives(&self, state: &[f64], torque: &[f64], out: &mut [f64]) {
        if let Ok(mut last) = self.last_force.lock() {
            last.copy_from_slice(torque);
        }
        self.inner.derivatives(state, torque, out)
    }
    fn tool_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.tool_state(state, pos, vel)
    }
    fn set_tool_velocity(&self, state: &mut [f64], vel: &[f64]) {
        self.inner.set_tool_velocity(state, vel)
    }
    fn motor_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.motor_state(state, pos, vel)
    }
    fn feedforward_torque(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.inner.feedforward_torque(tool_pos, tool_acc, out)
    }
    fn feedforward_torque_moving(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.inner.feedforward_torque_moving(tool_pos, tool_vel, tool_acc, out)
    }
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.rigid_motor_torque(angle, speed, acc, out)
    }
    fn twist(&self, torque: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.twist(torque, acc, out)
    }
    fn rigid_split_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], rotor_acc: &[f64], out: &mut [f64]) {
        self.inner.rigid_split_torque(angle, speed, acc, rotor_acc, out)
    }
    fn rigid_tensions(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_tensions(tool_pos, tool_acc, out)
    }
    fn rigid_guide_loads(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_guide_loads(tool_pos, tool_acc, out)
    }
    fn steady_friction(&self, tool_pos: &[f64], tool_vel: &[f64], out: &mut [f64]) {
        self.inner.steady_friction(tool_pos, tool_vel, out)
    }
    fn rigid_friction(&self, tool_pos: &[f64], tool_dir: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_friction(tool_pos, tool_dir, out)
    }
    /// The inner machine's, but NaN for the station error (a mover's error while its cam stands, which no planned pose
    /// has), then NaN for each own check (the rigid model tells no segment's warmth).
    fn rigid_checks(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        let start = out.len();
        self.inner.rigid_checks(tool_pos, tool_vel, tool_acc, out);
        for (check, value) in self.inner.rating_checks().iter().zip(&mut out[start..]) {
            if check.check == "station-settling" {
                *value = f64::NAN;
            }
        }
        out.extend(self.own_checks().iter().map(|_| f64::NAN));
    }
    fn energy(&self, state: &[f64]) -> f64 {
        self.inner.energy(state)
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.inner.set_payload_attached(attached)
    }
    fn links(&self) -> Vec<String> {
        self.inner.links()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        self.inner.set_link_change(link, factor, moment)
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.inner.set_dwelling(dwelling)
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.inner.set_brake_engaged(engaged)
    }
    fn valves(&self) -> Vec<(String, f64)> {
        self.inner.valves()
    }
    fn set_valve(&mut self, valve: usize, command: f64) {
        self.inner.set_valve(valve, command)
    }
    fn end_sensors(&self, valve: usize) -> [bool; 2] {
        self.inner.end_sensors(valve)
    }
    /// The inner machine's, then each segment's first-order warmth: the rise moves towards R_th P over the thermal time
    /// constant, the settled rise over one pass, with the loss of the step's forces held over it (exact for a held
    /// input, as `sim` warms the parts).
    fn after_step(&mut self, state: &mut [f64], dt: f64) {
        self.inner.after_step(state, dt);
        let force = self.last_force.lock().map(|f| f.clone()).unwrap_or_default();
        if force.len() != self.movers().n() {
            return;
        }
        let (_, loss) = self.segment_load(&self.positions(state), &force);
        let (slow, fast) = (1.0 - (-dt / self.time_constant).exp(), 1.0 - (-dt / self.window).exp());
        for (j, p) in loss.iter().enumerate() {
            let settled = self.thermal_resistance * p;
            self.rises[j] += slow * (settled - self.rises[j]);
            self.settled[j] += fast * (settled - self.settled[j]);
        }
    }
    fn memory(&self) -> Vec<f64> {
        [self.inner.memory(), self.rises.clone(), self.settled.clone()].concat()
    }
    fn set_memory(&mut self, memory: &[f64]) {
        let (inner, own) = memory.split_at(self.inner_memory.min(memory.len()));
        self.inner.set_memory(inner);
        let n = self.rises.len();
        if own.len() == 2 * n {
            self.rises.copy_from_slice(&own[..n]);
            self.settled.copy_from_slice(&own[n..]);
        }
    }
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        self.inner.set_part_temperatures(rises)
    }
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.inner.resonance_hz(tool_pose)
    }
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        self.inner.mode_shape(tool_pose)
    }
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        self.inner.modes(tool_pose)
    }
    fn belt_wraps(&self, tool_pose: &[f64]) -> Vec<f64> {
        self.inner.belt_wraps(tool_pose)
    }
    fn unreachable(&self, tool_pose: &[f64], slack: f64) -> Option<&'static str> {
        self.inner.unreachable(tool_pose, slack)
    }
    fn condition(&self, tool_pose: &[f64]) -> Option<f64> {
        self.inner.condition(tool_pose)
    }
    fn setpoint_slack(&self) -> f64 {
        self.inner.setpoint_slack()
    }
    fn cam(&self) -> Option<CamMaster> {
        self.inner.cam()
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        self.inner.cam_at(master, out)
    }
    fn gantry_motors(&self) -> Option<[usize; 2]> {
        self.inner.gantry_motors()
    }
    /// The inner machine's base signals, then per segment its current, copper loss, temperature rise and settled rise,
    /// the highest current and settled rise of them, then the inner machine's effect signals.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut infos = self.inner.signal_infos();
        let own = (1..=self.rises.len()).flat_map(|id| {
            [
                ("current", "quantity-segment-current", "A"),
                ("copper_loss", "quantity-copper-loss", "W"),
                ("temperature_rise", "quantity-segment-rise", "K"),
                ("settled_rise", "quantity-segment-settled-rise", "K"),
            ]
            .map(|(what, quantity, unit)| {
                SignalInfo::new(format!("segment.{id}.{what}"), &format!("segment-{id}"), quantity, unit)
            })
        });
        let highest = [
            SignalInfo::new("track.max_segment_current", "group-track", "quantity-segment-current", "A"),
            SignalInfo::new("track.max_settled_rise", "group-track", "quantity-segment-settled-rise", "K"),
        ];
        infos.splice(self.inner_base..self.inner_base, own.chain(highest));
        infos
    }
    fn base_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.base_signals(state, torque, out);
        let (current, loss) = self.segment_load(&self.positions(state), torque);
        for j in 0..self.rises.len() {
            out.extend([current[j], loss[j], self.rises[j], self.settled[j]]);
        }
        out.push(current.iter().fold(0.0, |a: f64, b| a.max(*b)));
        out.push(self.settled.iter().fold(0.0, |a: f64, b| a.max(*b)));
    }
    fn effect_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.effect_signals(state, torque, out)
    }
    fn loss_powers(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.loss_powers(state, torque, out)
    }
    /// The movers' scene, each segment shaded by its temperature rise over its rating, and each mover's speed.
    fn scene(&self, state: &[f64], out: &mut Scene) {
        self.inner.scene(state, out);
        let t = &self.movers().track;
        let limit = if self.rise_limit.is_finite() { self.rise_limit } else { 60.0 };
        let shades: Vec<f32> = (0..t.segments.len())
            .flat_map(|j| {
                let shade = (self.rises[j] / limit).clamp(-1.0, 1.0) as f32;
                std::iter::repeat_n(shade, if t.is_curve(j) { CURVE_STRIPS } else { 1 })
            })
            .collect();
        out.parts.iter_mut().filter(|p| p.id == "segment").zip(shades).for_each(|(p, shade)| p.highlight = shade);
        let n = self.movers().n();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.inner.tool_state(state, &mut pos, &mut vel);
        for k in 0..n {
            let (p, _) = self.movers().place(pos[k]);
            out.labels.push(Label {
                pos: [p[0], p[1], p[2] + 0.1],
                owner_key: MOVER_KEYS[k],
                value: vel[k],
                unit_key: "unit-m-s",
                decimals: 2,
            });
        }
    }
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        self.inner.scene_point(tool_pose)
    }
    fn collision_bodies(&self, state: &[f64]) -> Vec<sim_core::Aabb> {
        self.inner.collision_bodies(state)
    }
    fn collision_capsules(&self, state: &[f64]) -> Vec<sim_core::Capsule> {
        self.inner.collision_capsules(state)
    }
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.inner.friction_contacts()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.inner.friction_curve(i, speed, load_scale)
    }
    fn belt_sections(&self) -> Vec<BeltSection> {
        self.inner.belt_sections()
    }
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.inner.thermal_parts()
    }
}

#[cfg(test)]
mod tests;
