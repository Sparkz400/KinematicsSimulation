//! Linear delta: three vertical toothed-belt axes on the columns of a frame at 120 degrees, each carriage carrying a
//! parallelogram of two rods down to a plate that stays level (Festo EXPT tripod, the Kossel printers). The tool is the
//! plate's centre, x, y, z.
//!
//! Kinematics per leg i at azimuth phi_i, with e = column radius - plate radius (the plate's offset folded into the
//! columns, as the plate never turns), the carriage's joint height h_i on its column and the rod length L:
//!
//! ```text
//! joint      c_i = (e cos phi_i, e sin phi_i, h_i)
//! constraint |p - c_i|^2 = L^2
//! inverse    h_i = z + sqrt(L^2 - rho_i^2),  rho_i^2 = (x - e cos phi_i)^2 + (y - e sin phi_i)^2   (carriage above the plate)
//! forward    p = the lower of the two points at L from all three joints (trilateration)
//! Jacobian   J_x p' = J_q h', J_x rows u_i = p - c_i, J_q = diag(u_i,z); p' = J h', J = J_x^-1 J_q
//!            J_x p'' = J_q h'' + w, w_i = -|p' - e_z h_i'|^2
//! ```
//!
//! Dynamics at the D3's level (Codourey 1998, Olsson 2009): the rods' rotational inertia is left out and each rod pair's
//! mass m_rod split, a share r at its carriage and 1 - r at the plate. The machine is three carriages plus a point mass
//! at the plate, m_nt = plate + payload + 3 (1 - r) m_rod, and by virtual work
//!
//! ```text
//! M(h) h'' = F_belt + Q_g - m_nt J^T J_x^-1 w - F_guide + F_stop,   M = m_c 1 + m_nt J^T J,   Q_g,i = -g m_c - m_nt g J_zi
//! m_c = carriage + r m_rod
//! ```
//!
//! Each column is a belt axis as on the one-axis machine, turned upright with its motor at the top: the motor pulley
//! at the top and an idler at the bottom cut the belt into section 1, from the motor pulley straight down to the carriage
//! (length span + stroke - x), and section 2, from the motor pulley down round the idler and up to the carriage
//! (3 span + stroke + x), x = h - h_min the carriage's place in its stroke. The belt pulls the carriage up with T1 - T2
//! and the pulley with r (T2 - T1). Elastic (effects.belt_stretch) the sections are springs and pulley and rotor states of
//! their own behind the coupling or gearbox; rigid, carriage, pulley and rotor turn together, x = r a, rotor = i a.
//! The guides are LuGre contacts (`Lugre`), loaded across the rail by the rods' pull.

use sim_components::mech::{Belt, Gearbox, HardStop, HoldingBrake, Lugre, Ripple, SectionForce, ShaftFriction};
use sim_core::{
    AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label,
    Machine, MachineEntry, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS,
};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "delta-linear",
    key: "machine-delta-linear",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 10] = [
    "belt_damping",
    "guide_friction",
    "load_dependent_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "temperature",
    "holding_brake",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(LinearDelta::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("hard_stop.damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload; the elastic ones with the belt's tooth stiffness
/// once with and once without.
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.belt_tooth_stiffness = {}", elastic && payload)).unwrap();
        out.push(cfg);
    }
    out
}

type V3 = [f64; 3];
type M3 = [[f64; 3]; 3];

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

fn mul(m: &M3, v: V3) -> V3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// The inverse of a 3 x 3 matrix by its adjugate; `None` where it is singular.
fn inverse(m: &M3) -> Option<M3> {
    let det = dot(m[0], cross(m[1], m[2]));
    if !(det.abs() > 1e-300) || !det.is_finite() {
        return None;
    }
    let (c0, c1, c2) = (cross(m[1], m[2]), cross(m[2], m[0]), cross(m[0], m[1]));
    Some(std::array::from_fn(|i| [c0[i] / det, c1[i] / det, c2[i] / det]))
}

/// The 2-norm condition number of a 3 x 3 matrix, as the spatial delta picker's (copied so that
/// the machine crates stay apart): the square root of the largest over the smallest eigenvalue of J^T J.
pub fn condition_number(j: &M3) -> f64 {
    let jt = transpose(j);
    let a: M3 = std::array::from_fn(|r| std::array::from_fn(|c| dot(jt[r], jt[c])));
    let q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
    let p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    let p2 = (0..3).map(|i| (a[i][i] - q).powi(2)).sum::<f64>() + 2.0 * p1;
    let p = (p2 / 6.0).sqrt();
    if p < 1e-15 * q.abs() {
        return 1.0;
    }
    let b: M3 = std::array::from_fn(|r| std::array::from_fn(|c| (a[r][c] - if r == c { q } else { 0.0 }) / p));
    let half_det = (dot(b[0], cross(b[1], b[2])) / 2.0).clamp(-1.0, 1.0);
    let phi = half_det.acos() / 3.0;
    let (largest, smallest) = (q + 2.0 * p * phi.cos(), q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos());
    if smallest <= 0.0 { f64::INFINITY } else { (largest / smallest).sqrt() }
}

/// The leg geometry and the kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Legs {
    /// Column radius less plate radius, rod length.
    pub e: f64,
    pub l: f64,
    /// cos and sin of the columns' azimuths, 0, 120 and 240 degrees.
    pub azimuth: [(f64, f64); 3],
}

impl Legs {
    pub fn new(e: f64, l: f64) -> Self {
        let azimuth = [0.0f64, 120.0, 240.0].map(|deg| (deg.to_radians().cos(), deg.to_radians().sin()));
        Self { e, l, azimuth }
    }

    /// Where column i's joint line stands, x and y.
    fn column(&self, i: usize) -> [f64; 2] {
        let (c, s) = self.azimuth[i];
        [self.e * c, self.e * s]
    }

    /// The horizontal distance of a plate position from column i's joint line.
    pub fn reach(&self, i: usize, p: V3) -> f64 {
        let [cx, cy] = self.column(i);
        (p[0] - cx).hypot(p[1] - cy)
    }

    fn joint(&self, i: usize, h: f64) -> V3 {
        let [cx, cy] = self.column(i);
        [cx, cy, h]
    }

    /// Carriage joint heights for a plate position; `None` out of the rods' reach.
    pub fn inverse(&self, p: V3) -> Option<V3> {
        let mut out = [0.0; 3];
        for (i, h) in out.iter_mut().enumerate() {
            let d = self.l * self.l - self.reach(i, p).powi(2);
            if !(d >= 0.0) {
                return None;
            }
            *h = p[2] + d.sqrt();
        }
        Some(out)
    }

    /// The plate position for carriage heights: the lower point at a rod's length from all three joints. `None` where
    /// the legs cannot be assembled.
    pub fn forward(&self, h: V3) -> Option<V3> {
        let c = [0, 1, 2].map(|i| self.joint(i, h[i]));
        let ex = sub(c[1], c[0]);
        let d = norm(ex);
        let ex = ex.map(|v| v / d);
        let to3 = sub(c[2], c[0]);
        let i = dot(ex, to3);
        let ey = sub(to3, ex.map(|v| v * i));
        let ey_len = norm(ey);
        if !(d > 1e-12 && ey_len > 1e-12) {
            return None;
        }
        let ey = ey.map(|v| v / ey_len);
        let ez = cross(ex, ey);
        let j = dot(ey, to3);
        let x = d / 2.0;
        let y = (i * i + j * j - 2.0 * i * x) / (2.0 * j);
        let z2 = self.l * self.l - x * x - y * y;
        if !(z2 >= 0.0) {
            return None;
        }
        let base: V3 = std::array::from_fn(|k| c[0][k] + x * ex[k] + y * ey[k]);
        let (up, down): (V3, V3) = (
            std::array::from_fn(|k| base[k] + z2.sqrt() * ez[k]),
            std::array::from_fn(|k| base[k] - z2.sqrt() * ez[k]),
        );
        Some(if up[2] < down[2] { up } else { down })
    }

    /// J_x (rows: the rod vectors u_i = p - c_i) and the diagonal of J_q (u_i . e_z).
    fn constraint(&self, h: V3, p: V3) -> (M3, V3) {
        let jx = [0, 1, 2].map(|i| sub(p, self.joint(i, h[i])));
        (jx, [0, 1, 2].map(|i| jx[i][2]))
    }

    /// The Jacobian J[i][j] = dp_i / dh_j at carriage heights h and plate position p; `None` at a direct singularity.
    pub fn jacobian(&self, h: V3, p: V3) -> Option<M3> {
        let (jx, jq) = self.constraint(h, p);
        let inv = inverse(&jx)?;
        Some(std::array::from_fn(|i| std::array::from_fn(|j| inv[i][j] * jq[j])))
    }

    /// J' h' = J_x^-1 w: the plate's acceleration at carriage speeds `hv` and no carriage acceleration.
    pub fn velocity_term(&self, h: V3, p: V3, hv: V3, pv: V3) -> Option<V3> {
        let (jx, _) = self.constraint(h, p);
        let inv = inverse(&jx)?;
        let w: V3 = std::array::from_fn(|i| {
            let rel = [pv[0], pv[1], pv[2] - hv[i]];
            -dot(rel, rel)
        });
        Some(mul(&inv, w))
    }

    /// The pull of each rod pair on its carriage that holds a plate of `m` kg at rest against gravity `g`, as the
    /// multiplier lambda_i of the rod vector u_i (force on carriage i: lambda_i u_i): J_x^T lambda = -m g e_z.
    pub fn rod_pull(&self, h: V3, p: V3, m: f64, g: f64) -> Option<V3> {
        let (jx, _) = self.constraint(h, p);
        Some(mul(&transpose(&inverse(&jx)?), [0.0, 0.0, -m * g]))
    }
}

#[derive(Clone)]
pub struct LinearDelta {
    legs: Legs,
    /// The geometry as configured, for the scene: column and plate radius, the rods' spacing.
    column_radius: f64,
    plate_radius: f64,
    rod_spacing: f64,
    /// The carriage joints' upper stop height and the stroke below it.
    top_stop: f64,
    stroke: f64,
    elastic: bool,
    g: f64,
    /// Carriage with its share of the rods; plate with the rest of them; payload.
    m_c: f64,
    m_plate: f64,
    m_payload: f64,
    payload_attached: bool,
    brake_engaged: bool,
    /// The belt axis on each column, all alike.
    r: f64,
    /// Unstretched belt fed per radian of pulley (the radius holds at pretension, as on the one-axis machine).
    r_mat: f64,
    span_min: f64,
    belt: Belt,
    /// Unstretched belt per section at x = 0 with the pulley at angle 0.
    mat0: [f64; 2],
    friction: Lugre,
    shaft: ShaftFriction,
    ripple: Ripple,
    gearbox: Gearbox,
    stops: HardStop,
    brake: HoldingBrake,
    /// Rotor with the gearbox's input; pulley with the belt's mass.
    j_rotor: f64,
    j_pulley: f64,
    /// The working range: a cylinder of `radius` from `bottom` to `top`; the rods' tilt and the condition number refused
    /// above.
    radius: f64,
    top: f64,
    max_tilt: f64,
    max_condition: f64,
    axes: [AxisInfo; 3],
    motors: [MotorInfo; 3],
    /// `Machine::setpoint_slack`, worked out once (`rest_sag`) when first asked for: its 600 equilibria take 8 ms, 50
    /// times what the rest of a build takes, and a run that plans no path (a loop measurement, a sweep) never asks.
    rest_slack: std::sync::OnceLock<f64>,
}

// Elastic state: [h 1..3, pulley angle 1..3, rotor angle 1..3, h' 1..3, pulley speed 1..3, rotor speed 1..3,
// friction 1..3, brake 1..3, slack]; slack: seconds a belt section has spent without tension, as on the other belt machines.
const N_ELASTIC: usize = 25;
// Rigid state: [h 1..3, h' 1..3, friction 1..3, brake 1..3].
const N_RIGID: usize = 12;
/// Loss powers per column: guide, motor shaft and brake, gearbox, belt; then the stops, and what the ripple gives off.
const N_LOSSES: usize = 14;

const SECTIONS: [&str; 6] = ["belt.1", "belt.2", "belt.3", "belt.4", "belt.5", "belt.6"];
const SECTION_KEYS: [&str; 6] = [
    "belt-section-1-1",
    "belt-section-1-2",
    "belt-section-2-1",
    "belt-section-2-2",
    "belt-section-3-1",
    "belt-section-3-2",
];
const CARRIAGES: [&str; 3] = ["carriage-1", "carriage-2", "carriage-3"];
const CONTACTS: [&str; 3] = ["carriage.1", "carriage.2", "carriage.3"];

/// What one moment of the rigid model gives per column: carriage acceleration, the torque into the gearbox (after the
/// rotor's own acceleration) and out of it at the pulley.
struct Rigid {
    acc: V3,
    into: V3,
    out: V3,
}

impl LinearDelta {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let column_radius = cfg.positive("geometry.column_radius_m")?;
        let plate_radius = cfg.non_negative("geometry.plate_radius_m")?;
        if plate_radius >= column_radius {
            return Err(ConfigError::Invalid("geometry.plate_radius_m".into()));
        }
        let l = cfg.positive("geometry.rod_m")?;
        let share = cfg.non_negative("mass.rod_carriage_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("mass.rod_carriage_share".into()));
        }
        let m_rod = cfg.non_negative("mass.rod_pair_kg")?;
        let (radius, top, bottom) =
            (cfg.positive("workspace.radius_m")?, cfg.f64("workspace.top_z_m")?, cfg.f64("workspace.bottom_z_m")?);
        if bottom >= top {
            return Err(ConfigError::Invalid("workspace.bottom_z_m".into()));
        }
        let max_tilt = cfg.positive("workspace.max_rod_angle_deg")?.to_radians();
        let r = cfg.positive("geometry.pulley_radius_m")?;
        let belt = Belt::from_config(cfg)?;
        let j_rotor = cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?;
        let axis = |id: &'static str, key: &'static str, min: f64, max: f64| -> Result<AxisInfo, ConfigError> {
            Ok(AxisInfo {
                id,
                key,
                unit: "m",
                min,
                max,
                max_vel: cfg.positive(&format!("axis.{id}.max_velocity_m_s"))?,
                max_acc: cfg.positive(&format!("axis.{id}.max_acceleration_m_s2"))?,
                max_jerk: cfg.positive(&format!("axis.{id}.max_jerk_m_s3"))?,
            })
        };
        let mut m = Self {
            legs: Legs::new(column_radius - plate_radius, l),
            column_radius,
            plate_radius,
            rod_spacing: cfg.non_negative("geometry.rod_spacing_m")?,
            top_stop: cfg.f64("geometry.carriage_top_m")?,
            stroke: cfg.positive("geometry.stroke_m")?,
            elastic: cfg.bool("effects.belt_stretch")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            m_c: cfg.positive("mass.carriage_kg")? + share * m_rod,
            m_plate: cfg.positive("mass.plate_kg")? + 3.0 * (1.0 - share) * m_rod,
            m_payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            payload_attached: true,
            brake_engaged: false,
            r,
            r_mat: belt.rest_length(r),
            span_min: cfg.positive("geometry.span_min_m")?,
            mat0: [0.0; 2],
            friction: Lugre::from_config(cfg, "friction")?,
            shaft: ShaftFriction::from_config(cfg)?,
            ripple: Ripple::from_config(cfg)?,
            gearbox: Gearbox::from_config(cfg)?,
            stops: HardStop::from_config(cfg)?,
            brake: HoldingBrake::from_config(cfg, j_rotor)?,
            j_rotor,
            j_pulley: cfg.positive("pulley.inertia_kgm2")?,
            belt,
            radius,
            top,
            max_tilt,
            max_condition: cfg.positive("workspace.max_condition_number")?,
            axes: [
                axis("x", "axis-x", -radius, radius)?,
                axis("y", "axis-y", -radius, radius)?,
                axis("z", "axis-z", bottom, top)?,
            ],
            motors: [
                MotorInfo { id: "1", key: "motor-column-1" },
                MotorInfo { id: "2", key: "motor-column-2" },
                MotorInfo { id: "3", key: "motor-column-3" },
            ],
            rest_slack: std::sync::OnceLock::new(),
        };
        let geo = m.geo(0.0);
        m.mat0 = geo.map(|g| m.belt.rest_length(g));
        // The whole belt runs at carriage speed, so all of its mass sits on the pulley.
        m.j_pulley += m.belt.mass_per_m * (geo[0] + geo[1] - 2.0 * m.belt.tooth_length) * r * r;
        Ok(m)
    }

    /// How far the tool can be from where the motors' angles put it at rest anywhere in the working range, payload on or
    /// off: the belt stretched by the weight it holds and the coupling or gearbox twisted (with half its backlash), as
    /// `equilibrium` sets them up, through the forward kinematics; over 5 radii, 12 directions and 5 heights. Plus a
    /// tenth for what the grid steps over; at least the default 0.1 mm.
    fn rest_sag(&mut self) -> f64 {
        let mut most = 0f64;
        let (bottom, top) = (self.axes[2].min, self.top);
        for attached in [true, false] {
            self.payload_attached = attached;
            for ring in 0..=4 {
                let rho = self.radius * ring as f64 / 4.0;
                for turn in 0..12 {
                    let (sin, cos) = (turn as f64 * std::f64::consts::TAU / 12.0).sin_cos();
                    for k in 0..=4 {
                        let p = [rho * cos, rho * sin, bottom + (top - bottom) * k as f64 / 4.0];
                        if self.unreachable(&p, 0.0).is_some() {
                            continue;
                        }
                        let (mut state, mut torque, mut motors, mut speed, mut tool) =
                            (Vec::new(), [0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]);
                        self.equilibrium(&p, &mut state, &mut torque);
                        self.motor_state(&state, &mut motors, &mut speed);
                        self.forward(&motors, &mut tool);
                        most = most.max(norm(sub(tool, p)));
                    }
                }
            }
        }
        self.payload_attached = true;
        (1.1 * most).max(1e-4)
    }

    pub fn legs(&self) -> &Legs {
        &self.legs
    }

    /// The point mass at the plate.
    pub fn m_nt(&self) -> f64 {
        self.m_plate + if self.payload_attached { self.m_payload } else { 0.0 }
    }

    /// A carriage with its share of the rods.
    pub fn m_carriage(&self) -> f64 {
        self.m_c
    }

    fn h_min(&self) -> f64 {
        self.top_stop - self.stroke
    }

    /// Geometric length of a column's two belt sections at x. Each ends at the motor pulley once, where the teeth in mesh
    /// count as more belt (`Belt::tooth_length`).
    fn geo(&self, x: f64) -> [f64; 2] {
        let (s, teeth) = (self.span_min, self.belt.tooth_length);
        [s + self.stroke - x + teeth, 3.0 * s + self.stroke + x + teeth]
    }

    /// Column k's two sections in the elastic state `s`.
    fn sections(&self, s: &[f64], k: usize) -> [SectionForce; 2] {
        let (geo, fed) = (self.geo(s[k] - self.h_min()), self.r_mat * s[3 + k]);
        let rate = self.r_mat * s[12 + k];
        [
            self.belt.section(geo[0], self.mat0[0] - fed, -s[9 + k], -rate),
            self.belt.section(geo[1], self.mat0[1] + fed, s[9 + k], rate),
        ]
    }

    fn heights(&self, s: &[f64]) -> (V3, V3) {
        let at = if self.elastic { 9 } else { 3 };
        ([s[0], s[1], s[2]], [s[at], s[at + 1], s[at + 2]])
    }

    fn frictions(&self, s: &[f64]) -> V3 {
        let at = if self.elastic { 18 } else { 6 };
        [s[at], s[at + 1], s[at + 2]]
    }

    fn brakes(&self, s: &[f64]) -> V3 {
        let at = if self.elastic { 21 } else { 9 };
        [s[at], s[at + 1], s[at + 2]]
    }

    /// Rotor angles and speeds of a state.
    fn rotors(&self, s: &[f64]) -> (V3, V3) {
        if self.elastic {
            ([s[6], s[7], s[8]], [s[15], s[16], s[17]])
        } else {
            let k = self.gearbox.ratio / self.r;
            (std::array::from_fn(|i| k * (s[i] - self.h_min())), std::array::from_fn(|i| k * s[3 + i]))
        }
    }

    /// Where the plate is and the Jacobian there, for carriage heights; NaN where the legs cannot be assembled.
    fn pose(&self, h: V3) -> (V3, M3) {
        self.legs
            .forward(h)
            .and_then(|p| Some((p, self.legs.jacobian(h, p)?)))
            .unwrap_or(([f64::NAN; 3], [[f64::NAN; 3]; 3]))
    }

    /// The load's mass matrix M (carriages and plate), the velocity term m_nt J^T J_x^-1 w and the generalised gravity
    /// Q_g at carriage heights h and speeds v.
    fn load(&self, h: V3, v: V3) -> (M3, V3, V3) {
        let (p, j) = self.pose(h);
        let jt = transpose(&j);
        let m = self.m_nt();
        let mass: M3 = std::array::from_fn(|r| {
            std::array::from_fn(|c| m * dot(jt[r], jt[c]) + if r == c { self.m_c } else { 0.0 })
        });
        let velocity =
            self.legs.velocity_term(h, p, v, mul(&j, v)).map_or([f64::NAN; 3], |a| mul(&jt, a.map(|x| m * x)));
        let gravity: V3 = std::array::from_fn(|k| -self.g * self.m_c - m * self.g * j[2][k]);
        (mass, velocity, gravity)
    }

    /// Each guide's load across its rail: the horizontal part of the rods' pull that holds the plate at rest.
    // shortcut: the pull at rest, without what accelerating the plate adds (the load would then depend on the
    // acceleration it changes through the friction); take the rods' dynamic pull once the guide's life is judged.
    fn guide_loads(&self, h: V3) -> V3 {
        let (p, _) = self.pose(h);
        let pull = self.legs.rod_pull(h, p, self.m_nt(), self.g).unwrap_or([f64::NAN; 3]);
        std::array::from_fn(|k| (pull[k] * self.legs.reach(k, p)).abs())
    }

    /// The friction levels' factor per guide: they are given for a guide under its preload alone.
    fn load_scales(&self, h: V3) -> V3 {
        self.guide_loads(h).map(|load| self.friction.load_scale(load, 0.0))
    }

    /// Force of the stops on carriage k.
    fn stop_force(&self, h: f64, v: f64) -> f64 {
        self.stops.force(h - self.h_min(), v, self.stroke)
    }

    /// The rigid model's moment. The gearbox loses torque only on what it transmits (the motor's less what accelerates
    /// the rotor itself); the second pass takes the acceleration of the first for that, as on the one-axis machine.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> Rigid {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (h, v) = self.heights(s);
        let (rotor, speed) = self.rotors(s);
        let (brakes, frictions, scales) = (self.brakes(s), self.frictions(s), self.load_scales(h));
        let (mass, velocity, gravity) = self.load(h, v);
        // Belt force the motor would give with a loss-free gearbox and a massless rotor; a brake holds the rotor.
        let drive: V3 = std::array::from_fn(|k| {
            ratio
                * (self.ripple.apply(torque[k], rotor[k])
                    - self.shaft.torque(speed[k])
                    - self.brake.torque(self.brake_engaged, speed[k], brakes[k]).0)
                / r
        });
        let rest: V3 = std::array::from_fn(|k| {
            gravity[k] - velocity[k] - self.friction.eval(v[k], frictions[k], scales[k]).0 + self.stop_force(h[k], v[k])
        });
        let rotor_mass = ratio * ratio * self.j_rotor / (r * r);
        let solve = |loss: V3| -> V3 {
            let m: M3 = std::array::from_fn(|i| {
                std::array::from_fn(|c| {
                    mass[i][c] + if i == c { self.j_pulley / (r * r) + loss[i] * rotor_mass } else { 0.0 }
                })
            });
            inverse(&m).map_or([f64::NAN; 3], |inv| mul(&inv, std::array::from_fn(|k| loss[k] * drive[k] + rest[k])))
        };
        let first = solve([1.0; 3]);
        let loss: V3 =
            std::array::from_fn(|k| self.gearbox.loss_factor((drive[k] - rotor_mass * first[k]) * r, v[k] / r));
        let acc = solve(loss);
        let into: V3 = std::array::from_fn(|k| (drive[k] - rotor_mass * acc[k]) * r);
        Rigid { acc, into, out: std::array::from_fn(|k| loss[k] * into[k]) }
    }

    /// The carriages' acceleration in either model.
    fn carriage_acceleration(&self, s: &[f64], torque: &[f64]) -> V3 {
        if self.elastic {
            let mut d = [0.0; N_ELASTIC];
            self.derivatives(s, torque, &mut d);
            [d[9], d[10], d[11]]
        } else {
            self.rigid_solve(s, torque).acc
        }
    }

    /// The torque each coupling or gearbox passes to its pulley.
    fn gear_output(&self, s: &[f64], torque: &[f64]) -> V3 {
        if self.elastic {
            std::array::from_fn(|k| self.gearbox.torque(s[6 + k], s[15 + k], s[3 + k], s[12 + k]).output)
        } else {
            self.rigid_solve(s, torque).out
        }
    }

    /// Each section's tension; in the rigid model the pretension.
    fn tensions(&self, s: &[f64]) -> [f64; 6] {
        std::array::from_fn(
            |i| if self.elastic { self.sections(s, i / 2)[i % 2].tension } else { self.belt.pretension },
        )
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (h, v) = self.heights(s);
        let (mass, ..) = self.load(h, v);
        let mut kinetic = 0.5 * dot(v, mul(&mass, v));
        let (p, _) = self.pose(h);
        let potential = self.g * (self.m_nt() * p[2] + self.m_c * (h[0] + h[1] + h[2]));
        let mut elastic: f64 = (0..3).map(|k| self.stops.energy(h[k] - self.h_min(), self.stroke)).sum();
        if self.elastic {
            for k in 0..3 {
                kinetic += 0.5 * (self.j_pulley * s[12 + k] * s[12 + k] + self.j_rotor * s[15 + k] * s[15 + k]);
                let (geo, fed) = (self.geo(h[k] - self.h_min()), self.r_mat * s[3 + k]);
                elastic += self.belt.energy(geo[0], self.mat0[0] - fed)
                    + self.belt.energy(geo[1], self.mat0[1] + fed)
                    + self.gearbox.energy(s[6 + k], s[3 + k]);
            }
        } else {
            let (r, ratio) = (self.r, self.gearbox.ratio);
            kinetic += 0.5 * (self.j_pulley + ratio * ratio * self.j_rotor) / (r * r) * dot(v, v);
        }
        [kinetic, potential, elastic]
    }

    /// The loss powers in the order of `effect_infos`, and per column the gearbox loss as a torque at the rotor and the
    /// brake torque.
    fn losses(&self, s: &[f64], torque: &[f64]) -> ([f64; N_LOSSES], V3, V3) {
        let (h, v) = self.heights(s);
        let (rotor, speed) = self.rotors(s);
        let (brakes, frictions, scales) = (self.brakes(s), self.frictions(s), self.load_scales(h));
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        let (mut powers, mut gear_at_rotor, mut held) = ([0.0; N_LOSSES], [0.0; 3], [0.0; 3]);
        let ratio = self.gearbox.ratio;
        for k in 0..3 {
            held[k] = self.brake.torque(self.brake_engaged, speed[k], brakes[k]).0;
            let guide = self.friction.eval(v[k], frictions[k], scales[k]).0 * v[k];
            let (gear, belt) = match &rigid {
                Some(m) => {
                    gear_at_rotor[k] = (m.into[k] - m.out[k]) / ratio;
                    (gear_at_rotor[k] * speed[k], 0.0)
                }
                None => {
                    let g = self.gearbox.torque(s[6 + k], s[15 + k], s[3 + k], s[12 + k]);
                    gear_at_rotor[k] = g.rotor - g.output / ratio;
                    let (geo, fed, rate) =
                        (self.geo(h[k] - self.h_min()), self.r_mat * s[3 + k], self.r_mat * s[12 + k]);
                    let belt = self.belt.loss_power(geo[0], self.mat0[0] - fed, -v[k], -rate)
                        + self.belt.loss_power(geo[1], self.mat0[1] + fed, v[k], rate);
                    (self.gearbox.loss_power(s[6 + k], s[15 + k], s[3 + k], s[12 + k]), belt)
                }
            };
            powers[4 * k..4 * k + 4].copy_from_slice(&[
                guide,
                (self.shaft.torque(speed[k]) + held[k]) * speed[k],
                gear,
                belt,
            ]);
            powers[12] += self.stops.loss_power(h[k] - self.h_min(), v[k], self.stroke);
            powers[13] += (torque[k] - self.ripple.apply(torque[k], rotor[k])) * speed[k];
        }
        (powers, gear_at_rotor, held)
    }

    /// Whether a pose is within the working range (cylinder from bottom to top), widened by `slack`.
    fn in_envelope(&self, p: &[f64], slack: f64) -> bool {
        p[0].hypot(p[1]) <= self.radius + slack && p[2] <= self.top + slack && p[2] >= self.axes[2].min - slack
    }

    /// The torque breakdown, loss powers, energies, friction signals and belt stretch of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (h, v) = self.heights(s);
        let (_, speed) = self.rotors(s);
        let (mass, velocity, gravity) = self.load(h, v);
        let acc = self.carriage_acceleration(s, torque);
        let moving = mul(&mass, acc);
        let (powers, gear_at_rotor, held) = self.losses(s, torque);
        let (frictions, scales) = (self.frictions(s), self.load_scales(h));
        // The transpose r / i takes a force at the carriage to the motor; pulley and rotor counted as a rigid machine
        // would turn them.
        let inertia = (self.j_pulley + ratio * ratio * self.j_rotor) / (r * r);
        for k in 0..3 {
            let parts = [
                r / ratio * (moving[k] + velocity[k] + inertia * acc[k]),
                -r / ratio * gravity[k],
                self.shaft.torque(speed[k])
                    + held[k]
                    + gear_at_rotor[k]
                    + r / ratio * self.friction.eval(v[k], frictions[k], scales[k]).0,
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(powers);
        out.extend(self.energy_parts(s));
        for k in 0..3 {
            out.push(v[k]);
            out.extend(self.friction.split(v[k], frictions[k], scales[k]));
            out.push(scales[k]);
        }
        // Stretch per section: its length less its unstretched belt; in the rigid model that of pretension.
        for i in 0..6 {
            let (k, side) = (i / 2, i % 2);
            let geo = self.geo(h[k] - self.h_min())[side];
            let mat = if self.elastic {
                self.mat0[side] + if side == 0 { -1.0 } else { 1.0 } * self.r_mat * s[3 + k]
            } else {
                self.belt.rest_length(geo)
            };
            out.push(geo - mat);
        }
    }

    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        for motor in &self.motors {
            out.extend(TORQUE_PARTS.map(|part| {
                SignalInfo::new(
                    format!("motor.{}.torque_{part}", motor.id),
                    motor.key,
                    &format!("quantity-torque-{part}"),
                    "N m",
                )
            }));
        }
        for (k, motor) in self.motors.iter().enumerate() {
            let gear =
                if self.gearbox.is_coupling() { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}guide_{}", motor.id),
                CARRIAGES[k],
                "quantity-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", motor.id),
                motor.key,
                "quantity-shaft-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"));
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}belt_{}", motor.id),
                CARRIAGES[k],
                "quantity-damping-power",
                "W",
            ));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-hard_stop", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        for (k, contact) in CONTACTS.iter().enumerate() {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "m/s",
                    "load_scale" => "",
                    _ => "N",
                };
                out.push(SignalInfo::new(
                    format!("{contact}.friction_{part}"),
                    CARRIAGES[k],
                    &format!("quantity-friction-{}", part.replace('_', "-")),
                    unit,
                ));
            }
        }
        out.extend(
            SECTIONS
                .iter()
                .zip(SECTION_KEYS)
                .map(|(id, key)| SignalInfo::new(format!("{id}.stretch"), key, "quantity-section-stretch", "m")),
        );
    }

    /// The plate at `p` and the payload under it, as `scene` draws them.
    fn on_plate(&self, p: V3) -> Vec<Part> {
        let size = 2.4 * self.plate_radius.max(0.03);
        let mut parts = vec![Part::new("plate", Shape::Box { size: [size, size, 0.03] }, p, [0.55, 0.55, 0.6])];
        if self.payload_attached && self.m_payload > 0.0 {
            parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.1, 0.1, 0.08] },
                [p[0], p[1], p[2] - 0.06],
                [0.2, 0.7, 0.3],
            ));
        }
        parts
    }
}

impl Machine for LinearDelta {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// NaN where the pose is out of reach: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let h = self.legs.inverse([tool[0], tool[1], tool[2]]).unwrap_or([f64::NAN; 3]);
        for k in 0..3 {
            motors[k] = self.gearbox.ratio * (h[k] - self.h_min()) / self.r;
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let h = std::array::from_fn(|k| self.h_min() + motors[k] * self.r / self.gearbox.ratio);
        tool[..3].copy_from_slice(&self.legs.forward(h).unwrap_or([f64::NAN; 3]));
    }

    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    /// Each carriage held by its belt against its weight and the rods' share of the plate's, Q_g; elastic, each column's
    /// sections share their belt so that T1 - T2 is that force, as on the one-axis machine.
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let h = self.legs.inverse([tool[0], tool[1], tool[2]]).unwrap_or([f64::NAN; 3]);
        let (_, _, gravity) = self.load(h, [0.0; 3]);
        let (r, ratio) = (self.r, self.gearbox.ratio);
        state.clear();
        state.extend(h);
        if !self.elastic {
            state.resize(N_RIGID, 0.0);
            for k in 0..3 {
                let angle = ratio * (h[k] - self.h_min()) / r;
                torque[k] = self.ripple.invert(-(gravity[k] + self.stop_force(h[k], 0.0)) * r / ratio, angle);
            }
            return;
        }
        let (mut pulleys, mut rotors) = ([0.0; 3], [0.0; 3]);
        for k in 0..3 {
            let force = -(gravity[k] + self.stop_force(h[k], 0.0));
            let geo = self.geo(h[k] - self.h_min());
            let mat = self.belt.static_material(&geo, &[force, 0.0], self.mat0.iter().sum());
            pulleys[k] = (self.mat0[0] - mat[0]) / self.r_mat;
            let m: [f64; 2] = std::array::from_fn(|i| self.belt.section(geo[i], mat[i], 0.0, 0.0).material);
            // The pulley at rest: what the coupling passes on balances the belt, r (m2 - m1).
            let gear = -self.r_mat * (m[1] - m[0]);
            rotors[k] = ratio * (pulleys[k] + self.gearbox.static_twist(gear));
            torque[k] = self.ripple.invert(gear / ratio, rotors[k]);
        }
        state.extend(pulleys);
        state.extend(rotors);
        state.resize(N_ELASTIC, 0.0);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let (h, v) = self.heights(s);
        let (_, speed) = self.rotors(s);
        let (brakes, frictions, scales) = (self.brakes(s), self.frictions(s), self.load_scales(h));
        if !self.elastic {
            let acc = self.rigid_solve(s, torque).acc;
            out[..3].copy_from_slice(&v);
            out[3..6].copy_from_slice(&acc);
            for k in 0..3 {
                out[6 + k] = self.friction.eval(v[k], frictions[k], scales[k]).1;
                out[9 + k] = self.brake.torque(self.brake_engaged, speed[k], brakes[k]).1;
            }
            return;
        }
        let (mass, velocity, gravity) = self.load(h, v);
        let mut rhs = [0.0; 3];
        out[24] = 0.0;
        for k in 0..3 {
            let sec = self.sections(s, k);
            if sec.iter().any(|f| f.tension <= 0.0) {
                out[24] = 1.0;
            }
            let (ff, ds) = self.friction.eval(v[k], frictions[k], scales[k]);
            rhs[k] = sec[0].tension - sec[1].tension + gravity[k] - velocity[k] - ff + self.stop_force(h[k], v[k]);
            let gear = self.gearbox.torque(s[6 + k], s[15 + k], s[3 + k], s[12 + k]);
            let (brake, brake_rate) = self.brake.torque(self.brake_engaged, s[15 + k], brakes[k]);
            out[3 + k] = s[12 + k];
            out[6 + k] = s[15 + k];
            out[12 + k] = (gear.output + self.r_mat * (sec[1].material - sec[0].material)) / self.j_pulley;
            out[15 + k] = (self.ripple.apply(torque[k], s[6 + k]) - gear.rotor - self.shaft.torque(s[15 + k]) - brake)
                / self.j_rotor;
            out[18 + k] = ds;
            out[21 + k] = brake_rate;
        }
        out[..3].copy_from_slice(&v);
        out[9..12].copy_from_slice(&inverse(&mass).map_or([f64::NAN; 3], |inv| mul(&inv, rhs)));
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (h, v) = self.heights(s);
        let (p, j) = self.pose(h);
        pos[..3].copy_from_slice(&p);
        vel[..3].copy_from_slice(&mul(&j, v));
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (h, _) = self.heights(s);
        let (_, j) = self.pose(h);
        let v = inverse(&j).map_or([f64::NAN; 3], |inv| mul(&inv, [vel[0], vel[1], vel[2]]));
        if self.elastic {
            for k in 0..3 {
                (s[9 + k], s[12 + k], s[15 + k]) = (v[k], v[k] / self.r, self.gearbox.ratio * v[k] / self.r);
            }
        } else {
            s[3..6].copy_from_slice(&v);
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..3].copy_from_slice(&rotor);
        vel[..3].copy_from_slice(&speed);
    }

    /// Rigid, loss-free, at rest: (M h'' - Q_g + J_pulley / r^2 h'') r / i at the motor, plus the rotor's own
    /// J_rotor i h'' / r.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        self.feedforward_torque_moving(pos, &[0.0; 3], acc, out);
    }

    /// With the speed: h' = J^-1 p', h'' = J^-1 (p'' - J_x^-1 w), and at the carriage m_c h'' + m_nt J^T p'' - Q_g (which
    /// is M h'' + m_nt J^T J_x^-1 w - Q_g), plus pulley and rotor.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let p = [pos[0], pos[1], pos[2]];
        let h = self.legs.inverse(p).unwrap_or([f64::NAN; 3]);
        let (_, _, gravity) = self.load(h, [0.0; 3]);
        let (_, j) = self.pose(h);
        let inv = inverse(&j).unwrap_or([[f64::NAN; 3]; 3]);
        let (pv, pa) = ([vel[0], vel[1], vel[2]], [acc[0], acc[1], acc[2]]);
        let v = mul(&inv, pv);
        let bend = self.legs.velocity_term(h, p, v, pv).unwrap_or([f64::NAN; 3]);
        let carriage_acc = mul(&inv, std::array::from_fn(|i| pa[i] - bend[i]));
        let force = transpose(&j).map(|column| self.m_nt() * dot(column, pa));
        let (r, ratio) = (self.r, self.gearbox.ratio);
        for k in 0..3 {
            let at_carriage = (self.m_c + self.j_pulley / (r * r)) * carriage_acc[k] + force[k] - gravity[k];
            out[k] = at_carriage * r / ratio + self.j_rotor * ratio * carriage_acc[k] / r;
        }
    }

    /// Per column, F = m_c h'' + m_nt (J^T p'')_k - Q_g,k on the carriage (without pulley and rotor), T1 - T2 = F, and the
    /// belt keeps its length: (T1 - P) L1 + (T2 - P) L2 = 0. So T1 = P + F L2 / (L1 + L2), T2 = P - F L1 / (L1 + L2).
    fn rigid_tensions(&self, pos: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let p = [pos[0], pos[1], pos[2]];
        let h = self.legs.inverse(p).unwrap_or([f64::NAN; 3]);
        let (_, _, gravity) = self.load(h, [0.0; 3]);
        let (_, j) = self.pose(h);
        let pa = [acc[0], acc[1], acc[2]];
        let carriage_acc = inverse(&j).map_or([f64::NAN; 3], |inv| mul(&inv, pa));
        let force = transpose(&j).map(|column| self.m_nt() * dot(column, pa));
        let pre = self.belt.pretension;
        for k in 0..3 {
            let f = self.m_c * carriage_acc[k] + force[k] - gravity[k];
            let [l1, l2] = self.geo(h[k] - self.h_min());
            out.extend([pre + f * l2 / (l1 + l2), pre - f * l1 / (l1 + l2)]);
        }
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    /// As `Machine::adopt_state`, the same tool pose and speed, but what the belts, couplings, guides and brakes were
    /// doing is kept: rebuilt at rest, the belts dropped what they carried beyond the weight (the guides' friction and
    /// the plate's pull, 34 N of tension at 3 m/s) and the guides' bristles their deflection. Each column's section 1
    /// keeps its stretch (section 2 then its own, as the belt keeps its length) and its coupling the torque it passed.
    fn adopt_state(&self, old: &dyn Machine, old_state: &[f64]) -> Vec<f64> {
        let (mut pos, mut vel, mut before, mut after) = ([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]);
        old.tool_state(old_state, &mut pos, &mut vel);
        old.inverse(&pos, &mut before);
        self.inverse(&pos, &mut after);
        if old_state.len() == self.state_len() && before == after {
            return old_state.to_vec();
        }
        let mut state = Vec::new();
        self.equilibrium(&pos, &mut state, &mut [0.0; 3]);
        self.set_tool_velocity(&mut state, &vel);
        // The old machine is a linear delta too (`Sim::reconfigure` keeps the entry), elastic or rigid: its friction and
        // brake states.
        let was_elastic = old_state.len() == N_ELASTIC;
        let (from, to) = (if was_elastic { 18 } else { 6 }, if self.elastic { 18 } else { 6 });
        state[to..to + 6].copy_from_slice(&old_state[from..from + 6]);
        if self.elastic && was_elastic {
            state[24] = old_state[24];
            let (infos, mut row) = (old.signal_infos(), Vec::new());
            old.signals(old_state, &[0.0; 3], &mut row);
            let signal = |id: String| row[infos.iter().position(|s| s.id == id).unwrap()];
            for k in 0..3 {
                let geo = self.geo(state[k] - self.h_min())[0];
                state[3 + k] = (self.mat0[0] - geo + signal(format!("{}.stretch", SECTIONS[2 * k]))) / self.r_mat;
                state[6 + k] = self.gearbox.ratio
                    * (state[3 + k] + self.gearbox.static_twist(signal(format!("gearbox.{}.output_torque", k + 1))));
            }
        }
        state
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    /// What the belts stretch and the couplings twist at rest anywhere in the working range (`rest_sag`).
    fn setpoint_slack(&self) -> f64 {
        // On a copy: `rest_sag` switches the payload, and nothing else it reads changes after `new`.
        *self.rest_slack.get_or_init(|| self.clone().rest_sag())
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if tool.len() != 3 {
            return Some("error-target-out-of-range");
        }
        let p = [tool[0], tool[1], tool[2]];
        if !self.in_envelope(&p, slack) {
            return Some("error-delta-outside-envelope");
        }
        let Some(h) = self.legs.inverse(p) else { return Some("error-delta-out-of-reach") };
        if (0..3).any(|k| self.legs.reach(k, p) > self.legs.l * self.max_tilt.sin()) {
            return Some("error-delta-linear-rod-angle");
        }
        if h.iter().any(|h| *h < self.h_min() || *h > self.top_stop) {
            return Some("error-delta-linear-carriage-stop");
        }
        match self.legs.jacobian(h, p) {
            Some(j) if condition_number(&j) <= self.max_condition => None,
            _ => Some("error-delta-singularity"),
        }
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, motor) in self.motors.iter().enumerate() {
            out.push(SignalInfo::new(
                format!("carriage.{}.height", motor.id),
                CARRIAGES[k],
                "quantity-carriage-height",
                "m",
            ));
            out.push(SignalInfo::new(
                format!("{}.tension", SECTIONS[2 * k]),
                SECTION_KEYS[2 * k],
                "quantity-tension",
                "N",
            ));
            out.push(SignalInfo::new(
                format!("{}.tension", SECTIONS[2 * k + 1]),
                SECTION_KEYS[2 * k + 1],
                "quantity-tension",
                "N",
            ));
            out.push(SignalInfo::new(
                format!("pulley.{}.radial_load", motor.id),
                motor.key,
                "quantity-radial-load",
                "N",
            ));
            out.push(SignalInfo::new(
                format!("carriage.{}.guide_load", motor.id),
                CARRIAGES[k],
                "quantity-guide-load",
                "N",
            ));
            out.push(SignalInfo::new(
                format!("gearbox.{}.output_torque", motor.id),
                motor.key,
                "quantity-gearbox-torque",
                "N m",
            ));
        }
        out.push(SignalInfo::new("kinematics.condition", "group-kinematics", "quantity-condition-number", ""));
        out.push(SignalInfo::new("belt.slack_time", "group-belt", "quantity-slack-time", "s"));
        self.effect_infos(&mut out);
        out
    }

    /// The motor pulley takes half a turn of belt: both sections pull its shaft the same way, so the radial load is
    /// the sum of their tensions.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (h, _) = self.heights(s);
        let (tensions, gear, guides) = (self.tensions(s), self.gear_output(s, torque), self.guide_loads(h));
        for k in 0..3 {
            out.extend([
                h[k],
                tensions[2 * k],
                tensions[2 * k + 1],
                tensions[2 * k] + tensions[2 * k + 1],
                guides[k],
                gear[k],
            ]);
        }
        let (_, j) = self.pose(h);
        out.push(condition_number(&j));
        out.push(if self.elastic { s[24] } else { 0.0 });
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
    }

    fn friction_contacts(&self) -> Vec<FrictionContact> {
        (0..3).map(|k| FrictionContact { id: CONTACTS[k], owner_key: CARRIAGES[k] }).collect()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        if i < 3 { self.friction.steady(speed, load_scale) } else { [0.0; 3] }
    }

    /// Two sections per column, in the order of the signals `belt.<i>.tension`: column 1's to the carriage from above and
    /// round the idler, then column 2's and 3's.
    fn belt_sections(&self) -> Vec<BeltSection> {
        SECTIONS
            .iter()
            .zip(SECTION_KEYS)
            .map(|(id, owner_key)| BeltSection { id, owner_key, pretension: self.belt.pretension })
            .collect()
    }

    // shortcut: no part warms with its loss (`thermal_parts` empty): the couplings of the shipped direct drives lose
    // nothing, and the guides' heat is not modelled; add the guides as parts like the one-axis machine's once a guide's
    // temperature matters here.

    /// Each column's belt loop in the vertical plane through the column, seen from outside: the motor at the top, an
    /// idler at the bottom, the carriage on the front run. The scene helpers draw in an x-z plane at a depth y, so each
    /// loop is drawn in the plane parallel to x-z through its column, a little in front of it.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        use sim_core::scene::{belt, drive_unit, idler};
        let (h, _) = self.heights(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let (p, _) = self.pose(h);
        let r = self.r;
        let tensions = self.tensions(s);
        out.parts.clear();
        out.labels.clear();
        let grey = [0.62, 0.64, 0.68];
        let (top, bottom) = (self.top_stop + self.span_min, self.h_min() - self.span_min);
        let rim = self.column_radius + 0.08;
        out.parts.push(Part::new(
            "frame",
            Shape::Box { size: [2.0 * rim, 2.0 * rim, 0.04] },
            [0.0, 0.0, top + 0.12],
            grey,
        ));
        for k in 0..3 {
            let (c, sn) = self.legs.azimuth[k];
            let (cx, cy) = (self.column_radius * c, self.column_radius * sn);
            out.parts.push(Part::new(
                "column",
                Shape::Box { size: [0.08, 0.08, top - bottom + 0.2] },
                [cx * rim / self.column_radius, cy * rim / self.column_radius, 0.5 * (top + bottom)],
                grey,
            ));
            let pulley = if self.elastic { s[3 + k] } else { rotor[k] / self.gearbox.ratio };
            // The belt's plane, in front of the column's centre; x across it, z up.
            let y = cy - 0.05;
            let shade = [0, 1].map(|i| {
                ((tensions[2 * k + i] - self.belt.pretension) / self.belt.pretension.max(1.0)).clamp(-1.0, 1.0) as f32
            });
            // A positive pulley angle lifts the carriage on the run at x = cx + r: the pulley turns anticlockwise seen
            // from the front.
            drive_unit(&mut out.parts, [cx, top], y, r, pulley, rotor[k], [0.9, 0.45, 0.1]);
            idler(&mut out.parts, [cx, bottom], y, r, pulley);
            let path = [
                [cx + r, h[k]],
                [cx + r, top],
                [cx, top + r],
                [cx - r, top],
                [cx - r, bottom],
                [cx, bottom - r],
                [cx + r, bottom],
                [cx + r, h[k]],
            ];
            belt(&mut out.parts, &path, y, 0.025, 0.003 * 8.0, |i| shade[if i < 2 { 0 } else { 1 }]);
            let joint = [cx, cy, h[k]];
            out.parts.push(Part::new(
                "carriage",
                Shape::Box { size: [0.12, 0.06, 0.1] },
                [cx + r, cy - 0.02, h[k]],
                [0.2, 0.22, 0.26],
            ));
            // The two rods of the pair, apart along the carriage's width (tangential to the frame).
            let half = [-0.5 * self.rod_spacing * sn, 0.5 * self.rod_spacing * c, 0.0];
            let plate_end = [p[0] + self.plate_radius * c, p[1] + self.plate_radius * sn, p[2]];
            for side in [-1.0, 1.0] {
                let from = std::array::from_fn(|i| joint[i] + side * half[i]);
                let to = std::array::from_fn(|i| plate_end[i] + side * half[i]);
                out.parts.push(Part::new("rod", Shape::Segment { to, width: 0.012 }, from, [0.15, 0.15, 0.15]));
            }
            out.labels.push(Label {
                pos: [cx, cy, top + 0.25],
                owner_key: self.motors[k].key,
                value: rotor_speed[k] * 60.0 / std::f64::consts::TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        out.parts.extend(self.on_plate(p));
        out.labels.push(Label {
            pos: [p[0], p[1], p[2] - 0.15],
            owner_key: "axis-z",
            value: p[2],
            unit_key: "unit-m",
            decimals: 3,
        });
    }

    /// The plate's centre: the machine's frame is the scene's.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([tool_pose[0], tool_pose[1], tool_pose[2]])
    }

    /// The plate and the payload under it, where `scene` draws them; the rods are left out.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let (h, _) = self.heights(s);
        sim_core::scene::scene_boxes(&self.on_plate(self.pose(h).0), &["plate", "payload"])
    }
}

#[cfg(test)]
mod tests;
