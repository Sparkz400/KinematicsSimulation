//! A rotary indexing table: a driver turned by the motor through its gearbox steps a table of n stations on by a
//! station per turn of the driver, and holds it still in between (the dwell) while the stations work. Two drives:
//!
//! - `geneva-drive`: a Geneva (Maltese cross) wheel of n radial slots, a pin on a crank of radius a entering a slot
//!   radially at a centre distance C = a / sin(pi / n), and the locking disc on the crank holding the wheel while the
//!   pin is out.
//! - `cam-indexer`: a cam index drive (a globoid or roller-gear cam turning the turret's rollers), whose cam moves the
//!   turret by its motion law, here the modified sine, over `indexer.index_angle_deg` of the cam's turn and holds it by
//!   the cam's dwell ridge over the rest.
//!
//! The joint is the driver's angle theta (rad), 0 where the index is half done (the Geneva's pin deepest in its slot).
//! The index runs over |theta - 2 pi k| < h: the Geneva's half angle h = pi / 2 - pi / n, so it moves over (n - 2) /
//! (2 n) of the driver's turn and dwells over (n + 2) / (2 n); the cam's h is half its index angle.
//!
//! Geneva, with lambda = a / C = sin(pi / n) and x = theta - 2 pi k (Hartenberg & Denavit 1964, VDI 2727):
//!
//!   tan(phi) = lambda sin(x) / (1 - lambda cos(x))
//!   phi'  = lambda (cos(x) - lambda) / (1 - 2 lambda cos(x) + lambda^2)          (d phi / d theta)
//!   phi'' = -lambda (1 - lambda^2) sin(x) / (1 - 2 lambda cos(x) + lambda^2)^2
//!
//! The wheel starts and stops at rest (phi' = 0 at x = -+h), but with phi'' = tan(pi / n) at once: the acceleration
//! jumps, the jerk is infinite. The torque on the wheel steps there by J tan(pi / n) theta'^2, and so does the pin's
//! force on the slot's flank. The driver does not feel the step: its torque is J phi' phi'' theta'^2, and phi' is 0
//! there. The pin is r = C sqrt(1 - 2 lambda cos(x) + lambda^2) from the wheel's centre and pushes across the radial slot
//! with F = T / r.
//!
//! Cam index drive, u = (x + h) / (2 h) over the index: phi = (2 pi / n) s(u) with the modified sine s (`modified_sine`),
//! its peak acceleration 5.528 and speed 1.760 (times the step over (2 h)^2 and 2 h), no acceleration step at either
//! end. The cam's rollers on the turret's pitch radius r_p take F = T / r_p (shortcut: along the turret's tangent, the
//! cam's pressure angle left out; the makers' load tables take it in).
//!
//! The wheel (or turret), the table and the parts on it (`mass.payload_kg` at `table.station_radius_m`,
//! `effects.payload`) turn with phi: J(theta) = J_driver + J_t phi'^2, J_t = J_wheel + J_table + m r_s^2, and the speed
//! term J_t phi' phi'' theta'^2. Lossless and rigid: the table is horizontal and nothing but the driver's bearings
//! (`[joint]`) takes energy.
//!
//! In the dwell the locking disc (the cam's dwell ridge) holds the wheel within its play (`indexer.locking_play_rad`):
//! the table's index accuracy is half that play at the station radius, whatever the motor does. The table rings on
//! its shaft (`table.stiffness_nm_rad`, `table.damping_ratio`) after each index: the twist x of the table against the
//! wheel follows x'' + 2 zeta w_n x' + w_n^2 x = -phi'' theta'^2 through the index at the driver's speed, and what is
//! left at its end, A, dies as exp(-zeta w_n t): the table settles to `ratings.index_settling_tolerance_m` at the station
//! radius ln(A / tol) / (zeta w_n) after the index. The Geneva's step in acceleration at the end leaves A = tan(pi / n)
//! theta'^2 / w_n^2; the modified sine's smooth end far less.
//! shortcut: the settling is worked out at the driver's speed at each moment of the index, as if it ran the whole index
//! at that speed (exact under `cam.run`, an estimate on a servo cycle that speeds up through the index), and the ring
//! does not act back on the driver; model the table's twist as a joint of its own if the drive's loop must see it.

use crate::{axis, machine, master, payload};
use sim_components::cam::{Cam, Law, Segment};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_core::{CamMaster, Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::{PI, TAU};

/// The modified sine over u in [0, 1]: its lift s, ds/du and d2s/du2 (VDI 2143; peak d2s/du2 4 pi^2 / (4 + pi) =
/// 5.528 at u = 1/8 and 7/8, peak ds/du 4 pi / (4 + pi) = 1.760 at u = 1/2): a quarter sine of acceleration over the
/// first and last eighth, a half sine of a third of its frequency between.
pub fn modified_sine(u: f64) -> [f64; 3] {
    let k = 1.0 / (4.0 + PI);
    let ends = |base: f64| {
        let a = 4.0 * PI * u;
        [k * (base + PI * u - 0.25 * a.sin()), k * (PI - PI * a.cos()), k * 4.0 * PI * PI * a.sin()]
    };
    if u <= 0.125 {
        ends(0.0)
    } else if u >= 0.875 {
        ends(4.0)
    } else {
        let a = PI / 3.0 + 4.0 * PI * u / 3.0;
        [k * (2.0 + PI * u - 2.25 * a.sin()), k * (PI - 3.0 * PI * a.cos()), k * 4.0 * PI * PI * a.sin()]
    }
}

/// How the driver moves the wheel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Drive {
    /// A Geneva wheel: lambda = a / C = sin(pi / n), and the crank's radius a.
    Geneva { lambda: f64, crank_radius: f64 },
    /// A cam index drive with the modified sine, its rollers on `pitch_radius`.
    Cam { pitch_radius: f64 },
}

pub struct Indexer {
    pub drive: Drive,
    pub stations: usize,
    /// Half the driver's angle over which the index runs, rad.
    pub half: f64,
    pub j_driver: f64,
    /// The wheel's (turret's) and the table's own inertia, the parts apart.
    pub j_wheel_table: f64,
    pub payload: f64,
    pub station_radius: f64,
    pub attached: bool,
    pub play: f64,
    /// The table's mode on its shaft against the locked wheel: w_n (rad/s) and zeta.
    pub mode: (f64, f64),
    /// The tolerance the table settles to, rad.
    pub tolerance: f64,
    pub cam: Cam,
    pub master: Option<CamMaster>,
    limits: [f64; 3],
}

pub fn build_geneva(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    build(cfg, false)
}

pub fn build_cam(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    build(cfg, true)
}

fn build(cfg: &Config, cam: bool) -> Result<Box<dyn Machine>, ConfigError> {
    let joint = RotaryJoint::from_config(cfg)?;
    let indexer = Indexer::from_config(cfg, cam)?;
    let endless = indexer.master.is_some();
    let theta = axis(cfg, "driver", "axis-driver", "rad", joint.range)?;
    machine(cfg, indexer, (joint, None, "rad"), theta, endless)
}

impl Indexer {
    pub fn from_config(cfg: &Config, cam: bool) -> Result<Self, ConfigError> {
        let stations = cfg.positive("indexer.stations")?;
        let least = if cam { 2.0 } else { 3.0 };
        if stations.fract() != 0.0 || stations < least {
            return Err(ConfigError::Invalid("indexer.stations".into()));
        }
        let n = stations as usize;
        let (drive, half) = if cam {
            let angle = cfg.positive("indexer.index_angle_deg")?;
            if angle >= 360.0 {
                return Err(ConfigError::Invalid("indexer.index_angle_deg".into()));
            }
            (Drive::Cam { pitch_radius: cfg.positive("indexer.roller_pitch_radius_m")? }, 0.5 * angle.to_radians())
        } else {
            let lambda = (PI / n as f64).sin();
            (Drive::Geneva { lambda, crank_radius: cfg.positive("indexer.crank_radius_m")? }, 0.5 * PI - PI / n as f64)
        };
        let station_radius = cfg.positive("table.station_radius_m")?;
        let j_wheel_table = cfg.positive("indexer.wheel_inertia_kgm2")? + cfg.positive("table.inertia_kgm2")?;
        let payload = payload(cfg)?;
        let j_all = j_wheel_table + payload * station_radius * station_radius;
        let w_n = (cfg.positive("table.stiffness_nm_rad")? / j_all).sqrt();
        let one_turn = Cam::from_segments(&[Segment { master: TAU, slave: TAU, law: Law::Linear }])
            .expect("one turn at constant speed");
        let master = master(cfg, TAU * cfg.positive("indexer.turns_per_min")? / 60.0, one_turn.period)?;
        Ok(Indexer {
            drive,
            stations: n,
            half,
            j_driver: cfg.positive("indexer.driver_inertia_kgm2")?,
            j_wheel_table,
            payload,
            station_radius,
            attached: true,
            play: cfg.non_negative("indexer.locking_play_rad")?,
            mode: (w_n, cfg.positive("table.damping_ratio")?),
            tolerance: cfg.positive("ratings.index_settling_tolerance_m")? / station_radius,
            cam: one_turn,
            master,
            limits: [
                cfg.f64("ratings.indexer_max_contact_force_n").unwrap_or(f64::INFINITY),
                cfg.f64("ratings.index_max_error_m").unwrap_or(f64::INFINITY),
                cfg.f64("ratings.index_max_settling_s").unwrap_or(f64::INFINITY),
            ],
        })
    }

    /// The table's inertia with the parts as they are.
    pub fn j_table(&self) -> f64 {
        let r = self.station_radius;
        self.j_wheel_table + if self.attached { self.payload * r * r } else { 0.0 }
    }

    /// The angle of one station, rad.
    fn step(&self) -> f64 {
        TAU / self.stations as f64
    }

    /// The driver's angle from the middle of the index, x in [-h, h), or `None` in the dwell; and the index count k.
    fn phase(&self, theta: f64) -> (Option<f64>, f64) {
        let h = self.half;
        let k = ((theta + h) / TAU).floor();
        let w = theta + h - TAU * k;
        ((w < 2.0 * h).then_some(w - h), k)
    }

    /// The wheel's angle phi and its derivatives d phi / d theta, d2 phi / d theta2 at the driver's angle.
    pub fn motion(&self, theta: f64) -> [f64; 3] {
        let (x, k) = self.phase(theta);
        let step = self.step();
        let Some(x) = x else { return [(k + 1.0) * step, 0.0, 0.0] };
        match self.drive {
            Drive::Geneva { lambda: l, .. } => {
                let d = 1.0 - 2.0 * l * x.cos() + l * l;
                let local = (l * x.sin()).atan2(1.0 - l * x.cos());
                [k * step + 0.5 * step + local, l * (x.cos() - l) / d, -l * (1.0 - l * l) * x.sin() / (d * d)]
            }
            Drive::Cam { .. } => {
                let span = 2.0 * self.half;
                let [s, s1, s2] = modified_sine((x + self.half) / span);
                [k * step + step * s, step * s1 / span, step * s2 / (span * span)]
            }
        }
    }

    /// The lever of the contact from the wheel's centre, m: the Geneva's pin, the cam's rollers. `None` in the dwell.
    pub fn lever(&self, theta: f64) -> Option<f64> {
        let x = self.phase(theta).0?;
        Some(match self.drive {
            Drive::Geneva { lambda: l, crank_radius } => crank_radius / l * (1.0 - 2.0 * l * x.cos() + l * l).sqrt(),
            Drive::Cam { pitch_radius } => pitch_radius,
        })
    }

    /// The torque the drive puts on the wheel (table and parts) and the contact force, at the driver's angle, speed and
    /// acceleration.
    pub fn loads(&self, theta: f64, speed: f64, acc: f64) -> [f64; 2] {
        let [_, p1, p2] = self.motion(theta);
        let torque = self.j_table() * (p2 * speed * speed + p1 * acc);
        [torque, self.lever(theta).map_or(0.0, |r| torque / r)]
    }

    /// The table's twist ring left at the end of an index run at the driver's speed `speed` throughout, rad: its
    /// amplitude sqrt(x^2 + ((x' + zeta w_n x) / w_d)^2) from rest at the index's start. Over 2000 steps, each with the
    /// base acceleration at its middle held and the oscillator's own solution (exact for that held force).
    pub fn residual(&self, speed: f64) -> f64 {
        let (w, zeta) = self.mode;
        if speed.abs() < 1e-9 {
            return 0.0;
        }
        const STEPS: usize = 2000;
        let (sigma, wd) = (zeta * w, w * (1.0 - zeta * zeta).sqrt());
        let dt = 2.0 * self.half / speed.abs() / STEPS as f64;
        let (c, s, e) = ((wd * dt).cos(), (wd * dt).sin(), (-sigma * dt).exp());
        let phi = [[e * (c + sigma / wd * s), e * s / wd], [-e * w * w / wd * s, e * (c - sigma / wd * s)]];
        let (mut x, mut v) = (0.0, 0.0);
        for i in 0..STEPS {
            let theta = -self.half + (i as f64 + 0.5) * 2.0 * self.half / STEPS as f64;
            let rest = -self.motion(theta)[2] * speed * speed / (w * w);
            let y = x - rest;
            (x, v) = (rest + phi[0][0] * y + phi[0][1] * v, phi[1][0] * y + phi[1][1] * v);
        }
        x.hypot((v + sigma * x) / wd)
    }

    /// How long the table rings beyond the tolerance after an index at the driver's speed, s.
    pub fn settling_time(&self, speed: f64) -> f64 {
        let a = self.residual(speed);
        if a <= self.tolerance { 0.0 } else { (a / self.tolerance).ln() / (self.mode.0 * self.mode.1) }
    }
}

impl Mechanism for Indexer {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let [_, p1, p2] = self.motion(q[0]);
        mass[0] = self.j_driver + self.j_table() * p1 * p1;
        bias[0] = self.j_table() * p1 * p2 * qd[0] * qd[0];
    }
    fn potential(&self, _q: &[f64]) -> f64 {
        0.0
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    /// The speed term of J(theta) needs the feedforward's speed terms.
    fn tool_is_joints(&self) -> bool {
        false
    }
    fn cam(&self) -> Option<CamMaster> {
        self.master
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        out[0] = self.cam.at(master);
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        [
            ("table.angle", "group-table", "quantity-position", "rad"),
            ("table.speed", "group-table", "quantity-speed", "rad/s"),
            ("table.acceleration", "group-table", "quantity-acceleration", "rad/s²"),
            ("table.torque", "group-table", "quantity-table-torque", "N m"),
            ("indexer.contact_force", "group-indexer", "quantity-contact-force", "N"),
            ("table.index_error", "group-table", "quantity-index-error", "m"),
            ("table.settling_time", "group-table", "quantity-settling-time", "s"),
            ("indexer.inertia", "group-indexer", "quantity-driver-inertia", "kg m²"),
        ]
        .into_iter()
        .map(|(id, owner, quantity, unit)| SignalInfo::new(id, owner, quantity, unit))
        .collect()
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let [phi, p1, p2] = self.motion(q[0]);
        let [torque, contact] = self.loads(q[0], qd[0], qdd[0]);
        let moving = self.phase(q[0]).0.is_some();
        out.extend([
            phi,
            p1 * qd[0],
            p2 * qd[0] * qd[0] + p1 * qdd[0],
            torque,
            contact,
            // Held by the locking disc: half its play either way, at the stations.
            if moving { 0.0 } else { 0.5 * self.play * self.station_radius },
            if moving { self.settling_time(qd[0]) } else { 0.0 },
            self.j_driver + self.j_table() * p1 * p1,
        ]);
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        [
            ("indexer-contact-force", "indexer.contact_force"),
            ("index-accuracy", "table.index_error"),
            ("index-settling", "table.settling_time"),
        ]
        .into_iter()
        .zip(self.limits)
        .filter(|(_, limit)| limit.is_finite())
        .map(|((check, signal), limit)| RatingCheck { check, signal: signal.into(), motor: None, limit, life: false })
        .collect()
    }

    /// Seen from the front: the table at the origin with a mark per station turning with it, the driver beside it with
    /// its crank (or cam) mark; the motor behind the driver.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let r = self.station_radius;
        let phi = self.motion(q[0])[0];
        let driver = [-(r + 0.12), 0.0, 0.0];
        out.parts.push(Part::new("table", Shape::Cylinder { radius: r, length: 0.03 }, [0.0; 3], [0.45, 0.47, 0.5]));
        for i in 0..self.stations {
            out.parts.push(
                Part::new("table-mark", Shape::Box { size: [r, 0.004, 0.02] }, [0.0, -0.017, 0.0], [0.1, 0.1, 0.1])
                    .turned(-phi + self.step() * i as f64),
            );
        }
        out.parts.push(Part::new("driver", Shape::Cylinder { radius: 0.08, length: 0.03 }, driver, [0.3, 0.32, 0.36]));
        out.parts.push(
            Part::new("driver-mark", Shape::Box { size: [0.08, 0.004, 0.015] }, driver, [0.9, 0.8, 0.2]).turned(q[0]),
        );
        let motor = [driver[0], 0.12, 0.0];
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            motor,
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [motor[0], 0.19, 0.0], [0.1, 0.1, 0.1])
                .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [driver[0], 0.0, 0.16],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [0.0, 0.0, r + 0.05],
            owner_key: "group-table",
            value: phi.to_degrees(),
            unit_key: "unit-deg",
            decimals: 1,
        });
    }
}
