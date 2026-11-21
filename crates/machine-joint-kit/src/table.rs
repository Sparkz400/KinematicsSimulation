//! A rotary indexing table about a vertical axis, angle c (rad): the table's and the fixture's inertia, the workpiece's
//! as payload, no gravity torque. Its gear (`table.gear`) is "planetary" (the `[gearbox]` as it is), "strain-wave"
//! (the `[gearbox]` plus the life of `[strain_wave]`) or "worm" (the `[gearbox]` with the efficiency `[worm]` gives).
//!
//! A worm drives the table with its efficiency tan(gamma) / tan(gamma + rho') and is driven back by the table with
//! tan(gamma - rho') / tan(gamma) (`WormGear`, gamma the lead angle, rho' the friction angle on the flank), which the
//! gearbox carries as its back-drive ratio (`Gearbox::back_ratio`). A worm whose back efficiency is not positive locks
//! itself, and the table refuses it: the gearbox does not model a gear that holds the table against its load.
//!
//! A worm's motor lies across the table's axis: its rotor's spin (momentum) is about the worm's own axis, `worm.motor_axis`
//! (x, y, z across z; any length but 0; default along x), fixed to the base, while a planetary gear's rotor spins about
//! z with the table and a strain wave gear's against it (its flexspline turns against its wave generator).

use crate::{axis, machine, payload, required_life};
use sim_components::chain::Mechanism;
use sim_components::joint::RotaryJoint;
use sim_components::transmission::{StrainWave, WormGear};
use sim_core::{Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

pub struct Table {
    inertia: f64,
    payload: f64,
    attached: bool,
    radius: f64,
    strain_wave: Option<(StrainWave, f64)>,
    ratio: f64,
    /// The unit axis the motor's rotor spins about (fixed to the base).
    rotor_axis: [f64; 3],
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    // The table has no links to bend.
    sim_components::refuse_unmodelled(cfg, &["link_bending", "link_bending_reaction"])?;
    let mut joint = RotaryJoint::from_config(cfg)?;
    let mut strain_wave = None;
    match cfg.get("table.gear")?.as_str() {
        Some("planetary") => {}
        Some("strain-wave") => {
            strain_wave = Some((StrainWave::from_config(cfg)?, required_life(cfg).unwrap_or(f64::INFINITY)))
        }
        Some("worm") if cfg.bool("effects.gearbox_efficiency")? => {
            let gear = WormGear::from_config(cfg)?;
            let (worm, back) = (gear.efficiency(), gear.back_efficiency());
            if back <= 0.0 {
                return Err(ConfigError::Invalid(
                    "worm.lead_angle_deg (the worm locks itself: its back efficiency is not positive, and the table's gearbox does not model a gear that holds the table against its load)"
                        .into(),
                ));
            }
            joint.extra_efficiency = worm / joint.gearbox.efficiency;
            joint.gearbox.efficiency = worm;
            joint.gearbox.back_ratio = back / worm;
        }
        Some("worm") => {}
        _ => return Err(ConfigError::Invalid("table.gear".into())),
    }
    let rotor_axis = match cfg.get("table.gear")?.as_str() {
        Some("worm") if cfg.get("worm.motor_axis").is_ok() => {
            let a = cfg.f64s("worm.motor_axis")?;
            let size = a.iter().map(|v| v * v).sum::<f64>().sqrt();
            // A worm's axis crosses its wheel's at a right angle.
            if a.len() != 3 || !(size > 0.0) || !size.is_finite() || a[2].abs() > 1e-9 * size {
                return Err(ConfigError::Invalid("worm.motor_axis (x, y, z across the table's axis z)".into()));
            }
            [a[0] / size, a[1] / size, 0.0]
        }
        Some("worm") => [1.0, 0.0, 0.0],
        // The strain wave gear's flexspline (the output, its circular spline held) turns against its wave generator.
        Some("strain-wave") => [0.0, 0.0, -1.0],
        _ => [0.0, 0.0, 1.0],
    };
    let table = Table {
        inertia: cfg.positive("table.inertia_kgm2")?,
        payload: payload(cfg, "table.payload_inertia_kgm2")?,
        attached: true,
        radius: cfg.positive("table.radius_m")?,
        strain_wave,
        ratio: joint.ratio(),
        rotor_axis,
    };
    let c = axis(cfg, "c", "axis-c", "rad", joint.range)?;
    machine(cfg, table, vec![(joint, None, "rad")], vec![c])
}

impl Mechanism for Table {
    /// The table, fixture and workpiece turning about z, and the motor's rotor about its own axis (`rotor_axis`).
    fn frame_momentum(&self, _q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let turning = self.inertia + if self.attached { self.payload } else { 0.0 };
        let mut angular = self.rotor_axis.map(|a| a * rotor_spin[0]);
        angular[2] += turning * qd[0];
        Some(sim_core::FrameMomentum { angular, ..Default::default() })
    }
    fn dynamics(&self, _q: &[f64], _qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        mass[0] = self.inertia + if self.attached { self.payload } else { 0.0 };
        bias[0] = 0.0;
    }
    fn potential(&self, _q: &[f64]) -> f64 {
        0.0
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        self.strain_wave.iter().map(|(g, _)| g.signal_info("1", "joint-1")).collect()
    }
    fn signals(&self, _q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        if let Some((g, _)) = &self.strain_wave {
            out.push(g.signal(force[0], qd[0] * self.ratio));
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.strain_wave
            .iter()
            .filter(|(_, life)| life.is_finite())
            .map(|(g, life)| g.rating_check("1", 0, *life))
            .collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["fixture", "payload"]
    }

    /// The table seen from the front, its index mark turning, the fixture on it; the motor beneath it.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let r = self.radius;
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [2.2 * r, 2.2 * r, 0.1] },
            [0.0, 0.0, -0.1],
            [0.62, 0.64, 0.68],
        ));
        out.parts.push(Part::new(
            "table",
            Shape::Cylinder { radius: r, length: 0.04 },
            [0.0, 0.0, 0.25],
            [0.55, 0.55, 0.6],
        ));
        out.parts.push(
            Part::new("index-mark", Shape::Box { size: [1.8 * r, 0.004, 0.02] }, [0.0, -0.025, 0.25], [0.1, 0.1, 0.1])
                .turned(q[0]),
        );
        // The fixture as the box it turns within, for the collision with obstacles.
        out.parts.push(Part::new(
            "fixture",
            Shape::Box { size: [1.2 * r, 0.08, 0.1] },
            [0.0, 0.0, 0.32],
            [0.45, 0.45, 0.5],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.4 * r, 0.06, 0.4 * r] },
                [0.6 * r * q[0].cos(), -0.05, 0.25 + 0.6 * r * q[0].sin()],
                [0.2, 0.7, 0.3],
            ));
        }
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Cylinder { radius: 0.05, length: 0.12 },
            [r + 0.15, 0.0, 0.0],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new(
            "gearbox",
            Shape::Box { size: [0.1, 0.1, 0.1] },
            [r + 0.15, 0.0, 0.12],
            [0.25, 0.25, 0.28],
        ));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [r + 0.15, -0.07, 0.0], [0.1, 0.1, 0.1])
                .turned(rotors[0]),
        );
        out.labels.push(Label {
            pos: [r + 0.15, 0.0, -0.12],
            owner_key: "motor-joint-1",
            value: rotor_speeds[0] * 60.0 / TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [0.0, 0.0, 0.25 + 1.2 * r],
            owner_key: "axis-c",
            value: q[0].to_degrees(),
            unit_key: "unit-deg",
            decimals: 1,
        });
    }
}

#[cfg(test)]
mod tests {
    use crate::ROTARY_TABLE;

    /// The shipped worm (lead angle 8 degrees, mu 0.04, alpha_n 20 degrees): rho' = atan(0.04 / cos 20) = 2.4374 degrees,
    /// driven back tan(8 - 2.4376 degrees) / tan(8 degrees) = 0.097383 / 0.140541 = 0.6929, so the table builds; at a lead
    /// angle of 2 degrees (below rho') the worm locks itself and the table is refused.
    #[test]
    fn a_worm_that_locks_itself_is_refused() {
        let mut cfg = ROTARY_TABLE.config();
        cfg.set_text("table.gear = \"worm\"").unwrap();
        assert!((ROTARY_TABLE.build)(&cfg).is_ok());
        cfg.set_text("worm.lead_angle_deg = 2.0").unwrap();
        let err = (ROTARY_TABLE.build)(&cfg).err().expect("a locking worm is refused");
        assert!(format!("{err:?}").contains("locks itself"), "{err:?}");
    }

    /// The table turning at 1 rad/s, braked by its motor (-0.5 N m, 0.025 s, the gear passing over 40 W: the efficiency
    /// has not faded, tanh(80)) through the worm, with no loss but the worm's and a
    /// rotor too light to hold energy (1e-8 kg m2, 1e-4 at the table against its 3): the table drives the worm back, so
    /// of the energy the table gives up the motor takes in tan(gamma - rho') / tan(gamma) = 0.692974 (gamma 8 degrees,
    /// rho' = atan(0.04 / cos 20 degrees) = 2.43744 degrees) and the gear loses the rest; the worm driving would have
    /// passed 0.762938 (to 2e-5: the rotor's 2.2e-5 of the energy passes without loss). Rigid, through the loss factor;
    /// elastic, with the mesh friction (bounded at that back efficiency, `Gearbox::mesh_friction`), the budget closes (work, stored
    /// and the loss rows over the run).
    #[test]
    fn the_table_drives_the_worm_back_at_its_back_efficiency() {
        for elastic in [false, true] {
            let mut cfg = ROTARY_TABLE.config();
            for edit in [
                "table.gear = \"worm\"",
                "effects.joint_friction = false",
                "effects.motor_friction = false",
                "effects.gearbox_drag = false",
                "effects.torque_ripple = false",
                "gearbox.inertia_at_input_kgm2 = 1e-12",
            ] {
                cfg.set_text(edit).unwrap();
            }
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            // Elastic, the gear's damping of 30 N m s/rad on a rotor of 1e-8 (1e-4 at the table) would be past RK4's step.
            cfg.set_text(if elastic { "motor.rotor_inertia_kgm2 = 1e-6" } else { "motor.rotor_inertia_kgm2 = 1e-8" })
                .unwrap();
            cfg.set_text(&format!("effects.gearbox_mesh_friction = {elastic}")).unwrap();
            let mut m = (ROTARY_TABLE.build)(&cfg).unwrap();
            let pose = cfg.f64s("start.pose").unwrap();
            let (mut state, mut torque) = (Vec::new(), vec![0.0; m.motors().len()]);
            m.equilibrium(&pose, &mut state, &mut torque);
            m.set_tool_velocity(&mut state, &[1.0]);
            let motor = |m: &dyn sim_core::Machine, s: &[f64]| {
                let (mut pos, mut vel) = (vec![0.0], vec![0.0]);
                m.motor_state(s, &mut pos, &mut vel);
                pos[0]
            };
            let loss = |m: &dyn sim_core::Machine, s: &[f64]| {
                let mut signals = Vec::new();
                m.signals(s, &[-0.5], &mut signals);
                (m.signal_infos().iter().zip(signals))
                    .filter(|(i, _)| i.id.starts_with(sim_core::LOSS_PREFIX))
                    .map(|(_, p)| p)
                    .sum::<f64>()
            };
            let (start, mut rk, mut work, mut lost) = (m.energy(&state), sim_core::Rk4::new(state.len()), 0.0, 0.0);
            let (brake, h) = ([-0.5], 62.5e-6 / 4.0);
            for _ in 0..400 {
                for _ in 0..4 {
                    let (before, p0) = (motor(m.as_ref(), &state), loss(m.as_ref(), &state));
                    sim_core::advance(m.as_ref(), &mut rk, &mut state, &brake, h, 1);
                    lost += 0.5 * h * (p0 + loss(m.as_ref(), &state));
                    work += brake[0] * (motor(m.as_ref(), &state) - before);
                }
                m.after_step(&mut state, 62.5e-6);
            }
            let given = start - m.energy(&state);
            let off = given + work - lost;
            println!("elastic {elastic}: given {given} J, motor {work} J, lost {lost} J, off {off:e} J");
            assert!(given > 0.1, "{given}");
            // Closed to 2e-13 rigid and 9.3e-7 of what the table gave elastic (the mesh bristle's store is not booked).
            assert!(off.abs() < 2e-6 * given, "elastic {elastic}: {given} J, {work} J, {lost} J");
            if !elastic {
                assert!((-work / given - 0.692_974).abs() < 2e-5, "{} of {given} J", -work / given);
            }
        }
    }

    /// The rotor's momentum, J_r r w with the table at w = 1 rad/s (r the ratio, J_r motor and gear input), is about the
    /// worm's own axis (`worm.motor_axis`, any length), the table's J w about z; behind a planetary gear it adds to the
    /// table's about z, behind a strain wave gear it takes from it. An axis of 0 or one not across z is refused.
    #[test]
    fn a_worm_rotor_spins_about_its_own_axis() {
        let angular = |edits: &[&str]| -> Result<[f64; 3], sim_core::ConfigError> {
            let mut cfg = ROTARY_TABLE.config();
            cfg.set_text("effects.joint_elasticity = false").unwrap();
            for edit in edits {
                cfg.set_text(edit).unwrap();
            }
            let m = (ROTARY_TABLE.build)(&cfg)?;
            let (mut state, mut torque) = (Vec::new(), vec![0.0]);
            m.equilibrium(&cfg.f64s("start.pose").unwrap(), &mut state, &mut torque);
            m.set_tool_velocity(&mut state, &[1.0]);
            Ok(m.frame_momentum(&state).unwrap().angular)
        };
        let cfg = ROTARY_TABLE.config();
        let rotor = (cfg.f64("motor.rotor_inertia_kgm2").unwrap() + cfg.f64("gearbox.inertia_at_input_kgm2").unwrap())
            * cfg.f64("gearbox.ratio").unwrap();
        let table = cfg.f64("table.inertia_kgm2").unwrap() + cfg.f64("table.payload_inertia_kgm2").unwrap();
        let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(&b).all(|(a, b)| (a - b).abs() < 1e-12 * (1.0 + b.abs()));
        let worm = |axis: &str| angular(&["table.gear = \"worm\"", &format!("worm.motor_axis = {axis}")]);
        assert!(close(worm("[1.0, 0.0, 0.0]").unwrap(), [rotor, 0.0, table]));
        assert!(close(worm("[0.0, -2.0, 0.0]").unwrap(), [0.0, -rotor, table]));
        assert!(close(angular(&["table.gear = \"planetary\""]).unwrap(), [0.0, 0.0, table + rotor]));
        assert!(close(angular(&["table.gear = \"strain-wave\""]).unwrap(), [0.0, 0.0, table - rotor]));
        assert!(worm("[0.0, 0.0, 0.0]").is_err() && worm("[1.0, 0.0, 0.5]").is_err());
    }
}
