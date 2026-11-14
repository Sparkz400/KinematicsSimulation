//! A motor-driven rotary joint: motor, gearbox (ratio, torsional stiffness, backlash, efficiency), holding brake, the
//! motor's own friction and torque ripple, friction in the joint itself and end stops on its angle. What a machine of
//! arms that turn about their joints needs per joint; it knows no machine.
//!
//! The joint's angle (the gearbox output, the arm) and the rotor's angle are apart by the gearbox. Elastic: both are
//! states, and the gearbox's spring passes the torque between them (`Gearbox::torque`). Rigid: the rotor turns with the
//! arm times the ratio, and its inertia counts at the arm times the ratio squared (`reflected_inertia`).
//!
//! Config, read here:
//!
//! - `[motor]`, `[gearbox]`: as for every machine (`ShaftFriction`, `Ripple`, `Gearbox`, `HoldingBrake`).
//! - `[joint]`: `friction_coulomb_nm`, `friction_viscous_nm_s_rad`, `friction_smoothing_rad_s` (effects.joint_friction),
//!   `min_angle_deg`, `max_angle_deg` (the stops), `stop_stiffness_nm_rad`, `stop_damping_nm_s_rad`
//!   (effects.hard_stops).

use crate::mech::{GearTorque, Gearbox, HardStop, HoldingBrake, Ripple, ShaftFriction};
use sim_core::{Config, ConfigError};

#[derive(Debug, Clone)]
pub struct RotaryJoint {
    pub gearbox: Gearbox,
    /// Rotor and gearbox input, kg m^2: what the brake holds and the motor turns.
    pub rotor_inertia: f64,
    /// Motor bearing friction, at the rotor.
    pub shaft: ShaftFriction,
    pub ripple: Ripple,
    pub brake: HoldingBrake,
    /// Friction in the joint (bearings of the arm), at the output, N m.
    pub friction: ShaftFriction,
    /// The stops at both ends of the angle range, N m/rad and N m s/rad; zero with effects.hard_stops off.
    pub stops: HardStop,
    /// The angles of the stops, rad.
    pub range: (f64, f64),
    /// What a screw, rack or worm puts on the configured gearbox's efficiency (1 for none): kept apart so that the
    /// gearbox's temperature can set its own share (`JointMachine::set_part_temperatures`), and so that the torque at
    /// the gearbox output can be told from the force at the joint.
    pub extra_efficiency: f64,
    /// The configured gearbox has ratio 1: it is the coupling of a direct drive, also with a screw or rack behind it
    /// (which changes the ratio). Nothing then warms as a gearbox, and its loss is the coupling's.
    pub coupled: bool,
}

impl RotaryJoint {
    pub fn from_config(cfg: &Config) -> Result<Self, ConfigError> {
        let on = cfg.bool("effects.joint_friction")?;
        let (min, max) = (cfg.f64("joint.min_angle_deg")?.to_radians(), cfg.f64("joint.max_angle_deg")?.to_radians());
        if min >= max {
            return Err(ConfigError::Invalid("joint.max_angle_deg".into()));
        }
        let (stiffness, damping) =
            (cfg.positive("joint.stop_stiffness_nm_rad")?, cfg.non_negative("joint.stop_damping_nm_s_rad")?);
        let friction = ShaftFriction {
            coulomb: if on { cfg.non_negative("joint.friction_coulomb_nm")? } else { 0.0 },
            viscous: if on { cfg.non_negative("joint.friction_viscous_nm_s_rad")? } else { 0.0 },
            smoothing: cfg.positive("joint.friction_smoothing_rad_s")?,
        };
        Self::with(cfg, friction, Self::stops(cfg, stiffness, damping)?, (min, max))
    }

    /// A joint that slides (a carriage on its guides), its coordinate the travel in m: `[slide]` `stroke_m` (the stops
    /// at 0 and the stroke), `friction_coulomb_n`, `friction_viscous_n_s_m`, `friction_smoothing_m_s` (the guides,
    /// effects.joint_friction), `stop_stiffness_n_m`, `stop_damping_n_s_m` (effects.hard_stops). Motor and gearbox as for
    /// `from_config`; a transmission (`transmission::Converter`) turns the gearbox into one in N and m.
    pub fn linear_from_config(cfg: &Config) -> Result<Self, ConfigError> {
        let on = cfg.bool("effects.joint_friction")?;
        let friction = ShaftFriction {
            coulomb: if on { cfg.non_negative("slide.friction_coulomb_n")? } else { 0.0 },
            viscous: if on { cfg.non_negative("slide.friction_viscous_n_s_m")? } else { 0.0 },
            smoothing: cfg.positive("slide.friction_smoothing_m_s")?,
        };
        let stops =
            Self::stops(cfg, cfg.positive("slide.stop_stiffness_n_m")?, cfg.non_negative("slide.stop_damping_n_s_m")?)?;
        Self::with(cfg, friction, stops, (0.0, cfg.positive("slide.stroke_m")?))
    }

    fn stops(cfg: &Config, stiffness: f64, damping: f64) -> Result<HardStop, ConfigError> {
        Ok(if cfg.bool("effects.hard_stops")? {
            HardStop { stiffness, damping }
        } else {
            HardStop { stiffness: 0.0, damping: 0.0 }
        })
    }

    /// Motor and gearbox from `[motor]` and `[gearbox]`, with the joint's own friction, stops and range.
    pub fn with(
        cfg: &Config,
        friction: ShaftFriction,
        stops: HardStop,
        range: (f64, f64),
    ) -> Result<Self, ConfigError> {
        let rotor_inertia =
            cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?;
        let gearbox = Gearbox::from_config(cfg)?;
        Ok(Self {
            coupled: gearbox.is_coupling(),
            gearbox,
            rotor_inertia,
            shaft: ShaftFriction::from_config(cfg)?,
            ripple: Ripple::from_config(cfg)?,
            brake: HoldingBrake::from_config(cfg, rotor_inertia)?,
            friction,
            stops,
            range,
            extra_efficiency: 1.0,
        })
    }

    pub fn ratio(&self) -> f64 {
        self.gearbox.ratio
    }

    /// The rotor's inertia as the joint feels it in the rigid model.
    pub fn reflected_inertia(&self) -> f64 {
        self.gearbox.ratio * self.gearbox.ratio * self.rotor_inertia
    }

    /// What acts on the arm at the joint besides the gearbox, at its angle and speed: the joint's friction and the
    /// stops (positive turns the angle up).
    pub fn output_torque(&self, angle: f64, speed: f64) -> f64 {
        let (min, max) = self.range;
        self.stops.force(angle - min, speed, max - min) - self.friction.torque(speed)
    }

    /// Elastic model: the gearbox torques between rotor and arm, the rotor's acceleration under the motor's torque
    /// `command` (before ripple), and the rate of the brake's deflection `brake`.
    pub fn rotor(
        &self,
        command: f64,
        rotor: f64,
        rotor_speed: f64,
        angle: f64,
        speed: f64,
        brake: f64,
        engaged: bool,
    ) -> (GearTorque, f64, f64) {
        let gear = self.gearbox.torque(rotor, rotor_speed, angle, speed);
        let (held, rate) = self.brake.torque(engaged, rotor_speed, brake);
        let acc = (self.ripple.apply(command, rotor) - gear.rotor - self.shaft.torque(rotor_speed) - held)
            / self.rotor_inertia;
        (gear, acc, rate)
    }

    /// Rigid model: the torque the motor puts on the arm through a loss-free gearbox with a massless rotor at the
    /// rotor's angle and speed (ripple, motor friction and the brake taken off), and the brake's deflection rate.
    pub fn rigid_drive(&self, command: f64, rotor: f64, rotor_speed: f64, brake: f64, engaged: bool) -> (f64, f64) {
        let (held, rate) = self.brake.torque(engaged, rotor_speed, brake);
        (self.gearbox.ratio * (self.ripple.apply(command, rotor) - self.shaft.torque(rotor_speed) - held), rate)
    }

    /// Energy in the stops' springs at an angle.
    pub fn stop_energy(&self, angle: f64) -> f64 {
        let (min, max) = self.range;
        self.stops.energy(angle - min, max - min)
    }

    /// Powers lost at an angle and speed, rotor speed and brake: in the joint's friction, the stops, and the motor's
    /// friction with the brake (W).
    pub fn losses(&self, angle: f64, speed: f64, rotor_speed: f64, brake: f64, engaged: bool) -> [f64; 3] {
        let (min, max) = self.range;
        let held = self.brake.torque(engaged, rotor_speed, brake).0;
        [
            self.friction.torque(speed) * speed,
            self.stops.loss_power(angle - min, speed, max - min),
            (self.shaft.torque(rotor_speed) + held) * rotor_speed,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn joint_text() -> String {
        "format_version = 1\n[effects]\njoint_friction = true\nhard_stops = true\nmotor_friction = true\ntorque_ripple = false\ngearbox_backlash = false\ngearbox_efficiency = true\nthermal_feedback = false\nholding_brake = true\n\
            [motor]\nrotor_inertia_kgm2 = 1e-4\nfriction_coulomb_nm = 0.1\nfriction_viscous_nm_s_rad = 0.0\nfriction_smoothing_rad_s = 1e-3\nripple_fraction = 0.0\nripple_order = 6.0\ncogging_nm = 0.0\ncogging_order = 6.0\nbrake_torque_nm = 5.0\n\
            [gearbox]\nratio = 20.0\nefficiency = 0.9\nbacklash_rad = 0.0\nstiffness_nm_rad = 1e4\ndamping_nm_s_rad = 0.0\ninertia_at_input_kgm2 = 1e-4\n\
            [joint]\nfriction_coulomb_nm = 2.0\nfriction_viscous_nm_s_rad = 0.5\nfriction_smoothing_rad_s = 1e-3\nmin_angle_deg = -45.0\nmax_angle_deg = 90.0\nstop_stiffness_nm_rad = 1e5\nstop_damping_nm_s_rad = 0.0\n".to_string()
    }

    fn joint() -> RotaryJoint {
        RotaryJoint::from_config(&Config::parse(&joint_text()).unwrap()).unwrap()
    }

    /// By hand: joint friction 2 N m + 0.5 N m s/rad at 4 rad/s is 4 N m against the motion; 0.01 rad beyond the upper
    /// stop pushes back with 1e5 * 0.01 = 1000 N m; the rotor (1e-4 + 1e-4 kg m^2) is felt at the arm as 2e-4 * 400 =
    /// 0.08 kg m^2; 1 N m at the motor with 0.1 N m friction gives 18 N m at the arm in the rigid model.
    #[test]
    fn a_joint_adds_friction_stops_and_the_rotor_as_calculated() {
        let j = joint();
        assert!((j.output_torque(0.0, 4.0) + 4.0).abs() < 1e-9);
        assert!((j.output_torque(90f64.to_radians() + 0.01, 0.0) + 1000.0).abs() < 1e-6);
        assert!((j.output_torque(-45f64.to_radians() - 0.01, 0.0) - 1000.0).abs() < 1e-6);
        assert!((j.reflected_inertia() - 0.08).abs() < 1e-12);
        assert!((j.rigid_drive(1.0, 0.0, 10.0, 0.0, false).0 - 18.0).abs() < 1e-9);
        // Elastic: twisted by 1 mrad at the output, the gearbox passes 10 N m to the arm and takes 0.5 N m from the rotor.
        let (gear, acc, _) = j.rotor(0.0, 20.0 * 1e-3, 0.0, 0.0, 0.0, 0.0, false);
        assert!((gear.output - 10.0).abs() < 1e-9 && (acc + 0.5 / 2e-4).abs() < 1e-6);
        // The losses at 4 rad/s at the arm and 80 rad/s at the rotor: 16 W in the joint, 8 W in the motor bearings.
        let [joint_loss, stop_loss, motor_loss] = j.losses(0.0, 4.0, 80.0, 0.0, false);
        assert!((joint_loss - 16.0).abs() < 1e-9 && stop_loss == 0.0 && (motor_loss - 8.0).abs() < 1e-9);
    }
    /// A pinion of 50 mm radius (stiffness 1e7 N/m, backlash 0.1 mm, 95 %, 0.04 kg m^2) behind the gearbox of ratio 20
    /// (1e4 N m/rad, 90 %, rotor and input 2e-4 kg m^2), by hand: ratio 20 / 0.05 = 400 rad/m; the gearbox seen at the
    /// rack 1e4 / 0.05^2 = 4e6 N/m, in series with 1e7: 1 / (2.5e-7 + 1e-7) = 2.857143e6 N/m; efficiency 0.9 * 0.95 =
    /// 0.855; the pinion at the rotor 0.04 / 400 = 1e-4, so 3e-4 kg m^2. Backlash off: none; on, the gearbox's 2 mrad
    /// seen at the rack (0.1 mm) plus the rack's 0.1 mm, half of it 0.1 mm.
    #[test]
    fn a_converter_behind_the_gearbox_gives_the_hand_calculated_joint() {
        use crate::transmission::Converter;
        let pinion = Converter { lever: 0.05, stiffness: 1e7, backlash: 1e-4, efficiency: 0.95, inertia: 0.04 };
        let mut cfg = Config::parse(&joint_text()).unwrap();
        let mut j = RotaryJoint::from_config(&cfg).unwrap();
        pinion.apply(&cfg, &mut j).unwrap();
        let close = |a: f64, b: f64| (a / b - 1.0).abs() < 1e-9;
        assert!(
            close(j.ratio(), 400.0) && close(j.gearbox.stiffness, 2.857142857e6) && close(j.gearbox.efficiency, 0.855)
        );
        assert!(close(j.rotor_inertia, 3e-4) && j.gearbox.half_backlash == 0.0);
        cfg.set_text("effects.gearbox_backlash = true").unwrap();
        cfg.set_text("gearbox.backlash_rad = 0.002").unwrap();
        let mut j = RotaryJoint::from_config(&cfg).unwrap();
        pinion.apply(&cfg, &mut j).unwrap();
        assert!(close(j.gearbox.half_backlash, 1e-4));
    }

    /// The slide's own table: its stroke, its guides' friction in N, its stops in N/m.
    #[test]
    fn a_linear_joint_reads_its_slide() {
        let text = joint_text()
            + "[slide]
stroke_m = 0.8
friction_coulomb_n = 30.0
friction_viscous_n_s_m = 40.0
friction_smoothing_m_s = 1e-3
stop_stiffness_n_m = 2e6
stop_damping_n_s_m = 0.0
";
        let j = RotaryJoint::linear_from_config(&Config::parse(&text).unwrap()).unwrap();
        assert_eq!(j.range, (0.0, 0.8));
        // 30 N + 40 N s/m * 0.5 m/s against the motion; 1 mm beyond the far stop, 2000 N back.
        assert!((j.output_torque(0.4, 0.5) + 50.0).abs() < 1e-9);
        assert!((j.output_torque(0.801, 0.0) + 2000.0).abs() < 1e-6);
    }
}
