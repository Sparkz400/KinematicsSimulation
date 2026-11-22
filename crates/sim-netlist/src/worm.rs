//! A worm stage (`machine.worm`): what it passes on, driven forward by the motor and back by the load.
//!
//! A gearbox passes the torque t of its spring on as t eta to its output and t / i to its rotor (`mech::Gearbox`), and
//! eta fades from 1 at rest to its value within a power t w of 0.5 W at the output. A worm that turns back (back
//! efficiency above 0) does the same while the motor drives (with its forward efficiency), and while the load drives it
//! back passes t eta_back^-1 (as the gearbox, with its back efficiency): `factors`.
//!
//! A worm whose back efficiency is 0 or less locks itself: the wheel cannot turn the worm. It follows Coulomb's law at
//! its mesh, without a fade, in both models. A screw thread of lead angle g and friction angle p, sliding at the sign s
//! of its speed, puts into the mesh (at the wheel, i times the worm's torque) what the wheel takes, o, and a drag
//! D = into - o of s times
//!
//! - (1 / eta - 1) s o on the driving flank (s o > 0, eta = tan g / tan(g + p)), and
//! - (1 - eta_back) (-s o) on the overhauling one (s o < 0, eta_back = tan(g - p) / tan g, below 0 where g < p),
//!
//! so the mesh only ever takes power (D w >= 0); while the load drives it back the motor has to push, by -eta_back of
//! the load's torque over i, and the load's power and the motor's both go into the friction. At rest the friction holds
//! anything between the two flanks' levels (`levels`), which take in 0 for a locking worm: it holds its load with no
//! motor torque.
//!
//! The elastic model (`ConfigMachine::worm_drag`) has o as its gear spring's torque, a state: the wheel gets the
//! gearbox's output torque as it is, and the rotor the flank's drag besides, sliding at the rotor's speed. Below
//! `STICK_SPEED` it sticks (Karnopp), the drag taking out the rotor's speed over `STICK_TIME_S` within the levels and never
//! along the motion: a locking worm at rest stands.
//!
//! The rigid model (`ConfigMachine::rigid_locking`) has o as an unknown beside the accelerations. The overhauling flank
//! there has no rigid solution with a heavy wheel (Painleve's paradox): with the rotor's inertia at the wheel
//! R = i^2 J and the wheel's J_L, the wheel's equation J_L a = o + L and the rotor's into = i t - R a give
//! o (1 + R / J_L) + D(o) = i t - R L / J_L, whose left side falls with o on that flank where R / J_L < -eta_back: two
//! solutions or none (a stiff mesh there is unstable and jams into stick-slip). So the overhauling flank takes its load
//! from the torque the wheel holds at rest against the axes' forces, o_rest, not from o (`overhauling_drag`): the
//! same at a steady speed, a known drag otherwise. The drag is the larger of the two flanks' (each where above 0),
//! which joins them continuously and rises with o, so the solve has exactly one answer. A worm moving with no motor
//! torque under a load that drives it back brakes by (1 - eta_back) o_rest > o_rest and stops; at rest, and below
//! `STICK_SPEED`, it sticks (`rigid_locking`).

use crate::netlist::WormNode;

/// The power at the output over which the efficiency fades in, W: the gearbox's (`mech::Gearbox::loss_factor`).
const FADE_W: f64 = 0.5;

/// (factor on the torque at the output, factor on the torque at the rotor) of a worm that turns back, at the power
/// `power` = t w at the output (positive while the motor drives).
pub fn factors(worm: &WormNode, power: f64) -> (f64, f64) {
    let x = (power / FADE_W).tanh();
    ((if x >= 0.0 { worm.efficiency } else { worm.back_efficiency }).powf(x), 1.0)
}

/// The levels of a self-locking worm's friction at its rotor while the gear's torque at the rotor is `rotor` (t / i):
/// (sliding forward, sliding backward), both at least 0. Forward the flank that `rotor` loads is the driving one where
/// it is above 0, (1 / eta - 1) rotor, the overhauling one where below, (1 - eta_back) (-rotor); backward the other way.
pub fn levels(worm: &WormNode, rotor: f64) -> (f64, f64) {
    let (driving, overhauling) = (1.0 / worm.efficiency - 1.0, 1.0 - worm.back_efficiency);
    (
        driving * rotor.max(0.0) + overhauling * (-rotor).max(0.0),
        driving * (-rotor).max(0.0) + overhauling * rotor.max(0.0),
    )
}

/// Below this speed of its wheel, rad/s, a self-locking worm sticks where its friction can hold
/// (Karnopp's band): Coulomb's drag flips with the speed's sign, so a fixed step never lands on 0 but settles a step's
/// worth of acceleration short of it (1.6e-5 rad/s at 1 rad/s2 on the 15.6 us sub-step), booking a drag its averaged
/// motion does not have. Far below a commanded speed, far above that.
pub const STICK_SPEED: f64 = 1e-3;

/// The time over which a stuck worm's wheel takes out a speed inside the band, s: it decelerates at -w / this, so that
/// it comes to rest in place of creeping on; far above the sub-step (RK4 stays stable up to 2.8 of it).
pub const STICK_TIME_S: f64 = 1e-3;

/// What a self-locking worm moving at `speed` (the wheel's, not 0) drags on its overhauling flank, at the wheel and
/// against the motion: (1 - eta_back) times the torque `held` that the wheel holds at rest, where the load drives it
/// (s held < 0, s the sign of the speed), 0 where the load is lifted (see the module comment).
pub fn overhauling_drag(worm: &WormNode, held: f64, speed: f64) -> f64 {
    (1.0 - worm.back_efficiency) * (-speed.signum() * held).max(0.0)
}

/// The worm's efficiencies `loss_factor` times as lossy as at the reference temperature: 1 - eta becomes
/// (1 - eta) f both ways, the gearbox's law (`mech::ThermalFeedback::gearbox_efficiency`) applied to the worm's own
/// efficiencies. Forward not below 0.01, as the gearbox's; a self-locking worm stays locked (back efficiency at most 0)
/// and one that turns back keeps turning back (above 0, at least 0.01).
pub fn warmed(worm: &WormNode, loss_factor: f64) -> WormNode {
    let scaled = |eta: f64| 1.0 - (1.0 - eta) * loss_factor;
    let back = scaled(worm.back_efficiency);
    WormNode {
        efficiency: scaled(worm.efficiency).clamp(0.01, 1.0),
        back_efficiency: if worm.back_efficiency > 0.0 { back.clamp(0.01, 1.0) } else { back.min(0.0) },
        ..*worm
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Far from rest the factors are the efficiencies: forward 0.8 at the output; back 1 / 0.5 = 2 at the output of a
    /// worm that turns back; at rest it passes t to both.
    #[test]
    fn a_worm_passes_its_efficiencies() {
        let turning = WormNode { ratio: 40.0, efficiency: 0.8, back_efficiency: 0.5 };
        assert_eq!(factors(&turning, 100.0), (0.8, 1.0));
        assert_eq!(factors(&turning, -100.0), (2.0, 1.0));
        assert_eq!(factors(&turning, 0.0), (1.0, 1.0));
    }

    /// A locking worm (eta 0.4, eta_back -0.2) with 10 N m on its wheel behind a ratio of 40, 0.25 N m at the rotor:
    /// sliding forward lifts it, (1 / 0.4 - 1) 0.25 = 0.375 N m; backward lowers it, (1 + 0.2) 0.25 = 0.3 N m, so the
    /// motor pushes 0.3 - 0.25 = 0.05 N m; at rest it holds anything from -0.3 to 0.375, the load's 0.25 among it. With
    /// the load the other way the levels swap.
    #[test]
    fn a_locking_worm_slides_at_its_flanks_levels() {
        let locking = WormNode { ratio: 40.0, efficiency: 0.4, back_efficiency: -0.2 };
        let close = |(a, b): (f64, f64), (c, d): (f64, f64)| assert!((a - c).abs() < 1e-12 && (b - d).abs() < 1e-12);
        close(levels(&locking, 0.25), (0.375, 0.3));
        close(levels(&locking, -0.25), (0.3, 0.375));
        close(levels(&locking, 0.0), (0.0, 0.0));
    }
    /// Holding 10 N m at rest, the locking worm (eta_back = -0.2) lowering it (speed -1 against the held +10) drags
    /// (1 + 0.2) 10 = 12 N m; lifting it (speed +1) the overhauling flank carries nothing.
    #[test]
    fn a_locking_worm_drags_on_its_overhauling_flank_by_what_it_holds() {
        let locking = WormNode { ratio: 40.0, efficiency: 0.4, back_efficiency: -0.2 };
        assert!((overhauling_drag(&locking, 10.0, -1.0) - 12.0).abs() < 1e-12);
        assert!((overhauling_drag(&locking, -10.0, 1.0) - 12.0).abs() < 1e-12);
        assert_eq!(overhauling_drag(&locking, 10.0, 1.0), 0.0);
    }

    /// Warmed so that it loses half as much (f = 0.5): 1 - (1 - 0.8) 0.5 = 0.9 forward, 1 - 0.5 0.5 = 0.75 back; a
    /// locking worm's (eta 0.4, back -0.2) back efficiency becomes 1 - 1.2 0.5 = 0.4, which would turn back, so it stays
    /// at 0; cold (f = 2) it locks harder, 1 - 1.2 2 = -1.4, and forward 1 - 0.6 2 = -0.2, held at the floor 0.01.
    #[test]
    fn a_warm_worm_loses_less_and_a_locking_one_stays_locked() {
        let turning = WormNode { ratio: 40.0, efficiency: 0.8, back_efficiency: 0.5 };
        let locking = WormNode { efficiency: 0.4, back_efficiency: -0.2, ..turning };
        let close = |a: f64, b: f64| assert!((a - b).abs() < 1e-12, "{a} against {b}");
        let w = warmed(&turning, 0.5);
        close(w.efficiency, 0.9);
        close(w.back_efficiency, 0.75);
        close(warmed(&locking, 0.5).back_efficiency, 0.0);
        let cold = warmed(&locking, 2.0);
        close(cold.efficiency, 0.01);
        close(cold.back_efficiency, -1.4);
        assert_eq!(warmed(&turning, 1.0), turning);
    }
}
