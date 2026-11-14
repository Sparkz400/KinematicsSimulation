//! A worm stage (`machine.worm`): what it passes on, driven forward by the motor and back by the load.
//!
//! A gearbox passes the torque t of its spring on as t eta to its output and t / i to its rotor (`mech::Gearbox`), and
//! eta fades from 1 at rest to its value within a power t w of 0.5 W at the output. A worm does the same while the motor
//! drives (with its forward efficiency), and while the load drives it back passes t eta_back^-1 (as the gearbox, with
//! its back efficiency). A worm whose back efficiency is 0 or less locks itself: the wheel cannot turn the worm. While
//! the load drives it back it passes t to the output and eta_back t / i to the rotor, which is against the motion where
//! eta_back < 0: the motor has to push the load down, by -eta_back of the load's torque over i, and the load's power
//! and the motor's both go into the worm's friction. At rest the rotor takes none of it, so the worm holds whatever the
//! load with no motor torque; the rotor's share fades in as x t / i driving forward and -x eta_back t / i driven back
//! (x = tanh of the power over 0.5 W, negative driven back), the output's as t eta^x forward.
//!
//! In the rigid model a drag in proportion to the gear's torque has no consistent solution while the load drives back
//! (with the wheel's inertia above the rotor's at the wheel over -eta_back, it would decelerate harder the harder the
//! motor pushes down): there the worm's drag, o - i r at the wheel, is (1 - rotor factor) times the torque the output
//! holds at rest against the axes' forces (`ConfigMachine::rigid_locking`), which is the same at a steady speed.
//!
//! shortcut: the lock fades in over the gearbox's 0.5 W in place of sticking: a motor torque short of what lifts the
//! load creeps it up, and a push short of what lowers it creeps it down, at the speed whose power is about
//! 0.5 W atanh(the share of the torque it would take), the output getting a little more than the rotor puts in (at
//! most about 0.15 W) while it creeps up. Give the worm a stick state if a cycle starts one from rest at low torque.

use crate::netlist::WormNode;

/// The power at the output over which the efficiency fades in, W: the gearbox's (`mech::Gearbox::loss_factor`).
const FADE_W: f64 = 0.5;

/// (factor on the torque at the output, factor on the torque at the rotor) at the power `power` = t w at the output
/// (positive while the motor drives).
pub fn factors(worm: &WormNode, power: f64) -> (f64, f64) {
    let x = (power / FADE_W).tanh();
    match (x >= 0.0, worm.back_efficiency > 0.0) {
        (true, true) => (worm.efficiency.powf(x), 1.0),
        (true, false) => (worm.efficiency.powf(x), x),
        (false, true) => (worm.back_efficiency.powf(x), 1.0),
        (false, false) => (1.0, -x * worm.back_efficiency),
    }
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
    /// worm that turns back; a locking one (eta_back = -0.2) gives the output the torque t and the rotor -0.2 t / i,
    /// against the motion: the motor pushes. At rest a turning worm passes t / i to the rotor, a locking one nothing; at
    /// a forward power of 0.5 W the locking one passes tanh(1) = 0.761594 of t / i to the rotor and 0.8^0.761594 =
    /// 0.843711 of t to the output, driven back at 0.5 W -0.2 tanh(1) = -0.152319 of t / i to the rotor.
    #[test]
    fn a_worm_passes_its_efficiencies_and_a_locking_one_holds_at_rest() {
        let turning = WormNode { ratio: 40.0, efficiency: 0.8, back_efficiency: 0.5 };
        let locking = WormNode { back_efficiency: -0.2, ..turning };
        assert_eq!(factors(&turning, 100.0), (0.8, 1.0));
        assert_eq!(factors(&turning, -100.0), (2.0, 1.0));
        assert_eq!(factors(&locking, 100.0), (0.8, 1.0));
        assert_eq!(factors(&locking, -100.0), (1.0, -0.2));
        let (_, back) = factors(&locking, -0.5);
        assert!((back + 0.152_318_8).abs() < 1e-7, "{back}");
        let (output, rotor) = factors(&locking, 0.5);
        assert!((rotor - 0.761_594_2).abs() < 1e-7 && (output - 0.843_711).abs() < 1e-6, "{output} {rotor}");
        assert_eq!(factors(&turning, 0.0), (1.0, 1.0));
        assert_eq!(factors(&locking, 0.0), (1.0, 0.0));
    }

    /// Warmed so that it loses half as much (f = 0.5): 1 - (1 - 0.8) 0.5 = 0.9 forward, 1 - 0.5 0.5 = 0.75 back; a
    /// locking worm's -0.2 becomes 1 - 1.2 0.5 = 0.4, which would turn back, so it stays at 0; cold (f = 2) it locks
    /// harder, 1 - 1.2 2 = -1.4, and forward 1 - 0.2 2 = 0.6.
    #[test]
    fn a_warm_worm_loses_less_and_a_locking_one_stays_locked() {
        let turning = WormNode { ratio: 40.0, efficiency: 0.8, back_efficiency: 0.5 };
        let locking = WormNode { back_efficiency: -0.2, ..turning };
        let close = |a: f64, b: f64| assert!((a - b).abs() < 1e-12, "{a} against {b}");
        let w = warmed(&turning, 0.5);
        close(w.efficiency, 0.9);
        close(w.back_efficiency, 0.75);
        close(warmed(&locking, 0.5).back_efficiency, 0.0);
        let cold = warmed(&locking, 2.0);
        close(cold.efficiency, 0.6);
        close(cold.back_efficiency, -1.4);
        assert_eq!(warmed(&turning, 1.0), turning);
    }
}
