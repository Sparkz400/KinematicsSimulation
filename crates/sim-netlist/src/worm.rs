//! A worm stage (`machine.worm`): what it passes on, driven forward by the motor and back by the load.
//!
//! A gearbox passes the torque t of its spring on as t eta to its output and t / i to its rotor (`mech::Gearbox`), and
//! eta fades from 1 at rest to its value within a power t w of 0.5 W at the output. A worm does the same while the motor
//! drives (with its forward efficiency), and while the load drives it back passes t eta_back^-1 (as the gearbox, with
//! its back efficiency). A worm whose back efficiency is 0 or less locks itself: while the load drives it back it passes
//! t to the output and nothing to the rotor, so that the output stands where the rotor holds it, the load's power going
//! into the worm's friction. At rest it holds whatever the load; driving forward it fades in as x t / i at the rotor
//! (x = tanh of the power over 0.5 W) and t eta^x at the output.
//!
//! shortcut: a back efficiency below 0 counts as 0: the motor lowers a load through a self-locking worm with no torque,
//! where it would have to push by -eta_back of the load's power (a drag in proportion to the gear's torque has no
//! consistent rigid solution while the load drives: the worm would have to stick). And the lock fades in over the
//! gearbox's 0.5 W in place of sticking: a motor torque short of what lifts the load creeps it forward at the speed whose
//! power is about that, the output getting a little more than the rotor puts in (at most about 0.15 W) while it does.
//! Give the worm a stick state if a cycle lowers through a self-locking worm or starts one from rest at low torque.

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
        (false, false) => (1.0, 0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Far from rest the factors are the efficiencies: forward 0.8 at the output; back 1 / 0.5 = 2 at the output of a
    /// worm that turns back; a locking one (eta_back = -0.2) gives the output the torque t and the rotor nothing. At rest
    /// a turning worm passes t / i to the rotor, a locking one nothing; at a forward power of 0.5 W the locking one passes
    /// tanh(1) = 0.761594 of t / i to the rotor and 0.8^0.761594 = 0.843711 of t to the output.
    #[test]
    fn a_worm_passes_its_efficiencies_and_a_locking_one_holds_at_rest() {
        let turning = WormNode { ratio: 40.0, efficiency: 0.8, back_efficiency: 0.5 };
        let locking = WormNode { back_efficiency: -0.2, ..turning };
        assert_eq!(factors(&turning, 100.0), (0.8, 1.0));
        assert_eq!(factors(&turning, -100.0), (2.0, 1.0));
        assert_eq!(factors(&locking, 100.0), (0.8, 1.0));
        assert_eq!(factors(&locking, -100.0), (1.0, 0.0));
        let (output, rotor) = factors(&locking, 0.5);
        assert!((rotor - 0.761_594_2).abs() < 1e-7 && (output - 0.843_711).abs() < 1e-6, "{output} {rotor}");
        assert_eq!(factors(&turning, 0.0), (1.0, 1.0));
        assert_eq!(factors(&locking, 0.0), (1.0, 0.0));
    }
}
