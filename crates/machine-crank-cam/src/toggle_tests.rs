//! The toggle press by hand. The shipped geometry: links a = b = 0.15 m, crank r = 0.05 m, rod c = 0.25 m, the crank at
//! t0 = 120 degrees at the dead centre, so its centre C = (c - r cos(t0), -a - r sin(t0)) = (0.275, -0.1933013) m from
//! the top pivot. The independent numbers come from a circle intersection for the knee, the ram by Pythagoras and
//! central differences for the rates (not the closed-form chain of `toggle.rs`).

use super::*;
use crate::toggle::TogglePress;
use sim_components::analysis::{CycleLoad, Verdict};
use sim_components::{Command, Sim};
use std::f64::consts::PI;

fn shipped(changes: &[&str]) -> Config {
    let mut cfg = TOGGLE_PRESS.config();
    for c in changes {
        cfg.set_text(c).unwrap();
    }
    cfg
}

fn press(changes: &[&str]) -> TogglePress {
    TogglePress::from_config(&shipped(changes)).unwrap()
}

/// A constant press force over the whole stroke, the frame rigid, no gravity.
fn constant_force(force: f64) -> TogglePress {
    let mut p = press(&["effects.gravity = false", "effects.frame_stretch = false"]);
    p.process = Some(Table { at: vec![0.0, 0.1], value: vec![force, force] });
    p
}

fn judged(cfg: &Config) -> (CycleLoad, Vec<Verdict>) {
    let mut sim = Sim::new(&TOGGLE_PRESS, cfg).unwrap();
    assert!(sim.limit_violations.is_empty(), "{:?}", sim.limit_violations);
    let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
    let verdicts = sim_components::analysis::verdicts(cfg, &load).unwrap();
    (load, verdicts)
}

#[test]
fn toggle_press_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&TOGGLE_PRESS);
}

/// The ram at three crank angles and the dead centre. With a = b the ram is x = 2 a cos(phi) below the top pivot.
/// q = -90 degrees: the pin P = C + r (cos 30, sin 30) = (0.3183013, -0.1683013), |P| = 0.3600570; the knee on the
/// circles of 0.15 m about O and 0.25 m about P, below O-P: K = (0.0709256, -0.1321724), phi = 0.4925083 rad,
/// x = 0.2643449 m (35.655 mm above the bottom dead centre at a + b = 0.3 m), x' = 0.0364766 m/rad.
/// q = -20 degrees: P = (0.2663176, -0.1440609), K = (0.0163685, -0.1491042), phi = 0.1093409, x = 0.2982085 m (1.792 mm),
/// x' = 0.0108256 m/rad. q = -2 degrees: K = (0.0015278, -0.1499922), phi = 0.0101857, x = 0.2999844 m (0.0156 mm),
/// x' = 0.00090095 m/rad. At q = 0 the links stand in line: phi = 0, x = 0.3 m, x' = 0, and (D = K - P = (-c, 0),
/// K_phi = (a, 0), P_q = r (-sin(t0), cos(t0))) phi' = D . P_q / D . K_phi = c r sin(t0) / (-c a) = -r sin(t0) / a
/// = 0.0108253 / -0.0375 = -0.2886751, x'' = -(a + a^2 / b) phi'^2 = -0.025 m/rad^2.
#[test]
fn the_toggle_kinematics_match_the_hand_worked_linkage() {
    let p = press(&[]);
    for (deg, phi, x, x1) in [
        (-90.0, 0.4925083, 0.2643449, 0.0364766),
        (-20.0, 0.1093409, 0.2982085, 0.0108256),
        (-2.0, 0.0101857, 0.2999844, 0.00090095),
        (0.0, 0.0, 0.3, 0.0),
    ] {
        let pose = p.pose(f64::to_radians(deg));
        assert!((pose.phi[0] - phi).abs() < 1e-7, "{deg}: phi {}", pose.phi[0]);
        assert!((pose.x[0] - x).abs() < 1e-7, "{deg}: x {}", pose.x[0]);
        assert!((pose.x[1] - x1).abs() < 1e-7, "{deg}: x' {}", pose.x[1]);
    }
    let dc = p.pose(0.0);
    assert!((dc.phi[1] + 0.2886751).abs() < 1e-7 && (dc.x[2] + 0.025).abs() < 1e-9, "{dc:?}");
    // The rates are the differences of the positions, over the stroke and through the dead centre.
    let h = 1e-6;
    for q in [-2.1, -1.0, -0.3, 0.0, 0.15] {
        let (lo, hi, at) = (p.pose(q - h), p.pose(q + h), p.pose(q));
        for (k, (a, b, rate)) in
            [(lo.phi[0], hi.phi[0], at.phi[1]), (lo.phi[1], hi.phi[1], at.phi[2]), (lo.x[0], hi.x[0], at.x[1])]
                .into_iter()
                .chain([(lo.x[1], hi.x[1], at.x[2])])
                .enumerate()
        {
            assert!(((b - a) / (2.0 * h) - rate).abs() < 1e-7, "{q} rate {k}: {} against {rate}", (b - a) / (2.0 * h));
        }
    }
}

/// The inverse has two branches about the dead centre. 1 mm above it, x = 0.299 m: cos(phi) = (a^2 + x^2 - b^2) / (2 a x)
/// = x / (2 a) = 0.9966667, phi = +-0.0816724; before the dead centre the crank stands at -0.2646374 rad (-15.163
/// degrees), after it on the other side. 0.1 mm: phi = 0.0258206, q = -0.0871754 rad. Each branch's angle puts the ram
/// back where it was asked; the dead centre is one angle of both.
#[test]
fn the_inverse_has_a_branch_on_each_side_of_the_dead_centre() {
    let p = press(&[]);
    let before = p.crank_angle(0.001, true).unwrap();
    assert!((before + 0.2646374).abs() < 1e-7, "{before}");
    assert!((p.pose(before).phi[0] - 0.0816724).abs() < 1e-7);
    assert!((p.crank_angle(0.0001, true).unwrap() + 0.0871754).abs() < 1e-7);
    let after = p.crank_angle(0.001, false).unwrap();
    assert!(after > 0.0 && (p.pose(after).phi[0] + 0.0816724).abs() < 1e-7, "{after}");
    for q in [before, after] {
        assert!((0.3 - p.pose(q).x[0] - 0.001).abs() < 1e-12, "{q}");
    }
    assert!(p.crank_angle(0.0, true).unwrap().abs() < 1e-9 && p.crank_angle(0.0, false).unwrap().abs() < 1e-9);
    // Round trip over the stroke, the ram's distance to the crank and back.
    for k in 0..=40 {
        let q = -2.0943951 * k as f64 / 40.0;
        let back = p.crank_angle(0.3 - p.pose(q).x[0], true).unwrap();
        assert!((back - q).abs() < 1e-8, "{q} -> {back}");
    }
    // Beyond the links' reach: none.
    assert!(p.crank_angle(0.31, true).is_none() && p.crank_angle(-0.001, true).is_none());
}

/// The amplification: 10 N m of the motor through the ratio 10, loss-free, are 100 N m at the crank, which hold
/// F = 100 / x' against the ram: at -90 degrees 100 / 0.0364766 = 2741.48 N, at -5 degrees (x' = 0.0023513 m/rad)
/// 42530.15 N, 15.5 times as much for the same torque. The process torque of a constant force there is -F x'.
#[test]
fn the_toggle_amplifies_the_crank_torque_towards_the_dead_centre() {
    for (deg, force) in [(-90.0, 2741.4806), (-5.0, 42530.154)] {
        let p = constant_force(force);
        let q = f64::to_radians(deg);
        let mut torque = [0.0];
        // Moving down (q' > 0): the force acts.
        p.process_torque(&[q], &[0.1], &mut torque);
        assert!((torque[0] + 100.0).abs() < 1e-3, "{deg}: {torque:?}");
        assert!((p.holding_torque(q, force) - 100.0).abs() < 1e-3);
    }
    assert!((42530.154_f64 / 2741.4806 - 15.5136).abs() < 1e-3);
}

/// Holding 10 kN takes F x' at the crank: 1 mm before the dead centre (x' = 0.00793873 m/rad) 79.3873 N m, 8.18 N m at
/// the motor through 10 and 0.97; 0.1 mm before it (x' = 0.00234866) 23.4866 N m; at the dead centre nothing.
#[test]
fn the_holding_torque_falls_towards_the_dead_centre() {
    let p = press(&[]);
    for (d, torque) in [(0.001, 79.3873), (0.0001, 23.4866)] {
        let q = p.crank_angle(d, true).unwrap();
        assert!((p.holding_torque(q, 1e4) - torque).abs() < 1e-3, "{d}: {}", p.holding_torque(q, 1e4));
    }
    assert!((79.3873_f64 / (10.0 * 0.97) - 8.1843).abs() < 1e-3);
    assert_eq!(p.holding_torque(0.0, 1e4), 0.0);
}

/// The frame of 5e7 N/m stretches 20000 / 5e7 = 0.4 mm under the rated force: the "frame-deflection" limit, reached
/// there. At the dead centre the table's first segment, 12000 N falling by 2e6 N/m, meets the stretch:
/// F = 12000 - 2e6 F / 5e7, F = 12000 / 1.04 = 11538.46 N, and the ram stands F / k = 0.230769 mm above the links'
/// bottom. A rigid frame (effects.frame_stretch off) gives the table's 12000 N with the ram at the bottom.
#[test]
fn the_frame_stretches_under_the_press_force_and_the_ram_stops_short() {
    let cfg = shipped(&["effects.gravity = false"]);
    assert!((cfg.f64("ratings.rated_press_force_n").unwrap() / 5e7 - 0.0004).abs() < 1e-15);
    assert_eq!(cfg.f64("ratings.frame_max_deflection_m").unwrap(), 0.0004);
    let p = TogglePress::from_config(&cfg).unwrap();
    let (force, stretch) = p.press_force(0.0, 0.0);
    assert!((force - 11538.4615).abs() < 1e-3 && (stretch - 2.307692e-4).abs() < 1e-9, "{force} {stretch}");
    let mut out = Vec::new();
    p.signals(&[0.0], &[0.0], &[0.0], &[0.0], &mut out);
    let ids: Vec<String> = p.signal_infos().into_iter().map(|s| s.id.to_string()).collect();
    let signal = |id: &str| out[ids.iter().position(|s| s == id).unwrap()];
    assert!((signal("ram.height") - 2.307692e-4).abs() < 1e-9);
    assert!((signal("frame.deflection") - 2.307692e-4).abs() < 1e-9);
    assert!((signal("press.force") - 11538.4615).abs() < 1e-3);
    let rigid = press(&["effects.gravity = false", "effects.frame_stretch = false"]);
    assert_eq!(rigid.press_force(0.0, 0.0), (12000.0, 0.0));
    // The rated force at the stiffness: the check's limit exactly.
    let check = p.rating_checks().into_iter().find(|c| c.check == "frame-deflection").unwrap();
    assert_eq!((check.signal.as_str(), check.limit), ("frame.deflection", 0.0004));
}

/// J(q) = J_crank + m_pin r^2 + (m_up / 3 + m_knee) a^2 phi'^2 + m_x x'^2: m_pin = 1.25 kg (half the rod), m_up = 2 kg,
/// m_knee = 1.25 + 1 kg (the rod's and the lower link's halves), m_x = 20 + 1 + 8 = 29 kg (ram, half the link, die).
/// At the dead centre (phi' = -0.2886751, x' = 0): 0.01 + 0.003125 + 2.9166667 * 0.0225 / 12 = 0.01859375 kg m^2;
/// far from it, at -90 degrees (phi' = -0.2571471, x' = 0.0364766): 0.05605024 kg m^2, three times as much, the most of
/// it the ram's 29 * 0.0364766^2 = 0.0385857.
#[test]
fn the_reflected_inertia_far_from_and_at_the_dead_centre_match_the_hand_calculation() {
    let p = press(&[]);
    assert!((p.inertia(0.0) - 0.01859375).abs() < 1e-9, "{}", p.inertia(0.0));
    assert!((p.inertia(-PI / 2.0) - 0.05605024).abs() < 1e-7, "{}", p.inertia(-PI / 2.0));
    // The speed term is dJ/dq q'^2 / 2.
    let (q, h) = (-1.0, 1e-6);
    let (mut mass, mut bias) = ([0.0], [0.0]);
    let quiet = press(&["effects.gravity = false"]);
    quiet.dynamics(&[q], &[3.0], &mut mass, &mut bias);
    let slope = (quiet.inertia(q + h) - quiet.inertia(q - h)) / (2.0 * h);
    assert!((bias[0] - slope * 9.0 / 2.0).abs() < 1e-7, "{bias:?} {slope}");
    // Gravity's torque is dV/dq.
    let (mut heavy, h) = ([0.0], 1e-6);
    p.dynamics(&[q], &[0.0], &mut mass, &mut heavy);
    assert!((heavy[0] - (p.potential(&[q + h]) - p.potential(&[q - h])) / (2.0 * h)).abs() < 1e-6, "{heavy:?}");
}

/// The force capacity: the drive's 15 N m through 10 and 0.97 are 145.5 N m at the crank (below the gearbox's 192), so
/// at -90 degrees 145.5 / 0.0364766 = 3988.85 N, at -5 degrees (63 kN by the drive) the rated 20 kN. The capacity's use
/// at its limit is 1. At the dead centre at rest without gravity the links carry the press force in line, the lower one
/// F b / w = F, the upper one F, the rod nothing; at -20 degrees the rod's push about the crank's centre is the torque
/// F x' (virtual work), and the forces stay within the shipped ratings (30 kN links, 8 kN rod).
#[test]
fn the_force_capacity_and_the_link_forces_match_the_hand_calculation() {
    let p = press(&["effects.gravity = false"]);
    assert_eq!(p.crank_torque, 145.5);
    assert!((p.capacity(-PI / 2.0) - 3988.8543).abs() < 1e-3, "{}", p.capacity(-PI / 2.0));
    assert_eq!(p.capacity(f64::to_radians(-5.0)), 20000.0);
    let at_limit = constant_force(3988.8543);
    let mut out = Vec::new();
    at_limit.signals(&[-PI / 2.0], &[0.1], &[0.0], &[0.0], &mut out);
    assert!((out[6] - 1.0).abs() < 1e-6, "capacity use {}", out[6]);
    let [upper, lower, rod] = p.link_forces(0.0, 0.0, 0.0);
    assert!((upper - 11538.4615).abs() < 1e-3 && (lower - 11538.4615).abs() < 1e-3 && rod.abs() < 1e-6);
    let q = f64::to_radians(-20.0);
    let force = constant_force(10000.0);
    let [upper, lower, rod] = force.link_forces(q, 1e-3, 0.0);
    let pose = force.pose(q);
    let (s, c) = (force.t0 + q).sin_cos();
    let along = [(pose.knee[0] - pose.pin[0]) / force.c, (pose.knee[1] - pose.pin[1]) / force.c];
    // The rod pushes the pin back along -along; its torque on the crank, (-rod along) . P_q, is what the press force
    // puts there through the links (-F x', -108.256 N m).
    let crank = -rod * (along[0] * -force.r * s + along[1] * force.r * c);
    let mut process = [0.0];
    force.process_torque(&[q], &[1e-3], &mut process);
    assert!((crank - process[0]).abs() < 1e-6 && (process[0] + 108.25575).abs() < 1e-4, "{crank} {process:?}");
    assert!(upper > 0.0 && lower > 9999.0 && upper < 30000.0 && rod.abs() < 8000.0, "{upper} {lower} {rod}");
}

/// A forming force early in the stroke, where the mechanism gives little: the 12 kN table moved 20 mm up asks
/// 12000 * 0.0318 m/rad = 380 N m of the crank about q = -58 degrees, far beyond its 145.5: "press-capacity" fails, and
/// so does the motor's peak torque. The shipped table passes every verdict (with the cam's in
/// `every_toggle_press_cycle_passes_its_verdicts`).
#[test]
fn a_forming_force_early_in_the_stroke_overloads_the_drive() {
    let early = shipped(&["process.distance_m = [0.02, 0.021, 0.022, 0.023]"]);
    let (_, verdicts) = judged(&early);
    let failed = |check: &str| verdicts.iter().any(|v| v.check == check && !v.ok);
    assert!(failed("press-capacity"), "{verdicts:?}");
    assert!(failed("motor-peak"), "{verdicts:?}");
}

/// The shipped config judges the toggle's own checks; that its cycle and its cam example pass them (and every other
/// verdict) is `every_default_cycle_and_every_cam_passes_its_verdicts`.
#[test]
fn the_shipped_toggle_press_has_its_own_checks() {
    let m = (TOGGLE_PRESS.build)(&TOGGLE_PRESS.config()).unwrap();
    let checks: Vec<&str> = m.rating_checks().iter().map(|c| c.check).collect();
    assert_eq!(checks, ["press-capacity", "upper-link-force", "lower-link-force", "rod-force", "frame-deflection"]);
}

/// The dwell at the dead centre: the ram held with the press force (11538 N, the frame stretched 0.23 mm) while the
/// motor gives almost nothing; the same force 1 mm before the dead centre would take 11538 * 0.00793873 / (10 * 0.97)
/// = 9.44 N m of it.
#[test]
fn the_toggle_holds_the_press_force_at_the_dead_centre_with_little_motor_torque() {
    let mut sim = Sim::new(&TOGGLE_PRESS, &TOGGLE_PRESS.config()).unwrap();
    let (force, stretch) = (sim.signal_index("press.force").unwrap(), sim.signal_index("frame.deflection").unwrap());
    sim.command(Command::MoveTo(vec![-0.6])).unwrap();
    sim.run(1.0, |_| {});
    sim.command(Command::MoveTo(vec![0.0])).unwrap();
    sim.run(0.8, |_| {});
    let (mut sample, mut worst, mut least) = (Vec::new(), 0f64, f64::INFINITY);
    sim.run(0.2, |s| {
        s.sample(&mut sample);
        worst = worst.max(s.torque[0].abs());
        least = least.min(sample[force]);
        assert!((sample[stretch] - sample[force] / 5e7).abs() < 1e-12);
    });
    assert!((least - 11538.46).abs() < 5.0, "press force {least}");
    assert!(worst < 0.1 * 9.44, "the motor gives {worst} N m in the dwell");
}

/// The energy of a forming stroke: down the table from 3 mm to the dead centre, with the frame's stretch. Into the part
/// the table's area less what the stretch leaves: 22 J (11 + 8 + 3 over its three millimetres) less
/// 12000 u - 1e6 u^2 = 2.715976 J over the last u = 0.2307692 mm, 19.284024 J; into the frame F^2 / (2 k) = 1.331361 J;
/// 20.615385 J in all. A loss-free run (rigid, no friction, no gravity) from rest 5.5 mm above to rest at the dead centre
/// takes that much work from the motor: torque times its angle, step by step.
#[test]
fn a_forming_stroke_takes_the_hand_worked_energy_from_the_motor() {
    let mut cfg = lossless_config(&TOGGLE_PRESS);
    for c in ["effects.process_force = true", "effects.gravity = false", "effects.joint_elasticity = false"] {
        cfg.set_text(c).unwrap();
    }
    let mut sim = Sim::new(&TOGGLE_PRESS, &cfg).unwrap();
    sim.command(Command::MoveTo(vec![-0.6])).unwrap();
    sim.run(1.0, |_| {});
    let angle = |s: &Sim| {
        let (mut pos, mut vel) = ([0.0], [0.0]);
        s.machine.motor_state(&s.state, &mut pos, &mut vel);
        pos[0]
    };
    sim.command(Command::MoveTo(vec![0.0])).unwrap();
    let (mut work, mut last) = (0.0, angle(&sim));
    for _ in 0..(1.0 / sim.dt) as usize {
        let torque = sim.torque[0];
        sim.step();
        let now = angle(&sim);
        work += torque * (now - last);
        last = now;
    }
    assert!((work - 20.615385).abs() < 0.01 * 20.615385, "the motor did {work} J");
}

/// Friction feedforward from the machine's own model (`steady_friction`), without gravity: at a crank speed of 2 rad/s
/// the crank's bearings and the links' pins slide with 1 N m + 0.2 N m s/rad * 2 rad/s = 1.4 N m, through 10 and 0.97:
/// 0.1443299 N m; the motor shaft at 20 rad/s 0.05 + 2e-4 * 20 = 0.054 N m. 0.1983299 N m in all, its negative backwards.
#[test]
fn the_steady_friction_is_the_hand_calculated_one() {
    let m = (TOGGLE_PRESS.build)(&shipped(&["effects.gravity = false"])).unwrap();
    let mut out = [0.0];
    for v in [2.0, -2.0] {
        m.steady_friction(&[-1.0], &[v], &mut out);
        assert!((out[0] - 0.1983299 * v.signum()).abs() < 1e-6, "{v}: {out:?}");
    }
}
