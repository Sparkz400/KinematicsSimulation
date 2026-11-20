use super::*;
use sim_components::analysis::analyse_cycle;

#[test]
fn the_hexapod_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

fn mechanism(cfg: &Config) -> Hexapod {
    Hexapod::from_config(cfg).unwrap().1
}

/// The config with each leg's two parts as point masses at their joints (centre 0, no inertia of their own): the model
/// the hand calculations below were made for.
fn point_legs(cfg: &mut Config) {
    for key in ["base_part_centre_m", "base_part_inertia_kgm2", "platform_part_centre_m", "platform_part_inertia_kgm2"]
    {
        cfg.set_text(&format!("leg.{key} = 0.0")).unwrap();
    }
}

/// At the home pose (0, 0, 0.48 m, no turn) every leg spans the same: base joint at 0.4 m and -15 degrees, platform
/// joint at 0.2 m and -50 degrees, 35 degrees apart. Seen from above they are d^2 = 0.4^2 + 0.2^2 - 2 0.4 0.2 cos 35 =
/// 0.0689357 m^2 apart, so L = sqrt(0.48^2 + 0.0689357) = 0.5471158 m and the cylinder is out by L - 0.45 = 0.0971158 m.
#[test]
fn every_leg_has_the_hand_calculated_length_at_the_home_pose() {
    let geo = mechanism(&ENTRY.config()).geo;
    let mut q = [0.0; 6];
    assert!(geo.inverse(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], &mut q));
    for (i, v) in q.iter().enumerate() {
        assert!((v - 0.0971158).abs() < 1e-7, "leg {i} is out by {v}");
    }
}

/// The closed-form inverse and the Newton forward kinematics undo each other at three poses: home, a shift with a
/// tilt, and the far corner of the box with all three turns.
#[test]
fn the_forward_kinematics_undoes_the_inverse_at_three_poses() {
    let geo = mechanism(&ENTRY.config()).geo;
    for pose in
        [[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], [0.04, -0.02, 0.5, 0.05, 0.0, -0.03], [-0.06, 0.06, 0.44, -0.08, 0.08, 0.12]]
    {
        let (mut q, mut back) = ([0.0; 6], [0.0; 6]);
        assert!(geo.inverse(&pose, &mut q));
        geo.forward(&q, &mut back);
        for k in 0..6 {
            assert!((pose[k] - back[k]).abs() < 1e-12, "{pose:?} came back as {back:?}");
        }
    }
    // Legs that no pose fits (one 1 m out, the others closed) are refused, not answered with a wrong pose.
    let mut back = [0.0; 6];
    geo.forward(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], &mut back);
    assert!(back.iter().all(|v| v.is_nan()));
}

/// Holding the platform at rest at home, J^T f = (0, 0, m g, 0, 0, 0). By symmetry the six legs push alike, each with
/// its vertical share u_z = 0.48 / 0.5471158: f = m g / (6 u_z), m = 25 kg platform + 6 x 0.3 x 4 kg of legs + 30 kg
/// payload = 62.2 kg: 62.2 x 9.80665 / (6 x 0.877323) = 115.8772 N. The machine holds it with f lead / (2 pi) =
/// 0.184424 N m on each motor (coupling ratio 1, nothing lost at rest; the cogging switched off, which would add its
/// ripple at the rotor's angle).
#[test]
fn the_legs_carry_the_hand_calculated_static_forces() {
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.torque_ripple = false").unwrap();
    point_legs(&mut cfg);
    let home = [0.0, 0.0, 0.48, 0.0, 0.0, 0.0];
    let f = mechanism(&cfg).static_leg_forces(&home).unwrap();
    for v in f {
        assert!((v - 115.8772).abs() < 1e-3, "{f:?}");
    }
    let machine = (ENTRY.build)(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 6]);
    machine.equilibrium(&home, &mut state, &mut torque);
    for t in &torque {
        assert!((t - 0.184424).abs() < 1e-5, "{torque:?}");
    }
    // Off centre the forces differ, but still carry the weight and no moment: the sum of f_i u_i is (0, 0, m g).
    let pose = [0.05, -0.03, 0.5, 0.04, -0.06, 0.1];
    let f = mechanism(&cfg).static_leg_forces(&pose).unwrap();
    let legs = mechanism(&cfg).geo.legs(&pose);
    let lift: f64 = (0..6).map(|i| f[i] * legs.u[i][2]).sum();
    assert!((lift - 62.2 * 9.80665).abs() < 1e-9 && f.iter().any(|v| (v - f[0]).abs() > 1.0));
}

/// The same hold with the shipped legs' parts, by virtual work: the platform rising dz turns each leg's u_z = h / L by
/// s dz / L (s = 1 - u_z^2 = 0.2302956, L = 0.5471158 m), so the rod part (1.2 kg, centre 0.175 m below its joint)
/// rises by (1 - 0.175 s / L) dz and the body part (2.8 kg, centre 0.1 m above the base joint) by 0.1 s / L dz:
/// f u_z = g (55 kg / 6 + 1.2 (1 - 0.0736622) + 2.8 x 0.0420927) = g 10.3965 kg, f = 116.2065 N (0.28 % above the
/// point legs' 115.8772 N).
#[test]
fn the_legs_parts_add_their_hand_calculated_weight_to_the_hold() {
    let f = mechanism(&ENTRY.config()).static_leg_forces(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0]).unwrap();
    for v in f {
        assert!((v - 116.2065).abs() < 1e-3, "{f:?}");
    }
}

/// A pose that takes a leg beyond its stroke is refused, one that tilts the legs beyond the joints' 40 degrees too.
#[test]
fn poses_beyond_the_strokes_and_the_joints_are_refused() {
    let machine = (ENTRY.build)(&ENTRY.config()).unwrap();
    assert_eq!(machine.unreachable(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], 0.0), None);
    // 0.3 m up: L = sqrt(0.78^2 + 0.0689) = 0.8231 m, beyond 0.45 + 0.2.
    assert_eq!(machine.unreachable(&[0.0, 0.0, 0.78, 0.0, 0.0, 0.0], 0.0), Some("error-joint-beyond-stop"));
    // Low and far out: the legs lie flat.
    assert_eq!(machine.unreachable(&[0.25, 0.0, 0.25, 0.0, 0.0, 0.0], 0.0), Some("error-hexapod-joint-angle"));
}

/// The default cycle passes every verdict: the default is a sized example.
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped);
    for v in sim_components::analysis::verdicts(&cfg, &load).unwrap() {
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
}

/// What the platform puts on a compliant base at home (0, 0, 0.48 m), moving at v = (0.1, 0, 0.1) m/s and turning at
/// w = (0, 0, 0.2) rad/s, the rotors at rest: m = 62.2 kg (as above), so m v = (6.22, 0, 6.22) N s; about the origin
/// p x m v = (0, 0.48 x 6.22, 0) = (0, 2.9856, 0) N m s and I_zz w with I_zz = 0.7 + 6 x 1.2 kg x 0.2^2 m^2 = 0.988
/// kg m^2 (the platform and its joints' share of the legs; the payload is a point at the centre): 0.1976 N m s.
#[test]
fn the_platform_has_the_hand_calculated_momentum() {
    let mut cfg = ENTRY.config();
    point_legs(&mut cfg);
    let mech = mechanism(&cfg);
    let home = [0.0, 0.0, 0.48, 0.0, 0.0, 0.0];
    let mut q = [0.0; 6];
    assert!(mech.geo.inverse(&home, &mut q));
    let j = Geometry::jacobian(&mech.geo.legs(&home));
    let xi = [0.1, 0.0, 0.1, 0.0, 0.0, 0.2];
    let qd: Vec<f64> = (0..6).map(|r| (0..6).map(|c| j[r * 6 + c] * xi[c]).sum()).collect();
    let f = sim_components::chain::Mechanism::frame_momentum(&mech, &q, &qd, &[0.0; 6]).unwrap();
    for (got, want) in f.linear.iter().chain(&f.angular).zip([6.22, 0.0, 6.22, 0.0, 2.9856, 0.1976]) {
        assert!((got - want).abs() < 1e-9, "{f:?}");
    }
    assert!((f.mass - 62.2).abs() < 1e-12 && (f.first_moment[2] - 62.2 * 0.48).abs() < 1e-9 && f.gravity[2] < 0.0);
}

/// A payload with an inertia of its own (`mass.payload_inertia_kgm2`) turns with the platform: the same pose and twist as
/// above with the payload's 0.5, 0.5, 2.0 kg m^2 about the centre adds I_zz w_z = 2.0 x 0.2 = 0.4 N m s to the angular
/// momentum about z and nothing about x and y (R = 1 at home); absent, the payload is the point mass of before.
#[test]
fn the_payload_inertia_adds_its_turning_momentum() {
    let mut cfg = ENTRY.config();
    point_legs(&mut cfg);
    cfg.set_text("mass.payload_inertia_kgm2 = [0.5, 0.5, 2.0]").unwrap();
    let mech = mechanism(&cfg);
    let home = [0.0, 0.0, 0.48, 0.0, 0.0, 0.0];
    let mut q = [0.0; 6];
    assert!(mech.geo.inverse(&home, &mut q));
    let j = Geometry::jacobian(&mech.geo.legs(&home));
    let xi = [0.1, 0.0, 0.1, 0.0, 0.0, 0.2];
    let qd: Vec<f64> = (0..6).map(|r| (0..6).map(|c| j[r * 6 + c] * xi[c]).sum()).collect();
    let f = sim_components::chain::Mechanism::frame_momentum(&mech, &q, &qd, &[0.0; 6]).unwrap();
    for (got, want) in f.angular.iter().zip([0.0, 2.9856, 0.5976]) {
        assert!((got - want).abs() < 1e-9, "{f:?}");
    }
}

/// The legs' two parts as bodies of their own, lifting the platform at home (all legs alike, qd_i = u_z v_z, the twist
/// (0, 0, v_z, 0, ...)): with s = 1 - u_z^2 = 0.2302955, L = 0.5471158 m, u' = (v - (u . v) u) / L so |u'|^2 = v_z^2 s / L^2.
/// The base part (2.8 kg, centre 0.1 m, inertia 0.058) turns about the base joint: T = 1/2 (m c^2 + I) |u'|^2. The rod
/// part (1.2 kg, 0.175 m, 0.012) moves at v - c u': |.|^2 = v_z^2 (1 - 2 c s / L + c^2 s / L^2). The sum of all entries
/// of M_q is 2 T / (u_z v_z)^2, so against the point legs of the old model it is larger by
/// 6 (m_b c_b^2 + I_b) s / (L^2 u_z^2) = 0.5157661 and by 6 [m_p (c_p^2 s / L^2 - 2 c_p s / L) + I_p s / L^2] / u_z^2 =
/// -1.0857397 kg (the rod part's mass sits nearer the base than the joint): -0.5699735 kg together.
#[test]
fn the_legs_parts_add_the_hand_calculated_kinetic_energy() {
    let cfg = ENTRY.config();
    let mut points = cfg.clone();
    point_legs(&mut points);
    let sum = |cfg: &Config| {
        let mech = mechanism(cfg);
        let mut q = [0.0; 6];
        assert!(mech.geo.inverse(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], &mut q));
        let (mut m, mut b) = ([0.0; 36], [0.0; 6]);
        mech.dynamics(&q, &[0.0; 6], &mut m, &mut b);
        m.iter().sum::<f64>()
    };
    let delta = sum(&cfg) - sum(&points);
    assert!((delta + 0.5699735).abs() < 1e-6, "{delta}");
}

/// The dynamics with the legs' parts against Lagrange's equations by differences (as the platform's were checked): at a
/// pose off the middle and a speed, h_i = sum_jk dM_ij/dq_k qd_k qd_j - 1/2 sum_jk dM_jk/dq_i qd_j qd_k + dV/dq_i,
/// and at rest h is the legs' holding force (`static_leg_forces`, with the parts' weights).
#[test]
fn the_dynamics_with_the_legs_parts_agree_with_the_lagrange_equations() {
    let mech = mechanism(&ENTRY.config());
    let tool = [0.03, -0.02, 0.5, 0.04, -0.03, 0.06];
    let mut q = [0.0; 6];
    assert!(mech.geo.inverse(&tool, &mut q));
    let qd = [0.05, -0.08, 0.03, 0.07, -0.02, 0.06];
    let eval = |q: &[f64; 6], qd: &[f64; 6]| {
        let (mut m, mut b) = ([0.0; 36], [0.0; 6]);
        mech.dynamics(q, qd, &mut m, &mut b);
        (m, b)
    };
    let (_, bias) = eval(&q, &qd);
    let eps = 1e-5;
    let shifted = |k: usize, sign: f64| {
        let mut p = q;
        p[k] += sign * eps;
        p
    };
    let dm: Vec<[f64; 36]> = (0..6)
        .map(|k| {
            std::array::from_fn(|e| {
                (eval(&shifted(k, 1.0), &qd).0[e] - eval(&shifted(k, -1.0), &qd).0[e]) / (2.0 * eps)
            })
        })
        .collect();
    for i in 0..6 {
        let dv = (mech.potential(&shifted(i, 1.0)) - mech.potential(&shifted(i, -1.0))) / (2.0 * eps);
        let rate: f64 = (0..6).map(|j| (0..6).map(|k| dm[k][i * 6 + j] * qd[k]).sum::<f64>() * qd[j]).sum();
        let kinetic: f64 = (0..6).map(|j| (0..6).map(|k| dm[i][j * 6 + k] * qd[j] * qd[k]).sum::<f64>()).sum();
        let h = rate - 0.5 * kinetic + dv;
        assert!((bias[i] - h).abs() < 1e-4 * (1.0 + h.abs()), "leg {i}: {} against {h}", bias[i]);
    }
    let (_, rest) = eval(&q, &[0.0; 6]);
    let hold = mech.static_leg_forces(&tool).unwrap();
    for i in 0..6 {
        assert!((rest[i] - hold[i]).abs() < 1e-9 * (1.0 + hold[i].abs()), "{rest:?} against {hold:?}");
    }
}

/// Friction feedforward from the machine's own model, by hand: the platform at home rising at 0.1 m/s. Each leg (L =
/// 0.5471158 m, 0.48 m of it upwards) runs out at q' = 0.48 / 0.5471158 x 0.1 = 0.0877328 m/s. Its guide 20 N plus the
/// nut's preload, 0.05 / sqrt(tan b) Fpr l / 2 pi with tan b = 0.01 / (pi 0.025): 0.133809 N m at the screw, times
/// 2 pi / 0.01 = 628.3185: 84.0749 N; plus 60 N s/m x 0.0877328: 109.3388 N. Through the screw's ratio and its 0.9 (the
/// power, 9.6 W, far above the 0.5 W the efficiency fades in): 109.3388 / (628.3185 x 0.9) = 0.193354 N m. The motor
/// shaft at 628.3185 x 0.0877328 = 55.1241 rad/s: 0.05 tanh(110.2) + 2e-4 x 55.1241 = 0.061025 N m. So 0.254378 N m at
/// every motor.
#[test]
fn the_steady_friction_of_a_rising_platform_is_the_hand_calculated_one() {
    let machine = (ENTRY.build)(&ENTRY.config()).unwrap();
    let mut out = [0.0; 6];
    machine.steady_friction(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], &[0.0, 0.0, 0.1, 0.0, 0.0, 0.0], &mut out);
    for (k, torque) in out.iter().enumerate() {
        assert!((torque - 0.254378).abs() < 1e-5, "motor {k}: {torque} N m");
    }
}

/// The default cycle's figures as they were before the speed work of 16 November 2026, recorded then on Windows, when
/// the dynamics' forward kinematics started Newton from home at every call. Now it starts from the nearest point of a
/// grid of leg lengths (`warm_pose`), so the pose depends on the legs alone and not on the last call, and stops within the same 1e-13 m, a change at the rounding of the pose; the closed loop amplifies
/// such changes: moving only Newton's start from home by 1e-9 to 1e-7 m in the old code moved these figures by up to
/// 7e-5 (the energy motor 2 fed back), the peak torques by up to 3e-6, the path errors by up to 7e-5. The warm start
/// moved them by up to 1.1e-4 (motor 1's peak torque), the rest by up to 2.4e-5. Another platform's maths library,
/// which rounds sine and arc tangent differently in the last bit, moves them the same way. Hence 1e-3.
/// Re-recorded on 17 November 2026 for the legs' two swinging parts: with the parts as points at their joints the
/// cycle gives the old figures to within 1e-4; the body part's swing (inertia and weight) raises the torques by about
/// 1 %, the rod part's centre 0.175 m in from the platform joint lowers them by more: the peak and RMS torques fall by
/// 0.15 to 1.4 %, the energies drawn by up to 0.3 %, those fed back by up to 5.3 % (motor 2); motor 5's rise (peak
/// 0.2 %, RMS 0.3 %, fed back 3.3 %).
/// Re-recorded on 20 November 2026 for the legs' axial modes: the coupling's damper bridges its own spring, not the
/// screw's (`Gearbox::arm`), and the screw's own damping through the nut (`screw.axial_damping_ratio` 0.04) bridges the
/// screw's; the modes' damping ratios fall from 0.86 to 3.9 to 0.010 to 0.047. The peak torques move by -0.14 to
/// +0.67 %, the energies fed back by up to 1.8 % (motor 2), the path and tracking errors rise by 0.2 to 1.7 % (less
/// damped legs).
#[test]
fn the_default_cycle_keeps_its_figures() {
    const BEFORE: [f64; 30] = [
        1.0172814150352312,
        0.3003360995538258,
        11.335097051772516,
        1.0479368499180608,
        0.6666488750305876,
        0.24590014190256862,
        3.799748984409112,
        0.23315962469401844,
        1.125287486036543,
        0.4723190839797147,
        27.705953609589994,
        3.953860361641203,
        0.9694127108232625,
        0.43305965234362404,
        26.90005015310452,
        3.099833995553925,
        0.8904430267324808,
        0.3454924138567322,
        18.324715442889097,
        0.6384086947724753,
        1.1059039396433445,
        0.3565349533586985,
        13.61778422380816,
        0.9049632154862305,
        7.650205136525936e-06,
        1.9085734747373777e-06,
        1.1023298336828707e-05,
        1.270246570539309e-05,
        4.033220556592954e-06,
        2.5671773225890524e-05,
    ];
    assert_figures("hexapod", &cycle_figures(&ENTRY), &BEFORE, 1e-3);
}

/// The default cycle's key figures: per motor its peak and RMS torque and the energy it drew and fed back, then the
/// tool's path error (peak, RMS), tracking error, and the same of its turn.
fn cycle_figures(entry: &MachineEntry) -> Vec<f64> {
    let cfg = entry.config();
    let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    let mut out: Vec<f64> =
        load.motors.iter().flat_map(|m| [m.peak_torque, m.rms_torque, m.energy_in, m.energy_regenerated]).collect();
    out.extend([
        load.path_error_peak,
        load.path_error_rms,
        load.tracking_error_peak,
        load.path_angle_error_peak,
        load.path_angle_error_rms,
        load.tracking_angle_error_peak,
    ]);
    out
}

/// Every figure within `tol` of the recorded one, relative (absolute below 1e-12).
fn assert_figures(id: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{id}: {got:?}");
    for (k, (g, w)) in got.iter().zip(want).enumerate() {
        assert!((g - w).abs() <= tol * w.abs().max(1e-12), "{id}: figure {k} is {g:e}, was {w:e}; all: {got:?}");
    }
}
