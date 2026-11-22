use super::*;
use sim_components::analysis::analyse_cycle;

#[test]
fn the_tripod_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

fn mechanism(cfg: &Config) -> Tripod {
    Tripod::from_config(cfg).unwrap().1
}

/// The config with each leg's two parts as point masses at their joints (centre 0, no inertia of their own): the model
/// the hand calculations below were made for.
fn point_legs(cfg: &mut Config) {
    for key in ["base_part_centre_m", "base_part_inertia_kgm2", "platform_part_centre_m", "platform_part_inertia_kgm2"]
    {
        cfg.set_text(&format!("leg.{key} = 0.0")).unwrap();
    }
}

fn centred() -> Config {
    let mut cfg = ENTRY.config();
    point_legs(&mut cfg);
    cfg.set_text("mass.payload_offset_m = [0.0, 0.0, 0.0]").unwrap();
    cfg.set_text("effects.torque_ripple = false").unwrap();
    cfg
}

const LEVEL: [f64; 3] = [0.54, 0.0, 0.0];

/// Level at z = 0.54 m every leg runs from its base joint at 0.3 m to its platform joint at 0.2 m on the same radius:
/// L = sqrt(0.54^2 + 0.1^2) = 0.5491812 m, the cylinder out by L - 0.45 = 0.0991812 m. Tilted by 0.15 rad about x (no
/// turn: sin ry = 0) the centre moves by p_x = rp (1 - cos 0.15) / 2 = 0.0011229 m; leg 1 (at 0 degrees) then spans
/// (p_x - 0.1, 0, 0.54): 0.5489779 m; leg 2 (120 degrees) (p_x + 0.05, 0.173205 cos 0.15 - 0.259808, 0.54 + 0.173205
/// sin 0.15): 0.5750463 m; leg 3 (240 degrees) (p_x + 0.05, -0.173205 cos 0.15 + 0.259808, 0.54 - 0.173205 sin 0.15):
/// 0.5241851 m.
#[test]
fn the_legs_have_the_hand_calculated_lengths() {
    let geo = mechanism(&ENTRY.config()).geo;
    let mut q = [0.0; 3];
    assert!(geo.inverse(&LEVEL, &mut q));
    for (i, v) in q.iter().enumerate() {
        assert!((v - 0.0991812).abs() < 1e-7, "leg {i} is out by {v}");
    }
    assert!(geo.inverse(&[0.54, 0.15, 0.0], &mut q));
    for (v, want) in q.iter().zip([0.5489779, 0.5750463, 0.5241851]) {
        assert!((v + 0.45 - want).abs() < 1e-7, "{q:?}");
    }
}

/// The dependent coordinates at the tilts (0.15, 0.1) rad, from the closed form: tan g = -sin 0.15 sin 0.1 / (cos
/// 0.15 + cos 0.1) gives g = -0.0075203 rad, p_x = rp (cos 0.1 cos g + sin 0.15 sin 0.1 sin g - cos 0.15 cos g) / 2 =
/// 0.00061207 m, p_y = rp cos 0.1 sin g = -0.00149654 m; to second order p_x = rp (rx^2 - ry^2) / 4 = 0.000625 m, p_y =
/// -rp rx ry / 2 = -0.0015 m and g = -rx ry / 2 = -0.0075 rad. Each platform joint lies in its leg's plane (e_i . P_i
/// = 0), and the platform's normal is the pure tilt's: the turn is about the normal alone.
#[test]
fn the_parasitic_motion_is_the_hand_calculated_one() {
    let geo = mechanism(&ENTRY.config()).geo;
    let tool = [0.54, 0.15, 0.1];
    let place = geo.place(&tool);
    let [x, y, turn, _] = geo.parasitic(&tool);
    assert!((x - 0.00061207).abs() < 1e-8 && (y + 0.00149654).abs() < 1e-8, "{x} {y}");
    assert!((turn + 0.0075203).abs() < 1e-7, "{turn}");
    assert!((x - 0.000625).abs() < 2e-5 && (y + 0.0015).abs() < 2e-5 && (turn + 0.0075).abs() < 1e-4);
    for i in 0..3 {
        let joint = add(place.p, mul(&place.r, geo.anchors[i]));
        assert!(dot(geo.tangents[i], joint).abs() < 1e-15, "joint {i} leaves its plane");
    }
    let pure = matmul(&rot_x(0.15), &rot_y(0.1));
    for k in 0..3 {
        assert!((place.r[k][2] - pure[k][2]).abs() < 1e-15);
    }
    // Tilted about x alone there is no turn, and the pose's error is the centre's shift: 0.0011229 m.
    assert!((geo.parasitic(&[0.54, 0.15, 0.0])[3] - 0.0011229).abs() < 1e-7);
}

/// The inverse and the forward kinematics undo each other on a grid of 9 x 9 x 9 poses over the working range; legs
/// that no pose fits are refused, not answered with a wrong pose: one 1 m out and the others closed would put its
/// platform joint 1.45 m from its base joint, more than the 0.52 m between base joints, the 0.346 m between platform
/// joints and the 0.45 m of a closed leg allow.
#[test]
fn the_forward_kinematics_undoes_the_inverse_over_the_working_range() {
    let geo = mechanism(&ENTRY.config()).geo;
    for i in 0..9 {
        for j in 0..9 {
            for k in 0..9 {
                let pose = [0.5 + 0.01 * i as f64, -0.2 + 0.05 * j as f64, -0.2 + 0.05 * k as f64];
                let (mut q, mut back) = ([0.0; 3], [0.0; 3]);
                assert!(geo.inverse(&pose, &mut q));
                geo.forward(&q, &mut back);
                for c in 0..3 {
                    assert!((pose[c] - back[c]).abs() < 1e-12, "{pose:?} came back as {back:?}");
                }
            }
        }
    }
    let mut back = [0.0; 3];
    geo.forward(&[1.0, 0.0, 0.0], &mut back);
    assert!(back.iter().all(|v| v.is_nan()), "{back:?}");
}

/// Why the forward kinematics converges over the working range W (z 0.5 to 0.58 m, tilts +-0.2 rad). Weigh the error
/// as e = (dz, rp drx, rp dry) in the max norm: W's centre (the start) lies within r = 0.04 m of every pose of W. A chord
/// step maps the error e to (1 - K0^-1 K_mean) e, K_mean the mean of K along the segment to the solution, so it shrinks
/// it by at least kappa = max ||W (1 - K0^-1 K(x)) W^-1|| over a convex set holding all those segments: W grown by r
/// (z 0.46 to 0.62 m, tilts +-0.4 rad) holds every ball of radius r about a pose of W, and the iterates never leave
/// that ball. On a grid of 17^3 points of the grown box kappa is at most 0.595, and it changes by less than 0.07 between
/// neighbouring points, so kappa < 0.7 < 1 over the box: the chord iterations converge from the start to every pose of
/// W, by at least 0.7 a step. The faster steps with K at the first iterate are taken only where they halve the legs'
/// miss, so they cannot keep the iteration from converging: at the first that does not, the chord takes over.
#[test]
fn the_chord_iteration_contracts_over_the_grown_working_range() {
    let geo = mechanism(&ENTRY.config()).geo;
    let n = 17;
    let at = |i: usize, j: usize, k: usize| {
        let f = |v: usize| v as f64 / (n - 1) as f64;
        geo.chord_contraction(&[0.46 + 0.16 * f(i), -0.4 + 0.8 * f(j), -0.4 + 0.8 * f(k)])
    };
    let (mut most, mut step) = (0.0f64, 0.0f64);
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let here = at(i, j, k);
                most = most.max(here);
                for (a, b, c) in [(i + 1, j, k), (i, j + 1, k), (i, j, k + 1)] {
                    if a < n && b < n && c < n {
                        step = step.max((at(a, b, c) - here).abs());
                    }
                }
            }
        }
    }
    assert!(most < 0.6 && step < 0.075 && most + step < 0.7, "kappa up to {most}, changing by {step}");
}

/// Level, the Jacobian K (leg rates per (z', rx', ry')) is the geometry's: each leg's rate is the platform joint's
/// vertical speed times u_z = 0.54 / 0.5491812 = 0.9832820 (the parasitic motion is of second order and moves no leg
/// at first order): z' moves every joint by z', rx' the joint at (rp cos t, rp sin t) by rp sin t rx', ry' by
/// -rp cos t ry'. Rows (0.983282, 0, -0.196656), (0.983282, 0.170309, 0.098328), (0.983282, -0.170309, 0.098328).
/// With the tilts weighed by rp, K^T K = u_z^2 diag(3, 3/2, 3/2): the condition number is sqrt 2.
#[test]
fn the_jacobian_at_the_level_pose_is_the_geometrys() {
    let geo = mechanism(&ENTRY.config()).geo;
    let k = geo.jacobian(&LEVEL).unwrap();
    let want = [0.983282, 0.0, -0.196656, 0.983282, 0.170309, 0.098328, 0.983282, -0.170309, 0.098328];
    for (got, want) in k.iter().zip(want) {
        assert!((got - want).abs() < 1e-6, "{k:?}");
    }
    assert!((geo.condition(&LEVEL) - 2f64.sqrt()).abs() < 1e-9);
    // Its condition number stays below 1.7 over the working range: no singular pose in it.
    for c in [[0.5, 0.2, 0.2], [0.58, -0.2, 0.2], [0.5, -0.2, -0.2], [0.58, 0.2, -0.2]] {
        assert!(geo.condition(&c) < 1.7, "{c:?}");
    }
}

/// Holding the level platform at rest with its load centred: by symmetry each leg carries a third of the weight
/// vertically, along its inclination u_z = 0.9832820: f = m g / (3 u_z), m = 20 kg platform + 3 x 0.3 x 3 kg of legs +
/// 40 kg payload = 62.7 kg: 62.7 x 9.80665 / (3 x 0.983282) = 208.4437 N. The motor holds it with f lead / (2 pi) =
/// 0.331749 N m (coupling 1, nothing lost at rest, the ripple off).
#[test]
fn the_legs_hold_a_centred_load_with_the_hand_calculated_forces() {
    let cfg = centred();
    let f = mechanism(&cfg).static_leg_forces(&LEVEL).unwrap();
    for v in f {
        assert!((v - 208.4437).abs() < 1e-3, "{f:?}");
    }
    let machine = (ENTRY.build)(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 3]);
    machine.equilibrium(&LEVEL, &mut state, &mut torque);
    for t in &torque {
        assert!((t - 0.331749).abs() < 1e-5, "{torque:?}");
    }
}

/// The same hold with the shipped legs' parts, by virtual work: the platform rising dz turns each leg's u_z = h / L by
/// s dz / L (s = 0.0331565, L = 0.5491812 m), so the rod part (0.9 kg, centre 0.175 m below its joint) rises by
/// (1 - 0.0105655) dz and the body part (2.1 kg, centre 0.1 m above the base joint) by 0.0060374 dz:
/// f u_z = g (60 kg / 3 + 0.9 x 0.9894345 + 2.1 x 0.0060374) = g 20.903170 kg, f = 208.4754 N.
#[test]
fn the_legs_parts_add_their_hand_calculated_weight_to_the_hold() {
    let mut cfg = ENTRY.config();
    cfg.set_text("mass.payload_offset_m = [0.0, 0.0, 0.0]").unwrap();
    let f = mechanism(&cfg).static_leg_forces(&LEVEL).unwrap();
    for v in f {
        assert!((v - 208.4754).abs() < 1e-3, "{f:?}");
    }
}

/// The default payload sits 0.05 m off centre in x (and 0.1 m up, which a level platform does not feel): the centre of
/// mass of all 62.7 kg is c = 40 x 0.05 / 62.7 = 0.031898 m out. The legs' vertical shares V_i balance the weight and
/// its moment, V_i = m g (1/3 + 2 c cos t_i / (3 rp)) (sum V_i = m g, sum V_i rp cos t_i = m g c, sum V_i rp sin t_i =
/// 0), the radial and tangential rest taken by the legs' inclination and the base joints; f_i = V_i / u_z: 274.9330 N
/// on leg 1 (towards the load), 175.1991 N on legs 2 and 3.
#[test]
fn the_legs_hold_an_offset_load_with_the_hand_calculated_forces() {
    let mut cfg = ENTRY.config();
    point_legs(&mut cfg);
    cfg.set_text("effects.torque_ripple = false").unwrap();
    let f = mechanism(&cfg).static_leg_forces(&LEVEL).unwrap();
    for (v, want) in f.iter().zip([274.9330, 175.1991, 175.1991]) {
        assert!((v - want).abs() < 1e-3, "{f:?}");
    }
}

/// Rising level from rest at a = 2 m/s^2 with the load centred, the platform only shifts: each leg pushes
/// m (g + a) / (3 u_z) = 62.7 x 11.80665 / (3 x 0.983282) = 250.9544 N, with q'' = u_z a = 1.966564 m/s^2.
#[test]
fn a_leg_pushes_the_hand_calculated_force_for_an_acceleration_along_z() {
    let mech = mechanism(&centred());
    let mut q = [0.0; 3];
    assert!(mech.geo.inverse(&LEVEL, &mut q));
    let (mut mass, mut bias) = ([0.0; 9], [0.0; 3]);
    Mechanism::dynamics(&mech, &q, &[0.0; 3], &mut mass, &mut bias);
    let qdd = [1.966564; 3];
    for i in 0..3 {
        let f: f64 = (0..3).map(|j| mass[i * 3 + j] * qdd[j]).sum::<f64>() + bias[i];
        assert!((f - 250.9544).abs() < 1e-3, "leg {i}: {f} N");
    }
}

/// What the platform puts on a compliant base level at z = 0.54 m, rising at 0.1 m/s, the rotors at rest: m = 62.7 kg,
/// m v = (0, 0, 6.27) N s; its first moment m p + s = (2.0, 0, 62.7 x 0.54 + 4.0) = (2.0, 0, 37.858) kg m (the payload's
/// 40 kg at (0.05, 0, 0.1), the legs' shares symmetric); about the origin s x v = (2, 0, 4) x (0, 0, 0.1) = (0, -0.2,
/// 0) N m s from the offset, p x m v = 0.
#[test]
fn the_platform_has_the_hand_calculated_momentum() {
    let mut cfg = ENTRY.config();
    point_legs(&mut cfg);
    let mech = mechanism(&cfg);
    let mut q = [0.0; 3];
    assert!(mech.geo.inverse(&LEVEL, &mut q));
    let qd = [0.1 * 0.983282; 3];
    let f = Mechanism::frame_momentum(&mech, &q, &qd, &[0.0; 3]).unwrap();
    for (got, want) in f.linear.iter().chain(&f.angular).zip([0.0, 0.0, 6.27, 0.0, -0.2, 0.0]) {
        assert!((got - want).abs() < 1e-5, "{f:?}");
    }
    assert!((f.mass - 62.7).abs() < 1e-12 && (f.first_moment[0] - 2.0).abs() < 1e-9);
    assert!((f.first_moment[2] - 37.858).abs() < 1e-9 && f.gravity[2] < 0.0);
}

/// The new verdicts at a pose, from the rigid model's checks at rest, against their limits by hand. Level: each leg
/// 0.0991812 m out, 0.0008188 m short of the middle of its 0.2 m stroke; swung in by atan(0.1 / 0.54) = 10.4915
/// degrees at its base joint and as steep against the platform's normal; no parasitic error; the forces those of the
/// offset load above. The limits: 1500 N, 0.1 m, 30 and 35 degrees, 0.005 m. Beyond a joint's limit the pose is
/// refused: with the base joints limited to 10.4 degrees the level pose is out, at 10.5 it is in.
#[test]
fn the_new_verdicts_have_the_hand_calculated_values_and_limits() {
    let mut cfg = ENTRY.config();
    point_legs(&mut cfg);
    cfg.set_text("effects.torque_ripple = false").unwrap();
    let machine = (ENTRY.build)(&cfg).unwrap();
    let mut values = Vec::new();
    machine.rigid_checks(&LEVEL, &[0.0; 3], &[0.0; 3], &mut values);
    let checks = machine.rating_checks();
    let swing = 0.1f64.atan2(0.54);
    for (check, value) in checks.iter().zip(&values) {
        let i = check.motor.unwrap_or(0);
        let (want, limit) = match check.check {
            "tripod-leg-force" => ([274.9330, 175.1991, 175.1991][i], 1500.0),
            "tripod-leg-stroke" => (-0.0008188, 0.1),
            "tripod-revolute-angle" => (-swing, 30f64.to_radians()),
            "tripod-cone-angle" => (swing, 35f64.to_radians()),
            "tripod-parasitic" => (0.0, 0.005),
            _ => continue,
        };
        let tolerance = if check.check == "tripod-leg-force" { 1e-3 } else { 1e-7 };
        assert!((value - want).abs() < tolerance && (check.limit - limit).abs() < 1e-12, "{}: {value}", check.check);
    }
    for check in ["tripod-leg-force", "tripod-leg-stroke", "tripod-revolute-angle", "tripod-cone-angle"] {
        assert_eq!(checks.iter().filter(|c| c.check == check).count(), 3, "{check}");
    }
    assert_eq!(checks.iter().filter(|c| c.check == "tripod-parasitic").count(), 1);
    for (deg, refused) in [(10.4, Some("error-tripod-revolute-angle")), (10.5, None)] {
        cfg.set_text(&format!("workspace.max_revolute_angle_deg = {deg}")).unwrap();
        assert_eq!((ENTRY.build)(&cfg).unwrap().unreachable(&LEVEL, 0.0), refused);
    }
    cfg.set_text("workspace.max_cone_angle_deg = 10.4").unwrap();
    assert_eq!((ENTRY.build)(&cfg).unwrap().unreachable(&LEVEL, 0.0), Some("error-tripod-cone-angle"));
    // Without a tolerance the parasitic motion is not judged.
    let free = Config::parse(&ENTRY.default_config.replace("parasitic_tolerance_m = 0.005", "")).unwrap();
    assert!((ENTRY.build)(&free).unwrap().rating_checks().iter().all(|c| c.check != "tripod-parasitic"));
}

/// The default cycle passes every verdict: the default is a sized example.
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped);
    for v in sim_components::analysis::verdicts(&cfg, &load).unwrap() {
        println!("{} {:?} {} / {}", v.check, v.motor, v.value, v.limit);
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
}

/// The planner's trains (`Machine::trains`) are each leg's own mode with the other legs held: its coupling (6000 N m/rad
/// over the screw's lever 0.01 / 2 pi m/rad squared, 2.368705e9 N/m) in series with the screw at the nut's place (the
/// cylinder's extension q plus the 0.075 m from the fixed bearing to its start, `BallScrew::stiffness`) against the
/// leg's own entry M_kk of the mechanism's mass matrix with the others standing: sqrt(K / M_kk) / 2 pi. Level and
/// tilted, where the legs stand out unequally. Each lies between the lowest and the highest of the machine's modes with
/// all legs on their springs (Rayleigh's quotient of the leg's unit shape).
#[test]
fn the_trains_are_each_legs_mode_with_the_others_held() {
    let cfg = ENTRY.config();
    let (machine, mech) = Tripod::from_config(&cfg).unwrap();
    let screw = BallScrew::from_config(&cfg).unwrap();
    let coupling = 6000.0 / (0.01 / TAU).powi(2);
    for pose in [LEVEL, [0.52, 0.15, -0.1]] {
        let trains = machine.trains(&pose);
        let modes: Vec<f64> = machine.modes(&pose).iter().map(|m| m.hz).collect();
        let mut q = [0.0; 3];
        assert!(mech.inverse(&pose, &mut q));
        let (mut mass, mut bias) = (vec![0.0; 9], vec![0.0; 3]);
        mech.dynamics(&q, &[0.0; 3], &mut mass, &mut bias);
        for k in 0..3 {
            let k_leg = 1.0 / (1.0 / coupling + 1.0 / screw.stiffness(q[k] + 0.075));
            let hz = (k_leg / mass[k * 3 + k]).sqrt() / TAU;
            let train = trains[k].unwrap().hz;
            assert!((train / hz - 1.0).abs() < 1e-9, "{pose:?} leg {k}: {train} against {hz} Hz by hand");
            assert!(train >= modes[0] && train <= modes[2], "{pose:?} leg {k}: {train} outside {modes:?}");
        }
    }
}

/// The legs' two parts as bodies of their own, lifting the platform level (all legs alike, qd_i = u_z v_z, the twist
/// (0, 0, v_z, 0, 0, 0): the constraints e_i . v = 0 hold): with s = 1 - u_z^2 = 0.0331565, L = 0.5491812 m,
/// u' = (v - (u . v) u) / L so |u'|^2 = v_z^2 s / L^2. The base part (2.1 kg, centre 0.1 m, inertia 0.044) turns about
/// the base joint: T = 1/2 (m c^2 + I) |u'|^2. The rod part (0.9 kg, 0.175 m, 0.0092) moves at v - c u':
/// |.|^2 = v_z^2 (1 - 2 c s / L + c^2 s / L^2). The sum of all entries of M_q is 2 T / (u_z v_z)^2, so against the point
/// legs of the old model it is larger by 3 (m_b c_b^2 + I_b) s / (L^2 u_z^2) = 0.0221726 and by
/// 3 [m_p (c_p^2 s / L^2 - 2 c_p s / L) + I_p s / L^2] / u_z^2 = -0.0464701 kg: -0.0242976 kg together.
#[test]
fn the_legs_parts_add_the_hand_calculated_kinetic_energy() {
    // `centred` has the parts as points; the shipped parts are the default config's.
    let points = centred();
    let mut shipped = ENTRY.config();
    shipped.set_text("mass.payload_offset_m = [0.0, 0.0, 0.0]").unwrap();
    let sum = |cfg: &Config| {
        let mech = mechanism(cfg);
        let mut q = [0.0; 3];
        assert!(mech.geo.inverse(&LEVEL, &mut q));
        let (mut m, mut b) = ([0.0; 9], [0.0; 3]);
        Mechanism::dynamics(&mech, &q, &[0.0; 3], &mut m, &mut b);
        m.iter().sum::<f64>()
    };
    let delta = sum(&shipped) - sum(&points);
    assert!((delta + 0.0242976).abs() < 1e-6, "{delta}");
}

/// The dynamics with the legs' parts against Lagrange's equations by differences: at a pose off level and a speed,
/// h_i = sum_jk dM_ij/dq_k qd_k qd_j - 1/2 sum_jk dM_jk/dq_i qd_j qd_k + dV/dq_i, and at rest h is the legs' holding
/// force (`static_leg_forces`, with the parts' weights).
#[test]
fn the_dynamics_with_the_legs_parts_agree_with_the_lagrange_equations() {
    let mech = mechanism(&ENTRY.config());
    let tool = [0.56, 0.05, -0.04];
    let mut q = [0.0; 3];
    assert!(mech.geo.inverse(&tool, &mut q));
    let qd = [0.08, -0.05, 0.06];
    let eval = |q: &[f64; 3], qd: &[f64; 3]| {
        let (mut m, mut b) = ([0.0; 9], [0.0; 3]);
        Mechanism::dynamics(&mech, q, qd, &mut m, &mut b);
        (m, b)
    };
    let (_, bias) = eval(&q, &qd);
    let eps = 1e-5;
    let shifted = |k: usize, sign: f64| {
        let mut p = q;
        p[k] += sign * eps;
        p
    };
    let dm: Vec<[f64; 9]> = (0..3)
        .map(|k| {
            std::array::from_fn(|e| {
                (eval(&shifted(k, 1.0), &qd).0[e] - eval(&shifted(k, -1.0), &qd).0[e]) / (2.0 * eps)
            })
        })
        .collect();
    for i in 0..3 {
        let dv = (mech.potential(&shifted(i, 1.0)) - mech.potential(&shifted(i, -1.0))) / (2.0 * eps);
        let rate: f64 = (0..3).map(|j| (0..3).map(|k| dm[k][i * 3 + j] * qd[k]).sum::<f64>() * qd[j]).sum();
        let kinetic: f64 = (0..3).map(|j| (0..3).map(|k| dm[i][j * 3 + k] * qd[j] * qd[k]).sum::<f64>()).sum();
        let h = rate - 0.5 * kinetic + dv;
        assert!((bias[i] - h).abs() < 1e-4 * (1.0 + h.abs()), "leg {i}: {} against {h}", bias[i]);
    }
    let (_, rest) = eval(&q, &[0.0; 3]);
    let hold = mech.static_leg_forces(&tool).unwrap();
    for i in 0..3 {
        assert!((rest[i] - hold[i]).abs() < 1e-9 * (1.0 + hold[i].abs()), "{rest:?} against {hold:?}");
    }
}

/// The rotors' gyroscopic moment has nothing to do on the platform: a leg swings in its plane about its trunnion's axis
/// e_i, so u and u' both lie square to e_i and the moment the rotor needs, h u', lies square to e_i too, which the
/// trunnion's bearing carries; were it passed to the platform as a six-legged platform's couple, its force there, h (u x u') / L,
/// would lie along e_i, the constraint's direction, and A^T of its wrench (the generalised force on q) is 0. So no
/// term in the equations; the rotors' spin stays in the frame's momentum, whose rate the base feels.
#[test]
fn the_rotors_turning_with_their_legs_do_not_push_the_platform() {
    let mech = mechanism(&ENTRY.config());
    let tool = [0.52, 0.12, -0.08];
    let mut q = [0.0; 3];
    assert!(mech.geo.inverse(&tool, &mut q));
    let legs = mech.geo.legs(&tool);
    let a = Lu::new(&mech.geo.constraint_matrix(&legs)).unwrap().columns();
    let qd = [0.07, -0.05, 0.04];
    let xi: [f64; 6] = std::array::from_fn(|r| (0..3).map(|c| a[c][r] * qd[c]).sum());
    let (v, w) = ([xi[0], xi[1], xi[2]], [xi[3], xi[4], xi[5]]);
    let mut wrench = [0.0; 6];
    let mut size = 0.0f64;
    for i in 0..3 {
        let d = add(v, cross(w, legs.b[i]));
        let along = dot(legs.u[i], d);
        let turn: V3 = std::array::from_fn(|k| (d[k] - legs.u[i][k] * along) / legs.length[i]);
        let force = cross(legs.u[i], turn).map(|f| f / legs.length[i]);
        let e = mech.geo.tangents[i];
        assert!(dot(turn, e).abs() < 1e-15 && dot(legs.u[i], e).abs() < 1e-15, "leg {i} leaves its plane");
        let off = cross(force, e);
        assert!(dot(off, off).sqrt() < 1e-12 * dot(force, force).sqrt(), "leg {i}: {force:?} not along {e:?}");
        size = size.max(dot(force, force).sqrt());
        let moment = cross(legs.b[i], force);
        (0..3).for_each(|k| (wrench[k], wrench[3 + k]) = (wrench[k] + force[k], wrench[3 + k] + moment[k]));
    }
    assert!(size > 1e-3, "{size}");
    for c in 0..3 {
        let generalised: f64 = (0..6).map(|r| a[c][r] * wrench[r]).sum();
        assert!(generalised.abs() < 1e-12 * size, "{c}: {generalised}");
    }
}
