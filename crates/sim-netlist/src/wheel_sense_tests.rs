//! The wheels' spin in the frame momentum against the belt's own kinematics, for every shipped machine file.
//!
//! The belt passes each wheel at its rim speed u (relative to the wheel's centre, along the route). On a straight run
//! from wheel A to wheel B (direction t, from A's contact to B's) the belt does not stretch, so its speed along t is
//! the same at both ends: u_A + V_A . t = u_B + V_B . t, V the centres' velocities (exact also where the run turns: the
//! rim's velocity at a tangent point lies along t). At a clamp the belt rides with its body, u = 0. Propagated from a
//! clamp (or else from the section's first pulley, whose rim speed is r K v), this gives u at every idler and pulley
//! independently of the file's `speed` coefficients; `frame_momentum` takes u = route_sense * speed . v for an idler
//! and route_sense * r K v for a pulley, so the two must agree.

use super::*;
use crate::geometry::contacts;

fn shipped(name: &str) -> Config {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name);
    Config::parse(&std::fs::read_to_string(path).unwrap()).unwrap()
}

/// Every mismatch of a machine file's idlers and pulleys with the belt's kinematics, at a pose and per unit speed of
/// each axis in turn, and the sections checked (`section id/axis id`).
fn mismatches(name: &str, changes: &[&str]) -> (Vec<String>, Vec<String>) {
    let mut cfg = shipped(name);
    for change in changes {
        let _ = cfg.set_text(change);
    }
    let net = Netlist::parse(&cfg).unwrap();
    if net.scissor.is_some() || net.linkage.is_some() {
        return (Vec::new(), Vec::new());
    }
    let machine = ConfigMachine::new(&cfg, &net).unwrap();
    let q: Vec<f64> = (0..machine.n()).map(|i| if i < machine.nt() { 0.37 * machine.stroke[i] } else { 0.0 }).collect();
    let (mut wrong, mut checked) = (Vec::new(), Vec::new());
    for axis in 0..machine.nt() {
        let mut v = vec![0.0; machine.n()];
        v[axis] = 1.0;
        let h = 1e-6;
        let moved = |sign: f64| q.iter().zip(&v).map(|(q, v)| q + sign * h * v).collect::<Vec<_>>();
        let pulley_rim = |sec: usize, kk: usize| machine.route_sense(sec, kk) * machine.r[kk] * dot(&machine.k[kk], &v);
        for sec in 0..machine.sections.len() {
            let route = machine.route_waypoints(sec);
            // In the section's own plane.
            let plane = machine.plane(sec);
            let wheel = |w: Waypoint, at: &[f64]| machine.wheel(w, at, plane);
            let velocity = |w: Waypoint| {
                let (ahead, behind) = (wheel(w, &moved(1.0)).0.centre, wheel(w, &moved(-1.0)).0.centre);
                [0, 1].map(|k| (ahead[k] - behind[k]) / (2.0 * h))
            };
            // Only where the drawn route is the belt: its straight runs' length changes as the section's does (a route
            // drawn without its far wheels, or with a pulley that rides on an axis drawn standing, says nothing).
            let length = |at: &[f64]| {
                route
                    .windows(2)
                    .map(|w| {
                        let [p, r] = contacts(wheel(w[0], at).0, wheel(w[1], at).0);
                        (r[0] - p[0]).hypot(r[1] - p[1])
                    })
                    .sum::<f64>()
            };
            let rate = (length(&moved(1.0)) - length(&moved(-1.0))) / (2.0 * h);
            let wanted = dot(&machine.sections[sec].c, &v);
            // A route whose drawn wheels meet (a NaN length) says nothing either.
            if !((rate - wanted).abs() <= 0.05 * (1.0 + wanted.abs())) {
                continue;
            }
            checked.push(format!("{}/{}", machine.sections[sec].id, machine.axes[axis].id));
            let Some(start) = route
                .iter()
                .position(|w| matches!(w, Waypoint::Body(_)))
                .or_else(|| route.iter().position(|w| matches!(w, Waypoint::Pulley(_))))
            else {
                continue;
            };
            let mut u = vec![f64::NAN; route.len()];
            u[start] = match route[start] {
                Waypoint::Pulley(kk) => pulley_rim(sec, kk),
                _ => 0.0,
            };
            let step = |from: usize, to: usize, u: &mut Vec<f64>| {
                let (a, b) = (wheel(route[from], &q).0, wheel(route[to], &q).0);
                // Along the route from `from` to `to`, or back.
                let (first, second) = if from < to { (a, b) } else { (b, a) };
                let [p, r] = contacts(first, second);
                let length = (r[0] - p[0]).hypot(r[1] - p[1]);
                let t = [(r[0] - p[0]) / length, (r[1] - p[1]) / length];
                let (va, vb) = (velocity(route[from]), velocity(route[to]));
                u[to] = u[from] + (va[0] - vb[0]) * t[0] + (va[1] - vb[1]) * t[1];
            };
            for j in start + 1..route.len() {
                step(j - 1, j, &mut u);
            }
            for j in (0..start).rev() {
                step(j + 1, j, &mut u);
            }
            for (j, w) in route.iter().enumerate() {
                let claimed = match *w {
                    Waypoint::Idler(d) => {
                        let Some(first) = (0..machine.m()).find(|&kk| machine.sections[sec].f[kk] != 0.0) else {
                            continue;
                        };
                        machine.route_sense(sec, first) * dot(&machine.idlers[d].speed, &v)
                    }
                    Waypoint::Pulley(kk) => pulley_rim(sec, kk),
                    Waypoint::Body(_) => continue,
                };
                // The drawing's runs lie a few degrees off the axes: their speeds within 5 %, the sense exact.
                if (claimed - u[j]).abs() > 0.05 * (1.0 + u[j].abs()) {
                    wrong.push(format!(
                        "{name} {}: {w:?} per {}: file {claimed}, belt {}",
                        machine.sections[sec].id, machine.axes[axis].id, u[j]
                    ));
                }
            }
        }
    }
    eprintln!("{name}: {} sections checked", checked.len());
    (wrong, checked)
}

#[test]
fn the_shipped_files_wheels_turn_as_their_belts_run() {
    let mut wrong = Vec::new();
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines");
    for entry in std::fs::read_dir(dir).unwrap() {
        let name = entry.unwrap().file_name().into_string().unwrap();
        if name.ends_with(".toml") {
            wrong.extend(mismatches(&name, &["effects.idlers = true"]).0);
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}

/// The screw portal's family (screw portal, C portal, both five-axis files): with their far idlers in the routes each
/// belt's second section is drawn as it runs, x2 along x (standing) and y2 along y (flat on the bridge), so both are
/// checked against the belt's kinematics, and the far idlers' `speed` (+1, the sign the dual gantry's far idler has by
/// hand) matches. Drawn without the far idlers (before 22 November) neither section was checked.
#[test]
fn the_screw_portals_far_idlers_turn_as_their_belts_run() {
    for name in ["screw-portal.toml", "portal-c.toml", "five-axis.toml", "five-axis-table.toml"] {
        let (wrong, checked) = mismatches(name, &["effects.idlers = true"]);
        assert!(wrong.is_empty(), "{wrong:?}");
        for c in ["x2/x", "y2/y"] {
            assert!(checked.iter().any(|k| k.ends_with(c)), "{name}: {c} not in {checked:?}");
        }
    }
}

/// The XYZ portal draws its three loops as the screw portal's family: far_x on the frame, far_y on the bridge, the Z
/// motor on the Y carriage with far_z 0.5 m above its pulley. Every second section is checked against the belt's
/// kinematics along its own axis (it was none of them: drawn straight back to the clamp, and the Z pulley standing
/// still at its `at_m` while the carriage it rides on moved).
#[test]
fn the_xyz_portals_far_idlers_turn_as_their_belts_run() {
    let (wrong, checked) = mismatches("xyz-portal.toml", &["effects.idlers = true"]);
    assert!(wrong.is_empty(), "{wrong:?}");
    for c in ["x2/x", "y2/y", "z1/z", "z2/z"] {
        assert!(checked.iter().any(|k| k.ends_with(c)), "{c} not in {checked:?}");
    }
}

/// The XYZ portal's frame angular momentum by hand (machines/xyz-portal.toml as shipped), at x = 0.5, y = 0.25,
/// z = 0.1 m, moving along z alone at vz = 0.2 m/s, about the origin of the bodies' places. What moves:
/// - the Z slide and its payload, 2.5 + 1 kg at (x, y, z): (y m vz, -x m vz, 0);
/// - the Z pulley (1e-4 kg m^2 without its belt lump) and its motor's rotor (0.15e-4 kg m^2 through i = 10): the Z belt
///   stands in the x-z plane, its wheels about y; rising, the slide draws z1 out of the pulley, so the belt runs round
///   the loop clockwise seen from the front (against the routes, which run anticlockwise), +y at vz / r:
///   (1e-4 + 10 x 0.15e-4) vz / r with r = 0.02 m;
/// - the far idler without inertia (the file has no effects.idlers);
/// - the Z belt's runs, mu = 0.1 kg/m over 0.1 + z (clamp to pulley, +vz), 0.5 (pulley to far idler, -vz) and 0.4 - z
///   (far idler to clamp, +vz), each at the middle of its straight piece. Along the routes (anticlockwise) the clamp's
///   strand runs down left of the wheels' line, the other up right of it: the middles sideways by e1 = r sqrt(D1^2 -
///   r^2) / (2 D1) left (D1 = 0.1 + z), e2 = (r + r_f) / 2 sqrt(1 - ((r - r_f) / 0.5)^2) right, e3 = r_f sqrt(D3^2 -
///   r_f^2) / (2 D3) left (D3 = 0.4 - z, r_f = 0.012 m); about y, -x_mid p_z, they give mu vz (L1 e1 + L2 e2 + L3 e3),
///   the carriage's x cancelling (0.1 + z - 0.5 + 0.4 - z = 0); about x, y p_z, nothing (all at the carriage's y).
#[test]
fn the_xyz_portals_frame_momentum_with_its_z_belt_by_hand() {
    let cfg = shipped("xyz-portal.toml");
    let net = Netlist::parse(&cfg).unwrap();
    let machine = ConfigMachine::new(&cfg, &net).unwrap();
    let (x, y, z, vz) = (0.5, 0.25, 0.1, 0.2);
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(&[x, y, z], &mut state, &mut torque);
    machine.set_tool_velocity(&mut state, &[0.0, 0.0, vz]);
    let f = machine.frame_momentum(&state).unwrap();
    let (r, rf, mu, m) = (0.02, 0.012, 0.1, 3.5);
    let tangent = |d: f64, r: f64| r * (d * d - r * r).sqrt() / (2.0 * d);
    let e2 = 0.5 * (r + rf) * (1.0 - ((r - rf) / 0.5f64).powi(2)).sqrt();
    let runs = mu * vz * ((0.1 + z) * tangent(0.1 + z, r) + 0.5 * e2 + (0.4 - z) * tangent(0.4 - z, rf));
    let hand = [y * m * vz, -x * m * vz + (1e-4 + 10.0 * 0.15e-4) * vz / r + runs, 0.0];
    eprintln!("angular {:?}, by hand {hand:?} (runs {runs})", f.angular);
    for k in 0..3 {
        assert!((f.angular[k] - hand[k]).abs() < 1e-12, "{:?} against {hand:?}", f.angular);
    }
}

/// The dual and rack gantries' Y belts by hand (machines/dual-gantry.toml, rack-gantry.toml): a loop from the clamp on
/// the Y carriage to the motor's pulley (section y1, 0.1 + y), round it and back along the other strand to the far idler
/// on the bridge, 0.1 + stroke + 0.1 from the pulley, and round that to the clamp (y2). A positive turn of the pulley
/// feeds belt into y1 (its `pulleys` +1) and so takes it out of y2 (-1): the carriage moving at +vy draws y1 out and
/// the pulley turns at vy / r. Relative to the bridge, which carries the far idler and moves along x only, the belt runs
/// on y2's first span (pulley to far idler, along +y) at -vy and on its second (far idler to clamp, along -y) at +vy:
/// both against the route, at vy, so it passes the idler at vy the way the pulley's positive turn moves it (the route's
/// sense of y2's first pulley is -1): `speed = { y = 1 }`. Checked from the spans' own speeds over ground, not from the
/// drawing: these files draw their belts in the x-z plane, where a belt along y has no length (its route's length is NaN
/// there, which the wheels' check above skips). Every sign is as derived; the wheels' spin about y in the frame momentum,
/// which a belt along y cannot turn about, is open.
#[test]
fn the_gantries_y_belts_turn_their_far_idlers_as_they_run() {
    for name in ["dual-gantry.toml", "rack-gantry.toml"] {
        let cfg = shipped(name);
        let net = Netlist::parse(&cfg).unwrap();
        let machine = ConfigMachine::new(&cfg, &net).unwrap();
        let far = net.idlers.iter().position(|d| d.id == "far_y").unwrap();
        let section = |id: &str| machine.sections.iter().position(|s| s.id.ends_with(id)).unwrap();
        let (y1, y2) = (section("y1"), section("y2"));
        let kk = (0..machine.m()).find(|&kk| machine.sections[y1].f[kk] != 0.0).unwrap();
        assert_eq!((machine.sections[y1].f[kk], machine.sections[y2].f[kk]), (1.0, -1.0), "{name}");
        assert!((machine.k[kk][1] * machine.r[kk] - 1.0).abs() < 1e-12, "{name}");
        assert_eq!(
            machine.route_waypoints(y2),
            [Waypoint::Pulley(kk), Waypoint::Idler(far), Waypoint::Body(1)],
            "{name}"
        );
        // Per unit of vy: the spans' speed along y over ground less the bridge's (none along y), along each run's
        // direction on the route (+y from the pulley to the far idler, -y back to the clamp).
        let spans: Vec<_> = machine.spans.iter().filter(|s| s.section == y2).collect();
        assert_eq!(spans.len(), 2, "{name}");
        let along: Vec<f64> =
            spans.iter().zip([1.0, -1.0]).map(|(s, dir)| dir * s.velocity.as_ref().unwrap()[1][1]).collect();
        let claimed = machine.route_sense(y2, kk) * machine.idlers[far].speed[1];
        assert_eq!(along, [claimed, claimed], "{name}");
        assert_eq!((claimed, machine.idlers[far].speed[0]), (-1.0, 0.0), "{name}");
        // The belt's speed past the pulley on y2's first run is its rim speed, r K v along the route's sense.
        assert!((machine.route_sense(y2, kk) * machine.r[kk] * machine.k[kk][1] - along[0]).abs() < 1e-12, "{name}");
        // Lying flat (`plane = "x-y"`, the pulley on the bridge), the belt is drawn as it runs along y: both sections
        // are checked against the belt's kinematics along y, and their wheels match.
        let (wrong, checked) = mismatches(name, &["effects.idlers = true"]);
        assert!(wrong.is_empty(), "{wrong:?}");
        for c in ["y1/y", "y2/y"] {
            assert!(checked.iter().any(|k| k.ends_with(c)), "{name}: {c} not in {checked:?}");
        }
    }
}

/// The dual gantry's frame angular momentum by hand (machines/dual-gantry.toml as shipped), at x = 1.0 m,
/// y = 0.3 m, moving along y alone at vy = 0.2 m/s, about the origin of the bodies' places. Its Y belt lies flat on the
/// bridge (`plane = "x-y"`): the pulley 0.07 m before the bridge's y = 0, the far idler 0.93 m after it, both at the
/// bridge's x, the clamp on the Y carriage at y. What moves:
/// - the Y carriage and its payload, 4 + 1 kg at (x, y, 0): (0, 0, 5 x vy);
/// - the Y pulley (1e-4 kg m^2, its belt lumps left to the runs) and the motor's rotor (0.3e-4 kg m^2 through i = 5),
///   turning anticlockwise seen from above as the belt runs along y2's route at -vy (the pulley feeds y2 with -1): about
///   +z at -vy / r, r = 0.02 m: -(1e-4 + 5 x 0.3e-4) vy / r;
/// - the far idler (r_f = 0.012 m) without inertia (the file has no effects.idlers);
/// - the belt's runs, mu = 0.1 kg/m over the spans' lengths 0.1 + y (clamp to pulley, +vy), 1.0 (pulley to far idler,
///   -vy) and 0.9 - y (far idler to clamp, +vy), each at the middle of its straight piece between the tangents. The belt
///   runs anticlockwise round the loop seen from above: down the clamp's strand (left of the wheels' line, -x), round the
///   pulley and up the other strand (right, +x), round the far idler and down to the clamp (left). The middles' sideways
///   offsets: from a point at D from a wheel's centre the tangent touches r sqrt(D^2 - r^2) / D off the line, so the
///   run's middle half that, e1 = r sqrt(D1^2 - r^2) / (2 D1), D1 = y + 0.07, and e3 likewise with r_f and D3 = 0.93 - y;
///   between the pulley and the far idler (d = 1.0 m apart) e2 = (r + r_f) / 2 sqrt(1 - ((r - r_f) / d)^2). Their
///   momenta along y give about z mu vy (-(0.1 + y) e1 - 1.0 e2 - (0.9 - y) e3) (the bridge's x cancels: 0.1 + y - 1.0
///   + 0.9 - y = 0).
/// Nothing about x or y: every wheel turns about z and every run lies in the plane z = 0 moving along y. Drawn standing in
/// x-z as before, the wheels spun about y (2.5e-3 N m s of the 2.5e-3) and the runs lay at the pulley's drawn x = 1.5 m.
#[test]
fn the_dual_gantrys_frame_momentum_with_its_y_belt_flat_by_hand() {
    let cfg = shipped("dual-gantry.toml");
    let net = Netlist::parse(&cfg).unwrap();
    let machine = ConfigMachine::new(&cfg, &net).unwrap();
    let (x, y, vy) = (1.0, 0.3, 0.2);
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(&[x, y], &mut state, &mut torque);
    machine.set_tool_velocity(&mut state, &[0.0, vy]);
    let f = machine.frame_momentum(&state).unwrap();
    let (r, rf, mu) = (0.02, 0.012, 0.1);
    let tangent = |d: f64, r: f64| r * (d * d - r * r).sqrt() / (2.0 * d);
    let (e1, e2, e3) =
        (tangent(y + 0.07, r), 0.5 * (r + rf) * (1.0 - ((r - rf) / 1.0f64).powi(2)).sqrt(), tangent(0.93 - y, rf));
    let runs = mu * vy * (-(0.1 + y) * e1 - 1.0 * e2 - (0.9 - y) * e3);
    let lz = 5.0 * x * vy - (1e-4 + 5.0 * 0.3e-4) * vy / r + runs;
    eprintln!("angular {:?}, by hand [0, 0, {lz}] (runs {runs})", f.angular);
    assert!(f.angular[0].abs() < 1e-12 && f.angular[1].abs() < 1e-12, "{:?}", f.angular);
    assert!((f.angular[2] - lz).abs() < 1e-12, "{} against {lz}", f.angular[2]);
}
