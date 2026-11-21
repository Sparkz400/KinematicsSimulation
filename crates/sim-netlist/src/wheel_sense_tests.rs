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
        let velocity = |w: Waypoint| {
            let (ahead, behind) = (machine.wheel(w, &moved(1.0)).0.centre, machine.wheel(w, &moved(-1.0)).0.centre);
            [0, 1].map(|k| (ahead[k] - behind[k]) / (2.0 * h))
        };
        let pulley_rim = |sec: usize, kk: usize| machine.route_sense(sec, kk) * machine.r[kk] * dot(&machine.k[kk], &v);
        for sec in 0..machine.sections.len() {
            let route = machine.route_waypoints(sec);
            // Only where the drawn route is the belt: its straight runs' length changes as the section's does (a route
            // drawn without its far wheels, or with a pulley that rides on an axis drawn standing, says nothing).
            let length = |at: &[f64]| {
                route
                    .windows(2)
                    .map(|w| {
                        let [p, r] = contacts(machine.wheel(w[0], at).0, machine.wheel(w[1], at).0);
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
                let (a, b) = (machine.wheel(route[from], &q).0, machine.wheel(route[to], &q).0);
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
    }
}
