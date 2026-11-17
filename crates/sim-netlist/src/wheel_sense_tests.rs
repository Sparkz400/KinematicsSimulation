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
/// each axis in turn.
fn mismatches(name: &str, changes: &[&str]) -> Vec<String> {
    let mut cfg = shipped(name);
    for change in changes {
        let _ = cfg.set_text(change);
    }
    let net = Netlist::parse(&cfg).unwrap();
    if net.scissor.is_some() || net.linkage.is_some() {
        return Vec::new();
    }
    let machine = ConfigMachine::new(&cfg, &net).unwrap();
    let q: Vec<f64> = (0..machine.n()).map(|i| if i < machine.nt() { 0.37 * machine.stroke[i] } else { 0.0 }).collect();
    let (mut wrong, mut checked) = (Vec::new(), 0);
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
            if (rate - wanted).abs() > 0.05 * (1.0 + wanted.abs()) {
                continue;
            }
            checked += 1;
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
    eprintln!("{name}: {checked} sections checked");
    wrong
}

#[test]
fn the_shipped_files_wheels_turn_as_their_belts_run() {
    let mut wrong = Vec::new();
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines");
    for entry in std::fs::read_dir(dir).unwrap() {
        let name = entry.unwrap().file_name().into_string().unwrap();
        if name.ends_with(".toml") {
            wrong.extend(mismatches(&name, &["effects.idlers = true"]));
        }
    }
    assert!(wrong.is_empty(), "{}", wrong.join("\n"));
}
