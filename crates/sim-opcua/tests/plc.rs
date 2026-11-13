//! A PLC-commanded move runs in the simulator and reports back position and following error.
//! The PLC here is an OPC UA client in the test; a real controller speaks the same protocol.

use opcua::client::{ClientBuilder, IdentityToken, Session};
use opcua::crypto::SecurityPolicy;
use opcua::types::{AttributeId, DataValue, MessageSecurityMode, NodeId, NumericRange, StatusCode, TimestampsToReturn, Variant, WriteValue};
use sim_components::Sim;
use sim_core::MachineEntry;
use sim_machines::{MACHINES, translations};
use sim_opcua::{Link, NAMESPACE, serve};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

struct Plc {
    session: Arc<Session>,
    namespace: u16,
    _link: Link,
}

fn scratch(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name)
}

/// Starts the simulator's server on a free local port and connects a client to it.
async fn connect(entry: &'static MachineEntry, name: &str) -> Plc {
    connect_with(entry, &entry.config(), name).await
}

/// `connect` with the config `cfg`.
async fn connect_with(entry: &'static MachineEntry, cfg: &sim_core::Config, name: &str) -> Plc {
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("opc.tcp://{}/", listener.local_addr().unwrap());
    let labels = translations("en", entry);
    let link = serve(entry, cfg, &labels, listener, &scratch(&format!("{name}-server")), Duration::from_millis(10)).await.unwrap();

    let mut client = ClientBuilder::new()
        .application_name("test-plc")
        .application_uri("urn:test-plc")
        .pki_dir(scratch(&format!("{name}-client")))
        .create_sample_keypair(true)
        .trust_server_certs(true)
        .session_retry_limit(1)
        .client()
        .unwrap();
    let endpoint = (url.as_str(), SecurityPolicy::None.to_str(), MessageSecurityMode::None);
    let (session, event_loop) = client.connect_to_matching_endpoint(endpoint, IdentityToken::Anonymous).await.unwrap();
    event_loop.spawn();
    assert!(tokio::time::timeout(Duration::from_secs(5), session.wait_for_connection()).await.unwrap());
    // A client finds the simulator's variables through the namespace URI, like any PLC would.
    let namespace = session.get_namespace_index(NAMESPACE).await.unwrap();
    assert_eq!(namespace, link.namespace);
    Plc { session, namespace, _link: link }
}

impl Plc {
    fn node(&self, name: &str) -> NodeId {
        NodeId::new(self.namespace, name.to_string())
    }

    async fn read(&self, names: &[String]) -> Vec<Variant> {
        let nodes: Vec<_> = names.iter().map(|n| self.node(n).into()).collect();
        let values = self.session.read(&nodes, TimestampsToReturn::Neither, 0.0).await.unwrap();
        values
            .into_iter()
            .zip(names)
            .map(|(v, name)| {
                assert!(v.status.is_none_or(|s| s.is_good()), "{name}: {:?}", v.status);
                v.value.unwrap_or_else(|| panic!("{name} has no value"))
            })
            .collect()
    }

    async fn numbers(&self, names: &[String]) -> Vec<f64> {
        self.read(names).await.into_iter().map(|v| if let Variant::Double(x) = v { x } else { panic!("not a number: {v:?}") }).collect()
    }

    async fn write(&self, values: Vec<(String, Variant)>) -> Vec<StatusCode> {
        let writes: Vec<WriteValue> = values
            .into_iter()
            .map(|(name, value)| WriteValue {
                node_id: self.node(&name),
                attribute_id: AttributeId::Value as u32,
                index_range: NumericRange::None,
                value: DataValue { value: Some(value), ..Default::default() },
            })
            .collect();
        self.session.write(&writes).await.unwrap()
    }

    async fn busy(&self) -> bool {
        matches!(self.read(&["status.busy".to_string()]).await[0], Variant::Boolean(true))
    }
}

/// The variables are generated from the signal list: every signal of every machine can be read.
#[tokio::test(flavor = "multi_thread")]
async fn every_signal_of_every_machine_is_a_variable() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("signals-{}", entry.id)).await;
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let names: Vec<String> = sim.signal_infos().iter().map(|s| format!("signal.{}", s.id)).collect();
        assert!(names.len() > 15);
        tokio::time::sleep(Duration::from_millis(100)).await;
        let values = plc.numbers(&names).await;
        assert!(values.iter().all(|v| v.is_finite()));
        // The simulation runs in real time behind the server.
        let time = names.iter().position(|n| n == "signal.time").unwrap();
        tokio::time::sleep(Duration::from_millis(200)).await;
        let later = plc.numbers(&names).await;
        assert!(later[time] > values[time] + 0.05, "{}: time went from {} to {}", entry.id, values[time], later[time]);
        // Signals are read-only.
        assert!(!plc.write(vec![(names[2].clone(), Variant::Double(1.0))]).await[0].is_good());
    }
}

/// Phase 6: a PLC-commanded move runs in the simulator and reports back position and following error.
#[tokio::test(flavor = "multi_thread")]
async fn a_plc_commanded_move_runs_and_reports_position_and_following_error() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("move-{}", entry.id)).await;
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let axes = sim.machine.axes();
        let start = sim.control.set_pos.clone();
        // Every axis moves 0.2 of its range towards the middle.
        let target: Vec<f64> = axes.iter().zip(&start).map(|(a, s)| s + 0.2 * (a.max - a.min) * (0.5 * (a.min + a.max) - s).signum()).collect();
        let position: Vec<String> = axes.iter().map(|a| format!("signal.axis.{}.position", a.id)).collect();
        let error: Vec<String> = axes.iter().map(|a| format!("signal.axis.{}.following_error", a.id)).collect();

        let mut writes: Vec<(String, Variant)> = axes.iter().zip(&target).map(|(a, t)| (format!("command.target.{}", a.id), Variant::Double(*t))).collect();
        writes.push(("command.move".into(), Variant::Boolean(true)));
        assert!(plc.write(writes).await.iter().all(|s| s.is_good()));

        // The PLC polls while the move runs.
        let (mut was_busy, mut peak_error, mut polls) = (false, vec![0f64; axes.len()], 0);
        loop {
            tokio::time::sleep(Duration::from_millis(20)).await;
            let busy = plc.busy().await;
            for (peak, e) in peak_error.iter_mut().zip(plc.numbers(&error).await) {
                *peak = peak.max(e.abs());
            }
            was_busy |= busy;
            polls += 1;
            assert!(polls < 500, "{}: the move did not finish within ten seconds", entry.id);
            if was_busy && !busy {
                break;
            }
        }
        tokio::time::sleep(Duration::from_millis(300)).await;
        let reached = plc.numbers(&position).await;
        // The setpoint is the target; the tool is off it by what the gearboxes and belts give way under the weight
        // they hold (`at_rest_within`).
        let setpoint = plc.numbers(&axes.iter().map(|a| format!("signal.axis.{}.position_setpoint", a.id)).collect::<Vec<_>>()).await;
        let rest = at_rest_within(entry, &target);
        for i in 0..axes.len() {
            assert!((setpoint[i] - target[i]).abs() < 1e-6, "{}: axis {i} set to {} m, target {} m", entry.id, setpoint[i], target[i]);
            assert!((reached[i] - target[i]).abs() < rest, "{}: axis {i} at {} m, target {} m, within {rest} m", entry.id, reached[i], target[i]);
            // Below 1 mm, and on a fast machine 0.04 mm per m/s2 of its fastest axis (the delta's diagonal move at 50 m/s2
            // per axis: up to 1.4 mm, 2 mm allowed).
            let fastest = axes.iter().map(|a| a.max_acc).fold(0.0, f64::max);
            assert!(peak_error[i] > 1e-7 && peak_error[i] < f64::max(1e-3, 4e-5 * fastest), "{}: axis {i} reported a following error of {} m", entry.id, peak_error[i]);
        }
        let status = plc.read(&["status.refused".to_string()]).await;
        assert_eq!(status[0], Variant::from(""), "{}", entry.id);

        // A target outside the travel range is refused, and the PLC can see why.
        plc.write(vec![(format!("command.target.{}", axes[0].id), Variant::Double(axes[0].max + 1.0)), ("command.move".into(), Variant::Boolean(true))]).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(plc.read(&["status.refused".to_string()]).await[0], Variant::from("error-target-out-of-range"));
        assert!(!plc.write(vec![("command.move".into(), Variant::Double(1.0))]).await[0].is_good(), "a number is not a start signal");
    }
}

/// The PLC sends setpoints: a stream of positions every 20 ms is followed smoothly, because the
/// simulator interpolates between them and keeps the servo loops to itself.
#[tokio::test(flavor = "multi_thread")]
async fn streamed_setpoints_from_the_plc_are_followed() {
    let entry = MACHINES[0];
    let plc = connect(entry, "stream").await;
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let axis = &sim.machine.axes()[0];
    let start = sim.control.set_pos[0];
    let position = vec![format!("signal.axis.{}.position", axis.id)];
    let setpoint = format!("command.setpoint.{}", axis.id);
    // 20 mm in 25 steps of 0.8 mm: a slow ramp of 40 mm/s.
    for step in 1..=25 {
        assert!(plc.write(vec![(setpoint.clone(), Variant::Double(start + 0.0008 * step as f64))]).await[0].is_good());
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    tokio::time::sleep(Duration::from_millis(300)).await;
    let reached = plc.numbers(&position).await[0];
    assert!((reached - (start + 0.02)).abs() < 2e-4, "at {reached} m after the ramp, wanted {} m", start + 0.02);
    assert_eq!(plc.numbers(&[setpoint]).await[0], start + 0.02, "the setpoint reads back what was written");
}

/// Disable and Enable from the PLC: disabled, no motor gives torque and a move is refused with the reason;
/// enabled again, the move runs.
#[tokio::test(flavor = "multi_thread")]
async fn the_plc_disables_and_enables_the_drives() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("enable-{}", entry.id)).await;
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let torque: Vec<String> = sim.machine.motors().iter().map(|m| format!("signal.motor.{}.torque", m.id)).collect();
        let set = |name: &str| vec![(name.to_string(), Variant::Boolean(true))];
        let refused = || async { plc.read(&["status.refused".to_string()]).await.remove(0) };

        // Half a second into the cycle the machine is away from its start pose.
        assert!(plc.write(set("command.cycle")).await[0].is_good());
        tokio::time::sleep(Duration::from_millis(500)).await;
        assert!(plc.busy().await);

        assert!(plc.write(set("command.disable")).await[0].is_good());
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert!(plc.numbers(&torque).await.iter().all(|t| *t == 0.0), "{}: torque while disabled", entry.id);
        assert!(!plc.busy().await, "{}: the cycle runs on", entry.id);
        plc.write(set("command.move")).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(refused().await, Variant::from("error-disabled"), "{}", entry.id);

        // Enabled, the move back to the start pose is accepted and runs. The targets are written with it:
        // unwritten, they are where the axes stand.
        assert!(plc.write(set("command.enable")).await[0].is_good());
        tokio::time::sleep(Duration::from_millis(50)).await;
        let mut back: Vec<(String, Variant)> = sim.machine.axes().iter().zip(&sim.control.set_pos).map(|(a, p)| (format!("command.target.{}", a.id), Variant::Double(*p))).collect();
        back.extend(set("command.move"));
        plc.write(back).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(refused().await, Variant::from(""), "{}", entry.id);
        assert!(plc.busy().await, "{}: no move after enabling", entry.id);
        assert!(!plc.write(vec![("command.disable".into(), Variant::Double(1.0))]).await[0].is_good(), "a number is not a switch");
    }
}

/// Polls until a move that was started is over.
async fn move_done(plc: &Plc) {
    let mut was_busy = false;
    for _ in 0..500 {
        tokio::time::sleep(Duration::from_millis(20)).await;
        let busy = plc.busy().await;
        if was_busy && !busy {
            return;
        }
        was_busy |= busy;
    }
    panic!("the move did not start and finish within ten seconds");
}

/// The setpoint and target variables follow the controller where the PLC does not write them. They used to
/// keep the start pose: after a move, a setpoint written for one axis sent the other back to where it
/// started (0.25 m in 10 ms), and a target written for one axis moved the other back as well.
#[tokio::test(flavor = "multi_thread")]
async fn a_write_to_one_axis_leaves_the_others_where_they_are() {
    let entry = MACHINES.iter().copied().find(|m| Sim::new(m, &m.config()).unwrap().machine.axes().len() > 1).expect("no machine with two axes");
    let plc = connect(entry, "one-axis").await;
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let axes = sim.machine.axes();
    let names = |kind: &str| -> Vec<String> { axes.iter().map(|a| format!("{kind}.{}", a.id)).collect() };
    let refused = || async { plc.read(&["status.refused".to_string()]).await.remove(0) };
    // Every axis moves 0.2 of its range towards the middle.
    let target: Vec<f64> = axes.iter().zip(&sim.control.set_pos).map(|(a, s)| s + 0.2 * (a.max - a.min) * (0.5 * (a.min + a.max) - s).signum()).collect();
    let mut writes: Vec<(String, Variant)> = names("command.target").into_iter().zip(&target).map(|(n, t)| (n, Variant::Double(*t))).collect();
    writes.push(("command.move".into(), Variant::Boolean(true)));
    assert!(plc.write(writes).await.iter().all(|s| s.is_good()));
    move_done(&plc).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    // Both variables read where the axes now stand.
    for kind in ["command.setpoint", "command.target"] {
        let shown = plc.numbers(&names(kind)).await;
        assert!(shown.iter().zip(&target).all(|(s, t)| (s - t).abs() < 1e-9), "{kind} reads {shown:?} after a move to {target:?}");
    }

    // A setpoint for the second axis alone: it is followed, and the first axis stays.
    assert!(plc.write(vec![(names("command.setpoint").remove(1), Variant::Double(target[1] + 0.001))]).await[0].is_good());
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert_eq!(refused().await, Variant::from(""));
    let positions: Vec<String> = names("signal.axis").iter().map(|n| format!("{n}.position")).collect();
    let reached = plc.numbers(&positions).await;
    assert!((reached[0] - target[0]).abs() < 2e-4 && (reached[1] - target[1] - 0.001).abs() < 2e-4, "at {reached:?} after a setpoint for one axis from {target:?}");
    assert!((plc.numbers(&names("command.target")).await[1] - target[1] - 0.001).abs() < 1e-9, "the target does not read where the setpoint took the axis");

    // A target for the first axis alone, and a move: the second axis stays where the setpoint took it.
    plc.write(vec![(names("command.target").remove(0), Variant::Double(target[0] - 0.05)), ("command.move".into(), Variant::Boolean(true))]).await;
    move_done(&plc).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let reached = plc.numbers(&positions).await;
    assert!((reached[0] - target[0] + 0.05).abs() < 2e-4 && (reached[1] - target[1] - 0.001).abs() < 2e-4, "at {reached:?} after a move of one axis from {target:?}");
    assert_eq!(refused().await, Variant::from(""));
}

/// A refusal stays readable until a command of the same kind is accepted. It used to be wiped by the next
/// accepted command of any kind, so a PLC streaming setpoints never saw why its move was refused.
#[tokio::test(flavor = "multi_thread")]
async fn a_refusal_stays_until_a_command_of_its_kind_is_accepted() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("refused-{}", entry.id)).await;
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let axis = &sim.machine.axes()[0];
        let (target, setpoint) = (format!("command.target.{}", axis.id), format!("command.setpoint.{}", axis.id));
        let refused = || async { plc.read(&["status.refused".to_string()]).await.remove(0) };
        let go = || ("command.move".to_string(), Variant::Boolean(true));

        plc.write(vec![(target.clone(), Variant::Double(axis.max + 1.0)), go()]).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(refused().await, Variant::from("error-target-out-of-range"), "{}", entry.id);
        // Where the tool stands before the setpoints: under load a little off the setpoint (the gears' give).
        let before = plc.numbers(&[format!("signal.axis.{}.position", axis.id)]).await[0];
        // Accepted commands of other kinds: enabling, and setpoints that are followed.
        assert!(plc.write(vec![("command.enable".to_string(), Variant::Boolean(true))]).await[0].is_good());
        for step in 1..=5 {
            assert!(plc.write(vec![(setpoint.clone(), Variant::Double(sim.control.set_pos[0] + 0.0005 * step as f64))]).await[0].is_good());
            tokio::time::sleep(Duration::from_millis(20)).await;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
        let position = plc.numbers(&[format!("signal.axis.{}.position", axis.id)]).await[0];
        assert!((position - before - 0.0025).abs() < 2e-4, "{}: the setpoints were not followed", entry.id);
        assert_eq!(refused().await, Variant::from("error-target-out-of-range"), "{}: an accepted setpoint wiped the refused move", entry.id);
        // An accepted move settles it.
        plc.write(vec![(target.clone(), Variant::Double(sim.control.set_pos[0])), go()]).await;
        tokio::time::sleep(Duration::from_millis(100)).await;
        assert_eq!(refused().await, Variant::from(""), "{}", entry.id);
    }
}

/// Commands written in one write are carried out in the order written: enabling a machine that coasted
/// while disabled, then a path, then running it, plans the path from where the machine stands once enabled.
/// The path used to be planned before the enable was carried out, from where the machine stood when it was
/// disabled, and was refused with `error-trajectory-start`.
#[tokio::test(flavor = "multi_thread")]
async fn enable_path_and_run_in_one_write_run_the_path() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("enable-path-{}", entry.id)).await;
        let axes = Sim::new(entry, &entry.config()).unwrap().machine.axes().to_vec();
        let refused = || async { plc.read(&["status.refused".to_string()]).await.remove(0) };
        let set = |name: &str| (name.to_string(), Variant::Boolean(true));
        // Disabled in the middle of the cycle, the machine coasts away from its last setpoint.
        plc.write(vec![set("command.cycle")]).await;
        tokio::time::sleep(Duration::from_millis(500)).await;
        plc.write(vec![set("command.disable")]).await;
        tokio::time::sleep(Duration::from_millis(300)).await;
        let middle = axes.iter().map(|a| format!("{}", 0.5 * (a.min + a.max))).collect::<Vec<_>>().join(",");
        let written = plc.write(vec![set("command.enable"), ("command.path".into(), Variant::from(format!("line,{middle}").as_str())), set("command.run_path")]).await;
        assert!(written.iter().all(|s| s.is_good()));
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(refused().await, Variant::from(""), "{}", entry.id);
        assert!(plc.busy().await, "{}: the path does not run", entry.id);
    }
}

/// `command.run_path` written together with `command.path` runs that path, whichever comes first in the
/// write. With the path after it, the path written before was run while `command.path` read the new one.
#[tokio::test(flavor = "multi_thread")]
async fn run_path_written_before_the_path_runs_the_new_path() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("run-then-path-{}", entry.id)).await;
        let axes = Sim::new(entry, &entry.config()).unwrap().machine.axes().to_vec();
        let refused = || async { plc.read(&["status.refused".to_string()]).await.remove(0) };
        tokio::time::sleep(Duration::from_millis(200)).await;
        // The old path cannot be read: run, it is refused.
        plc.write(vec![("command.path".into(), Variant::from("line,0.5\ncircle,1,2"))]).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        let middle = axes.iter().map(|a| format!("{}", 0.5 * (a.min + a.max))).collect::<Vec<_>>().join(",");
        let new = format!("line,{middle}");
        plc.write(vec![("command.run_path".into(), Variant::Boolean(true)), ("command.path".into(), Variant::from(new.as_str()))]).await;
        tokio::time::sleep(Duration::from_millis(50)).await;
        assert_eq!(refused().await, Variant::from(""), "{}: the old path was run", entry.id);
        assert!(plc.busy().await, "{}: the new path does not run", entry.id);
        assert_eq!(plc.read(&["command.path".to_string()]).await[0], Variant::from(new.as_str()));
    }
}

/// How far `pose` is from the polyline through `points`.
fn off_path(points: &[(f64, Vec<f64>)], pose: &[f64]) -> f64 {
    let segment = |a: &[f64], b: &[f64]| {
        let (ab, ap): (Vec<f64>, Vec<f64>) = (a.iter().zip(b).map(|(a, b)| b - a).collect(), a.iter().zip(pose).map(|(a, p)| p - a).collect());
        let length = ab.iter().map(|d| d * d).sum::<f64>();
        let along = if length > 0.0 { (ab.iter().zip(&ap).map(|(d, p)| d * p).sum::<f64>() / length).clamp(0.0, 1.0) } else { 0.0 };
        ab.iter().zip(&ap).map(|(d, p)| (p - along * d).powi(2)).sum::<f64>().sqrt()
    };
    points.windows(2).map(|w| segment(&w[0].1, &w[1].1)).fold(f64::INFINITY, f64::min)
}

/// What `run_path` saw: the poses read on the way, the simulator time it was busy with the path (from the first poll
/// that read busy to the first that read idle), the largest step of `signal.time` between two polls, counted from a
/// read before the path was written (the busy time is that coarse), and the mean step.
struct Polled {
    poses: Vec<Vec<f64>>,
    busy: f64,
    largest_step: f64,
    mean_step: f64,
}

/// Runs `text` as a path from the PLC and polls until it is over.
async fn run_path(plc: &Plc, text: &str, time_optimal: bool, position: &[String]) -> Polled {
    let writes = vec![
        ("command.path".to_string(), Variant::from(text)),
        ("command.path_time_optimal".to_string(), Variant::Boolean(time_optimal)),
        ("command.run_path".to_string(), Variant::Boolean(true)),
    ];
    // The time before the write: the path starts after it. Without it the first poll, late behind a slow write, was
    // taken for the start (the delta busy for 0.27 s of a 0.31 s plan).
    let before = plc.numbers(&["signal.time".to_string()]).await[0];
    assert!(plc.write(writes).await.iter().all(|s| s.is_good()));
    // Busy and the time in one read, so that a slow answer does not put time between them.
    let mut names = position.to_vec();
    names.extend(["signal.time".into(), "status.busy".into()]);
    let (mut poses, mut started, mut polls, mut times) = (Vec::new(), None, 0, vec![before]);
    loop {
        tokio::time::sleep(Duration::from_millis(10)).await;
        let mut values = plc.read(&names).await;
        let busy = matches!(values.pop(), Some(Variant::Boolean(true)));
        let mut read: Vec<f64> = values.into_iter().map(|v| if let Variant::Double(x) = v { x } else { panic!("not a number: {v:?}") }).collect();
        let time = read.pop().unwrap();
        times.push(time);
        polls += 1;
        assert!(polls < 1000, "the path did not start and finish within ten seconds");
        match (busy, started) {
            (true, None) => started = Some(time),
            (false, Some(start)) => {
                let steps: Vec<f64> = times.windows(2).map(|w| w[1] - w[0]).collect();
                let (largest_step, mean_step) = (steps.iter().fold(0.0, |a: f64, b| a.max(*b)), (times[times.len() - 1] - times[0]) / steps.len() as f64);
                return Polled { poses, busy: time - start, largest_step, mean_step };
            }
            _ => {}
        }
        if started.is_some() {
            poses.push(read);
        }
    }
}

/// Polls `status.refused` until it reads `key`, for up to 5 s (a loaded machine answers a write late); what it read last.
async fn refusal(plc: &Plc, key: &str) -> Variant {
    let start = std::time::Instant::now();
    loop {
        let now = plc.read(&["status.refused".to_string()]).await.remove(0);
        if now == Variant::from(key) || start.elapsed() > Duration::from_secs(5) {
            return now;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
}

/// Paths over OPC UA: the PLC writes the text of a path file and starts it; the simulator plans it from where
/// the machine stands, as `Sim::plan_path` does, and runs it in real time. The poses the PLC reads lie on the
/// planned path within the path error the analysis gives for it, and the end is reached within the settling
/// tolerance. Paths that cannot be read, planned or run are refused with the reason, and nothing moves.
#[tokio::test(flavor = "multi_thread")]
async fn a_path_from_the_plc_is_planned_and_run() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("path-{}", entry.id)).await;
        let cfg = entry.config();
        let sim = Sim::new(entry, &cfg).unwrap();
        let axes = sim.machine.axes();
        let position: Vec<String> = axes.iter().map(|a| format!("signal.axis.{}.position", a.id)).collect();
        let refused = || async { plc.read(&["status.refused".to_string()]).await.remove(0) };
        let point = |share: f64| axes.iter().map(|a| format!("{}", a.min + share * (a.max - a.min))).collect::<Vec<_>>().join(",");
        let text = format!("# two lines\nline,{}\nline,{}\n", point(0.5), point(0.75));
        tokio::time::sleep(Duration::from_millis(200)).await;
        let at_rest = plc.numbers(&position).await;

        // Refused, with the reason, and the machine stays: a line that cannot be read, a path out of the
        // stroke, an arc on a machine with one axis.
        let beyond = axes.iter().map(|a| format!("{}", a.max + 0.1)).collect::<Vec<_>>().join(",");
        let mut bad = vec![("line,0.5\ncircle,1,2".to_string(), "opcua-error-path-line"), (format!("line,{beyond}"), "error-path-range")];
        if axes.len() == 1 {
            bad.push(("arc,0.6,0.25,0.5,0.25,cw".into(), "error-path"));
        }
        for (path, key) in bad {
            let written = plc.write(vec![("command.path".into(), Variant::from(path.as_str())), ("command.run_path".into(), Variant::Boolean(true))]).await;
            assert!(written.iter().all(|s| s.is_good()));
            assert_eq!(refusal(&plc, key).await, Variant::from(key), "{}: {path}", entry.id);
            assert!(!plc.busy().await, "{}: {path}", entry.id);
            let now = plc.numbers(&position).await;
            assert!(now.iter().zip(&at_rest).all(|(n, r)| (n - r).abs() < 1e-6), "{}: moved to {now:?} from {at_rest:?} on {path}", entry.id);
        }
        assert!(!plc.write(vec![("command.path".into(), Variant::Double(1.0))]).await[0].is_good(), "a number is not a path");

        // The same path the simulator would plan, and the path error the analysis gives for it.
        let pieces = sim_components::path::parse(&text).unwrap();
        let planned = sim.plan_path(&pieces, false).unwrap();
        let path_error = sim_components::analysis::analyse_trajectory(&mut Sim::new(entry, &cfg).unwrap(), planned.clone()).unwrap().path_error_peak;
        let slow = run_path(&plc, &text, false, &position).await;
        assert_eq!(refused().await, Variant::from(""), "{}: an accepted path leaves a refusal", entry.id);
        assert_eq!(plc.read(&["command.path".to_string()]).await[0], Variant::from(text.as_str()), "the path reads back what was written");
        // Polled while it ran: at least half the polls its planned time takes at the mean step seen (a loaded machine
        // polls less often, and the simulator still keeps to real time).
        let slow_plan = planned.last().unwrap().0;
        let polls = slow.poses.len() as f64;
        assert!(polls >= 0.5 * slow_plan / slow.mean_step, "{}: {polls} poses read in {slow_plan} s, a poll every {} s", entry.id, slow.mean_step);
        let worst = slow.poses.iter().map(|p| off_path(&planned, p)).fold(0.0, f64::max);
        assert!(worst <= path_error, "{}: {worst} m off the planned path, the analysis gives {path_error} m", entry.id);
        tokio::time::sleep(Duration::from_millis(300)).await;
        let end = &planned.last().unwrap().1;
        let reached = plc.numbers(&position).await;
        // The setpoint stands on the path's end; the tool is off it by what gears and belts give way under the load
        // there (`at_rest_within`; on the delta with its catalogue gearboxes more than the settling tolerance: the
        // cycle fails `settling` there, a sizing finding).
        let setpoint = plc.numbers(&axes.iter().map(|a| format!("signal.axis.{}.position_setpoint", a.id)).collect::<Vec<_>>()).await;
        assert!(setpoint.iter().zip(end).all(|(s, e)| (s - e).abs() < 1e-6), "{}: set to {setpoint:?}, the path ends at {end:?}", entry.id);
        let tolerance = at_rest_within(entry, end);
        assert!(reached.iter().zip(end).all(|(r, e)| (r - e).abs() < tolerance), "{}: at {reached:?}, the path ends at {end:?}, within {tolerance} m", entry.id);
        // The busy time is known to one poll: within the largest step of `signal.time` seen, and 1 ms.
        assert!((slow.busy - slow_plan).abs() <= slow.largest_step + 1e-3, "{}: busy for {} s, planned {slow_plan} s, polled up to {} s apart", entry.id, slow.busy, slow.largest_step);

        // Busy: a path started while one runs is refused.
        // Back by the middle, a stop on the way, so that it still runs when the second start arrives on a loaded machine.
        let back = format!("line,{}\nline,{}", point(0.5), at_rest.iter().map(|p| format!("{p}")).collect::<Vec<_>>().join(","));
        plc.write(vec![("command.path".into(), Variant::from(back.as_str())), ("command.run_path".into(), Variant::Boolean(true))]).await;
        let start = std::time::Instant::now();
        while !plc.busy().await {
            assert!(start.elapsed() < Duration::from_secs(5), "{}: not on the way back", entry.id);
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        plc.write(vec![("command.run_path".into(), Variant::Boolean(true))]).await;
        assert_eq!(refusal(&plc, "error-busy").await, Variant::from("error-busy"), "{}", entry.id);
        move_done(&plc).await;
        tokio::time::sleep(Duration::from_millis(300)).await;

        // Time-optimal: the same path from the same pose finishes sooner, as in the command-line tool.
        let fast_plan = sim.plan_path(&pieces, true).unwrap().last().unwrap().0;
        let fast = run_path(&plc, &text, true, &position).await;
        assert_eq!(refused().await, Variant::from(""), "{}", entry.id);
        // The plans: sooner by 5 % at least (the delta's 0.32 s, whose axis limits are near what its motors can do, is
        // the closest). The run keeps to its plan as the slow one does.
        assert!(fast_plan < 0.95 * slow_plan, "{}: planned {fast_plan} s time-optimal, {slow_plan} s within the axis limits", entry.id);
        assert!((fast.busy - fast_plan).abs() <= fast.largest_step + 1e-3, "{}: busy for {} s time-optimal, planned {fast_plan} s, polled up to {} s apart", entry.id, fast.busy, fast.largest_step);
    }
}

/// PLCopen axis variables: `axis.<id>.<name>`.
impl Plc {
    async fn axis_read(&self, axis: &str, name: &str) -> Variant {
        self.read(&[format!("axis.{axis}.{name}")]).await.remove(0)
    }

    async fn axis_flag(&self, axis: &str, name: &str) -> bool {
        matches!(self.axis_read(axis, name).await, Variant::Boolean(true))
    }

    async fn axis_number(&self, axis: &str, name: &str) -> f64 {
        self.numbers(&[format!("axis.{axis}.{name}")]).await[0]
    }

    async fn axis_write(&self, axis: &str, values: &[(&str, Variant)]) {
        let written = self.write(values.iter().map(|(name, value)| (format!("axis.{axis}.{name}"), value.clone())).collect()).await;
        assert!(written.iter().all(|s| s.is_good()), "{axis}: {values:?} -> {written:?}");
    }

    /// Polls `name` every 10 ms until it is true; panics after `seconds`. Returns the time it took.
    async fn until(&self, axis: &str, name: &str, seconds: f64) -> f64 {
        let start = std::time::Instant::now();
        while !self.axis_flag(axis, name).await {
            assert!(start.elapsed().as_secs_f64() < seconds, "{axis}.{name} not true within {seconds} s");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        start.elapsed().as_secs_f64()
    }

    /// Waits until the simulation has run `seconds` on (`signal.time`): a loaded machine runs it slower than the wall
    /// clock, so that a wait in wall time can end before the axis has settled.
    async fn simulated(&self, seconds: f64) {
        let time = || async { self.numbers(&["signal.time".to_string()]).await[0] };
        let (until, start) = (time().await + seconds, std::time::Instant::now());
        while time().await < until {
            assert!(start.elapsed() < Duration::from_secs(30), "the simulation did not run {seconds} s within 30 s");
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    /// A rising edge on `<fb>.Execute`: false, then true, in one write, with the inputs before them.
    async fn execute(&self, axis: &str, fb: &str, inputs: &[(&str, f64)]) {
        let mut values: Vec<(String, Variant)> = inputs.iter().map(|(name, v)| (format!("{fb}.{name}"), Variant::Double(*v))).collect();
        values.push((format!("{fb}.Execute"), Variant::Boolean(false)));
        values.push((format!("{fb}.Execute"), Variant::Boolean(true)));
        let values: Vec<(&str, Variant)> = values.iter().map(|(n, v)| (n.as_str(), v.clone())).collect();
        self.axis_write(axis, &values).await;
    }
}

/// How far the tool may be from its setpoint at rest at `pose`: what gearboxes and belts give way under the load there,
/// which the encoders do not see. The offset after 0.5 s at rest with `pose` as the start pose, times 1.2, and at
/// least 0.2 mm (some hundredths of a millimetre on the gantries; on the delta with its catalogue gearboxes up to
/// 0.69 mm at the poses of these tests, so the machine's whole 2.3 mm setpoint slack is not needed). A target off by
/// 1 mm fails the setpoint's own check to 1e-6 m.
fn at_rest_within(entry: &'static MachineEntry, pose: &[f64]) -> f64 {
    let mut cfg = entry.config();
    cfg.set_text(&format!("start.pose = {pose:?}")).unwrap();
    let mut sim = Sim::new(entry, &cfg).unwrap();
    sim.run(0.5, |_| {});
    let mut out = Vec::new();
    sim.sample(&mut out);
    let at = |id: String| out[sim.signal_index(&id).unwrap()];
    let sag = sim.machine.axes().iter().map(|a| (at(format!("axis.{}.position_setpoint", a.id)) - at(format!("axis.{}.position", a.id))).powi(2)).sum::<f64>().sqrt();
    (1.2 * sag).max(2e-4)
}

/// 0.2 of the axis range from `from` towards the middle.
fn towards_middle(axis: &sim_core::AxisInfo, from: f64) -> f64 {
    from + 0.2 * (axis.max - axis.min) * (0.5 * (axis.min + axis.max) - from).signum()
}

/// MC_Power, MC_Home, MC_MoveAbsolute and MC_MoveRelative on every axis of every machine: the state goes Disabled,
/// Standstill, Homing, DiscreteMotion and back, Done comes when the axis is there, and the axis is there.
#[tokio::test(flavor = "multi_thread")]
async fn plcopen_power_home_and_moves_reach_their_targets() {
    for entry in MACHINES.iter().copied() {
        let plc = connect(entry, &format!("plcopen-{}", entry.id)).await;
        let sim = Sim::new(entry, &entry.config()).unwrap();
        // The setpoint is the target to 1e-6 m; the tool is off it by what gearboxes and belts give way at the pose the
        // setpoints make (`at_rest_within`).
        let setpoints: Vec<String> = sim.machine.axes().iter().map(|a| format!("signal.axis.{}.position_setpoint", a.id)).collect();
        let index = |id: &str| sim.machine.axes().iter().position(|a| a.id == id).unwrap();
        for (axis, home) in sim.machine.axes().iter().zip(&sim.control.set_pos) {
            let id = axis.id;
            tokio::time::sleep(Duration::from_millis(50)).await;
            assert_eq!(plc.axis_read(id, "State").await, Variant::from("Disabled"), "{}", entry.id);
            assert_eq!(plc.axis_read(id, "StateID").await, Variant::Int32(1));
            plc.axis_write(id, &[("Power.Enable", Variant::Boolean(true))]).await;
            plc.until(id, "Power.Status", 1.0).await;
            assert!(plc.axis_flag(id, "Power.Valid").await && !plc.axis_flag(id, "IsHomed").await);
            assert_eq!(plc.axis_read(id, "State").await, Variant::from("Standstill"));

            plc.execute(id, "Home", &[]).await;
            plc.until(id, "Home.Done", 5.0).await;
            assert!(plc.axis_flag(id, "IsHomed").await && !plc.axis_flag(id, "Home.Busy").await);

            let target = towards_middle(axis, *home);
            // Busy and Active, and the state, from the edge on until the move is done: polled, as a loaded machine answers
            // late. At half the top speed, or slower so that the move takes 0.5 s of simulated time at least (some
            // axes were there in 0.1 s), a poll comes while it runs; one that ended unseen fails.
            let speed = f64::min(0.5 * axis.max_vel, (target - home).abs() / 0.5);
            plc.execute(id, "MoveAbsolute", &[("Position", target), ("Velocity", speed)]).await;
            let outputs: Vec<String> = ["MoveAbsolute.Busy", "MoveAbsolute.Active", "MoveAbsolute.Done", "State"].iter().map(|n| format!("axis.{id}.{n}")).collect();
            let start = std::time::Instant::now();
            loop {
                let read = plc.read(&outputs).await;
                if read[0] == Variant::Boolean(true) {
                    assert_eq!((&read[1], &read[3]), (&Variant::Boolean(true), &Variant::from("DiscreteMotion")), "{}.{id}", entry.id);
                    break;
                }
                assert!(read[2] != Variant::Boolean(true) && start.elapsed() < Duration::from_secs(5), "{}.{id}: MoveAbsolute not seen busy before {read:?}", entry.id);
                tokio::time::sleep(Duration::from_millis(5)).await;
            }
            plc.until(id, "MoveAbsolute.Done", 5.0).await;
            plc.simulated(0.3).await;
            let at = plc.axis_number(id, "ActualPosition").await;
            let set = plc.numbers(&setpoints).await;
            assert!((set[index(id)] - target).abs() < 1e-6, "{}.{id}: set to {} m, target {target} m", entry.id, set[index(id)]);
            let rest = at_rest_within(entry, &set);
            assert!((at - target).abs() < rest, "{}.{id}: at {at} m, target {target} m, within {rest} m", entry.id);
            assert!(plc.axis_number(id, "ActualVelocity").await.abs() < 1e-2);

            plc.execute(id, "MoveRelative", &[("Distance", -0.05 * (target - home).signum())]).await;
            plc.until(id, "MoveRelative.Done", 5.0).await;
            plc.simulated(0.3).await;
            let moved = plc.axis_number(id, "ActualPosition").await;
            let (set, wanted) = (plc.numbers(&setpoints).await, target - 0.05 * (target - home).signum());
            assert!((set[index(id)] - wanted).abs() < 1e-6, "{}.{id}: set to {} m, wanted {wanted} m", entry.id, set[index(id)]);
            let rest = at_rest_within(entry, &set);
            assert!((moved - wanted).abs() < rest, "{}.{id}: at {moved} m, wanted {wanted} m, within {rest} m", entry.id);
            assert_eq!(plc.axis_read(id, "State").await, Variant::from("Standstill"));
            assert_eq!(plc.read(&["status.refused".to_string()]).await[0], Variant::from(""));
        }
    }
}

/// MC_Stop in the middle of a move brakes with its deceleration: the move reports CommandAborted, Stop reports Done
/// once the axis stands, after about v / d, short of the target. The axis stays in Stopping while Execute is held,
/// refuses a move there, and is in Standstill once Execute falls.
#[tokio::test(flavor = "multi_thread")]
async fn plcopen_stop_brakes_a_move_in_the_middle() {
    let entry = MACHINES[0];
    let plc = connect(entry, "plcopen-stop").await;
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let axis = &sim.machine.axes()[0];
    let (id, home) = (axis.id, sim.control.set_pos[0]);
    plc.axis_write(id, &[("Power.Enable", Variant::Boolean(true))]).await;
    plc.until(id, "Power.Status", 1.0).await;
    let far = if home - axis.min > axis.max - home { axis.min + 0.05 * (axis.max - axis.min) } else { axis.max - 0.05 * (axis.max - axis.min) };
    let speed = 0.25 * axis.max_vel;
    plc.execute(id, "MoveAbsolute", &[("Position", far), ("Velocity", speed), ("Acceleration", axis.max_acc)]).await;
    tokio::time::sleep(Duration::from_millis(400)).await;
    let v = plc.axis_number(id, "ActualVelocity").await.abs();
    assert!((v - speed).abs() < 0.1 * speed, "at {v} m/s, wanted {speed} m/s");
    let deceleration = 1.0;
    plc.execute(id, "Stop", &[("Deceleration", deceleration)]).await;
    let took = plc.until(id, "Stop.Done", 2.0).await;
    assert!(plc.axis_flag(id, "MoveAbsolute.CommandAborted").await && !plc.axis_flag(id, "MoveAbsolute.Done").await);
    assert!((took - speed / deceleration).abs() < 0.08, "stopped in {took} s, v / d = {} s", speed / deceleration);
    tokio::time::sleep(Duration::from_millis(300)).await;
    let at = plc.axis_number(id, "ActualPosition").await;
    assert!((at - far).abs() > 0.05, "went on to the target: {at} m");
    assert_eq!(plc.axis_read(id, "State").await, Variant::from("Stopping"));
    plc.execute(id, "MoveRelative", &[("Distance", 0.01)]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(plc.axis_flag(id, "MoveRelative.Error").await);
    assert_eq!(plc.axis_read(id, "MoveRelative.ErrorID").await, Variant::UInt16(sim_opcua::plcopen::ERR_STATE));
    plc.axis_write(id, &[("Stop.Execute", Variant::Boolean(false))]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(plc.axis_read(id, "State").await, Variant::from("Standstill"));
    assert!(!plc.axis_flag(id, "Stop.Done").await, "Done stays after Execute fell");
}

/// Two axes move at the same time, each with its own profile, started 0.1 s apart; both arrive.
#[tokio::test(flavor = "multi_thread")]
async fn plcopen_axes_move_at_the_same_time() {
    let entry = MACHINES.iter().copied().find(|m| Sim::new(m, &m.config()).unwrap().machine.axes().len() > 1).expect("no machine with two axes");
    let plc = connect(entry, "plcopen-concurrent").await;
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let axes = sim.machine.axes();
    let targets: Vec<f64> = axes.iter().zip(&sim.control.set_pos).map(|(a, h)| towards_middle(a, *h)).collect();
    for axis in &axes[..2] {
        plc.axis_write(axis.id, &[("Power.Enable", Variant::Boolean(true))]).await;
    }
    plc.execute(axes[0].id, "MoveAbsolute", &[("Position", targets[0]), ("Velocity", 0.3 * axes[0].max_vel)]).await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    plc.execute(axes[1].id, "MoveAbsolute", &[("Position", targets[1]), ("Velocity", 0.3 * axes[1].max_vel)]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(plc.axis_flag(axes[0].id, "MoveAbsolute.Busy").await && plc.axis_flag(axes[1].id, "MoveAbsolute.Busy").await, "not both moving");
    for axis in &axes[..2] {
        plc.until(axis.id, "MoveAbsolute.Done", 5.0).await;
    }
    tokio::time::sleep(Duration::from_millis(300)).await;
    for (axis, target) in axes[..2].iter().zip(&targets) {
        let at = plc.axis_number(axis.id, "ActualPosition").await;
        assert!((at - target).abs() < 2e-4, "{}: at {at} m, target {target} m", axis.id);
        assert!(!plc.axis_flag(axis.id, "MoveAbsolute.CommandAborted").await);
    }
}

/// Errors: a move on an axis that is not powered fails with ErrorID 1 and the axis stays where it is. Drives switched
/// off from outside put a powered axis into ErrorStop (ErrorID 7); MC_Reset leaves it and switches the drives on again.
#[tokio::test(flavor = "multi_thread")]
async fn plcopen_errors_and_reset() {
    let entry = MACHINES[0];
    let plc = connect(entry, "plcopen-errors").await;
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let axis = &sim.machine.axes()[0];
    let id = axis.id;
    use sim_opcua::plcopen::{ERR_DRIVES_OFF, ERR_NOT_POWERED, ERR_RANGE};
    plc.execute(id, "MoveRelative", &[("Distance", 0.01)]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(plc.axis_flag(id, "MoveRelative.Error").await && !plc.axis_flag(id, "MoveRelative.Busy").await);
    assert_eq!(plc.axis_read(id, "MoveRelative.ErrorID").await, Variant::UInt16(ERR_NOT_POWERED));
    plc.axis_write(id, &[("MoveRelative.Execute", Variant::Boolean(false)), ("Power.Enable", Variant::Boolean(true))]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(!plc.axis_flag(id, "MoveRelative.Error").await, "the falling edge clears Error");
    plc.execute(id, "MoveAbsolute", &[("Position", axis.max + 0.1)]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(plc.axis_read(id, "MoveAbsolute.ErrorID").await, Variant::UInt16(ERR_RANGE));
    assert_eq!(plc.axis_read(id, "State").await, Variant::from("Standstill"), "a refused block is no axis error");

    plc.write(vec![("command.disable".into(), Variant::Boolean(true))]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert_eq!(plc.axis_read(id, "State").await, Variant::from("ErrorStop"));
    assert_eq!(plc.axis_read(id, "ErrorID").await, Variant::UInt16(ERR_DRIVES_OFF));
    assert!(plc.axis_flag(id, "Power.Error").await && !plc.axis_flag(id, "Power.Status").await);
    plc.execute(id, "MoveRelative", &[("Distance", 0.01)]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(plc.axis_flag(id, "MoveRelative.Error").await, "a move in ErrorStop");
    plc.execute(id, "Reset", &[]).await;
    plc.until(id, "Reset.Done", 1.0).await;
    plc.until(id, "Power.Status", 1.0).await;
    assert_eq!(plc.axis_read(id, "State").await, Variant::from("Standstill"));
    plc.execute(id, "MoveRelative", &[("Distance", 0.01)]).await;
    plc.until(id, "MoveRelative.Done", 2.0).await;
}

/// A crash (effects.crash: the machine with one axis into a wall it meets at x = 0.47) puts the powered axis into
/// ErrorStop with ErrorID 8, not 7. MC_Reset switches the drives on again, they stay on, and the axis moves out of
/// the wall. (They tripped again at once: the stop left the carriage inside the wall, and every step found it there.)
#[tokio::test(flavor = "multi_thread")]
async fn plcopen_a_crash_is_reported_reset_and_left() {
    use sim_opcua::plcopen::ERR_CRASH;
    // The wall lies across the stroke of the machine with one axis, where its scene draws the carriage.
    let entry = *MACHINES.iter().find(|e| e.config().f64s("start.pose").unwrap().len() == 1).unwrap();
    let mut cfg = entry.config();
    cfg.set_text(r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#).unwrap();
    cfg.set_text("effects.crash = true").unwrap();
    let plc = connect_with(entry, &cfg, "plcopen-crash").await;
    let id = Sim::new(entry, &cfg).unwrap().machine.axes()[0].id;
    plc.axis_write(id, &[("Power.Enable", Variant::Boolean(true))]).await;
    plc.until(id, "Power.Status", 1.0).await;
    plc.execute(id, "MoveAbsolute", &[("Position", 0.9)]).await;
    plc.until(id, "Error", 5.0).await;
    assert_eq!(plc.axis_read(id, "ErrorID").await, Variant::UInt16(ERR_CRASH));
    plc.execute(id, "Reset", &[]).await;
    plc.until(id, "Reset.Done", 1.0).await;
    plc.until(id, "Power.Status", 1.0).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert_eq!(plc.axis_read(id, "State").await, Variant::from("Standstill"), "ErrorID {:?}", plc.axis_read(id, "ErrorID").await);
    plc.execute(id, "MoveAbsolute", &[("Position", 0.1)]).await;
    plc.until(id, "MoveAbsolute.Done", 5.0).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    let at = plc.axis_number(id, "ActualPosition").await;
    assert!((at - 0.1).abs() < 2e-4 && plc.axis_flag(id, "Power.Status").await, "at {at} m");
}

/// Execute acts on its rising edge only: writing true again while it is true does not start the block again; Done
/// stays while Execute is held and clears when it falls.
#[tokio::test(flavor = "multi_thread")]
async fn plcopen_execute_is_edge_triggered() {
    let entry = MACHINES[0];
    let plc = connect(entry, "plcopen-edge").await;
    let id = Sim::new(entry, &entry.config()).unwrap().machine.axes()[0].id;
    plc.axis_write(id, &[("Power.Enable", Variant::Boolean(true))]).await;
    plc.execute(id, "MoveRelative", &[("Distance", 0.02)]).await;
    plc.until(id, "MoveRelative.Done", 2.0).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    let at = plc.axis_number(id, "ActualPosition").await;
    plc.axis_write(id, &[("MoveRelative.Execute", Variant::Boolean(true))]).await;
    tokio::time::sleep(Duration::from_millis(300)).await;
    assert!(plc.axis_flag(id, "MoveRelative.Done").await && !plc.axis_flag(id, "MoveRelative.Busy").await);
    assert!((plc.axis_number(id, "ActualPosition").await - at).abs() < 1e-4, "true written again moved the axis");
    assert_eq!(plc.axis_read(id, "MoveRelative.Execute").await, Variant::Boolean(true), "Execute reads back what was written");
    plc.axis_write(id, &[("MoveRelative.Execute", Variant::Boolean(false))]).await;
    tokio::time::sleep(Duration::from_millis(50)).await;
    assert!(!plc.axis_flag(id, "MoveRelative.Done").await);
    // Execute dropped while the move runs: the move goes on and Done is shown until the next rising edge.
    plc.axis_write(id, &[("MoveRelative.Execute", Variant::Boolean(true))]).await;
    plc.axis_write(id, &[("MoveRelative.Execute", Variant::Boolean(false))]).await;
    plc.until(id, "MoveRelative.Done", 2.0).await;
    tokio::time::sleep(Duration::from_millis(200)).await;
    assert!((plc.axis_number(id, "ActualPosition").await - at - 0.02).abs() < 2e-4);
    assert!(plc.axis_flag(id, "MoveRelative.Done").await);
}

/// The pacing variables: the nominal cycle, a measured period near it, a real-time factor near 1, and a reset that
/// starts the count anew.
#[tokio::test(flavor = "multi_thread")]
async fn pacing_is_measured_and_can_be_reset() {
    let plc = connect(MACHINES[0], "pacing").await;
    tokio::time::sleep(Duration::from_millis(1000)).await;
    let names: Vec<String> = ["cycle_s", "cycle_period_s", "jitter_max_s", "jitter_rms_s", "realtime_factor"].iter().map(|n| format!("status.{n}")).collect();
    let p = plc.numbers(&names).await;
    assert_eq!(p[0], 0.01);
    assert!((p[1] - 0.01).abs() < 0.005, "mean period {} s", p[1]);
    assert!(p[2] >= p[3] && p[3] >= 0.0 && p[2] < 0.2, "jitter max {} s, rms {} s", p[2], p[3]);
    assert!((p[4] - 1.0).abs() < 0.05, "real-time factor {}", p[4]);
    let count = |v: Variant| if let Variant::UInt64(n) = v { n } else { panic!("{v:?}") };
    let cycles = count(plc.read(&["status.cycles".into()]).await.remove(0));
    assert!(cycles > 50, "{cycles} cycles in a second");
    assert!(count(plc.read(&["status.steps".into()]).await.remove(0)) > 10_000);
    plc.write(vec![("command.reset_pacing".into(), Variant::Boolean(true))]).await;
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(count(plc.read(&["status.cycles".into()]).await.remove(0)) < cycles / 2);
}

/// A cycle longer than 0.1 s keeps real time: each tick runs the whole cycle's steps.
#[tokio::test(flavor = "multi_thread")]
async fn a_long_cycle_keeps_real_time() {
    let entry = MACHINES[0];
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let link = serve(entry, &entry.config(), &translations("en", entry), listener, &scratch("long-cycle-server"), Duration::from_millis(250)).await.unwrap();
    tokio::time::sleep(Duration::from_millis(1600)).await;
    let pacing = link.pacing.lock().unwrap().clone();
    assert!(pacing.cycles >= 4 && (pacing.realtime_factor() - 1.0).abs() < 0.05, "real-time factor {} over {} cycles", pacing.realtime_factor(), pacing.cycles);
}
