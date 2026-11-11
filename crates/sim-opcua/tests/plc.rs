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
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("opc.tcp://{}/", listener.local_addr().unwrap());
    let labels = translations("en", entry);
    let link = serve(entry, &entry.config(), &labels, listener, &scratch(&format!("{name}-server"))).await.unwrap();

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
        for i in 0..axes.len() {
            assert!((reached[i] - target[i]).abs() < 2e-4, "{}: axis {i} at {} m, target {} m", entry.id, reached[i], target[i]);
            assert!(peak_error[i] > 1e-7 && peak_error[i] < 1e-3, "{}: axis {i} reported a following error of {} m", entry.id, peak_error[i]);
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
