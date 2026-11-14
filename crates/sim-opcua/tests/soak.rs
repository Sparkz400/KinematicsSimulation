//! Soak test: the link runs for `SOAK_SECONDS` (default 20) while a PLC client moves the axes back and forth with
//! PLCopen function blocks, as fast as the handshake allows. Every move must end with Done at its target, no block
//! or axis may report an error, and the pacing must stay real-time. `SOAK_SECONDS=3600 cargo test -p sim-opcua
//! --release --test soak -- --nocapture` runs it for an hour and prints the numbers.

use opcua::client::{ClientBuilder, IdentityToken};
use opcua::crypto::SecurityPolicy;
use opcua::types::{
    AttributeId, DataValue, MessageSecurityMode, NodeId, NumericRange, TimestampsToReturn, Variant, WriteValue,
};
use sim_components::Sim;
use sim_machines::{MACHINES, translations};
use sim_opcua::{NAMESPACE, serve};
use std::path::PathBuf;
use std::time::{Duration, Instant};

/// Resident memory in bytes where the system tells it (Linux), else `None`.
fn resident() -> Option<u64> {
    let statm = std::fs::read_to_string("/proc/self/statm").ok()?;
    Some(statm.split_whitespace().nth(1)?.parse::<u64>().ok()? * 4096)
}

#[tokio::test(flavor = "multi_thread")]
async fn the_link_runs_move_cycles_for_a_long_time_without_errors_or_drift() {
    let seconds: f64 = std::env::var("SOAK_SECONDS").ok().and_then(|s| s.parse().ok()).unwrap_or(20.0);
    let entry = MACHINES
        .iter()
        .copied()
        .find(|m| Sim::new(m, &m.config()).unwrap().machine.axes().len() > 1)
        .expect("no machine with two axes");
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let axes: Vec<_> = sim.machine.axes().to_vec();
    let scratch = |name: &str| PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(name);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("opc.tcp://{}/", listener.local_addr().unwrap());
    let link = serve(
        entry,
        &entry.config(),
        &translations("en", entry),
        listener,
        &scratch("soak-server"),
        Duration::from_millis(10),
    )
    .await
    .unwrap();
    let mut client = ClientBuilder::new()
        .application_name("soak-plc")
        .application_uri("urn:soak-plc")
        .pki_dir(scratch("soak-client"))
        .create_sample_keypair(true)
        .trust_server_certs(true)
        .session_retry_limit(1)
        .client()
        .unwrap();
    let (session, event_loop) = client
        .connect_to_matching_endpoint(
            (url.as_str(), SecurityPolicy::None.to_str(), MessageSecurityMode::None),
            IdentityToken::Anonymous,
        )
        .await
        .unwrap();
    event_loop.spawn();
    assert!(session.wait_for_connection().await);
    let ns = session.get_namespace_index(NAMESPACE).await.unwrap();
    let node = |name: String| NodeId::new(ns, name);
    let write = |values: Vec<(String, Variant)>| {
        let writes: Vec<WriteValue> = values
            .into_iter()
            .map(|(name, value)| WriteValue {
                node_id: node(name),
                attribute_id: AttributeId::Value as u32,
                index_range: NumericRange::None,
                value: DataValue { value: Some(value), ..Default::default() },
            })
            .collect();
        let session = session.clone();
        async move { assert!(session.write(&writes).await.unwrap().iter().all(|s| s.is_good())) }
    };
    let read = |names: Vec<String>| {
        let nodes: Vec<_> = names.iter().map(|n| node(n.clone()).into()).collect();
        let session = session.clone();
        async move {
            session
                .read(&nodes, TimestampsToReturn::Neither, 0.0)
                .await
                .unwrap()
                .into_iter()
                .map(|v| v.value.unwrap())
                .collect::<Vec<Variant>>()
        }
    };

    for axis in &axes {
        write(vec![(format!("axis.{}.Power.Enable", axis.id), Variant::Boolean(true))]).await;
    }
    tokio::time::sleep(Duration::from_millis(200)).await;
    write(vec![("command.reset_pacing".into(), Variant::Boolean(true))]).await;
    let memory_before = resident();
    let started = Instant::now();
    let (mut moves, mut worst_miss) = (0u64, 0f64);
    while started.elapsed().as_secs_f64() < seconds {
        // All axes at once, between 30 % and 70 % of their range, at full speed.
        let share = if moves % 2 == 0 { 0.3 } else { 0.7 };
        let targets: Vec<f64> = axes.iter().map(|a| a.min + share * (a.max - a.min)).collect();
        // The handshake a PLC uses: Execute down, wait until Done reads false (the outputs are published once a
        // cycle, so a Done read right after the write can be the last move's), then the inputs and Execute up.
        let done: Vec<String> = axes.iter().map(|a| format!("axis.{}.MoveAbsolute.Done", a.id)).collect();
        write(axes.iter().map(|a| (format!("axis.{}.MoveAbsolute.Execute", a.id), Variant::Boolean(false))).collect())
            .await;
        while read(done.clone()).await.contains(&Variant::Boolean(true)) {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let mut values = Vec::new();
        for (axis, target) in axes.iter().zip(&targets) {
            values.push((format!("axis.{}.MoveAbsolute.Position", axis.id), Variant::Double(*target)));
            values.push((format!("axis.{}.MoveAbsolute.Execute", axis.id), Variant::Boolean(true)));
        }
        write(values).await;
        let watched: Vec<String> = axes
            .iter()
            .flat_map(|a| ["MoveAbsolute.Done", "MoveAbsolute.Error", "Error"].map(|n| format!("axis.{}.{n}", a.id)))
            .collect();
        let waited = Instant::now();
        loop {
            tokio::time::sleep(Duration::from_millis(10)).await;
            let flags: Vec<bool> =
                read(watched.clone()).await.into_iter().map(|v| v == Variant::Boolean(true)).collect();
            assert!(flags.chunks(3).all(|f| !f[1] && !f[2]), "error after {moves} moves: {flags:?}");
            if flags.chunks(3).all(|f| f[0]) {
                break;
            }
            assert!(waited.elapsed().as_secs() < 5, "move {moves} not done within 5 s");
        }
        // Settled at the target; no drift from move to move.
        tokio::time::sleep(Duration::from_millis(150)).await;
        let at = read(axes.iter().map(|a| format!("axis.{}.ActualPosition", a.id)).collect()).await;
        for (value, target) in at.iter().zip(&targets) {
            let Variant::Double(at) = value else { panic!("{value:?}") };
            worst_miss = worst_miss.max((at - target).abs());
        }
        moves += 1;
    }
    let names = ["cycle_period_s", "jitter_max_s", "jitter_rms_s", "realtime_factor", "overruns", "cycles", "steps"];
    let status = read(names.iter().map(|n| format!("status.{n}")).collect()).await;
    let pacing = link.pacing.lock().unwrap().clone();
    let refused = read(vec!["status.refused".into()]).await;
    let steps = if let Variant::UInt64(n) = status[6] { n } else { panic!() };
    println!(
        "soak {seconds} s: {moves} moves, worst miss {worst_miss:.2e} m, period {:.3} ms, jitter max {:.3} ms, rms {:.3} ms, {} overruns in {} cycles, real-time factor {:.4}, {steps} steps, memory {:?} -> {:?} bytes",
        pacing.period_s() * 1e3,
        pacing.jitter_max_s * 1e3,
        pacing.jitter_rms_s() * 1e3,
        pacing.overruns,
        pacing.cycles,
        pacing.realtime_factor(),
        memory_before,
        resident()
    );
    assert_eq!(refused[0], Variant::from(""));
    assert!(moves as f64 > seconds / 3.0, "{moves} moves in {seconds} s");
    assert!(worst_miss < 2e-4, "missed a target by {worst_miss} m");
    // Real time: the simulation keeps up with the wall clock, and the steps match the time it ran.
    assert!((pacing.realtime_factor() - 1.0).abs() < 0.02, "real-time factor {}", pacing.realtime_factor());
    assert!(
        (steps as f64 * sim.dt - started.elapsed().as_secs_f64()).abs() < 1.0 + 0.02 * seconds,
        "{steps} steps in {} s",
        started.elapsed().as_secs_f64()
    );
    assert!((pacing.period_s() - 0.01).abs() < 0.002, "mean period {} s", pacing.period_s());
    // Loose bounds: a desktop OS is no real-time system, and a CI host runs other jobs. On an idle host the jitter
    // is well under a millisecond; on one with all cores busy a plain sleep loop shows 8 ms RMS.
    assert!(
        pacing.jitter_rms_s() < 0.01 && pacing.jitter_max_s < 0.1,
        "jitter rms {} s, max {} s",
        pacing.jitter_rms_s(),
        pacing.jitter_max_s
    );
    assert!(
        (pacing.overruns as f64) < 0.1 * pacing.cycles as f64,
        "{} overruns in {} cycles",
        pacing.overruns,
        pacing.cycles
    );
    if let (Some(before), Some(after)) = (memory_before, resident()) {
        assert!(after < before + (64 << 20), "memory grew from {before} to {after} bytes");
    }
}
