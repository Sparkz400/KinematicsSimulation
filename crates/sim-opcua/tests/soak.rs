//! Soak test: the link runs for `SOAK_SECONDS` (default 20) while a PLC client moves the axes back and forth with
//! PLCopen function blocks, as fast as the handshake allows. Every move must end with Done at its target, no block
//! or axis may report an error, and the pacing's bookkeeping must add up: what the loop simulated and what it dropped
//! as too late is the wall time (on a host that keeps up nothing is dropped and the real-time factor is 1). Waits are
//! in simulated time, so a loaded host runs fewer moves but no other test. `SOAK_SECONDS=3600 cargo test -p sim-opcua
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
    // Powered once the loop says so, not after a fixed wait.
    let powered: Vec<String> = axes.iter().map(|a| format!("axis.{}.Power.Status", a.id)).collect();
    let since = Instant::now();
    while read(powered.clone()).await.contains(&Variant::Boolean(false)) {
        assert!(since.elapsed().as_secs() < 60, "the axes were not powered within a minute");
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    write(vec![("command.reset_pacing".into(), Variant::Boolean(true))]).await;
    let memory_before = resident();
    let started = Instant::now();
    let (mut moves, mut worst_miss) = (0u64, 0f64);
    let reader = &read;
    let time = move || async move {
        match reader(vec!["signal.time".into()]).await[0] {
            Variant::Double(t) => t,
            ref v => panic!("signal.time: {v:?}"),
        }
    };
    let simulated_start = time().await;
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
        let (waited, from) = (Instant::now(), time().await);
        loop {
            tokio::time::sleep(Duration::from_millis(10)).await;
            let flags: Vec<bool> =
                read(watched.clone()).await.into_iter().map(|v| v == Variant::Boolean(true)).collect();
            assert!(flags.chunks(3).all(|f| !f[1] && !f[2]), "error after {moves} moves: {flags:?}");
            if flags.chunks(3).all(|f| f[0]) {
                break;
            }
            // 5 s of simulated time; two minutes of wall time only against a hang.
            let took = time().await - from;
            assert!(took < 5.0 && waited.elapsed().as_secs() < 120, "move {moves} not done within 5 s ({took} s)");
        }
        // Settled at the target 0.15 s of simulated time on; no drift from move to move.
        let settled = time().await + 0.15;
        while time().await < settled {
            tokio::time::sleep(Duration::from_millis(5)).await;
        }
        let at = read(axes.iter().map(|a| format!("axis.{}.ActualPosition", a.id)).collect()).await;
        for (value, target) in at.iter().zip(&targets) {
            let Variant::Double(at) = value else { panic!("{value:?}") };
            worst_miss = worst_miss.max((at - target).abs());
        }
        moves += 1;
    }
    let names = ["cycle_period_s", "jitter_max_s", "jitter_rms_s", "realtime_factor", "overruns", "cycles", "steps"];
    // The steps and the simulated time in one read: published in the same tick.
    let status = read(names.iter().map(|n| format!("status.{n}")).chain(["signal.time".into()]).collect()).await;
    let pacing = link.pacing.lock().unwrap().clone();
    let refused = read(vec!["status.refused".into()]).await;
    let steps = if let Variant::UInt64(n) = status[6] { n } else { panic!() };
    println!(
        "soak {seconds} s: {moves} moves, worst miss {worst_miss:.2e} m, period {:.3} ms, jitter max {:.3} ms, rms {:.3} ms, {} overruns in {} cycles, real-time factor {:.4}, {:.3} s lost, {steps} steps, memory {:?} -> {:?} bytes",
        pacing.period_s() * 1e3,
        pacing.jitter_max_s * 1e3,
        pacing.jitter_rms_s() * 1e3,
        pacing.overruns,
        pacing.cycles,
        pacing.realtime_factor(),
        pacing.lost_s,
        memory_before,
        resident()
    );
    assert_eq!(refused[0], Variant::from(""));
    // Moves in the simulated time (a move and its handshake take about a second of it, the 0.15 s of settling
    // included); counted in wall time, a loaded host ran fewer than one in 3 s.
    let simulated = time().await - simulated_start;
    assert!(moves as f64 > simulated / 3.0, "{moves} moves in {simulated} s of simulated time");
    assert!(worst_miss < 2e-4, "missed a target by {worst_miss} m");
    // The steps are the simulated time, and the bookkeeping adds up: what was simulated and what was dropped as too
    // late is the wall time, to within the step carried over the reset and the one left at the end. Nothing is dropped
    // where no tick came more than a cycle late (the cap is ten cycles here).
    let Variant::Double(at) = status[7] else { panic!("{:?}", status[7]) };
    assert!((steps as f64 * sim.dt - at).abs() <= sim.dt + 1e-9, "{steps} steps at {at} s");
    assert!(
        (pacing.sim_s + pacing.lost_s - pacing.wall_s).abs() <= 2.0 * sim.dt + 1e-9,
        "simulated {} s and lost {} s of {} s",
        pacing.sim_s,
        pacing.lost_s,
        pacing.wall_s
    );
    if pacing.overruns == 0 {
        assert!(pacing.lost_s <= sim.dt, "lost {} s without an overrun", pacing.lost_s);
    }
    // The mean period is off the cycle by no more than the jitter's root mean square (whatever the host gave).
    assert!(
        (pacing.period_s() - 0.01).abs() <= pacing.jitter_rms_s() + 1e-12
            && pacing.jitter_rms_s() <= pacing.jitter_max_s + 1e-12,
        "mean period {} s, jitter rms {} s, max {} s",
        pacing.period_s(),
        pacing.jitter_rms_s(),
        pacing.jitter_max_s
    );
    if let (Some(before), Some(after)) = (memory_before, resident()) {
        assert!(after < before + (64 << 20), "memory grew from {before} to {after} bytes");
    }
}
