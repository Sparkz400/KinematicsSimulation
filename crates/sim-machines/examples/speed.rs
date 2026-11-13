fn main() {
    for entry in sim_machines::MACHINES {
        let mut cfg = entry.config();
        if std::env::var("SUBSTEPS").is_ok() { cfg.set_text(&format!("sim.substeps = {}", std::env::var("SUBSTEPS").unwrap())).unwrap(); }
        let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
        sim.command(sim_components::Command::StartCycle).unwrap();
        let t = std::time::Instant::now();
        sim.run(1.0, |_| {});
        let plain = t.elapsed().as_secs_f64();
        let mut v = Vec::new();
        let t = std::time::Instant::now();
        sim.run(1.0, |s| { v.clear(); s.sample(&mut v); });
        println!("{:20} {:7.3} s per simulated s, {:7.3} with sampling", entry.id, plain, t.elapsed().as_secs_f64());
    }
}
