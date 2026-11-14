//! The standard machine test suite. Every kinematics must pass it unchanged:
//! transformation round trip, static hold, energy conservation, step-size convergence, determinism, and the
//! energy balance with the losses the machine reports.

use crate::{ENERGY_PARTS, LOSS_PREFIX, Machine, MachineEntry, Rk4, advance};

pub const DT: f64 = 62.5e-6;
pub const SUBSTEPS: u32 = 4;
/// The suite's pulse: short enough to ring the belt, smooth enough to leave the gear mode alone.
const PULSE_S: f64 = 0.02;
const PULSE_ACC: f64 = 0.1;

pub fn run_standard_suite(entry: &MachineEntry) {
    let default = (entry.build)(&entry.config()).unwrap();
    round_trip(default.as_ref());
    static_hold(default.as_ref());
    determinism(default.as_ref());
    loss_budget(default.as_ref());
    let configs = (entry.suite_configs)();
    assert!(!configs.is_empty(), "{}: no loss-free config for the suite", entry.id);
    // A config to a thread: the six-axis arm's four took half a minute one after the other.
    std::thread::scope(|scope| {
        for config in &configs {
            scope.spawn(move || {
                let machine = (entry.build)(config).unwrap();
                round_trip(machine.as_ref());
                static_hold(machine.as_ref());
                energy_conservation(machine.as_ref());
                step_convergence(machine.as_ref());
                determinism(machine.as_ref());
                loss_budget(machine.as_ref());
            });
        }
    });
}

/// Repeatable measurement noise from a linear congruential generator: for tests, and for the drives'
/// sensor noise, which has to be the same in every run.
#[derive(Debug, Clone, Default)]
pub struct Noise(pub u64);

impl Noise {
    /// Evenly spread from 0 up to, but not including, 1.
    pub fn uniform(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Roughly normal with standard deviation 1: twelve uniform numbers summed.
    #[allow(clippy::should_implement_trait)] // an endless source, not an iterator: every test calls it by this name
    pub fn next(&mut self) -> f64 {
        (0..12).map(|_| self.uniform()).sum::<f64>() - 6.0
    }
}

pub fn mid_pose(m: &dyn Machine) -> Vec<f64> {
    m.axes().iter().map(|a| 0.5 * (a.min + a.max)).collect()
}

/// Forward of inverse returns the starting pose within 1e-9, on a grid over the whole workspace: the box of the
/// axis ranges, less the poses the machine reports it cannot go to (`Machine::unreachable`).
pub fn round_trip(m: &dyn Machine) {
    let axes = m.axes();
    let (mut motors, mut back) = (vec![0.0; m.motors().len()], vec![0.0; axes.len()]);
    let mut tried = 0;
    for k in 0..5usize.pow(axes.len() as u32) {
        let pose: Vec<f64> = axes
            .iter()
            .enumerate()
            .map(|(i, a)| a.min + (a.max - a.min) * ((k / 5usize.pow(i as u32)) % 5) as f64 / 4.0)
            .collect();
        if m.unreachable(&pose, 0.0).is_some() {
            continue;
        }
        tried += 1;
        m.inverse(&pose, &mut motors);
        m.forward(&motors, &mut back);
        for (p, b) in pose.iter().zip(&back) {
            assert!((p - b).abs() < 1e-9, "round trip at {pose:?} gave {back:?}");
        }
    }
    assert!(2 * tried > 5usize.pow(axes.len() as u32) / 2, "only {tried} poses of the grid are in the workspace");
}

/// At the equilibrium state, under the holding torques, nothing moves for one second.
pub fn static_hold(m: &dyn Machine) {
    let n = m.axes().len();
    let pose = mid_pose(m);
    let (mut state, mut hold) = (Vec::new(), vec![0.0; m.motors().len()]);
    m.equilibrium(&pose, &mut state, &mut hold);
    assert_eq!(state.len(), m.state_len());
    let mut rk = Rk4::new(state.len());
    for _ in 0..(1.0 / DT) as usize {
        advance(m, &mut rk, &mut state, &hold, DT, SUBSTEPS);
    }
    let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
    m.tool_state(&state, &mut pos, &mut vel);
    for i in 0..n {
        assert!((pos[i] - pose[i]).abs() < 1e-7, "axis {i} moved from {} to {}", pose[i], pos[i]);
        assert!(vel[i].abs() < 1e-6, "axis {i} has speed {}", vel[i]);
    }
}

/// A run that starts at rest, gets one smooth torque pulse and then only the holding torques.
pub struct PulseRun<'a> {
    m: &'a dyn Machine,
    rk: Rk4,
    pub state: Vec<f64>,
    pub hold: Vec<f64>,
    /// The torques of the last step.
    pub torque: Vec<f64>,
    delta: Vec<f64>,
    motor0: Vec<f64>,
    step: usize,
    dt: f64,
    pulse_s: f64,
    pub substeps: u32,
}

impl<'a> PulseRun<'a> {
    /// The pulse is sin^2 shaped, `pulse_s` long, and asks for `acc_fraction` of each axis's
    /// acceleration limit at its peak.
    pub fn new(m: &'a dyn Machine, dt: f64, acc_fraction: f64, pulse_s: f64) -> Self {
        let pose = mid_pose(m);
        let motors = m.motors().len();
        let (mut state, mut hold) = (Vec::new(), vec![0.0; motors]);
        m.equilibrium(&pose, &mut state, &mut hold);
        let acc: Vec<f64> = m.axes().iter().map(|a| acc_fraction * a.max_acc).collect();
        let (mut with, mut without) = (vec![0.0; motors], vec![0.0; motors]);
        m.feedforward_torque(&pose, &acc, &mut with);
        m.feedforward_torque(&pose, &vec![0.0; acc.len()], &mut without);
        let delta = with.iter().zip(&without).map(|(a, b)| a - b).collect();
        let mut run = Self { m, rk: Rk4::new(state.len()), torque: hold.clone(), state, hold, delta, motor0: vec![0.0; motors], step: 0, dt, pulse_s, substeps: SUBSTEPS };
        run.motor0 = run.motor_pos();
        run
    }

    fn motor_pos(&self) -> Vec<f64> {
        let n = self.m.motors().len();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.m.motor_state(&self.state, &mut pos, &mut vel);
        pos
    }

    pub fn tool_pos(&self) -> Vec<f64> {
        let n = self.m.axes().len();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.m.tool_state(&self.state, &mut pos, &mut vel);
        pos
    }

    pub fn run(&mut self, seconds: f64, mut each: impl FnMut(&Self)) {
        for _ in 0..(seconds / self.dt).round() as usize {
            let t = self.step as f64 * self.dt;
            let pulse = if t < self.pulse_s { (std::f64::consts::PI * t / self.pulse_s).sin().powi(2) } else { 0.0 };
            self.torque = self.hold.iter().zip(&self.delta).map(|(h, d)| h + d * pulse).collect();
            advance(self.m, &mut self.rk, &mut self.state, &self.torque, self.dt, self.substeps);
            self.step += 1;
            each(self);
        }
    }

    /// Energy minus the work of the constant holding torques: constant once the pulse is over.
    fn invariant(&self) -> f64 {
        let work: f64 =
            self.motor_pos().iter().zip(&self.motor0).zip(&self.hold).map(|((p, p0), h)| h * (p - p0)).sum();
        self.m.energy(&self.state) - work
    }
}

/// With friction and control off, total energy drifts less than 0.1 % over 10 s.
/// The drift is measured against the energy the pulse put in, which is far stricter than
/// measuring against the gravitational potential.
pub fn energy_conservation(m: &dyn Machine) {
    let mut run = PulseRun::new(m, DT, PULSE_ACC, PULSE_S);
    let before = run.invariant();
    run.run(PULSE_S, |_| {});
    let after = run.invariant();
    let injected = after - before;
    assert!(injected > 0.0, "the pulse put in no energy ({injected})");
    let mut worst = 0f64;
    run.run(10.0, |r| worst = worst.max((r.invariant() - after).abs()));
    assert!(worst < 1e-3 * injected, "energy drifted by {:.4} % of the {injected} J put in", 100.0 * worst / injected);
}

/// Tolerance of `loss_budget`, as a share of the work the motor torques did either way.
pub const LOSS_BUDGET_TOLERANCE: f64 = 1e-3;

/// The energy balance with losses: over a run with a strong torque pulse, the work of the motor torques
/// (sum of torque times rotor angle turned, exact for torques held over a step) equals the change of the
/// machine's energy plus the time integral of every loss power it reports (signals starting with `LOSS_PREFIX`),
/// to within `LOSS_BUDGET_TOLERANCE`. The run takes the integrator's own sub-steps (`DT / SUBSTEPS`, one each) as its
/// steps; the powers are taken at the end of each and summed times the step, which is the trapezoid rule but for the
/// two ends of the run. What is left is that rule's error on powers that change within a step (the bristles of a
/// friction contact settle within microseconds when the speed reverses), and on a power that jumps, up to half of
/// it times the step: a stop's damper takes c v^2 as soon as the axis touches it. On the linear delta, whose
/// carriages the pulse takes into their upper stops, one touches at 0.533 m/s 1.7 us before the end of a step:
/// 5000 N s/m * 0.533^2 = 1422 W, of which the trapezoid booked 0.5 * 1422 W * 62.5 us = 0.044 J where 0.002 J were
/// lost, 0.042 J off against the 0.018 J allowed; on the 15.6 us sub-steps at most 0.011 J.
/// The energy parts it reports (`energy.*`) add up to `Machine::energy`. A machine that reports no loss powers is
/// not checked.
pub fn loss_budget(m: &dyn Machine) {
    let infos = m.signal_infos();
    let pick = |prefix: &str| infos.iter().enumerate().filter(|(_, s)| s.id.starts_with(prefix)).map(|(i, _)| i).collect::<Vec<_>>();
    let (losses, parts) = (pick(LOSS_PREFIX), pick("energy."));
    if losses.is_empty() {
        return;
    }
    assert_eq!(parts.len(), ENERGY_PARTS.len(), "the energy parts are not all reported");
    let mut run = PulseRun::new(m, DT / SUBSTEPS as f64, 0.5, 0.1);
    run.substeps = 1;
    let start = m.energy(&run.state);
    let (mut before, mut work, mut gross, mut lost, mut signals) = (run.motor_pos(), 0.0, 0.0, 0.0, Vec::new());
    run.run(1.0, |r| {
        let after = r.motor_pos();
        let step: f64 = r.torque.iter().zip(after.iter().zip(&before)).map(|(t, (a, b))| t * (a - b)).sum();
        (work, gross, before) = (work + step, gross + step.abs(), after);
        signals.clear();
        m.signals(&r.state, &r.torque, &mut signals);
        lost += losses.iter().map(|&i| signals[i]).sum::<f64>() * r.dt;
        let (energy, sum) = (m.energy(&r.state), parts.iter().map(|&i| signals[i]).sum::<f64>());
        assert!((sum - energy).abs() <= 1e-9 * energy.abs().max(1.0), "energy parts add up to {sum}, not {energy}");
    });
    let stored = m.energy(&run.state) - start;
    let off = work - stored - lost;
    println!("loss budget: work {work:.6} J (gross {gross:.6}), stored {stored:.6} J, lost {lost:.6} J, off {off:.3e} J");
    assert!(off.abs() < LOSS_BUDGET_TOLERANCE * gross, "work {work} J, stored {stored} J, lost {lost} J: off by {off} J of {gross} J");
}

/// Halving the time step changes the result by less than 1 %.
pub fn step_convergence(m: &dyn Machine) {
    let travel = |dt: f64| {
        let mut run = PulseRun::new(m, dt, PULSE_ACC, PULSE_S);
        let start = run.tool_pos();
        run.run(1.0, |_| {});
        run.tool_pos().iter().zip(&start).map(|(p, s)| p - s).collect::<Vec<f64>>()
    };
    let (coarse, fine) = (travel(DT), travel(DT / 2.0));
    for (c, f) in coarse.iter().zip(&fine) {
        assert!(f.abs() > 1e-6, "axis did not move, the check would be empty");
        assert!((c - f).abs() < 0.01 * f.abs(), "step halving changed travel from {c} to {f}");
    }
}

/// The same config and the same commands give bit-identical traces.
pub fn determinism(m: &dyn Machine) {
    let trace = || {
        let mut run = PulseRun::new(m, DT, PULSE_ACC, PULSE_S);
        let mut bits = Vec::new();
        run.run(0.1, |r| bits.extend(r.state.iter().map(|v| v.to_bits())));
        bits
    };
    assert!(trace() == trace(), "two identical runs differ");
}
