//! The simulation in its own thread, on its own clock. The interface sends requests and reads
//! samples from a channel; it never blocks the physics, and the physics never waits for a frame.

use sim_components::{Command, LimitViolation, Sim};
use sim_core::{AxisInfo, Config, ConfigError, MachineEntry, MotorInfo, Scene, SignalInfo};
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub enum Request {
    Command(Command),
    /// A changed config for the running simulation: parameters and effect switches.
    Reconfigure(Config),
    Pause(bool),
    /// Simulated seconds per real second: 1 is real time, below 1 slow motion.
    Speed(f64),
    Quit,
}

#[derive(Debug, Clone)]
pub enum Event {
    /// One row of every signal, in the order of the signal list.
    Sample(Vec<f64>),
    Scene(Scene),
    /// A refused command or config, as a translation key and a detail.
    Refused(&'static str, String),
    Limits(Vec<LimitViolation>),
}

/// The simulation and the handling of requests, without any thread or clock. The session thread
/// drives it; tests drive it directly, which makes them exact.
pub struct Runner {
    pub sim: Sim,
    entry: &'static MachineEntry,
    /// Base steps between two samples: one sample per millisecond.
    sample_every: u64,
}

impl Runner {
    pub fn new(entry: &'static MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        let sim = Sim::new(entry, cfg)?;
        let sample_every = (1e-3 / sim.dt).round().max(1.0) as u64;
        Ok(Self { sim, entry, sample_every })
    }

    pub fn handle(&mut self, request: Request, events: &mut Vec<Event>) {
        match request {
            Request::Command(command) => {
                if let Err(e) = self.sim.command(command) {
                    events.push(Event::Refused(e.key, String::new()));
                }
            }
            Request::Reconfigure(cfg) => match self.sim.reconfigure(self.entry, &cfg) {
                Ok(()) => events.push(Event::Limits(self.sim.limit_violations.clone())),
                Err(e) => events.push(Event::Refused(e.key(), e.detail().into())),
            },
            Request::Pause(_) | Request::Speed(_) | Request::Quit => {}
        }
    }

    /// Runs whole base steps for `seconds` of simulated time and reports the samples.
    pub fn advance(&mut self, seconds: f64, events: &mut Vec<Event>) {
        for _ in 0..(seconds / self.sim.dt).round() as u64 {
            self.sim.step();
            if self.sim.step % self.sample_every == 0 {
                let mut sample = Vec::new();
                self.sim.sample(&mut sample);
                events.push(Event::Sample(sample));
            }
        }
    }
}

pub struct Session {
    requests: Sender<Request>,
    /// Behind a mutex only so that a session can live in shared application state.
    events: Mutex<Receiver<Event>>,
    thread: Option<JoinHandle<()>>,
    pub signals: Vec<SignalInfo>,
    pub axes: Vec<AxisInfo>,
    pub motors: Vec<MotorInfo>,
}

impl Session {
    pub fn start(entry: &'static MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        let mut runner = Runner::new(entry, cfg)?;
        let signals = runner.sim.signal_infos();
        let (axes, motors) = (runner.sim.machine.axes().to_vec(), runner.sim.machine.motors().to_vec());
        let (requests, inbox) = channel();
        let (outbox, events) = channel();
        let _ = outbox.send(Event::Limits(runner.sim.limit_violations.clone()));
        let thread = std::thread::spawn(move || {
            let (mut paused, mut speed, mut owed) = (false, 1.0, 0.0);
            let (mut last, mut last_scene) = (Instant::now(), Instant::now() - Duration::from_secs(1));
            let mut batch = Vec::new();
            loop {
                for request in inbox.try_iter() {
                    match request {
                        Request::Quit => return,
                        Request::Pause(on) => paused = on,
                        Request::Speed(factor) => speed = factor.clamp(0.0, 100.0),
                        other => runner.handle(other, &mut batch),
                    }
                }
                // The wall clock only decides how many steps are due; the core never sees it.
                let now = Instant::now();
                if !paused {
                    owed += (now - last).as_secs_f64() * speed;
                }
                last = now;
                // A machine too slow for the requested speed falls behind instead of piling up work.
                owed = f64::min(owed, 0.05);
                let whole = (owed / runner.sim.dt).floor() * runner.sim.dt;
                runner.advance(whole, &mut batch);
                owed -= whole;
                if now - last_scene > Duration::from_millis(8) {
                    let mut scene = Scene::default();
                    runner.sim.scene(&mut scene);
                    batch.push(Event::Scene(scene));
                    last_scene = now;
                }
                for event in batch.drain(..) {
                    if outbox.send(event).is_err() {
                        return;
                    }
                }
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        Ok(Self { requests, events: Mutex::new(events), thread: Some(thread), signals, axes, motors })
    }

    /// Everything the simulation has produced since the last call. Never waits.
    pub fn drain(&self) -> Vec<Event> {
        self.events.lock().map(|events| events.try_iter().collect()).unwrap_or_default()
    }

    pub fn send(&self, request: Request) {
        let _ = self.requests.send(request);
    }
}

impl Drop for Session {
    fn drop(&mut self) {
        self.send(Request::Quit);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}
