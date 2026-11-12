//! The simulation in its own thread, on its own clock. The interface sends requests and reads
//! samples from a channel; it never blocks the physics, and the physics never waits for a frame.

use sim_components::{Command, LimitViolation, Sim};
use sim_core::{AxisInfo, Config, ConfigError, MachineEntry, MotorInfo, Scene, SignalInfo};
use std::collections::VecDeque;
use std::sync::mpsc::{Sender, TryRecvError, channel};
use std::sync::{Arc, Mutex};
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
    /// The travelled path starts anew at this place in the scene: the machine has taken a trajectory. `None`
    /// ends it: the machine was reconfigured, started its cycle, or has no place for its tool in the scene.
    TrailStart(Option<[f64; 3]>),
    /// Where the tool has been since, in order: a point per 0.1 mm of travel, from every simulation step.
    Trail(Vec<[f64; 3]>),
    /// A refused command, as a translation key and a detail.
    Refused(&'static str, String),
    Limits(Vec<LimitViolation>),
    /// The answer to `Request::Reconfigure`, one for each: the config was taken, with the axis limits that
    /// ask too much of the motors under it, or it was refused, with a translation key and a detail.
    Configured(Result<Vec<LimitViolation>, (&'static str, String)>),
}

/// Samples the interface may be behind by: ten seconds at real time. Beyond that the oldest are dropped.
pub const BACKLOG: usize = 10_000;

/// Points of the travelled path that are kept: 10 m of travel. Beyond that the oldest are dropped.
pub const TRAIL: usize = 100_000;

/// Adds points to a travelled path and keeps the newest `TRAIL` of it.
pub fn extend_trail(trail: &mut Vec<[f64; 3]>, points: &[[f64; 3]]) {
    trail.extend_from_slice(points);
    trail.drain(..trail.len().saturating_sub(TRAIL));
}

/// What the simulation has produced and the interface has not read yet. It is bounded, so that a window
/// that stalls costs samples and not memory: of the samples the newest `BACKLOG` are kept, of the scenes
/// the newest, of the travelled path the newest `TRAIL` points. Answers to requests are few and are all kept.
#[derive(Debug, Default)]
pub struct Outbox {
    events: VecDeque<Event>,
    samples: usize,
}

impl Outbox {
    pub fn push(&mut self, event: Event) {
        match &event {
            Event::Scene(_) => self.events.retain(|waiting| !matches!(waiting, Event::Scene(_))),
            // The points that wait belong to the path that ends here.
            Event::TrailStart(_) => self.events.retain(|waiting| !matches!(waiting, Event::Trail(_))),
            Event::Trail(points) => {
                if let Some(Event::Trail(waiting)) = self.events.iter_mut().find(|waiting| matches!(waiting, Event::Trail(_))) {
                    return extend_trail(waiting, points);
                }
            }
            Event::Sample(_) if self.samples == BACKLOG => {
                let oldest = self.events.iter().position(|waiting| matches!(waiting, Event::Sample(_)));
                oldest.and_then(|at| self.events.remove(at));
            }
            Event::Sample(_) => self.samples += 1,
            _ => {}
        }
        self.events.push_back(event);
    }

    pub fn drain(&mut self) -> Vec<Event> {
        self.samples = 0;
        self.events.drain(..).collect()
    }
}

/// The simulation and the handling of requests, without any thread or clock. The session thread
/// drives it; tests drive it directly, which makes them exact.
pub struct Runner {
    pub sim: Sim,
    entry: &'static MachineEntry,
    /// Base steps between two samples: one sample per millisecond.
    sample_every: u64,
    /// A drive has tripped and this has been reported.
    tripped: bool,
    /// The last point reported of the travelled path; `None` while none is recorded.
    trail: Option<[f64; 3]>,
    /// The travelled path still grows: its trajectory runs. Finished, it stays shown as it is.
    recording: bool,
}

impl Runner {
    pub fn new(entry: &'static MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        let sim = Sim::new(entry, cfg)?;
        let sample_every = (1e-3 / sim.dt).round().max(1.0) as u64;
        Ok(Self { sim, entry, sample_every, tripped: false, trail: None, recording: false })
    }

    pub fn handle(&mut self, request: Request, events: &mut Vec<Event>) {
        match request {
            Request::Command(command) => {
                // A path the machine takes starts the travelled path anew, a cycle ends it; refused, neither.
                let (path, cycle) = (matches!(command, Command::Trajectory(_)), matches!(command, Command::StartCycle));
                match self.sim.command(command) {
                    Err(e) => events.push(Event::Refused(e.key, String::new())),
                    Ok(()) if path || cycle => self.start_trail(if path { self.sim.tool_point() } else { None }, events),
                    // Any other command ends the path: what the machine does next is not part of it.
                    Ok(()) => self.recording = false,
                }
            }
            Request::Reconfigure(cfg) => {
                let answer = self.sim.reconfigure(self.entry, &cfg);
                if answer.is_ok() {
                    self.start_trail(None, events);
                }
                events.push(Event::Configured(match answer {
                    Ok(()) => Ok(self.sim.limit_violations.clone()),
                    Err(e) => Err((e.key(), e.detail().into())),
                }));
            }
            Request::Pause(_) | Request::Speed(_) | Request::Quit => {}
        }
    }

    fn start_trail(&mut self, at: Option<[f64; 3]>, events: &mut Vec<Event>) {
        // Nothing to end where nothing is recorded: a parameter edit on a machine that runs no path.
        if at.is_some() || self.trail.is_some() {
            events.push(Event::TrailStart(at));
        }
        self.trail = at;
        self.recording = at.is_some();
    }

    /// Runs whole base steps for `seconds` of simulated time and reports the samples.
    pub fn advance(&mut self, seconds: f64, events: &mut Vec<Event>) {
        let mut travelled = Vec::new();
        for _ in 0..(seconds / self.sim.dt).round() as u64 {
            self.sim.step();
            if let (true, Some(last), Some(tool)) = (self.recording, self.trail, self.sim.tool_point()) {
                if (0..3).map(|i| (tool[i] - last[i]).powi(2)).sum::<f64>() > 1e-8 {
                    travelled.push(tool);
                    self.trail = Some(tool);
                }
            }
            // The trajectory is through: the path ends with it.
            self.recording &= self.sim.control.busy();
            if self.sim.step % self.sample_every == 0 {
                let mut sample = Vec::new();
                self.sim.sample(&mut sample);
                events.push(Event::Sample(sample));
            }
            // A trip takes every drive off, which otherwise shows only as a machine that stops. Said once
            // per trip; the drives stay tripped until they are enabled.
            let tripped = self.sim.control.drives.iter().any(|drive| drive.tripped);
            if tripped && !self.tripped {
                events.push(Event::Refused("ui-drive-tripped", String::new()));
            }
            self.tripped = tripped;
        }
        if !travelled.is_empty() {
            events.push(Event::Trail(travelled));
        }
    }
}

pub struct Session {
    requests: Sender<Request>,
    events: Arc<Mutex<Outbox>>,
    thread: JoinHandle<()>,
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
        let events = Arc::new(Mutex::new(Outbox::default()));
        let outbox = events.clone();
        if let Ok(mut outbox) = outbox.lock() {
            outbox.push(Event::Limits(runner.sim.limit_violations.clone()));
        }
        let thread = std::thread::spawn(move || {
            let (mut paused, mut speed, mut owed) = (false, 1.0, 0.0);
            let (mut last, mut last_scene) = (Instant::now(), Instant::now() - Duration::from_secs(1));
            let mut batch = Vec::new();
            loop {
                loop {
                    match inbox.try_recv() {
                        // Asked to end, or nobody is left to ask.
                        Ok(Request::Quit) | Err(TryRecvError::Disconnected) => return,
                        Ok(Request::Pause(on)) => paused = on,
                        Ok(Request::Speed(factor)) => speed = factor.clamp(0.0, 100.0),
                        Ok(other) => runner.handle(other, &mut batch),
                        Err(TryRecvError::Empty) => break,
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
                // Held only to move the batch over: the interface never makes the physics wait for a frame.
                let Ok(mut outbox) = outbox.lock() else { return };
                batch.drain(..).for_each(|event| outbox.push(event));
                drop(outbox);
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        Ok(Self { requests, events, thread, signals, axes, motors })
    }

    /// Everything the simulation has produced since the last call, less what `Outbox` dropped. Never waits.
    pub fn drain(&self) -> Vec<Event> {
        self.events.lock().map(|mut events| events.drain()).unwrap_or_default()
    }

    /// The simulation thread is gone. Nobody asked it to end while the session lives, so it panicked.
    pub fn ended(&self) -> bool {
        self.thread.is_finished()
    }

    pub fn send(&self, request: Request) {
        let _ = self.requests.send(request);
    }
}

impl Drop for Session {
    /// The thread is told to end and left to it. Waiting for it here would hold up the window for as long
    /// as the steps it is in the middle of take, when the machine is switched.
    fn drop(&mut self) {
        self.send(Request::Quit);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    /// A reader that is behind: the queue used to grow without bound. It keeps the newest samples, the
    /// newest scene and every answer.
    #[test]
    fn a_reader_that_is_behind_costs_samples_and_not_memory() {
        let mut outbox = Outbox::default();
        outbox.push(Event::Refused("first", String::new()));
        for n in 0..3 * BACKLOG {
            outbox.push(Event::Sample(vec![n as f64]));
            if n % 8 == 0 {
                outbox.push(Event::Scene(Scene::default()));
            }
            if n == BACKLOG {
                outbox.push(Event::Configured(Ok(Vec::new())));
            }
            assert!(outbox.events.len() <= BACKLOG + 3);
        }
        let events = outbox.drain();
        let samples: Vec<f64> = events.iter().filter_map(|e| if let Event::Sample(s) = e { Some(s[0]) } else { None }).collect();
        assert_eq!((samples.len(), samples[0], samples[BACKLOG - 1]), (BACKLOG, (2 * BACKLOG) as f64, (3 * BACKLOG - 1) as f64));
        assert!(samples.windows(2).all(|w| w[1] == w[0] + 1.0), "the samples that are left are out of order");
        assert_eq!(events.iter().filter(|e| matches!(e, Event::Scene(_))).count(), 1);
        assert!(matches!(events[0], Event::Refused("first", _)) && events.iter().any(|e| matches!(e, Event::Configured(Ok(_)))));
        // Read, the queue starts anew.
        outbox.push(Event::Sample(vec![0.0]));
        assert_eq!(outbox.drain().len(), 1);
    }

    /// The same through a session whose reader sleeps: what it then reads is bounded and ends with the
    /// newest state, and the simulation thread was never held up.
    #[test]
    fn a_session_whose_reader_sleeps_still_delivers_the_newest_state() {
        let entry = sim_machines::MACHINES[0];
        let session = Session::start(entry, &entry.config()).unwrap();
        session.send(Request::Speed(100.0));
        std::thread::sleep(Duration::from_millis(1500));
        let events = session.drain();
        let times: Vec<f64> = events.iter().filter_map(|e| if let Event::Sample(s) = e { Some(s[0]) } else { None }).collect();
        assert!(times.len() <= BACKLOG && events.len() <= BACKLOG + 2, "{} events waited", events.len());
        assert_eq!(events.iter().filter(|e| matches!(e, Event::Scene(_))).count(), 1);
        std::thread::sleep(Duration::from_millis(100));
        let newer: Vec<f64> = session.drain().iter().filter_map(|e| if let Event::Sample(s) = e { Some(s[0]) } else { None }).collect();
        // What was read first reaches up to where the next read goes on: nothing newer was dropped.
        assert!(!newer.is_empty() && (newer[0] - times[times.len() - 1] - 1e-3).abs() < 1e-9, "{:?} then {:?}", times.last(), newer.first());
    }

    /// Dropping a session (switching machine) does not wait for the simulation thread. With steps that
    /// take long, the join it used to do held the window up for as long as the thread was busy.
    #[test]
    fn dropping_a_session_does_not_wait_for_its_thread() {
        let entry = sim_machines::MACHINES[0];
        let mut slow = entry.config();
        slow.set_text("sim.substeps = 2000").unwrap();
        let session = Session::start(entry, &slow).unwrap();
        std::thread::sleep(Duration::from_millis(300));
        let start = Instant::now();
        drop(session);
        assert!(start.elapsed() < Duration::from_millis(20), "dropping the session took {:?}", start.elapsed());
    }
}
