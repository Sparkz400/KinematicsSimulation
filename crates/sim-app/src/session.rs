//! The simulation in its own thread, on its own clock. The interface sends requests and reads
//! samples from a channel; it never blocks the physics, and the physics never waits for a frame.

use sim_components::{Command, LimitViolation, Sim};
use sim_core::{AxisInfo, Config, ConfigError, Label, MachineEntry, MotorInfo, Scene, SignalInfo};
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
    /// How many times larger the 3D view draws the parts' deflections (`Scene::exaggeration`).
    Exaggeration(f64),
    /// The 3D view shows the machine swinging in its mode (`mode_scene`) in place of the live state, which waits.
    Mode(bool),
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

/// A point of the travelled path with what the 3D view may colour it by: where the plan had the tool at that
/// step, in the scene, and how fast the tool went.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrailPoint {
    pub at: [f64; 3],
    pub planned: [f64; 3],
    pub speed: f64,
}

/// What only the window reads, apart from `Event`s (which the model drains): the travelled path's points with
/// their speed and plan, and whether the machine tells a mode to show.
#[derive(Debug, Default)]
pub struct ViewNews {
    /// The path started anew since the last read: what the window holds of it ends.
    pub restarted: bool,
    pub trail: Vec<TrailPoint>,
    pub has_mode: bool,
}

/// Swings per second of the mode in the 3D view: slowed down to about one, so that the eye can follow it.
pub const MODE_SHOWN_HZ: f64 = 1.0;
/// How far the mode's amplitude swings, in m, per time the view draws the deflections larger, and at most: the
/// shape moves by it whichever of carriage and tool moves most (`Machine::mode_shape`). A few millimetres read on a
/// machine of metres, and up to a tenth of a metre the parts still hang together.
pub const MODE_TRAVEL: f64 = 0.004;
pub const MODE_TRAVEL_MOST: f64 = 0.1;

/// The machine at rest at its planned pose plus its mode (`Machine::mode_shape`) swinging at `MODE_SHOWN_HZ`,
/// `seconds` after it started, with the mode's true frequency as a label by the tool. `exaggeration` scales the
/// whole shape through its amplitude, and the scene draws it at true scale (`Scene::exaggeration` 1): the scene
/// draws the carriage's part of the shape at true scale and only the lean larger, which would distort the shape.
/// `None` where the machine tells no mode.
pub fn mode_scene(sim: &Sim, exaggeration: f64, seconds: f64) -> Option<Scene> {
    let machine = &sim.machine;
    let pose = sim.control.planned_pos();
    let (hz, shape) = machine.mode_shape(pose)?;
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(pose, &mut state, &mut torque);
    let amplitude =
        (MODE_TRAVEL * exaggeration).min(MODE_TRAVEL_MOST) * (std::f64::consts::TAU * MODE_SHOWN_HZ * seconds).sin();
    state.iter_mut().zip(&shape).for_each(|(s, d)| *s += amplitude * d);
    let mut scene = Scene { exaggeration: 1.0, ..Scene::default() };
    machine.scene(&state, &mut scene);
    let (mut pos, mut vel) = (vec![0.0; pose.len()], vec![0.0; pose.len()]);
    machine.tool_state(&state, &mut pos, &mut vel);
    (scene.tool, scene.planned) = (machine.scene_point(&pos), machine.scene_point(pose));
    let at = scene.tool.unwrap_or_default();
    // The live values drawn from the swinging state would show the swing, not the machine: only the mode's label.
    scene.labels.clear();
    scene.labels.push(Label {
        pos: [at[0] + 0.1, at[1], at[2]],
        owner_key: "ui-view-mode-label",
        value: hz,
        unit_key: "unit-hz",
        decimals: 1,
    });
    Some(scene)
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
                if let Some(Event::Trail(waiting)) =
                    self.events.iter_mut().find(|waiting| matches!(waiting, Event::Trail(_)))
                {
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
    /// A monitor's trip (`Controller::fault`) and the stop after it have been reported.
    fault_told: bool,
    stop_told: bool,
    /// The last point reported of the travelled path; `None` while none is recorded.
    trail: Option<[f64; 3]>,
    /// The travelled path still grows: its trajectory runs. Finished, it stays shown as it is.
    recording: bool,
    /// The points of `Event::Trail` since the session thread last took them, with their speed and plan, and whether
    /// the path started anew meanwhile (`ViewNews`).
    pub trail_points: Vec<TrailPoint>,
    pub trail_restarted: bool,
}

impl Runner {
    pub fn new(entry: &'static MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        let sim = Sim::new(entry, cfg)?;
        let sample_every = (1e-3 / sim.dt).round().max(1.0) as u64;
        Ok(Self {
            sim,
            entry,
            sample_every,
            tripped: false,
            fault_told: false,
            stop_told: false,
            trail: None,
            recording: false,
            trail_points: Vec::new(),
            trail_restarted: false,
        })
    }

    pub fn handle(&mut self, request: Request, events: &mut Vec<Event>) {
        match request {
            Request::Command(command) => {
                // A path the machine takes starts the travelled path anew, a cycle ends it; refused, neither.
                let (path, cycle) = (matches!(command, Command::Trajectory(_)), matches!(command, Command::StartCycle));
                match self.sim.command(command) {
                    Err(e) => events.push(Event::Refused(e.key, String::new())),
                    Ok(()) if path || cycle => {
                        self.start_trail(if path { self.sim.tool_point() } else { None }, events)
                    }
                    // Another command ends the path only if it ends the trajectory (drives off). A stop after the
                    // segment or an enable is taken while it runs and leaves it running.
                    Ok(()) => self.recording &= self.sim.control.busy(),
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
            Request::Pause(_) | Request::Speed(_) | Request::Exaggeration(_) | Request::Mode(_) | Request::Quit => {}
        }
    }

    fn start_trail(&mut self, at: Option<[f64; 3]>, events: &mut Vec<Event>) {
        // Nothing to end where nothing is recorded: a parameter edit on a machine that runs no path.
        if at.is_some() || self.trail.is_some() {
            events.push(Event::TrailStart(at));
            let first = at.map(|at| self.trail_point(at));
            self.trail_restarted = true;
            self.trail_points.clear();
            self.trail_points.extend(first);
        }
        self.trail = at;
        self.recording = at.is_some();
    }

    /// A point of the travelled path, with where the plan has the tool and the tool's speed now.
    /// shortcut: the speed is the length of the tool's speed vector, which mixes units on a machine with a turning
    /// axis; upgrade to the speed in the scene when one has such an axis.
    fn trail_point(&self, at: [f64; 3]) -> TrailPoint {
        let axes = self.sim.machine.axes().len();
        let (mut pos, mut vel) = (vec![0.0; axes], vec![0.0; axes]);
        self.sim.machine.tool_state(&self.sim.state, &mut pos, &mut vel);
        // Against the setpoint the drives follow, after input shaping, as the cycle analysis measures tracking: the
        // plan before shaping runs ahead of it by the shaper's delay on purpose, which is not an error.
        let planned = self.sim.machine.scene_point(&self.sim.control.set_pos).unwrap_or(at);
        TrailPoint { at, planned, speed: vel.iter().map(|v| v * v).sum::<f64>().sqrt() }
    }

    /// Runs whole base steps for `seconds` of simulated time and reports the samples.
    pub fn advance(&mut self, seconds: f64, events: &mut Vec<Event>) {
        let mut travelled = Vec::new();
        for _ in 0..(seconds / self.sim.dt).round() as u64 {
            self.sim.step();
            if let (true, Some(last), Some(tool)) = (self.recording, self.trail, self.sim.tool_point())
                && (0..3).map(|i| (tool[i] - last[i]).powi(2)).sum::<f64>() > 1e-8
            {
                travelled.push(tool);
                let point = self.trail_point(tool);
                self.trail_points.push(point);
                self.trail = Some(tool);
            }
            // The trajectory is through: the path ends with it.
            self.recording &= self.sim.control.busy();
            if self.sim.step.is_multiple_of(self.sample_every) {
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
            // A monitor's trip (effects.drive_monitoring): which, and each drive's fault reaction, at once while the
            // axes ramp down or coast; then how far each axis went until the machine stood.
            if let Some(fault) = &self.sim.control.fault {
                let motors = self.sim.machine.motors();
                if !self.fault_told {
                    let detail = format!("{}, {:.3} s", motors[fault.drive].id, fault.time);
                    events.push(Event::Refused("ui-drive-fault", detail));
                    events.push(Event::Refused(fault.monitor.key(), String::new()));
                    for (k, motor) in motors.iter().enumerate() {
                        events.push(Event::Refused(self.sim.control.fault_reaction(k).key(), motor.id.to_string()));
                    }
                    self.fault_told = true;
                }
                if let (Some(stopped), false) = (fault.stopped, self.stop_told) {
                    let axes = self.sim.machine.axes();
                    let distances: Vec<String> = axes
                        .iter()
                        .zip(&fault.stopping)
                        .map(|(axis, d)| format!("{} {:.1} m{}", axis.id, d * 1e3, axis.unit))
                        .collect();
                    events
                        .push(Event::Refused("ui-drive-stopped", format!("{stopped:.3} s; {}", distances.join(", "))));
                    self.stop_told = true;
                }
            } else {
                (self.fault_told, self.stop_told) = (false, false);
            }
        }
        if !travelled.is_empty() {
            events.push(Event::Trail(travelled));
        }
        // Bounded as the path is, for a runner whose points nobody takes (the tests).
        self.trail_points.drain(..self.trail_points.len().saturating_sub(TRAIL));
    }
}

pub struct Session {
    requests: Sender<Request>,
    events: Arc<Mutex<Outbox>>,
    thread: JoinHandle<()>,
    news: Arc<Mutex<ViewNews>>,
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
        let news = Arc::new(Mutex::new(ViewNews::default()));
        let window_news = news.clone();
        if let Ok(mut outbox) = outbox.lock() {
            outbox.push(Event::Limits(runner.sim.limit_violations.clone()));
        }
        let thread = std::thread::spawn(move || {
            let (mut paused, mut speed, mut owed, mut exaggeration) = (false, 1.0, 0.0, 1.0);
            // When the mode started showing; meanwhile the simulation waits.
            let mut mode: Option<Instant> = None;
            let (mut last, mut last_scene) = (Instant::now(), Instant::now() - Duration::from_secs(1));
            let mut batch = Vec::new();
            loop {
                loop {
                    match inbox.try_recv() {
                        // Asked to end, or nobody is left to ask.
                        Ok(Request::Quit) | Err(TryRecvError::Disconnected) => return,
                        Ok(Request::Pause(on)) => paused = on,
                        Ok(Request::Speed(factor)) => speed = factor.clamp(0.0, 100.0),
                        Ok(Request::Exaggeration(factor)) => exaggeration = factor.clamp(1.0, 1.0e4),
                        Ok(Request::Mode(on)) => mode = if on { mode.or(Some(Instant::now())) } else { None },
                        Ok(other) => runner.handle(other, &mut batch),
                        Err(TryRecvError::Empty) => break,
                    }
                }
                // The wall clock only decides how many steps are due; the core never sees it.
                let now = Instant::now();
                if !paused && mode.is_none() {
                    owed += (now - last).as_secs_f64() * speed;
                }
                last = now;
                // A machine too slow for the requested speed falls behind instead of piling up work.
                owed = f64::min(owed, 0.05);
                let whole = (owed / runner.sim.dt).floor() * runner.sim.dt;
                runner.advance(whole, &mut batch);
                owed -= whole;
                if now - last_scene > Duration::from_millis(8) {
                    let swinging =
                        mode.and_then(|since| mode_scene(&runner.sim, exaggeration, since.elapsed().as_secs_f64()));
                    let scene = swinging.unwrap_or_else(|| {
                        let mut scene = Scene { exaggeration, ..Scene::default() };
                        runner.sim.scene(&mut scene);
                        scene.planned = runner.sim.machine.scene_point(runner.sim.control.planned_pos());
                        scene
                    });
                    batch.push(Event::Scene(scene));
                    last_scene = now;
                    if let Ok(mut news) = news.lock() {
                        news.has_mode = runner.sim.machine.mode_shape(runner.sim.control.planned_pos()).is_some();
                    }
                }
                if let Ok(mut news) = news.lock() {
                    if std::mem::take(&mut runner.trail_restarted) {
                        (news.restarted, news.trail) = (true, Vec::new());
                    }
                    news.trail.append(&mut runner.trail_points);
                    let over = news.trail.len().saturating_sub(TRAIL);
                    news.trail.drain(..over);
                }
                // Held only to move the batch over: the interface never makes the physics wait for a frame.
                let Ok(mut outbox) = outbox.lock() else { return };
                batch.drain(..).for_each(|event| outbox.push(event));
                drop(outbox);
                std::thread::sleep(Duration::from_millis(1));
            }
        });
        Ok(Self { requests, events, thread, news: window_news, signals, axes, motors })
    }

    /// The travelled path's points with their speed and plan since the last call, whether the path started anew
    /// before them (`ViewNews`), and whether the machine tells a mode to show. Never waits long.
    pub fn view_news(&self) -> ViewNews {
        let take = |news: &mut ViewNews| ViewNews {
            restarted: std::mem::take(&mut news.restarted),
            trail: std::mem::take(&mut news.trail),
            has_mode: news.has_mode,
        };
        self.news.lock().map(|mut news| take(&mut news)).unwrap_or_default()
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
        let samples: Vec<f64> =
            events.iter().filter_map(|e| if let Event::Sample(s) = e { Some(s[0]) } else { None }).collect();
        assert_eq!(
            (samples.len(), samples[0], samples[BACKLOG - 1]),
            (BACKLOG, (2 * BACKLOG) as f64, (3 * BACKLOG - 1) as f64)
        );
        assert!(samples.windows(2).all(|w| w[1] == w[0] + 1.0), "the samples that are left are out of order");
        assert_eq!(events.iter().filter(|e| matches!(e, Event::Scene(_))).count(), 1);
        assert!(
            matches!(events[0], Event::Refused("first", _))
                && events.iter().any(|e| matches!(e, Event::Configured(Ok(_))))
        );
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
        let times: Vec<f64> =
            events.iter().filter_map(|e| if let Event::Sample(s) = e { Some(s[0]) } else { None }).collect();
        assert!(times.len() <= BACKLOG && events.len() <= BACKLOG + 2, "{} events waited", events.len());
        assert_eq!(events.iter().filter(|e| matches!(e, Event::Scene(_))).count(), 1);
        std::thread::sleep(Duration::from_millis(100));
        let newer: Vec<f64> =
            session.drain().iter().filter_map(|e| if let Event::Sample(s) = e { Some(s[0]) } else { None }).collect();
        // What was read first reaches up to where the next read goes on: nothing newer was dropped.
        assert!(
            !newer.is_empty() && (newer[0] - times[times.len() - 1] - 1e-3).abs() < 1e-9,
            "{:?} then {:?}",
            times.last(),
            newer.first()
        );
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
