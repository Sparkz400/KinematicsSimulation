//! PLC link: the simulator as an OPC UA server. A PLC (or any client) sends targets and setpoints
//! and reads back every signal. The servo loops stay inside the simulator; OPC UA is far too slow
//! to close them.
//!
//! The variables are generated from the simulation's signal list, never written per machine:
//!
//! | node (string id in the simulator's namespace) | type    | access | meaning                                   |
//! | --------------------------------------------- | ------- | ------ | ----------------------------------------- |
//! | `signal.<signal id>`                          | Double  | read   | every signal of the simulation            |
//! | `command.target.<axis id>`                    | Double  | write  | target of the next point-to-point move    |
//! | `command.move`                                | Boolean | write  | true starts the move to the targets       |
//! | `command.setpoint.<axis id>`                  | Double  | write  | streamed position setpoint, interpolated  |
//! | `command.cycle`                               | Boolean | write  | true starts the cycle, false stops it     |
//! | `command.disable`                             | Boolean | write  | true switches the drives off (brakes on)  |
//! | `command.enable`                              | Boolean | write  | true switches the drives on again         |
//! | `command.path`                                | String  | write  | a path, in the text of a path file        |
//! | `command.path_time_optimal`                   | Boolean | write  | plan the path as fast as the motors allow |
//! | `command.run_path`                            | Boolean | write  | true plans `command.path` and runs it     |
//! | `status.busy`                                 | Boolean | read   | a move, path or the cycle is under way    |
//! | `status.refused`                              | String  | read   | key of the last refused command, or empty |
//!
//! An axis the PLC does not write stays where it is: `command.target.<axis id>` reads the planned position
//! until it is written, and again after the move it was written for; `command.setpoint.<axis id>` reads the
//! setpoint the controller follows, which is the planned position whenever no setpoint is on its way. So a
//! write to one axis never carries an old value for the others.
//!
//! The path is planned from where the machine stands when `command.run_path` arrives, as
//! `Sim::plan_path` plans it (format: see `sim_components::path`). A path that cannot be read is refused with
//! `opcua-error-path-line`; one that cannot be planned or run with the key the planner or the controller gives.
//!
//! `status.refused` keeps a refusal until a command of the same kind (move, setpoint, cycle start, ...) is
//! accepted. Accepted commands of another kind, such as a stream of setpoints, do not wipe it.
//!
//! Next to these, every axis has PLCopen motion function blocks as handshake variables (see `plcopen` and
//! docs/plc.md), in the folder `axis.<axis id>`:
//!
//! | node                                     | type    | access | meaning                                           |
//! | ---------------------------------------- | ------- | ------ | ------------------------------------------------- |
//! | `axis.<id>.Power.Enable`                 | Boolean | write  | MC_Power: true powers the axis                    |
//! | `axis.<id>.Power.Status`, `.Valid`       | Boolean | read   | drives on; on and without error                   |
//! | `axis.<id>.Power.Error`, `.ErrorID`      | Boolean, UInt16 | read | the axis error while powered               |
//! | `axis.<id>.<FB>.Execute`                 | Boolean | write  | rising edge starts FB = Home, MoveAbsolute, MoveRelative, Stop, Reset |
//! | `axis.<id>.MoveAbsolute.Position`, `.Velocity`, `.Acceleration`, `.Deceleration`, `.Jerk` | Double | write | inputs, taken on the rising edge |
//! | `axis.<id>.MoveRelative.Distance`, ...   | Double  | write  | as for MoveAbsolute                               |
//! | `axis.<id>.Stop.Deceleration`            | Double  | write  | braking deceleration of MC_Stop                   |
//! | `axis.<id>.<FB>.Done`, `.Busy`, `.Active`, `.CommandAborted`, `.Error` | Boolean | read | outputs     |
//! | `axis.<id>.<FB>.ErrorID`                 | UInt16  | read   | why the block failed, 0 without error             |
//! | `axis.<id>.State`, `.StateID`            | String, Int32 | read | PLCopen state (Disabled 1, Standstill 2, ErrorStop 3, Stopping 4, Homing 5, DiscreteMotion 6) |
//! | `axis.<id>.ActualPosition`, `.ActualVelocity` | Double | read | from the simulated axis                      |
//! | `axis.<id>.IsHomed`, `.Error`, `.ErrorID` | Boolean, Boolean, UInt16 | read | homed; the axis error (ErrorStop) |
//!
//! The loop's real-time pacing is measured from the wall-clock times of its ticks, since the start or the last
//! `command.reset_pacing`:
//!
//! | node                         | type    | access | meaning                                                   |
//! | ---------------------------- | ------- | ------ | --------------------------------------------------------- |
//! | `status.cycle_s`             | Double  | read   | the loop's nominal cycle                                  |
//! | `status.cycle_period_s`      | Double  | read   | mean measured time between ticks                          |
//! | `status.jitter_max_s`        | Double  | read   | largest deviation of a tick period from the cycle         |
//! | `status.jitter_rms_s`        | Double  | read   | root mean square of that deviation                        |
//! | `status.overruns`            | UInt64  | read   | ticks that came more than a whole cycle late              |
//! | `status.cycles`              | UInt64  | read   | ticks measured                                            |
//! | `status.realtime_factor`     | Double  | read   | simulated time over wall-clock time                       |
//! | `status.steps`               | UInt64  | read   | simulation steps since the start                          |
//! | `command.reset_pacing`       | Boolean | write  | true starts the measurement anew                          |
//!
//! The clients' cycle, from the arrival times of their Read and Write requests (`ClientTiming`), reset with the pacing:
//!
//! | node                                                   | type    | access | meaning                                |
//! | ------------------------------------------------------ | ------- | ------ | -------------------------------------- |
//! | `status.client_writes`, `status.client_reads`          | UInt64  | read   | intervals measured                     |
//! | `status.client_write_mean_s`, `status.client_read_mean_s` | Double | read  | mean interval                          |
//! | `status.client_write_std_s`, `status.client_read_std_s`   | Double | read  | standard deviation of the intervals    |
//! | `status.client_write_max_s`, `status.client_read_max_s`   | Double | read  | largest interval                       |
//!
//! Endpoints, certificates and users: see `Security`. docs/plc.md has the IEC 61131-3 type of every node.

pub mod plcopen;

use opcua::crypto::SecurityPolicy;
use opcua::server::address_space::{AccessLevel, AddressSpace, VariableBuilder};
use opcua::server::diagnostics::NamespaceMetadata;
use opcua::server::node_manager::memory::{
    InMemoryNodeManager, InMemoryNodeManagerBuilder, InMemoryNodeManagerImpl, InMemoryNodeManagerImplBuilder,
    SimpleNodeManagerBuilder, SimpleNodeManagerImpl,
};
use opcua::server::node_manager::{
    MonitoredItemRef, MonitoredItemUpdateRef, ParsedReadValueId, RequestContext, ServerContext, WriteNode,
};
use opcua::server::{ANONYMOUS_USER_TOKEN_ID, CreateMonitoredItem, ServerBuilder, ServerHandle, ServerUserToken};
use opcua::sync::RwLock;
use opcua::types::{
    DataTypeId, DataValue, MessageSecurityMode, MonitoringMode, NodeId, ObjectId, StatusCode, TimestampsToReturn,
    Variant,
};
use plcopen::{Axis, ERR_CRASH, ERR_DRIVES_OFF, ERR_REFUSED, FBS};
use sim_components::{Command, Sim};
use sim_core::{Config, MachineEntry, Translations};
use std::path::Path;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use tokio::net::TcpListener;

pub const NAMESPACE: &str = "urn:kinematics-simulation";

/// (language code, translation file contents) for the link's own messages.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

/// A failure to start, as a translation key and a detail.
#[derive(Debug)]
pub struct LinkError(pub &'static str, pub String);

/// What a client wrote.
enum Input {
    Target(usize, f64),
    Move,
    Setpoint(usize, f64),
    Cycle(bool),
    Disable,
    Enable,
    Path(String),
    PathTimeOptimal(bool),
    RunPath,
    /// A PLCopen input of an axis: `Power.Enable`, `<FB>.Execute`, or a number such as `MoveAbsolute.Position`.
    AxisFlag(usize, String, bool),
    AxisNumber(usize, String, f64),
    ResetPacing,
}

/// The loop's real-time pacing, measured from the wall-clock times of its ticks since the last reset.
#[derive(Debug, Clone, Default)]
pub struct Pacing {
    /// The nominal cycle.
    pub cycle_s: f64,
    pub cycles: u64,
    pub wall_s: f64,
    pub sim_s: f64,
    jitter_square_sum: f64,
    pub jitter_max_s: f64,
    /// Ticks that came more than a whole cycle late.
    pub overruns: u64,
}

impl Pacing {
    fn record(&mut self, period: f64, simulated: f64) {
        let jitter = period - self.cycle_s;
        self.cycles += 1;
        self.wall_s += period;
        self.sim_s += simulated;
        self.jitter_square_sum += jitter * jitter;
        self.jitter_max_s = self.jitter_max_s.max(jitter.abs());
        self.overruns += u64::from(period > 2.0 * self.cycle_s);
    }

    pub fn period_s(&self) -> f64 {
        self.wall_s / self.cycles.max(1) as f64
    }

    pub fn jitter_rms_s(&self) -> f64 {
        (self.jitter_square_sum / self.cycles.max(1) as f64).sqrt()
    }

    pub fn realtime_factor(&self) -> f64 {
        if self.wall_s > 0.0 { self.sim_s / self.wall_s } else { 0.0 }
    }
}

/// The time between consecutive requests of one kind (Read or Write) from the clients, since the last reset.
#[derive(Debug, Clone, Default)]
pub struct Intervals {
    /// Intervals measured: one fewer than the requests.
    pub count: u64,
    sum_s: f64,
    square_sum_s2: f64,
    pub max_s: f64,
    last: Option<Instant>,
}

impl Intervals {
    /// A request arrived at `at`.
    pub fn record(&mut self, at: Instant) {
        if let Some(last) = self.last.replace(at) {
            let interval = at.saturating_duration_since(last).as_secs_f64();
            self.count += 1;
            self.sum_s += interval;
            self.square_sum_s2 += interval * interval;
            self.max_s = self.max_s.max(interval);
        }
    }

    pub fn mean_s(&self) -> f64 {
        self.sum_s / self.count.max(1) as f64
    }

    /// Standard deviation of the intervals around their mean (of the population: divided by the count).
    pub fn std_s(&self) -> f64 {
        (self.square_sum_s2 / self.count.max(1) as f64 - self.mean_s().powi(2)).max(0.0).sqrt()
    }
}

/// How regularly the clients read and write the simulator's variables: a PLC that reads and writes once a task cycle
/// shows its cycle and jitter here. One Read or Write request counts once, however many variables it holds; reads of
/// other attributes than the value (browsing) and subscriptions do not count.
#[derive(Debug, Clone, Default)]
pub struct ClientTiming {
    pub reads: Intervals,
    pub writes: Intervals,
}

/// The simple in-memory node manager, with the arrival of every Read and Write request timed.
pub struct Timed {
    simple: SimpleNodeManagerImpl,
    clients: Arc<Mutex<ClientTiming>>,
}

#[async_trait::async_trait]
impl InMemoryNodeManagerImpl for Timed {
    async fn init(&self, address_space: &mut AddressSpace, context: ServerContext) {
        self.simple.init(address_space, context).await
    }

    fn name(&self) -> &str {
        self.simple.name()
    }

    fn namespaces(&self) -> Vec<NamespaceMetadata> {
        self.simple.namespaces()
    }

    async fn read_values(
        &self,
        context: &RequestContext,
        address_space: &RwLock<AddressSpace>,
        nodes: &[&ParsedReadValueId],
        max_age: f64,
        timestamps_to_return: TimestampsToReturn,
    ) -> Vec<DataValue> {
        let arrived = Instant::now();
        let values = self.simple.read_values(context, address_space, nodes, max_age, timestamps_to_return).await;
        // Counted once answered: a read of the measurement is not in its own answer.
        self.clients.lock().unwrap().reads.record(arrived);
        values
    }

    async fn write(
        &self,
        context: &RequestContext,
        address_space: &RwLock<AddressSpace>,
        nodes_to_write: &mut [&mut WriteNode],
    ) -> Result<(), StatusCode> {
        // Before the write is carried out: `command.reset_pacing` clears what it is counted in.
        self.clients.lock().unwrap().writes.record(Instant::now());
        self.simple.write(context, address_space, nodes_to_write).await
    }

    async fn create_value_monitored_items(
        &self,
        context: &RequestContext,
        address_space: &RwLock<AddressSpace>,
        items: &mut [&mut &mut CreateMonitoredItem],
    ) {
        self.simple.create_value_monitored_items(context, address_space, items).await
    }

    async fn set_monitoring_mode(&self, context: &RequestContext, mode: MonitoringMode, items: &[&MonitoredItemRef]) {
        self.simple.set_monitoring_mode(context, mode, items).await
    }

    async fn modify_monitored_items(&self, context: &RequestContext, items: &[&MonitoredItemUpdateRef]) {
        self.simple.modify_monitored_items(context, items).await
    }

    async fn delete_monitored_items(&self, context: &RequestContext, items: &[&MonitoredItemRef]) {
        self.simple.delete_monitored_items(context, items).await
    }
}

/// How the server lets clients in. It always offers three endpoints at the same URL: security policy `None`, and
/// `Basic256Sha256` with `Sign` and with `SignAndEncrypt`. Its application instance certificate (self-signed, valid for
/// a year) is created in the pki folder on the first start: `own/cert.der` and `private/private.pem`.
#[derive(Debug, Clone, Default)]
pub struct Security {
    /// Accept every client certificate and store it in `<pki>/trusted` (for commissioning). Off, a client certificate
    /// that is not in `<pki>/trusted` is refused and stored in `<pki>/rejected`, to be moved to `trusted` by hand.
    pub trust_all: bool,
    /// The user name and password a client must give; `None` lets clients in anonymously. With a user the endpoint
    /// without security is not offered, so that the password never travels in clear.
    pub user: Option<(String, String)>,
}

pub struct Link {
    pub handle: ServerHandle,
    /// Index of the simulator's namespace on this server.
    pub namespace: u16,
    pub pacing: Arc<Mutex<Pacing>>,
    pub clients: Arc<Mutex<ClientTiming>>,
}

impl Drop for Link {
    fn drop(&mut self) {
        self.handle.cancel();
    }
}

fn number(value: &DataValue) -> Option<f64> {
    match value.value.as_ref()? {
        Variant::Double(v) => Some(*v),
        Variant::Float(v) => Some(*v as f64),
        Variant::Int16(v) => Some(*v as f64),
        Variant::Int32(v) => Some(*v as f64),
        Variant::Int64(v) => Some(*v as f64),
        _ => None,
    }
}

fn text(value: &DataValue) -> Option<String> {
    if let Some(Variant::String(s)) = &value.value { Some(s.as_ref().to_string()) } else { None }
}

fn flag(value: &DataValue) -> Option<bool> {
    if let Some(Variant::Boolean(on)) = value.value { Some(on) } else { None }
}

/// Starts the server on `listener` and the simulation behind it, in real time, in ticks of `cycle`.
/// `labels` names the variables for people browsing the server; `pki_dir` holds the server's keys.
/// Anonymous clients, and only client certificates already trusted: `Security::default()`.
pub async fn serve(
    entry: &'static MachineEntry,
    cfg: &Config,
    labels: &Translations,
    listener: TcpListener,
    pki_dir: &Path,
    cycle: Duration,
) -> Result<Link, LinkError> {
    serve_secured(entry, cfg, labels, listener, pki_dir, cycle, &Security::default()).await
}

/// `serve`, letting clients in as `security` says.
pub async fn serve_secured(
    entry: &'static MachineEntry,
    cfg: &Config,
    labels: &Translations,
    listener: TcpListener,
    pki_dir: &Path,
    cycle: Duration,
    security: &Security,
) -> Result<Link, LinkError> {
    let sim = Sim::new(entry, cfg).map_err(|e| LinkError(e.key(), e.detail().into()))?;
    // The PLCopen axes stream one setpoint per setpoint interval, counted in simulation steps.
    let interval = cfg.f64("profile.setpoint_interval_s").map_err(|e| LinkError(e.key(), e.detail().into()))?;
    let stream_steps = ((interval / sim.dt).round() as u64).max(1);
    let plc: Vec<Axis> =
        sim.machine.axes().iter().zip(&sim.control.set_pos).map(|(info, home)| Axis::new(info, *home)).collect();
    let address = listener.local_addr().map_err(|e| LinkError("opcua-error-start", e.to_string()))?;
    let clients = Arc::new(Mutex::new(ClientTiming::default()));
    let timed = {
        let clients = clients.clone();
        move |context: ServerContext, space: &mut AddressSpace| Timed {
            simple: SimpleNodeManagerBuilder::new(
                NamespaceMetadata { namespace_uri: NAMESPACE.to_owned(), ..Default::default() },
                "simulator",
            )
            .build(context, space),
            clients,
        }
    };
    let user_tokens: &[&str] = if security.user.is_some() { &["user"] } else { &[ANONYMOUS_USER_TOKEN_ID] };
    let mut builder = ServerBuilder::new();
    for (id, policy, mode) in [
        ("none", SecurityPolicy::None, MessageSecurityMode::None),
        ("basic256sha256_sign", SecurityPolicy::Basic256Sha256, MessageSecurityMode::Sign),
        ("basic256sha256_sign_encrypt", SecurityPolicy::Basic256Sha256, MessageSecurityMode::SignAndEncrypt),
    ] {
        // With a password, no endpoint without security: async-opcua takes a password sent in clear (an empty
        // encryption algorithm) even where the policy asks for it encrypted.
        if security.user.is_some() && policy == SecurityPolicy::None {
            continue;
        }
        builder = builder.add_endpoint(id, ("/", policy, mode, user_tokens));
    }
    if let Some((user, password)) = &security.user {
        builder = builder.add_user_token("user", ServerUserToken::user_pass(user.as_str(), password.as_str()));
    }
    let (server, handle) = builder
        .application_name("sim-opcua")
        // Not the namespace URI: the application URI becomes namespace 1, and the same URI twice
        // would make the variables' namespace index ambiguous.
        .application_uri(format!("{NAMESPACE}:server"))
        .product_uri(format!("{NAMESPACE}:server"))
        .create_sample_keypair(true)
        .pki_dir(pki_dir)
        .trust_client_certs(security.trust_all)
        .host(address.ip().to_string())
        .port(address.port())
        .discovery_urls(vec![format!("opc.tcp://{address}/")])
        .default_endpoint(if security.user.is_some() { "basic256sha256_sign_encrypt" } else { "none" })
        .with_node_manager(InMemoryNodeManagerBuilder::new(timed))
        .build()
        .map_err(|e| LinkError("opcua-error-start", e.to_string()))?;
    let manager = handle
        .node_managers()
        .get_of_type::<InMemoryNodeManager<Timed>>()
        .ok_or(LinkError("opcua-error-start", String::new()))?;
    let namespace = handle.get_namespace_index(NAMESPACE).ok_or(LinkError("opcua-error-start", String::new()))?;
    let node = |name: String| NodeId::new(namespace, name);

    // The address space, generated from the signal list and the machine's axes.
    let signals = sim.signal_infos();
    let axes: Vec<&'static str> = sim.machine.axes().iter().map(|a| a.id).collect();
    let axis_keys: Vec<&'static str> = sim.machine.axes().iter().map(|a| a.key).collect();
    let (inputs, inbox) = channel::<Input>();
    {
        let mut space = manager.address_space().write();
        let folder = node("simulator".into());
        space.add_folder(&folder, "simulator", labels.tr(entry.key), &NodeId::from(ObjectId::ObjectsFolder));
        let add_to = |space: &mut opcua::server::address_space::AddressSpace,
                      parent: &NodeId,
                      id: &NodeId,
                      name: &str,
                      label: String,
                      kind: DataTypeId,
                      start: Variant,
                      writable: bool| {
            let variable = VariableBuilder::new(id, name, label).data_type(kind).value(start).organized_by(parent);
            let access = if writable {
                AccessLevel::CURRENT_READ | AccessLevel::CURRENT_WRITE
            } else {
                AccessLevel::CURRENT_READ
            };
            variable.access_level(access).user_access_level(access).insert(space);
        };
        for (i, axis) in plc.iter().enumerate() {
            let parent = node(format!("axis.{}", axes[i]));
            space.add_folder(
                &parent,
                format!("axis.{}", axes[i]).as_str(),
                format!("{} · {}", labels.tr(axis_keys[i]), labels.tr("opcua-plcopen-axis")),
                &folder,
            );
            for (name, start, writable) in axis.variables(true, (0.0, 0.0)) {
                let kind = match start {
                    Variant::Boolean(_) => DataTypeId::Boolean,
                    Variant::UInt16(_) => DataTypeId::UInt16,
                    Variant::Int32(_) => DataTypeId::Int32,
                    Variant::String(_) => DataTypeId::String,
                    _ => DataTypeId::Double,
                };
                let label = format!("{} · {name}", labels.tr(axis_keys[i]));
                add_to(
                    &mut space,
                    &parent,
                    &node(format!("axis.{}.{name}", axes[i])),
                    &name,
                    label,
                    kind,
                    start,
                    writable,
                );
            }
        }
        let mut add = |id: &NodeId, name: &str, label: String, kind: DataTypeId, start: Variant, writable: bool| {
            add_to(&mut space, &folder, id, name, label, kind, start, writable)
        };
        for signal in &signals {
            let label =
                format!("{} · {} [{}]", labels.tr(&signal.owner_key), labels.tr(&signal.quantity_key), signal.unit);
            add(&node(format!("signal.{}", signal.id)), &signal.id, label, DataTypeId::Double, 0.0.into(), false);
        }
        for (i, axis) in axes.iter().enumerate() {
            for (kind, key) in [("target", "opcua-target"), ("setpoint", "opcua-setpoint")] {
                let name = format!("command.{kind}.{axis}");
                let label = format!("{} · {}", labels.tr(axis_keys[i]), labels.tr(key));
                add(&node(name.clone()), &name, label, DataTypeId::Double, sim.control.set_pos[i].into(), true);
            }
        }
        for (name, key, kind, start, writable) in [
            ("command.move", "opcua-move", DataTypeId::Boolean, Variant::from(false), true),
            ("command.cycle", "opcua-cycle", DataTypeId::Boolean, Variant::from(false), true),
            ("command.disable", "opcua-disable", DataTypeId::Boolean, Variant::from(false), true),
            ("command.enable", "opcua-enable", DataTypeId::Boolean, Variant::from(false), true),
            ("command.path", "opcua-path", DataTypeId::String, Variant::from(""), true),
            ("command.path_time_optimal", "opcua-path-time-optimal", DataTypeId::Boolean, Variant::from(false), true),
            ("command.run_path", "opcua-run-path", DataTypeId::Boolean, Variant::from(false), true),
            ("status.busy", "opcua-busy", DataTypeId::Boolean, Variant::from(false), false),
            ("status.refused", "opcua-refused", DataTypeId::String, Variant::from(""), false),
            ("status.cycle_s", "opcua-cycle-s", DataTypeId::Double, Variant::from(cycle.as_secs_f64()), false),
            ("status.cycle_period_s", "opcua-cycle-period", DataTypeId::Double, Variant::from(0.0), false),
            ("status.jitter_max_s", "opcua-jitter-max", DataTypeId::Double, Variant::from(0.0), false),
            ("status.jitter_rms_s", "opcua-jitter-rms", DataTypeId::Double, Variant::from(0.0), false),
            ("status.overruns", "opcua-overruns", DataTypeId::UInt64, Variant::from(0u64), false),
            ("status.cycles", "opcua-cycles", DataTypeId::UInt64, Variant::from(0u64), false),
            ("status.realtime_factor", "opcua-realtime-factor", DataTypeId::Double, Variant::from(0.0), false),
            ("status.steps", "opcua-steps", DataTypeId::UInt64, Variant::from(0u64), false),
            ("command.reset_pacing", "opcua-reset-pacing", DataTypeId::Boolean, Variant::from(false), true),
            ("status.client_writes", "opcua-client-writes", DataTypeId::UInt64, Variant::from(0u64), false),
            ("status.client_write_mean_s", "opcua-client-write-mean", DataTypeId::Double, Variant::from(0.0), false),
            ("status.client_write_std_s", "opcua-client-write-std", DataTypeId::Double, Variant::from(0.0), false),
            ("status.client_write_max_s", "opcua-client-write-max", DataTypeId::Double, Variant::from(0.0), false),
            ("status.client_reads", "opcua-client-reads", DataTypeId::UInt64, Variant::from(0u64), false),
            ("status.client_read_mean_s", "opcua-client-read-mean", DataTypeId::Double, Variant::from(0.0), false),
            ("status.client_read_std_s", "opcua-client-read-std", DataTypeId::Double, Variant::from(0.0), false),
            ("status.client_read_max_s", "opcua-client-read-max", DataTypeId::Double, Variant::from(0.0), false),
        ] {
            add(&node(name.into()), name, labels.tr(key).into(), kind, start, writable);
        }
    }
    // Writes arrive on the server's tasks; they are queued for the simulation loop.
    let on_write =
        |name: String, convert: fn(&DataValue, usize) -> Option<Input>, index: usize, inputs: Sender<Input>| {
            manager.inner().simple.add_write_callback(node(name), move |value, _| match convert(&value, index) {
                Some(input) => inputs.send(input).map_or(StatusCode::BadShutdown, |_| StatusCode::Good),
                None => StatusCode::BadTypeMismatch,
            });
        };
    for (i, axis) in axes.iter().enumerate() {
        on_write(format!("command.target.{axis}"), |v, i| Some(Input::Target(i, number(v)?)), i, inputs.clone());
        on_write(format!("command.setpoint.{axis}"), |v, i| Some(Input::Setpoint(i, number(v)?)), i, inputs.clone());
    }
    on_write("command.move".into(), |v, _| flag(v)?.then_some(Input::Move), 0, inputs.clone());
    on_write("command.cycle".into(), |v, _| Some(Input::Cycle(flag(v)?)), 0, inputs.clone());
    on_write("command.disable".into(), |v, _| flag(v)?.then_some(Input::Disable), 0, inputs.clone());
    on_write("command.enable".into(), |v, _| flag(v)?.then_some(Input::Enable), 0, inputs.clone());
    on_write("command.path".into(), |v, _| Some(Input::Path(text(v)?)), 0, inputs.clone());
    on_write("command.path_time_optimal".into(), |v, _| Some(Input::PathTimeOptimal(flag(v)?)), 0, inputs.clone());
    on_write("command.run_path".into(), |v, _| flag(v)?.then_some(Input::RunPath), 0, inputs.clone());
    // The client timing starts anew at once, in this write: the loop would count the writes that come before its tick.
    let (reset, timing) = (inputs.clone(), clients.clone());
    manager.inner().simple.add_write_callback(node("command.reset_pacing".into()), move |value, _| {
        match flag(&value) {
            Some(on) => {
                if on {
                    *timing.lock().unwrap() = ClientTiming::default();
                    return reset.send(Input::ResetPacing).map_or(StatusCode::BadShutdown, |_| StatusCode::Good);
                }
                StatusCode::Good
            }
            None => StatusCode::BadTypeMismatch,
        }
    });
    for (i, axis) in plc.iter().enumerate() {
        for (name, start, writable) in axis.variables(true, (0.0, 0.0)) {
            if !writable {
                continue;
            }
            let inputs = inputs.clone();
            let is_flag = matches!(start, Variant::Boolean(_));
            manager.inner().simple.add_write_callback(node(format!("axis.{}.{name}", axes[i])), move |value, _| {
                let input = if is_flag {
                    flag(&value).map(|on| Input::AxisFlag(i, name.clone(), on))
                } else {
                    number(&value).map(|v| Input::AxisNumber(i, name.clone(), v))
                };
                match input {
                    Some(input) => inputs.send(input).map_or(StatusCode::BadShutdown, |_| StatusCode::Good),
                    None => StatusCode::BadTypeMismatch,
                }
            });
        }
    }

    let pacing = Arc::new(Mutex::new(Pacing { cycle_s: cycle.as_secs_f64(), ..Default::default() }));
    tokio::spawn(server.run_with(listener));
    tokio::spawn(simulate(SimLoop {
        sim,
        inbox,
        handle: handle.clone(),
        namespace,
        axes,
        plc,
        stream_steps,
        interval,
        cycle,
        pacing: pacing.clone(),
        clients: clients.clone(),
    }));
    Ok(Link { handle, namespace, pacing, clients })
}

struct SimLoop {
    sim: Sim,
    inbox: Receiver<Input>,
    handle: ServerHandle,
    namespace: u16,
    axes: Vec<&'static str>,
    plc: Vec<Axis>,
    /// Simulation steps per setpoint interval, and the interval in seconds.
    stream_steps: u64,
    interval: f64,
    cycle: Duration,
    pacing: Arc<Mutex<Pacing>>,
    clients: Arc<Mutex<ClientTiming>>,
}

impl SimLoop {
    /// One setpoint interval of the PLCopen axes: each runs its profile, and the positions of those that moved go to
    /// the controller as one setpoint. A setpoint the controller refuses is an axis error for each of them.
    fn stream(&mut self) -> Option<&'static str> {
        let setpoint = self.sim.control.external_setpoint().to_vec();
        let moving: Vec<bool> = self.plc.iter().map(Axis::moving).collect();
        for (axis, at) in self.plc.iter_mut().zip(&setpoint) {
            axis.advance(self.interval, *at);
        }
        // Drives that went off in this tick (a trip, a crash) are the axes' error, ErrorID 7 or 8 after the steps, not
        // the refused setpoint.
        if !moving.contains(&true) || !self.sim.control.enabled() {
            return None;
        }
        let err = self.sim.command(Command::Setpoint(self.plc.iter().map(|a| a.pos).collect())).err()?;
        for (axis, _) in self.plc.iter_mut().zip(&moving).filter(|(_, m)| **m) {
            axis.fault(ERR_REFUSED);
        }
        Some(err.key)
    }
}

/// Steps the simulation in real time and publishes every signal once a cycle (a hundred times a second by default).
async fn simulate(mut s: SimLoop) {
    let Some(manager) = s.handle.node_managers().get_of_type::<InMemoryNodeManager<Timed>>() else { return };
    let namespace = s.namespace;
    let node = |name: String| NodeId::new(namespace, name);
    let signal_nodes: Vec<NodeId> = s.sim.signal_infos().iter().map(|i| node(format!("signal.{}", i.id))).collect();
    let (mut target, mut setpoint) = (s.sim.control.set_pos.clone(), s.sim.control.set_pos.clone());
    // Targets the PLC has written for the move to come.
    let mut target_written = vec![false; target.len()];
    // The last refusal: the kind of command and why.
    let mut refused: Option<(std::mem::Discriminant<Command>, &'static str)> = None;
    // The path the PLC has written, and whether to plan it as fast as the motors allow.
    let (mut path, mut time_optimal) = (String::new(), false);
    let mut sample = Vec::new();
    let actual: Vec<(Option<usize>, Option<usize>)> = s
        .axes
        .iter()
        .map(|id| {
            (s.sim.signal_index(&format!("axis.{id}.position")), s.sim.signal_index(&format!("axis.{id}.velocity")))
        })
        .collect();
    let axis_nodes: Vec<Vec<NodeId>> = s
        .plc
        .iter()
        .zip(&s.axes)
        .map(|(axis, id)| {
            axis.variables(true, (0.0, 0.0)).into_iter().map(|(name, _, _)| node(format!("axis.{id}.{name}"))).collect()
        })
        .collect();
    // Whether a PLCopen axis was powered at the last tick: switched off all together, the drives go off.
    let mut was_powered = false;
    let mut last = Instant::now();
    let mut next = last;
    let mut owed = 0.0;
    while !s.handle.token().is_cancelled() {
        // Ticks on a fixed grid of deadlines, so that the mean period is the cycle. A tick more than a cycle late is not
        // made up for with a burst of ticks: the steps owed are counted from the wall clock anyway. (A dedicated thread
        // with `std::thread::sleep` was tried: on a loaded Windows host it came late more often than the runtime's timer.)
        next += s.cycle;
        let now = Instant::now();
        if next > now {
            tokio::time::sleep(next - now).await;
        } else if now - next > s.cycle {
            next = now;
        }
        // In the order they arrived; `None` is a path to run, planned when its turn comes.
        let mut commands = Vec::new();
        let mut new_setpoint = false;
        for input in s.inbox.try_iter() {
            match input {
                Input::Target(i, v) => (target[i], target_written[i]) = (v, true),
                Input::Setpoint(i, v) => {
                    setpoint[i] = v;
                    new_setpoint = true;
                }
                Input::Move => {
                    commands.push(Some(Command::MoveTo(target.clone())));
                    target_written.fill(false);
                }
                Input::Cycle(on) => commands.push(Some(if on { Command::StartCycle } else { Command::Stop })),
                Input::Disable => commands.push(Some(Command::Disable)),
                Input::Enable => commands.push(Some(Command::Enable)),
                Input::Path(text) => path = text,
                Input::PathTimeOptimal(on) => time_optimal = on,
                Input::RunPath => commands.push(None),
                Input::AxisFlag(i, name, on) if name == "Power.Enable" => s.plc[i].power(on),
                Input::AxisFlag(i, name, on) => {
                    if let Some(k) = FBS.iter().position(|fb| name.strip_suffix(".Execute") == Some(*fb)) {
                        s.plc[i].execute(k, on, s.sim.control.external_setpoint()[i], s.sim.control.busy());
                    }
                }
                Input::AxisNumber(i, name, v) => _ = s.plc[i].set_param(&name, v),
                Input::ResetPacing => {
                    let mut pacing = s.pacing.lock().unwrap();
                    *pacing = Pacing { cycle_s: pacing.cycle_s, ..Default::default() };
                }
            }
        }
        // The controller's drives are on while a PLCopen axis is powered and not in ErrorStop (also after MC_Reset),
        // and go off when the last axis is switched off. Without PLCopen axes they are left to the commands above.
        let powered = s.plc.iter().any(|a| a.enable);
        if s.plc.iter().any(|a| a.enable && a.error_id == 0) && !s.sim.control.enabled() {
            commands.push(Some(Command::Enable));
        } else if was_powered && !powered {
            commands.push(Some(Command::Disable));
        }
        was_powered = powered;
        // Axes written together arrive together: one setpoint command per tick.
        if new_setpoint {
            commands.push(Some(Command::Setpoint(setpoint.clone())));
        }
        let trajectory = std::mem::discriminant(&Command::Trajectory(Vec::new()));
        for command in commands {
            // The path written last, also later in the same write, planned from where the commands before
            // left the machine; refused like the trajectory it would become.
            let command = match command {
                Some(command) => command,
                None => match sim_components::path::parse(&path) {
                    Err(_) => {
                        refused = Some((trajectory, "opcua-error-path-line"));
                        continue;
                    }
                    Ok(pieces) => match s.sim.plan_path(&pieces, time_optimal) {
                        Ok(points) => Command::Trajectory(points),
                        Err(e) => {
                            refused = Some((trajectory, e.key));
                            continue;
                        }
                    },
                },
            };
            let kind = std::mem::discriminant(&command);
            match s.sim.command(command) {
                Err(e) => refused = Some((kind, e.key)),
                // Only a command of the same kind settles a refusal: a refused move stays to be read while
                // setpoints stream on.
                Ok(()) if refused.is_some_and(|(refused_kind, _)| refused_kind == kind) => refused = None,
                Ok(()) => {}
            }
        }
        // The wall clock only decides how many steps are due. A slow host falls behind instead of
        // piling up work; only lag beyond two cycles counts as falling behind, so that long cycles keep real time.
        let now = Instant::now();
        let period = (now - last).as_secs_f64();
        owed = f64::min(owed + period, f64::max(0.1, 2.0 * s.cycle.as_secs_f64()));
        last = now;
        let steps_before = s.sim.step;
        while owed >= s.sim.dt {
            if s.sim.step.is_multiple_of(s.stream_steps)
                && let Some(key) = s.stream()
            {
                refused = Some((std::mem::discriminant(&Command::Setpoint(Vec::new())), key));
            }
            s.sim.step();
            owed -= s.sim.dt;
        }
        s.pacing.lock().unwrap().record(period, (s.sim.step - steps_before) as f64 * s.sim.dt);
        // Drives switched off by `command.disable`, a trip or a crash: an error for every powered axis.
        if !s.sim.control.enabled() {
            for axis in s.plc.iter_mut().filter(|a| a.enable && a.error_id == 0) {
                axis.fault(if s.sim.crashed { ERR_CRASH } else { ERR_DRIVES_OFF });
            }
        }

        // The mirrors follow the controller where the PLC has not written: after a move the axes stand
        // elsewhere, and a write to one axis would send the others back to a value from before it.
        for (i, planned) in s.sim.control.planned_pos().iter().enumerate() {
            if !target_written[i] {
                target[i] = *planned;
            }
        }
        setpoint.copy_from_slice(s.sim.control.external_setpoint());

        s.sim.sample(&mut sample);
        let mut values: Vec<(NodeId, DataValue)> =
            signal_nodes.iter().cloned().zip(sample.iter().map(|v| DataValue::new_now(*v))).collect();
        for (i, axis) in s.axes.iter().enumerate() {
            values.push((node(format!("command.target.{axis}")), DataValue::new_now(target[i])));
            values.push((node(format!("command.setpoint.{axis}")), DataValue::new_now(setpoint[i])));
        }
        values.push((node("command.path".into()), DataValue::new_now(path.as_str())));
        values.push((node("command.path_time_optimal".into()), DataValue::new_now(time_optimal)));
        values.push((node("status.busy".into()), DataValue::new_now(s.sim.control.busy())));
        values.push((node("status.refused".into()), DataValue::new_now(refused.map_or("", |(_, key)| key))));
        let pacing = s.pacing.lock().unwrap().clone();
        for (name, value) in [
            ("status.cycle_period_s", Variant::from(pacing.period_s())),
            ("status.jitter_max_s", pacing.jitter_max_s.into()),
            ("status.jitter_rms_s", pacing.jitter_rms_s().into()),
            ("status.overruns", pacing.overruns.into()),
            ("status.cycles", pacing.cycles.into()),
            ("status.realtime_factor", pacing.realtime_factor().into()),
            ("status.steps", s.sim.step.into()),
        ] {
            values.push((node(name.into()), DataValue::new_now(value)));
        }
        let clients = s.clients.lock().unwrap().clone();
        for (name, value) in [
            ("status.client_writes", Variant::from(clients.writes.count)),
            ("status.client_write_mean_s", clients.writes.mean_s().into()),
            ("status.client_write_std_s", clients.writes.std_s().into()),
            ("status.client_write_max_s", clients.writes.max_s.into()),
            ("status.client_reads", clients.reads.count.into()),
            ("status.client_read_mean_s", clients.reads.mean_s().into()),
            ("status.client_read_std_s", clients.reads.std_s().into()),
            ("status.client_read_max_s", clients.reads.max_s.into()),
        ] {
            values.push((node(name.into()), DataValue::new_now(value)));
        }
        let drives_on = s.sim.control.enabled();
        for ((axis, nodes), (position, velocity)) in s.plc.iter().zip(&axis_nodes).zip(&actual) {
            let at = |index: &Option<usize>| index.map_or(f64::NAN, |k| sample[k]);
            for (id, (_, value, _)) in nodes.iter().zip(axis.variables(drives_on, (at(position), at(velocity)))) {
                values.push((id.clone(), DataValue::new_now(value)));
            }
        }
        let _ =
            manager.set_values(s.handle.subscriptions(), values.iter().map(|(id, value)| (id, None, value.clone())));
    }
}

#[cfg(test)]
mod tests {
    use super::Intervals;
    use std::time::{Duration, Instant};

    /// Requests at intervals of 10, 10, 10 and 30 ms: four intervals, mean (30 + 30) / 4 = 15 ms, largest 30 ms;
    /// deviations −5, −5, −5 and +15 ms, so the standard deviation is √((3 · 25 + 225) / 4) = √75 = 8.660 ms. The
    /// first request starts the series and is no interval itself.
    #[test]
    fn intervals_give_count_mean_standard_deviation_and_largest() {
        let mut intervals = Intervals::default();
        let mut at = Instant::now();
        intervals.record(at);
        assert_eq!((intervals.count, intervals.mean_s(), intervals.std_s()), (0, 0.0, 0.0));
        for ms in [10, 10, 10, 30] {
            at += Duration::from_millis(ms);
            intervals.record(at);
        }
        assert_eq!(intervals.count, 4);
        assert!((intervals.mean_s() - 0.015).abs() < 1e-12, "{}", intervals.mean_s());
        assert!((intervals.max_s - 0.030).abs() < 1e-12, "{}", intervals.max_s);
        assert!((intervals.std_s() - 75f64.sqrt() * 1e-3).abs() < 1e-9, "{}", intervals.std_s());
    }

    /// Every key the link shows has a German and an English entry.
    #[test]
    fn every_link_message_exists_in_every_language() {
        let sources = [include_str!("lib.rs"), include_str!("main.rs")].concat();
        let keys: Vec<&str> = sources
            .split('"')
            .filter(|s| s.starts_with("opcua-") && s.len() > 6 && s.chars().all(|c| c.is_ascii_lowercase() || c == '-'))
            .collect();
        assert!(keys.len() >= 10, "{keys:?}");
        for (lang, text) in super::I18N {
            let tr = sim_core::Translations::parse(text);
            for key in &keys {
                assert!(tr.has(key), "no '{lang}' entry for '{key}'");
            }
        }
    }
}
