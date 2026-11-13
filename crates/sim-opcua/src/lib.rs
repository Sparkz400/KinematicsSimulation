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

use opcua::crypto::SecurityPolicy;
use opcua::server::address_space::{AccessLevel, VariableBuilder};
use opcua::server::diagnostics::NamespaceMetadata;
use opcua::server::node_manager::memory::{SimpleNodeManager, simple_node_manager};
use opcua::server::{ANONYMOUS_USER_TOKEN_ID, ServerBuilder, ServerHandle};
use opcua::types::{DataTypeId, DataValue, MessageSecurityMode, NodeId, ObjectId, StatusCode, Variant};
use sim_components::{Command, Sim};
use sim_core::{Config, MachineEntry, Translations};
use std::path::Path;
use std::sync::mpsc::{Receiver, Sender, channel};
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
}

pub struct Link {
    pub handle: ServerHandle,
    /// Index of the simulator's namespace on this server.
    pub namespace: u16,
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

/// Starts the server on `listener` and the simulation behind it, in real time.
/// `labels` names the variables for people browsing the server; `pki_dir` holds the server's keys.
pub async fn serve(entry: &'static MachineEntry, cfg: &Config, labels: &Translations, listener: TcpListener, pki_dir: &Path) -> Result<Link, LinkError> {
    let sim = Sim::new(entry, cfg).map_err(|e| LinkError(e.key(), e.detail().into()))?;
    let address = listener.local_addr().map_err(|e| LinkError("opcua-error-start", e.to_string()))?;
    let (server, handle) = ServerBuilder::new()
        .application_name("sim-opcua")
        // Not the namespace URI: the application URI becomes namespace 1, and the same URI twice
        // would make the variables' namespace index ambiguous.
        .application_uri(format!("{NAMESPACE}:server"))
        .product_uri(format!("{NAMESPACE}:server"))
        .create_sample_keypair(true)
        .pki_dir(pki_dir)
        .host(address.ip().to_string())
        .port(address.port())
        .discovery_urls(vec![format!("opc.tcp://{address}/")])
        .add_endpoint("none", ("/", SecurityPolicy::None, MessageSecurityMode::None, &[ANONYMOUS_USER_TOKEN_ID] as &[&str]))
        .with_node_manager(simple_node_manager(NamespaceMetadata { namespace_uri: NAMESPACE.to_owned(), ..Default::default() }, "simulator"))
        .build()
        .map_err(|e| LinkError("opcua-error-start", e.to_string()))?;
    let manager = handle.node_managers().get_of_type::<SimpleNodeManager>().ok_or(LinkError("opcua-error-start", String::new()))?;
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
        let mut add = |id: &NodeId, name: &str, label: String, kind: DataTypeId, start: Variant, writable: bool| {
            let variable = VariableBuilder::new(id, name, label).data_type(kind).value(start).organized_by(&folder);
            let access = if writable { AccessLevel::CURRENT_READ | AccessLevel::CURRENT_WRITE } else { AccessLevel::CURRENT_READ };
            variable.access_level(access).user_access_level(access).insert(&mut *space);
        };
        for signal in &signals {
            let label = format!("{} · {} [{}]", labels.tr(&signal.owner_key), labels.tr(&signal.quantity_key), signal.unit);
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
        ] {
            add(&node(name.into()), name, labels.tr(key).into(), kind, start, writable);
        }
    }
    // Writes arrive on the server's tasks; they are queued for the simulation loop.
    let on_write = |name: String, convert: fn(&DataValue, usize) -> Option<Input>, index: usize, inputs: Sender<Input>| {
        manager.inner().add_write_callback(node(name), move |value, _| match convert(&value, index) {
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

    tokio::spawn(server.run_with(listener));
    tokio::spawn(simulate(SimLoop { sim, inbox, handle: handle.clone(), namespace, axes }));
    Ok(Link { handle, namespace })
}

struct SimLoop {
    sim: Sim,
    inbox: Receiver<Input>,
    handle: ServerHandle,
    namespace: u16,
    axes: Vec<&'static str>,
}

/// Steps the simulation in real time and publishes every signal a hundred times a second.
async fn simulate(mut s: SimLoop) {
    let Some(manager) = s.handle.node_managers().get_of_type::<SimpleNodeManager>() else { return };
    let node = |name: String| NodeId::new(s.namespace, name);
    let signal_nodes: Vec<NodeId> = s.sim.signal_infos().iter().map(|i| node(format!("signal.{}", i.id))).collect();
    let (mut target, mut setpoint) = (s.sim.control.set_pos.clone(), s.sim.control.set_pos.clone());
    // Targets the PLC has written for the move to come.
    let mut target_written = vec![false; target.len()];
    // The last refusal: the kind of command and why.
    let mut refused: Option<(std::mem::Discriminant<Command>, &'static str)> = None;
    // The path the PLC has written, and whether to plan it as fast as the motors allow.
    let (mut path, mut time_optimal) = (String::new(), false);
    let mut sample = Vec::new();
    let mut tick = tokio::time::interval(Duration::from_millis(10));
    let mut last = Instant::now();
    let mut owed = 0.0;
    while !s.handle.token().is_cancelled() {
        tick.tick().await;
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
            }
        }
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
        // piling up work.
        let now = Instant::now();
        owed = f64::min(owed + (now - last).as_secs_f64(), 0.1);
        last = now;
        while owed >= s.sim.dt {
            s.sim.step();
            owed -= s.sim.dt;
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
        let mut values: Vec<(NodeId, DataValue)> = signal_nodes.iter().cloned().zip(sample.iter().map(|v| DataValue::new_now(*v))).collect();
        for (i, axis) in s.axes.iter().enumerate() {
            values.push((node(format!("command.target.{axis}")), DataValue::new_now(target[i])));
            values.push((node(format!("command.setpoint.{axis}")), DataValue::new_now(setpoint[i])));
        }
        values.push((node("command.path".into()), DataValue::new_now(path.as_str())));
        values.push((node("command.path_time_optimal".into()), DataValue::new_now(time_optimal)));
        values.push((node("status.busy".into()), DataValue::new_now(s.sim.control.busy())));
        values.push((node("status.refused".into()), DataValue::new_now(refused.map_or("", |(_, key)| key))));
        let _ = manager.set_values(s.handle.subscriptions(), values.iter().map(|(id, value)| (id, None, value.clone())));
    }
}

#[cfg(test)]
mod tests {
    /// Every key the link shows has a German and an English entry.
    #[test]
    fn every_link_message_exists_in_every_language() {
        let sources = [include_str!("lib.rs"), include_str!("main.rs")].concat();
        let keys: Vec<&str> = sources.split('"').filter(|s| s.starts_with("opcua-") && s.len() > 6 && s.chars().all(|c| c.is_ascii_lowercase() || c == '-')).collect();
        assert!(keys.len() >= 10, "{keys:?}");
        for (lang, text) in super::I18N {
            let tr = sim_core::Translations::parse(text);
            for key in &keys {
                assert!(tr.has(key), "no '{lang}' entry for '{key}'");
            }
        }
    }
}
