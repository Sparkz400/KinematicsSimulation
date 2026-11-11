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
//! | `status.busy`                                 | Boolean | read   | a move or the cycle is under way          |
//! | `status.refused`                              | String  | read   | key of the last refused command, or empty |

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
    let (mut refused, mut sample) = ("", Vec::new());
    let mut tick = tokio::time::interval(Duration::from_millis(10));
    let mut last = Instant::now();
    let mut owed = 0.0;
    while !s.handle.token().is_cancelled() {
        tick.tick().await;
        let mut commands = Vec::new();
        let mut new_setpoint = false;
        for input in s.inbox.try_iter() {
            match input {
                Input::Target(i, v) => target[i] = v,
                Input::Setpoint(i, v) => {
                    setpoint[i] = v;
                    new_setpoint = true;
                }
                Input::Move => commands.push(Command::MoveTo(target.clone())),
                Input::Cycle(on) => commands.push(if on { Command::StartCycle } else { Command::Stop }),
            }
        }
        // Axes written together arrive together: one setpoint command per tick.
        if new_setpoint {
            commands.push(Command::Setpoint(setpoint.clone()));
        }
        for command in commands {
            refused = s.sim.command(command).err().map_or("", |e| e.key);
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

        s.sim.sample(&mut sample);
        let mut values: Vec<(NodeId, DataValue)> = signal_nodes.iter().cloned().zip(sample.iter().map(|v| DataValue::new_now(*v))).collect();
        for (i, axis) in s.axes.iter().enumerate() {
            values.push((node(format!("command.target.{axis}")), DataValue::new_now(target[i])));
            values.push((node(format!("command.setpoint.{axis}")), DataValue::new_now(setpoint[i])));
        }
        values.push((node("status.busy".into()), DataValue::new_now(s.sim.control.busy())));
        values.push((node("status.refused".into()), DataValue::new_now(refused)));
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
        assert!(keys.len() >= 8, "{keys:?}");
        for (lang, text) in super::I18N {
            let tr = sim_core::Translations::parse(text);
            for key in &keys {
                assert!(tr.has(key), "no '{lang}' entry for '{key}'");
            }
        }
    }
}
