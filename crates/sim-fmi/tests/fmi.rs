//! The FMU's C functions, called as an importing tool calls them, on every registered machine and the machine files
//! of machines/: a move commanded through the setpoint inputs gives bit for bit the signals of a plain `Sim` given the
//! same setpoints, and what the standard forbids is refused.

use sim_components::{Command, Sim, load_config};
use sim_core::MachineEntry;
use sim_fmi::api::*;
use sim_fmi::{Kind, Model};
use sim_machines::MACHINES;
use std::ffi::{CString, c_char};
use std::path::PathBuf;
use std::ptr::null;

/// The FMU's model files unpacked into a folder of their own, as a tool unpacks them; its resources as a URI.
fn unpack(entry: &'static MachineEntry, name: &str) -> (Model, CString) {
    let text = entry.config().to_text().unwrap();
    let model = Model::new(entry, &text, "en").unwrap();
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("fmi-{name}-{}", entry.id));
    for (path, bytes) in model.files() {
        let file = dir.join(path);
        std::fs::create_dir_all(file.parent().unwrap()).unwrap();
        std::fs::write(file, bytes).unwrap();
    }
    // A space in the folder name would come percent-encoded; the URI keeps it plain where it can.
    let uri =
        format!("file:///{}", dir.join("resources").display().to_string().replace('\\', "/").trim_start_matches('/'));
    (model, CString::new(uri).unwrap())
}

/// The machine files of machines/ that the FMU builds from its resources (`resources/machine.toml`).
fn machine_files() -> Vec<&'static MachineEntry> {
    ["h-bot.toml", "corexy.toml", "xyz-portal.toml"]
        .iter()
        .map(|name| {
            sim_machines::load_file(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name))
                .unwrap_or_else(|e| panic!("{e}"))
        })
        .collect()
}

fn instantiate(guid: &str, resources: &CString, kind: i32) -> fmi2Component {
    let (name, guid) = (CString::new("test").unwrap(), CString::new(guid).unwrap());
    unsafe { fmi2Instantiate(name.as_ptr(), kind, guid.as_ptr(), resources.as_ptr(), null(), 0, 0) }
}

fn vr(model: &Model, kind: &Kind) -> u32 {
    model.variables.iter().position(|v| &v.kind == kind).unwrap() as u32
}

fn get_reals(c: fmi2Component, refs: &[u32]) -> Vec<f64> {
    let mut out = vec![0.0; refs.len()];
    assert_eq!(unsafe { fmi2GetReal(c, refs.as_ptr(), refs.len(), out.as_mut_ptr()) }, fmi2OK);
    out
}

fn start(c: fmi2Component) {
    unsafe {
        assert_eq!(fmi2SetupExperiment(c, 0, 0.0, 0.0, 0, 0.0), fmi2OK);
        assert_eq!(fmi2EnterInitializationMode(c), fmi2OK);
        assert_eq!(fmi2ExitInitializationMode(c), fmi2OK);
    }
}

#[test]
fn a_move_through_the_fmu_matches_a_plain_simulation_bit_for_bit() {
    for entry in MACHINES.iter().copied().chain(machine_files()) {
        let (model, resources) = unpack(entry, "move");
        // A machine file goes along with the FMU, a registered machine is found by its id.
        assert_eq!(
            model.files().iter().any(|(path, _)| path == "resources/machine.toml"),
            !sim_fmi::is_registered(entry),
            "{}",
            entry.id
        );
        let cfg = load_config(entry, &model.config_text).unwrap();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let signals: Vec<u32> = model
            .variables
            .iter()
            .enumerate()
            .filter(|(_, v)| matches!(v.kind, Kind::Signal(_)))
            .map(|(i, _)| i as u32)
            .collect();
        assert_eq!(signals.len(), sim.signal_infos().len(), "{}: every signal is a variable", entry.id);
        let c = instantiate(&model.guid, &resources, fmi2CoSimulation);
        assert!(!c.is_null(), "{}", entry.id);
        start(c);
        // Before the first step: the machine at rest at its start pose.
        let (mut expected, mut found) = (Vec::new(), get_reals(c, &signals));
        sim.sample(&mut expected);
        assert_eq!(found, expected, "{}: at the start", entry.id);

        // A ramp on the first axis at a fifth of its speed limit, one setpoint per setpoint interval, then a hold.
        let interval = cfg.positive("profile.setpoint_interval_s").unwrap();
        let axis = sim.machine.axes()[0].clone();
        let setpoint = vr(&model, &Kind::Setpoint(0));
        let mut target = sim.control.set_pos.clone();
        let (step, direction) =
            (0.2 * axis.max_vel * interval, if target[0] < 0.5 * (axis.min + axis.max) { 1.0 } else { -1.0 });
        let steps = (interval / sim.dt).round() as u64;
        for k in 0..40 {
            if k < 20 {
                target[0] += direction * step;
            }
            unsafe { assert_eq!(fmi2SetReal(c, &setpoint, 1, &target[0]), fmi2OK) };
            let time = k as f64 * interval;
            assert_eq!(unsafe { fmi2DoStep(c, time, interval, 1) }, fmi2OK, "{} at {time}", entry.id);
            if target != sim.control.external_setpoint() {
                sim.command(Command::Setpoint(target.clone())).unwrap();
            }
            for _ in 0..steps {
                sim.step();
            }
            sim.sample(&mut expected);
            found = get_reals(c, &signals);
            assert_eq!(found, expected, "{} at {time}", entry.id);
        }
        // It moved by the ramp, and the setpoint input reads back as set.
        let position = sim.signal_index(&format!("axis.{}.position", axis.id)).unwrap();
        assert!((found[position] - target[0]).abs() < 1e-3, "{}: {} / {}", entry.id, found[position], target[0]);
        assert_eq!(get_reals(c, &[setpoint])[0], target[0]);
        unsafe {
            assert_eq!(fmi2Terminate(c), fmi2OK);
            fmi2FreeInstance(c);
        }
    }
}

#[test]
fn what_the_standard_forbids_is_refused() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "errors");
    // Another GUID, another FMU type, resources that are not there.
    assert!(instantiate("{00000000-0000-0000-0000-000000000000}", &resources, fmi2CoSimulation).is_null());
    assert!(instantiate(&model.guid, &resources, 0).is_null());
    assert!(instantiate(&model.guid, &CString::new("file:///nowhere/at/all").unwrap(), fmi2CoSimulation).is_null());

    // A config file that is not one: refused on leaving initialisation, and accepted once the parameter is back to empty.
    let c = instantiate(&model.guid, &resources, fmi2CoSimulation);
    assert!(!c.is_null());
    let bad = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("fmi-bad.toml");
    std::fs::write(&bad, "this is = not [a config").unwrap();
    let config_file = vr(&model, &Kind::ConfigFile);
    let (bad, empty) = (CString::new(bad.display().to_string()).unwrap(), CString::new("").unwrap());
    unsafe {
        assert_eq!(fmi2SetString(c, &config_file, 1, &bad.as_ptr()), fmi2OK);
        assert_eq!(fmi2EnterInitializationMode(c), fmi2OK);
        assert_eq!(fmi2ExitInitializationMode(c), fmi2Error);
        assert_eq!(fmi2SetString(c, &config_file, 1, &empty.as_ptr()), fmi2OK);
        assert_eq!(fmi2ExitInitializationMode(c), fmi2OK);
    }
    let dt = model.config_text.lines().find_map(|l| l.trim().strip_prefix("dt_s = ")).unwrap().parse::<f64>().unwrap();
    unsafe {
        // A step that is not a whole number of base steps, one from elsewhere than where the simulation stands.
        assert_eq!(fmi2DoStep(c, 0.0, 1.5 * dt, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 1.0, dt, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 0.0, 16.0 * dt, 1), fmi2OK);
        assert_eq!(fmi2DoStep(c, 16.0 * dt, dt, 1), fmi2OK);
        // Parameters are fixed once initialised; outputs are never set; unknown value references.
        assert_eq!(fmi2SetString(c, &config_file, 1, &empty.as_ptr()), fmi2Error);
        let output = vr(&model, &Kind::Signal(1));
        assert_eq!(fmi2SetReal(c, &output, 1, &0.0), fmi2Error);
        let mut value = 0.0;
        assert_eq!(fmi2GetReal(c, &1_000_000, 1, &mut value), fmi2Error);
        assert_eq!(fmi2GetReal(c, &config_file, 1, &mut value), fmi2Error);
        let mut integer = 0;
        assert_eq!(fmi2GetInteger(c, &0, 1, &mut integer), fmi2Error);
        assert_eq!(fmi2GetInteger(c, null(), 0, &mut integer), fmi2OK);
        // What this FMU does not support; a state that is not one.
        assert_eq!(fmi2CancelStep(c), fmi2Error);
        assert_eq!(fmi2SetFMUstate(c, std::ptr::null_mut()), fmi2Error);
        let mut state = std::ptr::null_mut();
        assert_eq!(fmi2DeSerializeFMUstate(c, c"sim-fmi state 1".as_ptr(), 16, &mut state), fmi2Error);
        assert!(state.is_null());
        // The parameters read back; the machine is the one of the FMU.
        let machine = vr(&model, &Kind::Machine);
        let mut text: *const c_char = null();
        assert_eq!(fmi2GetString(c, &machine, 1, &mut text), fmi2OK);
        assert_eq!(std::ffi::CStr::from_ptr(text).to_str().unwrap(), entry.id);
        // A reset starts over: initialisation again, from time 0.
        assert_eq!(fmi2Reset(c), fmi2OK);
        assert_eq!(fmi2DoStep(c, 0.0, dt, 1), fmi2Error);
        start(c);
        assert_eq!(fmi2DoStep(c, 0.0, dt, 1), fmi2OK);
        assert_eq!(std::ffi::CStr::from_ptr(fmi2GetVersion()).to_str().unwrap(), "2.0");
        assert_eq!(std::ffi::CStr::from_ptr(fmi2GetTypesPlatform()).to_str().unwrap(), "default");
        fmi2FreeInstance(c);
    }
}

/// Values are read from initialisation mode on, as FMI 2.0 has it: read before, the time output stayed at 0 after a
/// start time of 5 s was set up. A step of infinite size (which looped for ever) or at a time that is not a number
/// (which passed every comparison) is refused.
#[test]
fn early_reads_and_steps_that_are_not_finite_are_refused() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "finite");
    let c = instantiate(&model.guid, &resources, fmi2CoSimulation);
    let time = vr(&model, &Kind::Signal(0));
    assert_eq!(model.variables[time as usize].name, "time");
    let dt = model.config_text.lines().find_map(|l| l.trim().strip_prefix("dt_s = ")).unwrap().parse::<f64>().unwrap();
    let mut value = 0.0;
    unsafe {
        assert_eq!(fmi2GetReal(c, &time, 1, &mut value), fmi2Error);
        assert_eq!(fmi2SetupExperiment(c, 0, 0.0, 5.0, 0, 0.0), fmi2OK);
        assert_eq!(fmi2EnterInitializationMode(c), fmi2OK);
        assert_eq!(fmi2ExitInitializationMode(c), fmi2OK);
        assert_eq!(get_reals(c, &[time]), [5.0]);
        assert_eq!(fmi2DoStep(c, f64::NAN, dt, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 5.0, f64::NAN, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 5.0, f64::INFINITY, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, f64::INFINITY, dt, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 5.0, dt, 1), fmi2OK);
        assert_eq!(get_reals(c, &[time]), [5.0 + dt]);
        fmi2FreeInstance(c);
    }
}

/// A start time that is not a number was accepted, and then every communication point passed; a stop time or a
/// tolerance that is defined but not finite is refused too, one that is not defined is not read.
#[test]
fn an_experiment_that_is_not_finite_is_refused() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "experiment");
    let c = instantiate(&model.guid, &resources, fmi2CoSimulation);
    let dt = model.config_text.lines().find_map(|l| l.trim().strip_prefix("dt_s = ")).unwrap().parse::<f64>().unwrap();
    unsafe {
        assert_eq!(fmi2SetupExperiment(c, 0, 0.0, f64::NAN, 0, 0.0), fmi2Error);
        assert_eq!(fmi2SetupExperiment(c, 0, 0.0, 0.0, 1, f64::INFINITY), fmi2Error);
        assert_eq!(fmi2SetupExperiment(c, 1, f64::NAN, 0.0, 0, 0.0), fmi2Error);
        assert_eq!(fmi2SetupExperiment(c, 0, f64::NAN, 2.0, 0, f64::NAN), fmi2OK);
        assert_eq!(fmi2EnterInitializationMode(c), fmi2OK);
        assert_eq!(fmi2ExitInitializationMode(c), fmi2OK);
        assert_eq!(fmi2DoStep(c, 7.0, dt, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 2.0, dt, 1), fmi2OK);
        fmi2FreeInstance(c);
    }
}

/// A finite step of 1e20 s ran u64::MAX base steps (the conversion saturates): the importer hung. More base steps
/// than a u32 holds are refused.
#[test]
fn a_step_too_long_to_run_is_refused() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "long");
    let c = instantiate(&model.guid, &resources, fmi2CoSimulation);
    let dt = model.config_text.lines().find_map(|l| l.trim().strip_prefix("dt_s = ")).unwrap().parse::<f64>().unwrap();
    start(c);
    unsafe {
        assert_eq!(fmi2DoStep(c, 0.0, 1e20, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 0.0, (u32::MAX as f64 + 1.0) * dt, 1), fmi2Error);
        assert_eq!(fmi2DoStep(c, 0.0, dt, 1), fmi2OK);
        fmi2FreeInstance(c);
    }
}

#[test]
fn effect_switches_and_the_drive_enable_act_on_the_simulation() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "switches");
    let cfg = load_config(entry, &model.config_text).unwrap();
    // Every effect of the config is a parameter, with the config's value as its start.
    for (path, value) in cfg.leaves() {
        if let (Some(name), Some(on)) = (path.strip_prefix("effects."), value.as_bool()) {
            let v = &model.variables[vr(&model, &Kind::Effect(name.into())) as usize];
            assert_eq!(v.start.as_deref(), Some(on.to_string().as_str()), "{path}");
        }
    }
    // The first effect switched over through its parameter gives the simulation of the config switched over.
    let (name, on) = cfg
        .leaves()
        .into_iter()
        .find_map(|(p, v)| Some((p.strip_prefix("effects.")?.to_string(), v.as_bool()?)))
        .unwrap();
    let mut switched = cfg.clone();
    switched.set_text(&format!("effects.{name} = {}", !on)).unwrap();
    let mut sim = Sim::new(entry, &switched).unwrap();
    let c = instantiate(&model.guid, &resources, fmi2CoSimulation);
    let (effect, enable) = (vr(&model, &Kind::Effect(name.clone())), vr(&model, &Kind::Enable));
    unsafe { assert_eq!(fmi2SetBoolean(c, &effect, 1, &(!on as i32)), fmi2OK) };
    start(c);
    let signals: Vec<u32> = model
        .variables
        .iter()
        .enumerate()
        .filter(|(_, v)| matches!(v.kind, Kind::Signal(_)))
        .map(|(i, _)| i as u32)
        .collect();
    let mut expected = Vec::new();
    // Drives off after the first step: the controller disables them as the command would.
    for k in 0..4 {
        if k == 1 {
            unsafe { assert_eq!(fmi2SetBoolean(c, &enable, 1, &0), fmi2OK) };
            sim.command(Command::Disable).unwrap();
        }
        let size = 16.0 * sim.dt;
        assert_eq!(unsafe { fmi2DoStep(c, k as f64 * size, size, 1) }, fmi2OK);
        for _ in 0..16 {
            sim.step();
        }
        sim.sample(&mut expected);
        assert_eq!(get_reals(c, &signals), expected, "step {k}");
    }
    let mut read = 1;
    unsafe {
        assert_eq!(fmi2GetBoolean(c, &effect, 1, &mut read), fmi2OK);
        assert_eq!(read != 0, !on);
        fmi2FreeInstance(c);
    }
}

/// Saved after a few steps, the state brings the simulation back bit for bit: set on the same instance after it ran
/// on, and serialized and set on a second instance, the steps after it give the same signals as the first time.
#[test]
fn a_saved_fmu_state_restores_the_simulation_bit_for_bit() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "state");
    let signals: Vec<u32> = model
        .variables
        .iter()
        .enumerate()
        .filter(|(_, v)| matches!(v.kind, Kind::Signal(_)))
        .map(|(i, _)| i as u32)
        .collect();
    let (setpoint, enable) = (vr(&model, &Kind::Setpoint(0)), vr(&model, &Kind::Enable));
    let interval: f64 = model
        .config_text
        .lines()
        .find_map(|l| l.trim().strip_prefix("setpoint_interval_s = "))
        .unwrap()
        .parse()
        .unwrap();
    let first: f64 = model.variables[setpoint as usize].start.as_ref().unwrap().parse().unwrap();
    // One communication step: a setpoint 1 mm on, the drives off at step 6.
    let step = |c: fmi2Component, k: usize| {
        let target = first + 0.001 * k.min(8) as f64;
        unsafe {
            assert_eq!(fmi2SetReal(c, &setpoint, 1, &target), fmi2OK);
            assert_eq!(fmi2SetBoolean(c, &enable, 1, &((k != 6) as i32)), fmi2OK);
            assert_eq!(fmi2DoStep(c, k as f64 * interval, interval, 1), fmi2OK, "step {k}");
        }
        get_reals(c, &signals)
    };
    let c = instantiate_started(&model, &resources);
    for k in 0..4 {
        step(c, k);
    }
    let mut state = std::ptr::null_mut();
    unsafe { assert_eq!(fmi2GetFMUstate(c, &mut state), fmi2OK) };
    let after: Vec<Vec<f64>> = (4..10).map(|k| step(c, k)).collect();
    // Back on the same instance; the state may be taken again into the same handle.
    unsafe {
        assert_eq!(fmi2SetFMUstate(c, state), fmi2OK);
        assert_eq!(fmi2GetFMUstate(c, &mut state), fmi2OK);
    }
    let again: Vec<Vec<f64>> = (4..10).map(|k| step(c, k)).collect();
    assert_eq!(again, after);
    // Serialized, and set on another instance of the FMU.
    let mut size = 0;
    let mut bytes = Vec::new();
    unsafe {
        assert_eq!(fmi2SerializedFMUstateSize(c, state, &mut size), fmi2OK);
        bytes.resize(size, 0);
        assert_eq!(fmi2SerializeFMUstate(c, state, bytes.as_mut_ptr() as *mut c_char, size), fmi2OK);
        assert_eq!(fmi2SerializeFMUstate(c, state, bytes.as_mut_ptr() as *mut c_char, size - 1), fmi2Error);
        assert!(bytes.starts_with(b"sim-fmi state 1\0"));
        assert_eq!(fmi2FreeFMUstate(c, &mut state), fmi2OK);
        assert!(state.is_null());
        fmi2FreeInstance(c);
    }
    let other = instantiate(&model.guid, &resources, fmi2CoSimulation);
    let mut restored = std::ptr::null_mut();
    unsafe {
        assert_eq!(fmi2DeSerializeFMUstate(other, bytes.as_ptr() as *const c_char, size, &mut restored), fmi2OK);
        assert_eq!(fmi2SetFMUstate(other, restored), fmi2OK);
        assert_eq!(fmi2FreeFMUstate(other, &mut restored), fmi2OK);
    }
    let elsewhere: Vec<Vec<f64>> = (4..10).map(|k| step(other, k)).collect();
    assert_eq!(elsewhere, after);
    // A cut-off state is refused.
    unsafe {
        assert_eq!(fmi2DeSerializeFMUstate(other, bytes.as_ptr() as *const c_char, size - 1, &mut restored), fmi2Error);
        fmi2FreeInstance(other);
    }
}

/// The outputs depend on no input directly: a setpoint changed at a communication point changes no output there (the
/// finite difference by hand is zero), and the directional derivative is zero. Only outputs by inputs are accepted.
#[test]
fn the_directional_derivative_is_the_finite_difference_zero() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "derivative");
    let c = instantiate_started(&model, &resources);
    let outputs: Vec<u32> =
        model.variables.iter().enumerate().filter(|(_, v)| v.causality() == "output").map(|(i, _)| i as u32).collect();
    let setpoint = vr(&model, &Kind::Setpoint(0));
    let interval: f64 = model
        .config_text
        .lines()
        .find_map(|l| l.trim().strip_prefix("setpoint_interval_s = "))
        .unwrap()
        .parse()
        .unwrap();
    unsafe { assert_eq!(fmi2DoStep(c, 0.0, interval, 1), fmi2OK) };
    let base = get_reals(c, &outputs);
    let u = get_reals(c, &[setpoint])[0];
    let h = 1e-4;
    unsafe { assert_eq!(fmi2SetReal(c, &setpoint, 1, &(u + h)), fmi2OK) };
    let by_hand: Vec<f64> = get_reals(c, &outputs).iter().zip(&base).map(|(a, b)| (a - b) / h).collect();
    let mut derivative = vec![f64::NAN; outputs.len()];
    unsafe {
        assert_eq!(
            fmi2GetDirectionalDerivative(
                c,
                outputs.as_ptr(),
                outputs.len(),
                &setpoint,
                1,
                &1.0,
                derivative.as_mut_ptr()
            ),
            fmi2OK
        );
        assert_eq!(derivative, by_hand);
        assert!(derivative.iter().all(|d| *d == 0.0));
        // An input as unknown, an output as known.
        assert_eq!(
            fmi2GetDirectionalDerivative(c, &setpoint, 1, &setpoint, 1, &1.0, derivative.as_mut_ptr()),
            fmi2Error
        );
        assert_eq!(
            fmi2GetDirectionalDerivative(c, &outputs[0], 1, &outputs[1], 1, &1.0, derivative.as_mut_ptr()),
            fmi2Error
        );
        fmi2FreeInstance(c);
    }
}

fn instantiate_started(model: &Model, resources: &CString) -> fmi2Component {
    let c = instantiate(&model.guid, resources, fmi2CoSimulation);
    start(c);
    c
}

#[test]
fn the_guid_changes_with_the_model_description() {
    let entry = MACHINES[0];
    let text = entry.config().to_text().unwrap();
    let (a, b) = (Model::new(entry, &text, "en").unwrap(), Model::new(entry, &text, "en").unwrap());
    assert_eq!(a.guid, b.guid);
    assert_ne!(a.guid, Model::new(entry, &text, "de").unwrap().guid, "the descriptions are part of it");
    let mut cfg = entry.config();
    // An effect switch: the FMU's boolean start values are the effects (`drive.braking_module` is no variable).
    let (name, on) = cfg
        .leaves()
        .into_iter()
        .filter(|(p, _)| p.starts_with("effects."))
        .find_map(|(p, v)| Some((p, v.as_bool()?)))
        .unwrap();
    cfg.set_text(&format!("{name} = {}", !on)).unwrap();
    assert_ne!(a.guid, Model::new(entry, &cfg.to_text().unwrap(), "en").unwrap().guid, "a start value is part of it");
}
