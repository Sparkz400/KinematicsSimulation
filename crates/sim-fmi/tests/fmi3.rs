//! The FMI 3.0 C functions, called as an importing tool calls them: the same simulation as the FMI 2.0 ones and a
//! plain `Sim`, bit for bit; FMU state and derivatives; what FMI 3.0 forbids or this FMU does not offer is refused.

use sim_components::{Command, Sim, load_config};
use sim_core::MachineEntry;
use sim_fmi::api3::*;
use sim_fmi::{Kind, Model};
use sim_machines::MACHINES;
use std::ffi::{CStr, CString, c_char};
use std::path::PathBuf;
use std::ptr::{null, null_mut};

/// The FMU's model files unpacked into a folder of their own; FMI 3.0 passes the resources as a plain path.
fn unpack(entry: &'static MachineEntry, name: &str) -> (Model, CString) {
    let text = entry.config().to_text().unwrap();
    let model = Model::with_fmi(entry, &text, "en", 3).unwrap();
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("fmi3-{name}-{}", entry.id));
    for (path, bytes) in model.files() {
        let file = dir.join(path);
        std::fs::create_dir_all(file.parent().unwrap()).unwrap();
        std::fs::write(file, bytes).unwrap();
    }
    let resources = format!("{}{}", dir.join("resources").display(), std::path::MAIN_SEPARATOR);
    (model, CString::new(resources).unwrap())
}

/// The machine files of machines/, which the FMU builds from its resources (`resources/machine.toml`).
fn machine_files() -> Vec<&'static MachineEntry> {
    ["h-bot.toml", "corexy.toml", "xyz-portal.toml"]
        .iter()
        .map(|name| {
            sim_machines::load_file(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name))
                .unwrap_or_else(|e| panic!("{e}"))
        })
        .collect()
}

fn instantiate(token: &str, resources: &CString) -> fmi3Instance {
    let (name, token) = (CString::new("test").unwrap(), CString::new(token).unwrap());
    unsafe {
        fmi3InstantiateCoSimulation(
            name.as_ptr(),
            token.as_ptr(),
            resources.as_ptr(),
            false,
            false,
            false,
            false,
            null(),
            0,
            null_mut(),
            None,
            None,
        )
    }
}

fn started(model: &Model, resources: &CString) -> fmi3Instance {
    let c = instantiate(&model.guid, resources);
    assert!(!c.is_null());
    unsafe {
        assert_eq!(fmi3EnterInitializationMode(c, false, 0.0, 0.0, false, 0.0), fmi3OK);
        assert_eq!(fmi3ExitInitializationMode(c), fmi3OK);
    }
    c
}

fn vr(model: &Model, kind: &Kind) -> u32 {
    model.variables.iter().position(|v| &v.kind == kind).unwrap() as u32
}

fn signals(model: &Model) -> Vec<u32> {
    model
        .variables
        .iter()
        .enumerate()
        .filter(|(_, v)| matches!(v.kind, Kind::Signal(_)))
        .map(|(i, _)| i as u32)
        .collect()
}

fn get(c: fmi3Instance, refs: &[u32]) -> Vec<f64> {
    let mut out = vec![0.0; refs.len()];
    assert_eq!(unsafe { fmi3GetFloat64(c, refs.as_ptr(), refs.len(), out.as_mut_ptr(), out.len()) }, fmi3OK);
    out
}

fn do_step(c: fmi3Instance, time: f64, size: f64) -> fmi3Status {
    let (mut event, mut terminate, mut early, mut last) = (true, true, true, f64::NAN);
    let status = unsafe { fmi3DoStep(c, time, size, true, &mut event, &mut terminate, &mut early, &mut last) };
    assert!(!event && !terminate && !early);
    if status == fmi3OK {
        assert!((last - (time + size)).abs() < 1e-12, "{last} / {}", time + size);
    }
    status
}

fn interval(model: &Model) -> f64 {
    model.config_text.lines().find_map(|l| l.trim().strip_prefix("setpoint_interval_s = ")).unwrap().parse().unwrap()
}

#[test]
fn a_move_through_the_fmi3_functions_matches_a_plain_simulation_bit_for_bit() {
    for entry in MACHINES.iter().copied().chain(machine_files()) {
        let (model, resources) = unpack(entry, "move");
        assert!(
            model.xml.contains(r#"fmiVersion="3.0""#)
                && model.xml.contains(&format!(r#"instantiationToken="{}""#, model.guid))
        );
        let cfg = load_config(entry, &model.config_text).unwrap();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let c = started(&model, &resources);
        let refs = signals(&model);
        let mut expected = Vec::new();
        sim.sample(&mut expected);
        assert_eq!(get(c, &refs), expected, "{}: at the start", entry.id);
        let interval = interval(&model);
        let axis = sim.machine.axes()[0].clone();
        let setpoint = vr(&model, &Kind::Setpoint(0));
        let mut target = sim.control.set_pos.clone();
        let (step, direction) =
            (0.2 * axis.max_vel * interval, if target[0] < 0.5 * (axis.min + axis.max) { 1.0 } else { -1.0 });
        let steps = (interval / sim.dt).round() as u64;
        for k in 0..30 {
            if k < 15 {
                target[0] += direction * step;
            }
            unsafe { assert_eq!(fmi3SetFloat64(c, &setpoint, 1, &target[0], 1), fmi3OK) };
            assert_eq!(do_step(c, k as f64 * interval, interval), fmi3OK, "{} at step {k}", entry.id);
            if target != sim.control.external_setpoint() {
                sim.command(Command::Setpoint(target.clone())).unwrap();
            }
            for _ in 0..steps {
                sim.step();
            }
            sim.sample(&mut expected);
            assert_eq!(get(c, &refs), expected, "{} at step {k}", entry.id);
        }
        unsafe {
            assert_eq!(fmi3Terminate(c), fmi3OK);
            fmi3FreeInstance(c);
        }
    }
}

/// As for FMI 2.0: the state taken after a few steps brings back the same signals, on the instance and serialized
/// on another one. The drives are switched off and on in between, and the setpoint moves.
#[test]
fn a_saved_fmi3_state_restores_the_simulation_bit_for_bit() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "state");
    let refs = signals(&model);
    let (setpoint, enable) = (vr(&model, &Kind::Setpoint(0)), vr(&model, &Kind::Enable));
    let (interval, first): (f64, f64) =
        (interval(&model), model.variables[setpoint as usize].start.as_ref().unwrap().parse().unwrap());
    let step = |c: fmi3Instance, k: usize| {
        let target = first + 0.001 * k.min(8) as f64;
        unsafe {
            assert_eq!(fmi3SetFloat64(c, &setpoint, 1, &target, 1), fmi3OK);
            assert_eq!(fmi3SetBoolean(c, &enable, 1, &(k != 5), 1), fmi3OK);
        }
        assert_eq!(do_step(c, k as f64 * interval, interval), fmi3OK, "step {k}");
        get(c, &refs)
    };
    let c = started(&model, &resources);
    (0..3).for_each(|k| drop(step(c, k)));
    let mut state = null_mut();
    unsafe { assert_eq!(fmi3GetFMUState(c, &mut state), fmi3OK) };
    let after: Vec<Vec<f64>> = (3..9).map(|k| step(c, k)).collect();
    unsafe { assert_eq!(fmi3SetFMUState(c, state), fmi3OK) };
    assert_eq!((3..9).map(|k| step(c, k)).collect::<Vec<_>>(), after);
    let mut size = 0;
    unsafe { assert_eq!(fmi3SerializedFMUStateSize(c, state, &mut size), fmi3OK) };
    let mut bytes = vec![0u8; size];
    unsafe {
        assert_eq!(fmi3SerializeFMUState(c, state, bytes.as_mut_ptr(), size), fmi3OK);
        assert_eq!(fmi3FreeFMUState(c, &mut state), fmi3OK);
        fmi3FreeInstance(c);
    }
    // Deserialized on a fresh instance that has not even been initialised.
    let other = instantiate(&model.guid, &resources);
    let mut restored = null_mut();
    unsafe {
        assert_eq!(fmi3DeserializeFMUState(other, bytes.as_ptr(), size, &mut restored), fmi3OK);
        assert_eq!(fmi3SetFMUState(other, restored), fmi3OK);
        assert_eq!(fmi3FreeFMUState(other, &mut restored), fmi3OK);
    }
    assert_eq!((3..9).map(|k| step(other, k)).collect::<Vec<_>>(), after);
    unsafe { fmi3FreeInstance(other) };

    // A state of the FMI 2.0 FMU of the same machine is another FMU's: refused.
    let fmi2 = Model::new(entry, &model.config_text, "en").unwrap();
    assert_ne!(fmi2.guid, model.guid);
    let other = started(&model, &resources);
    let foreign = [
        b"sim-fmi state 1\0".as_slice(),
        &(fmi2.guid.len() as u64).to_le_bytes(),
        fmi2.guid.as_bytes(),
        &bytes[16 + 8 + model.guid.len()..],
    ]
    .concat();
    unsafe {
        assert_eq!(fmi3DeserializeFMUState(other, foreign.as_ptr(), foreign.len(), &mut restored), fmi3Error);
        fmi3FreeInstance(other);
    }
}

/// A setpoint changed at a communication point changes no output there: the finite difference by hand is zero, and
/// so are the directional and the adjoint derivative.
#[test]
fn the_fmi3_derivatives_are_the_finite_difference_zero() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "derivative");
    let c = started(&model, &resources);
    let outputs: Vec<u32> =
        model.variables.iter().enumerate().filter(|(_, v)| v.causality() == "output").map(|(i, _)| i as u32).collect();
    let setpoints: Vec<u32> = model
        .variables
        .iter()
        .enumerate()
        .filter(|(_, v)| matches!(v.kind, Kind::Setpoint(_)))
        .map(|(i, _)| i as u32)
        .collect();
    assert_eq!(do_step(c, 0.0, interval(&model)), fmi3OK);
    let base = get(c, &outputs);
    let u = get(c, &setpoints);
    let h = 1e-4;
    let mut jacobian_by_hand = Vec::new();
    for (k, s) in setpoints.iter().enumerate() {
        unsafe { assert_eq!(fmi3SetFloat64(c, s, 1, &(u[k] + h), 1), fmi3OK) };
        jacobian_by_hand.extend(get(c, &outputs).iter().zip(&base).map(|(a, b)| (a - b) / h));
        unsafe { assert_eq!(fmi3SetFloat64(c, s, 1, &u[k], 1), fmi3OK) };
    }
    let mut column = vec![f64::NAN; outputs.len()];
    let mut row = vec![f64::NAN; setpoints.len()];
    let seed = vec![1.0; outputs.len().max(setpoints.len())];
    unsafe {
        for s in &setpoints {
            assert_eq!(
                fmi3GetDirectionalDerivative(
                    c,
                    outputs.as_ptr(),
                    outputs.len(),
                    s,
                    1,
                    seed.as_ptr(),
                    1,
                    column.as_mut_ptr(),
                    column.len()
                ),
                fmi3OK
            );
            assert!(column.iter().all(|d| *d == 0.0));
        }
        assert_eq!(
            fmi3GetAdjointDerivative(
                c,
                outputs.as_ptr(),
                outputs.len(),
                setpoints.as_ptr(),
                setpoints.len(),
                seed.as_ptr(),
                outputs.len(),
                row.as_mut_ptr(),
                row.len()
            ),
            fmi3OK
        );
        assert!(row.iter().all(|d| *d == 0.0));
        assert_eq!(
            fmi3GetDirectionalDerivative(
                c,
                setpoints.as_ptr(),
                1,
                outputs.as_ptr(),
                1,
                seed.as_ptr(),
                1,
                column.as_mut_ptr(),
                1
            ),
            fmi3Error
        );
        // One seed per known and one sensitivity per unknown, the adjoint the other way round; fewer are refused
        // (fewer, so that a count not checked writes within the buffers).
        assert!(setpoints.len() >= 2);
        let s = &setpoints[0];
        assert_eq!(
            fmi3GetDirectionalDerivative(
                c,
                outputs.as_ptr(),
                outputs.len(),
                s,
                1,
                seed.as_ptr(),
                1,
                column.as_mut_ptr(),
                outputs.len() - 1
            ),
            fmi3Error
        );
        assert_eq!(
            fmi3GetDirectionalDerivative(
                c,
                outputs.as_ptr(),
                outputs.len(),
                setpoints.as_ptr(),
                2,
                seed.as_ptr(),
                1,
                column.as_mut_ptr(),
                outputs.len()
            ),
            fmi3Error
        );
        assert_eq!(
            fmi3GetAdjointDerivative(
                c,
                outputs.as_ptr(),
                outputs.len(),
                setpoints.as_ptr(),
                setpoints.len(),
                seed.as_ptr(),
                outputs.len() - 1,
                row.as_mut_ptr(),
                row.len()
            ),
            fmi3Error
        );
        assert_eq!(
            fmi3GetAdjointDerivative(
                c,
                outputs.as_ptr(),
                outputs.len(),
                setpoints.as_ptr(),
                setpoints.len(),
                seed.as_ptr(),
                outputs.len(),
                row.as_mut_ptr(),
                row.len() - 1
            ),
            fmi3Error
        );
        fmi3FreeInstance(c);
    }
    assert!(jacobian_by_hand.iter().all(|d| *d == 0.0), "{jacobian_by_hand:?}");
}

#[test]
fn what_fmi3_forbids_or_this_fmu_does_not_offer_is_refused() {
    let entry = MACHINES[0];
    let (model, resources) = unpack(entry, "errors");
    let (name, token) = (CString::new("test").unwrap(), CString::new(model.guid.clone()).unwrap());
    unsafe {
        // Model Exchange and Scheduled Execution; another token; the FMI 2.0 FMU's GUID.
        assert!(
            fmi3InstantiateModelExchange(
                name.as_ptr(),
                token.as_ptr(),
                resources.as_ptr(),
                false,
                false,
                null_mut(),
                None
            )
            .is_null()
        );
        assert!(
            fmi3InstantiateScheduledExecution(
                name.as_ptr(),
                token.as_ptr(),
                resources.as_ptr(),
                false,
                false,
                null_mut(),
                None,
                null(),
                null(),
                null()
            )
            .is_null()
        );
    }
    assert!(instantiate("{00000000-0000-0000-0000-000000000000}", &resources).is_null());
    assert!(instantiate(&Model::new(entry, &model.config_text, "en").unwrap().guid, &resources).is_null());
    let c = instantiate(&model.guid, &resources);
    let time = vr(&model, &Kind::Signal(0));
    let mut value = 0.0;
    unsafe {
        // Values are read from initialisation on; a start time that is not finite is refused.
        assert_eq!(fmi3GetFloat64(c, &time, 1, &mut value, 1), fmi3Error);
        assert_eq!(fmi3EnterInitializationMode(c, false, 0.0, f64::NAN, false, 0.0), fmi3Error);
        assert_eq!(fmi3EnterInitializationMode(c, true, 1e-6, 2.0, true, 10.0), fmi3OK);
        assert_eq!(fmi3ExitInitializationMode(c), fmi3OK);
        assert_eq!(get(c, &[time]), [2.0]);
        // One value per value reference; types this FMU has no variables of.
        assert_eq!(fmi3GetFloat64(c, &time, 1, &mut value, 2), fmi3Error);
        let mut integer = 0i32;
        assert_eq!(fmi3GetInt32(c, &time, 1, &mut integer, 1), fmi3Error);
        assert_eq!(fmi3GetInt32(c, null(), 0, &mut integer, 0), fmi3OK);
        // No Event Mode, no Model Exchange functions, no clocks.
        assert_eq!(fmi3EnterEventMode(c), fmi3Error);
        assert_eq!(fmi3EnterStepMode(c), fmi3Error);
        let mut n = 0usize;
        assert_eq!(fmi3GetNumberOfContinuousStates(c, &mut n), fmi3Error);
        // A step from elsewhere is refused and leaves the time where it was.
        let (mut event, mut terminate, mut early, mut last) = (false, false, false, 0.0);
        assert_eq!(fmi3DoStep(c, 0.0, 0.01, true, &mut event, &mut terminate, &mut early, &mut last), fmi3Error);
        assert_eq!(last, 2.0);
        assert_eq!(do_step(c, 2.0, interval(&model)), fmi3OK);
        // A null pointer for values that are there is refused, not read as no values.
        assert_eq!(fmi3GetFloat64(c, null(), 1, &mut value, 1), fmi3Error);
        assert_eq!(fmi3GetFloat64(c, &time, 1, null_mut(), 1), fmi3Error);
        // Strings and Booleans read back; parameters are fixed after initialisation.
        let machine = vr(&model, &Kind::Machine);
        let mut text: *const c_char = null();
        assert_eq!(fmi3GetString(c, &machine, 1, &mut text, 1), fmi3OK);
        assert_eq!(CStr::from_ptr(text).to_str().unwrap(), entry.id);
        let empty = CString::new("").unwrap();
        assert_eq!(fmi3SetString(c, &vr(&model, &Kind::ConfigFile), 1, &empty.as_ptr(), 1), fmi3Error);
        let mut on = false;
        assert_eq!(fmi3GetBoolean(c, &vr(&model, &Kind::Enable), 1, &mut on, 1), fmi3OK);
        assert!(on);
        // A reset starts over.
        assert_eq!(fmi3Reset(c), fmi3OK);
        assert_eq!(fmi3GetFloat64(c, &time, 1, &mut value, 1), fmi3Error);
        assert_eq!(CStr::from_ptr(fmi3GetVersion()).to_str().unwrap(), "3.0");
        fmi3FreeInstance(c);
    }
}
