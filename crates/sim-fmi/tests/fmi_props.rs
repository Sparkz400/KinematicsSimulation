//! Property test of the FMU's C functions (api.rs, api3.rs), driven in-process as an importing tool drives them, with
//! arbitrary sequences of calls and arguments: calls in the wrong order, value references out of range, null where the
//! standard allows a check, counts that do not match, strings without sense, steps of odd sizes, damaged FMU states.
//! Every call returns OK, Warning, Discard or Error, never Fatal (a panic caught inside), and the process does not crash.
//!
//! Left out, because C cannot defend against them: a dangling pointer (an instance after `FreeInstance`, a state after
//! `FreeFMUstate`, a pointer that is not from this library), a buffer shorter than its count says, a string without its
//! terminating zero. And steps longer than 1 s: a step of up to u32::MAX base steps is the importer's to ask for and
//! runs as long as it takes.

use proptest::prelude::*;
use sim_core::MachineEntry;
use sim_fmi::Model;
use sim_fmi::api::*;
use sim_fmi::api3::*;
use std::ffi::{CString, c_char, c_void};
use std::path::PathBuf;
use std::ptr::{null, null_mut};
use std::sync::OnceLock;

struct Unpacked {
    model: Model,
    uri: CString,
    path: CString,
}

/// The FMU of the list's second machine (the lightest to step) for FMI 2 and 3, unpacked once: its resources as a URI (FMI 2) and as a path (FMI 3).
fn unpacked(fmi: u8) -> &'static Unpacked {
    static CELLS: [OnceLock<Unpacked>; 2] = [OnceLock::new(), OnceLock::new()];
    CELLS[(fmi - 2) as usize].get_or_init(|| {
        let entry: &'static MachineEntry = sim_machines::MACHINES[1];
        let model = Model::with_fmi(entry, &entry.config().to_text().unwrap(), "en", fmi).unwrap();
        let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("fmi{fmi}-props"));
        for (path, bytes) in model.files() {
            let file = dir.join(path);
            std::fs::create_dir_all(file.parent().unwrap()).unwrap();
            std::fs::write(file, bytes).unwrap();
        }
        let resources = dir.join("resources");
        let uri = format!("file:///{}", resources.display().to_string().replace('\\', "/").trim_start_matches('/'));
        let path = format!("{}{}", resources.display(), std::path::MAIN_SEPARATOR);
        Unpacked { model, uri: CString::new(uri).unwrap(), path: CString::new(path).unwrap() }
    })
}

/// One call of the C API with its arguments; references and buffers as the strategy drew them.
#[derive(Debug, Clone)]
enum Call {
    Setup {
        tolerance: Option<f64>,
        start: f64,
        stop: Option<f64>,
    },
    /// Reset, setup, initialisation: what a tool does to start stepping.
    Start,
    Enter,
    Exit,
    Terminate,
    Reset,
    GetReal {
        refs: Vec<u32>,
        null_refs: bool,
        null_values: bool,
        values: usize,
    },
    SetReal {
        refs: Vec<u32>,
        values: Vec<f64>,
        null_values: bool,
    },
    GetBoolean {
        refs: Vec<u32>,
        null_values: bool,
    },
    SetBoolean {
        refs: Vec<u32>,
        values: Vec<i32>,
    },
    Integer {
        refs: Vec<u32>,
        set: bool,
    },
    GetString {
        refs: Vec<u32>,
        null_values: bool,
    },
    SetString {
        refs: Vec<u32>,
        values: Vec<Option<Vec<u8>>>,
    },
    DoStep {
        at_now: bool,
        point: f64,
        size: f64,
        null_flags: bool,
    },
    GetState {
        slot: usize,
    },
    SetState {
        slot: usize,
    },
    FreeState {
        slot: usize,
    },
    Serialize {
        slot: usize,
        size_off: i64,
        null_bytes: bool,
    },
    Deserialize {
        bytes: Vec<u8>,
        from_slot: Option<usize>,
        flip: Option<(usize, u8)>,
        slot: usize,
    },
    Logging {
        on: bool,
        categories: Vec<Option<Vec<u8>>>,
        null_list: bool,
    },
    Derivative {
        unknowns: Vec<u32>,
        knowns: Vec<u32>,
        n_out: usize,
    },
    Unsupported,
    NullInstance,
}

fn number() -> impl Strategy<Value = f64> {
    prop_oneof![
        prop::sample::select(vec![
            0.0,
            -0.0,
            1.0,
            -1.0,
            1e-3,
            0.01,
            1e300,
            -1e300,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::MIN_POSITIVE
        ]),
        any::<f64>(),
    ]
}

/// References in range, just beyond it and far beyond it.
fn refs() -> impl Strategy<Value = Vec<u32>> {
    let n = unpacked(2).model.variables.len() as u32;
    prop::collection::vec(prop_oneof![0..n, n..n + 3, Just(u32::MAX)], 0..6)
}

fn text() -> impl Strategy<Value = Option<Vec<u8>>> {
    prop::option::weighted(0.9, prop::collection::vec(1u8..=255, 0..20))
}

/// Step sizes around the setpoint interval (10 ms) and the base step (62.5 µs), and senseless ones; at most 1 s.
fn size() -> impl Strategy<Value = f64> {
    prop::sample::select(vec![
        0.01,
        0.02,
        0.005,
        62.5e-6,
        1e-4,
        31.25e-6,
        0.0,
        -0.01,
        1e-12,
        1.0,
        0.0101,
        f64::NAN,
        f64::INFINITY,
        -f64::INFINITY,
    ])
}

fn call() -> impl Strategy<Value = Call> {
    prop_oneof![
        1 => (prop::option::of(number()), number(), prop::option::of(number()))
            .prop_map(|(tolerance, start, stop)| Call::Setup { tolerance, start, stop }),
        3 => Just(Call::Start),
        2 => Just(Call::Enter),
        2 => Just(Call::Exit),
        1 => Just(Call::Terminate),
        1 => Just(Call::Reset),
        1 => (refs(), any::<bool>(), any::<bool>(), 0..8usize)
            .prop_map(|(refs, null_refs, null_values, values)| Call::GetReal { refs, null_refs, null_values, values }),
        3 => (refs(), prop::collection::vec(number(), 0..6), any::<bool>())
            .prop_map(|(refs, values, null_values)| Call::SetReal { refs, values, null_values }),
        1 => (refs(), any::<bool>()).prop_map(|(refs, null_values)| Call::GetBoolean { refs, null_values }),
        1 => (refs(), prop::collection::vec(any::<i32>(), 0..6)).prop_map(|(refs, values)| Call::SetBoolean { refs, values }),
        1 => (refs(), any::<bool>()).prop_map(|(refs, set)| Call::Integer { refs, set }),
        1 => (refs(), any::<bool>()).prop_map(|(refs, null_values)| Call::GetString { refs, null_values }),
        1 => (refs(), prop::collection::vec(text(), 0..6)).prop_map(|(refs, values)| Call::SetString { refs, values }),
        4 => (prop::bool::weighted(0.8), number(), size(), any::<bool>())
            .prop_map(|(at_now, point, size, null_flags)| Call::DoStep { at_now, point, size, null_flags }),
        1 => (0..3usize).prop_map(|slot| Call::GetState { slot }),
        1 => (0..4usize).prop_map(|slot| Call::SetState { slot }),
        1 => (0..4usize).prop_map(|slot| Call::FreeState { slot }),
        1 => (0..4usize, -2..3i64, any::<bool>())
            .prop_map(|(slot, size_off, null_bytes)| Call::Serialize { slot, size_off, null_bytes }),
        1 => (prop::collection::vec(any::<u8>(), 0..64), prop::option::of(0..3usize), prop::option::of((any::<usize>(), any::<u8>())), 0..4usize)
            .prop_map(|(bytes, from_slot, flip, slot)| Call::Deserialize { bytes, from_slot, flip, slot }),
        1 => (any::<bool>(), prop::collection::vec(text(), 0..4), any::<bool>())
            .prop_map(|(on, categories, null_list)| Call::Logging { on, categories, null_list }),
        1 => (refs(), refs(), 0..6usize).prop_map(|(unknowns, knowns, n_out)| Call::Derivative { unknowns, knowns, n_out }),
        1 => Just(Call::Unsupported),
        1 => Just(Call::NullInstance),
    ]
}

/// What the caller keeps between calls: the communication point it believes in and its FMU states (3 slots; slot 3
/// stands for a null state).
struct Caller {
    time: f64,
    states: [*mut c_void; 3],
}

impl Caller {
    fn state(&self, slot: usize) -> *mut c_void {
        self.states.get(slot).copied().unwrap_or(null_mut())
    }
}

fn strings(values: &[Option<Vec<u8>>]) -> (Vec<Option<CString>>, Vec<*const c_char>) {
    let owned: Vec<Option<CString>> = values.iter().map(|v| v.clone().map(|b| CString::new(b).unwrap())).collect();
    let pointers = owned.iter().map(|s| s.as_ref().map_or(null(), |s| s.as_ptr())).collect();
    (owned, pointers)
}

fn or_null<T>(v: &[T], null: bool) -> *const T {
    if null { std::ptr::null() } else { v.as_ptr() }
}

fn or_null_mut<T>(v: &mut [T], null: bool) -> *mut T {
    if null { std::ptr::null_mut() } else { v.as_mut_ptr() }
}

/// The bytes of a state in `slot` as the API serializes them, if it does.
unsafe fn serialized(c: *mut c_void, state: *mut c_void, fmi3: bool) -> Option<Vec<u8>> {
    let mut size = 0usize;
    let ok = unsafe {
        if fmi3 {
            fmi3SerializedFMUStateSize(c, state, &mut size)
        } else {
            fmi2SerializedFMUstateSize(c, state, &mut size)
        }
    };
    if ok != 0 {
        return None;
    }
    let mut bytes = vec![0u8; size];
    let ok = unsafe {
        if fmi3 {
            fmi3SerializeFMUState(c, state, bytes.as_mut_ptr(), size)
        } else {
            fmi2SerializeFMUstate(c, state, bytes.as_mut_ptr() as *mut c_char, size)
        }
    };
    (ok == 0).then_some(bytes)
}

/// Makes one call on instance `c` with FMI `fmi`; the status it returned.
unsafe fn make(c: *mut c_void, fmi3: bool, call: &Call, caller: &mut Caller) -> i32 {
    unsafe {
        match call {
            Call::Setup { tolerance, start, stop } => {
                let status = if fmi3 {
                    fmi3EnterInitializationMode(
                        c,
                        tolerance.is_some(),
                        tolerance.unwrap_or(0.0),
                        *start,
                        stop.is_some(),
                        stop.unwrap_or(0.0),
                    )
                } else {
                    fmi2SetupExperiment(
                        c,
                        tolerance.is_some() as i32,
                        tolerance.unwrap_or(0.0),
                        *start,
                        stop.is_some() as i32,
                        stop.unwrap_or(0.0),
                    )
                };
                if status == 0 {
                    caller.time = *start;
                }
                status
            }
            Call::Start => {
                let setup = Call::Setup { tolerance: None, start: 0.0, stop: None };
                let calls = [Call::Reset, setup, Call::Enter, Call::Exit];
                calls
                    .iter()
                    .filter(|c| !fmi3 || !matches!(c, Call::Enter))
                    .map(|step| make(c, fmi3, step, caller))
                    .max()
                    .unwrap()
            }
            Call::Enter if fmi3 => fmi3EnterInitializationMode(c, false, 0.0, caller.time, false, 0.0),
            Call::Enter => fmi2EnterInitializationMode(c),
            Call::Exit if fmi3 => fmi3ExitInitializationMode(c),
            Call::Exit => fmi2ExitInitializationMode(c),
            Call::Terminate if fmi3 => fmi3Terminate(c),
            Call::Terminate => fmi2Terminate(c),
            Call::Reset if fmi3 => fmi3Reset(c),
            Call::Reset => fmi2Reset(c),
            Call::GetReal { refs, null_refs, null_values, values } => {
                let mut out = vec![0.0; refs.len().max(*values)];
                if fmi3 {
                    // nValues is the count drawn, the buffer as long as the longer of the two.
                    fmi3GetFloat64(
                        c,
                        or_null(refs, *null_refs),
                        refs.len(),
                        or_null_mut(&mut out, *null_values),
                        *values,
                    )
                } else {
                    fmi2GetReal(c, or_null(refs, *null_refs), refs.len(), or_null_mut(&mut out, *null_values))
                }
            }
            Call::SetReal { refs, values, null_values } => {
                let mut buffer = values.clone();
                buffer.resize(refs.len().max(values.len()), 0.5);
                if fmi3 {
                    fmi3SetFloat64(c, refs.as_ptr(), refs.len(), or_null(&buffer, *null_values), values.len())
                } else {
                    fmi2SetReal(c, refs.as_ptr(), refs.len(), or_null(&buffer, *null_values))
                }
            }
            Call::GetBoolean { refs, null_values } => {
                if fmi3 {
                    let mut out = vec![false; refs.len()];
                    fmi3GetBoolean(c, refs.as_ptr(), refs.len(), or_null_mut(&mut out, *null_values), refs.len())
                } else {
                    let mut out = vec![0; refs.len()];
                    fmi2GetBoolean(c, refs.as_ptr(), refs.len(), or_null_mut(&mut out, *null_values))
                }
            }
            Call::SetBoolean { refs, values } => {
                let mut buffer = values.clone();
                buffer.resize(refs.len().max(values.len()), 1);
                if fmi3 {
                    let flags: Vec<bool> = buffer.iter().map(|v| *v != 0).collect();
                    fmi3SetBoolean(c, refs.as_ptr(), refs.len(), flags.as_ptr(), values.len())
                } else {
                    fmi2SetBoolean(c, refs.as_ptr(), refs.len(), buffer.as_ptr())
                }
            }
            Call::Integer { refs, set } => {
                let mut buffer = vec![0i32; refs.len()];
                match (fmi3, set) {
                    (true, true) => fmi3SetInt32(c, refs.as_ptr(), refs.len(), buffer.as_ptr(), refs.len()),
                    (true, false) => fmi3GetInt32(c, refs.as_ptr(), refs.len(), buffer.as_mut_ptr(), refs.len()),
                    (false, true) => fmi2SetInteger(c, refs.as_ptr(), refs.len(), buffer.as_ptr()),
                    (false, false) => fmi2GetInteger(c, refs.as_ptr(), refs.len(), buffer.as_mut_ptr()),
                }
            }
            Call::GetString { refs, null_values } => {
                let mut out = vec![null::<c_char>(); refs.len()];
                let status = if fmi3 {
                    fmi3GetString(c, refs.as_ptr(), refs.len(), or_null_mut(&mut out, *null_values), refs.len())
                } else {
                    fmi2GetString(c, refs.as_ptr(), refs.len(), or_null_mut(&mut out, *null_values))
                };
                // What it handed out is readable until the next call.
                for p in out.iter().filter(|p| !p.is_null()) {
                    let _ = std::ffi::CStr::from_ptr(*p).to_bytes().len();
                }
                status
            }
            Call::SetString { refs, values } => {
                let mut values = values.clone();
                let given = values.len();
                values.resize(refs.len().max(given), Some(b"x".to_vec()));
                let (_owned, pointers) = strings(&values);
                if fmi3 {
                    fmi3SetString(c, refs.as_ptr(), refs.len(), pointers.as_ptr(), given)
                } else {
                    fmi2SetString(c, refs.as_ptr(), refs.len(), pointers.as_ptr())
                }
            }
            Call::DoStep { at_now, point, size, null_flags } => {
                let point = if *at_now { caller.time } else { *point };
                let status = if fmi3 {
                    let (mut event, mut terminate, mut early, mut last) = (true, true, true, f64::NAN);
                    let flag = |f: &mut bool| if *null_flags { null_mut() } else { f as *mut bool };
                    fmi3DoStep(
                        c,
                        point,
                        *size,
                        false,
                        flag(&mut event),
                        flag(&mut terminate),
                        flag(&mut early),
                        if *null_flags { null_mut() } else { &mut last },
                    )
                } else {
                    fmi2DoStep(c, point, *size, 0)
                };
                if status == 0 {
                    caller.time = point + size;
                }
                status
            }
            Call::GetState { slot } => {
                let state = &mut caller.states[*slot];
                if fmi3 { fmi3GetFMUState(c, state) } else { fmi2GetFMUstate(c, state) }
            }
            Call::SetState { slot } => {
                let state = caller.state(*slot);
                if fmi3 { fmi3SetFMUState(c, state) } else { fmi2SetFMUstate(c, state) }
            }
            Call::FreeState { slot } => {
                let mut none = null_mut();
                let state = caller.states.get_mut(*slot).unwrap_or(&mut none);
                // The API nulls what it frees, so the slot never dangles.
                if fmi3 { fmi3FreeFMUState(c, state) } else { fmi2FreeFMUstate(c, state) }
            }
            Call::Serialize { slot, size_off, null_bytes } => {
                let state = caller.state(*slot);
                let mut size = 0usize;
                let first = if fmi3 {
                    fmi3SerializedFMUStateSize(c, state, if *null_bytes { null_mut() } else { &mut size })
                } else {
                    fmi2SerializedFMUstateSize(c, state, if *null_bytes { null_mut() } else { &mut size })
                };
                // A size off by some bytes against a buffer as long as the larger of the two.
                let asked = (size as i64 + size_off).max(0) as usize;
                let mut bytes = vec![0u8; asked.max(size)];
                let second = if fmi3 {
                    fmi3SerializeFMUState(c, state, or_null_mut(&mut bytes, *null_bytes), asked)
                } else {
                    fmi2SerializeFMUstate(c, state, or_null_mut(&mut bytes, *null_bytes) as *mut c_char, asked)
                };
                first.max(second)
            }
            Call::Deserialize { bytes, from_slot, flip, slot } => {
                let mut bytes = match from_slot.and_then(|s| serialized(c, caller.states[s], fmi3)) {
                    Some(real) => real,
                    None => bytes.clone(),
                };
                if let Some((at, by)) = flip
                    && !bytes.is_empty()
                {
                    let n = bytes.len();
                    bytes[at % n] ^= by;
                }
                let mut none = null_mut();
                let target = caller.states.get_mut(*slot).unwrap_or(&mut none);
                let status = if fmi3 {
                    fmi3DeserializeFMUState(c, bytes.as_ptr(), bytes.len(), target)
                } else {
                    fmi2DeSerializeFMUstate(c, bytes.as_ptr() as *const c_char, bytes.len(), target)
                };
                // A state made for the null slot belongs to nobody: freed at once.
                if !none.is_null() {
                    if fmi3 {
                        fmi3FreeFMUState(c, &mut none)
                    } else {
                        fmi2FreeFMUstate(c, &mut none)
                    };
                }
                status
            }
            Call::Logging { on, categories, null_list } => {
                let (_owned, pointers) = strings(categories);
                let list = if *null_list { null() } else { pointers.as_ptr() };
                if fmi3 {
                    fmi3SetDebugLogging(c, *on, pointers.len(), list)
                } else {
                    fmi2SetDebugLogging(c, *on as i32, pointers.len(), list)
                }
            }
            Call::Derivative { unknowns, knowns, n_out } => {
                let mut out = vec![0.0; unknowns.len().max(*n_out)];
                let seed = vec![1.0; knowns.len()];
                if fmi3 {
                    fmi3GetDirectionalDerivative(
                        c,
                        unknowns.as_ptr(),
                        unknowns.len(),
                        knowns.as_ptr(),
                        knowns.len(),
                        seed.as_ptr(),
                        seed.len(),
                        out.as_mut_ptr(),
                        *n_out,
                    )
                } else {
                    fmi2GetDirectionalDerivative(
                        c,
                        unknowns.as_ptr(),
                        unknowns.len(),
                        knowns.as_ptr(),
                        knowns.len(),
                        seed.as_ptr(),
                        out.as_mut_ptr(),
                    )
                }
            }
            Call::Unsupported if fmi3 => {
                let r = 0u32;
                let mut value = 0f32;
                fmi3GetFloat32(c, &r, 1, &mut value, 1)
            }
            Call::Unsupported => fmi2CancelStep(c),
            Call::NullInstance if fmi3 => {
                fmi3DoStep(null_mut(), 0.0, 0.01, false, null_mut(), null_mut(), null_mut(), null_mut())
            }
            Call::NullInstance => fmi2GetReal(null_mut(), null(), 0, null_mut()),
        }
    }
}

fn instantiate(fmi3: bool, token: Option<&str>, resources: Option<&CString>, co_simulation: bool) -> *mut c_void {
    let name = CString::new("props").unwrap();
    let token = token.map(|t| CString::new(t).unwrap());
    let token = token.as_ref().map_or(null(), |t| t.as_ptr());
    let resources = resources.map_or(null(), |r| r.as_ptr());
    unsafe {
        match (fmi3, co_simulation) {
            (true, true) => fmi3InstantiateCoSimulation(
                name.as_ptr(),
                token,
                resources,
                false,
                false,
                false,
                false,
                null(),
                0,
                null_mut(),
                None,
                None,
            ),
            (true, false) => {
                fmi3InstantiateModelExchange(name.as_ptr(), token, resources, false, false, null_mut(), None)
            }
            (false, kind) => fmi2Instantiate(name.as_ptr(), if kind { 1 } else { 0 }, token, resources, null(), 0, 0),
        }
    }
}

fn run(fmi3: bool, calls: &[Call]) -> Result<(), TestCaseError> {
    let u = unpacked(if fmi3 { 3 } else { 2 });
    let resources = if fmi3 { &u.path } else { &u.uri };
    let c = instantiate(fmi3, Some(&u.model.guid), Some(resources), true);
    prop_assert!(!c.is_null());
    let mut caller = Caller { time: 0.0, states: [null_mut(); 3] };
    for (k, call) in calls.iter().enumerate() {
        let status = unsafe { make(c, fmi3, call, &mut caller) };
        prop_assert!((0..=3).contains(&status), "call {k}, {call:?}: status {status}");
    }
    unsafe {
        for state in &mut caller.states {
            if fmi3 {
                fmi3FreeFMUState(c, state)
            } else {
                fmi2FreeFMUstate(c, state)
            };
        }
        if fmi3 {
            fmi3FreeInstance(c)
        } else {
            fmi2FreeInstance(c)
        };
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(128))]

    #[test]
    fn fmi2_calls_in_any_order_give_a_status_never_a_crash(calls in prop::collection::vec(call(), 1..40)) {
        run(false, &calls)?;
    }

    #[test]
    fn fmi3_calls_in_any_order_give_a_status_never_a_crash(calls in prop::collection::vec(call(), 1..40)) {
        run(true, &calls)?;
    }

    /// A wrong token, no token, no or wrong resources, or not co-simulation: null, after a logged message.
    #[test]
    fn instantiation_with_wrong_arguments_gives_null(
        fmi3 in any::<bool>(),
        token in prop::option::of(prop_oneof![Just("{wrong}".to_string()), "[ -~]{0,20}"]),
        resources in prop::option::of(prop_oneof![Just("file:///nowhere/at/all".to_string()), "[ -~]{0,20}"]),
        co_simulation in any::<bool>(),
    ) {
        let resources = resources.map(|r| CString::new(r).unwrap());
        let c = instantiate(fmi3, token.as_deref(), resources.as_ref(), co_simulation);
        prop_assert!(c.is_null());
    }
}
