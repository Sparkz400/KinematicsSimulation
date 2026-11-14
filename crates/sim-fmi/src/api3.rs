//! The FMI 3.0 Co-Simulation C functions, on the same instance as the FMI 2.0 ones (`api::Instance`). The type
//! definitions follow the FMI 3.0 headers (fmi3PlatformTypes.h, fmi3FunctionTypes.h; Copyright Modelica Association
//! Project "FMI", BSD-2-Clause licence), written out in Rust.
//!
//! Supported: `fmi3InstantiateCoSimulation`, initialisation, `fmi3DoStep`, get and set of Float64, Boolean and String
//! variables (the other types have no variables: any value reference is refused), reset, terminate, FMU state get,
//! set, free and (de)serialization, directional and adjoint derivatives (zero, see `api`). Not supported, returning
//! `fmi3Error` or a null instance: Model Exchange and Scheduled Execution, Event Mode, Configuration Mode, clocks,
//! output derivatives, variable dependencies at run time (the model description has them).

#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]

use crate::api::{Instance, Logger, Value, get_strings, instantiate, slice, slice_mut, text, with};
use std::ffi::{c_char, c_void};
use std::path::Path;

pub type fmi3Instance = *mut c_void;
pub type fmi3InstanceEnvironment = *mut c_void;
pub type fmi3FMUState = *mut c_void;
pub type fmi3ValueReference = u32;
pub type fmi3Float64 = f64;
pub type fmi3Boolean = bool;
pub type fmi3String = *const c_char;
pub type fmi3Byte = u8;
pub type fmi3Status = std::ffi::c_int;

pub const fmi3OK: fmi3Status = 0;
pub const fmi3Warning: fmi3Status = 1;
pub const fmi3Discard: fmi3Status = 2;
pub const fmi3Error: fmi3Status = 3;
pub const fmi3Fatal: fmi3Status = 4;

pub type fmi3LogMessageCallback = unsafe extern "C" fn(fmi3InstanceEnvironment, fmi3Status, fmi3String, fmi3String);
pub type fmi3IntermediateUpdateCallback =
    unsafe extern "C" fn(fmi3InstanceEnvironment, f64, bool, bool, bool, bool, *mut bool, *mut f64);

#[unsafe(no_mangle)]
pub extern "C" fn fmi3GetVersion() -> *const c_char {
    c"3.0".as_ptr()
}

/// `resourcePath` is a plain absolute path to the resources folder, as FMI 3.0 has it.
///
/// # Safety
/// As the FMI standard demands of the caller: valid strings, the callback valid for the instance's life.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3InstantiateCoSimulation(
    instanceName: fmi3String,
    instantiationToken: fmi3String,
    resourcePath: fmi3String,
    _visible: fmi3Boolean,
    _loggingOn: fmi3Boolean,
    _eventModeUsed: fmi3Boolean,
    _earlyReturnAllowed: fmi3Boolean,
    _requiredIntermediateVariables: *const fmi3ValueReference,
    _nRequiredIntermediateVariables: usize,
    instanceEnvironment: fmi3InstanceEnvironment,
    logMessage: Option<fmi3LogMessageCallback>,
    _intermediateUpdate: Option<fmi3IntermediateUpdateCallback>,
) -> fmi3Instance {
    let resources = Path::new(unsafe { text(resourcePath) });
    instantiate(
        unsafe { text(instanceName) },
        Logger::Fmi3(logMessage),
        instanceEnvironment,
        resources,
        unsafe { text(instantiationToken) },
        3,
        true,
    )
}

/// Not supported: logs why and returns null.
///
/// # Safety
/// See `fmi3InstantiateCoSimulation`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3InstantiateModelExchange(
    instanceName: fmi3String,
    instantiationToken: fmi3String,
    resourcePath: fmi3String,
    _visible: fmi3Boolean,
    _loggingOn: fmi3Boolean,
    instanceEnvironment: fmi3InstanceEnvironment,
    logMessage: Option<fmi3LogMessageCallback>,
) -> fmi3Instance {
    let resources = Path::new(unsafe { text(resourcePath) });
    instantiate(
        unsafe { text(instanceName) },
        Logger::Fmi3(logMessage),
        instanceEnvironment,
        resources,
        unsafe { text(instantiationToken) },
        3,
        false,
    )
}

/// Not supported: logs why and returns null.
///
/// # Safety
/// See `fmi3InstantiateCoSimulation`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3InstantiateScheduledExecution(
    instanceName: fmi3String,
    instantiationToken: fmi3String,
    resourcePath: fmi3String,
    _visible: fmi3Boolean,
    _loggingOn: fmi3Boolean,
    instanceEnvironment: fmi3InstanceEnvironment,
    logMessage: Option<fmi3LogMessageCallback>,
    _clockUpdate: *const c_void,
    _lockPreemption: *const c_void,
    _unlockPreemption: *const c_void,
) -> fmi3Instance {
    let resources = Path::new(unsafe { text(resourcePath) });
    instantiate(
        unsafe { text(instanceName) },
        Logger::Fmi3(logMessage),
        instanceEnvironment,
        resources,
        unsafe { text(instantiationToken) },
        3,
        false,
    )
}

/// # Safety
/// `c` is null or an instance from `fmi3InstantiateCoSimulation`; it is not used afterwards.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3FreeInstance(c: fmi3Instance) {
    if !c.is_null() {
        drop(unsafe { Box::from_raw(c as *mut Instance) });
    }
}

/// # Safety
/// See `api::with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetDebugLogging(
    c: fmi3Instance,
    _loggingOn: fmi3Boolean,
    _nCategories: usize,
    _categories: *const fmi3String,
) -> fmi3Status {
    // Only warnings and errors are ever logged, and those always.
    unsafe { with(c, |_| Ok(fmi3OK)) }
}

/// # Safety
/// See `api::with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3EnterInitializationMode(
    c: fmi3Instance,
    toleranceDefined: fmi3Boolean,
    tolerance: f64,
    startTime: f64,
    stopTimeDefined: fmi3Boolean,
    stopTime: f64,
) -> fmi3Status {
    unsafe {
        with(c, |i| {
            i.setup(toleranceDefined.then_some(tolerance), startTime, stopTimeDefined.then_some(stopTime))?;
            i.enter_initialisation()
        })
    }
}

/// Without Event Mode, initialisation ends in Step Mode.
///
/// # Safety
/// See `api::with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3ExitInitializationMode(c: fmi3Instance) -> fmi3Status {
    unsafe { with(c, Instance::exit_initialisation) }
}

/// # Safety
/// See `api::with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3Terminate(c: fmi3Instance) -> fmi3Status {
    unsafe { with(c, Instance::terminate) }
}

/// # Safety
/// See `api::with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3Reset(c: fmi3Instance) -> fmi3Status {
    unsafe { with(c, Instance::reset) }
}

/// A scalar variable takes one value per value reference.
fn counts(nvr: usize, n_values: usize) -> Result<(), crate::api::Fail> {
    if nvr == n_values { Ok(()) } else { Err(("fmi-error-count", format!("{nvr} / {n_values}"))) }
}

/// # Safety
/// See `api::with`; `vr` points to `nvr` values, `values` to `nValues`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetFloat64(
    c: fmi3Instance,
    vr: *const fmi3ValueReference,
    nvr: usize,
    values: *mut f64,
    nValues: usize,
) -> fmi3Status {
    unsafe {
        with(c, |i| {
            counts(nvr, nValues)?;
            for (r, v) in slice(vr, nvr)?.iter().zip(slice_mut(values, nValues)?) {
                *v = i.get_real(*r)?;
            }
            Ok(fmi3OK)
        })
    }
}

/// # Safety
/// See `fmi3GetFloat64`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetFloat64(
    c: fmi3Instance,
    vr: *const fmi3ValueReference,
    nvr: usize,
    values: *const f64,
    nValues: usize,
) -> fmi3Status {
    unsafe {
        with(c, |i| {
            counts(nvr, nValues)?;
            for (r, v) in slice(vr, nvr)?.iter().zip(slice(values, nValues)?) {
                i.set_value(*r, Value::Real(*v))?;
            }
            Ok(fmi3OK)
        })
    }
}

/// # Safety
/// See `fmi3GetFloat64`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetBoolean(
    c: fmi3Instance,
    vr: *const fmi3ValueReference,
    nvr: usize,
    values: *mut fmi3Boolean,
    nValues: usize,
) -> fmi3Status {
    unsafe {
        with(c, |i| {
            counts(nvr, nValues)?;
            for (r, v) in slice(vr, nvr)?.iter().zip(slice_mut(values, nValues)?) {
                *v = i.bool_value(i.variable(*r, "Boolean")?);
            }
            Ok(fmi3OK)
        })
    }
}

/// # Safety
/// See `fmi3GetFloat64`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetBoolean(
    c: fmi3Instance,
    vr: *const fmi3ValueReference,
    nvr: usize,
    values: *const fmi3Boolean,
    nValues: usize,
) -> fmi3Status {
    unsafe {
        with(c, |i| {
            counts(nvr, nValues)?;
            for (r, v) in slice(vr, nvr)?.iter().zip(slice(values, nValues)?) {
                i.set_value(*r, Value::Bool(*v))?;
            }
            Ok(fmi3OK)
        })
    }
}

/// The strings stay valid until the next call of this function on the instance.
///
/// # Safety
/// See `fmi3GetFloat64`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetString(
    c: fmi3Instance,
    vr: *const fmi3ValueReference,
    nvr: usize,
    values: *mut fmi3String,
    nValues: usize,
) -> fmi3Status {
    if nvr != nValues {
        return unsafe { with(c, |_| counts(nvr, nValues).map(|_| fmi3OK)) };
    }
    unsafe { get_strings(c, vr, nvr, values) }
}

/// # Safety
/// See `fmi3GetFloat64`; every string is valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetString(
    c: fmi3Instance,
    vr: *const fmi3ValueReference,
    nvr: usize,
    values: *const fmi3String,
    nValues: usize,
) -> fmi3Status {
    unsafe {
        with(c, |i| {
            counts(nvr, nValues)?;
            for (r, v) in slice(vr, nvr)?.iter().zip(slice(values, nValues)?) {
                i.set_value(*r, Value::Text(text(*v).into()))?;
            }
            Ok(fmi3OK)
        })
    }
}

/// # Safety
/// See `api::with`; the out pointers are null or writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3DoStep(
    c: fmi3Instance,
    currentCommunicationPoint: f64,
    communicationStepSize: f64,
    _noSetFMUStatePriorToCurrentPoint: fmi3Boolean,
    eventHandlingNeeded: *mut fmi3Boolean,
    terminateSimulation: *mut fmi3Boolean,
    earlyReturn: *mut fmi3Boolean,
    lastSuccessfulTime: *mut f64,
) -> fmi3Status {
    unsafe {
        for flag in [eventHandlingNeeded, terminateSimulation, earlyReturn] {
            if let Some(flag) = flag.as_mut() {
                *flag = false;
            }
        }
        let status = with(c, |i| i.do_step(currentCommunicationPoint, communicationStepSize));
        // A refused step leaves the simulation where it was.
        if let (Some(time), Some(i)) = (lastSuccessfulTime.as_mut(), (c as *const Instance).as_ref()) {
            *time = i.now();
        }
        status
    }
}

/// # Safety
/// See `fmi2GetFMUstate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetFMUState(c: fmi3Instance, state: *mut fmi3FMUState) -> fmi3Status {
    unsafe { crate::api::state_get(c, state) }
}

/// # Safety
/// See `fmi2SetFMUstate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SetFMUState(c: fmi3Instance, state: fmi3FMUState) -> fmi3Status {
    unsafe { crate::api::state_set(c, state) }
}

/// # Safety
/// See `fmi2FreeFMUstate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3FreeFMUState(c: fmi3Instance, state: *mut fmi3FMUState) -> fmi3Status {
    unsafe { crate::api::state_free(c, state) }
}

/// # Safety
/// See `fmi2SerializedFMUstateSize`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SerializedFMUStateSize(
    c: fmi3Instance,
    state: fmi3FMUState,
    size: *mut usize,
) -> fmi3Status {
    unsafe { crate::api::state_size(c, state, size) }
}

/// # Safety
/// See `fmi2SerializeFMUstate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3SerializeFMUState(
    c: fmi3Instance,
    state: fmi3FMUState,
    serializedState: *mut fmi3Byte,
    size: usize,
) -> fmi3Status {
    unsafe { crate::api::state_serialize(c, state, serializedState, size) }
}

/// # Safety
/// See `fmi2DeSerializeFMUstate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3DeserializeFMUState(
    c: fmi3Instance,
    serializedState: *const fmi3Byte,
    size: usize,
    state: *mut fmi3FMUState,
) -> fmi3Status {
    unsafe { crate::api::state_deserialize(c, serializedState, size, state) }
}

/// ∂ unknowns / ∂ knowns times the seed: zero (see `api`).
///
/// # Safety
/// See `api::with`; the pointers point to as many values as their counts say.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetDirectionalDerivative(
    c: fmi3Instance,
    unknowns: *const fmi3ValueReference,
    nUnknowns: usize,
    knowns: *const fmi3ValueReference,
    nKnowns: usize,
    _seed: *const f64,
    nSeed: usize,
    sensitivity: *mut f64,
    nSensitivity: usize,
) -> fmi3Status {
    // One seed per known, one sensitivity per unknown.
    if nSeed != nKnowns || nSensitivity != nUnknowns {
        return unsafe { with(c, |_| counts(nKnowns, nSeed).and(counts(nUnknowns, nSensitivity)).map(|_| fmi3OK)) };
    }
    unsafe { crate::api::zero_derivative(c, unknowns, nUnknowns, knowns, nKnowns, sensitivity, nSensitivity) }
}

/// The seed times ∂ unknowns / ∂ knowns: zero (see `api`).
///
/// # Safety
/// See `fmi3GetDirectionalDerivative`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi3GetAdjointDerivative(
    c: fmi3Instance,
    unknowns: *const fmi3ValueReference,
    nUnknowns: usize,
    knowns: *const fmi3ValueReference,
    nKnowns: usize,
    _seed: *const f64,
    nSeed: usize,
    sensitivity: *mut f64,
    nSensitivity: usize,
) -> fmi3Status {
    // The adjoint the other way round: one seed per unknown, one sensitivity per known.
    if nSeed != nUnknowns || nSensitivity != nKnowns {
        return unsafe { with(c, |_| counts(nUnknowns, nSeed).and(counts(nKnowns, nSensitivity)).map(|_| fmi3OK)) };
    }
    unsafe { crate::api::zero_derivative(c, unknowns, nUnknowns, knowns, nKnowns, sensitivity, nSensitivity) }
}

macro_rules! no_variables {
    ($($name:ident($type_name:literal $(, $arg:ident: $ty:ty)*);)*) => {$(
        /// The FMU has no variables of this type: any value reference is refused.
        ///
        /// # Safety
        /// See `fmi3GetFloat64`.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $name(c: fmi3Instance, vr: *const fmi3ValueReference, nvr: usize $(, $arg: $ty)*) -> fmi3Status {
            $(let _ = $arg;)*
            unsafe { with(c, |i| slice(vr, nvr)?.first().map_or(Ok(fmi3OK), |r| i.variable(*r, $type_name).map(|_| fmi3OK))) }
        }
    )*};
}

no_variables! {
    fmi3GetFloat32("Float32", values: *mut f32, n: usize);
    fmi3SetFloat32("Float32", values: *const f32, n: usize);
    fmi3GetInt8("Int8", values: *mut i8, n: usize);
    fmi3SetInt8("Int8", values: *const i8, n: usize);
    fmi3GetUInt8("UInt8", values: *mut u8, n: usize);
    fmi3SetUInt8("UInt8", values: *const u8, n: usize);
    fmi3GetInt16("Int16", values: *mut i16, n: usize);
    fmi3SetInt16("Int16", values: *const i16, n: usize);
    fmi3GetUInt16("UInt16", values: *mut u16, n: usize);
    fmi3SetUInt16("UInt16", values: *const u16, n: usize);
    fmi3GetInt32("Int32", values: *mut i32, n: usize);
    fmi3SetInt32("Int32", values: *const i32, n: usize);
    fmi3GetUInt32("UInt32", values: *mut u32, n: usize);
    fmi3SetUInt32("UInt32", values: *const u32, n: usize);
    fmi3GetInt64("Int64", values: *mut i64, n: usize);
    fmi3SetInt64("Int64", values: *const i64, n: usize);
    fmi3GetUInt64("UInt64", values: *mut u64, n: usize);
    fmi3SetUInt64("UInt64", values: *const u64, n: usize);
    fmi3GetBinary("Binary", sizes: *mut usize, values: *mut *const u8, n: usize);
    fmi3SetBinary("Binary", sizes: *const usize, values: *const *const u8, n: usize);
    fmi3GetClock("Clock", values: *mut bool);
    fmi3SetClock("Clock", values: *const bool);
    fmi3GetIntervalDecimal("Clock", intervals: *mut f64, qualifiers: *mut i32);
    fmi3GetIntervalFraction("Clock", counters: *mut u64, resolutions: *mut u64, qualifiers: *mut i32);
    fmi3GetShiftDecimal("Clock", shifts: *mut f64);
    fmi3GetShiftFraction("Clock", counters: *mut u64, resolutions: *mut u64);
    fmi3SetIntervalDecimal("Clock", intervals: *const f64);
    fmi3SetIntervalFraction("Clock", counters: *const u64, resolutions: *const u64);
    fmi3SetShiftDecimal("Clock", shifts: *const f64);
    fmi3SetShiftFraction("Clock", counters: *const u64, resolutions: *const u64);
}

macro_rules! unsupported {
    ($($name:ident($($arg:ident: $ty:ty),*);)*) => {$(
        /// Not supported: returns `fmi3Error`.
        ///
        /// # Safety
        /// See `api::with`.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $name(c: fmi3Instance, $($arg: $ty),*) -> fmi3Status {
            $(let _ = $arg;)*
            unsafe { with(c, |_| Err(("fmi-error-unsupported", stringify!($name).into()))) }
        }
    )*};
}

unsupported! {
    fmi3EnterEventMode();
    fmi3EnterStepMode();
    fmi3EnterConfigurationMode();
    fmi3ExitConfigurationMode();
    fmi3GetNumberOfVariableDependencies(vr: fmi3ValueReference, n: *mut usize);
    fmi3GetVariableDependencies(dependent: fmi3ValueReference, element: *mut usize, independents: *mut fmi3ValueReference, elements: *mut usize, kinds: *mut i32, n: usize);
    fmi3EvaluateDiscreteStates();
    fmi3UpdateDiscreteStates(needUpdate: *mut bool, terminate: *mut bool, nominalsChanged: *mut bool, valuesChanged: *mut bool, nextDefined: *mut bool, next: *mut f64);
    fmi3GetOutputDerivatives(vr: *const fmi3ValueReference, nvr: usize, orders: *const i32, values: *mut f64, n: usize);
    fmi3ActivateModelPartition(clock: fmi3ValueReference, time: f64);
    fmi3EnterContinuousTimeMode();
    fmi3CompletedIntegratorStep(noSetFMUStatePriorToCurrentPoint: bool, enterEventMode: *mut bool, terminate: *mut bool);
    fmi3SetTime(time: f64);
    fmi3SetContinuousStates(states: *const f64, n: usize);
    fmi3GetContinuousStateDerivatives(derivatives: *mut f64, n: usize);
    fmi3GetEventIndicators(indicators: *mut f64, n: usize);
    fmi3GetContinuousStates(states: *mut f64, n: usize);
    fmi3GetNominalsOfContinuousStates(nominals: *mut f64, n: usize);
    fmi3GetNumberOfEventIndicators(n: *mut usize);
    fmi3GetNumberOfContinuousStates(n: *mut usize);
}
