# PLC virtual commissioning

The OPC UA link (`sim-opcua`) runs a machine in real time and lets a PLC drive it. This page covers the PLCopen motion interface, the real-time pacing and its measurement, the Structured Text examples and the soak test. The other nodes of the link (signals, targets, setpoints, paths, cycle) are in the [user guide](user-guide.md#connect-a-plc-over-opc-ua).
Deutsche Fassung: [SPS-Inbetriebnahme](de/sps.md).

## Start the link

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --host 192.168.0.10 --cycle-ms 10
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `MACHINE` | `t-gantry` | The machine, first if given |
| `--port N` | 4840 | TCP port |
| `--host IP` | 127.0.0.1 | Address to listen on. A PLC on the network needs an address of this computer that it can reach. Give the address itself, not `0.0.0.0`: the server advertises it in its endpoint URL |
| `--lang en\|de` | en | Language of the messages and the variable labels |
| `--cycle-ms N` | 10 | Cycle of the simulation loop, 1 to 1000 ms |
| `--pki DIR` | `pki` | Folder of the server's certificate and of the trusted and rejected client certificates, relative to the working directory (see [Security](#security)) |
| `--trust-all` | off | Accept every client certificate and store it among the trusted ones. For commissioning only |
| `--user NAME` | none | Clients must log in with this user name and the password in the environment variable `SIM_OPCUA_PASSWORD`; anonymous clients are refused. Without the variable the link does not start |

Ctrl+C stops the link and prints the measured pacing in a line of this form (the numbers are those of the soak run below, not of a Ctrl+C run):

```text
Real-time pacing (period, largest and RMS jitter in ms, overruns, real-time factor): 10.017 · 16.116 · 4.581 · 42 · 1.0000
```

At the start the link also prints the absolute path of its pki folder, and whether `--trust-all` or `--user` are on. At the end it prints the intervals of the clients' writes and reads (see [The client's cycle](#the-clients-cycle)), here those of the pattern in `sim-opcua/tests/interop.rs`, nine intervals of 10 ms and one of 40 ms three times over:

```text
Client writes (intervals, mean, standard deviation, largest in ms): 30 · 13.000 · 9.000 · 40.000
Client reads (intervals, mean, standard deviation, largest in ms): 30 · 13.000 · 9.000 · 40.000
```

## Security

The server offers three endpoints, all at the same URL `opc.tcp://HOST:PORT/`:

| Security policy | Message security mode | Client certificate checked |
| --- | --- | --- |
| None | None | No |
| Basic256Sha256 | Sign | Yes |
| Basic256Sha256 | SignAndEncrypt | Yes |

Basic256Sha256 is the policy that S7-1500, TwinCAT TF6100 and the CODESYS OPC UA client all offer. The newer Aes128-Sha256-RsaOaep and Aes256-Sha256-RsaPss are not served.

**The server's certificate.** On the first start the link creates its application instance certificate in the pki folder (`--pki`, default `pki` in the working directory; `/pki` is ignored by git):

| File | What it is |
| --- | --- |
| `own/cert.der` | The certificate: self-signed, RSA 2048 bits, valid for 365 days. Application URI `urn:kinematics-simulation:server`; the alternative names hold that URI, `localhost`, the computer's name and its IP addresses |
| `private/private.pem` | Its private key. Keep it to this computer |
| `trusted/` | Client certificates the server accepts |
| `rejected/` | Client certificates the server refused |

A PLC that connects with security has to trust `own/cert.der`: import it into the PLC's certificate store as a trusted partner (TIA Portal: certificate manager of the CPU; TwinCAT TF6100 and CODESYS: see below). After a year, or after a change of host name or address that the PLC checks, delete `own/` and `private/` and start again: a new certificate is made, which the PLC has to trust anew.

**Client certificates.** A client on a secured endpoint whose certificate is not in `trusted/` is refused, and its certificate is written to `rejected/`. To trust it, move the file from `rejected/` to `trusted/`; the next connection is accepted, without a restart. A certificate that lies in `rejected/` is refused even with `--trust-all`: move or delete it first. `--trust-all` accepts every client certificate and copies it into `trusted/`, where it stays after the link is stopped: use it while commissioning, then delete from `trusted/` what should not stay. The endpoint without security does not check client certificates; it is there for tools such as UaExpert on the same computer and for the tests.

**User name and password.** With `--user NAME` every endpoint asks for that user name and the password from `SIM_OPCUA_PASSWORD`, which is read from the environment so that it does not show in the process list:

```sh
SIM_OPCUA_PASSWORD=secret cargo run --release -p sim-opcua -- t-gantry --user plc
```

(PowerShell: `$env:SIM_OPCUA_PASSWORD = "secret"` before the command.) The password is sent encrypted with the server's key (Basic256Sha256) on every endpoint, also on the one without security. Anonymous clients are then refused. There is one user; roles (read only, write) are not distinguished.

## Node ids and data types

Every variable is a scalar (ValueRank −1) in the namespace `urn:kinematics-simulation`, with a string node id that is the same at every start of the same machine: it is made from the signal, axis and block names, never numbered. The namespace index is 2 at present (0 is OPC UA's, 1 the server's application URI); a client should still look it up from the URI (`OPC_UA_NamespaceGetIndexList`, `UA_GetNamespaceIndex`, `UA_NamespaceGetIndex`). The table gives each node's OPC UA data type and the IEC 61131-3 type a PLC variable needs to read or write it; `*` stands for any text, and the first row that matches counts. A test (`sim-opcua/tests/interop.rs`) browses every variable of every machine and checks its DataType, ValueRank, AccessLevel and value against this table.

| Node id | OPC UA type | IEC 61131-3 | Access |
| --- | --- | --- | --- |
| `signal.*` | Double | LREAL | read |
| `command.target.*`, `command.setpoint.*` | Double | LREAL | read, write |
| `command.move`, `command.cycle`, `command.disable`, `command.enable`, `command.path_time_optimal`, `command.run_path`, `command.reset_pacing` | Boolean | BOOL | read, write |
| `command.path` | String | STRING | read, write |
| `status.busy` | Boolean | BOOL | read |
| `status.refused` | String | STRING | read |
| `status.overruns`, `status.cycles`, `status.steps`, `status.client_writes`, `status.client_reads` | UInt64 | ULINT | read |
| `status.*` | Double | LREAL | read |
| `axis.*.ActualPosition`, `axis.*.ActualVelocity` | Double | LREAL | read |
| `axis.*.Enable`, `axis.*.Execute` | Boolean | BOOL | read, write |
| `axis.*.Position`, `axis.*.Distance`, `axis.*.Velocity`, `axis.*.Acceleration`, `axis.*.Deceleration`, `axis.*.Jerk` | Double | LREAL | read, write |
| `axis.*.ErrorID` | UInt16 | UINT | read |
| `axis.*.StateID` | Int32 | DINT | read |
| `axis.*.State` | String | STRING | read |
| `axis.*.Status`, `axis.*.Valid`, `axis.*.Error`, `axis.*.Done`, `axis.*.Busy`, `axis.*.Active`, `axis.*.CommandAborted`, `axis.*.IsHomed` | Boolean | BOOL | read |

What the PLC clients need:

- **Siemens S7-1500** (`OPC_UA_Connect`, `OPC_UA_NamespaceGetIndexList`, `OPC_UA_NodeGetHandleList`, `OPC_UA_ReadList`, `OPC_UA_WriteList`): the variables of the lists in a data block, of the IEC types above (LReal, Bool, UInt, DInt, ULInt, String). An S7 `String` holds at most 254 characters; for a longer `command.path` use `WString`, which the client also maps to OPC UA String. The S7-1200 has no `ULInt`: read the UInt64 counters with an S7-1500 or leave them out.
- **Beckhoff TF6100** (`UA_Connect`, `UA_GetNamespaceIndex`, `UA_NodeGetHandle`, `UA_Read`, `UA_Write`): the value goes as a pointer and a size, so the PLC variable's type decides what is sent. LREAL for every Double: a REAL writes a Float, which the link accepts, but a Double read into a REAL does not fit. `STRING(255)` by default; a longer path needs a longer STRING.
- **CODESYS OPC UA client** (PLCopen blocks `UA_Connect`, `UA_NamespaceGetIndex`, `UA_NodeGetHandle`, `UA_Read`, `UA_Write`): the value is an IN_OUT of type ANY, so its declared type is the one on the wire.

The link takes on a write of a number a Double, Float, Int16, Int32 or Int64, and on a write of a flag a Boolean; anything else is refused with `BadTypeMismatch`.

## PLCopen axes

Every axis of the machine has a folder `axis.<axis id>` with one set of handshake variables per function block. Node ids are strings in the namespace `urn:kinematics-simulation`, for example `axis.x.MoveAbsolute.Execute`.

| Node | Type | Access | Meaning |
| --- | --- | --- | --- |
| `axis.<id>.Power.Enable` | Boolean | write | MC_Power.Enable: `true` powers the axis |
| `axis.<id>.Power.Status` | Boolean | read | Enable is true and the drives are on |
| `axis.<id>.Power.Valid` | Boolean | read | Status and no axis error |
| `axis.<id>.Power.Error`, `.ErrorID` | Boolean, UInt16 | read | The axis error while Enable is true |
| `axis.<id>.Home.Execute` | Boolean | write | MC_Home |
| `axis.<id>.MoveAbsolute.Execute` | Boolean | write | MC_MoveAbsolute |
| `axis.<id>.MoveAbsolute.Position` | Double | write | Target, m |
| `axis.<id>.MoveAbsolute.Velocity`, `.Acceleration`, `.Deceleration`, `.Jerk` | Double | write | Limits of the move, m/s, m/s², m/s², m/s³. They start at the axis limits |
| `axis.<id>.MoveRelative.Execute` | Boolean | write | MC_MoveRelative |
| `axis.<id>.MoveRelative.Distance` | Double | write | Distance, m, from the axis setpoint at the rising edge |
| `axis.<id>.MoveRelative.Velocity`, `.Acceleration`, `.Deceleration`, `.Jerk` | Double | write | As for MoveAbsolute |
| `axis.<id>.Stop.Execute` | Boolean | write | MC_Stop |
| `axis.<id>.Stop.Deceleration` | Double | write | Braking deceleration, m/s² |
| `axis.<id>.Reset.Execute` | Boolean | write | MC_Reset |
| `axis.<id>.<FB>.Done`, `.Busy`, `.Active`, `.CommandAborted`, `.Error` | Boolean | read | Outputs of Home, MoveAbsolute, MoveRelative, Stop and Reset. `Active` reads the same as `Busy` |
| `axis.<id>.<FB>.ErrorID` | UInt16 | read | Why the block failed, 0 without error |
| `axis.<id>.State` | String | read | `Disabled`, `Standstill`, `ErrorStop`, `Stopping`, `Homing` or `DiscreteMotion` |
| `axis.<id>.StateID` | Int32 | read | The state as a number: 1 Disabled, 2 Standstill, 3 ErrorStop, 4 Stopping, 5 Homing, 6 DiscreteMotion (TwinCAT's `MC_AxisStates`) |
| `axis.<id>.ActualPosition`, `.ActualVelocity` | Double | read | Position and velocity of the simulated axis (`signal.axis.<id>.position`, `.velocity`) |
| `axis.<id>.IsHomed` | Boolean | read | MC_Home has finished once |
| `axis.<id>.Error`, `.ErrorID` | Boolean, UInt16 | read | The axis error. Not 0 means ErrorStop |

| ErrorID | Meaning |
| --- | --- |
| 1 | The axis is not powered (Disabled) |
| 2 | Not allowed in this state: any block but Reset in ErrorStop, a move in Stopping or Homing, Home in DiscreteMotion |
| 3 | A parameter is not a positive finite number |
| 4 | The target lies outside the axis range |
| 5 | A move, path or the cycle of the link's other commands is under way |
| 6 | The controller refused a streamed setpoint (axis error) |
| 7 | The drives were switched off while the axis was powered: `command.disable` or a DC-link trip (axis error) |
| 8 | The drives went off because the machine ran into an obstacle (`effects.crash`, axis error) |

How the blocks behave:

- A rising edge of `Execute` starts a block with the inputs it has at that moment. Writing `true` again while `Execute` is true does nothing.
- `Done`, `CommandAborted` and `Error` stay true while `Execute` is true. Falling `Execute` clears them once the block has finished. If `Execute` falls while the block is busy, the block runs on and its outputs stay until the next rising edge, so that a polling client sees them.
- The outputs are published once per loop cycle. A client that reads `Done` right after writing `Execute` can read the last command's value. The handshake that works: write `Execute` false, wait until `Done` reads false, write the inputs and `Execute` true, then poll `Busy`, `Done`, `CommandAborted` and `Error`. The Structured Text examples do this.
- Moves and Stop take over from the motion under way at the position and velocity the axis has. The block that was running reports `CommandAborted`, unless it is the same block started again: that one starts anew with cleared outputs.
- MC_Stop brakes with `Stop.Deceleration` to standstill and then reports `Done`. The axis stays in `Stopping` as long as `Stop.Execute` is true, and refuses moves there (ErrorID 2). It goes to `Standstill` when `Stop.Execute` falls.
- MC_Home moves the axis to its home position, at half the axis velocity and acceleration limits, and sets `IsHomed`. The home position is the axis value of the machine's start pose (`[start] pose` in the config). It does not search a switch and does not set a new zero: positions in the simulator are absolute.
- MC_Reset clears the axis error and leaves ErrorStop for Standstill (or Disabled when `Power.Enable` is false). After an ErrorID 7 or 8 it switches the drives on again; after a crash the axis can then be moved out of the obstacle, and a move further into it trips the drives again.
- MC_Power: the controller has one enable for all drives. The drives go on when an axis is powered, and off when the last axis is switched off. An axis whose `Power.Enable` is false is Disabled and refuses blocks, but its motor still holds position while another axis keeps the drives on. Switching an axis off aborts its motion.
- Parameters above the axis limits are cut to the limits. The profile is a trapezoid: velocity, acceleration and deceleration are kept, `Jerk` is accepted and not used.
- Do not mix the PLCopen blocks with `command.setpoint.*`, `command.move`, `command.cycle` or `command.run_path` on the same machine at the same time. Blocks started while those run fail with ErrorID 5; a stream started while blocks run makes the controller refuse one of the two.

### How it works, and its limits

The core controller moves all axes together on one synchronised profile, refuses a new move while one runs, and `Command::Stop` stops only the cycle after its segment. A PLCopen axis needs more: axes that move independently and at the same time, MC_Stop that brakes in the middle of a move, and a new move that aborts the old one. So the link runs one profile per axis itself and streams the positions of all axes to the controller with `Command::Setpoint`, one setpoint per `profile.setpoint_interval_s` (10 ms), counted in simulation steps. This is how a PLC drives servo drives over a fieldbus. The controller interpolates linearly between the setpoints and closes the servo loops.

Limits:

- The streamed profile has a step of 10 ms. Within a step the velocity is constant, so acceleration changes at each step. There is no jerk limitation and no input shaping of the PLCopen moves.
- A move that aborts a fast move towards a nearer target, or reverses, brakes with `Deceleration` and can overshoot. If the overshoot leaves the axis range, the controller refuses the setpoint and the axis goes to ErrorStop (ErrorID 6).
- No buffered or blended moves (`BufferMode`), no MC_MoveVelocity, MC_Halt, MC_ReadParameter, gearing or camming.
- The link adds the latency of OPC UA: a client sees the effect of a write after one to two loop cycles plus its own polling time.

## Real-time pacing and jitter

The simulation loop wakes on a fixed grid of deadlines, one per `--cycle-ms`. At each wake-up it carries out the commands that arrived, then runs as many simulation steps as the wall-clock time since the last wake-up allows (62.5 µs each for the machines here). So the number of steps follows the wall clock, not the number of wake-ups: a late wake-up runs more steps. A wake-up more than one cycle late is not made up for with a burst of extra wake-ups. If the host falls further behind than 0.1 s or two cycles, whichever is longer, the simulation drops the time beyond that instead of piling up work.

The link measures the time between wake-ups since the start or since `command.reset_pacing`:

| Node | Type | Meaning |
| --- | --- | --- |
| `status.cycle_s` | Double | The nominal cycle, s |
| `status.cycle_period_s` | Double | Mean measured time between wake-ups, s |
| `status.jitter_max_s` | Double | Largest deviation of one period from the cycle, s |
| `status.jitter_rms_s` | Double | Root mean square of that deviation, s |
| `status.overruns` | UInt64 | Wake-ups that came more than a whole cycle late (period over twice the cycle) |
| `status.cycles` | UInt64 | Wake-ups measured |
| `status.realtime_factor` | Double | Simulated time over wall-clock time |
| `status.steps` | UInt64 | Simulation steps since the start |
| `command.reset_pacing` | Boolean, write | `true` starts the measurement anew |

On Windows the runtime's timer follows the system clock of 15.6 ms. With a 10 ms cycle the wake-ups therefore come 0 to 15.6 ms late, which shows as jitter of several milliseconds while the mean period and the real-time factor stay exact. A dedicated thread with the standard library's high-resolution sleep was tried and was worse on a loaded host. On Linux the timer has a resolution of about 1 ms. Neither is a real-time system: other load on the host shows directly in the jitter.

### The client's cycle

Next to its own loop, the link measures how regularly the clients reach it: the time between one Read request to the simulator's variables and the next, and the same for Write requests. A PLC that reads and writes once in each task cycle shows its cycle time and its jitter here, as the server sees them: the PLC task's jitter, plus that of the network and of this computer's scheduling.

| Node | Type | Meaning |
| --- | --- | --- |
| `status.client_writes`, `status.client_reads` | UInt64 | Intervals measured (one fewer than the requests) |
| `status.client_write_mean_s`, `status.client_read_mean_s` | Double | Mean interval, s |
| `status.client_write_std_s`, `status.client_read_std_s` | Double | Standard deviation of the intervals (of the population, divided by their number), s |
| `status.client_write_max_s`, `status.client_read_max_s` | Double | Largest interval, s |

- One request counts once, however many variables it reads or writes. A PLC that reads two variables with two `UA_Read` blocks in the same cycle sends two requests and shows intervals of half a cycle: read with one request per cycle (`OPC_UA_ReadList`, or one block) while measuring. The Structured Text example `sim_cycle.st` has a switch for that.
- Only reads of variable values count; browsing (the other attributes) and subscriptions do not. A read is counted after it is answered, so a read of these variables does not appear in its own answer.
- All clients count together: while measuring, connect only the PLC (close UaExpert).
- `command.reset_pacing` starts these figures anew together with the loop's, in the write itself.
- The link prints them when it stops (see [Start the link](#start-the-link)).

## Structured Text examples

[`crates/sim-opcua/examples/plc/twincat/sim_plcopen.st`](../crates/sim-opcua/examples/plc/twincat/sim_plcopen.st) is for TwinCAT 3 with the library Tc3_PLCopen_OpcUa (UA_Connect, UA_GetNamespaceIndex, UA_NodeGetHandle, UA_Read, UA_Write, UA_NodeReleaseHandle, UA_Disconnect). [`crates/sim-opcua/examples/plc/codesys/sim_plcopen.st`](../crates/sim-opcua/examples/plc/codesys/sim_plcopen.st) is for CODESYS with the CODESYS OPC UA Client SL, which provides the PLCopen OPC UA client blocks.

Both files contain:

| Block | What it does |
| --- | --- |
| `SIM_Access` | One read or write of one variable: get the node handle, read or write the value, release the handle |
| `SIM_Handshake` | The handshake of one block on the simulator, as described above |
| `SIM_MC_Power`, `SIM_MC_Home`, `SIM_MC_MoveAbsolute`, `SIM_MC_MoveRelative`, `SIM_MC_Stop`, `SIM_MC_Reset` | Wrappers with the PLCopen inputs and outputs, plus the connection handle, the namespace index and the axis id |
| `PRG_SimDemo` | Connects, powers the axis, homes it, starts a move, stops it after 0.5 s, switches off and disconnects |

**These files were not compiled.** Neither TwinCAT nor CODESYS was available here. They follow the libraries' documentation and IEC 61131-3; names that may differ between library versions are marked `CHECK`. Every access gets and releases a node handle, which costs three round trips; production code keeps the handles. Adapt the server URL, the axis id and the positions to the machine.

`sim_cycle.st` in each folder adds `PRG_SimCycle`, a program for a 10 ms task that does what a PLC program does against the machine: connect with Basic256Sha256 SignAndEncrypt (or None) and, if set, user name and password; find the namespace index; switch the drives on with MC_Power; run MC_MoveAbsolute; write a path (`command.path`) and run it (`command.run_path`) until `status.busy` falls; switch off and disconnect. Meanwhile `SIM_CyclicRead` reads the axis position (`signal.axis.<id>.position`) and following error (`signal.axis.<id>.following_error`) every cycle through node handles that are got once, and the program keeps the largest following error. It uses the blocks of `sim_plcopen.st`. The path is written for the T-gantry (axes x and z, start pose x 0.1 m, z 0.35 m); change it and the target for another machine.

## Run the examples against sim-opcua on the same PC

Both need the PLC runtime on this computer; nothing of it ships with this project. The steps were written from the vendors' documentation and could not be tried here.

**1. Start the link** in a terminal, in the repository folder, with `--trust-all` for the first connection:

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --trust-all
```

It listens on `opc.tcp://127.0.0.1:4840/`, the URL in `PRG_SimCycle`, and prints the pki folder.

**2a. TwinCAT 3 (TF6100).**

1. In TwinCAT XAE (Visual Studio), make a new TwinCAT XAE project and add a PLC project (Standard PLC Project).
2. Under References add `Tc3_PLCopen_OpcUa`. TF6100 needs a licence; a 7-day trial licence can be activated under System > License.
3. Add the code: TwinCAT does not import `.st` files directly. For each `FUNCTION_BLOCK` and `PROGRAM` in `twincat/sim_plcopen.st` and `twincat/sim_cycle.st` add a POU of that name and paste the declaration (from the header to `END_VAR`) into its upper part and the body into the lower part. Put `E_SimKind` into a DUT.
4. Call `PRG_SimCycle` from the PLC task and set the task's cycle to 10 ms.
5. Activate the configuration, set the target to run mode (the local runtime needs a free core or Hyper-V settings as Beckhoff describes), log in and start.
6. TF6100 makes a client certificate on the first secured connection and checks the server's: if the connection fails with a certificate error, trust `pki/own/cert.der` in TF6100's PKI folder (see the TF6100 manual: it moves an unknown server certificate to its `rejected` folder, from where it is moved to `trusted`).
7. Watch `nStep` go to 100, `fbPosition.fValue` follow the move and the path, and `fLargestError`. In the link's terminal the client certificate now lies in `pki/trusted`: from now on start the link without `--trust-all`.

**2b. CODESYS Control Win.**

1. Install CODESYS Development System V3.5 with CODESYS Control Win (SoftPLC), start the SoftPLC from its tray icon.
2. Make a Standard project for the device "CODESYS Control Win V3 x64" and add the library "OPC UA Client" (CODESYS OPC UA Client SL) in the Library Manager. Without a licence it runs in demo mode for a limited time.
3. Add a POU for each block of `codesys/sim_plcopen.st` and `codesys/sim_cycle.st` and paste the declaration and the body; `E_SimKind` into a DUT.
4. In the task configuration make a cyclic task of 10 ms that calls `PRG_SimCycle`.
5. For the secured connection create the OPC UA client's certificate in the Security Screen and trust the server's (`pki/own/cert.der`) there; names differ between versions (`CHECK` in the file). Or set `xSecure := FALSE` for a first test on the endpoint without security.
6. Log in, download, start. Watch `iStep` go to 100, `fbPosition.lrValue` and `lrLargestError`.

**3. Read the cycle jitter.** Set `bReadFollowingError` (TwinCAT) or `xReadFollowingError` (CODESYS) to FALSE, so that the program sends one Read request per cycle, and stop the link with Ctrl+C after the program has run: the line `Client reads ...` gives the number of intervals, their mean (the task cycle, 10 ms, when each read returns within a cycle; 20 ms when it takes up to two), their standard deviation and the largest. While the program runs, the same figures are in `status.client_read_*`.

## Soak test

```sh
cargo test -p sim-opcua --release --test soak -- --nocapture
SOAK_SECONDS=3600 cargo test -p sim-opcua --release --test soak -- --nocapture
```

The test serves the T-gantry and moves both axes together between 30 % and 70 % of their range with MC_MoveAbsolute, as fast as the handshake allows, for `SOAK_SECONDS` (default 20). It fails if:

- a block or an axis reports an error, or `status.refused` is not empty;
- an axis stands more than 0.2 mm from its target after a move;
- the real-time factor is off 1 by more than 2 %, or the step count differs from the wall time by more than 1 s plus 2 %;
- the mean period is off the cycle by more than 2 ms, the RMS jitter exceeds 10 ms, the largest jitter 100 ms, or more than 10 % of the wake-ups are overruns;
- resident memory grows by more than 64 MB (measured on Linux only).

Result of a 20 s run on Windows 11 with all cores busy with other builds: 26 moves, largest miss 0.084 mm, mean period 10.017 ms, largest jitter 16.1 ms, RMS jitter 4.6 ms, 42 overruns in 2030 cycles, real-time factor 1.0000, 328898 steps. Memory is not measured on Windows. Runs differ in jitter with the load on the host.

## Fieldbus

A simulated drive on EtherCAT or PROFINET, instead of OPC UA, is discussed in [Fieldbus emulation](fieldbus-emulation.md).
