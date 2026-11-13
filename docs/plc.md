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

Ctrl+C stops the link and prints the measured pacing in a line of this form (the numbers are those of the soak run below, not of a Ctrl+C run):

```text
Real-time pacing (period, largest and RMS jitter in ms, overruns, real-time factor): 10.017 · 16.116 · 4.581 · 42 · 1.0000
```

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

How the blocks behave:

- A rising edge of `Execute` starts a block with the inputs it has at that moment. Writing `true` again while `Execute` is true does nothing.
- `Done`, `CommandAborted` and `Error` stay true while `Execute` is true. Falling `Execute` clears them once the block has finished. If `Execute` falls while the block is busy, the block runs on and its outputs stay until the next rising edge, so that a polling client sees them.
- The outputs are published once per loop cycle. A client that reads `Done` right after writing `Execute` can read the last command's value. The handshake that works: write `Execute` false, wait until `Done` reads false, write the inputs and `Execute` true, then poll `Busy`, `Done`, `CommandAborted` and `Error`. The Structured Text examples do this.
- Moves and Stop take over from the motion under way at the position and velocity the axis has. The block that was running reports `CommandAborted`, unless it is the same block started again: that one starts anew with cleared outputs.
- MC_Stop brakes with `Stop.Deceleration` to standstill and then reports `Done`. The axis stays in `Stopping` as long as `Stop.Execute` is true, and refuses moves there (ErrorID 2). It goes to `Standstill` when `Stop.Execute` falls.
- MC_Home moves the axis to its home position, at half the axis velocity and acceleration limits, and sets `IsHomed`. The home position is the axis value of the machine's start pose (`[start] pose` in the config). It does not search a switch and does not set a new zero: positions in the simulator are absolute.
- MC_Reset clears the axis error and leaves ErrorStop for Standstill (or Disabled when `Power.Enable` is false). After an ErrorID 7 it switches the drives on again.
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
