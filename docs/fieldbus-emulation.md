# Fieldbus emulation: a simulated drive on EtherCAT or PROFINET

Design note. The question: can the simulator appear to a real PLC as a servo drive on EtherCAT or PROFINET, so that the PLC's own motion stack (TwinCAT NC, CODESYS SoftMotion, S7-1500 technology objects) drives it, as it would drive a real drive? Today the PLC talks to the simulator over OPC UA ([PLC virtual commissioning](plc.md)), which is acyclic and far slower than a drive bus.
Deutsche Fassung: [Feldbus-Emulation](de/feldbus-emulation.md).

Result: possible, but not with the simulator alone. EtherCAT needs slave hardware; PROFINET RT class 1 works with a standard network card on Linux, but needs a 1 ms real-time loop. Neither was built here. Facts were checked on 2026-11-13; sources are linked.

## What a drive on the bus must do

| | EtherCAT | PROFINET |
| --- | --- | --- |
| Drive profile | CiA 402 over CoE (ETG.6010). For cyclic synchronous position mode (CSP): controlword 0x6040, statusword 0x6041, modes of operation 0x6060/0x6061, target position 0x607A, position actual 0x6064, velocity actual 0x606C, and the CiA 402 state machine ([Nanotec manual](https://en.nanotec.com/products/manual/c5-usb-en/modes%2Fcyc_sync_position.html)) | PROFIdrive. Telegram 1 or 3 (speed setpoint, position loop in the PLC), telegram 105 for S7-1500 positioning axes with DSC (position error XERR and gain KPC from the PLC, position loop in the drive). The word layouts of telegrams 3, 102, 105 and 111 were not checked against a Siemens list manual here |
| Device description | ESI (XML), vendor ID from the ETG (free with membership) | GSDML, vendor ID from PI |
| Cycle | 250 µs to 1 ms, distributed clocks with SYNC0 jitter under 100 ns ([Synapticon](https://www.synapticon.com/en/motion-control-academy/ethercat-fuer-servoantriebe-praktischer-leitfaden)) | RT class 1: 1 to 4 ms. IRT (isochronous, needed for DSC in isochronous mode) needs special hardware |
| Conformance | Vendors must test with the ETG conformance tool ([ETG](https://www.ethercat.org/en/conformance_for_vendors.htm)); not needed for a lab device | Certification by PI; not needed for a lab device |

## EtherCAT

An EtherCAT slave processes each frame on the fly in hardware, the EtherCAT Slave Controller (ESC). A standard network card cannot do that, so a conformant slave needs an ESC.

| Option | What it is | Licence, cost |
| --- | --- | --- |
| SOES on an ESC board | rt-labs' open-source slave stack. Hardware layers for LAN9252 (also on a Raspberry Pi), XMC4, TI Sitara ([repository](https://github.com/OpenEtherCATsociety/SOES)) | The repository's licence file says GPLv2 with a linking exception, rt-labs' product page says GPLv3 for evaluation and a commercial licence from €490 a year ([rt-labs](https://rt-labs.com/product/soes/)). Board about €50 to €200 |
| Beckhoff FC1100 | PCI card with an ET1100 ESC; a PC becomes a slave. Real-time "limited" per Beckhoff ([FC1100](https://www.beckhoff.com/fc1100)) | Commercial |
| Hilscher cifX with NXLFW-ECS firmware | PC card as a certified slave ([Hilscher](https://www.hilscher.com/products/multiprotocol-socs-stacks/stacks-for-industrial-ethernet-fieldbus/nxlfw-ecs)) | Commercial, card a few hundred euros |
| Beckhoff EL6695 bridge | Exchanges process data between two masters, with DC ([Beckhoff](https://infosys.beckhoff.com/content/1033/el6695/1317490955.html)). The simulator would need its own master (e.g. SOEM) | Terminal plus a second master; SOEM 2.x is GPLv3 or commercial |
| TwinCAT TE1111 EtherCAT Simulation | A second PC with TwinCAT emulates the slaves of a configuration on a standard network card, with DC and CoE ([Beckhoff](https://www.beckhoff.com/te1111)). Whether an outside program can feed its process data was not found | TwinCAT licence per simulated device |
| acontis EC-Simulator HiL | Emulates slaves on a standard network card, against TwinCAT and CODESYS masters, can run an FMU ([acontis](https://public.acontis.com/manuals/EC-Simulator/3.2/html/ec-simulator-hil/intro.html)) | Commercial. The simulator's FMU export (sim-fmi) could plug in here |
| Raw-socket slave on a network card | Software slave at the end of the line (e.g. the old ecslave project). No on-the-fly processing, no DC accuracy | Not conformant; useful only against tolerant masters |

SOEM is a master, not a slave. It is useful only on the simulator side of an EL6695 bridge.

## PROFINET

A PROFINET RT device sends and receives ordinary Ethernet frames, so a standard network card on Linux is enough for RT class 1.

| Option | What it is | Licence, cost |
| --- | --- | --- |
| p-net | rt-labs' open-source PROFINET device stack. Conformance class A/B, RT class 1, no IRT. Linux with any network card that can send raw frames ([rt-labs](https://rt-labs.com/product/p-net/)) | GPLv3 or commercial. No hardware cost |
| pnio (Rust) | Pure-Rust PROFINET RT device, Linux, one connection, reported to work with an S7-1515 ([docs.rs](https://docs.rs/crate/pnio/0.1.0)) | MIT/Apache-2.0. Version 0.1.0 from August 2026; its maturity was not checked |
| Hilscher cifX with NXLFW-PNS | PC card as a PROFINET device | Commercial, card a few hundred euros |
| Siemens SIMIT Unit PN | Hardware that emulates up to 128 or 256 PROFINET devices against a real controller, with drive behaviour per telegram; cycle down to 1 ms from SIMIT V10 ([Siemens](https://cache.industry.siemens.com/dl/files/317/109759317/att_956837/v1/SIMIT_enUS_en-US.pdf)) | Commercial |

Siemens PN Driver is a controller stack, not a device stack. PLCSIM Advanced does not simulate PROFINET IO with outside devices.

## Real-time constraints against this simulator

The simulator steps its physics at 62.5 µs, which is fast enough: one step costs 3 µs of CPU for the T-gantry (50 ms per simulated second, measured). What does not fit is the pacing. The link wakes every 10 ms (configurable down to 1 ms) on the operating system's timer and runs the steps owed in a burst. A drive on a bus must answer every bus cycle:

- PROFINET RT class 1 at 1 to 4 ms: needs a loop that wakes every bus cycle with jitter well under a cycle. On Linux with PREEMPT_RT and a pinned thread this is ordinary practice. On Windows it is not reachable without a real-time extension.
- EtherCAT with DC at 250 µs to 1 ms: the ESC takes the timing; the simulator must deliver the next actual position before the next SYNC0, every cycle. With an ESC on SPI (LAN9252) that is a hard real-time loop on Linux at 1 kHz or faster.
- The simulator's own setpoint interpolation (`profile.setpoint_interval_s`) would become the bus cycle, and its fieldbus delay and jitter effects (`profile.fieldbus_delay_cycles`, `fieldbus_jitter_cycles`) would be replaced by the real bus.

## Licences

The project is MIT/Apache-2.0. GPLv3 stacks (p-net, SOEM 2.x, SOES as rt-labs now states it) must not be linked into it. They can run as a separate process that talks to the simulator over a socket or shared memory; that keeps the simulator's licence, but the separate program is GPLv3. The Rust crate pnio is MIT/Apache-2.0 and could be linked.

## Recommended path

1. PROFINET first: a Linux PC with PREEMPT_RT, a second network port, and a device process (pnio, or p-net as a separate GPL process) that exposes one PROFIdrive telegram 3 drive per axis and exchanges setpoints and actual values with the simulator every 1 ms over shared memory. The simulator side needs a 1 ms real-time loop in place of the OPC UA link's 10 ms tick, and a PROFIdrive state machine (STW1/ZSW1). No hardware beyond a network card. This is the cheapest route to an S7-1500 or a CODESYS controller.
2. EtherCAT second: a LAN9252 board with SOES as a CiA 402 CSP slave, its process data passed to the simulator over SPI or a socket, with the simulator on Linux at 1 kHz. Or, for TwinCAT users with a second PC, TE1111 or the acontis EC-Simulator with the FMU from sim-fmi, if a licence is available.
3. Keep OPC UA with PLCopen blocks for virtual commissioning of sequences, where 10 ms and some jitter do not matter.

## Is it tractable here?

Not in this step. Each route needs hardware or a Linux real-time host to test against a real PLC, a GPLv3 boundary or a young crate, a new real-time loop in place of the link's tick, and a drive profile state machine. Built without a PLC and a bus to test it on, it would be untested code. It was therefore not built. The PLCopen interface over OPC UA is the part that can be tested here, and it is.
