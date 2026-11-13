# Security policy

## Reporting a vulnerability

Please do not open a public issue. Report it privately through
[GitHub's private vulnerability reporting](https://github.com/Sparkz400/KinematicsSimulation/security/advisories/new)
with the steps to reproduce. You should get an answer within two weeks.

## Scope

Only the latest release is supported.

The part most exposed is the OPC UA server (`sim-opcua`). It listens on `127.0.0.1` only and has no authentication;
do not forward its port to an untrusted network.

The simulator is not a safety function. Its results must not be used as the only check before running a real machine.
