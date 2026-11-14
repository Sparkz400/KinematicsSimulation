# Security policy

## Reporting a vulnerability

Please do not open a public issue. Report it privately through
[GitHub's private vulnerability reporting](https://github.com/Sparkz400/KinematicsSimulation/security/advisories/new)
with the steps to reproduce. You should get an answer within two weeks.

## Scope

Only the latest release is supported.

The part most exposed is the OPC UA server (`sim-opcua`). It listens on `127.0.0.1` unless `--host` names another
address. It offers Basic256Sha256, Aes128_Sha256_RsaOaep and Aes256_Sha256_RsaPss with Sign and SignAndEncrypt,
accepts only client certificates in its `trusted` folder (unless `--trust-all`), and can require a user name and
password or a user certificate (`--user`, `--user-cert`); without those, anonymous clients get in, and so does any
client on the endpoint without security.

Known residual risk: its OPC UA library, async-opcua 0.19.0, uses `rsa` 0.9, whose RSA decryption is not
constant-time (RUSTSEC-2023-0071, the "Marvin" timing side channel). The server decrypts with its private key on
every secured endpoint, in Sign as well as SignAndEncrypt mode, before it checks the client's certificate. Someone
who can reach the port and time very many handshakes might recover the server's key. No async-opcua release avoids
it yet (checked 14 November 2026). Keep the port off untrusted networks (loopback by default, a firewall that admits
only the PLC otherwise), and replace the key with `--renew-certificate` when exposure is suspected. Details in
[docs/plc.md](docs/plc.md#security).

The simulator is not a safety function. Its results must not be used as the only check before running a real machine.
