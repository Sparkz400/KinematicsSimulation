//! What a real PLC's OPC UA client meets: the secured endpoints and the trust of client certificates, the data types
//! of the variables against the table in docs/plc.md, and the server's measurement of the client's cycle.
//!
//! No independent client implementation was on the development machine (no Python `asyncua`/`opcua`, no UaExpert, no
//! TwinCAT or CODESYS), so the client here is async-opcua's own, as in the other tests.

use opcua::client::{ClientBuilder, IdentityToken, Session};
use opcua::crypto::SecurityPolicy;
use opcua::types::{
    AttributeId, BrowseDescription, BrowseDirection, BrowseResultMask, DataTypeId, DataValue, MessageSecurityMode,
    NodeClass, NodeId, NumericRange, ObjectId, QualifiedName, ReadValueId, ReferenceTypeId, TimestampsToReturn,
    Variant, WriteValue,
};
use sim_machines::{MACHINES, translations};
use sim_opcua::{Link, NAMESPACE, Security, serve_secured};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

fn scratch(name: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join("interop").join(name);
    // A fresh folder every run: certificates trusted or rejected by an earlier run would change the outcome.
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

/// The first machine's server on a free local port, its certificates in `pki`; the URL to connect to.
async fn start(pki: &Path, security: Security) -> (Link, String) {
    start_machine(MACHINES[0], pki, security).await
}

async fn start_machine(entry: &'static sim_core::MachineEntry, pki: &Path, security: Security) -> (Link, String) {
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("opc.tcp://{}/", listener.local_addr().unwrap());
    let labels = translations("en", entry);
    let link = serve_secured(entry, &entry.config(), &labels, listener, pki, Duration::from_millis(10), &security)
        .await
        .unwrap();
    (link, url)
}

/// A client with its own certificate in `pki`, connecting to `url` with `policy` and `mode` as `identity`; `None`
/// when the server does not let it in within 10 s.
async fn connect(
    url: &str,
    pki: &Path,
    policy: SecurityPolicy,
    mode: MessageSecurityMode,
    identity: IdentityToken,
) -> Option<Arc<Session>> {
    let mut client = ClientBuilder::new()
        .application_name("test-plc")
        .application_uri("urn:test-plc")
        .pki_dir(pki)
        .create_sample_keypair(true)
        .trust_server_certs(true)
        .session_retry_limit(0)
        .client()
        .unwrap();
    let (session, event_loop) =
        client.connect_to_matching_endpoint((url, policy.to_str(), mode), identity).await.ok()?;
    event_loop.spawn();
    match tokio::time::timeout(Duration::from_secs(10), session.wait_for_connection()).await {
        Ok(true) => Some(session),
        _ => None,
    }
}

fn node(namespace: u16, name: &str) -> NodeId {
    NodeId::new(namespace, name.to_string())
}

async fn read(session: &Session, namespace: u16, name: &str) -> Variant {
    let id: ReadValueId = node(namespace, name).into();
    let value = session.read(&[id], TimestampsToReturn::Neither, 0.0).await.unwrap().remove(0);
    assert!(value.status.is_none_or(|s| s.is_good()), "{name}: {:?}", value.status);
    value.value.unwrap()
}

async fn write(session: &Session, namespace: u16, name: &str, value: Variant) -> bool {
    let write = WriteValue {
        node_id: node(namespace, name),
        attribute_id: AttributeId::Value as u32,
        index_range: NumericRange::None,
        value: DataValue { value: Some(value), ..Default::default() },
    };
    session.write(&[write]).await.unwrap()[0].is_good()
}

/// The files in a folder of the server's pki.
fn certificates(dir: PathBuf) -> Vec<PathBuf> {
    std::fs::read_dir(dir).map(|d| d.filter_map(|e| Some(e.ok()?.path())).collect()).unwrap_or_default()
}

/// The server offers None, and Basic256Sha256 with Sign and with SignAndEncrypt. A client whose certificate the
/// server does not know is refused on the secured endpoints, and its certificate lands in `pki/rejected`; moved to
/// `pki/trusted`, as the commissioning engineer does, the same client connects with SignAndEncrypt, reads and writes.
#[tokio::test(flavor = "multi_thread")]
async fn a_trusted_client_reads_and_writes_with_basic256sha256_sign_and_encrypt() {
    let (server_pki, client_pki) = (scratch("trusted-server"), scratch("trusted-client"));
    let (link, url) = start(&server_pki, Security::default()).await;
    let ns = link.namespace;

    let endpoints = ClientBuilder::new()
        .application_name("probe")
        .application_uri("urn:probe")
        .pki_dir(scratch("trusted-probe"))
        .client()
        .unwrap()
        .get_server_endpoints_from_url(url.as_str())
        .await
        .unwrap();
    let mut offered: Vec<(String, MessageSecurityMode)> =
        endpoints.iter().map(|e| (e.security_policy_uri.to_string(), e.security_mode)).collect();
    offered.sort_by_key(|(uri, mode)| (uri.clone(), *mode as i32));
    let basic = SecurityPolicy::Basic256Sha256.to_uri().to_string();
    assert_eq!(
        offered,
        [
            (basic.clone(), MessageSecurityMode::Sign),
            (basic, MessageSecurityMode::SignAndEncrypt),
            (SecurityPolicy::None.to_uri().to_string(), MessageSecurityMode::None),
        ]
    );

    let secured = (SecurityPolicy::Basic256Sha256, MessageSecurityMode::SignAndEncrypt);
    assert!(connect(&url, &client_pki, secured.0, secured.1, IdentityToken::Anonymous).await.is_none());
    let rejected = certificates(server_pki.join("rejected"));
    assert_eq!(rejected.len(), 1, "{rejected:?}");
    // Trust it the way docs/plc.md says: move the file from rejected/ to trusted/.
    let trusted = server_pki.join("trusted").join(rejected[0].file_name().unwrap());
    std::fs::rename(&rejected[0], &trusted).unwrap();

    let session = connect(&url, &client_pki, secured.0, secured.1, IdentityToken::Anonymous).await.unwrap();
    assert!(matches!(read(&session, ns, "status.busy").await, Variant::Boolean(false)));
    assert!(write(&session, ns, "command.path_time_optimal", true.into()).await);
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(matches!(read(&session, ns, "command.path_time_optimal").await, Variant::Boolean(true)));
    // Sign alone works as well.
    let signed =
        connect(&url, &client_pki, SecurityPolicy::Basic256Sha256, MessageSecurityMode::Sign, IdentityToken::Anonymous);
    assert!(signed.await.is_some());
}

/// `trust_all` (`--trust-all`) lets an unknown client in and stores its certificate among the trusted ones.
#[tokio::test(flavor = "multi_thread")]
async fn trust_all_lets_an_unknown_client_in() {
    let (server_pki, client_pki) = (scratch("trust-all-server"), scratch("trust-all-client"));
    let (link, url) = start(&server_pki, Security { trust_all: true, user: None }).await;
    let session = connect(
        &url,
        &client_pki,
        SecurityPolicy::Basic256Sha256,
        MessageSecurityMode::SignAndEncrypt,
        IdentityToken::Anonymous,
    )
    .await
    .expect("refused although trust_all");
    assert!(matches!(read(&session, link.namespace, "status.busy").await, Variant::Boolean(false)));
    assert_eq!(certificates(server_pki.join("trusted")).len(), 1);
    assert!(certificates(server_pki.join("rejected")).is_empty());
}

/// With a user name and password, anonymous clients and a wrong password are refused, the right one gets in, on the
/// encrypted endpoint and on the one without security (where the password still goes encrypted with the server's key).
#[tokio::test(flavor = "multi_thread")]
async fn a_user_name_and_password_keep_others_out() {
    let (server_pki, client_pki) = (scratch("user-server"), scratch("user-client"));
    let security = Security { trust_all: true, user: Some(("plc".into(), "secret".into())) };
    let (link, url) = start(&server_pki, security).await;
    let encrypted = (SecurityPolicy::Basic256Sha256, MessageSecurityMode::SignAndEncrypt);
    let user = |password: &str| IdentityToken::new_user_name("plc", password);
    assert!(connect(&url, &client_pki, encrypted.0, encrypted.1, IdentityToken::Anonymous).await.is_none());
    assert!(connect(&url, &client_pki, encrypted.0, encrypted.1, user("wrong")).await.is_none());
    let session = connect(&url, &client_pki, encrypted.0, encrypted.1, user("secret")).await.unwrap();
    assert!(matches!(read(&session, link.namespace, "status.busy").await, Variant::Boolean(false)));
    let plain = connect(&url, &client_pki, SecurityPolicy::None, MessageSecurityMode::None, user("secret")).await;
    assert!(plain.is_some());
}

/// One row of the table "Node ids and data types" in docs/plc.md: the node id patterns (`*` any text), the OPC UA
/// type, the IEC 61131-3 type and whether a client may write.
struct Row {
    patterns: Vec<String>,
    opc: String,
    iec: String,
    writable: bool,
}

fn type_table() -> Vec<Row> {
    let doc = include_str!("../../../docs/plc.md").replace("\r\n", "\n");
    let section = doc.split("\n## Node ids and data types\n").nth(1).expect("docs/plc.md: no section");
    section
        .lines()
        .skip_while(|l| !l.starts_with('|'))
        .take_while(|l| l.starts_with('|'))
        .skip(2)
        .map(|line| {
            let cells: Vec<&str> = line.split('|').map(str::trim).collect();
            let patterns = cells[1].split('`').skip(1).step_by(2).map(String::from).collect();
            Row { patterns, opc: cells[2].into(), iec: cells[3].into(), writable: cells[4] == "read, write" }
        })
        .collect()
}

/// `name` matches `pattern`, where `*` stands for any text.
fn matches(pattern: &str, name: &str) -> bool {
    let parts: Vec<&str> = pattern.split('*').collect();
    let (first, last) = (parts[0], parts[parts.len() - 1]);
    if parts.len() == 1 {
        return name == pattern;
    }
    if !name.starts_with(first) || !name[first.len()..].ends_with(last) || name.len() < first.len() + last.len() {
        return false;
    }
    let mut rest = &name[first.len()..name.len() - last.len()];
    for part in &parts[1..parts.len() - 1] {
        match rest.find(part) {
            Some(at) => rest = &rest[at + part.len()..],
            None => return false,
        }
    }
    true
}

/// Every variable of the simulator's namespace, browsed from the Objects folder as a PLC's engineering tool does.
async fn variables(session: &Session, namespace: u16) -> Vec<NodeId> {
    let (mut to_browse, mut found) = (vec![NodeId::from(ObjectId::ObjectsFolder)], Vec::new());
    while let Some(parent) = to_browse.pop() {
        let description = BrowseDescription {
            node_id: parent,
            browse_direction: BrowseDirection::Forward,
            reference_type_id: ReferenceTypeId::Organizes.into(),
            include_subtypes: true,
            node_class_mask: 0,
            result_mask: BrowseResultMask::All as u32,
        };
        let mut result = session.browse(&[description], 0, None).await.unwrap().remove(0);
        loop {
            for reference in result.references.iter().flatten() {
                let id = reference.node_id.node_id.clone();
                if id.namespace != namespace {
                    continue;
                }
                match reference.node_class {
                    NodeClass::Variable => found.push(id),
                    NodeClass::Object => to_browse.push(id),
                    _ => {}
                }
            }
            if result.continuation_point.is_null() {
                break;
            }
            result = session.browse_next(false, &[result.continuation_point.clone()]).await.unwrap().remove(0);
        }
    }
    found
}

/// The IEC 61131-3 type a PLC declares for an OPC UA type, as Siemens, Beckhoff and CODESYS map them.
fn iec(opc: &str) -> &'static str {
    match opc {
        "Boolean" => "BOOL",
        "Int32" => "DINT",
        "UInt16" => "UINT",
        "UInt64" => "ULINT",
        "Float" => "REAL",
        "Double" => "LREAL",
        "String" => "STRING",
        other => panic!("no IEC type for {other}"),
    }
}

/// The PLC clients (S7-1500 OPC_UA_ReadList, TwinCAT UA_Read, CODESYS) need the variable declared with the type the
/// server reports. Every variable of every machine, found by browsing, has a row in the table of docs/plc.md: its
/// DataType, a scalar value rank, its access level and its value's type are those of the row, and the row's IEC type
/// is the one the OPC UA type maps to. Every row is met by some variable. The namespace index is 2 and the string node
/// ids are the same at every start.
#[tokio::test(flavor = "multi_thread")]
async fn the_type_table_matches_every_exposed_variable() {
    let table = type_table();
    assert!(table.len() > 10, "{} rows", table.len());
    for row in &table {
        assert_eq!(row.iec, iec(&row.opc), "{:?}", row.patterns);
    }
    let types = [
        ("Boolean", DataTypeId::Boolean),
        ("Int32", DataTypeId::Int32),
        ("UInt16", DataTypeId::UInt16),
        ("UInt64", DataTypeId::UInt64),
        ("Double", DataTypeId::Double),
        ("String", DataTypeId::String),
    ];
    let mut used = vec![false; table.len()];
    for entry in MACHINES.iter().copied() {
        let pki = scratch(&format!("types-{}", entry.id));
        let (link, url) = start_machine(entry, &pki, Security::default()).await;
        let none = (SecurityPolicy::None, MessageSecurityMode::None);
        let session = connect(&url, &pki.join("client"), none.0, none.1, IdentityToken::Anonymous).await.unwrap();
        assert_eq!(session.get_namespace_index(NAMESPACE).await.unwrap(), 2);
        let ids = variables(&session, link.namespace).await;
        assert!(ids.len() > 50, "{}: {} variables", entry.id, ids.len());
        // Of every variable: DataType, ValueRank, AccessLevel and Value.
        let attributes = [AttributeId::DataType, AttributeId::ValueRank, AttributeId::AccessLevel, AttributeId::Value];
        let reads: Vec<ReadValueId> = ids
            .iter()
            .flat_map(|id| {
                attributes.iter().map(|a| ReadValueId {
                    node_id: id.clone(),
                    attribute_id: *a as u32,
                    index_range: NumericRange::None,
                    data_encoding: QualifiedName::null(),
                })
            })
            .collect();
        let mut values = Vec::new();
        for chunk in reads.chunks(400) {
            values.extend(session.read(chunk, TimestampsToReturn::Neither, 0.0).await.unwrap());
        }
        for (id, got) in ids.iter().zip(values.chunks(4)) {
            let opcua::types::Identifier::String(name) = &id.identifier else { panic!("{id} is not a string id") };
            let name = name.as_ref();
            let row = table
                .iter()
                .position(|r| r.patterns.iter().any(|p| matches(p, name)))
                .unwrap_or_else(|| panic!("{}: {name} is not in the table of docs/plc.md", entry.id));
            used[row] = true;
            let row = &table[row];
            let expected = types.iter().find(|(t, _)| *t == row.opc).unwrap().1;
            assert_eq!(got[0].value, Some(Variant::NodeId(Box::new(expected.into()))), "{name}: DataType");
            assert_eq!(got[1].value, Some(Variant::Int32(-1)), "{name}: ValueRank");
            let Some(Variant::Byte(access)) = got[2].value else { panic!("{name}: AccessLevel {:?}", got[2]) };
            assert_eq!(access & 2 != 0, row.writable, "{name}: AccessLevel {access}");
            let value = got[3].value.as_ref().unwrap_or_else(|| panic!("{name} has no value"));
            let held = match value {
                Variant::Boolean(_) => "Boolean",
                Variant::Int32(_) => "Int32",
                Variant::UInt16(_) => "UInt16",
                Variant::UInt64(_) => "UInt64",
                Variant::Double(_) => "Double",
                Variant::String(_) => "String",
                other => panic!("{name}: {other:?}"),
            };
            assert_eq!(held, row.opc, "{name}: value {value:?}");
        }
    }
    for (row, used) in table.iter().zip(used) {
        assert!(used, "no variable matches {:?}", row.patterns);
    }
}

/// Waits until `deadline` by spinning: the runtime's timer follows Windows' 15.6 ms clock, far too coarse for a
/// pattern of 10 ms.
fn spin_until(deadline: Instant) {
    while Instant::now() < deadline {
        std::hint::spin_loop();
    }
}

/// The intervals of the pattern, in ms: nine of 10 ms and one of 40 ms, three times. Worked out by hand: 30 intervals
/// summing to 3 · (9 · 10 + 40) = 390 ms, mean 13 ms, largest 40 ms; mean of the squares (27 · 100 + 3 · 1600) / 30 =
/// 250 ms², standard deviation √(250 − 13²) = √81 = 9 ms.
const PATTERN_MS: [u64; 10] = [10, 10, 10, 10, 10, 10, 10, 10, 10, 40];
/// The server measures the client's cycle: a client writing on the pattern above (a PLC task of 10 ms that misses
/// three times for 30 ms) reads 30 intervals, mean 13 ms, largest 40 ms and standard deviation 9 ms back, and so does
/// a client reading on it. Tolerances: the mean ±0.5 ms (one loopback round trip at either end of 390 ms), the largest
/// ±5 ms and the standard deviation ±2 ms (a request arriving late on a loaded host).
#[tokio::test(flavor = "multi_thread")]
async fn the_server_measures_the_cycle_of_a_client_writing_every_10_ms() {
    let pki = scratch("jitter");
    let (link, url) = start(&pki, Security::default()).await;
    let ns = link.namespace;
    let none = (SecurityPolicy::None, MessageSecurityMode::None);
    let session = connect(&url, &pki.join("client"), none.0, none.1, IdentityToken::Anonymous).await.unwrap();
    for kind in ["write", "read"] {
        // Starts both measurements anew, in the write itself.
        assert!(write(&session, ns, "command.reset_pacing", true.into()).await);
        tokio::time::sleep(Duration::from_millis(50)).await;
        let mut at = Instant::now();
        for interval in std::iter::once(0).chain(PATTERN_MS.iter().copied().cycle().take(30)) {
            at += Duration::from_millis(interval);
            spin_until(at);
            if kind == "write" {
                assert!(write(&session, ns, "command.path_time_optimal", false.into()).await);
            } else {
                read(&session, ns, "status.busy").await;
            }
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
        // One read of all four: a read is counted after it is answered, so it is not in its own answer.
        let names = ["s", "_mean_s", "_std_s", "_max_s"].map(|what| format!("status.client_{kind}{what}"));
        let reads: Vec<ReadValueId> = names.iter().map(|n| node(ns, n).into()).collect();
        let got = session.read(&reads, TimestampsToReturn::Neither, 0.0).await.unwrap();
        let [count, mean, std, max] = [0, 1, 2, 3].map(|k| match got[k].value {
            Some(Variant::UInt64(n)) => n as f64,
            Some(Variant::Double(s)) => s * 1e3,
            ref other => panic!("{}: {other:?}", names[k]),
        });
        assert_eq!(count, 30.0, "{kind}s");
        assert!((mean - 13.0).abs() < 0.5, "{kind}s: mean {mean} ms");
        assert!((max - 40.0).abs() < 5.0, "{kind}s: largest {max} ms");
        assert!((std - 9.0).abs() < 2.0, "{kind}s: standard deviation {std} ms");
    }
    // The figures the log prints at shutdown are the same.
    let reads = link.clients.lock().unwrap().reads.clone();
    assert_eq!(reads.count, 31, "the pattern's 30 and the read of the results");
}
