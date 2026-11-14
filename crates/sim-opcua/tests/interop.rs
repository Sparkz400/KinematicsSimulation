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

/// The endpoints a server lists, as (security policy URI, mode), sorted.
fn offered(endpoints: &[opcua::types::EndpointDescription]) -> Vec<(String, MessageSecurityMode)> {
    let mut offered: Vec<(String, MessageSecurityMode)> =
        endpoints.iter().map(|e| (e.security_policy_uri.to_string(), e.security_mode)).collect();
    offered.sort_by_key(|(uri, mode)| (uri.clone(), *mode as i32));
    offered
}

/// What the server must list: Basic256Sha256, Aes128_Sha256_RsaOaep and Aes256_Sha256_RsaPss each with Sign and with
/// SignAndEncrypt, and None where no login is required. Written out here, not taken from `sim_opcua::POLICIES`.
fn listing(with_none: bool) -> Vec<(String, MessageSecurityMode)> {
    let mut listing = Vec::new();
    for policy in
        [SecurityPolicy::Basic256Sha256, SecurityPolicy::Aes128Sha256RsaOaep, SecurityPolicy::Aes256Sha256RsaPss]
    {
        for mode in [MessageSecurityMode::Sign, MessageSecurityMode::SignAndEncrypt] {
            listing.push((policy.to_uri().to_string(), mode));
        }
    }
    if with_none {
        listing.push((SecurityPolicy::None.to_uri().to_string(), MessageSecurityMode::None));
    }
    listing.sort_by_key(|(uri, mode)| (uri.clone(), *mode as i32));
    listing
}

/// A client that knows the server's endpoints, for listing them.
async fn endpoints(url: &str, name: &str) -> Vec<opcua::types::EndpointDescription> {
    let client = ClientBuilder::new().application_name("probe").application_uri("urn:probe").pki_dir(scratch(name));
    client.client().unwrap().get_server_endpoints_from_url(url).await.unwrap()
}

/// The server lists Aes128_Sha256_RsaOaep and Aes256_Sha256_RsaPss beside Basic256Sha256, each with Sign and with
/// SignAndEncrypt, and a trusted client reads over each of the new ones. With a login required, None is not listed.
#[tokio::test(flavor = "multi_thread")]
async fn the_aes_policies_are_listed_and_carry_a_session() {
    let (server_pki, client_pki) = (scratch("aes-server"), scratch("aes-client"));
    let (link, url) = start(&server_pki, Security { trust_all: true, ..Default::default() }).await;
    assert_eq!(offered(&endpoints(&url, "aes-probe").await), listing(true));
    for policy in [SecurityPolicy::Aes128Sha256RsaOaep, SecurityPolicy::Aes256Sha256RsaPss] {
        for mode in [MessageSecurityMode::Sign, MessageSecurityMode::SignAndEncrypt] {
            let session = connect(&url, &client_pki, policy, mode, IdentityToken::Anonymous)
                .await
                .unwrap_or_else(|| panic!("{policy:?} {mode:?} refused"));
            assert!(matches!(read(&session, link.namespace, "status.busy").await, Variant::Boolean(false)));
        }
    }
    drop(link);
    let user = Security { user: Some(("plc".into(), "secret".into())), ..Default::default() };
    let (_link, url) = start(&scratch("aes-user-server"), user).await;
    assert_eq!(offered(&endpoints(&url, "aes-user-probe").await), listing(false));
}

/// The server offers None, and every secured policy with Sign and with SignAndEncrypt. A client whose certificate the
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
    assert_eq!(offered(&endpoints), listing(true));

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
    let (link, url) = start(&server_pki, Security { trust_all: true, ..Default::default() }).await;
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

/// With a user name and password, anonymous clients and a wrong password are refused and the right one gets in. The
/// endpoint without security is not offered then: async-opcua would take a password sent there in clear.
#[tokio::test(flavor = "multi_thread")]
async fn a_user_name_and_password_keep_others_out() {
    let (server_pki, client_pki) = (scratch("user-server"), scratch("user-client"));
    let security = Security { trust_all: true, user: Some(("plc".into(), "secret".into())), ..Default::default() };
    let (link, url) = start(&server_pki, security).await;
    let encrypted = (SecurityPolicy::Basic256Sha256, MessageSecurityMode::SignAndEncrypt);
    let user = |password: &str| IdentityToken::new_user_name("plc", password);
    assert!(connect(&url, &client_pki, encrypted.0, encrypted.1, IdentityToken::Anonymous).await.is_none());
    assert!(connect(&url, &client_pki, encrypted.0, encrypted.1, user("wrong")).await.is_none());
    let session = connect(&url, &client_pki, encrypted.0, encrypted.1, user("secret")).await.unwrap();
    assert!(matches!(read(&session, link.namespace, "status.busy").await, Variant::Boolean(false)));
    let plain = connect(&url, &client_pki, SecurityPolicy::None, MessageSecurityMode::None, user("secret")).await;
    assert!(plain.is_none());
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
/// three times for 30 ms) reads back the count, mean, standard deviation and largest of the 30 intervals it wrote at,
/// and so does a client reading on it. The client times each request itself, from sending it to its answer; the
/// server takes its time in between, so each of its intervals is off the client's (between the middles of two
/// requests) by at most the two half round trips e_i: its mean by at most (h_0 + h_30) / 30 (h the half round trips),
/// its largest by the largest e_i, its standard deviation by the root mean square of the e_i (a standard deviation is
/// a seminorm: |std x - std y| <= std (x - y) <= rms (x - y)). On an idle host the client's own figures are the
/// pattern's 13, 40 and 9 ms; on a loaded one requests go out late, and the server has to follow what they did.
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
        let zero = Instant::now();
        let (mut at, mut timed) = (zero, Vec::new());
        for interval in std::iter::once(0).chain(PATTERN_MS.iter().copied().cycle().take(30)) {
            at += Duration::from_millis(interval);
            spin_until(at);
            let sent = zero.elapsed().as_secs_f64();
            if kind == "write" {
                assert!(write(&session, ns, "command.path_time_optimal", false.into()).await);
            } else {
                read(&session, ns, "status.busy").await;
            }
            timed.push((sent, zero.elapsed().as_secs_f64()));
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
        // The client's own intervals, in ms, and how far the server's may be off each of them.
        let (middle, half): (Vec<f64>, Vec<f64>) =
            timed.iter().map(|(sent, answered)| (0.5e3 * (sent + answered), 0.5e3 * (answered - sent))).unzip();
        let own: Vec<f64> = middle.windows(2).map(|w| w[1] - w[0]).collect();
        let off: Vec<f64> = half.windows(2).map(|w| w[0] + w[1]).collect();
        let n = own.len() as f64;
        let own_mean = own.iter().sum::<f64>() / n;
        let own_std = (own.iter().map(|x| (x - own_mean).powi(2)).sum::<f64>() / n).sqrt();
        let own_max = own.iter().copied().fold(0.0, f64::max);
        let rms = (off.iter().map(|e| e * e).sum::<f64>() / n).sqrt();
        // The server keeps seconds in f64 and sums squares: 1 ns covers its rounding.
        let rounding = 1e-6;
        assert!(
            (mean - own_mean).abs() <= (half[0] + half[30]) / n + rounding,
            "{kind}s: mean {mean} ms, the client's {own_mean} ms"
        );
        assert!(
            (max - own_max).abs() <= off.iter().copied().fold(0.0, f64::max) + rounding,
            "{kind}s: largest {max} ms, the client's {own_max} ms, round trips {off:?}"
        );
        assert!(
            (std - own_std).abs() <= rms + rounding,
            "{kind}s: standard deviation {std} ms, the client's {own_std} ms, round trips {off:?}"
        );
    }
    // The figures the log prints at shutdown are the same.
    let reads = link.clients.lock().unwrap().reads.clone();
    assert_eq!(reads.count, 31, "the pattern's 30 and the read of the results");
}

/// `start_machine` with the config `cfg`.
async fn start_config(cfg: &sim_core::Config, pki: &Path) -> (Link, String) {
    let entry = MACHINES[0];
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let url = format!("opc.tcp://{}/", listener.local_addr().unwrap());
    let labels = translations("en", entry);
    let security = Security::default();
    let link = serve_secured(entry, cfg, &labels, listener, pki, Duration::from_millis(10), &security).await.unwrap();
    (link, url)
}

/// An anonymous client on the endpoint without security.
async fn plain(url: &str, name: &str) -> Arc<Session> {
    let none = (SecurityPolicy::None, MessageSecurityMode::None);
    connect(url, &scratch(name), none.0, none.1, IdentityToken::Anonymous).await.unwrap()
}

/// A PLC writing IEC USINT, SINT, UINT, UDINT, ULINT or LINT to a numeric input used to get BadTypeMismatch. Each is
/// taken now, as the number it is; a 64-bit integer beyond 2^53 = 9 007 199 254 740 992, which a double cannot hold
/// exactly, is refused. The write's status comes from the server at once; the value read back from the loop's next tick.
#[tokio::test(flavor = "multi_thread")]
async fn unsigned_and_byte_integers_are_taken_as_numbers() {
    let (link, url) = start(&scratch("unsigned-server"), Security::default()).await;
    let session = plain(&url, "unsigned-client").await;
    let axis = sim_components::Sim::new(MACHINES[0], &MACHINES[0].config()).unwrap().machine.axes()[0].id;
    let name = format!("axis.{axis}.MoveAbsolute.Jerk");
    let exact = 1i64 << 53;
    for (value, number) in [
        (Variant::Byte(200), 200.0),
        (Variant::SByte(-100), -100.0),
        (Variant::UInt16(60_000), 60_000.0),
        (Variant::UInt32(4_000_000_000), 4.0e9),
        (Variant::UInt64(exact as u64), 9_007_199_254_740_992.0),
        (Variant::Int64(-exact), -9_007_199_254_740_992.0),
    ] {
        assert!(write(&session, link.namespace, &name, value.clone()).await, "{value:?} refused");
        let mut read_back = Variant::Empty;
        for _ in 0..200 {
            read_back = read(&session, link.namespace, &name).await;
            if read_back == Variant::Double(number) {
                break;
            }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
        assert_eq!(read_back, Variant::Double(number), "{value:?}");
    }
    for value in [Variant::UInt64(exact as u64 + 1), Variant::Int64(-exact - 1), Variant::from("1")] {
        assert!(!write(&session, link.namespace, &name, value.clone()).await, "{value:?} taken");
    }
}

/// `--config FILE` and `--set` (`sim_opcua::configured`) give the config the server serves: the start pose set on top of
/// the machine's config file is where the move targets read before any move, [0.2, 0.3] m instead of [0.1, 0.35] m. They
/// read the controller's setpoint there: where the motors stand that hold the tool at the start pose, under the belts'
/// stretch some 10 um off it, so not the start pose to the digit.
#[tokio::test(flavor = "multi_thread")]
async fn a_config_file_with_set_on_top_is_served() {
    let file = Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../machine-t-gantry/default.toml"));
    let sets = ["start.pose = [0.2, 0.3]".to_string()];
    let (cfg, trajectory) = sim_opcua::configured(MACHINES[0], Some(file), &sets, None).unwrap();
    assert!(trajectory.is_none());
    let (link, url) = start_config(&cfg, &scratch("config-server")).await;
    let session = plain(&url, "config-client").await;
    let sim = sim_components::Sim::new(MACHINES[0], &cfg).unwrap();
    for ((axis, at), set) in sim.machine.axes().iter().zip([0.2, 0.3]).zip(&sim.control.set_pos) {
        let target = read(&session, link.namespace, &format!("command.target.{}", axis.id)).await;
        assert!((set - at).abs() < 1e-4, "{}: set to {set} m at the start pose {at} m", axis.id);
        assert_eq!(target, Variant::Double(*set), "{}", axis.id);
    }
}

/// The server's certificate is valid for 365 days from its creation; `renew_certificate` sets it and its key aside and
/// the next start creates a new one, again valid for 365 days. Before the first start there is none.
#[tokio::test(flavor = "multi_thread")]
async fn the_certificate_lasts_365_days_and_is_renewed_on_request() {
    let pki = scratch("renew-server");
    assert!(sim_opcua::certificate_days_left(&pki).is_none());
    drop(start(&pki, Security::default()).await);
    let days = sim_opcua::certificate_days_left(&pki).unwrap();
    // Created a moment ago: 365 days less the seconds since (0.1 day is 2.4 hours).
    assert!((364.9..=365.0).contains(&days), "{days} days");
    let cert = pki.join("own").join("cert.der");
    let old = std::fs::read(&cert).unwrap();
    sim_opcua::renew_certificate(&pki).unwrap();
    assert!(sim_opcua::certificate_days_left(&pki).is_none());
    assert!(pki.join("own").join("cert.der.old").exists() && pki.join("private").join("private.pem.old").exists());
    drop(start(&pki, Security::default()).await);
    assert!(sim_opcua::certificate_days_left(&pki).unwrap() > 364.9);
    assert_ne!(std::fs::read(&cert).unwrap(), old, "the same certificate");
}

/// A user certificate (X.509 user token, `--user-cert`) lets in the client that signs with its key, and no one else:
/// not an anonymous client, not one with another certificate. None is not listed, as with a password.
#[tokio::test(flavor = "multi_thread")]
async fn a_user_certificate_lets_in_its_holder_only() {
    let users = scratch("x509-users");
    std::fs::create_dir_all(&users).unwrap();
    let make = |name: &str| {
        let description = opcua::types::ApplicationDescription {
            application_name: name.into(),
            application_uri: format!("urn:{name}").into(),
            ..Default::default()
        };
        let (cert, key) = (users.join(format!("{name}.der")), users.join(format!("{name}.pem")));
        opcua::crypto::CertificateStore::create_certificate_and_key(
            &opcua::crypto::X509Data::from(description),
            true,
            &cert,
            &key,
        )
        .unwrap();
        (cert, key)
    };
    let (operator, intruder) = (make("operator"), make("intruder"));
    let security = Security { trust_all: true, user_certs: vec![operator.0.clone()], ..Default::default() };
    let (link, url) = start(&scratch("x509-server"), security).await;
    assert_eq!(offered(&endpoints(&url, "x509-probe").await), listing(false));
    let client_pki = scratch("x509-client");
    let encrypted = (SecurityPolicy::Basic256Sha256, MessageSecurityMode::SignAndEncrypt);
    let holder = |(cert, key): &(PathBuf, PathBuf)| IdentityToken::new_x509_path(cert, key).unwrap();
    assert!(connect(&url, &client_pki, encrypted.0, encrypted.1, IdentityToken::Anonymous).await.is_none());
    assert!(connect(&url, &client_pki, encrypted.0, encrypted.1, holder(&intruder)).await.is_none());
    let session = connect(&url, &client_pki, encrypted.0, encrypted.1, holder(&operator)).await.unwrap();
    assert!(matches!(read(&session, link.namespace, "status.busy").await, Variant::Boolean(false)));
}

/// A user certificate the server cannot read stops the start with its file named: async-opcua would pass over it
/// without a word, and nobody could log in.
#[tokio::test(flavor = "multi_thread")]
async fn an_unreadable_user_certificate_stops_the_start() {
    let entry = MACHINES[0];
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let security = Security { user_certs: vec![PathBuf::from("no-such-user.der")], ..Default::default() };
    let labels = translations("en", entry);
    let pki = scratch("x509-missing");
    let started =
        serve_secured(entry, &entry.config(), &labels, listener, &pki, Duration::from_millis(10), &security).await;
    let Err(sim_opcua::LinkError(key, detail)) = started else { panic!("started") };
    assert_eq!((key, detail.as_str()), ("opcua-error-user-cert", "no-such-user.der"));
}
