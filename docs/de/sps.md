# SPS-Inbetriebnahme

Die OPC-UA-Kopplung (`sim-opcua`) lässt eine Maschine in Echtzeit laufen und eine SPS sie fahren. Diese Seite beschreibt die PLCopen-Motion-Schnittstelle, den Echtzeittakt und seine Messung, die Beispiele in Strukturiertem Text und den Dauertest. Die übrigen Knoten der Kopplung (Signale, Ziele, Sollwerte, Bahnen, Zyklus) stehen im [Benutzerhandbuch](benutzerhandbuch.md).
English version: [PLC virtual commissioning](../plc.md).

## Kopplung starten

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --host 192.168.0.10 --cycle-ms 10 --lang de
```

| Argument | Standard | Bedeutung |
| --- | --- | --- |
| `MACHINE` | `t-gantry` | Die Maschine, als Erstes, wenn angegeben |
| `--machine-file FILE` | keine | Eine in einer Datei beschriebene Maschine ([Maschinendateien](maschinendateien.md)), anstelle von `MACHINE` |
| `--config FILE` | die der Maschine | Die Konfiguration der Maschine, gelesen wie `sim-cli` sie liest: Ein Schlüssel, den die Maschine nicht hat, wird mit Namen abgewiesen |
| `--set PFAD=WERT` | keiner | Ein Konfigurationswert über der Konfiguration, in TOML-Schreibweise (`--set "start.pose = [0.2, 0.3]"`); wiederholbar, der Reihe nach angewandt |
| `--trajectory FILE` | keine | Lagen zu Zeitpunkten, die beim Start gefahren werden, wie `sim-cli run --trajectory` sie nimmt: eine Zeile je Lage, die Zeit in s, dann eine Lage je Achse, durch Kommas getrennt |
| `--port N` | 4840 | TCP-Port |
| `--host IP` | 127.0.0.1 | Adresse, auf der der Server lauscht. Eine SPS im Netz braucht eine Adresse dieses Rechners, die sie erreicht. Die Adresse selbst angeben, nicht `0.0.0.0`: Der Server nennt sie in seiner Endpunkt-URL |
| `--lang en\|de` | en | Sprache der Meldungen und der Variablenbeschriftungen |
| `--cycle-ms N` | 10 | Takt der Simulationsschleife, 1 bis 1000 ms |
| `--pki DIR` | `pki` | Ordner des Server-Zertifikats und der vertrauten und abgewiesenen Client-Zertifikate, relativ zum Arbeitsverzeichnis (siehe [Sicherheit](#sicherheit)) |
| `--trust-all` | aus | Jedes Client-Zertifikat annehmen und bei den vertrauten ablegen. Nur zur Inbetriebnahme |
| `--user NAME` | keiner | Clients müssen sich mit diesem Benutzernamen und dem Passwort aus der Umgebungsvariablen `SIM_OPCUA_PASSWORD` anmelden; anonyme Clients werden abgewiesen, und der Endpunkt ohne Sicherheit wird nicht angeboten. Ohne die Variable startet die Kopplung nicht |
| `--user-cert FILE` | keines | Ein Benutzerzertifikat (`.der` oder `.pem`): Ein Client, der mit seinem privaten Schlüssel signiert, meldet sich als dieser Benutzer an (X.509-Benutzer-Token). Wiederholbar; wie bei `--user` werden anonyme Clients abgewiesen, und der Endpunkt ohne Sicherheit wird nicht angeboten |
| `--renew-certificate` | aus | Zertifikat und Schlüssel des Servers beiseitelegen (`.old`) und neue erzeugen, 365 Tage gültig |

Strg+C beendet die Kopplung und gibt den gemessenen Takt in einer Zeile dieser Form aus (die Zahlen sind die des Dauertests unten, nicht eines Laufs mit Strg+C):

```text
Echtzeittakt (Taktdauer, größter und effektiver Jitter in ms, Überläufe, Echtzeitfaktor): 10.017 · 16.116 · 4.581 · 42 · 1.0000
```

Beim Start gibt die Kopplung außerdem den absoluten Pfad ihres pki-Ordners aus, ob `--trust-all`, `--user` oder `--user-cert` gesetzt sind, und eine Warnung, wenn das Zertifikat des Servers in weniger als 30 Tagen abläuft. Am Ende gibt sie die Abstände der Schreib- und Lesezugriffe der Clients aus (siehe [Der Takt des Clients](#der-takt-des-clients)), hier die des Musters in `sim-opcua/tests/interop.rs`, neun Abstände von 10 ms und einer von 40 ms, dreimal:

```text
Client-Schreibzugriffe (Abstände, Mittel, Standardabweichung, größter in ms): 30 · 13.000 · 9.000 · 40.000
Client-Lesezugriffe (Abstände, Mittel, Standardabweichung, größter in ms): 30 · 13.000 · 9.000 · 40.000
```

## Sicherheit

Der Server bietet sieben Endpunkte, alle unter derselben URL `opc.tcp://HOST:PORT/`:

| Sicherheitsrichtlinie | Nachrichtensicherheit | Client-Zertifikat geprüft |
| --- | --- | --- |
| None | None | Nein (nicht angeboten mit `--user` oder `--user-cert`) |
| Basic256Sha256 | Sign, SignAndEncrypt | Ja |
| Aes128_Sha256_RsaOaep | Sign, SignAndEncrypt | Ja |
| Aes256_Sha256_RsaPss | Sign, SignAndEncrypt | Ja |

Basic256Sha256 bieten S7-1500, TwinCAT TF6100 und der CODESYS-OPC-UA-Client alle an; die beiden Aes-Richtlinien sind die neueren aus OPC UA 1.04 und später, für Clients, die sie haben. Das sind alle Richtlinien von async-opcua 0.19, die OPC UA nicht abkündigt: Basic128Rsa15 und Basic256 bleiben weg.

**Restrisiko: RSA-Zeitverhalten (RUSTSEC-2023-0071, „Marvin“).** async-opcua 0.19.0, die neueste Version (geprüft am 14. November 2026), verwendet die Crate `rsa` 0.9, deren RSA-Entschlüsselung nicht in konstanter Zeit läuft; `rsa` 0.10.0 (8. November 2026) behebt das, aber noch keine async-opcua-Version verwendet sie. Der Server entschlüsselt an jedem gesicherten Endpunkt mit seinem privaten Schlüssel: Die OpenSecureChannel-Anfrage ist immer verschlüsselt, bei Sign wie bei SignAndEncrypt, und async-opcua entschlüsselt sie, bevor es nachsieht, ob dem Zertifikat des Clients vertraut wird; ein verschlüsseltes Passwort wird ebenfalls entschlüsselt. Wer den Port erreicht, kann also Verbindungsaufbauten schicken und die Antwortzeiten messen und mit sehr vielen davon womöglich den privaten Schlüssel des Servers gewinnen, dann aufgezeichnete Sitzungen entschlüsseln oder sich als Server ausgeben. Weder nur Sign noch nur vertraute Client-Zertifikate schließen das. Was es begrenzt:

- Die Kopplung lauscht auf 127.0.0.1, wenn `--host` keine andere Adresse nennt. `--host` nur eine Adresse in einem Inbetriebnahmenetz geben, das fremde Rechner nicht erreichen, nie eine aus dem Internet erreichbare; eine Firewallregel, die nur die Adresse der SPS zulässt, hält die anderen schon vom Verbindungsaufbau ab.
- Der Schlüssel ist der eigene des Servers, auf diesem Rechner erzeugt und für nichts anderes verwendet: `--renew-certificate` ersetzt ihn, sobald ein Verdacht besteht, spätestens nach einem Jahr.
- Der Simulator ist ein Inbetriebnahmewerkzeug, kein Produktionsserver: Ihn nicht unbeaufsichtigt in einem geteilten Netz laufen lassen.

Die Ausnahme in `deny.toml` bleibt, bis eine async-opcua-Version `rsa` 0.10 verwendet.

**Das Zertifikat des Servers.** Beim ersten Start legt die Kopplung ihr Anwendungsinstanz-Zertifikat im pki-Ordner an (`--pki`, Standard `pki` im Arbeitsverzeichnis; `/pki` ignoriert git):

| Datei | Was sie ist |
| --- | --- |
| `own/cert.der` | Das Zertifikat: selbst signiert, RSA 2048 Bit, 365 Tage gültig. Anwendungs-URI `urn:kinematics-simulation:server`; die alternativen Namen enthalten diesen URI, `localhost`, den Rechnernamen und seine IP-Adressen |
| `private/private.pem` | Sein privater Schlüssel. Er bleibt auf diesem Rechner |
| `trusted/` | Client-Zertifikate, die der Server annimmt |
| `rejected/` | Client-Zertifikate, die der Server abgewiesen hat |

Eine SPS, die sich mit Sicherheit verbindet, muss `own/cert.der` vertrauen: es in den Zertifikatsspeicher der SPS als vertrauten Partner importieren (TIA Portal: Zertifikatsmanager der CPU; TwinCAT TF6100 und CODESYS: siehe unten). Ab 30 Tagen vor Ablauf des Zertifikats warnt jeder Start. Dann, oder wenn sich Rechnername oder Adresse ändern und die SPS sie prüft, einmal mit `--renew-certificate` starten: Altes Zertifikat und alter Schlüssel bleiben als `own/cert.der.old` und `private/private.pem.old` neben den neuen, und dem neuen Zertifikat, 365 Tage gültig, muss die SPS neu vertrauen.

**Client-Zertifikate.** Ein Client an einem gesicherten Endpunkt, dessen Zertifikat nicht in `trusted/` liegt, wird abgewiesen, und sein Zertifikat wird nach `rejected/` geschrieben. Um ihm zu vertrauen, die Datei von `rejected/` nach `trusted/` verschieben; die nächste Verbindung wird angenommen, ohne Neustart. Ein Zertifikat, das in `rejected/` liegt, wird auch mit `--trust-all` abgewiesen: Es erst verschieben oder löschen. `--trust-all` nimmt jedes Client-Zertifikat an und kopiert es nach `trusted/`, wo es nach dem Beenden bleibt: Während der Inbetriebnahme verwenden, danach aus `trusted/` löschen, was nicht bleiben soll. Der Endpunkt ohne Sicherheit prüft keine Client-Zertifikate; er ist für Werkzeuge wie UaExpert auf demselben Rechner und für die Tests da.

**Benutzername und Passwort.** Mit `--user NAME` verlangt jeder Endpunkt diesen Benutzernamen und das Passwort aus `SIM_OPCUA_PASSWORD`, das aus der Umgebung gelesen wird, damit es nicht in der Prozessliste erscheint:

```sh
SIM_OPCUA_PASSWORD=geheim cargo run --release -p sim-opcua -- t-gantry --user sps
```

(PowerShell: vorher `$env:SIM_OPCUA_PASSWORD = "geheim"`.) Mit `--user` wird der Endpunkt ohne Sicherheit nicht angeboten, damit das Passwort nie im Klartext übertragen wird: Die SPS verbindet sich mit Sign oder SignAndEncrypt. Anonyme Clients werden dann abgewiesen. Es gibt einen Benutzer; Rollen (nur lesen, schreiben) werden nicht unterschieden.

**Benutzerzertifikate.** Mit `--user-cert FILE` (wiederholbar) meldet sich ein Client mit einem X.509-Benutzer-Token an: Er signiert Zertifikat und Nonce des Servers mit dem privaten Schlüssel dieses Zertifikats, und die Kopplung lässt ihn als den nach der Datei benannten Benutzer ein (`sps.der`: `sps`). Nur die angegebenen Zertifikate werden angenommen, verglichen über den Fingerabdruck; ihre Vertrauenskette wird nicht geprüft. async-opcua verlangt die Signatur des Tokens mit dem Verfahren von Basic128Rsa15 (RSA PKCS#1 v1.5 mit SHA-1), gleich welche Richtlinie der Endpunkt hat; das legt die Bibliothek fest und lässt sich nicht einstellen. Benutzername und Zertifikate lassen sich zusammen angeben.

## Knoten-IDs und Datentypen

Jede Variable ist ein Skalar (ValueRank −1) im Namensraum `urn:kinematics-simulation`, mit einer Zeichenketten-Knoten-ID, die bei jedem Start derselben Maschine gleich ist: Sie entsteht aus den Signal-, Achsen- und Bausteinnamen, nie durch Nummerieren. Der Namensraumindex ist derzeit 2 (0 ist der von OPC UA, 1 der Anwendungs-URI des Servers); ein Client sollte ihn trotzdem über den URI ermitteln (`OPC_UA_NamespaceGetIndexList`, `UA_GetNamespaceIndex`, `UA_NamespaceGetIndex`). Die Tabelle nennt für jeden Knoten den OPC-UA-Datentyp und den IEC-61131-3-Typ, den eine SPS-Variable zum Lesen oder Schreiben braucht; `*` steht für beliebigen Text, und die erste passende Zeile gilt. Ein Test (`sim-opcua/tests/interop.rs`) durchsucht jede Variable jeder Maschine und prüft DataType, ValueRank, AccessLevel und Wert gegen die englische Fassung dieser Tabelle.

| Knoten-ID | OPC-UA-Typ | IEC 61131-3 | Zugriff |
| --- | --- | --- | --- |
| `signal.*` | Double | LREAL | lesen |
| `command.target.*`, `command.setpoint.*` | Double | LREAL | lesen, schreiben |
| `command.move`, `command.cycle`, `command.disable`, `command.enable`, `command.path_time_optimal`, `command.run_path`, `command.reset_pacing` | Boolean | BOOL | lesen, schreiben |
| `command.path` | String | STRING | lesen, schreiben |
| `status.busy` | Boolean | BOOL | lesen |
| `status.refused` | String | STRING | lesen |
| `status.overruns`, `status.cycles`, `status.steps`, `status.client_writes`, `status.client_reads` | UInt64 | ULINT | lesen |
| `status.*` | Double | LREAL | lesen |
| `axis.*.ActualPosition`, `axis.*.ActualVelocity` | Double | LREAL | lesen |
| `axis.*.Enable`, `axis.*.Execute` | Boolean | BOOL | lesen, schreiben |
| `axis.*.Position`, `axis.*.Distance`, `axis.*.Velocity`, `axis.*.Acceleration`, `axis.*.Deceleration`, `axis.*.Jerk` | Double | LREAL | lesen, schreiben |
| `axis.*.ErrorID` | UInt16 | UINT | lesen |
| `axis.*.StateID` | Int32 | DINT | lesen |
| `axis.*.State` | String | STRING | lesen |
| `axis.*.Status`, `axis.*.Valid`, `axis.*.Error`, `axis.*.Done`, `axis.*.Busy`, `axis.*.Active`, `axis.*.CommandAborted`, `axis.*.IsHomed` | Boolean | BOOL | lesen |

Was die SPS-Clients brauchen:

- **Siemens S7-1500** (`OPC_UA_Connect`, `OPC_UA_NamespaceGetIndexList`, `OPC_UA_NodeGetHandleList`, `OPC_UA_ReadList`, `OPC_UA_WriteList`): die Variablen der Listen in einem Datenbaustein, mit den IEC-Typen oben (LReal, Bool, UInt, DInt, ULInt, String). Ein S7-`String` fasst höchstens 254 Zeichen; für ein längeres `command.path` `WString` nehmen, das der Client ebenfalls auf OPC-UA-String abbildet. Die S7-1200 kennt kein `ULInt`: die UInt64-Zähler mit einer S7-1500 lesen oder weglassen.
- **Beckhoff TF6100** (`UA_Connect`, `UA_GetNamespaceIndex`, `UA_NodeGetHandle`, `UA_Read`, `UA_Write`): Der Wert geht als Zeiger und Größe, also bestimmt der Typ der SPS-Variablen, was gesendet wird. LREAL für jedes Double: Ein REAL schreibt ein Float, das die Kopplung annimmt, aber ein Double passt beim Lesen nicht in ein REAL. Standardmäßig `STRING(255)`; eine längere Bahn braucht einen längeren STRING.
- **CODESYS-OPC-UA-Client** (PLCopen-Bausteine `UA_Connect`, `UA_NamespaceGetIndex`, `UA_NodeGetHandle`, `UA_Read`, `UA_Write`): Der Wert ist ein IN_OUT vom Typ ANY, also ist sein deklarierter Typ der auf der Leitung.

Die Kopplung nimmt beim Schreiben einer Zahl Double, Float, SByte, Byte, Int16, UInt16, Int32, UInt32, Int64 oder UInt64 an (IEC LREAL, REAL, SINT, USINT, INT, UINT, DINT, UDINT, LINT, ULINT), beim Schreiben eines Merkers Boolean; alles andere wird mit `BadTypeMismatch` abgewiesen, ebenso eine 64-Bit-Ganzzahl über 2^53, die ein Double nicht genau fasst.

## PLCopen-Achsen

Jede Achse der Maschine hat einen Ordner `axis.<Achsen-ID>` mit einem Satz Handshake-Variablen je Funktionsbaustein. Die Knoten-IDs sind Zeichenketten im Namensraum `urn:kinematics-simulation`, zum Beispiel `axis.x.MoveAbsolute.Execute`. Die Namen sind die englischen PLCopen-Namen und in beiden Sprachen gleich.

| Knoten | Typ | Zugriff | Bedeutung |
| --- | --- | --- | --- |
| `axis.<id>.Power.Enable` | Boolean | schreiben | MC_Power.Enable: `true` schaltet die Achse ein |
| `axis.<id>.Power.Status` | Boolean | lesen | Enable ist true und die Antriebe sind an |
| `axis.<id>.Power.Valid` | Boolean | lesen | Status und kein Achsfehler |
| `axis.<id>.Power.Error`, `.ErrorID` | Boolean, UInt16 | lesen | Der Achsfehler, auch wenn Enable false ist: Die Achse bleibt bis MC_Reset in ErrorStop |
| `axis.<id>.Home.Execute` | Boolean | schreiben | MC_Home |
| `axis.<id>.MoveAbsolute.Execute` | Boolean | schreiben | MC_MoveAbsolute |
| `axis.<id>.MoveAbsolute.Position` | Double | schreiben | Ziel, m |
| `axis.<id>.MoveAbsolute.Velocity`, `.Acceleration`, `.Deceleration`, `.Jerk` | Double | schreiben | Grenzen der Fahrt, m/s, m/s², m/s², m/s³. Sie beginnen bei den Achsgrenzen |
| `axis.<id>.MoveRelative.Execute` | Boolean | schreiben | MC_MoveRelative |
| `axis.<id>.MoveRelative.Distance` | Double | schreiben | Weg, m, ab dem Achssollwert bei der steigenden Flanke |
| `axis.<id>.MoveRelative.Velocity`, `.Acceleration`, `.Deceleration`, `.Jerk` | Double | schreiben | Wie bei MoveAbsolute |
| `axis.<id>.Stop.Execute` | Boolean | schreiben | MC_Stop |
| `axis.<id>.Stop.Deceleration` | Double | schreiben | Bremsverzögerung, m/s² |
| `axis.<id>.Reset.Execute` | Boolean | schreiben | MC_Reset |
| `axis.<id>.<FB>.Done`, `.Busy`, `.Active`, `.CommandAborted`, `.Error` | Boolean | lesen | Ausgänge von Home, MoveAbsolute, MoveRelative, Stop und Reset. `Active` liest dasselbe wie `Busy` |
| `axis.<id>.<FB>.ErrorID` | UInt16 | lesen | Warum der Baustein scheiterte, 0 ohne Fehler |
| `axis.<id>.State` | String | lesen | `Disabled`, `Standstill`, `ErrorStop`, `Stopping`, `Homing` oder `DiscreteMotion` |
| `axis.<id>.StateID` | Int32 | lesen | Der Zustand als Zahl: 1 Disabled, 2 Standstill, 3 ErrorStop, 4 Stopping, 5 Homing, 6 DiscreteMotion (wie `MC_AxisStates` in TwinCAT) |
| `axis.<id>.ActualPosition`, `.ActualVelocity` | Double | lesen | Lage und Geschwindigkeit der simulierten Achse (`signal.axis.<id>.position`, `.velocity`) |
| `axis.<id>.IsHomed` | Boolean | lesen | MC_Home ist einmal fertig geworden |
| `axis.<id>.Error`, `.ErrorID` | Boolean, UInt16 | lesen | Der Achsfehler. Ungleich 0 heißt ErrorStop |

| ErrorID | Bedeutung |
| --- | --- |
| 1 | Die Achse ist nicht eingeschaltet (Disabled) |
| 2 | In diesem Zustand nicht erlaubt: jeder Baustein außer Reset in ErrorStop, eine Fahrt in Stopping oder Homing, Home in DiscreteMotion |
| 3 | Ein Parameter ist keine positive endliche Zahl |
| 4 | Das Ziel liegt außerhalb des Achsbereichs |
| 5 | Eine Fahrt, Bahn oder der Zyklus der übrigen Befehle der Kopplung läuft |
| 6 | Der Regler hat einen gestreamten Sollwert abgelehnt (Achsfehler) |
| 7 | Die Antriebe wurden ausgeschaltet, während die Achse eingeschaltet war: `command.disable` oder eine Zwischenkreis-Abschaltung (Achsfehler) |
| 8 | Die Antriebe gingen aus, weil die Maschine gegen ein Hindernis fuhr (`effects.crash`, Achsfehler) |

So verhalten sich die Bausteine:

- Eine steigende Flanke an `Execute` startet einen Baustein mit den Eingängen, die er in diesem Moment hat. Erneutes Schreiben von `true`, während `Execute` true ist, bewirkt nichts.
- `Done`, `CommandAborted` und `Error` bleiben true, solange `Execute` true ist. Fällt `Execute`, werden sie gelöscht, sobald der Baustein fertig ist. Fällt `Execute`, während der Baustein arbeitet, läuft er weiter, und seine Ausgänge bleiben bis zur nächsten steigenden Flanke stehen, damit ein pollender Client sie sieht.
- Die Ausgänge werden einmal je Schleifentakt veröffentlicht. Ein Client, der `Done` gleich nach dem Schreiben von `Execute` liest, kann den Wert des vorigen Befehls lesen. Der Handshake, der funktioniert: `Execute` auf false schreiben, warten, bis `Done` false liest, die Eingänge und `Execute` true schreiben, dann `Busy`, `Done`, `CommandAborted` und `Error` pollen. Die Beispiele in Strukturiertem Text tun das.
- Fahrten und Stop übernehmen die laufende Bewegung mit der Lage und Geschwindigkeit, die die Achse hat. Der Baustein, der lief, meldet `CommandAborted`, außer es ist derselbe Baustein, neu gestartet: Der beginnt neu mit gelöschten Ausgängen.
- `Done`, `Busy`, `CommandAborted` und `Error` eines Bausteins sind nie zugleich true. Ein Baustein, der während `Busy` neu gestartet wird und dessen neue Eingänge abgewiesen werden (Ziel außerhalb des Bereichs, `Velocity` 0, ...), meldet nur `Error` und gibt seine Bewegung ab: Die Achse bremst mit der Verzögerung dieser Bewegung bis zum Stillstand, in `DiscreteMotion`, und kein Baustein meldet dafür `Done`. Eine neue Fahrt darf das Bremsen übernehmen. PLCopen Teil 1 lässt diesen Fall offen; die alte Bewegung unter einem Baustein weiterlaufen zu lassen, der `Error` zeigt, ließe die SPS ohne einen Baustein, dem sie gehört.
- MC_Stop bremst mit `Stop.Deceleration` bis zum Stillstand und meldet dann `Done`. Die Achse bleibt in `Stopping`, solange `Stop.Execute` true ist, und lehnt dort Fahrten ab (ErrorID 2). Sie geht nach `Standstill`, wenn `Stop.Execute` fällt.
- MC_Home fährt die Achse in ihre Grundstellung, mit der halben Geschwindigkeits- und Beschleunigungsgrenze der Achse, und setzt `IsHomed`. Die Grundstellung ist der Achswert der Startpose der Maschine (`[start] pose` in der Konfiguration). Es sucht keinen Schalter und setzt keinen neuen Nullpunkt: Lagen im Simulator sind absolut.
- Ein Achsfehler bleibt über `Power.Enable` false hinweg bestehen, wie im Zustandsdiagramm von PLCopen: Die Achse bleibt in ErrorStop, und `Power.Error` und `Power.ErrorID` zeigen den Fehler. MC_Reset löscht den Achsfehler und verlässt ErrorStop nach Standstill (oder Disabled, wenn `Power.Enable` false ist). Nach ErrorID 7 oder 8 schaltet es die Antriebe wieder ein; nach einem Aufprall lässt sich die Achse dann aus dem Hindernis fahren, und eine Fahrt weiter hinein schaltet die Antriebe wieder ab.
- MC_Power: Der Regler hat eine Freigabe für alle Antriebe. Die Antriebe gehen an, wenn eine Achse eingeschaltet wird, und aus, wenn die letzte Achse ausgeschaltet wird. Eine Achse mit `Power.Enable` false ist Disabled und lehnt Bausteine ab, ihr Motor hält aber die Lage, solange eine andere Achse die Antriebe an hält. Ausschalten einer Achse bricht ihre Bewegung ab.
- Parameter über den Achsgrenzen werden auf die Grenzen gekürzt. Das Profil ist ein Trapez: Geschwindigkeit, Beschleunigung und Verzögerung werden eingehalten, `Jerk` wird angenommen und nicht verwendet.
- Die PLCopen-Bausteine nicht gleichzeitig mit `command.setpoint.*`, `command.move`, `command.cycle` oder `command.run_path` an derselben Maschine verwenden. Bausteine, die gestartet werden, während diese laufen, scheitern mit ErrorID 5; ein Sollwertstrom, der startet, während Bausteine laufen, lässt den Regler einen der beiden ablehnen.

### Wie es arbeitet, und seine Grenzen

Der Kernregler fährt alle Achsen gemeinsam auf einem synchronisierten Profil, lehnt eine neue Fahrt ab, während eine läuft, und `Command::Stop` hält nur den Zyklus nach seinem Segment an. Eine PLCopen-Achse braucht mehr: Achsen, die unabhängig und gleichzeitig fahren, ein MC_Stop, das mitten in einer Fahrt bremst, und eine neue Fahrt, die die alte abbricht. Die Kopplung rechnet deshalb selbst ein Profil je Achse und streamt die Lagen aller Achsen mit `Command::Setpoint` an den Regler, einen Sollwert je `profile.setpoint_interval_s` (10 ms), gezählt in Simulationsschritten. So fährt eine SPS Servoantriebe über einen Feldbus. Der Regler interpoliert linear zwischen den Sollwerten und schließt die Regelkreise.

Grenzen:

- Das gestreamte Profil hat eine Schrittweite von 10 ms. Innerhalb eines Schritts ist die Geschwindigkeit konstant, die Beschleunigung ändert sich also an jedem Schritt. Es gibt keine Ruckbegrenzung und kein Input Shaping der PLCopen-Fahrten.
- Eine Fahrt, die eine schnelle Fahrt zu einem näheren Ziel abbricht oder umkehrt, bremst mit `Deceleration` und kann überschwingen. Verlässt das Überschwingen den Achsbereich, lehnt der Regler den Sollwert ab und die Achse geht nach ErrorStop (ErrorID 6).
- Keine gepufferten oder überschliffenen Fahrten (`BufferMode`), kein MC_MoveVelocity, MC_Halt, MC_ReadParameter, kein Getriebe und keine Kurvenscheibe.
- Die Kopplung fügt die Latenz von OPC UA hinzu: Ein Client sieht die Wirkung eines Schreibens nach ein bis zwei Schleifentakten plus seiner eigenen Pollzeit.

## Echtzeittakt und Jitter

Die Simulationsschleife wacht auf einem festen Raster von Zeitpunkten auf, einer je `--cycle-ms`. Bei jedem Aufwachen führt sie die eingetroffenen Befehle aus und rechnet dann so viele Simulationsschritte, wie die Uhrzeit seit dem letzten Aufwachen erlaubt (je 62,5 µs bei den Maschinen hier). Die Zahl der Schritte folgt also der Uhr, nicht der Zahl der Aufwachvorgänge: Ein spätes Aufwachen rechnet mehr Schritte. Ein Aufwachen, das mehr als einen Takt zu spät kommt, wird nicht mit einer Folge zusätzlicher Aufwachvorgänge nachgeholt. Fällt der Rechner weiter zurück als 0,1 s oder zwei Takte, je nachdem, was länger ist, verwirft die Simulation die Zeit darüber hinaus, statt Arbeit aufzustauen.

Die Kopplung misst die Zeit zwischen den Aufwachvorgängen seit dem Start oder seit `command.reset_pacing`:

| Knoten | Typ | Bedeutung |
| --- | --- | --- |
| `status.cycle_s` | Double | Der Nenntakt, s |
| `status.cycle_period_s` | Double | Mittlere gemessene Zeit zwischen zwei Aufwachvorgängen, s |
| `status.jitter_max_s` | Double | Größte Abweichung einer Periode vom Takt, s |
| `status.jitter_rms_s` | Double | Effektivwert dieser Abweichung, s |
| `status.overruns` | UInt64 | Aufwachvorgänge, die mehr als einen ganzen Takt zu spät kamen (Periode über dem doppelten Takt) |
| `status.cycles` | UInt64 | Gemessene Aufwachvorgänge |
| `status.realtime_factor` | Double | Simulierte Zeit durch Uhrzeit |
| `status.steps` | UInt64 | Simulationsschritte seit dem Start |
| `command.reset_pacing` | Boolean, schreiben | `true` beginnt die Messung neu |

Unter Windows folgt der Timer der Laufzeitumgebung der Systemuhr von 15,6 ms. Bei einem Takt von 10 ms kommen die Aufwachvorgänge daher 0 bis 15,6 ms zu spät; das zeigt sich als Jitter von einigen Millisekunden, während die mittlere Periode und der Echtzeitfaktor genau bleiben. Ein eigener Thread mit dem hochauflösenden Sleep der Standardbibliothek wurde erprobt und war auf einem belasteten Rechner schlechter. Unter Linux hat der Timer eine Auflösung von etwa 1 ms. Keins von beiden ist ein Echtzeitsystem: Andere Last auf dem Rechner zeigt sich unmittelbar im Jitter.

### Der Takt des Clients

Neben ihrer eigenen Schleife misst die Kopplung, wie regelmäßig die Clients sie erreichen: die Zeit zwischen einer Leseanforderung an die Variablen des Simulators und der nächsten, und dasselbe für Schreibanforderungen. Eine SPS, die in jedem Tasktakt einmal liest und schreibt, zeigt hier ihre Zykluszeit und ihren Jitter, wie der Server sie sieht: den Jitter der SPS-Task, dazu den des Netzes und der Ablaufsteuerung dieses Rechners.

| Knoten | Typ | Bedeutung |
| --- | --- | --- |
| `status.client_writes`, `status.client_reads` | UInt64 | Gemessene Abstände (eins weniger als Anforderungen) |
| `status.client_write_mean_s`, `status.client_read_mean_s` | Double | Mittlerer Abstand, s |
| `status.client_write_std_s`, `status.client_read_std_s` | Double | Standardabweichung der Abstände (der Grundgesamtheit, durch ihre Anzahl geteilt), s |
| `status.client_write_max_s`, `status.client_read_max_s` | Double | Größter Abstand, s |

- Eine Anforderung zählt einmal, wie viele Variablen sie auch liest oder schreibt. Eine SPS, die zwei Variablen mit zwei `UA_Read`-Bausteinen im selben Takt liest, sendet zwei Anforderungen und zeigt Abstände eines halben Takts: Beim Messen mit einer Anforderung je Takt lesen (`OPC_UA_ReadList` oder ein Baustein). Das Beispiel `sim_cycle.st` hat dafür einen Schalter.
- Nur das Lesen von Variablenwerten zählt; Durchsuchen (die übrigen Attribute) und Abonnements nicht. Ein Abonnement trägt den Takt des Clients nicht: Der Server tastet ab und veröffentlicht in seinem eigenen Veröffentlichungsintervall, und die Publish-Anfragen des Clients holen nur ab, was ansteht; ihr Zeitverhalten zeigte das Intervall des Servers und das Netz, nicht die SPS-Task. Ein Lesen wird gezählt, nachdem es beantwortet ist, also erscheint ein Lesen dieser Variablen nicht in seiner eigenen Antwort.
- Alle Clients zählen zusammen: Beim Messen nur die SPS verbinden (UaExpert schließen).
- `command.reset_pacing` beginnt diese Werte zusammen mit denen der Schleife neu, im Schreibzugriff selbst.
- Die Kopplung gibt sie beim Beenden aus (siehe [Kopplung starten](#kopplung-starten)).

## Beispiele in Strukturiertem Text

[`crates/sim-opcua/examples/plc/twincat/sim_plcopen.st`](../../crates/sim-opcua/examples/plc/twincat/sim_plcopen.st) ist für TwinCAT 3 mit der Bibliothek Tc3_PLCopen_OpcUa (UA_Connect, UA_GetNamespaceIndex, UA_NodeGetHandle, UA_Read, UA_Write, UA_NodeReleaseHandle, UA_Disconnect). [`crates/sim-opcua/examples/plc/codesys/sim_plcopen.st`](../../crates/sim-opcua/examples/plc/codesys/sim_plcopen.st) verwendet die PLCopen-Bausteine für OPC-UA-Clients, wie die PLCopen-Spezifikation OPC 30001 (PLCopen-CFB) v1.02 sie benennt, für eine CODESYS-Bibliothek, die sie bereitstellt.

Beide Dateien enthalten:

| Baustein | Was er tut |
| --- | --- |
| `SIM_Access` | Ein Lesen oder Schreiben einer Variablen: Knoten-Handle holen, Wert lesen oder schreiben, Handle freigeben |
| `SIM_Handshake` | Der Handshake eines Bausteins am Simulator, wie oben beschrieben |
| `SIM_MC_Power`, `SIM_MC_Home`, `SIM_MC_MoveAbsolute`, `SIM_MC_MoveRelative`, `SIM_MC_Stop`, `SIM_MC_Reset` | Hüllen mit den PLCopen-Ein- und -Ausgängen, dazu Verbindungs-Handle, Namensraumindex und Achsen-ID |
| `PRG_SimDemo` | Verbindet, schaltet die Achse ein, referenziert sie, startet eine Fahrt, stoppt sie nach 0,5 s, schaltet aus und trennt |

**Diese Dateien wurden nicht übersetzt und können es hier nicht werden.** TwinCAT und CODESYS lassen sich nur mit der Anmeldung an einem Herstellerkonto herunterladen, die die Entwicklung dieses Projekts nicht hat. Am 14. November 2026 wurden die Namen stattdessen gegen die öffentliche Dokumentation geprüft:

- TwinCAT: Beckhoff Infosys, TF6100 TwinCAT 3 OPC UA Client (Dokumentation 1.3.0 vom 8. April 2026), und Beckhoffs `TF6100_Samples` auf GitHub. Jeder Name ist geklärt bis auf einen: ob `UA_Write` als `cbData` eines STRING `LEN` oder `SIZEOF` nimmt, sagt die Dokumentation nicht (mit `CHECK` markiert).
- CODESYS: Die öffentliche Hilfe von CODESYS zeigt nur seine objektorientierte Bibliothek „OPC UA Client“ (Client, Read, Write, ...), nicht die `UA_*`-Bausteine von PLCopen; ob eine CODESYS-Bibliothek sie anbietet, unter welchem Namensraumpräfix, ließ sich nicht prüfen (mit `CHECK` am Anfang jeder Datei markiert). Die Namen folgen der PLCopen-Spezifikation v1.02, die auch `UA_NamespaceGetIndex`, `UA_NodeGetHandle`, `UA_Read` und `UA_Write` als auslaufend zugunsten ihrer `*List`-Varianten führt.

Die Dateien sind ein Ausgangspunkt, der auf einer echten SPS zu übersetzen und zu prüfen ist. Jeder Zugriff holt ein Knoten-Handle und gibt es wieder frei, was drei Umläufe kostet; Produktionscode behält die Handles. Server-URL, Achsen-ID und Lagen sind an die Maschine anzupassen.

`sim_cycle.st` in jedem Ordner fügt `PRG_SimCycle` hinzu, ein Programm für eine 10-ms-Task, das tut, was ein SPS-Programm mit der Maschine tut: sich mit Basic256Sha256 SignAndEncrypt (oder None) und, falls gesetzt, Benutzername und Passwort verbinden; den Namensraumindex ermitteln; die Antriebe mit MC_Power einschalten; MC_MoveAbsolute fahren; eine Bahn schreiben (`command.path`) und fahren (`command.run_path`), bis `status.busy` fällt; ausschalten und trennen. Währenddessen liest `SIM_CyclicRead` in jedem Takt die Achslage (`signal.axis.<id>.position`) und den Schleppfehler (`signal.axis.<id>.following_error`) über Knoten-Handles, die einmal geholt werden, und das Programm merkt sich den größten Schleppfehler. Es verwendet die Bausteine aus `sim_plcopen.st`. Die Bahn ist für das T-Portal geschrieben (Achsen x und z, Startpose x 0,1 m, z 0,35 m); für eine andere Maschine sie und das Ziel ändern.

## Die Beispiele gegen sim-opcua auf demselben Rechner fahren

Beide brauchen die SPS-Laufzeit auf diesem Rechner; nichts davon liegt diesem Projekt bei. Die Schritte folgen der Dokumentation der Hersteller und konnten hier nicht ausprobiert werden.

**1. Die Kopplung starten**, in einem Terminal im Ordner des Repositorys, für die erste Verbindung mit `--trust-all`:

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --trust-all --lang de
```

Sie lauscht auf `opc.tcp://127.0.0.1:4840/`, der URL in `PRG_SimCycle`, und nennt den pki-Ordner.

**2a. TwinCAT 3 (TF6100).**

1. In TwinCAT XAE (Visual Studio) ein neues TwinCAT-XAE-Projekt anlegen und ein SPS-Projekt hinzufügen (Standard PLC Project).
2. Unter References `Tc3_PLCopen_OpcUa` hinzufügen. TF6100 braucht eine Lizenz; eine 7-Tage-Testlizenz lässt sich unter System > License aktivieren.
3. Den Code einfügen: TwinCAT importiert `.st`-Dateien nicht direkt. Für jeden `FUNCTION_BLOCK` und jedes `PROGRAM` aus `twincat/sim_plcopen.st` und `twincat/sim_cycle.st` eine POU dieses Namens anlegen, die Deklaration (vom Kopf bis `END_VAR`) in den oberen Teil und den Rumpf in den unteren Teil kopieren. `E_SimKind` in eine DUT.
4. `PRG_SimCycle` aus der SPS-Task aufrufen und deren Zykluszeit auf 10 ms setzen.
5. Die Konfiguration aktivieren, das Zielsystem in den Run-Modus setzen (die lokale Laufzeit braucht einen freien Kern oder Hyper-V-Einstellungen, wie Beckhoff sie beschreibt), einloggen und starten.
6. TF6100 erzeugt beim ersten Start sein eigenes Client-Zertifikat, in seinem PKI-Ordner: `%TcInstallDir%\3.1\Target\OpcUa\Client\PKI` für Client 2.x, `%ProgramData%\Beckhoff\TF6100-OPC-UA\TcOpcUaClient\PKI` für 1.x. Beim ersten gesicherten Verbindungsaufbau legt es das Zertifikat des Servers in seinen `rejected`-Ordner: Es dort nach `trusted` verschieben und erneut verbinden.
7. Beobachten, wie `nStep` auf 100 geht, `fbPosition.fValue` der Fahrt und der Bahn folgt, und `fLargestError`. Im Terminal der Kopplung liegt das Client-Zertifikat jetzt in `pki/trusted`: Von nun an die Kopplung ohne `--trust-all` starten.

**2b. CODESYS Control Win.**

1. CODESYS Development System V3.5 mit CODESYS Control Win (SoftSPS) installieren und die SoftSPS über ihr Taskleistensymbol starten.
2. Ein Standardprojekt für das Gerät „CODESYS Control Win V3 x64“ anlegen und im Bibliotheksverwalter eine Bibliothek hinzufügen, die die PLCopen-Bausteine für OPC-UA-Clients bereitstellt (`UA_Connect`, `UA_NodeGetHandle`, ...). Die eigene Bibliothek „OPC UA Client“ von CODESYS ist objektorientiert und tut das nicht, soweit ihre öffentliche Hilfe zeigt: Dann sind die Dateien auf ihre Bausteine Client, Read und Write umzuschreiben.
3. Für jeden Baustein aus `codesys/sim_plcopen.st` und `codesys/sim_cycle.st` eine POU anlegen und Deklaration und Rumpf einfügen; `E_SimKind` in eine DUT.
4. In der Taskkonfiguration eine zyklische Task von 10 ms anlegen, die `PRG_SimCycle` aufruft.
5. Für die gesicherte Verbindung unter Ansicht > Security Screen, Registerkarte Geräte, das Zertifikat des Clients für das Gerät erzeugen. Beim ersten Versuch landet das Zertifikat des Servers (`pki/own/cert.der`) dort unter „Zertifikate in Quarantäne“: Es nach „Vertrauenswürdige Zertifikate“ ziehen. Oder für einen ersten Test am Endpunkt ohne Sicherheit `xSecure := FALSE` setzen.
6. Einloggen, laden, starten. Beobachten, wie `iStep` auf 100 geht, `fbPosition.lrValue` und `lrLargestError`.

**3. Den Takt-Jitter ablesen.** `bReadFollowingError` (TwinCAT) oder `xReadFollowingError` (CODESYS) auf FALSE setzen, damit das Programm eine Leseanforderung je Takt sendet, und die Kopplung nach dem Lauf mit Strg+C beenden: Die Zeile `Client-Lesezugriffe ...` nennt die Zahl der Abstände, ihr Mittel (der Tasktakt, 10 ms, wenn jedes Lesen innerhalb eines Takts zurückkommt; 20 ms, wenn es bis zu zwei braucht), ihre Standardabweichung und den größten. Während das Programm läuft, stehen dieselben Werte in `status.client_read_*`.

## Dauertest

```sh
cargo test -p sim-opcua --release --test soak -- --nocapture
SOAK_SECONDS=3600 cargo test -p sim-opcua --release --test soak -- --nocapture
```

Der Test bedient das T-Portal und fährt beide Achsen gemeinsam mit MC_MoveAbsolute zwischen 30 % und 70 % ihres Bereichs, so schnell der Handshake erlaubt, für `SOAK_SECONDS` (Standard 20). Er scheitert, wenn:

- ein Baustein oder eine Achse einen Fehler meldet oder `status.refused` nicht leer ist;
- eine Achse nach einer Fahrt mehr als 0,2 mm von ihrem Ziel steht;
- der Echtzeitfaktor um mehr als 2 % von 1 abweicht oder die Schrittzahl um mehr als 1 s plus 2 % von der Laufzeit abweicht;
- die mittlere Periode um mehr als 2 ms vom Takt abweicht, der effektive Jitter 10 ms, der größte Jitter 100 ms übersteigt oder mehr als 10 % der Aufwachvorgänge Überläufe sind;
- der belegte Speicher um mehr als 64 MB wächst (nur unter Linux gemessen).

Ergebnis eines Laufs von 20 s unter Windows 11, während alle Kerne mit anderen Builds belegt waren: 26 Fahrten, größte Zielabweichung 0,084 mm, mittlere Periode 10,017 ms, größter Jitter 16,1 ms, effektiver Jitter 4,6 ms, 42 Überläufe in 2030 Takten, Echtzeitfaktor 1,0000, 328898 Schritte. Der Speicher wird unter Windows nicht gemessen. Läufe unterscheiden sich im Jitter mit der Last auf dem Rechner.

## Feldbus

Ein simulierter Antrieb an EtherCAT oder PROFINET statt an OPC UA ist in [Feldbus-Emulation](feldbus-emulation.md) besprochen.
