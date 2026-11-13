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
| `--port N` | 4840 | TCP-Port |
| `--host IP` | 127.0.0.1 | Adresse, auf der der Server lauscht. Eine SPS im Netz braucht eine Adresse dieses Rechners, die sie erreicht. Die Adresse selbst angeben, nicht `0.0.0.0`: Der Server nennt sie in seiner Endpunkt-URL |
| `--lang en\|de` | en | Sprache der Meldungen und der Variablenbeschriftungen |
| `--cycle-ms N` | 10 | Takt der Simulationsschleife, 1 bis 1000 ms |

Strg+C beendet die Kopplung und gibt den gemessenen Takt in einer Zeile dieser Form aus (die Zahlen sind die des Dauertests unten, nicht eines Laufs mit Strg+C):

```text
Echtzeittakt (Taktdauer, größter und effektiver Jitter in ms, Überläufe, Echtzeitfaktor): 10.017 · 16.116 · 4.581 · 42 · 1.0000
```

## PLCopen-Achsen

Jede Achse der Maschine hat einen Ordner `axis.<Achsen-ID>` mit einem Satz Handshake-Variablen je Funktionsbaustein. Die Knoten-IDs sind Zeichenketten im Namensraum `urn:kinematics-simulation`, zum Beispiel `axis.x.MoveAbsolute.Execute`. Die Namen sind die englischen PLCopen-Namen und in beiden Sprachen gleich.

| Knoten | Typ | Zugriff | Bedeutung |
| --- | --- | --- | --- |
| `axis.<id>.Power.Enable` | Boolean | schreiben | MC_Power.Enable: `true` schaltet die Achse ein |
| `axis.<id>.Power.Status` | Boolean | lesen | Enable ist true und die Antriebe sind an |
| `axis.<id>.Power.Valid` | Boolean | lesen | Status und kein Achsfehler |
| `axis.<id>.Power.Error`, `.ErrorID` | Boolean, UInt16 | lesen | Der Achsfehler, solange Enable true ist |
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

So verhalten sich die Bausteine:

- Eine steigende Flanke an `Execute` startet einen Baustein mit den Eingängen, die er in diesem Moment hat. Erneutes Schreiben von `true`, während `Execute` true ist, bewirkt nichts.
- `Done`, `CommandAborted` und `Error` bleiben true, solange `Execute` true ist. Fällt `Execute`, werden sie gelöscht, sobald der Baustein fertig ist. Fällt `Execute`, während der Baustein arbeitet, läuft er weiter, und seine Ausgänge bleiben bis zur nächsten steigenden Flanke stehen, damit ein pollender Client sie sieht.
- Die Ausgänge werden einmal je Schleifentakt veröffentlicht. Ein Client, der `Done` gleich nach dem Schreiben von `Execute` liest, kann den Wert des vorigen Befehls lesen. Der Handshake, der funktioniert: `Execute` auf false schreiben, warten, bis `Done` false liest, die Eingänge und `Execute` true schreiben, dann `Busy`, `Done`, `CommandAborted` und `Error` pollen. Die Beispiele in Strukturiertem Text tun das.
- Fahrten und Stop übernehmen die laufende Bewegung mit der Lage und Geschwindigkeit, die die Achse hat. Der Baustein, der lief, meldet `CommandAborted`, außer es ist derselbe Baustein, neu gestartet: Der beginnt neu mit gelöschten Ausgängen.
- MC_Stop bremst mit `Stop.Deceleration` bis zum Stillstand und meldet dann `Done`. Die Achse bleibt in `Stopping`, solange `Stop.Execute` true ist, und lehnt dort Fahrten ab (ErrorID 2). Sie geht nach `Standstill`, wenn `Stop.Execute` fällt.
- MC_Home fährt die Achse in ihre Grundstellung, mit der halben Geschwindigkeits- und Beschleunigungsgrenze der Achse, und setzt `IsHomed`. Die Grundstellung ist der Achswert der Startpose der Maschine (`[start] pose` in der Konfiguration). Es sucht keinen Schalter und setzt keinen neuen Nullpunkt: Lagen im Simulator sind absolut.
- MC_Reset löscht den Achsfehler und verlässt ErrorStop nach Standstill (oder Disabled, wenn `Power.Enable` false ist). Nach ErrorID 7 schaltet es die Antriebe wieder ein.
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

## Beispiele in Strukturiertem Text

[`crates/sim-opcua/examples/plc/twincat/sim_plcopen.st`](../../crates/sim-opcua/examples/plc/twincat/sim_plcopen.st) ist für TwinCAT 3 mit der Bibliothek Tc3_PLCopen_OpcUa (UA_Connect, UA_GetNamespaceIndex, UA_NodeGetHandle, UA_Read, UA_Write, UA_NodeReleaseHandle, UA_Disconnect). [`crates/sim-opcua/examples/plc/codesys/sim_plcopen.st`](../../crates/sim-opcua/examples/plc/codesys/sim_plcopen.st) ist für CODESYS mit dem CODESYS OPC UA Client SL, der die PLCopen-Bausteine für OPC-UA-Clients bereitstellt.

Beide Dateien enthalten:

| Baustein | Was er tut |
| --- | --- |
| `SIM_Access` | Ein Lesen oder Schreiben einer Variablen: Knoten-Handle holen, Wert lesen oder schreiben, Handle freigeben |
| `SIM_Handshake` | Der Handshake eines Bausteins am Simulator, wie oben beschrieben |
| `SIM_MC_Power`, `SIM_MC_Home`, `SIM_MC_MoveAbsolute`, `SIM_MC_MoveRelative`, `SIM_MC_Stop`, `SIM_MC_Reset` | Hüllen mit den PLCopen-Ein- und -Ausgängen, dazu Verbindungs-Handle, Namensraumindex und Achsen-ID |
| `PRG_SimDemo` | Verbindet, schaltet die Achse ein, referenziert sie, startet eine Fahrt, stoppt sie nach 0,5 s, schaltet aus und trennt |

**Diese Dateien wurden nicht übersetzt.** Weder TwinCAT noch CODESYS standen hier zur Verfügung. Sie folgen der Dokumentation der Bibliotheken und IEC 61131-3; Namen, die sich zwischen Bibliotheksversionen unterscheiden können, sind mit `CHECK` markiert. Jeder Zugriff holt ein Knoten-Handle und gibt es wieder frei, was drei Umläufe kostet; Produktionscode behält die Handles. Server-URL, Achsen-ID und Lagen sind an die Maschine anzupassen.

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
