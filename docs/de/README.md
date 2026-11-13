# Portal-Physiksimulator

![Das T-Portal fährt seinen Pick-and-Place-Zyklus in der App](../images/t-gantry-cycle.gif)

Ein Simulator für Bewegungsmaschinen. Drehmoment, Drehzahl und Schleppfehler ergeben sich aus einem physikalischen Modell: Riementrume als Federn, Führungsreibung nach LuGre, ein Getriebe mit Spiel, eine Motorwicklung und ein Servoantrieb mit Kaskadenregelung.
Er enthält zwölf Maschinen: ein T-Portal mit zwei Motoren (X und Z an einem Zahnriemen), eine Einzelachse mit Zahnriemen, vier Delta-Picker (den D3 mit drei Armen, ausgelegt wie ein Autonox RL3-1200-3kg, einen D4 mit vierter Achse, die das Werkzeug dreht, einen planaren D2 mit zwei Armen, ausgelegt wie ein Codian D2-800, und einen Linear-Delta auf drei Zahnriemensäulen, ausgelegt wie ein Festo EXPT-95) und, aus einem Baukasten von Gelenken, eine Kugelgewindeachse, eine Zahnstangenachse, eine Linearmotorachse, einen Rundtisch, einen Sechsachsroboter aus einer URDF-Datei und einen SCARA ([Mechanismen](mechanismen.md)). Weitere Maschinen beschreiben [Maschinendateien](maschinendateien.md) ohne Rust-Code: ein H-Portal (H-Bot), einen CoreXY-Tisch, ein XYZ-Portal, ein Portal mit Kugelgewinde-Z und ein Portal mit Zahnstange (`machines/*.toml`).
Dazu gibt es sechs Bedienoberflächen: eine Desktop-App mit 3D-Ansicht und Oszilloskopen, ein Befehlszeilenprogramm, das CSV schreibt, einen OPC-UA-Server mit PLCopen-artigen Achsen für eine SPS, einen Export als FMU nach FMI 2.0 und 3.0 (Co-Simulation) für Simulink, OpenModelica, Dymola und andere Werkzeuge, ein Python-Modul und eine Auslegungsprüfung im Browser.
Das Programm ist in Rust geschrieben. Herstellerdaten zu Motoren, Getrieben, Antrieben und Riemenachsen liegen in [`catalog/`](../../catalog/).
**Schnellstart:** die Auslegungsprüfung im Browser unter **https://sparkz400.github.io/KinematicsSimulation/** ausprobieren (ohne Installation), die Windows- oder Linux-Version unter [Releases](https://github.com/Sparkz400/KinematicsSimulation/releases) herunterladen oder selbst bauen mit `cargo run --release -p sim-app` (siehe [Bauen und starten](#bauen-und-starten)).

![Der Delta-Picker in der App: Bewegung, 3D-Ansicht, Zyklusanalyse und Oszilloskope](../images/delta-d3-app.png)

*Der Delta-Picker (`delta-d3`) in der App. Seine mitgelieferte Konfiguration besteht eine Prüfung nicht, das Einschwingen an der Greifposition (0,68 mm gegen 0,5 mm); die obere Leiste zeigt das an.*

Diese Seite ist die deutsche Einstiegsseite. Die vollständige Projektbeschreibung ist das englische [README](../../README.md).

## Was das Programm aussagen kann und was nicht

Für eine Maschine, die in einer Konfigurationsdatei beschrieben ist, liefert es:

- Spitzenmoment, Effektivmoment und kubisches Mittel des Moments, Drehzahl, Strom, Übertemperatur der Wicklung und Energie je Motor über einen Zyklus oder eine Bahn
- wie viel vom Moment jedes Motors auf Trägheit, Schwerkraft, Reibung und Riemen entfällt, wo die Energie eines Zyklus verloren geht (jede Führung, Welle, jedes Getriebe, jeder Dämpfer, die Wicklungen, der Bremswiderstand) und was die Reibung jeder Führung an ihrem Arbeitspunkt tut
- ob dieser Zyklus die Grenzwerte der Bauteile in der Konfiguration einhält (Drehmoment-Drehzahl-Kennlinie, Wicklungstemperatur, Getriebe, Riemenkraft, schlaffes Trum und Zahnbelastung, Zwischenkreis und Bremswiderstand, Führungsbelastung und Radialkraft auf die Welle, Einschwingen an der Greifposition)
- für die Gelenkmaschinen zusätzlich die eigenen Prüfungen ihres Getriebes: Drehzahl, DN-Wert, Knickung und Lebensdauer eines Kugelgewindetriebs, Vorschubkraft einer Zahnstange, Lebensdauer eines Wellgetriebes
- wie viele Stunden die Lager der Scheibenwellen, die Planetenlager des Getriebes und die Führungen in diesem Zyklus halten, gegen eine geforderte Lebensdauer (ISO 281, die Getriebeformel von Siemens, die Formel der Führungshersteller; für den Riemen gibt es stattdessen eine zulässige Zahnbelastung, weil kein Riemenhersteller eine Lebensdauer veröffentlicht)
- wie sich diese Ergebnisse verschieben, wenn ein unsicherer Wert an seiner unteren oder oberen Grenze liegt, oder wenn alle unsicheren Werte zugleich abweichen
- wie weit die Werkzeugspitze von ihrer Bahn entfernt ist und was Reglerverstärkungen, Überschleifen und Input Shaping daran ändern
- welcher Katalogmotor mit seinem Antrieb (und Getriebe), auf welcher Katalogachse, der günstigste ist, der besteht; das eingebaute Gewicht steht für den Preis

Die Gleichungen sind gegen Handrechnungen, eine unabhängige Referenz in SciPy und Herstellerformeln geprüft; Einzelheiten stehen in [validation.md](../validation.md). Diese Prüfungen belegen, dass die Gleichungen richtig gelöst und die Katalogdaten richtig gelesen werden. Sie belegen nicht, dass die Parameter eine reale Maschine beschreiben.

**Es wurde keine reale Maschine vermessen.** Reibung, Dämpfung, Steifigkeit von Führung und Gestell, Riemenvorspannung, Wicklungswiderstand und Wicklungsinduktivität sowie die Daten von Zwischenkreis und Bremswiderstand sind Schätzungen. Die Konfigurationen sagen das selbst: [`exct-30.toml`](../../crates/machine-t-gantry/exct-30.toml) kennzeichnet jeden Wert mit `[catalogue]` oder `[estimate]`, und jede Konfiguration hat eine Tabelle `[bounds]` mit einer unteren und einer oberen Grenze für ihre Schätzwerte. Tendenzen und Vergleiche (dieser Motor gegen jenen, mit und ohne Input Shaping) sind belastbar. Absolute Zahlen sind so gut wie die Schätzungen. Mit `verdicts --bands` sehen Sie, wie weit sich ein Ergebnis mit ihnen verschiebt.

Weitere Grenzen, die zählen:

- Die Standardkonfigurationen der beiden Riemenmaschinen sind erfundene Platzhaltermaschinen, keine Produkte. Die Konfiguration des Deltas ist wie ein Produkt ausgelegt (Autonox RL3-1200-3kg, Siemens-Motor und -Getriebe aus dem Katalog), aber Armlängen, Massen und Trägheiten sind Schätzungen: Autonox gibt sie nur nach Registrierung heraus. Die Gelenkmaschinen (Gewindetrieb, Zahnstange, Linearmotor, Rundtisch, Roboter) sind ausgelegte Beispiele, deren Mechanik ein typisches Teil dieser Größe schätzt, nicht den Artikel eines Herstellers; die Roboter werden im Gelenkraum ausgelegt, ohne kartesische Bahnen.
- Der Motor ist eine einzige Ersatzwicklung (R, L, Gegen-EMK), die den Effektivwert des Strangstroms führt und höchstens das sqrt(3/2)-Fache der Zwischenkreisspannung erhält. Es gibt kein Drehstrommodell, keine PWM und keine Feldschwächung.
- Die Lebensdauern sind die Formeln der Hersteller und nicht mehr: keine Lebensdauerbeiwerte, ein Lager je Scheibenwelle, Tragzahlen, die zum Teil Schätzungen sind (Festo veröffentlicht für die EXCT keine). Sie sind nicht validiert.
- Die OPC-UA-Anbindung wurde gegen einen Client geprüft, der mit derselben Bibliothek geschrieben ist, nicht gegen eine SPS.
- Die FMU wurde mit dem Validator und dem Simulator von fmpy und mit OMSimulator (dem Co-Simulationswerkzeug von OpenModelica) geprüft; in Simulink oder Dymola importiert oder mit `fmusim` der Modelica Association oder dem FMU Compliance Checker geprüft wurde sie nicht ([FMI-Export](fmi.md)).
- Die Zahlen im Browser stimmen mit denen der Befehlszeile auf vier bis fünf signifikante Stellen überein, nicht Bit für Bit ([Auslegung im Browser](web.md)).
- Die Mausbedienung der App (Kamera, Bedienfelder) hat kein Mensch geprüft; Bildschirmfotos wurden geprüft.
- Das Einlesen von Antriebsaufzeichnungen und die Parameteranpassung wurden an Aufzeichnungen geprüft, die der Simulator selbst erzeugt hat.

## Bauen und starten

Benötigt wird eine stabile Rust-Toolchain, die die Edition 2024 unterstützt. Alle Versionen der Abhängigkeiten sind in [`Cargo.toml`](../../Cargo.toml) festgelegt. Die Katalogdaten sind einkompiliert; die Programme brauchen also keine Datendateien.

```sh
cargo run --release -p sim-app                    # die Desktop-App
cargo run -q --release -p sim-cli -- list         # das Befehlszeilenprogramm
cargo run --release -p sim-opcua -- t-gantry      # der OPC-UA-Server unter opc.tcp://127.0.0.1:4840/
cargo build --release -p sim-cli -p sim-fmi       # dann: target/release/sim-cli fmu t-gantry --out t-gantry.fmu
cargo test --workspace                            # alle Tests
```

`cargo test --workspace` baut auch die App mit ihren Grafikbibliotheken. Um nur die Physik, die Analyse und die Befehlszeile zu testen:

```sh
cargo test -p sim-core -p sim-components -p machine-t-gantry -p machine-single-axis -p machine-delta-d3 -p machine-joint-kit -p sim-netlist -p sim-machines -p sim-cli
```

Die App lässt sich im laufenden Betrieb auf Deutsch umschalten (Sprache). Das Befehlszeilenprogramm und der OPC-UA-Server geben ihre Meldungen mit `--lang de` auf Deutsch aus:

```text
$ cargo run -q --release -p sim-cli -- list --lang de
t-gantry	T-Portal
single-axis	Einzelachse
delta-d3	Delta-Picker (3 Arme)
screw-axis	Kugelgewindeachse
rack-axis	Zahnstangenachse
linear-motor-axis	Linearmotorachse
rotary-table	Rundschalttisch
urdf-arm	Sechsachsroboter (URDF)
scara	SCARA-Roboter (URDF)
```

Alle Befehle schreiben CSV auf die Standardausgabe oder mit `--out FILE` in eine Datei. Meldungen und Fehler gehen auf die Fehlerausgabe. Die Spaltennamen der CSV-Ausgabe sind englische Bezeichner und werden nicht übersetzt.

## Wie es weitergeht

| Dokument | Wofür |
| --- | --- |
| [Benutzerhandbuch](benutzerhandbuch.md) | Deutsch. Ein Abschnitt je Aufgabe: einen Motor auslegen, eine Katalogmaschine prüfen, suchen, Fehlerbänder, einen Zyklus verkürzen, eine Bahn fahren, Regler einstellen, eine Aufzeichnung nachfahren und Parameter anpassen, eine SPS anbinden, die App bedienen |
| [Python-Modul](python.md) | `kinematics_sim`: Konfigurationen, Läufe, Analyse, Prüfurteile, Bänder und Auslegung als NumPy-Arrays und Tabellen für pandas; ein Notebook mit einer Parameterstudie |
| [Auslegung im Browser](web.md) | Der Simulator als WebAssembly und eine statische Seite mit der Auslegungsprüfung |
| [FMI-Export](fmi.md) | FMUs nach FMI 2.0 und 3.0: Variablen, FMU-Zustand, Richtungsableitungen, Prüfung, Import in Simulink, OpenModelica und Dymola |
| [Virtuelle Inbetriebnahme mit einer SPS](sps.md) | PLCopen-artige Bausteine je Achse über OPC UA, Echtzeittakt und Jitter, Beispiele für TwinCAT und CODESYS, Dauertest |
| [Feldbus-Emulation](feldbus-emulation.md) | Entwurfsnotiz: einen EtherCAT- oder PROFINET-Antrieb für eine echte SPS nachbilden |

Die Referenzdokumente gibt es nur auf Englisch:

| Dokument | Wofür |
| --- | --- |
| [Maschinendateien](maschinendateien.md) | Deutsch. Eine Maschine in einer TOML- oder JSON-Datei beschreiben, wie eine Netzliste aus Bauteilen verschaltet, ohne Rust-Code; das Format, seine Prüfungen, ein durchgerechnetes Beispiel |
| [config-reference.md](../config-reference.md) | Jede Tabelle und jeder Schlüssel der Konfiguration, mit Einheit, Bedeutung, Quelle und dem zugehörigen Effektschalter |
| [model-reference.md](../model-reference.md) | Die Gleichungen, je Komponente und je Maschine, mit Annahmen und bekannten Vereinfachungen |
| [mechanismen.md](mechanismen.md) | Kugelgewindetrieb, Zahnstange, Linearmotor, Rundtisch und Roboter aus URDF-Dateien: die Maschinen, ihre Formeln, Prüfungen und Grenzen |
| [catalogue.md](../catalogue.md) | Was in `catalog/` liegt, woher es stammt, gefundene Druckfehler, was fehlt |
| [validation.md](../validation.md) | Jede Prüfung gegen etwas Unabhängiges, und was nicht validiert ist |
| [user-guide.md](../user-guide.md) | Die englische Fassung des Benutzerhandbuchs |
| [README](../../README.md) | Die englische Projektseite mit einem Rundgang in fünf Minuten und dem Aufbau des Quelltexts |
