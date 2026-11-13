# FMI-Export

`sim-cli fmu` schreibt eine Maschine mit ihrer Konfiguration als FMU (Functional Mock-up Unit) für FMI 2.0 Co-Simulation (die Vorgabe) oder FMI 3.0 Co-Simulation. Beide kommen aus einer Binärdatei, `sim_fmi` (`crates/sim-fmi`), die die C-Funktionen `fmi2*` und `fmi3*` nebeneinander exportiert. Die FMU rechnet dieselbe Simulation wie `sim-cli run`: Antriebe, Regler, jeden eingeschalteten Effekt. Das [Benutzerhandbuch](benutzerhandbuch.md#das-modell-in-simulink--modelica-über-fmi-nutzen) erklärt die Schritte, die Sollwerteingänge und die Parameter; diese Seite ergänzt die beiden Versionen, den FMU-Zustand, die Ableitungen, das fehlende Model Exchange, die Prüfung und den Import in andere Werkzeuge.

## Eine FMU schreiben

```sh
cargo build --release -p sim-fmi -p sim-cli
sim-cli fmu t-gantry --out t-gantry.fmu              # FMI 2.0 Co-Simulation
sim-cli fmu t-gantry --fmi 3 --out t-gantry-3.fmu    # FMI 3.0 Co-Simulation
```

`--fmi` nimmt `2` oder `3`; alles andere wird abgelehnt. `--binary DATEI` nennt die Binärdateien wie bisher. Die Binärdateien kommen in diese Ordner:

| Binärdatei | FMI 2.0 | FMI 3.0 |
| --- | --- | --- |
| `sim_fmi.dll` | `binaries/win64/sim_fmi.dll` | `binaries/x86_64-windows/sim_fmi.dll` (oder `aarch64-windows`) |
| `libsim_fmi.so` | `binaries/linux64/sim_fmi.so` | `binaries/x86_64-linux/sim_fmi.so` (oder `aarch64-linux`) |
| `libsim_fmi.dylib` | `binaries/darwin64/sim_fmi.dylib` | `binaries/x86_64-darwin/sim_fmi.dylib` oder `binaries/aarch64-darwin/sim_fmi.dylib` |

Für FMI 3.0 wird die Architektur aus dem Kopf der Binärdatei gelesen (dem Maschinenfeld einer PE-Datei, `e_machine` einer ELF-Datei, `cputype` einer 64-Bit-Mach-O-Datei). Eine Datei ohne einen solchen Kopf, darunter eine universelle (fat) Mach-O-Datei, wird abgelehnt: FMI 3.0 will einen Ordner je Architektur, also je Architektur eine `.dylib` bauen und beide übergeben.

## Variablen

Die Variablen sind in beiden Versionen dieselben; nur die Typnamen unterscheiden sich. Die Wertreferenz einer Variablen ist ihr Platz in der Liste.

| Variable | Typ FMI 2.0 | Typ FMI 3.0 | Kausalität | Variabilität |
| --- | --- | --- | --- | --- |
| `time` | Real | Float64 | independent | continuous |
| jedes andere Signal (`axis.x.position`, `motor.a.torque`, ...) | Real | Float64 | output | continuous |
| `command.setpoint.<Achs-ID>` | Real | Float64 | input | continuous |
| `command.enable` | Boolean | Boolean | input | discrete |
| `machine`, `config_file` | String | String | parameter | fixed |
| `effects.<Name>` | Boolean | Boolean | parameter | fixed |

In FMI 3.0 steht der Startwert eines Strings in einem Element `<Start value="..."/>`, wie das Schema von FMI 3.0 es verlangt. Die Modellstruktur führt jeden Ausgang als Ausgang und als Anfangsunbekannte. Die Ausgänge tragen `dependencies=""`: Kein Ausgang hängt direkt von einem Eingang ab, weil ein Sollwert die Ausgänge erst über den nächsten Kommunikationsschritt erreicht. Die Anfangsunbekannten tragen keine Abhängigkeitsliste; sie dürfen also von jedem Parameter abhängen.

Die Modellbeschreibung von FMI 3.0 hat `instantiationToken`, wo FMI 2.0 `guid` hat; beide sind ein Hash der Modellbeschreibung. Die FMU für FMI 2.0 und die für FMI 3.0 derselben Maschine und Konfiguration haben also verschiedene Kennungen und lehnen die Instanziierung und die FMU-Zustände der jeweils anderen ab.

## Fähigkeiten

| Attribut der Modellbeschreibung | FMI 2.0 | FMI 3.0 |
| --- | --- | --- |
| Co-Simulation | ja | ja |
| Model Exchange, Scheduled Execution | nein | nein |
| `canHandleVariableCommunicationStepSize` | true | true |
| `fixedInternalStepSize` | (nicht in FMI 2.0) | `sim.dt_s`, zum Beispiel 6.25e-05 |
| `canGetAndSetFMUstate` / `canGetAndSetFMUState` | true | true |
| `canSerializeFMUstate` / `canSerializeFMUState` | true | true |
| `providesDirectionalDerivative` / `providesDirectionalDerivatives` | true | true |
| `providesAdjointDerivatives` | (nicht in FMI 2.0) | true |
| `hasEventMode` | (nicht in FMI 2.0) | false |
| `canInterpolateInputs`, `maxOutputDerivativeOrder` | false, 0 | (nicht verwendet), 0 |

Funktionen von FMI 3.0, die etwas brauchen, was diese FMU nicht hat, geben `fmi3Error` mit einer Meldung zurück: Event Mode (`fmi3EnterEventMode`, `fmi3EnterStepMode`, `fmi3UpdateDiscreteStates`), Configuration Mode, Takte (Clocks) mit ihren Intervallen und Verschiebungen, Ausgangsableitungen, `fmi3GetVariableDependencies` (die Modellbeschreibung enthält die Abhängigkeiten) und jede Funktion von Model Exchange. `fmi3InstantiateModelExchange` und `fmi3InstantiateScheduledExecution` melden den Grund und geben null zurück. Get- und Set-Funktionen von Typen, von denen die FMU keine Variablen hat (Float32, die Ganzzahltypen, Binary, Clock), lehnen jede Wertreferenz ab und nehmen eine leere Liste an. `fmi3DoStep` meldet `eventHandlingNeeded`, `terminateSimulation` und `earlyReturn` immer als false und `lastSuccessfulTime` als die Zeit, bei der die Simulation steht (nach einem abgelehnten Schritt die Zeit davor).

## FMU-Zustand

`fmi2GetFMUstate` / `fmi3GetFMUState` kopieren den Zustand einer Instanz; `fmi2SetFMUstate` / `fmi3SetFMUState` stellen ihn wieder her; die Funktionen zum Freigeben, zur Größe, zum Serialisieren und Deserialisieren tun, was der Standard sagt.

Was ein Zustand enthält: den Modus der Instanz, die Startzeit, jeden Wert, den der Importeur gesetzt hat (Eingänge und Parameter), und für jeden Kommunikationsschritt seit der Initialisierung die Zahl der Grundschritte, den Freigabeeingang und die Sollwerteingänge, die er angewendet hat. Beim Wiederherstellen wird aus den Parametern eine neue Simulation gebaut, und diese Schritte werden wiederholt. Die Simulation ist deterministisch, also ergibt die Wiederholung Bit für Bit die Signale der gespeicherten Simulation (geprüft in `crates/sim-fmi/tests/fmi.rs` und `tests/fmi3.rs` und über fmpy in `tools/check_fmu.py`).

Grenzen:

- Einen Zustand wiederherzustellen dauert so lange, wie vom Start bis zum gespeicherten Punkt zu simulieren, und ein Zustand wächst mit jedem Kommunikationsschritt um 13 Byte plus je Achse 9 Byte (1 Byte für eine Achse, deren Sollwert nicht gesetzt wurde). Der Grund: Die Simulation hält die Maschine als `Box<dyn Machine>`, die sich nicht kopieren lässt; eine direkte Momentaufnahme braucht kopierbare `Machine` und `Sim`. Der Code markiert das mit `shortcut:` in `crates/sim-fmi/src/api.rs`.
- Ist `config_file` gesetzt, liest die Wiederholung diese Datei erneut. Eine seit dem Zustand auf der Platte geänderte Datei ergibt eine andere Simulation oder eine Ablehnung, wenn ihre Signale abweichen.
- Ein serialisierter Zustand wird gegen die GUID oder das Instanziierungs-Token der FMU und die Zahl ihrer Variablen geprüft; ein Zustand einer anderen FMU, ein abgeschnittener Zustand oder einer mit überzähligen Bytes wird mit `fmi2Error` / `fmi3Error` abgelehnt. Die bitgleiche Wiederholung wurde innerhalb eines Builds der Binärdatei geprüft; eine Binärdatei, die mit einem anderen Compiler oder für eine andere Plattform gebaut ist, wiederholt die Schritte mit ihren eigenen Gleitkommaergebnissen.
- Das serialisierte Format ist Little-Endian: der Text `sim-fmi state 1` und ein Null-Byte, die GUID, der Modus, die Startzeit, die gesetzten Werte und die Schritte.

## Richtungs- und adjungierte Ableitungen

`fmi2GetDirectionalDerivative`, `fmi3GetDirectionalDerivative` und `fmi3GetAdjointDerivative` nehmen Ausgänge (Real / Float64, Kausalität output) als Unbekannte und Sollwerteingänge als Bekannte an, im Initialization Mode und im Step Mode, und geben für jeden Eintrag null zurück. Das ist die genaue Antwort, kein Platzhalter: In Co-Simulation ist die Ableitung die partielle Ableitung der Ausgänge am aktuellen Kommunikationszeitpunkt nach den Eingängen an diesem Zeitpunkt. Die Ausgänge sind die Signale am Ende des letzten Schritts; ein neuer Sollwert geht zu Beginn des nächsten Schritts an den Regler. Einen Eingang zu setzen ändert also keinen Ausgang, bevor `DoStep` läuft, und jede partielle Ableitung ist null. Die Tests prüfen das gegen eine von Hand gebildete finite Differenz: Sie setzen einen Sollwert um 1e-4 m höher, lesen jeden Ausgang erneut und finden die Differenz null.

Was diese Funktionen nicht liefern, ist die Empfindlichkeit der Ausgänge nach dem nächsten Schritt gegenüber den Eingängen (wie weit sich die Achse in einem Schritt je Meter Sollwert bewegt). Der FMI-Standard definiert diese Größe für Co-Simulation nicht, also wird sie unter diesen Funktionsnamen nicht angeboten. Ein Importeur, der sie braucht, kann einen FMU-Zustand nehmen, mit zwei Eingangswerten schrittweise rechnen und den Zustand wiederherstellen.

Jede andere Referenz (ein Eingang als Unbekannte, ein Ausgang oder Parameter als Bekannte, `time`) wird mit `fmi2Error` / `fmi3Error` abgelehnt.

## Warum es kein Model Exchange gibt

Model Exchange übergibt die stetigen Zustände der FMU und ihre Ableitungen dem Integrator des Importeurs. Für diesen Simulator wäre das nur mit diesen Teilen richtig:

1. Stetige Zustände: der mechanische Zustand der Maschine und je Antrieb der Wicklungsstrom und die drei Energieintegrale des Schritts (`Sim::advance_with_windings` integriert sie zusammen als einen Vektor).
2. Ein Zeitereignis bei jedem Grundschritt `sim.dt_s` (62,5 µs in den mitgelieferten Konfigurationen): Dort tastet der Regler die Motoren ab, rechnet seine Kaskadenregelkreise und die Sollwertinterpolation und setzt die Wicklungsspannungen, die dann über den Schritt gehalten werden. Integratoren, Filter, Interpolator, Antriebszustände, Zwischenkreis- und Bremswiderstandslogik des Reglers sind diskrete Zustände, die sich nur bei diesen Ereignissen ändern.
3. Weitere Sprünge bei denselben Ereignissen: `Machine::after_step` ändert den Zustand nach jedem Schritt (ein Zahnsprung des Riemens ändert die Drehzahl einer Riemenscheibe auf einmal), die Bauteiltemperaturen werden mit der genauen Lösung eines über den Schritt gehaltenen Verzögerungsglieds erster Ordnung fortgeschrieben, und das Umschalten der Nutzlast ändert die Maschine.

Punkt 2 allein bedeutet 16 000 Ereignisse je simulierter Sekunde. Der Integrator des Importeurs startet nach jedem Ereignis neu, also macht ein Löser mit variabler Schrittweite nie einen Schritt länger als 62,5 µs und behandelt dazu bei jedem ein Ereignis; Model Exchange wäre langsamer als Co-Simulation und brächte nichts dafür. Die Ergebnisse wichen auch von `sim-cli run` und von der FMU für Co-Simulation ab, weil der Integrator des Importeurs das Runge-Kutta-Verfahren vierter Ordnung mit fester Schrittweite und Teilschritten ersetzt, das der Simulator verwendet und das die Tests Bit für Bit vergleichen. Schließlich brauchen die Punkte 1 bis 3 die inneren Zustände des Reglers und der Simulation, die `sim-components` privat hält; sie zu öffnen änderte die Schnittstellen `Sim` und `Machine`, gegen die die Maschinen-Crates geschrieben sind. Aus diesen Gründen bietet dieser Export in beiden FMI-Versionen nur Co-Simulation.

## Prüfung

Durchgeführt unter Windows 11 x86_64 am 13. November 2026 mit der Binärdatei aus `cargo build --release -p sim-fmi -p sim-cli`. Die FMUs waren die Standardkonfigurationen von `t-gantry` und `single-axis` und `t-gantry` mit `crates/machine-t-gantry/exct-30.toml`, jeweils für FMI 2.0 und FMI 3.0.

Rust-Tests (`cargo test -p sim-fmi -p sim-cli --release`): die Funktionen von FMI 2.0 und FMI 3.0 so aufgerufen, wie ein Importeur sie aufruft; die Signale jeder Maschine Bit für Bit gleich einer einfachen `Sim` mit denselben Sollwerten; der FMU-Zustand Bit für Bit wiederhergestellt auf derselben Instanz und, serialisiert, auf einer zweiten Instanz; Ableitungen gegen die finite Differenz von Hand; Ablehnungen.

fmpy 0.3.32 mit numpy 2.3.4 unter Python 3.13.17, aus PyPI in eine virtuelle Umgebung installiert:

```sh
python -m venv .venv
.venv/Scripts/python -m pip install fmpy==0.3.32 numpy==2.3.4
.venv/Scripts/python tools/check_fmu.py target/release/sim-cli.exe t-gantry-3.fmu t-gantry
.venv/Scripts/fmpy validate t-gantry-3.fmu
.venv/Scripts/fmpy simulate t-gantry-3.fmu --stop-time 0.1 --output-file sim3.csv
```

`tools/check_fmu.py` prüft die FMU mit dem Validator von fmpy (das XSD der FMI-Version der FMU, Variablennamen, Modellstruktur, Startwerte, Kausalität und Variabilität, Einheiten), simuliert sie mit fmpy 1 s lang im Standardschritt, vergleicht jeden Ausgang mit `sim-cli run` und nimmt dann über die Anbindung der C-Funktionen in fmpy nach 5 Schritten einen FMU-Zustand, prüft, dass die 7 Schritte danach auf derselben Instanz und serialisiert auf einer zweiten Bit für Bit wiederkommen, und prüft, dass Richtungsableitung und finite Differenz null sind. Ergebnis für alle sechs FMUs:

```
FMU state: restored bit for bit, serialized 360 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry  (FMI 2.0): 0 validator problems, 96 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 306 bytes and restored bit for bit; directional derivative and finite difference zero
single-axis  (FMI 2.0): 0 validator problems, 51 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 358 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry crates/machine-t-gantry/exct-30.toml (FMI 2.0): 0 validator problems, 94 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 360 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry  (FMI 3.0): 0 validator problems, 96 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 306 bytes and restored bit for bit; directional derivative and finite difference zero
single-axis  (FMI 3.0): 0 validator problems, 51 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
FMU state: restored bit for bit, serialized 358 bytes and restored bit for bit; directional derivative and finite difference zero
t-gantry crates/machine-t-gantry/exct-30.toml (FMI 3.0): 0 validator problems, 94 outputs, 101 rows, worst 5.0e-10 (axis.x.position): ok
```

Die größte Abweichung von 5e-10 ist die Rundung der 9 Stellen, die `sim-cli run` ausgibt. `fmpy validate` gab "No problems found." aus, und `fmpy simulate` lief ohne Fehler.

OMSimulator 3.0.0.post198 (das Co-Simulationswerkzeug von OpenModelica), aus PyPI in eine zweite virtuelle Umgebung installiert:

```sh
python -m pip install OMSimulator==3.0.0.post198
python -m OMSimulator --validate t-gantry-3.fmu
python -m OMSimulator t-gantry-3.fmu --stopTime 1.0 --resultFile oms-3.csv
```

Es meldete "t-gantry-2.fmu conforms to the FMI-2.0 schema" und "t-gantry-3.fmu conforms to the FMI-3.0 schema" und simulierte beide 1 s lang im Standardschritt von 0,01 s. Seine 101 Zeilen aller 96 Ausgänge weichen für beide Versionen um höchstens 5,0e-10 vom Spitzenwert des jeweiligen Signals von `sim-cli run` ab.

Nicht ausgeführt: `fmusim` der Modelica Association (Reference-FMUs) und der FMU Compliance Checker. Keines von beiden ist in einem Paketverzeichnis veröffentlicht: `pip index versions` findet keine Distribution namens `fmusim`, `reference-fmus`, `fmi-compliance-checker` oder `fmucheck`, und conda ist hier nicht installiert. Beide werden als Binärdateien in GitHub-Releases verteilt, die nicht heruntergeladen wurden.

## Import in andere Werkzeuge

Der Import in Simulink, OpenModelica (OMEdit) und Dymola wurde hier **nicht geprüft**: Keines dieser Werkzeuge stand zur Verfügung. Die Schritte folgen den dokumentierten Menüs der Werkzeuge. In jedem Werkzeug muss der Kommunikationsschritt ein ganzzahliges Vielfaches von `sim.dt_s` sein (62,5 µs in den mitgelieferten Konfigurationen); den Standardschritt der FMU nehmen, `profile.setpoint_interval_s` (10 ms), außer dieses Intervall wird in der Konfiguration geändert.

### Simulink

Simulink importiert FMUs für FMI 2.0 Co-Simulation mit dem FMU-Block; MathWorks führt den Import von FMI 3.0 Co-Simulation ab R2023b (die Versionshinweise des eigenen Release prüfen).

1. Die FMU schreiben (`sim-cli fmu t-gantry --out t-gantry.fmu` oder mit `--fmi 3`).
2. Im Library Browser Simulink Extras > FMU Import öffnen und den FMU-Block ins Modell ziehen (oder auf die Arbeitsfläche klicken, `FMU` tippen und den Block wählen).
3. Den Block öffnen und als FMU-Namen die `.fmu`-Datei angeben. Der Block zeigt `command.setpoint.<Achse>` und `command.enable` als Eingangsports und die gewählten Ausgänge als Ausgangsports.
4. Die Kommunikationsschrittweite des Blocks auf 0,01 s setzen (oder ein anderes ganzzahliges Vielfaches von `sim.dt_s`) und einen Löser mit fester Schrittweite verwenden, dessen Schritt sie teilt.
5. Im Parameterreiter bei Bedarf `config_file` oder die Schalter `effects.<Name>` setzen; nach der Initialisierung sind sie fest.
6. Die Sollwerteingänge mit dem Regler- oder SPS-Modell verbinden und starten.

### OpenModelica

Der Modelleditor OMEdit von OpenModelica importiert FMUs für FMI 2.0 als Modelica-Blöcke; für FMI 3.0 und um Co-Simulation direkt laufen zu lassen, OMSimulator verwenden, das hier von der Kommandozeile aus geprüft wurde (siehe oben).

1. OMEdit: File > Import > FMU..., die `.fmu`-Datei und ein Ausgabeverzeichnis wählen und bestätigen. OMEdit erzeugt einen Modelica-Block mit den Ein- und Ausgängen der FMU und lädt ihn in den Bibliotheksbaum.
2. Den Block in ein Modell ziehen, die Sollwerteingänge verbinden und das Intervall der Simulation auf den Kommunikationsschritt setzen.
3. OMSimulator: `OMSimulator t-gantry-3.fmu --stopTime 1.0 --resultFile result.csv` rechnet die FMU allein; in OMEdit öffnet File > New SSP Model ein OMSimulator-Modell, dem die FMU als Komponente hinzugefügt und mit anderen FMUs verbunden wird.

### Dymola

Dymola importiert FMUs für FMI 2.0; für den Import von FMI 3.0 die Dokumentation der eigenen Dymola-Version prüfen.

1. File > Import > FMU..., die `.fmu`-Datei wählen.
2. Im Importdialog Co-Simulation wählen (diese FMU hat keinen Teil für Model Exchange) und bestätigen. Dymola erzeugt einen Modelica-Block, der die FMU aufruft.
3. Den Block in ein Modell ziehen, die Sollwerteingänge verbinden und mit einem Ausgabeintervall simulieren, das ein ganzzahliges Vielfaches des Kommunikationsschritts ist.
