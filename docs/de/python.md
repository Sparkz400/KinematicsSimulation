# Python-Modul

`kinematics_sim` steuert den Simulator aus Python: die Maschinen, ihre Konfigurationen, Simulationsläufe, die
Zyklusanalyse, die Prüfurteile mit ihren Fehlerbändern, die Auslegung aus dem Katalog, den Zyklusoptimierer, den
Frequenzgang und das Anpassen und Identifizieren aus gemessenen Traces. Es hat die wichtigsten Befehle von `sim-cli` als
Funktionen (was fehlt, steht am Ende) und liefert dieselben Zahlen (die Tests in `crates/sim-py/tests` vergleichen sie). Ergebnisse
kommen als NumPy-Arrays und als Dicts gleich langer Spalten; `pandas.DataFrame(ergebnis)` nimmt jede Tabelle so, wie sie ist.

Quelle: `crates/sim-py` (PyO3 0.30.1, gebaut mit maturin 1.16.0). Das Wheel ist abi3: ein Wheel je Plattform dient jedem
CPython ab 3.10. Die Typ-Stubs (`kinematics_sim.pyi`) und die Markierung `py.typed` liegen im Wheel.

## Installieren

Das Modul liegt nicht auf PyPI. Das Wheel wird aus dem Repository gebaut und installiert:

```bash
pip install maturin==1.16.0
cd crates/sim-py
maturin build --release --out ../../target/wheels
pip install ../../target/wheels/kinematics_sim-*.whl
```

`maturin develop --release` installiert es stattdessen in die aktive virtuelle Umgebung. Für beides ist eine
Rust-Toolchain (1.98.1) nötig. Die CI baut das Wheel für Windows und Linux und hebt es als Artefakt auf.

## Benutzen

```python
import kinematics_sim as ks
import pandas as pd

ks.machines("de")                   # [{"id": "t-gantry", "name": ...}, ...]

cfg = ks.Config("t-gantry")         # die Standardkonfiguration; Config("t-gantry", path="meine.toml") oder text="..."
cfg["mass.payload_kg"]              # 2.0
cfg["mass.payload_kg"] = 3.5        # der Wert unter einem Punkt-Pfad
cfg.set('parts.motor = "1FK2104-5AK"')   # genau wie sim-cli --set: der Wert in TOML-Schreibweise
cfg.to_toml()                       # die Konfiguration als TOML

lauf = pd.DataFrame(ks.run(cfg, cycle=True, time=2.0))  # jedes Signal als Spalte, "time" zuerst
pd.DataFrame(ks.analyze(cfg))                           # eine Zeile je Motor, die Spalten von sim-cli analyze
pd.DataFrame(ks.verdicts(cfg, bands=True))              # eine Zeile je Prüfung und Motor, mit low, high, worst_ok
pd.DataFrame(ks.sizing(cfg, series="1FK2"))             # jeder Katalogmotor auf diesem Zyklus, bestandene zuerst
```

| Funktion | was sie tut | auf der Kommandozeile |
| --- | --- | --- |
| `machines(lang="en")` | die Maschinen der Registrierung, mit Namen auf Englisch oder Deutsch | `sim-cli list` |
| `Config(machine, text=None, path=None)` | die Standardkonfiguration einer Maschine, eine Konfiguration als TOML-Text oder eine Konfigurationsdatei | `--config DATEI` |
| `Config(machine_file=DATEI, text=None, path=None)` | dasselbe für eine Maschine aus einer Maschinendatei ([Maschinendateien](maschinendateien.md)); ein Fehler in der Datei nennt Datei und Zeile | `--machine-file DATEI` |
| `cfg[pfad] = wert`, `cfg.set("pfad = wert")` | ein Wert obendrauf. Ein unter `[parts]` genanntes Teil (`cfg["parts.motor"] = "1FK2104-5AK"`) wirkt wie `--motor` und die anderen Teil-Optionen: alle genannten Teile kommen zusammen aus dem Katalog, und jeder andere gesetzte Wert bleibt über ihren, gleich in welcher Reihenfolge beide gesetzt wurden | `--set`, `--motor` und die anderen Teil-Optionen |
| `signals(cfg)` | `{"id": [...], "unit": [...]}`, die Signale in der Reihenfolge von `run` | der Kopf von `sim-cli run` |
| `run(cfg, cycle=, move_to=, path=, trajectory=, time_optimal=, time=2.0, every=16, disable_at=)` | `{Signal-ID: Array}` über `time` Sekunden, alle `every` Grundschritte abgetastet | `sim-cli run` |
| `analyze(cfg, path=, trajectory=, time_optimal=, points=False)` | die Last je Motor über einen Durchlauf des Zyklus oder über eine Bahn; `points=True` fügt je Motor die Arbeitspunkte als (n, 2)-Array aus Drehzahl (rad/s) und Moment (N m) hinzu | `sim-cli analyze` |
| `verdicts(cfg, bands=False, samples=None, seed=1)` | die Prüfungen; mit `bands` ihr kleinster und größter Wert über die `[bounds]` und ob der ungünstigste Fall besteht; mit `samples` N Läufe zwischen den Grenzen und der Anteil, der bestand | `sim-cli verdicts [--bands] [--samples N --seed S]` |
| `sizing(cfg, supply=None, vendor=None, series="", gearboxes=None, axes=False)` | jeder Katalogmotor auf dem Zyklus der Konfiguration, bestandene und billigste zuerst; `supply` eine Netzspannung des Katalogs (ohne Angabe `"400v"`), `vendor` die eines Herstellers oder `"all"` aller, mit den Spalten `vendor` und `supply` | `sim-cli search` |
| `sweep(cfg, motor=0, from_hz=5.0, to_hz=500.0, count=120, amplitude=None)` | der Frequenzgang vom Moment eines Motors zu seiner Drehzahl: `frequency_hz`, `gain_rad_s_per_nm`, `phase_deg` | `sim-cli sweep` |
| `optimise(cfg, runs=40, nominal=False, worst=False, pareto=False)` | ein kürzerer Zyklus, der jedes Prüfurteil besteht: `cycle_s` und `dwell_error_peak_m` als (vorher, nachher), `runs`, `changes` (Spalten `setting`, `before`, `after`), `robust` (was die Bänder kosten, oder None), mit `pareto` die Front als Spalten | `sim-cli optimise` |
| `fit(cfg, trace_file=, map_file=, stiffness=False, stiffness_key=, sweep_file=, sweep_motor=0, amplitude=)` | was ein gemessener Trace oder Frequenzgang über die Konfiguration sagt: `quantity`, `of`, `config`, `fitted`, `standard_error` (NaN, wo die Kommandozeile die Zelle leer lässt) und `advice`, was zu ändern ist | `sim-cli fit` |
| `identify(trace_file=, map_file=, config=, motor=0, from_hz=, to_hz=, step_from=, step_to=, motor_inertia=, sweep_file=)` | der Antriebsstrang eines Motors aus seinem eigenen Trace: `quantity`, `of`, `value`, `standard_error`, `ci95_low`, `ci95_high`, `notes` (was der Trace nicht sagen konnte) und `overlay` (gemessene und modellierte Signale mit `time`) | `sim-cli identify [--overlay]` |

`path` und `trajectory` nehmen den Text einer Bahn- oder Trajektoriendatei (Formate: `sim_components::path`), keinen
Dateinamen: `ks.analyze(cfg, path=open("pick.path").read())`. Eine abgelehnte Konfiguration, ein abgelehnter Befehl oder
eine abgelehnte Bahn oder ein abgelehnter Trace löst `ValueError` mit der Meldung des Simulators auf Englisch aus; eine
unbekannte Maschine oder ein mit `cfg[pfad]` gelesener unbekannter Schlüssel `KeyError`; eine Ganzzahl über 64 Bit, mit
`cfg[pfad] = wert` gesetzt, `OverflowError` (die Ganzzahlen von TOML haben 64 Bit; vor dem 14. November 2026 wurde sie
stillschweigend eine Gleitkommazahl). `trace_file`, `map_file` und `sweep_file` nehmen Dateinamen, wie die
Kommandozeile. `run` rechnet höchstens 1e8 Grundschritte und tastet höchstens 2,5e8 Werte ab (2 GB Arrays); ein längerer
Lauf löst sofort `ValueError` aus, statt zu hängen oder den Speicher zu erschöpfen: `every` erhöhen oder in Stücken
rechnen.

Eine Änderung der Konfiguration wird geprüft wie eine Konfigurationsdatei: ein Schlüssel, den die Standardkonfiguration
der Maschine nicht hat, wird mit Namen abgelehnt.

## Parameterstudien

Jeder lange Aufruf gibt Pythons globale Interpretersperre frei, so dass ein Thread-Pool mehrere auf allen Prozessoren
rechnet (Beispiel in der englischen Fassung, [python.md](../python.md#parameter-sweeps)).

`crates/sim-py/examples/parameter_sweep.ipynb` ist ein durchgerechnetes Beispiel: die Nutzlast des T-Portals von 0,5 kg
bis 8 kg, Spitzen- und Effektivmoment, thermische Auslastung und die Prüfurteile je Masse, und die Signale eines Laufs
als DataFrame. Die Testsuite führt es von oben bis unten aus.

## Tests

```bash
cargo build -p sim-cli --release       # die Tests vergleichen damit
pip install "pytest==9.2.0" pandas matplotlib nbformat nbclient ipykernel
pytest crates/sim-py/tests
```

Die Suite prüft die Maschinenliste, das Lesen, Setzen und Ablehnen von Konfigurationswerten, den TOML-Rundweg, `run`
gegen `sim-cli run` (gleiche Abtastwerte auf 1e-8), `analyze` und `verdicts` gegen die Kommandozeile, dass eine schwerere
Nutzlast mehr Moment braucht, die Reihenfolge der Auslegungstabelle, dass zwei Threads dasselbe Ergebnis bekommen, und
das Notebook. 11 Tests bestehen unter Windows mit CPython 3.14 (13. November 2026). Hinzugekommen am 14. November 2026:
ein Lauf einer Maschinendatei gegen `sim-cli run --machine-file`, ihre Ablehnungen, `sweep`, `optimise`, `fit` und
`identify` gegen die Kommandozeile auf einem Durchlauf des Zyklus, der Überlauf von 2^63 (eins mehr als die größte
64-Bit-Ganzzahl) und die Grenze für `run(time=1e12)`; diese 10 bestehen unter Windows mit CPython 3.14.

## Nicht abgedeckt

`step`, `bode`, `tuning`, `margins`, `modes`, `sensitivity`, `replay`, `compare`, `report`, `check-dataset` und `fmu` der
Kommandozeile haben noch keine Funktion, die Suche `--worst` von `verdicts` und `search --motor-id` auch nicht. `fit` und
`identify` brechen beim ersten Fehler ab, wo die Kommandozeile schreibt, was sie hat, und dann scheitert. Meldungen gibt
es nur auf Englisch.
