# Python-Modul

`kinematics_sim` steuert den Simulator aus Python: die Maschinen, ihre Konfigurationen, Simulationsläufe, die
Zyklusanalyse, die Prüfurteile mit ihren Fehlerbändern und die Auslegung aus dem Katalog. Es ist die Kommandozeile
`sim-cli` als Funktionen und liefert dieselben Zahlen (die Tests in `crates/sim-py/tests` vergleichen sie). Ergebnisse
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
| `cfg[pfad] = wert`, `cfg.set("pfad = wert")` | ein Wert obendrauf. Ein unter `[parts]` genanntes Teil (`cfg["parts.motor"] = "1FK2104-5AK"`) wirkt wie `--motor` und die anderen Teil-Optionen: alle genannten Teile kommen zusammen aus dem Katalog, und jeder andere gesetzte Wert bleibt über ihren, gleich in welcher Reihenfolge beide gesetzt wurden | `--set`, `--motor` und die anderen Teil-Optionen |
| `signals(cfg)` | `{"id": [...], "unit": [...]}`, die Signale in der Reihenfolge von `run` | der Kopf von `sim-cli run` |
| `run(cfg, cycle=, move_to=, path=, trajectory=, time_optimal=, time=2.0, every=16, disable_at=)` | `{Signal-ID: Array}` über `time` Sekunden, alle `every` Grundschritte abgetastet | `sim-cli run` |
| `analyze(cfg, path=, trajectory=, time_optimal=, points=False)` | die Last je Motor über einen Durchlauf des Zyklus oder über eine Bahn; `points=True` fügt je Motor die Arbeitspunkte als (n, 2)-Array aus Drehzahl (rad/s) und Moment (N m) hinzu | `sim-cli analyze` |
| `verdicts(cfg, bands=False, samples=None, seed=1)` | die Prüfungen; mit `bands` ihr kleinster und größter Wert über die `[bounds]` und ob der ungünstigste Fall besteht; mit `samples` N Läufe zwischen den Grenzen und der Anteil, der bestand | `sim-cli verdicts [--bands] [--samples N --seed S]` |
| `sizing(cfg, supply="400v", series="", gearboxes=None, axes=False)` | jeder Katalogmotor auf dem Zyklus der Konfiguration, bestandene und billigste zuerst | `sim-cli search` |

`path` und `trajectory` nehmen den Text einer Bahn- oder Trajektoriendatei (Formate: `sim_components::path`), keinen
Dateinamen: `ks.analyze(cfg, path=open("pick.path").read())`. Eine abgelehnte Konfiguration, ein abgelehnter Befehl oder
eine abgelehnte Bahn löst `ValueError` mit der Meldung des Simulators auf Englisch aus; eine unbekannte Maschine oder ein
mit `cfg[pfad]` gelesener unbekannter Schlüssel `KeyError`.

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
das Notebook. 11 Tests bestehen unter Windows mit CPython 3.14 (13. November 2026).

## Nicht abgedeckt

`sweep`, `step`, `bode`, `tuning`, `margins`, `sensitivity`, `optimise`, `replay`, `fit`, `compare` und `fmu` der
Kommandozeile haben noch keine Funktion, die Suche `--worst` von `verdicts` auch nicht. Meldungen gibt es nur auf Englisch.
