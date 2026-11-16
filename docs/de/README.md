# Kinematics Simulation

**Eine Physik-Werkbank für Bewegungsmaschinen.** Beschreiben Sie eine Maschine (Portal, Delta-Picker, Roboter, Presse, Hexapod, Seilroboter und mehr), lassen Sie ihren Zyklus laufen und sehen Sie, was Motoren, Antriebe, Riemen, Getriebe und Führungen wirklich aushalten müssen, geprüft gegen die Kennwerte der Hersteller.

![Das T-Portal fährt seinen Pick-and-Place-Zyklus in der App](../images/t-gantry-cycle.gif)

**[Im Browser ausprobieren](https://sparkz400.github.io/KinematicsSimulation/)** (ohne Installation) · [Download](https://github.com/Sparkz400/KinematicsSimulation/releases) · [Dokumentation](https://sparkz400.github.io/KinematicsSimulation/docs/) · [English](../../README.md)

## Warum

Auslegungstabellen rechnen eine Maschine als starre Massen mit festen Wirkungsgraden. Dieser Simulator bildet den Antriebsstrang physikalisch ab: Riemen als Federn, Führungsreibung nach LuGre, Getriebespiel und -verluste, die Motorwicklung mit ihrer Temperatur, ein Servoantrieb mit Kaskadenregelung und Zwischenkreis. Moment, Schleppfehler, Erwärmung und Einschwingen ergeben sich aus diesem Modell statt aus Faustregeln.

## Funktionen

- **Prüfurteile zur Auslegung.** Spitzen-, Effektiv- und thermische Last jedes Motors, Antriebs, Getriebes, Riemens, jeder Führung und jedes Lagers über einen Zyklus, bestanden oder nicht gegen Katalogwerte, mit Lebensdauern von Lagern und Führungen.
- **Fehlerbänder.** Jeder geschätzte Wert hat Grenzen; sehen Sie, wie weit sich jedes Ergebnis verschiebt, einzeln, gesampelt oder in der ungünstigsten Ecke.
- **Katalogsuche.** Der leichteste Motor mit Antrieb und Getriebe (und Achse), der besteht, aus eingebauten Daten von Siemens, Beckhoff, B&R, SEW, Lenze, Kollmorgen, Yaskawa, Festo, Bahr, Neugart, Wittenstein und weiteren.
- **Bewegungsplanung.** Bahnen aus Geraden, Kreisbögen und Splines, ein zeitoptimaler Plan innerhalb aller Grenzen, Überschleifen, Input Shaping, elektronische Kurvenscheiben.
- **Dynamik.** Resonanzen und Eigenformen über den Arbeitsraum, Stabilitätsreserven der Regelkreise, Anhaltewege in Stoppkategorie 0, 1 und 2.
- **Optimierer.** Ein kürzerer Zyklus oder weniger Energie, der jede Prüfung weiterhin besteht.
- **Echte Daten.** Eine Antriebsaufzeichnung nachfahren und Masse, Trägheit, Reibung und Riemensteifigkeit daran anpassen.
- **Eigener Regler.** Die simulierte Maschine in Momenten- oder Drehzahlregelung aus Python, einer FMU oder einer SPS über OPC UA führen.

## Maschinen

| Familie | Maschinen |
| --- | --- |
| Achsen und Portale | T-Portal, Riemenachse, Kugelgewindeachse, Zahnstange, Linearmotor, Rundtisch, H-Bot, CoreXY, XYZ-Portal und Portal mit zwei Antrieben, Fünfachsmaschinen |
| Delta- und Parallelroboter | Delta mit 3 Armen, Delta mit 4 Achsen, Quattro-Art, planarer Delta, Linear-Delta, Parallel-SCARA, Hexapod (Zylinder und Kurbeln), Tripod |
| Roboter | Sechsachsroboter aus URDF (auch auf Linearschiene), SCARA, Palettierer, Wafer-Handler mit R-Theta-Z-Arm |
| Kurbel und Kurve | Servopresse, Kniehebelpresse, Rotationsmesser, fliegende Säge, Kurvenscheibe, Rundschalttisch (Malteserkreuz, Kurvenschrittgetriebe) |
| Weitere | Ebener und räumlicher Seilroboter, Auf- und Abwickler mit Tänzer, Scherenhubtisch |

Antriebe: permanenterregte Servomotoren, Schrittmotoren, Asynchronmotoren (U/f oder feldorientiert) und Pneumatikzylinder. Neue Maschinen lassen sich ohne Programmieren in einer [Maschinendatei](maschinendateien.md) beschreiben.

## Einstieg

**Desktop-App und Befehlszeile:** den Windows-Installer oder das Linux-Paket unter [Releases](https://github.com/Sparkz400/KinematicsSimulation/releases) herunterladen. Die App lässt sich im Betrieb auf Deutsch umschalten; Befehlszeile und OPC-UA-Server sprechen mit `--lang de` Deutsch.

**Python:**

```bash
pip install kinematics-sim
```

```python
import kinematics_sim as ks
import pandas as pd

cfg = ks.Config("t-gantry")         # eine eingebaute Maschine mit ihrer Standardkonfiguration
cfg["mass.payload_kg"] = 3.5        # jeden Wert ändern
print(pd.DataFrame(ks.verdicts(cfg)))   # jede Prüfung des Zyklus: Wert, Grenze, ok
```

**Aus dem Quelltext** (Rust stable, Edition 2024):

```bash
cargo run --release -p sim-app                                  # Desktop-App
cargo run --release -p sim-cli -- verdicts t-gantry --lang de   # Befehlszeile
```

Weiter mit dem [Benutzerhandbuch](benutzerhandbuch.md), dem [Überblick mit den Grenzen](ueberblick.md) und den [Beispielen](../../examples/README.md) (englisch).

## Wie genau ist es?

Jedes Bauteil und jede Kennzahl ist gegen eine Handrechnung getestet. Die Dynamik der Roboter stimmt mit MuJoCo auf 1e-14 überein, die der Deltas mit unabhängigen Lagrange-Rechnungen, und der Regelkreis des Antriebs mit einer unabhängigen Python-Implementierung. Eine reale Maschine, ein Kugelgewinde-Prüfstand des ISW Stuttgart, wurde nachgefahren: Das simulierte Motormoment liegt im Effektivwert 5 bis 13 % neben dem gemessenen.

Die Gleichungen sind geprüft; die Parameter einer bestimmten Maschine (Reibung, Dämpfung, Steifigkeit) sind oft Schätzungen, und die Konfigurationen kennzeichnen sie. Vergleiche und Tendenzen sind belastbar; absolute Zahlen sind so gut wie diese Werte. Einzelheiten: [Überblick und Grenzen](ueberblick.md) und [validation.md](../validation.md) (englisch). Messdaten Ihrer Maschine sind der wertvollste Beitrag: siehe [datasets](../../datasets/README.md).

## Schnittstellen

| Oberfläche | Wofür |
| --- | --- |
| Desktop-App | 3D-Ansicht, Oszilloskope, Zyklusanalyse, Suche und Einstellhilfe |
| `sim-cli` | Skripte und CI; Ausgabe als CSV |
| [Python](python.md) | Ergebnisse für NumPy und pandas, Parameterstudien, eigener Regler |
| [FMI 2.0 / 3.0](fmi.md) | Co-Simulation in Simulink, OpenModelica, Dymola |
| [OPC UA](sps.md) | Virtuelle Inbetriebnahme mit einer SPS (PLCopen-artige Achsen) |
| [Browser](web.md) | Demo und Auslegungsprüfung ohne Installation |

## Dokumentation

Auf Deutsch: [Benutzerhandbuch](benutzerhandbuch.md) · [Überblick und Grenzen](ueberblick.md) · [Maschinendateien](maschinendateien.md) · [Mechanismen](mechanismen.md) · [Python-Modul](python.md) · [Browser](web.md) · [FMI-Export](fmi.md) · [SPS-Inbetriebnahme](sps.md) · [Feldbus-Emulation](feldbus-emulation.md) · [Anbindungen](integrationen.md)

Nur auf Englisch: [Config reference](../config-reference.md) · [Model reference](../model-reference.md) · [Catalogue](../catalogue.md) · [Validation](../validation.md) · [Measurement procedure](../measurement-procedure.md) · [Roadmap](../../ROADMAP.md) · [Changelog](../../CHANGELOG.md)

## Lizenz

Wahlweise unter [MIT](../../LICENSE-MIT) oder [Apache-2.0](../../LICENSE-APACHE). Der OPC-UA-Server nutzt `async-opcua` (MPL-2.0); Einzelheiten im englischen [README](../../README.md#license).
