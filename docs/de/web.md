# Der Simulator im Browser (WebAssembly): 3D-Ansicht und Auslegungsprüfung

Kern, Maschinen, Katalog und Analyse lassen sich für `wasm32-unknown-unknown` bauen. `crates/sim-wasm` stellt sie über sechs
C-ABI-Funktionen bereit, und `crates/sim-wasm/web/index.html` ist eine statische Seite, die das Modul lädt: Maschine
wählen, Konfiguration (TOML) bearbeiten, und die Seite zeigt die Maschine in 3D, wie sie ihren Zyklus fährt, und rechnet
eine Auslegungsprüfung, die jedes Prüfurteil und die Last je Motor zeigt. Nichts verlässt den Browser; ein Server muss nur zwei Dateien ausliefern.

**Live-Seite:** https://sparkz400.github.io/KinematicsSimulation/, bei jedem Push auf `main` vom Workflow
[`pages.yml`](../../.github/workflows/pages.yml) gebaut.

## Bauen und öffnen

```bash
rustup target add wasm32-unknown-unknown
cargo build -p sim-wasm --release --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/release/sim_wasm.wasm crates/sim-wasm/web/
python -m http.server 8765 -d crates/sim-wasm/web
```

Dann http://localhost:8765/ öffnen. Browser laden WebAssembly nur über HTTP, nicht aus einer `file://`-Seite; jeder
statische Dateiserver genügt. Zum Veröffentlichen `index.html` und `sim_wasm.wasm` auf einen beliebigen statischen Host
kopieren. Der CI-Job `wasm` baut beides und hebt es als Artefakt `sizing-page` auf.

Kein wasm-bindgen und keine JavaScript-Werkzeuge: Die Seite schreibt eine Anfrage in den Speicher des Moduls und liest
JSON zurück, `cargo` ist das einzige Bauwerkzeug. Das Modul ist etwa 2,0 MB groß (1.979.671 Byte vor der 3D-Ansicht, 2.001.582 Byte mit
ihr, 14. November 2026).

## Was gerechnet wird

Dieselbe Rechnung wie `sim-cli verdicts` und `sim-cli analyze` für den Zyklus der Konfiguration: ein Durchlauf, gemessen,
nachdem die Maschine in den Zyklus eingeschwungen ist, beurteilt gegen die Motor- und Umrichterdaten und die `[ratings]`
der Konfiguration. Die Zahlen stimmen mit denen der Kommandozeile auf vier bis fünf gültige Stellen überein, nicht
bitgenau: WebAssembly rechnet `sin`, `exp` und Ähnliches mit Rusts libm, der native Build mit der Mathematikbibliothek
der Plattform, und die Simulation verstärkt die Unterschiede im letzten Bit. Ein gemessenes Beispiel: ein Spitzenmoment
von 0,4600761 N m im Browser gegen 0,4601061 N m aus `sim-cli`. Ein Urteil nahe an seiner Grenze kann deshalb anders
ausfallen. Unter `[parts]` genannte Teile kommen aus dem eingebauten Katalog. Mit der Standardkonfiguration des
T-Portals dauert die Prüfung 0,9 s im Chromium-Browserfenster des Entwicklungsrechners (13. November 2026).

| Anfrage (`call(kind, ...)`) | Zeilen der Anfrage | Antwort (JSON) |
| --- | --- | --- |
| 0 | Sprache (`en`, `de`) | jede Maschine: `[{"id", "name"}]` |
| 1 | Maschinen-ID | `{"config"}`: die Standardkonfiguration als TOML |
| 2 | Maschinen-ID, Sprache, dann die Konfiguration als TOML (leer: die Standardkonfiguration) | `machine`, `cycle_s`, `tripped`, `warnings`, `motors` (Spitzen- und Effektivmoment, Spitzendrehzahl, Ausnutzung der Drehmoment-Drehzahl-Kennlinie, thermische Auslastung, Wicklungserwärmung), `verdicts` (Prüfung, Bezeichnung, Motor, Wert, Grenze, ok) |

Ein Fehler kommt als `{"error": "..."}` mit der Meldung des Simulators zurück, und `call` gibt 1 zurück.

## Die 3D-Ansicht

Anfrage 3 (Maschinen-ID, Sprache, Konfiguration wie bei Anfrage 2) stellt die Maschine ruhend in ihre Startpose, startet
ihren Zyklus und antwortet `{"dt_s", "centre", "radius"}`: den Grundschritt und die Kugel um die Szene beim Start, auf die
die Kamera eingepasst wird. In jedem Bild ruft die Seite dann `frame(seconds)`, das die Simulation um so viele Sekunden
weiterrechnet (ganze Grundschritte) und einen Zeiger auf `frame_len()` 32-Bit-Gleitkommazahlen zurückgibt, kein JSON:

| Zahlen | was |
| --- | --- |
| 6, der Kopf | Simulationszeit (s), Zykluszeit (s, der letzte ganze Durchlauf von einem Zyklusstart zum nächsten; NaN davor), x, y, z des Werkzeugs in der Szene (m; NaN, wo die Maschine keine Abbildung hat), Anzahl der Teile |
| 20 je Teil | das Netz (0: Würfel der Kantenlänge 1, 1: Zylinder mit Durchmesser und Länge 1 entlang y, beide um den Ursprung), die 4×4-Matrix, die das Netz auf das Teil abbildet (spaltenweise, Szenenkoordinaten, z nach oben), die Farbe (Rot, Grün, Blau in 0..1) |

Die Matrix trägt die Form des Teils mit seiner Lage: Ein Riementrum (Segment) ändert seine Länge mit der Bewegung, und die
Zahl der Riemenstreifen kann sich von Bild zu Bild ändern; die Seite zeichnet, was das Bild enthält. Die Farben sind die
der Desktop-App (`scene::part_color`): eigene Farbe, Riemenspannung und Erwärmung wie dort.

Die Seite zeichnet mit WebGL2, direkt in der Seite geschrieben, ohne Bibliothek: Nichts wird von anderswo geladen, und zwei
Netze, ein Licht und eine Orbit-Kamera sind hundert Zeilen. Ziehen dreht, das Mausrad zoomt; Pause/Weiter und Neustart
(mit der Konfiguration im Editor). Die Seite rechnet in Echtzeit; eine Maschine, die der Browser nicht so schnell rechnen
kann, läuft langsamer, denn ein Bild rechnet höchstens 50 ms Simulationszeit. Die Legende zeigt Simulationszeit,
Zykluszeit und Werkzeugposition; ein oranger Würfel markiert das Werkzeug.

## Unterschiede zum nativen Programm

WebAssembly startet keine Threads. Die Analysen, die nativ die Prozessoren teilen (Fehlerbänder, Stichprobenläufe, die
Katalogsuche, die Stabilitätsreserven, der Vergleich eines Effekts an und aus), laufen dort nacheinander; der Code fällt
auf einen Thread zurück, sobald `std::thread::available_parallelism` einen meldet oder keinen kennt. Die Seite bietet nur
die einzelne Auslegungsprüfung, die einen Lauf braucht.

## Tests

`cargo test -p sim-wasm --release` rechnet die Anfragen des Moduls nativ: jede Maschine gelistet, die Auslegungsprüfung mit
denselben Prüfungen, Werten und derselben Zykluszeit wie die Analyse- und Prüffunktionen der Kommandozeile, eine
Konfiguration von der Seite wie eine Konfigurationsdatei gelesen (ein falsch geschriebener Schlüssel mit Namen
abgelehnt), und die C-Funktionen, die JSON zurückgeben. Für die 3D-Ansicht: das Bild jeder Maschine gegen ihre Szene aus einer
nativen Simulation, in der Startpose und nach 0,5 s ihres Zyklus (Netz, Mitte, Streckung entlang jeder Achse und Farbe
jedes Teils, die Werkzeugposition), zwei Datensätze und ein Begrenzungsquader von Hand gerechnet, und die Zykluszeit der
Ansicht gegen die der Auslegungsprüfung. Die Seite selbst wurde von Hand in Chrome ausgeführt: Die
Standardkonfiguration des T-Portals besteht jede Prüfung bei einer Zykluszeit von 6,080 s.
