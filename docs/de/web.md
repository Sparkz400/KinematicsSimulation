# Auslegungsprüfung im Browser (WebAssembly)

Kern, Maschinen, Katalog und Analyse lassen sich für `wasm32-unknown-unknown` bauen. `crates/sim-wasm` stellt sie über vier
C-ABI-Funktionen bereit, und `crates/sim-wasm/web/index.html` ist eine statische Seite, die das Modul lädt und im Browser
eine Auslegungsprüfung rechnet: Maschine wählen, Konfiguration (TOML) bearbeiten, und die Seite fährt den Zyklus und
zeigt jedes Prüfurteil und die Last je Motor. Nichts verlässt den Browser; ein Server muss nur zwei Dateien ausliefern.

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
JSON zurück, `cargo` ist das einzige Bauwerkzeug. Das Modul ist etwa 1,1 MB groß.

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

## Unterschiede zum nativen Programm

WebAssembly startet keine Threads. Die Analysen, die nativ die Prozessoren teilen (Fehlerbänder, Stichprobenläufe, die
Katalogsuche, die Stabilitätsreserven, der Vergleich eines Effekts an und aus), laufen dort nacheinander; der Code fällt
auf einen Thread zurück, sobald `std::thread::available_parallelism` einen meldet oder keinen kennt. Die Seite bietet nur
die einzelne Auslegungsprüfung, die einen Lauf braucht.

## Tests

`cargo test -p sim-wasm --release` rechnet die Anfragen des Moduls nativ: jede Maschine gelistet, die Auslegungsprüfung mit
denselben Prüfungen, Werten und derselben Zykluszeit wie die Analyse- und Prüffunktionen der Kommandozeile, eine
Konfiguration von der Seite wie eine Konfigurationsdatei gelesen (ein falsch geschriebener Schlüssel mit Namen
abgelehnt), und die C-Funktionen, die JSON zurückgeben. Die Seite selbst wurde von Hand in Chrome ausgeführt: Die
Standardkonfiguration des T-Portals besteht jede Prüfung bei einer Zykluszeit von 6,080 s.
