# Der Simulator im Browser (WebAssembly): 3D-Ansicht, Verläufe und Auslegungsprüfung

Kern, Maschinen, Maschinendateien, Katalog und Analyse lassen sich für `wasm32-unknown-unknown` bauen. `crates/sim-wasm`
stellt sie über einige C-ABI-Funktionen bereit, und `crates/sim-wasm/web/index.html` ist eine statische Seite, die das
Modul lädt: eine Maschine wählen (eingebaut, eine der Maschinendateien aus `machines/` oder eine TOML-Maschinendatei,
geöffnet oder in den Editor eingefügt), ihre Konfiguration bearbeiten, ihren Zyklus oder einen Pfad fahren, und die
Seite zeigt die Maschine in 3D mit ihren Hindernissen und der Spur des Werkzeugs, Zeitverläufe von Moment, Drehzahl und
Schleppfehler je Motor und eine Auslegungsprüfung mit jedem Prüfurteil, jeder Spalte von `sim-cli analyze` und den
Arbeitspunkten über der Drehmoment-Drehzahl-Kennlinie. Dazu ein Frequenzgang und eine Anpassung von Massen und Reibung
an eine gemessene Aufzeichnung. Nichts verlässt den Browser; ein Server muss nur zwei Dateien ausliefern. Die Seite ist
deutsch und englisch (Umschalter oben; zuerst die Sprache des Browsers).

**Live-Seite:** https://sparkz400.github.io/KinematicsSimulation/, bei jedem Push auf `main` vom Workflow
[`pages.yml`](../../.github/workflows/pages.yml) gebaut.

## Bauen und öffnen

```bash
rustup target add wasm32-unknown-unknown
cargo build -p sim-wasm --profile wasm --target wasm32-unknown-unknown
cp target/wasm32-unknown-unknown/wasm/sim_wasm.wasm crates/sim-wasm/web/
python -m http.server 8765 -d crates/sim-wasm/web
```

Dann http://localhost:8765/ öffnen. Browser laden WebAssembly nur über HTTP, nicht aus einer `file://`-Seite; jeder
statische Dateiserver genügt. Zum Veröffentlichen `index.html` und `sim_wasm.wasm` auf einen beliebigen statischen Host
kopieren. Der CI-Job `wasm` baut beides und hebt es als Artefakt `sizing-page` auf; `pages.yml` baut es ebenso und
veröffentlicht es auf GitHub Pages.

Das Profil `wasm` (Wurzel-`Cargo.toml`) ist das Release-Profil mit Link-Time-Optimierung und einer Codegen-Einheit: am
T-Portal etwa 5 % schneller und 10 % kleiner als `--release` (14. November 2026). Kein wasm-bindgen, kein wasm-opt und
keine JavaScript-Werkzeuge; `cargo` ist das einzige Bauwerkzeug. Das Modul hat 2.887.035 Byte (14. November 2026, mit
den eingebauten Maschinendateien).

## Geschwindigkeit

Das Modul läuft in zwei Web Workern, damit die Seite bedienbar bleibt: einer rechnet die 3D-Ansicht, der andere die
Auslegungsprüfung, den Frequenzgang und die Anpassung, die vorher die Seite sekundenlang blockierten. Jeder Worker hat
sein eigenes Modul und damit seine eigenen Maschinendateien; die Seite gibt eine geöffnete Datei beiden. Der Worker der
Ansicht beantwortet eine Anfrage nach der anderen mit der ganzen seit der letzten geschuldeten Simulationszeit (höchstens
0,25 s, sodass eine für Echtzeit zu langsame Maschine zurückfällt, statt Arbeit aufzustauen), und gibt das Bild als
übertragene Kopie zurück.

Gemessen am 14. November 2026 auf dem Entwicklungsrechner, während andere Builds liefen: Die Standardkonfiguration des
T-Portals rechnet 10 s in 0,70 s Prozessorzeit im Modul (Node 24, dasselbe wasm), etwa 14-fache Echtzeit mit den Proben
der Verläufe; das Modul aus dem Release-Profil ohne LTO brauchte 0,86 bis 1,05 s. Im kopflosen Chrome (Software-WebGL)
lief die Seite das T-Portal mit 1,01-facher Echtzeit, die Auslegungsprüfung daneben brauchte 4,0 s. Das frühere Drittel
der Echtzeit ließ sich nicht nachstellen: Das Modul selbst war auch damals etwa zehnmal schneller als Echtzeit; was die
Seite im Hauptthread darum tat (die Prüfung blockierte ihn, jedes Bild wartete auf den Schritt), liegt jetzt in Workern.

## Anfragen

Die Seite schreibt eine Anfrage in den Speicher aus `alloc`, ruft `call(kind, ptr, len)` und liest die Antwort, JSON, aus
`out_ptr()` und `out_len()`; `call` gibt 0 für ein Ergebnis zurück und 1 für `{"error": "..."}` mit der Meldung des
Simulators.

Eine Anfrage hat Zeilen: die Maschine, die Sprache (`en`, `de`) mit Optionen dahinter, durch Leerzeichen getrennt, dann
die Konfiguration als TOML (leer: die Standardkonfiguration), und nach einer Zeile `---` der Text, den eine Option nennt
(ein Pfad, eine Aufzeichnung). Eine Maschine ist eine eingebaute ID (`t-gantry`), eine Maschinendatei aus `machines/` im
Modul (`file:corexy.toml`) oder eine von der Seite übergebene (`upload:<Dateiname>`).

| Anfrage | Zeilen | Antwort (JSON) |
| --- | --- | --- |
| 0 | Sprache | jede Maschine: `[{"id", "name", "file"}]`, die eingebauten, dann die Maschinendateien |
| 1 | Maschine | `{"config"}`: die Standardkonfiguration (einer Maschinendatei: die Datei) |
| 2 | Maschine, Sprache und Optionen, Konfiguration | die Auslegungsprüfung: `machine`, `cycle_s`, `tripped`, `warnings`, die maschinenweiten Spalten von `sim-cli analyze`, `motors` (jede Spalte je Motor von `sim-cli analyze` unter ihrem Namen, `points` die Arbeitspunkte, `curve` der Umriss der Spitzenkennlinie, `s1` der Dauerbetrieb), `verdicts` |
| 3 | Maschine, Sprache und Optionen, Konfiguration | startet die 3D-Ansicht: `{"dt_s", "centre", "radius", "motors", "sample"}` |
| 4 | Dateiname, Sprache, Maschinendatei (TOML) | `{"id", "name", "config"}`: die Datei geprüft wie von `sim-cli --machine-file` (ein Fehler nennt Datei und Zeile) |
| 5 | Maschine, Sprache und Optionen, Konfiguration | `{"motor", "amplitude_nm", "points": [[Frequenz Hz, Verstärkung rad/s je N m, Phase °]]}` wie `sim-cli sweep` |
| 6 | Maschine, Sprache, Konfiguration, `---`, Aufzeichnung (CSV) | `{"rows": [[Größe, von, Konfiguration, angepasst, Standardfehler]], "advice"}` wie `sim-cli fit --trace` ohne `--map` |
| 7 | Sprache, dann Übersetzungsschlüssel, einer je Zeile | `{Schlüssel: Text}`: die Texte der Seite (`crates/sim-wasm/i18n/*.ftl`) und diese Schlüssel aus den Übersetzungen des Simulators |

Optionen: `path` fährt den Pfad nach `---` (Format: `sim_components::path`, die Datei von `sim-cli --path`) ab der
Startpose statt des Zyklus, in den Achsgrenzen oder mit `time-optimal` so schnell, wie die Motoren können (Anfragen 2
und 3); Anfrage 2 warnt wie `sim-cli`, wo der geplante Pfad von den gegebenen Geraden und Bögen abweicht. `motor=K`,
`from=HZ`, `to=HZ`, `count=N`, `amplitude=NM` stellen den Frequenzgang wie bei `sim-cli sweep` ein (die Seite fragt 40
Punkte statt 120 an: jeder ist ein eigener Lauf).

## Was gerechnet wird

Dieselbe Rechnung wie `sim-cli verdicts` und `sim-cli analyze` für den Zyklus der Konfiguration oder den Pfad, mit jeder
Spalte, die `sim-cli analyze` schreibt (Gehäuseerwärmung, Überlast des Antriebs, Effektiv- und Spitzenstrom, Energien,
Zwischenkreis, Lebensdauern, Riemenkraft, Bahn- und Folgefehler, Änderung beim Warmstart). Was die Seite nicht gibt:
`--breakdown` und `--budget` von `analyze` (die Effektansichten; die Seite rechnet ohne sie, wie `verdicts`), die
Fehlerbänder von `verdicts --bands` und die Katalogsuche.

Die Zahlen stimmen mit denen der Kommandozeile auf vier bis fünf gültige Stellen überein, nicht bitgenau: WebAssembly
rechnet `sin`, `exp` und Ähnliches mit Rusts libm, der native Build mit der Mathematikbibliothek der Plattform. Ein
gemessenes Beispiel: 0,4600761 N m Spitzenmoment im Browser gegen 0,4601061 N m aus `sim-cli`. Ein Urteil nahe an seiner
Grenze kann deshalb anders ausfallen. Unter `[parts]` genannte Teile kommen aus dem eingebauten Katalog.

Die Arbeitspunkte sind die der Analyse (Motordrehzahl und -moment, einmal je Millisekunde), über dem Umriss von
`DriveParams::torque_curve` (Spitzenmoment von Motor und Antrieb in beide Richtungen, an den Ecken und 40 Drehzahlen
dazwischen) und dem S1-Kasten (Bemessungsmoment bis zur Bemessungsdrehzahl), wie in der App.

## 3D-Ansicht, Verläufe und Spur

Anfrage 3 stellt die Maschine ruhend in ihre Startpose, startet Zyklus oder Pfad und antwortet Grundschritt, Kugel um die
Szene für die Kamera, die Motoren und die Zahlen je Probe. Dann rechnet `frame(seconds)` die Simulation um so viele
Sekunden weiter (ganze Grundschritte) und gibt einen Zeiger auf `frame_len()` 32-Bit-Gleitkommazahlen zurück:

| Zahlen | was |
| --- | --- |
| 8, der Kopf | Simulationszeit (s), Zykluszeit (s; NaN vor dem ersten ganzen Durchlauf), x, y, z des Werkzeugs (m; NaN ohne Abbildung), Anzahl der Teile, der Hindernisse, der Proben |
| 20 je Teil | das Netz (0: Würfel der Kantenlänge 1, 1: Zylinder mit Durchmesser und Länge 1 entlang y), die 4×4-Matrix (spaltenweise, Szenenkoordinaten, z nach oben), die Farbe |
| 20 je Hindernis | wie ein Teil, Netz 2: der Würfel durchscheinend, auf den Quader des Hindernisses gestreckt; grau, rot, solange ein Teil es berührt |
| `sample` je Probe | eine je Millisekunde seit dem letzten Bild: Zeit, x, y, z des Werkzeugs, sein Abstand von der Planposition (m), dann je Motor kleinster und größter Wert von Moment, Momentensollwert (N m), Drehzahl, Drehzahlsollwert (rad/s) und Schleppfehler (rad) über die Grundschritte dieser Millisekunde |

`frame` lehnt eine Zeit ab, die nicht endlich, kleiner als 0 oder größer als 60 s ist: Es gibt null zurück und lässt den
Fehler als Antwort bei `out_ptr`. (Eine unendliche Zeit trieb die Schrittzahl bis zur größten Ganzzahl und hängte die
Seite auf.)

Die Seite zeichnet mit WebGL2 ohne Bibliothek. Ziehen dreht, das Mausrad zoomt; Anhalten/Weiter und Neustart (mit
Konfiguration, Bewegung und Pfad aus dem Editor). Die Spur ist der Weg des Werkzeugs, ein Punkt je Millisekunde, gefärbt
nach seinem Abstand von der Planposition auf einer festen, auf der Seite eingestellten Skala: blau bei keinem, gelb bei
der Hälfte, rot beim eingestellten Wert (2 mm, solange nicht geändert), magenta darüber. Die letzten 40.000 Punkte
bleiben.

Die Zeitverläufe zeigen die letzten Sekunden des Laufs der Ansicht (das Fenster ist einstellbar): Moment mit
gestricheltem Momentensollwert, Drehzahl mit gestricheltem Drehzahlsollwert und den Schleppfehler, eine Farbe je Motor.
Ein eigener Canvas-Plotter von etwa hundert Zeilen zeichnet sie, ohne Bibliothek. Jede Pixelspalte zeichnet den
kleinsten und größten Wert der Proben, die sie überdeckt, und jede Probe hält den kleinsten und größten über ihre
Grundschritte, sodass die Spitze eines einzelnen Grundschritts bei jedem Zeitfenster sichtbar bleibt. Derselbe Plotter
zeichnet Arbeitspunkte und Frequenzgang.

## Unterschiede zum nativen Programm

WebAssembly startet keine Threads. Die Analysen, die nativ die Prozessoren teilen, laufen dort nacheinander. Die Seite
bietet Auslegungsprüfung, Frequenzgang und Anpassung. Maschinendateien sind TOML; JSON-Maschinendateien liest `sim-cli`
und die App, die Seite noch nicht. Die Anpassung nimmt eine Aufzeichnung in den Spalten des Simulators, keinen
Herstellerexport mit Zuordnungsdatei, und passt nur Massen und Reibung an (keine Steifigkeit, keinen Frequenzgang).

## Tests

`cargo test -p sim-wasm --release` rechnet die Anfragen des Moduls nativ: die Liste, die Auslegungsprüfung gegen die
Funktionen der Kommandozeile, Konfiguration von der Seite, die C-Funktionen; die acht Maschinendateien aus `machines/`,
eine übergebene Datei mit derselben Prüfung wie die eingebaute Kopie, ein falscher Schlüssel mit Datei und Zeile
abgelehnt; die Prüfung eines Pfads gegen `analyse_trajectory`, auch zeitoptimal, die Ansicht am Ende des Pfads wie ein
nativer Lauf, ein unlesbarer und ein leerer Pfad abgelehnt; `frame` mit unendlicher, NaN-, negativer und zu langer Zeit
abgelehnt; das Bild jeder Maschine gegen ihre native Szene, Datensätze, Begrenzungsquader und ein Hindernis von Hand,
die Zykluszeit der Ansicht, die Proben von 0,5 s (500 bei 1 ms, mit dem größten Moment und kleinsten Schleppfehler eines
nativen Laufs Schritt für Schritt); die Spalten von `analyze`, die Ecken der Kennlinie von Hand, der Frequenzgang gegen
`frequency_sweep`, eine Aufzeichnung ohne die Spalten des Simulators abgelehnt, die Texte in beiden Sprachen.

Die Seite selbst lief im kopflosen Chrome (puppeteer-core mit dem installierten Chrome, 14. November 2026): das T-Portal
in Echtzeit mit seinen Verläufen, die Prüfung bestanden bei 6,080 s Zyklus; auf Deutsch die CoreXY-Maschinendatei mit
einem Pfad aus Gerade, Bogen und Gerade samt Spur, ihre Prüfung (1,661 s) und ein Frequenzgang mit 40 Punkten (22 s).
