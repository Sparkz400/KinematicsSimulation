# Maschinendateien

Eine Maschinendatei beschreibt eine Maschine ohne Rust-Code und ohne Neuübersetzen: die Bauteile aus `sim-components`
wie eine Netzliste verschaltet (Motor, Getriebe, Riemenscheibe, Riemenabschnitte, Wagen, Führungen), dazu Achsen,
Grenzwerte, Fehlerbänder, Regelkreise und Zyklus einer Maschinenkonfiguration. Kommandozeile, App und SPS-Anbindung
laden sie zur Laufzeit aus einem Pfad. Die englische Fassung ist [machine-files.md](../machine-files.md); die
Referenzdokumente, auf die verwiesen wird, gibt es nur auf Englisch.

- [Was eine Maschinendatei ist](#was-eine-maschinendatei-ist)
- [Eine Maschinendatei verwenden](#eine-maschinendatei-verwenden)
- [Durchgerechnetes Beispiel: ein H-Bot](#durchgerechnetes-beispiel-ein-h-bot)
- [Die Knoten](#die-knoten)
- [Was geprüft wird, und wie Fehler lauten](#was-geprüft-wird-und-wie-fehler-lauten)
- [Das Modell dahinter](#das-modell-dahinter)
- [Was Maschinendateien noch nicht können](#was-maschinendateien-noch-nicht-können)
- [Wie es geprüft ist](#wie-es-geprüft-ist)
- [JSON, das JSON-Schema und Editoren](#json-das-json-schema-und-editoren)
- [Rust-Schnittstelle](#rust-schnittstelle)

## Was eine Maschinendatei ist

Eine Maschinenkonfiguration, wie die [Konfigurationsreferenz](../config-reference.md) sie beschreibt (`[sim]`,
`[parts]`, `[effects]`, `[belt]`, `[motor]`, `[gearbox]`, `[drive]`, `[ratings]`, `[profile]`, `[bounds]`, `[start]`,
`[[cycle.segment]]`, ...), und dazu eine Tabelle `[machine]`, die sagt, was womit verbunden ist:

```toml
format_version = 1          # der Konfiguration, wie in jeder Konfiguration

# ... die Wertetabellen einer Maschinenkonfiguration ...

[machine]
version = 1                 # von [machine]; eine Datei späterer Version wird abgelehnt, nicht falsch gelesen
id = "h-bot"                # unter diesem Namen führen die Oberflächen sie; Vorgabe: der Dateiname ohne Endung
name = "H-bot (machine file)"   # ein Übersetzungsschlüssel oder Klartext; Vorgabe: die id

[[machine.axis]]            # eine Tabelle je Knoten, jeder mit fester id
id = "x"
stroke_m = "geometry.stroke_x_m"   # eine Zahl, oder der Pfad einer Zahl in der Datei
# ...
```

Die Werte bleiben in den Wertetabellen, unter denselben Schlüsseln wie bei den mitgelieferten Maschinen. Darum
funktioniert alles andere mit einer Maschinendatei so, wie es ist: die Parameterliste, `--set`, der Katalog (`--motor`,
`--axis`, ...), die Fehlerbänder aus `[bounds]`, die Sensitivitätsanalyse, das Speichern. Ein Zahlenfeld eines Knotens
kann einen solchen Wert über seinen Pfad nennen (`radius_m = "geometry.pulley_radius_m"`); dann ändert eine Änderung
des Werts die Maschine. Eine Zahl selbst bleibt fest.

In `machines/` liegen drei Dateien:

| Datei | was sie ist |
| ----- | ----------- |
| `machines/single-axis.toml` | die Einzelachse aus `crates/machine-single-axis`, als Netzliste neu verschaltet |
| `machines/t-gantry.toml` | das T-Portal aus `crates/machine-t-gantry`, als Netzliste neu verschaltet (ohne Rahmennachgiebigkeit und Werkzeugbiegung, siehe [unten](#was-maschinendateien-noch-nicht-können)) |
| `machines/h-bot.toml` | das Beispiel unten: ein X-Y-Tisch, keine reale Maschine |

Am schnellsten kommt man zu einer neuen Maschine, indem man die ähnlichste kopiert und `[machine]` ändert.

## Eine Maschinendatei verwenden

Kommandozeile: `--machine-file DATEI` tritt an die Stelle der Kennung der Maschine, bei jedem Befehl außer `fmu`.

```
sim-cli verdicts --machine-file machines/h-bot.toml
sim-cli run --machine-file machines/t-gantry.toml --cycle --time 2 --out run.csv
sim-cli verdicts --machine-file machines/t-gantry.toml --bands --set belt.pretension_n=350
```

App: **Öffnen** in der Zeile der Konfigurationsdatei (oder **Öffnen…** für den Dateidialog) öffnet eine Maschinendatei wie eine Konfiguration, und die
Maschine wechselt mit. **Speichern** schreibt die geänderten Werte in die Datei zurück und behält ihre Kommentare; über
eine JSON-Datei wird nicht geschrieben. Die App kann auch gleich mit einer starten:
`sim-app --machine-file machines/h-bot.toml`.

SPS-Anbindung: `sim-opcua --machine-file machines/h-bot.toml [--port N] [--lang en|de]`, an Stelle der Kennung einer
Maschine.

## Durchgerechnetes Beispiel: ein H-Bot

Ein H-Bot ist ein X-Y-Tisch, dessen zwei Motoren fest an den Enden des X-Balkens stehen und einen Riemen antreiben, der
um Wagen und Kopf läuft. Drehen beide Motoren gleich, fährt X; drehen sie gegeneinander, fährt Y. Das ist die
Riemenführung des T-Portals mit flach liegender zweiter Achse. `machines/h-bot.toml` ist darum die Datei des T-Portals
mit diesen Änderungen:

1. In `[geometry]` wird `stroke_z_m` zu `stroke_y_m`, und `[axis.z]` wird zu `[axis.y]` (die Grenzen der Achse `y`).
2. `[machine]` nennt die Achsen `x` und `y`, gibt `y` keine Neigung (flach; das Z des T-Portals hat
   `incline_deg = 90`) und lässt den Überhang weg: der Kopf läuft auf seiner eigenen Führung auf dem Wagen.

Zuerst der Riemen. Die beiden Motorscheiben teilen ihn in drei Abschnitte mit je einer Spannung. Abschnitt 1 läuft von
seiner Klemme zu Motor A und wird je Meter x und je Meter y um 1 m länger; dreht Motor A vorwärts, speist er Riemen
hinein (`+1`). Abschnitt 2 läuft von A um den Kopf zu B und wird je Meter y um 2 m kürzer; A nimmt Riemen heraus
(`-1`), B speist ein (`+1`). Abschnitt 3 läuft von B zu seiner Klemme:

```toml
[[machine.section]]
id = "1"
length_m = 0.3                      # bei x = y = 0
axes = { x = 1, y = 1 }             # wie viel länger je Meter jeder Achse
pulleys = { pulley_a = 1 }          # +1: Vorwärtsdrehen dieser Scheibe speist Riemen ein
pos = [450, -80]                    # wo ein Knoteneditor ihn zeichnet; wird nicht gelesen

[[machine.section]]
id = "2"
length_m = 2.6
axes = { y = -2 }
pulleys = { pulley_a = -1, pulley_b = 1 }

[[machine.section]]
id = "3"
length_m = 1.3
axes = { x = -1, y = 1 }
pulleys = { pulley_b = -1 }
```

Allein daraus bestimmt der Lader die Kinematik: der Riemen muss seine Länge halten, also x = r/2 (a + b) und
y = r/2 (a - b) für die Scheibenwinkel a und b. Eine Führung, deren Abschnitte nicht bei jeder Bewegung der Achsen ihre
Länge halten können, oder deren Motoren nicht jede Achse festlegen, wird abgelehnt.

Die Antriebsstränge: jeder Motor dreht ein Getriebe, jedes Getriebe eine Scheibe. Die Daten von Motor und Getriebe
stehen in `[motor]`, `[drive]` und `[gearbox]` und gelten für alle Motoren:

```toml
[[machine.motor]]
id = "a"

[[machine.gearbox]]
id = "gearbox_a"
motor = "a"

[[machine.pulley]]
id = "pulley_a"
gearbox = "gearbox_a"
radius_m = "geometry.pulley_radius_m"
inertia_kgm2 = "pulley.inertia_kgm2"
```

(und dasselbe für `b`). Dann, was sich bewegt und was es führt. Ein Körper bewegt sich mit den Achsen, die er nennt; der
Kopf bewegt sich mit X und Y und trägt die Nutzlast:

```toml
[[machine.body]]
id = "carriage"
mass_kg = "mass.carriage_kg"
axes = ["x"]

[[machine.body]]
id = "head"
mass_kg = "mass.z_axis_kg"
payload_kg = "mass.payload_kg"
axes = ["x", "y"]

[[machine.guide]]
id = "guide_x"
axis = "x"
friction = "friction_x"             # die Tabelle mit ihren Reibwerten
carries = ["carriage", "head"]      # deren Gewicht quer zur Achse sie belastet

[[machine.guide]]
id = "guide_y"
axis = "y"
friction = "friction_z"
carries = ["head"]
```

Ausführen:

```
sim-cli verdicts --machine-file machines/h-bot.toml
```

```
check,motor,value,limit,result
motor-peak,a,0.09917,1.000,ok
motor-thermal,a,0.4428,100.0,ok
drive-current,a,0.2212,3.000,ok
drive-peak-current,a,0.8356,10.90,ok
gearbox-continuous,a,0.3766,40.00,ok
gearbox-input-speed,a,1914,13000,ok
gearbox-peak,a,1.326,64.00,ok
pulley-bearing-life,a,213589,20000,ok
gearbox-life,a,74415368568,20000,ok
belt-tooth-load,a,0.1164,1.000,ok
dc-link-energy,a,1.106,6.205,ok
dc-link-overvoltage,a,169.6,257.6,ok
motor-peak,b,0.09105,1.000,ok
motor-thermal,b,0.4562,100.0,ok
drive-current,b,0.2245,3.000,ok
drive-peak-current,b,0.7675,10.90,ok
gearbox-continuous,b,0.3850,40.00,ok
gearbox-input-speed,b,1914,13000,ok
gearbox-peak,b,1.182,64.00,ok
pulley-bearing-life,b,213351,20000,ok
gearbox-life,b,70668656880,20000,ok
belt-tooth-load,b,0.1038,1.000,ok
dc-link-energy,b,1.128,6.205,ok
dc-link-overvoltage,b,169.9,257.6,ok
settling,,0.00001314,0.0001000,ok
belt-tension,,345.7,1500,ok
belt-slack,,229.2,0,ok
gearbox-radial-load,,637.1,1500,ok
guide-load,,107.9,2000,ok
guide-life,,6789511,20000,ok
```

Jede Prüfung besteht. Die Motoren bleiben kühler als beim T-Portal (`motor-thermal` 0,44 gegen dessen 1,23): nichts muss
gegen die Schwerkraft gehalten werden. Die Signale folgen den ids: `axis.y.position`, `motor.a.torque`, `belt.2.tension`, `power.guide_y`, ...

## Die Knoten

Jeder Knoten hat eine `id`: Buchstaben, Ziffern, `_` und `-`, eindeutig in der ganzen Datei (nicht nur in ihrer
Liste), damit ein Verweis nicht zweierlei meinen kann und ein Editor sie behalten kann, wenn der Knoten verschoben wird.
Jeder Knoten darf ein `pos = [x, y]` haben, wo ein Knoteneditor ihn zeichnet; die Simulation liest es nicht. Ein
Zahlenfeld nimmt eine Zahl oder den Pfad einer Zahl in der Datei. Die Reihenfolge der Motoren in der Datei ist die der
Motorsignale und von `--motor N`.

| Liste | Feld | | Bedeutung |
| ----- | ---- | - | --------- |
| `axis` | `id` | Pflicht | die Achse; Signale `axis.<id>.*`, ihre Grenzen in `[axis.<id>]` |
| | `key` | | Übersetzungsschlüssel ihres Namens; Vorgabe `axis-<id>` |
| | `stroke_m` | Pflicht | Hub, m; die Achse läuft von 0 bis zu ihm |
| | `incline_deg` | | Winkel über der Waagrechten, Grad, 90 senkrecht und nach + steigend; Vorgabe 0 |
| `motor` | `id` | Pflicht | ein Servomotor mit seinem Antrieb; Signale `motor.<id>.*` |
| | `key` | | Übersetzungsschlüssel seines Namens; Vorgabe `motor-<id>` |
| `gearbox` | `motor` | Pflicht | der Motor, der es antreibt; ein Getriebe je Motor; Übersetzung 1 ist eine Kupplung |
| `pulley` | `gearbox` | Pflicht | das Getriebe, dessen Abtrieb sie dreht; eine Scheibe je Getriebe |
| | `radius_m` | Pflicht | Wirkradius, m |
| | `inertia_kgm2` | Pflicht | Trägheit ohne Riemen, kg m² |
| `section` | `length_m` | Pflicht | geometrische Länge bei allen Achsen auf 0, m |
| | `axes` | Pflicht | je Achsen-id: wie viel länger der Abschnitt je Meter dieser Achse wird |
| | `pulleys` | Pflicht | je Scheiben-id: `+1`, wo ihr Vorwärtsdrehen Riemen einspeist, `-1`, wo es Riemen herausnimmt |
| | `key` | | Übersetzungsschlüssel seines Namens; Vorgabe `belt-section-<id>`; Signale `belt.<id>.*` |
| `body` | `mass_kg` | Pflicht | Masse, kg |
| | `payload_kg` | | Nutzlast, die er trägt, solange `effects.payload` an ist und der Greifer sie hält, kg |
| | `axes` | Pflicht | die Achsen, mit denen er sich bewegt |
| `guide` | `axis` | Pflicht | die Achse, die sie führt; höchstens eine Führung je Achse; ihr Verlust ist `power.<id>` |
| | `friction` | Pflicht | die Tabelle ihrer Reibwerte, aufgebaut wie `[friction_x]` |
| | `carries` | | Körper, deren Gewicht quer zur Achse sie trägt |
| | `overhang` | | ein Körper, der über die Führung hinausragt: entlang `overhang_axis` beschleunigt, kippt er ihre Wagen |
| | `overhang_axis`, `lever_m`, `spacing_m` | mit `overhang` | die Achse, der Hebel des Schwerpunkts des Überhangs und der Abstand der beiden Führungswagen, m; das Kippen legt 2 m a Hebel / Abstand auf sie |
| | `lever_axis` | | eine Achse, deren Weg den Hebel verkürzt |
| | `side_load` | | die Führung trägt auch die Trägheitskraft m a des Überhangs; Vorgabe false |

Es gibt so viele Motoren wie Achsen. Eine Datei nennt jede Liste, die sie verwendet; `guide` darf fehlen (keine
Führungsreibung).

## Was geprüft wird, und wie Fehler lauten

Eine Maschinendatei wird beim Laden vollständig geprüft, bevor etwas läuft:

- das TOML oder JSON selbst, `format_version` und `machine.version`;
- jeder Knoten: kein Feld, das seine Art nicht hat, jedes Pflichtfeld vorhanden, jeder Wert vom richtigen Typ, ids
  gültig und eindeutig, jeder Verweis auf einen vorhandenen Knoten, jeder Pfad auf eine vorhandene Zahl;
- die Maschine: Massen, Radien und Längen positiv, ein Getriebe je Motor und eine Scheibe je Getriebe, so viele Motoren
  wie Achsen, eine Riemenführung, die die Riemenlänge hält und jede Achse festlegt;
- jeder andere Schlüssel gegen die Schlüssel der Konfigurationen der eingetragenen Maschinen (so wird `pretensoin_n`
  beim Namen abgelehnt), dazu `[axis.<id>]` je Achse, eine Reibtabelle je Führung und jeder Schlüssel, den die
  Netzliste über einen Pfad nennt;
- dann alles, was die Simulation bei ihrem Start liest, wie bei einer Konfiguration: der Antrieb, der Zyklus, die
  Startlage, die Katalogteile, die die Datei nennt.

Ein Fehler nennt Datei, Zeile und Schlüssel, in der Sprache der Oberfläche:

```
$ sim-cli verdicts --machine-file broken.toml --lang de
Die Konfiguration enthält einen Wert, den diese Maschine nicht kennt: broken.toml:604: machine.guide[guide_z].side_lod
```

Ein Knoten wird mit seiner Liste und seiner id genannt, `machine.guide[guide_z]`; bevor seine id gelesen ist, mit
seinem Platz in der Liste, `machine.gearbox[#2]` (ab 1). Geht es um einen Wert, den ein Knoten über seinen Pfad nennt,
ist die Zeile die des Felds im Knoten. Die Zeile wird bei TOML-Dateien gefunden; bei JSON steht nur der Schlüssel da.

## Das Modell dahinter

Eine Maschinendatei baut eine `ConfigMachine`: ein Riemenportal beliebiger Riemenführung mit n Achsen q und ebenso
vielen Motoren. Es ist das Modell der Crates Einzelachse und T-Portal, für jede Riemenführung aufgeschrieben (die
[Modellreferenz](../model-reference.md) leitet es für diese her):

- Riemenabschnitt s hat die Länge L_s = L0_s + Σ_i c_si q_i, dazu die Zähne im Eingriff an jeder angetriebenen
  Scheibe, an der er endet (`Belt::tooth_length`), und Scheibe k speist je Radiant f_sk r_k Riemen in ihn ein.
- Der Riemen zieht Achse i mit -Σ_s c_si T_s und dreht Scheibe k mit r_k Σ_s f_sk T_s (über das Riemenmaterial, mit dem
  elastischen Gesetz von `Belt`).
- Starr (effects.belt_stretch aus) hält der Riemen seine Länge: C q = F diag(r) a, also a = K q mit K als
  Kleinste-Quadrate-Lösung von F diag(r) K = C. Der Lader lehnt eine Führung ab, bei der K das C nicht genau
  wiedergibt oder nicht umkehrbar ist.
- In Ruhe halten die Spannungen jede Achse gegen die Schwerkraft, und jeder geschlossene Riemen behält sein Material:
  Abschnitte, die sich eine Scheibe teilen, sind ein Riemen, und er ist geschlossen, wo jede seiner Scheiben ihn auf
  einer Seite einspeist und auf einer anderen herausnimmt. Eine Maschine darf mehrere Riemen haben (etwa einen je
  Achse); ein Riemen, der auf einer Trommel endet, hat keine solche Summe.
- Die Masse auf Achse i ist die Summe der Körper, die sich mit ihr bewegen; die Achsen gelten als rechtwinklig
  zueinander, die Massenmatrix der Körper ist also diagonal. Scheiben und Läufer (J_Scheibe + i² J_Läufer je Motor, die
  Riemenmasse zu gleichen Teilen auf die Scheiben gelegt) kommen als K^T J K hinzu. Die Schwerkraft zieht Achse i mit
  m_i g sin(Neigung_i).
- Getriebe (Lose, Steifigkeit, Wirkungsgrad), Motorreibung, Drehmomentwelligkeit und Rastmoment, Zahneingriff,
  Haltebremse, Endanschläge an beiden Enden jedes Hubs, LuGre-Führungsreibung mit Lastabhängigkeit: die Bauteile aus
  `sim-components::mech`, wie die Crates sie verwenden.

Regler, Antriebe, Zwischenkreis, Wicklungserwärmung, Analysen, Urteile, Sensitivität und Fehlerbänder sind die jeder
Maschine; an ihnen ist nichts besonders für Maschinendateien.

## Was Maschinendateien noch nicht können

- Diese Effekte: `frame_compliance`, `tool_flex`, `idlers`, `cable_chain`, `process_force`, `thermal_feedback`,
  `belt_tooth_jump`, `belt_span_vibration`. Eine Datei, die einen einschaltet, wird abgelehnt (`effects.<Name> (not in
  machine files yet)`), damit ein Ergebnis nie einen verlangten Effekt auslässt. `part_heating` geht (es zeigt nur an).
- Andere Mechanismen als Riemen: Spindeln, Zahnstangen, Koppelgetriebe, Drehgelenke. Es gibt die Knotenarten oben.
- Verschiedene Motoren, Getriebe oder Antriebe je Achse: `[motor]`, `[gearbox]` und `[drive]` gelten für alle.
- Eine Strukturresonanz: die Maschine meldet keine, darum behält `effects.shaper_follows_pose` die feste
  Shaper-Frequenz, und die App hat keine Eigenform zu zeigen.
- Abschnittslängen sind Zahlen (oder Pfade), keine Formeln: ein Abschnitt, dessen Länge von einem Hub abhängt, folgt
  einer Änderung dieses Hubs nicht von selbst.
- Die 3D-Ansicht zeichnet eine schlichte Maschine (einen Balken, einen Quader je Körper, eine Antriebseinheit je Motor),
  nicht den Riemen.
- `sim-cli fmu` lehnt eine Maschinendatei ab: die FMU sucht ihre Maschine unter den eingetragenen.
- `sim-cli list` und die Maschinenliste der App zeigen nur die eingetragenen Maschinen; eine Datei wird geöffnet, nicht
  aufgeführt.

## Wie es geprüft ist

`crates/sim-machines/tests/machine_files.rs` lädt `machines/single-axis.toml` und `machines/t-gantry.toml` und hält sie
gegen ihre Crates:

- die Maschine selbst, mit fünf Sätzen von Effekten (Vorgabe, starr, ohne Verluste, ohne Schwerkraft, ohne
  Riemendetails und Nutzlast), in drei Lagen, mit gehaltener und abgesetzter Nutzlast und mit greifender Bremse:
  Kinematik, Ruhelage und Haltemomente, die Ableitungen eines aus der Ruhe verschobenen Zustands unter Momenten,
  Energie, Vorsteuerung, starre Riemenspannungen und jedes Signal, das beide haben: stimmen auf 1e-9 überein;
- der ganze Zyklus (Regler, Antriebe, Zwischenkreis, Wicklung), elastisch und starr, ohne die Geberstriche: jede
  Motorlast und jeder Urteilswert stimmt auf 1e-5. Genauer geht es nicht: die beiden summieren in anderer Reihenfolge,
  und während eine Führung aus der Haftung losbricht, wächst ein Unterschied von 1e-36 alle 30 Schritte auf das
  Zehnfache, bevor er sich bei der Konstantfahrt wieder schließt;
- dasselbe mit den Geberstrichen, wie ausgeliefert: dieselben Urteile, Grenzwerte und Bestanden oder nicht. Dort
  verschiebt ein Unterschied im letzten Bit einen Geberstrich um einen Schritt, und Schaltschwellen (der Bremschopper)
  tragen das weiter: das Crate mit einem um 1e-9 schwereren Wagen streut in ihrer höchsten Bremsleistung um 0,5 %, und
  die Datei muss innerhalb der dreifachen eigenen Streuung des Crates liegen, oder innerhalb 1e-4.

Der H-Bot hat kein Crate, gegen das er gehalten werden könnte. Alle drei Dateien bestehen die Standardprüfung, die jede
Maschine besteht (Hin- und Rückrechnung, statisches Halten, Determinismus, Energieerhaltung ohne Verluste, die
Verlustbilanz mit ihnen), und Motorwinkel und -momente des H-Bots werden gegen eine Handrechnung geprüft. Bei seiner
Standardprüfung ist die Getriebelose aus: nichts zieht seine Getriebe an eine Flanke, und der kleine Impuls der
Prüfung schlägt sie durch die Lose, was die Energie um 0,17 % driften lässt, mehr als die erlaubten 0,1 %. Das liegt
am Getriebekontakt bei der Schrittweite der Prüfung, nicht an der Netzliste: das Crate T-Portal ohne Schwerkraft driftet
auf dieselbe Weise um 0,19 %.

`crates/sim-machines/tests/machine_file_review.rs` hält fest, was eine unabhängige Durchsicht durch Versuch gefunden
hat und was behoben ist: eine Maschine mit zwei oder drei getrennten Riemen (ihre Ruhelage war falsch, wo eine Achse
senkrecht steht), eine Führung mit Überhang, wo die Antriebsmassen die Achsen koppeln, eine Achse, die kein Körper
bewegt, das Y des H-Bots entlang X gezeichnet, ungeprüfte `machine.id` und `machine.name`, Fehlerzeilen, die
anderswohin zeigten, und eine JSON-Version `1.0`.

Die Tests prüfen außerdem, dass eine JSON-Datei dieselbe Maschine ergibt wie ihr TOML, dass
`schema/machine-file.schema.json` aktuell ist, dass neun Arten kaputter Dateien mit dem richtigen Schlüssel und der
richtigen Zeile abgelehnt werden, und (in `sim-cli`, `sim-app` und `sim-opcua`), dass jede Oberfläche
Maschinendateien lädt, ausführt und ablehnt.

## JSON, das JSON-Schema und Editoren

Eine Datei, deren Name auf `.json` endet, wird als JSON gelesen: dieselben Tabellen als Objekte, die Knotenlisten als
Felder von Objekten. `format_version` und `machine.version` sind ganze Zahlen (`1`, nicht `1.0`).

`schema/machine-file.schema.json` (JSON Schema, Entwurf 2020-12) beschreibt `[machine]` vollständig, mit der Bedeutung
jedes Felds (auf Englisch); die Wertetabellen führt es als Objekte (ihre Schlüssel stehen in der
[Konfigurationsreferenz](../config-reference.md)). Es wird aus derselben Liste geschrieben, gegen die der Lader prüft
(`sim_netlist::netlist::NODES`). Nach einer Änderung dieser Liste schreibt man es neu mit

```
MACHINE_SCHEMA_WRITE=1 cargo test -p sim-machines --test machine_files the_json_schema_file_is_current
```

Das Format ist auch für einen Knoteneditor gedacht: ids sind fest und eindeutig in der Datei, Verweise gehen über ids,
`pos` hält den Platz eines Knotens auf der Zeichenfläche und wird sonst nicht beachtet, und nichts hängt an der
Reihenfolge der Knoten außer der Reihenfolge der Motoren und Achsen.

## Rust-Schnittstelle

- `sim_machines::load_file(path) -> Result<&'static MachineEntry, FileError>`: das, was die Oberflächen aufrufen. Der
  Eintrag wird wie einer aus `MACHINES` verwendet (`Sim::new(entry, &entry.config())`, die Analysen, `translations`).
  Er lebt so lange wie das Programm.
- `sim_netlist::from_text(toml_text, lines, fallback_id, known)`: dasselbe auf dem TOML-Text einer Datei, mit den
  Konfigurationen, deren Schlüssel eine Datei verwenden darf (`known`); gibt den Fehler mit seiner Zeile zurück, wo
  `lines` gesetzt ist. JSON wird in `sim_machines` umgewandelt, der einzigen Schicht, die es liest; die Schicht der
  Maschinen-Crates hat nur `toml`.
- `FileError { file, line, error }`, `error` ein `ConfigError` (ein Übersetzungsschlüssel und der Pfad, um den es
  geht), wie bei jeder Konfiguration.
- `sim_netlist::build(&Config)`: das `MachineEntry::build` jeder Maschinendatei; die Netzliste steckt in der
  Konfiguration.
- `sim_netlist::Netlist::parse(&Config)`, `ConfigMachine::new(&Config, &Netlist)`, `json_schema()` (als Wert; `sim_machines::json_schema()` schreibt ihn),
  `line_of(text, path)`.
