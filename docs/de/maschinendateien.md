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
- [Motoren, die sich unterscheiden](#motoren-die-sich-unterscheiden)
- [Die anderen Beispiele: ein CoreXY und ein XYZ-Portal](#die-anderen-beispiele-ein-corexy-und-ein-xyz-portal)
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

In `machines/` liegen fünf Dateien:

| Datei | was sie ist |
| ----- | ----------- |
| `machines/single-axis.toml` | die Einzelachse aus `crates/machine-single-axis`, als Netzliste neu verschaltet |
| `machines/t-gantry.toml` | das T-Portal aus `crates/machine-t-gantry`, als Netzliste neu verschaltet, mit seinen Umlenkrollen, seinem Rahmen, seiner Energiekette und seinen Riementrumen (die Werkzeugbiegung gibt es in Maschinendateien nicht, siehe [unten](#was-maschinendateien-noch-nicht-können)) |
| `machines/h-bot.toml` | das Beispiel unten: ein X-Y-Tisch, keine reale Maschine |
| `machines/corexy.toml` | ein CoreXY-Tisch: zwei feste Motoren, zwei gekreuzte Riemen ([unten](#die-anderen-beispiele-ein-corexy-und-ein-xyz-portal)) |
| `machines/xyz-portal.toml` | ein dreiachsiges kartesisches Portal, X, Y und Z je auf eigenem Riemen, mit einem kleineren eigenen Z-Motor |

Am schnellsten kommt man zu einer neuen Maschine, indem man die ähnlichste kopiert und `[machine]` ändert.

## Eine Maschinendatei verwenden

Kommandozeile: `--machine-file DATEI` tritt an die Stelle der Kennung der Maschine, bei jedem Befehl.

```
sim-cli verdicts --machine-file machines/h-bot.toml
sim-cli run --machine-file machines/t-gantry.toml --cycle --time 2 --out run.csv
sim-cli verdicts --machine-file machines/t-gantry.toml --bands --set belt.pretension_n=350
sim-cli fmu --machine-file machines/h-bot.toml --fmi 3 --out h-bot.fmu
```

Eine FMU einer Maschinendatei trägt die Datei unter ihren Ressourcen (`resources/machine.toml`) und baut die Maschine
beim Instanziieren daraus, für FMI 2.0 wie 3.0; das Werkzeug, das sie einliest, braucht sonst nichts.

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

| `axis` | `cable_chain` | | die Achse schleppt eine am Ende 0 befestigte Energiekette (`effects.cable_chain`, Werte in `[cable_chain]`); Vorgabe false |
| `pulley` | `at_m` | | wo die 3D-Ansicht sie zeichnet, `[x, y, z]`, m; Vorgabe an einem Ende der ersten Achse |
| `section` | `route` | | die Scheiben, Umlenkrollen und Körper (eine Klemme daran), an denen der Abschnitt vorbeiläuft, der Reihe nach, für die 3D-Ansicht; Vorgabe seine Scheiben und der Körper, an dem er geklemmt ist |
| `idler` | `speed` | Pflicht | Riemengeschwindigkeit an ihr je Geschwindigkeit jeder Achse, m/s je m/s (`effects.idlers`) |
| | `radius_m`, `inertia_kgm2`, `friction_nm` | Pflicht | ihr Radius, ihre Trägheit und ihr Lagerreibmoment |
| | `body`, `at_m` | | ein Körper, auf dem sie mitfährt, und wo die 3D-Ansicht sie zeichnet (von diesem Körper aus) |
| `frame` | `axis` | Pflicht | die waagrechte Achse, entlang der der Rahmen schwingt (`effects.frame_compliance`, elastisches Modell); höchstens einer |
| | `mass_kg`, `stiffness_n_m`, `damping_n_s_m` | Pflicht | Masse, Steifigkeit und Dämpfung des Rahmens in seiner Schwingung |
| `span` | `section` | Pflicht | ein freies Riementrum dieses Abschnitts, für seine Querschwingung (`effects.belt_span_vibration`) |
| | `length_m` | Pflicht | seine Länge bei allen Achsen auf 0, m |
| | `axes` | | wie viel länger es je Meter jeder Achse wird |

Es gibt so viele Motoren wie Achsen. Eine Datei nennt jede Liste, die sie verwendet; `guide`, `idler`, `frame` und
`span` dürfen fehlen. Ein Effekt, der Knoten braucht, die die Datei nicht hat (`frame_compliance` ohne `frame`,
`belt_span_vibration` ohne `span`), lehnt die Datei mit `machine.<Liste>` ab.

## Motoren, die sich unterscheiden

`[motor]`, `[gearbox]`, `[drive]`, `[ratings]` und `[parts]` gelten für jeden Motor. Eine Tabelle
`[<Tabelle>.<Motor-id>]` gibt einem Motor eigene Werte darüber, Schlüssel für Schlüssel; sie darf nur Schlüssel haben,
die die gemeinsame Tabelle hat:

```toml
[gearbox.mz]          # der Z-Motor aus machines/xyz-portal.toml hat eine eigene Getriebeübersetzung
ratio = 10.0

[motor.mz]            # und einen kleineren Motor
rotor_inertia_kgm2 = 0.15e-4
rated_current_a = 1.411

[drive.mz]            # seinen Antrieb dafür eingestellt
peak_torque_nm = 1.9
rated_torque_nm = 0.635
velocity_kp_nm_s_rad = 0.01

[ratings.mz]          # und was sein kleineres Getriebe und sein Antrieb erlauben
gearbox_rated_torque_nm = 18.0
drive_rated_current_a = 1.5

[parts.mz]            # oder Motor, Getriebe und Antrieb aus dem Katalog, nur für diesen Motor
motor = "1FK2203-2AG"
```

Der Antriebsstrang dieses Motors in der Maschine (Läufer, Getriebe, Motorreibung, Welligkeit, Bremse), sein Antrieb im
Regler (Strom- und Drehzahlregler, Drehmoment-Drehzahl-Kennlinie, Wicklung, Zwischenkreis) und seine Urteile (Motor,
Antriebsstrom, Getriebe, Lager, Riemenzähne, Bremsen) nehmen seine eigenen Werte; die Prüfungen der ganzen Maschine
behalten die gemeinsamen. Ein in `[parts.<id>]` genanntes Teil setzt die Werte dieses Motors über die Zahlen, wie
`[parts]` es tut; `belt` und `axis` gehören der Maschine und werden dort abgelehnt. Gemeinsam bleiben: die Taktzeiten
der Antriebe, der Rauschstartwert, ein gemeinsamer Zwischenkreis und das Bremsmodul. `sim-cli catalog` sucht den
gemeinsamen Motor; ein Motor mit eigenem `[parts.<id>]` behält sein Teil bei der Suche. Der zeitoptimale Bahnplaner
nimmt für jeden Motor die Kennlinie des gemeinsamen `[drive]`.

## Die anderen Beispiele: ein CoreXY und ein XYZ-Portal

`machines/corexy.toml` ist ein CoreXY-Tisch. Seine beiden Motoren stehen an den vorderen Ecken; jeder treibt einen
Riemen, der an beiden Enden am Kopf geklemmt ist, also teilt die Scheibe jedes Motors ihren Riemen in zwei Abschnitte.
Riemen A wird auf einer Seite seiner Scheibe um x + y länger, Riemen B um x - y, also sind die Scheibenwinkel
a = (x + y) / r und b = (x - y) / r: der Lader findet das allein aus den Abschnitten, und ein Motor allein bewegt den
Kopf diagonal. Die Brücke fährt in Y, der Kopf darauf in X. Seine Umlenkrollen (vier, als Vorgabe aus) und die `route`
jedes Abschnitts zeichnet die 3D-Ansicht.

`machines/xyz-portal.toml` ist ein Dreiachsportal: X, Y und Z je auf einer eigenen Riemenschleife um die Scheibe ihres
Motors und eine Umlenkrolle am fernen Ende, an ihrem Schlitten geklemmt, Z senkrecht. Sein Z-Motor ist kleiner, mit
einem Getriebe der Übersetzung 10, einem dafür eingestellten Antrieb und eigenen Grenzwerten ([oben](#motoren-die-sich-unterscheiden));
er hält Schlitten und Nutzlast, m g r / i = 3,5 kg · 9,81 m/s² · 0,02 m / 10 = 0,0686 N m.

Beide bestehen mit den Platzhalterwerten des T-Portals jedes Urteil ihres Zyklus (`sim-cli verdicts --machine-file ...`);
keine ist eine reale Maschine.

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
  `sim-components::mech`, wie die Crates sie verwenden. Jeder Motor aus seiner eigenen Konfiguration
  ([oben](#motoren-die-sich-unterscheiden)).
- Umlenkrollen (`machine.idler`, `effects.idlers`): Rolle d lässt Riemen mit s_d · v durch (ihr `speed`), ihre Trägheit
  J / r² kommt also als m_d s_d s_d^T zur Massenmatrix der Achsen (die dann voll ist), und ihr Lager zieht die Achsen
  mit F_d tanh(s_d · v / (w r)) s_d zurück, F_d ihr Reibmoment durch ihren Radius. Sie tragen kein Gewicht.
- Eine Energiekette an einer Achse: ihre bewegte Masse m(q) = mu (L0 + q / 2) braucht m dv/dt + m' v² / 2, und sie
  schleppt.
- Eine Prozesskraft je Achse, `process.force_<Achse>_n`, auf das Werkzeug, solange der Zyklus verweilt.
- Der Rahmen (`machine.frame`, elastisches Modell): er schwingt um u entlang einer waagrechten Achse f; die Körper
  bewegen sich über Grund mit v + e_f du/dt, und der Rahmen nimmt die Kräfte zurück, die sein Riemen, seine Führungen,
  Umlenkrollen, Kette und Anschläge auf sie geben, nicht aber die Prozesskraft, die das Werkstück auf dem Boden nimmt.
  Das Werkzeug bewegt sich mit dem Rahmen.
- Thermische Rückwirkung: der Wirkungsgrad jedes Getriebes und die viskose Reibung jeder Führung folgen ihrer
  Temperatur, die Steifigkeit des Riemens der Umgebung (`mech::ThermalFeedback`).
- Zahnsprung (elastisches Modell): an jeder Motorscheibe zwischen dem Abschnitt, den sie speist, und dem, aus dem sie
  nimmt (`mech::ToothJump`); eine Scheibe mit einem Abschnitt nur auf einer Seite (eine Trommel) lehnt den Effekt ab.
- Trumschwingung (`machine.span`): die erste Querschwingung jedes Trums bei der Spannung seines Abschnitts, gegen das,
  was die Antriebe anregen (`mech::span_frequency`).
- Die Eigenform des Riemens (elastisches Modell): die Achsen, und der Rahmen, wo er schwingt, auf der Steifigkeit des
  Riemens bei festgehaltenen Scheiben, K = Σ_s EA / L_s c_s c_s^T, gegen ihre Massenmatrix; die niedrigste ist
  `resonance_hz` in einer Lage, also stellt `effects.shaper_follows_pose` den Shaper darauf, und die App zeigt ihre
  Form (auf 1 m Werkzeugweg skaliert).

Regler, Antriebe, Zwischenkreis, Wicklungserwärmung, Analysen, Urteile, Sensitivität und Fehlerbänder sind die jeder
Maschine; an ihnen ist nichts besonders für Maschinendateien.

## Was Maschinendateien noch nicht können

- `tool_flex`: die Z-Achse des T-Portals, die sich am Wagen neigt. Dafür braucht es, wo Schwerpunkt und Werkzeugspitze
  dieser Achse unter ihrer Führung liegen und wie sich das mit ihrem Weg ändert, eine Geometrie, die eine Netzliste aus
  Riemen und Körpern nicht hat; ein Knoten dafür wäre der Sonderfall des T-Portals. Eine Datei, die ihn einschaltet,
  wird abgelehnt (`effects.tool_flex (not in machine files yet)`), damit ein Ergebnis nie einen verlangten Effekt
  auslässt.
- Andere Mechanismen als Riemen: Spindeln, Zahnstangen, Koppelgetriebe, Drehgelenke. Es gibt die Knotenarten oben.
- Andere Eigenformen als die des Riemens: der Rahmen geht in die des Riemens ein, wo er schwingt, die eigene
  Steifigkeit der Führungen und Motoren nicht, und die Biegung eines Körpers ist nicht modelliert.
- Abschnitts- und Trumlängen sind Zahlen (oder Pfade), keine Formeln: eine, die von einem Hub abhängt, folgt einer
  Änderung dieses Hubs nicht von selbst.
- Die 3D-Ansicht zeichnet den Riemen als gerade Streifen von Punkt zu Punkt der Route jedes Abschnitts, nicht um seine
  Scheiben herum.
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

Derselbe Vergleich läuft mit jedem Effekt, den Maschinendateien seit dem 13. November haben, eingeschaltet (Rahmen,
Umlenkrollen, Energiekette, Prozesskraft, thermische Rückwirkung mit warmen Teilen, Zahnsprung, Trumschwingung),
elastisch und starr, und mit allen zugleich: die Datei des T-Portals nennt ihre fünf Umlenkrollen, ihren Rahmen und
acht Trume und gibt die Zahlen des Crates auf 1e-9 (`the_effects_of_a_machine_file_are_its_crates`). Die
Handrechnungen der Effekte selbst sind die des Crates.

Der H-Bot, der CoreXY und das Portal haben kein Crate, gegen das sie gehalten werden könnten. Alle fünf Dateien bestehen die Standardprüfung, die jede
Maschine besteht (Hin- und Rückrechnung, statisches Halten, Determinismus, Energieerhaltung ohne Verluste, die
Verlustbilanz mit ihnen), und Handrechnungen prüfen Motorwinkel und -momente des H-Bots und des CoreXY, die eigene Getriebeübersetzung des
Portals (im Winkel und in der Drehzahl seines Z-Motors), das Moment, mit dem sein Z-Motor den Schlitten hält, den
eigenen Antrieb und die eigenen Grenzwerte seines Z-Motors, die Eigenform des Riemens der Einzelachse und die
Riemenstreifen, die die 3D-Ansicht zeichnet. CoreXY und Portal liegen (in X und Y) flach wie der H-Bot und laufen die
Standardprüfung aus demselben Grund ohne Lose. Bei seiner
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
- `sim_components::catalog::motor_config(&Config, motor_id)`: die Konfiguration, die ein Motor sieht
  ([Motoren, die sich unterscheiden](#motoren-die-sich-unterscheiden)); `Controller::drive_params_of(k)` ist sein
  Antrieb.
- `sim_machines::load_text(toml_text, id)`: der Text einer Maschinendatei, wie `load_file` ihn liest (das, was eine FMU
  mit ihrer `resources/machine.toml` tut).
