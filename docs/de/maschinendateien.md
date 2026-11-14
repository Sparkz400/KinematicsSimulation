# Maschinendateien

Eine Maschinendatei beschreibt eine Maschine ohne Rust-Code und ohne Neuübersetzen: die Bauteile aus `sim-components`
wie eine Netzliste verschaltet (Motor, Getriebe, Riemenscheibe, Riemenabschnitte, Wagen, Führungen, und neben den
Riemen Kugelgewindetriebe, Zahnstangen, Linearmotoren, Rundtische und Schneckengetriebe, oder ein ebenes
Parallelkoppelgetriebe), dazu Achsen,
Grenzwerte, Fehlerbänder, Regelkreise und Zyklus einer Maschinenkonfiguration. Kommandozeile, App und SPS-Anbindung
laden sie zur Laufzeit aus einem Pfad. Die englische Fassung ist [machine-files.md](../machine-files.md); die
Referenzdokumente, auf die verwiesen wird, gibt es nur auf Englisch.

- [Was eine Maschinendatei ist](#was-eine-maschinendatei-ist)
- [Eine Maschinendatei verwenden](#eine-maschinendatei-verwenden)
- [Durchgerechnetes Beispiel: ein H-Bot](#durchgerechnetes-beispiel-ein-h-bot)
- [Die Knoten](#die-knoten)
- [Motoren, die sich unterscheiden](#motoren-die-sich-unterscheiden)
- [Spindeln, Zahnstangen, Linearmotoren und Rundtische](#spindeln-zahnstangen-linearmotoren-und-rundtische)
- [Ein Koppelgetriebe: der ebene Delta](#ein-koppelgetriebe-der-ebene-delta)
- [Schneckengetriebe](#schneckengetriebe)
- [Die Umschlingung des Riemens und eine sich neigende Achse](#die-umschlingung-des-riemens-und-eine-sich-neigende-achse)
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
| | `stroke_m` | Pflicht | Hub, m; die Achse läuft von 0 bis zu ihm (eine Drehachse hat statt dessen `stroke_rad`) |
| | `stroke_rad` | | Bereich einer Drehachse ab 0, rad, an Stelle von `stroke_m` |
| | `incline_deg` | | Winkel über der Waagrechten, Grad, 90 senkrecht und nach + steigend; Vorgabe 0; bei einer Drehachse die Neigung der Achse, um die sie dreht, aus der Senkrechten (0, eine C-Achse) nach x (90, eine A-Achse) |
| `motor` | `id` | Pflicht | ein Servomotor mit seinem Antrieb; Signale `motor.<id>.*` |
| | `key` | | Übersetzungsschlüssel seines Namens; Vorgabe `motor-<id>` |
| `gearbox` | `motor` | Pflicht | der Motor, der es antreibt; ein Getriebe je Motor (keins bei einem Linearmotor); Übersetzung 1 ist eine Kupplung |
| `pulley` | `gearbox` | Pflicht | das Getriebe, dessen Abtrieb sie dreht; eine Scheibe, Spindel, Zahnstange oder ein Rundtisch je Getriebe |
| | `radius_m` | Pflicht | Wirkradius, m |
| | `inertia_kgm2` | Pflicht | Trägheit ohne Riemen, kg m² |
| `section` | `length_m` | Pflicht | geometrische Länge bei allen Achsen auf 0, m |
| | `axes` | Pflicht | je Achsen-id: wie viel länger der Abschnitt je Meter dieser Achse wird |
| | `pulleys` | Pflicht | je Scheiben-id: `+1`, wo ihr Vorwärtsdrehen Riemen einspeist, `-1`, wo es Riemen herausnimmt |
| | `key` | | Übersetzungsschlüssel seines Namens; Vorgabe `belt-section-<id>`; Signale `belt.<id>.*` |
| `body` | `mass_kg` | Pflicht | Masse, kg |
| | `payload_kg` | | Nutzlast, die er trägt, solange `effects.payload` an ist und der Greifer sie hält, kg |
| | `axes` | Pflicht | die Achsen, mit denen er sich bewegt |; Drehachsen darunter nehmen ihn mit herum, die erste ganz außen
| | `offset_m` | | auf Drehachsen: sein Schwerpunkt von dort, wo sich ihre Achsen treffen, `[x, y, z]` bei Winkel 0, m ([unten](#ein-portal-das-giert-und-werkzeuge-auf-drehachsen)) |
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
| `screw` | `gearbox`, `axis` | Pflicht | ein Kugelgewindetrieb am Abtrieb dieses Getriebes, dessen Mutter diese Achse treibt ([unten](#spindeln-zahnstangen-linearmotoren-und-rundtische)) |
| | `values` | Pflicht | die Tabelle seiner Werte, aufgebaut wie `[screw]` der Kugelgewindeachse des Gelenkbaukastens |
| `rack` | `gearbox`, `axis` | Pflicht | ein Ritzel am Abtrieb dieses Getriebes, das diese Achse an einer Zahnstange treibt |
| | `values` | Pflicht | die Tabelle seiner Werte, aufgebaut wie `[rack]` der Zahnstangenachse des Gelenkbaukastens |
| `linear_motor` | `motor`, `axis` | Pflicht | dieser Motor ist ein Linearmotor, der diese Achse treibt; er hat keinen Getriebeknoten |
| | `values` | Pflicht | die Tabelle seiner Werte, aufgebaut wie `[linear_motor]` der Linearmotorachse des Gelenkbaukastens |
| `rotary` | `gearbox`, `axis` | Pflicht | ein Rundtisch am Abtrieb dieses Getriebes: die Achse dreht sich, in rad |
| | `inertia_kgm2` | Pflicht | die Trägheit, die er dreht, kg m² |
| | `payload_inertia_kgm2` | | die seiner Last, solange `effects.payload` an ist und der Greifer sie hält, kg m² |
| | `strain_wave` | | die Tabelle der Lebensdauerwerte seines Wellgetriebes, aufgebaut wie `[strain_wave]` |
| `pulley`, `idler` | `clockwise` | | mit `effects.belt_wrap`: der Riemen läuft im Uhrzeigersinn um sie, von vorn gesehen (x nach rechts, z nach oben) und den Routen in ihrer Reihenfolge nach; Vorgabe false; `at_m` ist dann ihre Lage in der Ebene des Riemens (x, z) |
| `worm` | `gearbox` | Pflicht | dieses Getriebe ist eine Schneckenstufe ([unten](#schneckengetriebe)); eine Schnecke je Getriebe |
| | `ratio` | Pflicht | ihre Übersetzung, anstelle der des Getriebes |
| | `efficiency`, `back_efficiency` | Pflicht | angetrieben von der Schnecke (über 0, höchstens 1) und rückwärts vom Rad (höchstens 1; 0 oder weniger sperrt selbst) |
| `lean` | `body`, `axis` | Pflicht | ein Körper, der sich an seinem Wagen entlang einer waagrechten Achse neigt, mit der er fährt (`effects.tool_flex`, elastisches Modell); höchstens einer ([unten](#die-umschlingung-des-riemens-und-eine-sich-neigende-achse)) |
| | `com_drop_m`, `tool_drop_m` | Pflicht | sein Schwerpunkt und die Werkzeugspitze unter der Führung bei `lever_axis` auf 0, m |
| | `lever_axis` | | eine Achse, deren Weg beide zur Führung hebt |
| | `lateral_stiffness_n_m`, `tilt_stiffness_nm_rad`, `damping_n_s_m` | Pflicht | Steifigkeit der Führung seitlich und gegen Kippen, die Dämpfung der Neigung |
| `gantry` | `axis` | Pflicht | eine Brücke, die entlang dieser waagrechten Linearachse fährt, giert (`effects.gantry_yaw`); höchstens eine ([unten](#ein-portal-das-giert-und-werkzeuge-auf-drehachsen)) |
| | `across` | | die Linearachse entlang der Brücke; mit ihr ändert sich der Hebel ihrer Körper um die Mitte der Brücke |
| | `width_m`, `inertia_kgm2` | Pflicht | Abstand der beiden Seiten der Brücke, m; ihre eigene Trägheit um eine senkrechte Achse durch ihre Mitte, kg m² |
| | `stiffness_nm_rad`, `damping_nm_s_rad` | Pflicht | was das Gieren neben Antrieben und Riemen hält (ihre Anbindung und Führungen), und seine Dämpfung |
| | `across_start_m` | | wo `across` auf 0 steht, von der ersten Seite, m; Vorgabe ihr Hub in der Mitte |
| | `drives` | | je Motor, dessen Spindel, Zahnstange oder Linearmotor `axis` treibt: seine Lage quer über die Brücke von der Mitte, m; zwei ergeben ein Portal mit Doppelantrieb |
| | `sections` | | je Riemenabschnitt, der an den Enden der Brücke zieht: seine Länge je rad Gieren, m |
| | `tolerance_m` | | der Gleichlauffehler (Breite mal Gierwinkel), den die Prüfung `gantry-sync` zulässt, m |
| `tcp` | `axes` | Pflicht | der Werkzeugmittelpunkt: die drei Linearachsen entlang x (die erste der Datei), y (waagrecht) und z (senkrecht); höchstens einer |
| | `rotaries` | Pflicht | die Drehachsen, die das Werkzeug tragen, die äußerste zuerst |
| | `tool_m` | Pflicht | die Werkzeugspitze von dort, wo sich ihre Achsen treffen, `[x, y, z]` bei Winkel 0, m |

Es gibt so viele Motoren wie Achsen und einen mehr für jeden Antrieb eines Portals über den ersten hinaus. Eine Datei nennt jede Liste, die sie verwendet; `axis` und `motor` immer,
`gearbox`, `pulley` und `section`, wo Riemen getrieben werden, `body`, wo eine Achse einen bewegt; die anderen dürfen
fehlen. Ein Effekt, der Knoten braucht, die die Datei nicht hat (`frame_compliance` ohne `frame`,
`belt_span_vibration` ohne `span`, `tool_flex` ohne `lean`), lehnt die Datei mit `machine.<Liste>` ab; `belt_wrap` ohne
die Lagen, aus denen er gerechnet wird, wird mit dem abgelehnt, was fehlt.

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
der Antriebe, der Rauschstartwert, ein gemeinsamer Zwischenkreis und das Bremsmodul. `sim-cli search` sucht den
gemeinsamen Motor; ein Motor mit eigenem `[parts.<id>]` behält sein Teil bei der Suche.
`sim-cli search --machine-file DATEI --motor-id ID` sucht einen Motor allein: jeder Kandidat (mit seinem gepaarten
Antrieb, und mit `--gearboxes` den passenden Getrieben für die eigene Übersetzung dieses Motors) wird `[parts.<ID>]`,
die anderen Motoren bleiben, wie die Datei sie hat, die Drehzahlverstärkung `drive.<ID>.velocity_kp_nm_s_rad` dieses
Motors wird dafür nachgestellt, wie die gemeinsame für ein gemeinsames Teil (nach der Trägheit, die dieser Motor dreht,
und der seines Läufers), und die Kosten sind das Gewicht dieses einen Motors. Beim XYZ-Portal legt `--motor-id mz` den
Z-Motor hinter seinem Getriebe mit Übersetzung 10 aus, X und Y behalten ihre. Der zeitoptimale Bahnplaner
nimmt für jeden Motor die Kennlinie des gemeinsamen `[drive]`.

## Spindeln, Zahnstangen, Linearmotoren und Rundtische

Ein Motor kann eine Achse ohne Riemen treiben: über einen Knoten `screw`, `rack` oder `rotary` am Abtrieb seines
Getriebes (ein Getriebe der Übersetzung 1 ist die Kupplung), oder als `linear_motor`, der der Motor selbst ist und
keinen Getriebeknoten hat. Jeder treibt eine Achse, eine Achse hat höchstens einen, und Riemen und diese mischen sich in
einer Datei:

```toml
[[machine.screw]]          # machines/screw-portal.toml: X und Y an Riemen, Z an einem Kugelgewindetrieb 16 x 10
id = "screw_z"
gearbox = "gearbox_z"      # die Kupplung: [gearbox.mz] hat ratio = 1
axis = "z"
values = "screw_z"         # die Tabelle [screw_z], aufgebaut wie [screw] der Kugelgewindeachse des Gelenkbaukastens
```

Die Werte sind die des Gelenkbaukastens ([Mechanismen](mechanismen.md)), in einer Tabelle, die der Knoten nennt, so dass
zwei Spindeln zwei Tabellen haben: der Aufbau von `[screw]` für eine Spindel (Steigung, Nenn- und Kerndurchmesser,
Länge zwischen den Lagern, Lagerung, Wirkungsgrad, Steifigkeit von Mutter und Lagern, Vorspannung, Tragzahlen und
Faktoren, `max_dn`), der von `[rack]` für eine Zahnstange (Modul, Zähne, Schrägungswinkel, Wirkungsgrad, Steifigkeit,
Spiel, Ritzelträgheit, zulässige Vorschubkraft, Betriebsfaktor), der von `[linear_motor]` für einen Linearmotor
(Anziehungskraft, Polteilung, Reibbeiwert der Führung), der von `[strain_wave]` für das Wellgetriebe eines Rundtischs
(Nennmoment, Antriebsdrehzahl und Lebensdauer). Der Antriebsstrang wird gebaut wie im Gelenkbaukasten
(`transmission::Converter` hinter dem Getriebe): Steigung / 2 pi der Spindel oder der Radius des Ritzels als Hebel,
seine Steifigkeit in Reihe mit der des Getriebes, sein Wirkungsgrad auf dem des Getriebes, die Trägheit der Spindel
oder des Ritzels auf dem Läufer. Ein Spindelmotor braucht also T = J a / Hebel + F Hebel / eta, um eine Last F mit der
Beschleunigung a zu heben (J Läufer, Kupplung und Spindel), und der Motor einer Zahnstange dreht sich um i x / r.

- Mit `effects.guide_friction` bremst die Vorspannung der Mutter am Motor, und die Anziehung eines Linearmotors fügt
  den Reibbeiwert der Führung mal sich selbst hinzu; Anziehung und Gewicht des Primärteils drücken außerdem auf die
  Führung der Achse (`axis.<id>.guide_load`, von den Führungsprüfungen beurteilt).
- `[motor.<id>]` und `[drive.<id>]` eines Linearmotors lesen sich in N für N m und m für rad (Kraftkonstante in N/A,
  die Masse des Primärteils als `rotor_inertia_kgm2`); sein `[gearbox.<id>]` hat die Übersetzung 1 und ist die
  Anbindung des Primärteils.
- Ein Rundtisch dreht seine Achse in rad: die Achse hat `stroke_rad` statt `stroke_m`, ihre Grenzen sind
  `max_velocity_rad_s`, `max_acceleration_rad_s2` und `max_jerk_rad_s3` in `[axis.<id>]`, und was sie bewegt, ist
  `inertia_kgm2` (und `payload_inertia_kgm2`) des Rundtischs und die Körper, die auf ihr mitfahren
  ([unten](#ein-portal-das-giert-und-werkzeuge-auf-drehachsen)). Ihr `incline_deg` neigt die Achse, um die sie dreht;
  sie hat keine Führung und keine Energiekette.
- Signale und Prüfungen heißen nach dem Motor, wie die des Gelenkbaukastens nach seinem Gelenk: `joint.<Motor>.force`
  (was Spindel, Zahnstange, Linearmotor oder Rundtisch an die Achse gibt), `screw.<Motor>.speed`, `.dn` und
  `.damage_rate`, `rack.<Motor>.feed_force`, `strain_wave.<Motor>.damage_rate`; `gearbox.<Motor>.output_torque` ist das
  Moment vor Spindel oder Zahnstange. Die Prüfungen sind die des Baukastens: `screw-speed`, `screw-dn`,
  `screw-buckling`, `screw-static`, `screw-life`, `rack-feed-force`, `strain-wave-life` (die Lebensdauern, wo
  `ratings.required_life_h` gesetzt ist). Ein Motor, der keine Scheibe dreht, meldet kein
  `pulley.<Motor>.radial_load`, also übergehen ihn die Prüfungen der Riemenzähne und Scheibenlager.
- `effects.belt_stretch` schaltet das elastische Modell der ganzen Maschine: damit verdrehen sich auch die Getriebe der
  Spindeln, Zahnstangen und Rundtische (die Steifigkeit des Wandlers in Reihe), und die Eigenform des Riemens in einer
  Lage enthält sie, bei festgehaltenen Läufern.

`machines/screw-portal.toml` ist das XYZ-Portal, dessen Z-Riemen durch einen Kugelgewindetrieb 16 x 10 hinter einer
Balgkupplung ersetzt ist; die Bremse des Z-Motors hält den Schlitten (0,055 N m von ihren 0,5).
`machines/rack-gantry.toml` ist ein Zweiachsportal mit einem 3 m langen X an Zahnstange und Ritzel (Modul 2, 200 mm je
Umdrehung) hinter einem Getriebe der Übersetzung 10 und Y an einem Riemen. Beide bestehen jede Prüfung auf ihren
Zyklen (`screw-speed` 2402 von zulässigen 9833 U/min, `rack-feed-force` 180 von 2400 N). Die Kugelgewinde-,
Zahnstangen-, Linearmotor- und Rundtischachse des Gelenkbaukastens als Dateien geschrieben geben die Urteile des
Baukastens Ziffer für Ziffer.

## Ein Koppelgetriebe: der ebene Delta

Ein `linkage`-Knoten ist ein ebenes Parallelkoppelgetriebe: zwei Arme in der senkrechten x-z-Ebene, je einer an jeder
Seite der Basis, jeder von einem Motor über sein Getriebe gedreht, jeder mit einem Unterarmpaar hinab zu einer Platte,
die ihre Lage hält (ein Fünfgelenk, dessen untere Glieder Parallelogramme sind). Das Werkzeug ist die Mitte der
Platte, x quer und z nach oben, von der Achse der Schultern aus. `machines/planar-delta.toml` ist der registrierte
ebene Delta (`delta-d2`) so geschrieben, mit unveränderten Werten:

```toml
[[machine.axis]]           # ohne Hub: der Arbeitsraum des Koppelgetriebes gibt den Bereich
id = "x"

[[machine.axis]]
id = "z"

[[machine.motor]]          # und "2"; key = "motor-arm-1" benennt ihn wie das Crate
id = "1"

[[machine.gearbox]]        # und gearbox_2 an Motor 2
id = "gearbox_1"
motor = "1"

[[machine.linkage]]
id = "delta"
arms = ["gearbox_1", "gearbox_2"]   # der Arm bei +x, dann der bei -x
axes = ["x", "z"]                    # quer, dann nach oben
base_radius_m = "geometry.base_radius_m"
upper_arm_m = "geometry.upper_arm_m"
forearm_m = "geometry.forearm_m"
upper_arm_kg = "mass.upper_arm_kg"
# ... die übrigen Längen, Massen und der Arbeitsraum, siehe die Datei
```

Seine Felder: die Geometrie (`base_radius_m`, `platform_radius_m`, `upper_arm_m`, `forearm_m` länger als der Oberarm,
`forearm_spacing_m` für die 3D-Ansicht), die Massen (`upper_arm_kg` mit `upper_arm_cg_m` und `upper_arm_inertia_kgm2`
um die Schulter, `elbow_kg`, `forearm_pair_kg` mit dem am Ellbogen gezählten Anteil `forearm_elbow_share`,
`platform_kg`, `payload_kg`) und der Arbeitsraum (`half_width_m`, `top_z_m`, `height_edge_m`, `height_centre_m`: ein
Band unter der Oberkante, dessen Boden ein Bogen durch beide Ränder und die Mitte ist) mit der höchsten Konditionszahl
der Jacobi-Matrix, die ein Ziel haben darf, `max_condition_number`. Jedes ist eine Zahl oder der Pfad einer Zahl; die
Datei des ebenen Delta nennt `[geometry]`, `[mass]` und `[workspace]` des Crates.

Das Modell ist das des Crates (Modellreferenz, englisch): inverse Kinematik mit Ellbogen außen, direkte Kinematik als
unterer Schnittpunkt zweier Kreise von Unterarmlänge um die Ellbogen, die Jacobi-Matrix J = J_x^-1 J_q der
Beinbedingungen, und die Dynamik zweier Arme um ihre Gelenke mit einer Punktmasse an der Platte (die Masse jedes
Unterarmpaars geteilt, der Anteil r am Ellbogen): M(theta) theta'' = tau_gear + Q_g - m_nt J^T J_x^-1 w + tau_joint mit
M = I_b 1 + m_nt J^T J. Das Gelenk jedes Arms ist ein `RotaryJoint`, elastisch oder starr (`effects.joint_elasticity`),
sein Antriebsstrang aus der eigenen Konfiguration seines Motors ([oben](#motoren-die-sich-unterscheiden)), seine
Lagerreibung und Anschläge aus `[joint]`. Ein Ziel außerhalb des Arbeitsraums, außer Reichweite, jenseits eines
Gelenkanschlags oder über der Konditionszahl wird mit den Gründen des Crates abgelehnt, und die Prüfung `singularity`
misst den Zyklus an `ratings.max_condition_number`. Die Signale sind die des Crates: `arm.<motor>.angle`,
`gearbox.<motor>.output_torque`, `kinematics.condition`, die Momentaufteilung, Verlustleistungen und Energien; die
Getriebe erwärmen sich unter den Ids der Getriebeknoten.

Eine Datei mit Koppelgetriebe hat zwei Achsen (höchstens mit `key`: ohne Hub, Neigung oder Energiekette), zwei
Motoren, an jedem ein Getriebe, und das Koppelgetriebe; eine Scheibe, ein Abschnitt, Körper, eine Führung, Umlenkrolle,
ein Rahmen, Trum, eine Spindel, Zahnstange, ein Linearmotor, Rundtisch, eine Schnecke oder Neigung (`lean`) daneben
wird abgelehnt, ebenso ein zweites Koppelgetriebe. Die Effekte sind die des Crates (`[effects]` wie in seiner
Konfiguration); die des Riemens werden nicht gelesen.

## Schneckengetriebe

Ein `worm`-Knoten macht ein Getriebe zu einer Schneckenstufe: seine `ratio` ersetzt die des Getriebes, und mit
`effects.gearbox_efficiency` verliert es nach seinen eigenen Wirkungsgraden; Steifigkeit, Lose und Trägheit des
Getriebes bleiben:

```toml
[[machine.worm]]
id = "worm_c"
gearbox = "gear_c"         # das Getriebe vor einem Rundtisch etwa
ratio = 40.0
efficiency = 0.76          # von der Schnecke angetrieben: tan(gamma) / tan(gamma + rho')
back_efficiency = 0.5      # vom Rad zurück angetrieben: tan(gamma - rho') / tan(gamma); 0 oder weniger sperrt selbst
```

Solange der Motor treibt, gibt sie das Moment t ihrer Feder als t eta an den Abtrieb und t / i an den Läufer weiter, wie
ein Getriebe (`mech::Gearbox`, der Wirkungsgrad über 0,5 W eingeblendet). Von der Last zurück angetrieben gibt sie
t / eta_back weiter: bei einem Rückwirkungsgrad von 0,5 nimmt der Läufer die halbe Leistung der Last auf. Ein
Rückwirkungsgrad von 0 oder weniger sperrt sie: zurück angetrieben gibt sie t an den Abtrieb und nichts an den Läufer,
der Abtrieb steht also, wo der Läufer ihn hält, und die Leistung der Last geht in die Reibung der Schnecke; in Ruhe hält
sie jede Last ohne Motormoment (ihr Haltemoment ist 0). Mit demselben Wirkungsgrad in beide Richtungen ist sie der
Rundtisch des Gelenkbaukastens mit `table.gear = "worm"`. Ihre Wirkungsgrade folgen nicht ihrer Temperatur, und unter 0
zählen sie als 0 (siehe [was Maschinendateien noch nicht können](#was-maschinendateien-noch-nicht-können)).

## Die Umschlingung des Riemens und eine sich neigende Achse

**Die Umschlingung des Riemens** (`effects.belt_wrap`). Mit ihr sind die eingreifenden Zähne an jeder getriebenen
Scheibe Umschlingung r / p (p die Zahnteilung) anstelle von `ratings.belt_teeth_in_mesh`, im Gesetz des
Zahnüberspringens und in der Zahnlast des Riemens. Die Umschlingung wird in der Lage aus den Orten der Räder in der
Ebene des Riemens gerechnet (x und z der 3D-Ansicht): `at_m` der Scheibe und die Räder vor und nach ihr entlang des
Riemens, der vorletzte Punkt der Route des Abschnitts, der an der Scheibe endet, und der zweite der Route des
Abschnitts, der dort beginnt (eine Umlenkrolle mit ihrem `at_m`, von ihrem Körper aus, wo sie auf einem mitfährt; ein
Körper, eine Klemme an ihm, wo die 3D-Ansicht ihn zeichnet; eine andere Scheibe). `clockwise` jedes Rads sagt, in
welcher Richtung der Riemen um es läuft, den Routen nach; die geraden Trume des Riemens sind die passenden Tangenten,
und die Umschlingung ist, wie weit der Riemen zwischen ihnen um die Scheibe dreht. Für die Anordnungen der
mitgelieferten Crates ist das `mech::wrap_angle`, eine halbe Umdrehung weniger die Neigung beider Trume. Abgelehnt wird
eine Datei, wo ein Motor keine Scheibe treibt, eine Scheibe nicht eine Route beendet und eine andere beginnt, oder sie
oder ein Rad neben ihr kein `at_m` hat.

**Eine sich neigende Achse** (`effects.tool_flex`, elastisches Modell): ein `lean`-Knoten ist ein Körper, der sich an
seinem Wagen entlang einer waagrechten Achse neigt, wie die Z-Achse des T-Portals an ihrem X-Wagen. Sein Schwerpunkt
weicht gegen die Führung um w zur Seite aus; die Führung gibt seitlich und im Kippen nach: mit h = `com_drop_m` weniger
dem Weg der `lever_axis` ist die Steifigkeit am Schwerpunkt k = 1 / (1 / seitlich + h² / Kippen), und die
Werkzeugspitze, h_tool darunter, bewegt sich um k (1 / seitlich + h h_tool / Kippen) je Meter w. Der Körper fährt mit
der Geschwindigkeit seiner Achse plus w' über Grund, seine Masse koppelt also die Achsen und w; die Feder wirkt innen,
zwischen Wagen und Körper, die Prozesskraft an der Werkzeugspitze, und wenn die Hebelachse fährt, drückt die steifere
Feder sie mit -w² / 2 dk/dz zurück. Das Werkzeug sitzt an der Spitze (`tool_state`), die Dämpfung der Neigung ist
`power.tool_flex`, der Weg der Spitze `axis.<Achse>.tool_deflection`, und die Eigenform des Riemens schließt sie ein.

```toml
[[machine.lean]]           # machines/t-gantry.toml
id = "z_lean"
body = "z_axis"
axis = "x"
com_drop_m = "geometry.z_com_drop_m"
tool_drop_m = "geometry.z_tool_drop_m"
lever_axis = "z"
lateral_stiffness_n_m = "tool_flex.lateral_stiffness_n_m"
tilt_stiffness_nm_rad = "tool_flex.tilt_stiffness_nm_rad"
damping_n_s_m = "tool_flex.damping_n_s_m"
```

Die Datei des T-Portals hat beides, ausgeliefert aus (das Crate hat seine Neigung an): ihre Scheiben an den Enden von X
auf Höhe des Wagens und ihre Wagenrollen r + r_Rolle darüber und darunter, sodass der Riemen beide Scheiben waagrecht
verlässt, eine halbe Umdrehung in jeder Lage, wie im Crate; mit einem der beiden Effekte an ist sie ihr Crate auf 1e-9.

## Ein Portal, das giert, und Werkzeuge auf Drehachsen

**Eine Brücke, die giert** (`machine.gantry`, `effects.gantry_yaw`). Die Brücke eines Portals fährt entlang einer
waagrechten Achse und giert dabei ein wenig um die Senkrechte: ihre beiden Seiten, `width_m` voneinander, laufen um die
Breite mal den Gierwinkel psi auseinander (kleine Winkel). psi ist eine Koordinate mehr neben den Achsen. Was es hält:
die Anbindung der Brücke an ihre Wagen und deren Führungen, eine Feder `stiffness_nm_rad` mit `damping_nm_s_rad`;
Spindeln, Zahnstangen oder Linearmotoren an ihren Seiten (`drives`, jede mit ihrer Lage quer über die Brücke von der
Mitte, so dass ihr Abtrieb x + Seite psi fährt); und Riemenabschnitte, die an ihren Enden ziehen (`sections`, m Länge
je rad: der Zug des Abschnitts entlang der Achse mal die Lage des Endes quer dazu). Was es dreht: ihre eigene
`inertia_kgm2` und die Körper, die mit der Achse und entlang der Brücke (`across`) fahren. Ein solcher Körper sitzt
e = q_across + Versatz von der Mitte und fährt mit dem Gieren um e psi entlang der Achse; seine Masse koppelt also Achse
und Gieren (m e) und legt m e² auf die Trägheit des Gierens, über die Jacobimatrix seiner Lage (unten); das Werkzeug auf
ihm fährt mit. Signale `gantry.<id>.yaw` (rad), `.sync_error` (Breite mal Gierwinkel, m: wie weit die Seiten
auseinander sind) und `.moment` (k psi + c psi', N m, was die Anbindung trägt); Verlust `power.gantry_<id>`; Prüfung
`gantry-sync`, der größte |Gleichlauffehler| gegen `tolerance_m`. `ConfigMachine::racking_moment` gibt das Giermoment
bei einer Stellung und Beschleunigung im verlustfreien starren Modell: was Riemen, Antriebe und Schwerkraft auf das
Gieren legen, über das hinaus, was die Körper bei rechtwinklig gehaltener Brücke dreht. Der Effekt ist in den
mitgelieferten Dateien von H-Bot, CoreXY und Zahnstangenportal aus, damit ihre Ergebnisse bleiben, und in der des
Portals mit Doppelantrieb an.

```toml
[[machine.gantry]]         # machines/dual-gantry.toml
id = "bridge_yaw"
axis = "x"
across = "y"
width_m = "gantry.width_m"
inertia_kgm2 = "gantry.inertia_kgm2"
stiffness_nm_rad = "gantry.stiffness_nm_rad"
damping_nm_s_rad = "gantry.damping_nm_s_rad"
drives = { mx1 = -0.6, mx2 = 0.6 }
tolerance_m = "gantry.sync_tolerance_m"
```

- **Zwei Antriebe.** Zwei Zahnstangen auf der Achse des Portals, eine an jedem Ende: die Datei hat einen Motor mehr
  als Achsen, und die beiden legen x und psi zusammen fest. Jeder Antrieb folgt dem Sollwert der Achse an seinem
  eigenen Geber (Sollwertkopplung), so halten die beiden Lageregler die Brücke rechtwinklig, und ihre Momente teilen die
  Last nach dem Hebelgesetz: F2 - F1 = 2 m e a / B für einen Wagen m bei e, mit a beschleunigt. Mit dem Effekt aus
  halten die Führungen die Brücke rechtwinklig, und beide teilen gleich. `machines/dual-gantry.toml` hat ein X von 3 m
  auf zwei Zahnstangen mit Katalogmotoren (1FK2103-4AH an S210) und ein Y von 0,8 m auf einem Riemen auf einer Brücke
  von 1,2 m. Es besteht jedes Urteil seines Zyklus: `gantry-sync` 16 µm von 100, und das Ende, nahe dem der Wagen
  anfängt, trägt mehr (`rack-feed-force` 109 gegen 95 N). Sein Gieren schwingt mit sqrt((k + 2 k_Ende (B / 2)²) / J) / 2 pi,
  155,0 Hz mit J auf 20 kg m² angehoben und dem Wagen in der Mitte, 149,5 Hz mit ihm an einem Ende (`sim-cli modes`).
- **Ein Antrieb.** Die eine Zahnstange des Zahnstangenportals treibt seine Brücke an einem Ende
  (`drives = { mx = -0.5 }`): das ferne Ende hält nur die Feder der Anbindung, und es bleibt zurück, solange die Brücke
  beschleunigt; mit dem Effekt an wächst der Bahnfehler seines Zyklus von 0,07 auf 0,20 mm.
- **H-Bot und CoreXY.** Riemen ziehen an den Enden der Brücke. Beim H-Bot (`sections = { "1" = -0.3, "2" = 0.6, "3" = -0.3 }`)
  ist das Moment des Riemens auf die Brücke -0,3 m mal sein Zug am Kopf entlang der Brücke: sie giert, sobald der Kopf
  entlang der Brücke geschoben wird, wo er auch steht (-4,2 N m für den Kopf von 7 kg bei 2 m/s²), und ein Kopf
  außerhalb der Mitte lässt sie gieren, wenn die Brücke beschleunigt (-m e a). Ein starrer Riemen lässt den H-Bot gieren
  (der Kopf kann bei festgehaltenen Motoren entlang der Brücke fahren), er giert also in beiden Modellen; mit dem Effekt
  an wächst der Bahnfehler seines Zyklus von 0,09 auf 0,80 mm. Beim CoreXY zieht jeder Riemen an beiden Enden: sein
  Moment ist die Summe der Zugkräfte des einen Riemens gegen die des anderen, -0,19 N m für den Zug von 8 N, der einen
  H-Bot mit -2,4 gieren ließe, und sein Bahnfehler bleibt 0,06 mm. Ein starrer Riemen hält das Gieren eines CoreXY: sein
  starres Modell hat keins.

**Körper auf Drehachsen und der Werkzeugmittelpunkt.** Ein Körper darf mit Drehachsen fahren: `offset_m` ist sein
Schwerpunkt von dort, wo sich ihre Achsen bei Winkel 0 treffen, und er fährt auf ihnen herum, die erste in seinen
`axes` ganz außen. Das `incline_deg` einer Drehachse neigt die Achse, um die sie dreht, aus der Senkrechten (0, eine
C-Achse) nach x (90, eine A-Achse). Die Lage p(q) eines solchen Körpers ist der Weg der Linearachsen plus sein Versatz,
von seinen Drehachsen gedreht; sein Teil der Massenmatrix ist m J^T J (J = dp/dq, über zentrale Differenzen), der
Schwerkraft -m g J_z, und der Kräfte, die seine Geschwindigkeit verlangt, -m J^T (dJ/dt q'), die Zentripetal- und
Corioliskräfte: die Gleichungen von Lagrange, die Energie bleibt also erhalten. Ein Knoten `tcp` macht die Koordinaten
des Werkzeugs zu denen der Spitze: drei Linearachsen entlang x (die erste der Datei), y und z, die `rotaries`, die das
Werkzeug tragen, und `tool_m`, die Spitze von dort, wo sich ihre Achsen treffen. Die Achsen stehen bei der Spitze
weniger `tool_m`, von den Drehachsen gedreht: eine Bahn führt die Spitze gerade, während die Drehachsen drehen und die
Linearachsen das ausgleichen, und die Vorsteuerung hat den Anteil der Geschwindigkeit (`feedforward_torque_moving`).
Die Bereiche des Werkzeugs sind die Hübe, um |`tool_m`| erweitert, und eine Stellung, die die Linearachsen nicht
erreichen, wird abgelehnt (`error-tcp-joint-range`). Die 3D-Ansicht zeichnet das Werkzeug von den Drehachsen bis zur
Spitze und die Bahn des Werkzeugs an der Spitze.

```toml
[[machine.tcp]]            # machines/five-axis.toml
id = "tool_tip"
axes = ["x", "y", "z"]
rotaries = ["c", "a"]      # C um die Senkrechte trägt A um x
tool_m = [0.0, 0.0, -0.2]
```

`machines/portal-c.toml` ist das Spindelportal mit einer C-Achse auf seinem Z-Schlitten, die einen Greifer dreht, dessen
Spitze 0,1 m neben der Achse liegt: sein Zyklus trägt ein Teil, während C es um eine Vierteldrehung dreht, die Spitze
auf einer Geraden. `machines/five-axis.toml` ist das Spindelportal mit einem Gabelkopf, C um die Senkrechte und darin A
um x, die Spitze 0,2 m unter ihrem Schnittpunkt: sein Zyklus neigt A und schwenkt C um eine halbe Drehung um die
stehende Spitze, X und Y fahren dabei einen Kreis, und das Gewicht der Spindel liegt auf A. Beide bestehen jedes Urteil
ihres Zyklus. Getriebe und Trägheiten ihrer Drehachsen und die Massen der Köpfe sind Schätzwerte.

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
- die Maschine: Massen, Radien und Längen positiv, ein Getriebe je Motor (keins bei einem Linearmotor) und eine
  Scheibe, Spindel, Zahnstange oder ein Rundtisch je Getriebe, höchstens eins davon je Achse, eine Spindel mindestens so
  lang wie ihr Hub, so viele Motoren wie Achsen und einen mehr je Antrieb eines Portals über den ersten hinaus, die
  Antriebe eines Portals auf seiner Achse, die Achsen eines Werkzeugmittelpunkts entlang x, y und z, das `offset_m`
  eines Körpers nur auf Drehachsen, eine Riemenführung, die die Riemenlänge hält und jede Achse festlegt;
- jeder andere Schlüssel gegen die Schlüssel der Konfigurationen der eingetragenen Maschinen (so wird `pretensoin_n`
  beim Namen abgelehnt), dazu `[axis.<id>]` je Achse, eine Reibtabelle je Führung, die Wertetabellen von Spindeln,
  Zahnstangen, Linearmotoren und Wellgetrieben im Aufbau des Gelenkbaukastens und jeder Schlüssel, den die Netzliste
  über einen Pfad nennt;
- dann alles, was die Simulation bei ihrem Start liest, wie bei einer Konfiguration: der Antrieb, der Zyklus, die
  Startlage, die Katalogteile, die die Datei nennt.

Ein Fehler nennt Datei, Zeile und Schlüssel, in der Sprache der Oberfläche:

```
$ sim-cli verdicts --machine-file broken.toml --lang de
Die Konfiguration enthält einen Wert, den diese Maschine nicht kennt: broken.toml:674: machine.guide[guide_z].side_lod
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
  zueinander, die Massenmatrix der Körper ist also diagonal, bis auf die Körper auf Drehachsen oder quer auf einer
  gierenden Brücke, die m J^T J, ihren Teil der Schwerkraft und die Kräfte ihrer Geschwindigkeit hinzufügen
  ([oben](#ein-portal-das-giert-und-werkzeuge-auf-drehachsen)). Scheiben und Läufer (J_Scheibe + i² J_Läufer je Motor, die
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
- Spindel, Zahnstange, Linearmotor oder Rundtisch (`machine.screw`, ...): das Getriebe seines Motors wird eines in der
  Einheit der Achse, der Übersetzung i / Hebel (der `RotaryJoint` des Gelenkbaukastens mit dem
  `transmission::Converter` dahinter), und seine "Scheibe" ist die Achse selbst: K hat für ihn die Zeile e_i, die Achse
  nimmt den Abtrieb des Getriebes als ihre Kraft, die Gleichungen des Riemens überlassen ihm die Achse (in Ruhe und bei
  den starren Spannungen), und im elastischen Modell folgen die Zustände der Scheibe der Achse. Die Masse einer
  Drehachse ist die Trägheit ihres Rundtischs und dessen, was auf ihr mitfährt. Ein Antrieb an der Seite einer
  gierenden Brücke hat die Zeile e_i + Seite e_psi.
- Das Gieren eines Portals (`machine.gantry`, `effects.gantry_yaw`): eine Koordinate psi mehr nach den Achsen, mit
  Feder und Dämpfer, die Gierbeiwerte der Riemenabschnitte in C, die Seiten der Antriebe in K. Wo weniger Motoren als
  Koordinaten es festlegen (ein Antrieb, oder Riemen), legen die Motoren die Achsen fest und die Feder das Gieren
  (`forward` und die Vorsteuerung nehmen die Zeilen der Achsen); wo Riemen ihre Länge damit nicht halten können (ein
  CoreXY), hat das starre Modell kein Gieren.
- Ein Werkzeugmittelpunkt (`machine.tcp`): die Stellung des Werkzeugs geht über die Drehung von `tool_m` durch die
  Drehachsen in die Achsen über und zurück; Geschwindigkeiten und Beschleunigungen durch Ableiten dieser Abbildung.
- Eine Schneckenstufe (`machine.worm`): der Abtrieb ihres Getriebes bekommt t eta^x und ihr Läufer t / i,
  x = tanh(t w / 0,5 W), mit dem Rückwirkungsgrad, wo die Last treibt (x unter 0); eine selbsthemmende gibt dem Läufer
  treibend x t / i und zurück angetrieben nichts. Im starren Modell werden mit einer selbsthemmenden Schnecke die
  Abtriebe der Getriebe neben den Beschleunigungen gelöst, damit ein Läufer, der nichts aufnimmt, lösbar bleibt
  (`ConfigMachine::rigid_locking`).
- Ein sich neigender Körper (`machine.lean`, elastisches Modell): eine Koordinate w mehr neben den Achsen, die Masse des
  Körpers koppelt sie an seine Achse, [[M, m e_a], [m e_a^T, m]] [a; w''] = [F; tip p_a - k w - c w'], k und tip aus
  `geometry::flex_at`.
- Die Umschlingung des Riemens (`effects.belt_wrap`): `geometry::wrap` an jeder Scheibe aus den Tangenten ihrer
  Nachbarn, in der Lage.

Regler, Antriebe, Zwischenkreis, Wicklungserwärmung, Analysen, Urteile, Sensitivität und Fehlerbänder sind die jeder
Maschine; an ihnen ist nichts besonders für Maschinendateien.

## Was Maschinendateien noch nicht können

- `belt_wrap` neben Spindel, Zahnstange, Linearmotor oder Rundtisch: die Umschlingungen werden je Motor gemeldet, und
  ein solcher Motor dreht keine Scheibe; die Datei wird abgelehnt. Eine Scheibe steht, wo ihr `at_m` sie hinstellt (sie
  fährt auf keinem Körper mit), und die Lagen der Umschlingung sind die der 3D-Ansicht: die Abschnittslängen werden
  nicht gegen sie geprüft.
- `tool_flex`: ein sich neigender Körper, entlang einer waagrechten Achse; er neigt sich nur im elastischen Modell (wie
  im Crate).
- Eine selbsthemmende Schnecke (Rückwirkungsgrad 0 oder weniger) gibt zurück angetrieben nichts an ihren Läufer, der
  Motor senkt eine Last durch sie also ohne Moment, wo er bei einem Rückwirkungsgrad unter 0 schieben müsste; und ihre
  Sperre blendet über die 0,5 W des Getriebes ein, statt zu haften, sodass ein Motormoment, das die Last nicht ganz hebt,
  sie langsam vorwärts kriechen lässt. Die Wirkungsgrade einer Schnecke folgen nicht ihrer Temperatur.
- Eine Scherenhubbühne: die Mechanismen neben Riemen sind Spindeln, Zahnstangen, Linearmotoren und Rundtische,
  jeder treibt eine Achse gerade, und das eine Koppelgetriebe ist das des ebenen Delta, das sich nicht mit ihnen
  mischt. Eine Scherenhubbühne ist kein Fall davon: ein Motor treibt eine senkrechte Achse über eine Spindel oder
  einen Zylinder am Fuß der Schere, z = n L sin phi; dazu bräuchte es eine Spindel, die ein Koppelgetriebe statt
  einer Achse treibt. Eine Drehachse hat keine eigene Lagerreibung (nur die ihres Motors und Getriebes).
- Das Gieren eines Portals: eines je Datei, kleine Winkel (die Seiten fahren um die Breite mal den Winkel); die Antriebe
  folgen jeder dem gemeinsamen Sollwert, es gibt also keinen Master/Slave- oder kreuzgekoppelten Gleichlauf in den
  Antrieben (das wäre Sache der Regelung), und der Gleichlauffehler ist, was zwei gleiche Lageregler übrig lassen. Die
  Gierbeiwerte eines Abschnitts sind Zahlen aus seiner Führung, nicht aus seiner `route` gerechnet; ein Körper auf der
  Brücke, der nicht quer fährt, sitzt in ihrer Mitte; Umlenkrollen und Trums folgen dem Gieren nicht.
- Ein Körper auf Drehachsen ist eine Punktmasse in seinem Schwerpunkt: seine eigene Trägheit um ihn steckt im
  `inertia_kgm2` des Rundtischs. Ein Werkzeugmittelpunkt braucht drei rechtwinklige Linearachsen (x, y, z der
  3D-Ansicht) und die Drehachsen auf der Seite des Werkzeugs (ein Kopf): ein Rundtisch, der das Werkstück trägt, ist
  noch keiner. Einschwingen und Bahnfehler der Auswertung zählen den Fehler einer Drehachse in rad zu dem der
  Linearachsen in m.
- `effects.belt_tooth_jump` zusammen mit Spindel, Zahnstange, Linearmotor oder Rundtisch: seine Zustände liegen je
  Motor, und ein Motor ohne Scheibe hat keine; die Datei wird abgelehnt.
- Die Steifigkeit einer Spindel ist die bei der weichsten Mutterstellung für den ganzen Hub, wie im Gelenkbaukasten;
  das Bremsmoment ihrer Vorspannung sitzt am Motor, mit einer Glättung in rad/s.
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
acht Trume und gibt die Zahlen des Crates auf 1e-9 (`the_effects_of_a_machine_file_are_its_crates`); seit dem
14. November auch mit der sich neigenden Z-Achse (`machine.lean`) und der Umschlingung des Riemens aus den Lagen
seiner Scheiben und Rollen, deren Umschlingungen ebenfalls übereinstimmen. Die Handrechnungen der Effekte selbst sind
die des Crates.

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

`crates/sim-machines/tests/machine_file_mechanisms.rs` hält die Spindeln, Zahnstangen, Linearmotoren und Rundtische:
die Kugelgewindeachse des Gelenkbaukastens als Datei ist ihr Crate auf 1e-9 (elastisch und starr, mit gehaltener,
abgesetzter Last und mit Bremse: Kinematik, Ruhelage, Ableitungen, Energie, Vorsteuerung, die Signale der Spindel und
die Momentaufteilung, ihre Prüfungen); von Hand das Moment des Z-Motors des Spindelportals für eine Last und
Beschleunigung und die kritische Drehzahl seiner Spindel, Motorwinkel x i / r und Vorschubkraft des
Zahnstangenportals, der Strom eines Linearmotors für eine Kraft und seine Führungslast, das Moment eines Rundtischs in
rad; beide neuen Dateien bestehen die Standardprüfung; und eine kaputte Spindel wird mit Schlüssel und Zeile abgelehnt.

`crates/sim-machines/tests/machine_file_linkage.rs` hält das Koppelgetriebe: `machines/planar-delta.toml` ist das Crate
des ebenen Delta auf 1e-9, in seiner Grundkonfiguration und mit jedem seiner 36 Effekte einzeln umgeschaltet (Achsen und
Motoren, Kinematik, Ruhelage und Haltemoment, die Ableitungen eines aus der Ruhe verschobenen Zustands, Energie, beide
Vorsteuerungen, jedes Signal, die Erwärmung der Teile, was der Arbeitsraum ablehnt); sein Zyklus, elastisch, starr und
mit allen standardmäßig ausgeschalteten Effekten eingeschaltet, gibt die Signale des Crates in jedem 16. Schritt der
ersten halben Sekunde und seine Motorlasten und Urteile, auf 1e-9 (beide summieren in derselben Reihenfolge und stimmen
bitgenau überein, Encoderstriche eingeschlossen); die Datei besteht die Standardprüfung mit den verlustfreien
Konfigurationen des Crates; von Hand die Armwinkel bei (0,1, -0,5) m (-asin 0,2 = -11,537 Grad und 16,474 Grad) und das
Haltemoment in der Mitte (-18,358 N m an jedem Getriebe); und ein kaputtes Koppelgetriebe wird mit Schlüssel und Zeile
abgelehnt.

`crates/sim-machines/tests/machine_file_geometry.rs` hält die Umschlingung des Riemens aus den Lagen der Nachbarn einer
Scheibe von Hand in zwei Lagen (eine versetzte Umlenkrolle und eine Klemme, die nicht waagrecht liegt), das
Schneckengetriebe gegen den Rundtisch des Gelenkbaukastens mit seiner Schnecke (starr und elastisch, treibend und
zurück angetrieben), das Halten einer selbsthemmenden Schnecke und die Beschleunigung einer zurück angetriebenen von
Hand, und die Katalogsuche für den Z-Motor des XYZ-Portals allein.

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
