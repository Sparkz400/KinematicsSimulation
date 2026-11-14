# Mechanismen und Übertragungen

Neben den Riemenmaschinen hat der Simulator Maschinen aus je einem motorgetriebenen Gelenk pro Achse: eine
Kugelgewindeachse, eine Zahnstangenachse, eine Linearmotorachse, einen Rundschalttisch, einen Sechsachsroboter und einen
SCARA-Roboter, beide aus URDF-Dateien gelesen. Ihre Teile sind Komponenten in `sim-components`, damit eine in einer
Konfigurationsdatei beschriebene Maschine sie später verschalten kann; die sechs Maschinen liegen im Crate
`machine-joint-kit` und werden wie jede andere Maschine gerechnet und ausgelegt.

Die englische Fassung dieses Dokuments ist [mechanisms.md](../mechanisms.md). Die Namen der Prüfungen, Signale und
Konfigurationsschlüssel sind englische Bezeichner und in beiden Sprachen gleich.

- [Die Maschinen](#die-maschinen)
- [Wie sie aufgebaut sind](#wie-sie-aufgebaut-sind)
- [Kugelgewindetrieb](#kugelgewindetrieb)
- [Zahnstange und Ritzel](#zahnstange-und-ritzel)
- [Linearmotor](#linearmotor)
- [Rundtisch: Wellgetriebe und Schneckengetriebe](#rundtisch-wellgetriebe-und-schneckengetriebe)
- [Roboter aus URDF-Dateien](#roboter-aus-urdf-dateien)
- [Was von Hand nachgerechnet ist](#was-von-hand-nachgerechnet-ist)
- [Grenzen](#grenzen)

## Die Maschinen

| Id | Mechanik | Übertragung | Werkzeuglage | Eigene Prüfungen |
| --- | --- | --- | --- | --- |
| `screw-axis` | Schlitten, 800 mm | Kugelgewindetrieb 25 x 20 hinter einer Kupplung | x (m) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` |
| `rack-axis` | Schlitten, 3 m | schrägverzahntes Ritzel auf einer Zahnstange hinter einem Planetengetriebe i = 10, spielfrei | x (m) | `rack-feed-force` |
| `linear-motor-axis` | Schlitten, 1 m | Eisenkern-Linearmotor, keine | x (m) | `guide-load`, `guide-life` aus der Anziehungskraft |
| `rotary-table` | Tisch mit 3 kg m^2 plus Werkstück | Wellgetriebe i = 100 (oder Schnecke, oder Planetengetriebe) | c (rad) | `strain-wave-life` |
| `urdf-arm` | Sechsachsroboter, `urdf/six-axis.urdf` | Wellgetriebe i = 120 und 80 | sechs Gelenkwinkel (rad), oder x, y, z, rx, ry, rz im Werkzeugraum | `strain-wave-life` je Gelenk |
| `scara` | SCARA-Roboter, `urdf/scara.urdf` | Wellgetriebe; die Pinole auf einem Kugelgewindetrieb | Gelenke (rad, m, rad), oder x, y, z, c im Werkzeugraum | `strain-wave-life`, die Prüfungen der Spindel an Gelenk 3 |

Jede bekommt außerdem die Prüfungen, die jede Maschine bekommt: Spitzen- und Effektivmoment des Motors (`motor-peak`,
`motor-thermal`), Antriebsströme, Spitzen- und kubisch gemitteltes Moment, Antriebsdrehzahl und Lebensdauer des
Getriebes, wo `[ratings]` sie angibt, Zwischenkreis und Bremswiderstand, Einschwingen. `sim-cli list` nennt sie, und
jeder Befehl nimmt sie:

```text
sim-cli verdicts screw-axis
sim-cli run rack-axis --set rack.service_factor=1.5
sim-cli margins linear-motor-axis
```

Jede Standardkonfiguration ist ein ausgelegtes Beispiel: Ihr Zyklus besteht alle Prüfungen (Test
`every_default_cycle_passes_its_verdicts`). Ihre Werte sind mit `[catalogue]` markiert, wo sie aus einem Herstellerdokument
stammen (Motor Siemens 1FK2105-4AF und Umrichter S210 wie beim Delta-Roboter), sonst mit `[estimate]`: eine Spindel,
Zahnstange, ein Linearmotor oder Wellgetriebe dieser Größe, wie die Kataloge der Hersteller sie führen, nicht der Artikel
eines Herstellers. Ersetzen Sie sie durch das Datenblatt Ihres Teils.

## Wie sie aufgebaut sind

Eine Gelenkmaschine (`chain::JointMachine`) ist ein Mechanismus plus ein `joint::RotaryJoint` je Gelenk. Der Mechanismus
liefert die Dynamik der Last in Gelenkkoordinaten, M(q) q'' + h(q, q') = tau, und seine eigenen Signale, Prüfungen und
seine Szene. Das Gelenk liefert alles zwischen Motor und Gelenk: Motorreibung, Momentwelligkeit und Rastmoment,
Haltebremse, das Getriebe (Übersetzung, Torsionssteifigkeit, Dämpfung, Spiel, Wirkungsgrad), Reibung im Gelenk und seine
Endanschläge. Mit `effects.joint_elasticity` sind die Motoren eigene Körper hinter den Federn der Getriebe; ohne, im
starren Modell, dreht der Rotor mit dem Gelenk mal der Übersetzung, und jedes Getriebe verliert seinen Wirkungsgrad an
dem, was hindurchgeht.

Die Koordinate eines Gelenks ist ein Winkel (rad) oder ein Weg (m). Eine Spindel oder Zahnstange macht aus der Drehung am
Getriebeausgang einen Weg (`transmission::Converter`). Hinter das Getriebe gesetzt, wird das Paar zu einem Getriebe in N
und m mit

- Übersetzung i / Hebel, der Hebel ist der Weg je Radiant (Steigung / 2 pi oder der Teilkreisradius des Ritzels);
- Steifigkeit 1 / (Hebel^2 / k_Getriebe + 1 / k_Wandler): die Torsionssteifigkeit des Getriebes am Gelenk gesehen, in
  Reihe mit der von Spindel oder Zahnstange; die Dämpfung ebenso durch Hebel^2;
- Spiel: das des Getriebes am Gelenk gesehen plus das des Wandlers (mit `effects.gearbox_backlash`);
- Wirkungsgrad: das Produkt (mit `effects.gearbox_efficiency`);
- die Trägheit von Spindelwelle oder Ritzel am Rotor, durch i^2.

Das Signal `gearbox.<motor>.output_torque` ist die Gelenkkraft mal dem Hebel, durch den Wirkungsgrad des Wandlers,
solange der Motor treibt, und mal ihm, solange die Last zurücktreibt; die Getriebeprüfungen beurteilen so das Moment am
Getriebeausgang, vor Spindel oder Ritzel. Der Wirkungsgrad des Wandlers wird getrennt vom Getriebe geführt
(`RotaryJoint::extra_efficiency`): Mit `effects.thermal_feedback` ändert die Temperatur des Getriebes nur dessen Anteil. Ein Schiebegelenk liest Reibung, Hub und Anschläge seiner Führungen
aus `[slide]` (`RotaryJoint::linear_from_config`).

## Kugelgewindetrieb

`transmission::BallScrew`, Konfiguration `[screw]`. Die Formeln sind die der ISO 3408-5 und der Herstellerkataloge:

| Größe | Formel |
| --- | --- |
| Kritische Drehzahl | n_kr = 60 / (2 pi) (lambda L)^2 / L^2 sqrt(E I / (rho A)), ein gleichförmiger Stahlbalken mit dem Kerndurchmesser; lambda L = 1,875, pi, 3,927, 4,730 für fest-frei, los-los, fest-los, fest-fest. Zulässig: `critical_speed_factor` (0,8) mal davon. |
| Knicken | F_k = Faktor pi^2 E I / L^2, Faktor 1/4, 1, 2,046, 4; zulässig F_k / `buckling_safety_factor`. |
| Statische Last | C0a / `static_safety_factor`. |
| d n | Nenndurchmesser (mm) mal Drehzahl (1/min), gegen `max_dn` (0 prüft nicht). |
| Axialsteifigkeit | Welle A E / x (fest-fest: A E L / (x (L - x))), Mutter und Festlager in Reihe; an der weichsten Stellung der Mutter genommen. |
| Reibmoment der Vorspannung | T = K F_pr l / (2 pi), K = 0,05 / sqrt(tan beta) (THK); als T 2 pi / l zur Reibung der Führungen addiert. |
| Lebensdauer | L10 = (Ca / (fw F_m))^3 10^6 Umdrehungen. Eine vorgespannte Mutter trägt F_pr (1 + F / (2,83 F_pr))^(3/2), bis die Kraft die Vorspannung abhebt (Hiwin). |

Die Signale der Spindel: `screw.1.speed` (1/min), `screw.1.dn`, `screw.1.damage_rate` (der Anteil ihrer Lebensdauer, der
je Stunde verbraucht wird, 60 |n| (fw F_eq / Ca)^3 / 10^6). Die Lebensdauer, die eine Prüfung meldet, ist eins durch den
Mittelwert dieser Rate über den gemessenen Durchlauf (Palmgren-Miner), mal dem Lebensdauerfaktor a1 aus
`ratings.bearing_reliability_percent` (0,25 bei 99 %, wie für jede Lagerlebensdauer), gegen `ratings.required_life_h`. Die
Axialkraftprüfungen nehmen |Kraft|, auch auf Zug; für das Knicken liegt das auf der sicheren Seite.

Eine senkrechte Achse: `slide.incline_deg = 90` (die Schwerkraft zieht mit dem Sinus der Neigung längs des Weges).

## Zahnstange und Ritzel

`transmission::RackPinion`, Konfiguration `[rack]`: Normalmodul, Zähnezahl und Schrägungswinkel ergeben den
Teilkreisradius m z / (2 cos beta). Die Prüfung `rack-feed-force` hält die höchste Kraft auf die Zähne (Signal
`rack.1.feed_force`: die Kraft an der Last durch den Wirkungsgrad der Zahnstange, solange das Ritzel treibt, mal ihm,
solange die Last zurücktreibt) gegen `rated_feed_force_n` durch den Anwendungsfaktor K_A (`service_factor`). Die Hersteller von Zahnstangen veröffentlichen die
zulässige Vorschubkraft eines Paares, keine Lebensdauerformel; daher hat die Zahnstange keine Lebensdauerprüfung, das
Planetengetriebe davor hat seine eigene (`gearbox-life`).

## Linearmotor

Keine Übertragung: Das Getriebe des Gelenks hat die Übersetzung 1 und steht für die Befestigung des Primärteils (seine
Steifigkeit in N/m). Die Werte in `[motor]` und `[drive]` gelten in N statt N m und m statt rad: Kraftkonstante in N/A,
die Masse des Primärteils als Trägheit des Rotors (nicht in `mass.carriage_kg` enthalten), Geschwindigkeiten in m/s, Geberstriche je 2 pi m. Das Rastmoment
wiederholt sich einmal je Polteilung: `motor.cogging_order` wird durch 2 pi / `linear_motor.pole_pitch_m` ersetzt.

Die Anziehungskraft zwischen Primärteil und Magnetbahn (`attraction_force_n`) drückt auf die Führungen: Das Signal
`axis.x.guide_load` ist die Anziehungskraft plus das Gewicht von Schlitten, Last und Primärteil quer zur Führung, und die Führungsprüfungen (`guide-load`
gegen `ratings.guide_max_load_n`, `guide-life` aus `ratings.guide_dynamic_load_rating_n`, L10 = (C / P)^3 50 km)
beurteilen sie. `guide_friction_coefficient` mal die Anziehungskraft kommt zur Coulomb-Reibung der Führungen hinzu.

Die Szene zeigt die Geschwindigkeit des Motors in m/s; er hat keine Welle, die sich dreht.

## Rundtisch: Wellgetriebe und Schneckengetriebe

Der Tisch dreht um eine senkrechte Achse: M = Trägheit von Tisch und Werkstück, kein Schweremoment. `table.gear` wählt das
Getriebe:

- `"strain-wave"`: `transmission::StrainWave`, das Katalogverfahren von Harmonic Drive: L10 = L_n (T_r / T_av)^3 (n_r /
  n_av), T_av das kubische Mittel des Abtriebsmoments, gewichtet mit der Antriebsdrehzahl. Signal
  `strain_wave.1.damage_rate`, Prüfung `strain-wave-life`.
- `"worm"`: `transmission::WormGear` gibt den Wirkungsgrad aus Steigungswinkel gamma, Flankenreibwert mu und
  Normaleingriffswinkel alpha_n: tan gamma / tan(gamma + rho'), rho' = atan(mu / cos alpha_n). Der Rückwirkungsgrad,
  tan(gamma - rho') / tan gamma, wird berechnet (null oder kleiner: das Getriebe ist selbsthemmend), von der Dynamik aber
  noch nicht verwendet, siehe die Grenzen. Der Wirkungsgrad der Schnecke wird wie der eines Wandlers getrennt vom
  eingestellten Getriebe geführt. Der `worm`-Knoten einer Maschinendatei ist dieser Antrieb mit einem eigenen
  Rückwirkungsgrad, der bei 0 oder weniger selbst sperrt ([Maschinendateien](maschinendateien.md#schneckengetriebe)).
- `"planetary"`: das `[gearbox]`, wie es ist.

## Roboter aus URDF-Dateien

`rigid::Chain` liest eine URDF-Datei (Links mit ihren Trägheitsangaben, Gelenke der Typen revolute, continuous, prismatic
und fixed; feste Gelenke werden mit dem Körper davor vereinigt; kein xacro, keine Verzweigungen) und rechnet die Dynamik
mit dem rekursiven Newton-Euler-Verfahren (Luh, Walker und Paul 1980): die Gelenkmomente zu gegebenen Beschleunigungen,
die Massenmatrix Spalte für Spalte, Schwere- und Geschwindigkeitsterme. Die Last ist eine Punktmasse bei
`arm.payload_offset_m` vom Spitzenrahmen der URDF.

Konfiguration `[arm]`: `urdf_file` ist ein Pfad oder `builtin:six-axis` / `builtin:scara` für die Beispiele in
`crates/machine-joint-kit/urdf/`. `gear_ratios` hat einen Eintrag je Gelenk; die Grenzen jedes Gelenks stehen in `[axis.j1]`,
`[axis.j2]`, ... (`max_velocity_rad_s`, für ein Schiebegelenk `max_velocity_m_s`, und so weiter). Die Grenzen der URDF sind die Anschläge der Gelenke; ein Gelenk vom Typ continuous nimmt `min_angle_deg` und
`max_angle_deg` aus `[joint]`. Ein Schiebegelenk läuft auf der `[screw]`, mit seinem Motor gekuppelt (seine Übersetzung muss
1 sein; Steifigkeit und Dämpfung des `[gearbox]` stehen für die Kupplung, sein Wirkungsgrad und seine Trägheit zählen
nicht), mit der Reibung aus `[slide]` (die Pinole des SCARA). Eine fehlende Grenze der URDF ist 0; ein leerer Bereich
wird abgelehnt. `ratings.gearbox_reference_speed_rpm` wird abgelehnt, wo die Übersetzungen der Gelenke verschieden sind, da
die Getriebelebensdauer die Drehzahl jedes Motors über `gearbox.ratio` nimmt. Mit einer Tabelle `[strain_wave]` bekommt das Getriebe jedes Drehgelenks die Lebensdauerprüfung.

Die Werkzeuglage sind die Gelenkstellungen: Zyklen werden in Gelenken angegeben, und die Auslegung beurteilt Motoren,
Getriebe und Spindeln entlang dieser. Die Einschwingtoleranz ist dann in rad (`ratings.settling_tolerance_m` behält
seinen Namen); sie enthält die Verdrehung der Getriebe unter dem Gewicht des Arms, die die Geber der Motoren nicht sehen
(2,6 mrad an Gelenk 2 des Sechsachsbeispiels, ausgestreckt).

### Werkzeugraum

Mit `kinematics.tool_space = true` sind die Achsen der Maschine die Lage des Flanschs, und jeder Planer, der Zyklus, die
Kommandozeile und die App bewegen das Werkzeug wie beim Delta: Eine Gerade einer Bahn ist eine Gerade des Werkzeugs. Die
Gelenkmaschine rechnet die Lage über `inverse` des Mechanismus in die Gelenke und über `forward` zurück, die
Geschwindigkeit des Werkzeugs über die Jacobi-Matrix von `forward` (zentrale Differenzen) in die der Gelenke. Die
Vorsteuerung nimmt dann die Geschwindigkeitsterme hinzu: die Gelenkbeschleunigungen J^-1 (a - J' q'), J' q' als zweite
Differenz von `forward` entlang q', und die Zentripetal- und Coriolismomente des Newton-Euler-Modells (im Gelenkraum
lässt sie sie weg, wie bisher). Ziele werden abgelehnt außerhalb der Reichweite (`error-arm-out-of-reach`), jenseits
eines Gelenkanschlags (`error-joint-beyond-stop`) oder wo die Konditionszahl der Jacobi-Matrix über
`workspace.max_condition_number` liegt (`error-arm-singularity`); eine Bahn durch eine solche Lage mit
`error-path-range`, wie beim Delta. Die Arbeitsraumkarte und `ratings.max_condition_number` nehmen dieselbe Zahl.

- SCARA (vier Gelenke: zwei Arme um senkrechte Achsen, eine senkrechte Pinole, ein drehender Flansch): Die Achsen sind
  x, y, z des Flanschs (m) und seine Drehung c um die Senkrechte (rad: Gelenke 1 + 2 + 4 plus die Drehung bei null), in
  geschlossener Form. Längen und Richtungen der Arme bei null, der Ort von Gelenk 1, die Richtung der Pinole und die
  Drehung des Flanschs kommen aus der URDF bei null und werden an drei Stellungen gegen die Kette geprüft (eine URDF, die
  kein SCARA ist, wird abgelehnt). Gelenk 2 nach dem Kosinussatz, cos(beta) = (r^2 - L1^2 - L2^2) / (2 L1 L2), mit
  `kinematics.elbow` "right" (beta > 0: Gelenk 2 vom gestreckten Arm aus gegen den Uhrzeigersinn, der Ellbogen von oben
  gesehen rechts der Linie von der Basis zum Werkzeug) oder "left"; Gelenk 1 = atan2(y, x) - atan2(L2 sin beta, L1 +
  L2 cos beta). Die Konditionszahl ist die von d(x, y)/d(q1, q2): unendlich beim gestreckten und beim gefalteten Arm
  (25 bei 11 Grad Ellbogen, die Grenze des Beispiels).
- Sechs Gelenke: x, y, z des Flanschs (m) und seine Orientierung als Drehvektor rx, ry, rz (rad: Achse mal Winkel,
  gegen das Basissystem), numerisch. Die Gelenke werden von `kinematics.home_joints` aus entlang der Geraden von der
  Flanschlage der Grundstellung zum Ziel verfolgt, die Orientierung per Slerp, in Schritten von höchstens 20 mm und
  0,05 rad, jeder mit gedämpften kleinsten Quadraten auf der geometrischen Jacobi-Matrix (dq = J^T (J J^T + lambda^2
  I)^-1 e, lambda 1 mm, die Drehzeilen mal dem Abstand des Flanschs von Gelenk 2 in der Grundstellung), auf 1 um
  eingeschwungen, der letzte auf 1e-13 m. So bleibt die Lösung auf dem Zweig der Grundstellung (Schulter, Ellbogen,
  Handgelenk; bei einem Arm, dessen Zweige durch Singularitäten getrennt sind, wie bei den üblichen, hängt sie nicht vom
  Weg dorthin ab), und ein Schritt, der nicht einschwingt (außer Reichweite oder eine Singularität, die der Zweig nicht
  passieren kann), lehnt die Lage ab. Ein Flansch, der bis auf 0,01 rad eine halbe Drehung gegen das Basissystem
  gedreht ist, wird abgelehnt (`error-arm-orientation-half-turn`): Dort springt der Drehvektor. Die Konditionszahl ist
  die der geometrischen 6 x 6-Jacobi-Matrix mit den Drehzeilen skaliert wie oben.

Die Planer interpolieren jede Achse linear: Eine Gerade hält die Orientierung, wo beide Enden dieselbe haben, und dreht
sie per Slerp, wo die beiden Drehvektoren parallel sind (bei nach unten zeigendem Flansch ist eine Drehung um die
Senkrechte das nicht: Dort liegt die Drehung nahe am Slerp, ist es aber nicht). `config/scara-tool.toml` und
`config/arm-tool.toml` sind die Standardkonfigurationen mit eingeschaltetem Werkzeugraum, einer Startlage und einem
Zyklus aus Geraden (die Einschwingtoleranz des Arms 5 mm: Unter der Nutzlast ruht sein Flansch 4,0 mm unter der Lage
der Geber).

## Teleskopwelle mit zwei Kardangelenken (delta-d4)

Der Vierachs-Delta `delta-d4` (ein eigenes Crate auf dem D3, keine Gelenkmaschine) dreht sein Werkzeug über eine
Teleskopwelle mit je einem Kardangelenk an beiden Enden. Ein einzelnes, um beta gebeugtes Kardangelenk dreht seinen
Abtrieb ungleichförmig: tan chi = tan phi / cos beta, die Drehzahl zwischen cos beta und 1 / cos beta der des Antriebs,
zweimal je Umdrehung. Zwei gleich gebeugte Gelenke in einer Ebene, die Gabeln der Zwischenwelle in Phase
(Z-Anordnung), heben sich auf: Das zweite gibt phi zurück. Am Delta stehen die Motorwelle und die Werkzeugwelle beide
senkrecht (die Plattform bleibt waagerecht), also heben sich die Gelenke in jeder Lage auf, und das Werkzeug dreht mit
dem Getriebeabtrieb; nur die Welle dazwischen dreht ungleichförmig. Die Welle verdreht sich in Reihe mit dem
Werkzeuggetriebe (`effects.shaft_torsion`), ihr Gewichtsanteil belastet die Arme (`effects.shaft_mass`), und die
Gleitreibung des Teleskops zieht längs der Welle an der Plattform (`effects.telescope_friction`). Siehe die
Modellreferenz (docs/model-reference.md, Abschnitt „Delta picker D4“).

## Biegende Glieder und das Gestell unter der Maschine

Mit `effects.link_bending` (ausgeliefert aus) biegt sich jedes Glied eines Roboters mit einem Querschnitt in
`[link_bending]` (ein rundes Rohr je Gelenk: Außendurchmesser, Wand, Elastizitätsmodul) als Kragarm von seinem Gelenk
bis zum nächsten in seiner ersten Euler-Bernoulli-Eigenform, in der Ebene, in der sein Gelenk es dreht: modale Masse ein
Viertel seiner URDF-Masse plus alles dahinter, Steifigkeit 1,875104^4 / 4 E I / L^3, angetrieben von dem, was die Körper
dahinter von seiner Spitze brauchen (ihr m (a - g), ihr Moment über die Neigung der Spitze), und von seinem eigenen
Gewicht und seiner Trägheit längs des Glieds. Das Werkzeug schwingt im Werkzeugraum mit; die Gelenke spüren es nicht
(einseitig). Ein Stahlrohr 60 x 4 mm, 0,8 m lang und 4,5 kg schwer, schwingt allein mit den 88,96 Hz nach
Euler-Bernoulli und hängt unter seinem Gewicht 49,2 um durch, 1,3 % über dem exakten q L^4 / (8 E I).

Mit `effects.base_compliance` (jede Maschine und jede Maschinendatei, ausgeliefert aus) steht die Maschine auf einem
Gestell mit bis zu sechs Feder-Masse-Richtungen (`[base]`): Es nimmt die Änderung des Impulses alles dessen auf, was sich
auf ihm bewegt, die Rotoren eingeschlossen (`Machine::frame_momentum`; Schlitten, Tisch und Roboter geben ihren über
`Mechanism::frame_momentum`), und das Werkzeug bewegt sich mit ihm. Der Roboter auf seinem Sockel kippt, der Ständer des
Tisches verdreht sich, das Bett eines Schlittens schwingt längs der Achse. Siehe die Modellreferenz
(docs/model-reference.md, Abschnitt „Compliant base“).

## Kurbel- und Kurvenmechanismen (`machine-crank-cam`)

Die Kernfälle eines Servo-Auslegungswerkzeugs (Siemens SIZER), die ein Gelenk allein nicht abdeckt: eine Kurbel, die eine
Drehung in einen Hub wandelt, und zwei Achsen, die über eine elektronische Kurvenscheibe an einer laufenden Bahn oder Stange
arbeiten. Jede ist ein motorgetriebenes Gelenk (`chain::JointMachine`); Motor, Getriebe, Regelkreise, Erwärmung und alle
gemeinsamen Prüfungen sind dieselben wie oben.

| Id | Mechanismus | Übertragung | Werkzeugpose | Eigene Prüfungen |
| --- | --- | --- | --- | --- |
| `slider-crank` | Servopresse: Kurbel 50 mm, Pleuel 200 mm, Stößel 15 kg und Werkzeug 10 kg darunter, Presskraft bis 3 kN | Planetengetriebe 20 | Kurbelwinkel (rad), 0 im unteren Totpunkt | `rod-force`, `slider-guide-load`, `crank-bearing-load` |
| `rotary-knife` | Querschneider: Messerwalze mit 100 mm Radius, ein Messer, schneidet eine Bahn mit 1 m/s in Formate von 0,5 m | Planetengetriebe 10 | Walzenwinkel (rad), 0 mit einem Messer am Schnitt | |
| `flying-saw` | Wagen 40 kg mit Säge 25 kg, Material mit 1 m/s, in Längen von 1,5 m in einem Fenster von 0,2 m geschnitten | Schrägverzahntes Ritzel auf Zahnstange hinter Planetengetriebe 5 | x (m) | `rack-feed-force` |

**Schubkurbel.** Der Abstand des Schiebers vom Kurbelmittelpunkt ist x(θ) = r cos θ + √(l² − r² sin² θ), seine
Übersetzung x'(θ) = −r sin θ − r² sin θ cos θ / w mit w = √(l² − r² sin² θ), und x''(θ) = −r cos θ − r² cos 2θ / w −
r⁴ sin² θ cos² θ / w³. Die Pleuelmasse teilt sich in einen Anteil am Kurbelzapfen (`rod.crank_end_share`) und den Rest am
Schieber. Die Trägheit an der Kurbel hängt vom Winkel ab: J(θ) = J_Kurbel + m_Zapfen r² + m_x x'², m_x Schieber, Werkzeug
(Nutzlast) und Pleuelende am Schieber; ihr Geschwindigkeitsterm m_x x' x'' θ'² geht auch in die Vorsteuerung. Schwerkraft
längs der Führung (`slider.incline_deg`, −90 für einen Stößel unter der Kurbel) und am Zapfen. Die Prozesskraft
(`effects.process_force`) ist eine Tabelle über dem Abstand des Stößels vom unteren Totpunkt (`[process]`); sie drückt
zurück, solange der Stößel ausfährt (Umformen), nicht auf dem Rückweg, und wirkt an der Kurbel als −F x'(θ): nahe dem
Totpunkt kostet eine große Kraft wenig Moment. Signale: Lage, Geschwindigkeit und Beschleunigung des Stößels, Prozesskraft,
Pleuelkraft T = (F_längs − m_x ẍ) l / w, Querlast der Führung |m_x g cos β − T r sin θ / l|, Last des Kurbellagers (Zug
des Pleuels und Zapfenmasse auf ihrem Kreis) und J(θ). Geprüft gegen `ratings.rod_max_force_n`,
`slider_guide_max_load_n` und `crank_bearing_max_load_n`.

**Rotationsmesser.** Leitwert der Kurvenscheibe ist die Bahn in Metern. Im Synchronbereich ±α um jeden Schnitt
(`knife.sync_angle_deg`) läuft die Messerspitze mit der Bahn, Steigung 1 / R; zwischen den Schnitten führt ein
Bewegungsgesetz die Walze den Rest einer Messerteilung, 2π / n − 2α, über den Rest des Formats, L − 2Rα. Ein Format
kürzer als der Umfang durch n beschleunigt die Walze zwischen den Schnitten, ein längeres bremst sie: im Mittel
(2π / n − 2α) / (L − 2Rα) rad je Meter Bahn, mit 3-4-5 in der Mitte 1 / R + 1,875 × (das − 1 / R). Der Schnitt
(`effects.process_force`) bremst die Walze mit einer Kraft an der Spitze über dem Winkel vom Schnitt (`[cut]`).

**Fliegende Säge.** Leitwert ist das Material. Je Schnittlänge: Der Wagen kommt auf die Geschwindigkeit v des Materials,
während das Material v² / a läuft (`saw.sync_acceleration_m_s2` = a, die mittlere Beschleunigung), seine Geschwindigkeit
geht wie 3u² − 2u³ über (kein Beschleunigungssprung, Spitze 1,5 a); er legt dabei die Synchronisierstrecke v² / (2a)
zurück, läuft W / v Sekunden mit dem Material durch das Schnittfenster W, löst sich über weitere v² / (2a) und fährt den
Hub W + v² / a mit einem Bewegungsgesetz im Rest der Schnittlänge zurück, L − 2v² / a − W Material. Eine Schnittlänge
ohne Platz dafür wird abgewiesen (`saw.cut_length_m`). Der Schnitt drückt längs des Materials über dem Weg ins Fenster
auf den Wagen (`[cut]`).

**Elektronische Kurvenscheibe.** `sim_components::cam` hält eine Kurvenscheibe als Folgewert über einem Leitwert,
periodisch, der Folgewert steigt je Leitwertperiode um seinen Vorschub (Kurbel oder Messer eine Umdrehung, die Säge 0).
Segmente nehmen die Bewegungsgesetze der VDI 2143: `linear` (Synchronlauf), `dwell` (Rast), `3-4-5` und `4-5-6-7` (Polynome
5. und 7. Ordnung), `cycloid` (geneigte Sinuslinie) und `harmonic` (einfache Sinuslinie). Wo ein Gesetz an ein lineares
Segment oder eine Rast stößt, übernimmt es dessen Steigung, ohne Beschleunigung dort (Geschwindigkeit in Geschwindigkeit);
zwischen zwei Gesetzen ruht der Folgewert (Rast in Rast). Eine Kurventabelle (`cam.table_file`, CSV aus Leitwert,
Folgewert und optional Steigung) legt eine kubische Kurve durch ihre Punkte: mit gegebenen Steigungen je zwei Punkte,
ohne sie der periodische kubische Spline (stetige Folgebeschleunigung). Mit `cam.run = true` fährt die Steuerung die
Kurvenscheibe statt `[[cycle.segment]]` als Zyklus: der Leitwert läuft mit einer sin²-Rampe in `cam.ramp_s` hoch, die
Folgegeschwindigkeit ist s'(m) m', die Beschleunigung s''(m) m'² + s'(m) m''; ein Durchgang ist eine Leitwertperiode bei
voller Geschwindigkeit, sodass `analyze` und `verdicts` den eingeschwungenen Zustand messen; ein Stopp fährt den Leitwert
über dieselbe Rampe herunter. Verlangt die Kurvenscheibe bei der Leitgeschwindigkeit mehr Folgegeschwindigkeit,
-beschleunigung oder -ruck, als die Achsgrenzen erlauben, meldet das Programm `limit-cam-speed`, `-acceleration`,
`-jerk` (wie die Motorgrenzen). Die Schubkurbel läuft mit konstanter Drehzahl (`crank.motion = "constant"`, eine
Umdrehung je Hub wie eine mechanische Presse mit Schwungrad) oder mit der Servopressen-Kurve aus `[[cam.segment]]`
(schnelle Annäherung, langsam durch die Umformzone, schneller Rückhub) bei `crank.strokes_per_min`.

Die mitgelieferten Konfigurationen lassen `cam.run = false`: der Zyklus schwenkt die Kurbel vom oberen Totpunkt in den
unteren und zurück (Servopresse im Pendelhub), taktet das Messer um eine halbe Umdrehung hin und zurück und fährt den
Sägewagen über seinen Hub. Die Kurvenscheiben laufen mit `--set cam.run=true` (App: der Schalter der Gruppe
Elektronische Kurvenscheibe):

```text
sim-cli verdicts slider-crank --set cam.run=true
sim-cli analyze rotary-knife --set cam.run=true --set knife.format_length_m=0.4
sim-cli verdicts flying-saw --set cam.run=true --set material.speed_m_s=1.2
```

Moment und Schleppfehler kommen wie bei jeder Maschine aus dem Antriebsmodell; die Formeln der Kurvenscheiben und
Mechanismen sind in `cam.rs` und den Tests von `machine-crank-cam` von Hand nachgerechnet (unten).

## Was von Hand nachgerechnet ist

Komponententests (`sim-components`): kritische Drehzahl, Knicklast, Trägheit, Steifigkeit, Reibmoment der Vorspannung,
Lebensdauer der Spindel und ihre Signale und Prüfungen; der Wandler hinter einem Getriebe (Übersetzung,
Reihensteifigkeit, Spiel, Wirkungsgrad, Trägheit); Reibung und Anschläge des Schlittens; Teilkreisradius und
Vorschubkraft der Zahnstange; die Wirkungsgrade der Schnecke; die Lebensdauer des Wellgetriebes; das Pendel aus dem
Lehrbuch und die Energieerhaltung der Newton-Euler-Kette.

Maschinentests (`machine-joint-kit`): Jede Maschine besteht die Standardprüfungen (Hin- und Rückrechnung, statisches
Halten, Energieerhaltung ohne Verluste, Energiebilanz mit ihnen, Konvergenz bei halbem Zeitschritt, Determinismus), und
ihr Standardzyklus besteht alle Prüfungen. Im starren, verlustfreien Modell: Die Spindel erreicht 2400 1/min bei 0,8 m/s
und schiebt mit Masse mal der Beschleunigung, die der Lauf erreichte; Vorschubkraft und Getriebemoment der Zahnstange sind
m a und m a r; die Führungen des Linearmotors tragen 2500 + 13 g = 2627,49 N; das Getriebe des Tisches überträgt J alpha;
der ausgestreckte Sechsachsroboter hält 177,304 N m an Gelenk 2 (-1,47753 N m an seinem Motor) und 21,3785 N m an Gelenk
3, von Hand aus den Massen der URDF summiert; die Spindel des SCARA hält 5,5 g = 53,937 N, -0,171686 N m an ihrem Motor.
Im Werkzeugraum: Die geschlossene Form des SCARA gibt bei (0,325, 0,275) den Ellbogen bei ±pi/2 und Gelenk 1 bei 0 oder
1,404514 rad von Hand; Vorwärts nach Rückwärts gibt die Lagen auf 1e-9 zurück (den ganzen Bereich des SCARA, beim Arm um
seine Grundstellung gestreut, das Handgelenk auf der Seite der Grundstellung); jede geplante Lage einer Geraden liegt auf
1 um auf ihr und die Orientierung des Arms auf 1e-6 rad; die Gelenkgeschwindigkeiten sind J^-1 v von Hand
(1,538462 rad/s); eine Gerade in den gestreckten SCARA-Arm wird abgelehnt; der zeitoptimale Plan hält die Motoren
entlang einer Geraden unter ihrer Nenndrehzahl; beide Werkzeugzyklen bestehen ihre Prüfungen.
Eine 1,6 m lange Spindel fällt bei `screw-speed` durch, und eine geforderte Lebensdauer über der der Spindel bei
`screw-life`. Mit Verlusten: Getriebemoment und Vorschubkraft der Zahnstange sind die Kraft an der Last durch 0,97, die
Lebensdauer der Spindel schrumpft bei 99 % Zuverlässigkeit um a1, und die Temperaturrückführung behält den Wirkungsgrad
von Spindel, Zahnstange und Schnecke.

Tests der Kurbeln und Kurvenscheiben (`sim-components` cam.rs, `machine-crank-cam`): ein 3-4-5-Segment mit 0,1 m Hub über 2 rad
bei 4 rad/s erreicht 1,875 h V / T = 0,375 m/s, 5,7735 h V² / T² = 2,3094 m/s² und einen Ruck von 60 h V³ / T³ = 48 m/s³;
jedes Gesetz steigt von Rast zu Rast mit Ableitungen, die seine Differenzen sind; ein Gesetz zwischen zwei Synchronläufen
der Steigung 10 behält sie an seinen Enden und hat in der Mitte 10 + 1,875 (12 − 10); eine Sinustabelle mit Steigungen
und ihr periodischer Spline ohne laufen durch den Sinus; die sin²-Rampe des Leitwerts legt Rampe × V / 2 zurück. Die
Schubkurbel bei 60° (r 50 mm, l 200 mm): x = 0,2202562 m, x' = −0,0488454 m/rad, x'' = −0,0187556 m/rad²; J = 0,0869757 kg m²,
und rückwärts mit 2 rad/s, um 10 rad/s² schneller, gegen 1 kN −47,878563 N m an der Kurbel; bei −90° drückt das Pleuel
mit 1032,796 N, die Führung nimmt 258,199 N. Das Messer (R 0,1 m, ±10°) dreht auf einem Format von 0,5 m zwischen den
Schnitten im Mittel 12,758984 rad/m und in der Mitte 15,173095, auf 0,8 m 7,756072. Die Säge (1 m/s, 5 m/s²) synchronisiert
über 0,1 m mit 7,5 m/s² Spitze, läuft 0,2 m mit dem Material und fährt mit 0,8333 m/s zurück. Jede Maschine besteht die
Standardprüfungen, ihr Standardzyklus und ihre Kurvenscheibe bestehen alle Urteile, ein Durchgang der Kurvenscheibe dauert
eine Leitwertperiode (2, 0,5 und 1,5 s) und ein Stopp endet innerhalb der Rampe; die Kurvenscheibe des Messers bei einer
Grenze von 10 rad/s wird mit 15,17 rad/s gemeldet.

## Grenzen

- Ein Glied biegt sich in einer Eigenform in der Ebene, in der sein Gelenk es dreht; aus dieser Ebene heraus und in
  Torsion ist es starr, und Biegung und Gestell wirken nicht auf die Gelenke zurück (einseitig).

- Die Roboter werden im Gelenkraum ausgelegt: keine kartesischen Bahnen und keine inverse Kinematik des Werkzeugs.
- Jedes Gelenk eines Roboters hat denselben Motor, Antrieb und dasselbe Getriebe (nur die Übersetzung unterscheidet sich je
  Gelenk), da der Antrieb ein `[motor]` und ein `[drive]` je Maschine liest; keine Getriebelebensdauer für Roboter mit
  verschiedenen Übersetzungen der Gelenke.
- Die Steifigkeit einer Spindel wird für den ganzen Hub an der weichsten Stellung der Mutter genommen.
- Ein vom Abtrieb zurückgetriebenes Schneckengetriebe verliert wie jedes Getriebe (1 / Wirkungsgrad) und hemmt nie selbst.
- Keine Unwucht am Rundtisch, keine Lebensdauerformel der Zahnstange (keine veröffentlicht), keine Prüfung des
  Kreuzrollenlagers.
- Noch nicht gebaut: mechanische Kurvenscheiben mit Abtastrolle und ihre Kontaktkraft; das Förderband aus `[conveyor]`
  kann noch nicht Leitwert einer Kurvenscheibe sein (der Leitwert ist ein eigener, mit dem Geschwindigkeits- und
  Rampenmodell des Bands); kein MC_CamIn über OPC UA; eine geänderte Leitgeschwindigkeit gilt sofort, ohne Rampe;
  das Gelenk von Kurbel und Messer hat keine Anschläge, solange die Kurvenscheibe es weiterdreht; die Presskraft wirkt
  nur beim Ausfahren (keine Rückfederung), die Schnittkraft von Messer und Säge nur vorwärts.
