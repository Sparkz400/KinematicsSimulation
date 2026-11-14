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
- [Parallelkinematiken: Hexapod und Seilroboter](#parallelkinematiken-hexapod-und-seilroboter)
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

## Parallelkinematiken: Hexapod und Seilroboter

Zwei Parallelkinematiken haben eigene Crates, `machine-hexapod` und `machine-cable-robot`, und werden wie jede andere
Maschine gerechnet und ausgelegt (`sim-cli analyze hexapod`, `sim-cli verdicts cable-planar`, die 3D-Ansicht der App,
`search`):

| Id | Mechanik | Antriebe | Werkzeuglage | Eigene Prüfungen |
| --- | --- | --- | --- | --- |
| `hexapod` | Stewart-Gough-Plattform, 25 kg und bis zu 30 kg darauf, auf sechs elektrischen Zylindern (450 mm eingefahren, 200 mm Hub) | je Bein ein Kugelgewindetrieb 25 x 10 hinter einer Kupplung an einem 1FK2105-4AF | x, y, z (m), rx, ry, rz (rad, ein Drehvektor) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` der Spindel je Bein, unter der Kraft des Beins |
| `cable-planar` | ein Schlitten von 3 kg und bis zu 2 kg, ein Punkt in einer senkrechten Ebene, an 4 Seilen aus einem Gestell 2 m x 1,6 m | je Seil eine Winde: Trommel 25 mm hinter einem Planetengetriebe 5:1 | x, z (m) | `cable-max-tension`, `cable-min-tension` je Seil |
| `cable-spatial` | eine Plattform von 4 kg und bis zu 4 kg, sechs Freiheitsgrade, an 8 Seilen aus den Ecken eines Gestells 2 x 1,6 x 1,8 m | wie der ebene, Trommel 30 mm | x, y, z (m), rx, ry, rz (rad) | wie der ebene |

Ihre Maße sind allgemein gehalten, jeder Wert ein `[estimate]` bis auf die von Motor und Antrieb (Katalog, wie oben).
Jeder Standardzyklus besteht alle seine Prüfungen; die Beruhigungstoleranzen der Seilroboter sind Anforderungen ihrer
Größe (0,3 mm eben, 1 mm räumlich), nicht die 0,05 mm der Kugelgewindeachse.

### Hexapod

Jedes Bein ist ein Gelenk von `chain::JointMachine`, dessen Koordinate der Auszug q_i des Zylinders ist: Spindel, Mutter
und Lager, Kupplung, Spiel, die Reibung der Stangenführung und der Vorspannung der Mutter, die Anschläge an den
Hubenden und der Motor sind die der Kugelgewindeachse oben. Die Mechanik gibt Kinematik und Dynamik der Plattform:

- Inverse Kinematik geschlossen: Bein i reicht von seinem Basisgelenk B_i zum Plattformgelenk p + R a_i, seine Länge
  L_i = `leg.closed_length_m` + q_i. Die Gelenke sitzen paarweise: Basispaare bei 120 k ± `hexapod.base_half_angle_deg`,
  Plattformpaare bei 120 k ± (60 − `platform_half_angle_deg`), jedes Bein zum benachbarten Paar.
- Vorwärtskinematik nach Newton über die Bewegung der Plattform, von der Mitte des Werkzeugquaders aus:
  xi = J^-1 (q − q(x)), p += v, R = exp(w) R, bis die Beine um weniger als 1e-13 m verfehlen; Beinlängen, die keine
  Lage erfüllt (keine Konvergenz in 40 Schritten), geben NaN, was jeder Aufrufer ablehnt.
- Die Zeilen von J sind (u_i, b_i × u_i), u_i die Richtung des Beins und b_i = R a_i. Dynamik über die Bewegung der
  Plattform: M_xi xi' + c + g = J^T f, also im Beinraum M_q = J^-T M_xi J^-1 und h = J^-T (c + g − M_xi J^-1 a0), a0
  die Längenbeschleunigung der Beine bei der Bewegung (gegen die Lagrange-Gleichungen von M_q mit Differenzen geprüft:
  gleich auf 1e-9). Die Masse jedes Beins wird auf seine Enden aufgeteilt: `leg.platform_share` bewegt sich mit dem
  Plattformgelenk (eine Punktmasse in Masse und Trägheit der Plattform), der Rest ruht an der Basis.
- Abgelehnt: ein Bein über seinen Hub hinaus (`error-joint-beyond-stop`), ein Bein steiler gegen die Normale der Basis
  oder der Plattform als `workspace.max_joint_angle_deg` (`error-hexapod-joint-angle`), eine Lage, deren Jacobi-Matrix
  eine Konditionszahl über `workspace.max_condition_number` hat, die Drehungen mit dem Plattformradius gewichtet
  (`error-hexapod-singularity`). Die Arbeitsraumkarte zeigt diese Konditionszahl; das Signal `platform.condition` folgt
  ihr über den Zyklus.
- Prüfungen: die Kraft jedes Beins (`joint.<i>.force`) gegen Knicklast und statische Tragzahl seiner Spindel durch ihre
  Sicherheiten, Drehzahl, d n und Lebensdauer der Spindel; Motoren und Antriebe wie an jeder Maschine.

Unter ihrem Gewicht ist die Plattform, von festen Beinkräften gehalten, instabil (d²/dz² von m g z − Σ f_i L_i < 0),
eine echte Eigenschaft, die die Lageregler halten; die verlustfreien Konfigurationen der Standardprüfungen laufen daher
ohne Schwerkraft wie die des Sechsachsers, und die Schwerkraft prüfen die von Hand gerechneten Beinkräfte und die
Standardkonfiguration.

### Seilroboter

Ein Seil verlässt seine Winde entlang der Schwenkachse einer Umlenkrolle vom Radius r (`frame.exits_m`,
`frame.winch_directions`, `pulley.radius_m`), umschlingt sie und läuft gerade zu seinem Anlenkpunkt an der Plattform
(`platform.anchors_m`; beim ebenen treffen sich die Seile in der Mitte des Schlittens). Die Rolle schwenkt in die Ebene
des Anlenkpunkts; in dieser Ebene ist die Länge der Bogen r beta plus die Tangente l_f = sqrt(m² − r²), und dL/dP = −u,
u der Einheitsvektor vom Anlenkpunkt zum Ablaufpunkt.

- Statik: Die Strukturmatrix A (n x m) hat die Spalten (u_k, b_k × u_k) in den Freiheitsgraden der Plattform; die Seile
  halten eine Dyname w, wenn A t = w mit jeder Seilkraft im Band von `cable.min_tension_n` bis zur Nennkraft
  `cable.breaking_load_n` / `cable.safety_factor`. Seile können nur ziehen, daher verteilt ein kleines QP die Kräfte:
  die nächsten an `cable.reference_tension_n`, die die Dyname im Band halten. Sitzt kein Seil an einer Grenze, ist das
  die geschlossene Form t = t_ref + A^+ (w − A t_ref); sitzt eins dort, hält das QP es da und teilt den Rest auf, und
  anders als die verbesserte geschlossene Methode (die ein Seil nach dem anderen festhält und nie wieder freigibt)
  ändern sich seine Kräfte stetig mit der Lage. Gelöst wird es mit einem Active-Set-Verfahren am bestraften Problem
  (das auch sagt, welche Seile an einer Grenze sitzen) und dann genau für die freien Seile.
- Kraftfähiger Arbeitsraum: Eine Lage, deren Plattformgewicht (mit Nutzlast) keine Seilkräfte im Band halten, wird
  abgelehnt (`error-cable-wrench-infeasible`), auch als Ziel, Bahn oder Zykluslage. In der Mitte des ebenen Roboters
  tragen die vier Seile 139,81 N (oben) und 100,19 N (unten); 5 cm unter den oberen Rollen bräuchten die oberen Seile je
  1284 N.
- Elastisch (`effects.cable_stretch`, an): Ein Seil der freien Länge l ist gespannt eine Feder von EA / l
  (`cable.ea_n`), darunter schlaff; seine Steifigkeit ändert sich über den Arbeitsraum. Die Steifigkeit der Plattform an
  ihren Seilen, elastischer und geometrischer Anteil bei gehaltenen Trommeln, gibt ihre Eigenfrequenzen
  (`sim-cli modes`, die Resonanz des Input-Shapers): 24,6 Hz in der Mitte des ebenen Roboters. `effects.cable_damping`
  fügt `cable.specific_damping_n_s` / l hinzu. Starr (aus) behalten die Seile ihre Länge.
- Jede Winde ist ein `joint::RotaryJoint` in Metern Seil: Motor, Getriebe mit dem Trommelradius als Hebel, die Trägheit
  der Trommel, Reibung von Lagern und Rolle (`effects.winch_friction`), Anschläge bei leerer und voller Trommel.
  Elastisch sind Trommel und Läufer eigene Körper hinter der Feder des Getriebes; starr behalten die Seile ihre Länge,
  und die Trägheit der Winden kommt zu der der Plattform.
- Die Motorwinkel einer Lage sind die der freien Längen, die die Plattform dort in Ruhe halten, l = L / (1 + t / EA),
  t die Kräfte der geschlossenen Form (die der Verteilung, wo kein Seil an einer Grenze sitzt), sodass Lagesollwerte und
  Momentenvorsteuerung (die Kräfte der Verteilung für die Dyname der Plattform, dazu die Trägheit der Winden) dieselbe
  Vorspannung von den Seilen verlangen. Die Vorwärtskinematik ist Gauß-Newton über diese Längen; die Regelung braucht
  sie zweimal je Lagereglertakt (der Schleppfehler im Werkzeugraum), daher macht sie zuerst billige Schritte und nimmt
  die numerische Jacobi-Matrix nur nahe am Rand des Arbeitsraums.
- Signale je Seil: `cable.<k>.tension`, `cable.<k>.shortfall` (wie weit es unter der kleinsten Kraft liegt),
  `cable.<k>.length`; `platform.condition`, die Konditionszahl der Strukturmatrix. Prüfungen: `cable-max-tension`
  (höchste Seilkraft gegen die Nennkraft) und `cable-min-tension` (jedes Unterschreiten fällt durch), Motoren und
  Antriebe wie an jeder Maschine.

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

Maschinentests (`machine-hexapod`, `machine-cable-robot`): Alle drei bestehen die Standardprüfungen, und ihre
Standardzyklen bestehen alle Prüfungen. Hexapod: Jedes Bein ist in der Grundstellung (0, 0, 0,48 m) 0,5471158 m lang,
0,0971158 m ausgefahren, von Hand aus den Gelenkkreisen; vorwärts der inversen gibt drei Lagen auf 1e-12 zurück; die
Beine halten 62,2 kg in der Grundstellung mit je 115,8772 N, m g / (6 u_z), die Motoren mit 0,184424 N m (f Steigung /
2 pi); außermittig tragen die Kräfte zusammen weiter das Gewicht. Seilroboter: In der Mitte des ebenen ist jedes Seil
1,283539 m lang (Bogen und Tangente von Hand), die Kräfte sind 139,808 N und 100,192 N, die Steifigkeit der Seile
192512 N/m quer und 119500 N/m senkrecht, die tiefste Eigenfrequenz 24,605 Hz, die geometrische Steifigkeit 235 N/m
mehr; 5 cm unter den oberen Rollen wird die Lage abgelehnt; ein Seil, das die geschlossene Form unter sein Band nimmt,
wird an der Grenze gehalten und die anderen tragen den Rest (je 8,485 N von Hand); vorwärts der inversen gibt die Lagen
des räumlichen auf 1e-10 zurück.

## Grenzen

- Die Roboter werden im Gelenkraum ausgelegt: keine kartesischen Bahnen und keine inverse Kinematik des Werkzeugs.
- Jedes Gelenk eines Roboters hat denselben Motor, Antrieb und dasselbe Getriebe (nur die Übersetzung unterscheidet sich je
  Gelenk), da der Antrieb ein `[motor]` und ein `[drive]` je Maschine liest; keine Getriebelebensdauer für Roboter mit
  verschiedenen Übersetzungen der Gelenke.
- Die Steifigkeit einer Spindel wird für den ganzen Hub an der weichsten Stellung der Mutter genommen.
- Ein vom Abtrieb zurückgetriebenes Schneckengetriebe verliert wie jedes Getriebe (1 / Wirkungsgrad) und hemmt nie selbst.
- Keine Unwucht am Rundtisch, keine Lebensdauerformel der Zahnstange (keine veröffentlicht), keine Prüfung des
  Kreuzrollenlagers.
- Noch nicht gebaut: mechanische Kurvenscheiben, elektronische Kurvenscheiben und Profile für die fliegende Säge.
- Hexapod: Das Drehen der Beine um ihre Basisgelenke und die Trägheit der Nutzlast fehlen; eine Steifigkeit je Bein
  (an der weichsten Stellung der Mutter); noch keine Eigenfrequenzen (`resonance_hz`).
- Seilroboter: Masse und Durchhang der Seile fehlen; der kraftfähige Arbeitsraum ist der statische (die Beschleunigungen
  des Zyklus beurteilen die Seilkraftprüfungen, die Planung weicht ihnen nicht aus); nahe am Rand des Arbeitsraums rechnet
  der Dehnungsausgleich der Motoren mit den Kräften der geschlossenen Form, wo die Verteilung ein Seil an einer Grenze
  hält.
