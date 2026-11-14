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
| `palletiser` | Vierachs-Palettierer, `urdf/palletiser.urdf`, ein Parallelogramm hält den Flansch waagerecht | Getriebe i = 160, das Wellgetriebe des Flanschs 50 | x, y, z, rz (ausgeliefert im Werkzeugraum), oder die vier Motorgelenke | `strain-wave-life` je Gelenk |

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
die Massenmatrix, Schwere- und Geschwindigkeitsterme. Die Last ist ein starrer Körper am letzten Glied: ihre Masse
`arm.payload_kg`, ihr Schwerpunkt `arm.payload_offset_m` vom Spitzenrahmen der URDF und ihre Trägheit um den
Schwerpunkt `arm.payload_inertia_kgm2` (xx, yy, zz, xy, xz, yz im Spitzenrahmen), mit dem Satz von Steiner in den letzten
Körper eingerechnet (`Inertial::merged`).

Konfiguration `[arm]`: `urdf_file` ist ein Pfad oder `builtin:six-axis`, `builtin:scara`, `builtin:five-axis`,
`builtin:seven-axis` oder `builtin:palletiser` für die Beispiele in `crates/machine-joint-kit/urdf/`. `gear_ratios` hat
einen Eintrag je Motor; die Grenzen jedes Motorgelenks stehen in `[axis.j1]`, `[axis.j2]`, ... (`max_velocity_rad_s`,
für ein Schiebegelenk `max_velocity_m_s`, und so weiter). Die Grenzen der URDF sind die Anschläge der Gelenke; ein
Gelenk vom Typ continuous nimmt `min_angle_deg` und `max_angle_deg` aus `[joint]`. Ein Schiebegelenk läuft auf der
`[screw]`, mit seinem Motor gekuppelt (seine Übersetzung muss 1 sein; Steifigkeit und Dämpfung des `[gearbox]` stehen
für die Kupplung, sein Wirkungsgrad und seine Trägheit zählen nicht), mit der Reibung aus `[slide]` (die Pinole des
SCARA). Eine fehlende Grenze der URDF ist 0; ein leerer Bereich wird abgelehnt. Mit einer Tabelle `[strain_wave]` bekommt
das Getriebe jedes Drehgelenks die Lebensdauerprüfung.

Jedes Gelenk hat seinen eigenen Motor, Antrieb und sein eigenes Getriebe, wo die Konfiguration es sagt: `[motor.<k>]`,
`[drive.<k>]`, `[gearbox.<k>]`, `[strain_wave.<k>]`, `[joint.<k>]`, `[ratings.<k>]` und `[parts.<k>]` über den gemeinsamen Tabellen,
k = 1, 2, ... (`catalog::motor_config`, wie bei den Maschinendateien). Die ausgelieferten Arme sind so ausgelegt: Die
Hand des Sechsachsroboters (Gelenke 4 bis 6) hat den 1FK2104-5AF des Katalogs an einem S210 6SL3210-5HE10-8UF0 hinter
einem Wellgetriebe der Baugröße 20, seine Gelenke 1 bis 3 den 1FK2105-4AF hinter Baugröße 32; Pinole und Flansch des
SCARA den 1FK2104-5AF. `ratings.gearbox_reference_speed_rpm` wird abgelehnt, wo die Übersetzung eines Motors von seinem
eigenen `gearbox.ratio` abweicht, über das die Getriebelebensdauer die Drehzahl des Motors nimmt.

Die Werkzeuglage sind die Gelenkstellungen: Zyklen werden in Gelenken angegeben, und die Auslegung beurteilt Motoren,
Getriebe und Spindeln entlang dieser. Ihr Verweilen beurteilt `ratings.settling_tolerance_rad` für die Gelenke in rad und
`settling_tolerance_m` für die in m (die Pinole des SCARA); es enthält die Verdrehung der Getriebe unter dem Gewicht des
Arms, die die Geber der Motoren nicht sehen (2,6 mrad an Gelenk 2 des Sechsachsbeispiels, ausgestreckt). Die
Vorsteuerung nimmt die Geschwindigkeitsterme auch im Gelenkraum: die Coriolis- und Zentripetalmomente der mit der
Stellung veränderlichen M(q) (zwei Glieder eines ebenen Arms bei q2 = 90 Grad und q' = (1, 2) rad/s verlangen -6 und
0,75 N m, die die Vorsteuerung bisher wegließ; auf den ausgelieferten Zyklen, die ihre Gelenke mit bis zu 2 rad/s drehen,
ändern sich Spitzen- und Effektivmoment der Motoren um weniger als 0,3 %).

### Gekoppelte Gelenke: ein Parallelogramm

URDF kennt keine geschlossenen Ketten. Ein Parallelogramm, das Gelenke koppelt, wie das eines Palettierers, das seinen
Flansch waagerecht hält, ist `arm.joint_coupling`: eine Zeile je URDF-Gelenk, die URDF-Gelenke als Summen der
Motorgelenke, q_urdf = C q. Die Maschine hat einen Motor je Spalte; die Dynamik ist die der Kette, auf die Motorgelenke
gebracht, M = C^T M_urdf C und h = C^T h_urdf (C ist fest und gibt keinen eigenen Term; die Kräfte der Kopplung leisten
keine Arbeit). Ein Motorgelenk, das allein in einer Zeile mit dem Faktor c steht, nimmt die Grenzen dieses URDF-Gelenks
geteilt durch c. `[]` ist ein Motor je URDF-Gelenk.
shortcut: Die Stangen des Parallelogramms stecken in den Gliedern, mit denen sie sich bewegen, nicht in eigenen Körpern.

Der Palettierer (`palletiser`, `urdf/palletiser.urdf`, etwa 20 kg Nutzlast, 1,6 m Reichweite) hat fünf URDF-Gelenke und
vier Motoren: den Drehturm, den Oberarm, den Winkel des Unterarms gegen die Basis (ein Parallelogramm von der Basis aus
treibt ihn: Gelenk 3 = Motor 3 - Motor 2) und die Drehung des Flanschs; das Handgelenk dreht den Winkel des Unterarms
zurück (Gelenk 4 = - Motor 3), so bleibt der Flansch überall waagerecht. Seine Gelenke 1 bis 3 haben den 1FK2106-4AF des
Katalogs an Antrieben 6SL3210-5HE15-0UF0 hinter Getrieben von 600 N m Nennmoment (eine Schätzung), der Flansch einen
1FK2104-5AF. Ausgeliefert im Werkzeugraum: x, y, z und die Drehung rz des Flanschs. Die App zeichnet ihn wie die Arme.

### Eine pendelnde Last

Mit `effects.payload_swing` (in jeder ausgelieferten Konfiguration aus) hängt die Last an einem Gelenk am Flansch, um zwei
Achsen quer zur Linie vom Flansch zu ihrem Schwerpunkt (`arm.payload_offset_m`, die Pendellänge), gehalten von
`arm.payload_swing_stiffness_nm_rad` und gedämpft von `arm.payload_swing_damping_nm_s_rad` je Achse: ein Karton in einem
weichen Vakuumgreifer oder ein Sack in einer Schlinge. Die beiden Pendelgelenke treibt kein Motor
(`chain::Mechanism::passive_joints`): Ihre Winkel und Geschwindigkeiten folgen den anderen Zuständen der Gelenkmaschine,
ihre Dynamik ist die der Kette mit den Körpern des Pendels, die Leistung des Dämpfers der Verlust
`power.payload_swing` und die Winkel die Signale `swing.1.angle`, `swing.2.angle`. Vorsteuerung und Ruhelagen sehen die
Last fest am Flansch. Der Karton des Palettierers pendelt mit 2,2 Hz (sqrt(k / (m l^2) + g / l) / 2 pi mit 120 N m/rad,
20 kg, 0,2 m).
shortcut: Ohne Last tragen die Pendelgelenke eine eigene Einheitsträgheit, getrennt vom Arm.

### Werkzeugraum

Mit `kinematics.tool_space = true` sind die Achsen der Maschine die Lage des Flanschs, und jeder Planer, der Zyklus, die
Kommandozeile und die App bewegen das Werkzeug wie beim Delta: Eine Gerade einer Bahn ist eine Gerade des Werkzeugs. Die
Gelenkmaschine rechnet die Lage über `inverse` des Mechanismus in die Gelenke und über `forward` zurück, die
Geschwindigkeit des Werkzeugs über die Jacobi-Matrix von `forward` (zentrale Differenzen) in die der Gelenke. Die
Vorsteuerung nimmt die Geschwindigkeitsterme hinzu: die Gelenkbeschleunigungen J^-1 (a - J' q'), J' q' als zweite
Differenz von `forward` entlang q', und die Zentripetal- und Coriolismomente des Newton-Euler-Modells. Ziele werden
abgelehnt außerhalb der Reichweite (`error-arm-out-of-reach`), jenseits eines Gelenkanschlags
(`error-joint-beyond-stop`) oder wo die Konditionszahl der Jacobi-Matrix über `workspace.max_condition_number` liegt
(`error-arm-singularity`); eine Bahn durch eine solche Lage mit `error-path-range`, wie beim Delta. Die
Arbeitsraumkarte und `ratings.max_condition_number` nehmen dieselbe Zahl. Der zeitoptimale Planer lehnt einen Plan von
mehr als zehn Millionen Punkten ab (`error-path-torque`): Wo die Motoren nur ein Kriechen zulassen, füllte er den
Speicher.

- SCARA (vier Gelenke: zwei Arme um senkrechte Achsen, eine senkrechte Pinole, ein drehender Flansch,
  `kinematics.tool_axes = ["x", "y", "z", "rz"]`): Die Achsen sind x, y, z des Flanschs (m) und seine Drehung c um die
  Senkrechte (rad: Gelenke 1 + 2 + 4 plus die Drehung bei null), in geschlossener Form. Längen und Richtungen der Arme
  bei null, der Ort von Gelenk 1, die Richtung der Pinole und die Drehung des Flanschs kommen aus der URDF bei null und
  werden an drei Stellungen gegen die Kette geprüft. Gelenk 2 nach dem Kosinussatz, cos(beta) = (r^2 - L1^2 - L2^2) /
  (2 L1 L2), mit `kinematics.elbow` "right" (beta > 0: Gelenk 2 vom gestreckten Arm aus gegen den Uhrzeigersinn, der
  Ellbogen von oben gesehen rechts der Linie von der Basis zum Werkzeug) oder "left"; Gelenk 1 = atan2(y, x) - atan2(L2
  sin beta, L1 + L2 cos beta). Die Konditionszahl ist die von d(x, y)/d(q1, q2): unendlich beim gestreckten und beim
  gefalteten Arm (25 bei 11 Grad Ellbogen, die Grenze des Beispiels).
- Jede andere Kette von 4 bis 7 Gelenken: Die Achsen sind die von x, y, z (m) und rx, ry, rz (rad), die
  `kinematics.tool_axes` nennt, so viele wie Motoren oder einer weniger (ein redundanter Arm). Die Orientierung ist der
  Drehvektor (Achse mal Winkel, im Basissystem) der Drehung gegen die Orientierung des Flanschs bei Gelenken null: Eine
  URDF, deren Flansch bei null nach unten zeigt, steht dort bei 0, nicht eine halbe Drehung gegen das Basissystem. Eine
  Lage bis auf 0,01 rad an einer halben Drehung dagegen wird abgelehnt (`error-arm-orientation-half-turn`): Dort springt
  der Drehvektor. Die Gelenke mit gedämpften kleinsten Quadraten auf der Jacobi-Matrix der Werkzeugachsen (zentrale
  Differenzen; dq = J^T (J J^T + lambda^2 I)^-1 e, lambda 1 mm, die Drehungen mal dem Abstand des Flanschs von Gelenk 2
  in `kinematics.home_joints`), entlang der Geraden von der letzten Lösung zum Ziel in Schritten von höchstens 20 mm
  und 0,05 rad, jeder auf 1 um eingeschwungen, der letzte auf 1e-13 m. Ein Weg von der letzten Lösung, der einen
  Schritt jenseits von `workspace.max_condition_number` beendet, kann den Zweig gewechselt haben und wird von der
  Grundstellung aus neu gegangen, ebenso einer, der nicht ankommt; so bleibt die Lösung auf dem Zweig der Grundstellung
  (Schulter, Ellbogen, Handgelenk). Von der letzten Lösung aus braucht eine Inverse entlang eines Zyklus beim
  Sechsachsroboter im Release 7,7 us, von der Grundstellung aus 74 us. Ein redundanter Arm (sieben Gelenke, sechs Achsen)
  nutzt sein überzähliges Gelenk, um der Grundstellung am nächsten zu bleiben: Jede Iteration bewegt sich auch im
  Nullraum von J, um (I - J^T (J J^T)^-1 J)(home - q), bis dieser Schritt unter 1e-11 rad liegt; seine
  Gelenkgeschwindigkeiten und -beschleunigungen sind die der Inversen selbst, durch zentrale Differenzen entlang der
  Geschwindigkeit und Beschleunigung des Werkzeugs. Die Konditionszahl ist die der gewichteten Jacobi-Matrix (beim
  redundanten Arm die ihrer Transponierten).

Eine Gerade einer Bahn in x, y, z, rx, ry, rz bewegt den Ort gerade und dreht die Orientierung per Slerp (`path`, die
sphärische Interpolation der Quaternionen, Shoemake 1985): um eine feste Achse mit gleichmäßiger Rate, auch zwischen
Drehvektoren, die nicht parallel sind, wo die Gerade zwischen ihnen eine andere Drehung ist (eine Vierteldrehung um x zu
einer um y: auf halbem Weg 70,5 Grad um ihre Winkelhalbierende, die Gerade 63,6 Grad). Geraden mit weniger
Orientierungsachsen (c des SCARA, rz des Palettierers, rx, ry des Fünfachsroboters) fahren jede Achse gerade.

`config/scara-tool.toml` und `config/arm-tool.toml` sind die Standardkonfigurationen mit eingeschaltetem Werkzeugraum,
einer Startlage und einem Zyklus aus Geraden (die Einschwingtoleranzen des Arms 5 mm und 5 mrad: Unter der Nutzlast ruht
sein Flansch 2,4 mm neben der Lage der Geber und um 3,2 mrad gedreht). `config/arm5-tool.toml` und `config/arm7-tool.toml` sind das
Fünfachsbeispiel (`urdf/five-axis.urdf`: der Sechsachsroboter ohne die Drehung des Flanschs, bei null nach unten zeigend;
Achsen x, y, z, rx, ry, wohin das Werkzeug zeigt) und das Siebenachsbeispiel (`urdf/seven-axis.urdf`: ein Drehgelenk im
Oberarm; Achsen x, y, z, rx, ry, rz), jedes mit einem Zyklus aus Geraden, für `--config` mit der Maschine `urdf-arm`.
Ihre Proportionen sind allgemein, jeder Wert eine Schätzung.

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
entlang einer Geraden unter ihrer Nenndrehzahl; beide Werkzeugzyklen bestehen ihre Prüfungen. Die Arme vom 14. November:
Die Vorsteuerung im Gelenkraum hat die Coriolis- und Zentripetalmomente eines Zweigelenkarms von Hand (-6 und 0,75 N m bei
q2 = 90 Grad, q' = (1, 2) rad/s); die eigene Trägheit einer Last zählt um das letzte Gelenk (1,2 N m gegen 1,15 der
Punktmasse bei 1 rad/s^2); der Slerp-Mittelpunkt zweier Vierteldrehungen ist 70,53 Grad um ihre Winkelhalbierende, und
eine geplante Gerade des Werkzeugs des Arms geht durch ihn; die warm gestartete Inverse gibt die Gelenke eines Wegs von
der Grundstellung (1e-9 rad); der Siebenachsroboter erreicht eine Lage auf 1e-9, der Abstand seiner Gelenke von der
Grundstellung ohne Anteil im Nullraum der Jacobi-Matrix; ein bei null nach unten zeigender Flansch steht bei keiner
Drehung; der Flansch des Palettierers bleibt über die Bereiche seiner Gelenke waagerecht, und sein pendelnder Karton hat
m g l (1 - cos 0,1) + k 0,1^2 / 2 = 0,79597 J um 0,1 rad gedreht und (m l^2 + I) / 2 = 0,775 J bei 1 rad/s, und hält
verlustfrei die Energie; jedes Gelenk der Arme hat seinen eigenen Katalogmotor; ein zeitoptimaler Plan eines Kriechens
wird jenseits von zehn Millionen Punkten abgelehnt.
Eine 1,6 m lange Spindel fällt bei `screw-speed` durch, und eine geforderte Lebensdauer über der der Spindel bei
`screw-life`. Mit Verlusten: Getriebemoment und Vorschubkraft der Zahnstange sind die Kraft an der Last durch 0,97, die
Lebensdauer der Spindel schrumpft bei 99 % Zuverlässigkeit um a1, und die Temperaturrückführung behält den Wirkungsgrad
von Spindel, Zahnstange und Schnecke.

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
