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
| `palletiser` | Vierachs-Palettierer, `urdf/palletiser.urdf`, ein Parallelogramm hält den Flansch waagerecht | Getriebe i = 160, das Wellgetriebe des Flanschs 50 | x, y, z, rz (ausgeliefert im Werkzeugraum), oder die vier Motorgelenke | `strain-wave-life` je Gelenk |
| `parallel-scara` | Fünfgelenk mit Parallelogramm-Unterarmen in der waagerechten Ebene (eigenes Crate, keine Gelenkmaschine); Z-Hub und Werkzeugdrehung an der Platte | Planetengetriebe i = 20 an den Armen; der Z-Hub über einen Kugelgewindetrieb 16 x 20 hinter einer Kupplung; die Drehung über ein Planetengetriebe i = 8 | x, y (m), z (m), c (rad) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` am Z-Hub |

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
| `disc-cam` | Kurvenscheibe: Grundkreis 60 mm, ein Rollenstößel von 2 kg (und ein Werkzeug von 1 kg), gegen seine Rückholfeder um 30 mm gehoben, 60 Umdrehungen je Minute | Planetengetriebe 10 | Scheibenwinkel (rad) | `disc-contact-force`, `disc-pressure-angle` |

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

**Kurvenscheibe.** Eine mechanische Kurvenscheibe: der Servo dreht die Scheibe, und die Scheibe hebt einen Rollenstößel
auf einer geraden Führung, von einer Rückholfeder (F0 + k s, `follower.spring_preload_n`, `spring_n_m`) und seinem
Gewicht auf ihr gehalten. Der Hub s(θ) des Stößels ist das Hubgesetz der Scheibe, Segmente der Gesetze der VDI 2143 über
eine Umdrehung (`[[disc.segment]]`, ein 3-4-5-Hub von 30 mm über 120°, eine Rast, der Rückhub, eine Rast). Die Achse des
Stößels liegt um e (`follower.offset_m`) neben der Scheibenmitte, seine Rolle vom Radius r_r rollt auf der Scheibe vom
Grundkreis r_b: der Rollenmittelpunkt liegt s0 + s über der Mitte, s0 = √((r_b + r_r)² − e²), und der Druckwinkel,
zwischen der Achse des Stößels und der gemeinsamen Normalen im Kontakt, ist tan φ = (s' − e) / (s0 + s), s' = ds/dθ (ein
positiver Versatz senkt ihn beim Hub). Die Kontaktkraft entlang der Normalen ist N = (m s̈ + F0 + k s + m g) / cos φ mit
s̈ = s''(θ) θ'² + s'(θ) θ''; die Führung nimmt N sin φ quer. Die Scheibe spürt J(θ) = J_Scheibe + m s'², den
Geschwindigkeitsterm m s' s'' θ'² und das Moment von Feder und Gewicht (F0 + k s + m g) s' (ihr Potential
F0 s + k s² / 2 + m g s): das Moment schwingt mit dem Hubgesetz und treibt die Scheibe beim Rückhub zurück. Signale: Hub,
Geschwindigkeit und Beschleunigung des Stößels, die Kontaktkraft, der Druckwinkel, die Führungslast und J(θ); geprüft
gegen `ratings.disc_max_contact_force_n` und `disc_max_pressure_angle_deg`. Mit `cam.run` dreht die Scheibe mit
`disc.turns_per_min` weiter, ihr Leitwert virtuell. Die Rolle bleibt auf der Scheibe: wo N unter null fiele, höbe der
Stößel ab, was das Signal zeigt und das Modell nicht trennt (eine so knapp ausgelegte Feder). Die Mitte des Bereichs der
Scheibe liegt auf der ersten Hälfte des Hubs, wo s'' > 0 ist und Feder und Gewicht die Scheibe dort halten, wo sie gegen
sie gehalten wird; hinter der Mitte des Hubs drehen sie sie weg.

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
über dieselbe Rampe herunter, und eine neue Leitgeschwindigkeit unter einer laufenden Kurvenscheibe (ein anderes
`web.speed_m_s`, angewandt, während sie läuft) fährt von der Geschwindigkeit des Leitwerts über `cam.ramp_s` auf die neue,
von dort, wo er steht: Weg v0 t + (v1 − v0) U(t), U die Rampe der Höhe eins (`cam::master_change`). Verlangt die
Kurvenscheibe bei der Leitgeschwindigkeit mehr Folgegeschwindigkeit, -beschleunigung oder -ruck, als die Achsgrenzen
erlauben, meldet das Programm beim Anwenden der Konfiguration `limit-cam-speed`, `-acceleration`, `-jerk` (wie die
Motorgrenzen) und prüft es als Urteile des Zyklus `cam-speed`, `cam-acceleration`, `cam-jerk`, je Folgeachse, gegen die
Achsgrenzen. Die Schubkurbel läuft mit konstanter Drehzahl (`crank.motion = "constant"`, eine
Umdrehung je Hub wie eine mechanische Presse mit Schwungrad) oder mit der Servopressen-Kurve aus `[[cam.segment]]`
(schnelle Annäherung, langsam durch die Umformzone, schneller Rückhub) bei `crank.strokes_per_min`.

Die mitgelieferten Konfigurationen lassen `cam.run = false`: der Zyklus schwenkt die Kurbel vom oberen Totpunkt in den
unteren und zurück (Servopresse im Pendelhub), taktet das Messer um eine halbe Umdrehung hin und zurück und fährt den
Sägewagen über seinen Hub und dreht die Kurvenscheibe eine halbe Umdrehung hin und zurück. Neben jeder liegt
`config/<id>-cam.toml`, dieselbe Konfiguration mit `cam.run = true`, die jedes Urteil besteht (die Grenzen der
Kurvenscheibe darunter); oder die Kurvenscheiben laufen mit `--set cam.run=true` (App: der Schalter der Gruppe
Elektronische Kurvenscheibe):

```text
sim-cli verdicts slider-crank --config crates/machine-crank-cam/config/slider-crank-cam.toml
sim-cli verdicts slider-crank --set cam.run=true
sim-cli analyze rotary-knife --set cam.run=true --set knife.format_length_m=0.4
sim-cli verdicts flying-saw --set cam.run=true --set material.speed_m_s=1.2
```

Moment und Schleppfehler kommen wie bei jeder Maschine aus dem Antriebsmodell; die Formeln der Kurvenscheiben und
Mechanismen sind in `cam.rs` und den Tests von `machine-crank-cam` von Hand nachgerechnet (unten).

## Fünfgelenk in der waagerechten Ebene (parallel-scara)

Der Parallel-SCARA `parallel-scara` (eigenes Crate, keine Gelenkmaschine) ist ein liegendes Fünfgelenk: zwei Arme
drehen um senkrechte Achsen bei x = ±R, jeder trägt ein Unterarmpaar (ein Parallelogramm, die Platte behält ihre
Ausrichtung) zur Platte, wie bei Mitsubishis RP-Baureihe. Kinematik und Dynamik sind die des planaren Deltas D2, flach
gelegt (in das Crate kopiert: eine Maschine darf das Crate einer anderen nicht benutzen); die Koordinaten der Platte
sind x und y. Die Schwerkraft wirkt entlang -z, aus der Ebene heraus, auf die Gelenklager: die Motoren der Arme tragen
kein Gewicht, sie beschleunigen nur die Platte und was sie trägt, und halten im Stillstand nichts. An der Platte lassen
sich zwei Achsen zuschalten: ein Z-Hub (`[z_axis]`, Motor 3), eine Pinole auf einem Kugelgewindetrieb hinter einer
Kupplung, die Schlitten und Nutzlast gegen die Schwerkraft hält (m g Steigung / 2 pi, 0,187 N m für 6 kg bei 20 mm
Steigung), und eine Drehung des Werkzeugs um z (`[tool_turn]`, Motor 4) hinter einem eigenen Getriebe. Jede hat eigene
Tabellen für Motor, Getriebe, Antrieb und Grenzwerte (`[motor.3]`, `[gearbox.4]`, ...). Alles an der Platte (die
Einheiten mit ihren Motoren, der Schlitten, die Nutzlast) zählt zur Masse der Platte in der Ebene; Hub und Drehung
stehen senkrecht zur Ebene und koppeln sonst nicht mit ihr. Die Werkzeugpose ist (x, y), (x, y, z), (x, y, c) oder
(x, y, z, c), wie die beiden Schalter es sagen. Hub und Drehung sind starre Gelenke: Verdrehung und Spiel von Spindel
und Drehgetriebe sind nicht modelliert. Siehe die [Modellreferenz](../model-reference.md).

## Zweiteilige Plattform mit Zahnstangen-Übersetzung (delta-h4)

Der Vierarm-Delta `delta-h4` (ein eigenes Crate, keine Gelenkmaschine) ist von der Bauart Quattro / H4 (Pierrots H4,
der Adept/Omron Quattro): vier Arme auf der Basis im Abstand von 90 Grad, jeder mit einem Parallelogramm aus Unterarmen,
hinab zu einer bewegten Plattform aus zwei Teilen. Die Arme 1 und 2 tragen Teil A, die Arme 3 und 4 Teil B: zwei
benachbarte Arme, deren Ellbogenachsen rechtwinklig zueinander stehen, sodass sich kein Teil drehen kann (ein
Parallelogramm lässt seinem Plattformende eine Drehung, um die Achse seines Ellbogens; die Achsen zweier gegenüber
liegender Arme sind parallel und ließen ein Teil um sie drehen). Die Teile verschieben sich gegeneinander entlang der
Waagerechten d = (1, -1, 0) / sqrt 2, jedes um die halbe Verschiebung h vom Werkzeugpunkt in die andere Richtung; entlang
der Linie zwischen ihnen, d = (1, 1, 0) / sqrt 2, bewegte die Verschiebung die vier Unterarmenden wie das Absenken der
Plattform in der Mitte, und die Jacobi-Matrix wäre dort singulär. Ein Ritzel zwischen je einer Zahnstange an jedem Teil
rollt um h / r_p (seine Mitte, auf einem Schlitten zwischen den Teilen geführt, bleibt im Werkzeugpunkt), und eine
Zahnradstufe mit Übersetzung i_p dreht das Werkzeug: c = kappa h, kappa = i_p / r_p (ausgeliefert 30 rad/m: ±180 Grad
sind h = ±104,7 mm).

Die Kinematik ist je Bein die des D3 am Unterarmende p -+ h d (geschlossene Rückwärtsrechnung; vorwärts nach Newton auf
den vier Bedingungen, ausgehend von der Trilateration des D3), die Dynamik die des D3 nach dem Prinzip der virtuellen
Arbeit mit der Jacobi-Matrix jedes Körpers: die beiden Teile, Schlitten und Nutzlast in p und die Drehung des Werkzeugs
I_t c'^2 / 2 = I_t kappa^2 h'^2 / 2. Dieser letzte Term prägt die Maschine: Das Trägheitsmoment des Werkzeugs zählt in
der Verschiebung mal kappa zum Quadrat (die 0,013 kg m^2 von Werkzeug und Nutzlast als 11,7 kg), und Verdrehung und
Spiel der Armgetriebe zählen in der Drehung des Werkzeugs mal kappa. Mit `effects.amplifier_compliance` ist das Werkzeug
ein eigener Körper an der Feder der Übersetzung (mit `effects.amplifier_backlash` mit einer Totzone darin), wie das
Werkzeug des D4 an seiner Welle. Siehe die Konfigurationsreferenz (docs/config-reference.md, Abschnitt „Quattro-type
delta H4“).

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

Die Kurve der Presse wechselt die Geschwindigkeit mit 4-5-6-7-Gesetzen: ein 3-4-5 in die langsame Zone verlangt an seinen
Enden 6 Δv / Δm² + 60 c / Δm³ = 67,4 rad/rad³ Ruck (Δv 1,468, Δm 0,5 rad, c −0,067 rad), bei π rad/s Leitwert
2090 rad/s³, mehr als die 2000 der Kurbel; das 4-5-6-7 hat dort keinen eigenen Ruck. Die Drehzahlregler der vier
Maschinen haben Tn 0,01 s (das Messer Kp 0,25): der Integrator nimmt die Reibung am Ende einer Fahrt dreimal so schnell
auf wie mit 0,03 s, und die Fehler beim Verweilen fallen auf 0,13 mrad (Kurbel), 0,16 mrad (Messer), 0,14 mm (Säge) und
0,23 mrad (Scheibe), mit mindestens 35,8° und 8 dB Reserve in jedem Lauf über die Grenzen (`sim-cli margins <id>
--bands`); die Toleranzen des Einschwingens sind 0,3 mrad, 0,3 mrad, 0,25 mm und 0,5 mrad.

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
Grenze von 10 rad/s wird mit 15,17 rad/s gemeldet und fällt dort beim Urteil `cam-speed` durch. Eine neue
Bahngeschwindigkeit unter der laufenden Kurvenscheibe des Messers fährt den Leitwert von 1 auf 0,8 m/s: 0,9 m/s in der
Mitte der Rampe, 0,8 m/s nach 0,5 s, 0,45 m auf dem Weg, ohne Sprung der Geschwindigkeit. Die Kurvenscheibe bei 60°
(Mitte eines 3-4-5-Hubs von 30 mm über 120°, r_b 60 mm, r_r 15 mm, e 10 mm): s0 = 0,0743303 m, s' = 0,0268574 m/rad,
tan φ = 0,188709, φ = 10,687°; bei 2π rad/s drücken Feder und Gewicht mit 159,420 N, die Kontaktkraft ist 162,234 N, die
Führungslast 30,084 N und das Moment an der Scheibe 4,28161 N m.

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
- Das Förderband aus `[conveyor]` kann noch nicht Leitwert einer Kurvenscheibe sein (der Leitwert ist ein eigener, mit
  dem Geschwindigkeits- und Rampenmodell des Bands); eine neue Leitgeschwindigkeit beginnt ihre Rampe bei der
  Geschwindigkeit des Leitwerts ohne seine Beschleunigung (ein Sprung der Beschleunigung, wo sie während einer Rampe
  kommt); das Gelenk von Kurbel, Messer und Kurvenscheibe hat keine Anschläge, solange die Kurvenscheibe es weiterdreht;
  der Stößel der Kurvenscheibe ist ein geradegeführter (kein Schwinghebel) und hebt nie von der Scheibe ab; die Presskraft wirkt
  nur beim Ausfahren (keine Rückfederung), die Schnittkraft von Messer und Säge nur vorwärts.
- Der Linear-Delta (`delta-linear`, drei Riemenachsen an Säulen, die sich neigen können,
  `geometry.column_incline_deg`) ist eine Parallelkinematik und keine dieser Maschinen; siehe die
  [Modellreferenz](../model-reference.md#linear-delta).

- Noch nicht gebaut: mechanische Kurvenscheiben, elektronische Kurvenscheiben und Profile für die fliegende Säge.
- Hexapod: Das Drehen der Beine um ihre Basisgelenke und die Trägheit der Nutzlast fehlen; eine Steifigkeit je Bein
  (an der weichsten Stellung der Mutter); noch keine Eigenfrequenzen (`resonance_hz`).
- Seilroboter: Masse und Durchhang der Seile fehlen; der kraftfähige Arbeitsraum ist der statische (die Beschleunigungen
  des Zyklus beurteilen die Seilkraftprüfungen, die Planung weicht ihnen nicht aus); nahe am Rand des Arbeitsraums rechnet
  der Dehnungsausgleich der Motoren mit den Kräften der geschlossenen Form, wo die Verteilung ein Seil an einer Grenze
  hält.
