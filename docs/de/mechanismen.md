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
- [R-Theta-Z-Arm mit Bändern (wafer-handler)](#r-theta-z-arm-mit-bändern-wafer-handler)
- [Parallelkinematiken: Hexapod und Seilroboter](#parallelkinematiken-hexapod-und-seilroboter)
- [Pneumatikzylinder neben Servoachsen](#pneumatikzylinder-neben-servoachsen-machinecylinder)
- [Auf- und Abwickler mit Tänzer](#auf--und-abwickler-mit-tänzer-machine-winder)
- [Brückenkran](#brückenkran-machine-overhead-crane)
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
| `arm-on-track` | der Sechsachsroboter auf einem Schlitten von 80 kg auf einer Bodenschiene von 4 m, `urdf/arm-on-track.urdf` | die Schiene: ein Ritzel auf einer Zahnstange hinter einem Planetengetriebe i = 10; der Arm wie `urdf-arm` | die Schiene (m) und die sechs Gelenke (rad), oder im Werkzeugraum die Schiene neben x, y, z, rx, ry, rz | `rack-feed-force`, `gearbox-radial-load` an der Schiene; `guide-load`, `guide-life` ihrer Führungswagen; `strain-wave-life` je Armgelenk |
| `parallel-scara` | Fünfgelenk mit Parallelogramm-Unterarmen in der waagerechten Ebene (eigenes Crate, keine Gelenkmaschine); Z-Hub und Werkzeugdrehung an der Platte | Planetengetriebe i = 20 an den Armen; der Z-Hub über einen Kugelgewindetrieb 16 x 20 hinter einer Kupplung; die Drehung über ein Planetengetriebe i = 8 | x, y (m), z (m), c (rad) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` am Z-Hub |
| `wafer-handler` | R-Theta-Z-Arm aus zwei gleich langen Gliedern in der waagerechten Ebene, Unterarm und Effektor über Bänder 2:1 und 1:2 an die Schulter gekoppelt (eigenes Crate, keine Gelenkmaschine); eine Säule, die sich dreht und hebt | Wellgetriebe i = 50 an Säule und Oberarm; der Hub über einen Kugelgewindetrieb 16 x 10 hinter einer Kupplung | r (m), theta (rad), z (m) | `wafer-slip`, `effector-direction`, `elbow-band-slack`, `wrist-band-slack`; die Prüfungen der Spindel am Hub |

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
  Reihe mit der von Spindel oder Zahnstange. Die der Spindel ist ihre Welle zwischen Festlager und Mutter, E A / a, in
  Reihe mit Mutter und Lagern: mit `effects.screw_stiffness_with_position` an der Stelle der Mutter
  (`screw.fixed_bearing_at_m` der Weg am Festlager), sonst an der weichsten Stelle für den ganzen Hub;
- Dämpfung: die des Getriebes durch Hebel^2 nur über der Feder des Getriebes, die von Spindel oder Zahnstange nur über
  deren Feder, c = 2 zeta sqrt(K_c m) aus `screw.axial_damping_ratio` oder `rack.damping_ratio` und der bewegten Masse m
  (Mutter, Lager, Führungen); zwei gedämpfte Federn in Reihe, die Maschine führt dafür einen Zustand mehr
  (`Gearbox::arm`). Mit der Dämpfung des Getriebes allein (K_g = k_Getriebe / Hebel^2) schwingt eine Masse daran bei
  tiefen Frequenzen mit c (K_c / (K_c + K_g))^2 gedämpft, nicht mit c;
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
Planetengetriebe davor hat seine eigene (`gearbox-life`). `pressure_angle_deg` (Normalschnitt, optional, 20 nach DIN 867,
wo er fehlt, unter 45) ergibt die Kraft, die Ritzel und Zahnstange auseinanderdrückt, F tan(alpha) / cos(beta), die die
Radiallast des Ritzels auf der Getriebewelle mitzählt.

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
`builtin:seven-axis`, `builtin:palletiser` oder `builtin:arm-on-track` für die Beispiele in `crates/machine-joint-kit/urdf/`. `gear_ratios` hat
einen Eintrag je Motor; die Grenzen jedes Motorgelenks stehen in `[axis.j1]`, `[axis.j2]`, ... (`max_velocity_rad_s`,
für ein Schiebegelenk `max_velocity_m_s`, und so weiter). Die Grenzen der URDF sind die Anschläge der Gelenke; ein
Gelenk vom Typ continuous nimmt `min_angle_deg` und `max_angle_deg` aus `[joint]`. Ein Schiebegelenk läuft auf der
`[screw]`, mit seinem Motor gekuppelt (seine Übersetzung muss 1 sein; Steifigkeit und Dämpfung des `[gearbox]` stehen
für die Kupplung, sein Wirkungsgrad und seine Trägheit zählen nicht), mit der Reibung aus `[slide]` (die Pinole des
SCARA); wo die Konfiguration eine `[rack]` hat, läuft es stattdessen auf einem Ritzel auf dieser Zahnstange hinter seinem
eigenen Getriebe (Übersetzung aus `gear_ratios`, beliebig), wie die Zahnstangenachse, mit `rack-feed-force` und der
Radialkraft des Ritzels auf die Getriebewelle (`rack.<k>.radial_load`, die Zahnkraft mal sqrt(1 + (tan 20° / cos
beta)^2), gegen `ratings.shaft_max_radial_load_n`: `gearbox-radial-load`, wie bei den Zahnstangen der Maschinendateien).
Eine fehlende Grenze der URDF ist 0; ein leerer Bereich wird abgelehnt. Mit einer Tabelle `[strain_wave]` bekommt
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
Last fest am Flansch. Der Karton des Palettierers pendelt mit 1,61 Hz
um die Achse quer zu seiner 0,6-m-Seite und mit 1,82 Hz um die andere (sqrt((k + m g l) / (m l^2 + I)) / 2 pi mit 120 N m/rad,
20 kg, 0,2 m und dem Eigenträgheitsmoment I = 0,75 und 0,4167 kg m^2 des Kartons: 159,2 / (0,8 + 0,75), 159,2 / (0,8 + 0,4167)).
shortcut: Ohne Last tragen die Pendelgelenke eine eigene Einheitsträgheit, getrennt vom Arm.

### Ein Roboter auf einer Schiene

`arm-on-track` (`urdf/arm-on-track.urdf`, `config/arm-on-track.toml`) ist der Sechsachsroboter des Beispiels auf einem
Schlitten, der 4 m entlang x auf zwei Schienen fährt: ein Schiebegelenk zuerst, dann die sechs des Arms. Der Schlitten
(80 kg: Platte, Sockel, Führungswagen, Motor, Getriebe und Ritzel der Schiene) ist ein Körper der Kette wie jedes Glied,
die Schiene also ein Gelenk mehr im Newton-Euler-Modell, und die Kopplung zwischen Schiene und Arm steckt in den
Gleichungen der Kette: Der schwenkende Arm schiebt an der Schiene (Gelenk 2 mit 3 rad/s² bei quer zur Schiene
ausgestrecktem Arm verlangt von der Schiene -3 · Σ m y = -43,5 N, Σ m y = 14,505 kg m von Arm und Last), und die
Beschleunigung der Schiene erreicht jedes Gelenk des Arms (bei 2 m/s² hält Gelenk 2 -2 · 14,505 = -29,0 N m). Der Motor
der Schiene ist der 1FK2105-4AF des Katalogs an einem S210, hinter einem Siemens NRB080 mit i = 10 (Katalogwerte, ohne
sein Spiel für ein vorgespanntes Ritzel) und dem Ritzel der Zahnstangenachse mit 200 mm je Umdrehung; 130,5 kg fahren
darauf mit bis zu 1 m/s und 3 m/s².

`[track]` setzt den Schlitten auf vier Führungswagen, `block_spacing_m` (0,4 m) längs der Schienen auseinander und
`rail_spacing_m` (0,5 m) quer. Ihre Last kommt aus der Reaktion der Kette an ihrem ersten Gelenk: was die Basis an die
Kette gibt, F = dP/dt + Σ m g z und M_O = dH_O/dt + Σ c × m g z (die Änderungen von Impuls P und Drehimpuls H der Kette
um den Ursprung der Basis als zentrale Differenzen), in das System des Schlittens zwischen den Wagen gebracht. Sein
Schub längs der Schienen ist der der Zahnstange; den Rest teilen die Wagen als starre Lager, jeder F_z / 4 ± M_x / (2 w)
∓ M_y / (2 l) auf seiner Schiene und F_y / 4 ± M_z / (2 l) quer dazu, seine Last die Summe beider Beträge (die
äquivalente Last des Katalogs). Die Seite der Zahnstange (`track.rack_side`, ±1 längs der y-Achse des Schlittens) bestimmt, auf welcher Seite die Abstoßkraft des Schrägritzels zu F_y kommt, ihr Hebelarm (`track.rack_lever_m`) macht aus dem Schub des Antriebs ein Giermoment auf die Wagen, M_z + Seite · Hebel · F_x; die Vorgaben +1 und 0 sind die Lasten, wie sie bisher gezählt wurden. Das Signal `axis.j1.guide_load` ist die Last des höchstbelasteten Wagens mal vier, damit
die Führungsurteile, die eine Last gleich auf einen Satz von vier verteilen und den Satz als einen bewerten
(`parts.axis = "<wagen>x4"`), diesen Wagen beurteilen: `guide-load` gegen `ratings.guide_max_load_n`, `guide-life` aus
`ratings.guide_dynamic_load_rating_n` und der Geschwindigkeit der Schiene. Die Momente entscheiden die Größe: Auf dem
Standardzyklus trägt der höchstbelastete Wagen in der Spitze 895 N gegen die 320 N, die ein Viertel des Gewichts gibt.
Ausgeliefert: vier HIWIN HGH20CA (Katalog), 3580 N gegen 73360 N, eine Lebensdauer von 1,4 Millionen Stunden.

Redundanz: sieben Gelenke für eine Werkzeuglage aus sechs. Im Werkzeugraum (`config/arm-on-track-tool.toml`) ist die
Stellung der Schiene eine eigene Achse, `"j1"` vor denen der Lage in `kinematics.tool_axes`: sieben Achsen, die Schiene
vorgegeben und die sechs Gelenke des Arms mit gedämpften kleinsten Quadraten für die Lage des Flanschs gelöst, bei der
Schiene, wo sie steht. Eine Gerade einer Bahn bewegt Schiene und Flansch zusammen, jedes gerade; hält der Flansch,
während die Schiene fährt, greift der Arm zurück. Der Standardzyklus in Gelenken greift bei 0,5 m links des Arms und legt
bei 3,5 m rechts ab, der Arm schwenkt herum und greift aus, während die Schiene fährt; der Zyklus im Werkzeugraum greift
und legt mit Geraden des Flanschs ab, die Schiene trägt ihn 3 m, während der Flansch von 0,9 m vor und 0,6 m links des
Arms zu 0,7 m vor und 0,3 m rechts geht, 150 mm hoch und um 0,5 rad gedreht.

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
  `kinematics.tool_axes` nennt, so viele wie Motoren oder einer weniger (ein redundanter Arm); `"j1"` davor macht die
  Stellung des ersten Gelenks zu einer Achse (die einer Schiene: siehe
  [Ein Roboter auf einer Schiene](#ein-roboter-auf-einer-schiene)). Die Orientierung ist der
  Drehvektor (Achse mal Winkel, im Basissystem) der Drehung gegen die Orientierung des Flanschs bei Gelenken null: Eine
  URDF, deren Flansch bei null nach unten zeigt, steht dort bei 0, nicht eine halbe Drehung gegen das Basissystem. Mit
  zwei der drei Drehungen (rx, ry des Fünfachsarms) sagen sie, wohin das Werkzeug zeigt: die kürzeste Drehung von der
  Richtung des Flanschs entlang der fehlenden Achse bei Gelenken null zu der, in die es jetzt zeigt, ihr fehlender Teil
  0, wie auch immer die Gelenke das Werkzeug um diese Richtung drehen; die Winkelprüfungen des Flanschs beurteilen nur
  diese Drehung, und eine Bewegung hält sie mit der fehlenden Drehung bei 0 ein. Eine Lage bis auf 0,01 rad an einer halben Drehung dagegen wird abgelehnt (`error-arm-orientation-half-turn`): Dort springt
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
`link_bending.directions` fügt die Biegung quer zu dieser Ebene (`out_of_plane`) und die Verdrehung um die Länge des
Glieds hinzu (`torsion`, Schubmodul `link_bending.shear_modulus_pa`), je eine erste Eigenform, die Körper dahinter als
Masse und Trägheit an der Spitze. Mit `effects.link_bending_reaction` wirkt die Biegung zurück: Gelenke, Getriebe und
Glieder sind ein gekoppeltes System, die Motoren spüren das Schwingen der Glieder und regen es über ihre Regelkreise an,
Energie und Dämpfung der Biegung gehören zur Maschine, und `modes` gibt die gekoppelten Eigenformen bei festgehaltenen
Motoren (ein Glied auf einem Getriebe schwingt wie die zwei Federn in Reihe).

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
| `toggle-press` | Kniehebelpresse: Hebel von 150 mm in Linie zwischen Kopfstück und Stößel, eine Kurbel von 50 mm drückt das Knie über ein Pleuel von 250 mm, Stößel 20 kg und Werkzeug 8 kg, Hub 57,7 mm, Nennkraft 20 kN, Presskraft bis 12 kN | Planetengetriebe 10 | Kurbelwinkel (rad), 0 im Totpunkt der Hebel | `press-capacity`, `upper-link-force`, `lower-link-force`, `rod-force`, `frame-deflection` |
| `geneva-drive` | Rundschalttisch mit 6 Stationen über ein Malteserkreuz mit 6 Schlitzen, Kurbel 60 mm, Achsabstand 120 mm; Tisch 0,4 kg m² mit sechs Teilen zu 0,5 kg auf 200 mm; ein Schritt je Sekunde | Planetengetriebe 20 | Kurbelwinkel (rad), 0 in der Mitte eines Schritts | `indexer-contact-force`, `index-accuracy`, `index-settling` |
| `cam-indexer` | derselbe Tisch über ein Kurvenschrittgetriebe (Rollenkurve, modifizierter Sinus über 120° der Kurvendrehung, Rollen auf 80 mm) | Planetengetriebe 20 | Kurvenwinkel (rad), 0 in der Mitte eines Schritts | `indexer-contact-force`, `index-accuracy`, `index-settling` |

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
`disc.turns_per_min` weiter, ihr Leitwert virtuell. Mit `effects.follower_lift_off` hebt der Stößel ab, wo N unter null fiele: sein Spalt
g zur Scheibe ist ein passives Gelenk (Hub s + g), der Kontakt der Linienkontakt der Stahlrolle (nach Palmgren d =
3,84e-5 Q^0,9 / l^0,8 in mm und N über ihre Breite l, `follower.roller_width_m`: 4e8 N/m bei 160 N) mit einem Dämpfer bei
`contact_damping_ratio` der kritischen Dämpfung mit dieser Steifigkeit, nur auf Druck, die Kraft auf die Scheibe also nie
negativ und ohne Sprung bei der Berührung, dazu wirkt die Reibung der Führung (`follower.guide_friction_n`); in Ruhe ist
die Rolle um ihre Last eingedrückt (0,45 µm). Die Leistung des Dämpfers wird als Verlust gebucht
(`power.follower_contact`), die Bilanz schließt über einen Aufprall. Signale `follower.gap` (negativ: eingedrückt) und
`follower.off_disc` (der Kontakt trägt nichts). Aus, bleibt die Rolle auf der Scheibe, und nur das Signal zeigt, wo N
unter null fiele. Die Mitte des Bereichs der
Scheibe liegt auf der ersten Hälfte des Hubs, wo s'' > 0 ist und Feder und Gewicht die Scheibe dort halten, wo sie gegen
sie gehalten wird; hinter der Mitte des Hubs drehen sie sie weg.

**Kniehebelpresse.** Zwei Hebel in Linie zwischen dem Drehpunkt O im Kopfstück und dem Stößel, der obere a, der untere
b, ihr Gelenk das Knie; eine Kurbel vom Radius r drückt das Knie über ein Pleuel der Länge c zur Seite. Eine Kurbel und
kein Kugelgewindetrieb: der kurbelgetriebene Kniehebel ist die klassische Form und behält den drehenden Antrieb und die
Kurvenscheibe der Kiste; der spindelgetriebene Kniehebel (das Knie an der Mutter) ist nicht gebaut. Mit dem oberen Hebel
um φ aus der Senkrechten steht der Stößel x = a cos φ + √(b² − a² sin² φ) unter O (die Formel der Schubkurbel, der obere
Hebel ihre Kurbel), x = a + b, wo die Hebel gestreckt stehen (φ = 0, der Totpunkt). Das Gelenk ist der Kurbelwinkel q von
diesem Totpunkt; der Mittelpunkt der Kurbel liegt so, dass das Pleuel dort waagerecht liegt, die Kurbel um
`crank.angle_at_dead_centre_deg` zu ihm (90: quer, die Übersetzung fällt linear in den Totpunkt; gegen 180 in Linie,
eine lange flache Rast). Das Knie schließt das Viergelenk O–Knie–Kurbelzapfen–Kurbelmitte: φ aus den zwei Kreisen in
geschlossener Form, φ' = D·P_q / D·K_φ und φ'' = −(E·E + D·(K_φφ φ'² − P_qq)) / D·K_φ aus der festen Länge des Pleuels
(D = K − P, E = K_φ φ' − P_q), also x' = x_φ φ' und x'' = x_φφ φ'² + x_φ φ''. Die Übersetzung x' (m/rad) ist im
Totpunkt null: ein Kurbelmoment τ hält am Stößel die Kraft τ / x', die in den Totpunkt hinein ohne Grenze wächst. Die
Umkehrung von der Stößelhöhe hat zwei Äste, vor und nach dem Totpunkt (`TogglePress::crank_angle`); die Anschläge
(`joint.max_angle_deg`) sagen, ob die Kurbel durch ihn hindurch darf. Die Massen: Kurbel und Pleuelende am Zapfen, der
obere Hebel ein gleichförmiger Stab (m a² / 3 um O), das Knie mit dem anderen Pleuelende und einem Anteil des unteren
Hebels, der Stößel mit Werkzeug und dem Rest: J(q) = J_Kurbel + m_Zapfen r² + (m_oben / 3 + m_Knie) a² φ'² + m_x x'²,
mit seinem Geschwindigkeitsterm und dem Moment der Gewichte. Die Prozesskraft (`[process]`, über der Höhe des Stößels
über dem unteren Totpunkt) drückt den Stößel hoch, solange er abwärts fährt oder steht, so dass sie durch eine Rast im
Totpunkt hält, und klingt über `process.smoothing_m_s` ab, wenn er steigt. Das Gestell federt unter ihr auf
(`effects.frame_stretch`, `frame.stretch_stiffness_n_m` = k), eine Feder in Reihe: der Stößel bleibt F / k vor dem Punkt
stehen, an den ihn die Hebel stellen, und die Kraft ist die Wurzel von F = Tabelle(d + F / k), Abschnitt für Abschnitt
der Tabelle gelöst. Signale: die Höhe des Stößels über dem unteren Totpunkt (mit der Auffederung), seine Geschwindigkeit
und Beschleunigung, die Übersetzung x', die Presskraft, das Kraftvermögen in der Stellung und sein Anteil, die
Auffederung des Gestells, die Kräfte der beiden Hebel (Zweigelenkstäbe, ihre Bolzen tragen dasselbe) und des Pleuels,
und J(q). Prüfungen: `press-capacity`, die Presskraft in jeder Stellung innerhalb von
min(`ratings.rated_press_force_n`, τ_max / |x'|), τ_max das Spitzenmoment des Antriebs über Übersetzung und
Wirkungsgrad des Getriebes (höchstens `gearbox_max_torque_nm`): die Nennkraft nur über die letzten Millimeter, mit dem
Abstand vom Totpunkt fallend, so dass eine Umformkraft früh im Hub sie verfehlt (und `motor-peak` des Motors);
`upper-link-force` und `lower-link-force` gegen `ratings.link_max_force_n`, `rod-force` gegen `rod_max_force_n`,
`frame-deflection` gegen `frame_max_deflection_m`. Der mitgelieferte Zyklus fährt vom oberen Totpunkt des Stößels (die
Kurbel in Linie mit dem Pleuel, wo sie das Gewicht des Stößels ohne Moment hält) in einer Bewegung in den Totpunkt,
wobei die Übersetzung den Stößel über die letzten Millimeter verlangsamt, rastet dort unter 11,5 kN, während der Motor fast nichts gibt, und fährt zurück;
`config/toggle-press-cam.toml` tut dasselbe mit einer Kurvenscheibe bei 30 Hüben je Minute.

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
Sägewagen über seinen Hub, dreht die Kurvenscheibe eine halbe Umdrehung hin und zurück und fährt den Stößel der
Kniehebelpresse vom oberen Totpunkt in den unteren und zurück. Neben jeder liegt
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

## R-Theta-Z-Arm mit Bändern (wafer-handler)

Der Wafer-Handler `wafer-handler` (eigenes Crate, keine Gelenkmaschine) ist der R-θ-Z-Arm der Halbleiter- und
Flachbildschirmfertigung. Eine Säule hebt (z, Motor 3, Kugelgewindetrieb 16 x 10 hinter einer Kupplung) und dreht
(θ, Motor 1) einen Arm aus zwei Gliedern derselben Länge L = 0,32 m. Motor 2 fährt auf der Säule mit und dreht den
Oberarm um φ, seinen Winkel zur radialen Linie. Im Oberarm läuft ein Stahlband von einer Scheibe, die fest an der Säule
sitzt, zu einer Scheibe des halben Radius am Unterarm: Der Unterarm dreht sich gegen den Oberarm um −2 φ. Im Unterarm
dreht ein Band von einer Scheibe am Oberarm zu einer doppelt so großen am Effektor diesen gegen den Unterarm um φ. So
bleibt das Handgelenk auf der radialen Linie, 2 L cos φ von der Säulenachse, und der Effektor zeigt weiter entlang
dieser Linie: Ein Motor fährt den Wafer gerade hinein und heraus (r = 2 L cos φ + e, e = 0,18 m vom Handgelenk zur
Wafermitte). Die Werkzeuglage ist (r, θ, z).

dr/dφ = −2 L sin φ ist am größten, wenn das Handgelenk über der Säule steht (φ = 90°), und verschwindet bei voller
Streckung (φ = 0), der singulären Lage: Über `workspace.max_condition_number` (5: φ über 11,5°, r bis 0,807 m) wird die
Konditionszahl 1 / sin φ abgelehnt, die Anschläge des Oberarms liegen bei 10° und 110°. Das Trägheitsmoment am R-Motor
ändert sich stark mit der Reichweite: Die 0,55 kg m² der Glieder bei r = 0,12 m sind 1,35 kg entlang r, ihre 0,19 kg m² bei
r = 0,75 m 2,22 kg (und gegen die volle Streckung ohne Grenze); der θ-Motor spürt den ganzen Arm, bei r = 0,12 m
0,15 kg m², bei r = 0,75 m 0,60 kg m², mit r² wachsend. Die beiden Motoren sind über die Bewegung des Arms gekoppelt:
Das Drehen zieht den Arm nach außen (Motor 2 hält ihn, der Wafer wird mit r θ̇² zur Achse beschleunigt), das Ausfahren
während einer Drehung bremst Motor 1 (die Coriolis-Beschleunigung des Wafers 2 ṙ θ̇).

Der Wafer liegt auf den Reibpads des Effektors (oder auf Vakuumpads, oder ein Kantengreifer hält ihn, `wafer.hold`).
Seine Beschleunigung in der Ebene über den Zyklus ist das Prüfurteil `wafer-slip`, gegen μ g (μ (g + A p / m) auf
Vakuumpads, die Beschleunigung des Greifers an der Kante) über einen Sicherheitsfaktor: Das begrenzt die Taktzeit eines
Handlers. Der ausgelieferte Zyklus zwischen einem Load Port und einer Kammer, 90° auseinander bei r = 0,55 m und 0,2 m
übereinander (zur Kammer drehen und dabei heben, ausfahren und einschwingen, den Wafer absetzen, einfahren, beim
Zurückdrehen senken, unter den nächsten Wafer ausfahren und einschwingen, ihn anheben, einfahren: 7,76 s) ist darauf
ausgelegt: 0,3 g über 1,2 erlauben 2,45 m/s² (2,04 an der unteren Grenze von μ), die Reichweite ist auf 1,5 m/s²
begrenzt, die Drehung auf 4 rad/s und 36 rad/s² mit dem Wafer auf r = 0,05 m nahe an die Säulenachse zurückgezogen
(0,8 m/s² nach außen und 1,8 m/s² entlang); der Wafer erreicht 1,83 m/s². Die Bänder sind Federn mit Vorspannung (jedes Trum
k = 9,4e5 N/m, 150 N): Ihre Dehnung lässt Unterarm und Effektor hinter Säule und Oberarm zurückbleiben, der Wafer bleibt
in r zurück, und der Effektor dreht sich aus seiner radialen Linie (`effector.direction_error`; beim Verweilen an einer
Station geprüft, `effector-direction`); ein Trum, das seine ganze Vorspannung verliert, wird schlaff (`elbow-band-slack`,
`wrist-band-slack` prüfen den Ausschlag gegen die Hälfte davon). Mit elastischen Getrieben (`effects.joint_elasticity`) sind
Kupplung und Spindel des Hubs ebenfalls eine Feder (in Reihe 4,56e7 N/m am fernen Ende der Mutter): Die Säule schwingt
darauf mit 321 Hz bei gehaltenem Läufer, was den Standardzyklus um weniger als 0,1 % verschiebt (die größte Kraft der
Spindel 172,4 auf 172,5 N). Siehe die
[Modellreferenz](../model-reference.md#wafer-handler) und die
[Konfigurationsreferenz](../config-reference.md#wafer-handler).

## Parallelkinematiken: Hexapod und Seilroboter

Drei Parallelkinematiken haben eigene Crates, `machine-hexapod`, `machine-tripod` und `machine-cable-robot`, und werden
wie jede andere Maschine gerechnet und ausgelegt (`sim-cli analyze hexapod`, `sim-cli verdicts cable-planar`, die
3D-Ansicht der App, `search`):

| Id | Mechanik | Antriebe | Werkzeuglage | Eigene Prüfungen |
| --- | --- | --- | --- | --- |
| `hexapod` | Stewart-Gough-Plattform, 25 kg und bis zu 30 kg darauf, auf sechs elektrischen Zylindern (450 mm eingefahren, 200 mm Hub) | je Bein ein Kugelgewindetrieb 25 x 10 hinter einer Kupplung an einem 1FK2105-4AF | x, y, z (m), rx, ry, rz (rad, ein Drehvektor) | `screw-speed`, `screw-dn`, `screw-buckling`, `screw-static`, `screw-life` der Spindel je Bein, unter der Kraft des Beins |
| `hexapod-rotary` | Stewart-Plattform auf sechs Kurbeln (6-RSS), 35 kg und ein Sitz von 80 kg, Kurbeln 200 mm, Stangen 550 mm | je Kurbel ein Planetengetriebe SP100 50:1 an einem 1FT2104-5AF | x, y, z (m), rx, ry, rz (rad, ein Drehvektor) | `crank-dead-centre`, `rod-buckling`, `crank-ball-joint`, `platform-ball-joint` je Bein, das Getriebemoment |
| `cable-planar` | ein Schlitten von 3 kg und bis zu 2 kg, ein Punkt in einer senkrechten Ebene, an 4 Seilen aus einem Gestell 2 m x 1,6 m | je Seil eine Winde: Trommel 25 mm hinter einem Planetengetriebe 5:1 | x, z (m) | `cable-max-tension`, `cable-min-tension` je Seil |
| `cable-spatial` | eine Plattform von 4 kg und bis zu 4 kg, sechs Freiheitsgrade, an 8 Seilen aus den Ecken eines Gestells 2 x 1,6 x 1,8 m | wie der ebene, Trommel 30 mm | x, y, z (m), rx, ry, rz (rad) | wie der ebene |
| `tripod` | 3-RPS-Schwenktisch, 20 kg und 40 kg außermittig darauf, auf drei elektrischen Zylindern (450 mm eingefahren, 200 mm Hub), ein Drehgelenk an der Basis, ein Kugelgelenk an der Plattform | je Bein ein Kugelgewindetrieb 20 x 10 (Hiwin 20-10K3-FDC) hinter einer Kupplung an einem 1FK2104-4AF | z (m), rx, ry (rad, Neigungen um x und y) | die der Spindel je Bein; `tripod-leg-force`, `tripod-leg-stroke`, `tripod-revolute-angle`, `tripod-cone-angle` je Bein; `tripod-parasitic` |

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
  Lage erfüllt (keine Konvergenz in 40 Schritten), geben NaN, was jeder Aufrufer ablehnt. Die Dynamik, viermal je
  Integrationsschritt, beginnt Newton stattdessen bei der Lage am nächsten Punkt eines 1-mm-Rasters der Beinlängen (aus
  der Quadermitte gefunden, behalten, bis die Beine seine Zelle verlassen), verschoben um J^-1 (q − q_Raster): zwei
  Schritte statt etwa fünf. Diese Lage weicht von der aus der Quadermitte um Newtons Toleranz ab und hängt nur von q ab,
  nicht davon, was vorher aufgerufen wurde (ein Lauf, seine Wiedergabe und die gestörten Läufe einer Anpassung geben
  dieselben Zahlen); die Kennzahlen des Standardzyklus ändern sich damit so viel, wie eine Änderung in der Rundung der
  Lage sie ohnehin bewegt.
- Die Zeilen von J sind (u_i, b_i × u_i), u_i die Richtung des Beins und b_i = R a_i. Dynamik über die Bewegung der
  Plattform: M_xi xi' + c + g = J^T f, also im Beinraum M_q = J^-T M_xi J^-1 und h = J^-T (c + g − M_xi J^-1 a0), a0
  die Längenbeschleunigung der Beine bei der Bewegung (gegen die Lagrange-Gleichungen von M_q mit Differenzen geprüft:
  gleich auf 1e-9). Jedes Bein sind zwei Körper, die mit seiner Richtung schwenken: das Stangenteil (`leg.platform_share` von
  `leg.mass_kg`, Schwerpunkt `leg.platform_part_centre_m` vom Plattformgelenk) und das Gehäuseteil (der Rest,
  Schwerpunkt `leg.base_part_centre_m` vom Basisgelenk), jedes mit seiner Trägheit um den Schwerpunkt quer zum Bein; ihre
  kinetische Energie geht in M_xi und c ein, ihr Gewicht in g. Die Nutzlast kann eine eigene Trägheit haben
  (`mass.payload_inertia_kgm2`, 0: eine Punktmasse).
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

### Tripod

Eine Plattform auf drei Beinen veränderlicher Länge: z und zwei Neigungen, wie sie Schwenktische, Werkzeugköpfe und
Bewegungsplattformen nutzen. Die Bauform ist 3-RPS: jedes Bein ein elektrischer Zylinder wie am Hexapod (Motor in
Linie, Kugelgewindetrieb, Kupplung), an der Basis in einem Drehzapfen (Drehgelenk um die Tangente des Basiskreises, so
schwenkt das Bein in der senkrechten Ebene durch die Mitte der Basis) und an der Plattform in einem Kugelgelenk. Die
andere übliche Bauform, 3-PRS (senkrechte Schlitten mit Streben fester Länge, wie im Sprint-Z3-Kopf), hat dieselben
Zwangsbedingungen und dieselbe abhängige Bewegung; Beine veränderlicher Länge sind die elektrischen Zylinder des
Hexapods, Teile, die der Simulator hat. Jedes Bein ist ein Gelenk von `chain::JointMachine` wie am Hexapod; die Mechanik
gibt:

- Die Lage: z, die Höhe der Plattformmitte, und die Neigungen rx, ry, mit R = Rx(rx) Ry(ry) Rz(g): die reine Neigung
  R0 = Rx(rx) Ry(ry), dann eine Drehung g um die Normale der Plattform. Die drei Zwangsbedingungen (jedes
  Plattformgelenk bleibt in der Ebene seines Beins, e_i · (p + R a_i) = 0 mit den Gelenken bei 0, 120 und 240 Grad)
  geben die drei abhängigen Koordinaten geschlossen: tan g = −sin rx sin ry / (cos rx + cos ry), p_x = rp (cos ry cos g
  + sin rx sin ry sin g − cos rx cos g) / 2, p_y = rp cos ry sin g; in zweiter Ordnung p_x ≈ rp (rx² − ry²) / 4, p_y ≈
  −rp rx ry / 2, g ≈ −rx ry / 2. Die Normale der Plattform ist genau die der reinen Neigung: die Drehung geht nur um sie.
- Inverse Kinematik geschlossen (die Beinlängen |p + R a_i − B_i|). Die Bewegung (v, w) der Plattform löst G xi = (q',
  0), die Zeilen von G (u_i, b_i × u_i) für die Beine und (e_i, b_i × e_i) für die Zwangsbedingungen; die Raten des
  Werkzeugs sind S xi (z' = v_z und die Neigungsraten aus w), die Jacobi-Matrix also K = (S A)^-1, A die ersten drei
  Spalten von G^-1.
- Vorwärtskinematik mit Sehnenschritten x += K(home)^-1 (q − q(x)) von der Mitte des Werkzeugquaders aus; nach dem
  ersten Schritte mit K an diesem ersten Punkt (nahe der Lösung weit schneller: 5 bis 6 Auswertungen der Beine über den
  Quader), solange jeder den Fehler der Beine halbiert, sonst endgültig wieder Sehnenschritte; bis die Beine um weniger
  als 1e-13 m verfehlen, NaN (abgelehnt) nach 100 Schritten. Warum das im Arbeitsraum konvergiert: mit dem Fehler
  gewichtet als (dz, rp drx, rp dry) in der Maximumnorm liegt jede Lage des Quaders höchstens r = 0,04 m von seiner Mitte,
  und ein Sehnenschritt verkleinert den Fehler mindestens um kappa = max ||W (1 − K0^-1 K(x)) W^-1|| über den um r
  vergrößerten Quader (z 0,46 bis 0,62 m, Neigungen ±0,4 rad), der jede Strecke enthält, entlang der die Schritte laufen.
  Auf einem Gitter von 17³ Punkten dort ist kappa höchstens 0,595 und ändert sich zwischen Nachbarn um weniger als
  0,07, also kappa < 0,7: die Schritte konvergieren von der Mitte zu jeder Lage des Quaders (Test
  `the_chord_iteration_contracts_over_the_grown_working_range`); die schnelleren Schritte können das nicht verhindern,
  denn jeder muss den Fehler halbieren oder an die Sehne zurückgeben.
- Dynamik über die Bewegung der Plattform, die Zwangskräfte herausprojiziert: G xi' = (q'' − a0, −c0) gibt xi' = A q''
  + beta; die Plattform M_xi xi' + c + g = J^T f + C^T lambda, M_xi mit dem ersten Moment s = R s_b einer außermittigen
  Nutzlast; A^T nimmt lambda heraus (C A = 0) und A^T J^T = 1, also M_q = A^T M_xi A, h = A^T (M_xi beta + c + g). Die
  Nutzlast ist ein Körper für sich (`mass.payload_kg`, `payload_offset_m`, `payload_inertia_kgm2`); die Masse jedes
  Beins wird aufgeteilt wie am Hexapod.
- Signale: je Bein `leg.<i>.from_middle` (sein Auszug von der Hubmitte), `leg.<i>.swing` (sein Winkel in seiner Ebene
  gegen die Normale der Basis, am Drehgelenk), `leg.<i>.cone` (sein Winkel gegen die Normale der Plattform, der Kegel
  des Kugelgelenks); die der Plattform `platform.condition`, die parasitären `platform.x`, `platform.y`, `platform.turn`
  und `platform.pose_error`, der größte Abstand eines Punkts des Gelenkkreises von dort, wohin ihn die reine Neigung um
  (0, 0, z) brächte, sqrt(|d|² + c² + 2 c |(R0^T d)_xy|) mit d = (p_x, p_y, 0) und c = 2 rp |sin(g/2)|.
- Abgelehnt: ein Bein über seinen Hub hinaus (`error-joint-beyond-stop`), ein Schwenkwinkel über
  `workspace.max_revolute_angle_deg` (`error-tripod-revolute-angle`), ein Kegelwinkel über `workspace.max_cone_angle_deg`
  (`error-tripod-cone-angle`), eine Konditionszahl von K (Neigungen mit rp gewichtet) über
  `workspace.max_condition_number` (`error-tripod-singularity`). Waagrecht ist die Konditionszahl √2, im ausgelieferten
  Quader höchstens 1,62.
- Prüfungen: die der Spindeln wie am Hexapod; `tripod-leg-force` (`joint.<i>.force` gegen `leg.max_force_n`),
  `tripod-leg-stroke` (`from_middle` gegen den halben Hub), `tripod-revolute-angle`, `tripod-cone-angle` (gegen die
  Grenzen der Gelenke) und `tripod-parasitic` (`platform.pose_error` gegen `ratings.parasitic_tolerance_m`, wo gegeben).

### Hexapod mit Drehantrieben (6-RSS)

`hexapod-rotary` (Crate `machine-hexapod-rotary`) ist die Bauart von Bewegungssimulatoren und schnellen
Sechsachs-Positionierern: sechs Servomotoren mit Planetengetrieben auf der Basis, jeder dreht eine Kurbel von 200 mm,
die Spitze jeder Kurbel ist über eine Stange von 550 mm mit einem Kugelgelenk an beiden Enden mit der Plattform
verbunden. Die Standardkonfiguration ist eine kleine Bewegungsplattform: 35 kg Plattform mit einem Sitz samt Insasse,
80 kg (Schwerpunkt 300 mm über den Plattformgelenken, mit eigener Trägheit), auf sechs 1FT2104-5AF mit Getrieben
SP100-04-50 an S210-Umrichtern 6SL3210-5HE10-8UF0 (Katalog). Werkzeuglage und Achsen wie beim Hexapod; die Gelenke sind die Kurbelwinkel (rad, 0 waagrecht,
positiv hebt die Spitze). Jede Kurbel ist ein Gelenk von `chain::JointMachine` mit einem `joint::RotaryJoint`
(Getriebesteifigkeit, Spiel, Wirkungsgrad, Reibung des Kurbellagers, Anschläge) wie bei den Delta-Pickern. Siehe die
[Modellreferenz](../model-reference.md#hexapod-with-rotary-actuators-6-rss) und die
[Konfigurationsreferenz](../config-reference.md#hexapod-with-rotary-actuators).

- Geometrie: die Mitten der Kurbelachsen paarweise bei 120 k ± `platform.base_half_angle_deg` auf
  `platform.base_radius_m`, die Plattformgelenke paarweise bei 120 k ± (60 − `platform_half_angle_deg`) auf
  `platform_radius_m`; jede Kurbelachse waagrecht, der Basisradius um ± `crank.axis_turn_deg` gedreht (im Paar
  gespiegelt), damit die Stange wenig aus der Kurbelebene kippt.
- Inverse Kinematik je Bein geschlossen: mit e = P_i − B_i ist a cos θ + b sin θ = c, a = e · d_i, b = e_z,
  c = (|e|² + r² − L²) / 2r; θ = atan2(b, a) ∓ acos(c / √(a² + b²)), der Zweig einmal durch `crank.branch` festgelegt
  ("out": die Spitze vom Partner des Paars weg; "in" der andere); keine Lösung, wo |c| > √(a² + b²).
- Vorwärtskinematik nach Newton über die Bewegung der Plattform, wie beim Hexapod; auch die Dynamik beginnt bei der
  Lage am nächsten Punkt eines 1-mrad-Rasters der Kurbelwinkel, verschoben um K (q − q_Raster).
- Jacobi-Matrix: A ξ = diag(u_i · t_i) θ', die Zeilen von A (u_i, b_i × u_i), u_i die Richtung der Stange und t_i die
  Tangente der Kurbelspitze; u_i · t_i ist der Hebelarm des Motors. Die Konditionszahl der inversen Jacobi-Matrix
  (Drehungen mit dem Plattformradius gewichtet) zeigen die Arbeitsraumkarte und das Signal `kinematics.condition`; sie wächst
  ohne Grenze, wenn sich eine Kurbel ihrem Totpunkt nähert (alle sechs zugleich dort skalieren ihre Zeilen gleich und
  lassen sie unverändert: diese Lage lehnt der Abstand vom Totpunkt selbst ab).
- Abgelehnt: eine Stange, die nicht hinreicht (`error-target-out-of-range`), eine Kurbel näher an ihrem Totpunkt (die
  Stange in der Ebene von Kurbel und Achse, der Hebelarm null) als `workspace.min_dead_centre_margin_deg`
  (`error-rss-dead-centre`), ein Kugelgelenk über `workspace.crank_joint_max_angle_deg` (die Stange aus der Kurbelebene)
  oder `platform_joint_max_angle_deg` (gegen die Normale der Plattform; `error-rss-ball-joint`), zwei Stangen näher als
  `workspace.min_rod_clearance_m` Achse zu Achse (`error-rss-rods-cross`), eine Stange näher als `workspace.min_body_clearance_m` an einer Kante der Plattform oder der Basis (`error-rss-rods-cross-body`), eine Konditionszahl über
  `workspace.max_condition_number` (`error-rss-singularity`), eine Kurbel über ihre Anschläge hinaus.
- Dynamik über virtuelle Arbeit in den Kurbelwinkeln: Plattform und Nutzlast als ein starrer Körper mit Schwerpunkt
  außerhalb der Plattformmitte, jede Stange als schlanker Stab (ein Sechstel ihrer Masse an jedem Ende und zwei Drittel
  in der Mitte: genau für Masse, Schwerpunkt und Querträgheit eines gleichförmigen Stabs, was die Aufteilung der
  Delta-Crates nicht ist), jede Kurbel mit Trägheit und Gewicht, die Rotoren über die Übersetzung; Schwerkraft; Energie
  und Impuls für die Standardprüfungen und die nachgiebige Basis.
- Prüfungen außer denen der Motoren: die Getriebemomente (`gearbox-peak`, `gearbox-continuous`, `gearbox-input-speed`
  gegen die Nennwerte des SP100), je Bein `crank-dead-centre` (Winkel der Stange zur Tangente der Kurbel gegen 90°
  weniger `ratings.crank_dead_centre_margin_deg`, 20°), `rod-buckling` (Druckkraft der Stange gegen ihre Euler-Last
  π² E I / L² des Rohrs aus `[rod]`, durch `ratings.rod_buckling_safety_factor`), `crank-ball-joint` und
  `platform-ball-joint` (die Winkel der Gelenke gegen ihre Grenzen) und `singularity`.
- Der Standardzyklus: Hub 50 mm auf, dann Längs- und Querbewegung 60 mm, Rollen
  und Nicken 0,12 rad, Gieren 0,15 rad, jeweils aus der Lage davor (die eine geht zurück, während die nächste einsetzt),
  und zurück in die waagrechte Lage, wo der Sitz ab- und wieder aufgesetzt wird (das Umschalten der Nutzlast).

Wie beim Hexapod läuft die von festen Kurbelmomenten gehaltene Plattform unter ihrem Gewicht aus ihrer Lage, daher
laufen die verlustfreien Konfigurationen der Standardprüfungen ohne Schwerkraft (und ohne Getriebespiel, das ohne Last
an keiner Flanke anliegt); die Schwerkraft prüfen die von Hand gerechneten Haltemomente und die Standardkonfiguration.

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
- Die eigene Masse der Seile (`cable.mass_per_m_kg`, ausgeliefert 3 g/m und 10 g/m, [Schätzung]; 0 lässt sie weg): Die
  Hälfte des Gewichts jedes Seilstücks hängt an der Plattform, und das Seil hängt durch (flache Parabel): Bei der
  Sehnenkraft T ist es um (w l_h)² l_f / (24 T²) länger als seine Sehne, also mit t0 = EA (L − l) / l, der Kraft des
  geraden Seils, t0 = T − c / (2 T²), c = (w l_h)² EA l_f / (12 L); seine Steifigkeit dT/dt0 ist Ernsts
  Ersatzmodul E / (1 + (w l_h)² EA / (12 T³)), und ein schlaffes Seil hängt noch mit einer kleinen Kraft. Gegen die
  genaue elastische Kettenlinie stimmt der Zug am Anlenkpunkt auf 9e-5 N bei 20 N und 2e-4 N bei 5 N (das gerade Seil
  verfehlt ihn dort um 0,17 N und 1,6 N).
- Trommeln und Läufer der Winden drehen sich um `frame.winch_spin_axes` (sonst waagerecht und quer zur Richtung des
  Seils an der Winde; bei senkrechter die Achse der Rolle in Ruhe), und ihr Drall ist in der Last des Gestells.
- Die zeitoptimale Planung hält die Kraft jedes Seils im Starrkörpermodell im Band (`cable.min_tension_n` bis zur
  Nennkraft, `profile.tension_reserve` des Raums): Der Standardzyklus des ebenen Roboters braucht 1,684 s statt 1,653 s
  ohne das Band, bei dem ein Seil bis auf 0,63 N fiel; cable-spatial 1,207 s statt 1,143 s.
- Jede Winde ist ein `joint::RotaryJoint` in Metern Seil: Motor, Getriebe mit dem Trommelradius als Hebel, die Trägheit
  der Trommel, Reibung von Lagern und Rolle (`effects.winch_friction`), Anschläge bei leerer und voller Trommel.
  Elastisch sind Trommel und Läufer eigene Körper hinter der Feder des Getriebes; starr behalten die Seile ihre Länge,
  und die Trägheit der Winden kommt zu der der Plattform.
- Die Motorwinkel einer Lage sind die der freien Längen, die die Plattform dort in Ruhe halten, l = L / (1 + t / EA),
  t die Kräfte der geschlossenen Form (die der Verteilung, wo kein Seil an einer Grenze sitzt), sodass Lagesollwerte und
  Momentenvorsteuerung (die Kräfte der Verteilung für die Dyname der Plattform, dazu die Trägheit der Winden) dieselbe
  Vorspannung von den Seilen verlangen. Die Vorwärtskinematik ist Gauß-Newton über diese Längen; die Regelung braucht
  sie zweimal je Lagereglertakt (der Schleppfehler im Werkzeugraum), daher macht sie zuerst billige Schritte und nimmt
  die numerische Jacobi-Matrix nur nahe am Rand des Arbeitsraums. Mit dehnbaren Seilen sucht sie danach die Ruhelage
  der Plattform auf ihnen nach Newton; dieses Newton beginnt bei der Lage am nächsten Punkt eines 1-mm-Rasters der
  freien Seillängen (wie zuvor gefunden, behalten, bis die Längen seine Zelle verlassen) mit deren Jacobi-Matrix
  (Sehnenverfahren, die Matrix neu bestimmt, wo ein Schritt scheitert), was das Gauß-Newton und die meisten Residuen
  spart. Die Lage weicht um Newtons Toleranz (1e-12 m) ab und hängt nur von den Motorwinkeln ab; die Kennzahlen der
  Standardzyklen ändern sich damit um bis zu 1,5e-5.
- Signale je Seil: `cable.<k>.tension`, `cable.<k>.shortfall` (wie weit es unter der kleinsten Kraft liegt),
  `cable.<k>.length`; `platform.condition`, die Konditionszahl der Strukturmatrix. Prüfungen: `cable-max-tension`
  (höchste Seilkraft gegen die Nennkraft) und `cable-min-tension` (jedes Unterschreiten fällt durch), Motoren und
  Antriebe wie an jeder Maschine.

Die Kurve der Presse wechselt die Geschwindigkeit mit 4-5-6-7-Gesetzen: ein 3-4-5 in die langsame Zone verlangt an seinen
Enden 6 Δv / Δm² + 60 c / Δm³ = 67,4 rad/rad³ Ruck (Δv 1,468, Δm 0,5 rad, c −0,067 rad), bei π rad/s Leitwert
2090 rad/s³, mehr als die 2000 der Kurbel; das 4-5-6-7 hat dort keinen eigenen Ruck. Die Drehzahlregler der vier
Maschinen haben Tn 0,01 s (das Messer Kp 0,25): der Integrator nimmt die Reibung am Ende einer Fahrt dreimal so schnell
auf wie mit 0,03 s, und die Fehler beim Verweilen fallen auf 0,13 mrad (Kurbel), 0,16 mrad (Messer), 0,14 mm (Säge) und
0,23 mrad (Scheibe), mit mindestens 35,8° und 8 dB Reserve (am 15. November neu eingestellt, damit die Reserven auch ohne Reibung halten:
das Messer Kp 0,18, mindestens 37,8° und 9,9 dB, Schleppfehler im Zyklus 8,86 → 12,1 mrad; die Säge Tn 0,04 s, kv
57 1/s und Kp 0,25 mit einem Kerbfilter auf der Resonanz von Getriebe und Zahnstange, 360 Hz, 250 Hz breit
(`effects.notch_filter`, je Maschine der erste Schlüssel ihrer Datei), mindestens 40,2° und 11,51 dB, nach einer langen
Fahrt 12,3 µm Bewegung in 0,2 s) in jedem Lauf über die Grenzen (`sim-cli margins <id>
--bands`); die Toleranzen des Einschwingens sind 0,3 mrad, 0,3 mrad, 0,25 mm und 0,5 mrad.

## Pneumatikzylinder neben Servoachsen (`[[machine.cylinder]]`)

Die meisten Bestückautomaten haben neben ihren Servoachsen einen pneumatischen Antrieb: einen Z-Hub, einen
Greiferschlitten, einen Stopper. Eine Maschinendatei fügt ihn mit einem Knoten `[[machine.cylinder]]` hinzu, dessen
Tabelle `values` Zylinder, Ventil, Schlauch, Drosseln und Dämpfung enthält (`sim_components::pneumatic`, Schlüssel in
[config-reference.md](../config-reference.md), Modell in [model-reference.md](../model-reference.md)). Er ist keine
Werkzeugachse: kein Antrieb bewegt ihn, und die Steuerung plant für ihn nichts. Der Zyklus schaltet sein Ventil, sobald
ein Segment angekommen ist (`valves = { z = "extend" }`), und wartet auf seine Endlagensensoren
(`wait = ["z.extended"]`), bevor die Verweilzeit zählt und das nächste Segment beginnt.

`machines/pneumatic-axis.toml`: die riemengetriebene Servo-X von `single-axis.toml` trägt einen Festo
DSBC-32-100-PPVA-N3 (32 mm Kolben, 12 mm Stange, 100 mm Hub, PPV-Dämpfung 17 mm, 0,4 J), der nach unten ausfährt, an
einem 5/2-Ventil mit Abluftdrosseln; 0,5 kg bewegt, 0,5 kg gegriffen. Der Zyklus: X zum Aufnehmen, Z ab, warten,
greifen, Z auf, warten, X zum Ablegen, Z ab, loslassen, Z auf. Mit den ausgelieferten Einstellungen fährt Z in 0,55 bis 0,56 s
aus und in 0,65 s ein (Ventil 7 ms, die Stangenseite entlüftet aus dem Versorgungsdruck, gedrosselt 0,216 m/s, am Ende
die Dämpfungsschraube), tritt mit 0,22 bis 0,24 m/s und einer Bewegungsenergie von 0,012 bis 0,029 J gegen die zulässigen 0,4 J in die
Dämpfung ein (`cushion-energy`). Die Prüfung ist die Regel des Herstellers für die eigene Dämpfung des Zylinders: die
Bewegungsenergie beim Eintritt in die Dämpfung gegen die höchste Aufprallenergie der Tabelle (Festo-Datenblatt DSBC,
„Kräfte und Aufprallenergie“, v = sqrt(2 E / (m1 + m2)), S. 13); die Arbeit des Gewichts über die Dämpfungslänge, m g L,
gehört zu Festos Regel für externe Stoßdämpfer und wird nicht addiert (1 kg hängend über die 17 mm: 0,167 J mehr, die
diese Prüfung nicht zählt). Er trifft mit 0,06 bis 0,09 m/s auf den Puffer, ohne aus dem Sensorbereich zurückzuprallen, und hat
ein Lastverhältnis von 0,024 (`load-factor`, 1 kg Gewicht gegen 415 N beim Einfahren). Ein Doppelhub verbraucht 1,10
Normliter, 474 J am Kompressor (0,12 kWh/m³); von Hand beim Versorgungsdruck und der Wandtemperatur 1,15 l, 4 % weniger,
weil die Verweilzeiten zu kurz sind, als dass die heiße Luft abkühlt.

Was der Zylinder für die Auslegung liefert: Hubzeiten hin und zurück, den Geschwindigkeitsverlauf, Geschwindigkeit und
Bewegungsenergie beim Eintritt in die Dämpfung gegen ihren Nennwert, die Geschwindigkeit am Endanschlag, Rückprall und
Einschwingen, das Lastverhältnis, die Luft je Doppelhub und ihre Energie am Kompressor, die Druck- und
Temperaturverläufe beider Kammern.

## Auf- und Abwickler mit Tänzer (`machine-winder`)

Die klassische Auslegungsaufgabe der Verarbeitungsmaschinen, und die einzige Maschine, deren Motoren ein Material koppelt
und deren Last sich im Lauf ändert: `winder` führt eine Bahn (Kunststofffolie, Papier, Metallfolie) von einem Abwickler über
eine Tänzerwalze zu einem Aufwickler, mit einer Bahngeschwindigkeit und unter einem Bahnzug. Die ausgelieferte
Konfiguration: eine PET-Folie von 50 µm, 400 mm breit, mit bis zu 2 m/s (in einer halben Sekunde auf Geschwindigkeit,
4 m/s²) und 100 N, auf 3-Zoll-Kernen (r_c 45 mm), die bis 300 mm wachsen (r_f 150 mm; 1286 m Folie, 36 kg), jeder Wickel
an einem 1FK2206-2AF mit Haltebremse hinter einem Planetengetriebe mit Übersetzung 7, an einem S120-Motor-Module von
5 A mit einem Braking Module für beide; der Tänzer eine Walze
von 80 mm auf einer senkrechten Führung, 4 kg und ein Gewicht von 2 kg, von einem Pneumatikzylinder nach unten gedrückt,
±50 mm Weg. Die Bahnposition läuft über die ersten 6 m des Wickels (`roll.run_from_m`, `run_m`), wo der Abwickler voll
ist und der Aufwickler auf seinem Kern; der Zyklus fährt 3 m vor und zurück. Das Modell steht in der
[Modellreferenz](../model-reference.md#winder-with-dancer).

| Kennung | Mechanismus | Übertragung | Werkzeuglage | Eigene Prüfungen |
| --- | --- | --- | --- | --- |
| `winder` | Abwickler, zwei Bahnabschnitte von 1 m über eine Tänzerwalze, Aufwickler | Planetengetriebe mit Übersetzung 7 je Wickel | Bahnposition (m), die aufgewickelte Bahn | `web-tension`, `web-slack`, `dancer-travel`; je Motor `inertia-ratio`, `torque-at-full-roll`, `torque-at-core`, `speed-at-core`, `tension-torque-at-full-roll` |

**Was sich im Lauf ändert.** Der Radius eines Wickels wächst je Umdrehung um die Dicke der Bahn, r² = r_c² + h s / π; die
Motordrehzahl für eine Bahngeschwindigkeit v ist i v / r, am höchsten am Kern (311 rad/s bei 2 m/s), das Moment für
einen Bahnzug T r / i, am höchsten beim vollen Wickel (2,14 N m bei 100 N), und die Trägheit, die er sieht, wächst mit
r⁴: über den Wickel vom 0,23- bis zum 10,4-fachen der von Läufer und Getriebe. Eine Auslegung muss beide Enden halten;
ein Zyklus fährt Meter eines Wickels von 1286 m, darum tragen die eigenen Prüfurteile Bahngeschwindigkeit, Beschleunigung
und Bahnzug des Zyklus an den vollen Wickel und an den Kern (Modellreferenz, „Own verdicts“). Mit dem ausgelieferten
Motor entscheidet das Trägheitsverhältnis (10,4 von erlaubten 15) über den Motor, nicht sein Moment: ein 1FK2205-2AF
(3,15 kg cm²) hätte das Moment, aber beim vollen Wickel das 22-fache seiner Trägheit.

**Die Bahn** dehnt sich in zwei Abschnitten von EA / L (je 80 kN/m), trägt ihre Dehnung von Abschnitt zu Abschnitt und
hängt unter null Zug durch. Der Tänzer schwingt auf ihnen mit 26,1 Hz, seine Walze mit 56,9 Hz. Die Wickel folgen der
Bahnposition als elektronisches Getriebe beim Radius ihrer gewickelten Länge, mit dem Moment des Bahnzugs und der
Trägheit des Wickels vorgesteuert; der Lageregler des Tänzers korrigiert die Drehzahl des Abwicklers: ein mit 100 N
gewickelter und mit 80 N gefahrener Wickel ließe den Tänzer bei 2 m/s um 0,25 mm/s wandern (der Verzug), was der Regler
ausgleicht. Der Bahnzug-Sollwert fällt über den Radius des Aufwicklers, von 100 N am Kern auf 79 N beim vollen Wickel,
über die Kraft des Zylinders.

**Halt.** Ein Halt der Kategorie 2 (`sim-cli stop winder --category 2`, die Bahn entlang ihres Wegs mit den 4 m/s² der
Achse gebremst) hält die Bahn bei jedem Radius zwischen 30 und 250 N. Ein Halt der Kategorie 1 bremst jeden Motor an
seiner eigenen AUS3-Rampe: in der Mitte des Wickels, wo die Radien gleich sind, behält die Bahn ihren Zug; nahe einem
Ende verzögern die beiden Oberflächen im Verhältnis ihrer Radien (1,35 gegen 0,40 m/s² am vollen Wickel und am Kern),
der Tänzer läuft nach 0,46 s in seinen Anschlag und die Bahn wird über 1200 N gedehnt. Kategorie 0 lässt die
Haltebremsen die Wickel binnen etwa 50 ms greifen, und die Bahn hängt durch. Am Wickler hält nur ein Halt entlang der
Bahn die Bahn; nach einem Halt der Kategorie 1 halten die Haltebremsen den Zug, den der Tänzer hält.

**Drehzahlregler.** Der Aufwickler auf seinem Kern hat ein Zehntel der Trägheit des vollen Abwicklers am Motor, darum
hat jeder seine eigene Verstärkung (1,2 und 3,0 N m s/rad, Tn 30 ms): `sim-cli margins winder` gibt 73,6° und 20,5 dB am
Abwickler, 58,8° und 9,56 dB am Aufwickler (52,0° und 9,19 dB ohne Reibung), über die Grenzen mindestens 54,9° und
8,91 dB (`--bands`).

```text
sim-cli verdicts winder
sim-cli run winder --cycle --time 10 --stop-at 7.5 --stop-category 2 --out winder.csv
sim-cli stop winder --category 2
sim-cli verdicts winder --set web.tension_n=150 --set axis.web.max_velocity_m_s=3.0
```

## Mover auf geschlossener Strecke (`machine-mover-track`)

Ein lineares Transportsystem wie Beckhoffs XTS oder B&Rs ACOPOStrak: Motorsegmente aus Linearmotorspulen in einer
geschlossenen Schleife, und darauf unabhängige Mover, jeder eine Masse auf Rollen mit einer Magnetplatte, jeder von den
Spulen unter ihm mit eigenem Lageregler angetrieben. Die ausgelieferte Konfiguration: sechs Mover von 651 g (AT9014) mit
je 600 g, auf einer Schleife aus zwei Geraden von 1 m aus je vier Modulen von 250 mm (AT2000) und zwei 180°-Kurven von
267 mm Breite, an jedem Ende eines Bogens von 130,3 mm eine Klothoide von 100 mm (die Aufteilung ist geschätzt; Beckhoff
veröffentlicht die Breite und dass es eine Klothoide ist); 3,019 m Umfang. Ein Mover gibt 100 N Spitze und 30 N
Dauerkraft, 4 m/s, 10 g ohne Last (die 5-polige Magnetplatte an 48 V; mit der Nutzlast bewegt die Platte
1,65 kg mit 60 m/s², die Achsen sind auf die Hälfte, 30 m/s², begrenzt); Kraftkonstante, Spulenwiderstand, Anzugskraft (300 N) und
Polteilung sind geschätzt.

| Id | Mechanismus | Übertragung | Werkzeugpose | Eigene Prüfungen |
| --- | --- | --- | --- | --- |
| `mover-track` | 1 bis 7 Mover auf einer geschlossenen Schleife aus Geraden und Kurven (6 ausgeliefert) | Linearmotor je Mover: die Spulen unter ihm | Weg jedes Movers entlang der Schleife (m), nicht umgebrochen | je Mover `mover-guide-load`, `curve-holding`, `station-settling` (Taktzyklus); `mover-gap`; je Segment `segment-current`, `segment-temperature` |

**Zwei Zyklen, beide elektronische Kurvenscheiben.** Eine geschlossene Strecke fährt nur vorwärts, daher ist der Zyklus
eine Kurvenscheibe (`[cam]`): `track.cycle = "index"` fährt jeden Mover um eine Stationsteilung (die
Schleife durch die Zahl der Mover, 0,503 m) mit einem 4-5-6-7-Gesetz in 0,4 s weiter und lässt ihn den Rest der Periode
an seiner Station stehen, 1,5-mal je Sekunde (2,75 m/s, 23,6 m/s² Spitze); `flying.toml` fährt `"flying"`: jeder Mover
fährt der Reihe nach über 0,3 m der ersten Geraden synchron mit einem Förderband von 1,0 m/s, dann um die Schleife zurück
und wieder in den Prozess (6 Produkte zu 0,3 m je Umlauf: die Mover rücken über dem Prozess auf eine Produktteilung
zusammen und fahren mit bis zu 2,78 m/s zurück). `default.toml` liefert `cam.run = false` aus, wie die Kurbel- und
Kurvenmaschinen: dort fahren die Mover nach den `[[cycle.segment]]` 0,25 bis 0,3 m vor, nehmen ein Produkt
und bringen es durch einen Punkt, an dem sie nicht anhalten, zum Start zurück, den
Zyklus, den die allgemeinen Tests und Werkzeuge fahren; `--set cam.run=true` fährt den Taktzyklus.

**Kurven.** Die Rollen eines Movers spannen seinen Radstand w, daher folgt sein Körper dem Mittel der Krümmung darüber:
die Querbeschleunigung v² k steigt über den Radstand an, wo eine Kurve mit festem Radius beginnt, und über die
Klothoide, wo es eine gibt; ihre Änderung, der Querruck 2 v a k + v³ dk/ds, ist, was die Klothoide senkt (bei 2 m/s
1228 m/s³ in einen Kreis, 614 m/s³ in die Klothoide). Die Mover laufen außen an der Schleife, die Magnetplatte den
Spulen zugewandt, daher zieht die Fliehkraft m v² k einen Mover gegen die Anzugskraft von den Spulen weg: die Rollen
tragen sqrt((F_a − m v² k)² + (m g)²) (`mover-guide-load`), und der Anteil m v² k / F_a (`curve-holding`) muss unter 1
bleiben, wo der Mover abhebt. `mass.centre_offset_m` = e legt den Massenschwerpunkt der Mover außerhalb der Mittellinie
(beim gezeichneten Mover 35 mm; ausgeliefert 0): in einer Kurve läuft er auf R + e mit v (1 + e k), seine
Querbeschleunigung ist v² k (1 + e k) (38,9 m/s² mitten im Bogen bei 2 m/s und e = 35 mm, 30,7 bei e = 0), seine
kinetische Energie ½ m v² (1 + e k)²; die Tangentialkraft der sich über die Klothoide ändernden Masse ist in der Dynamik
(die Energie bleibt erhalten). Der Versatz gilt für Moverkörper und Nutzlast, nicht für die Magnetplatte.

**Spulen und Segmente.** Der Strom eines Movers ist seine Kraft durch die Kraftkonstante; er fließt in den Spulen unter
seiner Magnetplatte, auf die Segmente nach der Länge der Platte auf jedem verteilt. Der Strom eines Segments ist die
Summe der Anteile seiner Mover (`segment-current`, gegen die 48 A Spitze des Moduls), sein Kupferverlust R I² jedes
Anteils; es erwärmt sich über einen Wärmewiderstand und eine Zeitkonstante (`segment.<j>.temperature_rise`), und
`segment-temperature` beurteilt die Erwärmung, auf die sein über einen Durchlauf gemittelter Verlust einschwingt: Mover,
die auf einem Segment halten, heizen es. Der Abstand zwischen Nachbarn entlang der Mittellinie, abzüglich einer
Körperlänge, gegen den kleinsten Abstand von 2 mm (`mover-gap`); die Abweichung eines stehenden Movers an seiner Station
gegen die Wiederholgenauigkeit von ±10 µm (`station-settling`).

Modell und Grenzen in der Dokumentation des Crates (`crates/machine-mover-track/src/lib.rs`).

```text
sim-cli verdicts mover-track --set cam.run=true
sim-cli verdicts mover-track --config crates/machine-mover-track/flying.toml
sim-cli verdicts mover-track --set cam.run=true --set track.index_rate_hz=2.5 --set track.clothoid_m=0.0
```

## Brückenkran (`machine-overhead-crane`)

`overhead-crane` ist ein Einträger-Brückenkran mit 250 kg Tragfähigkeit: die Brücke fährt 10 m auf ihrer Kranbahn (x),
die Katze 5 m am Träger (y), das Hubwerk auf der Katze wickelt ein einsträngiges Drahtseil von 6 mm auf seine Trommel. Die
Werkzeuglage ist der Haken, wie die Antriebe ihn stellen: x der Brücke, y der Katze und die Höhe z der Last über dem
Boden (m); das Pendeln der Last hat eigene Signale und eine eigene Prüfung. Jeder Antrieb ist ein Servomotor hinter
einem Planetengetriebe (Siemens 1FK2104-6AF an Brücke und Katze, 1FK2105-6AF am Hubwerk, S210), Rad oder Trommel der
Hebel: 30:1 auf Rädern von 80 mm, 25:1 auf 50 mm, 40:1 auf einer Trommel von 80 mm.

- **Das Seil** läuft gerade von der Seilrolle der Katze in 5 m Höhe zum Lastpunkt (Haken und Last ein Massenpunkt). Gespannt
  zieht es mit t = EA e / l + c / l (L' − l'), e = L − l die Dehnung über die freie Länge, drücken kann es nicht: unter null
  wird es schlaff. Es zieht die Last zur Katze, die Katze (x an der Brücke, y an der Katze) und über die Ableitung seiner
  Federenergie die Trommel mit t (1 + e / (2 l)): das Pendeln der Last wirkt auf beide Fahrantriebe zurück.
- **Die Last** ist ein Kugelpendel an diesem Seil; ihre Schwingdauer bei kleinem Ausschlag 2π √(L / g) (2,84 s an 2 m
  Seil, 4,01 s an 4 m). Starr (`effects.rope_stretch` aus) behält das Seil seine Länge, seine Kraft ist die
  Zwangskraft (das Bezugsmodell für ein gespanntes Seil; es kann auch drücken).
- **Die Antriebe** sind starr, mit Getriebewirkungsgrad, Motorreibung und Haltebremse wie an jeder Maschine; die Räder
  rollen gegen `crane.rolling_coefficient` mal ihre Last (`effects.guide_friction`).
- **Die Motorwinkel einer Lage** sind die des leeren Hakens; die Last hängt um ihre eigene Dehnung tiefer (6,8 mm an
  4 m), und die Trommel steht still, wenn der Haken die Last nimmt oder abgibt. Aufnehmen und Absetzen sind quasistatisch.
- **Eingangsformer gegen das Pendeln.** Die Maschine meldet die Pendelfrequenz √(g / L) / 2π einer Lage; mit
  `effects.shaper_follows_pose` (in der Konfiguration an) folgt die Periode des Formers der Seillänge, wo der Plan steht,
  sodass eine Fahrt, die dabei hebt, für die sich ändernde Länge geformt wird. `effects.input_shaping` selbst ist in der
  ausgelieferten Konfiguration aus, wie an jeder Maschine; eingeschaltet mit `--set effects.input_shaping=true`. Gemessen
  auf dem Rückweg des leeren Hakens, der beim Fahren von 4 m auf 2 m Seil hebt: ungeformt 0,43 m Restpendeln, mitgeführter
  ZV (der ausgelieferte, eine halbe Pendelperiode hinter dem Plan) 0,017 m, ein ZV für die Länge am Ende der Fahrt
  0,124 m, einer für die am Anfang 0,105 m (ZVD, eine ganze Periode dahinter: 0,012, 0,047 und 0,041 m).
- **Der Zyklus** (fahren und absenken, zurück, aufnehmen, heben) ist ein kurzer Positionierzyklus, 7,0 s je Durchgang
  (geformt 12,8 s), damit jede Prüfung über die Zyklen aller Maschinen ihn ganz sieht: aus der Fahrhöhe (z = 3 m, 2 m
  Seil) 0,2 m und 0,1 m zur Ablage, dabei 0,15 m absenken. Geformt besteht er alle Prüfungen (Restpendeln 4,3 mm,
  Seilkraft höchstens 2727 N und nie schlaff, Hubwerksbremse 5,5 von 7,5 N m belastet); ungeformt pendelt die Last
  0,32 m, und `residual-sway` fällt durch, ebenso `thermal-warm-start` (das Pendeln eines Durchgangs ändert den
  nächsten). Ein echter Werkstattkranzyklus über die Kranbahn dauert eine Minute: ein Ziel von [8.0, 4.0, 1.0].
- **Die Drehzahlregler** sind so steif eingestellt wie an den anderen Maschinen (Brücke und Katze etwa 60 Hz). Der des
  Hubwerks wird in Ruhe unter den 265 kg (`sim-cli margins`) mit ausgeschaltetem Wirkungsgradeffekt gemessen: Der
  Wirkungsgrad seines Getriebes (92 %), unter 0,5 W eingeblendet, wirkt sonst als Dämpfer von 1,1e6 N s/m am Seil,
  4,5 N m s/rad am Motor, von dem das Testmoment nicht losbricht, und der Kreis zeigt keine Durchtrittsfrequenz. Ohne die
  Getriebeverluste, mit einem Zehntel des eigenen Nennmoments des Hubmotors (0,66 N m) als Testmoment, liegt sie bei
  50,4 Hz mit 73,7° und 21,0 dB (bei 0,32 N m, einem Zehntel des gemeinsamen Antriebswerts: 48,5 Hz, 83,1°, 21,3 dB; die
  Reibung des Motors dämpft das kleinere Signal stärker).
- **Eigene Prüfungen:** `rope-max-tension`, `rope-min-tension` (kein Schlaffseil), `residual-sway` (Amplitude des
  Pendelns bei jedem Verweilen gegen `crane.residual_sway_max_m`), `hoist-brake` (Seilmoment am Hubmotor gegen das
  Bremsmoment durch `hoist.brake_safety_factor`).

```text
sim-cli verdicts overhead-crane --set effects.input_shaping=true
sim-cli verdicts overhead-crane
```

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
wird jenseits von zehn Millionen Punkten abgelehnt. Der Roboter auf seiner Schiene: Die Schiene schiebt mit -3 · 14,505 =
-43,515 N, während Gelenk 2 den ausgestreckten Arm mit 3 rad/s² schwenkt, Gelenk 2 hält -2 · 14,505 = -29,01 N m, während
die Schiene mit 2 m/s² beschleunigt, und die Schiene schiebt 130,5 · 2 = 261 N; die Führungswagen tragen 4 (1279,768 / 4 +
142,2455 / 1) = 1848,75 N in Ruhe bei quer ausgestrecktem Arm und 2656,85 N beim Beschleunigen, die Welle des Ritzels
261 · 1,071978 N; die Führungsurteile kippen an ihren Grenzen; im Werkzeugraum steht die Schiene auf 1e-12 m, wo die Lage
es sagt, und Vorwärts der Inversen gibt die Lage auf 1e-9 entlang der ganzen Schiene zurück.
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
Führungslast 30,084 N und das Moment an der Scheibe 4,28161 N m. Die Kniehebelpresse (Hebel 150 mm, Kurbel 50 mm um 120°
zum im Totpunkt waagerechten Pleuel, Pleuel 250 mm), gegen einen Kreisschnitt für das Knie und zentrale Differenzen: bei
−90° der Kurbel steht der Stößel 35,655 mm über dem unteren Totpunkt mit x' = 0,0364766 m/rad, bei −20° 1,792 mm und
0,0108256, bei −2° 0,0156 mm und 0,00090095, im Totpunkt x' = 0, φ' = −0,2886751, x'' = −0,025 m/rad²; 1 mm darüber
steht die Kurbel auf dem einen Ast bei −0,2646374 rad, auf dem anderen ebenso weit dahinter; 10 N m des Motors (100 N m
an der Kurbel) halten bei −90° 2741,5 N und bei −5° 42530 N; 10 kN brauchen 1 mm vor dem Totpunkt 79,387 N m an der
Kurbel, 0,1 mm davor 23,487 N m; das Gestell (5e7 N/m) federt unter den 20 kN Nennkraft 0,4 mm auf, aus den 12 kN der
Tabelle werden 11538,46 N bei 0,2308 mm Auffederung; J = 0,01859375 kg m² im Totpunkt und 0,05605024 fern davon; das
Kraftvermögen bei −90° ist 145,5 / 0,0364766 = 3988,85 N; ein Umformhub aus der Ruhe in die Ruhe nimmt im verlustfreien
Lauf 20,615 J vom Motor (19,284 J ins Teil, 1,331 J ins Gestell); in der Rast gibt der Motor weniger als ein Zehntel der
9,44 N m, die dieselbe Kraft 1 mm vor dem Totpunkt bräuchte; eine um 20 mm nach oben gelegte Umformkraft verfehlt
`press-capacity` und `motor-peak`.

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

Maschinentests (`machine-tripod`): die Standardprüfungen und der Standardzyklus mit allen Prüfungen. Waagrecht bei z =
0,54 m ist jedes Bein 0,5491812 m lang; um 0,15 rad um x geneigt verschiebt sich die Mitte um rp (1 − cos 0,15) / 2 =
1,1229 mm und die Beine sind 0,5489779, 0,5750463 und 0,5241851 m lang; bei (0,15, 0,1) rad stimmen Drehung und
Verschiebung der geschlossenen Form (−7,5203 mrad, 0,61207 und −1,49654 mm) mit ihrer zweiten Ordnung überein und halten
jedes Plattformgelenk in der Ebene seines Beins; vorwärts der inversen auf 1e-12 an 9³ Lagen; der Sehnenschritt zieht
über den vergrößerten Quader um weniger als 0,7 zusammen; waagrecht sind die Zeilen der Jacobi-Matrix u_z (1, rp sin
t_i, −rp cos t_i) und ihre Konditionszahl √2; die Beine halten die mittigen 62,7 kg mit m g / (3 u_z) = 208,4437 N
(0,331749 N m an den Motoren), die außermittige Last nach dem Momentengleichgewicht mit 274,9330 und 175,1991 N, und
drücken beim Heben mit 2 m/s² je 250,9544 N; der Impuls der Plattform auf einer nachgiebigen Basis trägt den Versatz der
Nutzlast; Wert und Grenze jeder neuen Prüfung von Hand, und die Grenzen der Gelenke lehnen die waagrechte Lage 0,1°
unter ihren 10,49° ab.


Maschinentests (`machine-wafer-handler`): die Standard-Testreihe mit jeder Mischung aus starren und elastischen Getrieben
und Bändern; bei φ = 60°, θ = 30° liegt das Handgelenk bei (0,277128, 0,16) auf der radialen Linie und die Wafermitte bei
(0,433013, 0,25), dr/dφ = −0,554256 m/rad; das Trägheitsmoment am R-Motor von Hand M_φφ = m₁ c₁² + I₁ + m₂ (L² + c₂² −
2 L c₂ cos 2φ) + I₂ + (m₃ + m_w) 4 L² sin² φ, bei r = 0,12 m 0,5499656 kg m² (2,485862e-4 kg m² am Motor), bei r = 0,75 m
0,1876677 (1,036671e-4); am θ-Motor 0,153033 und 0,604751 kg m²; dreht der Arm bei r = 0,75 m mit sqrt(μ g / r) =
1,980571 rad/s, beschleunigt der Wafer mit μ g = 2,941995 m/s², dem 1,2-Fachen der Grenze, und Motor 2 hält den Arm mit
0,0162955 N m; fährt er bei r = 0,5 m mit 0,5 m/s aus, während er sich mit 1 rad/s dreht, beschleunigt der Wafer mit
(−0,5, 1,0) m/s²; ein Moment von 1 N m am Effektor dehnt die Bänder um β = 2,955083e-4 und γ = 5,819704e-4 rad
(K₁ = 1692, K₂ = 2303 N m/rad) und dreht ihn um 3,300103e-4 rad aus seiner radialen Linie; die starren Bänder tragen
dT₁ = −4,271710 N und dT₂ = 4,379926 N bei θ̈ = 10 rad/s² mit dem Handgelenk über der Säule; der Z-Motor hält 11,2276 kg
mit 0,1752378 N m; die Eigenformen gegen die von numpy (18,20 bis 186,45 Hz) und die Säule auf Spindel und Kupplung, von Hand 320,8907 Hz; der Halt auf Pads, Vakuum und Kante; ein
Trum wird über seine Vorspannung hinaus schlaff; der Standardzyklus besteht alle Prüfungen, und eine Reichweite, die mit
3 m/s² anläuft, lässt den Wafer rutschen.

Tests des Pneumatikzylinders (`sim-components` pneumatic.rs, `sim-machines/tests/pneumatic.rs`): p A bei 6 bar auf
beiden Seiten (482,5 und 414,7 N, Festo druckt 483 und 415); der gesperrte Durchfluss C p ρ0 nach ISO 6358 und das
elliptische Gesetz; die Geschwindigkeit mit Abluftdrossel C ρ0 R T0 / A2 (0,1443 m/s auf 1 %, die des ausgelieferten Z
0,2156 m/s auf 3 %); die Eigenfrequenz auf den Luftfedern adiabat (29,64 Hz) und isotherm (25,05 Hz); die Luft eines
Doppelhubs (1,1119 Normliter); die geschlossene Füllzeit, solange der Durchfluss gesperrt ist (2,555 ms); die Energie
eines geschlossenen Zylinders und die Enthalpiebilanz eines offenen; die Hubzeit auf 1 % beim halben Schritt; die
Prüfurteile `cushion-energy` und `load-factor` im ausgelieferten Zyklus; ein für den Schritt zu steifer Zylinder wird
abgelehnt. Die Datei besteht die Standard-Testreihe mit dem Zylinder in Ruhe.

Tests des Wicklers (`machine-winder`): die Standard-Testreihe (starr und elastisch, mit und ohne das Gewicht auf dem
Tänzer) und das Halten in Ruhe an beiden Enden des Wickels und bei einem Viertel. Nach 640 m hat der Aufwickler
110,503 mm Radius, der Abwickler 110,969 mm (8231,352 und 4904,800 rad gedreht); der Wickel fasst 1286,482 m. Die
Trägheit des Wickels bei 110,503 mm ist 0,1366427 kg m², voll 0,4485586 kg m², das Trägheitsverhältnis 10,4026 beim
vollen Wickel und 0,2319 am Kern. Das Moment des Bahnzugs durch die Übersetzung: −2,142857 und 0,642857 N m am Anfang des
Wickels, −0,507857 und 1,692857 N m an seinem Ende (die 79 N der Kennlinie). Das Moment, um die Bahn bei 640 m um 2 m/s²
zu beschleunigen: 1,7824107 N m am Aufwickler in Ruhe, 1,7818049 N m bei 2 m/s, −0,8150135 und −0,8144080 N m am
Abwickler, mit den 2,5 N der Tänzerwalze zwischen den Abschnitten; im starren Modell beschleunigen diese Momente die Bahn
mit genau 2 m/s² bei ruhendem Tänzer. Die verlustfreien Läufe der Testreihe laufen in der Mitte des Wickels, wo die Radien
gleich sind; am Anfang des Wickels wandert der Zug, den ihre festen Haltemomente halten, mit den auseinanderlaufenden
Radien vom Wickelzug weg, und die Dehnung, die die Bahn hinein- und hinausträgt (`power.web_transport`), ist das, um was
sich die Energie ändert. Der stationäre Zug eines Abschnitts bei 1 gegen 1,0005 m/s: EA (v2 / v1 − 1) =
40,000 N (die Abschnittsgleichung in erster Ordnung 39,980 N). Der Tänzer bei 120 N Last und auf 100 N gehaltenen Wickeln
steht 39,9948 mm tief (die Radien der Wickel 2,88 µm kleiner, nachdem sie die Bahn abgegeben haben). Der Tänzer schwingt
auf der Bahn mit 26,071 Hz, seine Walze mit 56,941 Hz. Was den Abwickler verlässt, kommt am Aufwickler an oder bleibt in
den Abschnitten, auf 1e-7 m über 3 m Lauf. Ohne den Tänzerregler wandert der Tänzer bei einem mit 100 N gewickelten und
mit 80 N gefahrenen Wickel um −0,24969 mm/s bei 2 m/s, mit ihm stellt sich die Korrektur auf 0,49938 mm/s ein. Die eigenen
Prüfurteile: das Trägheitsverhältnis 10,4026, die Drehzahl am Kern 311,11 rad/s, jedes schlägt fehl, wenn sein Grenzwert
knapp unter seinem Wert liegt. Ein Halt der Kategorie 2 und, in der Mitte des Wickels, einer der Kategorie 1 halten die
Bahn zwischen 30 und 250 N; ein Halt der Kategorie 1 nahe einem Ende des Wickels überlastet sie.

## Grenzen

- Ein Glied biegt sich in einer Eigenform je Richtung (in der Ebene seines Gelenks; quer dazu und in Torsion mit
  `link_bending.directions`); mit `effects.link_bending_reaction` wirkt die Biegung auf die Gelenke zurück, ohne
  Lagranges Glieder zweiter Ordnung in der kleinen Biegung.
- Die Steifigkeit einer Spindel wird für den ganzen Hub an der weichsten Stellung der Mutter genommen.
- Der Roboter auf seiner Schiene: Der Schub der Zahnstange wird durch den Ursprung des Schlittens genommen (kein eigenes
  Moment auf die Wagen), die Kreiselmomente der Motorläufer des Arms fehlen in der Führungslast, und die Wagen sind
  starre Lager (keine Steifigkeit der Führung: der Schlitten kippt nicht auf ihnen); die Führungslast wird dem
  zeitoptimalen Plan nicht mitgeteilt (`rigid_guide_loads`), der nur die Grenzen der Motoren und Getriebe einhält.
- Ein Pneumatikzylinder ist keine Werkzeugachse und wirkt nicht auf die Achse zurück, die ihn trägt; sein Ventil
  schaltet auf und zu (kein Proportionalventil, kein servopneumatischer Lageregelkreis); ein Schlauch ist ein
  Durchflusselement, dessen Volumen in die Totvolumen geht; Festo druckt für die VUVG-Ventile kein C und b nach ISO 6358,
  daher sind die des ausgelieferten Ventils Schätzungen.
- Ein vom Abtrieb zurückgetriebenes Schneckengetriebe verliert wie jedes Getriebe (1 / Wirkungsgrad) und hemmt nie selbst.
- Keine Unwucht am Rundtisch, keine Lebensdauerformel der Zahnstange (keine veröffentlicht), keine Prüfung des
  Kreuzrollenlagers.
- Das Förderband aus `[conveyor]` kann noch nicht Leitwert einer Kurvenscheibe sein (der Leitwert ist ein eigener, mit
  dem Geschwindigkeits- und Rampenmodell des Bands); eine neue Leitgeschwindigkeit beginnt ihre Rampe bei der
  Geschwindigkeit des Leitwerts ohne seine Beschleunigung (ein Sprung der Beschleunigung, wo sie während einer Rampe
  kommt); der Stößel der Kurvenscheibe ist ein geradegeführter (kein Schwinghebel); die Presskraft der
  Schubkurbel wirkt nur beim Ausfahren (keine Rückfederung), die der Kniehebelpresse auch im Stillstand und klingt ab,
  wenn der Stößel steigt (die Auffederung des Gestells ist eine masselose Feder, die Hebel sind starr), die Schnittkraft von Messer und Säge nur vorwärts.
- Der Linear-Delta (`delta-linear`, drei Riemenachsen an Säulen, die sich neigen können,
  `geometry.column_incline_deg`) ist eine Parallelkinematik und keine dieser Maschinen; siehe die
  [Modellreferenz](../model-reference.md#linear-delta).

- Hexapod: Das Drehen der Beine um ihre Basisgelenke und die Trägheit der Nutzlast fehlen; eine Steifigkeit je Bein
  (an der weichsten Stellung der Mutter). Seine Eigenfrequenzen werden bei gehaltenen Motoren angegeben wie die der Roboter (`sim-cli modes hexapod`).
- Tripod: Das Schwenken der Beine um ihre Basisgelenke fehlt wie am Hexapod; keine Reibung und kein Spiel in Dreh- und
  Kugelgelenken; eine Steifigkeit je Bein (an der weichsten Stellung der Mutter); die Konvergenz der Vorwärtskinematik
  ist für den ausgelieferten Quader gezeigt (eine Konfiguration mit anderer Geometrie oder anderem Quader sollte die
  Prüfung jenes Tests wiederholen).

- Wickler: beide Wickel haben denselben Kern, vollen Radius, Motor und dasselbe Getriebe; die Masse der Bahn in den
  Abschnitten, die Änderung der Abschnittslängen mit den Radien der Wickel, die Querkontraktion der Bahn und die
  Nachgiebigkeit des Wickels selbst (gleitende Lagen, Teleskopieren) fehlen; der Radius wird aus der gewickelten Länge
  berechnet (ein gemessener Durchmesser oder das Drehzahlverhältnis, `rewind.radius_from_speeds`, ist nur ein Signal);
  die Reibung der Wellendichtungen folgt der Temperatur der Getriebe nicht (ihr Wirkungsgrad und ihr Leerlaufmoment
  schon); der ausgelieferte Zyklus fährt 3 m der 1286 m des Wickels an seinem Anfang, das andere Ende des Wickels wird über
  die eigenen Prüfurteile beurteilt, nicht gefahren; Überschleifen hat auf der einen, umkehrenden Achse nichts zu
  überschleifen; der Antrieb passt seinen Drehzahlregler nicht an die Trägheit des Wickels an (45-fach vom Kern bis
  voll), darum sind die ausgelieferten Verstärkungen die des ausgelieferten Abschnitts (`[drive.unwind]` eine eigene),
  und ein Lauf nahe dem Ende des Wickels braucht sie vertauscht.
- Seilroboter: Die Trägheit der Seile fehlt (ein Drittel der Masse jedes Seils bewegt sich mit seinem Anlenkpunkt,
  0,5 % der Masse der räumlichen Plattform), ebenso das mit der Lage wandernde Gewicht des Durchhangs (5e-5 N); der
  kraftfähige Arbeitsraum der Lagen ist der statische (die Beschleunigungen eines Zyklus hält die zeitoptimale Planung
  im Band, in den Achsgrenzen beurteilen sie die Seilkraftprüfungen); nahe am Rand des Arbeitsraums rechnet
  der Dehnungsausgleich der Motoren mit den Kräften der geschlossenen Form, wo die Verteilung ein Seil an einer Grenze
  hält.
