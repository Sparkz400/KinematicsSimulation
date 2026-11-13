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
| `urdf-arm` | Sechsachsroboter, `urdf/six-axis.urdf` | Wellgetriebe i = 120 und 80 | sechs Gelenkwinkel (rad) | `strain-wave-life` je Gelenk |
| `scara` | SCARA-Roboter, `urdf/scara.urdf` | Wellgetriebe; die Pinole auf einem Kugelgewindetrieb | Gelenke (rad, m, rad) | `strain-wave-life`, die Prüfungen der Spindel an Gelenk 3 |

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
  eingestellten Getriebe geführt.
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
