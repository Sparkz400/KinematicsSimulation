# Benutzerhandbuch

Ein Abschnitt je Aufgabe. Jeder nennt den Befehl, erklärt die Ausgabe und führt die Grenzen auf, die für diese Aufgabe zählen. Die Physik dahinter steht in der [Modellreferenz](../model-reference.md), die Konfigurationsschlüssel stehen in der [Konfigurationsreferenz](../config-reference.md).

Die Referenzdokumente, auf die dieses Handbuch verweist, gibt es nur auf Englisch. Die englische Fassung dieses Handbuchs ist [user-guide.md](../user-guide.md).

- [Für alle Befehle gleich](#für-alle-befehle-gleich)
- [Einen Motor für einen Zyklus auslegen](#einen-motor-für-einen-zyklus-auslegen)
- [Eine reale Katalogmaschine prüfen](#eine-reale-katalogmaschine-prüfen)
- [Den leichtesten Motor finden](#den-leichtesten-motor-finden)
- [Einfluss unsicherer Parameter erkennen und mit Fehlerbändern beurteilen](#einfluss-unsicherer-parameter-erkennen-und-mit-fehlerbändern-beurteilen)
- [Einen Zyklus verkürzen](#einen-zyklus-verkürzen)
- [Eine Bahn fahren und den Bahnfehler ablesen](#eine-bahn-fahren-und-den-bahnfehler-ablesen)
- [Regler einstellen und Input Shaping nutzen](#regler-einstellen-und-input-shaping-nutzen)
- [Eine Antriebsaufzeichnung nachfahren und Parameter anpassen](#eine-antriebsaufzeichnung-nachfahren-und-parameter-anpassen)
- [Signale aufzeichnen, Antriebe sperren](#signale-aufzeichnen-antriebe-sperren)
- [Eine SPS über OPC UA anbinden](#eine-sps-über-opc-ua-anbinden)
- [Die App bedienen](#die-app-bedienen)

In den Beispielen steht `sim-cli` für `cargo run -q --release -p sim-cli --`. Jede gezeigte Ausgabe stammt von dem Befehl, der darüber steht. Wo ein Befehl Meldungen ausgibt, lief er mit `--lang de`; in den Beispielen ist die Option der Kürze halber weggelassen. Die Spaltennamen der CSV-Ausgabe und die Namen der Prüfungen sind englische Bezeichner und in beiden Sprachen gleich.

## Für alle Befehle gleich

```text
sim-cli COMMAND MACHINE [options]
```

`MACHINE` ist `t-gantry` (T-Portal) oder `single-axis` (Einzelachse); `sim-cli list` gibt beide aus. Diese Optionen kennt jeder Befehl außer `list`:

| Option | Wirkung |
| --- | --- |
| `--config FILE` | Diese Konfiguration statt der Standardkonfiguration der Maschine verwenden. Die Konfiguration der EXCT-30 ist `crates/machine-t-gantry/exct-30.toml` |
| `--motor ARTICLE` | Einen Motor aus dem Katalog Siemens S210 in die Konfiguration einsetzen, z. B. `1FK2205-4AF`. Setzt `parts.motor` |
| `--supply 230v` oder `400v` | Die Netzspannung, für die die Motordaten gelten. Setzt `parts.motor_supply`. Ohne Angabe gilt der Wert der Konfiguration |
| `--gearbox SIZE-FRAME-RATIO` | Ein Kataloggetriebe in die Konfiguration einsetzen (Baugröße-Flansch-Übersetzung), z. B. `NRK070-04-5`. Setzt `parts.gearbox` |
| `--set path=value` | Einen einzelnen Konfigurationswert ändern, z. B. `--set mass.payload_kg=3.0`. Mehrfach möglich. Wird nach den Bauteilen angewendet; einzelne Werte eines Katalogbauteils lassen sich so überschreiben |
| `--out FILE` | Die CSV-Daten in eine Datei schreiben statt auf die Standardausgabe |
| `--lang en` oder `de` | Sprache der Meldungen. Die Spaltennamen der CSV-Ausgabe werden nicht übersetzt |

Regeln, über die man leicht stolpert:

- `--set` ändert nur Werte, die es gibt. Ein falsch geschriebener Schlüssel wird abgelehnt (`Konfigurationswert fehlt: mass.cariage_kg`), ein Wert vom falschen Typ ebenso.
- `--motor`, `--supply` und `--gearbox` brauchen eine Tabelle `[parts]` in der Konfiguration. Alle drei mitgelieferten Konfigurationen haben eine.
- Jede Option außer `--cycle`, `--time-optimal` und `--bands` erwartet einen Wert. Eine Option ohne ihren Wert wird abgelehnt (`Dieser Option fehlt ihr Wert: --time`), ebenso eine Option, die der Befehl nicht kennt, etwa eine falsch geschriebene (`Diese Option kennt der Befehl nicht: --cylce`). Eine Zahl muss endlich sein: `--time inf` wird abgelehnt.
- `--cycle`, `--move`, `--trajectory` und `--path` schließen einander aus (`Von diesen Optionen kann nur eine zugleich angegeben werden: --cycle --move`).
- Eine Konfigurationsdatei darf nur Schlüssel enthalten, die die Standardkonfiguration der Maschine hat: Ein falsch geschriebener Schlüssel wird abgelehnt (`Die Konfiguration enthält einen Wert, den diese Maschine nicht kennt: mass.cariage_kg`). Ausgenommen sind `[bounds]`, das seine eigene Prüfung hat, und `ratings.pulley_max_torque_nm`.
- Zahlen werden beim Lesen der Konfiguration geprüft. NaN und unendlich werden für jeden Wert abgelehnt, eine negative Zahl dort, wo die Größe nicht negativ sein kann (eine Nutzlast, eine Dämpfung, ein Spiel, die Vorspannung des Riemens), und eine Startpose außerhalb der Verfahrbereiche. Die Meldung nennt den Schlüssel: `Konfigurationswert liegt außerhalb des gültigen Bereichs: gearbox.backlash_rad`.
- Ein Fehler gibt eine Zeile auf der Fehlerausgabe aus; der Befehl endet mit dem Code 2.
- Verlangen die Achsgrenzwerte mehr von einem Motor, als die Antriebsdaten zulassen, geben Befehle, die unmittelbar eine Simulation starten (`run`), eine Warnung auf der Fehlerausgabe aus und laufen weiter:

  ```text
  $ sim-cli run t-gantry --set axis.x.max_velocity_m_s=2.0 --time 0.01 --out x.csv
  Die Achsgrenzwerte verlangen mehr Motordrehzahl, als der Antrieb zulässt: Motor A: 600.0000 > 314.0000
  Die Achsgrenzwerte verlangen mehr Motordrehzahl, als der Antrieb zulässt: Motor B: 600.0000 > 314.0000
  ```

  Verglichen wird die Motordrehzahl bei den höchsten Achsgeschwindigkeiten mit `drive.rated_speed_rad_s` und das Drehmoment des Starrkörpermodells bei den höchsten Achsbeschleunigungen mit `drive.peak_torque_nm`, jeweils für jede Kombination der Achsrichtungen.

## Einen Motor für einen Zyklus auslegen

Der Zyklus ist die Liste der `[[cycle.segment]]` in der Konfiguration: Zielpose, Verweilzeit und ob die Nutzlast bei der Ankunft angehängt ist. Um ihn zu ändern, bearbeiten Sie eine Kopie der Konfiguration; mit `--set` lässt sich der Zyklus nicht ändern.

```text
$ sim-cli analyze t-gantry
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km
a,6.080000,0.644237,0.166790,200.2390,0.1709,0.1313,202.62,334.58,0.3708,1.25,85.244,10.435,29.12,5.954,1.319e-4,4.409e-5,1.559e-4,4.588e-5,0.198428,0.000,0.000,222.98,0.0000e0,6.4544e2,1.8007e2,4.6452e6
b,6.080000,0.466063,0.154163,200.2192,0.1227,0.1214,202.62,334.58,0.3427,1.07,74.769,15.322,35.58,7.241,1.319e-4,4.409e-5,1.559e-4,4.588e-5,0.176294,5.281,0.869,224.65,0.0000e0,6.4544e2,1.8007e2,4.6452e6
```

Der Simulator startet den Zyklus, lässt den ersten Durchlauf verstreichen und misst den zweiten. Je Motor entsteht eine Zeile. Spalten, die die ganze Maschine beschreiben und nicht einen Motor, wiederholen sich in jeder Zeile.

| Spalte | Einheit | Bedeutung |
| --- | --- | --- |
| `cycle_s` | s | Dauer des gemessenen Durchlaufs (Zykluszeit) |
| `peak_torque_nm` | N m | Spitzenmoment: größter Betrag des Motormoments (aus dem Wicklungsstrom, an der Motorwelle) |
| `rms_torque_nm` | N m | Effektivmoment des Motors über den Durchlauf |
| `peak_speed_rad_s` | rad/s | Größter Betrag der Motordrehzahl |
| `utilisation` | Anteil | Genutzter Anteil der Drehmoment-Drehzahl-Kennlinie: größtes Verhältnis zwischen dem Moment, das der Drehzahlregler verlangt (vor der Momentengrenze), und dem Moment, das die Kennlinie bei dieser Drehzahl hergibt. Über 1 verlangt der Zyklus mehr, als die Kennlinie hergibt: Der Antrieb fährt an seiner Momentengrenze, und die Achse bleibt zurück |
| `thermal_utilisation` | Anteil | Effektivmoment geteilt durch `drive.rated_torque_nm`. Nur zur Information; die Prüfung `motor-thermal` beurteilt die Übertemperatur der Wicklung, und die beruht auf `motor.rated_current_a` |
| `belt_tension_min_n`, `_max_n` | N | Kleinste und größte Riemenkraft in einem beliebigen Riementrum, einmal je Millisekunde betrachtet. Gilt für die ganze Maschine |
| `rms_current_a` | A | Effektivwert des Wicklungsstroms |
| `winding_rise_k` | K | Übertemperatur der Wicklung, eingeschwungen: der Wert, auf den sie sich einstellt, wenn sich dieser Zyklus lange wiederholt |
| `energy_in_j` | J | Aufgenommene Energie: elektrische Energie, die die Wicklung bei positiver Leistung aufgenommen hat |
| `energy_regenerated_j` | J | Rückgespeiste Energie beim Bremsen |
| `peak_regenerated_power_w` | W | Höchste rückgespeiste Leistung |
| `largest_regeneration_j` | J | Größte Energie, die in einer einzelnen Bremsung zurückgespeist wurde (ein ununterbrochener Abschnitt negativer Leistung) |
| `path_error_peak_m`, `_rms_m` | m | Bahnfehler: Abstand des Werkzeugs von der Bahn, die sein Sollwert gezeichnet hat. Gilt für die ganze Maschine |
| `tracking_error_peak_m` | m | Abstand des Werkzeugs von der Stelle, an der sein Sollwert im selben Augenblick war. Gilt für die ganze Maschine |
| `dwell_error_peak_m` | m | Größter Abstand vom Ziel beim Verweilen, gemessen in der zweiten Hälfte einer Verweilzeit. Gilt für die ganze Maschine |
| `cubic_mean_torque_nm` | N m | Kubisches Mittel des Moments: dritte Wurzel aus dem Mittelwert der dritten Potenz des Moments, gewichtet mit der Drehzahl, und zwar des Motormoments. Die Prüfung `gearbox-continuous` bildet dasselbe Mittel aus dem Getriebe-Abtriebsmoment |
| `braking_resistor_energy_j` | J | Energie, die der simulierte Bremswiderstand in Wärme umgesetzt hat. 0 ohne `effects.dc_link` |
| `braking_resistor_mean_power_w` | W | Diese Energie geteilt durch die Zykluszeit |
| `dc_link_peak_voltage_v` | V | Höchste Zwischenkreisspannung. Bleibt ohne `effects.dc_link` bei `drive.bus_voltage_v` |
| `belt_slack_time_s` | s | Zeit, in der irgendein Riementrum ohne Riemenkraft war. Gilt für die ganze Maschine |
| `radial_load_peak_n` | N | Größte Radialkraft des Riemens auf eine Scheibenwelle. Gilt für die ganze Maschine |
| `guide_load_peak_n` | N | Größte Führungsbelastung, und zwar der Führung mit der höchsten äquivalenten Belastung. Gilt für die ganze Maschine |
| `guide_life_km` | km | L10 = (C / P)³ · 50 km für diese Führung. Leer ohne `ratings.guide_dynamic_load_rating_n` |

Die genauen Definitionen stehen in der [Modellreferenz](../model-reference.md#analysis-quantities).

`--points FILE` schreibt zusätzlich die Betriebspunkte, einen je Millisekunde und Motor (`motor,speed_rad_s,torque_nm`), um sie über der Drehmoment-Drehzahl-Kennlinie aufzutragen.

Danach beurteilen Sie den Zyklus:

```text
$ sim-cli verdicts t-gantry
check,motor,value,limit,result
motor-peak,a,0.1709,1.000,ok
motor-thermal,a,1.248,100.0,ok
gearbox-continuous,a,0.7995,40.00,ok
gearbox-input-speed,a,1912,13000,ok
gearbox-peak,a,2.228,64.00,ok
dc-link-energy,a,6.029,6.205,ok
dc-link-overvoltage,a,223.0,257.6,ok
...
settling,,0.00004588,0.0001000,ok
belt-tension,,334.6,1500,ok
belt-slack,,202.6,0,ok
gearbox-radial-load,,645.4,1500,ok
guide-load,,180.1,2000,ok
```

Die Namen der Prüfungen sind Bezeichner. In der App heißt jede Prüfung so, wie in der zweiten Spalte angegeben.

| Prüfung | Name in der App | Wert | Grenze | Einheit |
| --- | --- | --- | --- | --- |
| `motor-peak` | Genutzter Anteil der Drehmoment-Drehzahl-Kennlinie | `utilisation` (das verlangte Moment, nicht das abgegebene) | 1 | Anteil |
| `motor-thermal` | Übertemperatur der Wicklung, eingeschwungen | `winding_rise_k` | `motor.rated_temperature_rise_k` | K |
| `pulley-torque` | Moment an der Riemenscheibe | Spitze des Getriebe-Abtriebsmoments (Signal `gearbox.<motor id>.output_torque`) | `ratings.pulley_max_torque_nm` | N m |
| `gearbox-continuous` | Getriebe-Abtriebsmoment, kubisches Mittel | kubisches Mittel des Getriebe-Abtriebsmoments, gewichtet mit der Drehzahl | `ratings.gearbox_rated_torque_nm` | N m |
| `gearbox-input-speed` | Getriebe-Antriebsdrehzahl, Spitze | höchste Motordrehzahl | `ratings.gearbox_max_input_speed_rpm` | 1/min |
| `gearbox-peak` | Getriebe-Abtriebsmoment, Spitze | Spitze des Getriebe-Abtriebsmoments, in jedem Simulationsschritt betrachtet | `ratings.gearbox_max_torque_nm` | N m |
| `dc-link-energy` | Größte Bremsenergie in den Zwischenkreis | Energie in die Zwischenkreiskondensatoren | Raum zwischen der Zwischenkreisspannung (`drive.bus_voltage_v`) und der Einschaltspannung des Bremswiderstands | J |
| `braking-resistor-continuous` | Bremswiderstand, mittlere Leistung | mittlere Leistung am Bremswiderstand | `ratings.braking_resistor_continuous_w` | W |
| `braking-resistor-peak` | Bremswiderstand, Spitzenleistung | Spitzenleistung am Bremswiderstand | `ratings.braking_resistor_peak_w` | W |
| `dc-link-overvoltage` | Höchste Zwischenkreisspannung | höchste Zwischenkreisspannung (nur mit `effects.dc_link`) | `ratings.dc_link_overvoltage_v` | V |
| `settling` | Abstand vom Ziel beim Verweilen | `dwell_error_peak_m` | `ratings.settling_tolerance_m` | m |
| `belt-tension` | Größte Riemenkraft | größte Riemenkraft | `ratings.belt_max_tension_n` | N |
| `belt-slack` | Kleinste Riemenkraft | kleinste Riemenkraft | `ratings.belt_min_tension_n` | N |
| `gearbox-radial-load` | Radialkraft auf eine Scheibenwelle, Spitze | `radial_load_peak_n` | `ratings.shaft_max_radial_load_n` | N |
| `guide-load` | Größte Führungsbelastung | `guide_load_peak_n` | `ratings.guide_max_load_n` | N |

Grenzen bei dieser Aufgabe:

- Ein Grenzwert, den die Konfiguration nicht angibt, wird nicht geprüft. Eine fehlende Zeile heißt nicht „bestanden“.
- `belt-slack` ist die einzige Prüfung, bei der der Wert über der Grenze bleiben muss. Sie fällt auch dann durch, wenn irgendein Trum irgendwann ohne Riemenkraft war.
- `dc-link-energy` ist immer `ok`, wenn die Konfiguration einen Bremswiderstand hat, der einspringen kann, auch wenn der Wert die Grenze erreicht oder überschreitet. Die beiden Zeilen zum Bremswiderstand erscheinen nur, wenn er gebraucht wird. Alle drei Standardkonfigurationen simulieren den Zwischenkreis (`effects.dc_link = true`); ist er abgeschaltet, sind die Zeilen zum Bremsen Schätzungen aus dem, was der Motor zurückspeist.
- Eine Einschaltspannung des Bremswiderstands, die nicht über `drive.bus_voltage_v` liegt, wird abgelehnt (`Konfigurationswert liegt außerhalb des gültigen Bereichs: ratings.braking_threshold_v`), ob simuliert oder nicht: Sie ist ein Grenzwert für eine andere Zwischenkreisspannung.
- Werte werden mit vier signifikanten Stellen und ohne Exponent ausgegeben; deshalb erscheint `settling` als `0.00004588`.
- `motor-peak` beurteilt, was der Drehzahlregler vor der Momentengrenze verlangt. Ein Zyklus, der an der Momentengrenze fährt, besteht die Prüfung nicht; das abgegebene Moment, das die Kennlinie nie überschreitet, ist `peak_torque_nm` aus `analyze`.
- Mit Input Shaping warten eine Verweilzeit und der Greifer, bis der geformte Sollwert angekommen ist, also eine Periode des Shapers nach dem Plan. `settling` wird dann ab diesem Zeitpunkt beurteilt.
- Die Momente an Getriebe und Riemenscheibe sind das Getriebe-Abtriebsmoment, das Signal `gearbox.<motor id>.output_torque`. Im elastischen Modell ist das, was die Feder des Getriebes an die Riemenscheibe weitergibt, mit dem Wirkungsgrad; im Starrkörpermodell ist es der Wert, den der Löser dafür ermittelt. Das Moment, das den Rotor selbst beschleunigt, zählt also nicht gegen das Getriebe, und eine Last, die das Getriebe rückwärts treibt (Bremsen, Absenken), zählt mit dem, was sie auf den Abtrieb bringt. `cubic_mean_torque_nm` aus `analyze` ist weiterhin das des Motors.
- Die Motor-, Reibungs- und Antriebswerte der Standardkonfigurationen sind Platzhalter. Worauf die absoluten Zahlen beruhen, steht im README.

**Ein Zwischenkreis für alle Antriebe.** `effects.shared_dc_link` (Gemeinsamer Zwischenkreis aller Antriebe; abgeschaltet, braucht `effects.dc_link`) gibt den Antrieben einer Maschine einen gemeinsamen Zwischenkreis, wie an einem Line Module eines SINAMICS S120. Was ein bremsender Motor zurückspeist, nimmt ein treibender auf; nur die Summe der Leistungen lädt die Kondensatoren. Der Zwischenkreis hat die Kapazität aller Antriebe zusammen (jeder bringt `ratings.dc_link_capacitance_uf` mit), einen Bremschopper mit einem Bremswiderstand der Leistungen aus `[ratings]` und eine Abschaltung. Jeder Motor zeigt dann dieselbe Zwischenkreisspannung und dieselbe Leistung am Bremswiderstand, und die Prüfungen zum Bremsen erscheinen einmal je Motor mit denselben gemeinsamen Werten. An einer Maschine mit einem Antrieb ändert der Schalter nichts.

```text
$ sim-cli verdicts t-gantry --set effects.shared_dc_link=true
...
dc-link-energy,a,6.142,6.205,ok
dc-link-overvoltage,a,224.1,257.6,ok
...
dc-link-energy,b,6.142,6.205,ok
dc-link-overvoltage,b,224.1,257.6,ok
...
```

Im Standardzyklus ist die höchste Zwischenkreisspannung 224,65 V mit getrennten Zwischenkreisen (Motor B, an der Einschaltspannung des Bremschoppers) und 224,06 V mit dem gemeinsamen. Der Bremswiderstand wird dann nicht gebraucht; seine beiden Zeilen fehlen.

## Eine reale Katalogmaschine prüfen

Eine Konfiguration kann eine reale Maschine mit Werten aus dem Katalog beschreiben. [`exct-30.toml`](../../crates/machine-t-gantry/exct-30.toml) tut das für eine Festo EXCT-30 mit zwei Motoren Siemens 1FK2205-4AF an Umrichtern S210 bei 400 V. Jeder Wert darin ist mit `[catalogue]` (Katalog) oder `[estimate]` (Schätzung) gekennzeichnet.

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,...
a,1.934000,4.060260,1.547724,157.1821,0.3251,0.3365,239.14,558.35,1.2137,7.76,...
b,1.934000,3.353654,1.483132,157.1735,0.2686,0.3224,239.14,558.35,1.1628,7.11,...
```

Vier Werte dieser Konfiguration hat der Konstrukteur so gewählt, dass jede Prüfung auch im ungünstigsten Fall der Fehlerbänder besteht ([unten](#einfluss-unsicherer-parameter-erkennen-und-mit-fehlerbändern-beurteilen)): eine Vorspannung des Riemens von 450 N, einen Ruckgrenzwert in X von 150 m/s³, Input Shaping mit festen 25 Hz und das Überschleifen. Die Gründe stehen in den Kommentaren der Datei.

Wie die Standardkonfigurationen hat auch diese `effects.dc_link = true`. Die Zeilen zum Bremsen in `verdicts` sind also simuliert, nicht geschätzt: Die Zwischenkreisspannung steigt bis zur Einschaltspannung des Bremschoppers von 770 V, und der Bremswiderstand setzt den Rest in Wärme um.

```text
dc-link-energy,a,9.943,9.943,ok
braking-resistor-continuous,a,8.859,100.0,ok
braking-resistor-peak,a,213.7,3000,ok
dc-link-overvoltage,a,770.0,820.0,ok
```

**Bauteile nach Namen.** `[parts]` in einer Konfiguration oder `--motor`, `--supply` und `--gearbox` auf der Befehlszeile benennen einen Motor und ein Getriebe aus dem Katalog. Deren Katalogwerte ersetzen dann die Zahlen der Konfiguration:

| Bauteil | Was es setzt |
| --- | --- |
| Motor | `motor.rotor_inertia_kgm2`, `motor.torque_constant_nm_a` (Stillstandsmoment geteilt durch Stillstandsstrom), `motor.max_current_a`, `motor.torque_constant_at_max_current_nm_a`, `motor.rated_current_a` (der Wert des Katalogs, höchstens das 1,05-Fache des Stillstandsstroms), `motor.thermal_time_constant_s` (wo das Projektierungshandbuch sie nennt), `drive.peak_torque_nm` (begrenzt auf `ratings.pulley_max_torque_nm` geteilt durch die Getriebeübersetzung), `drive.rated_torque_nm`, `drive.rated_speed_rad_s`, `drive.bus_voltage_v` (die Zwischenkreisspannung: 540 V bei `400v`, 325 V bei `230v`), `drive.no_load_speed_rad_s` (sqrt(3/2) × Zwischenkreisspannung geteilt durch die Drehmomentkonstante) sowie `ratings.braking_threshold_v` und `ratings.dc_link_overvoltage_v`, beide mit der Zwischenkreisspannung skaliert |
| Getriebe | `gearbox.ratio`, `gearbox.efficiency`, `gearbox.stiffness_nm_rad`, `gearbox.backlash_rad` und `gearbox.inertia_at_input_kgm2` (die beiden letzten, wo das Handbuch sie nennt), `ratings.gearbox_rated_torque_nm` und `ratings.gearbox_max_torque_nm` (wo die Konfiguration sie hat) |
| Beide | `drive.velocity_kp_nm_s_rad`, für das Bauteil nachgeführt (siehe unten) |

Was ein Bauteil nach Namen **nicht** setzt und was daraus folgt:

- **Wicklungswiderstand und Wicklungsinduktivität.** Siemens veröffentlicht sie für 1FK2 und 1FT2 nicht. Die Werte der Konfiguration bleiben, und mit ihnen die Verstärkungen des Stromreglers.
- **Die Zwischenkreiskapazität und die Leistungen des Bremswiderstands.** Sie bleiben, wie die Konfiguration sie hat. Die Einschaltspannung des Bremswiderstands und die Abschaltspannung behalten ihr Verhältnis zur Zwischenkreisspannung. Für Platzhalter ist das vertretbar, mehr nicht: Setzen Sie alle fünf Zwischenkreis-Grenzwerte für den realen Antrieb, bevor Sie den Zeilen zum Bremsen trauen.
- **Antriebsdrehzahl und Radialkraft des Getriebes.** Die Getriebedateien enthalten sie nicht. `ratings.gearbox_max_input_speed_rpm` und `ratings.shaft_max_radial_load_n` bleiben, wie die Konfiguration sie hat.

**Die Drehzahlverstärkung wird nachgeführt.** Ein Motor mit deutlich leichterem Rotor schwingt mit einer Verstärkung, die für einen schwereren eingestellt wurde. Das ist ein Fehler der Reglereinstellung, nicht des Motors. Überall, wo ein Bauteil benannt wird (`[parts]`, `--motor`, `--gearbox`, die Suche), wird `drive.velocity_kp_nm_s_rad` deshalb skaliert: mit der Quadratwurzel aus (Verhältnis der Trägheiten, die die Motoren drehen) × (Verhältnis der Rotorträgheiten, die Trägheit des Getriebes eingeschlossen). Bezugspunkt sind die Zahlen der Konfiguration, wie die Datei sie nennt, bevor das Bauteil eingesetzt wird: Die Verstärkung in einer Konfiguration gilt als eingestellt für den Rotor, das Getriebe und die Massen, die in dieser Konfiguration stehen. `--set drive.velocity_kp_nm_s_rad=...` auf der Befehlszeile wird nach den Bauteilen angewendet und überschreibt die nachgeführte Verstärkung.

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --motor 1FK2204-6AF
motor-peak,a,0.4001,1.000,ok
motor-thermal,a,13.38,100.0,ok
...
settling,,0.00003980,0.0001000,ok
belt-slack,,236.9,0,ok
```

Der Rotor des 1FK2204-6AF hat ein Drittel der Trägheit des 1FK2205-4AF aus der Konfiguration; die Verstärkung geht von 0,6 auf 0,31 N m s/rad. Bleibt die Verstärkung bei 0,6, schwingt derselbe Motor, und seine Wicklung würde sich um Tausende Kelvin erwärmen.

Eine unbekannte Artikelnummer wird abgelehnt: `Der Katalog kennt dieses Bauteil nicht: Motor 1FK9`. Ebenso ein Motor, den der Katalog für die gewählte Netzspannung nicht anbietet.

## Den leichtesten Motor finden

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --series 1FK2
Ohne Simulation eines Zyklus ausgeschlossen, nach Katalogwerten gegen die Achsgrenzwerte: 13 / 28 Kandidaten
motor,gearbox,mass_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
1FK2104-5AF,,2.85,ok,settling,0.5083,3.5749,22.73
1FK2104-5AK,,2.85,ok,settling,0.5081,3.5812,46.58
1FK2204-5AF,,2.90,ok,motor-peak,0.5260,3.7239,23.92
1FK2104-6AF,,3.40,ok,gearbox-radial-load,0.5026,3.6284,12.40
1FK2204-6AF,,3.50,ok,gearbox-radial-load,0.5030,3.7905,13.38
1FK2205-2AF,,3.75,ok,gearbox-radial-load,0.5026,3.9032,16.44
1FK2205-4AF,,5.20,ok,gearbox-radial-load,0.5025,4.0603,7.76
...
1FK2204-5AK,,2.90,FAIL,motor-thermal,1.9796,3.7279,197.96
1FK2103-2AH,,1.18,FAIL,limit-motor-torque,NaN,NaN,NaN
...
1FK2208-3AC,,12.60,FAIL,limit-motor-speed,NaN,NaN,NaN
```

Die Suche fährt den Zyklus der Konfiguration mit jedem Motor des Katalogs S210 ([`motors_s210.csv`](../../catalog/siemens/motors_s210.csv)), der für die Netzspannung angeboten wird. Zuerst kommen die Motoren, die bestehen, der leichteste voran. Danach folgen die, die nicht bestehen, der knappste voran. Ein Kandidat, dessen Antriebe wegen Überspannung im Zwischenkreis abschalten, steht hinter allen diesen, mit `inf` als `tightest_utilisation`: Seine Werte enden mit dem Abschalten.

| Option | Wirkung |
| --- | --- |
| `--series PREFIX` | Nur Artikelnummern, die so beginnen, z. B. `1FK2` oder `1FT2105` |
| `--supply 230v` oder `400v` | Netzspannung, für die gesucht wird. Standard `400v` |
| `--gearboxes SERIES` | Jeden Motor zusätzlich mit jedem Getriebe dieser Baureihe probieren (`NRB`, `NRK`, `NLC` oder eine Baugröße wie `NRK070`), das an seinen Flansch passt und dessen Übersetzung zwischen der Hälfte und dem Doppelten der Übersetzung der Konfiguration liegt. Nur bei einer Maschine, deren Konfiguration eine Übersetzung über 1 hat |

| Spalte | Bedeutung |
| --- | --- |
| `motor`, `gearbox` | Der Kandidat. `gearbox` ist leer, wo das Getriebe der Konfiguration geblieben ist |
| `mass_kg` | Masse des Motors ohne Bremse |
| `result` | `ok`, wenn jede Prüfung bestanden ist |
| `tightest_check` | Die Prüfung, die ihrer Grenze am nächsten kommt oder sie am weitesten überschreitet. Zwei Zwischenkreisprüfungen bleiben außen vor, solange sie bestehen, weil ihr Verhältnis nichts über die Reserve sagt: `dc-link-energy`, wenn ein Bremswiderstand arbeitet (die Kondensatoren füllen sich dann planmäßig bis zur Grenze), und `dc-link-overvoltage` (die Spannung liegt nie unter dem Ruhewert des Zwischenkreises). Bei einem Kandidaten, der nicht simuliert wurde: der Grund (`limit-motor-speed`, `limit-motor-torque`) |
| `tightest_utilisation` | Wert geteilt durch Grenze dieser Prüfung |
| `peak_torque_nm` | Höchstes Spitzenmoment aller Motoren der Maschine |
| `winding_rise_k` | Höchste eingeschwungene Übertemperatur der Wicklung über alle Motoren |

Bevor ein Zyklus simuliert wird, werden die Katalogwerte gegen die Achsgrenzwerte geprüft: die Motordrehzahl bei der höchsten Achsgeschwindigkeit gegen die Bemessungsdrehzahl, das Moment des Starrkörpermodells bei der höchsten Beschleunigung gegen das Maximalmoment des Motors. Was hier scheitert, wird mit `NaN` aufgeführt und nicht simuliert.

Die Drehzahlverstärkung jedes Kandidaten wird nach der Regel aus [Bauteile nach Namen](#eine-reale-katalogmaschine-prüfen) nachgeführt: Quadratwurzel aus (Verhältnis der Trägheiten, die die Motoren drehen) × (Verhältnis der Rotorträgheiten), bezogen auf die Zahlen der Konfiguration. `--gearbox NAME` für die ganze Suche und `--gearboxes SERIES` liefern für dasselbe Paar dasselbe Ergebnis: Die Trägheit des Getriebes hat einen eigenen Schlüssel (`gearbox.inertia_at_input_kgm2`) und bleibt, wenn ein Motor eingesetzt wird.

Mit Getrieben, an der Einzelachse:

```text
$ sim-cli search single-axis --series 1FK2104 --gearboxes NRK070
Ohne Simulation eines Zyklus ausgeschlossen, nach Katalogwerten gegen die Achsgrenzwerte: 12 / 35 Kandidaten
motor,gearbox,mass_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
1FK2104-4AF,NRK070-04-3,2.05,ok,settling,0.4749,0.8992,3.97
1FK2104-4AF,NRK070-04-4,2.05,ok,settling,0.4632,0.7356,2.50
1FK2104-4AF,NRK070-04-5,2.05,ok,settling,0.4814,0.5543,1.91
...
1FK2104-4AF,NRK070-04-7,2.05,FAIL,limit-motor-speed,NaN,NaN,NaN
```

Grenzen bei dieser Aufgabe:

- **Das Gewicht steht für den Preis.** Keine Katalogdatei enthält Preise. Das Gewicht ist das des Motors; die Getriebedateien enthalten keine Masse.
- Das Nachführen der Verstärkung ist eine Faustregel, keine Reglereinstellung nach dem Frequenzgang der Maschine. Ein Kandidat kann immer noch an seiner Verstärkung scheitern. Prüfen Sie einen gewählten Motor mit `tuning`.
- Die Vorauswahl nach dem Moment lässt den Getriebewirkungsgrad außer Acht.
- Gesucht wird nur nach Motoren und Getrieben. Antriebe, Riemen und Achsen werden nicht variiert, ebenso wenig Kombinationen verschiedener Motoren je Achse.
- Die Zwischenkreiskapazität und der Bremswiderstand bleiben, wie die Konfiguration sie hat; die Einschaltspannung des Bremswiderstands und die Abschaltspannung werden auf die Zwischenkreisspannung des Motors skaliert (siehe den vorigen Abschnitt). Bei einer Standardkonfiguration werden die Zeilen zum Bremsen bei jedem Kandidaten gegen Platzhalter beurteilt.
- Ein Kandidat, der an den Achsgrenzwerten mehr als seine Bemessungsdrehzahl verlangt, besteht nie, auch wenn sein simulierter Zyklus bestehen würde.

## Einfluss unsicherer Parameter erkennen und mit Fehlerbändern beurteilen

Eine Konfiguration gibt jedem unsicheren Wert eine untere und eine obere Grenze. Sie stehen in `[bounds]`, einer Tabelle, die die Konfiguration spiegelt:

```toml
[bounds]
belt.pretension_n = [350.0, 550.0]
friction_x.coulomb_n = [12.0, 26.0]
```

**Empfindlichkeit.** Der Zyklus wird mit jedem Parameter an seinem unteren und an seinem oberen Wert gefahren, immer einer allein. Die Zeilen sind danach sortiert, wie stark sich die Ergebnisse ändern:

```text
$ sim-cli sensitivity t-gantry --config crates/machine-t-gantry/exct-30.toml --group mass --group friction_x --group belt
parameter,value,low,high,peak_torque_low,peak_torque_high,rms_torque_low,rms_torque_high,cycle_time_low,cycle_time_high,belt_tension_min_low_n,belt_tension_min_high_n
mass.z_axis_kg,4.7,3.7600000000000002,5.64,-0.0672,+0.0671,-0.0673,+0.0688,+0.0000,+0.0000,251.36,227.13
friction_x.viscous_n_s_m,20.0,10.0,40.0,-0.0341,+0.0683,-0.0161,+0.0444,+0.0000,+0.0000,247.11,223.14
mass.carriage_kg,7.16,5.728000000000001,8.592,-0.0645,+0.0652,-0.0463,+0.0483,+0.0000,+0.0000,254.50,224.13
mass.payload_kg,3.0,2.4000000000000004,3.5999999999999996,-0.0364,+0.0358,-0.0272,+0.0275,+0.0000,+0.0000,247.00,231.17
friction_x.coulomb_n,20.0,12.0,26.0,-0.0222,+0.0167,-0.0086,+0.0072,+0.0000,+0.0000,244.57,235.20
...
belt.pretension_n,450.0,350.0,550.0,-0.0000,-0.0002,+0.0000,-0.0000,+0.0000,+0.0000,139.15,339.14
...
belt.tooth_mesh_stiffness_n_m,5000000.0,2500000.0,10000000.0,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,239.14,239.14
```

| Spalte | Bedeutung |
| --- | --- |
| `parameter`, `value` | Pfad in der Konfiguration und sein Wert dort |
| `low`, `high` | Unterer und oberer Wert, mit denen der Zyklus gefahren wurde: die Grenzen der Konfiguration oder, wo es keine gibt, der Wert mal (1 − delta) und (1 + delta) |
| `peak_torque_low`, `_high` | Relative Änderung des Spitzenmoments (ein Anteil: `+0.0671` sind +6,71 %), und zwar des Motors, bei dem es sich am stärksten ändert |
| `rms_torque_low`, `_high` | Dasselbe für das Effektivmoment |
| `cycle_time_low`, `_high` | Relative Änderung der Zykluszeit |
| `belt_tension_min_low_n`, `_high_n` | Kleinste Riemenkraft in N beim unteren und beim oberen Wert. Ein absoluter Wert, keine Änderung |

| Option | Wirkung |
| --- | --- |
| `--group TABLE` | Nur diese Tabelle der Konfiguration variieren. Mehrfach möglich. Ohne die Option wird jede Tabelle variiert außer `sim`, `effects`, `drive`, `profile`, `start`, `axis` und `temperature` |
| `--delta FRACTION` | Breite für Werte ohne Grenzen. Standard `0.2` |

Die Massen im Beispiel haben in der Konfiguration keine Grenzen und wurden deshalb um 20 % variiert. Ein Parameter, dessen geänderten Wert die Maschine ablehnt, fehlt in der Tabelle. Die Vorspannung ändert die Momente kaum und steht deshalb weit unten, sie bestimmt aber die kleinste Riemenkraft: 139 N an ihrer unteren Grenze. Die letzte Zeile zeigt einen Parameter eines abgeschalteten Effekts: Er ändert nichts.

**Fehlerbänder zu den Prüfungen.**

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands
check,motor,value,limit,result,low,high,worst_case
motor-peak,a,0.3251,1.000,ok,0.3162,0.3450,ok
motor-thermal,a,7.764,100.0,ok,7.508,8.494,ok
pulley-torque,a,3.405,12.50,ok,3.294,3.635,ok
dc-link-energy,a,9.943,9.943,ok,4.971,23.16,ok
braking-resistor-continuous,a,8.859,100.0,ok,4.999,14.00,ok
braking-resistor-peak,a,213.7,3000,ok,185.9,228.6,ok
dc-link-overvoltage,a,770.0,820.0,ok,708.6,790.0,ok
...
settling,,0.00003505,0.0001000,ok,0.00003164,0.00004055,ok
belt-tension,,558.4,2160,ok,458.4,658.3,ok
belt-slack,,239.1,0,ok,139.1,339.1,ok
gearbox-radial-load,,1005,2000,ok,805.1,1205,ok
guide-load,,185.3,2000,ok,177.3,286.6,ok
```

`low` und `high` sind der kleinste und der größte Wert, den die Prüfung über alle Läufe angenommen hat: die Konfiguration, wie sie ist, und jeder Parameter mit Grenzen an seiner unteren und an seiner oberen Grenze. `worst_case` ist nur dann `ok`, wenn die Prüfung in jedem Lauf bestanden wurde. Hier besteht jede Prüfung in jedem Lauf: Mit der Vorspannung an ihrer unteren Grenze von 350 N behält das schlaffste Trum 139 N Riemenkraft, das Werkzeug steht beim Verweilen höchstens 0,041 mm neben seinem Ziel, und der Riemen zieht mit höchstens 1205 N an einer Scheibenwelle. Dieser Lauf hat 123 Zyklen simuliert (61 Werte mit Grenzen), auf allen Prozessorkernen, und dauerte etwa 2 s auf dem Rechner, auf dem dieses Handbuch entstand.

Ein früherer Stand dieser Konfiguration fiel hier durch: Mit 300 N Vorspannung wurde ein auf 200 N eingestellter Riemen schlaff, und am weichen Ende der Grenzen für das Gestell stand das Werkzeug beim Verweilen 0,16 mm neben seinem Ziel, bei einer Toleranz von 0,1 mm (die Werkzeugposition enthält das Schwanken des Gestells). Die Vorspannung wurde auf 450 N erhöht, der Ruckgrenzwert in X von 500 auf 150 m/s³ gesenkt, und Input Shaping und das Überschleifen wurden eingeschaltet. Der Zyklus dauert jetzt 1,934 s, vorher 2,136 s.

**Alle Werte mit Grenzen zugleich.** Mit `--samples N` (Stichproben) wird der Zyklus N-mal gefahren, jedes Mal mit allen Werten mit Grenzen zwischen ihren Grenzen:

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --samples 200
check,motor,value,limit,result,low,high,worst_case,pass_share
motor-peak,a,0.3251,1.000,ok,0.3063,0.3791,ok,1.000
motor-thermal,a,7.764,100.0,ok,7.303,9.353,ok,1.000
pulley-torque,a,3.405,12.50,ok,3.170,3.992,ok,1.000
dc-link-energy,a,9.943,9.943,ok,4.812,29.92,ok,1.000
braking-resistor-continuous,a,8.859,100.0,ok,0.1610,14.76,ok,1.000
braking-resistor-peak,a,213.7,3000,ok,22.17,240.7,ok,1.000
dc-link-overvoltage,a,770.0,820.0,ok,657.4,789.9,ok,1.000
...
settling,,0.00003505,0.0001000,ok,0.00002482,0.00005487,ok,1.000
belt-tension,,558.4,2160,ok,450.8,668.9,ok,1.000
belt-slack,,239.1,0,ok,121.6,340.1,ok,1.000
gearbox-radial-load,,1005,2000,ok,802.3,1215,ok,1.000
guide-load,,185.3,2000,ok,172.1,324.2,ok,1.000
```

Die Werte werden als lateinischer Hyperwürfel gezogen: Der Bereich jedes Werts wird in N gleiche Teile geschnitten, jeder Teil kommt in genau einem Lauf vor, und die Reihenfolge der Teile ist je Wert zufällig. Die Zufallszahlen stammen aus `--seed S` (Standard 1); derselbe Aufruf ergibt also dieselbe Tabelle. `low`, `high` und `worst_case` sind die Hüllkurve dieser Läufe und der Konfiguration, wie sie ist. `pass_share` ist der Anteil der Läufe, die Konfiguration, wie sie ist, eingeschlossen, in denen die Prüfung bestanden wurde. Hier sind die Bänder breiter als mit einem Wert allein (122 N statt 139 N im schlaffsten Trum, 0,055 mm statt 0,041 mm neben dem Ziel beim Verweilen, 1215 N an einer Scheibenwelle), und jede Prüfung besteht weiterhin in allen 201 Läufen. Das dauerte etwa 4 s.

`--samples` ohne `--bands`, 0 Stichproben, eine gebrochene Zahl und `--seed` ohne `--samples` werden abgelehnt (Code 2). Die Läufe beider Arten nutzen alle Prozessorkerne.

Grenzen bei dieser Aufgabe:

- **`--bands` allein ändert immer nur einen Wert.** Liegen zwei Werte gleichzeitig daneben, kann ein Ergebnis sein Band verlassen. `--samples` zeigt das.
- **Stichproben finden die ungünstigste Ecke nicht.** Eine Stichprobe liegt nie genau auf einer Grenze (das tut `--bands` allein), und einige Dutzend Läufe finden die ungünstigste Kombination vieler Werte nicht. Lassen Sie beides laufen.
- Die Grenzen sind selbst Schätzungen. In den Standardkonfigurationen haben nur Reibung, Dämpfung, Steifigkeit, Vorspannung, Welligkeit und Getriebewirkungsgrad Grenzen, nicht aber Massen, Trägheiten und Spiel.
- Wo eine Grenze einen Grenzwert der Bauteile ändert (die Konfiguration der EXCT-30 gibt ihren Zwischenkreis-Grenzwerten Grenzen), zeigt die Spalte `limit` weiterhin den Nennwert.
- Eine Grenze für einen Schlüssel, den die Konfiguration nicht hat, wird abgelehnt. Ebenso ein Paar, dessen unterer Wert über dem oberen liegt.

## Einen Zyklus verkürzen

```text
$ sim-cli optimise t-gantry --config crates/machine-t-gantry/exct-30.toml
setting,before,after
cycle_s,1.9340,1.4920
dwell_error_peak_m,3.505e-5,6.965e-5
axis.x.max_acceleration_m_s2,25.0,53.99999999999999
axis.x.max_jerk_m_s3,150.0,337.5
axis.x.max_velocity_m_s,2.5,3.0
axis.z.max_acceleration_m_s2,15.0,32.4
axis.z.max_jerk_m_s3,500.0,1125.0
axis.z.max_velocity_m_s,1.5,1.7999999999999998
effects.input_shaping,true,false
```

Der Optimierer probiert das Überschleifen und höhere Grenzwerte für Geschwindigkeit, Beschleunigung und Ruck (alle Achsen gemeinsam, in Schritten von ×1,5 und ×1,2). Eine Änderung bleibt, wenn der Zyklus kürzer wird und weiterhin jede Prüfung besteht. Dazu gehört `settling`: Eine schnellere Bewegung, nach der das Werkzeug an der Greifposition noch schwingt, zählt nicht. Input Shaping allein macht keinen Zyklus kürzer, weil jeder Halt eine Periode des Shapers wartet. Es wird deshalb dort probiert, wo es helfen kann: Eine schnellere Einstellung, die nur an `settling` scheitert, wird mit dem Shaper noch einmal gefahren (und mit `effects.shaper_follows_pose`, wo die Maschine eine Resonanz meldet) und übernommen, wenn sie dann besteht und kürzer ist. Ist der Shaper schon eingeschaltet, wird auch das Abschalten probiert. Die ersten beiden Zeilen sind die Zykluszeit und der Abstand vom Ziel beim Verweilen, vorher und nachher. Die übrigen Zeilen nennen, was in der Konfiguration zu ändern ist. Es wird keine Konfigurationsdatei geändert.

In der Konfiguration der EXCT-30 sind das Überschleifen und der Shaper schon eingeschaltet. Der Optimierer erhöht hier deshalb die Grenzwerte und schaltet den Shaper ab: Der Zyklus fällt von 1,93 s auf 1,49 s, und das Werkzeug steht beim Verweilen 0,070 mm neben seinem Ziel statt 0,035 mm.

`--runs N` begrenzt die Zahl der simulierten Zyklen (Standard 40).

Grenzen bei dieser Aufgabe:

- Eine Suche durch Probieren, kein Nachweis des Optimums. Sie endet bei der ersten Einstellung, von der aus kein einzelner Schritt mehr eine Verbesserung bringt.
- Hubhöhen und Zwischenpunkte des Zyklus werden nicht variiert.
- Der Zyklus muss in der vorliegenden Konfiguration bereits jede Prüfung bestehen, sonst wird der Befehl abgelehnt.
- Ein Kandidat, dessen Achsgrenzwerte die Motorgrenzen überschreiten (die Warnung aus [Für alle Befehle gleich](#für-alle-befehle-gleich)), wird nicht angenommen.
- Beurteilt wird gegen die Konfiguration mit Nennwerten. Lassen Sie `verdicts --bands` über das Ergebnis laufen. Das Ergebnis oben nimmt die Reserven zurück, die die Konfiguration für ihren ungünstigsten Fall bekommen hat: Mit diesen Einstellungen fallen bei `--bands` die Prüfungen `settling` (0,35 mm) und `belt-slack` (0 N) durch.

**Überschleifen von Hand:** `--set effects.corner_blending=true`, mit `profile.blend_time_s` als längster Vorlaufzeit. Das nächste Segment beginnt, während das laufende noch bremst. Das gilt nur dort, wo die Maschine nicht anhalten muss: keine Verweilzeit, keine Greiferaktion, keine Achse mit Richtungsumkehr, und nur auf den Rampen. Bei Zyklen der Form Heben, Verfahren, Senken bleibt jede Achse innerhalb ihrer Grenzwerte; bei anderen Zyklusformen wird die Summe zweier Segmente auf einer Achse nicht gegen die Grenzwerte geprüft.

## Eine Bahn fahren und den Bahnfehler ablesen

Eine Bahndatei hat ein Stück je Zeile. Die Bahn beginnt dort, wo die Maschine steht (`start.pose`).

```text
# line, arc, line from the start pose (0.1, 0.35)
line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.05
```

| Zeile | Bedeutung |
| --- | --- |
| `line,a,b` | Gerade zu diesem Punkt: eine Position je Achse, in Metern |
| `arc,a,b,ca,cb,cw` oder `ccw` | Auf einem Kreis um den Mittelpunkt (ca, cb) zum Punkt (a, b). `cw` (im Uhrzeigersinn) und `ccw` (gegen den Uhrzeigersinn) gelten mit der ersten Achse nach rechts und der zweiten nach oben |
| `spline,a,b` | Auf einer glatten Kurve durch diesen Punkt. Aufeinanderfolgende Zeilen mit `spline` bilden eine Kurve, ein Stützpunkt je Zeile |
| leer oder mit `#` beginnend | Wird übersprungen |

Ein Kreisbogen, dessen Ende der Endpunkt des Stücks vor ihm ist (dieselben Zahlen, auf einen Nanometer), ist ein Vollkreis. Jeder andere Kreisbogen dreht von diesem Punkt bis zu seinem eigenen Ende, wie kurz auch immer: Ein Bogen von 80 µm ist ein Bogen von 80 µm. Das angegebene Ende muss auf dem Kreis um den Mittelpunkt durch den Endpunkt des Stücks davor liegen, auf 10 µm genau. Der Kreisbogen wird von dort gelegt, wo die Bahn wirklich steht, und nimmt seinen Radius von dort. Er kann also bis zu so weit neben seinem angegebenen Ende enden; dann ist dieser Abstand Teil der Abweichung, die der Planer meldet (`Die geplante Bahn liegt ...`, siehe unten, und „Geplante Bahn neben der vorgegebenen“ in der App).

Das erste Stück hat keinen Punkt vor sich. Eine Bahn beginnt am Sollwert der stehenden Maschine, den Riemendehnung und Getriebeverdrehung einige hundertstel Millimeter neben die Startpose legen. Für einen Kreisbogen als erstes Stück gelten deshalb beide Toleranzen mit 0,1 mm gegen den Ort, an dem die Maschine steht: Sein Ende darf so weit neben dem Kreis liegen, und ein Kreisbogen, der näher als 0,1 mm an diesem Ort endet, ist ein Vollkreis. Ein Kreisbogen kürzer als 0,1 mm kann daher nicht das erste Stück sein. Soll einer gefahren werden, geben Sie seinen Anfang zuerst mit einer `line` zur Startpose an; der Kreisbogen wird dann an diesem Punkt gemessen. Kreisbögen brauchen eine Maschine mit zwei Achsen.

Ein Spline läuft durch alle seine Stützpunkte, ohne Sprung in Position, Richtung oder Krümmung an einem Stützpunkt (ein kubischer Spline über dem geraden Abstand zwischen den Punkten). Er verlässt die Gerade oder den Kreisbogen vor ihm in der Richtung dieses Stücks und endet in der Richtung, mit der die Gerade oder der Kreisbogen nach ihm beginnt. An einem Ende der Bahn, oder wo keine Gerade und kein Kreisbogen anschließt, endet er ohne Krümmung. Splines brauchen wie Kreisbögen eine Maschine mit zwei Achsen. Stützpunkte und Richtungen, bei denen die Kurve auf der Stelle umkehren müsste, werden abgelehnt (`Die Bahn lässt sich nicht auslegen: ...`).

Ein Spline kann über seine Stützpunkte hinausschwingen: Zwischen ihnen baucht die Kurve nach außen aus, umso mehr, je schärfer sie abbiegen muss. Eine Kurve, die den Verfahrbereich verlässt, obwohl alle Stützpunkte darin liegen, wird abgelehnt (`Die Bahn verlässt den Verfahrbereich. ...`). Abhilfe: die Stützpunkte nach innen rücken oder dort, wo sie ausbaucht, einen ergänzen.

```text
$ sim-cli analyze t-gantry --path corner.path
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.128000,0.627751,...,1.545e-4,5.518e-5,1.545e-4,0.000e0,...

$ sim-cli analyze t-gantry --path corner.path --time-optimal
a,1.729000,0.663606,...,1.854e-4,6.665e-5,1.854e-4,0.000e0,...
```

Bei einer Bahn ist `cycle_s` die geplante Dauer plus 0,3 s Ruhe am Ende, damit das Einschwingen des Werkzeugs mitzählt.

Zwei Planer legen den Zeitverlauf der Bahn fest:

| | Standard | `--time-optimal` |
| --- | --- | --- |
| Geschwindigkeit und Beschleunigung | Die Grenzwerte jeder Achse (`[axis.*]`) | Was die Motoren erlauben: ihre Bemessungsdrehzahl und `profile.torque_reserve` mal ihre Drehmoment-Drehzahl-Kennlinie |
| Ruck | Der Grenzwert jeder Achse | Ebenso |
| An einer Ecke (Richtungsänderung über 1 mrad) | Hält an | Hält an |
| Wo eine Gerade auf einen Kreisbogen trifft | Bremst so weit ab, dass der Beschleunigungssprung innerhalb des halben Ruckgrenzwerts bleibt; mit `profile.path_tolerance_m` führt ein Übergangsbogen die Bahn mit Geschwindigkeit in den Kreisbogen (siehe unten) | Ebenso |
| Auf einem Spline | Jeder Abschnitt zwischen zwei Stützpunkten bekommt einen Grenzwert für Geschwindigkeit, einen für Beschleunigung und einen für Ruck, aus seiner engsten Krümmung und der schnellsten Änderung der Krümmung auf ihm (an 257 Stellen abgetastet, um 5 % als Reserve erhöht), aufgeteilt wie auf Kreisbögen | Ebenso, aus dem, was die Motoren erlauben |
| Wo ein Spline auf eine Gerade oder einen Kreisbogen trifft | Die Krümmung springt; er bremst ab wie von einer Geraden in einen Kreisbogen. Ein Übergangsbogen wird dort nicht eingesetzt | Ebenso |
| Modell für die Planung | Keines, nur die Achsgrenzwerte | Das Starrkörpermodell: keine Reibung, keine Riemendehnung |

Der zeitoptimale Plan hält an jeder geplanten Pose jeden Motor innerhalb der Momentenreserve und der Bemessungsdrehzahl und jede Achse innerhalb ihres Ruckgrenzwerts. Beim voreingestellten Ruckgrenzwert von 100 m/s³ entscheidet fast überall der Ruck: Das Beispiel erreicht die Bemessungsdrehzahl auf seiner ersten Geraden und nutzt 0,16 des erlaubten Drehmoments. Ein höherer Ruckgrenzwert macht es schneller (0,78 s plus Ruhe bei 2000 m/s³).

So lesen Sie das Ergebnis:

- `path_error_peak_m` ist der Abstand der Werkzeugspitze von der Linie, die ihr Sollwert gezeichnet hat, gleich an welcher Stelle. Ein Zurückbleiben längs der Bahn zählt nicht. Das ist die Zahl für die Konturgenauigkeit.
- `tracking_error_peak_m` ist der Abstand von der Stelle, an der der Sollwert im selben Augenblick ist. Er enthält das Zurückbleiben.
- Am T-Portal ist die Werkzeugspitze die Spitze der sich neigenden Z-Achse (`effects.tool_flex`) auf einem Gestell, das schwankt (`effects.frame_compliance`), gesehen vom Boden, auf dem das Werkstück steht. Sie ist nicht die Position des Schlittens auf dem Träger und nicht das, was die Motorgeber melden.

**Kreisbögen mit Geschwindigkeit anfahren.** Wo eine Gerade in einen Kreisbogen übergeht, springt die Beschleunigung zum Mittelpunkt von 0 auf v²/r. Der Standardplaner fährt diese Stelle deshalb im Kriechgang an: 0,07 m/s bei 0,1 m Radius und 100 m/s³. Mit `profile.path_tolerance_m` über 0 setzen beide Planer dort, und wo zwei Kreisbögen aufeinandertreffen, zuerst Übergangsbögen (Klothoiden) ein: Kurven, deren Krümmung sich gleichmäßig mit dem Weg ändert. Damit einer hineinpasst, behält der Kreisbogen seinen Mittelpunkt und bekommt einen etwas kleineren Radius. Die geplante Bahn liegt also ein wenig neben den Geraden und Kreisbögen der Datei, und die Fehlerausgabe sagt, wie weit:

```text
$ sim-cli analyze t-gantry --path corner.path --set profile.path_tolerance_m=0.0001 --lang de
Die geplante Bahn liegt bis zu 0.1000 mm neben den vorgegebenen Geraden und Kreisbögen (Übergangsbögen und Kreisbögen, die neben ihrem angegebenen Ende enden).
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,...
a,1.890000,0.627751,...,1.133e-4,...
```

1,89 s statt 2,13 s, und jede Achse bleibt innerhalb ihrer Grenzwerte für Geschwindigkeit, Beschleunigung und Ruck. Die Regeln:

| | |
| --- | --- |
| Abweichung eines Kreisbogens mit Radius r bei Übergangsbögen der Länge L | e = L² / (24 r). Um so viel rückt der Kreisbogen nach innen; auf den Übergangsbögen liegt die Bahn näher an der vorgegebenen |
| Länge der Übergangsbögen | L = v³ dk / (0,4 j): was der schnellste Übergang der Bahn braucht, um mit der Geschwindigkeit v seines langsameren Stücks und 0,4 des kleinsten Ruckgrenzwerts j durchfahren zu werden. dk ist der Sprung der Krümmung, 1 / r von einer Geraden in einen Kreisbogen. Aber nicht länger, als die zulässige Abweichung am engsten Kreisbogen erlaubt: L = Wurzel(24 · Abweichung · r_min) |
| Geschwindigkeit auf einem Übergangsbogen, der kürzer ist als nötig | v = (0,4 j L / dk)^(1/3) |
| Beispiel oben (r = 0,1 m, 100 m/s³, Kreisbogen mit 0,4 m/s) | Nötig 16 mm, erlaubt Wurzel(24 · 0,0001 · 0,1) = 15,5 mm: Der Kreisbogen rückt um die vollen 0,1 mm nach innen und wird mit 0,396 m/s statt 0,071 m/s angefahren. Mit 1 mm zulässiger Abweichung sind die Übergangsbögen die 16 mm lang, die sie brauchen, und die Bahn weicht um 0,107 mm ab |

Eine Länge gilt für alle Übergangsbögen einer Bahn. Ein Kreisbogen an einer Ecke oder am Anfang oder Ende der Bahn behält seinen Radius (er muss dort enden, wo die Datei es sagt) und bekommt deshalb keinen Übergangsbogen zu einer Geraden. Ist ein Stück zu kurz für seine Übergangsbögen oder würde ein Übergangsbogen einen Halbkreis oder mehr drehen (eine zulässige Abweichung, die gegen den Radius eines engen Kreisbogens groß ist), wird die Länge für die ganze Bahn halbiert, bis zu achtmal; danach wird die Bahn wie vorgegeben geplant. Mit einer zulässigen Abweichung von 0 (dem Standard) ändert sich nichts.

**Eine Bahn mit einem Spline.** Zwei Stützpunkte zwischen zwei Geraden:

```text
# line, a curve through two via points, line; from the start pose (0.1, 0.35)
line,0.3,0.35
spline,0.45,0.3
spline,0.55,0.15
line,0.55,0.05
```

```text
$ sim-cli analyze t-gantry --path curve.path
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.101000,0.626580,...,1.590e-4,5.564e-5,1.590e-4,0.000e0,...

$ sim-cli analyze t-gantry --path curve.path --time-optimal
a,1.974000,0.662041,...,1.703e-4,6.001e-5,1.703e-4,0.000e0,...
```

2,101 s innerhalb der Achsgrenzwerte und 1,974 s so schnell, wie die Motoren es erlauben, beide mit 0,3 s Ruhe. Die Kurve verlässt die erste Gerade waagerecht und kommt auf der letzten senkrecht nach unten an. `profile.path_tolerance_m` ändert an dieser Bahn nichts: Sie hat keinen Kreisbogen, und Splines bekommen keine Übergangsbögen.

Andere Wege, eine Bahn vorzugeben:

- `--trajectory FILE`: Posen zu Zeitpunkten, als CSV mit der Zeit in Sekunden und einer Position je Achse pro Zeile. Zeilen, die nicht mit einer Zahl beginnen, werden übersprungen. Die erste Zeit muss 0 sein, und die erste Pose muss dort liegen, wo die Maschine steht (auf 0,1 mm genau); die Zeiten müssen steigen; jede Pose muss im Verfahrbereich liegen (bis 0,1 mm darüber hinaus wird angenommen, siehe die Grenzen unten); die Bahn endet im Stillstand. Die Steuerung legt eine kubische Kurve durch die Punkte.
- `sim-cli run MACHINE --path FILE` oder `--trajectory FILE` schreibt alle Signale statt der Analyse.

Grenzen bei dieser Aufgabe:

- Ohne zulässige Abweichung ist eine verrundete Ecke mit dem Standardplaner langsam: 0,07 m/s in einen Kreisbogen mit 0,1 m Radius bei 100 m/s³.
- Splines: Wo einer an eine Gerade oder einen Kreisbogen anschließt, springt die Krümmung. Die Bahn durchfährt diesen Übergang deshalb langsam, wie von einer Geraden in einen Kreisbogen ohne zulässige Abweichung. Splines bekommen keine Übergangsbögen; `profile.path_tolerance_m` lässt sie und die Kreisbögen neben ihnen, wie sie vorgegeben sind. Ein Abschnitt hat einen Grenzwert, nach seiner engsten Stelle; ein Abschnitt mit einer engen Biegung ist also auf ganzer Länge langsam.
- Übergangsbögen: eine Länge für die ganze Bahn, keiner an einem Kreisbogen, der eine Ecke oder ein Ende der Bahn berührt, und ihre Länge wird mit den Geschwindigkeiten der Achsgrenzwerte bestimmt, auch für den zeitoptimalen Plan (das Beispiel braucht mit 0,1 mm zulässiger Abweichung 1,499 s, ohne 1,729 s).
- Der zeitoptimale Plan kennt weder den Riemen noch die Führungen noch den Zwischenkreis. Lassen Sie `analyze` oder `verdicts` darüber laufen.
- Der zeitoptimale Plan ist nicht der schnellste mögliche. Er hat je Bahnstück einen Grenzwert für Geschwindigkeit und Beschleunigung, und auf einem Kreisbogen teilt er die Grenzen der Motoren auf die Richtungen längs der Bahn und zum Mittelpunkt auf, was einen Teil ungenutzt lässt. Im vollständigen Modell überschreiten die Motoren die Bemessungsdrehzahl um das, was die Regelkreise hinzufügen (0,1 % im Beispiel).
- Eine Maschine, die am Ende eines Hubs steht, hat ihren Sollwert einige hundertstel Millimeter dahinter: Der Sollwert liegt dort, wo die Motoren stehen, unter Riemendehnung und Getriebeverdrehung. Eine Bahn von dort beginnt so weit außerhalb des Verfahrbereichs und wird gefahren. Posen, die mehr als 0,1 mm außerhalb liegen, werden abgelehnt (`Die Bahn verlässt den Verfahrbereich. ...` bei einer Bahn aus Stücken, `Die zeitlich vorgegebene Bahn braucht mindestens zwei Punkte im Verfahrbereich, mit steigenden Zeiten ab 0.` bei Posen mit Zeiten).
- Die Nutzlast ist so, wie die Maschine sie beim Planen hat (angehängt, am Start).
- `verdicts`, `optimise`, `search` und `sensitivity` arbeiten nur mit dem Zyklus, nicht mit einer Bahn.
- Bahnen gibt es nicht über OPC UA. In der App stehen sie unter „Bahn“ (siehe [Die App bedienen](#die-app-bedienen)).

## Regler einstellen und Input Shaping nutzen

Die Verstärkungen stehen in `[drive]`: `kv_1_s` (Lageverstärkung Kv), `velocity_kp_nm_s_rad` und `velocity_tn_s` (Drehzahlregler: Verstärkung Kp und Nachstellzeit Tn), `current_kp_v_a` und `current_tn_s` (Stromregler). Die drei folgenden Befehle geben ein Testsignal nur auf den Lagesollwert, ohne Vorsteuerung. Sie zeigen also die Regelkreise selbst.

**Eine Zeile je Achse:**

```text
$ sim-cli tuning t-gantry
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.02900,0.0625,0.49450,9.507
z,0.04300,0.0000,0.08525,8.625

$ sim-cli tuning t-gantry --set drive.kv_1_s=150
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.01450,0.2691,0.49725,21.961
z,0.00525,0.0691,0.03100,74.895
```

| Spalte | Einheit | Bedeutung |
| --- | --- | --- |
| `rise_time_s` | s | Anstiegszeit: von 10 % bis 90 % eines Sprungs von 1 mm |
| `overshoot` | Anteil | Überschwingen: größter Ausschlag über den Sprung hinaus, als Anteil des Sprungs |
| `settling_time_s` | s | Einschwingzeit: Danach bleibt das Werkzeug innerhalb von 2 % des Sprungs |
| `bandwidth_hz` | Hz | Bandbreite: Frequenz, bei der der Betrag des Führungsfrequenzgangs erstmals unter 1/√2 fällt. Leer, wenn das bis 200 Hz nie geschieht |

Mit Kv = 50 1/s verhält sich der Lageregelkreis wie ein Verzögerungsglied erster Ordnung mit der Zeitkonstante 1/Kv: Bandbreite etwa Kv / 2π = 8 Hz. Z zeigt das deutlich. In X enthält die Werkzeugposition das Schwanken des Gestells: Der Sprung regt die schwach gedämpfte Eigenschwingung des Gestells (etwa 21 Hz) an. Daher kommen die 6 % Überschwingen und die halbe Sekunde, bis das Werkzeug innerhalb von 2 % bleibt. Mit dem dreifachen Kv schwingt X um 27 % über.

**Die Sprungantwort als Kurve:**

```text
$ sim-cli step t-gantry --axis 0
time_s,setpoint,response
0.000000,1.000000000e-3,0.000000000e0
0.000250,1.000000000e-3,3.343570421e-10
0.000500,1.000000000e-3,1.406628745e-8
...
```

`--axis N` (0 ist die erste Achse), `--size M` (Standard 0.001 m), `--time SECONDS` (Standard 0.5). `response` ist die Werkzeugposition bezogen auf die Stelle, an der das Werkzeug stand, mit vier Werten je Millisekunde.

**Der Führungsfrequenzgang:**

```text
$ sim-cli bode t-gantry --axis 0 --count 8
frequency_hz,gain,gain_db,phase_deg
1.0000,9.750295e-1,-0.220,-10.436
2.1317,9.597195e-1,-0.357,-18.626
4.5440,8.808630e-1,-1.102,-34.525
9.6863,7.038070e-1,-3.051,-59.107
20.6478,1.292231e0,2.227,169.624
...
```

`gain` ist die Amplitude des Werkzeugs geteilt durch die Amplitude des Sollwerts. Der Betrag über 1 bei 20,6 Hz ist die Eigenschwingung des Gestells, die in der Werkzeugposition enthalten ist. Optionen: `--from HZ` (1), `--to HZ` (200), `--count N` (40), `--amplitude M` (0.0005 m).

**Die Mechanik ohne Regelung:**

```text
$ sim-cli sweep single-axis --from 20 --to 200 --count 12
frequency_hz,gain_rad_s_per_nm,phase_deg
20.0000,2.376702e1,34.641
...
131.5866,5.087283e1,-29.235
```

Ein sinusförmiges Moment an einem Motor, zusätzlich zu den Haltemomenten, gemessen an der Drehzahl dieses Motors. Riemenresonanzen zeigen sich als Spitzen; die Last, die gegen den gehaltenen Motor schwingt, zeigt sich als Einbruch. Optionen: `--motor N` (Index, 0 ist der erste), `--from` (5), `--to` (500), `--count` (60), `--amplitude NM` (0.05). Frequenzen müssen über null liegen: `--from 0` wird abgelehnt (`Konfigurationswert liegt außerhalb des gültigen Bereichs: frequency`). Bei eingeschalteter Reibung misst eine kleine Amplitude vor allem die Haftreibung; die Tests verwenden 1,5 N m, um den Einbruch des Riemens zu finden.

**Die Bandsperre:** `--set effects.notch_filter=true`, mit `drive.notch_frequency_hz` und `drive.notch_bandwidth_hz`. Sie wirkt auf den Momentensollwert.

**Input Shaping.** Ein ZVD-Shaper verteilt jeden Sollwert auf drei Stufen im Abstand einer halben Schwingungsperiode, sodass sich die Schwingungen, die sie anregen, gegenseitig auslöschen:

```text
--set effects.input_shaping=true --set profile.shaper_frequency_hz=25 --set profile.shaper_damping=0.03
```

Stellen Sie die Frequenz auf die Resonanz des Werkzeugs ein. Am T-Portal ändert sie sich mit der Z-Höhe. Mit `--set effects.shaper_follows_pose=true` (Input Shaper folgt der Pose) übernimmt der Shaper die Frequenz deshalb von der Maschine an der geplanten Pose. Die Maschine meldet die tiefere der beiden Eigenschwingungen Schlitten am Riemen und Z-Achse an der Führung (24,6 Hz bei z = 0,05 m und 35,6 Hz bei z = 0,15 m für die Standardkonfiguration bei x = 0,9 m). Die Einzelachse meldet keine Resonanz; dort bleibt der Shaper bei der festen Frequenz.

Was es am Zyklus der EXCT-30 bewirkt. In der Konfiguration ist der Shaper eingeschaltet, mit festen 25 Hz; der erste Lauf schaltet ihn ab:

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml --set effects.input_shaping=false
motor,cycle_s,...,path_error_peak_m,...,dwell_error_peak_m
a,1.852000,...,5.979e-4,...,7.036e-5

$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
a,1.934000,...,5.580e-4,...,3.505e-5
```

Mit dem Shaper steht das Werkzeug beim Verweilen 0,035 mm neben seinem Ziel statt 0,070 mm.

Das Nachschwingen des Werkzeugs nach einer Verfahrbewegung ist das Signal `axis.x.tool_deflection` aus `run`.

Grenzen bei dieser Aufgabe:

- Der Sprung ist 1 mm groß. Bei eingeschalteter Reibung zeigt er auch die Haftreibung.
- Der Shaper behandelt eine Resonanz. Die Eigenschwingung des Gestells und die Steifigkeit der Regelkreise stecken nicht in der Frequenz, die die Maschine meldet.
- Der Sollwert jeder Bewegung endet eine Schwingungsperiode später. Wo der Zyklus verweilt oder der Greifer arbeitet, warten beide auf den geformten Sollwert. Der Zyklus oben ist deshalb 82 ms länger: zwei Verweilzeiten, je 41 ms. Segmente ohne Verweilzeit und ohne Greiferaktion laufen nach Plan weiter.
- Der größte Teil des Bahnfehlers an der EXCT-30 entsteht, weil das Gestell unter den Beschleunigungskräften nachgibt (0,60 mm Spitze ohne den Shaper, 0,56 mm mit ihm). Dagegen richtet ein Shaper, der auf die Resonanz des Werkzeugs eingestellt ist, wenig aus.
- Es wurde mit keinem realen Antrieb verglichen. Antriebshersteller dokumentieren ihre internen Filter nicht, und die Totzeiten hier sind ganze Takte ohne Jitter.
- Eine Verstärkung, die bei den Nennwerten in Ordnung ist, kann instabil sein, wenn Rotorträgheit, Riemensteifigkeit oder Induktivität abweichen. Das trat an der Konfiguration der EXCT-30 auf; deren Drehzahlverstärkung wurde deshalb von 1,4 auf 0,6 gesenkt. Lassen Sie nach der Reglereinstellung `sensitivity` laufen.

## Eine Antriebsaufzeichnung nachfahren und Parameter anpassen

**Was eine Aufzeichnung ist.** Eine CSV-Datei mit einer Kopfzeile. Benötigte Spalten, bezogen auf die Motorwelle:

| Spalte | Einheit |
| --- | --- |
| `time` | s |
| `motor.<id>.angle_setpoint` | rad |
| `motor.<id>.angle` | rad |
| `motor.<id>.speed` | rad/s |
| `motor.<id>.torque` | N m |

`<id>` ist `a` und `b` am T-Portal und `m` an der Einzelachse. Eine Einheit in eckigen Klammern hinter einem Spaltennamen wird ignoriert (`time[s]`), weitere Spalten ebenso. Eine Datei, die `sim-cli run` geschrieben hat, ist also eine Aufzeichnung. Eine Zeile je Profiltakt (1 ms) gibt die Lagesollwerte exakt wieder. Mindestens vier Zeilen; eine kürzere Aufzeichnung wird abgelehnt.

**Nachfahren.** Die Lagesollwerte der Aufzeichnung werden in Werkzeugposen umgerechnet und durch den Simulator gefahren:

```text
$ sim-cli run single-axis --cycle --time 2.5 --set effects.payload=false --set mass.carriage_kg=5.5 --out measured.csv
$ sim-cli replay single-axis --trace measured.csv --set effects.payload=false --out overlay.csv
motor,torque_rms_nm,torque_peak_nm,following_error_rms_rad,following_error_peak_rad
m,2.290489e-2,7.933285e-2,1.856409e-4,1.384926e-3
```

Die „gemessene“ Maschine ist hier der Simulator mit einem Schlitten, der 1,5 kg schwerer ist als in der Konfiguration. Die Tabelle nennt je Motor den Effektivwert und die Spitze der Differenz zwischen gemessenem und simuliertem Drehmoment und Schleppfehler. `--out` schreibt beides nebeneinander, eine Zeile je Zeile der Aufzeichnung:

```text
time,motor.m.torque_measured,motor.m.torque_simulated,motor.m.speed_measured,motor.m.speed_simulated,motor.m.following_error_measured,motor.m.following_error_simulated
```

**Anpassung, Stufe 1: Starrkörperwerte aus einer Aufzeichnung.**

```text
$ sim-cli fit single-axis --trace measured.csv --set effects.payload=false
quantity,of,config,fitted,standard_error
moved_mass_kg,x,6.546751e0,8.060483e0,2.277e-2
viscous_friction_n_s_m,x,2.697711e1,2.633275e1,3.071e-1
coulomb_friction_n,x,2.117146e1,2.185140e1,2.736e-1
constant_force_n,x,3.237100e-3,9.376128e-3,8.854e-2
Bewegte Masse, Achse x: +1.514 kg ± 0.023. Um diesen Betrag sind die mit der Achse bewegten Massen zu ändern. Lastmasse und die Trägheit von Rotor und Scheibe kann die Aufzeichnung nicht trennen.
Viskose Reibung, Achse x: × 0.976 ± 0.011. Die viskose Reibung der Führung der Achse und der Motorwellen ist mit diesem Faktor zu multiplizieren; trennen kann die Aufzeichnung beide nicht.
Coulomb-Reibung, Achse x: × 1.032 ± 0.013. Coulomb- und Haftreibung der Führung der Achse und der Motorwellen sind mit diesem Faktor zu multiplizieren; trennen kann die Aufzeichnung beide nicht.
Konstante Kraft, Achse x: +0.006 N ± 0.089. So viel mehr braucht die gemessene Achse als die Konfiguration: ein Gewicht, eine Neigung oder ein Kabelzug, die der Konfiguration fehlen.
```

Je Achse wird nach der Methode der kleinsten Quadrate angepasst: Kraft = Masse × Beschleunigung + viskose Reibung × Geschwindigkeit + Coulomb-Reibung × Vorzeichen(Geschwindigkeit) + Konstante. Dieselbe Schätzung läuft über die Aufzeichnung (`fitted`) und über die Simulation derselben Sollwerte mit der Konfiguration (`config`). Die Differenz der beiden ist das, worum die Konfiguration danebenliegt: hier +1,514 kg bei wahren 1,5 kg. Die Hinweiszeilen gehen auf die Fehlerausgabe. Es wird keine Konfiguration geändert.

**Anpassung, Stufe 2: Riemensteifigkeit aus einem Frequenzgang.**

```text
$ sim-cli sweep single-axis --from 50 --to 130 --count 30 --amplitude 1.5 --set belt.stiffness_ea_n=3.5e5 --out sweep.csv
$ sim-cli fit single-axis --sweep sweep.csv --amplitude 1.5
quantity,of,config,fitted,standard_error
resonance_hz,m,8.452246e1,7.242377e1,1.285e0
resonance_damping,m,7.932269e-2,7.912026e-2,
Steifigkeit zwischen Motor und Last: × 0.734 ± 0.026. Die Riemensteifigkeit ist mit diesem Faktor zu multiplizieren. Das gilt erst, wenn die Massen stimmen: diese zuerst berichtigen und erneut messen.
```

Die Datei mit dem Frequenzgang braucht die Spalten `frequency_hz` und `gain_rad_s_per_nm` (Motormoment zu Motordrehzahl). `fit` misst denselben Frequenzgang an der Konfiguration, bei den Frequenzen der Datei, am Motor `--sweep-motor N` (Standard 0) mit `--amplitude NM` (Standard 0.05), und vergleicht, wo der Einbruch liegt. Der „gemessene“ Riemen hat hier das 0,7-Fache der Steifigkeit; die Anpassung ergibt 0,734 ± 0,026.

Was die Anpassung ermitteln kann und was nicht:

| Wird ermittelt | Wird nicht ermittelt oder nicht getrennt |
| --- | --- |
| Bewegte Masse je Achse (Rotoren und Riemenscheiben als Masse an der Achse gerechnet) | Lastmasse gegenüber der Trägheit von Rotor und Riemenscheibe |
| Viskose Reibung je Achse | Führungsreibung gegenüber der Reibung der Motorwellen |
| Coulomb-Reibung je Achse | Die Dämpfung des Riemens gegenüber der Reibung (die Dämpfung des Einbruchs enthält Reibung und Getriebe) |
| Konstante Kraft je Achse (Gewicht, Neigung, Kabelzug) | Steifigkeit aus der Aufzeichnung einer Bewegung: Regelkreise und Reibung dämpfen das Nachschwingen, es braucht einen Frequenzgang |
| Ein Faktor für die Steifigkeit zwischen Motor und Last, aus einem Frequenzgang | Welche Feder zwischen Motor und Last danebenliegt: Der Faktor umfasst alle |
| | Totzeiten und Filter des Antriebs (Stufe 3 ist nicht gebaut) |
| | Spiel, Zahnsteifigkeit, Steifigkeit von Gestell und Z-Achse |

Grenzen bei dieser Aufgabe:

- **Der Greifer.** Eine Aufzeichnung sagt nicht, wann eine Nutzlast kommt und geht. Die Aufzeichnung eines Zyklus wird deshalb mit der Nutzlast so nachgefahren, wie die Konfiguration sie am Start hat. Die Zyklusaufzeichnung einer Maschine, die greift, angepasst gegen eine Konfiguration mit eingeschalteter Nutzlast, liefert ein falsches Ergebnis: Der Standardzyklus der Einzelachse ergibt −0,837 kg. Zeichnen Sie ohne Nutzlast auf oder schalten Sie sie wie oben ab.
- `start.pose` der Konfiguration muss dort liegen, wo die Aufzeichnung beginnt.
- Eine Achse, die die Aufzeichnung nicht in beide Richtungen bewegt, wird nicht angepasst (`Achse x: nicht angepasst. Die Aufzeichnung muss sie in beide Richtungen bewegen.`). Eine Richtung allein kann Coulomb-Reibung nicht von einer konstanten Kraft trennen.
- Eine falsche Reibung zeigt sich als einige Prozent falsche Masse und umgekehrt; das liegt an den Getriebeverlusten. Ändern Sie die Konfiguration und passen Sie erneut an.
- Der Steifigkeitsfaktor gilt erst, wenn die Massen stimmen.
- Ein Frequenzgang mit kleiner Amplitude zeigt bei eingeschalteter Reibung keinen Einbruch. Derselbe Frequenzgang mit den voreingestellten 0,05 N m ergibt `Keine Steifigkeit angepasst: Der gemessene Frequenzgang oder der der Konfiguration zeigt keinen Einbruch mit beiden Flanken im gemessenen Bereich.` Das ist ein Fehler wie jeder andere: `fit` endet mit dem Code 2, nachdem es geschrieben hat, was eine Aufzeichnung ergab. Der Frequenzgang muss außerdem den Einbruch und seine beiden Flanken enthalten. Seine Frequenzen müssen über null liegen.
- All das wurde an Aufzeichnungen geprüft, die der Simulator selbst erzeugt hat, mit zusätzlichem Rauschen. Eine Aufzeichnung eines realen Antriebs wurde nicht gelesen. Bei realen Exporten müssen die Spalten umbenannt und die Einheiten in rad, rad/s und N m an der Motorwelle umgerechnet werden.

## Signale aufzeichnen, Antriebe sperren

```text
$ sim-cli run single-axis --cycle --time 2.5 --out trace.csv
```

Schreibt jedes Signal als CSV, eine Spalte je Signal mit der Einheit in der Kopfzeile (`axis.x.position[m]`, `motor.m.torque[N m]`, ...).

| Option | Wirkung |
| --- | --- |
| `--move a,b` | Eine Punkt-zu-Punkt-Bewegung zu dieser Pose, alle Achsen synchron |
| `--cycle` | Den konfigurierten Zyklus fahren |
| `--trajectory FILE`, `--path FILE [--time-optimal]` | Siehe [Eine Bahn fahren](#eine-bahn-fahren-und-den-bahnfehler-ablesen) |
| keine davon | Die Maschine steht in ihrer Startpose |
| `--time SECONDS` | Simulierte Zeit. Standard 2 |
| `--every STEPS` | Jeden N-ten Basisschritt schreiben. Standard 16, das ist eine Zeile je Millisekunde |
| `--disable-at SECONDS` | Die Antriebe zu diesem Zeitpunkt sperren |

Die Signale (in Klammern der Name, unter dem die App die Größe zeigt):

| Signal | Einheit | Bedeutung |
| --- | --- | --- |
| `time` | s | Simulierte Zeit |
| `payload` | 0 oder 1 | Ob die Nutzlast angehängt ist (Angehängt) |
| `axis.<id>.position`, `.velocity` | m, m/s | Position und Geschwindigkeit des Werkzeugs (am T-Portal in X: der Werkzeugspitze über dem Boden, also Schlitten auf dem Träger plus `frame.deflection` plus `axis.x.tool_deflection`) |
| `axis.<id>.position_setpoint`, `.velocity_setpoint`, `.acceleration_setpoint` | m, m/s, m/s² | Lagesollwert, Geschwindigkeitssollwert und Beschleunigungssollwert der Steuerung, nach dem Input Shaping und vor der Feldbus-Totzeit |
| `axis.<id>.following_error` | m | Schleppfehler der Motoren, auf die Werkzeugachsen umgerechnet |
| `motor.<id>.angle`, `.speed` | rad, rad/s | Winkel und Drehzahl der Motorwelle, vor dem Getriebe |
| `motor.<id>.torque`, `.torque_command` | N m | Drehmoment aus dem Wicklungsstrom und der Momentensollwert des Drehzahlreglers dafür, nach der Momentengrenze |
| `motor.<id>.current`, `.voltage`, `.electrical_power` | A, V, W | Strom, Spannung und elektrische Leistung der Wicklung |
| `motor.<id>.winding_temperature` | K | Übertemperatur der Wicklung |
| `motor.<id>.dc_link_voltage`, `.braking_resistor_power` | V, W | Zwischenkreisspannung und Leistung am Bremswiderstand. Mit `effects.shared_dc_link` für jeden Motor gleich |
| `motor.<id>.speed_command`, `.following_error`, `.angle_setpoint` | rad/s, rad, rad | Lageregler: Drehzahlsollwert, Schleppfehler, Winkelsollwert |
| `gearbox.<motor id>.output_torque` | N m | Getriebe-Abtriebsmoment, an der Riemenscheibe. Das beurteilen die Prüfungen `pulley-torque`, `gearbox-peak` und `gearbox-continuous` |
| `belt.<n>.tension` | N | Riemenkraft im Riementrum n |
| `axis.<id>.belt_stretch` | m | Riemendehnung: Werkzeugposition minus dem, was die Winkel der Riemenscheiben ergeben |
| `axis.<id>.friction` | N | Reibkraft der Führung |
| `belt.slack_time` | s | Zeit ohne Riemenkraft: Sekunden, die irgendein Riementrum ohne Riemenkraft war, aufsummiert |
| `pulley.<id>.radial_load` | N | Radialkraft des Riemens auf die Scheibenwelle |
| `axis.<id>.guide_load` | N | Führungsbelastung |
| `frame.deflection` | m | Auslenkung des Gestells in X (T-Portal) |
| `axis.x.tool_deflection` | m | Auslenkung der Werkzeugspitze gegenüber dem Schlitten in X (T-Portal) |

**Antriebe sperren.** Mit `--disable-at` geben die Antriebe ab diesem Zeitpunkt kein Moment mehr ab. Die Motor-Haltebremsen fallen ein (`effects.holding_brake`, an beiden Maschinen, nur im elastischen Modell):

```text
$ sim-cli run t-gantry --move 0.5,0.2 --time 1.5 --disable-at 0.5 --out disable.csv
```

In diesem Lauf steht die Z-Achse bei 0,498 s auf 0,2276 m und bewegt sich; bei 1,498 s ruht sie auf 0,2192 m. Eine Bremse, die bei Drehzahl einfällt, rutscht mit ihrem Haltemoment (`motor.brake_torque_nm`) und nicht mehr, bis die Achse steht. Mit `--set effects.holding_brake=false` fällt die Achse bis zum unteren Endanschlag (z = −0,00008 m bei 1,498 s). Eine laufende Bewegung oder ein laufender Zyklus wird verworfen und nicht fortgesetzt.

## Eine SPS über OPC UA anbinden

```sh
cargo run --release -p sim-opcua -- t-gantry --port 4840 --lang de
```

Das gibt `OPC-UA-Server erreichbar unter opc.tcp://127.0.0.1:4840/` aus und lässt die Maschine in Echtzeit laufen, bis der Server beendet wird. Der Name der Maschine, `--port` (Standard 4840) und `--lang` sind die einzigen Argumente. Eine Maschine, die der Simulator nicht hat, ein Port oder eine Sprache, die nicht lesbar sind, und jedes andere Wort werden mit einer Meldung und dem Code 2 abgelehnt.

| Eigenschaft | Wert |
| --- | --- |
| Endpunkt | `opc.tcp://127.0.0.1:PORT/` |
| Sicherheit | Keine. Anonymer Zugriff |
| Namensraum | `urn:kinematics-simulation` |
| Knoten-IDs | Zeichenketten in diesem Namensraum, wie in der Tabelle unten |
| Aktualisierungsrate | Jedes Signal wird 100-mal je Sekunde veröffentlicht |

| Knoten | Typ | Zugriff | Bedeutung |
| --- | --- | --- | --- |
| `signal.<signal id>` | Double | lesen | Jedes Signal der [Signaltabelle](#signale-aufzeichnen-antriebe-sperren), z. B. `signal.axis.x.position`, `signal.motor.a.torque` |
| `command.target.<axis id>` | Double | schreiben | Fahrziel der nächsten Punkt-zu-Punkt-Bewegung, z. B. `command.target.x`. Zeigt die geplante Position, bis es geschrieben wird |
| `command.move` | Boolean | schreiben | `true` startet die Fahrt zu den Zielen |
| `command.setpoint.<axis id>` | Double | schreiben | Laufend übertragener Lagesollwert von der SPS. Der Simulator interpoliert über `profile.setpoint_interval_s` dorthin. Zeigt den Sollwert, dem die Steuerung folgt |
| `command.cycle` | Boolean | schreiben | `true` startet den Zyklus, `false` hält ihn nach dem laufenden Segment an |
| `command.disable` | Boolean | schreiben | `true` sperrt die Antriebe; die Bremsen fallen ein |
| `command.enable` | Boolean | schreiben | `true` gibt die Antriebe wieder frei |
| `status.busy` | Boolean | lesen | Eine Bewegung oder der Zyklus läuft („In Bewegung“) |
| `status.refused` | String | lesen | Schlüssel des zuletzt abgelehnten Befehls, oder leer |

So verhalten sich die Befehle:

- Die Regelkreise der Antriebe bleiben im Simulator. Die SPS sendet Ziele oder Lagesollwerte, kein Drehmoment.
- Zahlen dürfen als Double, Float, Int16, Int32 oder Int64 geschrieben werden. Alles andere wird mit `BadTypeMismatch` beantwortet.
- Sollwerte mehrerer Achsen, die innerhalb derselben 10 ms geschrieben werden, gelten zusammen als ein Sollwert.
- Eine Achse, die nicht geschrieben wird, bleibt, wo sie ist. `command.target.<axis id>` zeigt die geplante Position, bis es geschrieben wird, und wieder, sobald die Fahrt befohlen ist, für die es geschrieben wurde. `command.setpoint.<axis id>` zeigt den Sollwert, dem die Steuerung folgt; das ist die geplante Position, solange kein Sollwert unterwegs ist. Nach einer Fahrt oder einem Zyklus schickt ein Schreiben auf eine Achse die anderen also nicht dorthin zurück, wo sie vorher standen.
- `status.refused` enthält einen Schlüssel wie `error-busy`, `error-target-out-of-range`, `error-setpoint-too-far` oder `error-disabled`. Er bleibt stehen, bis ein Befehl derselben Art (Fahrt, Sollwert, Zyklusstart, Halt, Sperren, Freigeben) angenommen wird. Angenommene Befehle anderer Art, etwa laufend übertragene Sollwerte, lassen ihn stehen.
- Ein laufend übertragener Sollwert wird abgelehnt, solange eine Bewegung oder der Zyklus läuft.
- Ein Sollwert, der vom letzten (oder, wenn keiner unterwegs ist, vom Standort der Achse) weiter entfernt liegt, als die Achse mit ihrer Höchstgeschwindigkeit in einem `profile.setpoint_interval_s` fährt (8 mm an der X-Achse des T-Portals), wird mit `error-setpoint-too-far` abgelehnt und nicht ausgeführt. Eine SPS, die weiter zurückliegt, holt mit einer Fahrt auf. Laufend übertragenen Sollwerten, einer je Intervall, folgt die Achse mit deren Geschwindigkeit, bis zur Höchstgeschwindigkeit; ein Sollwert, der zu früh kommt, trifft die Achse vor dem letzten an, und sie folgt ihm um so viel schneller.
- Kommt der Rechner mit der Echtzeit nicht mit, bleibt die Simulation zurück, statt Arbeit aufzustauen.

Grenzen bei dieser Aufgabe:

- **Nicht an einer realen SPS geprüft.** Der Client in den Tests ist mit derselben Bibliothek geschrieben wie der Server. Das Zusammenspiel mit Clients von Siemens, Beckhoff oder CODESYS, deren Datentypen, Sicherheitseinstellungen und Zyklus-Jitter sind ungeprüft.
- Der Server bindet sich nur an 127.0.0.1 und bietet nur die Standardkonfiguration der Maschine an. Es gibt weder `--config` noch `--set`.
- Keine Sicherheitsprofile, keine Benutzerkonten, keine Methoden, keine Konfigurationsänderungen, keine Bahnen über OPC UA.
- Der Server schreibt seine Schlüssel in einen Ordner `pki` im Arbeitsverzeichnis.

## Die App bedienen

```sh
cargo run --release -p sim-app
```

Das Fenster zeigt in der Mitte die 3D-Ansicht, links ein Bedienfeld und rechts die Oszilloskope.

**3D-Ansicht.** Aus einfachen Körpern aufgebaut. Riemenscheiben, Motorwellen und Umlenkrollen drehen sich; die Riemenzähne laufen mit (alle 40 mm gezeichnet, nicht in der Zahnteilung); Riementrume färben sich oberhalb der Vorspannung rot und unterhalb blau. Beschriftungen zeigen die Drehzahl jedes Motors in 1/min und die Position jeder Achse. Ziehen mit der linken Maustaste dreht die Ansicht, das Mausrad zoomt. Ist eine Bahn geplant (der Teil „Bahn“ im rechten Bedienfeld), zeichnet die Ansicht die geplante Bahn als gelbe Linie dort, wo das Werkzeug fahren wird, und darüber in Hellblau die Bahn, die die Werkzeugspitze wirklich fährt, sobald die Maschine sie abfährt.

**Linkes Bedienfeld, von oben nach unten:**

| Bedienelement | Wirkung |
| --- | --- |
| Maschine | Wechselt die Maschine. Startet die Simulation mit der Standardkonfiguration dieser Maschine neu |
| Sprache | English oder Deutsch, im laufenden Betrieb umschaltbar |
| Pause / Weiter | Hält die Simulation an und setzt sie fort |
| Geschwindigkeit (1 = Echtzeit) | Simulierte Sekunden je realer Sekunde, 0,01 bis 2 |
| Ansicht: Vorne, Seite, Oben, Schräg | Voreingestellte Kamerapositionen |
| Bewegung: ein Ziel je Achse | Ziel der nächsten Bewegung |
| Ziel anfahren | Punkt-zu-Punkt-Bewegung, alle Achsen synchron |
| Zyklus starten | Fährt den konfigurierten Zyklus, bis er angehalten wird |
| Nach Segment anhalten | Fährt das laufende Segment zu Ende und bleibt dann stehen. Einen Schnellhalt gibt es nicht |
| Antriebe sperren / Antriebe freigeben | Antriebe aus (die Bremsen fallen ein) und wieder ein |
| Meldungen | Abgelehnte Befehle und Konfigurationswerte, mit der Schaltfläche Leeren |
| Effekte | Ein Kontrollkästchen je Schalter in `[effects]`. Wirkt sofort, ohne Neustart |
| Parameter | Jeder Zahlenwert der Konfiguration, nach Tabellen gruppiert. Ändern durch Ziehen; der Wert gilt sofort in der laufenden Simulation. Ein Wert, den die Simulation ablehnt, wird unter „Meldungen“ genannt und zurückgenommen |

**Rechtes Bedienfeld:**

| Bedienelement | Wirkung |
| --- | --- |
| Zeitfenster [s] | Sekunden Verlauf, die die Oszilloskope zeigen, 0,2 bis 60 |
| Einfrieren | Hält die Kurven fest; die Simulation läuft weiter |
| CSV exportieren | Schreibt jedes Signal über das sichtbare Zeitfenster nach `traces-<time>.csv` im Arbeitsverzeichnis |
| Oszilloskope (Überschrift) | Welche Größen gezeichnet werden |
| Analyse: Zyklus analysieren und beurteilen | Die Zyklusanalyse und die Prüfungen, als drei Tabellen: die Kennwerte der Maschine (Zykluszeit, Bahnfehler, Abstand vom Ziel beim Verweilen, kleinste Riemenkraft, Zeit mit schlaffem Trum, Radialkraft auf eine Scheibenwelle, Führungsbelastung, Lebensdauer der Führung), die Belastung je Motor (Spitzenmoment, Effektivmoment, kubisch gemitteltes Moment, Übertemperatur der Wicklung, aufgenommene und rückgespeiste Energie, Energie und mittlere Leistung am Bremswiderstand, höchste Zwischenkreisspannung) und eine Zeile je Prüfung |
| Analyse: mit Fehlerbändern, Stichproben | Angekreuzt fährt „Zyklus analysieren und beurteilen“ den Zyklus zusätzlich mit jedem Wert aus `[bounds]` an seiner unteren und oberen Grenze (zwei weitere Zyklen je Wert) und zeigt eine vierte Tabelle: je Prüfung den kleinsten und den größten Wert, das Ergebnis im ungünstigsten Fall und „Bestandene Läufe [%]“. Dasselbe wie `sim-cli verdicts --bands`. Steht im Zahlenfeld „Stichproben“ neben dem Kontrollkästchen eine Zahl über 0, wird der Zyklus stattdessen so oft mit allen Werten aus `[bounds]` zugleich zwischen ihren Grenzen gefahren, und die Tabelle trägt die Überschrift „Die Prüfungen mit allen unsicheren Werten zugleich zwischen ihren Grenzen“. Dasselbe wie `--bands --samples N`; die App nimmt immer den Startwert (`--seed`) 1 |
| Analyse: Empfindlichkeit | Die acht Parameter mit `[bounds]`, von denen der Zyklus am stärksten abhängt |
| Analyse: Zyklus optimieren | Der Optimierer. „Diese Einstellungen in die laufende Maschine übernehmen“ setzt sein Ergebnis in die Konfiguration ein |
| Motorsuche | Netzspannung (230v oder 400v), der Anfang der Artikelnummern (leer: jeder Motor des Katalogs, das dauert Minuten) und wahlweise eine Getriebebaureihe wie `NRK` oder `NRK070`. „Suchen“ fährt jeden solchen Motor auf dem Zyklus und listet sie auf, bestandene zuerst, leichteste zuerst, mit der knappsten Prüfung oder dem Grund, aus dem ein Motor nicht gefahren wurde. „Einsetzen“ in einer Zeile benennt diesen Motor, seine Netzspannung und sein Getriebe in `[parts]` der laufenden Konfiguration. Dasselbe wie `sim-cli search` |
| Bahn | Ein Dateiname, eingetippt. „Geraden, Kreisbögen und Splines“ ist eine Bahndatei, geplant innerhalb der Achsgrenzwerte oder, angekreuzt, „So schnell, wie die Motoren es erlauben“; „Posen mit Zeiten“ ist eine Datei mit einer zeitlich vorgegebenen Bahn. „Planen, fahren und analysieren“ plant die Bahn von dort, wo die Maschine steht, analysiert sie mit einer eigenen Simulation (Dauer, Abweichung durch Übergangsbögen, Bahnfehler, Abstand hinter dem Sollwert, Belastungen, Prüfungen), zeichnet die geplante Bahn als Diagramm im Bedienfeld und als Linie in der 3D-Ansicht und lässt die Maschine in der 3D-Ansicht sie fahren. Dasselbe wie `sim-cli analyze --path` oder `--trajectory` |
| Aufzeichnung | Ein Dateiname, eingetippt. „Nachfahren und anpassen“ fährt die Antriebsaufzeichnung nach und zeigt je Motor, wie weit die Simulation von ihr abweicht (Moment und Schleppfehler, Effektivwert und Spitze), und je Achse die angepasste Masse, Reibung und konstante Kraft neben den Werten der Konfiguration, mit dem, was in der Konfiguration zu ändern ist. Dasselbe wie `sim-cli replay` und `fit --trace`. Die Startpose der Konfiguration muss dort liegen, wo die Aufzeichnung beginnt |
| Reglereinstellung: Sprungantwort und Frequenzgang messen | Sprungantwort und Führungsfrequenzgang einer Achse, mit Anstiegszeit, Überschwingen, Einschwingzeit und Bandbreite |
| Diagramme der Oszilloskope | Ein Diagramm je Größe, eine Linie je Achse oder Motor. Sollwerte sind gestrichelt in der Farbe ihres Istwerts gezeichnet. Gemeinsame Zeitachse und gemeinsamer Cursor; Min, Max, Spanne und Effektivwert je Kurve; die Einheiten wechseln von selbst auf mm und µm |

In den Ergebnistabellen der Analyse steht „in Ordnung“ für `ok` und „NICHT BESTANDEN“ für `FAIL`.

Das rechte Bedienfeld rollt als Ganzes, und jeder seiner Teile (Oszilloskope, Analyse, Motorsuche, Bahn, Aufzeichnung, Reglereinstellung) lässt sich unter seiner Überschrift einklappen. Ein Teil klappt von selbst auf, wenn sein Ergebnis eintrifft. Die Tabellen sind für die Standardbreite des Bedienfelds ausgelegt: Lange Namen brechen um, und Zahlen haben vier Stellen.

Die Schaltflächen dieser Teile rechnen im Hintergrund mit eigenen Simulationen, und zwar mit der Konfiguration, wie sie in diesem Augenblick ist; solange steht ein Wartezeichen an Stelle der Schaltfläche. Die laufende Maschine wird nicht gestört. Ein Ergebnis wird nicht neu berechnet, wenn sich danach ein Parameter ändert. Es wird stattdessen als „für eine frühere Konfiguration berechnet“ gekennzeichnet, und nichts davon wird in die geänderte Konfiguration übernommen: „Diese Einstellungen übernehmen“, „Einsetzen“ und das Fahren einer geplanten Bahn antworten dann mit einer Meldung (die Änderung zurücknehmen oder neu berechnen). Einen Motor mit „Einsetzen“ zu benennen macht die Suche, aus der er stammt, nicht ungültig; danach lässt sich eine andere Zeile versuchen.

Eine Datei, die fehlt oder nicht lesbar ist, eine nicht lesbare Zeile darin, eine andere Netzspannung als 230v und 400v und eine Suche, die keinen Motor findet oder kein Getriebe der Baureihe, das an einen passt, werden unter „Meldungen“ beantwortet. Ebenso eine Bahn, die die Maschine nicht fahren kann, weil sie sich seit dem Planen bewegt hat oder in Bewegung ist.

Ein Antrieb, der wegen Überspannung im Zwischenkreis abschaltet, wird unter „Meldungen“ genannt, einmal je Abschaltung, für die laufende Maschine und für einen analysierten Zyklus oder eine analysierte Bahn. „Antriebe freigeben“ bringt die Antriebe zurück.

Was die laufende Maschine mit einer Änderung tut:

- Eine geänderte Getriebeübersetzung (`gearbox.ratio`) oder ein geänderter Scheibenradius (`geometry.pulley_radius_m`) baut den Zustand der Maschine an der Pose und mit der Geschwindigkeit des Werkzeugs neu auf, mit Riemen und Getriebe so, wie sie dort in Ruhe sind. Die Riemenkräfte bleiben bis auf einige Prozent, wie sie waren.
- Eine Änderung am Input Shaper (sein Schalter, die Frequenz, der Dämpfungsgrad oder „Input Shaper folgt der Pose“) wird übernommen, sobald die geplante Bewegung das nächste Mal steht. So springt der Sollwert nicht. Ein Zyklus ohne Verweilzeit und ohne Greiferwechsel steht nie; dort wartet die Änderung auf „Nach Segment anhalten“.
- Ein Zyklus, der im Lauf Segmente verliert, läuft in den Segmenten weiter, die er hat.

Meldungen: Eine Meldung, die nur die letzte wiederholt, wird nicht hinzugefügt. Endet die Simulation oder eine Berechnung im Hintergrund unerwartet, sagt das eine Meldung; die Maschine erneut zu wählen startet die Simulation neu. Steht das Fenster länger als zehn Sekunden simulierter Zeit, verlieren die Oszilloskope die Werte davor.

Grenzen bei dieser Aufgabe:

- Die App startet nur mit den Standardkonfigurationen. Sie kann keine Konfigurationsdatei laden; die Konfiguration der EXCT-30 steht in ihr also nicht zur Verfügung.
- Bauteile nach Namen setzt nur die Motorsuche; ein Feld, um einen Namen einzutippen, gibt es nicht. Eine Zahl eines benannten Bauteils, die im Parameterfeld geändert wird, wird wieder überschrieben.
- Der Zyklus selbst lässt sich in der App nicht bearbeiten.
- Nicht in der App, nur auf der Befehlszeile: der Frequenzgang der Mechanik (`sweep`), die Anpassung der Riemensteifigkeit aus einem Frequenzgang (`fit --sweep`), die Vergleichsdatei eines Nachfahrens (`replay --out`), die Betriebspunkte (`analyze --points`) und eine Empfindlichkeitsrechnung über Tabellen ohne `[bounds]`.
- Die Tabelle der Motorsuche zeigt nur die knappste Prüfung; Spitzenmoment und Übertemperatur der Wicklung eines Kandidaten stehen in `sim-cli search` oder nach „Einsetzen“ in der Zyklusanalyse.
- Dateinamen werden eingetippt; einen Dateidialog gibt es nicht. Ein relativer Name gilt ab dem Verzeichnis, in dem die App gestartet wurde.
- Die Linien der geplanten und der gefahrenen Bahn in der 3D-Ansicht hat kein Mensch am Bildschirm geprüft.
- Eine Bahn wird von dort analysiert, wo die Maschine steht, wenn die Schaltfläche gedrückt wird. Eine Maschine, die dann in Bewegung ist, lehnt das Fahren ab.
- Der Basis-Zeitschritt `sim.dt_s` lässt sich im laufenden Betrieb nicht ändern.
- Die Mausbedienung hat kein Mensch geprüft. Bildschirmfotos des Fensters in beiden Sprachen wurden geprüft.
- Nur für den Desktop.
