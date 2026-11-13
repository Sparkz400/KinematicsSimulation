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
- [Sehen, wohin Moment und Energie gehen](#sehen-wohin-moment-und-energie-gehen)
- [Signale aufzeichnen, Antriebe sperren](#signale-aufzeichnen-antriebe-sperren)
- [Eine SPS über OPC UA anbinden](#eine-sps-über-opc-ua-anbinden)
- [Das Modell in Simulink / Modelica über FMI nutzen](#das-modell-in-simulink--modelica-über-fmi-nutzen)
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
- Jede Option außer `--cycle`, `--time-optimal`, `--bands` und `--worst` erwartet einen Wert. Eine Option ohne ihren Wert wird abgelehnt (`Dieser Option fehlt ihr Wert: --time`), ebenso eine Option, die der Befehl nicht kennt, etwa eine falsch geschriebene (`Diese Option kennt der Befehl nicht: --cylce`). Eine Zahl muss endlich sein: `--time inf` wird abgelehnt.
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
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km,mains_energy_j,guide_life_h,pulley_bearing_life_h,gearbox_life_h,belt_tooth_utilisation
a,6.080000,0.559623,0.165360,200.4514,0.1476,0.1302,204.64,334.35,0.3676,1.23,85.026,10.095,28.21,5.865,1.369e-4,4.382e-5,1.706e-4,4.734e-5,0.196871,0.000,0.000,221.41,0.0000e0,6.4481e2,1.7784e2,4.7379e6,0.000,4.9950e6,2.1252e5,6.7359e9,1.9095e-1
b,6.080000,0.460106,0.153385,200.4378,0.1211,0.1208,204.64,334.35,0.3410,1.05,74.368,15.293,35.03,7.082,1.369e-4,4.382e-5,1.706e-4,4.734e-5,0.175176,5.344,0.879,224.65,0.0000e0,6.4481e2,1.7784e2,4.7379e6,0.000,4.9950e6,2.1222e5,1.0201e10,1.5154e-1
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
| `mains_energy_j` | J | Mit `effects.regenerative_supply`: Energie, die das Line Module über den Durchlauf ins Netz zurückgespeist hat (nach seinem Wirkungsgrad). Ohne 0 |
| `guide_life_h` | h | `guide_life_km` geteilt durch die mittlere Geschwindigkeit der Achse dieser Führung. Für die ganze Maschine |
| `pulley_bearing_life_h` | h | Nominelle Lebensdauer des Lagers, das den Riemenzug an der Scheibenwelle dieses Motors trägt: (C / P)³ · 10⁶ / (60 n), C = `ratings.pulley_bearing_load_rating_n`, P das mit der Drehzahl gewichtete kubische Mittel der Radialkraft, n die mittlere Drehzahl der Scheibe in 1/min |
| `gearbox_life_h` | h | Lebensdauer der Planetenlager des Getriebes nach der Formel von Siemens, 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3). Leer ohne `ratings.gearbox_rated_torque_nm` und `gearbox_reference_speed_rpm` (die EXCT-30 hat kein Getriebe) |
| `belt_tooth_utilisation` | Anteil | Zahnbelastung des Riemens an der Scheibe dieses Motors geteilt durch das, was seine Zähne bei der Drehzahl der Scheibe zulassen (Verfahren von Megadyne, mit `ratings.belt_service_factor`), das Höchste im Zyklus. Keine Lebensdauer: kein Riemenhersteller veröffentlicht eine |

Die genauen Definitionen stehen in der [Modellreferenz](../model-reference.md#analysis-quantities).

`--points FILE` schreibt zusätzlich die Betriebspunkte, einen je Millisekunde und Motor (`motor,speed_rad_s,torque_nm`), um sie über der Drehmoment-Drehzahl-Kennlinie aufzutragen.

Danach beurteilen Sie den Zyklus:

```text
$ sim-cli verdicts t-gantry
check,motor,value,limit,result
motor-peak,a,0.1476,1.000,ok
motor-thermal,a,1.227,100.0,ok
drive-current,a,0.3676,3.000,ok
drive-peak-current,a,1.246,10.90,ok
gearbox-continuous,a,0.7955,40.00,ok
gearbox-input-speed,a,1914,13000,ok
gearbox-peak,a,2.175,64.00,ok
pulley-bearing-life,a,212521,20000,ok
gearbox-life,a,6735878139,20000,ok
belt-tooth-load,a,0.1910,1.000,ok
dc-link-energy,a,5.865,6.205,ok
dc-link-overvoltage,a,221.4,257.6,ok
...
settling,,0.00004734,0.0001000,ok
belt-tension,,334.4,1500,ok
belt-slack,,204.6,0,ok
gearbox-radial-load,,644.8,1500,ok
guide-load,,177.8,2000,ok
guide-life,,4994954,20000,ok
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
| `pulley-bearing-life` | Lebensdauer des Lagers der Scheibenwelle | `pulley_bearing_life_h` | `ratings.required_life_h` | h |
| `gearbox-life` | Lebensdauer der Planetenlager des Getriebes | `gearbox_life_h` | `ratings.required_life_h` | h |
| `guide-life` | Lebensdauer der Führung | `guide_life_h` | `ratings.required_life_h` | h |
| `belt-tooth-load` | Zahnbelastung des Riemens | `belt_tooth_utilisation` | 1 | Anteil |
| `gearbox-thermal` | Übertemperatur des Getriebes, eingeschwungen | Endwert des wärmsten Getriebes im Zyklus (nur mit `effects.part_heating`; keine bei Direktantrieb) | `thermal.gearbox_max_rise_k` | K |
| `guide-thermal` | Übertemperatur der Führung, eingeschwungen | Endwert der wärmsten Führung im Zyklus (nur mit `effects.part_heating`) | `thermal.guide_max_rise_k` | K |
| `belt-tooth-jump` | Zahnsprünge des Riemens im Zyklus | wie oft die Zähne des Riemens über eine angetriebene Scheibe gesprungen sind (nur mit `effects.belt_tooth_jump`) | 0 | Anzahl |
| `belt-span-resonance` | Erste Eigenform eines Riementrums, Abstand von einer Anregung bei konstanter Geschwindigkeit (Anteil) | wie nahe die erste Quereigenfrequenz eines freien Trums dem kam, was der Antrieb anregt, bei konstanter Geschwindigkeit, als Anteil der Eigenfrequenz (nur mit `effects.belt_span_vibration`); besteht ab dem Grenzwert | `ratings.belt_span_resonance_band` | Anteil |

Grenzen bei dieser Aufgabe:

- Ein Grenzwert, den die Konfiguration nicht angibt, wird nicht geprüft. Eine fehlende Zeile heißt nicht „bestanden“. Ebenso werden `gearbox-thermal` und `guide-thermal` bei ausgeschaltetem `effects.part_heating` und `effects.thermal_feedback` nicht geprüft, `belt-tooth-jump` ohne `effects.belt_tooth_jump` und `belt-span-resonance` ohne `effects.belt_span_vibration`. Ist der Effekt an, erreicht die X-Führung des EXCT-30 im eingeschwungenen Zustand 25 K von 40 K und fällt im ungünstigsten Fall von `--bands` durch (51 K an der oberen Grenze ihres Wärmewiderstands); die Wärmewiderstände sind Schätzungen (siehe Konfigurationsreferenz).
- `belt-slack` und die drei Lebensdauern sind die Prüfungen, bei denen der Wert über der Grenze bleiben muss. `belt-slack` fällt auch dann durch, wenn irgendein Trum irgendwann ohne Riemenkraft war.
- Die Lebensdauern sind die Formeln der Hersteller mit Tragzahlen, die zum Teil Schätzungen sind; was sie auslassen, steht in der [Modellreferenz](../model-reference.md#analysis-quantities). Die Lager der Motoren selbst werden nicht abgeschätzt: In diesen Konfigurationen zieht der Riemen an keiner Motorwelle.
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
dc-link-energy,a,6.166,6.205,ok
dc-link-overvoltage,a,224.3,257.6,ok
...
dc-link-energy,b,6.166,6.205,ok
dc-link-overvoltage,b,224.3,257.6,ok
...
```

Im Standardzyklus ist die höchste Zwischenkreisspannung 224,65 V mit getrennten Zwischenkreisen (Motor B, an der Einschaltspannung des Bremschoppers) und 224,05 V mit dem gemeinsamen. Der Bremswiderstand wird dann nicht gebraucht; seine beiden Zeilen fehlen.

**Ins Netz zurückspeisen.** `effects.regenerative_supply` (standardmäßig aus, braucht `effects.dc_link`) setzt an die Stelle des einfachen Gleichrichters ein rückspeisefähiges Line Module, wie ein Active Line Module an einer SINAMICS S120: Es hält den Zwischenkreis auch von oben auf dem Netzniveau und speist, was die Motoren zurückgeben, ins Netz zurück, bis `ratings.line_module_max_power_w`, wovon `ratings.line_module_efficiency` dort ankommt. Der Bremswiderstand nimmt dann nur, was über die Leistung des Line Modules hinausgeht. `analyze` gibt die zurückgespeiste Energie je Motor aus (letzte Spalte, `mains_energy_j`), die Energiebilanz bekommt die Zeilen `mains` und `line_module` (dessen Verlust), die App zeigt beides, und zwei Prüfungen beurteilen das Line Module: `line-module-continuous` (seine mittlere Leistung, bezogen und zurückgespeist, gegen `line_module_rated_power_w`) und `line-module-peak` (seine höchste Leistung gegen `line_module_max_power_w`). Die mitgelieferten Werte sind das kleinste Active Line Module der S120, 16 kW Bemessungs- und 35 kW Maximalleistung, weit über dem, was diese Maschinen brauchen. Beim Standard-T-Portal gehen von den 25,8 J, die die Motoren je Zyklus zurückgeben, 25,2 J ins Netz; der Bremswiderstand verbrennt nichts, und der Zwischenkreis bleibt bei 155,13 V.

## Eine reale Katalogmaschine prüfen

Eine Konfiguration kann eine reale Maschine mit Werten aus dem Katalog beschreiben. [`exct-30.toml`](../../crates/machine-t-gantry/exct-30.toml) tut das für eine Festo EXCT-30 mit zwei Motoren Siemens 1FK2205-4AF an Umrichtern S210 bei 400 V. Jeder Wert darin ist mit `[catalogue]` (Katalog) oder `[estimate]` (Schätzung) gekennzeichnet.

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,...
a,2.048000,3.659797,1.459734,156.9832,0.2930,0.3173,257.77,554.97,1.1446,6.88,...
b,2.048000,3.036510,1.391431,156.9736,0.2430,0.3025,257.77,554.97,1.0908,6.24,...
```

Fünf Werte dieser Konfiguration hat der Konstrukteur so gewählt, dass die Prüfungen auch im ungünstigsten Fall der Fehlerbänder besteht ([unten](#einfluss-unsicherer-parameter-erkennen-und-mit-fehlerbändern-beurteilen)): eine Vorspannung des Riemens von 450 N, einen Ruckgrenzwert in X von 150 m/s³, Input Shaping mit festen 25 Hz und einem zweiten Shaper bei 17,5 Hz und das Überschleifen. Die Gründe stehen in den Kommentaren der Datei.

Wie die Standardkonfigurationen hat auch diese `effects.dc_link = true`. Die Zeilen zum Bremsen in `verdicts` sind also simuliert, nicht geschätzt: Die Zwischenkreisspannung steigt bis zur Einschaltspannung des Bremschoppers von 770 V, und der Bremswiderstand setzt den Rest in Wärme um.

```text
dc-link-energy,a,9.943,9.943,ok
braking-resistor-continuous,a,8.303,100.0,ok
braking-resistor-peak,a,200.5,3000,ok
dc-link-overvoltage,a,770.0,820.0,ok
```

**Bauteile nach Namen.** `[parts]` in einer Konfiguration oder `--motor`, `--supply`, `--gearbox`, `--drive`, `--belt` und `--axis` auf der Befehlszeile benennen einen Motor, ein Getriebe, einen Antrieb, einen Riemen und eine Achse aus dem Katalog. Deren Katalogwerte ersetzen dann die Zahlen der Konfiguration. Sie werden in der Reihenfolge Achse, Riemen, Getriebe, Motor, Antrieb eingesetzt; ein neben einer Achse benannter Riemen ersetzt also den Riemen der Achse:

| Bauteil | Was es setzt |
| --- | --- |
| Motor | `motor.rotor_inertia_kgm2`, `motor.torque_constant_nm_a` (Stillstandsmoment geteilt durch Stillstandsstrom), `motor.max_current_a`, `motor.torque_constant_at_max_current_nm_a`, `motor.rated_current_a` (der Wert des Katalogs, höchstens das 1,05-Fache des Stillstandsstroms), `motor.thermal_time_constant_s` (wo das Projektierungshandbuch sie nennt), `drive.peak_torque_nm` (begrenzt auf `ratings.pulley_max_torque_nm` geteilt durch die Getriebeübersetzung), `drive.rated_torque_nm`, `drive.rated_speed_rad_s`, `drive.bus_voltage_v` (die Zwischenkreisspannung: 540 V bei `400v`, 325 V bei `230v`), `drive.no_load_speed_rad_s` (sqrt(3/2) × Zwischenkreisspannung geteilt durch die Drehmomentkonstante) sowie `ratings.braking_threshold_v` und `ratings.dc_link_overvoltage_v`, beide mit der Zwischenkreisspannung skaliert |
| Getriebe | `gearbox.ratio`, `gearbox.efficiency`, `gearbox.stiffness_nm_rad`, `gearbox.backlash_rad` und `gearbox.inertia_at_input_kgm2` (die beiden letzten, wo das Handbuch sie nennt), `ratings.gearbox_rated_torque_nm` und `ratings.gearbox_max_torque_nm` (wo die Konfiguration sie hat) |
| Motor am S120 (`--supply s120`) | Dieselben Schlüssel aus den S120-Daten: für einen 1FK7 oder 1FT7 aus [`motors_s120.csv`](../../catalog/siemens/motors_s120.csv) und seinem Datenblatt (Maximalmoment und -strom, thermische Zeitkonstante), für einen 1FK2 oder 1FT2 aus seinem S120-Projektierungshandbuch. Ein 1FK7 oder 1FT7 bringt auch seine Wicklung mit: `motor.resistance_ohm` und `motor.inductance_h` sind das Dreifache der Strangwerte des Datenblatts (die eine Wicklung des Modells führt den Strangstrom bei dreifacher Strangspannung), und der Stromregler wird damit neu eingestellt, `drive.current_kp_v_a` = L / T mit der Zeitkonstante T des Stromkreises der Konfiguration (ihre Induktivität geteilt durch ihre Verstärkung) und `drive.current_tn_s` = L / R. Der Zwischenkreis hat 540 V |
| Antrieb | Ein S210-Umrichter ([`converters_s210.csv`](../../catalog/siemens/converters_s210.csv)) oder ein S120 Motor Module ([`motor_modules_s120.csv`](../../catalog/siemens/motor_modules_s120.csv)) mit seiner Artikelnummer: `ratings.drive_rated_current_a` und `ratings.drive_max_current_a`, sein Bemessungs- und sein Maximalausgangsstrom (bei einem Double Motor Module je Achse). `verdicts` beurteilt den Effektivwert des Motorstroms im Zyklus gegen den ersten (`drive-current`) und seine Spitze gegen den zweiten (`drive-peak-current`). Die Strombegrenzung des Antriebs selbst wird nicht simuliert |
| Riemen | Als Profil-Breite, z. B. `AT5-25`: ein Riemen MEGALINEAR von Megadyne mit Standard-Stahlcord ([`megadyne/belts.csv`](../../catalog/megadyne/belts.csv); T5, AT3, AT5, AT10 und die HTD-kompatiblen MTD3, MTD5, MTD8). `belt.stiffness_ea_n` (die zulässige Zugkraft geteilt durch die Dehnung bei ihr, 4 mm/m), `belt.mass_kg_m`, `belt.tooth_pitch_m` und, wo die Konfiguration sie hat, `ratings.belt_max_tension_n` (die zulässige Zugkraft, ein Viertel der Bruchkraft), `ratings.belt_width_mm` und die Zahnbelastungstabelle `ratings.belt_tooth_load_rpm` und `_n_cm`. Eine Breite zwischen zwei Standardbreiten wird interpoliert, eine außerhalb im Verhältnis zur nächsten genommen |
| Achse | Mit Namen: `EXCT-15`, `EXCT-30`, `EXCT-100` (Festo, [`festo/exct.toml`](../../catalog/festo/exct.toml)) und die Achsen und X/Z-Portale von Bahr als Baureihe-Größe, z. B. `ELZI-40` oder `ELZ-60S` ([`bahr/belt_axes.csv`](../../catalog/bahr/belt_axes.csv)). Ein X/Z-Portal gehört zu einer Konfiguration mit Z-Hub, eine Einzelachse zu einer mit einem Hub; umgekehrt, und ein Hub, den die Achse nicht hat, wird abgelehnt. Setzt `geometry.pulley_radius_m`, den Riemen (wie oben, für Profil und Breite der Achse: die EXCT von Festo als AT, die HTD-Riemen von Bahr als MTD; die Dehnsteifigkeit der EXCT aus Festos angegebener Dehnung), `ratings.belt_teeth_in_mesh` (die Hälfte der Zähne der Scheibe), `ratings.pulley_max_torque_nm`, wo die Konfiguration es hat (Festos Maximalmoment; Bahrs Spitzenvorschubkraft mal Radius), und die bewegten Massen: Die Portal-Datenblätter von Bahr nennen sie je Achse (Z-Achse, und der Schlitten als X weniger Z); Festo nennt die Z-Masse seiner Momentformel, und der Schlitten wird so angepasst, dass die Maschine bei 45 Grad Festos Moment braucht, wie `exct-30.toml` geschrieben wurde. Die Hübe bleiben; eine EXCT erhält den kürzesten Z-Hub, den Festo anbietet und der den der Konfiguration fasst. Die Einzelachsen von Bahr nennen keine bewegte Masse, also bleibt der Schlitten der Konfiguration |
| Alle außer dem Antrieb | `drive.velocity_kp_nm_s_rad`, für das Bauteil nachgeführt (siehe unten) |

Was ein Bauteil nach Namen **nicht** setzt und was daraus folgt:

- **Wicklungswiderstand und Wicklungsinduktivität.** Siemens veröffentlicht sie für 1FK2 und 1FT2 nicht. Die Werte der Konfiguration bleiben, und mit ihnen die Verstärkungen des Stromreglers. (Für 1FK7 und 1FT7 am S120 werden sie gesetzt, siehe oben.)
- **Der Antrieb zum Motor.** Einen Motor zu benennen benennt nicht seinen Antrieb; die Antriebs-Grenzwerte der Konfiguration bleiben. Die Suche ordnet jedem Motor seinen Antrieb zu (unten).
- **Die Zwischenkreiskapazität und die Leistungen des Bremswiderstands.** Sie bleiben, wie die Konfiguration sie hat. Die Einschaltspannung des Bremswiderstands und die Abschaltspannung behalten ihr Verhältnis zur Zwischenkreisspannung. Für Platzhalter ist das vertretbar, mehr nicht: Setzen Sie alle fünf Zwischenkreis-Grenzwerte für den realen Antrieb, bevor Sie den Zeilen zum Bremsen trauen.
- **Antriebsdrehzahl, Radialkraft und Lebensdauer des Getriebes.** Die Getriebedateien enthalten sie nicht. `ratings.gearbox_max_input_speed_rpm`, `ratings.shaft_max_radial_load_n`, `ratings.pulley_bearing_load_rating_n` und `ratings.gearbox_reference_speed_rpm` bleiben, wie die Konfiguration sie hat.

**Die Drehzahlverstärkung wird nachgeführt.** Ein Motor mit deutlich leichterem Rotor schwingt mit einer Verstärkung, die für einen schwereren eingestellt wurde. Das ist ein Fehler der Reglereinstellung, nicht des Motors. Überall, wo ein Bauteil benannt wird (`[parts]`, `--motor`, `--gearbox`, die Suche), wird `drive.velocity_kp_nm_s_rad` deshalb skaliert: mit der Quadratwurzel aus (Verhältnis der Trägheiten, die die Motoren drehen) × (Verhältnis der Rotorträgheiten, die Trägheit des Getriebes eingeschlossen). Bezugspunkt sind die Zahlen der Konfiguration, wie die Datei sie nennt, bevor das Bauteil eingesetzt wird: Die Verstärkung in einer Konfiguration gilt als eingestellt für den Rotor, das Getriebe und die Massen, die in dieser Konfiguration stehen. `--set drive.velocity_kp_nm_s_rad=...` auf der Befehlszeile wird nach den Bauteilen angewendet und überschreibt die nachgeführte Verstärkung.

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --motor 1FK2204-6AF
motor-peak,a,0.3636,1.000,ok
motor-thermal,a,12.07,100.0,ok
...
settling,,0.00004415,0.0001000,ok
belt-slack,,252.0,0,ok
```

Der Rotor des 1FK2204-6AF hat ein Drittel der Trägheit des 1FK2205-4AF aus der Konfiguration; die Verstärkung geht von 0,45 auf 0,23 N m s/rad. (Für diesen Motor ist zufällig auch 0,45 ruhig.) Der 1FK2104-5AF hat ein Neuntel der Trägheit: Bleibt die Verstärkung bei 0,45, schwingt er, und seine Wicklung kommt nie zur Ruhe; auf 0,13 angepasst fährt er den Zyklus mit 1,22 N m Effektivmoment.

Eine unbekannte Artikelnummer wird abgelehnt: `Der Katalog kennt dieses Bauteil nicht: Motor 1FK9`. Ebenso ein Motor, den der Katalog für die gewählte Netzspannung nicht anbietet, und unbekannte Antriebe, Riemen und Achsen.

Die Achse von `exct-30.toml` mit Namen einzusetzen gibt deren eigene Zahlen zurück (der Test `an_axis_by_name_gives_the_numbers_of_the_hand_written_exct_30`): Der neu angepasste Schlitten hat 7,136 kg gegen 7,16 kg in der Datei, die Riemenmasse aus der AT5-Tabelle 97,9 g/m gegen den Schätzwert 0,1 kg/m der Datei.

## Den leichtesten Motor finden

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --series 1FK2
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 13 / 28 candidates
Search time on 16: 0.3 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,5.70,ok,pulley-bearing-life,0.6309,3.2886,20.44
,1FK2104-5AK,,6SL3210-5HE11-5UF0,2.85,5.70,ok,pulley-bearing-life,0.6309,3.2977,41.56
,1FK2204-5AF,,6SL3210-5HE10-8UF0,2.90,5.80,ok,pulley-bearing-life,0.6310,3.4296,21.55
,1FK2104-6AF,,6SL3210-5HE11-0UF0,3.40,6.80,ok,pulley-bearing-life,0.6311,3.3446,11.27
,1FK2204-6AF,,6SL3210-5HE11-0UF0,3.50,7.00,ok,pulley-bearing-life,0.6311,3.4468,12.07
,1FK2205-2AF,,6SL3210-5HE11-0UF0,3.75,7.50,ok,pulley-bearing-life,0.6311,3.5158,14.67
,1FK2205-4AF,,6SL3210-5HE11-5UF0,5.20,10.40,ok,pulley-bearing-life,0.6310,3.6598,6.88
...
,1FK2204-5AK,,6SL3210-5HE11-5UF0,2.90,5.80,FAIL,motor-thermal,1.6796,3.4378,167.96
,1FK2203-2AK,,6SL3210-5HE10-4UF0,1.16,2.32,FAIL,limit-motor-torque,NaN,NaN,NaN
...
,1FK2208-3AC,,6SL3210-5HE13-5UF0,12.60,25.20,FAIL,limit-motor-speed,NaN,NaN,NaN
```

An der EXCT-30 hat jeder Motor, der besteht, dieselbe knappste Prüfung, die Lebensdauer der Lager der Scheibenwellen (20000 h gefordert von etwa 31700 h), die kaum vom Motor abhängt: Vergleichen Sie die Motoren nach `winding_rise_k` und `peak_torque_nm`.

Die Suche fährt den Zyklus der Konfiguration mit jedem Motor des Katalogs, der für die Netzspannung angeboten wird, jeden mit dem Antrieb, den der Katalog ihm zuordnet, auf allen Prozessorkernen. Zuerst kommen die Motoren, die bestehen, der günstigste voran. Danach folgen die, die nicht bestehen, der knappste voran. Ein Kandidat, dessen Antriebe wegen Überspannung im Zwischenkreis abschalten, steht hinter allen diesen, mit `inf` als `tightest_utilisation`: Seine Werte enden mit dem Abschalten.

**Kosten.** Keine Katalogdatei enthält einen Preis, und Siemens veröffentlicht keine Listenpreise auf einem Server, der sich automatisch lesen lässt (die Industry Mall lehnt solche Zugriffe ab). Für den Preis steht die eingebaute Masse: `cost_kg` ist Motor und Getriebe mal der Zahl der Motoren, dazu die Achse, wo eine des Katalogs probiert wird (`--axes`). Bestandene Kandidaten werden danach geordnet, dann nach `mass_kg`. Bei einer Motorsuche an einer festen Maschine ergeben beide dieselbe Reihenfolge.

| Option | Wirkung |
| --- | --- |
| `--series PREFIX` | Nur Artikelnummern, die so beginnen, z. B. `1FK2`, `1FT2105` oder `1FK7` |
| `--supply 230v`, `400v` oder `s120` | `230v` und `400v`: die Motoren des S210 ([`motors_s210.csv`](../../catalog/siemens/motors_s210.csv)) bei dieser Netzspannung, jeder mit dem S210-Umrichter, den der Katalog D 32 neben ihm nennt (bei 230 V der einphasige, wo es einen gibt). `s120`: die Motoren an einem SINAMICS S120 an 400 V, die 1FK7 und 1FT7 aus [`motors_s120.csv`](../../catalog/siemens/motors_s120.csv) mit Datenblatt und die 1FK2 und 1FT2 aus ihren S120-Handbüchern, jeder mit seinem Single Motor Module (siehe [Bauteile nach Namen](#eine-reale-katalogmaschine-prüfen)). Standard `400v` |
| `--gearboxes SERIES` | Jeden Motor zusätzlich mit jedem Getriebe dieser Baureihe probieren (`NRB`, `NRK`, `NLC` oder eine Baugröße wie `NRK070`), das an seinen Flansch passt und dessen Übersetzung zwischen der Hälfte und dem Doppelten der Übersetzung der Konfiguration liegt. Nur bei einer Maschine, deren Konfiguration eine Übersetzung über 1 hat, und nur für 1FK2 und 1FT2 (für sie sind die Getriebe gebaut) |
| `--axes` | Zusätzlich jede Achse des Katalogs probieren, die zur Art der Maschine passt (siehe unten) |

| Spalte | Bedeutung |
| --- | --- |
| `axis` | Die Achse des Katalogs, auf der der Motor probiert wurde; leer für die eigene der Konfiguration |
| `motor`, `gearbox` | Der Kandidat. `gearbox` ist leer, wo das Getriebe der Konfiguration geblieben ist |
| `drive` | Der Antrieb, den der Katalog dem Motor zuordnet; der Zyklus wird gegen seine Ströme beurteilt (`drive-current`, `drive-peak-current`) |
| `mass_kg` | Masse des Motors ohne Bremse und seines Getriebes (Projektierungshandbuch von Siemens 01/2023, siehe [den Katalog](../catalogue.md)) |
| `cost_kg` | Was für den Preis steht: die eingebaute Masse, siehe oben |
| `result` | `ok`, wenn jede Prüfung bestanden ist |
| `tightest_check` | Die Prüfung, die ihrer Grenze am nächsten kommt oder sie am weitesten überschreitet. Zwei Zwischenkreisprüfungen bleiben außen vor, solange sie bestehen, weil ihr Verhältnis nichts über die Reserve sagt: `dc-link-energy`, wenn ein Bremswiderstand arbeitet (die Kondensatoren füllen sich dann planmäßig bis zur Grenze), und `dc-link-overvoltage` (die Spannung liegt nie unter dem Ruhewert des Zwischenkreises). Bei einem Kandidaten, der nicht simuliert wurde: der Grund (`limit-motor-speed`, `limit-motor-torque`, `axis-stroke`, `axis-payload`, `axis-speed`) |
| `tightest_utilisation` | Wert geteilt durch Grenze dieser Prüfung |
| `peak_torque_nm` | Höchstes Spitzenmoment aller Motoren der Maschine |
| `winding_rise_k` | Höchste eingeschwungene Übertemperatur der Wicklung über alle Motoren |

Bevor ein Zyklus simuliert wird, werden die Katalogwerte gegen die Achsgrenzwerte geprüft: die Motordrehzahl bei der höchsten Achsgeschwindigkeit gegen die Bemessungsdrehzahl, das Moment des Starrkörpermodells bei der höchsten Beschleunigung gegen das Maximalmoment des Motors. Was hier scheitert, wird mit `NaN` aufgeführt und nicht simuliert. Der Fehlerkanal nennt, wie viele Kandidaten das waren und wie lange die Suche auf wie vielen Kernen gedauert hat.

Die Drehzahlverstärkung jedes Kandidaten wird nach der Regel aus [Bauteile nach Namen](#eine-reale-katalogmaschine-prüfen) nachgeführt: Quadratwurzel aus (Verhältnis der Trägheiten, die die Motoren drehen) × (Verhältnis der Rotorträgheiten), bezogen auf die Zahlen der Konfiguration. `--gearbox NAME` für die ganze Suche und `--gearboxes SERIES` liefern für dasselbe Paar dasselbe Ergebnis: Die Trägheit des Getriebes hat einen eigenen Schlüssel (`gearbox.inertia_at_input_kgm2`) und bleibt, wenn ein Motor eingesetzt wird.

Mit Getrieben, an der Einzelachse:

```text
$ sim-cli search single-axis --series 1FK2104 --gearboxes NRK070
Ohne Simulation eines Zyklus ausgeschlossen, nach Katalogwerten gegen die Achsgrenzwerte: 12 / 35 Kandidaten
Dauer der Suche auf 16: 0.3 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK2104-4AF,NRK070-04-3,6SL3210-5HE10-4UF0,3.45,3.45,ok,gearbox-radial-load,0.4188,0.7670,3.85
,1FK2104-4AF,NRK070-04-4,6SL3210-5HE10-4UF0,3.45,3.45,ok,settling,0.4568,0.6009,2.50
,1FK2104-4AF,NRK070-04-5,6SL3210-5HE10-4UF0,3.45,3.45,ok,settling,0.4637,0.5294,1.91
...
```

Die Motoren des S120 an der EXCT-30:

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --supply s120 --series 1FK7
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 32 / 61 candidates
Search time on 16: 0.7 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
,1FK7042-2AF7,,6SL3120-1TE13-0AD0,4.60,9.20,ok,pulley-bearing-life,0.6311,3.5060,18.72
,1FK7042-2AK7,,6SL3120-1TE15-0AD0,4.60,9.20,ok,pulley-bearing-life,0.6311,3.5060,54.09
...
```

**Achskombinationen.** `--axes` behält Zyklus, Nutzlast und Hübe der Konfiguration und probiert jeden Motor auf jeder Achse des Katalogs, die zur Art der Maschine passt: beim T-Portal die Festo EXCT-15, -30 und -100 und die X/Z-Portale ELZI 30, 40 und 60 von Bahr; bei der Einzelachse die 56 Einzelachsen von Bahr. Jede Achse wird eingesetzt wie mit `[parts] axis` ([Bauteile nach Namen](#eine-reale-katalogmaschine-prüfen)). Bevor ein Motor probiert wird, schließen die Katalogwerte der Achse sie aus, wenn der Hub länger ist, als sie hat (`axis-stroke`; eine EXCT erhält den kürzesten Z-Hub, den Festo anbietet und der den der Konfiguration fasst), die Nutzlast über ihrer Nennlast liegt (`axis-payload`; Festos Nennlast, bei Bahr die dynamische Vertikallast des Schlittens gegen Z-Achse und Nutzlast) oder ein Achsgrenzwert der Konfiguration über ihrer Geschwindigkeit oder Beschleunigung liegt (`axis-speed`). Eine solche Achse hat eine Zeile ohne Motor. Auf dem Zyklus der EXCT-30, alle Motoren des S210 an 400 V:

```text
$ sim-cli search t-gantry --config crates/machine-t-gantry/exct-30.toml --axes
Ruled out by catalogue numbers against the axis limits, without simulating a cycle: 124 / 338 candidates
Search time on 16: 4.6 s
axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k
ELZI-40,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,24.75,ok,settling,0.9078,3.9932,25.08
ELZI-40,1FK2104-5AK,,6SL3210-5HE11-5UF0,2.85,24.75,ok,settling,0.9087,3.9946,51.85
ELZI-40,1FK2204-5AF,,6SL3210-5HE10-8UF0,2.90,24.85,ok,dc-link-energy,0.6367,3.8864,26.96
...
EXCT-30,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,46.80,ok,pulley-bearing-life,0.6310,3.2869,20.39
...
EXCT-100,1FK2104-5AF,,6SL3210-5HE10-8UF0,2.85,56.93,ok,settling,0.7292,5.0201,52.65
...
EXCT-30,1FK2204-5AK,,6SL3210-5HE11-5UF0,2.90,46.90,FAIL,motor-thermal,1.6727,3.4351,167.27
...
ELZI-30,,,,NaN,NaN,FAIL,axis-speed,NaN,NaN,NaN
EXCT-15,,,,NaN,NaN,FAIL,axis-stroke,NaN,NaN,NaN
```

Das ELZI 40 von Bahr ist die leichteste Achse, die diesen Zyklus trägt (19,05 kg bei 1 m mal 0,25 m, gegen 41,1 kg für die EXCT-30), und steht deshalb vorn. Die EXCT-15 hat keinen Z-Hub von 250 mm; das ELZI 30 erlaubt 2 m/s, die Konfiguration verlangt 2,5 m/s. Lesen Sie die ELZI-Zeilen mit Vorsicht: Input Shaper, Riemenvorspannung, Reibung, Rahmen und Werkzeug sind Schätzwerte für die EXCT-30, und Bahr veröffentlicht weder die Nennlast des Portals noch die Dehnung des Riemens (der Riemen ist Megadynes HTD-kompatibler MTD5 in der Breite des Datenblatts).

Grenzen bei dieser Aufgabe:

- **Das Gewicht steht für den Preis.** Siehe oben. Das Gewicht des Antriebs zählt nicht mit: Die Umrichterdateien enthalten keines.
- Das Nachführen der Verstärkung ist eine Faustregel, keine Reglereinstellung nach dem Frequenzgang der Maschine. Ein Kandidat kann immer noch an seiner Verstärkung scheitern. Prüfen Sie einen gewählten Motor mit `tuning`.
- Die Vorauswahl nach dem Moment lässt den Getriebewirkungsgrad außer Acht.
- Die Achssuche behält alles von der Konfiguration, was die Achse nicht setzt: Getriebe, Reibung, Rahmen, Input Shaper und Verstärkungen. Die Einzelachsen von Bahr nennen keine bewegte Masse, also bleibt bei ihnen der Schlitten der Konfiguration; die maximalen Längen von Bahr gelten als längster Hub.
- Riemen werden nicht variiert; ein Riemen wird mit `--belt` benannt. Kombinationen verschiedener Motoren je Achse werden nicht gesucht.
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
friction_x.viscous_n_s_m,20.0,10.0,40.0,-0.0435,+0.0869,-0.0172,+0.0469,+0.0000,+0.0000,264.84,242.04
mass.z_axis_kg,4.7,3.7600000000000002,5.64,-0.0620,+0.0622,-0.0675,+0.0689,+0.0000,+0.0000,268.68,246.11
mass.carriage_kg,7.16,5.728000000000001,8.592,-0.0540,+0.0549,-0.0472,+0.0487,+0.0000,+0.0000,272.14,243.18
mass.payload_kg,3.0,2.4000000000000004,3.5999999999999996,-0.0342,+0.0343,-0.0279,+0.0283,+0.0000,+0.0000,265.01,250.05
friction_x.coulomb_n,20.0,12.0,26.0,-0.0188,+0.0142,-0.0096,+0.0078,+0.0000,+0.0000,263.44,253.31
...
belt.pretension_n,450.0,350.0,550.0,+0.0001,-0.0001,+0.0000,-0.0000,+0.0000,+0.0000,157.78,357.77
...
belt.tooth_mesh_stiffness_n_m,5000000.0,2500000.0,10000000.0,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,+0.0000,257.77,257.77
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

Die Massen im Beispiel haben in der Konfiguration keine Grenzen und wurden deshalb um 20 % variiert. Ein Parameter, dessen geänderten Wert die Maschine ablehnt, fehlt in der Tabelle. Die Vorspannung ändert die Momente kaum und steht deshalb weit unten, sie bestimmt aber die kleinste Riemenkraft: 158 N an ihrer unteren Grenze. Die letzte Zeile zeigt einen Parameter eines abgeschalteten Effekts: Er ändert nichts.

**Fehlerbänder zu den Prüfungen.**

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands
check,motor,value,limit,result,low,high,worst_case
motor-peak,a,0.2930,1.000,ok,0.2826,0.3151,ok
motor-thermal,a,6.882,100.0,ok,6.640,7.565,ok
drive-current,a,1.145,5.000,ok,1.125,1.198,ok
drive-peak-current,a,2.874,19.00,ok,2.770,3.091,ok
pulley-torque,a,3.126,12.50,ok,2.993,3.395,ok
pulley-bearing-life,a,31788,20000,ok,17349,67792,FAIL
belt-tooth-load,a,0.3529,1.000,ok,0.2941,0.5042,ok
dc-link-energy,a,9.943,9.943,ok,4.971,23.24,ok
braking-resistor-continuous,a,8.303,100.0,ok,4.537,13.16,ok
braking-resistor-peak,a,200.5,3000,ok,138.7,218.2,ok
dc-link-overvoltage,a,770.0,820.0,ok,709.1,790.0,ok
...
settling,,0.00003947,0.0001000,ok,0.00003446,0.00004434,ok
belt-tension,,555.0,2140,ok,455.0,655.0,ok
belt-slack,,257.8,0,ok,157.8,357.8,ok
gearbox-radial-load,,1000,2000,ok,800.4,1200,ok
guide-load,,185.0,2000,ok,177.0,276.7,ok
guide-life,,3024172,20000,ok,378021,24193374,ok
```

`low` und `high` sind der kleinste und der größte Wert, den die Prüfung über alle Läufe angenommen hat: die Konfiguration, wie sie ist, und jeder Parameter mit Grenzen an seiner unteren und an seiner oberen Grenze. `worst_case` ist nur dann `ok`, wenn die Prüfung in jedem Lauf bestanden wurde. Hier besteht jede Prüfung bis auf eine in jedem Lauf: Mit der Vorspannung an ihrer unteren Grenze von 350 N behält das schlaffste Trum 158 N Riemenkraft, das Werkzeug steht beim Verweilen höchstens 0,044 mm neben seinem Ziel, und der Riemen zieht mit höchstens 1200 N an einer Scheibenwelle. Die Lager der Scheibenwellen fallen durch: 31800 h wie geschrieben, 17300 h mit dem Riemen 100 N zu stramm, bei 20000 h gefordert. Ihre Tragzahl ist eine Schätzung (Festo veröffentlicht keine; siehe den Kommentar in der Konfiguration). Das heißt also, dass nach diesen Lagern bei Festo zu fragen ist, nicht, dass die Achse verschleißt. Dieser Lauf hat 127 Zyklen simuliert (63 Werte mit Grenzen), auf allen Prozessorkernen, und dauerte etwa 2,6 s auf dem Rechner, auf dem dieses Handbuch entstand.

Ein früherer Stand dieser Konfiguration fiel hier durch: Mit 300 N Vorspannung wurde ein auf 200 N eingestellter Riemen schlaff, und am weichen Ende der Grenzen für das Gestell stand das Werkzeug beim Verweilen 0,16 mm neben seinem Ziel, bei einer Toleranz von 0,1 mm (die Werkzeugposition enthält das Schwanken des Gestells). Die Vorspannung wurde auf 450 N erhöht, der Ruckgrenzwert in X von 500 auf 150 m/s³ gesenkt, und Input Shaping und das Überschleifen wurden eingeschaltet; später kam für die ungünstigste Ecke (unten) ein zweiter Shaper bei 17,5 Hz hinzu. Der Zyklus dauert jetzt 2,048 s, vorher 2,136 s.

**Alle Werte mit Grenzen zugleich.** Mit `--samples N` (Stichproben) wird der Zyklus N-mal gefahren, jedes Mal mit allen Werten mit Grenzen zwischen ihren Grenzen:

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --samples 200
check,motor,value,limit,result,low,high,worst_case,pass_share
motor-peak,a,0.2930,1.000,ok,0.2799,0.3435,ok,1.000
motor-thermal,a,6.882,100.0,ok,6.421,8.347,ok,1.000
drive-current,a,1.145,5.000,ok,1.107,1.257,ok,1.000
drive-peak-current,a,2.874,19.00,ok,2.743,3.371,ok,1.000
pulley-torque,a,3.126,12.50,ok,2.959,3.719,ok,1.000
pulley-bearing-life,a,31788,20000,ok,11887,126380,FAIL,0.860
belt-tooth-load,a,0.3529,1.000,ok,0.2876,0.5665,ok,1.000
dc-link-energy,a,9.943,9.943,ok,4.781,28.61,ok,1.000
braking-resistor-continuous,a,8.303,100.0,ok,0.03481,13.44,ok,1.000
braking-resistor-peak,a,200.5,3000,ok,6.788,231.5,ok,1.000
dc-link-overvoltage,a,770.0,820.0,ok,675.3,789.5,ok,1.000
...
settling,,0.00003947,0.0001000,ok,0.00002569,0.00005131,ok,1.000
belt-tension,,555.0,2140,ok,445.9,665.0,ok,1.000
belt-slack,,257.8,0,ok,139.6,357.9,ok,1.000
gearbox-radial-load,,1000,2000,ok,797.3,1209,ok,1.000
guide-load,,185.0,2000,ok,173.1,308.7,ok,1.000
guide-life,,3024172,20000,ok,291334,22756879,ok,1.000
```

Die Werte werden als lateinischer Hyperwürfel gezogen: Der Bereich jedes Werts wird in N gleiche Teile geschnitten, jeder Teil kommt in genau einem Lauf vor, und die Reihenfolge der Teile ist je Wert zufällig. Die Zufallszahlen stammen aus `--seed S` (Standard 1); derselbe Aufruf ergibt also dieselbe Tabelle. `low`, `high` und `worst_case` sind die Hüllkurve dieser Läufe und der Konfiguration, wie sie ist. `pass_share` ist der Anteil der Läufe, die Konfiguration, wie sie ist, eingeschlossen, in denen die Prüfung bestanden wurde. Hier sind die Bänder breiter als mit einem Wert allein (140 N statt 158 N im schlaffsten Trum, 0,051 mm statt 0,044 mm neben dem Ziel beim Verweilen, 1209 N an einer Scheibenwelle), und jede Prüfung besteht in allen 201 Läufen außer den Lagern der Scheibenwellen, die in 14 % der Läufe keine 20000 h halten (mindestens 11900 h). Das dauerte etwa 4 s.

`--samples` ohne `--bands`, 0 Stichproben, eine gebrochene Zahl und `--seed` ohne `--samples` werden abgelehnt (Code 2). Die Läufe beider Arten nutzen alle Prozessorkerne.

**Die ungünstigste Ecke.** Mit `--worst` suchen die Fehlerbänder je Prüfung die Kombination der Grenzen, die sie am ungünstigsten macht:

```text
$ sim-cli verdicts t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --worst
Cycles simulated in the search for the worst corners: 2428
check,motor,worst,limit,worst_case,at_high_bound,at_low_bound
motor-peak,a,0.4109,1.000,ok,...
...
pulley-bearing-life,a,10258,20000,FAIL,...
...
dc-link-energy,a,35.66,36.57,ok,...
...
settling,,0.00009484,0.0001000,ok,...
belt-tension,,687.7,2140,ok,...
belt-slack,,86.89,0,ok,...
gearbox-radial-load,,1231,1500,ok,...
guide-load,,354.9,1000,ok,...
guide-life,,115600,20000,ok,...
```

Sie beginnt mit den Läufen mit einem Wert allein: Jede Prüfung bekommt die Ecke, an der jeder Wert mit Grenzen an der Grenze steht, die diese Prüfung allein ungünstiger machte. Dann wird bis zu dreimal jeder Wert, der die Prüfung bewegt hat, einzeln an seine andere Grenze gesetzt, und was sie ungünstiger macht, bleibt (alle solchen Wechsel zusammen oder der beste allein, je nachdem, was ungünstiger ist). Die letzten zwei Spalten nennen die Werte an ihrer oberen und an ihrer unteren Grenze, getrennt durch `;`. „Ungünstiger“ wird nach dem Abstand zur Grenze beurteilt, weil manche Grenzwerte selbst Grenzen haben: in dieser Konfiguration Kapazität und Schaltschwelle des Zwischenkreises; die Spalte `limit` zeigt daher den Grenzwert an dieser Ecke. `dc-link-energy` wird stattdessen nach der Energie beurteilt: Die Prüfung fällt nur durch, wo der Bremswiderstand gebraucht wird und keinen Grenzwert hat, und mit Widerstand füllt sich der Zwischenkreis an jeder Ecke mit kräftigem Bremsen bis zu seinem Platz, sodass ihr Abstand nichts aussagt (hier 35,66 J von 36,57 J Platz, die Kapazität an ihrer oberen Grenze; 9,94 J, wie konfiguriert). Die Läufe mit einem Wert allein und die Konfiguration, wie sie ist, zählen ebenfalls, sodass das Ungünstigste nie günstiger ist als die Bänder; ist einer von ihnen der ungünstigste, stehen die Werte, die er lässt, wie sie konfiguriert sind, in keiner der beiden Spalten. Die Suche brauchte hier 2428 Zyklen und 50 s.

An der EXCT-30 fand sie, was keine der beiden anderen Arten fand: Mit dem 25-Hz-Shaper allein und allen Werten zugleich an ihrer ungünstigsten Grenze stand das Werkzeug beim Verweilen 0,115 mm neben seinem Ziel, bei 0,1 mm zulässig (0,043 mm mit einem Wert allein, 0,053 mm in 200 Stichproben). Ein zweiter Shaper bei 17,5 Hz, der Spitze im Frequenzgang des Werkzeugs, bringt es auf 0,095 mm (der Kommentar zu `profile.shaper2_frequency_hz` in der Datei nennt, was sonst versucht wurde). Der Riemen behält im schlaffsten Trum 87 N, und die Wellen tragen 1231 N von 1500 N an der unteren Grenze ihres Grenzwerts. Die Lager der Scheibenwellen halten an ihrer ungünstigsten Ecke 10300 h.

`--worst` ohne `--bands` oder mit `--samples` oder `--seed` wird abgelehnt (Code 2). Die App hat es als „ungünstigste Ecke“ neben den Fehlerbändern.

Grenzen bei dieser Aufgabe:

- **`--bands` allein ändert immer nur einen Wert.** Liegen zwei Werte gleichzeitig daneben, kann ein Ergebnis sein Band verlassen. `--samples` zeigt das.
- **Stichproben finden die ungünstigste Ecke nicht.** Eine Stichprobe liegt nie genau auf einer Grenze (das tut `--bands` allein), und einige Dutzend Läufe finden die ungünstigste Kombination vieler Werte nicht. `--worst` sucht sie, aber als Suche, nicht als Beweis: Ein ungünstigster Fall, den keine Kette einzelner Wechsel von der Ecke der Vorzeichen aus erreicht, oder einer zwischen den Grenzen (eine Steifigkeit, die eine Resonanz auf den Zyklus legt), kann verfehlt werden. Lassen Sie alle drei laufen.
- Die Grenzen sind selbst Schätzungen. In den Standardkonfigurationen haben nur Reibung, Dämpfung, Steifigkeit, Vorspannung, Welligkeit und Getriebewirkungsgrad Grenzen, nicht aber Massen, Trägheiten und Spiel.
- Wo eine Grenze einen Grenzwert der Bauteile ändert (die Konfiguration der EXCT-30 gibt ihren Zwischenkreis-Grenzwerten Grenzen), zeigt die Spalte `limit` weiterhin den Nennwert.
- Eine Grenze für einen Schlüssel, den die Konfiguration nicht hat, wird abgelehnt. Ebenso ein Paar, dessen unterer Wert über dem oberen liegt.

## Einen Zyklus verkürzen

```text
$ sim-cli optimise t-gantry --config crates/machine-t-gantry/exct-30.toml
setting,before,after
cycle_s,2.0480,1.4920
dwell_error_peak_m,3.947e-5,9.462e-5
axis.x.max_acceleration_m_s2,25.0,53.99999999999999
axis.x.max_jerk_m_s3,150.0,337.5
axis.x.max_velocity_m_s,2.5,3.0
axis.z.max_acceleration_m_s2,15.0,32.4
axis.z.max_jerk_m_s3,500.0,1125.0
axis.z.max_velocity_m_s,1.5,1.7999999999999998
effects.input_shaping,true,false
```

Der Optimierer probiert das Überschleifen und höhere Grenzwerte für Geschwindigkeit, Beschleunigung und Ruck (alle Achsen gemeinsam, in Schritten von ×1,5 und ×1,2). Eine Änderung bleibt, wenn der Zyklus kürzer wird und weiterhin jede Prüfung besteht. Dazu gehört `settling`: Eine schnellere Bewegung, nach der das Werkzeug an der Greifposition noch schwingt, zählt nicht. Input Shaping allein macht keinen Zyklus kürzer, weil jeder Halt eine Periode des Shapers wartet. Es wird deshalb dort probiert, wo es helfen kann: Eine schnellere Einstellung, die nur an `settling` scheitert, wird mit dem Shaper noch einmal gefahren (und mit `effects.shaper_follows_pose`, wo die Maschine eine Resonanz meldet) und übernommen, wenn sie dann besteht und kürzer ist; eine, die mit dem Shaper noch allein an `settling` scheitert, wird mit einem zweiten Shaper noch einmal gefahren (siehe [Regler einstellen und Input Shaping nutzen](#regler-einstellen-und-input-shaping-nutzen)). Ist der Shaper schon eingeschaltet, wird auch das Abschalten probiert. Die ersten beiden Zeilen sind die Zykluszeit und der Abstand vom Ziel beim Verweilen, vorher und nachher. Die übrigen Zeilen nennen, was in der Konfiguration zu ändern ist. Es wird keine Konfigurationsdatei geändert.

In der Konfiguration der EXCT-30 sind das Überschleifen und der Shaper schon eingeschaltet. Der Optimierer erhöht hier deshalb die Grenzwerte und schaltet den Shaper ab (beide Shaper): Der Zyklus fällt von 2,05 s auf 1,49 s, und das Werkzeug steht beim Verweilen 0,095 mm neben seinem Ziel statt 0,039 mm.

`--runs N` begrenzt die Zahl der simulierten Zyklen (Standard 40).

**Hubhöhen und gerundete Hübe.** Auch die Geometrie des Zyklus ist eine Variable. Ein Heben, Verfahren und Senken (gerade nach oben von der Aufnahme, auf dieser Höhe verfahren, gerade nach unten zur Ablage; das Standard-T-Portal hat zwei) wird zuerst so niedrig probiert, wie `profile.min_clearance_m` es zulässt: so weit über der höheren von Aufnahme und Ablage, etwa die Höhe eines Hindernisses dazwischen. Die mitgelieferten T-Portal-Konfigurationen setzen den Wert auf ihre Hübe, dort ändert sich also nichts. Mit einem Hindernis von 0,1 m:

```text
$ sim-cli optimise t-gantry --set profile.min_clearance_m=0.1
setting,before,after
cycle_s,6.0800,3.3640
...
cycle.segment[1-2-3].lift_m,0.3500,0.1500
cycle.segment[4-5-0].lift_m,0.3500,0.1500
```

Das Absenken allein spart 2,000 s: vier Z-Bewegungen um 0,2 m kürzer bei 0,4 m/s (`the_optimiser_lowers_the_lifts_to_the_clearance_and_saves_the_hand_calculated_time`). Die Abschnitte zählen ab 0. Zuletzt, wenn sonst nichts mehr hilft, probiert der Optimierer auch, die beiden Ecken eines Hubs zu runden: ein Abschnitt, der gerade nach oben, auf einem Spline durch einen Zwischenpunkt und gerade nach unten fährt (`via` in [`[[cycle.segment]]`](../config-reference.md#cyclesegment)), getaktet von der Bahnplanung, solange die Bahn den Abstand hält; eine solche Zeile lautet `cycle.segment[1-2-3].via_m,,0.5000 0.3000`. Am Standard-T-Portal hat das nie gewonnen: Ein Bogen ist langsamer als zwei Halte mit Überschleifen. In der App schreibt „Diese Einstellungen in die laufende Maschine übernehmen“ den neuen Zyklus in die Konfiguration (beim Speichern wird die Datei dann ohne ihre Kommentare geschrieben).

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

Ein Kreisbogen, dessen Ende der Endpunkt des Stücks vor ihm ist (dieselben Zahlen, auf einen Nanometer), ist ein Vollkreis. Jeder andere Kreisbogen dreht von diesem Punkt bis zu seinem eigenen Ende, wie kurz auch immer: Ein Bogen von 80 µm ist ein Bogen von 80 µm. Das angegebene Ende muss auf dem Kreis um den Mittelpunkt durch den Endpunkt des Stücks davor liegen, auf 10 µm genau. Der Kreisbogen wird von dort gelegt, wo die Bahn wirklich steht, und behält den Radius, den er dort hat. Er endet also um den Unterschied der beiden Radien neben seinem angegebenen Ende. Über Kreisbögen, die um denselben Mittelpunkt aufeinander folgen, summieren sich diese Abstände, wenn jedes angegebene Ende weiter außen liegt als das vorige: Vier Kreisbögen, jeder 9 µm weiter außen, enden 36 µm neben dem letzten angegebenen Ende. Endet die Bahn neben einem angegebenen Ende, ist der größte dieser Abstände Teil der Abweichung, die der Planer meldet (`Die geplante Bahn liegt ...`, siehe unten, und „Geplante Bahn neben der vorgegebenen“ in der App). Die Befehlszeile meldet sie ab 0,05 µm, dem Kleinsten, das sie anzeigen kann.

Das erste Stück hat keinen Punkt vor sich. Eine Bahn beginnt am Sollwert der stehenden Maschine, den Riemendehnung und Getriebeverdrehung bis zu 0,1 mm neben die Startpose legen (einige hundertstel Millimeter beim voreingestellten Portal). Ein Kreisbogen als erstes Stück wird deshalb an dem Ort gemessen, an dem die Maschine steht: Sein Ende darf 0,1 mm neben dem Kreis durch diesen Ort liegen, und es muss mindestens 1 mm von diesem Ort entfernt sein. Näher lässt sich ein Vollkreis nicht von einem kurzen Kreisbogen unterscheiden; deshalb wird keiner von beiden geraten und die Bahn abgelehnt (`Die Bahn beginnt mit einem Kreisbogen, der näher als 1 mm an dem Ort endet, an dem die Maschine steht. ...`). Soll eine Bahn mit einem Vollkreis oder mit einem kürzeren Kreisbogen beginnen, geben Sie den Anfang zuerst mit einer `line` zur Startpose an; der Kreisbogen wird dann an diesem Punkt gemessen. Kreisbögen brauchen eine Maschine mit zwei Achsen.

Ein Spline läuft durch alle seine Stützpunkte, ohne Sprung in Position, Richtung oder Krümmung an einem Stützpunkt (ein kubischer Spline über dem geraden Abstand zwischen den Punkten). Er verlässt die Gerade oder den Kreisbogen vor ihm in der Richtung dieses Stücks und endet in der Richtung, mit der die Gerade oder der Kreisbogen nach ihm beginnt. An einem Ende der Bahn, oder wo keine Gerade und kein Kreisbogen anschließt, endet er ohne Krümmung. Splines brauchen wie Kreisbögen eine Maschine mit zwei Achsen. Stützpunkte und Richtungen, bei denen die Kurve auf der Stelle umkehren müsste, werden abgelehnt (`Die Bahn lässt sich nicht auslegen: ...`).

Ein Spline kann über seine Stützpunkte hinausschwingen: Zwischen ihnen baucht die Kurve nach außen aus, umso mehr, je schärfer sie abbiegen muss. Eine Kurve, die den Verfahrbereich verlässt, obwohl alle Stützpunkte darin liegen, wird abgelehnt (`Die Bahn verlässt den Verfahrbereich. ...`). Abhilfe: die Stützpunkte nach innen rücken oder dort, wo sie ausbaucht, einen ergänzen.

```text
$ sim-cli analyze t-gantry --path corner.path
motor,cycle_s,peak_torque_nm,...,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,...
a,2.128000,0.547780,...,1.579e-4,5.575e-5,1.679e-4,0.000e0,...

$ sim-cli analyze t-gantry --path corner.path --time-optimal
a,1.729000,0.639460,...,1.915e-4,6.738e-5,1.915e-4,0.000e0,...
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
| Wo ein Spline auf eine Gerade oder einen Kreisbogen trifft | Die Krümmung springt; er bremst ab wie von einer Geraden in einen Kreisbogen. Mit `profile.path_tolerance_m` wird das Ende des Splines auf die Krümmung seines Nachbarn ausgeglichen, und der Übergang wird zügig durchfahren (unten) | Ebenso |
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
a,1.890000,0.547780,...,1.120e-4,...
```

1,89 s statt 2,13 s, und jede Achse bleibt innerhalb ihrer Grenzwerte für Geschwindigkeit, Beschleunigung und Ruck. Die Regeln:

| | |
| --- | --- |
| Abweichung eines Kreisbogens mit Radius r bei Übergangsbögen der Länge L | e = L² / (24 r). Um so viel rückt der Kreisbogen nach innen; auf den Übergangsbögen liegt die Bahn näher an der vorgegebenen |
| Länge der Übergangsbögen | L = v³ dk / (0,4 j): was der schnellste Übergang der Bahn braucht, um mit der Geschwindigkeit v seines langsameren Stücks und 0,4 des kleinsten Ruckgrenzwerts j durchfahren zu werden. dk ist der Sprung der Krümmung, 1 / r von einer Geraden in einen Kreisbogen. Aber nicht länger, als die zulässige Abweichung am engsten Kreisbogen erlaubt: L = Wurzel(24 · Abweichung · r_min) |
| Geschwindigkeit auf einem Übergangsbogen, der kürzer ist als nötig | v = (0,4 j L / dk)^(1/3) |
| Beispiel oben (r = 0,1 m, 100 m/s³, Kreisbogen mit 0,4 m/s) | Nötig 16 mm, erlaubt Wurzel(24 · 0,0001 · 0,1) = 15,5 mm: Der Kreisbogen rückt um die vollen 0,1 mm nach innen und wird mit 0,396 m/s statt 0,071 m/s angefahren. Mit 1 mm zulässiger Abweichung sind die Übergangsbögen die 16 mm lang, die sie brauchen, und die Bahn weicht um 0,107 mm ab |

Eine Länge gilt für alle Übergangsbögen einer Bahn. Ein Kreisbogen an einer Ecke, am Anfang oder Ende der Bahn oder neben einem Spline muss dort enden, wo die Datei es sagt. Trifft sein anderes Ende auf eine Gerade, bekommt er dort trotzdem seinen Übergangsbogen: Sein Kreis wird kleiner und rückt zum festen Ende hin, sodass er dieses Ende in derselben Richtung durchläuft und an der Geraden Platz für den Übergangsbogen lässt (für Kreisbögen bis zu einem halben Umlauf). Ein an beiden Enden fester Kreisbogen behält seinen Radius. Ist ein Stück zu kurz für seine Übergangsbögen oder würde ein Übergangsbogen einen Halbkreis oder mehr drehen (eine zulässige Abweichung, die gegen den Radius eines engen Kreisbogens groß ist), wird die Länge für die ganze Bahn halbiert, bis zu achtmal; danach wird die Bahn wie vorgegeben geplant. Mit einer zulässigen Abweichung von 0 (dem Standard) ändert sich nichts.

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
a,2.101000,0.547145,...,1.646e-4,5.616e-5,1.679e-4,0.000e0,...

$ sim-cli analyze t-gantry --path curve.path --time-optimal
a,1.974000,0.586259,...,1.722e-4,6.077e-5,1.722e-4,0.000e0,...
```

2,101 s innerhalb der Achsgrenzwerte und 1,974 s so schnell, wie die Motoren es erlauben, beide mit 0,3 s Ruhe. Die Kurve verlässt die erste Gerade waagerecht und kommt auf der letzten senkrecht nach unten an. Dabei hat sie die Krümmung, die ihre Stützpunkte verlangen, und bremst an beiden Übergängen ab wie in einen Kreisbogen. Mit `--set profile.path_tolerance_m=0.0001` werden die Enden des Splines ausgeglichen: Nahe jedem Übergang wird der Spline so gebogen, dass seine Krümmung dort die der Geraden ist (0), über eine Länge l = sqrt(Abweichung / (0,01728 dk)) für einen Krümmungssprung dk, höchstens den halben Abschnitt und nur so lang, wie die Geschwindigkeit es braucht. Die Stützpunkte bleiben, wo sie sind; der Spline weicht um höchstens die zulässige Abweichung ab. Hier: 1,985 s innerhalb der Achsgrenzwerte und 1,809 s so schnell, wie die Motoren es erlauben. Der ausgeglichene Teil ändert seine Krümmung etwa neunmal so schnell wie eine Klothoide gleicher Länge und wird deshalb langsamer durchfahren als ein Übergangsbogen zwischen Gerade und Kreisbogen (siehe die [Modellreferenz](../model-reference.md#transition-curves-profilepath_tolerance_m)).

Andere Wege, eine Bahn vorzugeben:

- `--trajectory FILE`: Posen zu Zeitpunkten, als CSV mit der Zeit in Sekunden und einer Position je Achse pro Zeile. Zeilen, die nicht mit einer Zahl beginnen, werden übersprungen. Die erste Zeit muss 0 sein, und die erste Pose muss dort liegen, wo die Maschine steht (auf 0,1 mm genau); die Zeiten müssen steigen; jede Pose muss im Verfahrbereich liegen (bis 0,1 mm darüber hinaus wird angenommen, siehe die Grenzen unten); die Bahn endet im Stillstand. Die Steuerung legt eine kubische Kurve durch die Punkte.
- `sim-cli run MACHINE --path FILE` oder `--trajectory FILE` schreibt alle Signale statt der Analyse.

Grenzen bei dieser Aufgabe:

- Ohne zulässige Abweichung ist eine verrundete Ecke mit dem Standardplaner langsam: 0,07 m/s in einen Kreisbogen mit 0,1 m Radius bei 100 m/s³.
- Splines: Ohne zulässige Abweichung springt die Krümmung, wo einer an eine Gerade oder einen Kreisbogen anschließt; die Bahn durchfährt diesen Übergang deshalb langsam, wie von einer Geraden in einen Kreisbogen. Mit einer zulässigen Abweichung wird der ausgeglichene Übergang zügig durchfahren, aber langsamer, als eine Klothoide bei gleicher Abweichung erlaubte. Ein Abschnitt hat einen Grenzwert, nach seiner engsten Stelle; ein Abschnitt mit einer engen Biegung ist also auf ganzer Länge langsam.
- Übergangsbögen zwischen Geraden und Kreisbögen: eine Länge für die ganze Bahn, sodass ein Kreisbogen, dessen Übergänge weniger bräuchten, trotzdem um das einrückt, was diese Länge verlangt. Keiner an einem an beiden Enden festen Kreisbogen (zwischen zwei Ecken oder einer Ecke und einem Spline), an einem an einem Ende festen mit einem Kreisbogen am anderen, oder an einem, der mehr als einen halben Umlauf dreht. Ihre Länge folgt den Geschwindigkeiten, die der Plan erlaubt: den Achsgrenzwerten oder bei `--time-optimal` den Motoren (das Beispiel braucht mit 0,1 mm zulässiger Abweichung 1,499 s, ohne 1,729 s).
- Der zeitoptimale Plan kennt die Zuggrenzen des Riemens (`ratings.belt_max_tension_n`, `belt_min_tension_n`): Er hält die Zugkraft jedes Abschnitts im starren Modell innerhalb von `profile.tension_reserve` (0,8) des Spielraums bis dorthin; der Rest bleibt für Reibung und Riemenschwingung (siehe die [Modellreferenz](../model-reference.md#as-fast-as-the-motors-allow---time-optimal)). Die Momentgrenzen von Getriebe und Riemenscheibe, die Führungen und den Zwischenkreis kennt er nicht. Lassen Sie `analyze` oder `verdicts` darüber laufen.
- Der zeitoptimale Plan ist nicht der schnellste mögliche. Er hat je Bahnstück einen Grenzwert für Geschwindigkeit und Beschleunigung, und auf einem Kreisbogen teilt er die Grenzen der Motoren auf die Richtungen längs der Bahn und zum Mittelpunkt auf, was einen Teil ungenutzt lässt. Im vollständigen Modell überschreiten die Motoren die Bemessungsdrehzahl um das, was die Regelkreise hinzufügen (0,1 % im Beispiel).
- Eine Maschine, die am Ende eines Hubs steht, hat ihren Sollwert einige hundertstel Millimeter dahinter: Der Sollwert liegt dort, wo die Motoren stehen, unter Riemendehnung und Getriebeverdrehung. Eine Bahn von dort beginnt so weit außerhalb des Verfahrbereichs und wird gefahren. Posen, die mehr als 0,1 mm außerhalb liegen, werden abgelehnt (`Die Bahn verlässt den Verfahrbereich. ...` bei einer Bahn aus Stücken, `Die zeitlich vorgegebene Bahn braucht mindestens zwei Punkte im Verfahrbereich, mit steigenden Zeiten ab 0.` bei Posen mit Zeiten).
- Die Nutzlast ist so, wie die Maschine sie beim Planen hat (angehängt, am Start).
- `verdicts`, `optimise`, `search` und `sensitivity` arbeiten nur mit dem Zyklus, nicht mit einer Bahn.
- In der App stehen Bahnen unter „Bahn“ (siehe [Die App bedienen](#die-app-bedienen)); eine SPS sendet sie über OPC UA (siehe [Eine SPS über OPC UA anbinden](#eine-sps-über-opc-ua-anbinden)).

## Regler einstellen und Input Shaping nutzen

Die Verstärkungen stehen in `[drive]`: `kv_1_s` (Lageverstärkung Kv), `velocity_kp_nm_s_rad` und `velocity_tn_s` (Drehzahlregler: Verstärkung Kp und Nachstellzeit Tn), `current_kp_v_a` und `current_tn_s` (Stromregler). Die drei folgenden Befehle geben ein Testsignal nur auf den Lagesollwert, ohne Vorsteuerung. Sie zeigen also die Regelkreise selbst.

**Eine Zeile je Achse:**

```text
$ sim-cli tuning t-gantry
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.02875,0.0644,0.44750,10.649
z,0.04325,0.0000,0.08550,9.200

$ sim-cli tuning t-gantry --set drive.kv_1_s=150
axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz
x,0.01150,0.3004,0.49875,22.161
z,0.00575,0.1603,0.04175,59.778
```

| Spalte | Einheit | Bedeutung |
| --- | --- | --- |
| `rise_time_s` | s | Anstiegszeit: von 10 % bis 90 % eines Sprungs von 1 mm |
| `overshoot` | Anteil | Überschwingen: größter Ausschlag über den Sprung hinaus, als Anteil des Sprungs |
| `settling_time_s` | s | Einschwingzeit: Danach bleibt das Werkzeug innerhalb von 2 % des Sprungs |
| `bandwidth_hz` | Hz | Bandbreite: Frequenz, bei der der Betrag des Führungsfrequenzgangs erstmals unter 1/√2 fällt. Leer, wenn das bis 200 Hz nie geschieht |

Mit Kv = 50 1/s verhält sich der Lageregelkreis wie ein Verzögerungsglied erster Ordnung mit der Zeitkonstante 1/Kv: Bandbreite etwa Kv / 2π = 8 Hz. Z zeigt das deutlich. In X enthält die Werkzeugposition das Schwanken des Gestells: Der Sprung regt die schwach gedämpfte Eigenschwingung des Gestells (etwa 21 Hz) an. Daher kommen die 6 % Überschwingen und die 0,45 s, bis das Werkzeug innerhalb von 2 % bleibt. Mit dem dreifachen Kv schwingt X um 30 % über.

**Stabilitätsreserven der Drehzahlregelkreise.** `margins` misst den Drehzahlregelkreis jedes Motors aufgeschnitten: Zu dem Moment, das der Regler fordert, kommt ein sinusförmiges Moment hinzu, während die Maschine ihre Startpose hält, und die Antwort des Reglers ergibt den Frequenzgang des offenen Kreises L bei 60 Frequenzen von 10 Hz bis 2 kHz (so arbeitet auch die Messfunktion eines Antriebs):

```text
$ sim-cli margins t-gantry
run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,result
,a,188.1,56.8,9.05,1.897,277.3,ok
,b,182.1,54.0,8.95,1.895,277.3,ok

$ sim-cli margins single-axis --bands
run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,result
,m,185.6,57.9,11.15,1.665,253.5,ok
belt.damping_n_s_m=50.0,m,190.6,52.1,11.18,1.717,231.7,ok
belt.damping_n_s_m=200.0,m,178.8,65.1,10.97,1.664,253.5,ok
...
```

| Spalte | Einheit | Bedeutung |
| --- | --- | --- |
| `run` | | Leer für die Konfiguration, wie sie ist; mit `--bands` eine Zeile je Motor für jeden Wert aus `[bounds]` an seiner unteren und an seiner oberen Grenze |
| `crossover_hz` | Hz | Durchtrittsfrequenz, bei der \|L\| = 1 ist, die mit der kleinsten Phasenreserve |
| `phase_margin_deg` | ° | Phasenreserve: 180° plus die Phase von L dort (die Phase ohne Sprünge verfolgt, am ersten Durchtritt zwischen −360° und 0° genommen); negativ: der Kreis ist instabil |
| `gain_margin_db` | dB | Amplitudenreserve: um wie viel die Verstärkung steigen darf, bis L −1 erreicht, an der ungünstigsten Stelle, an der die Phase durch −180° fällt und die kein späteres Steigen durch −180° mit \|L\| > 1 aufhebt (eine Nyquist-Zählung; Fallen und Steigen jenseits von −1 unterhalb des Durchtritts ist bedingt stabil und zählt nicht); negativ, wo dort \|L\| > 1 ist (instabil); `inf`, wenn es bis 2 kHz keine solche Stelle gibt |
| `peak_sensitivity` | | Das größte \|1 / (1 + L)\|: 1 durch den kürzesten Abstand von L zu −1. Bis 2 heißt mindestens 6 dB und 29° |
| `peak_frequency_hz` | Hz | Wo es am größten ist |
| `result` | | `ok`, oder `FAIL`, wo eine Reserve negativ ist oder die Messung nicht gültig: Der Kreis lief dabei in seine Momentengrenze oder bewegte sich von selbst (schwang). Dann ist der Kreis instabil, und seine Reserven sind `NaN` |

Die Drehzahlverstärkungen der Standardkonfigurationen wurden damit gewählt: mindestens 35° Phasenreserve und 6 dB Amplitudenreserve in jedem Lauf von `--bands`, auch mit `--set effects.motor_friction=false --set effects.guide_friction=false` (die Reibung dämpft den Kreis nahe dem Stillstand, wie bei der Messung, die die Maschine hält, aber nicht bei Konstantfahrt). Ergebnis: T-Portal 0,04 N m s/rad, mindestens 47,4° und 8,2 dB, ohne Reibung 39,1° und 7,3 dB; Einzelachse 0,025, mindestens 48,6° und 10,4 dB, ohne Reibung 37,8° und 9,6 dB. Die Verstärkung von 0,08, die die Konfigurationen vorher hatten, ließ dem T-Portal 12° und 2 dB (Motor B pendelte bei Konstantfahrt im Getriebespiel) und der Einzelachse 1° und 0,3 dB (sie wurde mit dem Riemen an seiner steifsten Grenze instabil). Die EXCT-30 wurde ebenso eingestellt, von 0,6 (34,9° und 6,6 dB, ohne Reibung 29,6° und 5,8 dB) auf 0,45: mindestens 47,3° und 9,1 dB, ohne Reibung 38,6° und 8,3 dB (die Zahlen für die verwendeten Verstärkungen so gemessen wie seit dem 13. November, mit stetig verfolgter Phase; die für die früheren Verstärkungen von vorher).

**Die Sprungantwort als Kurve:**

```text
$ sim-cli step t-gantry --axis 0
time_s,setpoint,response
0.000000,1.000000000e-3,0.000000000e0
0.000250,1.000000000e-3,-3.611875521e-10
0.000500,1.000000000e-3,2.716861028e-9
...
```

`--axis N` (0 ist die erste Achse), `--size M` (Standard 0.001 m), `--time SECONDS` (Standard 0.5). `response` ist die Werkzeugposition bezogen auf die Stelle, an der das Werkzeug stand, mit vier Werten je Millisekunde.

**Der Führungsfrequenzgang:**

```text
$ sim-cli bode t-gantry --axis 0 --count 8
frequency_hz,gain,gain_db,phase_deg
1.0000,9.820613e-1,-0.157,-10.626
2.1317,9.730009e-1,-0.238,-19.226
4.5440,8.970231e-1,-0.944,-36.183
9.6863,7.278681e-1,-2.759,-62.980
20.6478,7.982035e-1,-1.958,160.711
...
```

`gain` ist die Amplitude des Werkzeugs geteilt durch die Amplitude des Sollwerts. Die Phase, die bei 20,6 Hz über 160° hinaus dreht, ist die Eigenschwingung des Gestells, die in der Werkzeugposition enthalten ist. Optionen: `--from HZ` (1), `--to HZ` (200), `--count N` (40), `--amplitude M` (0.0005 m).

**Die Mechanik ohne Regelung:**

```text
$ sim-cli sweep single-axis --from 20 --to 200 --count 12
frequency_hz,gain_rad_s_per_nm,phase_deg
20.0000,5.522398e1,-82.358
...
131.5866,9.881837e1,-60.793
```

Ein sinusförmiges Moment an einem Motor, zusätzlich zu den Haltemomenten, gemessen an der Drehzahl dieses Motors. Riemenresonanzen zeigen sich als Spitzen; die Last, die gegen den gehaltenen Motor schwingt, zeigt sich als Einbruch. Optionen: `--motor N` (Index, 0 ist der erste), `--from` (5), `--to` (500), `--count` (120), `--amplitude NM` (1,5 N m oder 40 % des Spitzenmoments des Antriebs, wo das weniger ist). Frequenzen müssen über null liegen: `--from 0` wird abgelehnt (`Konfigurationswert liegt außerhalb des gültigen Bereichs: frequency`). Bei eingeschalteter Reibung misst eine kleine Amplitude vor allem die Haftreibung: Bei 0,05 N m zeigt keine der Standardmaschinen einen Einbruch. Die voreingestellte Amplitude findet ihn an jedem Motor beider Standardkonfigurationen und von `exct-30.toml` (21,6 Hz am T-Portal, 20,8 Hz an der EXCT-30, 84,3 Hz an der Einzelachse). Eine größere ist nicht besser: Bei 2 N m zeigt die Einzelachse einen zweiten, falschen Einbruch bei 62 Hz. Die Zeilen dürfen in beliebiger Reihenfolge der Frequenz stehen, und `--from` darf über `--to` liegen.

**Die Bandsperre:** `--set effects.notch_filter=true`, mit `drive.notch_frequency_hz` und `drive.notch_bandwidth_hz`. Sie wirkt auf den Momentensollwert.

**Input Shaping.** Ein ZVD-Shaper verteilt jeden Sollwert auf drei Stufen im Abstand einer halben Schwingungsperiode, sodass sich die Schwingungen, die sie anregen, gegenseitig auslöschen:

```text
--set effects.input_shaping=true --set profile.shaper_frequency_hz=25 --set profile.shaper_damping=0.03
```

Stellen Sie die Frequenz auf die Resonanz des Werkzeugs ein. Am T-Portal ändert sie sich mit der Z-Höhe. Mit `--set effects.shaper_follows_pose=true` (Input Shaper folgt der Pose) übernimmt der Shaper die Frequenz deshalb von der Maschine an der geplanten Pose. Die Maschine meldet die tiefere der beiden Eigenschwingungen Schlitten am Riemen und Z-Achse an der Führung (24,6 Hz bei z = 0,05 m und 35,6 Hz bei z = 0,15 m für die Standardkonfiguration bei x = 0,9 m). Die Einzelachse meldet keine Resonanz; dort bleibt der Shaper bei der festen Frequenz.

Was es am Zyklus der EXCT-30 bewirkt. In der Konfiguration ist der Shaper eingeschaltet, mit festen 25 Hz und einem zweiten bei 17,5 Hz; der erste Lauf schaltet beide ab:

```text
$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml --set effects.input_shaping=false
motor,cycle_s,...,path_error_peak_m,...,dwell_error_peak_m
a,1.852000,...,6.167e-4,...,5.825e-5

$ sim-cli analyze t-gantry --config crates/machine-t-gantry/exct-30.toml
a,2.048000,...,5.378e-4,...,3.947e-5
```

Mit den Shapern steht das Werkzeug beim Verweilen 0,039 mm neben seinem Ziel statt 0,058 mm, und der Zyklus dauert 2,048 s statt 1,852 s.

Das Nachschwingen des Werkzeugs nach einer Verfahrbewegung ist das Signal `axis.x.tool_deflection` aus `run`.

**Eine zweite Resonanz.** `--set profile.shaper2_frequency_hz=62` fügt einen zweiten ZVD-Shaper hinzu (Dämpfung `profile.shaper2_damping`), mit dem ersten gefaltet: neun Kopien des Plans statt drei, und jede Bewegung endet um beide Perioden später. Ein Shaper bei f_s lässt von einer ungedämpften Eigenschwingung bei f den Anteil cos²(π f / (2 f_s)) übrig: einer bei 25 Hz lässt 53 % einer Eigenschwingung bei 62 Hz, beide zusammen lassen von keiner etwas (der Test `a_second_shaper_suppresses_a_second_resonance_by_the_zvd_formula`). 0, wie in den beiden Standardkonfigurationen, heißt kein zweiter Shaper; die EXCT-30 hat einen bei 17,5 Hz. Er bleibt bei seiner Frequenz, wenn der erste der Pose folgt. `optimise` probiert ihn dort, wo eine schnellere Einstellung mit dem Shaper noch allein an `settling` scheitert, bei dieser Frequenz oder, wo sie 0 ist, bei der Resonanz, die die Maschine an der Startpose meldet, wenn diese mehr als 10 % von der Frequenz des ersten Shapers abweicht.

**Feldbus-Totzeit und Jitter.** `--set effects.fieldbus_delay=true` lässt die Antriebe die Motorsollwerte `profile.fieldbus_delay_cycles` Profiltakte zu spät verwenden. `--set effects.fieldbus_jitter=true` (Feldbus-Jitter der Sollwerte) lässt jedes Telegramm 0 bis `profile.fieldbus_jitter_cycles` Takte länger brauchen, gleichverteilt und in jedem Lauf gleich (aus `drive.noise_seed`); ein verspätetes Telegramm lässt den Antrieben das vorige. Bei 1 m/s an der Einzelachse legt ein Takt Jitter den Lagesollwert des Antriebs 0 oder 0,25 rad zurück, je zur Hälfte: ein Effektivwert von 0,18 rad. Das geht mit und ohne die Totzeit.

Grenzen bei dieser Aufgabe:

- Der Sprung ist 1 mm groß. Bei eingeschalteter Reibung zeigt er auch die Haftreibung.
- Der Shaper behandelt höchstens zwei Resonanzen (`profile.shaper2_frequency_hz`, oben). Die Eigenschwingung des Gestells und die Steifigkeit der Regelkreise stecken nicht in der Frequenz, die die Maschine meldet.
- Der Sollwert jeder Bewegung endet eine Schwingungsperiode später. Wo der Zyklus verweilt oder der Greifer arbeitet, warten beide auf den geformten Sollwert. Der Zyklus oben ist deshalb 82 ms länger: zwei Verweilzeiten, je 41 ms. Segmente ohne Verweilzeit und ohne Greiferaktion laufen nach Plan weiter.
- Der größte Teil des Bahnfehlers an der EXCT-30 entsteht, weil das Gestell unter den Beschleunigungskräften nachgibt (0,62 mm Spitze ohne den Shaper, 0,57 mm mit ihm). Dagegen richtet ein Shaper, der auf die Resonanz des Werkzeugs eingestellt ist, wenig aus.
- Es wurde mit keinem realen Antrieb verglichen. Antriebshersteller dokumentieren ihre internen Filter nicht, und die Totzeiten hier sind ganze Takte. Der Feldbus kann Jitter haben (oben), die Regelkreise im Antrieb selbst haben keinen.
- Die Reserven werden mit einem Zehntel des Nennmoments gemessen. Nahe dem Stillstand dämpft die Coulomb-Reibung des Motors (über 0,5 rad/s geglättet) den Kreis bei dieser Amplitude, die Reserven sind mit Reibung also größer; für Konstantfahrt gelten die ohne Reibung.
- Eine Verstärkung, die bei den Nennwerten in Ordnung ist, kann instabil sein, wenn Rotorträgheit, Riemensteifigkeit oder Induktivität abweichen. Das trat an der Konfiguration der EXCT-30 auf; deren Drehzahlverstärkung wurde deshalb von 1,4 auf 0,6 gesenkt (und später für die Reserven oben auf 0,45). Lassen Sie nach der Reglereinstellung `sensitivity` laufen.

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
m,2.266321e-2,6.778671e-2,6.425452e-4,3.661569e-3
```

Die „gemessene“ Maschine ist hier der Simulator mit einem Schlitten, der 1,5 kg schwerer ist als in der Konfiguration. Die Tabelle nennt je Motor den Effektivwert und die Spitze der Differenz zwischen gemessenem und simuliertem Drehmoment und Schleppfehler. `--out` schreibt beides nebeneinander, eine Zeile je Zeile der Aufzeichnung:

```text
time,motor.m.torque_measured,motor.m.torque_simulated,motor.m.speed_measured,motor.m.speed_simulated,motor.m.following_error_measured,motor.m.following_error_simulated
```

**Anpassung, Stufe 1: Starrkörperwerte aus einer Aufzeichnung.**

```text
$ sim-cli fit single-axis --trace measured.csv --set effects.payload=false
quantity,of,config,fitted,standard_error
moved_mass_kg,x,6.549761e0,8.067588e0,1.618e-2
viscous_friction_n_s_m,x,2.681992e1,2.621072e1,2.192e-1
coulomb_friction_n,x,2.131795e1,2.196390e1,1.953e-1
constant_force_n,x,-2.163662e-4,2.050984e-3,6.293e-2
Bewegte Masse, Achse x: +1.518 kg ± 0.016. Um diesen Betrag sind die mit der Achse bewegten Massen zu ändern. Lastmasse und die Trägheit von Rotor und Scheibe kann die Aufzeichnung nicht trennen.
Viskose Reibung, Achse x: × 0.977 ± 0.008. Die viskose Reibung der Führung der Achse und der Motorwellen ist mit diesem Faktor zu multiplizieren; trennen kann die Aufzeichnung beide nicht.
Coulomb-Reibung, Achse x: × 1.030 ± 0.009. Coulomb- und Haftreibung der Führung der Achse und der Motorwellen sind mit diesem Faktor zu multiplizieren; trennen kann die Aufzeichnung beide nicht.
Konstante Kraft, Achse x: +0.002 N ± 0.063. So viel mehr braucht die gemessene Achse als die Konfiguration: ein Gewicht, eine Neigung oder ein Kabelzug, die der Konfiguration fehlen.
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

Die Datei mit dem Frequenzgang braucht die Spalten `frequency_hz` und `gain_rad_s_per_nm` (Motormoment zu Motordrehzahl). `fit` misst denselben Frequenzgang an der Konfiguration, bei den Frequenzen der Datei, am Motor `--sweep-motor N` (Standard 0) mit `--amplitude NM` (Standard wie bei `sweep`), und vergleicht, wo der Einbruch liegt. Hat der Frequenzgang mehr als einen Einbruch (das T-Portal hat drei, bei etwa 22, 62 und 107 Hz), wird in beiden der mit der niedrigsten Frequenz verglichen. Der „gemessene“ Riemen hat hier das 0,7-Fache der Steifigkeit; die Anpassung ergibt 0,734 ± 0,026.

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
- Ein Frequenzgang mit kleiner Amplitude zeigt bei eingeschalteter Reibung keinen Einbruch. Ein Frequenzgang mit 0,05 N m ergibt `Keine Steifigkeit angepasst: Der gemessene Frequenzgang oder der der Konfiguration zeigt keinen Einbruch mit beiden Flanken im gemessenen Bereich.` Das ist ein Fehler wie jeder andere: `fit` endet mit dem Code 2, nachdem es geschrieben hat, was eine Aufzeichnung ergab. Der Frequenzgang muss außerdem den Einbruch und seine beiden Flanken enthalten. Seine Frequenzen müssen über null liegen.
- All das wurde an Aufzeichnungen geprüft, die der Simulator selbst erzeugt hat, mit zusätzlichem Rauschen. Eine Aufzeichnung eines realen Antriebs wurde nicht gelesen. Bei realen Exporten müssen die Spalten umbenannt und die Einheiten in rad, rad/s und N m an der Motorwelle umgerechnet werden.

## Sehen, wohin Moment und Energie gehen

Sechs Ansichten nehmen jeweils einen Effekt für sich heraus, statt ihn im Momentverlauf zu lassen: wofür jeder Motor sein Moment braucht, wohin die Energie eines Zyklus geht, was die Reibung jeder Führung tut, wie warm Wicklungen, Getriebe und Führungen werden, was jeder Riemenabschnitt trägt und was ein Effekt ändert, wenn man ihn ausschaltet. Wie jede Größe berechnet wird, steht in der [Modellreferenz](../model-reference.md#effect-views).

**Momentenaufteilung.** `--breakdown DATEI` schreibt je Motor Spitze und Effektivwert der vier Anteile seines Moments über den analysierten Durchlauf, und die des Moments selbst:

```text
$ cargo run -q --release -p sim-cli -- analyze t-gantry --breakdown breakdown.csv
$ cat breakdown.csv
motor,part,peak_torque_nm,rms_torque_nm
a,inertia,2.955173e-1,6.164674e-2
a,gravity,1.372931e-1,1.193030e-1
a,friction,1.403381e-1,8.273484e-2
a,rest,6.341280e-2,1.184636e-2
a,torque,5.596234e-1,1.653596e-1
b,...
```

- **inertia** (Trägheit): was jede bewegte Masse beschleunigt, und die Rotoren und Riemenscheiben, die sich mit der Last drehen.
- **gravity** (Schwerkraft): was die Last gegen ihr Gewicht hält. Beim T-Portal hält jeder Motor m_z g r / (2 i) = 0,137 N m, einer nach oben, einer nach unten.
- **friction** (Reibung): Führungen, Motorwellen und Bremsen, Getriebeverluste, Umlenkrollen, Energiekette.
- **rest** (Rest): alles andere, vor allem Riemen und Getriebe: die Last, die am Riemen hinter den Motoren zurückbleibt, Schwingen, Lose. Dazu Welligkeit und Rastmoment, Zahneingriff, Prozesskraft und Endanschläge.

In jedem Augenblick ergeben die vier Anteile zusammen das Motormoment. Ihre Spitzen und Effektivwerte tun das nicht: Sie fallen zu verschiedenen Zeiten an. Ein Rest, der gegen das Moment groß ist, heißt: Riemen oder Getriebe tragen viel; im starren Modell ist er fast null.

**Energiebilanz.** `--budget DATEI` schreibt, wohin die über den Durchlauf aus der Einspeisung bezogene Energie ging, in Joule und als Anteil daran:

```text
$ cargo run -q --release -p sim-cli -- analyze t-gantry --budget budget.csv
$ cat budget.csv
item,energy_j,share
supply,1.393495e2,1.000000
copper.a,3.287117e0,0.023589
copper.b,2.827614e0,0.020292
resistor,5.343855e0,0.038349
power.guide_x,5.101324e1,0.366081
power.guide_z,1.911054e1,0.137141
...
power.shaft_a,2.454226e1,0.176120
power.shaft_b,2.454228e1,0.176120
power.gearbox_a,4.378065e0,0.031418
power.gearbox_b,4.305584e0,0.030898
power.belt,2.860734e-3,0.000021
...
windings,4.348351e-7,0.000000
capacitors,3.313464e-5,0.000000
energy.kinetic,2.731025e-8,0.000000
energy.potential,-1.018865e-7,-0.000000
energy.elastic,-2.145527e-7,-0.000000
payload,-2.304748e-4,-0.000002
unaccounted,-8.388325e-4,-0.000006
```

| Zeile | Bedeutung |
| --- | --- |
| `supply` | Energie aus der Einspeisung. Mit `effects.dc_link` was der Gleichrichter (oder das Line Module) aus dem Netz in den Zwischenkreis bezogen hat; ohne, was die Antriebe bezogen haben, weniger was sie zurückgespeist haben |
| `copper.<motor>` | Kupferverlust jeder Wicklung |
| `resistor` | Der Bremswiderstand (mit `effects.dc_link`) |
| `mains`, `line_module` | Mit `effects.regenerative_supply`: was das Line Module ins Netz zurückgespeist hat, und was es dabei verloren hat |
| `power.<ort>` | Jeder Verlust der Maschine: Führungsreibung je Führung, Umlenkrollen, Energiekette, Wellenreibung und Bremse je Motor, Getriebe je Motor, Riemendämpfung, Dämpfung von Rahmen und Z-Führung, Endanschläge, und `power.other` für Welligkeit, Rastmoment, Zahneingriff und Prozesskraft (diese können Energie abgeben wie aufnehmen) |
| `windings`, `capacitors` | Was die Induktivität der Wicklungen und die Zwischenkreiskondensatoren am Ende mehr halten als am Anfang |
| `energy.kinetic`, `.potential`, `.elastic` | Was die Maschine am Ende des Durchlaufs mehr hält als am Anfang; bei einem wiederholten Zyklus nahe null |
| `payload` | Energie, die mit der abgesetzten Nutzlast ging, weniger der, die mit der aufgenommenen kam |
| `unaccounted` | Was die Zeilen von der Einspeisung übrig lassen. Unter 0,1 % davon auf jeder Konfiguration, auch den verlustfreien. Die elektrische Seite allein (Einspeisung gegen Kupfer, Widerstand, Netz, Line Module, Wicklungen, Kondensatoren und die Arbeit der Momente an den Wellen) wird auf ein Millionstel der Einspeisung geprüft: Der Wicklungsstrom wird zusammen mit der Mechanik integriert |

Beim Standard-T-Portal nehmen die Führungen die Hälfte, die Motorwellen ein Drittel, Getriebe, Wicklungen und Bremswiderstand je einige Prozent; die Riemendämpfung fast nichts.

**Reibung je Führung.** Die App zeichnet je Führung die Reibkraft beim stationären Gleiten über der Geschwindigkeit (Coulomb-Niveau, viskoser Anteil und der Stribeck-Anstieg zum Stillstand hin) bei der jetzigen Führungslast, und den Punkt, an dem die Führung gerade ist. Ein Punkt neben der Kurve ist eine Führung, die nicht stationär gleitet: beim Losbrechen, Umkehren oder Halten. Die Kommandozeile hat diese Ansicht nicht; `sim-cli run` schreibt dieselben Signale (`axis.<id>.friction_speed`, `_coulomb`, `_viscous`, `_stiction`, `_load_scale`).

Die Momentanteile, Verlustleistungen und Energieanteile sind ebenfalls Signale: `sim-cli run` schreibt sie, die Scopes der App können sie zeigen und eine SPS kann sie lesen.

**Temperaturen.** Die App zeichnet die Übertemperatur jeder Motorwicklung über das Zeitfenster der Oszilloskope und, mit dem Effekt „Erwärmung von Getriebe und Führung“ (`effects.part_heating`, standardmäßig aus), die jedes Getriebes und jeder Führung. Jedes davon erwärmt sich mit seinem eigenen Verlust (`power.gearbox_*`, `power.guide_*`) über einen Wärmewiderstand, mit einer Zeitkonstante und bis zu einer zulässigen Übertemperatur; alle drei stehen in `[thermal]` der Konfiguration. Die Tabelle zeigt für jedes die Übertemperatur jetzt, ihren Endwert, wenn der analysierte Zyklus lange wiederholt wird (für ein Bauteil: Wärmewiderstand mal mittlerer Verlust), die zulässige Übertemperatur und den Anteil des Endwerts daran, über 100 % mit „(über dem Zulässigen)“ markiert. Der Anteil ist der des Endwerts, weil die Übertemperatur jetzt viele Minuten lang nur wenige Prozent davon ist; `gearbox-thermal` und `guide-thermal` prüfen denselben Endwert. Ein Direktantrieb (EXCT-30: Getriebeübersetzung 1) hat kein Getriebe, das sich erwärmt; sein `power.gearbox_*` ist die Dämpfung der Kupplung und heißt so. In der 3D-Ansicht nehmen Motorgehäuse, Getriebe, der Schlitten und (T-Portal) die Z-Achse die Farbe ihrer Übertemperatur gegenüber der zulässigen an: blau kalt, gelb bei der Hälfte, rot bei der zulässigen, magenta darüber; die Ansicht zeichnet diese Skala. Mit den Standard-Zeitkonstanten von 20 bis 40 Minuten zeigt eine Minute der Oszilloskope nur einen langsamen Anstieg; wer ein Bauteil einschwingen sehen will, verkürzt seine Zeitkonstante. Mit dem Effekt „Temperaturen wirken auf Reibung, Getriebeverluste und Riemensteifigkeit“ (`effects.thermal_feedback`, standardmäßig aus) erwärmen sich die Bauteile ebenso und wirken zurück: die viskose Reibung einer Führung folgt der Viskosität ihres Schmierstoffs (das 0,28-Fache bei 30 K über 20 °C), ein Getriebe verliert weniger, wenn sein Öl dünner wird (ein Getriebe mit 95 % hat bei 30 K 96,8 %, wenn die Hälfte seines Verlusts als viskos gilt), und die Steifigkeit des Riemens folgt ein wenig der Umgebungstemperatur. Die Werte stehen in `[thermal_feedback]`: Umgebungstemperatur, Bezugstemperatur, für die Reibung und Wirkungsgrad der Konfiguration gelten, Viskositätsverlauf des Schmierstoffs, viskoser Anteil des Getriebeverlusts und Steifigkeitsverlust des Riemens je Kelvin. Der Effekt ersetzt `effects.temperature`, dessen zwei von Hand gesetzte Faktoren nicht verwendet werden, solange er an ist. In einem Zyklus von Sekunden erwärmen sich die Bauteile mit den Standard-Zeitkonstanten von 20 bis 40 Minuten kaum; die Ergebnisse des Zyklus ändern sich daher nur, wo die Umgebungs- von der Bezugstemperatur abweicht. Wer ein Bauteil dort einschwingen sehen will, wo Verlust und Wärmeabfuhr sich ausgleichen, verkürzt seine Zeitkonstante. Die Kommandozeile schreibt die Übertemperaturen mit `run` (Signale `motor.<id>.winding_temperature`, `temperature.<bauteil>`).

**Riemenzähne und Trume.** Mit `effects.belt_tooth_jump` (standardmäßig aus) steigen die Zähne des Riemens aus den Lücken einer Motorscheibe, wenn die Zugkraft dort übersteigt, womit die Kraft im Leertrum sie hält (`belt.tooth_jump_factor` mal Zähne im Eingriff mal Teilung durch Radius, mal Kraft im Leertrum: teilen sich beide Trume die Zugkraft gleich, mit den Standardwerten etwa das 1,5-Fache der Vorspannung), und der Riemen rutscht einen Zahn weiter oder zurück. Die Signale `pulley.<motor>.tooth_jumps` zählen die Sprünge, `pulley.<motor>.tooth_slip` ist der Schlupf des Riemens über der Scheibe; `power.tooth_jump` ist, was die Sprünge verbrauchen, in der Energiebilanz. Jeder Sprung im Zyklus lässt `belt-tooth-jump` durchfallen. Mit `effects.belt_span_vibration` (standardmäßig aus) hat jedes freie Trum seine erste Quereigenfrequenz, Wurzel(Kraft / Masse je Meter) / (2 · Länge), als Signal `belt.span.<i>.frequency`, und `belt.span_detuning` gibt an, wie weit die nächstliegende davon von dem entfernt ist, was der Antrieb anregt: Zahneingriff (Riemengeschwindigkeit durch Teilung) sowie Drehmomentwelligkeit und Rastmoment der Motoren. `belt-span-resonance` fällt durch, wo ein Trum bei konstanter Geschwindigkeit der Achsen näher als `ratings.belt_span_resonance_band` (10 %) an einer Anregung läuft. Berechnet werden nur die Frequenzen, nicht wie weit ein Trum ausschwingt.

**Riemenabschnitte.** Je Abschnitt die Riemenkraft (die Vorspannung gestrichelt) und die Dehnung in mm über das Zeitfenster der Oszilloskope, und eine Tabelle beider Werte jetzt mit der zulässigen Kraft (`ratings.belt_max_tension_n`) und dem Anteil daran. Die zulässige Kraft ist nicht eingezeichnet: beim Fünffachen der Vorspannung würde sie die Linien flach drücken. Die Dehnung ist die Länge des Abschnitts weniger seines ungedehnten Riemens, ein Abschnitt bei Vorspannung ist also auch gedehnt (0,18 mm auf 0,3 m bei 300 N und EA = 5·10⁵ N). Die 3D-Ansicht färbt jeden Abschnitt nach derselben Kraft: das eigene Dunkelgrau des Riemens bei Vorspannung, darüber zu Rot hin (rot bei doppelter Vorspannung), darunter zu Blau hin (blau, wenn er schlaff ist); die Ansicht zeichnet diese Skala mit ihren Zahlen. `sim-cli run` schreibt die Signale `belt.<i>.tension` und `belt.<i>.stretch`.

**Ein Effekt ein und aus.** `compare --effect NAME` lässt den analysierten Durchlauf des Zyklus zweimal laufen, mit `effects.NAME` ein und aus, gleich wie die Konfiguration ihn hat, und schreibt die gewählten Signale nebeneinander, einmal je Millisekunde; `--differences DATEI` schreibt je Signal Spitze und Effektivwert von ein weniger aus:

```text
$ cargo run -q --release -p sim-cli -- compare t-gantry --effect guide_friction --signal motor.a.torque --signal motor.b.torque --out compare.csv --differences diff.csv
$ head -3 compare.csv
time[s],motor.a.torque.on[N m],motor.a.torque.off[N m],motor.b.torque.on[N m],motor.b.torque.off[N m]
0.0000,5.644133914e-2,1.035313264e-1,-1.450023784e-1,-1.152924340e-1
0.0010,5.731815816e-2,1.039299274e-1,-1.441924234e-1,-1.149019169e-1
$ cat diff.csv
signal,unit,peak_difference,rms_difference
motor.a.torque,N m,9.628014e-2,4.599526e-2
motor.b.torque,N m,9.224554e-2,4.627908e-2
```

Ohne `--signal` wird jedes Signal geschrieben. In der App wählt der Teil „Effekt ein und aus“ der Effekt-Ansichten den Effekt und eine Größe der Oszilloskope (Moment, Drehzahl, Schleppfehler, ...), lässt den Vergleich im Hintergrund mit der Konfiguration laufen, wie sie ist, und legt beide Läufe übereinander: ein durchgezogen, aus gestrichelt in derselben Farbe, eine Farbe je Achse oder Motor, mit einer Tabelle der Differenz. Ein Effekt, der den Zeitablauf des Zyklus ändert (Eckenverschleifen, Input Shaping), wird nur über die Zeit verglichen, die beide Durchläufe haben.

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

**Antriebe sperren.** Mit `--disable-at` geben die Antriebe ab diesem Zeitpunkt kein Moment mehr ab. Die Motor-Haltebremsen fallen ein (`effects.holding_brake`, an beiden Maschinen, im elastischen und im starren Modell):

```text
$ sim-cli run t-gantry --move 0.5,0.2 --time 1.5 --disable-at 0.5 --out disable.csv
```

In diesem Lauf steht die Z-Achse bei 0,498 s auf 0,2276 m und bewegt sich; bei 1,498 s ruht sie auf 0,2192 m. Eine Bremse, die bei Drehzahl einfällt, rutscht mit ihrem Haltemoment (`motor.brake_torque_nm`) und nicht mehr, bis die Achse steht. Mit `--set effects.holding_brake=false` fällt die Achse bis zum unteren Endanschlag (z = −0,00008 m bei 1,498 s). Eine laufende Bewegung oder ein laufender Zyklus wird verworfen und nicht fortgesetzt. Solange die Antriebe gesperrt sind, kühlen ihre Wicklungen ab (mit `effects.winding_heating`, um den Faktor e je `motor.thermal_time_constant_s`), und ein Zwischenkreis über dem Netzniveau, nach einer Abschaltung, entlädt sich über seinen Entladewiderstand mit `ratings.dc_link_discharge_time_constant_s` (130 s, eine Schätzung) bis auf das Netzniveau.

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
| `command.path` | String | schreiben | Eine Bahn im Text einer Bahndatei (siehe [Eine Bahn fahren](#eine-bahn-fahren-und-den-bahnfehler-ablesen)), ein Stück je Zeile. Zeigt, was geschrieben wurde |
| `command.path_time_optimal` | Boolean | schreiben | `true` legt die Bahn so schnell aus, wie die Motoren erlauben (wie `--time-optimal`), `false` innerhalb der Achsgrenzen (Standard) |
| `command.run_path` | Boolean | schreiben | `true` legt `command.path` von dort aus, wo die Maschine steht, und fährt sie |
| `status.busy` | Boolean | lesen | Eine Bewegung, eine Bahn oder der Zyklus läuft („In Bewegung“) |
| `status.refused` | String | lesen | Schlüssel des zuletzt abgelehnten Befehls, oder leer |

So verhalten sich die Befehle:

- Die Regelkreise der Antriebe bleiben im Simulator. Die SPS sendet Ziele oder Lagesollwerte, kein Drehmoment.
- Zahlen dürfen als Double, Float, Int16, Int32 oder Int64 geschrieben werden. Alles andere wird mit `BadTypeMismatch` beantwortet.
- Sollwerte mehrerer Achsen, die innerhalb derselben 10 ms geschrieben werden, gelten zusammen als ein Sollwert.
- Eine Achse, die nicht geschrieben wird, bleibt, wo sie ist. `command.target.<axis id>` zeigt die geplante Position, bis es geschrieben wird, und wieder, sobald die Fahrt befohlen ist, für die es geschrieben wurde. `command.setpoint.<axis id>` zeigt den Sollwert, dem die Steuerung folgt; das ist die geplante Position, solange kein Sollwert unterwegs ist. Nach einer Fahrt oder einem Zyklus schickt ein Schreiben auf eine Achse die anderen also nicht dorthin zurück, wo sie vorher standen.
- Eine Bahn wird erst geschrieben und dann gestartet: `command.path` (und `command.path_time_optimal`) zusammen mit `command.run_path` oder davor schreiben. Zusammen geschrieben, gelten `command.path` und `command.path_time_optimal` in beliebiger Reihenfolge im Schreibvorgang: gefahren wird die zuletzt geschriebene Bahn. Die Befehle eines Schreibvorgangs werden in der geschriebenen Reihenfolge ausgeführt, und die Bahn wird ausgelegt, wenn sie an der Reihe ist, von dort aus, wo die Maschine dann steht (etwa nach einem davor geschriebenen `command.enable`), genau wie `sim-cli run --path` sie auslegt, und in Echtzeit gefahren.
- `status.refused` enthält einen Schlüssel wie `error-busy`, `error-target-out-of-range`, `error-setpoint-too-far` oder `error-disabled`. Für eine Bahn: `opcua-error-path-line` (eine Zeile, die sich nicht lesen lässt), `error-path` (etwa ein Kreisbogen an einer Maschine mit einer Achse), `error-path-first-arc`, `error-path-range` (außerhalb des Hubs), `error-path-torque` (nur zeitoptimal), `error-busy`, `error-disabled`. Er bleibt stehen, bis ein Befehl derselben Art (Fahrt, Bahn, Sollwert, Zyklusstart, Halt, Sperren, Freigeben) angenommen wird. Angenommene Befehle anderer Art, etwa laufend übertragene Sollwerte, lassen ihn stehen.
- Ein laufend übertragener Sollwert wird abgelehnt, solange eine Bewegung oder der Zyklus läuft.
- Ein Sollwert, der vom letzten (oder, wenn keiner unterwegs ist, vom Standort der Achse) weiter entfernt liegt, als die Achse mit ihrer Höchstgeschwindigkeit in einem `profile.setpoint_interval_s` fährt (8 mm an der X-Achse des T-Portals), wird mit `error-setpoint-too-far` abgelehnt und nicht ausgeführt. Eine SPS, die weiter zurückliegt, holt mit einer Fahrt auf. Laufend übertragenen Sollwerten, einer je Intervall, folgt die Achse mit deren Geschwindigkeit, bis zur Höchstgeschwindigkeit; ein Sollwert, der zu früh kommt, trifft die Achse vor dem letzten an, und sie folgt ihm um so viel schneller.
- Kommt der Rechner mit der Echtzeit nicht mit, bleibt die Simulation zurück, statt Arbeit aufzustauen.

Grenzen bei dieser Aufgabe:

- **Nicht an einer realen SPS geprüft.** Der Client in den Tests ist mit derselben Bibliothek geschrieben wie der Server. Das Zusammenspiel mit Clients von Siemens, Beckhoff oder CODESYS, deren Datentypen, Sicherheitseinstellungen und Zyklus-Jitter sind ungeprüft.
- Der Server bindet sich nur an 127.0.0.1 und bietet nur die Standardkonfiguration der Maschine an. Es gibt weder `--config` noch `--set`.
- Keine Sicherheitsprofile, keine Benutzerkonten, keine Methoden, keine Konfigurationsänderungen.
- Bahnen: `status.refused` nennt den Schlüssel, nicht die Zeile, die sich nicht lesen ließ, und Posen zu Zeitpunkten (`--trajectory`) gibt es über OPC UA nicht. Eine Bahn wird innerhalb des 10-ms-Takts der Simulation ausgelegt: Eine lange zeitoptimale Bahn, deren Auslegung länger dauert, lässt die Simulation in diesem Moment hinter der Echtzeit zurückfallen.
- Der Server schreibt seine Schlüssel in einen Ordner `pki` im Arbeitsverzeichnis.

## Das Modell in Simulink / Modelica über FMI nutzen

```sh
cargo build --release -p sim-fmi                                   # die Binärdatei der FMU, target/release/sim_fmi.dll
sim-cli fmu t-gantry --config crates/machine-t-gantry/exct-30.toml --out exct30.fmu
```

`fmu` schreibt die Maschine mit ihrer Konfiguration (der Standardkonfiguration, oder `--config`, `--motor`, `--supply`, `--gearbox` und `--set` wie bei jedem Befehl) als FMU für FMI 2.0 Co-Simulation. Die FMU rechnet dieselbe Simulation wie `sim-cli run`: Antriebe, Regler, jeden eingeschalteten Effekt. Importiert wird sie in Simulink (FMU-Block), OpenModelica (`File > Import > FMU` oder `importFMU`), Dymola (`File > Import > FMU`) oder jedem anderen Werkzeug für FMI 2.0; ein Regler- oder SPS-Modell dort sendet die Lagesollwerte und liest die Signale zurück. FMI 2.0 und nicht 3.0, weil jedes dieser Werkzeuge 2.0 Co-Simulation importiert und der Simulator nichts braucht, was nur 3.0 hat.

Was in der FMU steht:

| Datei | Inhalt |
| --- | --- |
| `modelDescription.xml` | Die Variablen unten, erzeugt aus der Signalliste und der Konfiguration der Maschine, mit Einheiten und Beschreibungen in der Sprache von `--lang` |
| `resources/config.toml` | Die Konfiguration wie auf der Kommandozeile angegeben, Katalogteile eingesetzt |
| `resources/model.txt` | Maschinenkennung und Sprache |
| `binaries/win64/sim_fmi.dll` | Der Simulator. Eine Binärdatei für jede Maschine; die Ressourcen sagen, welche sie rechnet |

Ohne weitere Angabe nimmt `fmu` die neben `sim-cli` gebaute Binärdatei (`target/release/sim_fmi.dll`, unter Linux `libsim_fmi.so`); fehlt sie, bricht es ab und sagt, wie sie gebaut wird. `--binary DATEI` nennt sie stattdessen und darf mehrfach stehen: Eine `.dll` kommt nach `binaries/win64`, eine `.so` nach `binaries/linux64`, eine `.dylib` nach `binaries/darwin64` (die Endung in beliebiger Schreibung: `SIM_FMI.DLL` wird als `sim_fmi.dll` abgelegt); zwei für dieselbe Plattform werden abgelehnt. Eine FMU für Windows und Linux entsteht, indem `sim-fmi` unter Linux gebaut (`cargo build --release -p sim-fmi`) und beide Dateien übergeben werden: `--binary sim_fmi.dll --binary libsim_fmi.so`.

| Variable | Typ | Kausalität | Bedeutung |
| --- | --- | --- | --- |
| `time` | Real | independent | Simulierte Zeit, s, ab der Startzeit des Experiments |
| jedes andere Signal der [Signaltabelle](#signale-aufzeichnen-antriebe-sperren) | Real | output | Z. B. `axis.x.position`, `motor.a.torque`, `belt.1.tension`, mit seiner Einheit |
| `command.setpoint.<Achskennung>` | Real | input | Lagesollwert, wie `command.setpoint.<Achskennung>` über OPC UA. Startwert: die Startpose |
| `command.enable` | Boolean | input | `false` schaltet die Antriebe ab (Bremsen fallen ein), `true` wieder ein. Startwert `true` |
| `machine` | String | parameter | Die Maschinenkennung |
| `config_file` | String | parameter | Eine Konfigurationsdatei statt der in der FMU; leer (der Startwert) für die in der FMU |
| `effects.<Name>` | Boolean | parameter | Jeder Effektschalter, Startwert wie in der Konfiguration; ein hier gesetzter Wert gilt vor dem der Konfigurationsdatei |

Die Namen sind die Signalkennungen wie in den CSV-Spalten (Namenskonvention „flat“). Die Wertreferenz einer Variablen ist ihr Platz in der Liste.

Wie ein Schritt abläuft:

- Die FMU rechnet selbst im Grundschritt `sim.dt_s` (62,5 µs in den mitgelieferten Konfigurationen) mit ihrem eigenen Integrator. Ein Kommunikationsschritt, der kein ganzzahliges Vielfaches davon ist, wird abgelehnt (`fmi2Error`, mit Meldung), ebenso ein Schritt ab einer anderen Zeit als der, bei der die Simulation steht, eine Zeit oder Schrittweite, die keine endliche Zahl ist, und ein Schritt von mehr als 4294967295 Grundschritten (etwa 3 Tage simulierter Zeit bei 62,5 µs). `fmi2SetupExperiment` lehnt eine Startzeit ab, die keine endliche Zahl ist, ebenso eine Stoppzeit oder Toleranz, die als definiert übergeben wird. Interpoliert wird nichts. Ausgänge werden ab dem Initialisierungsmodus gelesen, wie FMI 2.0 es vorsieht; `fmi2GetReal` davor wird abgelehnt.
- Zu Beginn jedes Kommunikationsschritts geht ein geänderter Sollwerteingang als ein Sollwert an den Regler, der über `profile.setpoint_interval_s` (10 ms) dorthin interpoliert, genau wie beim Sollwert einer SPS über OPC UA. Der Kommunikationsschritt sollte deshalb dieses Intervall sein: Es ist die Schrittweite des Standardexperiments der Modellbeschreibung. Bei kürzeren Schritten beginnt jeder neue Sollwert eine neue Interpolation von dort, wo die letzte angekommen war; das folgt dem Strom mit etwas Verzug. Mit `profile.setpoint_interval_s` gleich dem Kommunikationsschritt in der Konfiguration stimmen beide überein.
- Einen Sollwert, den der Regler ablehnt (zu weit für ein Intervall, `error-setpoint-too-far`; außerhalb des Hubs), meldet die FMU als Warnung, der Schritt läuft ohne ihn, und `fmi2DoStep` gibt `fmi2Warning` zurück. Eine Achse, deren Eingang nicht gesetzt wird, bleibt, wo der Regler sie hat.
- Parameter lassen sich bis zum Ende der Initialisierung setzen. Eine Konfigurationsdatei, die die Liste der Signale oder ihre Einheiten ändert, wird am Ende der Initialisierung mit dem ersten abweichenden Signal abgelehnt: Für diese Konfiguration eine neue FMU exportieren.
- Ihre Signale sind Bit für Bit die von `sim-cli run` mit denselben Sollwerten zu denselben Zeiten (geprüft).
- Nach einer Abschaltung wegen Überspannung im Zwischenkreis mit einem Bremswiderstand, der für die Bremsenergie zu klein ist, löst ein sofort wieder auf `true` gesetztes `command.enable` erneut aus, und wieder, bis die nach der Abschaltung austrudelnde Achse steht: Ihre Bewegungsenergie fließt in den Zwischenkreis, sobald die Antriebe sie bremsen. Beobachtet wurden 7 bis 57 erneute Abschaltungen über 6 bis 60 ms, bevor der Zwischenkreis auf Netzniveau zur Ruhe kommt und die Antriebe eingeschaltet bleiben. Das ist die Physik eines zu kleinen Widerstands, kein Fehler der FMU; warten, bis die Achse steht, oder den Widerstand auslegen (`sim-cli verdicts`).

Grenzen bei dieser Aufgabe:

- **Nicht mit einem unabhängigen FMI-Werkzeug geprüft.** Hier stand keines zur Verfügung (`fmpy` ist im Projekt nicht installiert). Die Tests rufen die Funktionen der FMU aus Rust auf und vergleichen mit dem Simulator; Archiv und Modellbeschreibung wurden außerdem mit `zipfile` und dem XML-Parser von Python gelesen und die Binärdatei über `ctypes` gefahren. Der Import in Simulink, OpenModelica oder Dymola ist ungeprüft.
- Nur Co-Simulation, kein Model Exchange. Kein FMU-Zustand (`fmi2GetFMUstate` und seinesgleichen geben `fmi2Error` zurück), Werkzeuge, die einen Schritt zurücknehmen, können sie also nicht nutzen; keine Richtungsableitungen, keine asynchronen Schritte.
- Eingänge sind nur Lagesollwerte: keine Fahrziele, kein Zyklus- oder Bahnbefehl und kein Momenteingang. Die Regelkreise bleiben in der FMU.
- Meldungen und Beschreibungen in einer Sprache, der von `--lang` beim Schreiben der FMU.
- Eine 64-Bit-Binärdatei für Windows, solange keine für Linux wie oben hinzukommt; keine 32-Bit-Binärdateien.

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
| Konfigurationsdatei: Datei, Öffnen, Speichern | Ein Dateiname, eingetippt. „Öffnen“ lädt sie als Konfiguration der gezeigten Maschine, wie `--config FILE`: Sie ersetzt die laufende Konfiguration und startet die Simulation neu, wie ein Wechsel der Maschine. Eine Datei, die nicht lesbar ist oder die die Maschine ablehnt (ein falsch geschriebener Schlüssel, ein Wert außerhalb seines Bereichs), wird unter „Meldungen“ beantwortet, und die laufende Konfiguration bleibt. „Speichern“ schreibt die Konfiguration, wie sie jetzt ist, mit jeder Änderung aus den Feldern „Effekte“ und „Parameter“ und an den Bauteilnamen. Wo nur Werte geändert wurden, behält die Datei die Kommentare und den Aufbau der Datei, aus der die Konfiguration gelesen wurde (der geöffneten Datei oder der Standardkonfiguration der Maschine); wo eine Liste geändert wurde, wird sie schlicht geschrieben, ohne Kommentare. Ein Wechsel der Maschine leert das Dateifeld, und „Speichern“ schreibt nicht über eine Datei, die die Konfiguration einer anderen Maschine enthält: Es meldet das unter Meldungen und lässt die Datei, wie sie ist |
| Konfigurationsdatei: Öffnen…, Speichern unter… | Dasselbe mit dem Dateidialog des Systems, der dort beginnt, wohin das Feld zeigt. Das Fenster wartet, solange der Dialog offen ist |
| Konfigurationsdatei: Motor-Artikelnummer, Netzspannung, Getriebe, Artikelnummer des Antriebs, Riemen, Achse, Bauteile einsetzen | Die Bauteile, die `[parts]` der Konfiguration benennt, so gezeigt, wie die Konfiguration sie hat. „Bauteile einsetzen“ benennt das Eingetippte in der laufenden Konfiguration, wie `--motor`, `--supply`, `--gearbox`, `--drive`, `--belt` und `--axis` und wie „Einsetzen“ der Motorsuche: einen Motor mit seiner Artikelnummer (`1FK2104-5AK`), die Netzspannung `230v`, `400v` oder `s120`, ein Getriebe als Baugröße-Flansch-Übersetzung (`NRK070-04-5`), einen Antrieb mit seiner Artikelnummer (`6SL3210-5HE11-5UF0`), einen Riemen als Profil-Breite (`AT5-25`), eine Achse mit Namen (`EXCT-30`, `ELZI-40`); ein leeres Feld benennt keines. Ein Name, den der Katalog nicht hat, wird unter „Meldungen“ abgelehnt, ebenso jeder Name in einer Konfiguration ohne `[parts]` |
| Analyse: Zyklus analysieren und beurteilen | Die Zyklusanalyse und die Prüfungen, als drei Tabellen: die Kennwerte der Maschine (Zykluszeit, Bahnfehler, Abstand vom Ziel beim Verweilen, kleinste Riemenkraft, Zeit mit schlaffem Trum, Radialkraft auf eine Scheibenwelle, Führungsbelastung, Lebensdauer der Führung), die Belastung je Motor (Spitzenmoment, Effektivmoment, kubisch gemitteltes Moment, Übertemperatur der Wicklung, aufgenommene und rückgespeiste Energie, Energie und mittlere Leistung am Bremswiderstand, höchste Zwischenkreisspannung) und eine Zeile je Prüfung |
| Analyse: mit Fehlerbändern, Stichproben | Angekreuzt fährt „Zyklus analysieren und beurteilen“ den Zyklus zusätzlich mit jedem Wert aus `[bounds]` an seiner unteren und oberen Grenze (zwei weitere Zyklen je Wert) und zeigt eine vierte Tabelle: je Prüfung den kleinsten und den größten Wert, das Ergebnis im ungünstigsten Fall und „Bestandene Läufe [%]“. Dasselbe wie `sim-cli verdicts --bands`. Steht im Zahlenfeld „Stichproben“ neben dem Kontrollkästchen eine Zahl über 0, wird der Zyklus stattdessen so oft mit allen Werten aus `[bounds]` zugleich zwischen ihren Grenzen gefahren, und die Tabelle trägt die Überschrift „Die Prüfungen mit allen unsicheren Werten zugleich zwischen ihren Grenzen“. Dasselbe wie `--bands --samples N`; die App nimmt immer den Startwert (`--seed`) 1. Mit „ungünstigste Ecke“ (nur zusammen mit den Fehlerbändern) sucht sie zusätzlich je Prüfung die ungünstigste Ecke der Grenzen und zeigt eine Tabelle mit dem ungünstigsten Wert, dem Grenzwert dort, dem Ergebnis und den Werten an ihrer oberen und an ihrer unteren Grenze. Dasselbe wie `--bands --worst`; das braucht viele Zyklen mehr |
| Effekt-Ansichten | Klappt auf, wenn ein Zyklus analysiert wurde; ihr eigener Knopf „Zyklus analysieren und beurteilen“ tut dasselbe wie der im Teil Analyse. „Wofür jeder Motor sein Moment braucht“: je Motor ein Plot über das Zeitfenster der Oszilloskope mit den vier Anteilen seines Moments (Trägheit, Schwerkraft, Reibung, Riemen und Rest) und gestrichelt dem Moment selbst, das sie zusammen ergeben; Zeitachse und Cursor teilt er mit den Oszilloskopen. „Momentanteile über den analysierten Zyklus“: je Anteil und Motor Spitze und Effektivwert, wie `sim-cli analyze --breakdown` sie schreibt. „Energiebilanz des analysierten Zyklus“: die Einspeisung, jeder Verlust und Speicher, der nicht genau null ist, und was nicht zugeordnet ist, in J und in Prozent der Einspeisung, wie `--budget` sie schreibt. „Reibung je Führung“: je Führung die Reibkraft beim stationären Gleiten über der Geschwindigkeit (durchgezogen) und ihre Coulomb-, viskosen und Haftanteile (gestrichelt) bei der jetzigen Führungslast, der Punkt, an dem die Führung gerade ist, und eine Tabelle mit Geschwindigkeit, den drei Anteilen, der Kraft und dem Lastfaktor jetzt. „Temperaturen“: die Übertemperatur jeder Wicklung und, mit der Erwärmung von Getriebe und Führung, jedes Getriebes und jeder Führung über das Zeitfenster der Oszilloskope, die Tabelle mit jetzt, Endwert im analysierten Zyklus, zulässiger Übertemperatur und Anteil des Endwerts daran, und die Farbskala der 3D-Ansicht. „Riemenabschnitte“: Riemenkraft (Vorspannung gestrichelt) und Dehnung je Abschnitt, die Tabelle beider jetzt und die Farbskala des Riemens. „Effekt ein und aus“: ein Effekt, eine Größe und „Vergleichen“; beide Läufe übereinander, aus gestrichelt, und je Linie Spitze und Effektivwert der Differenz. Über jedem Plot die Namen seiner Linien in ihren Farben. Siehe [Sehen, wohin Moment und Energie gehen](#sehen-wohin-moment-und-energie-gehen) |
| Analyse: Betriebspunkte | Nach „Zyklus analysieren und beurteilen“: das Motormoment über der Motordrehzahl einmal je Millisekunde des Zyklus, eine Farbe je Motor, über der Drehmoment-Drehzahl-Kennlinie der Motoren (gestrichelt: das Spitzenmoment in beide Richtungen bis zur Bemessungsdrehzahl, das bis zur Leerlaufdrehzahl auf null fällt), und darunter dieselben Punkte als rollbare Tabelle. Dasselbe wie `sim-cli analyze --points` |
| Analyse: Empfindlichkeit | Die acht Parameter mit `[bounds]`, von denen der Zyklus am stärksten abhängt |
| Analyse: Zyklus optimieren | Der Optimierer. „Diese Einstellungen in die laufende Maschine übernehmen“ setzt sein Ergebnis in die Konfiguration ein |
| Motorsuche | Netzspannung (230v, 400v oder s120), der Anfang der Artikelnummern (leer: jeder Motor des Katalogs) und wahlweise eine Getriebebaureihe wie `NRK` oder `NRK070`; angekreuzt probiert „Jede Achse des Katalogs“ jeden Motor zusätzlich auf jeder Achse des Katalogs, die zur Art der Maschine passt (`--axes`). „Suchen“ fährt jeden solchen Motor auf dem Zyklus und listet sie auf, bestandene zuerst, günstigste zuerst (die eingebaute Masse steht für den Preis), mit Getriebe, Achse, der Masse von Motor und Getriebe, der eingebauten Masse und der knappsten Prüfung oder dem Grund, aus dem ein Motor oder eine Achse nicht gefahren wurde. „Einsetzen“ in einer Zeile benennt diesen Motor, seine Netzspannung, sein Getriebe, seinen Antrieb und seine Achse in `[parts]` der laufenden Konfiguration. Dasselbe wie `sim-cli search` |
| Bahn | Ein Dateiname, eingetippt. „Geraden, Kreisbögen und Splines“ ist eine Bahndatei, geplant innerhalb der Achsgrenzwerte oder, angekreuzt, „So schnell, wie die Motoren es erlauben“; „Posen mit Zeiten“ ist eine Datei mit einer zeitlich vorgegebenen Bahn. „Planen, fahren und analysieren“ plant die Bahn von dort, wo die Maschine steht, analysiert sie mit einer eigenen Simulation (Dauer, Abweichung durch Übergangsbögen, Bahnfehler, Abstand hinter dem Sollwert, Belastungen, Prüfungen), zeichnet die geplante Bahn als Diagramm im Bedienfeld und als Linie in der 3D-Ansicht und lässt die Maschine in der 3D-Ansicht sie fahren. Dasselbe wie `sim-cli analyze --path` oder `--trajectory` |
| Aufzeichnung | Ein Dateiname, eingetippt. „Nachfahren und anpassen“ fährt die Antriebsaufzeichnung nach und zeigt je Motor, wie weit die Simulation von ihr abweicht (Moment und Schleppfehler, Effektivwert und Spitze), und je Achse die angepasste Masse, Reibung und konstante Kraft neben den Werten der Konfiguration, mit dem, was in der Konfiguration zu ändern ist. Dasselbe wie `sim-cli replay` und `fit --trace`. Die Startpose der Konfiguration muss dort liegen, wo die Aufzeichnung beginnt. Darunter „Aufzeichnung gegen Simulation“: Motormoment, Motordrehzahl und Schleppfehler am Motor über der Zeit, die Aufzeichnung durchgezogen und die Simulation gestrichelt, eine Farbe je Motor. „CSV exportieren“ dort schreibt sie nach `replay-overlay.csv` im Arbeitsverzeichnis, dieselbe Datei wie `sim-cli replay --out` |
| Frequenzgang der Mechanik | Von, bis, Frequenzen und Momentamplitude, wie `sim-cli sweep` sie nimmt (anfangs 5 bis 500 Hz, 120, und 1,5 N m oder 40 % des Spitzenmoments des Antriebs, wo das weniger ist). „Jeden Motor messen“ misst den Frequenzgang im offenen Kreis vom Moment jedes Motors zu seiner Drehzahl, wie `sim-cli sweep --motor N` für einen: Betrag in dB und Phase über der Frequenz, eine Linie je Motor, und je Motor den Einbruch, bei dem die Last am Riemen schwingt (Resonanz und Dämpfungsgrad des Einbruchs mit der niedrigsten Frequenz; leer, wo der Bereich keinen Einbruch mit beiden Flanken enthält). Die Voreinstellungen zeigen ihn an jedem Motor der Standardmaschinen. Vorher taten sie das nicht: Die Amplitude war 0,05 N m, bei der die Haftreibung die Motoren hält, nicht der Bereich. Ein Wechsel der Maschine oder das Öffnen einer Konfiguration setzt die Amplitude auf die Voreinstellung dieser Konfiguration, sofern keine andere eingetippt wurde |
| Frequenzgang: Gemessener Frequenzgang, gemessen an, Riemensteifigkeit anpassen | Eine Datei, wie `sim-cli sweep` sie schreibt (die Spalten `frequency_hz` und `gain_rad_s_per_nm`), gemessen an dem angewählten Motor mit der Momentamplitude darüber. „Riemensteifigkeit anpassen“ rechnet den Frequenzgang der Konfiguration bei den Frequenzen der Datei und zeigt Resonanz und Dämpfungsgrad beider und was in der Konfiguration zu ändern ist: den Faktor auf die Riemensteifigkeit. Dasselbe wie `sim-cli fit --sweep DATEI --sweep-motor N --amplitude A` |
| Reglereinstellung: Sprungantwort und Frequenzgang messen | Sprungantwort und Führungsfrequenzgang einer Achse, mit Anstiegszeit, Überschwingen, Einschwingzeit und Bandbreite |
| Diagramme der Oszilloskope | Ein Diagramm je Größe, eine Linie je Achse oder Motor. Sollwerte sind gestrichelt in der Farbe ihres Istwerts gezeichnet. Gemeinsame Zeitachse und gemeinsamer Cursor; Min, Max, Spanne und Effektivwert je Kurve; die Einheiten wechseln von selbst auf mm und µm |

In den Ergebnistabellen der Analyse steht „in Ordnung“ für `ok` und „NICHT BESTANDEN“ für `FAIL`.

Das rechte Bedienfeld rollt als Ganzes, und jeder seiner Teile (Oszilloskope, Konfigurationsdatei, Analyse, Effekt-Ansichten, Motorsuche, Bahn, Aufzeichnung, Frequenzgang der Mechanik, Reglereinstellung) lässt sich unter seiner Überschrift einklappen. Ein Teil klappt von selbst auf, wenn sein Ergebnis eintrifft. Die Tabellen sind für die Standardbreite des Bedienfelds ausgelegt: Lange Namen brechen um, und Zahlen haben vier Stellen.

Die Schaltflächen dieser Teile rechnen im Hintergrund mit eigenen Simulationen, und zwar mit der Konfiguration, wie sie in diesem Augenblick ist; solange steht ein Wartezeichen an Stelle der Schaltfläche. Die laufende Maschine wird nicht gestört. Ein Ergebnis wird nicht neu berechnet, wenn sich danach ein Parameter ändert. Es wird stattdessen als „für eine frühere Konfiguration berechnet“ gekennzeichnet, und nichts davon wird in die geänderte Konfiguration übernommen: „Diese Einstellungen übernehmen“, „Einsetzen“ und das Fahren einer geplanten Bahn antworten dann mit einer Meldung (die Änderung zurücknehmen oder neu berechnen). Einen Motor mit „Einsetzen“ zu benennen macht die Suche, aus der er stammt, nicht ungültig; danach lässt sich eine andere Zeile versuchen. Eine Achse der Achssuche zu benennen schon, denn sonst lief die Suche auf der eigenen Achse der Konfiguration.

Eine Datei, die fehlt oder nicht lesbar ist, eine nicht lesbare Zeile darin, eine andere Netzspannung als 230v, 400v und s120 und eine Suche, die keinen Motor findet oder kein Getriebe der Baureihe, das an einen passt, werden unter „Meldungen“ beantwortet. Ebenso eine Bahn, die die Maschine nicht fahren kann, weil sie sich seit dem Planen bewegt hat oder in Bewegung ist.

Ein Antrieb, der wegen Überspannung im Zwischenkreis abschaltet, wird unter „Meldungen“ genannt, einmal je Abschaltung, für die laufende Maschine und für einen analysierten Zyklus oder eine analysierte Bahn. „Antriebe freigeben“ bringt die Antriebe zurück.

Was die laufende Maschine mit einer Änderung tut:

- Eine geänderte Getriebeübersetzung (`gearbox.ratio`) oder ein geänderter Scheibenradius (`geometry.pulley_radius_m`) baut den Zustand der Maschine an der Pose und mit der Geschwindigkeit des Werkzeugs neu auf, mit Riemen und Getriebe so, wie sie dort in Ruhe sind. Die Riemenkräfte bleiben bis auf einige Prozent, wie sie waren.
- Eine Änderung am Input Shaper (sein Schalter, die Frequenz, der Dämpfungsgrad oder „Input Shaper folgt der Pose“) wird übernommen, sobald die geplante Bewegung das nächste Mal steht. So springt der Sollwert nicht. Ein Zyklus ohne Verweilzeit und ohne Greiferwechsel steht nie; dort wartet die Änderung auf „Nach Segment anhalten“.
- Ein Zyklus, der im Lauf Segmente verliert, läuft in den Segmenten weiter, die er hat.

Meldungen: Eine Meldung, die nur die letzte wiederholt, wird nicht hinzugefügt. Endet die Simulation oder eine Berechnung im Hintergrund unerwartet, sagt das eine Meldung; die Maschine erneut zu wählen startet die Simulation neu. Steht das Fenster länger als zehn Sekunden simulierter Zeit, verlieren die Oszilloskope die Werte davor.

Grenzen bei dieser Aufgabe:

- Die App startet mit der Standardkonfiguration der Maschine; eine andere, etwa die Konfiguration der EXCT-30 `crates/machine-t-gantry/exct-30.toml`, wird unter „Konfigurationsdatei“ geöffnet. Ein `--set` gibt es nicht: Werte werden im Parameterfeld geändert.
- Eine Zahl eines benannten Bauteils, die im Parameterfeld geändert wird, wird wieder überschrieben.
- Der Zyklus selbst lässt sich in der App nicht bearbeiten.
- Nicht in der App, nur auf der Befehlszeile: eine Empfindlichkeitsrechnung über Tabellen ohne `[bounds]` und die Betriebspunkte einer Bahn (die App zeigt die des Zyklus).
- Die Tabelle der Motorsuche zeigt nur die knappste Prüfung; Spitzenmoment und Übertemperatur der Wicklung eines Kandidaten stehen in `sim-cli search` oder nach „Einsetzen“ in der Zyklusanalyse.
- Dateinamen von Bahnen, Aufzeichnungen und gemessenen Frequenzgängen werden eingetippt; nur die Konfigurationsdatei hat einen Dateidialog. Ein relativer Name gilt ab dem Verzeichnis, in dem die App gestartet wurde.
- Die Linien der geplanten und der gefahrenen Bahn in der 3D-Ansicht hat kein Mensch am Bildschirm geprüft.
- Eine Bahn wird von dort analysiert, wo die Maschine steht, wenn die Schaltfläche gedrückt wird. Eine Maschine, die dann in Bewegung ist, lehnt das Fahren ab.
- Der Basis-Zeitschritt `sim.dt_s` lässt sich im laufenden Betrieb nicht ändern.
- Die Mausbedienung hat kein Mensch geprüft. Bildschirmfotos des Fensters in beiden Sprachen wurden geprüft.
- Nur für den Desktop.
