# Anbindungen: CAD, FreeCAD, Onshape und der Auslegungsbericht

Der Simulator legt eine Maschine anhand einer Konfigurationsdatei aus. Diese Seite beschreibt, wie Werte aus dem
CAD in diese Datei kommen und wie die Bewertungen als Dokument herauskommen, das ein Kunde unterschreibt. Die
englische Fassung ist [../integrations.md](../integrations.md).

| Teil | Ort | Stand |
|---|---|---|
| STEP → Masse, Schwerpunkt, Trägheit → Konfiguration | `tools/cad/step_mass.py` | fertig, getestet |
| FreeCAD-Makro: Teile markieren, exportieren, Auslegung prüfen | `tools/cad/SimSizing.FCMacro` | fertig, ohne Oberfläche getestet und einmal in der Oberfläche |
| Onshape-Masseneigenschaften → Konfiguration | `tools/cad/onshape_mass.py` | Prototyp: Offline-Weg getestet, API-Weg ungetestet |
| Auslegungsbericht, PDF und HTML | `sim-cli report`, Knopf „Auslegungsbericht…“ in der App | fertig, getestet |

## Woher ein Wert kommt

Jeder Wert einer Konfiguration kann in einem Kommentar direkt darüber sagen, woher er kommt:

- `# [catalogue] …` aus einem Herstellerdokument gelesen oder abgeleitet;
- `# [estimate] …` nicht veröffentlicht, unsere Schätzung (mit unterem und oberem Wert in `[bounds]`);
- `# [cad] <Quelle>: m=…, COM=(…) m, I_com=[[…]] kg m2` von den Werkzeugen unten aus einem CAD-Modell berechnet.

Eine Markierung gilt für die Schlüssel direkt darunter, bis zu einer Leerzeile oder dem nächsten Kommentar, so dass
eine Gruppe von Werten eine Markierung teilt; eine `[cad]`-Zeile, die die CAD-Werkzeuge je Schlüssel schreiben, gilt
nur für den Schlüssel darunter. Eine Markierung im Kommentar am Ende der Zeile eines Schlüssels gilt für diesen. Eine
Markierung direkt über oder hinter einer Tabellenüberschrift (`[mass]`) gilt für jeden Schlüssel der Tabelle ohne
eigene. Stehen in einem Kommentarblock Markierungen auf mehreren Zeilen, zählt die dem Schlüssel nächste Zeile; nennt
eine Zeile zwei („[catalogue] … its efficiency [estimate]“), werden beide angegeben, und ihr Wortlaut sagt, welcher
Wert welcher ist. Ein Wert in einer Inline-Tabelle (`mass = { … }`) bekommt keine Markierung. Der Auslegungsbericht
führt jede Eingabe mit ihrer Quelle auf.

## Rollen: welches Teil wohin gehört

Teile werden mit einer Rolle markiert. Mehrere Teile einer Rolle werden zusammengefasst: Massen addiert, ein
gemeinsamer Schwerpunkt, die Trägheitstensoren mit dem Satz von Steiner dorthin verschoben.

| Maschine | `carriage` | `arm` | `payload` |
|---|---|---|---|
| `t-gantry` | `mass.carriage_kg` | `mass.z_axis_kg` | `mass.payload_kg` |
| `single-axis` | `mass.carriage_kg` | — | `mass.payload_kg` |
| `delta-d3` | `mass.platform_kg` | `mass.upper_arm_kg`, `upper_arm_cg_m`, `upper_arm_inertia_kgm2` | `mass.payload_kg` |

Delta-Oberarm: einen Oberarm so modellieren, dass seine Schulterachse auf der x-Achse des CAD durch den Ursprung
liegt. `upper_arm_cg_m` ist der Abstand seines Schwerpunkts von dieser Achse, `upper_arm_inertia_kgm2` seine
Trägheit um sie.

Geschrieben werden nur Werte in `[mass]`. Der volle Tensor und der Schwerpunkt stehen im `[cad]`-Kommentar; die
Modelle nutzen heute nicht mehr davon (zum Beispiel wird `geometry.z_com_drop_m` des T-Portals nicht aus dem CAD
gesetzt). Zwei Dinge überlassen die Werkzeuge Ihnen: `[bounds]` eines Werts, der nun aus dem CAD kommt, bleiben, wie
sie waren, so dass `--bands` ihn weiter über den Bereich der alten Schätzung variiert (enger fassen oder entfernen);
und die `upper_arm_*`-Werte des Deltas sind der Arm ohne Ellbogenblock, der in `mass.elbow_kg` steht: den
Ellbogenblock nicht markieren, sonst zählt er doppelt. `delta-d3` ist nutzbar, sobald der Simulator diese Maschine
registriert (`sim-cli list`).

## (a) STEP-Dateien

OpenCascade kommt von PyPI (`cadquery-ocp`, mit Wheels für Python 3.10 bis 3.14):

```bash
.venv/Scripts/python -m pip install cadquery-ocp
```

```bash
python tools/cad/step_mass.py MODELL.step --map tools/cad/example-map.toml --machine t-gantry --out cad.toml --report
```

```bash
sim-cli verdicts t-gantry --config cad.toml
```

Die Zuordnungsdatei gibt Dichten (`[materials]`, kg/m³; Stahl, Aluminium, Edelstahl, POM und Kohlefaser sind
eingebaut) und Regeln `[[part]]` mit `match` (ein Muster auf den Produktnamen im STEP), `role` und `material`; die
erste passende Regel gilt, nicht zugeordnete Teile werden auf der Fehlerausgabe genannt und nicht gezählt. Die
Längeneinheit der Datei rechnet der Leser in Meter um. `--config` wählt die Ausgangskonfiguration (sonst die
`default.toml` der Maschine); sie muss vollständig sein, und nur die zugeordneten Zeilen in `[mass]` und ihre
`[cad]`-Kommentare ändern sich, jede andere Zeile bleibt Byte für Byte.

Getestet gegen von Hand gerechnete Körper: Quader, Vollzylinder, Hohlrohr, ein Teil abseits des
Ursprungs, eine Datei in Metern und ein Verbund gegen OpenCascades eigenes Ergebnis (`tools/cad/test_cadmass.py`).

## (b) FreeCAD

FreeCAD 1.x: `tools/cad/SimSizing.FCMacro` ausführen (Makro → Makros…, mit dem Makroordner auf `tools/cad`, oder den
ganzen Ordner kopieren). Im Dialog:

1. Teile im Baum wählen und mit Rolle und Werkstoff oder einer Dichte markieren. Die Markierungen sind Eigenschaften
   des Objekts (`SimRole`, `SimMaterial`, `SimDensity`), im Eigenschafteneditor sichtbar und in der `.FCStd`
   gespeichert.
2. Einstellungen: Maschine, das Programm `sim-cli`, die Ausgangskonfiguration und die Ausgabedatei; in FreeCADs
   Parametern (`Macros/SimSizing`) gespeichert.
3. „Export and check“ fasst die markierten Teile je Rolle zusammen (Formen in mm, in SI umgerechnet, jede in ihrer
   globalen Lage), schreibt die Konfiguration mit `[cad]`-Zeilen (Quelle: der Name des Dokuments), ruft
   `sim-cli verdicts` darauf auf und zeigt jede Prüfung mit Wert, Grenze, Reserve und Ergebnis, nicht bestandene rot,
   in einer Tabelle und in der Ausgabeansicht.

Die Logik steht in `tools/cad/simsizing.py` und läuft ohne Oberfläche; `tools/cad/test_freecad_macro.py` führt sie
unter `freecadcmd` an einem markierten Quader und Zylinder aus und vergleicht Massen und Trägheiten mit Handwerten.

Reserve im Makro: `sim-cli verdicts` gibt keine Richtung an. Stimmt „Wert ≤ Grenze“ mit dem Ergebnis überein, ist es
eine Höchstwert-Prüfung mit der Reserve (Grenze − Wert) / Grenze; sonst eine Mindestwert-Prüfung (Lebensdauern,
Riemenlose) mit (Wert − Grenze) / Wert, wie im Auslegungsbericht.

## (c) Onshape: Entwurf und Prototyp

**Ziel.** Wie bei FreeCAD: Massen und Trägheiten markierter Teile in eine Konfiguration und die Bewertungen zurück.

**Wege.**

- *REST-Anbindung (gewählt).* Onshapes Endpunkt `GET /api/v9/partstudios/d/{did}/{w|v|m}/{id}/e/{eid}/massproperties`
  liefert Masse, Volumen, Schwerpunkt und Trägheit eines Part Studios, von Onshape mit den dort zugewiesenen
  Werkstoffen berechnet. Ein kleiner Client liest ihn und schreibt die Konfiguration mit `tools/cad/cadmass.py` wie die
  beiden anderen Werkzeuge. Läuft überall, wo Python läuft, braucht keinen Code in Onshape.
- *FeatureScript-Feature.* Läuft in Onshape und könnte eine Rolle als Attribut an Teile hängen, kann aber den
  Simulator nicht aufrufen, und ob es einen Trägheitstensor auswerten kann, wurde hier nicht geprüft. Später
  zumindest zum Markieren nützlich.
- *Onshape-App mit OAuth im Browser*, die Bewertungen in Onshape zeigt: braucht einen Server und eine
  App-Registrierung; der Schritt nach dem REST-Client.

**Rollen.** Ein Part Studio je Rolle (später ein Teilefilter): der Eintrag `-all-` seiner Masseneigenschaften ist die
Rolle.

**Was bekannt ist.** Aus Onshapes Dokumentation (Part-Studios-Leitfaden, API-Schlüssel-Leitfaden): die Antwort hat
`bodies["-all-"]` mit `mass` (3 Zahlen), `centroid` (9), `inertia` (27), `hasMass` und `massMissingCount`; die Zahlen
passen zu SI-Einheiten (Masse durch Volumen im Beispiel ergibt 7850 kg/m³), und jede Größe scheint ihr Wert gefolgt
von unterer und oberer Schranke zu sein. Anfragen werden mit einem API-Schlüssel signiert: HMAC-SHA256 über Methode,
Nonce, Datum, Content-Type, Pfad und Query, jeweils mit Zeilenumbruch, in Kleinbuchstaben, Base64.

**Was nicht bestätigt ist**, weil es ein Konto und Schlüssel braucht, die dieses Projekt nicht hat: ob der Tensor um
den Schwerpunkt oder um den Ursprung gilt (`--inertia-about centroid|origin`, Vorgabe centroid), die Reihenfolge der
Schranken und ob eine so gebaute Signatur angenommen wird. Ein Teil ohne Werkstoff macht `massMissingCount` ungleich
null; der Prototyp lehnt das ab, statt das Teil wegzulassen.

**Prototyp.** `tools/cad/onshape_mass.py`:

```bash
python tools/cad/onshape_mass.py --machine t-gantry --role carriage=carriage.json --role payload=payload.json --out cad.toml
```

Eine Quelle ist eine gespeicherte massproperties-Antwort (angemeldet im API Explorer von Onshape speichern: geht ohne
Schlüssel) oder eine Part-Studio-URL, abgerufen mit `ONSHAPE_ACCESS_KEY` und `ONSHAPE_SECRET_KEY` (ungetestet).
`tools/cad/test_onshape.py` prüft die Umrechnung an einer Antwort in der Form des dokumentierten Beispiels, beide
Bezugspunkte des Tensors, das Ablehnen von Teilen ohne Werkstoff und die Signatur gegen das dokumentierte Verfahren.

**Zum Fertigstellen, mit Schlüsseln:** einen Stahlquader 100 × 200 × 300 mm abseits des Ursprungs modellieren,
abrufen und mit den Handwerten vergleichen; das klärt Bezugspunkt und Signatur. Danach ein Teilefilter, damit ein
Part Studio alle Rollen enthalten kann.

## (d) Der Auslegungsbericht

```bash
sim-cli report t-gantry --config crates/machine-t-gantry/exct-30.toml --bands --lang de --out bericht.pdf
```

Endet `--out` auf `.pdf`, entsteht ein PDF (A4), sonst eigenständiges HTML (Diagramme als eingebettetes SVG; bei
Bedarf aus dem Browser drucken). `--lang de` schreibt ihn deutsch, `--bands` fügt die Fehlerbänder hinzu, und jede
Option der anderen Befehle (`--motor`, `--set`, …) wirkt wie dort. Das Ergebnis (alle bestanden oder wie viele nicht)
steht auf der Fehlerausgabe; der Rückgabewert ist in beiden Fällen 0. In der App schreibt „Auslegungsbericht…“ neben
den Knöpfen der Konfigurationsdatei den Bericht der Konfiguration, wie sie gerade ist, mit Fehlerbändern, wenn sie im
Analysebereich angehakt sind.

Inhalt:

1. **Maschine**: Name, Konfigurationsdatei, Datum, Simulatorversion, Zykluszeit und Gesamtergebnis; die in `[parts]`
   benannten Teile und die Werte aus `[mass]` und `[geometry]`, jeder mit seiner Quelle.
2. **Zyklus**: Motormoment und -drehzahl über den gemessenen Durchlauf und die Arbeitspunkte gegen die
   Drehmoment-Drehzahl-Kennlinie des Antriebs.
3. **Bewertungen**: jede Prüfung mit Wert, Grenze, Reserve und Ergebnis; nicht bestandene Zeilen rot. Die Reserve ist
   eins minus die Auslastung der Prüfung (Wert durch Grenze; bei Lebensdauern und Riemenlose Grenze durch Wert),
   negativ heißt überschritten. Die Werte sind die, die `sim-cli verdicts` ausgibt.
4. **Fehlerbänder** (mit `--bands`): je Prüfung der niedrigste und höchste Wert mit jedem begrenzten Parameter auf
   seinen Grenzen und ob der ungünstigste Fall besteht.
5. **Eingaben und ihre Quellen**: jeder Wert des Laufs mit `[catalogue]`, `[estimate]` oder `[cad]` aus den
   Kommentaren, „Teil in [parts]“, wo ein aus dem Katalog benanntes Teil ihn gebracht hat, „für diesen Lauf gesetzt“, wo
   er von der Datei abweicht (`--set`, ein Teilename auf der Kommandozeile, eine Änderung in der App), „nicht
   angegeben“, wo die Datei nichts sagt.
6. **Freigabe**: erstellt von, geprüft von und abgenommen vom Kunden, jeweils mit Name, Firma, Datum und Unterschrift.

Das PDF schreibt der Simulator selbst (`sim_components::report`), mit den Standardschriften Helvetica und ohne fremde
Bibliothek; Zeichen außerhalb von Latin-1 erscheinen als „?“.

## Tests

```bash
cargo test -p sim-components report
```

```bash
cargo test -p sim-cli --test cli report
```

```bash
SIM_CLI=target/release/sim-cli.exe .venv/Scripts/python -m unittest discover tools/cad
```

Die Python-Tests überspringen, was sie nicht ausführen können (kein OpenCascade, kein FreeCAD, kein `sim-cli`, das
die aktuellen Konfigurationen liest), und sagen warum. `FREECADCMD` nennt `freecadcmd`, wo es nicht von selbst
gefunden wird.
