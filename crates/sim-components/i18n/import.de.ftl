# Import von Herstellertraces, die maschinenunabhängige Identifikation und die Datensatzprüfung.
error-import-layout = Die Tracedatei hat nicht den Aufbau ihres Formats: { $arg }
error-import-unit = Unbekannte Einheit; in der Zuordnung `unit` oder `scale` nach N m angeben: { $arg }
error-import-per-rev = Eine Einheit, die kein Winkel ist, braucht `per_rev` (Einheiten je Motorumdrehung) in der Zuordnung: { $arg }
error-import-reference = Ein Moment in Prozent oder Promille braucht `reference` (N m bei 100 %) in der Zuordnung: { $arg }
error-import-signal = Die Tracedatei hat kein Signal { $arg }
error-import-ambiguous = Mehr als ein Signal der Tracedatei passt zu { $arg }; den vollen Namen angeben.
error-import-map = Die Zuordnungsdatei fehlt oder hat einen falschen Wert: { $arg }
error-import-time = Die Zeiten des Trace lassen sich nicht auf ein Raster legen: { $arg }
error-identify-friction = Motor { $arg }: keine Trägheit und Reibung ermittelt. Der Trace muss ihn je mindestens 0,1 s in beide Richtungen drehen.
error-identify-lag = Motor { $arg }: keine Verzögerung ermittelt. Der Trace muss im Stillstand beginnen und der Sollwert muss sich ändern.
error-identify-two-mass = Kein Zweimassenmodell: der Frequenzgang braucht eine Senke, eine Spitze darüber und drei Punkte unter einem Drittel der Senke.
error-identify-ringing = Zwischen den angegebenen Zeiten wurde kein Ausschwingen gefunden.
dataset-ok = Datensatz { $arg }: keine Probleme gefunden.
dataset-problem = Problem im Datensatz: { $arg }
error-identify-response = Der Trace ist für einen Frequenzgang bis hinab zu --from zu kurz: er braucht acht Fenster von je vier Perioden der niedrigsten Frequenz.
