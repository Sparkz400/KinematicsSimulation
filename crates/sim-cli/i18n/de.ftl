cli-usage = Aufruf: sim-cli list | run MASCHINE [--move a,b | --cycle] [--time SEKUNDEN] [--every SCHRITTE] | analyze MASCHINE [--points DATEI] | sweep MASCHINE [--motor N] [--from HZ] [--to HZ] [--count N] [--amplitude NM] | step MASCHINE [--axis N] [--size M] [--time SEKUNDEN] | bode MASCHINE [--axis N] [--from HZ] [--to HZ] [--count N] [--amplitude M] | tuning MASCHINE; für alle: [--config DATEI] [--set pfad=wert]... [--out DATEI] [--lang en|de]
cli-error-unknown-machine = Unbekannte Maschine: { $arg }
cli-error-bad-argument = Dieses Argument ist nicht lesbar: { $arg }
cli-error-io = Dateifehler: { $arg }
