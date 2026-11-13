# Feldbus-Emulation: ein simulierter Antrieb an EtherCAT oder PROFINET

Entwurfsnotiz. Die Frage: Kann der Simulator einer echten SPS als Servoantrieb an EtherCAT oder PROFINET erscheinen, sodass die Motion-Software der SPS (TwinCAT NC, CODESYS SoftMotion, S7-1500-Technologieobjekte) ihn wie einen echten Antrieb fährt? Heute spricht die SPS über OPC UA mit dem Simulator ([SPS-Inbetriebnahme](sps.md)); das ist azyklisch und weit langsamer als ein Antriebsbus.
English version: [Fieldbus emulation](../fieldbus-emulation.md).

Ergebnis: möglich, aber nicht mit dem Simulator allein. EtherCAT braucht Slave-Hardware; PROFINET RT-Klasse 1 geht mit einer gewöhnlichen Netzwerkkarte unter Linux, braucht aber eine Echtzeitschleife mit 1 ms. Gebaut wurde hier keins von beiden. Die Fakten wurden am 13.11.2026 geprüft; die Quellen sind verlinkt.

## Was ein Antrieb am Bus können muss

| | EtherCAT | PROFINET |
| --- | --- | --- |
| Antriebsprofil | CiA 402 über CoE (ETG.6010). Für zyklisch synchrone Lageregelung (CSP): Steuerwort 0x6040, Statuswort 0x6041, Betriebsart 0x6060/0x6061, Sollposition 0x607A, Istposition 0x6064, Istgeschwindigkeit 0x606C und die CiA-402-Zustandsmaschine ([Nanotec-Handbuch](https://en.nanotec.com/products/manual/c5-usb-en/modes%2Fcyc_sync_position.html)) | PROFIdrive. Telegramm 1 oder 3 (Drehzahlsollwert, Lageregler in der SPS), Telegramm 105 für S7-1500-Positionierachsen mit DSC (Lageabweichung XERR und Verstärkung KPC von der SPS, Lageregler im Antrieb). Die Wortbelegung der Telegramme 3, 102, 105 und 111 wurde hier nicht an einem Siemens-Listenhandbuch geprüft |
| Gerätebeschreibung | ESI (XML), Vendor-ID von der ETG (mit Mitgliedschaft kostenlos) | GSDML, Vendor-ID von PI |
| Zyklus | 250 µs bis 1 ms, Distributed Clocks mit SYNC0-Jitter unter 100 ns ([Synapticon](https://www.synapticon.com/en/motion-control-academy/ethercat-fuer-servoantriebe-praktischer-leitfaden)) | RT-Klasse 1: 1 bis 4 ms. IRT (isochron, nötig für DSC im Taktsynchronbetrieb) braucht besondere Hardware |
| Konformität | Hersteller müssen mit dem ETG-Konformitätstest prüfen ([ETG](https://www.ethercat.org/en/conformance_for_vendors.htm)); für ein Laborgerät nicht nötig | Zertifizierung durch PI; für ein Laborgerät nicht nötig |

## EtherCAT

Ein EtherCAT-Slave verarbeitet jeden Frame im Durchlauf in Hardware, dem EtherCAT Slave Controller (ESC). Eine gewöhnliche Netzwerkkarte kann das nicht; ein konformer Slave braucht also einen ESC.

| Möglichkeit | Was es ist | Lizenz, Kosten |
| --- | --- | --- |
| SOES auf einer ESC-Platine | Quelloffener Slave-Stack von rt-labs. Hardwareschichten für LAN9252 (auch am Raspberry Pi), XMC4, TI Sitara ([Repository](https://github.com/OpenEtherCATsociety/SOES)) | Die Lizenzdatei im Repository nennt GPLv2 mit Linking-Ausnahme, die Produktseite von rt-labs GPLv3 zur Evaluierung und eine kommerzielle Lizenz ab 490 € im Jahr ([rt-labs](https://rt-labs.com/product/soes/)). Platine etwa 50 bis 200 € |
| Beckhoff FC1100 | PCI-Karte mit ET1100-ESC; ein PC wird Slave. Echtzeit laut Beckhoff „eingeschränkt“ ([FC1100](https://www.beckhoff.com/fc1100)) | Kommerziell |
| Hilscher cifX mit Firmware NXLFW-ECS | PC-Karte als zertifizierter Slave ([Hilscher](https://www.hilscher.com/products/multiprotocol-socs-stacks/stacks-for-industrial-ethernet-fieldbus/nxlfw-ecs)) | Kommerziell, Karte einige hundert Euro |
| Beckhoff-Brücke EL6695 | Tauscht Prozessdaten zwischen zwei Mastern aus, mit DC ([Beckhoff](https://infosys.beckhoff.com/content/1033/el6695/1317490955.html)). Der Simulator bräuchte einen eigenen Master (z. B. SOEM) | Klemme und ein zweiter Master; SOEM 2.x ist GPLv3 oder kommerziell |
| TwinCAT TE1111 EtherCAT Simulation | Ein zweiter PC mit TwinCAT bildet die Slaves einer Konfiguration auf einer gewöhnlichen Netzwerkkarte nach, mit DC und CoE ([Beckhoff](https://www.beckhoff.com/te1111)). Ob ein fremdes Programm seine Prozessdaten speisen kann, wurde nicht gefunden | TwinCAT-Lizenz je simuliertem Gerät |
| acontis EC-Simulator HiL | Bildet Slaves auf einer gewöhnlichen Netzwerkkarte nach, gegen TwinCAT- und CODESYS-Master, kann eine FMU rechnen ([acontis](https://public.acontis.com/manuals/EC-Simulator/3.2/html/ec-simulator-hil/intro.html)) | Kommerziell. Die FMU des Simulators (sim-fmi) könnte hier eingesetzt werden |
| Raw-Socket-Slave auf einer Netzwerkkarte | Software-Slave am Ende der Linie (z. B. das alte Projekt ecslave). Keine Verarbeitung im Durchlauf, keine DC-Genauigkeit | Nicht konform; nur gegen nachsichtige Master brauchbar |

SOEM ist ein Master, kein Slave. Es nützt nur auf der Simulatorseite einer EL6695-Brücke.

## PROFINET

Ein PROFINET-RT-Gerät sendet und empfängt gewöhnliche Ethernet-Frames; für RT-Klasse 1 reicht eine gewöhnliche Netzwerkkarte unter Linux.

| Möglichkeit | Was es ist | Lizenz, Kosten |
| --- | --- | --- |
| p-net | Quelloffener PROFINET-Device-Stack von rt-labs. Konformitätsklasse A/B, RT-Klasse 1, kein IRT. Linux mit jeder Netzwerkkarte, die Rohframes senden kann ([rt-labs](https://rt-labs.com/product/p-net/)) | GPLv3 oder kommerziell. Keine Hardwarekosten |
| pnio (Rust) | PROFINET-RT-Device rein in Rust, Linux, eine Verbindung, nach eigener Angabe mit einer S7-1515 erprobt ([docs.rs](https://docs.rs/crate/pnio/0.1.0)) | MIT/Apache-2.0. Version 0.1.0 vom August 2026; seine Reife wurde nicht geprüft |
| Hilscher cifX mit NXLFW-PNS | PC-Karte als PROFINET-Device | Kommerziell, Karte einige hundert Euro |
| Siemens SIMIT Unit PN | Hardware, die bis zu 128 oder 256 PROFINET-Geräte gegenüber einer echten Steuerung nachbildet, mit Antriebsverhalten je Telegramm; Zyklus ab SIMIT V10 bis 1 ms ([Siemens](https://cache.industry.siemens.com/dl/files/317/109759317/att_956837/v1/SIMIT_enUS_en-US.pdf)) | Kommerziell |

Siemens PN Driver ist ein Controller-Stack, kein Device-Stack. PLCSIM Advanced simuliert kein PROFINET IO mit fremden Geräten.

## Echtzeitanforderungen und dieser Simulator

Der Simulator rechnet seine Physik in Schritten von 62,5 µs; das ist schnell genug: Ein Schritt kostet beim T-Portal 3 µs Rechenzeit (50 ms je simulierter Sekunde, gemessen). Was nicht passt, ist der Takt. Die Kopplung wacht alle 10 ms auf (einstellbar bis 1 ms), mit dem Timer des Betriebssystems, und rechnet die fälligen Schritte am Stück. Ein Antrieb am Bus muss in jedem Buszyklus antworten:

- PROFINET RT-Klasse 1 mit 1 bis 4 ms: braucht eine Schleife, die in jedem Buszyklus mit einem Jitter weit unter einem Zyklus aufwacht. Unter Linux mit PREEMPT_RT und einem festgelegten Thread ist das üblich. Unter Windows ist das ohne Echtzeiterweiterung nicht erreichbar.
- EtherCAT mit DC bei 250 µs bis 1 ms: Der ESC gibt den Takt vor; der Simulator muss die nächste Istposition vor dem nächsten SYNC0 liefern, in jedem Zyklus. Mit einem ESC an SPI (LAN9252) ist das eine harte Echtzeitschleife unter Linux mit 1 kHz oder mehr.
- Die Sollwertinterpolation des Simulators (`profile.setpoint_interval_s`) würde zum Buszyklus, und seine Feldbusverzögerung und sein Feldbusjitter (`profile.fieldbus_delay_cycles`, `fieldbus_jitter_cycles`) würden durch den echten Bus ersetzt.

## Lizenzen

Das Projekt steht unter MIT/Apache-2.0. GPLv3-Stacks (p-net, SOEM 2.x, SOES nach der heutigen Angabe von rt-labs) dürfen nicht hineingelinkt werden. Sie können als eigener Prozess laufen, der über einen Socket oder gemeinsamen Speicher mit dem Simulator spricht; das lässt die Lizenz des Simulators unberührt, das eigene Programm steht dann unter GPLv3. Das Rust-Crate pnio steht unter MIT/Apache-2.0 und könnte gelinkt werden.

## Empfohlener Weg

1. Zuerst PROFINET: ein Linux-PC mit PREEMPT_RT, ein zweiter Netzwerkanschluss und ein Device-Prozess (pnio, oder p-net als eigener GPL-Prozess), der je Achse einen Antrieb mit PROFIdrive-Telegramm 3 anbietet und Soll- und Istwerte jede Millisekunde über gemeinsamen Speicher mit dem Simulator austauscht. Auf der Simulatorseite braucht es eine Echtzeitschleife mit 1 ms anstelle des 10-ms-Takts der OPC-UA-Kopplung und eine PROFIdrive-Zustandsmaschine (STW1/ZSW1). Keine Hardware außer einer Netzwerkkarte. Das ist der günstigste Weg zu einer S7-1500 oder einer CODESYS-Steuerung.
2. Danach EtherCAT: eine LAN9252-Platine mit SOES als CiA-402-CSP-Slave, deren Prozessdaten über SPI oder einen Socket an den Simulator gehen, mit dem Simulator unter Linux bei 1 kHz. Oder, für TwinCAT-Anwender mit einem zweiten PC, TE1111 oder der acontis EC-Simulator mit der FMU aus sim-fmi, wenn eine Lizenz da ist.
3. OPC UA mit PLCopen-Bausteinen bleibt für die virtuelle Inbetriebnahme von Abläufen, bei denen 10 ms und etwas Jitter keine Rolle spielen.

## Ist es hier machbar?

Nicht in diesem Schritt. Jeder Weg braucht Hardware oder einen Linux-Echtzeitrechner, um gegen eine echte SPS zu prüfen, eine GPLv3-Grenze oder ein junges Crate, eine neue Echtzeitschleife anstelle des Takts der Kopplung und eine Zustandsmaschine des Antriebsprofils. Ohne SPS und Bus zum Prüfen gebaut, wäre es ungeprüfter Code. Es wurde daher nicht gebaut. Die PLCopen-Schnittstelle über OPC UA ist der Teil, der sich hier prüfen lässt, und sie ist geprüft.
