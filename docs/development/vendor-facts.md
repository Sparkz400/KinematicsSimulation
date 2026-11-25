# Vendor facts for the KinematicsSimulation catalogue (research, read-only)

Method: PDFs were downloaded to the scratchpad `dl/` folder for reading only and text-extracted with pdftotext. Nothing in the repo was edited. "Printed page" = the number printed on the page, not the PDF index.

## 1. Harmonic Drive CSF / CSG / SHG: efficiency and no-load running torque

Sources (same as the repo's meta files):
- CSG/CSF: Harmonic Drive LLC, "Reducer Catalog, Component Sets CSG/CSF, Engineering Data", TI-26108 REV 1, https://www.harmonicdrive.net/_hd/content/documents1/CSG-CSF_Component.pdf
- SHG/SHF: TI-26121 REV 1, https://www.harmonicdrive.net/_hd/content/documents1/SHG-SHF_Component.pdf

### Efficiency
- The vendor prints NO numeric efficiency table. Efficiency is graphs only.
  - CSG/CSF: Table 046-1 (conditions), Graph 046-1 (load compensation Ke vs torque ratio), Graphs 047-1 to 047-8 (efficiency at rated torque vs ambient temperature, curves for 500/1000/2000/3500 rpm input), printed pages 46-47.
  - Graph split on p.47: size 8/11/14 ratio 30 (047-1), 50 and 80 (047-2), 100 (047-3); sizes 17-100 ratio 30 (047-4), 50 (047-5), 80 and 100 (047-6), 120 (047-7), 160 (047-8). So there is one curve set per ratio group and size group, not per size.
  - SHG/SHF: Table 090-1, Graph 090-1 (Ke), Graphs 090-2 to 090-4 and 091-1 to 091-5 (ratios 30, 50/80, 100, then 30, 50, 80/100, 120, 160), printed pages 90-91.
- Conditions printed (Table 046-1 / 090-1): load = the rated torque, grease Harmonic Grease SK-1A/SK-2 (recommended quantity), recommended housing tolerances. Temperature is the x-axis of each graph, input speed is the curve parameter. Oil lubrication: "Contact us".
- Load correction: Ke from Graph 046-1 / 090-1 (Ke = 1 at and above rated torque, falls with torque ratio); efficiency = Ke x R. The only numeric anchors in the text:
  - CSF-20-80, 1000 rpm, 20 C, grease: R = 78 % at rated torque; at load torque 19.6 Nm (ratio 0.58) Ke = 0.93, so 73 %. (CSG-CSF doc, printed p.46, Table 046-1 worked example.)
  - SHF-20-80, 1000 rpm, 20 C, grease: R = 82 %, Ke = 0.93, so 76 %. (SHG-SHF doc, printed p.90.)
  - Those are the only printed numbers I could extract. They are NOT at 2000 rpm / 25 C.
- The graphs are vector art with no data table. They are now digitised from the PDF paths by `catalog/tools/extract_harmonicdrive_efficiency.py` (readings in `catalog/tools/harmonicdrive_efficiency_readings.csv`, method in the `efficiency_full_load` note of `catalog/harmonicdrive/gearbox_*.meta.toml`): 2000 rpm, 25 C, rated torque, grease, about 0.70 to 0.80.
- The two printed anchors (78 % and 82 % at ratio 80, size 20, 1000 rpm, 20 C) came out at 79.1 % and 81.6 % from the digitised graphs.

### No-load running torque
CSG/CSF (printed p.44-45): Table 044-5 conditions (ratio 100, grease, "measured after 2 hours at 2000 rpm input"). Absolute values for ratio 100 are graphs only (Graphs 045-1 to 045-4 by input speed 500/1000/2000/3500 rpm, log scale vs size). Numeric part is Table 044-6, "Component Set No Load Torque Compensation", Ncm, to ADD to the ratio-100 graph value:

| Size | r30 | r50 | r80 | r120 | r160 |
|---|---|---|---|---|---|
| 8 | 0.4 | 0.2 | -- | -- | -- |
| 11 | 0.7 | 0.3 | -- | -- | -- |
| 14 | 1.1 | 0.5 | 0.1 | -- | -- |
| 17 | 1.8 | 0.8 | 0.1 | -0.1 | -- |
| 20 | 2.7 | 1.2 | 0.2 | -0.1 | -0.3 |
| 25 | 5.0 | 2.2 | 0.3 | -0.2 | -0.6 |
| 32 | 10 | 4.5 | 0.7 | -0.5 | -1.2 |
| 40 | -- | 8.0 | 1.2 | -0.9 | -2.2 |
| 45 | -- | 11 | 1.7 | -1.3 | -3.0 |
| 50 | -- | 15 | 2.3 | -1.7 | -4.0 |
| 58 | -- | 22 | 3.4 | -2.5 | -6.1 |
| 65 | -- | 31 | 4.7 | -3.5 | -8.4 |
| 80 | -- | 55 | 8.5 | -6.2 | -15 |
| 90 | -- | 77 | 12 | -8.7 | -21 |
| 100 | -- | 100 | 16 | -12 | -28 |

Reading caveat: the r160 column is vertically offset in the PDF text; I assigned its 11 values to sizes 20..100 (the 11 sizes the repo's CSF table has at ratio 160) in order. Values for r30..r120 are read unambiguously. Confidence: r30-r120 high, r160 medium.

SHG/SHF (printed p.88-89): Table 088-5 same conditions; Table 089-1 compensation, Ncm (as printed with sign):

| Size | r30 | r50 | r80 | r120 | r160 |
|---|---|---|---|---|---|
| 14 | +1.2 | +0.5 | +0.1 | -- | -- |
| 17 | +2.1 | +0.9 | +0.1 | -0.1 | -- |
| 20 | +3.1 | +1.4 | +0.2 | -0.2 | -0.4 |
| 25 | +5.7 | +2.5 | +0.4 | -0.3 | -0.7 |
| 32 | +11.7 | +5.2 | +0.8 | -0.6 | -1.4 |
| 40 | -- | +9.2 | +1.4 | -1.0 | -2.5 |
| 45 | -- | +12.7 | +2.0 | -1.4 | -3.5 |
| 50 | -- | +17.0 | +2.6 | -1.9 | -4.6 |
| 58 | -- | +25.8 | +4.0 | -2.9 | -7.0 |
| 65 | -- | -- | +5.4 | -4.0 | -9.7 |

Same r160 caveat (8 values to sizes 20..65, medium confidence). Ratio-100 base values: graphs only (Graphs 089-1 ff.). The starting torque is a separate table (Table 087-2, SHG) and is not covered here.

## 2. Rexroth CKR and Bahr: stroke to module length

### Rexroth CKR
Source: Bosch Rexroth, "Compact modules CKK/CKR", R999000499/2025-05 (https://www.boschrexroth.com/media/e8278bcd-f4c8-4191-814e-9e557f6c5f27).
- "Technical data / General technical data", printed p.64: columns Carriage Lca (without / with connection plate), Additional length Lad (without / with), smin, Lmax. Values (mm), "without connection plate" version, which the repo uses:

| Size | Lca | Lad | Lca (with plate) | Lad (with plate) | smin | Lmax |
|---|---|---|---|---|---|---|
| 070 | 80 | 10 | 60 | 30 | 40 | 1500 |
| 070L | 108 | 10 | 95 | 23 | 40 | 1500 |
| 090 | 102 | 25 | 60 | 67 | 40 | 5500 |
| 090L | 156 | 25 | 125 | 56 | 40 | 5500 |
| 110 | 170 | 25 | 110 | 85 | 50 | 5500 |
| 110L | 215 | 25 | 155 | 85 | 50 | 5500 |
| 145 | 180 | 25 | 125 | 80 | 60 | 5500 |
| 145L | 240 | 25 | 190 | 75 | 60 | 5500 |
| 200 | 265 | 25 | 190 | 100 | 80 | 10000 |
| 200L | 405 | 25 | 305 | 125 | 80 | 10000 |
| 280 | 485 | 25 | 375 | 135 | 195 | 5500 (printed "5, 500") |

  Cross-check: Lca + Lad is identical with and without plate in every row (e.g. 110: 170+25 = 110+85), which confirms the column assignment. Confidence high.
- Length rule, "Project planning / Calculation principles / Length calculation", printed p.176: "L = seff + se + Lca + Lad" (CKR-280-DB adds Lw), with "seff = smax - 2 x se" and se the stroke reserve (recommended 20 mm for linear systems; the figure shows se/2 at each end). With se = 0: L = smax + Lca + Lad, which is what the repo's meta states. Note the formula as printed in the text layer reads "seff + se", not "seff + 2 se"; I did not see a visual rendering, so check the figure if a nonzero reserve is ever modelled. Weight rule: ms = kg fix + kg var x L + mca (p.177).

### Bahr Modultechnik (Norgren data sheets)
Source: data sheets "Linearsystem ELZ ..." etc., e.g. https://cdn.norgren.com/pdf/de_21_1_100_ELZ.pdf (sheet 21.1.100, 11/25, (c) 2023 Bahr Modultechnik GmbH), page 2 "Dimensionen (mm)", ordering key line.
- Rule printed in every sheet checked (all 15 sheet files named in catalog/bahr/belt_axes.csv contain the line): "Grundlänge + Verstellweg = Gesamtlänge" (basic length + travel = total length). Order example: ELZ 40, travel 1275 mm, key "1500" = 225 + 1275 = 1500. Also "Bei Verlängerung der Schlittenlänge erhöht sich die Grundlänge um die Verlängerung" (a longer carriage adds its extra length to the basic length).
- Basic length ("Grundlänge", dimension L) of the standard carriage, ELZ sheet, mm: 30: 158, 40: 225, 60: 290, 60S: 315, 80: 375, 80S: 395, 100: 530, 125: 625. Max carriage length 450 (30/40/60), 650 (60S), 950 (80/100/125).
- Seen in other sheets (column "Grund-länge"/"Basic length"): ELHZ 60 290, 80 315, 100 375; ELSZ 30 182, 40 230, 60 280, 60S 305, 80 365, 80S 365, 100 535, 125 595; ELSD 40 260, 60 320, 60S 345, 80 415, 80S 415, 100 585; ELVZ 60 290, 60S 315, 80 375, 80S 395, 100 530, 125 630 (a second ELVZ table, presumably a longer-carriage or double version, prints 460/510/570/610). The ELZ-W, ELZD-W, ELZT, ELZZ, DLSZ, DLVZ, DLZPVI, LLZS, ELZU, ELZI sheets carry the same rule; I did not tabulate their basic lengths. Confidence: ELZ high (read directly), the others medium (read from a text layer with merged columns; verify before use).

## 3. KUKA KRL CIRC with missing components

Source (primary KUKA document, third-party hosted copy): "KUKA System Software 8.3, Operating and Programming Instructions for System Integrators", KSS 8.3 SI V4, issued 14.01.2015, section 11.5.3 "LIN, CIRC", printed p.369-370 of 491. Copy: https://www.mroelectric.com/static/app/product/pdfs/00-291-556.pdf
- End point: "If not all components of the end point are specified, the controller takes the values of the previous position for the missing components." (p.370)
- Auxiliary point: same sentence for its missing components. In addition: "The orientation angles and the Status and Turn specifications for an auxiliary point are always disregarded." The aux point cannot be approximated. Coordinates are in BASE.
- So: previous position (the start point of the CIRC), not the auxiliary point's value and not necessarily "current". Only if no previous position is known to the controller (SPTP/SCIRC text, 11.6.3 p.378 and 11.6.5 p.381) the current robot position is used. For SLIN/SCIRC with a previous circle that had a circular angle (CA), the value of the end point actually reached is used outside a spline block, the programmed one inside.
- Contrast: CIRC_REL/LIN_REL (11.5.4, p.370-371): missing components are set to 0 (the absolute value stays unchanged); the aux point is read relative to the current position.
- Confidence: high for 8.3; other 8.x editions not checked.

## 4. Festo cylinder cushioning energy (DSBC-32, as in machines/pneumatic-axis.toml)

Repo uses a DSBC-32-100-PPVA-N3 and rated_cushion_energy_j = 0.4.
Source: Festo, "Standards-based cylinders DSBC, to ISO 15552", Datasheet, 2026/09, printed p.13, table "Forces [N] and impact energy [J]" (https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2026/EN/DSBC_EN.PDF)
- "Max. impact energy in the end positions", row DSBC-...: 32: 0.4 J (footnote: 0.1 J with trunnion kit DAMT), 40: 0.7, 50: 1.0, 63: 1.3, 80: 1.8, 100: 2.5, 125: 3.3. Row -L/-U/-T1/-T3/-T4: 0.2/0.35/0.5/0.65/0.9/1.25/1.65; -L1: 0.1/0.2/0.3/0.4/0.9/1.25/1.65; -KE: 0.32/0.56/0.8/1/1.4/2/2.6. The catalogue does not say which cushioning (P, PPV, PPS) the row is for.
- Definition: the vendor gives a maximum permissible kinetic energy at the end position, with "Permissible impact velocity v = sqrt(2 E / (m1 + m2))" and "Maximum permissible mass m2 = 2 E / v^2 - m1" (m1 moving mass of the drive, m2 moving payload; same page). Only 1/2 m v^2 is used: NO weight term, NO driving-force term, no mention of vertical or inclined mounting in this cushioning rule. Elsewhere (p.18 sizing example) vertical load is handled for the holding force only (50 % of theoretical force rule).
- A rule that DOES add weight and force exists, but for external hydraulic shock absorbers (YSR/DYSR), not cylinder cushioning: Festo, "Calculation aid for cushioning components", Data sheet 2024/05, p.2: A = F + m g sin(alpha) (F = cylinder force minus friction; alpha = 90 deg downward gives G = +m g, upward -m g), "Wtotal = 1/2 m v^2 + A s < Wmax", s = absorber stroke; v on impact should be taken as 1.25 ... 2 x mean speed (factor 2 for strokes < 50 mm, 1.5 for 50-100 mm, 1.25 for > 100 mm). Copy read: https://www.motionworld.com/assets/festo-dysd-series-cushioning-calculation-aid.pdf (the festo.com copy returned 403).
- Conclusion: for DSBC-style cylinders Festo's published check is kinetic energy at the cushion entry vs E_max. Adding m g s_cushion + F s_cushion (the shock-absorber formula) would be our extrapolation, not Festo's cylinder rule. Confidence: high on the DSBC text; medium on whether Festo's online sizing tool does more (not checked).

## 5. G-code feed with rotary axes

### Siemens SINUMERIK (primary)
Source: "SINUMERIK 840D sl / 828D Fundamentals, Programming Manual", 08/2018, 6FC5398-1BP40-6BA2, CNC SW 4.8 SP3, section 8.1 "Feedrate (G93, G94, G95, F, FGROUP, FL, FGREF)", printed p.107-114. https://cache.industry.siemens.com/dl/files/800/109760800/att_962405/v1/840Dsl_828D_fundamentals_progr_man_0818_en-US.pdf
- F acts on the path axes in FGROUP. Default FGROUP is the geometry axes (FGROUP() restores it, p.112). A rotary axis not in FGROUP is a synchronized axis (time-synchronous, no F of its own).
- FGREF[rotary axis] = effective reference radius. "To ensure compatibility ... the factor 1 degree = 1 mm is activated on system power up and RESET", i.e. default radius 360 mm/(2 pi) = 57.296 mm, "independent of ... MD10240 and the currently active G70/G71/G700/G710" (p.113). Inch: 57.296/25.4.
- Rotary axis only, with the FGREF default: F is degrees/min (p.114 notice: "FGREF evaluation also works if only rotary axes are programmed ... normal F interpretation as degree/min applies ... only if the radius reference corresponds to the FGREF default"). With another FGREF the same F means path speed at that radius.
- Linear + rotary together, both in FGROUP: path length is built from linear mm and angle x FGREF (default 1 deg = 1 mm), and F is that path feed. Printed example (p.107-108, FGROUP(X,A), G91 G710 F100): X10 = 100 mm/min, 10 mm, about 6 s; X10 A10 = path 14.14 mm, about 8 s; A10 alone = 100 deg/min, 10 deg, about 6 s. With FGREF[A]=360/(2 pi) in inch mode the page shows "1 degree = 1 inch".
- Special case printed on p.113-114 (FGROUP(X,Y,Z,A), G1 G91 A10 F100 then A10 X0.0001 F100): rotary-only block F = 100 deg/min, block with a tiny linear move F = 100 mm/min (or inch/min).
- G93 (inverse time) is separate. Confidence: high.

### Fanuc
- I could NOT retrieve the primary feed-function section. The Fanuc manuals I could fetch were B-64304EN-2/01 (0i-D MC), B-64484EN-2/03 (30i/31i/32i-B MC, from servicetex.ru) and a 0i-F lathe manual; their extracted text lacks the "Feed functions" chapter. Only secondary material: forum and tutorial pages say F is deg/min for a rotary-only block and a mixed block treats degrees as units like mm (G94). The parameter lists in B-64484EN-2/03 give "mm/min, inch/min, degree/min (machine unit)" for feedrate parameters, and the 0i-D list gives a rotation-axis unit of deg/min; that supports the unit statement for rotary axes but does not give the mixed-block rule. Verdict: unverified from primary; do not cite a Fanuc page.

## 6. Linear transport system mover (repo's default.toml cites Beckhoff XTS AT2000, AT9014, AT9001)

- Beckhoff product page AT9014-x070-x550 (https://www.beckhoff.com/en-en/products/motion/xts-linear-product-transport/at8x00-at9xxx-ath9xxx-xts-guide-rails-and-movers/at9014-x070-x550.html):
  - Technical data "Weight 680 g" (for the page's technical-data table); ordering text: "mover, spring-loaded, length 70 mm, 651 g, mounted with magnetic plate set AT9001-0550" (the -0550/-1550 variants) and 680 g for the NCT-suitable -1070 variants. So 651 g vs 680 g are both on the page, depending on the variant. The repo's 651 g matches the standard variant.
  - "Mover length 70 mm", min. magnet pitch 72 mm, min. distance between two movers 2 mm. Force: peak 100 N, continuous 30 N for AT9001-x550 (5-pin), straight sections. Life test "30,000 km with 600 g at 1.5 m/s". No payload rating on the page itself (the 600 g is a test load).
- Centre of gravity of the mover relative to guide rail/motor: NOT found anywhere I could read. Payload offset or centrifugal limit curves: NOT found (neither on the product page, the XTS Operating Instructions technical data page, nor the start-up manual page 22).
- Beckhoff infosys XTS operating instructions "Technical data" (https://infosys.beckhoff.com/content/1033/xts_ba/12669242251.html): offsets to the direction of travel lateral <= 0.1 mm and in height <= 0.1 mm "when using a Beckhoff guide rail"; "Maximum linear axis torque": Mx 50 Nm, My 15 Nm, Mz 10 Nm, drive axis 6 Nm (applies to standard and NCT modules); air gap 0.5 / 1 / 3 mm (1 mm max for EcoLine). The page does not say these Mx/My/Mz limits belong to the AT9014 specifically, so treat them as system-level, medium confidence.
- Beckhoff news (secondary): single mover up to 20 kg in a circular transport system, up to 100 kg with certain configurations; not an AT9014 limit.
- The older XTS Technical description (manualslib copy, v1.2, AT9011 generation) mentions about 350 g, which is not the AT9014.
- Verdict: the mass (651 g) is sourced; COG and offset or centrifugal curves would need the XTS documentation set (mover "load" diagrams) or Beckhoff support; no primary number found.

## 7. Servo drive on a missed cyclic setpoint

- No primary text found stating extrapolate, hold or fault for missed position setpoints with DSC in S120/S210. What I could source:
  - S120 Function Manual "Drive functions", 06/2020, 6SL3097-5AB00-0BP3, section 5.19 "Dynamic Servo Control (DSC)": DSC needs isochronous operation; with DSC the position setpoints/extended setpoint channel are not used; the function module "DSC with spline" (p0108.6 = 1) interpolates the position setpoint with a spline for smooth speed/torque precontrol (telegrams 125, 126, 136, 138, 139, 146, 148, 149); otherwise linear interpolation of the position. This is interpolation between received setpoints, not behaviour on a missing one.
  - Sign-of-life tolerance: SINAMICS G130/G150 List Manual (LH2), 07/2016, A5E03263479A, p0925 "PROFIdrive clock synchronous sign-of-life tolerance": "Sets the number of tolerated consecutive sign-of-life errors of the clock-cycle synchronous master"; monitoring off at p0925 = 65535; dependency F01912 (cause text: maximum permissible number of errors in the controller sign-of-life exceeded in cyclic operation; remedy: check bus, p2045, telegram failure rate p0925). That is the same parameter family as S120 (the S120 Drive functions manual lists p0925 "PROFIdrive isochronous sign-of-life tolerance" in the central-probe/isochronous parameter list). Source copy: https://www.manualslib.com/manual/1313634/Siemens-G130.html?page=292 (list manual page 292). Confidence on p0925 and F01912: medium-high (G130/G150 list manual, not the S120/S210 one).
  - I did not find what the drive applies to the position setpoint during the tolerated misses (last value, extrapolation) in any Siemens document I could read. A blog claim that the drive keeps the last velocity command is unverified. Not found at all: S210-specific text, CiA 402 CSP/interpolated-position text (IEC 61800-7, paywalled).

## 8. Stepper motors and drives in catalog/

- The repo's catalog/ has NO stepper vendors: vendors present are apex, bahr, beckhoff, br, festo, gates, harmonicdrive, hiwin, kollmorgen, lenze, megadyne, mitsubishi, neugart, rexroth, sew, siemens, wittenstein, yaskawa (grep for "stepper" in catalog finds nothing). So no primary check against a catalogued vendor is possible.
- General (secondary only): most motor datasheets quote rated phase current as RMS ("amps/phase", peak = 1.414 x RMS) per Lin Engineering (https://linengineering.com/news/amps-phase-vs.-amps-peak) and AMCI (https://www.amci.com/tutorials/tutorials-stepper-motor-drivers-rms-vs-peak-current.asp); drivers may specify RMS, peak or full-scale (TI), so each driver's datasheet must be read. Analog Devices/Trinamic QSH datasheets and Nanotec/Oriental Motor conventions were not read. Confidence: medium, secondary sources.

## 9. Derating of S210 converters and S120 booksize Motor Modules (`catalog/siemens/derating_s210.csv`, `derating_s120.csv`)

Documents (Siemens' own, cache.industry.siemens.com; the PDFs are not committed, figures only):
- S210OI: SINAMICS S210 servo drive system with S-1FK2 and S-1FT2, Operating Instructions, 09/2022, A5E41702836B AL (https://cache.industry.siemens.com/dl/files/184/109801184/att_1118289/v1/S210_S-1FK2_S-1FT2_op_instr_0922_en-US.pdf).
- D32: catalogue D 32, SINAMICS S210 servo drive system, January 2024 (https://cache.industry.siemens.com/dl/files/381/109754381/att_1261452/v1/motion-control-drives-D32-complete-English-2024-01.pdf).
- D21.4N: Catalog News D 21.4 N, SINAMICS S120 and SIMOTICS, October 2017 (https://cache.industry.siemens.com/dl/files/875/109751875/att_934896/v1/motion-control-drives-D21-4-N-complete-English-2017-10.pdf).
- CD: SINAMICS S120 Power Units in Booksize C/D-Type Format, Manual, 01/2017, 6SL3097-4AC20-0BP1 (https://cache.industry.siemens.com/dl/files/808/109744808/att_909444/v1/s120_booksize_cd_type_equip_man_0117_en-US.pdf).
- Not used: GH2 12/2004 (6SL3097-2AC00-0BP1; its derating figures are garbled in the PDF text; it only confirms 4 kHz without and 16 kHz with derating). A current GH2 for the C/D-type article numbers was not found on Siemens' cache server.

S210 (both the 6SL3210-5H.. "S210" and the 6SL5310-1B.. "S210 (New)" converters; D32 says its technical data hold for all S210 servo converters):
- Altitude and ambient temperature together, S210OI Table 12-9, printed p.517 (PDF p.519), "Maximum permissible output current depending on the installation altitude and ambient temperature", output current in % at 50 / 45 / 40 degC: up to 1000 m 100/100/100; up to 2000 m 90/100/100; up to 3000 m 80/90/100; up to 4000 m 70/80/90. D32 p.2/10 says the same in words: up to 1000 m none, from 1000 m 10 % of the current or 5 K per 1000 m, at most 4000 m (CSA 2000 m); from 2000 m a 1 AC converter needs an isolating transformer.
- Inferred cells: the PDF's text layer merges cells (the 1000 m row shows one "100"; at 2000 m the 45 degC cell is empty). The 1000 m row and 2000 m at 45 degC are read as 100 % from the layout and the rule; the page was not rendered. Marked `inferred` in the table's `basis`.
- Ambient 0 ... 50 degC (D32 p.2/10; S210OI 12.2.4). The rated currents' own reference temperature is not stated in so many words (the 100 % column at 2000 and 3000 m is 40 degC).
- Pulse frequency fixed at 8 kHz (S210OI 12.2.5, printed p.518; D32 p.2/12 ff.); no derating by it, no setting found.
- Not found: whether the table applies to the maximum output current too.

S120 booksize Motor Modules (D21.4N p.7/8, 7/14, 7/23; CD printed p.36 and p.65):
- Altitude: none up to 1000 m, above up to 4000 m either the output current reduced by 10 % per 1000 m or the ambient temperature by 5 K per 1000 m. The CD manual's table row also prints "3.5 K per 500 m" (7 K per 1000 m) beside the 5 K: the two disagree in the same table, unresolved; quoted both. The simulator implements the current reduction (the first, current-based alternative), never both at once. Above 2000 m a line supply with a grounded neutral or an isolating transformer. Only a rule, no per-module table: 0.9 / 0.8 / 0.7 at 2000 / 3000 / 4000 m are derived.
- Ambient: none up to 40 degC, above that up to 55 degC 2.67 % per K (CD p.36); the curves (D21.4N p.7/14 Single, p.7/23 Double) run from 100 % at 40 degC to about 60 % at 55 degC (measured 60.6 %).
- Pulse frequency, rated 4 kHz, curve by module group (D21.4N p.7/14; Double p.7/23; CD Fig. 3-20 p.65, a JPEG read by eye): 3 A to 132 A and Double: 100 % to 4 kHz, 60 / 40 / 30 % at 8 / 12 / 16 kHz; 200 A: 100 % to 4 kHz, a straight line to 40 % at 12 kHz (about 70 % at 8 kHz, not a vertex), 30 % at 16 kHz. Vertices from the vector graphics, error about 0.5 %. At 16 kHz the ambient temperature must not exceed 45 degC (CD p.65). The 30 A S6 cycle holds only up to 8 kHz (CD p.63), not modelled.
- Not stated by Siemens: whether the factors multiply (the documents say only that the current derating by pulse frequency and ambient temperature must be taken into account; altitude and temperature are alternatives). The simulator multiplies the three, an assumption on the safe side. And whether the derating applies to the maximum output current too (the axes read "Permissible output current %"); the simulator derates the rated and the base load current and leaves the maximum current as printed.
- Related, not used: S210OI 12.1.5 "Derating factors" (printed p.360) is the MOTOR's derating by altitude and ambient, not the converter's.

## 11. Linear guides of Bosch Rexroth and THK, and the bases of C (catalog/rexroth/linear_guides.csv, catalog/thk/linear_guides.csv)

- Bosch Rexroth, "Ball rail systems, High Precision BSHP", R999000485/2025-11 (https://www.boschrexroth.com/media/ec1fd87f-8f69-4a5e-9596-e3f0089bc0f9): every dimension table's footnote: C, Mt and ML "based on a travel of 100,000 m according to DIN ISO 14728-1. Often only 50,000 m are actually stipulated. For comparison: Multiply the values C, Mt and ML by 1.26". The factor is 2^(1/3) = 1.2599 (ball guides: life goes with C cubed). Table: sizes 15 to 45 pages 51 to 73 (block types FNS, FLS, FKS, SNS, SLS, SKS, SNH, SLH, FNN, FKN, SNN, SKN), sizes 55 and 65 pages 75 to 85; rail mass page 113.
- The same document, page 20: Mt0 and Mt are moments "around the x-axis" (the roll), ML0 and ML "around the transverse axis y or the vertical axis z" (pitch and yaw): one number for both, so both columns carry it. Dynamic moments Mt and ML are printed, for 100 km as C is.
- THK, LM Guide General Catalog (https://tech.thk.com/en/products/pdf/en_a01_060.pdf, page A1-67): C "based on a reference distance of 50 km for an LM Guide with balls and 100 km for an LM Guide with rollers"; the page converts C100 = C50 / 1.26 for balls (1.23 for rollers). SHS and HSR are ball guides, so THK's C is on HIWIN's basis (50 km), Rexroth's is not.
- THK's static moments MA (pitching), MB (yawing) and MC (rolling) are named on page A-8 of the General Catalog's "A Technical Descriptions of the Products" (figure legend: "MA Moment in the pitching direction, MB ... yawing, MC ... rolling"); the dimension tables (SHS A1-99, A1-101, A1-103; HSR A1-193) print them per block and for two blocks in close contact, and MC for one block only. THK prints no dynamic moments, no seal resistance and no preload force on those pages.
- THK's whole-catalogue link (E_01_LMGuide.pdf on tech.thk.com) answers an HTML page, not a PDF; the catalogue is only available in chunks named by their first page, SHS under `/pdf/` and HSR under `/pdfs/` (the directory differs).
- Not found: seal resistance of Rexroth blocks as a number (only a diagram of the displacement resistance), a preload force of either maker.

## 12. Festo ELGA-TB variants (belt_axes_elga.csv) and what is left out

- ELGA-TB, 2026/02 (https://ftp.festo.com/Public/PNEUMATIC/SOFTWARE_SERVICE/Documentation/2026/EN/ELGA-TB_EN.PDF): KF-F1 pages 32 to 34 (loads "for a service life of 5000 km", as KF), RF pages 49 to 51 and RF-F1 pages 66 to 68 (loads "for a service life of 10000 km"; the 5000 km values are not printed for them). RF sheets print one row per slide (ELGA-..., -S, -L) and the T8 option (feed force, driving torque, no-load torque): the standard slide is the row taken.
- The catalogue stores the printed load with its distance (`dynamic_load_distance_m`), it does not convert. Festo gives no conversion rule between 5000 and 10000 km for the roller guide; the 100 km comparison values (KF p. 17) are for comparing with ISO and JIS ratings of other makers and "must not be used for the drive".
- ELGC-TB (2026/02): not extracted. Its sheet prints units inside the cells ("75 N 120 N 250 N") and merged cells for belt pitch and speed that can only be assigned to sizes by their position, and a stroke list whose size-60 entry is unclear.
- Spindle axes (EGC-BS, ELGA-BS, ELGC-BS): the belt-axis table does not fit (a screw lead, no belt).

## 13. Autonox (checked 25 November 2026, no registration)

- Public: autonoxfinder.com (Shopware shop "autonox Finder") product pages show a properties tab without login: kinematics, degrees of freedom, nominal payload, working area, pose repeatability (ISO 9283), bearing type, mounting position and the robot mass without drive technology; each product has a "Connecting dimensions (PDF)" and a "3D-model (PDF)". The tabular product overview lists all mechanics with article numbers.
- Behind a login ("password-protected download area" of each product page): technical data sheets (load capacity diagrams), operating instructions, gearbox manuals; prices on request. No arm lengths, link masses or inertias in anything public; they could only be measured off the connecting-dimension drawings.
- Autonox sells robot mechanics only (DELTA, SCARA, articulated): no linear axes.
