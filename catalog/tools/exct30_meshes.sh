#!/bin/sh
# The CAD meshes of crates/machine-t-gantry/exct-30.toml for the app's 3D view, from the vendors' downloads
# (kept out of git, as their terms ask; see `cad_mesh` in crates/sim-app/src/main.rs):
#
#   cad-source/festo-exct30/8026576_EXCT-30-1000-250-KF-W-HH-R-T0-DE.glb   Festo CAD portal, glTF
#   cad-source/siemens-motor/1FK2205-4AF10-0MA0-Z_B22+M01+R15_3D_de.STP     Siemens CAx download, STEP
#
# Needs python, gltfpack and FreeCAD (FREECADCMD, default: where winget puts it). Run from the repository root.
#
# The Festo model is in mm, y up, the beam along -z, the front towards +x; its carriage stands at X = 0 and
# its tool plate 66 mm above the lowest Z. In it the drive heads are 1354 mm apart (span_min_m = 0.177) and
# the Z column's centre plane is at x = 79 mm, where the simulation's belt plane (scene y = -0.05) lies.
# Each origin below is where the simulation's part sits in that pose, in the Festo model's axes:
#   beam      end / 2 along the beam, 45 mm behind the belt plane: (34, 0, -626)
#   carriage  its centre, 20 mm behind the belt plane:              (99, 0, -122)
#   z-axis    the column's middle, z - 0.125 = -59 mm:              (129, -59, -122)
#   clamp     on the column top, span + z + 0.015 = 258 mm:         (129, 258, -122)
# Left out: the two cable chains (nodes 9 and 29 to 31), whose loops would not follow the motion.
# The motor model has a planetary gearbox in front of the flange; the EXCT-30 is driven directly, so only
# the motor's own parts are kept. Its flange face sits on the drive head's back, 58 mm in front of the
# simulation's motor part.
set -e
FREECADCMD="${FREECADCMD:-$LOCALAPPDATA/Programs/FreeCAD 1.1/bin/freecadcmd.exe}"
festo=cad-source/festo-exct30/8026576_EXCT-30-1000-250-KF-W-HH-R-T0-DE.glb
motor="cad-source/siemens-motor/1FK2205-4AF10-0MA0-Z_B22+M01+R15_3D_de.STP"
out=crates/sim-app/assets/meshes/exct-30
work=$(mktemp -d)
mkdir -p "$out"

part() {  # part id, nodes, origin
    python catalog/tools/glb_part.py "$festo" "$work/$1.glb" "nodes=$2" "origin=$3" turn=-90
    gltfpack -noq -i "$work/$1.glb" -o "$out/$1.glb"
}
part beam 1,24,25,26,27,28,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51 34,0,-626
part carriage 2,20,21,22,23 99,0,-122
part z-axis 3,6,7,8 129,-59,-122
part clamp 4,5,10,11,12,13,14,15,16,17,18,19 129,258,-122

"$FREECADCMD" catalog/tools/step_to_glb.py --pass "$motor" "$work/motor.glb" \
    keep=0617,0661,0713,0754,0786,0833,0885,0916,0963,0969,0970,0972,0974 origin=0,58,0 | grep step_to_glb
# FreeCAD meshes its round faces into some 250 000 triangles: a tenth still shows every edge in the view.
gltfpack -noq -si 0.1 -i "$work/motor.glb" -o "$out/motor.glb"
rm -r "$work"
