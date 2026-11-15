( A G-code program for the T-gantry, read by --path from its extension: the corner of corner.path in mm, )
( X along the beam and Z up, so the arc lies in the X-Z plane (G18, centre offsets I and K from the start). )
( F is the speed along the path in mm/min. See docs/user-guide.md, "Programs: G-code and robot programs". )
G21 G90 G18
G1 X400 Z350 F12000
G1 X500
G2 X600 Z250 I0 K-100
G1 Z100 F6000
G0 X300 Z300
M30
