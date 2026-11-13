"""Some of a vendor glTF model's parts as one part's mesh for the app's 3D view (`cad_mesh` in
crates/sim-app/src/main.rs), keeping the vendor's colours. The glTF counterpart of step_to_glb.py.

    python glb_part.py <in.glb> list
    python glb_part.py <in.glb> <out.glb> nodes=1,2,3 origin=x,y,z [turn=deg]

"list" prints each of the root's child nodes with its box in the file's own axes, in mm. nodes picks those
children. origin is the point in the file's axes, in mm, that becomes the part's reference point in the
simulation. turn turns about the vertical (y) axis, in degrees, so that x runs along the beam, as the app's
3D view has it (scene x right, scene z up: glTF x, y). Only the root's matrix is replaced; the meshes,
materials and buffers stay as they are, and gltfpack then drops what no node uses:

    gltfpack -noq -i <out.glb> -o crates/sim-app/assets/meshes/<config name>/<part id>.glb
"""

import json
import math
import struct
import sys

IDENTITY = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]


def product(a, b):
    """Two column-major 4x4 matrices multiplied, as glTF stores them."""
    return [sum(a[r + 4 * k] * b[k + 4 * c] for k in range(4)) for c in range(4) for r in range(4)]


def apply(m, p):
    return [m[r] * p[0] + m[4 + r] * p[1] + m[8 + r] * p[2] + m[12 + r] for r in range(3)]


def read(path):
    data = open(path, "rb").read()
    if data[:4] != b"glTF":
        sys.exit(f"glb_part: {path} is not a binary glTF file")
    length = struct.unpack("<I", data[12:16])[0]
    return json.loads(data[20 : 20 + length]), data[20 + length :]


def write(path, gltf, rest):
    text = json.dumps(gltf).encode()
    text += b" " * (-len(text) % 4)
    head = struct.pack("<4sII", b"glTF", 2, 12 + 8 + len(text) + len(rest)) + struct.pack("<I4s", len(text), b"JSON")
    open(path, "wb").write(head + text + rest)


def box(gltf, root_matrix, index):
    """The node's box in the file's axes, in mm, from its meshes' position bounds."""
    node = gltf["nodes"][index]
    m = product(root_matrix, node.get("matrix", IDENTITY))
    corners = []
    for primitive in gltf["meshes"][node["mesh"]]["primitives"] if "mesh" in node else []:
        a = gltf["accessors"][primitive["attributes"]["POSITION"]]
        corners += [apply(m, [x, y, z]) for x in (a["min"][0], a["max"][0]) for y in (a["min"][1], a["max"][1]) for z in (a["min"][2], a["max"][2])]
    if not corners:
        return None
    return [round(1000 * min(c[i] for c in corners)) for i in range(3)], [round(1000 * max(c[i] for c in corners)) for i in range(3)]


def main(args):
    if len(args) < 2:
        sys.exit(__doc__)
    gltf, rest = read(args[0])
    scene = gltf["scenes"][gltf.get("scene", 0)]
    if len(scene["nodes"]) != 1:
        sys.exit("glb_part: expects one root node, as the vendors' files have")
    root = gltf["nodes"][scene["nodes"][0]]
    if "matrix" not in root and any(k in root for k in ("translation", "rotation", "scale")):
        sys.exit("glb_part: the root node is placed by translation/rotation/scale; only a matrix is read")
    root_matrix = root.get("matrix", IDENTITY)
    children = root.get("children", [])
    if args[1] == "list":
        for index in children:
            print(index, gltf["nodes"][index].get("name", ""), box(gltf, root_matrix, index))
        return
    options = dict(a.split("=", 1) for a in args[2:])
    unknown = set(options) - {"nodes", "origin", "turn"}
    if unknown or "nodes" not in options or "origin" not in options:
        sys.exit(__doc__)
    picked = [int(n) for n in options["nodes"].split(",")]
    missing = [n for n in picked if n not in children]
    if missing:
        sys.exit(f"glb_part: nodes {missing} are not children of the root; see list")
    x, y, z = (float(v) / 1000 for v in options["origin"].split(","))
    turn = math.radians(float(options.get("turn", "0")))
    c, s = math.cos(turn), math.sin(turn)
    about_y = [c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, 0, 0, 0, 1]
    to_origin = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -x, -y, -z, 1]
    root["matrix"] = product(about_y, product(to_origin, root_matrix))
    root["children"] = picked
    # gltfpack keeps every mesh some node names, used or not: unhook the meshes of the nodes left out.
    for index in children:
        if index not in picked:
            gltf["nodes"][index].pop("mesh", None)
    write(args[1], gltf, rest)
    boxes = [b for b in (box(gltf, root["matrix"], n) for n in picked) if b]
    low = [min(b[0][i] for b in boxes) for i in range(3)]
    high = [max(b[1][i] for b in boxes) for i in range(3)]
    print(f"glb_part: {len(picked)} nodes, x {low[0]}..{high[0]}  y {low[1]}..{high[1]}  z {low[2]}..{high[2]} mm -> {args[1]}")


if __name__ == "__main__":
    main(sys.argv[1:])
