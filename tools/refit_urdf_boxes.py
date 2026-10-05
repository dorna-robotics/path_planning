"""Re-fit the URDF's collision boxes to the robot's link meshes.

    cd ~/Downloads/workspace/workspace && sudo python3 ~/Downloads/path_planning/tools/refit_urdf_boxes.py
    cd ~/Downloads/path_planning && sudo cmake --install build/rpi-arm64      # ships resources/urdf

What it does, per link of ``resources/urdf/dorna_ta.urdf``:
  * decodes the link's mesh (``workspace/static/CAD/robot_A*.glb`` + ``.bin``, Draco-compressed glTF,
    the same files the viewer draws) and expresses it in the PLANNER's link frame, through the scene
    tree's solid frame and the planner's own FK at the zero pose (``Planner.link_frames``);
  * keeps the authored boxes' COUNT and ORIENTATION (they encode which part each box covers), assigns
    every mesh point to the nearest box, and re-fits each box's centre and size to its points (MARGIN per
    side, 0: exactly on the mesh); three rounds settle it;
  * the base link's boxes start FLOOR above the mounting plane (z = 0 of j0_link, where the casting
    is bolted to the carriage) so the base never "collides" with what it stands on;
  * reports coverage (mesh points inside the union of boxes) and writes the URDF in place.

Needs: DracoPy (``sudo pip3 install DracoPy --break-system-packages``), a scene with the core
(examples/base is used), the workspace package on the path.
"""
import json, os, re, sys, yaml, tempfile
import numpy as np

MARGIN = 0.0        # mm, each side: the boxes sit exactly on the mesh — the planner's own padding (bench boxes) is the only margin
FLOOR = 1.0         # mm, the base link's boxes start this far above its mounting plane
HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "resources", "urdf", "dorna_ta.urdf")
WS = os.path.normpath(os.path.join(HERE, "..", "..", "workspace", "workspace"))   # the sibling checkout; not ~ (sudo makes that /root)
CAD = os.path.join(WS, "static", "CAD")
EX = os.path.join(os.path.dirname(WS), "examples", "base")
PAIRS = [("robot_A0", "j0_link"), ("robot_A1", "j1_link"), ("robot_A2", "j2_link"), ("robot_A3", "j3_link"),
         ("robot_A4", "j4_link"), ("robot_A5", "j5_link"), ("robot_flange", "j6_link")]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def node_T(n):
    if "matrix" in n:
        return np.array(n["matrix"]).reshape(4, 4).T
    S = np.diag(list(n.get("scale", [1, 1, 1])) + [1]); R = np.eye(4); R[:3, :3] = quat_to_R(n.get("rotation", [0, 0, 0, 1]))
    T = np.eye(4); T[:3, 3] = n.get("translation", [0, 0, 0]); return T @ R @ S


def mesh_points(name):
    import DracoPy
    g = json.load(open(f"{CAD}/{name}.glb")); binary = open(f"{CAD}/{name}.glb.bin", "rb").read(); pts = []

    def walk(ni, parent_T):
        n = g["nodes"][ni]; T = parent_T @ node_T(n)
        if "mesh" in n:
            for prim in g["meshes"][n["mesh"]]["primitives"]:
                ext = prim.get("extensions", {}).get("KHR_draco_mesh_compression")
                if ext is None:
                    raise RuntimeError(f"{name}: a primitive is not Draco-compressed; extend the loader")
                bv = g["bufferViews"][ext["bufferView"]]; blob = binary[bv.get("byteOffset", 0): bv.get("byteOffset", 0) + bv["byteLength"]]
                P = np.asarray(DracoPy.decode(blob).points, dtype=float)
                pts.append((T @ np.c_[P, np.ones(len(P))].T).T[:, :3])
        for c in n.get("children", []):
            walk(c, T)
    for root in g["scenes"][g.get("scene", 0)]["nodes"]:
        walk(root, np.eye(4))
    return np.vstack(pts)


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]); Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def link_frames_at_zero():
    """{planner link: (T_link_from_solid, solid name)} — the constant relation between the scene
    tree's solid frames and the planner's link frames, read at the zero pose with the planner's base
    where the core puts it (Core._planner_base)."""
    sys.path.insert(0, WS)
    from workspace.workspace import Workspace
    from workspace.j2 import render_file
    d = tempfile.mkdtemp(prefix="refit_"); files = []
    for rel in yaml.safe_load(open(os.path.join(EX, "launch.yaml")))["scene"]:
        out = os.path.join(d, os.path.basename(rel).replace(".j2", ".yaml"))
        yaml.safe_dump(yaml.safe_load(render_file(os.path.join(EX, rel))), open(out, "w"), sort_keys=False); files.append(out)
    ws = Workspace(config_path=files, port=8999, project_dir=EX); core = ws.components["core"]
    J = [0] * 8; core.robot_api.joint = lambda: list(J); core._last_joints = None; ws.compute_collision_boxes(0.0); core.check_collision(J, True)
    fr = core.planner.link_frames(J)
    out = {}
    for solid, link in PAIRS:
        G = fr[link].copy(); G[:3, 3] *= 1000.0
        out[link] = (np.linalg.inv(G) @ np.array(core.assembly[solid]._world_T, dtype=float), solid)
    return out


def main():
    frames = link_frames_at_zero()
    s = open(URDF).read(); new_s = s
    for m in re.finditer(r'<link name="(\w+)">(.*?)</link>', s, re.S):
        link, body = m.group(1), m.group(2)
        cols = list(re.finditer(r'<collision>\s*<origin xyz="([^"]+)" rpy="([^"]+)"/>\s*<geometry>\s*<box size="([^"]+)"/>\s*</geometry>\s*</collision>', body, re.S))
        if not cols or link not in frames:
            continue
        T_link_from_solid, solid = frames[link]
        P = mesh_points(solid); L = (T_link_from_solid @ np.c_[P, np.ones(len(P))].T).T[:, :3]
        boxes = []
        for c in cols:
            xyz = np.array([float(v) for v in c.group(1).split()]) * 1000; rpy = [float(v) for v in c.group(2).split()]
            size = np.array([float(v) for v in c.group(3).split()]) * 1000
            boxes.append({"c": xyz, "R": rpy_to_R(*rpy), "rpy": c.group(2), "size": size, "src": c.group(0), "old": size.copy()})
        for _ in range(3):
            D = [np.linalg.norm(np.maximum(np.abs((L - b["c"]) @ b["R"]) - b["size"] / 2, 0), axis=1) for b in boxes]
            owner = np.argmin(np.vstack(D), axis=0)
            for i, b in enumerate(boxes):
                pts = L[owner == i]
                if len(pts) < 20:
                    continue
                Lb = (pts - b["c"]) @ b["R"]; lo, hi = Lb.min(0) - MARGIN, Lb.max(0) + MARGIN
                b["c"] = b["c"] + b["R"] @ ((lo + hi) / 2); b["size"] = hi - lo
        if link == "j0_link":
            for b in boxes:                       # axis-aligned on the base: clip from below
                bottom = b["c"][2] - b["size"][2] / 2
                if bottom < FLOOR:
                    top = b["c"][2] + b["size"][2] / 2; b["c"][2] = (FLOOR + top) / 2; b["size"][2] = top - FLOOR
        inside = np.zeros(len(L), bool)
        for b in boxes:
            inside |= np.all(np.abs((L - b["c"]) @ b["R"]) <= b["size"] / 2 + 0.5, axis=1)
        print(f"{link}: {len(L)} mesh points, coverage {100 * inside.mean():.1f}%")
        for b in boxes:
            print(f"    size {np.round(b['old'], 1)} -> {np.round(b['size'], 1)} mm, centre {np.round(b['c'], 1)} mm")
            new = (f'<collision>\n      <origin xyz="{b["c"][0] / 1000:.6f} {b["c"][1] / 1000:.6f} {b["c"][2] / 1000:.6f}" rpy="{b["rpy"]}"/>\n'
                   f'      <geometry>\n        <box size="{b["size"][0] / 1000:.6f} {b["size"][1] / 1000:.6f} {b["size"][2] / 1000:.6f}"/>\n      </geometry>\n    </collision>')
            new_s = new_s.replace(b["src"], new, 1)
    open(URDF, "w").write(new_s); print("written:", os.path.normpath(URDF))


if __name__ == "__main__":
    main()
