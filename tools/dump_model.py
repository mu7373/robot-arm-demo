import pathlib
import sys, json, numpy as np
ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
from render import load_glb

js, b = load_glb(str(ROOT / "assets" / "robot_articulated.glb"))
nodes = js["nodes"]
par = {}
for i, n in enumerate(nodes):
    for c in n.get("children", []): par[c] = i

JOINTS = ["J1","J2","J3","J4","J5"]
AXIS = {"J1":[0,1,0], "J2":[0,1,0], "J3":[0,1,0], "J4":[0,1,0], "J5":[0,1,0]}
LIM  = {"J1":[-180,180], "J2":[-120,120], "J3":[-120,120], "J4":[-120,120], "J5":[-180,180]}

out = {
  "source": "robot_articulated.glb (模型1)",
  "note": "节点局部 TRS 与关节轴均为实测。旋转四元数按 [x,y,z,w]，与 glTF 一致。",
  "nodes": [],
  "jointNames": JOINTS,
  "tcpNode": "tcp"
}
name_of = {i: n.get("name") for i, n in enumerate(nodes)}
for i, n in enumerate(nodes):
    out["nodes"].append({
        "name": n.get("name"),
        "parent": name_of.get(par.get(i), None),
        "translation": n.get("translation", [0,0,0]),
        "rotation": n.get("rotation", [0,0,0,1]),
        "hasMesh": "mesh" in n
    })
out["joints"] = [{"name": j, "axis": AXIS[j], "limitDeg": LIM[j],
                  "node": j, "qZero": None} for j in JOINTS]
# 关节静止旋转（= R_fix），写进去好让纯 JS 端不必再解析 GLB
idx = {n.get("name"): i for i, n in enumerate(nodes)}
for j in out["joints"]:
    q = nodes[idx[j["name"]]].get("rotation", [0,0,0,1])
    j["qZero"] = q
with open(str(ROOT / "assets" / "robot-model.json"), "w", encoding="utf-8") as f:
    json.dump(out, f, ensure_ascii=False, indent=1)
print("wrote robot-model.json", len(out["nodes"]), "nodes")
for j in out["joints"]:
    print(" ", j["name"], "axis", j["axis"], "limit", j["limitDeg"], "qZero", [round(v,4) for v in j["qZero"]])
