"""
verify_fk.py —— 三档 FK 验证的独立复算侧。

刻意换一条数学路线：JS 侧走「四元数连乘 → 转矩阵」，这里走「轴角 Rodrigues → 齐次矩阵」，
且直接从 GLB 二进制读节点 TRS，不读 robot-model.json。两条实现互不共享代码，
对得上才说明 FK 本身是对的，而不是同一处 bug 被复制了两遍。

用法：python tools/verify_fk.py
"""
import json
import math
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
GLB = ROOT / "assets" / "robot_articulated.glb"
JSDATA = ROOT / "report" / "_fk_js.json"

JOINTS = ["J1", "J2", "J3", "J4", "J5"]
AXIS = {"J1": np.array([0, 1.0, 0]), "J2": np.array([0, 1.0, 0]),
        "J3": np.array([0, 1.0, 0]), "J4": np.array([0, 1.0, 0]), "J5": np.array([0, 1.0, 0])}
LIM = {"J1": (-180, 180), "J2": (-120, 120), "J3": (-120, 120), "J4": (-120, 120), "J5": (-180, 180)}


def read_glb_json(path):
    with open(path, "rb") as f:
        magic, ver, total = struct.unpack("<III", f.read(12))
        assert magic == 0x46546C67, "not a GLB"
        clen, ctype = struct.unpack("<II", f.read(8))
        return json.loads(f.read(clen))


def read_glb_bin(path):
    with open(path, "rb") as f:
        f.read(12)
        clen, _ = struct.unpack("<II", f.read(8))
        f.seek(8 + clen)
        blen, _ = struct.unpack("<II", f.read(8))
        return f.read(blen)


def read_accessor(js, bin_, index):
    """只支持本文件用得到的 float VEC3（POSITION）访问器。

    注意 byteStride：这个 glb 的 POSITION 是和法线交错存放的，按 12 字节紧密读取会
    读到别的属性上，坐标全是垃圾 —— 这是踩过的坑，所以这里显式按 stride 取。
    """
    acc = js["accessors"][index]
    bv = js["bufferViews"][acc["bufferView"]]
    off = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
    n = acc["count"]
    stride = bv.get("byteStride") or 12
    if stride == 12:
        return np.frombuffer(bin_, dtype=np.float32, count=n * 3, offset=off).reshape(n, 3).astype(float)
    raw = np.frombuffer(bin_, dtype=np.uint8, count=(n - 1) * stride + 12, offset=off)
    out = np.empty((n, 3), dtype=np.float32)
    for k in range(n):
        out[k] = np.frombuffer(raw[k * stride: k * stride + 12].tobytes(), dtype=np.float32)
    return out.astype(float)



def quat_to_mat(q):
    """四元数 [x,y,z,w] → 3x3，走的是「先归一化再按分量展开」的常规写法。"""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rodrigues(axis, th):
    """轴角 → 3x3，用 Rodrigues 闭式，不经过四元数。"""
    k = np.asarray(axis, float)
    k = k / (np.linalg.norm(k) or 1.0)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def load_robot():
    js = read_glb_json(GLB)
    nodes = js["nodes"]
    par = {}
    for i, n in enumerate(nodes):
        for c in n.get("children", []):
            par[c] = i
    name = {i: n.get("name") for i, n in enumerate(nodes)}
    order, idx = [], {n.get("name"): i for i, n in enumerate(nodes)}

    def rec(i):
        order.append(i)
        for c in nodes[i].get("children", []):
            rec(c)

    scene = js["scenes"][js.get("scene", 0)]["nodes"]
    for r in scene:
        rec(r)
    return nodes, par, name, order, idx


NODES, PAR, NAME, ORDER, IDX = load_robot()
js_ = None   # GLB 的 JSON 部分，main() 里装入，供第四档读网格用


def worlds(q_rad, axis_override=None):
    """给 5 个关节角（弧度），返回 节点名 → 世界齐次矩阵。"""
    W = {}
    for i in ORDER:
        n = NODES[i]
        t = np.array(n.get("translation", [0, 0, 0]), float)
        R = quat_to_mat(n.get("rotation", [0, 0, 0, 1]))
        nm = NAME[i]
        if nm in JOINTS:
            j = JOINTS.index(nm)
            lo, hi = (math.radians(v) for v in LIM[nm])
            th = min(hi, max(lo, q_rad[j]))
            ax = (axis_override or {}).get(nm, AXIS[nm])
            R = R @ rodrigues(ax, th)      # 右乘：在关节自身局部坐标系里转
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = t
        p = PAR.get(i)
        W[nm] = (W[NAME[p]] @ M) if (p is not None and NAME[p] in W) else M
    return W


def fk(q_rad):
    """给 5 个关节角（弧度），返回 tcp 的世界齐次矩阵。"""
    return worlds(q_rad)["tcp"]


def rpy_deg(m):
    R = m[:3, :3]
    sy = -R[2, 0]
    cy = math.hypot(R[0, 0], R[1, 0])
    ry = math.atan2(sy, cy)
    if cy > 1e-9:
        rx = math.atan2(R[2, 1], R[2, 2])
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx, rz = math.atan2(-R[1, 2], R[1, 1]), 0.0
    return [math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def main():
    global js_
    js_ = read_glb_json(GLB)
    data = json.loads(JSDATA.read_text(encoding="utf-8"))
    lines = []
    ok = True

    # 零位基准（由 GLB 场景链独立算得，与 joint_map.json 的 world_at_rest 换算后一致）
    REST_TCP = np.array([-1.600982, 12.491081, -5.126753])
    p0 = fk([0, 0, 0, 0, 0])[:3, 3]
    e0 = float(np.linalg.norm(p0 - REST_TCP))
    lines.append(f"第一档 全零位形：numpy 零位 TCP = ({p0[0]:+.6f}, {p0[1]:+.6f}, {p0[2]:+.6f})")
    lines.append(f"        与 GLB 基准点误差 = {e0:.2e}  {'OK' if e0 < 1e-5 else 'FAIL'}")
    ok &= e0 < 1e-5
    ej = float(np.linalg.norm(p0 - np.array(data["zero"]["p"])))
    lines.append(f"        与 JS 零位读数误差 = {ej:.2e}  {'OK' if ej < 1e-9 else 'FAIL'}")
    ok &= ej < 1e-9

    # 第二档
    lines.append("第二档 单关节 ±90°（隔离轴向/枢轴错误）：")
    emax = 0.0
    for s in data["single"]:
        j = JOINTS.index(s["joint"])
        q = [0.0] * 5
        q[j] = math.radians(s["deg"])
        p = fk(q)[:3, 3]
        e = float(np.linalg.norm(p - np.array(s["p"])))
        emax = max(emax, e)
        lines.append(f"        {s['joint']} {s['deg']:+4d}°  numpy=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})"
                     f"  JS=({s['p'][0]:+.4f},{s['p'][1]:+.4f},{s['p'][2]:+.4f})  |Δ|={e:.2e}")
    lines.append(f"        最大误差 = {emax:.2e}  {'OK' if emax < 1e-9 else 'FAIL'}")
    ok &= emax < 1e-9

    verdict = "PASS" if ok else "FAIL"
    lines.append("第三档 随机 100 组关节角（与 numpy 逐点对拍）：")
    emax = 0.0
    ermax = 0.0
    for s in data["random"]:
        m = fk(s["q"])
        p = m[:3, 3]
        e = float(np.linalg.norm(p - np.array(s["p"])))
        emax = max(emax, e)
        rn = np.array(rpy_deg(m))
        rj = np.array(s["rpyDeg"])
        # 欧拉角在 ±180° 处会跳变，取环形最小差
        d = np.abs(((rn - rj + 180) % 360) - 180)
        ermax = max(ermax, float(d.max()))
    lines.append(f"        位置最大误差 = {emax:.2e}  {'OK' if emax < 1e-9 else 'FAIL'}（判据 < 1e-9）")
    lines.append(f"        RPY 最大误差 = {ermax:.2e}°  {'OK' if ermax < 1e-6 else 'FAIL'}")
    ok &= emax < 1e-9 and ermax < 1e-6

    # 第四档：几何自洽性 —— 转轴必须让相邻连杆不脱开
    # 这一档是前三条抓不住的：前三档只对拍"同一条链算得一致"，链上轴配错了照样全对。
    # 判据：绕某个轴转满一圈，子连杆与父连杆的最小顶点距离应始终很小（<0.3u=30mm）；
    #       轴错了，子连杆会从自己的外壳里旋出去，距离会涨到另一根轴的几倍。
    lines.append("第四档 几何自洽性（转满一圈，相邻连杆最小顶点距离，用来验证转轴配对了没有）：")
    bin_ = read_glb_bin(GLB)
    mesh_v = {}
    for i, n in enumerate(NODES):
        if "mesh" not in n:
            continue
        prim = js_["meshes"][n["mesh"]]["primitives"][0]
        mesh_v[NAME[i]] = read_accessor(js_, bin_, prim["attributes"]["POSITION"])[::3]
    pairs = [("J1", "link_base", "link_1"), ("J2", "link_1", "link_2"),
             ("J3", "link_2", "link_3"), ("J4", "link_3", "link_4"),
             ("J5", "link_4", "link_5")]

    def gap(jn, parent, child, axis):
        worst = 0.0
        for deg in range(0, 360, 30):
            q = [0.0] * 5
            q[JOINTS.index(jn)] = math.radians(deg)
            W = worlds(q, {jn: axis})
            A = (W[parent] @ np.hstack([mesh_v[parent], np.ones((len(mesh_v[parent]), 1))]).T).T[:, :3]
            B = (W[child] @ np.hstack([mesh_v[child], np.ones((len(mesh_v[child]), 1))]).T).T[:, :3]
            best = 1e9
            for k in range(0, len(A), 256):
                d = np.linalg.norm(A[k:k + 256, None, :] - B[None, :, :], axis=2)
                best = min(best, float(d.min()))
            worst = max(worst, best)
        return worst

    def cyl_cos(jn, links):
        """枢轴附近顶点做圆柱面拟合，返回每个局部轴的 |cos|。

        关节外壳是回转体，其圆柱轴就是真正的转轴。取离枢轴 r 以内的顶点，
        协方差最小特征向量即轴向；半径要从 0.6u 扫到 3.0u，取顶点数够（≥150）
        且与某一根局部轴最贴合的那个结果 —— 太小取不到面，太大混进连杆本体。
        """
        i = IDX[jn]
        W = worlds([0.0] * 5)
        piv = (W[jn] @ np.array([0, 0, 0, 1.0]))[:3]
        R = W[jn][:3, :3]
        best = [0.0, 0.0, 0.0]
        for link in links:
            # mesh_v 存的是节点局部坐标，必须先乘世界矩阵再和世界枢轴比
            P = (W[link] @ np.hstack([mesh_v[link], np.ones((len(mesh_v[link]), 1))]).T).T[:, :3]
            d = np.linalg.norm(P - piv, axis=1)
            for rad in (0.6, 0.8, 1.0, 1.4, 1.8, 2.2, 2.6):
                sel = P[d < rad]
                if len(sel) < 50:   # mesh_v 已按 1/3 抽稀
                    continue
                Q = sel - sel.mean(0)
                C = Q.T @ Q / len(Q)
                _, v = np.linalg.eigh(C)
                ax = v[:, 0]
                cos = [abs(float(ax @ R[:, k])) for k in range(3)]
                if max(cos) > max(best):
                    best = cos
        return best

    cands = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}
    lines.append("        关节   局部X      局部Y      局部Z      采用的轴   判据")
    for jn, parent, child in pairs:
        g = {k: gap(jn, parent, child, v) for k, v in cands.items()}
        used = "XYZ"[int(np.argmax(np.abs(AXIS[jn])))]
        spread = max(g.values()) - min(g.values())
        if spread > 0.02:
            # 脱开距离能分辨 —— 用它
            best = min(g, key=g.get)
            method = "脱开距离"
            okj = (best == used) and g[used] < 0.30
        else:
            # 三个轴的脱开距离都一样小（连杆本来就互相插着），改用圆柱面拟合
            cos = cyl_cos(jn, [child, parent])
            best = "XYZ"[int(np.argmax(cos))]
            method = "圆柱拟合 " + "/".join(f"{c:.3f}" for c in cos)
            okj = (best == used) and max(cos) > 0.95
        flag = "OK" if okj else "FAIL"
        if not okj:
            ok = False
        lines.append(f"        {jn}   {g['X']:8.4f}  {g['Y']:8.4f}  {g['Z']:8.4f}      +{used}      "
                     f"{method}  → +{best}  {flag}")
    lines.append("        说明：数值单位是模型单位（1u = 100 mm）；0.0x u 的残差是三角面片离散化造成的，"
                 "不是真的脱开。")


    verdict = "PASS" if ok else "FAIL"
    lines.append(f"\n结论：{verdict}")
    text = "\n".join(lines)
    print(text)
    (ROOT / "report" / "fk_verify.txt").write_text(text, encoding="utf-8")
    (ROOT / "report" / "fk_verify.json").write_text(json.dumps(
        {"verdict": verdict, "zeroP": p0.tolist(), "singleMaxErr": emax, "randomMaxErr": emax},
        ensure_ascii=False, indent=1), encoding="utf-8")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
