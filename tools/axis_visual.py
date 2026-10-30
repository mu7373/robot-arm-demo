"""
_axis_visual.py —— 调试用：把候选转轴画出来，看机械臂到底绕哪根线转。

做法：
  1. 计算该关节枢轴的世界坐标与世界轴向；
  2. 沿该轴画一条 ±2.5u 的直线（投影到屏幕，PIL 画）；
  3. 在枢轴处画一个方块标记；
  4. 把关节角 0 / ±45 / ±90 度排成一张对照图。
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render import gather, load_glb, render, trs, world_matrices  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
GLB = ROOT / "assets" / "robot_articulated.glb"


def rod(a, th):
    a = np.asarray(a, float)
    a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)


def build(js, th):
    W = [None] * len(js["nodes"])

    def rec(i, P):
        nd = js["nodes"][i]
        M = trs(nd)
        if nd.get("name") in th:
            axv, t = th[nd["name"]]
            R4 = np.eye(4)
            R4[:3, :3] = rod(axv, t)
            M = M @ R4
        W[i] = P @ M
        for c in nd.get("children", []):
            rec(c, W[i])

    for r in js["scenes"][0]["nodes"]:
        rec(r, np.eye(4))
    return W


def project(eye, ctr, up, W, H, fovy, pts):
    eye = np.asarray(eye, float)
    f = np.asarray(ctr, float) - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, np.asarray(up, float))
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    fl = 1.0 / math.tan(math.radians(fovy) / 2)
    aspect = W / H
    P = np.asarray(pts, float)
    d = P - eye
    cx, cy, cz = d @ s, d @ u, d @ f          # 相机系（cz 为前向深度）
    out = []
    for k in range(len(P)):
        if cz[k] <= 0.05:
            out.append(None)
        else:
            out.append(((cx[k] * fl / aspect / cz[k] * 0.5 + 0.5) * W,
                        (1 - (cy[k] * fl / cz[k] * 0.5 + 0.5)) * H))
    return out


def main():
    js, bin_ = load_glb(GLB)
    W0 = world_matrices(js)
    T0, C0 = gather(js, bin_, W0)
    lo, hi = T0.reshape(-1, 3).min(0), T0.reshape(-1, 3).max(0)
    ctr = (lo + hi) / 2
    d = float(np.linalg.norm(hi - lo))
    eye = ctr + d * 1.15 * np.array([0.15, 0.10, 0.98]) / np.linalg.norm([0.15, 0.10, 0.98])
    up = (0.0, 1.0, 0.0)
    fovy = 42.0
    W, H = 470, 400

    idx = {nd["name"]: i for i, nd in enumerate(js["nodes"])}
    out = ROOT / "report"
    for jn, cand in (("J1", [(1, 0, 0), (0, 1, 0)]), ("J2", [(0, 0, 1), (0, 1, 0)])):
        i = idx[jn]
        piv = (W0[i] @ np.array([0, 0, 0, 1.0]))[:3]
        tiles = []
        for axv in cand:
            ax_w = (W0[i] @ np.array([*axv, 0.0]))[:3]
            ax_w /= np.linalg.norm(ax_w)
            for deg in (-90, 0, 90):
                Wm = build(js, {jn: (axv, math.radians(deg))})
                T, C = gather(js, bin_, Wm)
                im = render(T, C, eye, ctr, up, W=W, H=H, fovy=fovy)
                dr = ImageDraw.Draw(im)
                p0, p1 = project(eye, ctr, up, W, H, fovy, [piv - 2.5 * ax_w, piv + 2.5 * ax_w])
                if p0 and p1:
                    dr.line([p0, p1], fill=(255, 90, 90), width=2)
                pc = project(eye, ctr, up, W, H, fovy, [piv])[0]
                if pc:
                    dr.ellipse([pc[0] - 4, pc[1] - 4, pc[0] + 4, pc[1] + 4], outline=(255, 220, 60))
                axes = "XYZ"[int(np.argmax(np.abs(axv)))]
                dr.text((8, 6), f"{jn} local {axes}  {deg:+d}deg", fill=(255, 225, 130))
                tiles.append(im)
        sheet = Image.new("RGB", (W * 3, H * 2), (10, 10, 12))
        for k, t in enumerate(tiles):
            sheet.paste(t, ((k % 3) * W, (k // 3) * H))
        sheet.save(out / f"_axisvis_{jn}.png")
        print("saved", out / f"_axisvis_{jn}.png")


if __name__ == "__main__":
    main()
