"""
animation.py —— 把 assets/program-circle.json 里的示教程序渲成一段动画 GIF。

为什么不用浏览器截图：本机沙箱里 Chrome 一启动就崩在 crashpad（`OpenProcess: 拒绝访问 (0x5)`），
这条路在实验环境里走不通。这里的做法是用 tools/render.py 那个纯 numpy+Pillow 的软件渲染器，
按 MOVJ 的时间曲线逐帧算关节角、逐帧重算世界矩阵，自己光栅化出画面。

输出：
  report/anim_circle.gif   整段示教程序循环播放（约 12 fps）
  report/frame_*.png       关键姿态静帧
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fastkin import chain, rodrigues  # noqa: E402
from render import gather, load_glb, render, trs  # noqa: E402
from robotlib import ROOT, UNIT_MM  # noqa: E402
import plotlib as pl  # noqa: E402

FPS = 12
W, H = 760, 520
ACCEL_RATIO = 0.2
PEAK_FACTOR = 1.0 / (1.0 - ACCEL_RATIO)
OMEGA_MAX = math.radians(90.0)

GLB = ROOT / "assets" / "robot_articulated.glb"
PROG = ROOT / "assets" / "program-circle.json"


def trapezoid(s, a=ACCEL_RATIO):
    s = np.clip(s, 0.0, 1.0)
    k = 2 * a * (1 - a)
    return np.where(s <= a, s ** 2 / k,
                    np.where(s >= 1 - a, 1 - (1 - s) ** 2 / k, (s - a / 2) / (1 - a)))


def build_W(js, theta_by_name):
    """按关节角重算 GLB 全部节点的世界矩阵（相当于 three.js 每帧 updateMatrixWorld）。"""
    n = len(js["nodes"])
    Wm = [None] * n
    axes = {j["name"]: np.array(j["axis"], float) for j in chain().joints}

    def rec(i, P):
        nd = js["nodes"][i]
        M = trs(nd)
        nm = nd.get("name")
        if nm in theta_by_name:
            R4 = np.eye(4)
            R4[:3, :3] = rodrigues(axes[nm], np.array([theta_by_name[nm]]))[0]
            M = M @ R4
        Wm[i] = P @ M
        for c in nd.get("children", []):
            rec(c, Wm[i])

    for r in js["scenes"][js.get("scene", 0)]["nodes"]:
        rec(r, np.eye(4))
    return Wm


def make_projector(eye, target, up, W, H, fovy):
    """与 render() 内部完全一致的世界→屏幕投影，用来把目标圆和轨迹叠到画面上。"""
    eye = np.array(eye, float)
    target = np.array(target, float)
    up = np.array(up, float)
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    view = np.array([[*(s), -(s @ eye)], [*(u), -(u @ eye)], [*(-f), f @ eye],
                     [0, 0, 0, 1]], float)
    fl = 1.0 / math.tan(math.radians(fovy) / 2)
    aspect = W / H

    def project(p):
        pc = (view @ np.array([p[0], p[1], p[2], 1.0]))[:3]
        if pc[2] > -0.05:
            return None
        return ((pc[0] * fl / aspect / -pc[2] * 0.5 + 0.5) * W,
                (1 - (pc[1] * fl / -pc[2] * 0.5 + 0.5)) * H)

    return project


def main():
    prog = json.loads(PROG.read_text(encoding="utf-8"))
    poses = np.radians(np.array(prog["posesDeg"], float))
    circ = prog["circle"]
    C = np.array(circ["center"], float)
    R = float(circ["radius"])
    U = np.array(circ["u"], float)
    V = np.array(circ["v"], float)
    n = len(poses)

    dq = np.abs(np.diff(np.vstack([poses, poses[:1]]), axis=0))
    T = np.maximum(PEAK_FACTOR * dq.max(axis=1) / OMEGA_MAX, 0.35)
    total = float(T.sum())
    nframes = max(8, int(round(total * FPS)))
    nn = chain().joint_name

    def q_at(time_s):
        t = time_s % total
        acc = 0.0
        for i in range(n):
            if t <= acc + T[i] or i == n - 1:
                s = (t - acc) / T[i]
                u = float(trapezoid(np.array([s]))[0])
                return poses[i] + (poses[(i + 1) % n] - poses[i]) * u
            acc += T[i]
        return poses[0]

    js, bin_ = load_glb(GLB)

    # ---- 用整段动画的包围盒定相机 ----
    lo = np.array([1e9] * 3)
    hi = -np.array([1e9] * 3)
    for k in range(0, nframes, max(1, nframes // 8)):
        q = q_at(k / FPS)
        Tm = gather(js, bin_, build_W(js, {nm: q[i] for i, nm in enumerate(nn)}))[0]
        lo = np.minimum(lo, Tm.reshape(-1, 3).min(0))
        hi = np.maximum(hi, Tm.reshape(-1, 3).max(0))
    ctr = (lo + hi) / 2
    dist = float(np.linalg.norm(hi - lo)) * 1.02
    # 圆心所在的圆画在 y-z 平面里，所以相机从 +x 方向看过去，圆才是正面朝观众的
    eye_dir = np.array([0.94, 0.16, 0.30])
    eye = ctr + dist * eye_dir / np.linalg.norm(eye_dir)
    up = (0.0, 1.0, 0.0)
    print(f"包围盒 {np.round(lo,2)} ~ {np.round(hi,2)}，相机距离 {dist:.1f}u，共 {nframes} 帧")

    # 地面上叠一个圆，标出示教目标轨迹
    ring = np.concatenate([
        (C + R * (np.cos(t) * U + np.sin(t) * V))[None, :] for t in np.linspace(0, 2 * np.pi, 121)
    ])

    frames, stills = [], []
    project = make_projector(eye, ctr, up, W, H, 42.0)
    ring = np.array([C + R * (np.cos(t) * U + np.sin(t) * V)
                     for t in np.linspace(0, 2 * np.pi, 181)])
    trail = []
    for k in range(nframes):
        q = q_at(k / FPS)
        Wm = build_W(js, {nm: q[i] for i, nm in enumerate(nn)})
        Tt, Cc = gather(js, bin_, Wm)
        img = render(Tt, Cc, eye, ctr, up, W=W, H=H, fovy=42.0,
                     bg=(0.08, 0.09, 0.11), ground=(1, 1.095, (-13.0, 13.0), 2.0))
        d = ImageDraw.Draw(img)
        # 目标圆（虚线示意）与实际走过的轨迹
        ring_px = [project(p) for p in ring]
        for a, b in zip(ring_px, ring_px[1:]):
            if a and b:
                d.line([a, b], fill=(90, 110, 70), width=1)
        tcp_w = chain().tcp(q[None, :])[0]
        trail.append(project(tcp_w))
        pts = [p for p in trail if p]
        if len(pts) > 1:
            d.line(pts, fill=(80, 220, 140), width=2)
        # 目标圆（用黄绿色细线示意，画在画面上更方便，这里只画文字说明）
        tcp = tcp_w * UNIT_MM
        f14, f12 = pl.font(15), pl.font(12)
        d.rectangle([12, 12, 330, 96], fill=(18, 22, 30))
        d.text((22, 18), "实验一 · 空中画圆（MOVJ 示教程序）", font=f14, fill=(240, 244, 250))
        d.text((22, 42), f"t = {k/FPS:5.2f} s / {total:.2f} s", font=f12, fill=(160, 200, 240))
        d.text((22, 60), "TCP (mm) = "
               f"({tcp[0]:+7.1f}, {tcp[1]:+7.1f}, {tcp[2]:+7.1f})", font=f12, fill=(200, 208, 220))
        # 关节角条
        for i in range(5):
            x0, y0 = 22 + i * 58, 78
            frac = (q[i] - chain().lim[i, 0]) / (chain().lim[i, 1] - chain().lim[i, 0])
            d.rectangle([x0, y0, x0 + 48, y0 + 6], fill=(46, 54, 68))
            d.rectangle([x0, y0, x0 + max(2, 48 * float(frac)), y0 + 6], fill=(90, 170, 240))
            d.text((x0, y0 + 9), f"J{i+1}", font=pl.font(10), fill=(150, 160, 175))
        frames.append(img)
        if k in (0, nframes // 4, nframes // 2, 3 * nframes // 4):
            stills.append(img)

    REPORT = ROOT / "report"
    gif = REPORT / "anim_circle.gif"
    frames_q = [f.convert("P", palette=Image.ADAPTIVE, colors=128) for f in frames]
    frames_q[0].save(gif, save_all=True, append_images=frames_q[1:],
                     duration=int(1000 / FPS), loop=0, optimize=True)
    print(f"已写 {gif}  ({gif.stat().st_size/1024:.0f} KB, {len(frames)} 帧)")
    for i, im in enumerate(stills):
        p = REPORT / f"frame_{i}.png"
        im.save(p)
    print(f"已写 {len(stills)} 张静帧")


if __name__ == "__main__":
    main()
