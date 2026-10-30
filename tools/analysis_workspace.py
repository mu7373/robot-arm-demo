"""
analysis_workspace.py —— 要求 d：关节限位内蒙特卡洛采样 → 正运动学 → 末端可达点云，
画出可达区域图，标注外边界与不可达空洞，并统计数值。

报告结论要能回答三件事：
  1) 这台 5 自由度臂的可达空间有多大（外边界）；
  2) 里面有没有"空洞"（手臂够不到的内腔）；
  3) 哪些区域在实际作业空间之外（低于工作面 y=y_ground 的部分）。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from robotlib import ROOT, UNIT_MM, reach_stats  # noqa: E402
import plotlib as pl  # noqa: E402

N_SAMPLE = 60000
SEED = 20240607
Y_GROUND = 1.095          # 场景里地面格栅所在高度（模型单位）
REPORT = ROOT / "report"
REPORT.mkdir(exist_ok=True)


def polar_envelope(r, th, nbin=180, qlo=2.0, qhi=98.0):
    """按角度分箱，取每个角扇区内半径的 2%/98% 分位作为内/外边界。

    用分位数而不是极值：蒙特卡洛采样在稀疏区域会冒出个别孤立点，
    直接取 max/min 会把边界拉成毛刺，分位数对采样密度不敏感。
    """
    edges = np.linspace(-math.pi, math.pi, nbin + 1)
    rmax = np.full(nbin, np.nan)
    rmin = np.full(nbin, np.nan)
    idx = np.clip(np.digitize(th, edges) - 1, 0, nbin - 1)
    for b in range(nbin):
        m = idx == b
        if m.any():
            rmax[b] = np.percentile(r[m], qhi)
            rmin[b] = np.percentile(r[m], qlo)
    return edges, rmax, rmin


def find_holes(P, cell=0.6, margin=0.5, ax=(0, 2)):
    """平面投影栅格化：被占格围住、且与外界不连通的空格 = 不可达空洞。

    ax 指定投影到哪两个坐标轴，默认 (0,2) 即俯视 X-Z；
    (1,2) 则是工作平面截面 Y-Z。
    """
    x, z = P[:, ax[0]], P[:, ax[1]]
    x0, x1 = x.min() - margin, x.max() + margin
    z0, z1 = z.min() - margin, z.max() + margin
    nx = int(math.ceil((x1 - x0) / cell))
    nz = int(math.ceil((z1 - z0) / cell))
    occ = np.zeros((nx, nz), bool)
    ix = np.clip(((x - x0) / cell).astype(int), 0, nx - 1)
    iz = np.clip(((z - z0) / cell).astype(int), 0, nz - 1)
    occ[ix, iz] = True
    # 从边界洪水填充标出"外部空格"
    outside = np.zeros_like(occ)
    stack = [(i, 0) for i in range(nx)] + [(i, nz - 1) for i in range(nx)] \
        + [(0, j) for j in range(nz)] + [(nx - 1, j) for j in range(nz)]
    while stack:
        i, j = stack.pop()
        if i < 0 or j < 0 or i >= nx or j >= nz or outside[i, j] or occ[i, j]:
            continue
        outside[i, j] = True
        stack.extend([(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)])
    holes = (~occ) & (~outside)
    # 只保留连通面积 ≥3 格的空洞：单格多半是采样稀疏造成的假洞
    lab = np.zeros_like(holes, dtype=int)
    cur = 0
    kept = np.zeros_like(holes)
    for i, j in np.argwhere(holes):
        if lab[i, j]:
            continue
        cur += 1
        stack2 = [(i, j)]
        comp = []
        while stack2:
            a, b = stack2.pop()
            if a < 0 or b < 0 or a >= nx or b >= nz or lab[a, b] or not holes[a, b]:
                continue
            lab[a, b] = cur
            comp.append((a, b))
            stack2.extend([(a + 1, b), (a - 1, b), (a, b + 1), (a, b - 1)])
        if len(comp) >= 3:
            for a, b in comp:
                kept[a, b] = True
    n_single = int(holes.sum() - kept.sum())
    holes = kept
    cells = np.argwhere(holes)
    return dict(cell=cell, x0=x0, z0=z0, nx=nx, nz=nz, occ=occ, holes=holes,
                region=~outside, n_singleCell=n_single, n_cells=int(len(cells)),
                area=float(len(cells)) * cell * cell,
                cells=[(x0 + (i + .5) * cell, z0 + (j + .5) * cell) for i, j in cells])


def main():
    print(f"采样 {N_SAMPLE} 组关节角（种子 {SEED}）…")
    st = reach_stats(N_SAMPLE, SEED)
    P = st["points"]
    r = np.hypot(P[:, 0], P[:, 2])
    th = np.arctan2(P[:, 2], P[:, 0])
    y = P[:, 1]
    print(f"  水平半径 r: {r.min():.3f} ~ {r.max():.3f} u  (最大 {r.max()*UNIT_MM:.0f} mm)")
    print(f"  高度    y: {y.min():.3f} ~ {y.max():.3f} u  (最高 {y.max()*UNIT_MM:.0f} mm)")

    above = y >= Y_GROUND
    print(f"  工作面上方占比 {above.mean()*100:.1f}%，"
          f"下方（伸到地面以下，实际会撞工作台）{100-above.mean()*100:.1f}%")

    # 体积：用 1.0u 立方栅格数格子
    g = 1.0
    key = np.unique(np.floor(P / g).astype(np.int64), axis=0)
    vol = len(key) * g ** 3
    print(f"  可达体积（1u 体素近似）= {vol:.0f} u³ = {vol*(UNIT_MM/1000)**3:.3f} m³")

    holes = find_holes(P[above])
    print(f"  俯视投影内检出的不可达空洞：{holes['n_cells']} 格 "
          f"× {holes['cell']}u = {holes['area']:.2f} u²")

    # 轴向校正后，这台臂的可达域是 x 方向约 500mm 厚的竖直薄片，
    # 俯视/立面投影都退化成细条，真正能看出形状的是"工作平面截面" —— 取中间一层 x 切片。
    xmid = float(np.median(P[:, 0]))
    band = np.abs(P[:, 0] - xmid) < 1.2
    sec = find_holes(P[above & band][:, [2, 1]], cell=0.5, margin=0.4, ax=(0, 1))

    edges, rmax, rmin = polar_envelope(r[above], th[above])

    # ---------------- 画图 ----------------
    W, H = 1560, 1180
    img, d = pl.new_canvas(W, H)
    d.text((40, 24), "要求 d｜末端可达区域分析（蒙特卡洛采样 %d 组关节角 → 正运动学）" % N_SAMPLE,
           font=pl.font(22), fill=(24, 30, 42))
    d.text((40, 56), f"1 模型单位 = {UNIT_MM:.0f} mm；采样为关节限位内均匀分布，"
                     f"坐标为 GLB 场景坐标（+Y 朝上）；该臂五个转轴近似平行，"
                     f"可达域是 x 方向约 {(P[:,0].max()-P[:,0].min())*UNIT_MM:.0f} mm 厚的竖直薄片",
           font=pl.font(13), fill=(100, 108, 122))

    m = UNIT_MM
    # (a) 立面 X-Y
    a = pl.Axes(img, d, (90, 130, 760, 590), equal=True,
                title="(a) 立面投影  X–Y", xlabel="X / mm", ylabel="高度 Y / mm")
    sx, sy = P[:, 0] * m, y * m
    a.extend(sx, sy)
    a.frame()
    a.scatter(sx[~above], sy[~above], r=1, color=(210, 210, 216))
    a.scatter(sx[above], sy[above], r=1, color=(74, 132, 220))
    a.hline(Y_GROUND * m, color=(220, 140, 60), width=2, dash=True)
    a.annotate(P[:, 0].max() * m * 0.55, Y_GROUND * m, " 工作面 y=1.095u（灰点在其下方）",
               color=(200, 120, 40), dx=6, dy=10)
    a.legend([("工作面以上", (74, 132, 220)), ("工作面以下", (210, 210, 216)),
              ("工作面", (220, 140, 60))], corner="tl")

    # (b) 俯视 X-Z + 内外边界
    b = pl.Axes(img, d, (860, 130, 1520, 590), equal=True,
                title="(b) 俯视投影  X–Z", xlabel="X / mm", ylabel="Z / mm")
    b.extend(P[:, 0] * m, P[:, 2] * m)
    b.frame()
    b.scatter(P[above, 0] * m, P[above, 2] * m, r=1, color=(74, 132, 220))
    ang = (edges[:-1] + edges[1:]) / 2
    ok = ~np.isnan(rmax)
    b.line(np.cos(ang[ok]) * rmax[ok] * m, np.sin(ang[ok]) * rmax[ok] * m,
           color=(214, 60, 48), width=2)
    ok2 = ~np.isnan(rmin)
    b.line(np.cos(ang[ok2]) * rmin[ok2] * m, np.sin(ang[ok2]) * rmin[ok2] * m,
           color=(30, 150, 110), width=2)
    hx = [c[0] * m for c in holes["cells"]]
    hz = [c[1] * m for c in holes["cells"]]
    if hx:
        b.scatter(hx, hz, r=3, color=(240, 160, 20))
        hcx, hcz = float(np.mean(hx)), float(np.mean(hz))
        b.annotate(hcx, hcz, f"不可达空洞\n{holes['n_cells']} 格", color=(200, 120, 10))
    b.legend([("可达点（俯视）", (74, 132, 220)), ("外边界 r_max(θ)", (214, 60, 48)),
              ("内边界 r_min(θ)", (30, 150, 110)), ("不可达空洞", (240, 160, 20))], corner="tl")

    # (c) 半径-高度剖面
    c = pl.Axes(img, d, (90, 700, 760, 1110),
                title="(c) 半径–高度剖面  r–Y", xlabel="水平半径 r / mm", ylabel="高度 Y / mm")
    c.extend(r * m, y * m)
    c.frame()
    c.scatter(r[~above] * m, y[~above] * m, r=1, color=(212, 212, 218))
    c.scatter(r[above] * m, y[above] * m, r=1, color=(74, 132, 220))
    c.hline(Y_GROUND * m, color=(220, 140, 60), width=2, dash=True)
    c.annotate(r.max() * m * 0.62, Y_GROUND * m + 60,
               f"外边界 r_max = {r.max()*m:.0f} mm", color=(200, 60, 50))
    c.annotate(30, y.max() * m * 0.94, f"最高点 {y.max()*m:.0f} mm", color=(60, 110, 200))

    # (d) 工作平面截面 Z–Y 占位栅格：被可达格点围住、又和外界不连通的空格 = 不可达空洞
    dd = pl.Axes(img, d, (880, 700, 1520, 1110), equal=True,
                 title=f"(d) 工作平面截面 Z–Y（取 x≈{xmid*m:+.0f}mm 的中间一层）",
                 xlabel="Z / mm（水平）", ylabel="Y / mm（竖直）")
    h_info = sec
    gz0, gy0, cell2 = h_info["x0"], h_info["z0"], h_info["cell"]
    dd.xlim = (gz0 * m, (gz0 + h_info["nx"] * cell2) * m)
    dd.ylim = (gy0 * m, (gy0 + h_info["nz"] * cell2) * m)
    dd.frame()
    occ, hl, reg = h_info["occ"], h_info["holes"], h_info["region"]
    cs = cell2 * m
    ii, jj = np.where(reg)
    if len(ii):
        zd = (gz0 + np.array([ii.min(), ii.max() + 1]) * cell2) * m
        yd = (gy0 + np.array([jj.min(), jj.max() + 1]) * cell2) * m
    else:
        zd = yd = np.array([0.0, 1.0])
    for i in range(h_info["nx"]):
        for j in range(h_info["nz"]):
            if not reg[i, j]:
                continue
            zc = gz0 * m + i * cs
            yc = gy0 * m + j * cs
            p0, p1 = dd.P(zc, yc), dd.P(zc + cs, yc + cs)
            dd.d.rectangle([p0[0], p1[1], p1[0], p0[1]], fill=(196, 220, 246))
    for i, j in np.argwhere(hl):
        zc = gz0 * m + i * cs
        yc = gy0 * m + j * cs
        p0, p1 = dd.P(zc, yc), dd.P(zc + cs, yc + cs)
        dd.d.rectangle([p0[0], p1[1], p1[0], p0[1]], fill=(245, 165, 40))
    # 外边界：可达栅格的实际最小/最大 z 与最小/最大 y，画虚线并标注
    zlo, zhi = float(zd[0]), float(zd[1])
    ylo, yhi = float(yd[0]), float(yd[1])
    dd.vline(zlo, color=(214, 60, 48), width=2, dash=True)
    dd.vline(zhi, color=(214, 60, 48), width=2, dash=True)
    dd.hline(ylo, color=(214, 60, 48), width=2, dash=True)
    dd.hline(yhi, color=(214, 60, 48), width=2, dash=True)
    dd.annotate(zlo, yhi, f" 外边界 Z[{zlo:.0f},{zhi:.0f}] Y[{ylo:.0f},{yhi:.0f}] mm",
                color=(200, 60, 50), size=12, dx=6, dy=-6)
    if h_info["n_cells"]:
        cz = np.mean([c[0] for c in h_info["cells"]]) * m
        cy = np.mean([c[1] for c in h_info["cells"]]) * m
        dd.annotate(cz, cy, f"不可达空洞 {h_info['n_cells']} 格\n（这一层任何姿态都够不到）",
                    color=(190, 110, 10), size=12, dx=30, dy=-46, arrow=True)
    else:
        dd.annotate(0, ylo + 0.30 * (yhi - ylo),
                    "内腔空洞：0 处\n可达域在截面内单连通，无被围住的盲区",
                    color=(30, 150, 110), size=12, dx=-140, dy=0)
    dd.legend([("可达（至少一个姿态能到达）", (196, 220, 246)),
               ("不可达空洞", (245, 165, 40)),
               ("外边界", (214, 60, 48))], corner="tr")

    img.save(REPORT / "fig_workspace.png")
    print(f"已写 {REPORT / 'fig_workspace.png'}")

    summary = {
        "nSamples": int(N_SAMPLE), "seed": SEED,
        "unitMM": UNIT_MM,
        "x": [float(P[:, 0].min()), float(P[:, 0].max())],
        "y": [float(y.min()), float(y.max())],
        "z": [float(P[:, 2].min()), float(P[:, 2].max())],
        "radius": [float(r.min()), float(r.max())],
        "aboveWorkSurfaceRatio": float(above.mean()),
        "volumeVoxel": float(vol),
        "holes": {"cells": holes["n_cells"], "areaUnit2": holes["area"], "cellSize": holes["cell"]},
        "boundary": {"rmax": [float(np.nanmax(rmax)), ],
                     "rmin": [float(np.nanmin(rmin))]},
    }
    (REPORT / "workspace.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1),
                                           encoding="utf-8")
    print(f"已写 {REPORT / 'workspace.json'}")


if __name__ == "__main__":
    main()
