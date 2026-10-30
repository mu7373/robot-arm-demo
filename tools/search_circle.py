"""
_search_circle2.py —— 在修正后的"全 +Y"轴上重新找一条可完整逆解的画圆路径。

上一版圆心 (7,11,0) 是按错误的轴向配的；换成正确轴向后 x 方向的可达范围
被限制在约 [-4.2, 0.75]u，所以圆的平面必须落在这个薄片里。
这里做一次网格搜索：圆心 (x0,y0,z0)、半径 R，平面法向 (1,0,0)（即 y-z 平面内的圆），
每组用 32 个随机起点解出第一点，再沿圆周连续 240 步，统计残差与关节跳变。
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fastkin import chain  # noqa: E402

STEPS = 120
U = np.array([0.0, 1.0, 0.0])
V = np.array([0.0, 0.0, 1.0])


def pts_of(center, radius, n):
    c = np.asarray(center, float)
    return [c + radius * (np.cos(2 * np.pi * k / n) * U + np.sin(2 * np.pi * k / n) * V)
            for k in range(n)]


def try_circle(center, radius, rng, nseed=32):
    ch = chain()
    pts = pts_of(center, radius, STEPS)
    seeds = rng.uniform(ch.lim[:, 0], ch.lim[:, 1], size=(nseed, 5))
    q, r = ch.ik(pts[0], seeds, iters=200)
    k = int(np.argmin(r))
    if r[k] > 1e-6:
        return None
    prev, worst, path = q[k], float(r[k]), [q[k]]
    for tgt in pts[1:]:
        s = np.vstack([prev[None, :], seeds[:nseed - 1]])
        q, r = ch.ik(tgt, s, iters=60)
        good = np.where(r < 1e-8)[0]
        if len(good) == 0:
            k = int(np.argmin(r))
            worst = max(worst, float(r[k]))
            prev = q[k]
        else:
            k = good[int(np.argmin(np.abs(q[good] - prev).sum(axis=1)))]
            prev = q[k]
        path.append(prev)
    path = np.array(path)
    jumps = float(np.degrees(np.max(np.abs(np.diff(np.vstack([path, path[:1]]), axis=0)))))
    return worst, jumps, path


def main():
    rng = np.random.default_rng(2024)
    rows = []
    for x0 in (-3.0, -1.5, 0.0, 0.5):
        for y0 in (9.5, 11.0, 12.5):
            for z0 in (-2.0, -4.5, -7.0):
                for R in (1.5, 2.0):
                    res = try_circle((x0, y0, z0), R, rng)
                    if res is None:
                        continue
                    worst, jumps, _ = res
                    rows.append((worst, jumps, x0, y0, z0, R))
                    if worst < 1e-6:
                        print(f"  可解 x0={x0:+.1f} y0={y0:.1f} z0={z0:+.1f} R={R:.1f} "
                              f"残差 {worst*1000:.4f}mm 跳变 {jumps:.1f}°", flush=True)
    rows.sort()
    print(f"\n试试看：共 {len(rows)} 组第一点可解，其中整圈残差 <1e-6 的 "
          f"{sum(1 for w, *_ in rows if w < 1e-6)} 组；残差最小的 10 组：")
    for worst, jumps, x0, y0, z0, R in rows[:10]:
        print(f"  x0={x0:+.1f} y0={y0:.1f} z0={z0:+.1f} R={R:.1f}  "
              f"残差 {worst*1000:8.3f} mm  相邻跳变 {jumps:6.1f}°")


if __name__ == "__main__":
    main()
