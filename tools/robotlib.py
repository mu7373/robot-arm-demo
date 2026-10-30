"""
robotlib.py —— 离线侧的运动学工具（numpy）。

FK 直接复用 verify_fk.py 那份已经和三档验证通过、与 JS 实现误差 1.3e-14 的实现，
保证「报告里画的曲线」和「网页里看到的动作」出自同一套数学。
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from verify_fk import fk, JOINTS as JOINT_NAMES, LIM, rpy_deg  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
LIM_RAD = np.array([[math.radians(a), math.radians(b)] for a, b in (LIM[j] for j in JOINT_NAMES)])
UNIT_MM = 100.0


def tcp(q):
    return fk(list(q))[:3, 3].copy()


def jacobian(q, eps=1e-6):
    p0 = tcp(q)
    J = np.zeros((3, 5))
    for i in range(5):
        qq = list(q)
        qq[i] += eps
        J[:, i] = (tcp(qq) - p0) / eps
    return J


def ik(target, seed=None, iters=300, lam=0.06, tol=1e-6, step_max=0.25):
    """
    阻尼最小二乘（Levenberg-Marquardt 风格）位置逆解：
        dq = Jᵀ (J Jᵀ + λ²I)⁻¹ e ,  q ← clamp(q + dq)
    λ 越大越稳越慢；本模型 5 自由度解 3 维位置，冗余 2，解不唯一，
    所以用「上一帧解」当种子，路径才连续。
    """
    q = np.zeros(5) if seed is None else np.array(seed, float)
    tgt = np.asarray(target, float)
    hist = []
    for _ in range(iters):
        p = tcp(q)
        e = tgt - p
        n = float(np.linalg.norm(e))
        hist.append(n)
        if n < tol:
            break
        J = jacobian(q)
        JJt = J @ J.T + (lam ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, e)
        mx = float(np.max(np.abs(dq)))
        if mx > step_max:
            dq *= step_max / mx
        q = np.clip(q + dq, LIM_RAD[:, 0], LIM_RAD[:, 1])
    return q, float(np.linalg.norm(tgt - tcp(q))), hist


def reach_stats(n=40000, seed=7):
    rng = np.random.default_rng(seed)
    Q = LIM_RAD[:, 0] + (LIM_RAD[:, 1] - LIM_RAD[:, 0]) * rng.random((n, 5))
    P = np.array([tcp(q) for q in Q])
    r = np.linalg.norm(P[:, [0, 2]], axis=1)
    return {
        "points": P,
        "x": (P[:, 0].min(), P[:, 0].max()),
        "y": (P[:, 1].min(), P[:, 1].max()),
        "z": (P[:, 2].min(), P[:, 2].max()),
        "r": (r.min(), r.max()),
    }


if __name__ == "__main__":
    s = reach_stats()
    P = s["points"]
    print(f"采样 {len(P)} 点")
    print(f"  x ∈ [{s['x'][0]:.3f}, {s['x'][1]:.3f}] u")
    print(f"  y ∈ [{s['y'][0]:.3f}, {s['y'][1]:.3f}] u  (高度)")
    print(f"  z ∈ [{s['z'][0]:.3f}, {s['z'][1]:.3f}] u")
    print(f"  水平半径 r ∈ [{s['r'][0]:.3f}, {s['r'][1]:.3f}] u")
    print(f"  1u = {UNIT_MM:.0f} mm  ⇒ 最大水平半径 {s['r'][1]*UNIT_MM:.0f} mm")
    for q in ([0, 0, 0, 0, 0], [0, -30, 30, 0, 0], [90, -20, 40, 0, 0]):
        p = tcp([math.radians(v) for v in q])
        print(f"  q={q} → TCP ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
