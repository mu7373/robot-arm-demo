"""
plan.py —— 生成示教程序：空中画圆（要求 b 的"自选动作"）。

关键点：5 自由度 + 位置级 IK 有 2 个冗余自由度，逐点独立逆解会跳到完全不同的分支
上（关节跳变 200°~340°），串联起来就成了乱舞。所以这里用**路径连续法**：
沿圆周把角度切成 360 小步，每一步以上一步的解为种子，保证关节曲线连续；
再从 360 个连续解里等间隔抽出 12 个作为"示教位姿"，用 MOVJ 串联。

圆心取在 rho=7u（距基座轴线 700mm）、高 1100mm 处，半径 200mm —— 这个位置避开了
基座轴线附近的"内腔"空洞（高度 11 处内边界约 450mm），整圈都可达。
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from fastkin import chain  # noqa: E402
from robotlib import ROOT, UNIT_MM  # noqa: E402

RADIUS = 2.0
N_POSES = 12
N_STEPS = 360

# 圆心与平面：轴向校正后这台臂的五个转轴近似平行（世界方向都以 ±x 为主），
# 末端可达域是 x 方向约 500mm 厚的竖直薄片，所以圆必须画在 y-z 平面里才有解。
CENTER = np.array([-1.5, 11.0, -4.5])
NORMAL = np.array([1.0, 0.0, 0.0])                 # 圆平面法向 = 机械臂的"厚度"方向
U = np.array([0.0, 1.0, 0.0])                      # 平面内竖直方向
V = np.cross(NORMAL, U)                            # = (0, 0, 1)，平面内水平方向


def target_at(t: float) -> np.ndarray:
    return CENTER + RADIUS * (math.cos(t) * U + math.sin(t) * V)


def main():
    c = chain()
    rng = np.random.default_rng(7)

    # ---- 起点：多起点全局找一个解 ----
    q0, r0 = c.ik(target_at(0.0), rng.uniform(c.lim[:, 0], c.lim[:, 1], size=(96, 5)),
                  iters=300)
    k = int(np.argmin(r0))
    q = q0[k]
    print(f"起点 IK 残差 {r0[k]:.2e} u，q = {np.degrees(q).round(1)}")

    # ---- 路径连续法：360 小步 ----
    path = [q]
    prev = q
    worst = 0.0
    for i in range(1, N_STEPS + 1):
        tgt = target_at(2 * math.pi * i / N_STEPS)
        seeds = np.vstack([prev, prev + rng.normal(0, 0.02, (3, 5))])
        qs, rs = c.ik(tgt, seeds, iters=120)
        j = int(np.argmin(rs))
        # 取"残差够小且离上一步最近"的解；若最近解残差偏大则退回最优解
        good = np.where(rs < 1e-8)[0]
        if len(good):
            j = int(good[np.argmin(np.abs(qs[good] - prev).sum(axis=1))])
        worst = max(worst, rs[j])
        prev = qs[j]
        path.append(prev)
    path = np.array(path)
    print(f"连续路径 {len(path)} 步，最大残差 {worst:.2e} u = {worst*UNIT_MM:.4f} mm")
    step = np.degrees(np.max(np.abs(np.diff(path, axis=0))))
    print(f"每 1° 圆周的关节最大步进 {step.max():.3f}°，累计行程 "
          f"{np.degrees(np.abs(np.diff(path, axis=0)).sum()):.0f}°")

    # ---- 抽出 12 个示教位姿 ----
    idx = [int(round(i * N_STEPS / N_POSES)) for i in range(N_POSES)]
    poses = path[idx]
    pj = np.degrees(np.max(np.abs(np.diff(np.vstack([poses, poses[:1]]), axis=0))))
    print(f"12 个示教位姿之间最大关节跳变 {pj:.1f}°（越小越顺）")

    # ---- MOVJ 串联后离理想圆有多远 ----
    dev = []
    for i in range(N_POSES):
        qa, qb = poses[i], poses[(i + 1) % N_POSES]
        for k2 in range(1, 60):
            s = k2 / 60
            p = c.tcp(qa + (qb - qa) * s)[0]
            d = p - CENTER
            d_ax = float(np.dot(d, np.cross(U, V)))
            rad = float(np.linalg.norm(d - d_ax * np.cross(U, V)))
            dev.append(math.hypot(d_ax, rad - RADIUS))
    dev = np.array(dev)
    print(f"MOVJ 造成的路径偏差：最大 {dev.max()*UNIT_MM:.1f} mm，平均 {dev.mean()*UNIT_MM:.1f} mm")

    out = {
        "name": "空中画圆",
        "note": (f"圆心 ({CENTER[0]:.1f}, {CENTER[1]:.1f}, {CENTER[2]:.1f})u "
                 f"= ({CENTER[0]*UNIT_MM:.0f}, {CENTER[1]*UNIT_MM:.0f}, {CENTER[2]*UNIT_MM:.0f}) mm，"
                 f"半径 {RADIUS*UNIT_MM:.0f} mm，画在 y-z 竖直平面内（法向 +x），"
                 f"{N_POSES} 个示教位姿，MOVJ 串联"),
        "unit": {"q": "deg", "position": "model unit", "1u": f"{UNIT_MM:.0f} mm"},
        "circle": {"center": CENTER.tolist(), "radius": RADIUS,
                   "normal": NORMAL.tolist(), "u": U.tolist(), "v": V.tolist()},
        "posesDeg": [[round(float(math.degrees(v)), 3) for v in qq] for qq in poses],
        "continuity": {"maxJointStepDeg": float(step.max()),
                       "maxPoseJumpDeg": float(pj)},
        "ikResidual": [float(worst)],
        "movjDeviationMM": {"max": float(dev.max() * UNIT_MM),
                            "mean": float(dev.mean() * UNIT_MM)},
    }
    path_out = ROOT / "assets" / "program-circle.json"
    path_out.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"已写 {path_out}")

    print("\n示教位姿表（度）→ 末端 TCP（mm）")
    for i, qq in enumerate(poses):
        p = c.tcp(qq)[0] * UNIT_MM
        print(f"  P{i+1:<2d} " + " ".join(f"{math.degrees(v):+7.1f}" for v in qq) +
              f"   → ({p[0]:+7.1f}, {p[1]:+7.1f}, {p[2]:+7.1f})  偏差 "
              f"{np.linalg.norm(c.tcp(qq)[0]-target_at(2*math.pi*i/N_POSES))*UNIT_MM:.2f} mm")


if __name__ == "__main__":
    main()
