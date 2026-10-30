"""
analysis_trajectory.py —— 要求 c：逐帧记录末端 TCP，画轨迹路径图与位置/速度–时间曲线，
并量化"MOVJ 关节空间插值"相对理想圆的偏差（插值效果与平滑性）。

时间模型（与 src/kinematics.js、contract.js 保持一致）：
    T_seg = 1.25 * max_j|Δq_j| / ω_max          （1.25 = 梯形曲线峰值系数 1/(1−a)，a=0.2）
    s(t)  = 梯形速度曲线积分 → 归一化进度
    峰值关节速度正好等于 ω_max = 90°/s，不会超速。
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
import plotlib as pl  # noqa: E402

ACCEL_RATIO = 0.2
PEAK_FACTOR = 1.0 / (1.0 - ACCEL_RATIO)        # 梯形曲线峰值/均值 = 1.25
OMEGA_MAX = math.radians(90.0)                 # 关节最大角速度
FS = 120.0                                     # 采样率（Hz），远高于 60fps 便于求导


def trapezoid(s, a=ACCEL_RATIO):
    s = np.clip(s, 0.0, 1.0)
    k = 2 * a * (1 - a)
    out = np.where(s <= a, s ** 2 / k,
                   np.where(s >= 1 - a, 1 - (1 - s) ** 2 / k, (s - a / 2) / (1 - a)))
    return out


def trapezoid_vel(s, a=ACCEL_RATIO):
    """归一化速度（相对平均速度），用于画速度曲线。"""
    s = np.clip(s, 0.0, 1.0)
    k = 2 * a * (1 - a)
    return np.where(s <= a, 2 * s / k,
                    np.where(s >= 1 - a, 2 * (1 - s) / k, 1.0 / (1 - a)))


def build_trajectory(poses_deg):
    c = chain()
    q = np.radians(np.array(poses_deg, float))
    n = len(q)
    # 相邻位姿的关节空间距离 → 每段时长
    dq = np.abs(np.diff(np.vstack([q, q[:1]]), axis=0))
    dmax = dq.max(axis=1)
    T = PEAK_FACTOR * dmax / OMEGA_MAX
    T = np.maximum(T, 0.35)                    # 最短 0.35s，避免"瞬移"
    times, qs = [], []
    t0 = 0.0
    for i in range(n):
        seg_n = max(2, int(round(T[i] * FS)))
        s = np.linspace(0, 1, seg_n, endpoint=False)
        u = trapezoid(s)
        for k in range(seg_n):
            times.append(t0 + s[k] * T[i])
            qs.append(q[i] + (q[(i + 1) % n] - q[i]) * u[k])
        t0 += T[i]
    times = np.array(times)
    qs = np.array(qs)
    return c, q, qs, times, T


def main():
    prog_path = ROOT / "assets" / "program-circle.json"
    prog = json.loads(prog_path.read_text(encoding="utf-8"))
    poses_deg = prog["posesDeg"]
    circ = prog["circle"]
    C = np.array(circ["center"], float)
    R = float(circ["radius"])
    NAX = np.array(circ["normal"], float)
    U = np.array(circ["u"], float)
    V = np.array(circ["v"], float)

    c, q0, qs, t, T = build_trajectory(poses_deg)
    P = c.tcp(qs)                                    # (N,3) 模型单位
    Pm = P * UNIT_MM                                 # mm
    dt = np.diff(t)
    dt[dt <= 0] = 1e-6
    v = np.linalg.norm(np.diff(Pm, axis=0), axis=1) / dt          # mm/s
    vm = np.concatenate([[v[0]], v])                              # 对齐长度
    v_smooth = np.convolve(vm, np.ones(5) / 5, mode="same")        # 轻平滑，压掉数值毛刺
    a_acc = np.gradient(v_smooth, t)                              # mm/s²

    # 相对理想圆的位置偏差
    d = Pm - C * UNIT_MM
    d_perp = d - np.outer(d @ NAX, NAX)
    r_now = np.linalg.norm(d_perp, axis=1)
    dev = np.hypot(d @ NAX, r_now - R * UNIT_MM)                  # 面外偏差 + 半径偏差

    path_len = float(np.sum(np.linalg.norm(np.diff(Pm, axis=0), axis=1)))
    stats = {
        "cycles": 1,
        "samples": int(len(t)),
        "duration_s": float(t[-1]),
        "pathLength_mm": path_len,
        "idealCircle_mm": 2 * math.pi * R * UNIT_MM,
        "peakSpeed_mm_s": float(v_smooth.max()),
        "meanSpeed_mm_s": float(path_len / t[-1]),
        "peakAccel_mm_s2": float(np.abs(a_acc).max()),
        "deviation_mm": {"max": float(dev.max()), "mean": float(dev.mean()),
                         "rms": float(np.sqrt((dev ** 2).mean()))},
        "segmentDuration_s": [float(x) for x in T],
        "tcpRest": [float(x) for x in P[0]],
    }
    print("=== 要求 c 轨迹分析 ===")
    print(f"  采样 {len(t)} 点，总时长 {t[-1]:.2f} s")
    print(f"  路径长度 {path_len:.1f} mm（理想整圆 {2*math.pi*R*UNIT_MM:.1f} mm）")
    print(f"  峰值速度 {v_smooth.max():.1f} mm/s，平均 {path_len/t[-1]:.1f} mm/s，"
          f"峰值加速度 {np.abs(a_acc).max():.0f} mm/s²")
    print(f"  相对理想圆偏差：最大 {dev.max():.2f} mm，RMS {stats['deviation_mm']['rms']:.2f} mm")

    # ---------------- 图 ----------------
    img, dr = pl.new_canvas(1560, 1180)
    dr.text((40, 24), "要求 c｜轨迹分析：MOVJ 关节空间插值的实际末端轨迹", font=pl.font(22),
            fill=(24, 30, 42))
    dr.text((40, 56), f"{len(poses_deg)} 个示教位姿 → MOVJ 串联，{len(t)} 帧采样（{FS:.0f} Hz）",
            font=pl.font(13), fill=(100, 108, 122))

    # (a) 平面内轨迹 vs 理想圆
    pu = (Pm - C * UNIT_MM) @ U
    pv = (Pm - C * UNIT_MM) @ V
    ax = pl.Axes(img, dr, (90, 130, 760, 590), equal=True,
                 title="(a) 轨迹在圆平面内的形状", xlabel="平面内 U 方向 / mm",
                 ylabel="平面内 V 方向 / mm")
    ax.extend(np.concatenate([pu, [R * UNIT_MM, -R * UNIT_MM]]),
              np.concatenate([pv, [R * UNIT_MM, -R * UNIT_MM]]))
    ax.frame()
    th = np.linspace(0, 2 * math.pi, 361)
    ax.line(R * UNIT_MM * np.cos(th), R * UNIT_MM * np.sin(th),
            color=(150, 160, 175), width=2)
    ax.line(pu, pv, color=(214, 60, 48), width=2)
    ax.scatter(pu, pv, r=2, color=(214, 60, 48))
    for i in range(len(poses_deg)):
        k = int(np.searchsorted(t, np.concatenate([[0], np.cumsum(T)])[i]))
        k = min(k, len(pu) - 1)
        ax.scatter([pu[k]], [pv[k]], r=5, color=(20, 100, 200))
        ax.annotate(pu[k], pv[k], f"P{i+1}", color=(20, 90, 190), size=12, dx=6, dy=4)
    ax.legend([("理想圆（示教目标）", (150, 160, 175)), ("MOVJ 实际轨迹", (214, 60, 48)),
               ("示教位姿", (20, 100, 200))], corner="tl")

    # (b) XYZ 位置–时间
    bx = pl.Axes(img, dr, (900, 130, 1520, 590),
                 title="(b) 末端位置–时间曲线", xlabel="时间 t / s", ylabel="位置 / mm")
    bx.extend(t, np.concatenate([Pm[:, 0], Pm[:, 1], Pm[:, 2]]))
    bx.frame()
    cols = [(214, 60, 48), (30, 150, 110), (40, 120, 210)]
    for i, (nm, cl) in enumerate(zip(("X", "Y", "Z"), cols)):
        bx.line(t, Pm[:, i], color=cl, width=2)
    bx.legend([("X", cols[0]), ("Y", cols[1]), ("Z", cols[2])], corner="br")

    # (c) 速度–时间
    cx = pl.Axes(img, dr, (90, 700, 760, 1110),
                 title="(c) 末端线速度–时间曲线", xlabel="时间 t / s", ylabel="速度 / (mm/s)")
    cx.extend(t, v_smooth)
    cx.frame()
    cx.line(t, v_smooth, color=(214, 60, 48), width=2)
    cx.hline(float(np.mean(v_smooth)), color=(120, 128, 140), width=1, dash=True)
    cx.annotate(t[len(t) // 2], np.mean(v_smooth), f" 平均 {np.mean(v_smooth):.0f} mm/s",
                color=(110, 118, 130), size=12, dy=14)
    cx.annotate(t[np.argmax(v_smooth)], v_smooth.max() * 0.92,
                f"峰值 {v_smooth.max():.0f} mm/s", color=(190, 50, 40), size=12,
                dx=-140, dy=-4)
    # 段边界竖线
    tb = np.concatenate([[0], np.cumsum(T)])
    for x0 in tb[1:-1]:
        cx.vline(float(x0), color=(220, 224, 232), width=1)

    # (d) 偏差–时间
    dx_ = pl.Axes(img, dr, (900, 700, 1520, 1110),
                  title="(d) 相对理想圆的位置偏差–时间", xlabel="时间 t / s",
                  ylabel="偏差 / mm")
    dx_.extend(t, dev)
    dx_.frame()
    dx_.line(t, dev, color=(232, 145, 30), width=2)
    dx_.hline(float(dev.mean()), color=(120, 128, 140), width=1, dash=True)
    dx_.annotate(t[int(len(t) * 0.55)], dev.max(),
                 f"最大 {dev.max():.1f} mm", color=(190, 110, 20), size=13, dy=-14)
    dx_.annotate(t[int(len(t) * 0.55)], float(dev.mean()),
                 f"平均 {dev.mean():.1f} mm（关节空间直线 ≠ 笛卡尔圆）",
                 color=(110, 118, 130), size=12, dy=12)

    (ROOT / "report").mkdir(exist_ok=True)
    img.save(ROOT / "report" / "fig_trajectory.png")
    (ROOT / "report" / "trajectory.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=1), encoding="utf-8")
    print(f"  已写 {ROOT/'report'/'fig_trajectory.png'}")
    print(f"  已写 {ROOT/'report'/'trajectory.json'}")


if __name__ == "__main__":
    main()
