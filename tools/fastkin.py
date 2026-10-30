"""
fastkin.py —— 向量化正运动学 / 解析雅可比 / 批量阻尼最小二乘逆解。

为什么单独写一份：`tcp()` 是逐点纯 Python，做"上百个种子 × 上千步"的搜索时慢到没法用。
这里把整条链的矩阵乘法一次性广播到 B 个样本上，并用解析雅可比
    J_k = a_k × (p_tcp − p_k)
（a_k、p_k 为第 k 个关节的世界轴与世界枢轴）替代数值微分，快 3~4 个数量级。

节点层级与静止旋转全部读自 assets/robot-model.json（由 GLB 实测导出），
与 src/kinematics.js、tools/verify_fk.py 同源，因此结果可以和 FK 三档验证对齐。
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "assets" / "robot-model.json"

# ---------------------------------------------------------------- 基础旋转


def quat_to_mat(q) -> np.ndarray:
    """四元数 [x,y,z,w] → 3x3 旋转矩阵。"""
    x, y, z, w = np.asarray(q, float)
    n = x * x + y * y + z * z + w * w
    if n < 1e-15:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w),     s * (x * z + y * w)],
        [s * (x * y + z * w),     1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w),     s * (y * z + x * w),     1 - s * (x * x + y * y)],
    ])


def rodrigues(axis, theta) -> np.ndarray:
    """轴角 → 旋转矩阵，theta 可广播为形状 (B,)；axis 为 (3,)。返回 (B,3,3)。"""
    a = np.asarray(axis, float)
    a = a / (np.linalg.norm(a) or 1.0)
    th = np.atleast_1d(np.asarray(theta, float))
    c, s = np.cos(th)[:, None, None], np.sin(th)[:, None, None]
    x, y, z = a
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)
    I = np.eye(3)
    return c * I + s * K + (1 - c) * np.outer(a, a)


# ---------------------------------------------------------------- 模型装载

class Chain:
    def __init__(self, model_path: Path = MODEL_PATH):
        m = json.loads(Path(model_path).read_text(encoding="utf-8"))
        self.nodes = m["nodes"]
        self.names = [n["name"] for n in self.nodes]
        idx = {n: i for i, n in enumerate(self.names)}
        self.n = len(self.nodes)
        self.parent = np.array([idx[p] if p else -1 for p, n in
                                zip([x["parent"] for x in self.nodes], self.nodes)])
        self.t = np.array([n["translation"] or [0, 0, 0] for n in self.nodes], float)
        self.Rfix = np.array([quat_to_mat(n["rotation"]) if n["rotation"] else np.eye(3)
                              for n in self.nodes])
        self.joints = m["joints"]
        self.joint_node = np.array([idx[j["node"]] for j in self.joints])
        self.joint_axis = np.array([j["axis"] for j in self.joints], float)
        self.joint_name = [j["name"] for j in self.joints]
        self.lim = np.radians(np.array([j["limitDeg"] for j in self.joints], float))
        self.tcp_node = idx[m["tcpNode"]]
        self.has_joint = np.full(self.n, -1)
        for k, ni in enumerate(self.joint_node):
            self.has_joint[ni] = k
        # 拓扑序：确保父节点在前
        self.order = []
        seen = set()

        def visit(i):
            if i in seen:
                return
            p = self.parent[i]
            if p >= 0:
                visit(p)
            seen.add(i)
            self.order.append(i)

        for i in range(self.n):
            visit(i)

    def fk(self, TH: np.ndarray):
        """TH: (B,5) 关节角(rad)。返回 (tcp 位置 (B,3), 每关节世界枢轴 (B,5,3), 世界轴 (B,5,3))。"""
        TH = np.atleast_2d(np.asarray(TH, float))
        B = TH.shape[0]
        Rw = np.zeros((B, self.n, 3, 3))
        pw = np.zeros((B, self.n, 3))
        for i in self.order:
            p = self.parent[i]
            k = self.has_joint[i]
            if k >= 0:
                Rl = self.Rfix[i] @ rodrigues(self.joint_axis[k], TH[:, k])
            else:
                Rl = np.broadcast_to(self.Rfix[i], (B, 3, 3))
            if p < 0:
                Rw[:, i] = Rl
                pw[:, i] = self.t[i]
            else:
                Rw[:, i] = Rw[:, p] @ Rl
                pw[:, i] = Rw[:, p] @ self.t[i] + pw[:, p]
        jp = pw[:, self.joint_node]                    # (B,5,3)
        # 世界轴 = 世界旋转矩阵 × 关节局部轴
        ja = np.einsum("bkij,kj->bki", Rw[:, self.joint_node], self.joint_axis)
        return pw[:, self.tcp_node], jp, ja

    def tcp(self, TH) -> np.ndarray:
        return self.fk(TH)[0]

    def jacobian(self, TH: np.ndarray, p_tcp=None, jp=None, ja=None) -> np.ndarray:
        """解析雅可比 (B,3,5)：J_k = a_k × (p_tcp − p_k)。"""
        if p_tcp is None:
            p_tcp, jp, ja = self.fk(TH)
        d = p_tcp[:, None, :] - jp                     # (B,5,3)
        return np.transpose(np.cross(ja, d), (0, 2, 1))  # (B,3,5)

    def ik(self, target, seeds, iters=160, tol=1e-9, lam0=0.1, step_max=0.25,
           lim=None) -> tuple[np.ndarray, np.ndarray]:
        """对同一个目标做批量逆解。target (3,)，seeds (B,5)。返回 (q (B,5), 残差 (B,))。"""
        lim = self.lim if lim is None else lim
        target = np.asarray(target, float)
        q = np.clip(np.atleast_2d(np.asarray(seeds, float)), lim[:, 0], lim[:, 1])
        B = q.shape[0]
        lam = np.full(B, lam0)
        best_q, best_r = q.copy(), np.full(B, np.inf)
        for _ in range(iters):
            p, jp, ja = self.fk(q)
            e = target - p
            r = np.linalg.norm(e, axis=1)
            imp = r < best_r
            best_r = np.where(imp, r, best_r)
            best_q = np.where(imp[:, None], q, best_q)
            if np.all(best_r < tol):
                break
            J = np.transpose(np.cross(ja, p[:, None, :] - jp), (0, 2, 1))  # (B,3,5)
            A = J @ np.transpose(J, (0, 2, 1))
            A = A + (lam ** 2)[:, None, None] * np.eye(3)
            dq = np.einsum("bki,bk->bi", J, np.linalg.solve(A, e[:, :, None])[..., 0])
            mx = np.max(np.abs(dq), axis=1)
            scale = np.where(mx > step_max, step_max / np.maximum(mx, 1e-12), 1.0)
            dq = dq * scale[:, None]
            qn = np.clip(q + dq, lim[:, 0], lim[:, 1])
            rn = np.linalg.norm(target - self.fk(qn)[0], axis=1)
            good = rn < r
            q = np.where(good[:, None], qn, q)
            lam = np.where(good, np.maximum(0.005, lam * 0.6), np.minimum(5.0, lam * 2.2))
        return best_q, best_r


_CACHE: Chain | None = None


def chain(model_path: Path = MODEL_PATH) -> Chain:
    global _CACHE
    if _CACHE is None:
        _CACHE = Chain(model_path)
    return _CACHE


if __name__ == "__main__":
    c = chain()
    print("节点", c.n, "关节", c.joint_name, "tcp 节点", c.names[c.tcp_node])
    print("限位(deg)\n", np.degrees(c.lim))
    p = c.tcp(np.zeros((1, 5)))[0]
    print("零位 TCP =", np.round(p, 6))
    q, r = c.ik(np.array([2.0, 11.0, -3.0]), np.random.default_rng(0).uniform(
        c.lim[:, 0], c.lim[:, 1], size=(64, 5)))
    print("批量 IK 最优残差 =", r.min(), " q =", np.degrees(q[np.argmin(r)]).round(2))
