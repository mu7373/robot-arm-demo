/**
 * calibration.js —— 工具坐标系（TCP 四点法）与用户坐标系（三点法）
 *
 * 纯数学 + 数据存取：不 import three、不碰 DOM，可以直接在 Node 里做单元测试。
 * 位姿一律用「基坐标下的位置 + 3×3 旋转矩阵」表示：
 *     pose = { p: [x, y, z], R: [[r00, r01, r02], [r10, ...], [r20, ...]] }   // R 行主序
 *
 * ── 四点法（工具坐标系）
 *   工具尖在 4 个不同姿态下碰同一个点 P，则对每个姿态 i：
 *       p_i + R_i · t = P        （t 是工具尖在法兰坐标系下的偏移，待求）
 *   移项成线性方程  [R_i | -I] · [t; P] = -p_i，4 个姿态共 12 个方程、6 个未知数，
 *   最小二乘解出 t 与 P；把每个点代回去的残差就是标定误差（可直接量化）。
 *
 * ── 三点法（用户坐标系）
 *   原点 P0、X 方向点 P1、XY 平面内一点 P2：
 *       x = unit(P1 - P0)
 *       z = unit(x × (P2 - P0))
 *       y = z × x
 *   坐标系 = { origin: P0, x, y, z }，基坐标 ↔ 用户坐标就是一正一反两次投影。
 */

const EPS = 1e-9;

/* ══════════════════════════════════════════════════════════════
 * 向量 / 矩阵小工具
 * ══════════════════════════════════════════════════════════════ */

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0]
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

function unit(a) {
  const n = norm(a);
  return n < EPS ? null : scale(a, 1 / n);
}

const matVec = (R, v) => [
  R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
  R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
  R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]
];

const matT = (R) => [
  [R[0][0], R[1][0], R[2][0]],
  [R[0][1], R[1][1], R[2][1]],
  [R[0][2], R[1][2], R[2][2]]
];

const matMul = (A, B) => [0, 1, 2].map((i) => [0, 1, 2].map((j) =>
  A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j]));

/** 高斯消元（部分选主元）。奇异返回 null。 */
function solveLinear(A, b) {
  const n = b.length;
  const M = A.map((row, i) => row.slice().concat([b[i]]));

  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) {
      if (Math.abs(M[r][col]) > Math.abs(M[piv][col])) piv = r;
    }
    if (Math.abs(M[piv][col]) < 1e-12) return null;
    if (piv !== col) { const tmp = M[col]; M[col] = M[piv]; M[piv] = tmp; }

    const d = M[col][col];
    for (let c = col; c <= n; c++) M[col][c] /= d;

    for (let r = 0; r < n; r++) {
      if (r === col) continue;
      const f = M[r][col];
      if (!f) continue;
      for (let c = col; c <= n; c++) M[r][c] -= f * M[col][c];
    }
  }
  return M.map((row) => row[n]);
}

/* ══════════════════════════════════════════════════════════════
 * 四点法：工具坐标系
 * ══════════════════════════════════════════════════════════════ */

/**
 * @param {Array<{p:number[], R:number[][]}>} poses 至少 4 个法兰位姿
 * @returns {{offset:number[], fixedPoint:number[], residuals:number[], rms:number, maxError:number, count:number}}
 */
export function solveToolFromPoses(poses) {
  if (!Array.isArray(poses) || poses.length < 4) {
    throw new Error('[calibration] 工具坐标系至少要 4 个示教点');
  }

  // 法方程 (AᵀA)x = Aᵀb，x = [t(3), P(3)]
  const M = Array.from({ length: 6 }, () => new Array(6).fill(0));
  const rhs = new Array(6).fill(0);

  for (const pose of poses) {
    const R = pose.R;
    const p = pose.p;
    for (let r = 0; r < 3; r++) {
      const row = [R[r][0], R[r][1], R[r][2], 0, 0, 0];
      row[3 + r] = -1;
      const val = -p[r];
      for (let i = 0; i < 6; i++) {
        rhs[i] += row[i] * val;
        for (let j = 0; j < 6; j++) M[i][j] += row[i] * row[j];
      }
    }
  }

  const x = solveLinear(M, rhs);
  if (!x) {
    throw new Error('[calibration] 示教姿态太接近（或共面），方程奇异。请让 4 个姿态的方向差别更大一些。');
  }

  const offset = [x[0], x[1], x[2]];
  const fixedPoint = [x[3], x[4], x[5]];

  const residuals = poses.map((pose) =>
    norm(sub(add(pose.p, matVec(pose.R, offset)), fixedPoint)));
  const rms = Math.sqrt(residuals.reduce((s, r) => s + r * r, 0) / residuals.length);
  const maxError = Math.max(...residuals);

  return { offset, fixedPoint, residuals, rms, maxError, count: poses.length };
}

/** 由法兰位姿 + 工具偏移算工具尖位置（基坐标） */
export function toolTipFromPose(pose, offset) {
  return add(pose.p, matVec(pose.R, offset));
}

/* ══════════════════════════════════════════════════════════════
 * 三点法：用户坐标系
 * ══════════════════════════════════════════════════════════════ */

/**
 * @param {number[]} p0 原点
 * @param {number[]} p1 X 轴正方向上的一点
 * @param {number[]} p2 XY 平面内（X 轴正侧）一点
 * @returns {{origin:number[], x:number[], y:number[], z:number[]}}
 */
export function makeUserFrame(p0, p1, p2) {
  const x = unit(sub(p1, p0));
  if (!x) throw new Error('[calibration] 原点与 X 方向点重合了');

  const v2 = sub(p2, p0);
  const z = unit(cross(x, v2));
  if (!z) throw new Error('[calibration] 三点共线，确定不了 XY 平面');

  // 右手系：y = z × x。这样构造出来 z 的方向总是让第三点落在 +Y 侧，
  // 所以不需要再判"点选在轴的哪一边"。
  const y = cross(z, x);         // 已是单位向量（x、z 正交且单位）

  return {
    origin: [p0[0], p0[1], p0[2]],
    x, y, z
  };
}

/** 基坐标点 → 用户坐标点 */
export function pointToUser(frame, p) {
  const d = sub(p, frame.origin);
  return [dot(frame.x, d), dot(frame.y, d), dot(frame.z, d)];
}

/** 用户坐标点 → 基坐标点 */
export function pointToBase(frame, p) {
  return add(frame.origin,
    add(scale(frame.x, p[0]), add(scale(frame.y, p[1]), scale(frame.z, p[2]))));
}

/** 基坐标旋转矩阵 → 用户坐标旋转矩阵（Aᵀ·R，A 的列为 x/y/z） */
export function rotationToUser(frame, R) {
  const A = [
    [frame.x[0], frame.y[0], frame.z[0]],
    [frame.x[1], frame.y[1], frame.z[1]],
    [frame.x[2], frame.y[2], frame.z[2]]
  ];
  return matMul(matT(A), R);
}

/**
 * 三点法自检：三个定义点代回去应该正好落在 (0,0,0)、(d,0,0)、(u,v,0)。
 * 说明：三点构造的坐标系本来就能让这三个点精确落位，这里的残差主要是
 * 浮点噪声，真正有信息量的是「正交性」和「第四个验证点」。
 */
export function verifyUserFrame(frame, points) {
  if (!frame) throw new Error('[calibration] verifyUserFrame 需要坐标系');
  const [p0, p1, p2] = points;

  const d = norm(sub(p1, p0));
  const u = dot(sub(p2, p0), frame.x);
  const v = dot(sub(p2, p0), frame.y);

  const got = [pointToUser(frame, p0), pointToUser(frame, p1), pointToUser(frame, p2)];
  const expected = [[0, 0, 0], [d, 0, 0], [u, v, 0]];
  const residuals = got.map((g, i) => norm(sub(g, expected[i])));

  const orthogonality = [
    Math.abs(dot(frame.x, frame.y)),
    Math.abs(dot(frame.x, frame.z)),
    Math.abs(dot(frame.y, frame.z))
  ];

  return {
    residuals,
    maxError: Math.max(...residuals),
    orthogonality,
    maxOrthogonality: Math.max(...orthogonality),
    axesLengthError: Math.max(
      Math.abs(norm(frame.x) - 1),
      Math.abs(norm(frame.y) - 1),
      Math.abs(norm(frame.z) - 1)
    )
  };
}

/** 基坐标 ↔ 用户坐标 往返一次的最大误差（三个定义点 + 可选验证点） */
export function roundTripError(frame, points) {
  let max = 0;
  for (const p of points) {
    max = Math.max(max, norm(sub(pointToBase(frame, pointToUser(frame, p)), p)));
  }
  return max;
}

/* ══════════════════════════════════════════════════════════════
 * 数据存取
 * ══════════════════════════════════════════════════════════════ */

export const DEFAULT_STORAGE_KEY = 'robotArm.calibration.v1';

/** 组一份可存档的标定记录（加版本号和时间戳） */
export function buildRecord({ tool = null, user = null, verification = null, meta = {} } = {}) {
  return {
    version: 1,
    savedAt: new Date().toISOString(),
    tool,
    user,
    verification,
    meta
  };
}

function getStorage(storage) {
  if (storage) return storage;
  return (typeof localStorage !== 'undefined') ? localStorage : null;
}

/** 存到 localStorage（可传入假 storage 方便测试） */
export function saveCalibration(record, { key = DEFAULT_STORAGE_KEY, storage = null } = {}) {
  const s = getStorage(storage);
  if (!s) throw new Error('[calibration] 当前环境没有 localStorage，无法保存');
  s.setItem(key, JSON.stringify(record));
  return record;
}

/** 从 localStorage 读回；没有则返回 null */
export function loadCalibration({ key = DEFAULT_STORAGE_KEY, storage = null } = {}) {
  const s = getStorage(storage);
  if (!s) return null;
  const raw = s.getItem(key);
  if (!raw) return null;
  try {
    return JSON.parse(raw);
  } catch {
    return null;
  }
}

export function clearCalibration({ key = DEFAULT_STORAGE_KEY, storage = null } = {}) {
  const s = getStorage(storage);
  if (s) s.removeItem(key);
}

/** 下载成 .json（浏览器） */
export function downloadCalibration(record, filename = 'calibration.json') {
  if (typeof document === 'undefined') {
    throw new Error('[calibration] downloadCalibration 只能在浏览器里用');
  }
  const blob = new Blob([JSON.stringify(record, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** 校验一份从文件读进来的记录，返回 { ok, errors } */
export function validateRecord(record) {
  const errors = [];
  if (!record || typeof record !== 'object') errors.push('不是对象');
  else {
    if (record.version !== 1) errors.push(`版本号不是 1（收到 ${record.version}）`);
    if (record.tool && (!Array.isArray(record.tool.offset) || record.tool.offset.length !== 3)) {
      errors.push('tool.offset 不是三元数组');
    }
    if (record.user && (!Array.isArray(record.user.origin))) errors.push('user.origin 缺失');
  }
  return { ok: errors.length === 0, errors };
}

export default {
  solveToolFromPoses,
  makeUserFrame,
  pointToUser,
  pointToBase,
  rotationToUser,
  verifyUserFrame,
  roundTripError,
  buildRecord,
  saveCalibration,
  loadCalibration,
  downloadCalibration
};
