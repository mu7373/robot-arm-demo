/**
 * workspace.js —— 可达区域分析（B 侧：要求 d）
 *
 * 方法：在关节限位内均匀采样关节角 → 正运动学 → 得到一堆末端 TCP 点，这就是
 * "这台机器能到的所有地方"的离散近似（蒙特卡洛法）。采样点越多越密，边界越准。
 *
 * 三个必须讲清楚的坑（报告里要写）：
 *   1) 点云不是一个实心盒子。它在 x 方向很薄（这台臂的转轴近似平行），
 *      所以"立面/俯视投影"会退化成细条，必须补一张工作平面截面图才看得懂；
 *   2) 空洞判定用"栅格 + 从边界洪水填充"：没被外部连通的空格子才算内腔；
 *   3) 千万别用"离点云最近距离"当可达性判据——采样点间距本身就有量级，
 *      量出来的距离是采样稀疏造成的假象，不是不可达。
 *
 * 依赖：fk.js
 */

import { sampleWorkspace, forwardKinematics, JOINT_NAMES } from './fk.js';

/**
 * 采样点云。
 * @param {object} model createModel() 的结果
 * @param {object} [opts] { count = 30000, seed = 20240607 }
 * @returns {{points:Float64Array, count:number, seed:number}}
 */
export function sampleCloud(model, opts = {}) {
  const count = opts.count ?? 30000;
  const seed = opts.seed ?? 20240607;
  return { points: sampleWorkspace(model, count, seed), count, seed };
}

/** 扁平点云 → [[x,y,z],...] */
export function toTriples(flat) {
  const out = new Array((flat.length / 3) | 0);
  for (let i = 0; i < out.length; i++) {
    out[i] = [flat[i * 3], flat[i * 3 + 1], flat[i * 3 + 2]];
  }
  return out;
}

/**
 * 基本统计：包围盒、水平半径、工作面上/下比例、体素体积、直方图。
 * @param {Float64Array|number[][]} pts
 * @param {number} groundY 工作面高度（模型单位）
 */
export function workspaceStats(pts, groundY = 1.095) {
  const triples = pts instanceof Float64Array ? toTriples(pts) : pts;
  const n = triples.length;
  if (!n) throw new Error('[workspace] 点云为空');

  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  let rMin = Infinity, rMax = 0, above = 0;
  const rList = [];

  for (const p of triples) {
    for (let k = 0; k < 3; k++) {
      if (p[k] < min[k]) min[k] = p[k];
      if (p[k] > max[k]) max[k] = p[k];
    }
    const r = Math.hypot(p[0], p[2]);
    rList.push(r);
    if (r < rMin) rMin = r;
    if (r > rMax) rMax = r;
    if (p[1] >= groundY) above++;
  }

  // 体素体积：把点云栅格化成 1u 的格子，数被占的格子数
  const cell = 1.0;
  const occ = new Set();
  for (const p of triples) {
    occ.add(`${Math.floor(p[0] / cell)},${Math.floor(p[1] / cell)},${Math.floor(p[2] / cell)}`);
  }

  rList.sort((a, b) => a - b);
  const q = (f) => rList[Math.min(rList.length - 1, Math.floor(f * rList.length))];

  return {
    count: n,
    min, max,
    size: [max[0] - min[0], max[1] - min[1], max[2] - min[2]],
    radiusMin: rMin,
    radiusMax: rMax,
    radiusP2: q(0.02),
    radiusP98: q(0.98),
    aboveGround: above / n,
    belowGround: 1 - above / n,
    groundY,
    volumeApprox: occ.size * cell ** 3,
    cell
  };
}

/**
 * 极坐标外/内边界：把点云按方位角分箱，每箱取半径的 98%/2% 分位。
 * 用分位数而不是 max/min —— 蒙特卡洛总有孤立点，用极值会把边界拉成毛刺。
 *
 * @param {Float64Array|number[][]} pts
 * @param {object} [opts] { bins = 72, lowPct = 2, highPct = 98 }
 * @returns {{theta:number[], rOuter:number[], rInner:number[]}}
 */
export function polarEnvelope(pts, opts = {}) {
  const bins = opts.bins ?? 72;
  const loP = (opts.lowPct ?? 2) / 100;
  const hiP = (opts.highPct ?? 98) / 100;
  const triples = pts instanceof Float64Array ? toTriples(pts) : pts;

  const buckets = Array.from({ length: bins }, () => []);
  for (const p of triples) {
    const th = Math.atan2(p[2], p[0]);              // 俯视图里 x-z 平面的角
    const idx = Math.min(bins - 1, Math.floor(((th + Math.PI) / (2 * Math.PI)) * bins));
    buckets[idx].push(Math.hypot(p[0], p[2]));
  }

  const theta = [], rOuter = [], rInner = [];
  for (let i = 0; i < bins; i++) {
    theta.push(-Math.PI + ((i + 0.5) / bins) * 2 * Math.PI);
    const b = buckets[i].sort((a, c) => a - c);
    if (!b.length) { rOuter.push(0); rInner.push(0); continue; }
    rOuter.push(b[Math.min(b.length - 1, Math.floor(hiP * b.length))]);
    rInner.push(b[Math.min(b.length - 1, Math.floor(loP * b.length))]);
  }
  return { theta, rOuter, rInner };
}

/**
 * 二维占位栅格。
 * @param {number[][]} pts   [[x,y,z],...]
 * @param {object} opts
 *        opts.ax    取哪两个轴投影，默认 [0,2]（俯视 x-z）
 *        opts.cell  格子边长（模型单位），默认 0.5
 *        opts.band  可选：先按"第三轴 ≈ 某值"切一层，{ axis:1, value:0, half:1.2 }
 * @returns {{occ:Uint8Array, nx:number, ny:number, cell:number, x0:number, y0:number, ax:number[]}}
 */
export function gridOccupancy(pts, opts = {}) {
  const ax = opts.ax ?? [0, 2];
  const cell = opts.cell ?? 0.5;
  let list = pts;

  if (opts.band) {
    const { axis, value, half } = opts.band;
    list = pts.filter((p) => Math.abs(p[axis] - value) <= half);
  }
  if (!list.length) throw new Error('[workspace] 切片内没有点，换个大一点的 band');

  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const p of list) {
    const a = p[ax[0]], b = p[ax[1]];
    if (a < x0) x0 = a; if (a > x1) x1 = a;
    if (b < y0) y0 = b; if (b > y1) y1 = b;
  }
  const nx = Math.floor((x1 - x0) / cell) + 1;
  const ny = Math.floor((y1 - y0) / cell) + 1;
  const occ = new Uint8Array(nx * ny);

  for (const p of list) {
    const i = Math.floor((p[ax[0]] - x0) / cell);
    const j = Math.floor((p[ax[1]] - y0) / cell);
    if (i >= 0 && i < nx && j >= 0 && j < ny) occ[j * nx + i] = 1;
  }
  return { occ, nx, ny, cell, x0, y0, ax, bandPoints: list.length };
}

/**
 * 找内腔空洞：从栅格外圈往里做洪水填充，能填到的就是"外部"，
 * 剩下的空格子被可达点包围 → 是真正的不可达空洞。
 *
 * @param {{occ:Uint8Array, nx:number, ny:number}} grid
 * @param {object} [opts] opts.minCells 小于这个格数的连通块判为采样假象，默认 3
 */
export function findHoles(grid, opts = {}) {
  const { occ, nx, ny } = grid;
  const minCells = opts.minCells ?? 3;
  const outside = new Uint8Array(nx * ny);
  const stack = [];

  const pushIf = (i, j) => {
    if (i < 0 || i >= nx || j < 0 || j >= ny) return;
    const k = j * nx + i;
    if (outside[k] || occ[k]) return;
    outside[k] = 1;
    stack.push(k);
  };

  for (let i = 0; i < nx; i++) { pushIf(i, 0); pushIf(i, ny - 1); }
  for (let j = 0; j < ny; j++) { pushIf(0, j); pushIf(nx - 1, j); }

  while (stack.length) {
    const k = stack.pop();
    const i = k % nx, j = (k / nx) | 0;
    pushIf(i - 1, j); pushIf(i + 1, j); pushIf(i, j - 1); pushIf(i, j + 1);
  }

  // 剩下既不是点也不是外部的，就是被包围的区域
  const region = new Uint8Array(nx * ny);
  for (let k = 0; k < nx * ny; k++) region[k] = (!occ[k] && !outside[k]) ? 1 : 0;

  // 连通分量过滤
  const holes = [];
  const seen = new Uint8Array(nx * ny);
  for (let k = 0; k < nx * ny; k++) {
    if (!region[k] || seen[k]) continue;
    const comp = [];
    const st = [k];
    seen[k] = 1;
    while (st.length) {
      const c = st.pop();
      comp.push(c);
      const i = c % nx, j = (c / nx) | 0;
      const nb = [[i - 1, j], [i + 1, j], [i, j - 1], [i, j + 1]];
      for (const [ii, jj] of nb) {
        if (ii < 0 || ii >= nx || jj < 0 || jj >= ny) continue;
        const kk = jj * nx + ii;
        if (region[kk] && !seen[kk]) { seen[kk] = 1; st.push(kk); }
      }
    }
    if (comp.length >= minCells) holes.push(comp);
  }

  const holeCells = holes.reduce((a, h) => a + h.length, 0);
  return {
    region, holes, holeCells,
    holeCount: holes.length,
    singleCellRejected: region.reduce((a, v) => a + v, 0) - holeCells
  };
}

/**
 * 一次算完要求 d 需要的全部量（页面上一键出结果）。
 */
export function analyzeWorkspace(model, opts = {}) {
  const cloud = sampleCloud(model, opts);
  const triples = toTriples(cloud.points);
  const stats = workspaceStats(triples, opts.groundY ?? 1.095);
  const envelope = polarEnvelope(triples, opts.envelope);
  const medianX = median(triples.map((p) => p[0]));
  const grid = gridOccupancy(triples, {
    ax: [2, 1],
    cell: opts.cell ?? 0.5,
    band: { axis: 0, value: medianX, half: opts.bandHalf ?? 1.2 }
  });
  const holes = findHoles(grid, opts);
  return { cloud, triples, stats, envelope, grid, holes, sliceX: medianX, jointCount: JOINT_NAMES.length };
}

function median(arr) {
  const a = [...arr].sort((x, y) => x - y);
  return a.length ? a[(a.length / 2) | 0] : 0;
}

export default {
  sampleCloud, toTriples, workspaceStats, polarEnvelope,
  gridOccupancy, findHoles, analyzeWorkspace
};
