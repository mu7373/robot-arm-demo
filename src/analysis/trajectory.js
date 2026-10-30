/**
 * trajectory.js —— 轨迹几何分析（B 侧：要求 c 的"路径图 + 偏差"部分）
 *
 * 要求 c 要回答两件事：
 *   1) 关节空间插值到底让末端走了条什么路？（路径图）
 *   2) 它离"理想目标路径"差多远？（偏差-时间曲线）
 *
 * 这里只做几何，不做统计绘图。速度/加速度在 recorder.js 的 analyzeSamples 里，
 * 因为那是"随时间"的量，跟采样方式绑定；这里是"随空间"的量，跟目标形状绑定。
 *
 * 依赖：无（纯数学）
 */

/** 向量小工具，避免为一两个运算引入依赖 */
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0]
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const unit = (a) => { const n = norm(a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

/**
 * 生成圆上的目标点（示教位姿的几何目标）。
 *
 * @param {object} circle
 *        circle.center [x,y,z]
 *        circle.radius 数
 *        circle.normal 平面法向（不要求单位化）
 *        circle.u      平面内第一个基向量（可选，默认取与 normal 不平行的轴）
 *        circle.count  采样点数
 *        circle.startAngle 起始角（弧度，默认 0）
 * @returns {{points:number[][], center:number[], radius:number, u:number[], v:number[], normal:number[]}}
 */
export function circleTargets(circle) {
  const center = circle.center.map(Number);
  const radius = Number(circle.radius);
  const normal = unit(circle.normal ?? [1, 0, 0]);

  let u = circle.u ? unit(circle.u.map(Number)) : defaultU(normal);
  // 正交化，保证 u 真的在平面内
  const proj = dot(u, normal);
  u = unit([u[0] - proj * normal[0], u[1] - proj * normal[1], u[2] - proj * normal[2]]);
  const v = cross(normal, u);

  const count = circle.count ?? 12;
  const a0 = circle.startAngle ?? 0;
  const points = [];
  for (let i = 0; i < count; i++) {
    const th = a0 + (2 * Math.PI * i) / count;
    points.push([
      center[0] + radius * (Math.cos(th) * u[0] + Math.sin(th) * v[0]),
      center[1] + radius * (Math.cos(th) * u[1] + Math.sin(th) * v[1]),
      center[2] + radius * (Math.cos(th) * u[2] + Math.sin(th) * v[2])
    ]);
  }
  return { points, center, radius, u, v, normal };
}

function defaultU(normal) {
  const ax = Math.abs(normal[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const c = cross(normal, ax);
  return norm(c) > 1e-9 ? unit(c) : [0, 1, 0];
}

/**
 * 点到整圆的真实距离（不是到圆心的距离）：
 *   把点分解成"平面内径向 ρ"和"离面高度 h"，距离 = sqrt((ρ-R)² + h²)
 *
 * @param {number[]} p
 * @param {{center:number[], radius:number, normal:number[]}} circle
 */
export function distanceToCircle(p, circle) {
  const n = unit(circle.normal);
  const d = sub(p, circle.center);
  const h = dot(d, n);                          // 离面分量
  const radial = [d[0] - h * n[0], d[1] - h * n[1], d[2] - h * n[2]];
  const rho = norm(radial);
  return Math.hypot(rho - circle.radius, h);
}

/**
 * 逐采样算"离理想圆多远"。
 * @param {Array} samples  recorder 的 samples（含 tcp 数组）
 * @param {object} circle  同上
 * @returns {{t:number[], dev:number[], max:number, rms:number, mean:number}}
 */
export function deviationSeries(samples, circle) {
  const geo = {
    center: circle.center.map(Number),
    radius: Number(circle.radius),
    normal: circle.normal ?? [1, 0, 0]
  };
  const t = [];
  const dev = [];
  for (const s of samples ?? []) {
    if (!Array.isArray(s.tcp)) continue;
    t.push(s.t);
    dev.push(distanceToCircle(s.tcp, geo));
  }
  let max = 0, sum = 0, sum2 = 0;
  for (const d of dev) { max = Math.max(max, d); sum += d; sum2 += d * d; }
  const n = dev.length || 1;
  return { t, dev, max, mean: sum / n, rms: Math.sqrt(sum2 / n) };
}

/** 折线长度（mm） */
export function polylineLength(points) {
  let L = 0;
  for (let i = 1; i < (points?.length ?? 0); i++) {
    L += norm(sub(points[i], points[i - 1]));
  }
  return L;
}

/**
 * 路径是否闭合（首尾距离），以及绕行比例 =
 *   实际路径长 / 理想圆周长。>1 说明多绕了，<1 说明抄近道了。
 */
export function pathShape(samples, circle) {
  const pts = (samples ?? []).filter((s) => Array.isArray(s.tcp)).map((s) => s.tcp);
  const L = polylineLength(pts);
  const ideal = 2 * Math.PI * Number(circle.radius);
  return {
    pointCount: pts.length,
    pathLength: L,
    idealLength: ideal,
    ratio: ideal > 0 ? L / ideal : 0,
    closureGap: pts.length > 1 ? norm(sub(pts[pts.length - 1], pts[0])) : 0
  };
}

/**
 * 把轨迹投影到圆所在平面，得到 (U, V) 二维坐标——画"轨迹图"用。
 * @returns {Array<[number, number]>}
 */
export function projectToPlane(points, circle) {
  const geo = circleTargets({ ...circle, count: 1 });
  const u = geo.u, v = geo.v;
  return (points ?? []).map((p) => {
    const d = sub(p, geo.center);
    return [dot(d, u), dot(d, v)];
  });
}

/**
 * 一次性打包要求 c 需要的所有几何量，方便页面上直接出一张图。
 * @param {Array} samples
 * @param {object} circle
 */
export function analyzeTrajectory(samples, circle) {
  const withPts = (samples ?? []).filter((s) => Array.isArray(s.tcp));
  const dev = deviationSeries(withPts, circle);
  const shape = pathShape(withPts, circle);
  const uv = projectToPlane(withPts.map((s) => s.tcp), circle);
  return { deviation: dev, shape, plane: uv, sampleCount: withPts.length };
}

export default {
  circleTargets, distanceToCircle, deviationSeries,
  polylineLength, pathShape, projectToPlane, analyzeTrajectory
};
