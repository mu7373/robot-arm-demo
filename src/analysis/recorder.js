/**
 * recorder.js —— 轨迹采样录制（B 侧：要求 c 的数据来源）
 *
 * 为什么要单独一层：示教器每帧只知道"当前 5 个关节角 + 末端位姿"，
 * 而要求 c 需要的是"随时间变化的完整序列"。中间这一层要保证三件事：
 *   1) 时间从 0 开始（用绝对时间戳算速度会得到垃圾数据）；
 *   2) 带帧号（掉帧时才知道真实 dt 该用多少）；
 *   3) 可导出 JSON，离线再算图表（浏览器里算也行，但要在报告里可复现）。
 *
 * 依赖：无（纯数据结构操作，不依赖 three / DOM）
 */

const nowMs = () =>
  (typeof performance !== 'undefined' && performance.now)
    ? performance.now()
    : Date.now();

/** 把 {x,y,z} 或 [x,y,z] 统一成数组 */
function toXYZ(p) {
  if (!p) return null;
  if (Array.isArray(p)) return [Number(p[0]), Number(p[1]), Number(p[2])];
  if (typeof p === 'object') return [Number(p.x), Number(p.y), Number(p.z)];
  return null;
}

export class SampleRecorder {
  /**
   * @param {object} [opts]
   *        opts.maxSamples 上限，超出后停止追加（防止开一晚上采爆内存），默认 20000
   *        opts.autoTimestamp 没有传 t 时是否用内部时钟，默认 true
   */
  constructor(opts = {}) {
    this.maxSamples = opts.maxSamples ?? 20000;
    this.autoTimestamp = opts.autoTimestamp ?? true;
    this.meta = {};
    this._reset();
  }

  _reset() {
    this.samples = [];
    this._frame = 0;
    this._t0 = 0;
    this._recording = false;
  }

  /** 开始录制。可以带一段元信息，会原样进 JSON（例如 {program:'空中画圆', poses:12}）。 */
  start(meta = {}) {
    this._reset();
    this.meta = { ...meta, startedAt: new Date().toISOString() };
    this._t0 = nowMs();
    this._recording = true;
  }

  /**
   * 追加一帧。
   * @param {object} frame
   *        frame.jointsRad 5 个关节角（弧度）
   *        frame.tcp       {p:{x,y,z}} 或 [x,y,z]
   *        frame.t        可选，秒。不传就用内部时钟
   *        ...rest        其余字段原样保存（例如 {seg: 2}）
   * @returns {object|null} 保存下来的那条，或 null（未录制/已满）
   */
  push(frame) {
    if (!this._recording) return null;
    if (this.samples.length >= this.maxSamples) {
      this._recording = false;
      return null;
    }
    const t = (typeof frame?.t === 'number')
      ? frame.t
      : (this.autoTimestamp ? (nowMs() - this._t0) / 1000 : 0);

    const p = toXYZ(frame?.tcp?.p ?? frame?.tcp);
    const rec = {
      frame: this._frame++,
      t,
      jointsRad: (frame?.jointsRad ?? []).map(Number),
      tcp: p
    };
    const { jointsRad, tcp, t: _t, ...rest } = frame ?? {};
    Object.assign(rec, rest);
    this.samples.push(rec);
    return rec;
  }

  stop() {
    this._recording = false;
    return this.toJSON();
  }

  get isRecording() { return this._recording; }
  get count() { return this.samples.length; }

  /** 导出结构：meta + samples，够离线复现全部图表 */
  toJSON() {
    return {
      meta: { ...this.meta, sampleCount: this.samples.length, fpsEstimate: estimateFps(this.samples) },
      samples: this.samples
    };
  }
}

/** 从采样里估 FPS（用相邻 dt 的中位数，抗掉帧干扰） */
export function estimateFps(samples) {
  if (!samples || samples.length < 2) return 0;
  const dt = [];
  for (let i = 1; i < samples.length; i++) {
    const d = samples[i].t - samples[i - 1].t;
    if (d > 1e-6) dt.push(d);
  }
  if (!dt.length) return 0;
  dt.sort((a, b) => a - b);
  const med = dt[Math.floor(dt.length / 2)];
  return med > 0 ? 1 / med : 0;
}

/**
 * 由采样算速度/加速度曲线。差分用真实 dt，不用名义帧率——
 * 掉帧时名义帧率会把速度算出假的尖峰。
 *
 * @param {Array} samples SampleRecorder 产出的 samples
 * @param {object} [opts] opts.smooth 平滑窗口（奇数，默认 1 不平滑）
 * @returns {{
 *   t:number[], speed:number[], accel:number[], dist:number[], reach:number[],
 *   duration:number, pathLength:number, peakSpeed:number, meanSpeed:number,
 *   peakAccel:number, sampleCount:number
 * }}
 */
export function analyzeSamples(samples, opts = {}) {
  const pts = (samples ?? []).filter((s) => Array.isArray(s.tcp));
  const n = pts.length;
  const t = new Array(n);
  const speed = new Array(n).fill(0);
  const accel = new Array(n).fill(0);
  const dist = new Array(n).fill(0);
  const reach = new Array(n).fill(0);

  for (let i = 0; i < n; i++) {
    t[i] = pts[i].t;
    const [x, y, z] = pts[i].tcp;
    reach[i] = Math.hypot(x, z);          // 水平距基座的距离（要求 c 的辅助曲线）
  }

  let pathLength = 0;
  for (let i = 1; i < n; i++) {
    const dt = t[i] - t[i - 1];
    const a = pts[i - 1].tcp, b = pts[i].tcp;
    const d = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    pathLength += d;
    dist[i] = pathLength;
    if (dt > 1e-9) speed[i] = d / dt;     // 首点为 0
  }
  dist[0] = 0;

  // 加速度用速度差分
  for (let i = 2; i < n; i++) {
    const dt = t[i] - t[i - 1];
    if (dt > 1e-9) accel[i] = Math.abs(speed[i] - speed[i - 1]) / dt;
  }

  const sm = opts.smooth ?? 1;
  const speedOut = sm > 1 ? movingAverage(speed, sm) : speed;

  let peakSpeed = 0, sum = 0, peakAccel = 0;
  for (let i = 0; i < n; i++) {
    peakSpeed = Math.max(peakSpeed, speedOut[i]);
    sum += speedOut[i];
    peakAccel = Math.max(peakAccel, accel[i]);
  }

  return {
    t, speed: speedOut, accel, dist, reach,
    duration: n ? t[n - 1] - t[0] : 0,
    pathLength,
    peakSpeed,
    meanSpeed: n ? sum / n : 0,
    peakAccel,
    sampleCount: n
  };
}

/** 滑动平均（只用于画图，不改原始数据） */
export function movingAverage(arr, win = 5) {
  const w = Math.max(1, win | 1);
  const half = (w - 1) / 2;
  const out = new Array(arr.length);
  for (let i = 0; i < arr.length; i++) {
    let s = 0, c = 0;
    for (let k = i - half; k <= i + half; k++) {
      if (k >= 0 && k < arr.length) { s += arr[k]; c++; }
    }
    out[i] = c ? s / c : 0;
  }
  return out;
}

/**
 * 一段程序的关节角总行程（deg），用来解释"为什么这段这么慢"。
 */
export function jointTravel(samples) {
  if (!samples || samples.length < 2) return [0, 0, 0, 0, 0];
  const first = samples[0].jointsRad ?? [];
  const last = samples[samples.length - 1].jointsRad ?? [];
  return first.map((v, i) => Math.abs((last[i] ?? v) - v) * 180 / Math.PI);
}

export default { SampleRecorder, analyzeSamples, estimateFps, movingAverage, jointTravel };
