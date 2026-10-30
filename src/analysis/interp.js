/**
 * interp.js —— MOVJ 示教程序的链路组织（B 侧：要求 b 的插值内核 + 要求 c 的数据来源）
 *
 * 分工边界：
 *   · 单段关节空间插值的数学在 fk.js（jointInterp / jointSpaceDuration），这里不重复实现；
 *   · 本文件负责"把若干个示教位姿串成一条可执行的程序"：分段、每段时长、速度倍率、
 *     按时间轴取姿态。A 的示教器面板只需要 buildMovjProgram + sampleProgram 两个函数。
 *
 * 为什么 MOVJ 用关节空间而不是直线插值：
 *   关节角线性变化时，末端 TCP 走的是 FK 映射出来的弧线，不是直线；
 *   好处是每个关节都在自己的限位内匀速走、不会中途反向，坏处是末端路径不可预测。
 *   要求 c 的"偏差-时间"曲线就是在量化这个坏处有多大。
 *
 * 依赖：fk.js（纯数学，不依赖 three / DOM）
 */

import {
  JOINT_NAMES,
  jointInterp,
  jointSpaceDuration,
  trapezoidProgress
} from './fk.js';

/** 每个关节的默认最大角速度（rad/s）。教学用保守值，调用方可覆盖。 */
export const DEFAULT_MAX_JOINT_SPEED = Object.freeze([
  Math.PI / 2, Math.PI / 2, Math.PI / 2, Math.PI / 2, Math.PI / 2
]);

/** 单段最短时长（秒）。防止两点几乎重合时 T→0、速度爆表。 */
export const MIN_SEGMENT_DURATION = 0.35;

/**
 * 把示教位姿序列编译成 MOVJ 程序。
 *
 * @param {number[][]} posesRad  示教位姿（每个是 5 个关节角，弧度），至少 2 个
 * @param {object} [opts]
 *        opts.maxSpeedRad  每关节最大角速度，默认 DEFAULT_MAX_JOINT_SPEED
 *        opts.speedScale   速度倍率 0.1~1.0（示教器面板那个 10%~100%）
 *        opts.accelRatio   梯形曲线加速段占比，默认 0.2
 *        opts.loop         末段是否回到起点（画圆时要），默认 false
 * @returns {{
 *   segments: Array<{index:number, from:number, to:number, qA:number[], qB:number[],
 *                    duration:number, baseDuration:number, maxDeltaDeg:number}>,
 *   totalDuration:number, accelRatio:number, speedScale:number, posesRad:number[][]
 * }}
 */
export function buildMovjProgram(posesRad, opts = {}) {
  const {
    maxSpeedRad = DEFAULT_MAX_JOINT_SPEED,
    speedScale = 1,
    accelRatio = 0.2,
    loop = false
  } = opts;

  if (!Array.isArray(posesRad) || posesRad.length < 2) {
    throw new Error('[interp] buildMovjProgram 至少需要 2 个示教位姿');
  }
  const scale = Math.min(1, Math.max(0.01, Number(speedScale) || 1));

  const seq = loop ? [...posesRad, posesRad[0]] : posesRad;
  const segments = [];
  let t0 = 0;

  for (let i = 0; i < seq.length - 1; i++) {
    const qA = seq[i].map(Number);
    const qB = seq[i + 1].map(Number);
    const base = jointSpaceDuration(qA, qB, maxSpeedRad);
    // 速度倍率直接作用在时长上：50% 就是走两倍久
    const duration = Math.max(MIN_SEGMENT_DURATION, base / scale);
    let maxDelta = 0;
    for (let k = 0; k < qA.length; k++) {
      maxDelta = Math.max(maxDelta, Math.abs(qB[k] - qA[k]));
    }
    segments.push({
      index: i,
      from: i,
      to: i + 1,
      qA,
      qB,
      baseDuration: base,
      duration,
      maxDeltaDeg: (maxDelta * 180) / Math.PI,
      tStart: t0,
      tEnd: t0 + duration
    });
    t0 += duration;
  }

  return {
    segments,
    totalDuration: t0,
    accelRatio,
    speedScale: scale,
    posesRad: seq,
    loop
  };
}

/**
 * 按绝对时间取程序在某一刻的关节角。A 的动画循环就是每帧调这个。
 *
 * @param {object} program buildMovjProgram 的返回值
 * @param {number} t       秒
 * @returns {{q:number[], segIndex:number, s:number, u:number, done:boolean}}
 */
export function sampleProgram(program, t) {
  const T = program.totalDuration;
  const tt = Math.min(Math.max(t, 0), T);

  let seg = program.segments[program.segments.length - 1];
  for (const s of program.segments) {
    if (tt < s.tEnd) { seg = s; break; }
  }

  const span = seg.duration || 1;
  const s = Math.min(1, Math.max(0, (tt - seg.tStart) / span));
  const u = trapezoidProgress(s, program.accelRatio);
  const q = jointInterp(seg.qA, seg.qB, s, {
    profile: 'trapezoid',
    accelRatio: program.accelRatio
  });

  return { q, segIndex: seg.index, s, u, done: tt >= T - 1e-9 };
}

/**
 * 把整段程序离散成时间轴采样（离线出图、录制回放都用这个）。
 *
 * @param {object} program
 * @param {object} [opts] { fps = 120, duration = program.totalDuration }
 * @returns {Array<{t:number, q:number[], segIndex:number, s:number}>}
 */
export function programTimeline(program, opts = {}) {
  const fps = opts.fps ?? 120;
  const duration = opts.duration ?? program.totalDuration;
  const steps = Math.max(2, Math.round(duration * fps));
  const out = [];
  for (let k = 0; k <= steps; k++) {
    const t = (duration * k) / steps;
    out.push({ t, ...sampleProgram(program, t) });
  }
  return out;
}

/**
 * 程序自检：所有段的关节角是否都在限位内、相邻段衔接处有没有跳变。
 * 这是"要求 b 连续执行"最容易被老师追问的点——段与段之间不能有突变。
 *
 * @param {object} program
 * @param {{lower:number[], upper:number[]}} limits 弧度
 */
export function checkProgram(program, limits) {
  const issues = [];
  let maxJumpDeg = 0;
  let jumpAt = -1;

  program.segments.forEach((seg, i) => {
    seg.qA.forEach((v, k) => {
      if (v < limits.lower[k] - 1e-9 || v > limits.upper[k] + 1e-9) {
        issues.push(`段${i + 1} 起点 ${JOINT_NAMES[k]}=${((v * 180) / Math.PI).toFixed(2)}° 超限`);
      }
    });
    seg.qB.forEach((v, k) => {
      if (v < limits.lower[k] - 1e-9 || v > limits.upper[k] + 1e-9) {
        issues.push(`段${i + 1} 终点 ${JOINT_NAMES[k]}=${((v * 180) / Math.PI).toFixed(2)}° 超限`);
      }
    });
    if (i > 0) {
      let d = 0;
      for (let k = 0; k < seg.qA.length; k++) {
        d = Math.max(d, Math.abs(seg.qA[k] - program.segments[i - 1].qB[k]));
      }
      const dDeg = (d * 180) / Math.PI;
      if (dDeg > maxJumpDeg) { maxJumpDeg = dDeg; jumpAt = i; }
      if (dDeg > 1e-6) issues.push(`段${i} 与段${i + 1} 衔接处跳变 ${dDeg.toFixed(3)}°`);
    }
  });

  return {
    ok: issues.length === 0,
    issues,
    maxJumpDeg,
    jumpAt,
    totalDuration: program.totalDuration,
    segmentCount: program.segments.length
  };
}

export default { buildMovjProgram, sampleProgram, programTimeline, checkProgram };
