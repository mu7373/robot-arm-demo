/**
 * fkVerify.js —— 正运动学三档验证（B 侧：分工里"完成三档正确性验证"）
 *
 * 验证什么：用 fk.js 里"自己手写的 FK"去核对 three.js 场景树算出来的 matrixWorld。
 * 两条链路完全独立：
 *   · fk.js       —— 读 robot-model.json（关节名、局部轴、限位、静止四元数 + 节点位移），
 *                    自己做四元数连乘；
 *   · three 场景树 —— GLTFLoader 建出来的父子关系，updateMatrixWorld 逐级乘。
 * 两条独立实现给出同一个数，才说明"我对模型的理解"是错的概率很低。
 *
 * 三档的用意不同（PPT 第 30 页的验证方法）：
 *   一档 全零位形   → 证明基准位形对。错了说明原点/单位/世界变换有问题；
 *   二档 单关节 ±90° → 隔离轴向与枢轴。一次只动一个关节，
 *                     错了能直接定位到是哪个关节配错了轴；
 *   三档 随机 100 组 → 证明不是"碰巧在几个特例上对"，全空间都对。
 *
 * 本文件不 import three：只用到 Object3D 的 getObjectByName / updateMatrixWorld /
 * quaternion.set / matrixWorld.elements 这几个通用接口，因此在 node 里也能跑。
 *
 * 依赖：fk.js
 */

import {
  JOINT_NAMES,
  forwardKinematics,
  quatMul,
  quatFromAxisAngle,
  quatNormalize,
  radToDeg
} from './fk.js';

/**
 * 零位形 TCP 的权威值（模型单位）。
 * 由 tools/verify_fk.py 独立算出：直接读 GLB 顶点、按节点层级连乘，
 * 不经过 three、也不经过 fk.js。三档验证拿它当"外部基准"。
 */
export const GLB_BASELINE_TCP = Object.freeze([
  -1.6009807921272521,
  12.491080831009272,
  -5.1267520973035881
]);

/** 位置误差判据（模型单位；1u = 100 mm，1e-9 u = 0.1 nm，远严于工程需要） */
export const POS_TOL = 1e-9;
/**
 * 姿态误差判据（度）。
 *
 * 为什么不是 1e-6：GLB 里的旋转四元数本身是 float32 存的，相对精度约 1.2e-7，
 * 换算成角度就是 6.9e-6° 量级。也就是说哪怕两条链路都完全正确，
 * 姿态夹角也不可能是 0——1e-4° 已经在 float32 噪声之上留了一个数量级余量，
 * 同时仍然比任何有物理意义的姿态误差（哪怕 0.001°）严得多。
 */
export const RPY_TOL = 1e-4;

const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

/**
 * 把关节角写进 three 场景树（等价于 A 的 setJoint：qZero ⊗ R_axis(θ)）。
 * 不用 contract.js 的 setJoint，是为了不依赖它的私有实现细节。
 */
function applyToScene(model, robot, qRad) {
  model.joints.forEach((j, i) => {
    const node = robot.getObjectByName(j.node, true);
    if (!node) throw new Error(`[fkVerify] 场景里找不到关节节点 ${j.node}`);
    const theta = Math.min(model.upper[i], Math.max(model.lower[i], Number(qRad?.[i]) || 0));
    const q = quatNormalize(quatMul(j.qZero, quatFromAxisAngle(j.axis, theta)));
    node.quaternion.set(q[0], q[1], q[2], q[3]);
  });
  robot.updateMatrixWorld(true);
}

/** 从场景树读出 tcp 的世界坐标 */
function sceneTCP(robot, tcpName) {
  const tcp = robot.getObjectByName(tcpName, true);
  if (!tcp) throw new Error(`[fkVerify] 场景里找不到 ${tcpName}`);
  const e = tcp.matrixWorld.elements;      // three 的矩阵是列主序
  return [e[12], e[13], e[14]];
}

/**
 * 从 matrixWorld 里取出世界姿态（四元数），列主序，并先归一化掉缩放。
 *
 * 注意：不能用 tcp.quaternion —— 那是节点自己的【局部】静止姿态，
 * 而 FK 算出来的是【世界】姿态，两者根本不是同一个东西（差着整条父链）。
 * 直接对比会得到离谱的 180°，这正是第一版验证误报 FAIL 的原因。
 */
function sceneWorldQuat(robot, tcpName) {
  const tcp = robot.getObjectByName(tcpName, true);
  const e = tcp.matrixWorld.elements;
  // 三列各自归一化，去掉可能的缩放
  const col = (i) => {
    const v = [e[i], e[i + 1], e[i + 2]];
    const n = Math.hypot(v[0], v[1], v[2]) || 1;
    return v.map((x) => x / n);
  };
  const c0 = col(0);
  const c1 = col(4);
  const c2 = col(8);
  const m00 = c0[0], m10 = c0[1], m20 = c0[2];
  const m01 = c1[0], m11 = c1[1], m21 = c1[2];
  const m02 = c2[0], m12 = c2[1], m22 = c2[2];

  const tr = m00 + m11 + m22;
  let x, y, z, w, s;
  if (tr > 0) {
    s = Math.sqrt(tr + 1) * 2;
    w = 0.25 * s;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    w = (m21 - m12) / s;
    x = 0.25 * s;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
  } else if (m11 > m22) {
    s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    w = (m02 - m20) / s;
    x = (m01 + m10) / s;
    y = 0.25 * s;
    z = (m12 + m21) / s;
  } else {
    s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    w = (m10 - m01) / s;
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = 0.25 * s;
  }
  return quatNormalize([x, y, z, w]);
}

/**
 * 三档验证。
 *
 * @param {object} model   createModel(robot-model.json) 的结果
 * @param {object} robot   three 的 GLTF scene（A 的 loadRobot 返回值里的 robot）
 * @param {object} [opts]
 *        opts.randomCount 三档随机组数，默认 100
 *        opts.seed        随机种子，保证可复现
 * @returns {object} 逐档明细 + 总结论
 */
export function verifyFK(model, robot, opts = {}) {
  if (!model || !robot) throw new Error('[fkVerify] 需要 (model, robot) 两个参数');
  const N = opts.randomCount ?? 100;
  const tcpName = model.tcpNode ?? 'tcp';
  const zeros = JOINT_NAMES.map(() => 0);

  /* ── 一档：全零位形 ── */
  applyToScene(model, robot, zeros);
  const fkZero = forwardKinematics(model, zeros).tcp.p;
  const scZero = sceneTCP(robot, tcpName);
  const tier1 = {
    fk: fkZero,
    scene: scZero,
    baseline: GLB_BASELINE_TCP,
    errVsBaseline: dist(fkZero, GLB_BASELINE_TCP),
    errVsScene: dist(fkZero, scZero)
  };
  tier1.pass = tier1.errVsBaseline < 1e-6 && tier1.errVsScene < POS_TOL;

  /* ── 二档：单关节 ±90° ── */
  const tier2 = { rows: [], maxErr: 0 };
  for (let i = 0; i < model.joints.length; i++) {
    for (const sign of [+1, -1]) {
      const q = zeros.slice();
      q[i] = (sign * Math.PI) / 2;
      applyToScene(model, robot, q);
      const fkP = forwardKinematics(model, q).tcp.p;
      const scP = sceneTCP(robot, tcpName);
      const e = dist(fkP, scP);
      tier2.rows.push({
        joint: JOINT_NAMES[i],
        angleDeg: 90 * sign,
        fk: fkP,
        scene: scP,
        err: e
      });
      tier2.maxErr = Math.max(tier2.maxErr, e);
    }
  }
  tier2.pass = tier2.maxErr < POS_TOL;

  /* ── 三档：随机 N 组 ── */
  let seed = opts.seed ?? 20240607;
  const rnd = () => {
    seed = (seed * 1664525 + 1013904223) >>> 0;
    return seed / 4294967296;
  };
  const tier3 = { rows: 0, maxPosErr: 0, maxRpyErrDeg: 0 };
  for (let k = 0; k < N; k++) {
    const q = model.joints.map((_, i) =>
      model.lower[i] + (model.upper[i] - model.lower[i]) * rnd()
    );
    applyToScene(model, robot, q);
    const fkRes = forwardKinematics(model, q).tcp;
    const scP = sceneTCP(robot, tcpName);
    const e = dist(fkRes.p, scP);
    tier3.maxPosErr = Math.max(tier3.maxPosErr, e);
    tier3.rows++;
    // 姿态：拿【世界】四元数对比，用 |点积| 算夹角，避免 RPY 在 ±180° 处的跳变
    const sq = sceneWorldQuat(robot, tcpName);
    const dotq = Math.abs(
      sq[0] * fkRes.quat[0] + sq[1] * fkRes.quat[1] + sq[2] * fkRes.quat[2] + sq[3] * fkRes.quat[3]
    );
    const angDeg = radToDeg(2 * Math.acos(Math.min(1, dotq)));
    tier3.maxRpyErrDeg = Math.max(tier3.maxRpyErrDeg, angDeg);
  }
  tier3.pass = tier3.maxPosErr < POS_TOL && tier3.maxRpyErrDeg < RPY_TOL;

  // 收尾：把场景还原成零位，别把界面留在半个姿态上
  applyToScene(model, robot, zeros);

  const pass = tier1.pass && tier2.pass && tier3.pass;
  return {
    pass,
    tier1,
    tier2,
    tier3,
    tolerance: { pos: POS_TOL, rpyDeg: RPY_TOL },
    note: '位置误差单位是模型单位：1u = 100 mm。判据 1e-9u = 0.1 nm，远严于工程精度。'
  };
}

/** 把结果压成中文文本，方便直接贴进实验报告 */
export function formatReport(res) {
  const lines = [];
  lines.push('=== 正运动学三档验证 ===');
  lines.push(`一档 全零位形：FK=(${res.tier1.fk.map((v) => v.toFixed(6)).join(', ')})`);
  lines.push(`  与 GLB 独立基准误差 ${res.tier1.errVsBaseline.toExponential(2)} u`);
  lines.push(`  与场景树误差     ${res.tier1.errVsScene.toExponential(2)} u   ${res.tier1.pass ? 'OK' : 'FAIL'}`);
  lines.push('二档 单关节 ±90°：');
  for (const r of res.tier2.rows) {
    lines.push(
      `  ${r.joint} ${r.angleDeg > 0 ? '+' : ''}${r.angleDeg}°  FK=(${r.fk.map((v) => v.toFixed(4)).join(', ')})` +
      `  |Δ|=${r.err.toExponential(2)}`
    );
  }
  lines.push(`  最大误差 ${res.tier2.maxErr.toExponential(2)} u   ${res.tier2.pass ? 'OK' : 'FAIL'}`);
  lines.push(`三档 随机 ${res.tier3.rows} 组：位置最大 ${res.tier3.maxPosErr.toExponential(2)} u，` +
    `姿态最大 ${res.tier3.maxRpyErrDeg.toExponential(2)}°   ${res.tier3.pass ? 'OK' : 'FAIL'}`);
  lines.push(`结论：${res.pass ? 'PASS' : 'FAIL'}`);
  return lines.join('\n');
}

export default { verifyFK, formatReport, GLB_BASELINE_TCP, POS_TOL, RPY_TOL };
