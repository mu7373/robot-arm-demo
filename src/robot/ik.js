/**
 * ik.js —— 位置逆解（数值法，阻尼最小二乘）
 *
 * 为什么标定要它：四点法要求「工具尖在 4 个不同姿态下碰同一个点」。
 * 纯手工拖滑块几乎不可能把尖精确停到同一个点，所以页面用这里把工具尖
 * 送到指定位置，再记录当时的法兰位姿 —— 和真实示教器"对准标定针"是一个意思。
 *
 * 只做位置（3 个约束），5 个关节有 2 个冗余自由度，正好让不同初值收敛出
 * 不同姿态 —— 四点法需要的姿态差异就是这么来的。
 */

import * as THREE from 'three';

const LOCAL_Z = new THREE.Vector3(0, 0, 1);

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

function limitsOf(joint) {
  const lower = Number.isFinite(joint.lower) ? joint.lower : -Math.PI;
  const upper = Number.isFinite(joint.upper) ? joint.upper : Math.PI;
  return { lower, upper };
}

/**
 * 把工具尖（tcp 世界坐标）送到 target。
 *
 * 位置只有 3 个约束，5 个关节有 2 维零空间。给 nullDir 时，会在保持尖点不动
 * 的前提下沿这个关节方向漂移姿态 —— 四点法需要"同一个点、不同姿态"，
 * 否则不管怎么扰动初值，逆解都会收敛回同一个构型，解出来的工具偏移是 0。
 *
 * @param {object}   opts
 * @param {THREE.Object3D} opts.robot       机器人根节点
 * @param {object}   opts.controller        RobotController（用它的 joints 与 setJoint）
 * @param {THREE.Object3D} opts.tcp        末端节点（用来取尖点世界坐标）
 * @param {THREE.Vector3|number[]} opts.target 目标点
 * @param {number}   [opts.maxIter=80]
 * @param {number}   [opts.tol=1e-4]        位置收敛阈值；给 0 表示"跑满 maxIter"
 * @param {number}   [opts.damping=0.05]    阻尼系数，越大越稳、越慢
 * @param {number}   [opts.maxStep=0.2]     单次迭代每个关节最大步长（rad）
 * @param {number[]} [opts.nullDir=null]    零空间漂移方向（长度 = 关节数）
 * @param {number}   [opts.nullGain=0.02]   每次迭代沿零空间走多远（rad）
 * @returns {{ok:boolean, error:number, iterations:number, angles:number[]}}
 */
export function solvePositionIK({
  robot, controller, tcp, target,
  maxIter = 80, tol = 1e-4, damping = 0.05, maxStep = 0.2,
  nullDir = null, nullGain = 0.02
} = {}) {
  if (!robot || !controller || !tcp) {
    throw new Error('[ik] 需要 robot / controller / tcp');
  }
  const joints = controller.joints;
  const n = joints.length;
  const goal = target.isVector3 ? target.clone() : new THREE.Vector3(target[0], target[1], target[2]);

  const angles = joints.map((j) => Number(j.angle) || 0);

  const apply = (arr) => {
    for (let i = 0; i < n; i++) controller.setJoint(joints[i], arr[i]);
    robot.updateMatrixWorld(true);
  };

  const tip = new THREE.Vector3();

  apply(angles);
  tcp.getWorldPosition(tip);
  let err = tip.distanceTo(goal);
  let iter = 0;

  for (; iter < maxIter; iter++) {
    // 纯位置收敛时，误差足够小就停；带零空间漂移时则跑满 maxIter，
    // 否则"已经在目标点上"会让漂移一次都不执行（err=0 直接退出）。
    if (!nullDir && err <= tol) break;
    const e = new THREE.Vector3().subVectors(goal, tip);

    // 数值雅可比：每列 = 该关节转轴 × (尖点 - 关节)
    const cols = joints.map((joint) => {
      const jp = new THREE.Vector3();
      const jq = new THREE.Quaternion();
      joint.node.getWorldPosition(jp);
      joint.node.getWorldQuaternion(jq);
      const axis = LOCAL_Z.clone().applyQuaternion(jq).normalize();
      const r = new THREE.Vector3().subVectors(tip, jp);
      return new THREE.Vector3().crossVectors(axis, r);
    });

    // M = J·Jᵀ + λ²I（3×3，对称）
    let m00 = 0, m01 = 0, m02 = 0, m11 = 0, m12 = 0, m22 = 0;
    for (const c of cols) {
      m00 += c.x * c.x; m01 += c.x * c.y; m02 += c.x * c.z;
      m11 += c.y * c.y; m12 += c.y * c.z; m22 += c.z * c.z;
    }
    const lam = damping * damping;
    m00 += lam; m11 += lam; m22 += lam;

    const det = m00 * (m11 * m22 - m12 * m12)
      - m01 * (m01 * m22 - m12 * m02)
      + m02 * (m01 * m12 - m11 * m02);
    if (Math.abs(det) < 1e-12) break;   // 奇异位形，救不回来就停

    const i00 = (m11 * m22 - m12 * m12) / det;
    const i01 = (m02 * m12 - m01 * m22) / det;
    const i02 = (m01 * m12 - m02 * m11) / det;
    const i11 = (m00 * m22 - m02 * m02) / det;
    const i12 = (m01 * m02 - m00 * m12) / det;
    const i22 = (m00 * m11 - m01 * m01) / det;

    const y0 = i00 * e.x + i01 * e.y + i02 * e.z;
    const y1 = i01 * e.x + i11 * e.y + i12 * e.z;
    const y2 = i02 * e.x + i12 * e.y + i22 * e.z;

    // 零空间分量：N·g = g - Jᵀ(JJᵀ+λ²I)⁻¹Jg，保证（一阶意义上）尖点不动
    let ng = null;
    if (nullDir) {
      const jgx = cols.reduce((s, c, i) => s + c.x * nullDir[i], 0);
      const jgy = cols.reduce((s, c, i) => s + c.y * nullDir[i], 0);
      const jgz = cols.reduce((s, c, i) => s + c.z * nullDir[i], 0);
      const gx = i00 * jgx + i01 * jgy + i02 * jgz;
      const gy = i01 * jgx + i11 * jgy + i12 * jgz;
      const gz = i02 * jgx + i12 * jgy + i22 * jgz;
      ng = cols.map((c, i) => nullDir[i] - (c.x * gx + c.y * gy + c.z * gz));
    }

    for (let i = 0; i < n; i++) {
      const c = cols[i];
      let dq = c.x * y0 + c.y * y1 + c.z * y2;
      if (ng) dq += nullGain * ng[i];
      dq = clamp(dq, -maxStep, maxStep);
      const { lower, upper } = limitsOf(joints[i]);
      angles[i] = clamp(angles[i] + dq, lower, upper);
    }

    apply(angles);
    tcp.getWorldPosition(tip);
    err = tip.distanceTo(goal);
  }

  return { ok: err <= tol, error: err, iterations: iter, angles: angles.slice() };
}

export default solvePositionIK;
