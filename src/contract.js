/**
 * contract.js —— 实验一 A / B 两人共用的接口契约
 * 计算机图形与动画 · CM11 · 机械臂仿真
 *
 * ── 事实源（唯一）
 *   1) 模型1/images/joint_map.json  关节名、转轴、限位单位deg
 *   2) 运行时 GLB 里各关节节点的静止旋转(qZero)   —— 非单位阵，必须保留
 *   不依赖 GLB extras（模型1 的 GLB 完全没有 extras）
 *   不依赖 robot_spec.yaml / robot.urdf（那两份臂长 3.33，与 GLB 的 6.54 不是同一台）
 *
 * ── 约定
 *   · 关节角内部一律弧度，只有界面显示才转角度
 *   · 关节绕【节点局部 +Z】旋转（joint_map.json 的 convention 原文）
 *   · 位姿 = qZero ⊗ Rz(θ)，不是直接把 rotation 覆盖成 θ
 *   · 位姿一律走 getTCPPose()，关节角一律走 getJointAnglesRad()
 *
 * ── 用法
 *   import { RobotController } from './contract.js';
 *   const ctl = new RobotController();
 *   const spec = await (await fetch('./assets/joint_map.json')).json();
 *   const { robot, joints, tcp } = await ctl.loadRobot('./assets/robot_articulated.glb', spec);
 *   ctl.setJoint(joints[0], Math.PI / 6);
 *   console.table(ctl.dumpJoints());
 *   console.log(ctl.getTCPPose());
 *
 * ── 注意：B 的 fk.js 不要 import 本文件。三档验证要的是"两套独立实现互相印证"，
 *   如果 fk.js 复用这里的场景树，就成了自己验证自己。fk.js 只接收纯数据：
 *   { limits:[{lower,upper}...], dh:[...], qZero:[...], tcpOffsetLocal }。
 */

import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

/* ══════════════════════════════════════════════════════════════
 * 常量
 * ══════════════════════════════════════════════════════════════ */

/** 运动轴：模型1 的 5 个关节全部绕节点局部 +Z（右手定则）。 */
export const LOCAL_AXIS = Object.freeze(new THREE.Vector3(0, 0, 1));

/** MOVJ 默认每个关节的最大速度 rad/s。教学用保守值，调用方可覆盖。 */
export const DEFAULT_MAX_JOINT_SPEED = Object.freeze([
  Math.PI / 2, Math.PI / 2, Math.PI / 2, Math.PI / 2, Math.PI / 2
]);

/** 内部临时量，避免每帧 new */
const _dq = new THREE.Quaternion();
const _v = new THREE.Vector3();
const _qt = new THREE.Quaternion();
const _eu = new THREE.Euler(0, 0, 0, 'XYZ');

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

/* ══════════════════════════════════════════════════════════════
 * 速度规划：梯形曲线
 * ══════════════════════════════════════════════════════════════ */

/**
 * 归一化梯形速度曲线。给定归一化时间 s∈[0,1] 与加速段占比 a∈(0,0.5]，
 * 返回归一化位移 u∈[0,1]：两端速度为 0、中段匀速 —— MOVJ 的标准速度规划。
 * 这就是要求 b「连续执行平滑」、要求 c「速度曲线无突变」的算法依据。
 *
 * 推导：总位移 = 1，面积 = vmax*(1-a)，故 vmax = 1/(1-a)。
 *   s<=a    : u = s² / (2a(1-a))
 *   a<s<1-a : u = (s - a/2) / (1-a)
 *   s>=1-a  : u = 1 - (1-s)² / (2a(1-a))
 */
export function trapezoidProgress(s, a = 0.2) {
  const t = clamp(s, 0, 1);
  const acc = clamp(a, 1e-3, 0.5);
  const k = 2 * acc * (1 - acc);
  if (t <= acc) return (t * t) / k;
  if (t >= 1 - acc) return 1 - ((1 - t) * (1 - t)) / k;
  return (t - acc / 2) / (1 - acc);
}

/* ══════════════════════════════════════════════════════════════
 * 工具
 * ══════════════════════════════════════════════════════════════ */

/** 把 JSON 存成文件下载（Firefox 需要 a 在 DOM 里且延迟 revoke）。 */
export function downloadJSON(filename, data) {
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

/** 兼容两种入参：joint_map.json 原始对象，或旧的 {关节名: 节点名}。 */
export function normalizeSpec(jointMap) {
  if (!jointMap || typeof jointMap !== 'object') {
    throw new Error('[contract] loadRobot 第二个参数应为 joint_map.json 解析后的对象');
  }
  if (Array.isArray(jointMap.joints)) {
    return jointMap.joints;
  }
  const keys = Object.keys(jointMap);
  if (keys.length && keys.every((k) => typeof jointMap[k] === 'string')) {
    console.warn(
      '[contract] 收到旧的 {关节名: 节点名} 映射，缺限位与转轴信息，全部按 ±180° 处理。\n' +
      '          请改传 joint_map.json（J2/J3/J4 的真实限位是 ±120°）。'
    );
    return keys.map((n) => ({ name: n, axis: 'z', limit: { lower: -180, upper: 180 } }));
  }
  throw new Error(
    '[contract] 无法识别的 jointMap 结构。需要 joint_map.json（顶层含 joints 数组），' +
    '或旧的 {关节名: 节点名}。收到顶层键：' + keys.join(', ')
  );
}

function collectNames(root) {
  const out = [];
  root.traverse((o) => { if (o.name) out.push(o.name); });
  return out;
}

/* ══════════════════════════════════════════════════════════════
 * RobotController
 * ══════════════════════════════════════════════════════════════ */

export class RobotController {
  constructor() {
    this.loader = new GLTFLoader();
    this.robot = null;
    this.joints = [];
    this.tcp = null;
    this.spec = null;

    this.samples = [];
    this._recording = false;
    this._t0 = 0;
    this._frame = 0;
  }

  /* ────────────── 加载 ────────────── */

  /**
   * @param {string} url        GLB 路径，如 './assets/robot_articulated.glb'
   * @param {object} jointMap   joint_map.json 解析后的对象
   * @param {object} [opts]     { expectJointCount?: number }
   * @returns {Promise<{robot: THREE.Object3D, joints: Array, tcp: THREE.Object3D}>}
   */
  async loadRobot(url, jointMap, opts = {}) {
    const gltf = await this.loader.loadAsync(url);
    this.robot = gltf.scene;
    this.spec = jointMap;

    // tcp 找不到绝不能悄悄兜底 —— 孤儿 Object3D 的 matrixWorld 恒为单位阵，读数永远是 0,0,0
    this.tcp = this.robot.getObjectByName('tcp', true) || null;
    if (!this.tcp) {
      throw new Error(
        '[contract] GLB 里找不到名为 "tcp" 的节点。\n' +
        '          模型1 的期望链：link_base → J1 → link_1 → J2 → link_2 → J3 → link_3 → J4 → link_4 → J5 → link_5 → link_flange → tcp\n' +
        '          实际节点：' + collectNames(this.robot).join(', ')
      );
    }

    const specs = normalizeSpec(jointMap);
    const joints = [];
    const missingNode = [];
    const badAxis = [];

    for (const js of specs) {
      const axisName = String(js.axis ?? 'z').toLowerCase();
      if (axisName !== 'z') badAxis.push(`${js.name}:${js.axis}`);

      const nodeName = js.node || js.name;
      const node = this.robot.getObjectByName(nodeName, true);
      if (!node) { missingNode.push(nodeName); continue; }

      const lower = THREE.MathUtils.degToRad(Number(js.limit?.lower ?? -180));
      const upper = THREE.MathUtils.degToRad(Number(js.limit?.upper ?? 180));
      if (!Number.isFinite(lower) || !Number.isFinite(upper) || !(lower < upper)) {
        throw new Error(`[contract] 关节 ${js.name} 的限位非法：lower=${js.limit?.lower} upper=${js.limit?.upper}`);
      }

      joints.push({
        name: js.name,
        node,
        axis: LOCAL_AXIS.clone(),
        lower,
        upper,
        angle: 0,                              // 弧度
        qZero: node.quaternion.clone(),        // 静止旋转 R_fix，非单位阵，必须保住
        spec: js
      });
    }

    if (badAxis.length) {
      throw new Error(
        '[contract] 只支持"绕节点局部 +Z 旋转"的约定，以下关节不符：' + badAxis.join(', ') +
        '\n          若换模型，请先确认 joint_map.json 的 convention 与节点实测一致。'
      );
    }
    if (missingNode.length) {
      throw new Error(
        '[contract] GLB 里找不到这些关节节点：' + missingNode.join(', ') +
        '\n          现有节点：' + collectNames(this.robot).join(', ')
      );
    }

    const expect = opts.expectJointCount ?? specs.length;
    if (joints.length !== specs.length || joints.length !== expect) {
      // 关键断言：这里不 throw 的话，就是"空数组也能 resolve"那个坑
      throw new Error(
        `[contract] 关节数量不对：spec 写 ${specs.length}，实际找到 ${joints.length}，期望 ${expect}`
      );
    }
    if (specs.length !== 5) {
      console.warn(`[contract] 本课件按 5 自由度讲，当前 spec 是 ${specs.length} 关节，确认用的是模型1。`);
    }

    this.joints = joints;
    this.resetPose();                 // 全部归零并写入 qZero，再统一推一次世界矩阵
    return { robot: this.robot, joints: this.joints, tcp: this.tcp };
  }

  /* ────────────── 关节 ────────────── */

  getJoint(name) {
    return this.joints.find((j) => j.name === name) || null;
  }

  /**
   * 驱动单关节。内部钳制，返回实际生效的弧度。
   * 注意不是覆盖 rotation，而是 qZero ⊗ Rz(θ) —— 丢掉 qZero 会让整条臂姿态全错。
   */
  setJoint(joint, angleRad) {
    if (!joint || !joint.node) throw new Error('[contract] setJoint 收到无效的 joint');
    const a = clamp(Number(angleRad) || 0, joint.lower, joint.upper);
    joint.angle = a;
    joint.node.quaternion.copy(joint.qZero).multiply(_dq.setFromAxisAngle(LOCAL_AXIS, a));
    return a;
  }

  setJointByName(name, angleRad) {
    return this.setJoint(this.getJoint(name), angleRad);
  }

  /** 一次性设整组关节角（弧度）。 */
  setAllJoints(radArray) {
    const out = [];
    for (let i = 0; i < this.joints.length; i++) {
      out.push(this.setJoint(this.joints[i], radArray?.[i] ?? 0));
    }
    return out;
  }

  applyPose(poseRad) { return this.setAllJoints(poseRad); }

  /** 回零位形（各关节 0°）。 */
  resetPose() {
    return this.setAllJoints(this.joints.map(() => 0));
  }

  getJointAnglesRad() { return this.joints.map((j) => j.angle); }
  getJointAnglesDeg() { return this.joints.map((j) => THREE.MathUtils.radToDeg(j.angle)); }

  /** 供 console.table 用（调试三板斧之一）。 */
  dumpJoints() {
    return this.joints.map((j) => ({
      名称: j.name,
      节点: j.node.name,
      角度: `${THREE.MathUtils.radToDeg(j.angle).toFixed(3)}°`,
      限位: `${THREE.MathUtils.radToDeg(j.lower).toFixed(0)}° ~ ${THREE.MathUtils.radToDeg(j.upper).toFixed(0)}°`
    }));
  }

  /* ────────────── 位姿 ────────────── */

  /**
   * 末端 TCP 位姿。每次调用先 robot.updateMatrixWorld(true)，把父链一路推到根
   * ——只更新 tcp 子树（tcp.updateMatrixWorld）会让滑块动完读到上一帧的数。
   * @returns {{p:{x,y,z}, rpy:{r,p,y}, quat:{x,y,z,w}}}
   */
  getTCPPose() {
    if (!this.robot || !this.tcp) throw new Error('[contract] 还没加载模型，先 await loadRobot()');
    this.robot.updateMatrixWorld(true);

    this.tcp.getWorldPosition(_v);
    this.tcp.getWorldQuaternion(_qt);
    _eu.setFromQuaternion(_qt, 'XYZ');   // 只在显示层转 RPY，内部一律四元数

    return {
      p: { x: _v.x, y: _v.y, z: _v.z },
      rpy: { r: _eu.x, p: _eu.y, y: _eu.z },
      quat: { x: _qt.x, y: _qt.y, z: _qt.z, w: _qt.w }
    };
  }

  /** 末端 TCP 的 4×4 世界矩阵（标定 / 对拍用）。 */
  getTCPMatrix4() {
    if (!this.robot || !this.tcp) throw new Error('[contract] 还没加载模型');
    this.robot.updateMatrixWorld(true);
    return this.tcp.matrixWorld.clone();
  }

  /* ────────────── MOVJ 插值 ────────────── */

  /**
   * 关节空间插值（MOVJ 内核）。A 的面板只传进度 s，不关心曲线。
   * @param {number[]} poseA 起点关节角（弧度）
   * @param {number[]} poseB 终点关节角（弧度）
   * @param {number}   s     归一化时间 ∈[0,1]
   * @param {object}  [opts] { profile: 'trapezoid'|'linear'|'smoothstep', accelRatio: 0.2 }
   * @returns {number[]} 插值后的关节角（弧度），长度与 poseA 一致
   */
  jointInterp(poseA, poseB, s, opts = {}) {
    const { profile = 'trapezoid', accelRatio = 0.2 } = opts;
    let u;
    if (profile === 'linear') {
      u = clamp(s, 0, 1);
    } else if (profile === 'smoothstep') {
      const t = clamp(s, 0, 1);
      u = t * t * (3 - 2 * t);
    } else {
      u = trapezoidProgress(s, accelRatio);
    }
    return poseA.map((v, i) => v + ((poseB[i] ?? v) - v) * u);
  }

  /**
   * 按关节最大速度算这段 MOVJ 要走多久（秒）。T = max_i |Δθ_i| / ω_i。
   * 有了 T，进度就是 s = t/T，速度倍率直接乘在 T 上 —— 面板那个 10%~100% 就落在这里。
   */
  static jointSpaceDuration(poseA, poseB, maxSpeedRad = DEFAULT_MAX_JOINT_SPEED) {
    let T = 0;
    for (let i = 0; i < poseA.length; i++) {
      const w = Array.isArray(maxSpeedRad) ? maxSpeedRad[i] : maxSpeedRad;
      const d = Math.abs((poseB[i] ?? poseA[i]) - poseA[i]);
      if (w > 0) T = Math.max(T, d / w);
    }
    return T;
  }

  /* ────────────── 轨迹记录（要求 c 的数据来源） ────────────── */

  recordStart() {
    this.samples = [];
    this._frame = 0;
    this._t0 = performance.now();       // 归零，否则算出来的是绝对时间戳，速度是垃圾
    this._recording = true;
  }

  /**
   * 每帧调一次。extra 可塞 { instruction:'MOVJ', segment:2, target:'pose3' }。
   * t 从 0 开始按秒递增，配 frame 序号 —— 掉帧时才算得出真实 dt。
   */
  appendSample(extra = {}) {
    if (!this._recording) return null;
    const now = performance.now();
    const s = {
      frame: this._frame++,
      t: (now - this._t0) / 1000,
      jointsRad: this.getJointAnglesRad(),
      jointsDeg: this.getJointAnglesDeg(),
      tcp: this.getTCPPose(),
      ...extra
    };
    this.samples.push(s);
    return s;
  }

  /** @returns {Array} 采样数组；download=true 时同时触发下载 */
  recordStop(opts = {}) {
    const { download = true, filename = 'samples.json' } = opts;
    this._recording = false;
    if (download) downloadJSON(filename, this.samples);
    return this.samples;
  }

  getSamples() { return this.samples; }
  get isRecording() { return this._recording; }
}

export default RobotController;
