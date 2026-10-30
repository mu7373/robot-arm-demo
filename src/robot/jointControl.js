/**
 * jointControl.js 关节控制、限位钳制
 * THREE 对象从函数参数传入，文件内部不再 import three
 *
 * 关节对象以 contract.js / robotLoader.js 的输出为准：
 *   { name, node, axis, lower, upper, angle, qZero, spec }
 * 同时兼容旧写法 { bone, min, max }，免得换成旧对象就直接崩。
 *
 * 位姿公式与 contract.js 一致：q = qZero ⊗ Rz(θ)，不是直接覆盖 rotation。
 */

const LOCAL_Z = [0, 0, 1];

function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

/** 读取关节节点：新字段 node，旧字段 bone 都认 */
function jointNode(joint) {
  return joint?.node ?? joint?.bone ?? null;
}

/** 读取限位（弧度）：新字段 lower/upper，旧字段 min/max 都认 */
function jointLimits(joint) {
  const lower = joint?.lower ?? joint?.min;
  const upper = joint?.upper ?? joint?.max;
  const ok = Number.isFinite(lower) && Number.isFinite(upper) && lower < upper;
  if (!ok) {
    throw new Error(
      `[jointControl] 关节「${joint?.name ?? '?'}」缺少有效限位：` +
      `lower=${joint?.lower ?? joint?.min}, upper=${joint?.upper ?? joint?.max}`
    );
  }
  return { lower, upper };
}

/**
 * 设置单个关节角度（带限位）
 * @param {object} THREE 页面传入的three对象
 * @param {object} joint 关节对象 {name,node,axis,lower,upper,angle,qZero}
 * @param {number} theta 目标角度，弧度
 * @returns {number} 实际生效的弧度（被钳制后的值）
 */
export function setJointAngle(THREE, joint, theta) {
  const node = jointNode(joint);
  if (!node) throw new Error(`[jointControl] 关节「${joint?.name ?? '?'}」没有 node（或旧字段 bone）`);
  if (!joint.qZero) throw new Error(`[jointControl] 关节「${joint?.name ?? '?'}」缺少 qZero 静止姿态`);

  const { lower, upper } = jointLimits(joint);
  const raw = Number(theta);
  const safeTheta = clamp(Number.isFinite(raw) ? raw : 0, lower, upper);

  const axis = joint.axis instanceof THREE.Vector3
    ? joint.axis
    : new THREE.Vector3(LOCAL_Z[0], LOCAL_Z[1], LOCAL_Z[2]);

  const r = new THREE.Quaternion().setFromAxisAngle(axis, safeTheta);
  node.quaternion.copy(joint.qZero).multiply(r);
  node.updateMatrixWorld(true);

  joint.angle = safeTheta;   // 与 contract.js 的 joint.angle 保持同一语义
  return safeTheta;
}

/**
 * 批量设置全部关节
 * @param {object} THREE
 * @param {Array} joints robotLoader返回的joints数组
 * @param {number[]} thetas [J1,J2,J3,J4,J5] 弧度数组，缺省按 0 处理
 * @returns {number[]} 实际生效的弧度数组
 */
export function setAllJoints(THREE, joints, thetas) {
  const applied = [];
  for (let i = 0; i < joints.length; i++) {
    applied.push(setJointAngle(THREE, joints[i], thetas?.[i] ?? 0));
  }
  return applied;
}