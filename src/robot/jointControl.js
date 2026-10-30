/**
 * jointControl.js 关节控制、限位钳制
 * THREE 对象从函数参数传入，文件内部不再 import three
 */

function clamp(val, min, max) {
  return Math.max(min, Math.min(max, val));
}

/**
 * 设置单个关节角度（带限位）
 * @param {object} THREE 页面传入的three对象
 * @param {object} joint 关节对象 {bone, qZero, min, max}
 * @param {number} theta 目标角度，弧度
 */
export function setJointAngle(THREE, joint, theta) {
  const safeTheta = clamp(theta, joint.min, joint.max);
  const r = new THREE.Quaternion();
  r.setRotationFromAxisAngle(new THREE.Vector3(0,0,1), safeTheta);
  joint.bone.quaternion.copy(joint.qZero).multiply(r);
  joint.bone.updateMatrixWorld(true);
}

/**
 * 批量设置全部关节
 * @param {object} THREE
 * @param {Array} joints robotLoader返回的joints数组
 * @param {number[]} thetas [J1,J2,J3,J4,J5] 弧度数组
 */
export function setAllJoints(THREE, joints, thetas) {
  for(let i=0;i<joints.length;i++){
    setJointAngle(THREE, joints[i], thetas[i]);
  }
}