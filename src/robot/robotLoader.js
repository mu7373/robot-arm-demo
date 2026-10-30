// robotLoader.js
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { normalizeSpec } from '../contract.js';

/**
 * 工具：收集模型内部所有节点名称，用于报错调试
 * @param {THREE.Object3D} root
 * @returns {string[]}
 */
function collectNames(root) {
  const out = [];
  root.traverse((o) => { if (o.name) out.push(o.name); });
  return out;
}

/**
 * 独立机器人加载器 robotLoader
 * @param {string} glbUrl glb模型路径
 * @param {object} jointMap joint_map.json解析后的对象
 * @returns {Promise<{robot:THREE.Object3D,joints:Array,tcp:THREE.Object3D}>}
 */
export async function robotLoader(glbUrl, jointMap) {
  const loader = new GLTFLoader();
  const gltf = await loader.loadAsync(glbUrl);
  const robot = gltf.scene;

  // 查找tcp末端节点
  const tcp = robot.getObjectByName('tcp', true);
  if (!tcp) {
    throw new Error(`[robotLoader] GLB找不到tcp节点。现有节点：${collectNames(robot).join(', ')}`);
  }

  // 解析joint_map，兼容新旧格式
  const specs = normalizeSpec(jointMap);
  const joints = [];
  const missingNode = [];

  const LOCAL_AXIS = new THREE.Vector3(0,0,1);

  for (const js of specs) {
    const nodeName = js.node || js.name;
    const node = robot.getObjectByName(nodeName, true);
    if (!node) {
      missingNode.push(nodeName);
      continue;
    }
    // 角度限位 deg -> rad
    const lower = THREE.MathUtils.degToRad(Number(js.limit?.lower ?? -180));
    const upper = THREE.MathUtils.degToRad(Number(js.limit?.upper ?? 180));

    joints.push({
      name: js.name,
      node,
      axis: LOCAL_AXIS.clone(),
      lower,
      upper,
      angle: 0,
      qZero: node.quaternion.clone(), // 保存模型原始静止姿态
      spec: js
    });
  }

  // -------- 核心校验：确认5个关节全部识别 --------
  if (missingNode.length > 0) {
    throw new Error(`[robotLoader] 缺失骨骼节点：${missingNode.join(', ')}`);
  }
  if (joints.length !== 5) {
    throw new Error(`[robotLoader] 关节数量校验失败！期望=5，实际读到=${joints.length}`);
  }

  // 打印识别到的关节名字，控制台看J1‑J5是否齐全
  console.log('[robotLoader] ✅识别成功，关节列表：', joints.map(j=>j.name));

  return { robot, joints, tcp };
}