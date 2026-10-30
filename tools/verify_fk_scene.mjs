/**
 * verify_fk_scene.mjs —— 在没有浏览器的环境下跑一遍"三档验证"
 *
 * 页面上的「运行三档验证」按钮做的是同一件事，但它要求你开着浏览器、手动点。
 * 这个脚本让你在命令行里一条命令跑完，方便：
 *   · 提交前自检（不用开浏览器）
 *   · 在服务器 / CI 上跑
 *   · 把输出直接贴进报告
 *
 * 它做的是：用 three.js 的 GLTFLoader 真的把 GLB 解析成场景树，
 * 再用 fk.js 那套自己写的四元数连乘去对拍——和页面里完全一样的逻辑。
 *
 * 用法：
 *   node tools/verify_fk_scene.mjs
 *   npm run verify:scene
 *
 * 注：node 里没有 DOM，GLTFLoader 加载纹理时会去碰 self / createImageBitmap /
 * URL.createObjectURL，所以下面先补几个桩。我们只要几何与节点层级，纹理无所谓。
 */

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import * as THREE from 'three';

// —— DOM 桩：必须在 import GLTFLoader 之前就位 ——
globalThis.self = globalThis;
globalThis.createImageBitmap = async () => ({ width: 1, height: 1, close() {} });
if (!globalThis.URL.createObjectURL) globalThis.URL.createObjectURL = () => 'blob:stub';
if (!globalThis.URL.revokeObjectURL) globalThis.URL.revokeObjectURL = () => {};

const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
const { createModel } = await import('../src/analysis/fk.js');
const { verifyFK, formatReport } = await import('../src/analysis/fkVerify.js');

const ROOT = join(dirname(fileURLToPath(import.meta.url)), '..');

const glb = readFileSync(join(ROOT, 'assets', 'robot_articulated.glb'));
const ab = glb.buffer.slice(glb.byteOffset, glb.byteOffset + glb.byteLength);

const gltf = await Promise.race([
  new GLTFLoader().parseAsync(ab, ''),
  new Promise((_, rej) =>
    setTimeout(() => rej(new Error('GLTFLoader 解析超时（15s）')), 15000)
  )
]);

const model = createModel(
  JSON.parse(readFileSync(join(ROOT, 'assets', 'robot-model.json'), 'utf8'))
);

const res = verifyFK(model, gltf.scene, { randomCount: 100 });

console.log(formatReport(res));
console.log('');
console.log('判据：位置 < ' + res.tolerance.pos.toExponential(0) + ' u，' +
  '姿态 < ' + res.tolerance.rpyDeg.toExponential(0) + '°');
console.log('说明：' + res.note);
console.log('');

if (!res.pass) {
  console.error('✗ 验证未通过，先别提交。');
  process.exit(1);
}
console.log('✓ 三档验证通过。');
