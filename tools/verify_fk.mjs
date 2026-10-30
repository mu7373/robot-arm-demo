/**
 * verify_fk.mjs —— 三档 FK 验证的 JS 侧。产出「待对拍」数据，由 verify_fk.py 独立复算比对。
 *
 *   第一档：全零位形，末端位姿必须等于模型零位形的基准 TCP
 *   第二档：单关节转到 ±90°，逐一隔离每个关节的轴向与枢轴
 *   第三档：限位内随机 100 组关节角，与独立的 numpy 实现逐点对拍
 *
 * 用法：node tools/verify_fk.mjs
 */
import { readFileSync, writeFileSync } from 'node:fs';
import {
  createModel, forwardKinematics, degToRad, radToDeg, makeRng, JOINT_NAMES
} from '../src/analysis/fk.js';

const model = createModel(JSON.parse(readFileSync(new URL('../assets/robot-model.json', import.meta.url))));
const r5 = (v) => v;   // 全精度落盘，否则比对误差里混进的是 JSON 舍入，不是数学误差

const result = { zero: null, single: [], random: [] };

/* ── 第一档：全零位形 ── */
{
  const { tcp } = forwardKinematics(model, [0, 0, 0, 0, 0]);
  result.zero = {
    p: tcp.p.map(r5),
    rpyDeg: tcp.rpy.map((a) => Number(radToDeg(a).toFixed(6))),
    quat: tcp.quat.map(r5)
  };
}

/* ── 第二档：单关节 ±90° ── */
for (let i = 0; i < JOINT_NAMES.length; i++) {
  for (const deg of [90, -90]) {
    const q = [0, 0, 0, 0, 0];
    q[i] = degToRad(deg);
    const { tcp } = forwardKinematics(model, q);
    result.single.push({
      joint: JOINT_NAMES[i], deg, i,
      q: q.map(r5),
      p: tcp.p.map(r5),
      rpyDeg: tcp.rpy.map((a) => Number(radToDeg(a).toFixed(6)))
    });
  }
}

/* ── 第三档：随机 100 组（可复现） ── */
{
  const rnd = makeRng(12345);
  for (let k = 0; k < 100; k++) {
    const q = model.joints.map((j) => j.lower + (j.upper - j.lower) * rnd());
    const { tcp } = forwardKinematics(model, q);
    result.random.push({ q: q.map(r5), p: tcp.p.map(r5), rpyDeg: tcp.rpy.map((a) => Number(radToDeg(a).toFixed(6))) });
  }
}

const out = new URL('../report/_fk_js.json', import.meta.url);
writeFileSync(out, JSON.stringify(result, null, 1));
console.log('[JS] 零位 TCP =', result.zero.p.map((v) => v.toFixed(6)).join(', '));
console.log('[JS] 单关节样本', result.single.length, '组；随机样本', result.random.length, '组');
console.log('[JS] 已写', out.pathname);

