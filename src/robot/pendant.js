/**
 * pendant.js —— 机械臂示教器面板（纯 DOM，不依赖 three，不依赖 CSS 文件）
 *
 * ── 功能（对应报告「示教器」一栏）
 *   · 5 个关节滑块（J1~J5）
 *   · 每个关节的数值输入（单位：度）
 *   · 每个关节的当前角度显示
 *   · 末端 TCP 的 xyz / RPY / 四元数
 *   · 使能 / 失能
 *   · 急停（急停后必须复位才能再次使能）
 *   · 速度倍率 10% ~ 100%（影响关节从当前角走到目标角的速度）
 *
 * ── 为什么读数可信
 *   面板上所有角度都是【每一帧从场景真读】出来的，不是把滑块值抄一遍：
 *   关节角用 q = qZero ⊗ Rz(θ) 反解（θ = 2·atan2(rel.z, rel.w)），
 *   TCP 用 controller.getTCPPose()（内部先 robot.updateMatrixWorld(true) 再取世界矩阵）。
 *   所以拖滑块时只有“目标”在变，而“当前”始终等于模型真实姿态。
 *
 * ── 用法
 *   import { RobotController } from './src/contract.js';
 *   import { createPendant } from './src/robot/pendant.js';
 *
 *   const ctl = new RobotController();
 *   await ctl.loadRobot('./assets/robot_articulated.glb', jointMap);
 *   const pendant = createPendant({ container: '#panel', controller: ctl });
 *
 *   兼容 robotLoader 的返回：把 { robot, joints } 里的 joints 塞进一个
 *   带 joints 字段的对象即可（没有 setJoint 时本文件自己写节点四元数）。
 */

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;

const SPEED_MIN = 10;
const SPEED_MAX = 100;

/** 与 contract.js 的 DEFAULT_MAX_JOINT_SPEED 保持一致（rad/s） */
const DEFAULT_MAX_JOINT_SPEED = Math.PI / 2;

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

/* ══════════════════════════════════════════════════════════════
 * 四元数小工具（只用来从场景反解关节角，不引入 three）
 * ══════════════════════════════════════════════════════════════ */

function qMul(a, b) {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
  };
}

const qConj = (q) => ({ x: -q.x, y: -q.y, z: -q.z, w: q.w });

function nodeOf(joint) {
  return joint.node || joint.bone || null;
}

function limitsOf(joint) {
  const lower = Number.isFinite(joint.lower) ? joint.lower
    : (Number.isFinite(joint.min) ? joint.min : -Math.PI);
  const upper = Number.isFinite(joint.upper) ? joint.upper
    : (Number.isFinite(joint.max) ? joint.max : Math.PI);
  return { lower, upper };
}

/**
 * 从节点当前四元数反解关节角（弧度）：q = qZero ⊗ Rz(θ) → θ
 * 节点或 qZero 缺失时退回 joint.angle，保证面板不会因为对象不规范就空白。
 */
function readJointAngle(joint) {
  const node = nodeOf(joint);
  const q0 = joint.qZero;
  if (!node || !node.quaternion || !q0) {
    return Number(joint.angle) || 0;
  }
  const rel = qMul(qConj(q0), node.quaternion);
  // atan2(z, w) 与长度无关，不必再归一化；值域 (-π, π]
  return 2 * Math.atan2(rel.z, rel.w);
}

/* ══════════════════════════════════════════════════════════════
 * 样式（只注入一次，类名统一 pd- 前缀，避免污染主页样式）
 * ══════════════════════════════════════════════════════════════ */

const STYLE_ID = 'pd-pendant-style';

function injectStyle() {
  if (document.getElementById(STYLE_ID)) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = `
.pd-root {
  --pd-bg: #141a22;
  --pd-panel: #1b222c;
  --pd-panel-2: #222b37;
  --pd-line: #2b3644;
  --pd-text: #e2e9f3;
  --pd-muted: #8ea0b5;
  --pd-accent: #3fa9f5;
  --pd-ok: #3ddc97;
  --pd-warn: #ffb020;
  --pd-err: #ff5a5f;
  width: 360px;
  box-sizing: border-box;
  padding: 12px;
  border: 1px solid var(--pd-line);
  border-radius: 12px;
  background: var(--pd-bg);
  color: var(--pd-text);
  font: 13px/1.45 system-ui, "Segoe UI", "Microsoft YaHei", sans-serif;
  user-select: none;
}
.pd-root *, .pd-root *::before, .pd-root *::after { box-sizing: border-box; }

.pd-head { display: flex; align-items: center; gap: 8px; margin-bottom: 10px; }
.pd-title { font-size: 14px; font-weight: 650; letter-spacing: .5px; }
.pd-status {
  margin-left: auto; font-size: 12px; padding: 2px 8px; border-radius: 999px;
  border: 1px solid var(--pd-line); color: var(--pd-muted); background: var(--pd-panel);
}
.pd-status[data-mode="enabled"] { color: var(--pd-ok); border-color: rgba(61, 220, 151, .45); }
.pd-status[data-mode="estop"] { color: var(--pd-err); border-color: rgba(255, 90, 95, .55); }

.pd-bar { display: flex; gap: 8px; margin-bottom: 10px; }
.pd-btn {
  flex: 1; padding: 7px 10px; border-radius: 8px; cursor: pointer;
  border: 1px solid var(--pd-line); background: var(--pd-panel-2); color: var(--pd-text);
  font: inherit; font-weight: 600; transition: background .12s, border-color .12s, opacity .12s;
}
.pd-btn:hover:not(:disabled) { border-color: var(--pd-accent); }
.pd-btn:disabled { opacity: .45; cursor: not-allowed; }
.pd-btn[data-role="enable"][data-on="true"] { color: var(--pd-ok); border-color: rgba(61, 220, 151, .5); }
.pd-btn[data-role="estop"] { color: #ffd9da; background: #3a1d20; border-color: #6c2a2e; }
.pd-btn[data-role="estop"]:hover { background: #4a2226; border-color: var(--pd-err); }

.pd-speed { display: flex; align-items: center; gap: 8px; margin-bottom: 10px; font-size: 12px; color: var(--pd-muted); }
.pd-speed input[type="range"] { flex: 1; accent-color: var(--pd-accent); }
.pd-speed output { min-width: 46px; text-align: right; color: var(--pd-text); font-variant-numeric: tabular-nums; }

.pd-joint { padding: 7px 0; border-top: 1px solid var(--pd-line); }
.pd-joint__top { display: flex; align-items: baseline; gap: 6px; margin-bottom: 4px; }
.pd-joint__name { font-weight: 650; color: var(--pd-accent); }
.pd-joint__cur { margin-left: auto; font-variant-numeric: tabular-nums; }
.pd-joint__row { display: flex; align-items: center; gap: 6px; }
.pd-joint__slider { flex: 1; accent-color: var(--pd-accent); min-width: 0; }
.pd-jog {
  width: 26px; height: 26px; padding: 0; flex: none;
  border-radius: 6px; border: 1px solid var(--pd-line);
  background: var(--pd-panel-2); color: var(--pd-text);
  font: 700 15px/1 system-ui, sans-serif; cursor: pointer;
  transition: border-color .12s, background .12s;
}
.pd-jog:hover:not(:disabled) { border-color: var(--pd-accent); background: #28333f; }
.pd-jog:active:not(:disabled) { background: #2f3d4b; }
.pd-jog:disabled { opacity: .4; cursor: not-allowed; }
.pd-joint__num {
  width: 76px; padding: 4px 6px; text-align: right;
  border-radius: 6px; border: 1px solid var(--pd-line);
  background: var(--pd-panel); color: var(--pd-text);
  font: inherit; font-variant-numeric: tabular-nums;
}
.pd-joint__num:focus { outline: 2px solid rgba(63, 169, 245, .45); outline-offset: 0; border-color: var(--pd-accent); }
.pd-joint__limit { margin-top: 2px; font-size: 11px; color: var(--pd-muted); }

.pd-tcp { margin-top: 10px; padding-top: 9px; border-top: 1px solid var(--pd-line); }
.pd-tcp__title { font-size: 12px; color: var(--pd-muted); margin-bottom: 6px; }
.pd-tcp__grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 4px 10px; font-variant-numeric: tabular-nums; }
.pd-tcp__cell { display: flex; justify-content: space-between; gap: 4px; }
.pd-tcp__k { color: var(--pd-muted); }
.pd-tcp__v { color: var(--pd-text); }
.pd-tcp__sub { grid-column: 1 / -1; margin-top: 4px; font-size: 11px; color: var(--pd-muted); }
.pd-tcp__quat { grid-column: 1 / -1; display: flex; justify-content: space-between; gap: 8px; color: var(--pd-muted); }
.pd-foot { margin-top: 8px; font-size: 11px; color: var(--pd-muted); line-height: 1.4; }
`;
  document.head.appendChild(style);
}

/* ══════════════════════════════════════════════════════════════
 * 主函数
 * ══════════════════════════════════════════════════════════════ */

/**
 * @param {object} options
 * @param {HTMLElement|string} options.container 挂载容器（元素或选择器）
 * @param {object} options.controller  RobotController（需要 joints；有 setJoint / getTCPPose 更好）
 * @param {number|number[]} [options.maxJointSpeed=π/2] 100% 时各关节最大速度（rad/s）
 * @param {number} [options.speedPercent=100] 初始速度倍率 10~100
 * @param {boolean} [options.initiallyEnabled=true] 初始是否使能
 * @param {boolean} [options.autoTick=true] 是否自带 requestAnimationFrame 驱动；false 时需外部调 update(dt)
 * @param {string} [options.title='示教器']
 * @param {(state:object)=>void} [options.onStateChange] 状态变化回调
 * @returns {object} 面板控制对象
 */
export function createPendant(options = {}) {
  const {
    container,
    controller,
    maxJointSpeed = DEFAULT_MAX_JOINT_SPEED,
    speedPercent = SPEED_MAX,
    initiallyEnabled = true,
    autoTick = true,
    title = '示教器',
    onStateChange = null
  } = options;

  if (!controller || !Array.isArray(controller.joints) || controller.joints.length === 0) {
    throw new Error('[pendant] 需要传入已加载的 controller（含 joints 数组），先 await loadRobot(...)');
  }
  const host = typeof container === 'string' ? document.querySelector(container) : container;
  if (!host) throw new Error('[pendant] 找不到挂载容器：' + String(container));

  injectStyle();

  const joints = controller.joints;
  const n = joints.length;

  const speedAt = (i) => (Array.isArray(maxJointSpeed) ? (maxJointSpeed[i] ?? DEFAULT_MAX_JOINT_SPEED) : maxJointSpeed);

  const state = {
    /** 'enabled' | 'disabled' | 'estop' */
    mode: initiallyEnabled ? 'enabled' : 'disabled',
    speedPercent: clamp(Number(speedPercent) || SPEED_MAX, SPEED_MIN, SPEED_MAX)
  };

  /** 目标角（弧度），和“当前角”分开存：当前角永远从场景读 */
  const targets = new Array(n).fill(0);

  /** 点动方向：-1 / 0 / +1（按住时持续生效） */
  const jogDir = new Array(n).fill(0);

  /** 短按一次走多少度 */
  const JOG_STEP_DEG = 0.5;

  /** 按住超过这么久才算“连续点动”，否则只走一步 */
  const JOG_HOLD_DELAY = 120;

  /* ────────────── DOM ────────────── */

  const el = (tag, cls, text) => {
    const node = document.createElement(tag);
    if (cls) node.className = cls;
    if (text !== undefined) node.textContent = text;
    return node;
  };

  const root = el('div', 'pd-root');

  const head = el('div', 'pd-head');
  head.appendChild(el('span', 'pd-title', title));
  const statusEl = el('span', 'pd-status');
  head.appendChild(statusEl);
  root.appendChild(head);

  const bar = el('div', 'pd-bar');
  const enableBtn = el('button', 'pd-btn', '失能');
  enableBtn.type = 'button';
  enableBtn.dataset.role = 'enable';
  const estopBtn = el('button', 'pd-btn', '急停');
  estopBtn.type = 'button';
  estopBtn.dataset.role = 'estop';
  bar.append(enableBtn, estopBtn);
  root.appendChild(bar);

  const speedRow = el('label', 'pd-speed');
  speedRow.appendChild(el('span', '', '速度倍率'));
  const speedInput = document.createElement('input');
  speedInput.type = 'range';
  speedInput.min = String(SPEED_MIN);
  speedInput.max = String(SPEED_MAX);
  speedInput.step = '1';
  speedInput.value = String(state.speedPercent);
  const speedOut = el('output', '', state.speedPercent + '%');
  speedRow.append(speedInput, speedOut);
  root.appendChild(speedRow);

  const rows = joints.map((joint, i) => {
    const { lower, upper } = limitsOf(joint);
    const lowerDeg = lower * RAD;
    const upperDeg = upper * RAD;

    const row = el('div', 'pd-joint');

    const top = el('div', 'pd-joint__top');
    top.appendChild(el('span', 'pd-joint__name', joint.name || `J${i + 1}`));
    const curEl = el('span', 'pd-joint__cur', '0.000°');
    top.appendChild(curEl);
    row.appendChild(top);

    const line = el('div', 'pd-joint__row');
    const btnMinus = el('button', 'pd-jog', '−');
    btnMinus.type = 'button';
    btnMinus.dataset.dir = '-1';
    btnMinus.setAttribute('aria-label', `${joint.name || 'J' + (i + 1)} 负向点动`);

    const btnPlus = el('button', 'pd-jog', '+');
    btnPlus.type = 'button';
    btnPlus.dataset.dir = '1';
    btnPlus.setAttribute('aria-label', `${joint.name || 'J' + (i + 1)} 正向点动`);

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.className = 'pd-joint__slider';
    slider.min = String(lowerDeg);
    slider.max = String(upperDeg);
    slider.step = '0.5';
    slider.value = '0';
    slider.setAttribute('aria-label', `${joint.name || 'J' + (i + 1)} 关节滑块`);

    const num = document.createElement('input');
    num.type = 'number';
    num.className = 'pd-joint__num';
    num.min = String(Math.ceil(lowerDeg));
    num.max = String(Math.floor(upperDeg));
    num.step = '0.1';
    num.value = '0';
    num.setAttribute('aria-label', `${joint.name || 'J' + (i + 1)} 目标角度（度）`);

    line.append(btnMinus, slider, btnPlus, num);
    row.appendChild(line);
    row.appendChild(el('div', 'pd-joint__limit',
      `限位 ${lowerDeg.toFixed(0)}° ~ ${upperDeg.toFixed(0)}°`));
    root.appendChild(row);

    return { row, slider, num, curEl, lowerDeg, upperDeg, btnMinus, btnPlus };
  });

  const tcp = el('div', 'pd-tcp');
  tcp.appendChild(el('div', 'pd-tcp__title', '末端 TCP（世界坐标，取自主场景）'));
  const tcpGrid = el('div', 'pd-tcp__grid');
  const cell = (key) => {
    const wrap = el('div', 'pd-tcp__cell');
    wrap.appendChild(el('span', 'pd-tcp__k', key));
    const v = el('span', 'pd-tcp__v', '—');
    wrap.appendChild(v);
    tcpGrid.appendChild(wrap);
    return v;
  };
  const tcpEls = {
    x: cell('X'), y: cell('Y'), z: cell('Z'),
    r: cell('R'), p: cell('P'), y2: cell('Y')
  };
  // 在 RPY 前插一行小标题，避免第二个 “Y” 和位置的 Y 混淆
  const rpyHead = el('div', 'pd-tcp__sub', '姿态 RPY（度）');
  tcpGrid.insertBefore(rpyHead, tcpEls.r.parentElement);
  const posHead = el('div', 'pd-tcp__sub', '位置 XYZ');
  tcpGrid.insertBefore(posHead, tcpEls.x.parentElement);
  const quatRow = el('div', 'pd-tcp__quat');
  const quatEls = [];
  for (const k of ['w', 'x', 'y', 'z']) {
    const wrap = document.createElement('span');
    wrap.appendChild(el('span', 'pd-tcp__k', k + ' '));
    const v = el('span', 'pd-tcp__v', '—');
    wrap.appendChild(v);
    quatRow.appendChild(wrap);
    quatEls.push(v);
  }
  tcpGrid.appendChild(quatRow);
  tcp.appendChild(tcpGrid);
  root.appendChild(tcp);

  root.appendChild(el('div', 'pd-foot',
    '读数取自场景真实姿态：关节 q = qZero ⊗ Rz(θ) 反解，TCP 取世界矩阵。'));

  host.appendChild(root);

  /* ────────────── 应用 / 读取 ────────────── */

  function applyJoint(wJoint, angleRad) {
    const { lower, upper } = limitsOf(wJoint);
    const a = clamp(angleRad, lower, upper);

    if (typeof controller.setJoint === 'function') {
      controller.setJoint(wJoint, a);          // contract.js 的正式通道
      return a;
    }

    // 兜底：只拿到 robotLoader 的 joints 时，自己写 qZero ⊗ Rz(θ)
    const node = nodeOf(wJoint);
    const q0 = wJoint.qZero;
    if (!node || !node.quaternion || !q0) return a;
    const half = a / 2;
    const q = qMul(q0, { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) });
    if (typeof node.quaternion.set === 'function') node.quaternion.set(q.x, q.y, q.z, q.w);
    else Object.assign(node.quaternion, q);
    if (typeof node.updateMatrixWorld === 'function') node.updateMatrixWorld(true);
    wJoint.angle = a;
    return a;
  }

  function readPose() {
    if (typeof controller.getTCPPose === 'function') {
      try { return controller.getTCPPose(); } catch { return null; }
    }
    return null;
  }

  /* ────────────── 渲染 ────────────── */

  const fmtDeg = (rad) => (rad * RAD).toFixed(3) + '°';
  const fmt = (v, d = 3) => (Number.isFinite(v) ? Number(v).toFixed(d) : '—');

  let lastPaint = 0;

  /** 把真实姿态刷到面板上（默认 30Hz，避免每帧写 DOM） */
  function paint(force = false) {
    const now = performance.now();
    if (!force && now - lastPaint < 33) return;
    lastPaint = now;

    for (let i = 0; i < n; i++) {
      rows[i].curEl.textContent = fmtDeg(readJointAngle(joints[i]));
    }

    const pose = readPose();
    if (pose) {
      tcpEls.x.textContent = fmt(pose.p.x);
      tcpEls.y.textContent = fmt(pose.p.y);
      tcpEls.z.textContent = fmt(pose.p.z);
      tcpEls.r.textContent = fmt(pose.rpy.r * RAD, 2) + '°';
      tcpEls.p.textContent = fmt(pose.rpy.p * RAD, 2) + '°';
      tcpEls.y2.textContent = fmt(pose.rpy.y * RAD, 2) + '°';
      quatEls[0].textContent = fmt(pose.quat.w, 4);
      quatEls[1].textContent = fmt(pose.quat.x, 4);
      quatEls[2].textContent = fmt(pose.quat.y, 4);
      quatEls[3].textContent = fmt(pose.quat.z, 4);
    }
  }

  function renderControls() {
    const enabled = state.mode === 'enabled';
    const estop = state.mode === 'estop';

    statusEl.dataset.mode = state.mode;
    statusEl.textContent = enabled ? '已使能' : (estop ? '急停' : '未使能');

    enableBtn.textContent = enabled ? '失能' : '使能';
    enableBtn.dataset.on = String(enabled);
    enableBtn.disabled = estop;

    estopBtn.textContent = estop ? '解除急停' : '急停';

    for (const r of rows) {
      r.slider.disabled = !enabled;
      r.num.disabled = !enabled;
      r.btnMinus.disabled = !enabled;
      r.btnPlus.disabled = !enabled;
    }
    speedInput.disabled = estop;
    speedOut.textContent = state.speedPercent + '%';
    speedInput.value = String(state.speedPercent);
  }

  /** 目标值写回滑块 / 数字框（不覆盖用户正在输入的内容） */
  function renderTargets() {
    for (let i = 0; i < n; i++) {
      const deg = clamp(targets[i] * RAD, rows[i].lowerDeg, rows[i].upperDeg);
      rows[i].slider.value = String(deg);
      rows[i].num.value = deg.toFixed(1);
    }
  }

  function emit() {
    if (typeof onStateChange === 'function') onStateChange(getState());
  }

  /* ────────────── 运动 ────────────── */

  /** 当前姿态作为目标（使能/失能/急停切换时调用，避免“跳一下”） */
  function freezeTargets() {
    for (let i = 0; i < n; i++) targets[i] = readJointAngle(joints[i]);
    renderTargets();
  }

  /**
   * 一帧运动：各关节以 maxSpeed × 速度倍率 朝目标走，走不到就停，不越过。
   * 用“限速逼近”而不是“插值重开”：拖动滑块过程中改目标不会把速度打断成 0。
   */
  function tick(dt) {
    if (state.mode === 'enabled') {
      const ratio = state.speedPercent / 100;
      for (let i = 0; i < n; i++) {
        const joint = joints[i];
        const cur = readJointAngle(joint);

        // 点动优先：按住超过 JOG_HOLD_DELAY 才连续走，短按由 finish() 走一步
        if (jogDir[i] !== 0) {
          const held = performance.now() - (rows[i]._jogT0 || 0);
          if (held < JOG_HOLD_DELAY) continue;

          const { lower, upper } = limitsOf(joint);
          const next = clamp(cur + Math.sign(jogDir[i]) * speedAt(i) * ratio * dt, lower, upper);
          applyJoint(joint, next);
          targets[i] = next;
          const deg = next * RAD;
          rows[i].slider.value = String(deg);
          rows[i].num.value = deg.toFixed(1);
          continue;
        }

        const delta = targets[i] - cur;
        const step = speedAt(i) * ratio * dt;
        if (Math.abs(delta) <= step) {
          if (delta !== 0) applyJoint(joint, targets[i]);
        } else {
          applyJoint(joint, cur + Math.sign(delta) * step);
        }
      }
    }
    paint();
  }

  let rafId = 0;
  let lastT = 0;

  function loop(t) {
    const dt = lastT ? Math.min(0.1, (t - lastT) / 1000) : 0;
    lastT = t;
    tick(dt);
    rafId = requestAnimationFrame(loop);
  }

  /* ────────────── 交互 ────────────── */

  function setTargetDeg(i, deg, { syncUI = true } = {}) {
    const r = rows[i];
    const raw = Number(deg);
    // 输入框正在敲 "-" 或被清空时 Number 是 NaN：保持原目标，但界面要回填合法值
    const base = Number.isFinite(raw) ? raw : targets[i] * RAD;
    const v = clamp(base, r.lowerDeg, r.upperDeg);
    targets[i] = v * DEG;
    if (syncUI) {
      r.slider.value = String(v);
      r.num.value = v.toFixed(1);
    }
  }

  rows.forEach((r, i) => {
    r.slider.addEventListener('input', () => {
      setTargetDeg(i, Number(r.slider.value), { syncUI: false });
      r.num.value = Number(r.slider.value).toFixed(1);
    });
    r.num.addEventListener('input', () => {
      const v = parseFloat(r.num.value);
      if (!Number.isFinite(v)) return;      // 允许中途输入 "-" 或空
      setTargetDeg(i, v, { syncUI: false });
      r.slider.value = String(clamp(v, r.lowerDeg, r.upperDeg));
    });
    r.num.addEventListener('change', () => {
      // 失焦时把越界/非法输入改回合法值
      setTargetDeg(i, parseFloat(r.num.value), { syncUI: true });
    });
  });

  /* ── 点动：按住连续走，短按走一步 ── */
  rows.forEach((r, i) => {
    const start = (dir) => (ev) => {
      if (state.mode !== 'enabled') return;
      ev.preventDefault();
      r._jogT0 = performance.now();
      jogDir[i] = dir;
      try { ev.currentTarget.setPointerCapture?.(ev.pointerId); } catch { /* 忽略 */ }
    };
    const finish = (dir) => () => {
      if (jogDir[i] !== dir) return;
      const held = performance.now() - (r._jogT0 || 0);
      jogDir[i] = 0;
      targets[i] = readJointAngle(joints[i]);

      // 短按（没到连续点动的门槛）走固定一步，松手即停
      if (held < JOG_HOLD_DELAY) {
        applyJoint(joints[i], targets[i] + dir * JOG_STEP_DEG * DEG);
        targets[i] = readJointAngle(joints[i]);
        const deg = targets[i] * RAD;
        r.slider.value = String(deg);
        r.num.value = deg.toFixed(1);
        paint(true);
      }
    };
    r.btnMinus.addEventListener('pointerdown', start(-1));
    r.btnPlus.addEventListener('pointerdown', start(1));
    r.btnMinus.addEventListener('pointerup', finish(-1));
    r.btnPlus.addEventListener('pointerup', finish(1));
    r.btnMinus.addEventListener('pointercancel', finish(-1));
    r.btnPlus.addEventListener('pointercancel', finish(1));
  });

  // 指针在任何地方松开都要停下来，避免"松手了还在走"
  window.addEventListener('pointerup', () => {
    for (let i = 0; i < n; i++) {
      if (jogDir[i] !== 0) {
        jogDir[i] = 0;
        targets[i] = readJointAngle(joints[i]);
      }
    }
  });
  window.addEventListener('blur', () => {
    for (let i = 0; i < n; i++) jogDir[i] = 0;
  });

  speedInput.addEventListener('input', () => {
    state.speedPercent = clamp(Number(speedInput.value), SPEED_MIN, SPEED_MAX);
    speedOut.textContent = state.speedPercent + '%';
    emit();
  });

  function setEnabled(on) {
    if (state.mode === 'estop') return false;          // 急停中必须先复位
    const next = on ? 'enabled' : 'disabled';
    if (state.mode === next) return true;
    state.mode = next;
    freezeTargets();                                   // 使能/失能都不许跳姿态
    renderControls();
    paint(true);
    emit();
    return true;
  }

  enableBtn.addEventListener('click', () => setEnabled(state.mode !== 'enabled'));

  function emergencyStop() {
    if (state.mode === 'estop') return;
    state.mode = 'estop';
    freezeTargets();                                   // 立刻停在当前位置
    renderControls();
    paint(true);
    emit();
  }

  function resetEmergency() {
    if (state.mode !== 'estop') return;
    state.mode = 'disabled';                           // 复位后仍需重新使能
    freezeTargets();
    renderControls();
    paint(true);
    emit();
  }

  estopBtn.addEventListener('click', () => {
    if (state.mode === 'estop') resetEmergency();
    else emergencyStop();
  });

  /* ────────────── 对外接口 ────────────── */

  function getState() {
    return {
      mode: state.mode,
      enabled: state.mode === 'enabled',
      estop: state.mode === 'estop',
      speedPercent: state.speedPercent,
      targetsDeg: targets.map((t) => t * RAD),
      actualDeg: joints.map((j) => readJointAngle(j) * RAD),
      tcp: readPose()
    };
  }

  /** 进度推进。autoTick=true 时由内部 rAF 调用，外部一般不用管 */
  function update(dt) {
    if (!Number.isFinite(dt) || dt <= 0) dt = 0;
    tick(Math.min(dt, 0.1));
  }

  function setPoseDeg(degArray) {
    for (let i = 0; i < n && i < degArray.length; i++) {
      setTargetDeg(i, degArray[i], { syncUI: true });
    }
  }

  function setSpeedPercent(p) {
    state.speedPercent = clamp(Number(p) || SPEED_MIN, SPEED_MIN, SPEED_MAX);
    renderControls();
    emit();
  }

  function dispose() {
    if (rafId) cancelAnimationFrame(rafId);
    rafId = 0;
    root.remove();
  }

  /**
   * 外部改过机器人姿态后（比如标定页用逆解把工具尖送到标定点），
   * 必须调一次 sync()，否则面板还记着旧目标，下一帧会把机械臂拽回去。
   */
  function sync() {
    freezeTargets();
    paint(true);
  }

  /* ────────────── 初始化 ────────────── */

  freezeTargets();
  renderControls();
  paint(true);

  if (autoTick) {
    lastT = 0;
    rafId = requestAnimationFrame(loop);
  }

  return {
    root,
    setEnabled,
    enable: () => setEnabled(true),
    disable: () => setEnabled(false),
    emergencyStop,
    resetEmergency,
    setSpeedPercent,
    setPoseDeg,
    update,
    sync,
    getState,
    dispose
  };
}

export default createPendant;
