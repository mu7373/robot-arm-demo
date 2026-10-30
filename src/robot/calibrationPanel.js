/**
 * calibrationPanel.js —— 标定面板（工具坐标系四点法 / 用户坐标系三点法）
 *
 * 面板自己会：读法兰位姿 → 用逆解把工具尖送到标定点 → 记录 → 解算 → 显示验证数据 → 存盘。
 * 页面只需要挂载它，并在它动过机械臂之后调用 pendant.sync()。
 *
 * 用法：
 *   const panel = createCalibrationPanel({
 *     container: '#cal',
 *     controller: ctl,        // RobotController
 *     robot,                  // gltf.scene
 *     tcp,                    // 末端节点
 *     modelToolOffset: jointMap.tcp?.offset_local,   // 模型自带的真值，用于对比
 *     onRobotMoved: () => pendant.sync()
 *   });
 */

import * as THREE from 'three';
import { solvePositionIK } from './ik.js';
import {
  solveToolFromPoses,
  makeUserFrame,
  toolTipFromPose,
  pointToUser,
  verifyUserFrame,
  roundTripError,
  buildRecord,
  saveCalibration,
  loadCalibration,
  downloadCalibration,
  validateRecord,
  DEFAULT_STORAGE_KEY
} from './calibration.js';

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const STYLE_ID = 'cp-calibration-style';

/** 四点法第 2~4 点：沿这些关节方向做零空间漂移，尖点不动、姿态改变 */
const TOOL_DRIFT = [
  null,
  [0, 1, -1, 0, 0],
  [0, -1, 1, 0, 1],
  [1, 0, 0, 1, -1]
];

/** 三点法自动示教时，X 方向和平面内的方向（世界系） */
const USER_DIR_X = [1, 0, 0];
const USER_DIR_PLANE = [0, 0, -1];
const USER_SPAN = 2.2;          // u

function injectStyle() {
  if (document.getElementById(STYLE_ID)) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = `
.cp-root {
  --cp-bg: #141a22; --cp-panel: #1b222c; --cp-line: #2b3644;
  --cp-text: #e2e9f3; --cp-muted: #8ea0b5; --cp-accent: #3fa9f5;
  --cp-ok: #3ddc97; --cp-warn: #ffb020; --cp-err: #ff5a5f;
  width: 388px; box-sizing: border-box; padding: 12px;
  border: 1px solid var(--cp-line); border-radius: 12px; background: var(--cp-bg);
  color: var(--cp-text); font: 12.5px/1.5 system-ui, "Segoe UI", "Microsoft YaHei", sans-serif;
  user-select: none;
}
.cp-root * { box-sizing: border-box; }
.cp-head { display: flex; align-items: center; gap: 8px; margin-bottom: 4px; }
.cp-head strong { font-size: 14px; letter-spacing: .5px; }
.cp-badge { margin-left: auto; font-size: 11px; padding: 2px 8px; border-radius: 999px;
  border: 1px solid var(--cp-line); color: var(--cp-muted); }
.cp-badge[data-ok="true"] { color: var(--cp-ok); border-color: rgba(61,220,151,.45); }
.cp-sec { margin-top: 10px; padding-top: 9px; border-top: 1px solid var(--cp-line); }
.cp-title { display: flex; align-items: baseline; gap: 6px; font-size: 12px; color: var(--cp-muted); margin-bottom: 6px; }
.cp-title b { color: var(--cp-text); font-size: 12.5px; font-weight: 650; }
.cp-btns { display: flex; flex-wrap: wrap; gap: 6px; }
.cp-btn {
  padding: 5px 9px; border-radius: 7px; cursor: pointer; font: inherit;
  border: 1px solid var(--cp-line); background: #222b37; color: var(--cp-text);
  transition: border-color .12s, color .12s, opacity .12s;
}
.cp-btn:hover:not(:disabled) { border-color: var(--cp-accent); }
.cp-btn:disabled { opacity: .45; cursor: not-allowed; }
.cp-btn[data-done="true"] { color: var(--cp-ok); border-color: rgba(61,220,151,.45); }
.cp-btn[data-kind="primary"] { border-color: rgba(63,169,245,.55); color: #cfe7fb; }
.cp-list { margin-top: 6px; display: grid; gap: 2px; font-size: 11.5px; color: var(--cp-muted); }
.cp-kv { display: flex; justify-content: space-between; gap: 10px; }
.cp-kv > span:last-child { color: var(--cp-text); font-variant-numeric: tabular-nums; }
.cp-kv--ok > span:last-child { color: var(--cp-ok); }
.cp-kv--warn > span:last-child { color: var(--cp-warn); }
.cp-msg { min-height: 17px; margin-top: 8px; font-size: 11.5px; color: var(--cp-warn); }
.cp-msg[data-kind="err"] { color: var(--cp-err); }
.cp-msg[data-kind="ok"] { color: var(--cp-ok); }
.cp-live { margin-top: 6px; display: grid; gap: 2px; font-size: 11.5px; }
.cp-live__row { display: flex; gap: 8px; }
.cp-live__tag { flex: none; width: 58px; color: var(--cp-muted); }
.cp-live__val { color: var(--cp-text); font-variant-numeric: tabular-nums; }
.cp-foot { margin-top: 6px; font-size: 10.5px; color: var(--cp-muted); }
`;
  document.head.appendChild(style);
}

export function createCalibrationPanel(options = {}) {
  const {
    container,
    controller,
    robot,
    tcp,
    modelToolOffset = null,
    onRobotMoved = null,
    onCalibrated = null,
    storageKey = DEFAULT_STORAGE_KEY
  } = options;

  if (!controller || !robot || !tcp) {
    throw new Error('[calibrationPanel] 需要 controller / robot / tcp');
  }
  const host = typeof container === 'string' ? document.querySelector(container) : container;
  if (!host) throw new Error('[calibrationPanel] 找不到容器：' + String(container));

  injectStyle();

  const state = {
    pin: null,
    toolPoses: [],
    tool: null,
    userPoints: { origin: null, x: null, xy: null },
    user: null,              // { frame, verify, roundTrip, points }
    busy: false
  };

  /* ────────────── 小工具 ────────────── */

  const fmt = (v, d = 3) => (Number.isFinite(v) ? Number(v).toFixed(d) : '—');
  /** 误差用自适应精度：小到 1e-4 以下就换科学计数法，免得全显示成 0.000000 */
  const fmtErr = (v) => {
    if (!Number.isFinite(v)) return '—';
    if (v === 0) return '0';
    return Math.abs(v) < 1e-4 ? v.toExponential(2) : v.toFixed(6);
  };
  const vec = (v, d = 3) => `(${fmt(v[0], d)}, ${fmt(v[1], d)}, ${fmt(v[2], d)})`;
  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

  function tipPos() {
    robot.updateMatrixWorld(true);
    const p = new THREE.Vector3();
    tcp.getWorldPosition(p);
    return [p.x, p.y, p.z];
  }

  function flangePose() {
    const flange = robot.getObjectByName('link_flange', true) || tcp.parent;
    robot.updateMatrixWorld(true);
    const p = new THREE.Vector3();
    const q = new THREE.Quaternion();
    const s = new THREE.Vector3();
    flange.matrixWorld.decompose(p, q, s);
    const m = new THREE.Matrix4().makeRotationFromQuaternion(q).elements;
    return {
      p: [p.x, p.y, p.z],
      R: [[m[0], m[4], m[8]], [m[1], m[5], m[9]], [m[2], m[6], m[10]]]
    };
  }

  const nextFrame = () => new Promise((r) => requestAnimationFrame(() => r()));

  function message(text, kind = 'info') {
    msgEl.textContent = text || '';
    if (kind === 'info') msgEl.removeAttribute('data-kind');
    else msgEl.dataset.kind = kind;
  }

  /** 逆解把工具尖送到 point；额外参数透传给 solvePositionIK */
  function goTo(point, opts = {}) {
    const r = solvePositionIK({
      robot, controller, tcp,
      target: new THREE.Vector3(point[0], point[1], point[2]),
      ...opts
    });
    if (onRobotMoved) onRobotMoved();
    return r;
  }

  /* ────────────── DOM ────────────── */

  const el = (tag, cls, text) => {
    const n = document.createElement(tag);
    if (cls) n.className = cls;
    if (text !== undefined) n.textContent = text;
    return n;
  };

  const root = el('div', 'cp-root');

  const head = el('div', 'cp-head');
  head.appendChild(el('strong', '', '标定'));
  const badge = el('span', 'cp-badge', '未标定');
  head.appendChild(badge);
  root.appendChild(head);

  // ── 工具坐标系 ──
  const toolSec = el('div', 'cp-sec');
  const toolTitle = el('div', 'cp-title');
  toolTitle.appendChild(el('b', '', '工具坐标系'));
  toolTitle.appendChild(el('span', '', '四点法'));
  toolSec.appendChild(toolTitle);

  const toolBtns = el('div', 'cp-btns');
  const toolPointBtns = [];
  for (let k = 0; k < 4; k++) {
    const b = el('button', 'cp-btn', `示教点 ${k + 1}`);
    b.type = 'button';
    b.dataset.act = 'tool-point';
    b.dataset.k = String(k);
    toolBtns.appendChild(b);
    toolPointBtns.push(b);
  }
  toolSec.appendChild(toolBtns);

  const toolBtns2 = el('div', 'cp-btns');
  toolBtns2.style.marginTop = '6px';
  const solveToolBtn = el('button', 'cp-btn', '计算工具坐标系');
  solveToolBtn.type = 'button';
  solveToolBtn.dataset.act = 'solve-tool';
  solveToolBtn.dataset.kind = 'primary';
  const clearToolBtn = el('button', 'cp-btn', '清除');
  clearToolBtn.type = 'button';
  clearToolBtn.dataset.act = 'clear-tool';
  toolBtns2.append(solveToolBtn, clearToolBtn);
  toolSec.appendChild(toolBtns2);

  const toolList = el('div', 'cp-list');
  toolSec.appendChild(toolList);
  root.appendChild(toolSec);

  // ── 用户坐标系 ──
  const userSec = el('div', 'cp-sec');
  const userTitle = el('div', 'cp-title');
  userTitle.appendChild(el('b', '', '用户坐标系'));
  userTitle.appendChild(el('span', '', '三点法'));
  userSec.appendChild(userTitle);

  const userBtns = el('div', 'cp-btns');
  const autoUserBtn = el('button', 'cp-btn', '自动示教 3 点');
  autoUserBtn.type = 'button';
  autoUserBtn.dataset.act = 'auto-user';
  autoUserBtn.dataset.kind = 'primary';
  userBtns.appendChild(autoUserBtn);
  for (const [kind, label] of [['origin', '取原点'], ['x', '取 X 点'], ['xy', '取 XY 点']]) {
    const b = el('button', 'cp-btn', label);
    b.type = 'button';
    b.dataset.act = 'user-point';
    b.dataset.kind = kind;
    userBtns.appendChild(b);
  }
  userSec.appendChild(userBtns);

  const solveUserBtn = el('button', 'cp-btn', '计算用户坐标系');
  solveUserBtn.type = 'button';
  solveUserBtn.dataset.act = 'solve-user';
  solveUserBtn.dataset.kind = 'primary';
  solveUserBtn.style.marginTop = '6px';
  userSec.appendChild(solveUserBtn);

  const userList = el('div', 'cp-list');
  userSec.appendChild(userList);
  root.appendChild(userSec);

  // ── 验证数据 ──
  const verifySec = el('div', 'cp-sec');
  const verifyTitle = el('div', 'cp-title');
  verifyTitle.appendChild(el('b', '', '验证数据'));
  verifyTitle.appendChild(el('span', '', '误差可量化'));
  verifySec.appendChild(verifyTitle);
  const verifyList = el('div', 'cp-list');
  verifySec.appendChild(verifyList);
  root.appendChild(verifySec);

  // ── 实时读数 ──
  const liveSec = el('div', 'cp-sec');
  const liveTitle = el('div', 'cp-title');
  liveTitle.appendChild(el('b', '', '当前 TCP 读数'));
  liveSec.appendChild(liveTitle);
  const liveList = el('div', 'cp-live');
  const liveBase = el('div', 'cp-live__row');
  liveBase.append(el('span', 'cp-live__tag', '基坐标'), el('span', 'cp-live__val', '—'));
  const liveBaseRot = el('div', 'cp-live__row');
  liveBaseRot.append(el('span', 'cp-live__tag', ''), el('span', 'cp-live__val', '—'));
  const liveUser = el('div', 'cp-live__row');
  liveUser.append(el('span', 'cp-live__tag', '用户坐标'), el('span', 'cp-live__val', '未标定'));
  liveList.append(liveBase, liveBaseRot, liveUser);
  liveSec.appendChild(liveList);
  root.appendChild(liveSec);

  // ── 存取 ──
  const ioBtns = el('div', 'cp-btns');
  ioBtns.style.marginTop = '10px';
  const saveBtn = el('button', 'cp-btn', '保存');
  saveBtn.type = 'button';
  saveBtn.dataset.act = 'save';
  const loadBtn = el('button', 'cp-btn', '载入');
  loadBtn.type = 'button';
  loadBtn.dataset.act = 'load';
  const downloadBtn = el('button', 'cp-btn', '下载 JSON');
  downloadBtn.type = 'button';
  downloadBtn.dataset.act = 'download';
  ioBtns.append(saveBtn, loadBtn, downloadBtn);
  root.appendChild(ioBtns);

  const msgEl = el('div', 'cp-msg');
  root.appendChild(msgEl);

  root.appendChild(el('div', 'cp-foot', '误差单位 u = 模型单位（与 GLB 一致，1 u 约 10 cm）。'));

  host.appendChild(root);

  /* ────────────── 渲染 ────────────── */

  function renderToolList() {
    toolList.textContent = '';
    toolPointBtns.forEach((b, k) => {
      b.dataset.done = String(Boolean(state.toolPoses[k]));
    });
    if (!state.pin) {
      toolList.appendChild(el('div', 'cp-kv', '还没有标定点：先点「示教点 1」。'));
      return;
    }
    const row = el('div', 'cp-kv');
    row.append(el('span', '', '标定点'), el('span', '', vec(state.pin)));
    toolList.appendChild(row);
    state.toolPoses.forEach((pose, k) => {
      const r = el('div', 'cp-kv');
      r.append(el('span', '', `点 ${k + 1} 法兰`), el('span', '', vec(pose.p)));
      toolList.appendChild(r);
    });
  }

  function renderUserList() {
    userList.textContent = '';
    const items = [
      ['origin', '原点'],
      ['x', 'X 点'],
      ['xy', 'XY 点']
    ];
    for (const [kind, label] of items) {
      const p = state.userPoints[kind];
      const r = el('div', 'cp-kv');
      r.append(el('span', '', label), el('span', '', p ? vec(p) : '—'));
      userList.appendChild(r);
    }
  }

  function renderVerify() {
    verifyList.textContent = '';
    const add = (k, v, cls) => {
      const row = el('div', 'cp-kv' + (cls ? ' cp-kv--' + cls : ''));
      row.append(el('span', '', k), el('span', '', v));
      verifyList.appendChild(row);
    };

    if (state.tool) {
      add('工具偏移 t', vec(state.tool.offset));
      if (modelToolOffset) {
        add('模型 tcp 偏移', vec(modelToolOffset));
        add('两者之差', fmtErr(dist(state.tool.offset, modelToolOffset)) + ' u',
          dist(state.tool.offset, modelToolOffset) < 1e-3 ? 'ok' : 'warn');
      }
      add('四点残差 RMS', fmtErr(state.tool.rms) + ' u');
      add('四点残差 最大', fmtErr(state.tool.maxError) + ' u');
      add('同一点最大互差', fmtErr(toolSpread()) + ' u');
    } else {
      add('工具坐标系', '未计算');
    }

    if (state.user) {
      const v = state.user.verify;
      add('用户系原点', vec(state.user.frame.origin));
      add('X 轴', vec(state.user.frame.x));
      add('Z 轴（法向）', vec(state.user.frame.z));
      add('轴正交性', fmtErr(v.maxOrthogonality));
      add('定义点回代残差', fmtErr(v.maxError) + ' u');
      add('往返变换误差', fmtErr(state.user.roundTrip) + ' u');
    } else {
      add('用户坐标系', '未计算');
    }
  }

  /** 标定后同一点的最大互差：把每个姿态的尖点按 t 算出来，两两比距离 */
  function toolSpread() {
    if (!state.tool) return 0;
    const pts = state.toolPoses
      .filter((pose) => pose && pose.p && pose.R)
      .map((pose) => toolTipFromPose(pose, state.tool.offset));
    let max = 0;
    for (let i = 0; i < pts.length; i++) {
      for (let j = i + 1; j < pts.length; j++) {
        max = Math.max(max, dist(pts[i], pts[j]));
      }
    }
    return max;
  }

  // 让验证表能调用

  function renderBadge() {
    const ok = Boolean(state.tool) || Boolean(state.user);
    badge.dataset.ok = String(ok);
    badge.textContent = ok ? '已标定' : '未标定';
  }

  function renderAll() {
    renderToolList();
    renderUserList();
    renderVerify();
    renderBadge();
  }

  /* ────────────── 实时读数 ────────────── */

  let rafId = 0;
  let lastLive = 0;

  function readPose() {
    if (typeof controller.getTCPPose === 'function') {
      try { return controller.getTCPPose(); } catch { return null; }
    }
    return null;
  }

  function paintLive() {
    rafId = requestAnimationFrame(paintLive);   // 先续帧，再判断节流，否则节流一次循环就断了

    const now = performance.now();
    if (now - lastLive < 100) return;
    lastLive = now;

    const pose = readPose();
    if (pose) {
      liveBase.lastChild.textContent =
        `X ${fmt(pose.p.x)}  Y ${fmt(pose.p.y)}  Z ${fmt(pose.p.z)}`;
      liveBaseRot.lastChild.textContent =
        `R ${fmt(pose.rpy.r * RAD, 2)}°  P ${fmt(pose.rpy.p * RAD, 2)}°  Y ${fmt(pose.rpy.y * RAD, 2)}°`;
    }

    if (state.user) {
      const p = tipPos();
      const pu = pointToUser(state.user.frame, p);
      liveUser.lastChild.textContent =
        `X ${fmt(pu[0])}  Y ${fmt(pu[1])}  Z ${fmt(pu[2])}`;
    } else {
      liveUser.lastChild.textContent = '未标定';
    }
  }

  /* ────────────── 动作 ────────────── */

  async function recordToolPoint(k) {
    if (state.busy) return;
    state.busy = true;
    try {
      if (k === 0) {
        state.pin = tipPos();
        state.toolPoses = [];
        message('标定点已定在当前位置，接下来 3 个点会自动对准它。');
      } else {
        if (!state.pin) { message('先示教点 1，确定标定点。', 'err'); return; }
        if (!state.toolPoses[0]) { message('示教点 1 还没记录。', 'err'); return; }

        // 先回标定点，再沿零空间漂移姿态：尖点不动，法兰姿态变化
        const back = goTo(state.pin);
        if (!back.ok) {
          message(`点 ${k + 1} 回不到标定点（残差 ${back.error.toExponential(2)}）。`, 'err');
          return;
        }
        await nextFrame();

        const drift = goTo(state.pin, {
          nullDir: TOOL_DRIFT[k] || TOOL_DRIFT[1],
          nullGain: 0.03,
          maxIter: 45,
          tol: 0
        });
        if (drift.error > 1e-2) {
          message(`点 ${k + 1} 换姿态时尖点漂了 ${drift.error.toExponential(2)}，重试一次。`, 'err');
          return;
        }

        // 漂移过程中尖点会被带偏一点点，最后再对准一次（保持新姿态）
        const snap = goTo(state.pin);
        if (!snap.ok) {
          message(`点 ${k + 1} 对准失败（残差 ${snap.error.toExponential(2)}）。`, 'err');
          return;
        }
        await nextFrame();
      }

      const pose = flangePose();
      state.toolPoses[k] = { p: pose.p, R: pose.R, tip: tipPos() };
      state.tool = null;
      message(`已记录示教点 ${k + 1}。`);
      renderAll();
    } finally {
      state.busy = false;
    }
  }

  function solveTool() {
    if (state.toolPoses.filter(Boolean).length < 4) {
      message('工具坐标系需要 4 个示教点。', 'err');
      return null;
    }
    try {
      const res = solveToolFromPoses(state.toolPoses.map(({ p, R }) => ({ p, R })));
      state.tool = res;
      renderAll();
      message(`工具坐标系解算完成：残差 RMS ${res.rms.toExponential(2)} u。`, 'ok');
      if (onCalibrated) onCalibrated(buildRecordSnapshot());
      return res;
    } catch (e) {
      message(e.message, 'err');
      return null;
    }
  }

  async function recordUserPoint(kind) {
    if (state.busy) return;
    state.busy = true;
    try {
      state.userPoints[kind] = tipPos();
      state.user = null;
      renderAll();
      message(`已记录${kind === 'origin' ? '原点' : kind === 'x' ? ' X 点' : ' XY 平面点'}。`);
    } finally {
      state.busy = false;
    }
  }

  async function autoUser() {
    if (state.busy) return;
    state.busy = true;
    try {
      const p0 = tipPos();
      const p1 = [0, 1, 2].map((i) => p0[i] + USER_DIR_X[i] * USER_SPAN);
      const p2 = [0, 1, 2].map((i) => p1[i] + USER_DIR_PLANE[i] * USER_SPAN);

      message('正在把工具尖依次送到 3 个示教点…');
      for (const [idx, pt, label] of [[0, p0, '原点'], [1, p1, 'X 点'], [2, p2, 'XY 点']]) {
        if (idx > 0) {
          const r = goTo(pt);
          if (!r.ok) {
            message(`${label}不可达（残差 ${r.error.toExponential(2)}），换个小一点的范围。`, 'err');
            return;
          }
          await nextFrame();
          state.userPoints[['origin', 'x', 'xy'][idx]] = tipPos();
        } else {
          state.userPoints.origin = p0;
        }
      }
      state.user = null;
      renderAll();
      message('3 个示教点已记录，点「计算用户坐标系」。');
    } finally {
      state.busy = false;
    }
  }

  function solveUser() {
    const { origin, x, xy } = state.userPoints;
    if (!origin || !x || !xy) {
      message('用户坐标系需要原点、X 点、XY 点三个示教点。', 'err');
      return null;
    }
    try {
      const frame = makeUserFrame(origin, x, xy);
      const verify = verifyUserFrame(frame, [origin, x, xy]);
      const roundTrip = roundTripError(frame, [origin, x, xy, tipPos()]);
      state.user = { frame, verify, roundTrip, points: { origin, x, xy } };
      renderAll();
      message(`用户坐标系解算完成：正交性 ${verify.maxOrthogonality.toExponential(2)}。`, 'ok');
      if (onCalibrated) onCalibrated(buildRecordSnapshot());
      return frame;
    } catch (e) {
      message(e.message, 'err');
      return null;
    }
  }

  /* ────────────── 存档 ────────────── */

  function buildRecordSnapshot() {
    const verification = {};
    if (state.tool) {
      verification.maxToolResidual = state.tool.maxError;
      verification.toolRms = state.tool.rms;
      verification.toolSpread = toolSpread();
      if (modelToolOffset) verification.modelOffsetDelta = dist(state.tool.offset, modelToolOffset);
    }
    if (state.user) {
      verification.userOrthogonality = state.user.verify.maxOrthogonality;
      verification.userRoundTrip = state.user.roundTrip;
    }

    return buildRecord({
      tool: state.tool ? {
        offset: state.tool.offset,
        fixedPoint: state.tool.fixedPoint,
        residuals: state.tool.residuals,
        rms: state.tool.rms,
        maxError: state.tool.maxError,
        count: state.tool.count,
        poses: state.toolPoses.map(({ p, R }) => ({ p, R }))
      } : null,
      user: state.user ? {
        origin: state.user.frame.origin,
        x: state.user.frame.x,
        y: state.user.frame.y,
        z: state.user.frame.z,
        points: state.user.points,
        verify: state.user.verify,
        roundTrip: state.user.roundTrip
      } : null,
      verification,
      meta: { model: 'hanz_robot_5dof', unit: 'u' }
    });
  }

  function save() {
    if (!state.tool && !state.user) { message('还没有可保存的标定结果。', 'err'); return null; }
    const record = buildRecordSnapshot();
    saveCalibration(record, { key: storageKey });
    message(`已保存到浏览器本地存储（${new Date().toLocaleTimeString()}）。`, 'ok');
    return record;
  }

  function applyRecord(record) {
    const check = validateRecord(record);
    if (!check.ok) { message('标定文件无效：' + check.errors.join('；'), 'err'); return false; }

    state.tool = record.tool ? {
      offset: record.tool.offset,
      fixedPoint: record.tool.fixedPoint,
      residuals: record.tool.residuals || [],
      rms: record.tool.rms ?? 0,
      maxError: record.tool.maxError ?? 0,
      count: record.tool.count ?? 4
    } : null;
    state.toolPoses = (record.tool?.poses || []).map(({ p, R }) => ({ p, R, tip: null }));

    state.user = record.user ? {
      frame: {
        origin: record.user.origin,
        x: record.user.x,
        y: record.user.y,
        z: record.user.z
      },
      verify: record.user.verify || { maxOrthogonality: 0, maxError: 0 },
      roundTrip: record.user.roundTrip ?? 0,
      points: record.user.points || {}
    } : null;
    state.userPoints = {
      origin: state.user?.points.origin || null,
      x: state.user?.points.x || null,
      xy: state.user?.points.xy || null
    };

    renderAll();
    if (onCalibrated) onCalibrated(buildRecordSnapshot());
    return true;
  }

  function load() {
    const record = loadCalibration({ key: storageKey });
    if (!record) { message('本地没有保存过的标定数据。', 'err'); return null; }
    if (!applyRecord(record)) return null;
    message('已载入本地保存的标定数据。', 'ok');
    return record;
  }

  /* ────────────── 事件 ────────────── */

  root.addEventListener('click', async (ev) => {
    const btn = ev.target.closest('button[data-act]');
    if (!btn) return;
    const act = btn.dataset.act;

    if (act === 'tool-point') await recordToolPoint(Number(btn.dataset.k));
    else if (act === 'solve-tool') solveTool();
    else if (act === 'clear-tool') {
      state.toolPoses = [];
      state.tool = null;
      state.pin = null;
      renderAll();
      message('已清除工具坐标系示教点。');
    } else if (act === 'auto-user') await autoUser();
    else if (act === 'user-point') await recordUserPoint(btn.dataset.kind);
    else if (act === 'solve-user') solveUser();
    else if (act === 'save') save();
    else if (act === 'load') load();
    else if (act === 'download') {
      if (!state.tool && !state.user) { message('还没有可下载的标定结果。', 'err'); return; }
      downloadCalibration(buildRecordSnapshot(), 'calibration.json');
      message('已开始下载 calibration.json。', 'ok');
    }
  });

  renderAll();
  paintLive();

  return {
    root,
    solveTool,
    solveUser,
    getRecord: buildRecordSnapshot,
    save,
    load,
    applyRecord,
    getTool: () => state.tool,
    getUser: () => state.user,
    dispose() { if (rafId) cancelAnimationFrame(rafId); root.remove(); }
  };
}

export default createCalibrationPanel;
