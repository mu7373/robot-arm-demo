# 实验一 机械臂仿真（一）——模型导入、关节运动控制与示教器界面

计算机图形与动画 · CM11 · 支撑课程目标 CO4 · 8 学时

Three.js / WebGL 实现的 5 自由度协作臂仿真：加载模型、示教器界面、示教编程（MOVJ 空中画圆）、
轨迹分析、可达区域分析。四项要求 a~d 全部有代码 + 图表 + 数字可查。

---

## 1. 怎么跑起来

需要 Node.js 18+（推荐 22/24）与 Python 3（只跑离线出图时才需要，要 numpy + Pillow）。

```bash
npm install          # 装 three
npm start            # 起本地服务，默认 8123
```

然后浏览器打开：

| 页面 | 内容 |
|---|---|
| `http://127.0.0.1:8123/index.html` | 示教器界面（要求 a、b） |
| `http://127.0.0.1:8123/analysis_test.html` | B 侧分析工作台（FK 验证、轨迹、可达区域） |

**必须走 http://，不能双击 file:// 打开**——ES Module 和 .glb 会被同源策略拦掉。

一条命令重跑全部离线分析（验证 + 出图 + 动画，约 1 分钟）：

```bash
npm run all
```

---

## 2. 目录

```
assets/                  模型与数据（唯一事实源）
  robot_articulated.glb    5 自由度协作臂（11.2 MB，8227 三角面）
  robot-model.json         从 GLB 实测导出的节点层级/局部轴/限位/静止四元数
  joint_map.json           原始关节表（限位可信，convention 与 motion_axis 不可信，见下）
  program-circle.json      12 个示教位姿 + 理想圆几何
  robot_spec.yaml / robot.urdf   教材对照文件（臂长 3.33，与 GLB 的 6.54 不是同一台，勿用于计算）
src/
  contract.js              A/B 共用接口契约（RobotController：加载、驱动、位姿、MOVJ、录制）
  robot/robotLoader.js     独立加载器
  robot/jointControl.js    关节控制
  analysis/                ← B 侧
    fk.js                   正运动学（纯数学，不依赖 three/DOM）
    fkVerify.js             三档验证（自己写的 FK ↔ three 场景树对拍）
    interp.js               MOVJ 程序编排（分段、时长、速度倍率、段间跳变自检）
    recorder.js             轨迹采样录制 + 速度/加速度分析
    trajectory.js           轨迹几何：点到圆距离、路径长度、平面投影
    workspace.js            蒙特卡洛可达域：点云、极坐标边界、占位栅格、洪水填充找空洞
    axisDecision.md         关节转轴判定（重要，见第 5 节）
tools/                   离线脚本（Python）
  dump_model.py             GLB → robot-model.json
  verify_fk.py / .mjs       三档 + 几何自洽验证（两套独立实现）
  fastkin.py                向量化运动学 + 批量阻尼最小二乘 IK
  plan.py                   示教位姿规划（路径连续法求逆解）
  analysis_trajectory.py    要求 c 出图
  analysis_workspace.py     要求 d 出图
  animation.py              离线渲染 MOVJ 动画 GIF
  render.py / plotlib.py    自写的软件渲染器与绘图库（不依赖 matplotlib）
report/                  全部成果物
  fk_verify.txt / .json     验证结果
  fig_trajectory.png        要求 c：轨迹 / 位置-时间 / 速度-时间 / 偏差-时间
  fig_workspace.png         要求 d：立面、俯视、半径-高度、工作平面截面
  anim_circle.gif           要求 b：MOVJ 连续执行动画（50 帧）
  trajectory.json / workspace.json   图表背后的原始数据
report.docx              实验报告
```

---

## 3. 分工（三人）

| 姓名 | 学号 | 分工 |
|---|---|---|
| 高非亚（组长） | 2025090913005 | 搭建本地运行环境，加载机械臂模型，实现关节驱动控制；开发示教器界面，完成工具坐标系、用户坐标系标定；实现示教编程，记录多组位姿，调用插值函数完成 MOVJ 程序运行，处理急停、速度倍率等功能，完成动画录屏。 |
| 董官平 | 2025090909001 | 编写 DH 正解 FK 算法并完成三档正确性验证；实现 MOVJ 关节插值、轨迹采样录制；蒙特卡洛采样求解机械臂可达工作区域并生成点云图表；解析采样数据绘制轨迹路径、位置‑时间、速度‑时间曲线，参与撰写实验报告与图表整理。 |
| 潘舒雅 | 2025090906028 | 建立实验验收清单，逐项检查 a、b、c、d 四项要求。编写独立测试用例，检查关节限位、角度转换、坐标变换、急停和异常输入。负责接口联调，检查 A 的界面能否正确调用 B 的插值和记录函数。检查模型、资源路径、浏览器控制台报错、页面加载和重复执行问题。统一测试截图、录屏素材、测试结果与报告格式。 |

B 侧（董官平）的实际交付物与文件的对应关系：

| 分工条目 | 交付文件 | 自检方式 |
|---|---|---|
| DH 正解 FK + 三档验证 | `src/analysis/fk.js`、`src/analysis/fkVerify.js`、`tools/verify_fk.py`、`tools/verify_fk.mjs` | `npm run verify`，或页面按钮"运行三档验证" |
| MOVJ 关节插值 | `src/analysis/interp.js`、`assets/program-circle.json` | 页面"载入 12 个示教位姿"→ 看段数与段间跳变 |
| 轨迹采样录制 | `src/analysis/recorder.js` | 页面"连续执行"→"导出 samples.json" |
| 蒙特卡洛可达域 + 点云图表 | `src/analysis/workspace.js`、`tools/analysis_workspace.py` | 页面按钮，或 `python tools/analysis_workspace.py` |
| 轨迹/位置/速度曲线 | `src/analysis/trajectory.js`、`tools/analysis_trajectory.py` | `python tools/analysis_trajectory.py` |
| 实验报告与图表整理 | `report.docx`、`report/` | 文档内每个数字都能在 `report/*.json` 里查到 |

---

## 4. 关键数字（报告里引用的都是这些）

### 正运动学验证

| 档 | 内容 | 结果 |
|---|---|---|
| 一档 | 全零位形 TCP | (−1.600981, +12.491081, −5.126752)，与 GLB 独立基准误差 1.52e−06 u |
| 二档 | 单关节 ±90°（10 组） | 最大误差 1.03e−14 u |
| 三档 | 随机 100 组 | 位置 1.38e−14 u，姿态 4.98e−07° |
| 四档 | 几何自洽（转满一圈不散架） | 五关节全部判定为局部 +Y |

位置误差单位是模型单位，1u = 100 mm。判据 1e−9 u，远严于工程精度。

### 要求 c 轨迹分析

| 项 | 值 |
|---|---|
| 示教位姿 / 程序段数 | 12 个 / 12 段（闭环） |
| 采样 | 504 点，总时长 4.19 s（120 Hz） |
| 路径长度 | 1243.1 mm（理想整圆 1256.6 mm，比值 0.989） |
| 峰值 / 平均速度 | 377.0 / 296.6 mm/s |
| 峰值加速度 | 5506 mm/s² |
| 相对理想圆偏差 | 最大 9.52 mm，均值 3.7 mm，RMS 4.59 mm |

### 要求 d 可达区域

| 项 | 值 |
|---|---|
| 采样 | 60000 组关节角（限位内均匀分布，种子 20240607） |
| 水平半径 | 1 ~ 1633 mm（原始极值；页面显示 2% 分位下界，会到 0） |
| 高度 | −1103 ~ 1994 mm |
| 工作面上方占比 | 70.9% |
| 可达体积（1u 体素近似） | 4.409 m³ |
| x 方向厚度 | 493 mm |
| 内腔空洞 | 0 处（该截面单连通） |

---

## 5. 两个必须先知道的坑

### 5.1 关节轴是局部 +Y，不是 joint_map.json 写的 +Z

**这会直接决定动画会不会散架。** `src/contract.js:39` 与 `src/robot/robotLoader.js:49` 现在写的是
`new THREE.Vector3(0, 0, 1)`，按实测应当改成 `(0, 1, 0)`。

三条独立判据（圆柱面拟合、转满一圈的连杆脱开距离、以及"相邻轴正交"这条先验为何会误导人）
全部写在 **`src/analysis/axisDecision.md`**。改法只需要一行。

### 5.2 四份参数文件互相矛盾

| 来源 | J1 平移 | 模长 | 是否可信 |
|---|---|---|---|
| `assets/robot_articulated.glb`（实际渲染的那个） | 2.853, 4.463, 3.834 | 6.538 | 权威 |
| `assets/joint_map.json` | 2.853, −3.834, 4.463 | 6.538 | 同一台，差绕 X +90° |
| `assets/robot_spec.yaml` | 0.405, −0.381, 3.293 | 3.330 | 另一条运动学链 |
| `assets/robot.urdf` | 0.405, −0.381, 3.293 | 3.330 | 由 spec 同源生成，同错 |

驱动与 FK 只用 GLB 实测值（`assets/robot-model.json`）。`robot_spec.yaml` / `robot.urdf`
是教材对照材料，臂长差一倍，拿它算出来的角度会全错。

---

## 6. 已知限制

- **可达区域是竖直薄片**：这台臂的 J1/J2/J4 转轴平行，x 方向伸展只有 493 mm。这不是算错，
  是这台模型的真实构型；报告里专门用"工作平面截面"图来说明。
- **只做位置级逆解**：IK 只约束 TCP 的三个位置自由度，不对姿态做约束（5 自由度臂有 2 个冗余）。
- **离线渲染不是 WebGL**：`tools/render.py` 是自写的 numpy 软件光栅化器，用来在没有浏览器的
  环境里出图和出 GIF。它与页面的 WebGL 渲染是两条独立路径，视觉细节会有差别（没做阴影/材质，
  只做了平面光照 + z-buffer）。
- **浏览器视觉未做自动化回归**：页面需要在真实浏览器里人工看一眼。
