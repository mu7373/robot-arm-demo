# -*- coding: utf-8 -*-
"""生成 _repo/report.docx —— 实验一 实验报告"""
import json
import pathlib

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ROOT = pathlib.Path(__file__).resolve().parent
OUT = ROOT / "report.docx"

BODY_FONT = "宋体"
HEAD_FONT = "黑体"
MONO_FONT = "Consolas"
BODY_PT = 10.5

doc = Document()

# ---------- 页面 ----------
sec = doc.sections[0]
sec.page_width = Cm(21.0)
sec.page_height = Cm(29.7)
sec.top_margin = Cm(2.2)
sec.bottom_margin = Cm(2.2)
sec.left_margin = Cm(2.4)
sec.right_margin = Cm(2.4)
USABLE = 21.0 - 2.4 * 2  # 16.2 cm

# 正文默认样式
style = doc.styles["Normal"]
style.font.name = BODY_FONT
style.font.size = Pt(BODY_PT)
style.element.rPr.rFonts.set(qn("w:eastAsia"), BODY_FONT)
style.paragraph_format.space_before = Pt(0)
style.paragraph_format.space_after = Pt(4)
style.paragraph_format.line_spacing = 1.35


def _set_run(run, font=None, size=None, bold=None, color=None, mono=False):
    f = MONO_FONT if mono else (font or BODY_FONT)
    run.font.name = f
    run._element.rPr.rFonts.set(qn("w:eastAsia"), BODY_FONT if not mono else f)
    if size:
        run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if color:
        run.font.color.rgb = RGBColor(*color)


def h(level, text):
    sizes = {1: 15, 2: 12.5, 3: 11}
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.space_before = Pt(10 if level == 1 else 8)
    pf.space_after = Pt(4)
    pf.keep_with_next = True
    r = p.add_run(text)
    _set_run(r, font=HEAD_FONT, size=sizes[level], bold=True)
    return p


def para(text, bold=False, size=None, indent=True, align=None, after=4):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(after)
    if indent:
        p.paragraph_format.first_line_indent = Pt(BODY_PT * 2)
    if align == "center":
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.first_line_indent = Pt(0)
    r = p.add_run(text)
    _set_run(r, size=size, bold=bold)
    return p


def bullet(text, level=0):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.left_indent = Cm(0.6 + 0.55 * level)
    pf.first_line_indent = Cm(-0.42)
    pf.space_after = Pt(2)
    r = p.add_run(("· " if level == 0 else "– ") + text)
    _set_run(r)
    return p


def mono(text, after=6):
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.left_indent = Cm(0.7)
    pf.space_before = Pt(2)
    pf.space_after = Pt(after)
    pf.line_spacing = 1.05
    for i, line in enumerate(text.split("\n")):
        if i:
            p.add_run("\n")
        r = p.add_run(line)
        _set_run(r, size=9, mono=True)
    return p


def caption(text):
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(2)
    p.paragraph_format.space_after = Pt(8)
    r = p.add_run(text)
    _set_run(r, size=9, bold=False, color=(0x44, 0x44, 0x44))
    return p


def shade(cell, hex_fill):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_fill)
    tcPr.append(shd)


def table(headers, rows, widths, font_pt=9, head_fill="E8EDF3", first_col_bold=False):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False

    tblPr = t._tbl.tblPr
    layout = OxmlElement("w:tblLayout")
    layout.set(qn("w:type"), "fixed")
    tblPr.append(layout)

    total = sum(widths)
    cms = [w / total * USABLE for w in widths]

    hdr = t.rows[0].cells
    for i, txt in enumerate(headers):
        hdr[i].width = Cm(cms[i])
        shade(hdr[i], head_fill)
        p = hdr[i].paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_after = Pt(1)
        r = p.add_run(str(txt))
        _set_run(r, font=HEAD_FONT, size=font_pt, bold=True)

    for row in rows:
        cells = t.add_row().cells
        for i, txt in enumerate(row):
            cells[i].width = Cm(cms[i])
            p = cells[i].paragraphs[0]
            p.paragraph_format.space_after = Pt(1)
            p.paragraph_format.line_spacing = 1.1
            if i > 0 or not first_col_bold:
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            r = p.add_run(str(txt))
            _set_run(r, size=font_pt, bold=(first_col_bold and i == 0))
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t


def figure(rel_path, width_cm, cap):
    p = ROOT / rel_path
    if not p.exists():
        para(f"[图缺失：{rel_path}]", size=9)
        return
    doc.add_picture(str(p), width=Cm(width_cm))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    doc.paragraphs[-1].paragraph_format.space_after = Pt(2)
    caption(cap)


# ---------- 数据 ----------
prog = json.loads((ROOT / "assets/program-circle.json").read_text(encoding="utf-8"))
traj = json.loads((ROOT / "report/trajectory.json").read_text(encoding="utf-8"))
ws = json.loads((ROOT / "report/workspace.json").read_text(encoding="utf-8"))
fv = json.loads((ROOT / "report/fk_verify.json").read_text(encoding="utf-8"))

# ================= 标题 =================
t = doc.add_paragraph()
t.alignment = WD_ALIGN_PARAGRAPH.CENTER
t.paragraph_format.space_before = Pt(6)
t.paragraph_format.space_after = Pt(2)
r = t.add_run("实验一  机械臂仿真（一）")
_set_run(r, font=HEAD_FONT, size=19, bold=True)

t2 = doc.add_paragraph()
t2.alignment = WD_ALIGN_PARAGRAPH.CENTER
t2.paragraph_format.space_after = Pt(10)
r = t2.add_run("模型导入、关节运动控制与示教器界面")
_set_run(r, font=HEAD_FONT, size=12.5, bold=False, color=(0x33, 0x33, 0x33))

table(
    ["项目", "内容"],
    [
        ["课程", "计算机图形与动画（模块 CM11，支撑课程目标 CO4，8 学时）"],
        ["实验平台", "Three.js / WebGL，5 自由度协作臂，1 模型单位 = 100 mm"],
        ["小组成员", "高非亚 2025090913005／董官平 2025090909001／潘舒雅 2025090906028"],
        ["要求", "a 示教器交互与坐标系标定　b 示教编程与连续执行　c 轨迹分析　d 可达区域分析"],
    ],
    widths=[1.1, 5.2],
    font_pt=9,
    first_col_bold=True,
)

# ================= 一、实验目的 =================
h(1, "一、实验目的与要求")
para("本实验在 WebGL 环境下建立 5 自由度协作臂的可交互仿真：把关节参数从三维模型里读出来、"
     "用场景树完成正运动学、做出可点动的示教器、再以“空中画圆”为任务演示示教编程，"
     "最后量化两件事——指令轨迹与理想几何差多少、机械臂到底能够到哪儿。")
para("四项要求及其判据（缺一不可）：")
bullet("a　示教器交互与坐标系标定：关节点动、工具坐标系（TCP）与用户坐标系标定、标定后坐标变换正确。")
bullet("b　示教编程与连续执行：记录 ≥ 4 个示教位姿，用 MOVJ 指令串联成程序并连续执行成动画。")
bullet("c　轨迹分析：逐帧记录末端 TCP，绘制轨迹路径图与位置／速度–时间曲线，分析插值效果与平滑性。")
bullet("d　可达区域分析：在关节限位内蒙特卡洛采样，经正运动学生成 TCP 点云，画出可达域并标注边界与空洞。")

# ================= 二、环境 =================
h(1, "二、实验环境")
table(
    ["项", "版本／说明"],
    [
        ["操作系统", "Windows 11"],
        ["运行时", "Node.js 24.19.0（本地静态服务器）、Python 3.13（离线分析出图）"],
        ["图形库", "three.js 0.169.0（WebGLRenderer / GLTFLoader / OrbitControls）"],
        ["面板库", "lil-gui 风格的纯 DOM 面板（自写，无第三方 UI 依赖）"],
        ["模型", "robot_articulated.glb，11.2 MB，8227 个三角面，14 个节点"],
        ["离线渲染", "自写 numpy + Pillow 软件光栅化器（tools/render.py），不依赖 matplotlib / GPU"],
    ],
    widths=[1.2, 5.1],
    font_pt=9,
    first_col_bold=True,
)
para("必须通过 http:// 访问而不能直接双击打开 file:// 页面：ES Module 与 .glb 模型会被同源策略拦截。"
     "本地用 node server.mjs 起一个零依赖静态服务器（默认 8123 端口）即可。", indent=True)

# ================= 三、原理 =================
h(1, "三、原理与方法")

h(2, "3.1 关节的归一化坐标变换")
para("场景树里每个关节节点都带一个静止旋转 R_fix（模型导出时就已经烘进节点里），它保证“节点的某个局部轴"
     "恰好是该关节的转动轴”。于是单个关节的变换可以干净地拆成三级：")
mono("T_joint(θ) = T_translate(t_j) · R_fix · R_axis(θ)\n"
     "  t_j      常量，枢轴在父坐标系里的位置（GLB 节点 translation）\n"
     "  R_fix    常量，节点的静止旋转（GLB 节点 rotation，即 qZero）\n"
     "  R_axis   唯一的变量，绕实测转轴转 θ")
para("这正好就是三维引擎每帧沿层级重算 matrixWorld 所做的事，因此“写一遍 FK”和“让场景树自己算”"
     "是同一件事，两者可以互相校对——这正是后面三档验证的基础。")

h(2, "3.2 正运动学与数值实现")
para("沿 机器人底座 → J1 → link_1 → … → J5 → link_5 → link_flange → tcp 逐级左乘世界矩阵：")
mono("R_local = R_fix · R_axis(θ_k)\n"
     "R_world[k] = R_world[parent] · R_local\n"
     "p_world[k] = R_world[parent] · t_k + p_world[parent]")
para("实现成两套完全独立的代码：浏览器侧 src/analysis/fk.js（纯数学，不依赖 three，可用 node 直接跑），"
     "离线侧 tools/verify_fk.py（numpy，用轴角 Rodrigues 公式，直接读 GLB 二进制）。"
     "两套之间不共享任何函数，因此对拍才有意义。")

h(2, "3.3 MOVJ 与轨迹采样")
para("MOVJ（关节空间插值）不规划笛卡尔直线，而是把起止两组关节角按同一条归一化时间曲线插值：")
mono("q(s) = q_A + s · (q_B − q_A)\n"
     "s = trapezoid(t / T),  T = 1.25 · max_k|Δq_k| / ω_max")
para("梯形速度曲线两端加速度有限、中段匀速，比线性插值少了启动与停止时的速度突变；系数 1.25 来自"
     "加速段占比 a=0.2 时的峰值系数 1/(1−a)，用它把时长换算成“峰值关节速度恰好等于 ω_max=90°/s”。"
     "末端轨迹由各段关节角经 FK 逐帧算出，120 Hz 采样后差分得到速度、加速度。")

h(2, "3.4 蒙特卡洛可达域")
para("在五个关节的限位内均匀采样 60000 组关节角，逐组经 FK 算出 TCP 位置，即得末端可达点云。"
     "在此之上做三件事：极坐标分角度统计外边界 r_max(θ) 与内边界 r_min(θ)（取 2%／98% 分位数，"
     "避免采样孤立点把边界拉成毛刺）；把点云栅格化成占位图；从栅格外沿做洪水填充，"
     "被可达点围住却自身为空的格子即为“不可达空洞”。")

# ================= 四、模型核查 =================
h(1, "四、模型与参数核查")

h(2, "4.1 素材里有四份互相矛盾的参数")
para("工程目录下的 assets/ 提供了四份描述同一台机器人的文件，但它们不是同一套运动学参数：")
table(
    ["来源", "J1 平移（模型单位）", "由 J1 算出的臂长", "结论"],
    [
        ["robot_articulated.glb（实际渲染的模型）", "2.853, 4.463, 3.834", "6.538", "权威，以它为准"],
        ["joint_map.json", "2.853, −3.834, 4.463", "6.538", "同一台，差绕 X 轴 +90°"],
        ["robot_spec.yaml", "0.405, −0.381, 3.293", "3.330", "另一条运动学链，勿用于计算"],
        ["robot.urdf", "0.405, −0.381, 3.293", "3.330", "由 spec 同源生成，同错"],
    ],
    widths=[3.0, 2.4, 1.4, 2.2],
    font_pt=9,
)
para("robot_spec.yaml 与 robot.urdf 的臂长只有 GLB 的一半，若拿它们算角度，结果会整体错位。"
     "本实验的做法是：把 GLB 的节点层级、静止旋转、限位一次性导出成 assets/robot-model.json，"
     "之后驱动与 FK 只认这一份，joint_map.json 仅当关节名与限位的参考。")

h(2, "4.2 转轴不是 joint_map 里写的 +Z")
para("joint_map.json 的 convention 字段写的是“绕节点局部 +Z 转”，若照此实现，关节转到 ±90° 时"
     "连杆会从关节外壳里旋出去（视觉上就是“散架”）。实测结论是五个关节都绕节点局部 +Y，依据三条独立判据：")
bullet("判据一（最硬）：关节外壳是回转体，其轴线就是转轴。取枢轴附近的连杆顶点做协方差分析，"
       "最小特征向量即轴线，与三个候选局部轴比 |cos|——五个关节对局部 Y 的 |cos| 均为 0.994~1.000，对 X、Z 均 < 0.1。")
bullet("判据二：把单关节从 0° 转到 360°（12 步），每步算相邻连杆顶点云的最小距离。绕 Y 时始终贴合"
       "（< 0.18 单位，属三角面片离散误差），绕 X 或 Z 时 91° 处距离跳到 0.8~3.0 单位。")
bullet("判据三（反例）：一开始按“相邻关节轴应互相垂直”的先验去搜索，选出了 J1 绕 X、其余绕 Y 的组合，"
       "四对相邻轴点积全为 0，看起来完美——但它是错的。实测 J1 与 J2 的世界轴平行且反向（点积 −1），"
       "这台臂的 J1/J2 是一对平行轴。先验只能用来生成候选，判定必须靠几何测量。")
para("完整判据表与复现方式见仓库内 src/analysis/axisDecision.md。")

# ================= 五、要求 a =================
h(1, "五、要求 a：示教器界面与坐标标定")

h(2, "5.1 界面构成")
para("示教器界面（index.html）分 8 个区，全部用原生 DOM 实现：")
table(
    ["区", "内容"],
    [
        ["运行状态", "使能／失能、急停、回零位、速度倍率 10%~100%"],
        ["关节", "5 组“滑块 + 数值框 + 限位标签”，显示单位度，内部一律弧度"],
        ["末端位姿", "TCP 位置（mm）、RPY（度）、四元数，以及“自写 FK ↔ 场景树”的实时偏差"],
        ["坐标系标定", "TCP 偏移 xyz、用户坐标系平移 xyz 与绕 Z 转角，可应用／清零"],
        ["位姿示教", "记录当前位姿、位姿列表、MOVJ 连续执行、停止、导出 samples.json"],
        ["轨迹图表", "速度–时间与距基座距离–时间双曲线，附轨迹点数／路径长度／峰值速度"],
        ["显示开关", "轨迹线、可达点云、关节轴、地面网格、计算可达区域"],
        ["自检", "一键跑三档验证，日志框输出每一档的误差"],
    ],
    widths=[1.35, 5.0],
    font_pt=9,
    first_col_bold=True,
)

h(2, "5.2 三个关键实现细节")
bullet("限位钳制写在驱动函数内部而不是界面层：setJoint 里先 clamp(θ, lower, upper) 再写四元数，"
       "这样无论从滑块、数值框、MOVJ 还是自检调用，都不可能越限。")
bullet("内部一律用四元数，只在显示层转欧拉角：RPY 在 ±180° 附近不连续，若把欧拉角当状态存，"
       "姿态显示会跳变。")
bullet("位姿读取统一走一个入口：先 robot.updateMatrixWorld(true) 再取 TCP 的世界位姿，"
       "否则滑块动完立刻读会读到上一帧。")

h(2, "5.3 坐标系标定")
para("工具坐标系：TCP 在法兰坐标系下的偏移量（实测 offset_local = (0, 0, 1.336) 模型单位，"
     "即 133.6 mm，沿法兰 +Z）。界面上的“TCP 偏移”把它叠加到末端的显示位姿上，"
     "用于验证标定后读数是否符合预期。")
para("用户坐标系：界面提供平移三参数与绕 Z 转角，把世界坐标变换到用户坐标系下显示。"
     "标定正确性的判据是：同一物理点在两个坐标系下的读数经变换矩阵回代后相互吻合。")

# ================= 六、要求 b =================
h(1, "六、要求 b：示教编程与连续执行")

h(2, "6.1 任务与位姿")
para("任务定为“空中画圆”：在竖直平面 x = −150 mm 上，以 (−150, 1100, −450) mm 为圆心、半径 200 mm、"
     "等分 12 点作为示教位姿，用 MOVJ 依次串联，第 12 段闭合回第 1 点。")
para("逆解用阻尼最小二乘（DLS）加路径连续法：每个位姿以上一个位姿为初始种子，"
     "再叠加若干小扰动做多起点，取残差最小且离上一步最近的一组解，从而避免 5 自由度臂的冗余解在"
     "相邻位姿间来回跳支路。求出的 12 组位姿如下（角度单位：度；TCP 单位：mm）：")

rows = []
center = prog["circle"]["center"]
radius = prog["circle"]["radius"]
u = prog["circle"]["u"]
v = prog["circle"]["v"]
import math as _m
for i, q in enumerate(prog["posesDeg"]):
    th = 2 * _m.pi * i / 12
    p = [center[d] + radius * (_m.cos(th) * u[d] + _m.sin(th) * v[d]) for d in range(3)]
    rows.append([f"P{i+1}"] + [f"{x:+.1f}" for x in q] +
                [f"{p[0]*100:.1f}, {p[1]*100:.1f}, {p[2]*100:.1f}"])
table(["位姿", "J1", "J2", "J3", "J4", "J5", "TCP x, y, z"],
      rows, widths=[0.7, 1.0, 1.0, 0.9, 0.9, 0.9, 2.2], font_pt=8.5)
para(f"全部 12 个位姿的逆解残差为 {max(prog['ikResidual']):.2e} 模型单位（可视为精确解）；"
     f"示教位姿之间最大的单关节跳变为 {prog['continuity']['maxPoseJumpDeg']:.1f}°，"
     "说明相邻位姿共用同一条逆解支路，程序可以连续执行而不会突然翻转姿态。")

h(2, "6.2 程序编排")
bullet("每个 MOVJ 段的时长由段内最大关节增量决定：T = 1.25 · max|Δq| / ω_max，下限 0.35 s。")
bullet("速度倍率作用在时长上（duration = base / scale），不改变轨迹形状，只改变快慢——"
       "与真实示教器的倍率语义一致。")
bullet("执行前用 checkProgram 自检段间跳变，超过阈值就在日志里报警，避免“程序能跑但姿态乱跳”。")
bullet("急停与“停止”按钮在执行循环的每一帧检查，可中途打断并停在当前位姿。")

figure("report/frame_2.png", 14.2,
       "图 1　MOVJ 程序执行中的一帧（离线渲染，HUD 显示时间、TCP 坐标与五个关节角的当前值；"
       "浅绿色为目标圆，亮绿色为已走过的轨迹）")
para("在线动画为 report/anim_circle.gif（50 帧，825 KB）。离线渲染用自写的软件光栅化器，"
     "与页面的 WebGL 渲染是两条独立路径，视觉细节（材质、阴影）会有差异，但几何是同一份 FK 输出。")

# ================= 七、要求 c =================
h(1, "七、要求 c：轨迹分析")
para("连续执行时以 120 Hz 逐帧记录末端 TCP，得到 %d 个采样点。由于示教点在圆上但 MOVJ 走的是"
     "关节空间的直线，末端实际走出的是一条“内切多边形的圆滑版”，与理想圆之间存在系统性偏差。"
     % traj["samples"])

figure("report/fig_trajectory.png", 16.0,
       "图 2　要求 c：轨迹路径图与位置／速度／偏差–时间曲线")

dv = traj["deviation_mm"]
table(
    ["指标", "数值"],
    [
        ["示教位姿数 / 程序段数", "12 / 12（闭环）"],
        ["采样点数 / 总时长", f"{traj['samples']} 点 / {traj['duration_s']:.2f} s（120 Hz）"],
        ["路径长度", f"{traj['pathLength_mm']:.1f} mm（理想整圆 {traj['idealCircle_mm']:.1f} mm，"
                     f"比值 {traj['pathLength_mm']/traj['idealCircle_mm']:.3f}）"],
        ["峰值 / 平均速度", f"{traj['peakSpeed_mm_s']:.1f} / {traj['meanSpeed_mm_s']:.1f} mm/s"],
        ["峰值加速度", f"{traj['peakAccel_mm_s2']:.0f} mm/s²"],
        ["相对理想圆的偏差", f"最大 {dv['max']:.2f} mm，均值 {dv['mean']:.1f} mm，RMS {dv['rms']:.2f} mm"],
    ],
    widths=[2.0, 4.6],
    font_pt=9,
    first_col_bold=True,
)

h(2, "7.1 怎么读这三张图")
bullet("轨迹形状（图 2a）：红色是 MOVJ 实际走出的曲线，灰色是理想圆。12 个蓝色点是示教位姿，"
       "都精确落在圆上；段与段之间红色向圆心方向内凹，最大内凹 9.5 mm。这是关节空间插值的固有特征——"
       "两个示教点之间，关节走直线而末端走弧，弧的弦高就体现为内凹。")
bullet("位置–时间（图 2b）：X 分量是一条水平直线，因为整个圆画在 x = −150 mm 的竖直平面内，"
       "这条“恒为常数”的曲线反过来验证了轨迹确实没有侧向漂移。Y 与 Z 都是平滑的正弦型曲线，"
       "没有台阶，说明位姿序列连续。")
bullet("速度–时间（图 2c）：12 段梯形首尾相接，整段内峰值 377 mm/s、均值 297 mm/s；"
       "段与段之间速度掉到接近 0，然后再升起来。这个“顿挫”不是 bug：每段 MOVJ 都要求起止时刻关节速度为零，"
       "12 段独立执行就必然是 12 次起步–停车。若要连贯，需要改用连续路径插值（实验二内容）。")
bullet("偏差–时间（图 2d）：每段中间偏差最大、两端为 0——正好对应“示教点精确、中途偏离”的弦高规律，"
       "与轨迹形状图互相印证。")

# ================= 八、要求 d =================
h(1, "八、要求 d：可达区域分析")
para("在五个关节的限位内均匀采样 %d 组关节角（随机种子 %d），逐组正向计算 TCP 位置，得到点云。"
     "单位换算 1 模型单位 = %.0f mm。" % (ws["nSamples"], ws["seed"], ws["unitMM"]))

figure("report/fig_workspace.png", 16.0,
       "图 3　要求 d：可达域立面投影、俯视投影、半径–高度剖面、工作平面截面占位图")

table(
    ["指标", "数值"],
    [
        ["水平半径范围", f"{ws['radius'][0]*100:.0f} ~ {ws['radius'][1]*100:.0f} mm"],
        ["高度范围", f"{ws['y'][0]*100:.0f} ~ {ws['y'][1]*100:.0f} mm"],
        ["x 方向（进深）范围", f"{ws['x'][0]*100:.0f} ~ {ws['x'][1]*100:.0f} mm，厚度 {(ws['x'][1]-ws['x'][0])*100:.0f} mm"],
        ["工作面上方占比", f"{ws['aboveWorkSurfaceRatio']*100:.1f}%（工作面高度 109.5 mm）"],
        ["可达体积（体素近似）", f"{ws['volumeVoxel']:.0f} 模型单位³ = {ws['volumeVoxel']/1000:.3f} m³"],
        ["俯视投影中的不可达空洞", f"{ws['holes']['cells']} 格（单格 0.6 单位）"],
    ],
    widths=[2.1, 4.5],
    font_pt=9,
    first_col_bold=True,
)

h(2, "8.1 结论：可达域是一片竖直薄片")
para("图 3a 的立面投影与 3b 的俯视投影都退化成细条，这不是画错了，而是这台臂的真实构型："
     "J1、J2、J4 的转轴互相平行（都在水平方向），五个关节只能在一个平面内伸缩与摆动，"
     "因此末端只能在厚度约 493 mm 的竖直薄片内运动，而不是常见的“球壳状”三维工作空间。")
para("这一点在 8a 的“半径–高度剖面”里看得最清楚：半径从 0 到 1633 mm 连续铺满，"
     "高度可达 1994 mm，但在半径小于约 350 mm 的近身区域存在不可达的空心——"
     "因为连杆本身有长度，机械臂“收不回来”。")

h(2, "8.2 空洞的判定方法")
para("把点云投影成占位栅格后，从栅格外沿开始做洪水填充，凡是从外沿能连通的空格标记为“外部”，"
     "剩下被可达点完全围住的空格就是空洞。为避免把采样稀疏造成的零星空格误判为空洞，"
     "先按连通分量过滤：单格的孤立空格判为采样假象，只有 ≥3 格连通的块才算真空洞。"
     f"本模型在俯视投影下的真空洞数为 {ws['holes']['cells']}，即该投影几乎单连通。")

h(2, "8.3 一个方法论上的教训")
para("最初我们用“目标点到最近可达点的距离”当作可达性判据，结果 60000 个点的云在目标附近仍有"
     "0.2~2 个单位的“最近距离”，于是误判多处不可达。实际上那是采样密度造成的假象："
     "点云平均间距约 0.5 单位，最近距离天然就是这个量级。判据必须建立在“是否已被占据的格子”上，"
     "而不是“最近的一个采样点离我多远”。")

# ================= 九、验证 =================
h(1, "九、正确性验证")
para("正运动学的正确性用四档递进的方式验证。前三档是数值对拍，第四档是几何自洽性。"
     "位置判据取 1e−9 模型单位（= 1e−7 mm），远严于工程精度。")
table(
    ["档", "内容", "结果"],
    [
        ["一档", "全零位形末端位姿 vs GLB 独立基准", "误差 1.52e−06 单位；浏览器侧与 numpy 侧互差 6.30e−15"],
        ["二档", "五个关节分别 ±90°，共 10 组", "最大误差 1.03e−14 单位"],
        ["三档", "随机 100 组关节角，自写 FK vs 场景树", "位置 1.38e−14 单位，姿态 4.98e−07 度"],
        ["四档", "转满一圈看连杆是否脱开（几何自洽）", "五关节全部判定为局部 +Y"],
    ],
    widths=[0.8, 3.6, 4.2],
    font_pt=9,
)
table(
    ["关节", "绕局部 X", "绕局部 Y", "绕局部 Z", "判定"],
    [
        ["J1", "2.0527", "0.1479", "2.0512", "+Y"],
        ["J2", "2.9844", "0.0548", "2.5364", "+Y"],
        ["J3", "（三轴相同，改圆柱拟合）", "0.012 / 1.000 / 0.013", "—", "+Y"],
        ["J4", "0.8036", "0.0632", "0.8335", "+Y"],
        ["J5", "0.8850", "0.0598", "0.8847", "+Y"],
    ],
    widths=[0.9, 1.6, 2.4, 1.4, 0.8],
    font_pt=9,
)
para("第四档的数值是“转满一圈后相邻连杆顶点云的最小距离”（模型单位）。轴配对时连杆始终贴合，"
     "数值在 0.18 以内；轴配错时连杆从外壳里旋出，跳到 0.8 以上。J3 因相邻连杆本身同轴且短，"
     "三个轴都不脱开，故改用 4.2 节的圆柱面拟合判定（对局部 Y 的 |cos| = 1.000）。")

# ================= 十、结论 =================
h(1, "十、结论与讨论")
para("1. 四项要求全部完成且都有可复现的数字支撑：a 的示教器与标定、b 的 12 位姿 MOVJ 闭环程序、"
     "c 的轨迹四联图、d 的可达域四联图。")
para("2. 正运动学的正确性不靠“看起来对”来担保：三档数值对拍把误差压到 1e−14 量级，"
     "第四档几何自洽性把“轴到底对不对”这件事从审美判断变成了可测量的距离判据。")
para("3. MOVJ 与理想圆的偏差有明确物理解释，不是误差累积：偏差在每段中点最大、在示教点为零，"
     "峰值 9.5 mm、RMS 4.59 mm，与弦高规律一致。速度曲线出现 12 次顿挫，源于每段独立执行时的"
     "零起止速度要求。")
para("4. 可达域呈竖直薄片，源于 J1/J2/J4 转轴平行这一真实构型；近身半径 350 mm 内不可达，"
     "源于连杆自身长度。这两条都不是算法缺陷，而是这台机械臂的几何属性，也正好说明了为什么"
     "“示教 + 关节插值”这种看似笨拙的方式仍是工业现场的主流：它避开了在薄片状可达域里做笛卡尔规划。")

h(1, "十一、分工")
table(
    ["姓名（学号）", "承担工作"],
    [
        ["高非亚（2025090913005）",
         "搭建本地运行环境、加载机械臂模型、实现关节驱动控制；开发示教器界面，完成工具坐标系与"
         "用户坐标系标定；实现示教编程、记录多组位姿、调用插值函数完成 MOVJ 运行，处理急停与速度倍率；"
         "完成动画录屏。"],
        ["董官平（2025090909001）",
         "编写正运动学 FK 算法并完成三档正确性验证；实现 MOVJ 关节插值、轨迹采样录制；"
         "蒙特卡洛采样求解可达工作区域并生成点云图表；绘制轨迹路径、位置–时间、速度–时间曲线；"
         "参与撰写实验报告与图表整理。"],
        ["潘舒雅（2025090906028）",
         "建立验收清单，逐项检查 a、b、c、d；编写独立测试用例，检查关节限位、角度转换、坐标变换、"
         "急停与异常输入；负责接口联调，检查界面能否正确调用插值函数与记录函数；"
         "检查模型与资源路径、控制台报错、页面加载与重复执行；统一截图与报告格式。"],
    ],
    widths=[1.5, 5.1],
    font_pt=9,
    first_col_bold=True,
)

# 页脚页码
footer_p = sec.footer.paragraphs[0]
footer_p.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = footer_p.add_run()
_set_run(r, size=9, mono=True)
fld = OxmlElement("w:fldSimple")
fld.set(qn("w:instr"), "PAGE")
r._element.addnext(fld)

doc.save(OUT)
print("saved", OUT)
