"""GLB 离屏软件渲染器 —— 不依赖浏览器/WebGL，纯 numpy + Pillow 光栅化。
用途：在没有 GPU/浏览器的环境里核验模型朝向、姿态是否正确，并出报告插图。
"""
import json, struct, math
import numpy as np
from PIL import Image

# ─────────────────────────── GLB 解析 ───────────────────────────
def load_glb(path):
    d = open(path, 'rb').read()
    _, ver, total = struct.unpack('<III', d[:12])
    off, js, bin_ = 12, None, None
    while off < total:
        clen, ctype = struct.unpack('<II', d[off:off + 8])
        chunk = d[off + 8: off + 8 + clen]
        if ctype == 0x4E4F534A:
            js = json.loads(chunk.decode('utf-8'))
        elif ctype == 0x004E4942:
            bin_ = chunk
        off += 8 + clen
    return js, bin_

CT = {5120: ('b', 1), 5121: ('B', 1), 5122: ('h', 2), 5123: ('H', 2), 5125: ('I', 4), 5126: ('f', 4)}
NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}

def read_accessor(js, bin_, idx):
    acc = js['accessors'][idx]
    n = NC[acc['type']]
    fmt, size = CT[acc['componentType']]
    bv = js['bufferViews'][acc['bufferView']]
    base = bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    stride = bv.get('byteStride') or n * size
    out = np.zeros((acc['count'], n), dtype=np.float64)
    for i in range(acc['count']):
        o = base + i * stride
        out[i] = struct.unpack_from('<' + fmt * n, bin_, o)
    return out if n > 1 else out[:, 0]

# ─────────────────────────── 变换 ───────────────────────────
def q2m(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])

def trs(n):
    M = np.eye(4)
    s = n.get('scale', [1, 1, 1])
    M[:3, :3] = q2m(n.get('rotation', [0, 0, 0, 1])) @ np.diag(s)
    M[:3, 3] = n.get('translation', [0, 0, 0])
    return M

def world_matrices(js):
    nodes = js['nodes']
    W = [None] * len(nodes)
    def rec(i, P):
        W[i] = P @ trs(nodes[i])
        for c in nodes[i].get('children', []):
            rec(c, W[i])
    for r in js['scenes'][js.get('scene', 0)]['nodes']:
        rec(r, np.eye(4))
    return W

# ─────────────────────────── 三角化收集 ───────────────────────────
def gather(js, bin_, W, joint_override=None):
    """返回 (V, N, C) —— 世界坐标顶点、面法线、面颜色，长度均为三角面数"""
    tris, cols = [], []
    pal = [(0.78, 0.80, 0.83), (0.86, 0.55, 0.18), (0.35, 0.55, 0.78),
           (0.70, 0.72, 0.75), (0.55, 0.58, 0.62), (0.90, 0.90, 0.92)]
    ci = 0
    for i, n in enumerate(js['nodes']):
        if 'mesh' not in n:
            continue
        M = W[i]
        if joint_override and n.get('name') in joint_override:
            M = joint_override[n.get('name')]
        for prim in js['meshes'][n['mesh']]['primitives']:
            if 'indices' not in prim:
                continue
            P = read_accessor(js, bin_, prim['attributes']['POSITION'])
            idx = read_accessor(js, bin_, prim['indices']).astype(int)
            V = (M @ np.hstack([P, np.ones((len(P), 1))]).T).T[:, :3]
            T = V[idx.reshape(-1, 3)]
            tris.append(T)
            col = pal[ci % len(pal)]
            if 'material' in prim:
                mat = js['materials'][prim['material']]
                pbr = mat.get('pbrMetallicRoughness', {})
                bc = pbr.get('baseColorFactor')
                if bc:
                    col = tuple(bc[:3])
            cols.append(np.tile(np.array(col), (len(T), 1)))
            ci += 1
    return np.concatenate(tris), np.concatenate(cols)

# ─────────────────────────── 光栅化 ───────────────────────────
def render(T, C, eye, target, up, W=1280, H=900, fovy=40.0, bg=(0.07, 0.08, 0.10),
           light=(-0.45, -0.75, 0.55), ground=None, near=0.05):
    eye = np.array(eye, float); target = np.array(target, float); up = np.array(up, float)
    f = target - eye; f /= np.linalg.norm(f)
    s = np.cross(f, up); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    view = np.array([[*s, -s @ eye], [*u, -u @ eye], [*-f, f @ eye], [0, 0, 0, 1]], float)

    Vc = (view @ np.hstack([T.reshape(-1, 3), np.ones((T.shape[0] * 3, 1))]).T).T[:, :3].reshape(-1, 3, 3)
    z = -Vc[:, :, 2]

    img = np.zeros((H, W, 3), np.float64)
    img[:] = bg
    zbuf = np.full((H, W), 1e9)

    fl = 1.0 / math.tan(math.radians(fovy) / 2)
    aspect = W / H

    # 地面网格：在 ground=(axis, value) 指定的平面上画线，帮助判断哪个轴朝上
    if ground is not None:
        axis, val, span, step = ground
        grid = bg
        def line(p0, p1, col):
            n = 120
            for t in np.linspace(0, 1, n):
                p = np.array(p0) * (1 - t) + np.array(p1) * t
                pc = (view @ np.array([*p, 1.0]))[:3]
                if pc[2] > -0.05: continue
                sx = int((pc[0] * fl / aspect / -pc[2] * 0.5 + 0.5) * W)
                sy = int((1 - (pc[1] * fl / -pc[2] * 0.5 + 0.5)) * H)
                if 0 <= sx < W and 0 <= sy < H:
                    img[sy, sx] = 0.9 * np.array(col) + 0.1 * img[sy, sx]
        r0, r1 = span
        for k in np.arange(r0, r1 + 1e-9, step):
            for disc in (0, 1):
                a = [0, 0, 0]; a[disc] = k
                b = [0, 0, 0]; b[1 - disc] = k
                a[axis] = val; b[axis] = val
                a[(axis + 1) % 3] = r0; b[(axis + 1) % 3] = r0
                a[(axis + 2) % 3] = r0; b[(axis + 2) % 3] = r1
                line(a, b, (0.22, 0.30, 0.38))
                a[(axis + 2) % 3] = r1; b[(axis + 2) % 3] = r1
                a[(axis + 1) % 3] = r0; b[(axis + 1) % 3] = r1
                line(a, b, (0.22, 0.30, 0.38))

    L = np.array(light, float); L /= np.linalg.norm(L)

    for k in range(len(T)):
        if (z[k] <= near).any():
            continue
        p = Vc[k]
        sx = (p[:, 0] * fl / aspect / -p[:, 2] * 0.5 + 0.5) * W
        sy = (1 - (p[:, 1] * fl / -p[:, 2] * 0.5 + 0.5)) * H
        iz = 1.0 / -p[:, 2]
        x0, x1 = int(max(0, math.floor(sx.min()))), int(min(W - 1, math.ceil(sx.max())))
        y0, y1 = int(max(0, math.floor(sy.min()))), int(min(H - 1, math.ceil(sy.max())))
        if x1 < x0 or y1 < y0:
            continue
        d = (sy[1] - sy[2]) * (sx[0] - sx[2]) + (sx[2] - sx[1]) * (sy[0] - sy[2])
        if abs(d) < 1e-12:
            continue
        nrm = np.cross(T[k][1] - T[k][0], T[k][2] - T[k][0])
        ln = np.linalg.norm(nrm)
        if ln < 1e-12:
            continue
        nrm /= ln
        # 双面光照
        sh = 0.32 + 0.68 * abs(float(nrm @ L))
        rgb = np.clip(C[k] * sh, 0, 1)

        xs = np.arange(x0, x1 + 1) + 0.5
        ys = np.arange(y0, y1 + 1) + 0.5
        X, Y = np.meshgrid(xs, ys)
        l0 = ((sy[1] - sy[2]) * (X - sx[2]) + (sx[2] - sx[1]) * (Y - sy[2])) / d
        l1 = ((sy[2] - sy[0]) * (X - sx[2]) + (sx[0] - sx[2]) * (Y - sy[2])) / d
        l2 = 1 - l0 - l1
        m = (l0 >= -1e-9) & (l1 >= -1e-9) & (l2 >= -1e-9)
        if not m.any():
            continue
        zz = 1.0 / (l0 * iz[0] + l1 * iz[1] + l2 * iz[2])
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        upd = m & (zz < sub)
        if upd.any():
            sub[upd] = zz[upd]
            img[y0:y1 + 1, x0:x1 + 1][upd] = rgb

    return Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))


def look_at_eye(center, dist, azim_deg, elev_deg):
    a, e = math.radians(azim_deg), math.radians(elev_deg)
    return np.array(center) + dist * np.array([math.cos(e) * math.cos(a),
                                               math.cos(e) * math.sin(a),
                                               math.sin(e)])


def up_vector(cam_up):
    return {'z': (0, 0, 1), 'y': (0, 1, 0), 'x': (1, 0, 0)}[cam_up]


def fit_eye(T, cam_up, azim=35, elev=18, pad=1.25):
    """按模型的包围盒和'哪个轴朝上'自动取景"""
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    ctr = (lo + hi) / 2
    size = float(np.linalg.norm(hi - lo))
    return look_at_eye(ctr, size * pad, azim, elev), ctr, (lo, hi)
