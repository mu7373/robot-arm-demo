"""
plotlib.py —— 用 Pillow 手搓的一个极简绘图库（本机 Python 没有 matplotlib）。

只提供报告里真正用到的东西：坐标系、刻度、网格、散点、折线、文字、图例。
两个画布坐标系：
  px / py   —— 图像像素，左上角原点，y 向下
  data      —— 数据坐标，y 向上（Axes 内部负责翻转）
"""
from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

FONT_CANDIDATES = [
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\msyhl.ttc",
    r"C:\Windows\Fonts\simhei.ttf",
    r"C:\Windows\Fonts\simsun.ttc",
    r"C:\Windows\Fonts\arial.ttf",
]

_font_cache: dict[int, ImageFont.FreeTypeFont] = {}


def font(size: int) -> ImageFont.FreeTypeFont:
    if size in _font_cache:
        return _font_cache[size]
    for p in FONT_CANDIDATES:
        if Path(p).exists():
            try:
                f = ImageFont.truetype(p, size)
                _font_cache[size] = f
                return f
            except Exception:
                continue
    f = ImageFont.load_default()
    _font_cache[size] = f
    return f


def nice_ticks(lo: float, hi: float, target: int = 6) -> list[float]:
    """给出落在 [lo, hi] 内的"好看"刻度值。"""
    if hi <= lo:
        return [lo]
    raw = (hi - lo) / max(1, target)
    mag = 10 ** math.floor(math.log10(raw))
    for m in (1, 2, 2.5, 5, 10):
        step = m * mag
        if step >= raw:
            break
    start = math.ceil(lo / step) * step
    out = []
    v = start
    while v <= hi + step * 1e-9:
        out.append(round(v, 10))
        v += step
    return out


def fmt_tick(v: float) -> str:
    if abs(v) < 1e-12:
        return "0"
    if abs(v - round(v)) < 1e-9:
        return f"{int(round(v))}"
    return f"{v:g}"


class Axes:
    """一块数据绘图区。坐标范围由数据自动或手动给定。"""

    def __init__(self, img: Image.Image, draw: ImageDraw.ImageDraw,
                 box: tuple[int, int, int, int], xlim=None, ylim=None,
                 title: str = "", xlabel: str = "", ylabel: str = "",
                 equal: bool = False, pad: float = 0.06,
                 grid: bool = True, ticks: int = 6):
        self.img, self.d = img, draw
        self.x0, self.y0, self.x1, self.y1 = box   # 数据区外框（像素）
        self.title, self.xlabel, self.ylabel = title, xlabel, ylabel
        self.grid, self._nticks = grid, ticks
        self.equal = equal
        self._pad = pad
        self._auto_x: list[float] = []
        self._auto_y: list[float] = []
        self.xlim, self.ylim = xlim, ylim
        self._legend: list[tuple[str, tuple[int, int, int]]] = []

    # ---------- 数据范围 ----------
    def extend(self, xs, ys):
        self._auto_x.extend(xs)
        self._auto_y.extend(ys)

    def _resolve(self):
        if self.xlim is None:
            if not self._auto_x:
                self.xlim = (0.0, 1.0)
            else:
                lo, hi = min(self._auto_x), max(self._auto_x)
                sp = (hi - lo) or 1.0
                self.xlim = (lo - sp * self._pad, hi + sp * self._pad)
        if self.ylim is None:
            if not self._auto_y:
                self.ylim = (0.0, 1.0)
            else:
                lo, hi = min(self._auto_y), max(self._auto_y)
                sp = (hi - lo) or 1.0
                self.ylim = (lo - sp * self._pad, hi + sp * self._pad)
        if self.equal:
            # 让数据单位在 x/y 上等长：取较大的跨度，居中
            w = self.xlim[1] - self.xlim[0]
            h = self.ylim[1] - self.ylim[0]
            pw = self.x1 - self.x0
            ph = self.y1 - self.y0
            need = max(w / pw, h / ph)
            cw, ch = need * pw, need * ph
            cx = (self.xlim[0] + self.xlim[1]) / 2
            cy = (self.ylim[0] + self.ylim[1]) / 2
            self.xlim = (cx - cw / 2, cx + cw / 2)
            self.ylim = (cy - ch / 2, cy + ch / 2)

    def X(self, v: float) -> float:
        a, b = self.xlim
        return self.x0 + (v - a) / (b - a) * (self.x1 - self.x0)

    def Y(self, v: float) -> float:
        a, b = self.ylim
        return self.y1 - (v - a) / (b - a) * (self.y1 - self.y0)

    def P(self, x, y):
        return self.X(x), self.Y(y)

    # ---------- 绘图 ----------
    def frame(self):
        self._resolve()
        d = self.d
        # 网格 + 刻度
        xt, yt = nice_ticks(*self.xlim, self._nticks), nice_ticks(*self.ylim, self._nticks)
        if self.grid:
            for v in xt:
                px = self.X(v)
                d.line([(px, self.y0), (px, self.y1)], fill=(232, 235, 240), width=1)
            for v in yt:
                py = self.Y(v)
                d.line([(self.x0, py), (self.x1, py)], fill=(232, 235, 240), width=1)
        d.rectangle([self.x0, self.y0, self.x1, self.y1], outline=(120, 128, 140), width=1)
        f = font(13)
        for v in xt:
            px = self.X(v)
            d.line([(px, self.y1), (px, self.y1 + 5)], fill=(120, 128, 140), width=1)
            s = fmt_tick(v)
            w = d.textlength(s, font=f)
            d.text((px - w / 2, self.y1 + 8), s, font=f, fill=(70, 78, 92))
        for v in yt:
            py = self.Y(v)
            d.line([(self.x0 - 5, py), (self.x0, py)], fill=(120, 128, 140), width=1)
            s = fmt_tick(v)
            w = d.textlength(s, font=f)
            d.text((self.x0 - 9 - w, py - 8), s, font=f, fill=(70, 78, 92))
        if self.title:
            ft = font(17)
            w = d.textlength(self.title, font=ft)
            d.text(((self.x0 + self.x1) / 2 - w / 2, self.y0 - 34), self.title,
                   font=ft, fill=(32, 38, 50))
        if self.xlabel:
            fx = font(14)
            w = d.textlength(self.xlabel, font=fx)
            d.text(((self.x0 + self.x1) / 2 - w / 2, self.y1 + 30), self.xlabel,
                   font=fx, fill=(70, 78, 92))
        if self.ylabel:
            fy = font(14)
            # 先量出刻度文字最大宽度，避免纵轴标题压到刻度上
            widest = max([d.textlength(fmt_tick(v), font=font(13)) for v in yt] + [0])
            w = d.textlength(self.ylabel, font=fy)
            tmp = Image.new("RGBA", (int(w) + 4, 24), (0, 0, 0, 0))
            ImageDraw.Draw(tmp).text((2, 2), self.ylabel, font=fy, fill=(70, 78, 92))
            tmp = tmp.rotate(90, expand=True)
            self.img.paste(tmp, (int(self.x0 - 16 - widest - tmp.width),
                                 int((self.y0 + self.y1) / 2 - tmp.height / 2)), tmp)

    def scatter(self, xs, ys, r: int = 2, color=(52, 120, 220)):
        d, xl, yl = self.d, self.xlim, self.ylim
        for x, y in zip(xs, ys):
            if x < xl[0] or x > xl[1] or y < yl[0] or y > yl[1]:
                continue
            px, py = self.X(x), self.Y(y)
            d.ellipse([px - r, py - r, px + r, py + r], fill=color)

    def line(self, xs, ys, color=(210, 60, 50), width: int = 2, dash: bool = False):
        pts = list(zip(xs, ys))
        if len(pts) < 2:
            return
        pp = [self.P(*p) for p in pts]
        if not dash:
            self.d.line(pp, fill=color, width=width, joint="curve")
        else:
            for i in range(0, len(pp) - 1, 2):
                self.d.line([pp[i], pp[i + 1]], fill=color, width=width)

    def hline(self, y, color=(150, 150, 150), width: int = 1, dash=False):
        if dash:
            for i in range(self.x0, self.x1, 10):
                self.d.line([(i, self.Y(y)), (i + 5, self.Y(y))], fill=color, width=width)
        else:
            self.d.line([(self.x0, self.Y(y)), (self.x1, self.Y(y))], fill=color, width=width)

    def vline(self, x, color=(150, 150, 150), width: int = 1, dash=False):
        if dash:
            for i in range(self.y0, self.y1, 10):
                self.d.line([(self.X(x), i), (self.X(x), i + 5)], fill=color, width=width)
        else:
            self.d.line([(self.X(x), self.y0), (self.X(x), self.y1)], fill=color, width=width)

    def circle_data(self, cx, cy, r, color=(60, 60, 70), width: int = 1, dash=True):
        """在数据坐标里画圆（用折线近似，因为像素半径不可直接用）。"""
        n = 240
        xs = [cx + r * math.cos(2 * math.pi * i / n) for i in range(n + 1)]
        ys = [cy + r * math.sin(2 * math.pi * i / n) for i in range(n + 1)]
        self.line(xs, ys, color=color, width=width, dash=dash)

    def annotate(self, x, y, text, color=(30, 36, 46), size: int = 13,
                 dx: int = 8, dy: int = -8, arrow: bool = False):
        px, py = self.P(x, y)
        f = font(size)
        if arrow:
            self.d.line([(px, py), (px + dx, py + dy)], fill=color, width=1)
        self.d.text((px + dx, py + dy - size / 2 - 2), text, font=f, fill=color)

    def legend(self, items, corner: str = "tr"):
        f = font(13)
        w = max(int(self.d.textlength(t, font=f)) for t, _ in items) + 34
        h = 8 + len(items) * 20
        if corner == "tr":
            bx, by = self.x1 - w - 8, self.y0 + 8
        elif corner == "tl":
            bx, by = self.x0 + 8, self.y0 + 8
        else:
            bx, by = self.x1 - w - 8, self.y1 - h - 8
        self.d.rounded_rectangle([bx, by, bx + w, by + h], radius=5,
                                 fill=(255, 255, 255), outline=(200, 206, 216))
        for i, (t, c) in enumerate(items):
            cy = by + 8 + i * 20
            self.d.line([(bx + 8, cy + 7), (bx + 26, cy + 7)], fill=c, width=3)
            self.d.text((bx + 30, cy), t, font=f, fill=(40, 46, 58))


def new_canvas(w: int, h: int, bg=(255, 255, 255)):
    img = Image.new("RGB", (w, h), bg)
    return img, ImageDraw.Draw(img)


def caption(draw, x, y, text, size=14, color=(60, 66, 78)):
    draw.text((x, y), text, font=font(size), fill=color)
