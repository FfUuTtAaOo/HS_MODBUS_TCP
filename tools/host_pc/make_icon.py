"""生成上位机图标 hs_host.ico（纯标准库，无需 Pillow）。

图形含义：深色圆角底 + 六维力传感器圆形弹性体 + 六条辐射力/力矩轴。
采用 4x 超采样再降采样，得到平滑边缘。
"""

from __future__ import annotations

import math
import struct

SIZE = 64                 # 输出尺寸
SS = 4                    # 超采样倍数
W = SIZE * SS             # 超采样画布边长

BG = (0x10, 0x31, 0x4F)   # 深海军蓝底
DISC = (0xF7, 0xFA, 0xFC)  # 弹性体白盘
AXIS = (0xE2, 0x45, 0x3C)  # 力/力矩轴 红


def _rounded_rect(x: float, y: float, cx: float, cy: float,
                  hw: float, hh: float, r: float) -> bool:
    dx, dy = abs(x - cx), abs(y - cy)
    if dx > hw or dy > hh:
        return False
    if dx <= hw - r or dy <= hh - r:
        return True
    ex, ey = dx - (hw - r), dy - (hh - r)
    return ex * ex + ey * ey <= r * r


def _disc(x: float, y: float, cx: float, cy: float, r: float) -> bool:
    dx, dy = x - cx, y - cy
    return dx * dx + dy * dy <= r * r


def _seg_dist(px_: float, py_: float, x1: float, y1: float,
              x2: float, y2: float) -> float:
    vx, vy = x2 - x1, y2 - y1
    wx, wy = px_ - x1, py_ - y1
    seg = vx * vx + vy * vy
    t = 0.0 if seg == 0 else max(0.0, min(1.0, (wx * vx + wy * vy) / seg))
    dx, dy = px_ - (x1 + t * vx), py_ - (y1 + t * vy)
    return (dx * dx + dy * dy) ** 0.5


def _in_tri(px_: float, py_: float, a, b, c) -> bool:
    def sign(p, q, r):
        return (p[0] - r[0]) * (q[1] - r[1]) - (q[0] - r[0]) * (p[1] - r[1])
    d1, d2, d3 = sign((px_, py_), a, b), sign((px_, py_), b, c), sign((px_, py_), c, a)
    neg = d1 < 0 or d2 < 0 or d3 < 0
    pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (neg and pos)


def render_rgba() -> bytearray:
    """渲染 W x W 的 RGBA 画布（自上而下）。"""
    c = W / 2.0
    canvas = bytearray(W * W * 4)

    # 几何参数（按超采样尺度换算）
    k = SS
    pad = 3 * k
    body_r = 25.5 * k          # 弹性体半径
    hole_r = 8.0 * k           # 中心轮毂
    shaft_from = 10.5 * k      # 力的作用半径起点
    shaft_to = 19.0 * k        # 轴身末端
    shaft_w = 2.6 * k          # 轴身半宽
    head_len = 4.5 * k         # 箭头长
    head_hw = 4.6 * k          # 箭头半宽

    # 六轴：0/60/120/180/240/300 度
    axes = []
    for i in range(6):
        ang = math.radians(i * 60.0 + 90.0)
        ca, sa = math.cos(ang), math.sin(ang)
        p0 = (c + ca * shaft_from, c - sa * shaft_from)
        p1 = (c + ca * shaft_to, c - sa * shaft_to)
        tip = (c + ca * (shaft_to + head_len), c - sa * (shaft_to + head_len))
        # 箭头基部两点
        nx, ny = -sa, -ca
        b1 = (p1[0] + nx * head_hw, p1[1] + ny * head_hw)
        b2 = (p1[0] - nx * head_hw, p1[1] - ny * head_hw)
        axes.append((p0, p1, tip, b1, b2))

    for y in range(W):
        fy = y + 0.5
        row = y * W * 4
        for x in range(W):
            fx = x + 0.5
            rgb = None
            if _rounded_rect(fx, fy, c, c, c - pad, c - pad, 12.0 * k):
                rgb = BG
                if _disc(fx, fy, c, c, body_r):
                    rgb = DISC
                    if hole_r > 0 and _disc(fx, fy, c, c, hole_r):
                        rgb = (0xDC, 0xE6, 0xF0)
                    for p0, p1, tip, b1, b2 in axes:
                        if _seg_dist(fx, fy, p0[0], p0[1], p1[0], p1[1]) <= shaft_w:
                            rgb = AXIS
                            break
                        if _in_tri(fx, fy, tip, b1, b2):
                            rgb = AXIS
                            break
            if rgb is None:
                continue
            i = row + x * 4
            canvas[i] = rgb[0]
            canvas[i + 1] = rgb[1]
            canvas[i + 2] = rgb[2]
            canvas[i + 3] = 0xFF
    return canvas


def downsample(canvas: bytearray) -> bytearray:
    """区域平均降采样到 SIZE x SIZE（含 alpha 预乘还原）。"""
    out = bytearray(SIZE * SIZE * 4)
    n = SS * SS
    for y in range(SIZE):
        for x in range(SIZE):
            r = g = b = a = 0
            for dy in range(SS):
                base = ((y * SS + dy) * W + x * SS) * 4
                for dx in range(SS):
                    i = base + dx * 4
                    al = canvas[i + 3]
                    r += canvas[i] * al
                    g += canvas[i + 1] * al
                    b += canvas[i + 2] * al
                    a += al
            o = (y * SIZE + x) * 4
            if a:
                out[o] = min(255, int(r / a + 0.5))
                out[o + 1] = min(255, int(g / a + 0.5))
                out[o + 2] = min(255, int(b / a + 0.5))
            out[o + 3] = min(255, int(a / n + 0.5))
    return out


def to_ico(rgba: bytearray, path: str) -> None:
    """把 RGBA（自上而下）写成 32bpp BMP 型 ICO。"""
    # 像素数据：BMP 为自下而上，通道序 BGRA
    pixels = bytearray(SIZE * SIZE * 4)
    for y in range(SIZE):
        src = y * SIZE * 4
        dst = (SIZE - 1 - y) * SIZE * 4
        for x in range(SIZE):
            s = src + x * 4
            d = dst + x * 4
            pixels[d] = rgba[s + 2]        # B
            pixels[d + 1] = rgba[s + 1]    # G
            pixels[d + 2] = rgba[s]        # R
            pixels[d + 3] = rgba[s + 3]    # A

    # AND 掩码：32bpp 下由 alpha 决定透明，填 0 即可；每行按 4 字节对齐
    mask_row = ((SIZE + 31) // 32) * 4
    and_mask = bytes(mask_row * SIZE)

    header = struct.pack("<IiiHHIIiiII", 40, SIZE, SIZE * 2, 1, 32, 0,
                         len(pixels), 0, 0, 0, 0)
    image = header + bytes(pixels) + and_mask
    icondir = struct.pack("<HHH", 0, 1, 1)
    entry = struct.pack("<BBBBHHII", SIZE % 256, SIZE % 256, 0, 0, 1, 32,
                        len(image), 22)
    with open(path, "wb") as f:
        f.write(icondir + entry + image)


def main() -> None:
    rgba = downsample(render_rgba())
    to_ico(rgba, "hs_host.ico")
    import os
    print("已生成 hs_host.ico:", os.path.getsize("hs_host.ico"), "字节")


if __name__ == "__main__":
    main()
