# -*- coding: utf-8 -*-
"""
hs_app.py —— 六维力传感器 Modbus-TCP 上位机（tkinter，纯标准库）

启动：
    python hs_app.py
    python hs_app.py --host 192.168.1.12 --port 502 --order CDAB

无硬件时可先启动模拟器：
    python hs_simulator.py --port 502
"""

from __future__ import annotations

import argparse
import csv
import os
import queue
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from collections import deque
from tkinter import filedialog, messagebox, ttk
from typing import Callable, List, Optional

from hs_modbus import AXIS_NAMES, ByteOrder, Format, FreqMode, SensorClient
from hs_xlsx import XlsxLimitError, XlsxTableWriter

# ---------------------------------------------------------------------
#  外观常量
# ---------------------------------------------------------------------
FONT_FAMILY = "Microsoft YaHei UI"
COLORS = ("#d94a4a", "#2f9e5f", "#2f6fd0", "#d98a1f", "#8b5cf6", "#0f9b9b")
BG = "#f5f6f8"
CARD = "#ffffff"
EDGE = "#d8dce2"
TXT = "#22262b"
TXT_DIM = "#707780"
GRID = "#e8ebef"
AXIS = "#c6cbd2"

# 「保存数据」写出的 Excel 表头：就是六维力的六个通道，顺序固定
SAVE_HEADERS = ("FX", "FY", "FZ", "TX", "TY", "TZ")


def app_dir() -> str:
    """
    程序所在目录。

    打包成单文件 exe 后 `sys.executable` 就是 exe 自身，
    所以数据文件默认落在 exe 同目录；源码运行时落在脚本目录。
    """
    if getattr(sys, "frozen", False):
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))

FMT_LABEL = {Format.MV: "原始数据 (mV)", Format.KG: "矩阵滤波 (kg)",
             Format.N: "矩阵滤波 (N)"}


# =====================================================================
#  波形绘制组件
# =====================================================================
class WaveCanvas(tk.Canvas):
    """多通道实时波形（tkinter Canvas 自绘，自动量程 + 滚动时间轴）。"""

    def __init__(self, master, window_sec: float = 5.0, **kw):
        super().__init__(master, bg=CARD, highlightthickness=1,
                         highlightbackground=EDGE, **kw)
        self.window_sec = window_sec
        self.enabled = [True] * 6
        self.units = ["N"] * 6
        self._last_draw = 0.0

    def set_units(self, units: List[str]) -> None:
        self.units = list(units)

    def draw(self, samples: List, force: bool = False) -> None:
        now = time.time()
        if not force and now - self._last_draw < 0.03:
            return
        self._last_draw = now

        w = self.winfo_width()
        h = self.winfo_height()
        if w <= 1:
            w = max(self.winfo_reqwidth(), 400)
        if h <= 1:
            h = max(self.winfo_reqheight(), 200)
        self.delete("all")

        pad_l, pad_r, pad_t, pad_b = 62, 12, 14, 26
        pw = w - pad_l - pad_r
        ph = h - pad_t - pad_b
        if pw < 20 or ph < 20:
            return

        # ---- 网格 ----
        for i in range(6):
            x = pad_l + pw * i / 5
            self.create_line(x, pad_t, x, pad_t + ph, fill=GRID)
        for i in range(5):
            y = pad_t + ph * i / 4
            self.create_line(pad_l, y, pad_l + pw, y, fill=GRID)
        self.create_line(pad_l, pad_t, pad_l, pad_t + ph, fill=AXIS)
        self.create_line(pad_l, pad_t + ph, pad_l + pw, pad_t + ph, fill=AXIS)

        active = [i for i in range(6) if self.enabled[i]]
        if not samples or not active:
            self.create_text(w / 2, h / 2, fill=TXT_DIM,
                             font=(FONT_FAMILY, 11),
                             text="暂无数据 —— 点击「开始转换」进入连续采集"
                             if not samples else "未勾选任何通道")
            return

        t_end = samples[-1][0]
        t_start = t_end - self.window_sec

        # ---- 自动量程 ----
        lo, hi = float("inf"), float("-inf")
        for _, vals in samples:
            for i in active:
                v = vals[i]
                if v < lo:
                    lo = v
                if v > hi:
                    hi = v
        if lo == float("inf"):
            lo, hi = -1.0, 1.0
        if hi - lo < 1e-9:
            span = max(abs(hi) * 0.2, 1.0)
            lo, hi = lo - span, hi + span
        margin = (hi - lo) * 0.12
        lo -= margin
        hi += margin

        def sy(v: float) -> float:
            return pad_t + ph * (1.0 - (v - lo) / (hi - lo))

        def sx(t: float) -> float:
            return pad_l + pw * (t - t_start) / self.window_sec

        # ---- Y 轴刻度 ----
        for i in range(5):
            v = hi - (hi - lo) * i / 4
            y = pad_t + ph * i / 4
            self.create_text(pad_l - 6, y, anchor="e", fill=TXT_DIM,
                             font=(FONT_FAMILY, 8), text=_fmt_tick(v))

        # ---- X 轴刻度 ----
        for i in range(6):
            rel = -self.window_sec + self.window_sec * i / 5
            x = pad_l + pw * i / 5
            self.create_text(x, pad_t + ph + 12, fill=TXT_DIM,
                             font=(FONT_FAMILY, 8), text=f"{rel:+.1f}s")

        # ---- 曲线（按像素宽度抽稀，避免点数过多）----
        max_points = max(min(int(pw * 2), 800), 200)
        step = max(1, len(samples) // max_points)
        picked = samples[::step]
        if (len(picked) - 1) * step != len(samples) - 1:
            picked.append(samples[-1])

        for i in active:
            coords = []
            for t, vals in picked:
                coords.append(sx(t))
                coords.append(sy(vals[i]))
            if len(coords) >= 4:
                self.create_line(*coords, fill=COLORS[i], width=2,
                                 capstyle="round", joinstyle="round")

        # ---- 图例 ----
        x = pad_l + 8
        for i in active:
            self.create_rectangle(x, pad_t + 6, x + 10, pad_t + 16,
                                  fill=COLORS[i], outline="")
            self.create_text(x + 15, pad_t + 11, anchor="w", fill=TXT,
                             font=(FONT_FAMILY, 8),
                             text=f"{AXIS_NAMES[i]} ({self.units[i]})")
            x += 74


def _fmt_tick(v: float) -> str:
    a = abs(v)
    if a >= 10000 or (0 < a < 0.01):
        return f"{v:.2e}"
    if a >= 100:
        return f"{v:.0f}"
    if a >= 1:
        return f"{v:.2f}"
    return f"{v:.3f}"


# =====================================================================
#  主窗口
# =====================================================================
class MainWindow:

    def __init__(self, root: tk.Tk, host: str, port: int, order: str,
                 write_order: Optional[str] = None,
                 save_dir: Optional[str] = None):
        self.root = root
        # 三条路径的字节序都按固件的真实情况分开设置：
        #   轮询 FC03 = CDAB（低字在前）／写入 FC10 = DCBA（小端）／主动推流 = DCBA
        self.cli = SensorClient(host, port,
                                byte_order=order,
                                push_byte_order=write_order or order,
                                write_byte_order=write_order or order)
        self.cli.on_log = self._on_log_thread
        self.cli.on_state = self._on_state_thread
        self.cli.on_error = self._on_error_thread
        # 主动推流帧的回调：保存数据时用它逐帧攒数据（接收线程里执行）
        self.cli.on_push = self._on_push_thread

        self.ui_queue: "queue.Queue" = queue.Queue()
        self._async_fn: Optional[Callable] = None
        self._async_ok: Optional[Callable] = None
        self._busy = False
        self._connected = False

        self.recording = False
        self._csv_file = None
        self._csv_writer = None
        self._csv_rows = 0
        self._csv_last_ts = 0.0

        # ---- 六维力数据保存（Excel）----
        self.save_dir = save_dir or app_dir()
        self.saving = False
        self._xlsx: Optional[XlsxTableWriter] = None
        self._save_path = ""
        self._save_rows = 0
        self._save_skipped = 0
        self._save_t0 = 0.0
        self._save_pending: deque = deque()
        self._save_lock = threading.Lock()

        self._last_drawn_count = 0
        self._rate_count = 0
        self._rate_t0 = time.time()
        self.sample_rate = 0.0
        self.overload_thresholds: List[float] = [1e30] * 6

        root.title("六维力传感器上位机 — Modbus-TCP")
        root.configure(bg=BG)
        self._setup_fonts()
        self._build_ui(host, port, order)
        self._set_connected_ui(False)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        root.after(40, self._tick)

    # ------------------------------------------------------------------
    #  字体与样式
    # ------------------------------------------------------------------
    def _setup_fonts(self) -> None:
        fams = set(tkfont.families())
        fam = FONT_FAMILY if FONT_FAMILY in fams else (
            "微软雅黑" if "微软雅黑" in fams else "SimHei")
        self.f_family = fam
        default = tkfont.nametofont("TkDefaultFont")
        default.configure(family=fam, size=9)
        tkfont.nametofont("TkTextFont").configure(family=fam, size=9)
        tkfont.nametofont("TkFixedFont").configure(
            family="Consolas" if "Consolas" in fams else "Courier New", size=9)

    # ------------------------------------------------------------------
    #  界面构建
    # ------------------------------------------------------------------
    def _build_ui(self, host: str, port: int, order: str) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(2, weight=1)

        self._build_toolbar(host, port, order)

        mid = ttk.Frame(self.root, padding=(8, 6, 8, 0))
        mid.grid(row=1, column=0, sticky="nsew")
        mid.columnconfigure(1, weight=1)
        mid.rowconfigure(0, weight=1)
        self._build_control_panel(mid)
        self._build_display(mid)

        self._build_notebook()

        self._build_log()

    # ---------------- 顶部连接栏 ----------------
    def _build_toolbar(self, host: str, port: int, order: str) -> None:
        bar = ttk.Frame(self.root, padding=(10, 8, 10, 4))
        bar.grid(row=0, column=0, sticky="ew")

        # ---- 第一行：连接 ----
        row1 = ttk.Frame(bar)
        row1.pack(fill="x")
        ttk.Label(row1, text="设备地址").pack(side="left")
        self.var_host = tk.StringVar(value=host)
        ttk.Entry(row1, textvariable=self.var_host, width=16).pack(side="left", padx=(4, 10))

        ttk.Label(row1, text="端口").pack(side="left")
        self.var_port = tk.StringVar(value=str(port))
        ttk.Entry(row1, textvariable=self.var_port, width=6).pack(side="left", padx=(4, 10))

        self.btn_connect = ttk.Button(row1, text="连接", command=self.on_connect, width=8)
        self.btn_connect.pack(side="left", padx=4)
        self.btn_disconnect = ttk.Button(row1, text="断开", command=self.on_disconnect,
                                         width=8, state="disabled")
        self.btn_disconnect.pack(side="left", padx=4)

        self.lamp = tk.Canvas(row1, width=14, height=14, bg=BG, highlightthickness=0)
        self.lamp.pack(side="right", padx=(6, 2))
        self._lamp_item = self.lamp.create_oval(2, 2, 12, 12, fill="#c0392b", outline="")
        self.var_conn = tk.StringVar(value="未连接")
        ttk.Label(row1, textvariable=self.var_conn, width=30).pack(side="right")

        # ---- 第二行：字节序（三条路径口径不同，分开设置）----
        row2 = ttk.Frame(bar)
        row2.pack(fill="x", pady=(5, 0))
        ttk.Label(row2, text="轮询 FC03 字节序").pack(side="left")
        self.var_order = tk.StringVar(value=order)
        cb = ttk.Combobox(row2, textvariable=self.var_order, width=7,
                          state="readonly", values=list(ByteOrder.ALL))
        cb.pack(side="left", padx=(4, 10))
        cb.bind("<<ComboboxSelected>>", self._on_order_changed)

        ttk.Label(row2, text="单次 / 写入 FC10 / 推流 字节序").pack(side="left")
        self.var_worder = tk.StringVar(value=ByteOrder.WRITE_DEFAULT)
        cb2 = ttk.Combobox(row2, textvariable=self.var_worder, width=7,
                           state="readonly", values=list(ByteOrder.ALL))
        cb2.pack(side="left", padx=(4, 10))
        cb2.bind("<<ComboboxSelected>>", self._on_order_changed)

        ttk.Button(row2, text="读取字节序自检", command=self.on_probe_order,
                   width=14).pack(side="left", padx=(0, 4))
        ttk.Button(row2, text="写入字节序自检", command=self.on_verify_write_order,
                   width=14).pack(side="left", padx=4)

        ttk.Label(
            row2, foreground="#7a5230",
            text=f"真机默认 轮询={ByteOrder.POLL_DEFAULT} / 单次·写入·推流="
                 f"{ByteOrder.WRITE_DEFAULT}（文档示例是 ABCD，与固件不一致）"
        ).pack(side="left", padx=10)

    # ---------------- 左侧控制面板 ----------------
    def _build_control_panel(self, parent: ttk.Frame) -> None:
        # 面板分组较多，窗口不够高时靠滚动条看全。
        # 滚轮只绑在本面板的控件上（Tk 事件不会从子控件冒到父控件），
        # 所以不会连带把右下角的日志一起滚走。
        outer = ttk.Frame(parent)
        outer.grid(row=0, column=0, sticky="nsw", padx=(0, 8))
        outer.rowconfigure(0, weight=1)

        self.panel_canvas = tk.Canvas(outer, bg=BG, highlightthickness=0, bd=0,
                                      yscrollincrement=20)
        self.panel_sb = ttk.Scrollbar(outer, orient="vertical",
                                      command=self.panel_canvas.yview)
        self.panel_canvas.configure(yscrollcommand=self.panel_sb.set)
        self.panel_canvas.grid(row=0, column=0, sticky="nsew")
        self.panel_sb.grid(row=0, column=1, sticky="ns")
        self.panel_sb.grid_remove()          # 内容没超高就不占位置
        self._panel_sb_on = False

        box = ttk.Frame(self.panel_canvas)
        self._panel_box = box
        self._panel_win = self.panel_canvas.create_window((0, 0), window=box,
                                                          anchor="nw")
        box.bind("<Configure>", self._on_panel_content)
        self.panel_canvas.bind("<Configure>", self._on_panel_canvas)
        col = 0

        # ---- 数据流控制 ----
        g = ttk.LabelFrame(box, text="数据流控制", padding=8)
        g.grid(row=col, column=0, sticky="ew", pady=(0, 6)); col += 1
        for txt, cb in (("开始连续转换", self.on_start),
                        ("停止发送", self.on_stop),
                        ("单次触发", self.on_single)):
            ttk.Button(g, text=txt, command=cb, width=18).pack(fill="x", pady=2)

        self.var_single = tk.StringVar(value="最近单次结果：—")
        ttk.Label(g, textvariable=self.var_single, wraplength=180,
                  foreground=TXT_DIM).pack(fill="x", pady=(4, 0))

        # ---- 输出单位 ----
        g = ttk.LabelFrame(box, text="输出单位 (0x0032)", padding=8)
        g.grid(row=col, column=0, sticky="ew", pady=(0, 6)); col += 1
        self.var_fmt = tk.IntVar(value=Format.N)
        for txt, val in (("原始数据 mV", Format.MV),
                         ("矩阵滤波 kg", Format.KG),
                         ("矩阵滤波 N", Format.N)):
            ttk.Radiobutton(g, text=txt, value=val, variable=self.var_fmt,
                            command=self.on_set_format).pack(anchor="w")

        # ---- 清零 ----
        g = ttk.LabelFrame(box, text="零点管理", padding=8)
        g.grid(row=col, column=0, sticky="ew", pady=(0, 6)); col += 1
        ttk.Button(g, text="自动清零（去皮）", command=self.on_zero,
                   width=18).pack(fill="x", pady=2)
        ttk.Button(g, text="取消清零", command=self.on_unzero,
                   width=18).pack(fill="x", pady=2)

        # ---- 采样频率（扩展寄存器）----
        g = ttk.LabelFrame(box, text="采样频率 (0x0200 扩展)", padding=8)
        g.grid(row=col, column=0, sticky="ew", pady=(0, 6)); col += 1
        self.var_freq = tk.IntVar(value=FreqMode.F500)
        for txt, val in (("500 Hz", FreqMode.F500), ("1000 Hz", FreqMode.F1000)):
            ttk.Radiobutton(g, text=txt, value=val, variable=self.var_freq,
                            command=self.on_set_freq).pack(anchor="w")

        # ---- 显示设置 ----
        g = ttk.LabelFrame(box, text="显示设置", padding=8)
        g.grid(row=col, column=0, sticky="ew", pady=(0, 6)); col += 1
        row = ttk.Frame(g); row.pack(fill="x")
        ttk.Label(row, text="时间窗").pack(side="left")
        self.var_window = tk.StringVar(value="5")
        ttk.Combobox(row, textvariable=self.var_window, width=5, state="readonly",
                     values=("1", "2", "5", "10", "20")).pack(side="left", padx=4)
        ttk.Label(row, text="秒").pack(side="left")

        self.var_ch = [tk.BooleanVar(value=True) for _ in range(6)]
        grid = ttk.Frame(g); grid.pack(fill="x", pady=(4, 0))
        for i, name in enumerate(AXIS_NAMES):
            cb = tk.Checkbutton(grid, text=name, variable=self.var_ch[i],
                                fg=COLORS[i], activeforeground=COLORS[i],
                                command=self._on_channel_toggle, bg=BG,
                                selectcolor=CARD, highlightthickness=0)
            cb.grid(row=i // 3, column=i % 3, sticky="w")

        # ---- 「开始保存数据 / 记录 CSV」不在这里 ----
        # 这两个按钮已挪到右下角日志栏，紧挨「保存日志」，一进界面就能看到，
        # 不再埋在左侧面板的最底部。

        # 面板宽度/高度贴合内容：内容高度撑起整块 mid 区域，
        # 窗口被压矮时 canvas 矮于内容，滚动条才会出现。
        box.update_idletasks()
        self.panel_canvas.configure(height=box.winfo_reqheight(),
                                    width=box.winfo_reqwidth())
        self._bind_panel_wheel(box)

    # ---------------- 左侧面板滚动 ----------------
    def _on_panel_content(self, event=None) -> None:
        self.panel_canvas.configure(scrollregion=self.panel_canvas.bbox("all"))
        self._sync_panel_sb()

    def _on_panel_canvas(self, event) -> None:
        self.panel_canvas.itemconfigure(self._panel_win, width=event.width)
        self._sync_panel_sb()

    def _sync_panel_sb(self) -> None:
        """内容比可视区高时才把滚动条摆出来，平时不占位置。"""
        bbox = self.panel_canvas.bbox("all")
        h = self.panel_canvas.winfo_height()
        need = bool(bbox) and h > 1 and (bbox[3] - bbox[1]) > h
        if need == self._panel_sb_on:
            return
        self._panel_sb_on = need
        if need:
            self.panel_sb.grid(row=0, column=1, sticky="ns")
        else:
            self.panel_sb.grid_remove()

    def _bind_panel_wheel(self, w) -> None:
        w.bind("<MouseWheel>", self._on_panel_wheel)
        for ch in w.winfo_children():
            self._bind_panel_wheel(ch)

    def _on_panel_wheel(self, event) -> str:
        if self._panel_sb_on:
            self.panel_canvas.yview_scroll(-int(event.delta / 120), "units")
        return "break"

    # ---------------- 右侧数值 + 波形 ----------------
    def _build_display(self, parent: ttk.Frame) -> None:
        right = ttk.Frame(parent)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        right.rowconfigure(1, weight=1)

        # 数值面板
        card = tk.Frame(right, bg=CARD, highlightthickness=1,
                        highlightbackground=EDGE)
        card.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        for c in range(6):
            card.columnconfigure(c, weight=1)

        self.val_labels: List[tk.Label] = []
        self.unit_labels: List[tk.Label] = []
        for i, name in enumerate(AXIS_NAMES):
            cell = tk.Frame(card, bg=CARD, padx=8, pady=6)
            cell.grid(row=0, column=i, sticky="nsew")
            tk.Label(cell, text=name, bg=CARD, fg=COLORS[i],
                     font=(self.f_family, 11, "bold")).pack(anchor="w")
            v = tk.Label(cell, text="—", bg=CARD, fg=TXT,
                         font=(self.f_family, 17, "bold"))
            v.pack(anchor="w")
            u = tk.Label(cell, text="", bg=CARD, fg=TXT_DIM,
                         font=(self.f_family, 8))
            u.pack(anchor="w")
            self.val_labels.append(v)
            self.unit_labels.append(u)

        # 波形
        self.wave = WaveCanvas(right, window_sec=5.0, width=900, height=300)
        self.wave.grid(row=1, column=0, sticky="nsew")

    # ---------------- 参数配置 Notebook ----------------
    def _build_notebook(self) -> None:
        wrap = ttk.Frame(self.root, padding=(8, 6, 8, 0))
        wrap.grid(row=2, column=0, sticky="nsew")
        wrap.columnconfigure(0, weight=1)
        wrap.rowconfigure(0, weight=1)

        nb = ttk.Notebook(wrap)
        nb.grid(row=0, column=0, sticky="nsew")

        self._build_info_tab(nb)
        self._build_zero_tab(nb)
        self._build_threshold_tab(nb)
        self._build_network_tab(nb)
        self._build_selftest_tab(nb)

        # 让中部随窗口伸缩
        self.root.rowconfigure(1, weight=3)
        self.root.rowconfigure(2, weight=2)

    def _scroll_tab(self, nb: ttk.Notebook, title: str) -> ttk.Frame:
        """创建带滚动条的标签页，返回内容容器。"""
        outer = ttk.Frame(nb)
        nb.add(outer, text=title)
        canvas = tk.Canvas(outer, bg=BG, highlightthickness=0)
        sb = ttk.Scrollbar(outer, orient="vertical", command=canvas.yview)
        inner = ttk.Frame(canvas, padding=8)
        canvas.create_window((0, 0), window=inner, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        return inner

    # ---- 设备信息 ----
    def _build_info_tab(self, nb: ttk.Notebook) -> None:
        f = self._scroll_tab(nb, "设备信息")
        f.columnconfigure(3, weight=1)

        self.var_sn = tk.StringVar(value="—")
        self.var_fw = tk.StringVar(value="—")
        self.var_status = tk.StringVar(value="—")
        self.var_comm_err = tk.StringVar(value="—")
        self.var_status_txt = tk.StringVar(value="")

        rows = [("序列号 (0x0005)", self.var_sn),
                ("固件版本 (0x000D)", self.var_fw),
                ("系统状态 (0x000F)", self.var_status),
                ("通信错误计数 (0x0010)", self.var_comm_err)]
        for i, (name, var) in enumerate(rows):
            ttk.Label(f, text=name, width=22).grid(row=i, column=0, sticky="w", pady=3)
            ttk.Label(f, textvariable=var, foreground=TXT).grid(
                row=i, column=1, sticky="w", padx=6)

        ttk.Label(f, textvariable=self.var_status_txt, foreground="#c0392b",
                  wraplength=700).grid(row=len(rows), column=0, columnspan=4,
                                       sticky="w", pady=(4, 0))
        ttk.Button(f, text="读取设备信息", command=self.on_read_info).grid(
            row=0, column=2, rowspan=2, padx=12, sticky="w")

    # ---- 零点偏移 ----
    def _build_zero_tab(self, nb: ttk.Notebook) -> None:
        f = self._scroll_tab(nb, "零点偏移 (0x0050)")
        ttk.Label(f, foreground=TXT_DIM,
                  text="单位：力 N、力矩 N·m。输出值 = 原始值 − 零点偏移。").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))

        self.var_zero = [tk.StringVar(value="0") for _ in range(6)]
        for i, name in enumerate(AXIS_NAMES):
            ttk.Label(f, text=name, width=5).grid(row=1 + i // 3, column=(i % 3) * 2,
                                                  sticky="e", pady=3)
            ttk.Entry(f, textvariable=self.var_zero[i], width=16).grid(
                row=1 + i // 3, column=(i % 3) * 2 + 1, sticky="w", padx=(4, 16))

        bar = ttk.Frame(f)
        bar.grid(row=3, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Button(bar, text="读取", command=self.on_read_zero, width=12).pack(side="left")
        ttk.Button(bar, text="写入", command=self.on_write_zero, width=12).pack(
            side="left", padx=8)

    # ---- 过载阈值 ----
    def _build_threshold_tab(self, nb: ttk.Notebook) -> None:
        f = self._scroll_tab(nb, "过载阈值 (0x0070)")
        ttk.Label(f, foreground=TXT_DIM,
                  text="任一通道绝对值 ≥ 阈值时，系统状态字 bit3 (OVERLOAD) 置 1；"
                       "写入值须落在下方范围内，否则被拒绝并置 bit4 (PARAM_ERR)。").grid(
            row=0, column=0, columnspan=6, sticky="w", pady=(0, 6))

        self.var_ovl = [tk.StringVar(value="0") for _ in range(6)]
        self.var_rng = [tk.StringVar(value="—") for _ in range(6)]
        for i, name in enumerate(AXIS_NAMES):
            ttk.Label(f, text=name, width=5).grid(row=1 + i // 3, column=(i % 3) * 2,
                                                  sticky="e", pady=3)
            ttk.Entry(f, textvariable=self.var_ovl[i], width=14).grid(
                row=1 + i // 3, column=(i % 3) * 2 + 1, sticky="w", padx=(4, 16))

        ttk.Label(f, text="允许范围：", foreground=TXT_DIM).grid(
            row=3, column=0, sticky="w", pady=(8, 0))
        for i in range(6):
            ttk.Label(f, textvariable=self.var_rng[i], foreground=TXT_DIM).grid(
                row=4 + i // 3, column=(i % 3) * 2, columnspan=2, sticky="w")

        bar = ttk.Frame(f)
        bar.grid(row=6, column=0, columnspan=6, sticky="w", pady=(8, 0))
        ttk.Button(bar, text="读取阈值", command=self.on_read_ovl, width=12).pack(side="left")
        ttk.Button(bar, text="写入阈值", command=self.on_write_ovl, width=12).pack(
            side="left", padx=8)

    # ---- 网络参数 ----
    def _build_network_tab(self, nb: ttk.Notebook) -> None:
        f = self._scroll_tab(nb, "网络参数 (0x0100)")
        ttk.Label(f, foreground="#c0392b",
                  text="网络参数写入后需重启传感器才生效，请确认新地址可达。").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))

        self.var_mac = tk.StringVar()
        self.var_ip = tk.StringVar()
        self.var_mask = tk.StringVar()
        self.var_gw = tk.StringVar()
        for i, (name, var) in enumerate((("MAC 地址 (0x0100)", self.var_mac),
                                         ("IP 地址 (0x0106)", self.var_ip),
                                         ("子网掩码 (0x0108)", self.var_mask),
                                         ("默认网关 (0x010A)", self.var_gw))):
            ttk.Label(f, text=name, width=22).grid(row=1 + i, column=0,
                                                   sticky="w", pady=3)
            ttk.Entry(f, textvariable=var, width=22).grid(row=1 + i, column=1,
                                                          sticky="w", padx=6)

        bar = ttk.Frame(f)
        bar.grid(row=5, column=0, columnspan=3, sticky="w", pady=(8, 0))
        ttk.Button(bar, text="读取", command=self.on_read_net, width=12).pack(side="left")
        ttk.Button(bar, text="写入", command=self.on_write_net, width=12).pack(
            side="left", padx=8)

    # ---- 自检（扩展寄存器）----
    def _build_selftest_tab(self, nb: ttk.Notebook) -> None:
        f = self._scroll_tab(nb, "自检 (扩展 0x0210)")
        ttk.Label(f, foreground=TXT_DIM,
                  text="以下为固件内部扩展寄存器，不在对外协议文档中，用于生产自检。").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 6))

        self.var_self_test = tk.StringVar(value="—")
        ttk.Label(f, textvariable=self.var_self_test, justify="left",
                  font=("Consolas", 9)).grid(row=1, column=0, columnspan=3,
                                             sticky="w")
        ttk.Button(f, text="读取自检寄存器", command=self.on_read_selftest).grid(
            row=2, column=0, sticky="w", pady=8)

    # ---------------- 日志 ----------------
    def _build_log(self) -> None:
        wrap = ttk.Frame(self.root, padding=(8, 6, 8, 8))
        wrap.grid(row=3, column=0, sticky="ew")
        wrap.columnconfigure(1, weight=1)

        top = ttk.Frame(wrap)
        top.grid(row=0, column=0, columnspan=2, sticky="ew")
        self.var_log_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text="记录通信报文（十六进制）",
                        variable=self.var_log_on).pack(side="left")
        ttk.Button(top, text="清空日志", command=self.clear_log).pack(side="left", padx=8)
        ttk.Button(top, text="保存日志", command=self.save_log).pack(side="left")

        # ---- 六维力数据保存 / CSV 记录 ----
        # 紧挨「保存日志」放，一进界面就能看到（原来埋在左侧面板最底部）
        self.btn_save = ttk.Button(top, text="开始保存数据", width=14,
                                   command=self.on_save_toggle)
        self.btn_save.pack(side="left", padx=(18, 0))
        self.btn_rec = ttk.Button(top, text="记录 CSV", width=10,
                                  command=self.on_record)
        self.btn_rec.pack(side="left", padx=(6, 0))

        self.var_save = tk.StringVar(value="未保存")
        self.var_rec = tk.StringVar(value="未记录")
        ttk.Label(top, textvariable=self.var_save,
                  foreground="#b0691a").pack(side="left", padx=(14, 0))
        ttk.Label(top, textvariable=self.var_rec,
                  foreground=TXT_DIM).pack(side="right")

        self.log_text = tk.Text(wrap, height=7, bg="#1e2228", fg="#d6dae0",
                                insertbackground="#d6dae0", relief="flat",
                                font=("Consolas", 9), wrap="none")
        sb = ttk.Scrollbar(wrap, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=sb.set)
        self.log_text.grid(row=1, column=0, sticky="ew")
        sb.grid(row=1, column=1, sticky="ns")
        wrap.columnconfigure(0, weight=1)
        self.log_text.tag_configure("TX", foreground="#6db3f2")
        self.log_text.tag_configure("RX", foreground="#7ed6a5")
        self.log_text.tag_configure("ERR", foreground="#ff8a80")
        self.log_text.tag_configure("SYS", foreground="#c9b458")

        bar = ttk.Frame(self.root)
        bar.grid(row=4, column=0, sticky="ew", padx=10, pady=(0, 6))
        self.var_stat = tk.StringVar(value="就绪")
        ttk.Label(bar, textvariable=self.var_stat).pack(side="left")
        self.var_stat2 = tk.StringVar(value="")
        ttk.Label(bar, textvariable=self.var_stat2, foreground=TXT_DIM).pack(side="right")

    # ==================================================================
    #  日志 / 状态线程回调（在接收线程中执行）
    # ==================================================================
    def _on_log_thread(self, direction: str, text: str) -> None:
        self.ui_queue.put(("log", direction, text, None))

    def _on_state_thread(self, connected: bool, msg: str) -> None:
        self.ui_queue.put(("state", connected, msg, None))

    def _on_error_thread(self, msg: str) -> None:
        self.ui_queue.put(("error", None, msg, None))

    def _on_push_thread(self, vals: List[float]) -> None:
        """
        接收线程回调：把每一帧六维力数据塞进待写缓冲。

        这里只入队、不做任何耗时操作（落盘交给界面线程），
        否则会拖慢 socket 收包，高速推流时会丢帧。
        """
        if not self.saving:
            return
        with self._save_lock:
            self._save_pending.append((time.time(), list(vals)))

    # ==================================================================
    #  界面操作
    # ==================================================================
    def _set_connected_ui(self, connected: bool) -> None:
        self._connected = connected
        self.btn_connect.configure(state="disabled" if connected else "normal")
        self.btn_disconnect.configure(state="normal" if connected else "disabled")
        color = "#27ae60" if connected else "#c0392b"
        self.lamp.itemconfigure(self._lamp_item, fill=color)

    def on_connect(self) -> None:
        host = self.var_host.get().strip()
        try:
            port = int(self.var_port.get())
        except ValueError:
            messagebox.showerror("参数错误", "端口必须是整数")
            return
        self.cli.host, self.cli.port = host, port
        self.cli.byte_order = self.var_order.get()
        self.cli.write_byte_order = self.var_worder.get()
        self.cli.push_byte_order = self.var_worder.get()

        def work():
            """连接后一次性把设备参数全部读回（避免多次异步互相阻塞）。"""
            self.cli.connect()
            # 先实测真机可访问范围与轮询字节序，再按实测结果读参数，
            # 否则会稳定复现「异常 02 非法数据地址」
            caps = self.cli.probe_capabilities()
            info = {
                "caps": caps,
                "sn": self.cli.get_sn(),
                "fw": self.cli.get_fw_version_str(),
                "format": self.cli.get_format(),
                "status": self.cli.get_status(),
                "comm_err": self.cli.get_comm_error(),
                "zero": self.cli.get_zero_offsets(),
                "ovl": self.cli.get_overload(),
                "rng": self.cli.get_ranges(),
                "net": self.cli.get_network(),
            }
            return info

        def done(info):
            caps = info["caps"]
            for n in caps["notes"]:
                self.log("SYS", n)
            self.var_order.set(self.cli.byte_order)
            self.var_sn.set(info["sn"])
            self.var_fw.set(info["fw"])
            self.var_status.set(f"0x{info['status']:04X}")
            self.var_comm_err.set(str(info["comm_err"]))
            self.var_status_txt.set(
                "状态位：" + "；".join(self.cli.describe_status(info["status"])))
            fmt = info["format"]
            self.var_fmt.set(fmt if fmt in (0, 1, 2) else Format.N)
            self._apply_units()

            for i, v in enumerate(info["zero"]):
                self.var_zero[i].set(f"{v:.6g}")
            self.overload_thresholds = list(info["ovl"])
            for i, v in enumerate(info["ovl"]):
                self.var_ovl[i].set(f"{v:.6g}")
            for i, (lo, hi) in enumerate(info["rng"]):
                unit = "N·m" if i >= 3 else "N"
                self.var_rng[i].set(f"{AXIS_NAMES[i]}: " + self._fmt_range(lo, hi, unit))
            net = info["net"]
            self.var_mac.set(net["mac"])
            self.var_ip.set(net["ip"])
            self.var_mask.set(net["subnet"])
            self.var_gw.set(net["gateway"])

            self.log("SYS", f"已连接 {host}:{port}  序列号 {info['sn']}  "
                            f"固件 {info['fw']}  IP {net['ip']}")
            self.log("SYS", f"字节序：轮询 {self.cli.byte_order} / "
                            f"单次·推流 {self.cli.push_byte_order} / "
                            f"写入 {self.cli.write_byte_order}")

        self._run_async(work, done, "正在连接设备并探测参数", require_conn=False)

    @staticmethod
    def _fmt_range(lo: Optional[float], hi: Optional[float], unit: str) -> str:
        """阈值范围显示；真机读不到的项显示为「不可读」而不是编造数值。"""
        lo_s = "?" if lo is None else f"{lo:.4g}"
        hi_s = "不可读" if hi is None else f"{hi:.4g}"
        return f"{unit}  {lo_s} ~ {hi_s}"

    def on_disconnect(self) -> None:
        if self.saving:
            self._finish_saving("连接断开", stop_device=False)
        if self.recording:
            self.on_record()
        self.cli.close()
        self.log("SYS", "已断开连接")

    def on_probe_order(self) -> None:
        """实测真机可访问范围 + 轮询字节序，并把结论写进日志。"""
        def work():
            return self.cli.probe_capabilities()

        def done(caps):
            for n in caps["notes"]:
                self.log("SYS", n)
            self.log("SYS", f"  力数据块可读 {caps['force_regs']} 个寄存器"
                            f"（协议文档写 12）")
            self.log("SYS", f"  阈值范围块可读 {caps['range_regs']} 个寄存器"
                            f"（协议文档写 24）")
            for o, lo, hi, ok in caps["poll_table"]:
                self.log("SYS", f"  字节序 {o}: min={lo:<14.6g}"
                                 f" max={hi:<14.6g} {'合理' if ok else '-'}")
            self.var_order.set(caps["poll_order"])
            self.log("SYS", f"轮询字节序判定为 {caps['poll_order']}"
                            f"（{ByteOrder.DESC.get(caps['poll_order'], '?')}）")

        self._run_async(work, done, "正在探测设备能力")

    def on_verify_write_order(self) -> None:
        """写入-回读往返自检 FC10 写路径的字节序（自愈，不改动设备参数）。"""
        def work():
            return self.cli.verify_write_order()

        def done(res):
            order, table = res
            for o, err in table:
                mark = "命中" if err <= 1e-3 else "不符"
                self.log("SYS", f"  写入字节序 {o}: 回读偏差 {err:<12.6g} {mark}")
            self.var_worder.set(order)
            self.cli.write_byte_order = order
            self.cli.push_byte_order = order
            self.log("SYS", f"写入/推流字节序判定为 {order}"
                            f"（{ByteOrder.DESC.get(order, '?')}）；"
                            f"设备参数已按原值还原")

        self._run_async(work, done, "正在做写入字节序往返自检")

    def _on_order_changed(self, _evt=None) -> None:
        self.cli.byte_order = self.var_order.get()
        self.cli.write_byte_order = self.var_worder.get()
        self.cli.push_byte_order = self.var_worder.get()
        self.log("SYS", f"字节序已切换为 轮询={self.cli.byte_order} "
                        f"单次/写入/推流={self.cli.write_byte_order}")

    def _on_channel_toggle(self) -> None:
        self.wave.enabled = [v.get() for v in self.var_ch]
        self.wave.draw(self.cli.wave_snapshot(self._window_sec()), force=True)

    def _window_sec(self) -> float:
        try:
            return float(self.var_window.get())
        except ValueError:
            return 5.0

    # ---------------- 控制指令 ----------------
    def on_start(self) -> None:
        self._run_async(lambda: self.cli.start(), None, "正在启动连续转换")
        self.log("SYS", "发送 0x0002 开始连续转换")

    def on_stop(self) -> None:
        self._run_async(lambda: self.cli.stop(), None, "正在停止发送")
        self.log("SYS", "发送 0x0001 停止发送")

    def on_single(self) -> None:
        def work():
            return self.cli.single_shot()

        def done(vals):
            src = self.cli.last_single_src
            if src == "push":
                tag = f"单次帧 {self.cli.push_byte_order}"
            else:
                tag = f"回退轮询 0x0033 {self.cli.byte_order}"
            txt = "  ".join(f"{AXIS_NAMES[i]}={'—' if v is None else f'{v:.3f}'}"
                            for i, v in enumerate(vals))
            self.var_single.set(f"最近单次结果：\n{txt}\n来源：{tag}")
            self.log("SYS", f"单次转换结果（{tag}）：{txt}")
            if src == "poll":
                self.log("ERR", "没等到设备回送的单次数据帧，已回退轮询读 0x0033 —— "
                                "旧固件把这一帧只发到 RS485（见 Docs/modbus-tcp-实现审查.md P1-6）")
            if any(v is None for v in vals):
                self.log("ERR", "有通道读不到：力数据块读不全，Mz 缺高 16 位"
                                "（旧固件 mb_reg_read 的 case 范围少一格）")

        self._run_async(work, done, "正在执行单次转换")

    def on_set_format(self) -> None:
        fmt = self.var_fmt.get()
        self._run_async(lambda: self.cli.set_format(fmt), None, "正在切换输出单位")
        self._apply_units()

    def on_set_freq(self) -> None:
        mode = self.var_freq.get()
        self._run_async(lambda: self.cli.set_freq_mode(mode), None, "正在设置采样频率")
        self.log("SYS", f"设置采样频率为 {FreqMode.NAME[mode]}")

    def on_zero(self) -> None:
        def work():
            return self.cli.zero()

        def done(offsets):
            for i, v in enumerate(offsets):
                self.var_zero[i].set(f"{v:.6g}")
            self.log("SYS", "自动清零完成，零点偏移："
                            + " ".join(f"{v:.4f}" for v in offsets))

        self._run_async(work, done, "正在执行自动清零（采集 20 点求平均）")

    def on_unzero(self) -> None:
        def work():
            self.cli.unzero()
            return self.cli.get_zero_offsets()

        def done(vals):
            for i, v in enumerate(vals):
                self.var_zero[i].set(f"{v:.6g}")
            self.log("SYS", "已取消清零，零点偏移归零")

        self._run_async(work, done, "正在取消清零")

    # ---------------- 参数读写 ----------------
    def on_read_info(self) -> None:
        def work():
            return (self.cli.get_sn(), self.cli.get_fw_version_str(),
                    self.cli.get_status(), self.cli.get_comm_error())

        def done(res):
            sn, fw, status, err = res
            self.var_sn.set(sn)
            self.var_fw.set(fw)
            self.var_status.set(f"0x{status:04X}")
            self.var_comm_err.set(str(err))
            self.var_status_txt.set("状态位：" + "；".join(self.cli.describe_status(status)))

        self._run_async(work, done, "正在读取设备信息")

    def on_read_zero(self) -> None:
        def work():
            return self.cli.get_zero_offsets()

        def done(vals):
            for i, v in enumerate(vals):
                self.var_zero[i].set(f"{v:.6g}")

        self._run_async(work, done, "正在读取零点偏移")

    def on_write_zero(self) -> None:
        try:
            vals = [float(v.get()) for v in self.var_zero]
        except ValueError:
            messagebox.showerror("输入错误", "零点偏移必须是数字")
            return

        def work():
            self.cli.set_zero_offsets(vals)
            return self.cli.get_zero_offsets()

        def done(back):
            for i, v in enumerate(back):
                self.var_zero[i].set(f"{v:.6g}")
            self.log("SYS", "零点偏移写入成功：" + " ".join(f"{v:.4f}" for v in back))

        self._run_async(work, done, "正在写入零点偏移")

    def on_read_ovl(self) -> None:
        def work():
            return self.cli.get_overload(), self.cli.get_ranges()

        def done(res):
            ovl, rng = res
            self.overload_thresholds = list(ovl)
            for i, v in enumerate(ovl):
                self.var_ovl[i].set(f"{v:.6g}")
            for i, (lo, hi) in enumerate(rng):
                unit = "N·m" if i >= 3 else "N"
                self.var_rng[i].set(f"{AXIS_NAMES[i]}: " + self._fmt_range(lo, hi, unit))

        self._run_async(work, done, "正在读取过载阈值")

    def on_write_ovl(self) -> None:
        try:
            vals = [float(v.get()) for v in self.var_ovl]
        except ValueError:
            messagebox.showerror("输入错误", "过载阈值必须是数字")
            return

        def work():
            self.cli.set_overload(vals)
            time.sleep(0.05)
            return self.cli.get_overload(), self.cli.get_status()

        def done(res):
            back, status = res
            self.overload_thresholds = list(back)
            for i, v in enumerate(back):
                self.var_ovl[i].set(f"{v:.6g}")
            changed = any(abs(a - b) > 1e-6 for a, b in zip(vals, back))
            if status & (1 << 4) or changed:
                self.var_status.set(f"0x{status:04X}")
                self.log("ERR", "写入被设备拒绝：数值超出允许范围（PARAM_ERR）")
                messagebox.showwarning(
                    "写入被拒绝",
                    "部分阈值超出允许范围，设备已忽略该值并置位 PARAM_ERR。\n"
                    "请参考「允许范围」列重新输入。")
            else:
                self.log("SYS", "过载阈值写入成功")

        self._run_async(work, done, "正在写入过载阈值")

    def on_read_net(self) -> None:
        def work():
            return self.cli.get_network()

        def done(net):
            self.var_mac.set(net["mac"])
            self.var_ip.set(net["ip"])
            self.var_mask.set(net["subnet"])
            self.var_gw.set(net["gateway"])

        self._run_async(work, done, "正在读取网络参数")

    def on_write_net(self) -> None:
        if not messagebox.askyesno(
                "确认写入",
                "网络参数写入后需重启传感器才生效。\n"
                "若新 IP 与当前网段不匹配，重启后将无法连接。\n\n确定写入吗？"):
            return
        mac, ip = self.var_mac.get().strip(), self.var_ip.get().strip()
        mask, gw = self.var_mask.get().strip(), self.var_gw.get().strip()

        def work():
            self.cli.set_mac(mac)
            self.cli.set_ip(ip)
            self.cli.set_subnet(mask)
            self.cli.set_gateway(gw)
            return self.cli.get_network()

        def done(net):
            self.log("SYS", f"网络参数已写入：{net}（重启后生效）")

        self._run_async(work, done, "正在写入网络参数")

    def on_read_selftest(self) -> None:
        def work():
            return self.cli.get_self_test()

        def done(st):
            lines = [
                f"错误标志字   : 0x{st['error_flags']:08X}",
                f"ADC ID/错误  : 0x{st['adc_id']:02X} / 0x{st['adc_error']:02X}",
                f"W5500 版本   : 0x{st['w5500_version']:02X}",
                f"PHY Link     : {'UP' if st['w5500_phy_link'] else 'DOWN'}",
                f"Socket 就绪  : {st['w5500_sockets_ok']} 路",
                f"RS485 UART   : {'OK' if st['rs485_uart_ok'] else 'FAIL'}",
                f"RS485 DMA    : {'OK' if st['rs485_dma_ok'] else 'FAIL'}",
                f"Flash 有效   : {'是' if st['flash_valid'] else '否'}",
                f"  标定矩阵   : {'OK' if st['flash_matrix_ok'] else 'NG'}",
                f"  零点数据   : {'OK' if st['flash_zero_ok'] else 'NG'}",
                f"  配置数据   : {'OK' if st['flash_config_ok'] else 'NG'}",
            ]
            self.var_self_test.set("\n".join(lines))

        self._run_async(work, done, "正在读取自检寄存器")

    # ---------------- 六维力数据保存（Excel）----------------
    def on_save_toggle(self) -> None:
        """
        「开始保存数据」→ 把六维力数据逐帧写进 Excel；再点一次 → 停止。
        文件默认落在程序（.exe）同目录，不用每次选路径。
        """
        if self.saving:
            self._finish_saving("手动停止")
            return
        if not self.cli.connected:
            messagebox.showinfo("未连接", "请先连接设备，再开始保存数据")
            return

        self._save_path = os.path.join(
            self.save_dir, time.strftime("hs_force_%Y%m%d_%H%M%S.xlsx"))
        try:
            writer = XlsxTableWriter(self._save_path, headers=SAVE_HEADERS,
                                     sheet_name="六维力数据")
        except OSError as e:
            messagebox.showerror(
                "无法创建文件",
                f"{self._save_path}\n\n{e}\n\n"
                "如果程序放在只读目录，请把它挪到可写目录，\n"
                "或用 --save-dir 指定一个可写目录。")
            return

        # 丢掉按下按钮之前残留在缓冲里的帧，文件从这一刻开始记
        with self._save_lock:
            self._save_pending.clear()
        self._xlsx = writer
        self._save_rows = 0
        self._save_skipped = 0
        self._save_t0 = time.time()
        self.saving = True
        self.btn_save.configure(text="停止保存数据")
        self.var_save.set(f"保存中：0 行 · {os.path.basename(self._save_path)}")
        self.log("SYS", f"开始保存六维力数据 → {self._save_path}")

        # 「开始保存」同时开始采集：设备没在推流的话一帧都存不到
        self._run_async(lambda: self.cli.start(), None, "正在启动连续转换")
        self.log("SYS", "保存期间自动开启连续转换（写 0x0002 = 1）")

    def _drain_save_buffer(self) -> None:
        """把接收线程攒下的数据帧写进 Excel（只在界面线程调用）。"""
        writer = self._xlsx
        if writer is None:
            return
        with self._save_lock:
            if not self._save_pending:
                return
            batch = list(self._save_pending)
            self._save_pending.clear()

        for _ts, vals in batch:
            try:
                if writer.add_row(vals):
                    self._save_rows += 1
            except XlsxLimitError as e:
                self.log("ERR", str(e))
                self._finish_saving("达到 Excel 单表行数上限", stop_device=False)
                return
        self._save_skipped = writer.skipped_rows
        skip = f"（跳过 {self._save_skipped}）" if self._save_skipped else ""
        self.var_save.set(f"保存中：{self._save_rows} 行{skip} · "
                          f"{os.path.basename(self._save_path)}")

    def _finish_saving(self, why: str, stop_device: bool = True) -> None:
        """收尾：写完缓冲里最后几帧、生成 .xlsx、把按钮恢复回去。"""
        if self._xlsx is None:
            self.saving = False
            return
        self._drain_save_buffer()            # 缓冲里可能还有最后几帧
        writer, self._xlsx = self._xlsx, None
        self.saving = False
        self.btn_save.configure(text="开始保存数据")

        try:
            rows = writer.close()
        except OSError as e:
            self.var_save.set("保存失败")
            self.log("ERR", f"写入 Excel 失败：{e}")
            messagebox.showerror("保存失败", f"{self._save_path}\n\n{e}")
            return

        elapsed = time.time() - self._save_t0
        self.var_save.set(f"已保存 {rows} 行 / {elapsed:.1f} s · "
                          f"{os.path.basename(self._save_path)}")
        self.log("SYS", f"数据保存结束（{why}）：写入 {rows} 行，"
                        f"用时 {elapsed:.1f} s，文件 {self._save_path}")
        if self._save_skipped:
            self.log("ERR", f"另有 {self._save_skipped} 行六个通道全无效被跳过"
                            "（设备未标定时读数就是 NaN）")
        if rows == 0:
            self.log("ERR", "一行都没写进 Excel —— 设备可能没在连续转换，"
                            "或六个通道读数全为 NaN（未标定）")

        if stop_device and self.cli.connected:
            self._run_async(lambda: self.cli.stop(), None, "正在停止发送")
            self.log("SYS", "已停止连续转换（写 0x0001 = 1）")

    # ---------------- CSV 记录 ----------------
    def on_record(self) -> None:
        if self.recording:
            self.recording = False
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
            self.btn_rec.configure(text="记录 CSV")
            self.var_rec.set(f"已停止，共 {self._csv_rows} 行")
            self.log("SYS", f"数据记录结束，共 {self._csv_rows} 行")
            return

        path = filedialog.asksaveasfilename(
            title="保存数据记录", defaultextension=".csv",
            initialfile=time.strftime("hs_data_%Y%m%d_%H%M%S.csv"),
            filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            self._csv_file = open(path, "w", newline="", encoding="utf-8-sig")
        except OSError as e:
            messagebox.showerror("无法创建文件", str(e))
            return
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ["时间戳", "本地时间", "格式", "Fx", "Fy", "Fz", "Mx", "My", "Mz"])
        self._csv_rows = 0
        self._csv_last_ts = 0.0
        self.recording = True
        self.btn_rec.configure(text="停止记录")
        self.var_rec.set(os.path.basename(path))
        self.log("SYS", f"开始记录数据：{path}")

    def _write_csv(self) -> None:
        if not self.recording or not self._csv_writer:
            return
        # 取最近 30 秒窗口，足够覆盖 UI 刷新间隔，不会漏点
        rows = [r for r in self.cli.wave_snapshot(30.0) if r[0] > self._csv_last_ts]
        if not rows:
            return
        fmt = Format.NAME.get(self.var_fmt.get(), "N")
        for ts, vals in rows:
            self._csv_writer.writerow(
                [f"{ts:.6f}", time.strftime("%H:%M:%S", time.localtime(ts)), fmt]
                + ["" if (v is None or v != v) else f"{v:.6f}" for v in vals])
            self._csv_last_ts = ts
            self._csv_rows += 1
        self._csv_file.flush()
        self.var_rec.set(f"记录中：{self._csv_rows} 行")

    # ---------------- 日志 ----------------
    def log(self, tag: str, text: str) -> None:
        ts = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{ts}] {text}\n", tag)
        if int(self.log_text.index("end-1c").split(".")[0]) > 800:
            self.log_text.delete("1.0", "200.0")
        self.log_text.see("end")

    def clear_log(self) -> None:
        self.log_text.delete("1.0", "end")

    def save_log(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存日志", defaultextension=".txt",
            initialfile=time.strftime("hs_log_%Y%m%d_%H%M%S.txt"),
            filetypes=[("文本文件", "*.txt"), ("所有文件", "*.*")])
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.log_text.get("1.0", "end"))
            self.log("SYS", f"日志已保存至 {path}")
        except OSError as e:
            messagebox.showerror("保存失败", str(e))

    # ==================================================================
    #  异步执行辅助
    # ==================================================================
    def _run_async(self, fn: Callable, on_ok: Optional[Callable] = None,
                   desc: str = "", require_conn: bool = True) -> None:
        if require_conn and not self.cli.connected:
            messagebox.showinfo("未连接", "请先连接设备")
            return
        if self._busy:
            return
        self._busy = True
        if desc:
            self.var_stat.set(desc + " ...")

        def work():
            try:
                res = fn()
                self.ui_queue.put(("async", on_ok, res, None))
            except Exception as e:                       # noqa: BLE001
                self.ui_queue.put(("async", on_ok, None, e))

        threading.Thread(target=work, daemon=True).start()

    # ==================================================================
    #  周期刷新
    # ==================================================================
    def _tick(self) -> None:
        try:
            self._drain_queue()
            # 保存落盘不依赖连接状态：断线时缓冲里已有数据也要写完
            self._drain_save_buffer()
            if self._connected:
                self._update_values()
                self._update_wave()
                self._write_csv()
                self._update_rate()
        finally:
            self.root.after(40, self._tick)

    def _drain_queue(self) -> None:
        while True:
            try:
                item = self.ui_queue.get_nowait()
            except queue.Empty:
                break
            if not item or len(item) != 4:
                continue
            kind, a, b, c = item
            if kind == "log":
                if self.var_log_on.get():
                    self.log(a, b)
            elif kind == "state":
                self._set_connected_ui(bool(a))
                self.var_conn.set(b)
                self.log("SYS", b)
            elif kind == "error":
                self.log("ERR", str(b))
                self.var_stat.set("错误：" + str(b))
            elif kind == "async":
                self._busy = False
                self.var_stat.set("就绪")
                on_ok, res, err = a, b, c
                if err is not None:
                    self.log("ERR", f"操作失败：{err}")
                    self.var_stat.set(f"操作失败：{err}")
                elif on_ok is not None:
                    try:
                        on_ok(res)
                    except Exception as e:               # noqa: BLE001
                        self.log("ERR", f"回调异常：{e}")

    def _apply_units(self) -> None:
        fmt = self.var_fmt.get()
        units = ([Format.UNIT_FORCE[fmt]] * 3) + ([Format.UNIT_TORQUE[fmt]] * 3)
        for i, u in enumerate(units):
            self.unit_labels[i].configure(text=u)
        self.wave.set_units(units)

    def _update_values(self) -> None:
        vals = self.cli.latest_values()
        if not vals:
            return
        for i, v in enumerate(vals):
            if v is None or v != v:                     # None 或 NaN
                self.val_labels[i].configure(text="—", fg=TXT_DIM)
                continue
            self.val_labels[i].configure(text=f"{v:+.3f}")
            over = abs(v) >= abs(self.overload_thresholds[i])
            self.val_labels[i].configure(fg="#c0392b" if over else TXT)

    def _update_wave(self) -> None:
        if self.cli.push_count == self._last_drawn_count:
            return
        self._last_drawn_count = self.cli.push_count
        samples = self.cli.wave_snapshot(self._window_sec())
        self.wave.draw(samples)

    def _update_rate(self) -> None:
        now = time.time()
        if now - self._rate_t0 >= 1.0:
            n = self.cli.push_count
            self.sample_rate = (n - self._rate_count) / (now - self._rate_t0)
            self._rate_count = n
            self._rate_t0 = now
            self.var_stat2.set(
                f"接收速率 {self.sample_rate:6.1f} Hz   "
                f"累计 {n} 帧   错误 {self.cli.error_count} 次   "
                f"字节序 轮询{self.cli.byte_order}/单次·写·推流{self.cli.write_byte_order}")

    # ==================================================================
    def on_close(self) -> None:
        try:
            if self.recording:
                self.on_record()
            if self.saving:
                # 关窗时先把已经采到的数据落盘，别丢
                self._finish_saving("程序退出", stop_device=False)
            self.cli.close()
        except Exception:                                # noqa: BLE001
            pass
        self.root.destroy()


# =====================================================================
def enable_dpi_awareness() -> float:
    """Windows 高分屏适配，返回缩放系数。"""
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
        return windll.shcore.GetScaleFactorForDevice(0) / 100.0
    except Exception:                                    # noqa: BLE001
        return 1.0


def main() -> int:                                       # pragma: no cover
    ap = argparse.ArgumentParser(description="六维力传感器 Modbus-TCP 上位机")
    ap.add_argument("--host", default="192.168.1.12")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--order", default=ByteOrder.POLL_DEFAULT,
                    choices=list(ByteOrder.ALL), help="FC03 轮询字节序")
    ap.add_argument("--write-order", default=ByteOrder.WRITE_DEFAULT,
                    choices=list(ByteOrder.ALL), help="FC10 写入 / 主动推流 字节序")
    ap.add_argument("--save-dir", default=None,
                    help="「开始保存数据」写出的 Excel 存放目录（默认程序所在目录）")
    ap.add_argument("--selftest", action="store_true",
                    help="仅构建界面后退出，用于检查环境")
    args = ap.parse_args()

    scale = enable_dpi_awareness()
    root = tk.Tk()
    if scale != 1.0:
        try:
            root.tk.call("tk", "scaling", 1.3333 * scale)
        except tk.TclError:
            pass
    root.geometry(f"{int(1380 * scale)}x{int(880 * scale)}")
    root.minsize(int(1080 * scale), int(680 * scale))
    app = MainWindow(root, args.host, args.port, args.order, args.write_order,
                     args.save_dir)

    if args.selftest:
        root.update_idletasks()
        root.update()
        print("界面构建成功：", root.winfo_width(), "x", root.winfo_height())
        print("数据保存目录：", app.save_dir)
        # 顺便验一下 .xlsx 写出链路（打包后的 exe 最容易在这里缺东西）
        import shutil
        import tempfile
        tmp = tempfile.mkdtemp(prefix="hs_selftest_")
        try:
            probe = os.path.join(tmp, "probe.xlsx")
            w = XlsxTableWriter(probe, headers=SAVE_HEADERS, sheet_name="六维力数据")
            for i in range(5):
                w.add_row([float(i), 1.0, 2.0, 3.0, 4.0, 5.0])
            n = w.close()
            print(f"Excel 写出自检：{n} 行，{os.path.getsize(probe)} 字节")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
        app.cli.close()
        root.destroy()
        return 0

    root.mainloop()
    return 0


if __name__ == "__main__":                               # pragma: no cover
    sys.exit(main())
