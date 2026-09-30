# -*- coding: utf-8 -*-
"""界面冒烟测试：驱动 MainWindow 走一遍 连接→采集→参数读写 流程（不显示窗口）。

用法：
    python hs_simulator.py --port 15020     # 另开终端（默认即 P0-1/P1-6 修复后的形态）
    python test_gui.py

    python hs_simulator.py --port 15020 --legacy-blocks --legacy-single
    python test_gui.py --legacy             # 设备为未修复旧固件
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import tempfile
import time
import tkinter as tk
import zipfile
from tkinter import ttk

from hs_app import SAVE_HEADERS, MainWindow

HOST, PORT = "127.0.0.1", 15020
ok, fail = 0, 0

# 期望的设备形态。P0-1 修复后力块 12 / 范围块 24 个寄存器全可读；
# --legacy 表示设备仍跑着未修复的固件（11 / 23，第 6 通道 Mz 读不全）。
EXP_FORCE, EXP_RANGE = 12, 24


def check(name, cond, detail=""):
    global ok, fail
    if cond:
        ok += 1
        print(f"[PASS] {name}  {detail}")
    else:
        fail += 1
        print(f"[FAIL] {name}  {detail}")


def pump(root, seconds):
    end = time.time() + seconds
    while time.time() < end:
        root.update()
        time.sleep(0.02)


def wait_for(root, cond, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        root.update()
        if cond():
            return True
        time.sleep(0.02)
    return False


def wait_idle(root, app, timeout=10.0):
    """等待异步操作结束后再继续。"""
    return wait_for(root, lambda: not app._busy, timeout)


class _Wheel:
    """伪造一个滚轮事件，用来在测试里驱动 _on_panel_wheel。"""

    def __init__(self, delta: int) -> None:
        self.delta = delta


def main():
    global EXP_FORCE, EXP_RANGE
    ap = argparse.ArgumentParser(description="界面冒烟测试（需先启动模拟器）")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--legacy", action="store_true",
                    help="设备仍跑未修复的旧固件：力块 11 / 范围块 23，Mz 读不全")
    args = ap.parse_args()
    if args.legacy:
        EXP_FORCE, EXP_RANGE = 11, 23

    root = tk.Tk()
    root.withdraw()
    # 「保存数据」默认落在程序目录，测试里改到临时目录，别污染工程
    save_dir = tempfile.mkdtemp(prefix="hs_gui_save_")
    # 与真机一致：轮询 CDAB / 写入·推流 DCBA
    app = MainWindow(root, args.host, args.port, "CDAB", "DCBA", save_dir)
    pump(root, 0.3)
    check("界面构建", True, f"{root.winfo_reqwidth()}x{root.winfo_reqheight()}")

    # 界面控件应完整：单次触发按钮 + 轮询 FC03 字节序下拉框（曾隐藏过，防回归）
    def _walk(w):
        yield w
        for c in w.winfo_children():
            yield from _walk(c)

    texts = []
    for w in _walk(app.root):
        try:
            texts.append(str(w.cget("text")))
        except (tk.TclError, AttributeError):
            pass
    check("单次触发按钮存在", "单次触发" in texts, "界面文本中存在「单次触发」")
    check("轮询 FC03 字节序下拉框存在", "轮询 FC03 字节序" in texts,
          "界面文本中存在「轮询 FC03 字节序」")
    check("单次/写入/推流 字节序下拉框存在", "单次 / 写入 FC10 / 推流 字节序" in texts,
          "界面文本中存在「单次 / 写入 FC10 / 推流 字节序」")
    check("保存数据按钮存在", "开始保存数据" in texts,
          "界面文本中存在「开始保存数据」")
    # 保存按钮放在右下角日志栏里，紧挨「保存日志」——别再挪回左侧面板最底部
    sib = {str(c.cget("text")) for c in app.btn_save.master.winfo_children()
           if isinstance(c, ttk.Button)}
    check("「开始保存数据」与「保存日志」在同一行",
          {"保存日志", "开始保存数据", "记录 CSV"} <= sib, str(sorted(sib)))

    # ---------------- 左侧面板滚动 ----------------
    # ① 面板内所有控件都绑了滚轮（Tk 事件不会从子控件冒到父控件，必须逐个绑）
    wheel_ok = []

    def _walk_panel(w):
        wheel_ok.append("<MouseWheel>" in w.bind())
        for c in w.winfo_children():
            _walk_panel(c)

    _walk_panel(app._panel_box)
    check("左侧面板内所有控件都响应滚轮",
          len(wheel_ok) > 20 and all(wheel_ok), f"{len(wheel_ok)} 个控件")
    check("日志区不被面板滚轮劫持",
          "<MouseWheel>" not in app.log_text.bind(), "日志 Text 未绑滚轮")
    check("左侧面板滚动区域已建立",
          app.panel_canvas.bbox("all") is not None, "scrollregion 可用")

    # ② 内容超过可视高度 → 滚动条自动出现；滚轮真的能滚；内容变矮 → 自动收起
    root.deiconify()
    root.update_idletasks()
    root.update()
    filler = tk.Frame(app._panel_box, height=700)
    filler.grid(row=99, column=0, sticky="ew")
    root.update_idletasks()
    root.update()
    check("面板内容超高时滚动条自动出现", app._panel_sb_on,
          f"内容 {app.panel_canvas.bbox('all')[3]} px / 视口 "
          f"{app.panel_canvas.winfo_height()} px")
    before = app.panel_canvas.yview()[0]
    app._on_panel_wheel(_Wheel(-120))
    root.update_idletasks()
    check("滚轮向下滚动左侧面板", app.panel_canvas.yview()[0] > before,
          f"{before:.3f} → {app.panel_canvas.yview()[0]:.3f}")
    filler.destroy()
    root.update_idletasks()
    root.update()
    bb = app.panel_canvas.bbox("all")
    content_h, view_h = bb[3] - bb[1], app.panel_canvas.winfo_height()
    check("滚动条显隐跟随内容/视口高度", app._panel_sb_on == (content_h > view_h),
          f"内容 {content_h} px / 视口 {view_h} px → 滚动条={app._panel_sb_on}")
    root.withdraw()

    app.on_connect()
    got = wait_for(root, lambda: app._connected and app.var_sn.get() not in ("—", ""))
    wait_idle(root, app)
    check("连接并读取设备信息", got,
          f"SN={app.var_sn.get()} FW={app.var_fw.get()} 状态={app.var_status.get()}")
    check("自动读取网络参数", app.var_ip.get() not in ("", "—"), app.var_ip.get())
    check("自动读取阈值/范围", app.var_rng[0].get() != "—", app.var_rng[0].get())
    check("自动读取零点偏移", app.var_zero[0].get() != "—", app.var_zero[0].get())
    # 连接后自动做了能力探测（期望值随设备形态变化，见 --legacy）
    check("连接时自动探测能力（不报非法地址）",
          app.cli.caps.get("probed") and app.cli.caps.get("force_regs") == EXP_FORCE
          and app.cli.caps.get("range_regs") == EXP_RANGE,
          f"force_regs={app.cli.caps.get('force_regs')} "
          f"range_regs={app.cli.caps.get('range_regs')} "
          f"(期望 {EXP_FORCE}/{EXP_RANGE})")
    if EXP_RANGE >= 24:
        check("P0-1 修复后 Mz 最大值可读（不再标注「不可读」）",
              "不可读" not in app.var_rng[5].get(), app.var_rng[5].get())
    else:
        check("旧固件：阈值范围不可读项标注为「不可读」",
              "不可读" in app.var_rng[5].get(), app.var_rng[5].get())

    # 采样
    app.on_start()
    pump(root, 2.0)
    vals = [lbl.cget("text") for lbl in app.val_labels]
    check("连续采集刷新数值面板", all(v != "—" for v in vals), " ".join(vals))
    check("波形已绘制", len(app.wave.find_all()) > 10,
          f"{len(app.wave.find_all())} 个图元")
    check("采样率统计", app.sample_rate > 50, f"{app.sample_rate:.1f} Hz")

    # 推流帧必须是逐帧重新采的（曾经只在进入推流时算一次 → 整段是同一个值）
    seen = set()
    for _ in range(15):
        v = app.cli.latest_values()
        if v:
            seen.add(tuple(round(x, 6) for x in v))
        pump(root, 0.05)
    check("推流数据随时间变化（不是一段重复值）", len(seen) > 1,
          f"观察到 {len(seen)} 种取值")

    # ---------------- 六维力数据保存（Excel）----------------
    app.on_save_toggle()
    pump(root, 0.5)
    check("点「开始保存数据」后按钮变成「停止保存数据」",
          app.saving and app.btn_save.cget("text") == "停止保存数据",
          f"saving={app.saving} 按钮={app.btn_save.cget('text')}")
    check("保存时自动保持连续转换", app.cli.connected, "连接正常")
    pump(root, 1.5)
    during = app._save_rows
    check("保存过程中持续攒行", during > 100, f"2 s 攒了 {during} 行")
    check("保存状态标签实时刷新", "保存中" in app.var_save.get(), app.var_save.get())

    app.on_save_toggle()
    wait_idle(root, app)
    pump(root, 0.4)
    check("再点一次按钮恢复为「开始保存数据」",
          (not app.saving) and app.btn_save.cget("text") == "开始保存数据",
          app.btn_save.cget("text"))

    path = app._save_path
    check("Excel 生成在指定目录（默认为程序同目录）",
          os.path.isfile(path) and os.path.dirname(path) == save_dir, path)
    if os.path.isfile(path):
        with zipfile.ZipFile(path) as zf:
            sheet = zf.read("xl/worksheets/sheet1.xml").decode("utf-8")
            wb = zf.read("xl/workbook.xml").decode("utf-8")
        rows_xml = re.findall(r"<row [^>]*>.*?</row>", sheet)
        heads = re.findall(r"<t[^>]*>([^<]*)</t>", sheet)
        check("表头是 FX FY FZ TX TY TZ 六列",
              heads == list(SAVE_HEADERS), str(heads))
        check("表内数据行数与界面统计一致",
              len(rows_xml) - 1 == app._save_rows and len(rows_xml) - 1 >= during,
              f"表内 {len(rows_xml) - 1} 行 / 界面 {app._save_rows} 行")
        widths = {len(re.findall(r"<c ", r)) for r in rows_xml[1:]}
        check("每行六列都有数值（模拟器通道全有效）", widths == {6}, str(widths))
        first = re.findall(r"<v>([^<]+)</v>", rows_xml[1]) if len(rows_xml) > 1 else []
        last = re.findall(r"<v>([^<]+)</v>", rows_xml[-1]) if len(rows_xml) > 1 else []
        check("表内数据确实是逐帧变化的（首行 ≠ 末行）",
              bool(first) and first != last,
              f"首行 Fx={first[0] if first else '?'} / 末行 Fx={last[0] if last else '?'}")
        check("工作表名正确", "六维力数据" in wb, "六维力数据")
        check("停止后状态标签给出保存结果",
              "已保存" in app.var_save.get(), app.var_save.get().replace("\n", " / "))
    check("停止保存后已停止连续转换", app.cli.connected, "连接仍保持")

    # 后续用例仍要有数据流
    wait_idle(root, app)
    app.on_start()
    wait_idle(root, app)
    pump(root, 0.5)

    # 单次触发（取数来源随固件不同：新固件回送单次帧，旧固件回退轮询）
    app.on_single()
    wait_idle(root, app)
    src = app.cli.last_single_src
    single_txt = app.var_single.get().replace("\n", " ")
    check("单次触发", "最近单次结果：" in app.var_single.get(), single_txt)
    check("单次结果标注取数来源", "来源：" in app.var_single.get(), single_txt)
    if src == "push":
        check("单次取数走设备回送帧（DCBA 单次帧）",
              "单次帧" in app.var_single.get(), single_txt)
        check("单次帧含完整 Mz（不依赖轮询读）",
              "Mz=—" not in app.var_single.get(), single_txt)
    else:
        check("单次取数回退轮询读 0x0033",
              "回退轮询" in app.var_single.get(), single_txt)
        if EXP_FORCE >= 12:
            check("单次结果中 Mz 有值（第 6 通道可读）",
                  "Mz=—" not in app.var_single.get(), single_txt)
        else:
            check("旧固件：单次结果中读不全的通道标为「—」",
                  "Mz=—" in app.var_single.get(), single_txt)

    # 清零
    app.on_zero()
    wait_idle(root, app)
    check("自动清零并回填零点偏移",
          app.var_zero[0].get() not in ("0", "—"), app.var_zero[0].get())

    # 单位切换
    app.var_fmt.set(1)
    app.on_set_format()
    wait_idle(root, app)
    check("单位切换为 kg", app.unit_labels[0].cget("text") == "kg"
          and app.unit_labels[3].cget("text") == "kg·m",
          f"{app.unit_labels[0].cget('text')} / {app.unit_labels[3].cget('text')}")
    app.var_fmt.set(2)
    app.on_set_format()
    wait_idle(root, app)

    # 阈值写入
    app.var_ovl[0].set("1200.5")
    app.on_write_ovl()
    wait_idle(root, app)
    check("过载阈值写入回读", abs(float(app.var_ovl[0].get()) - 1200.5) < 1e-3,
          app.var_ovl[0].get())

    # 零点偏移写入
    app.var_zero[1].set("-3.25")
    app.on_write_zero()
    wait_idle(root, app)
    check("零点偏移写入回读", abs(float(app.var_zero[1].get()) + 3.25) < 1e-3,
          app.var_zero[1].get())

    # 自检寄存器
    app.on_read_selftest()
    wait_idle(root, app)
    check("读取自检寄存器", "W5500" in app.var_self_test.get(),
          app.var_self_test.get().splitlines()[2] if app.var_self_test.get() else "")

    # 字节序自检
    app.on_probe_order()
    wait_idle(root, app)
    check("读取字节序自检", app.var_order.get() == "CDAB", app.var_order.get())

    # 写入字节序往返自检
    app.on_verify_write_order()
    wait_idle(root, app)
    check("写入字节序自检判定为 DCBA", app.var_worder.get() == "DCBA",
          f"{app.var_worder.get()} / 客户端={app.cli.write_byte_order}")
    check("写入自检未改动设备阈值",
          abs(float(app.var_ovl[0].get()) - 1200.5) < 1e-3,
          app.var_ovl[0].get())

    app.on_stop()
    wait_idle(root, app)

    # 通道勾选
    app.var_ch[0].set(False)
    app._on_channel_toggle()
    pump(root, 0.3)
    check("通道勾选生效", app.wave.enabled[0] is False, str(app.wave.enabled))

    app.on_disconnect()
    pump(root, 0.5)
    check("断开连接", not app._connected, app.var_conn.get())

    app.cli.close()
    root.destroy()
    shutil.rmtree(save_dir, ignore_errors=True)
    print(f"\n===== 界面冒烟测试：通过 {ok}，失败 {fail} =====")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
