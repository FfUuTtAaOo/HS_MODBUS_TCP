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
import sys
import time
import tkinter as tk

from hs_app import MainWindow

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
    # 与真机一致：轮询 CDAB / 写入·推流 DCBA
    app = MainWindow(root, args.host, args.port, "CDAB", "DCBA")
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
    print(f"\n===== 界面冒烟测试：通过 {ok}，失败 {fail} =====")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
