# -*- coding: utf-8 -*-
"""
test_link.py —— 上位机协议层无界面联调自测

覆盖：设备信息、网络参数、能力探测、字节序判定、阈值读写与越界拦截、
      单次触发、零点偏移、连续推流、停止推流、异常码、写入字节序往返。

用法：
    1) 另开一个终端启动模拟器（默认参数即 P0-1 修复后的固件形态）：
           python hs_simulator.py --port 15020
    2) 运行本脚本：
           python test_link.py --port 15020

    连真机（只读项会通过，写项会改设备阈值，谨慎）：
           python test_link.py --host 192.168.1.12

    若设备仍跑着未修复的旧固件（Mz 读不全 / 单次帧只发 RS485），把期望换成旧形态：
           python test_link.py --port 15020 --force-regs 11 --range-regs 23 --no-single-push
    （模拟器侧对应 `hs_simulator.py --legacy-blocks --legacy-single`）
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

from hs_modbus import (ByteOrder, Format, ModbusException, ModbusTimeoutError,
                       Reg, SensorClient)

PASS, FAIL = 0, 0


def check(name: str, cond: bool, detail: str = "") -> None:
    global PASS, FAIL
    mark = "PASS" if cond else "FAIL"
    if cond:
        PASS += 1
    else:
        FAIL += 1
    print(f"[{mark}] {name}" + (f"  -> {detail}" if detail else ""))


def fmt_vals(vals) -> str:
    return " ".join("   ——   " if v is None else f"{v:8.3f}" for v in vals)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=15020)
    ap.add_argument("--order", default=None, choices=list(ByteOrder.ALL),
                    help="把读/写/推流三条路径统一为该字节序（模拟器需 --order 同值）")
    ap.add_argument("--force-regs", type=int, default=12,
                    help="期望的力数据块可读寄存器数；已修复固件为 12，未修复旧固件为 11")
    ap.add_argument("--range-regs", type=int, default=24,
                    help="期望的阈值范围块可读寄存器数；已修复固件为 24，未修复旧固件为 23")
    ap.add_argument("--no-single-push", action="store_true",
                    help="设备不回送单次帧（未修复旧固件）：单次查询应回退轮询读")
    ap.add_argument("--legacy-sticky", action="store_true",
                    help="设备为未修复 TCP 粘包的旧固件：同批到达的请求只应答第一帧")
    ap.add_argument("--timeout", type=float, default=1.5)
    args = ap.parse_args()

    kw = {}
    if args.order:
        kw = dict(byte_order=args.order, push_byte_order=args.order,
                  write_byte_order=args.order)
    cli = SensorClient(args.host, args.port, timeout=args.timeout, **kw)
    cli.connect()
    print(f"== 已连接 {args.host}:{args.port}  "
          f"读={cli.byte_order} 写={cli.write_byte_order} "
          f"推流={cli.push_byte_order} ==\n")

    try:
        # ---------- 1. 设备信息 ----------
        sn = cli.get_sn()
        check("读序列号 (0x0005)", sn != "", repr(sn))
        fw = cli.get_fw_version_str()
        check("读固件版本 (0x000D)", fw.startswith("v"), fw)
        status = cli.get_status()
        check("读系统状态 (0x000F)", isinstance(status, int),
              f"0x{status:04X} / {cli.describe_status(status)}")
        err = cli.get_comm_error()
        check("读通信错误计数 (0x0010)", isinstance(err, int), str(err))

        # ---------- 2. 网络参数 ----------
        net = cli.get_network()
        check("读网络参数 (0x0100~0x010B)",
              net["ip"].count(".") == 3, str(net))

        # ---------- 3. 能力探测 ----------
        caps = cli.probe_capabilities()
        print("     能力探测：")
        for n in caps["notes"]:
            print(f"       * {n}")
        check("力数据块可读长度", caps["force_regs"] == args.force_regs,
              f"实测 {caps['force_regs']}（期望 {args.force_regs}）")
        check("阈值范围块可读长度", caps["range_regs"] == args.range_regs,
              f"实测 {caps['range_regs']}（期望 {args.range_regs}）")

        # ---------- 4. 字节序判定 ----------
        print("     字节序判据表：")
        for o, lo, hi, ok in caps["poll_table"]:
            print(f"       {o}: min={lo:<14.6g} max={hi:<14.6g} {'合理' if ok else '-'}")
        want = args.order or ByteOrder.POLL_DEFAULT
        check("轮询字节序自动判定", caps["poll_order"] == want,
              f"判定={caps['poll_order']} 期望={want}")

        # ---------- 5. 阈值范围 / 过载阈值 ----------
        ranges = cli.get_ranges()
        check("读阈值范围 (0x007C)",
              ranges[0][0] is not None and ranges[0][1] is not None
              and ranges[0][0] < ranges[0][1],
              f"Fx=[{ranges[0][0]}, {ranges[0][1]}]  Mz={ranges[5]}")
        if args.range_regs < 24:
            check("旧固件：范围块缺失项显示为 None（不编造数值）",
                  ranges[5][1] is None, f"Mz 最大值 = {ranges[5][1]}")
        else:
            check("Mz 最大值可读（P0-1 已修复）",
                  ranges[5][0] is not None and ranges[5][1] is not None,
                  f"Mz 范围 = [{ranges[5][0]}, {ranges[5][1]}]")

        ovl = cli.get_overload()
        check("读过载阈值 (0x0070)", len(ovl) == 6,
              str([round(v, 3) for v in ovl]))

        cli.set_overload([1000.0, 2000.0, 3000.0, 150.0, 150.0, 150.0])
        time.sleep(0.05)
        ovl2 = cli.get_overload()
        check("写过载阈值并回读", abs(ovl2[0] - 1000.0) < 1e-3, str(ovl2[:3]))

        # 越界写入应被拒绝并置 PARAM_ERR(bit4)
        st_before = cli.get_status()
        cli.set_overload([9.9e9, 2000.0, 3000.0, 150.0, 150.0, 150.0])
        time.sleep(0.05)
        ovl3 = cli.get_overload()
        st_after = cli.get_status()
        check("越界阈值写入被拒绝",
              abs(ovl3[0] - 1000.0) < 1e-3 and (st_after & (1 << 4)) != 0,
              f"回读={ovl3[0]:.3f} 状态=0x{st_after:04X}(前 0x{st_before:04X})")

        # ---------- 6. 写入字节序往返自检 ----------
        winner, table = cli.verify_write_order()
        print("     写入字节序往返：")
        for o, d in table:
            print(f"       {o}: 回读偏差 = {d}")
        want_w = args.order or ByteOrder.WRITE_DEFAULT
        check("写入字节序往返判定", winner == want_w,
              f"判定={winner} 期望={want_w}")
        time.sleep(0.05)
        check("往返自检后阈值未被改动",
              abs(cli.get_overload()[0] - 1000.0) < 1e-3,
              str(cli.get_overload()[:3]))

        # ---------- 7. 输出格式 ----------
        cli.set_format(Format.N)
        time.sleep(0.05)
        check("写/读输出格式 (0x0032)",
              cli.get_format() == Format.N, "N (2)")

        # ---------- 8. 单次触发（优先消费设备回送的单次数据帧） ----------
        cli.clear_wave()
        p0 = cli.push_count
        vals = cli.single_shot()
        finite = [v for v in vals if v is not None]
        check("单次转换返回 6 维数据 (0x0003)",
              len(vals) == 6 and any(abs(v) > 1e-6 for v in finite),
              fmt_vals(vals))
        if args.no_single_push:
            check("设备不回送单次帧时回退轮询读 0x0033",
                  cli.last_single_src == "poll",
                  f"来源={cli.last_single_src}（期望 poll）")
            if args.force_regs < 12:
                check("旧固件：力块读不全时第 6 通道为 None（不编造数值）",
                      vals[5] is None, fmt_vals(vals))
        else:
            check("单次取数走设备回送帧（与连续推流同字节序 DCBA）",
                  cli.last_single_src == "push",
                  f"来源={cli.last_single_src}（期望 push）")
            check("单次帧含完整 Mz（不依赖轮询读）",
                  vals[5] is not None and abs(vals[5]) > 1e-9, fmt_vals(vals))

        # ---------- 8b. 单次返回帧（Modbus-TCP 主动回送） ----------
        # 固件写 0x0003 后除置位单次标志外，还会向 Modbus-TCP 通道回送一帧
        # 33 字节数据帧（TID=0000 / LEN=001B / FC=03 / 6×float32）。
        time.sleep(0.3)
        got = cli.push_count - p0
        if args.no_single_push:
            check("旧固件：单次转换不向 Modbus-TCP 回送帧",
                  got == 0, f"收到 {got} 帧（期望 0 帧）")
        else:
            check("单次转换向 Modbus-TCP 回送一帧数据 (0x0003)",
                  got == 1, f"收到 {got} 帧（期望 1 帧）")
            w_ss = cli.wave_snapshot(5.0)
            check("单次返回帧可解析为六维数据",
                  bool(w_ss) and len(w_ss[-1][1]) == 6
                  and any(abs(x) > 1e-6 for x in w_ss[-1][1]),
                  fmt_vals(w_ss[-1][1]) if w_ss else "—")

        # ---------- 9. 清零 / 取消清零 ----------
        offsets = cli.zero()
        check("自动清零 (0x0030) 并读零点偏移",
              len(offsets) == 6, fmt_vals(offsets))
        cli.set_zero_offsets([1.5, -2.5, 3.5, 0.25, -0.5, 0.75])
        back = cli.get_zero_offsets()
        check("手动写零点偏移并回读",
              abs(back[0] - 1.5) < 1e-3 and abs(back[4] + 0.5) < 1e-3,
              str([round(v, 3) for v in back]))
        cli.unzero()
        time.sleep(0.05)
        back2 = cli.get_zero_offsets()
        check("取消清零 (0x00031)",
              all(abs(v) < 1e-6 for v in back2), str(back2))

        # ---------- 10. 连续推流 ----------
        cli.clear_wave()
        cli.reset_stats()
        cli.start()
        t0 = time.time()
        time.sleep(2.0)
        push = cli.push_count
        wave = cli.wave_snapshot(10.0)
        cli.stop()
        rate = push / max(time.time() - t0, 1e-9)
        check("开始连续转换 (0x0002) 并接收主动推流",
              push > 50 and len(wave) > 50,
              f"收到 {push} 帧，实测 {rate:.1f} Hz，波形缓存 {len(wave)} 点")
        check("推流帧按推流字节序解析",
              bool(wave) and all(len(v) == 6 and all(abs(x) < 1e6 for x in v)
                                 for _, v in wave),
              f"末帧 = {fmt_vals(wave[-1][1]) if wave else '—'}")

        cli.stop()
        time.sleep(0.3)
        c1 = cli.push_count
        time.sleep(0.4)
        check("停止发送 (0x0001) 后推流停止", cli.push_count - c1 <= 2,
              f"停止后新增 {cli.push_count - c1} 帧")

        # ---------- 11. 块尾寄存器的边界行为（P0-1 的直接验证） ----------
        # 修复后：按文档的整段请求都成功，且块尾那一格（力块的 Mz 高字
        #         0x003E / 范围块的 Mz 最大值高字 0x0093）能读到。
        # 旧固件：valid_range() 放行、mb_reg_read() 的 case 少一格 → 异常 0x02。
        if args.force_regs >= 12:
            try:
                tail = cli.read_registers(Reg.FORCE, 12)
                check("按文档读 0x0033 ×12 成功（P0-1 已修复）",
                      len(tail) == 12, f"末位 0x003E = 0x{tail[-1]:04X}")
            except ModbusException as e:
                check("按文档读 0x0033 ×12 成功（P0-1 已修复）", False, str(e))
        else:
            try:
                cli.read_registers(Reg.FORCE, 12)
                check("旧固件：按文档读 0x0033 ×12 被拒", False, "竟然成功了")
            except ModbusException as e:
                check("旧固件：按文档读 0x0033 ×12 被拒", e.code == 0x02, str(e))

        if args.range_regs >= 24:
            try:
                tail = cli.read_registers(Reg.RANGE, 24)
                check("按文档读 0x007C ×24 成功（P0-1 已修复）",
                      len(tail) == 24, f"末位 0x0093 = 0x{tail[-1]:04X}")
            except ModbusException as e:
                check("按文档读 0x007C ×24 成功（P0-1 已修复）", False, str(e))
        else:
            try:
                cli.read_registers(Reg.RANGE, 24)
                check("旧固件：按文档读 0x007C ×24 被拒", False, "竟然成功了")
            except ModbusException as e:
                check("旧固件：按文档读 0x007C ×24 被拒", e.code == 0x02, str(e))

        # ---------- 12. 异常处理 ----------
        try:
            cli.read_registers(0x0201, 1)          # 未定义地址
            check("非法地址返回异常码", False, "未抛出异常")
        except ModbusException as e:
            check("非法地址返回异常码", e.code == 0x02, str(e))
        except ModbusTimeoutError as e:
            check("非法地址返回异常码", False, f"超时而非异常码: {e}")

        # ---------- 12b. Modbus TCP 协议一致性 ----------
        # 直接用原始 PDU 收发，覆盖常规 API 到不了的规范边界。
        def raw(pdu: bytes):
            """发一帧请求，返回完整响应帧（异常帧也原样返回，不抛）。"""
            try:
                return cli._request(pdu)
            except ModbusException as e:
                return b""      # 异常码由调用方另行确认
            except ModbusTimeoutError:
                return None

        def exc_of(pdu: bytes):
            """返回响应的异常码；正常响应返回 0；无响应返回 -1。"""
            try:
                cli._request(pdu)
                return 0
            except ModbusException as e:
                return e.code
            except ModbusTimeoutError:
                return -1

        # (a) 响应帧长度必须与 MBAP 的 LEN 字段自洽：总长 = 6 + LEN
        #     三个功能码各取一个「一定成功且无副作用」的请求：
        #       FC03 读力数据、FC06 写 freq_mode=0（回到默认 500 Hz）、
        #       FC10 把当前 IP 原值写回（NET 块、偶数个寄存器，值不变）
        ip_now = cli.read_registers(Reg.IP, 2)
        for label, pdu in (
                ("FC03", struct.pack(">BHH", 0x03, Reg.FORCE, args.force_regs)),
                ("FC06", struct.pack(">BHH", 0x06, Reg.FREQ_MODE, 0)),
                ("FC10", struct.pack(">BHHBHH", 0x10, Reg.IP, 2, 4,
                                     ip_now[0], ip_now[1]))):
            f = raw(pdu)
            if f:
                declared = (f[4] << 8) | f[5]
                check(f"{label} 响应长度与 LEN 字段自洽（总长 = 6 + LEN）",
                      len(f) == 6 + declared,
                      f"实收 {len(f)} 字节，LEN={declared}")
            else:
                check(f"{label} 响应长度与 LEN 字段自洽（总长 = 6 + LEN）",
                      False, "未收到响应")

        # (b) 不支持的功能码 → 异常 0x01（而不是断连或静默）
        check("不支持的功能码 0x04 → 异常 0x01",
              exc_of(struct.pack(">BHH", 0x04, 0x0000, 1)) == 0x01,
              f"实际 {exc_of(struct.pack('>BHH', 0x04, 0x0000, 1)):#04x}")
        check("不支持的功能码 0x17 → 异常 0x01",
              exc_of(struct.pack(">BHH", 0x17, 0x0000, 1)) == 0x01,
              "FC17(读写多寄存器) 本设备未实现")

        # (c) 数量越界 → 异常 0x03（规范：FC03 数量 1..125）
        check("FC03 数量 0 → 异常 0x03",
              exc_of(struct.pack(">BHH", 0x03, Reg.FORCE, 0)) == 0x03, "")
        check("FC03 数量 126 → 异常 0x03",
              exc_of(struct.pack(">BHH", 0x03, Reg.FORCE, 126)) == 0x03,
              "超出规范上限 125")

        # (d) 地址不在任何块内 → 异常 0x02（块模型）
        check("FC03 跨块请求 → 异常 0x02",
              exc_of(struct.pack(">BHH", 0x03, 0x003E, 2)) == 0x02,
              "0x003E..0x003F 跨块")

        # (e) FC10 的 byte_cnt 必须等于 count×2，否则 0x03
        bad = struct.pack(">BHHBH", 0x10, Reg.OVERLOAD, 1, 4, 0x0001)   # 声称 4 字节只给 2
        check("FC10 字节数不匹配 → 异常 0x03", exc_of(bad) == 0x03, "")

        # (f) 协议 ID ≠ 0 的帧应被丢弃（不应答），且不能把连接带偏
        try:
            cli._sock.sendall(struct.pack(">HHHB", 0x7FFF, 0x0001, 6, 1)
                              + struct.pack(">BHH", 0x03, Reg.FORCE, 1))
            time.sleep(0.25)
            alive = cli.read_registers(Reg.FORCE, 1)
            check("协议 ID ≠ 0 的帧被丢弃且连接不受影响",
                  len(alive) == 1, "后续正常请求仍有响应")
        except (OSError, ModbusException, ModbusTimeoutError) as e:
            check("协议 ID ≠ 0 的帧被丢弃且连接不受影响", False, repr(e))

        # (g) TCP 粘包：把 n 个请求塞进同一个报文段（一次 sendall）发出。
        #     TCP 是字节流，客户端把两个小请求并进同一段是常态 —— 上位机
        #     的「连接后自动探测」跑在后台线程、用户同时操作界面时就会这样。
        #     服务端必须按 MBAP LEN 逐帧切分，把同批请求**全部**应答；
        #     旧固件只解析第一帧，其余静默丢弃 → 上层表现为「等待响应超时」。
        #     这里用**独立连接**测，避免和 cli 的接收线程抢 socket。
        n_sticky = 3
        s2 = socket.create_connection((cli.host, cli.port), timeout=3.0)
        s2.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            s2.sendall(b"".join(
                struct.pack(">HHHBBHH", 0xA000 + i, 0, 6, 1, 0x03,
                            Reg.FORCE, args.force_regs)
                for i in range(1, n_sticky + 1)))
            buf, got = b"", []
            t0 = time.time()
            s2.settimeout(0.3)
            while time.time() - t0 < args.timeout:
                try:
                    chunk = s2.recv(8192)
                except socket.timeout:
                    if got:
                        break
                    continue
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 7:
                    ln = (buf[4] << 8) | buf[5]
                    if len(buf) < 6 + ln:
                        break
                    got.append(((buf[0] << 8) | buf[1], 6 + ln, buf[7]))
                    buf = buf[6 + ln:]
                if len(got) >= n_sticky:
                    break
        finally:
            s2.close()

        tids = [g[0] for g in got]
        want = [0xA001, 0xA002, 0xA003] if not args.legacy_sticky else [0xA001]
        check(f"TCP 粘包：一帧含 {n_sticky} 个请求时{'全部' if not args.legacy_sticky else '仅第一个'}"
              f"应答（TID {'A001~A003' if not args.legacy_sticky else 'A001'}）",
              tids == want,
              f"收到 TID {[hex(t) for t in tids]}，长度 {[g[1] for g in got]}，"
              f"FC {[hex(g[2]) for g in got]}")
        if not args.legacy_sticky and got:
            check("TCP 粘包：同批每个响应都完整且为 FC03",
                  all(g[1] == 9 + args.force_regs * 2 and g[2] == 0x03 for g in got),
                  f"期望每个 {9 + args.force_regs * 2} 字节")

        # ---------- 13. 网络参数写回 ----------
        cli.set_ip("192.168.1.200")
        time.sleep(0.05)
        check("写 IP 并回读 (0x0106)",
              cli.get_network()["ip"] == "192.168.1.200",
              cli.get_network()["ip"])
        cli.set_ip("192.168.1.12")

        # ---------- 14. 统计 ----------
        print(f"\n统计：TX {cli.tx_frame_count} 帧 / RX {cli.rx_frame_count} 帧 / "
              f"错误 {cli.error_count} 次")
    finally:
        cli.close()

    print(f"\n===== 结果：通过 {PASS} 项，失败 {FAIL} 项 =====")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
