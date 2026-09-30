#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hs_mapscan.py — 六维力传感器「真机寄存器地图」扫描器

背景：协议文档（Docs/modbus-tcp-protocol-v2.0.md）、源码树（Core/Inc/mb_register.h）
与设备里实际运行的固件三者可能不一致。上位机若按文档硬编码地址，就会出现
「连接时某条读取报 0x02 非法数据地址」。

本工具只做 FC03 读取（不写任何寄存器，对设备无副作用），输出：

    1. 真实可读地址白名单（按连续区间合并）→ 等价于固件 valid_range() 的白名单
    2. 每个区间的最大可读长度
    3. 各区间原始寄存器值与多种 float32 字节序的解析对照

用法：
    python hs_mapscan.py                        # 默认 192.168.1.12:502
    python hs_mapscan.py 192.168.1.12 502
    python hs_mapscan.py --out result.txt
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time

UNIT_ID = 1

ORDERS = ("ABCD", "CDAB", "BADC", "DCBA")


def order_bytes(a: int, b: int, order: str) -> bytes:
    """把两个寄存器按指定字节序还原成 4 字节。"""
    A = struct.pack(">H", a)
    B = struct.pack(">H", b)
    return {
        "ABCD": A + B,
        "CDAB": B + A,
        "BADC": A[::-1] + B[::-1],
        "DCBA": B[::-1] + A[::-1],
    }[order]


def f32(a: int, b: int, order: str) -> float:
    return struct.unpack(">f", order_bytes(a, b, order))[0]


class Device:
    """极简裸 socket 客户端：单连接、顺序请求、宽松收包。"""

    def __init__(self, host: str, port: int, wait: float = 0.30,
                 quiet: float = 0.05):
        self.host, self.port = host, port
        self.wait, self.quiet = wait, quiet
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.tid = 0

    def reconnect(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
        time.sleep(0.5)
        self.sock = socket.create_connection((self.host, self.port), timeout=3.0)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass

    def read(self, addr: int, count: int, retry: bool = True) -> bytes:
        """FC03 读，返回设备回包原始字节；超时返回 b''。"""
        self.tid = (self.tid % 0xFFFE) + 1
        req = struct.pack(">HHHBBHH", self.tid, 0, 6, UNIT_ID, 0x03, addr, count)
        try:
            self.sock.sendall(req)
        except OSError:
            if not retry:
                return b""
            self.reconnect()
            return self.read(addr, count, retry=False)

        buf = b""
        self.sock.settimeout(self.wait)
        try:
            buf += self.sock.recv(4096)
        except socket.timeout:
            if not retry:
                return b""
            # 设备偶发「哑一下」，重连重试一次
            try:
                self.reconnect()
                return self.read(addr, count, retry=False)
            except OSError:
                return b""
        except OSError:
            if not retry:
                return b""
            try:
                self.reconnect()
                return self.read(addr, count, retry=False)
            except OSError:
                return b""

        self.sock.settimeout(self.quiet)
        while True:
            try:
                chunk = self.sock.recv(4096)
            except (socket.timeout, OSError):
                break
            if not chunk:
                break
            buf += chunk
        return buf

    def read_regs(self, addr: int, count: int):
        """返回 (寄存器列表, 说明)。失败时 regs 为 None。"""
        raw = self.read(addr, count)
        if not raw:
            return None, "无响应"
        if len(raw) < 9:
            return None, f"回包过短({len(raw)}B)"
        fc = raw[7]
        if fc & 0x80:
            return None, f"异常 {raw[8]:02X}"
        if fc != 0x03:
            return None, f"FC={fc:02X}"
        n = raw[8]
        data = raw[9:9 + n]
        if len(data) < 2 * count:
            return None, f"数据不足(声明{n}B)"
        return list(struct.unpack(f">{n // 2}H", data))[:count], "OK"


def merge_ranges(addrs) -> list:
    out = []
    for a in sorted(addrs):
        if out and a == out[-1][1] + 1:
            out[-1][1] = a
        else:
            out.append([a, a])
    return out


def max_count(dev: Device, addr: int, ceiling: int = 125) -> int:
    """二分求该地址起最大可读寄存器数。"""
    lo, hi, best = 1, ceiling, 0
    while lo <= hi:
        mid = (lo + hi) // 2
        regs, _ = dev.read_regs(addr, mid)
        if regs is not None and len(regs) == mid:
            best = mid
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def main() -> int:
    ap = argparse.ArgumentParser(description="真机寄存器地图扫描")
    ap.add_argument("host", nargs="?", default="192.168.1.12")
    ap.add_argument("port", nargs="?", type=int, default=502)
    ap.add_argument("--scan-end", type=lambda s: int(s, 0), default=0x0230,
                    help="扫描上界（含），默认 0x0230")
    ap.add_argument("--out", default="", help="同时把报告写入文本文件")
    args = ap.parse_args()

    lines: list = []

    def emit(text: str = "") -> None:
        print(text)
        lines.append(text)

    try:
        dev = Device(args.host, args.port)
    except OSError as e:
        print(f"连接 {args.host}:{args.port} 失败：{e}")
        return 2

    emit(f"# 真机寄存器地图   {args.host}:{args.port}")
    emit(f"# 扫描时间 {time.strftime('%Y-%m-%d %H:%M:%S')}")
    emit("")

    # ---------- 1. 单寄存器可读性扫描 ----------
    emit(f"== 1. 地址可读性扫描 0x0000 ~ 0x{args.scan_end:04X}（count=1） ==")
    ok_addrs, bad_addrs = [], []
    scan_wait, scan_quiet = dev.wait, dev.quiet
    dev.wait, dev.quiet = 0.18, 0.02          # 扫描阶段用更短超时提速
    total = args.scan_end + 1
    for a in range(0, total):
        regs, _ = dev.read_regs(a, 1)
        (ok_addrs if regs is not None else bad_addrs).append(a)
        if (a + 1) % 64 == 0 or a == total - 1:
            print(f"    ... 已扫描 {a + 1}/{total}（可读 {len(ok_addrs)}）",
                  file=sys.stderr, flush=True)
    dev.wait, dev.quiet = scan_wait, scan_quiet
    ranges = merge_ranges(ok_addrs)

    emit(f"可读 {len(ok_addrs)} 个 / 不可读 {len(bad_addrs)} 个")
    emit("")
    emit("== 2. 可读区间（连续合并）与最大可读长度 ==")
    report = []
    for lo, hi in ranges:
        n = max_count(dev, lo, min(125, hi - lo + 1))
        span = hi - lo + 1
        flag = "" if n == span else f"  <-- 整段可读 {span}，但一次最多读 {n}"
        emit(f"  0x{lo:04X} ~ 0x{hi:04X}  ({span:>3} 个寄存器)  单次最大可读 {n:>3}{flag}")
        report.append({"start": lo, "end": hi, "span": span, "max_batch": n})

    # ---------- 3. 关键区段内容解析 ----------
    emit("")
    emit("== 3. 关键区段寄存器内容与字节序对照 ==")
    dumps = [
        (0x0005, 8, "设备信息 SN"),
        (0x000D, 4, "固件版本/状态字/错误计数"),
        (0x0030, 14, "控制 + 六维力数据区"),
        (0x0050, 12, "零点偏移"),
        (0x0070, 12, "过载阈值"),
        (0x007C, 23, "阈值允许范围（实际可读长度）"),
        (0x0100, 12, "网络参数"),
        (0x0210, 6, "自检扩展"),
    ]
    for addr, count, title in dumps:
        regs, why = dev.read_regs(addr, count)
        emit("")
        emit(f"-- {title}  0x{addr:04X} x{count} -> {why}")
        if regs is None:
            continue
        emit("   reg  : " + " ".join(f"{v:04X}" for v in regs))
        if count % 2 == 0 or len(regs) >= 2:
            for o in ORDERS:
                vals = []
                for i in range(0, len(regs) - 1, 2):
                    vals.append(f32(regs[i], regs[i + 1], o))
                emit(f"   {o} : " + ", ".join(f"{v:>12.5g}" for v in vals))

    # ---------- 4. 边界确认 ----------
    emit("")
    emit("== 4. 关键边界确认 ==")
    for addr, count in [(0x0033, 11), (0x0033, 12), (0x0030, 14), (0x0030, 15),
                        (0x007C, 23), (0x007C, 24), (0x0092, 1), (0x0093, 1),
                        (0x003D, 1), (0x003E, 1)]:
        regs, why = dev.read_regs(addr, count)
        emit(f"  0x{addr:04X} x{count:<3} -> {why}")

    dev.close()

    payload = {"host": args.host, "port": args.port,
               "ranges": report, "readable": ok_addrs}
    with open("_mapscan_result.json", "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
    emit("")
    emit("原始结果已写入 _mapscan_result.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
