#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hs_probe.py — 六维力传感器 Modbus-TCP 真机诊断探针

用途：当上位机报「设备异常」或数据不对时，脱离上位机、用原始字节直接探测设备，
逐条定位是「哪个地址被拒 / 哪条响应长度不对 / 字节序是什么」。

只做 FC03 读取（不写任何寄存器），对设备无副作用。

用法：
    python hs_probe.py                       # 默认 192.168.1.12:502
    python hs_probe.py 192.168.1.12 502
    python hs_probe.py --raw 0x007C 24       # 单独探测任意地址
"""

from __future__ import annotations

import socket
import struct
import sys
import time

HOST_DEFAULT = "192.168.1.12"
PORT_DEFAULT = 502
UNIT_ID = 1

EXC_TEXT = {
    0x01: "非法功能码",
    0x02: "非法数据地址",
    0x03: "非法数据值",
    0x04: "从站设备故障",
    0x05: "确认(处理中)",
    0x06: "从站设备忙",
}


def build_read(tid: int, addr: int, count: int) -> bytes:
    """MBAP + PDU：FC03 读保持寄存器。"""
    return struct.pack(">HHHBBHH", tid, 0, 6, UNIT_ID, 0x03, addr, count)


def drain(sock: socket.socket, first_wait: float = 0.25,
          quiet: float = 0.12) -> bytes:
    """发送后收集设备返回的全部字节（读到静默为止），容忍长度字段错误。"""
    buf = b""
    sock.settimeout(first_wait)
    try:
        buf += sock.recv(4096)
    except socket.timeout:
        return buf
    sock.settimeout(quiet)
    while True:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
    return buf


def parse_frame(raw: bytes) -> dict:
    """宽松解析一帧，长度字段单独报告（不据此切分）。"""
    if len(raw) < 8:
        return {"ok": False, "why": f"回包过短（{len(raw)} 字节）", "hex": raw.hex(" ")}

    tid, pid, length, uid, fc = struct.unpack(">HHHBB", raw[:8])
    info = {
        "tid": tid, "pid": pid, "len_field": length, "uid": uid, "fc": fc,
        "total": len(raw),
        # 标准约定：LEN 应等于后面所有字节数 = 总长 - 6
        "len_should": len(raw) - 6,
        "hex": raw.hex(" "),
    }

    if fc & 0x80:
        code = raw[8] if len(raw) > 8 else 0
        info.update(ok=False, exception=code,
                    why=f"异常 {code:02X} ({EXC_TEXT.get(code, '未知')})")
        return info

    if fc != 0x03:
        info.update(ok=False, why=f"功能码不是 0x03（收到 0x{fc:02X}）")
        return info

    byte_cnt = raw[8] if len(raw) > 8 else 0
    data = raw[9:9 + byte_cnt]
    info["byte_cnt"] = byte_cnt
    info["data_bytes"] = len(data)
    if len(data) < byte_cnt:
        info.update(ok=False,
                    why=f"数据不足：声明 {byte_cnt} 字节，实到 {len(data)} 字节")
        return info
    if len(raw) != 9 + byte_cnt:
        info.update(ok=False,
                    why=f"帧长不一致：MBAP声明 LEN={length}（应={len(raw)-6}），"
                        f"实发 {len(raw)} 字节 = 9+{byte_cnt}"
                        f"（多出 {len(raw)-9-byte_cnt} 字节）")
        return info

    info.update(ok=True, regs=list(struct.unpack(f">{byte_cnt // 2}H", data)))
    return info


def probe(sock: socket.socket, tid: int, addr: int, count: int, label: str) -> dict:
    sock.sendall(build_read(tid, addr, count))
    raw = drain(sock)
    res = parse_frame(raw) if raw else {"ok": False, "why": "无响应（超时）", "hex": ""}
    res.update(addr=addr, count=count, label=label)
    return res


def as_float(regs, order="CDAB"):
    """把 2 个寄存器按指定字节序还原 float32。"""
    if regs is None or len(regs) < 2:
        return None
    a, b = regs[0], regs[1]
    import struct as _s
    A = _s.pack(">H", a)
    B = _s.pack(">H", b)
    if order == "ABCD":
        p = A + B
    elif order == "CDAB":          # 低字在前 + 每字大端（固件 FC03 读路径）
        p = B + A
    elif order == "BADC":
        p = A[::-1] + B[::-1]
    else:                          # DCBA
        p = B[::-1] + A[::-1]
    return _s.unpack(">f", p)[0]


def probe_burst(host: str, port: int, n: int = 3) -> int:
    """
    TCP 粘包探测（对应缺陷 P0-4）。

    把 n 个 FC03 请求**一次性**发出去（同一个 TCP 报文段），数一数收到几个响应。
    已修复的固件应逐个应答 → 返回 n；
    未修复的固件把整段当「一个请求」、只解析第一帧 → 恒返回 1。

    用独立连接，避免打扰调用方已有的会话。
    """
    s = socket.create_connection((host, port), timeout=3.0)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        s.sendall(b"".join(build_read(0xB000 + i, 0x0033, 12)
                           for i in range(1, n + 1)))
        buf, got = bytearray(), 0
        t0 = time.time()
        s.settimeout(0.3)
        while time.time() - t0 < 2.0:
            try:
                chunk = s.recv(4096)
            except socket.timeout:
                if got:
                    break
                continue
            if not chunk:
                break
            buf += chunk
            while len(buf) >= 7:
                length = struct.unpack(">H", buf[4:6])[0]
                if length < 2 or 6 + length > 260:
                    del buf[:1]
                    continue
                if len(buf) < 6 + length:
                    break
                del buf[:6 + length]
                got += 1
            if got >= n:
                break
        return got
    finally:
        s.close()


BLOCKS = [
    (0x0005, 8, "SN 序列号"),
    (0x000D, 2, "固件版本"),
    (0x000F, 1, "系统状态字"),
    (0x0010, 1, "通信错误计数"),
    (0x0032, 1, "输出格式"),
    (0x0033, 12, "六维力数据"),
    (0x0050, 12, "零点偏移"),
    (0x0070, 12, "过载阈值"),
    (0x007C, 24, "阈值允许范围"),
    (0x0100, 12, "网络参数 MAC/IP/掩码/网关"),
    (0x0200, 1, "采样频率模式"),
    (0x0210, 6, "自检扩展寄存器"),
]


def main() -> int:
    args = sys.argv[1:]
    if args and args[0] == "--raw":
        host, port = HOST_DEFAULT, PORT_DEFAULT
        addr, count = int(args[1], 0), int(args[2])
        targets = [(addr, count, "自定义")]
    else:
        host = args[0] if len(args) > 0 else HOST_DEFAULT
        port = int(args[1]) if len(args) > 1 else PORT_DEFAULT
        targets = BLOCKS

    try:
        sock = socket.create_connection((host, port), timeout=3.0)
    except OSError as e:
        print(f"连接 {host}:{port} 失败：{e}")
        return 2

    print(f"设备 {host}:{port}   探测 {len(targets)} 项（仅 FC03 读取）")
    print("=" * 78)

    tid = 1
    bad = 0
    for addr, count, label in targets:
        try:
            res = probe(sock, tid, addr, count, label)
        except OSError as e:
            print(f"[{label:<22}] 0x{addr:04X} x{count:<3} 通信中断：{e}")
            sock = socket.create_connection((host, port), timeout=3.0)
            res = probe(sock, tid, addr, count, label)
        tid += 1

        head = f"[{label:<22}] 0x{addr:04X} x{count:<3}"
        if res["ok"]:
            note = ""
            if res["len_field"] != res["len_should"]:
                note = (f"  ⚠ MBAP长度={res['len_field']} 应为 {res['len_should']}"
                        f"（多出 {res['total'] - 6 - res['len_field']} 字节）")
            print(f"{head} OK   {res['total']:>3} 字节  {note}")
        else:
            bad += 1
            print(f"{head} ✗ {res['why']}")
        print(f"{'':<26}RX: {res['hex']}")

        regs = res.get("regs")
        if regs and count >= 2 and count % 2 == 0:
            vals = [as_float(regs[i:i + 2], "CDAB") for i in range(0, len(regs) - 1, 2)]
            shown = ", ".join(f"{v:.4g}" if v is not None else "?" for v in vals[:6])
            print(f"{'':<26}CDAB 解析: {shown}")
        elif len(regs or []) == 1:
            print(f"{'':<26}值: 0x{regs[0]:04X} ({regs[0]})")

        time.sleep(0.05)

    sock.close()
    print("=" * 78)
    print(f"完成：{len(targets) - bad}/{len(targets)} 项正常")

    # ---- TCP 粘包探测（缺陷 P0-4）----
    # 单独读一条协议没问题，不代表连续/并发场景没问题：真机上「读取数据都有
    # 问题」正是出在这里。把 3 个请求塞进同一个报文段，看设备是否逐个应答。
    n_req = 3
    got = probe_burst(host, port, n_req)
    print("-" * 78)
    if got >= n_req:
        print(f"[TCP 粘包探测] 一帧发出 {n_req} 个请求 → 收到 {got} 个响应   正常")
    elif got == 1:
        print(f"[TCP 粘包探测] 一帧发出 {n_req} 个请求 → 只收到 {got} 个响应   "
              f"✗ 缺陷 P0-4")
        print("               固件只处理同批的第一个请求，其余被静默丢弃；")
        print("               需烧入含 P0-4 修复的固件（build/Verify/hs.hex，")
        print("               副本 hs_v1_0_fix_tcp_sticky.hex）。")
    else:
        print(f"[TCP 粘包探测] 一帧发出 {n_req} 个请求 → 收到 {got} 个响应（异常）")
    print("=" * 78)

    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
