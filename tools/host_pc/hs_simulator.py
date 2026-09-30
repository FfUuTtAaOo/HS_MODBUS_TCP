# -*- coding: utf-8 -*-
"""
hs_simulator.py —— 六维力传感器 Modbus-TCP 设备模拟器（纯标准库）

用途：
  * 没有硬件时验证上位机（hs_app.py）的功能与界面；
  * 作为协议实现的对照参考（寄存器映射、字节序、推流帧格式）。

行为与固件 Core/Src/mb_register.c + modbus_tcp.c 保持一致：

  * 保持寄存器 FC 03 / 06 / 10，异常码 0x02 / 0x03；
  * 读地址必须整体落在某个寄存器块内，否则返回 0x02；
  * 写 0x0002 后按 freq_mode 频率主动推送 33 字节数据帧
        TID=0000, PID=0000, LEN=001B, UID=01, FC=03, BYTE_CNT=18, 6×float32
    推送给所有已连接客户端（与固件一致）；
  * 写 0x0003（单次转换）后**先回 FC06 应答，再主动回送一帧数据**，
    随后自动回到停止状态 —— 单次返回帧与连续推流帧格式完全相同
    （与固件 main.c 的 send_mode == 2 分支一致）；
    `--legacy-single` 复刻修复前的固件：该帧只发 RS485，TCP 侧无回帧；
  * **三条路径三种 float32 字节序**（与固件一致，默认值）：
        FC03 读   → CDAB（先低字，字内大端）
        FC10 写   → DCBA（buf_to_float 按小端解释数据字节）
        主动推流  → DCBA（标准小端）
  * 寄存器块边界与 mb_register.h 的 MB_BLK_* 宏一致（CTRL 到 0x003E、
    RANGE 到 0x0093），也就是 **P0-1 修复之后**的形态。

用法：
    python hs_simulator.py                       # 默认：固件已修复（块尾 0x003E/0x0093）
    python hs_simulator.py --legacy-blocks       # 复刻未修复固件（Mz 读不全）
    python hs_simulator.py --legacy-single       # 复刻未修复固件（单次帧不发 TCP）
    python hs_simulator.py --legacy-blocks --legacy-single    # 完全复刻旧固件
    python hs_simulator.py --order ABCD          # 三条路径统一为 ABCD（参考实现）
    python hs_simulator.py --port 15020 --rate 100
"""

from __future__ import annotations

import argparse
import math
import random
import socket
import struct
import threading
import time
from typing import Dict, List, Optional, Tuple

from hs_modbus import ByteOrder, Reg, bytes_to_float, float_to_bytes

# =====================================================================
#  寄存器块定义
# =====================================================================
# 与 mb_register.h 的 MB_BLK_* 宏一致（_END 为 inclusive 的末地址）。
# 这就是 P0-1 修复后的正确形态：
#     CTRL  到 0x003E —— 含 Mz 的高 16 位，力块共 12 个寄存器
#     RANGE 到 0x0093 —— 含 Mz 最大值的高 16 位，范围块共 24 个寄存器
BLOCKS: List[Tuple[int, int]] = [
    (0x0001, 0x0003),      # STOP / START / SINGLE
    (0x0005, 0x0010),      # SN + FW + STATUS + COMM_ERR
    (0x0030, 0x003E),      # ZERO / UNZERO / FORMAT / FORCE ×6 = 12 寄存器
    (0x0050, 0x005B),      # 零点偏移
    (0x0070, 0x007B),      # 过载阈值
    (0x007C, 0x0093),      # 阈值范围 6×(min+max) = 24 寄存器
    (0x0100, 0x010B),      # 网络参数
    (0x0200, 0x0200),      # freq_mode
    (0x0210, 0x0215),      # 自检
]

# 未修复固件的形态（--legacy-blocks 复刻，用于回归旧固件）：
# mb_reg_read() 的 case 范围比块尾各少一格，于是整段请求虽然通过了
# valid_range()，末位寄存器却落到 default → 异常 0x02，Mz 读不全。
BLOCKS_LEGACY: List[Tuple[int, int]] = list(BLOCKS)
BLOCKS_LEGACY[2] = (0x0030, 0x003D)      # 缺 Mz 高字 0x003E
BLOCKS_LEGACY[5] = (0x007C, 0x0092)      # 缺 Mz 最大值高字 0x0093

EX_ILLEGAL_FC = 0x01
EX_ILLEGAL_ADDR = 0x02
EX_ILLEGAL_DATA = 0x03


class DeviceState:
    """传感器的寄存器与运行状态模型。"""

    def __init__(self, poll_order: str = ByteOrder.POLL_DEFAULT,
                 push_order: str = ByteOrder.PUSH_DEFAULT,
                 write_order: str = ByteOrder.WRITE_DEFAULT,
                 rate: float = 500.0):
        # 与固件一致：FC03 读、FC10 写、主动推流 三条路径字节序互不相同
        self.poll_order = poll_order
        self.push_order = push_order
        self.write_order = write_order
        self.rate = rate
        self.lock = threading.RLock()

        # ---- 只读信息 ----
        self.sn = "HS-01234567"
        self.fw_version = 0x0100                     # v1.00 (BCD)
        self.status = 0x0000
        self.comm_error = 0

        # ---- 控制 ----
        self.data_format = 2                         # 默认 N
        self.freq_mode = 0                           # 0 = 500Hz
        self.send_mode = 0                           # 0 = 停, 1 = 连续, 2 = 单次

        # ---- 数据 ----
        self.force = [0.0] * 6
        self.force_zero = [0.0] * 6
        self.zero_busy_until = 0.0

        # ---- 阈值 / 范围（出厂默认，见协议 3.4.3 / 3.4.4）----
        self.overload = [2000.0, 2000.0, 5000.0, 200.0, 200.0, 200.0]
        self.range_min = [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]
        self.range_max = [50000.0, 50000.0, 100000.0, 5000.0, 5000.0, 5000.0]

        # ---- 网络参数（与 main.c 默认值一致）----
        self.mac = [0x00, 0x08, 0xDC, 0x01, 0x02, 0x03]
        self.ip = [192, 168, 1, 12]
        self.subnet = [255, 255, 255, 0]
        self.gateway = [192, 168, 1, 1]

        self.frame_seq = 0
        self._t0 = time.time()

    # ------------------------------------------------------------------
    def generate_sample(self) -> List[float]:
        """生成一组模拟的六维力数据（已扣除零点偏移）。"""
        with self.lock:
            fmt = self.data_format
            zero = list(self.force_zero)
        t = time.time() - self._t0

        fx = 12.0 * math.sin(2 * math.pi * 0.30 * t) + 2.0 * math.sin(2 * math.pi * 1.7 * t)
        fy = 8.0 * math.sin(2 * math.pi * 0.50 * t + 1.0)
        fz = 150.0 + 20.0 * math.sin(2 * math.pi * 0.22 * t)
        mx = 3.0 * math.sin(2 * math.pi * 0.41 * t + 0.5)
        my = 2.0 * math.sin(2 * math.pi * 0.63 * t + 2.1)
        mz = 1.5 * math.sin(2 * math.pi * 0.87 * t + 0.9)
        vals = [fx, fy, fz, mx, my, mz]
        vals = [v + random.gauss(0.0, abs(v) * 0.002 + 0.01) for v in vals]

        if fmt == 0:                                  # mV：原始电压，不扣零点
            return [v * 0.05 for v in vals]
        if fmt == 1:                                  # kg / kg·m
            vals = [v / 9.80665 for v in vals]
        return [v - z for v, z in zip(vals, zero)]    # N 格式扣除零点偏移

    def update_force(self) -> None:
        """由推流线程周期性刷新 g_sensor.force。"""
        vals = self.generate_sample()
        with self.lock:
            self.force = vals
            # 过载判断（仅告警，不触发硬件保护）
            for i, v in enumerate(vals):
                if abs(v) >= abs(self.overload[i]):
                    self.status |= (1 << 3)
                    break
            else:
                self.status &= ~(1 << 3)


class DeviceServer:
    """Modbus-TCP 服务器模拟器。"""

    def __init__(self, host: str = "0.0.0.0", port: int = 502,
                 order: Optional[str] = None, rate: float = 500.0,
                 verbose: bool = True, legacy_blocks: bool = False,
                 legacy_single: bool = False,
                 legacy_sticky: bool = False,
                 push_order: Optional[str] = None,
                 write_order: Optional[str] = None):
        self.host = host
        self.port = port
        # 传了 order 就把三条路径统一（参考实现模式）；否则用固件真实的三套
        self.dev = DeviceState(
            poll_order=order or ByteOrder.POLL_DEFAULT,
            push_order=(order or push_order or ByteOrder.PUSH_DEFAULT),
            write_order=(order or write_order or ByteOrder.WRITE_DEFAULT),
            rate=rate)
        self.blocks = BLOCKS_LEGACY if legacy_blocks else BLOCKS
        # 旧固件（P1-6 修复前）：写 0x0003 的那一帧只发到 RS485，TCP 客户端收不到
        self.legacy_single = legacy_single
        # 旧固件（TCP 粘包修复前）：一次 recv 到的整段被当成「一个请求」，
        # 只处理第一帧，同批到达的第 2..N 个请求被静默丢弃
        self.legacy_sticky = legacy_sticky
        self.verbose = verbose
        self.clients: List[socket.socket] = []
        self.clients_lock = threading.Lock()
        self.running = False
        self._srv: Optional[socket.socket] = None
        # 单次转换（写 0x0003）触发标记：应答发完之后再回送那一帧数据
        self._single_shot_pending = False

    # ------------------------------------------------------------------
    def log(self, msg: str) -> None:
        if self.verbose:
            print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)

    def start(self) -> None:
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(8)
        self.running = True
        self.log(f"模拟传感器已启动：{self.host}:{self.port}  "
                 f"读={self.dev.poll_order} 写={self.dev.write_order} "
                 f"推流={self.dev.push_order}  频率={self.dev.rate:g}Hz  "
                 f"块尾={'已修复(0x003E/0x0093)' if self.blocks is BLOCKS else '旧固件(0x003D/0x0092)'}")
        threading.Thread(target=self._push_loop, name="sim-push", daemon=True).start()
        try:
            while self.running:
                try:
                    conn, addr = self._srv.accept()
                except OSError:
                    break
                self.log(f"客户端接入 {addr[0]}:{addr[1]}")
                with self.clients_lock:
                    self.clients.append(conn)
                threading.Thread(target=self._serve_client, args=(conn, addr),
                                 name=f"sim-cli-{addr[1]}", daemon=True).start()
        finally:
            self.stop()

    def stop(self) -> None:
        self.running = False
        if self._srv:
            try:
                self._srv.close()
            except OSError:
                pass
        with self.clients_lock:
            for c in self.clients:
                try:
                    c.close()
                except OSError:
                    pass
            self.clients.clear()

    # ------------------------------------------------------------------
    def _push_loop(self) -> None:
        """连续模式下向所有客户端主动推送数据帧。"""
        while self.running:
            time.sleep(0.002)
            with self.dev.lock:
                active = self.dev.send_mode == 1
            if not active:
                continue

            interval = 1.0 / max(self.dev.rate, 1.0)
            t_next = time.perf_counter()
            while self.running:
                with self.dev.lock:
                    if self.dev.send_mode != 1:
                        break
                # 每一帧都重新生成一次力数据。真机是每个采样周期都重新采集的，
                # 若只在进入推流时算一次，整段推流会推出同一个值
                # （波形变成直线、保存下来的 Excel 每行都一样）。
                self.dev.update_force()
                with self.dev.lock:
                    seq = self.dev.frame_seq
                    self.dev.frame_seq += 1
                    vals = list(self.dev.force)
                frame = self._build_push_frame(seq, vals)
                self._broadcast(frame)
                t_next += interval
                delay = t_next - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
                else:
                    t_next = time.perf_counter()

    def _build_push_frame(self, seq: int, vals: List[float]) -> bytes:
        payload = b"".join(float_to_bytes(v, self.dev.push_order) for v in vals)
        return (struct.pack(">HHHBB", 0x0000, 0x0000, 0x001B, 0x01, 0x03)
                + bytes([len(payload)]) + payload)          # 33 字节

    def _emit_single_shot(self) -> None:
        """单次转换：向所有已连接客户端回送一帧数据，随后回到停止状态。

        对应固件 main.c 中 `send_mode == 2` 且 `output_interface == 2` 的分支。
        帧格式与连续推流帧完全相同（TID=0000 / LEN=001B / FC=03 / 24 字节）。
        """
        self.dev.update_force()
        with self.dev.lock:
            seq = self.dev.frame_seq
            self.dev.frame_seq += 1
            vals = list(self.dev.force)
            self.dev.send_mode = 0           # 单次发完自动归零
        self._broadcast(self._build_push_frame(seq, vals))

    def _broadcast(self, frame: bytes) -> None:
        with self.clients_lock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(frame)
                except OSError:
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)
                try:
                    c.close()
                except OSError:
                    pass

    # ------------------------------------------------------------------
    def _serve_client(self, conn: socket.socket, addr) -> None:
        conn.settimeout(0.5)
        buf = bytearray()
        try:
            while self.running:
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buf.extend(chunk)
                while len(buf) >= 7:
                    tid, pid, length = struct.unpack(">HHH", bytes(buf[:6]))
                    total = 6 + length
                    if length < 2 or total > 260:
                        del buf[:1]
                        continue
                    if len(buf) < total:
                        break
                    frame = bytes(buf[:total])
                    del buf[:total]
                    # 与固件一致：协议 ID 不为 0 的帧直接丢弃，不应答
                    if pid != 0:
                        continue
                    resp = self._handle_request(tid, frame)
                    if resp:
                        try:
                            conn.sendall(resp)
                        except OSError:
                            return
                    # 单次转换：FC06 应答发完之后，再回送那一帧数据
                    if self._single_shot_pending:
                        self._single_shot_pending = False
                        self._emit_single_shot()
                    if self.legacy_sticky:
                        # 复刻「TCP 粘包修复」之前的固件：固件把一次 recv
                        # 取回的整段缓冲当作一个请求交给协议引擎，协议引擎
                        # 只解析第一帧 —— 同批到达的其余请求全部被静默丢弃。
                        del buf[:]
                        break
        finally:
            with self.clients_lock:
                if conn in self.clients:
                    self.clients.remove(conn)
            try:
                conn.close()
            except OSError:
                pass
            self.log(f"客户端断开 {addr[0]}:{addr[1]}")

    # ------------------------------------------------------------------
    def _handle_request(self, tid: int, frame: bytes) -> Optional[bytes]:
        uid = frame[6]
        fc = frame[7]
        pdu = frame[8:]

        if fc == 0x03:
            if len(pdu) < 4:
                return self._exception(tid, uid, fc, EX_ILLEGAL_DATA)
            addr, count = struct.unpack(">HH", pdu[:4])
            data, exc = self._read_regs(addr, count)
            if exc:
                return self._exception(tid, uid, fc, exc)
            body = bytes([fc, len(data)]) + data
            return struct.pack(">HHHB", tid, 0, len(body) + 1, uid) + body

        if fc == 0x06:
            if len(pdu) < 4:
                return self._exception(tid, uid, fc, EX_ILLEGAL_DATA)
            addr, value = struct.unpack(">HH", pdu[:4])
            exc = self._write_single(addr, value)
            if exc:
                return self._exception(tid, uid, fc, exc)
            body = bytes([fc]) + pdu[:4]
            return struct.pack(">HHHB", tid, 0, len(body) + 1, uid) + body

        if fc == 0x10:
            if len(pdu) < 5:
                return self._exception(tid, uid, fc, EX_ILLEGAL_DATA)
            addr, count, byte_cnt = struct.unpack(">HHB", pdu[:5])
            if byte_cnt != count * 2:
                return self._exception(tid, uid, fc, EX_ILLEGAL_DATA)
            exc = self._write_multi(addr, count, pdu[5:5 + byte_cnt])
            if exc:
                return self._exception(tid, uid, fc, exc)
            body = bytes([fc]) + struct.pack(">HH", addr, count)
            return struct.pack(">HHHB", tid, 0, len(body) + 1, uid) + body

        return self._exception(tid, uid, fc, EX_ILLEGAL_FC)

    @staticmethod
    def _exception(tid, uid, fc, exc) -> bytes:
        body = bytes([fc | 0x80, exc])
        return struct.pack(">HHHB", tid, 0, len(body) + 1, uid) + body

    def _valid_range(self, addr: int, count: int) -> int:
        """
        与固件 `mb_reg_read()` 前面的 `valid_range()` 完全一致，**返回异常码**（0 = 通过）：

          * count == 0 或 count > 125 → 0x03（非法数据值）—— 注意固件在这里给的是
            0x03 而不是 0x02，早期模拟器统一返回 0x02 属于保真度缺口，已修正；
          * 地址未整体落在同一个寄存器块内 → 0x02（非法数据地址）。
        """
        if count == 0 or count > 125:
            return EX_ILLEGAL_DATA
        end = addr + count - 1
        if any(addr >= s and end <= e for s, e in self.blocks):
            return 0
        return EX_ILLEGAL_ADDR

    # ------------------------------------------------------------------
    def _append_float_reg(self, out: bytearray, sub: int, value: float) -> None:
        """
        追加 1 个寄存器（2 字节）的 float32 片段。

        与固件一致：float32 占 2 个寄存器，sub=0 输出**线上字节序列**的前 2 字节，
        sub=1 输出后 2 字节 —— 配合 poll_order=CDAB 时正好是「低字在前、字内大端」，
        与固件 mb_reg_read() 的行为完全一致。
        """
        raw = float_to_bytes(value, self.dev.poll_order)
        out += raw[sub * 2:sub * 2 + 2]

    def _read_regs(self, addr: int, count: int) -> Tuple[bytes, int]:
        exc = self._valid_range(addr, count)
        if exc:
            return b"", exc
        d = self.dev
        out = bytearray()
        with d.lock:
            # 真实设备一直在采样，读力数据区前刷新一次；
            # 否则「没启动推流就直接轮询」会读到初值 0（与真机不符）。
            if addr < Reg.FORCE + Reg.FORCE_COUNT and addr + count > Reg.FORCE:
                d.update_force()
            for i in range(count):
                a = addr + i
                if a in (Reg.STOP, Reg.START, Reg.SINGLE,
                         Reg.ZERO_TRIG, Reg.UNZERO_TRIG):
                    out += b"\x00\x00"
                elif Reg.SN <= a < Reg.SN + Reg.SN_COUNT:
                    idx = (a - Reg.SN) * 2
                    chars = (d.sn + "\x00" * 16)[:16]
                    out += bytes([ord(chars[idx]), ord(chars[idx + 1])])
                elif a == Reg.FW_VERSION:
                    out += bytes([0x00, (d.fw_version >> 8) & 0xFF])
                elif a == Reg.FW_VERSION + 1:
                    out += bytes([0x00, d.fw_version & 0xFF])
                elif a == Reg.STATUS:
                    out += struct.pack(">H", d.status)
                elif a == Reg.COMM_ERR:
                    out += struct.pack(">H", d.comm_error)
                elif a == Reg.DATA_FORMAT:
                    out += struct.pack(">H", d.data_format)
                elif Reg.FORCE <= a < Reg.FORCE + Reg.FORCE_COUNT:
                    off = a - Reg.FORCE
                    ch = off // 2
                    val = d.force[ch] if ch < 6 else 0.0
                    self._append_float_reg(out, off % 2, val)
                elif Reg.ZERO_OFFSET <= a < Reg.ZERO_OFFSET + Reg.ZERO_OFFSET_COUNT:
                    off = a - Reg.ZERO_OFFSET
                    ch = off // 2
                    val = d.force_zero[ch] if ch < 6 else 0.0
                    self._append_float_reg(out, off % 2, val)
                elif Reg.OVERLOAD <= a < Reg.OVERLOAD + Reg.OVERLOAD_COUNT:
                    off = a - Reg.OVERLOAD
                    ch = off // 2
                    val = d.overload[ch] if ch < 6 else 0.0
                    self._append_float_reg(out, off % 2, val)
                elif Reg.RANGE <= a < Reg.RANGE + Reg.RANGE_COUNT:
                    off = a - Reg.RANGE
                    ch, sub = off // 4, off % 4
                    val = (d.range_min[ch] if sub < 2 else d.range_max[ch]) \
                        if ch < 6 else 0.0
                    self._append_float_reg(out, sub % 2, val)
                elif Reg.MAC <= a < Reg.MAC + 3:
                    k = (a - Reg.MAC) * 2
                    out += bytes([d.mac[k], d.mac[k + 1]])
                elif Reg.MAC + 3 <= a < Reg.IP:
                    out += b"\x00\x00"
                elif Reg.IP <= a < Reg.IP + 2:
                    k = (a - Reg.IP) * 2
                    out += bytes([d.ip[k], d.ip[k + 1]])
                elif Reg.SUBNET <= a < Reg.SUBNET + 2:
                    k = (a - Reg.SUBNET) * 2
                    out += bytes([d.subnet[k], d.subnet[k + 1]])
                elif Reg.GATEWAY <= a < Reg.GATEWAY + 2:
                    k = (a - Reg.GATEWAY) * 2
                    out += bytes([d.gateway[k], d.gateway[k + 1]])
                elif a == Reg.FREQ_MODE:
                    out += struct.pack(">H", d.freq_mode)
                elif a == Reg.ST_ERROR:
                    out += b"\x00\x00"
                elif a == Reg.ST_ERROR_HI:
                    out += b"\x00\x00"
                elif a == Reg.ST_ADC_ID:
                    out += bytes([0x0B, 0x00])
                elif a == Reg.ST_W5500:
                    out += bytes([0x04, 0x84])
                elif a == Reg.ST_RS485:
                    out += bytes([0xC0, 0x00])
                elif a == Reg.ST_FLASH:
                    out += bytes([0xF0, 0x00])
                else:
                    return b"", EX_ILLEGAL_ADDR
        return bytes(out), 0

    # ------------------------------------------------------------------
    def _write_single(self, addr: int, value: int) -> int:
        d = self.dev
        with d.lock:
            if addr == Reg.STOP:
                d.send_mode = 0
            elif addr == Reg.START:
                d.send_mode = 1
            elif addr == Reg.SINGLE:
                # 复刻固件：写 0x0003 → send_mode = 2，
                # 主循环向当前输出通道回送一帧后自动归零。
                # 真机默认 output_interface = 2（Modbus-TCP），这一帧发给所有 TCP 客户端。
                # --legacy-single 复刻修复前的固件：该帧只发到 RS485，TCP 侧没有回帧。
                d.send_mode = 0 if self.legacy_single else 2
                self._single_shot_pending = not self.legacy_single
            elif addr == Reg.ZERO_TRIG:
                d.force_zero = list(d.force)
            elif addr == Reg.UNZERO_TRIG:
                d.force_zero = [0.0] * 6
            elif addr == Reg.DATA_FORMAT:
                if value > 2:
                    return EX_ILLEGAL_DATA
                d.data_format = value
            elif addr == Reg.STATUS:
                d.status = value
            elif addr == Reg.FREQ_MODE:
                if value > 1:
                    return EX_ILLEGAL_DATA
                d.freq_mode = value
            else:
                return EX_ILLEGAL_ADDR
        return 0

    def _write_multi(self, addr: int, count: int, data: bytes) -> int:
        exc = self._valid_range(addr, count)
        if exc:
            return exc
        if count % 2 != 0 and not (Reg.MAC <= addr < Reg.IP):
            return EX_ILLEGAL_DATA
        d = self.dev

        def f32(off: int) -> float:
            # 与固件 buf_to_float() 一致：按**写入字节序**解释 PDU 数据字节
            return bytes_to_float(data[off:off + 4], d.write_order)

        with d.lock:
            if addr >= Reg.ZERO_OFFSET and addr < Reg.ZERO_OFFSET + 12:
                for i in range(count // 2):
                    ch = (addr - Reg.ZERO_OFFSET) // 2 + i
                    if ch < 6:
                        d.force_zero[ch] = f32(i * 4)
            elif addr >= Reg.OVERLOAD and addr < Reg.OVERLOAD + 12:
                for i in range(count // 2):
                    ch = (addr - Reg.OVERLOAD) // 2 + i
                    if ch >= 6:
                        break
                    val = f32(i * 4)
                    if not (d.range_min[ch] <= val <= d.range_max[ch]):
                        d.status |= (1 << 4)              # PARAM_ERR
                        continue
                    d.overload[ch] = val
            elif addr >= Reg.RANGE and addr < Reg.RANGE + 24:
                for i in range(count // 2):
                    off = (addr - Reg.RANGE) + i * 2
                    ch, sub = off // 4, off % 4
                    if ch >= 6:
                        break
                    if sub < 2:
                        d.range_min[ch] = f32(i * 4)
                    else:
                        d.range_max[ch] = f32(i * 4)
            elif addr >= Reg.MAC and addr <= Reg.GATEWAY + 2:
                for i in range(count):
                    val = struct.unpack(">H", data[i * 2:i * 2 + 2])[0]
                    a = addr + i
                    if Reg.MAC <= a < Reg.MAC + 3:
                        k = (a - Reg.MAC) * 2
                        d.mac[k], d.mac[k + 1] = val >> 8, val & 0xFF
                    elif Reg.IP <= a < Reg.IP + 2:
                        k = (a - Reg.IP) * 2
                        d.ip[k], d.ip[k + 1] = val >> 8, val & 0xFF
                    elif Reg.SUBNET <= a < Reg.SUBNET + 2:
                        k = (a - Reg.SUBNET) * 2
                        d.subnet[k], d.subnet[k + 1] = val >> 8, val & 0xFF
                    elif Reg.GATEWAY <= a < Reg.GATEWAY + 2:
                        k = (a - Reg.GATEWAY) * 2
                        d.gateway[k], d.gateway[k + 1] = val >> 8, val & 0xFF
            else:
                return EX_ILLEGAL_ADDR
        return 0


def main() -> None:                                   # pragma: no cover
    ap = argparse.ArgumentParser(description="六维力传感器 Modbus-TCP 模拟器")
    ap.add_argument("--host", default="0.0.0.0", help="监听地址，默认 0.0.0.0")
    ap.add_argument("--port", type=int, default=502, help="监听端口，默认 502")
    ap.add_argument("--order", default=None, choices=list(ByteOrder.ALL),
                    help="把读/写/推流三条路径统一为该字节序（参考实现模式）")
    ap.add_argument("--poll-order", default=None, choices=list(ByteOrder.ALL),
                    help="FC03 读字节序，默认 CDAB（与固件一致）")
    ap.add_argument("--push-order", default=None, choices=list(ByteOrder.ALL),
                    help="主动推流字节序，默认 DCBA（与固件一致）")
    ap.add_argument("--write-order", default=None, choices=list(ByteOrder.ALL),
                    help="FC10 写字节序，默认 DCBA（与固件一致）")
    ap.add_argument("--legacy-blocks", "--fixed-blocks", action="store_true",
                    dest="legacy_blocks",
                    help="复刻未修复的旧固件：块尾 0x003D/0x0092，Mz 读不全")
    ap.add_argument("--legacy-single", action="store_true",
                    help="复刻未修复的旧固件：写 0x0003 不回送 TCP 帧（只发 RS485）")
    ap.add_argument("--legacy-sticky", action="store_true",
                    help="复刻未修复的旧固件：一次 recv 只处理第一帧，"
                         "同批到达的其余请求被丢弃（TCP 粘包）")
    ap.add_argument("--rate", type=float, default=500.0, help="推流频率 Hz，默认 500")
    ap.add_argument("--quiet", action="store_true", help="不打印连接日志")
    args = ap.parse_args()

    srv = DeviceServer(args.host, args.port, args.order, args.rate,
                       verbose=not args.quiet,
                       legacy_blocks=args.legacy_blocks,
                       legacy_single=args.legacy_single,
                       legacy_sticky=args.legacy_sticky,
                       push_order=args.push_order,
                       write_order=args.write_order)
    try:
        srv.start()
    except KeyboardInterrupt:
        srv.stop()
        print("\n模拟器已停止")


if __name__ == "__main__":                            # pragma: no cover
    main()
