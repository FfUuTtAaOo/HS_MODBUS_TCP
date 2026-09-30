# -*- coding: utf-8 -*-
"""
hs_modbus.py —— 中科米点六维力传感器 Modbus-TCP 协议层（纯标准库）

适配协议：Docs/modbus-tcp-protocol-v2.0.md
设备角色：Modbus TCP Server，默认 192.168.1.12:502，Unit ID = 0x01，支持 4 路并发连接。

本模块与界面无关，可单独作为脚本/其他程序调用：

    from hs_modbus import SensorClient, Reg, ByteOrder

    cli = SensorClient("192.168.1.12")
    cli.connect()
    print(cli.get_sn(), cli.get_fw_version())
    cli.set_format(Format.N)          # 切换为 N 单位
    cli.start()                       # 开始连续发送（设备主动推流）
    ...
    cli.stop()
    cli.close()

────────────────────────────────────────────────────────────────────────
关于 float32 字节序（重要：轮询与推流是两套）
────────────────────────────────────────────────────────────────────────
协议文档 v2.0 示例（0.1 → 3D CC CC CD、1000.0 → 44 7A 00 00）是标准大端 ABCD，
但文档正文却把 float32 标注为「小端」，自相矛盾（示例才是对的）。

而固件里同一份数据存在**两条字节序不同的路径**：

  1) FC03 轮询（Core/Src/mb_register.c mb_reg_read）
       val = lo ? (uint16_t)u : (uint16_t)(u >> 16);   /* 第一个寄存器 = 低 16 位 */
       寄存器再按 Modbus 惯例大端输出
     → 线上字节序 = CDAB（低字在前 + 字内大端）

  2) 主动推流（Core/Src/main.c rs485_send_continuous）
       buf[9+i*4+0] = (uint8_t)(u);
       buf[9+i*4+1] = (uint8_t)(u >> 8);   ...          /* 标准小端 */
     → 线上字节序 = DCBA（标准小端）

实测真机推送帧的 24 字节数据，只有按 DCBA 解析才能得到量级合理的六个值，
其余三种字节序都会得到 1e17 量级的荒谬值 —— 与上面的源码分析一致。

因此本模块把字节序拆成三项，默认 POLL=CDAB / WRITE=PUSH=DCBA：
调用方通常不需要关心，直接使用高层 API 即可；界面提供手动切换与自动探测。

「单次转换」（写 0x0003）属于**推流路径**：固件在 Modbus-TCP 通道上会回送一帧
与连续推流帧格式完全相同的 33 字节数据（TID=0000 / FC=03 / 6×float32，DCBA），
所以 single_shot() 也按 PUSH 字节序解析 —— 见该方法文档。
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from collections import deque
from typing import Callable, Dict, List, Optional, Sequence, Tuple

__all__ = [
    "Reg", "Format", "FreqMode", "ByteOrder", "StatusBit",
    "AXIS_NAMES", "AXIS_UNITS_FORCE",
    "SensorClient", "ModbusError", "ModbusTimeoutError", "ModbusException",
    "bytes_to_float", "float_to_bytes", "hexdump",
]


# =====================================================================
#  1. 寄存器地址表（与 Core/Inc/mb_register.h 一一对应）
# =====================================================================
class Reg:
    """保持寄存器地址。float32 占 2 个寄存器，地址为低字所在寄存器。"""

    # ---- 控制指令（只写，写事件触发，触发后自动归零）----
    STOP = 0x0001           # 停止发送
    START = 0x0002          # 开始连续数据转换（开始主动推流）
    SINGLE = 0x0003         # 单次转换

    # ---- 设备信息（只读）----
    SN = 0x0005             # 序列号，8 个寄存器 = 16 字符 ASCII
    SN_COUNT = 8
    FW_VERSION = 0x000D     # 固件版本，2 个寄存器（major, minor）
    STATUS = 0x000F         # 系统状态字
    COMM_ERR = 0x0010       # 通信错误累计计数

    # ---- 清零 / 输出格式 ----
    ZERO_TRIG = 0x0030      # 触发自动清零（去皮）
    UNZERO_TRIG = 0x0031    # 取消清零（恢复绝对零点）
    DATA_FORMAT = 0x0032    # 0 = mV，1 = kg，2 = N（可读可写）

    # ---- 六维力 / 力矩数据（只读，6 × float32 = 12 个寄存器）----
    # P0-1 之前：mb_reg_read() 的 case 只覆盖到 0x003D（11 个寄存器），
    # 而 MB_BLK_CTRL_END 是 0x003E，于是整段 12 寄存器请求通过 valid_range()
    # 之后在末位落到 default → 异常 02，Mz 的高 16 位读不到。
    # 现已修复（case 补到 MB_REG_TORQUE_MZ + 1），本段 12 个寄存器可全读。
    FORCE = 0x0033          # Fx, Fy, Fz, Mx, My, Mz
    FORCE_COUNT = 12

    # ---- 零点偏移（可读可写，6 × float32）----
    ZERO_OFFSET = 0x0050
    ZERO_OFFSET_COUNT = 12

    # ---- 过载阈值（可读可写，6 × float32）----
    OVERLOAD = 0x0070
    OVERLOAD_COUNT = 12

    # ---- 阈值允许范围（可读可写，12 × float32：min/max 交替）----
    # 同 P0-1：case 原本只到 0x0092，现已补到 MB_REG_RANGE_MZ_MAX + 1 = 0x0093，
    # 24 个寄存器可全读。
    RANGE = 0x007C
    RANGE_COUNT = 24

    # ---- 网络参数（可读可写；写后需重启生效）----
    MAC = 0x0100            # 3 个有效寄存器（6 字节），另有 3 个填充寄存器
    IP = 0x0106             # 2 个寄存器
    SUBNET = 0x0108         # 2 个寄存器
    GATEWAY = 0x010A        # 2 个寄存器

    # ---- 扩展寄存器（不在客户文档中，固件内部使用）----
    FREQ_MODE = 0x0200      # 0 = 500 Hz，1 = 1000 Hz
    ST_ERROR = 0x0210       # 自检：错误标志字 0
    ST_ERROR_HI = 0x0211    # 自检：错误标志字 1
    ST_ADC_ID = 0x0212      # 自检：ADC ID + ERROR 字节
    ST_W5500 = 0x0213       # 自检：W5500 版本 + PHY Link + 速率
    ST_RS485 = 0x0214       # 自检：RS485 UART + DMA 状态
    ST_FLASH = 0x0215       # 自检：Flash 有效标志 / 矩阵 / 零点 / 配置


class Format:
    """0x0032 输出格式取值。"""
    MV = 0
    KG = 1
    N = 2

    NAME = {MV: "mV", KG: "kg", N: "N"}
    # 每种格式下 力 / 力矩 的单位
    UNIT_FORCE = {MV: "mV", KG: "kg", N: "N"}
    UNIT_TORQUE = {MV: "mV", KG: "kg·m", N: "N·m"}


class FreqMode:
    F500 = 0
    F1000 = 1

    NAME = {F500: "500 Hz", F1000: "1000 Hz"}


class ByteOrder:
    """
    float32 在 4 个连续字节中的排列方式（ABCD 表示标准大端 4 字节）。

    ABCD : 标准大端（协议文档示例、通用 Modbus 约定）
    DCBA : 标准小端
    CDAB : 字交换（低字在前）—— 当前固件读路径的实际行为（默认）
    BADC : 字节交换
    """
    ABCD = "ABCD"
    DCBA = "DCBA"
    CDAB = "CDAB"
    BADC = "BADC"

    ALL = (ABCD, CDAB, BADC, DCBA)

    # 轮询（FC03）/ 写入（FC10）/ 主动推送 三条路径的字节序**互不相同**：
    #   FC03 读   mb_register.c mb_reg_read():   先低字、每字再大端  → CDAB
    #   FC10 写   mb_register.c buf_to_float():  按小端解释数据字节     → DCBA
    #   主动推送  main.c rs485_send_continuous(): u>>0, >>8, >>16, >>24 → DCBA
    # 实测真机：推送帧 24 字节只有按 DCBA 才能解出量级合理的六个值，
    # 其余三种字节序都会得到 1e17 量级的荒谬值。
    POLL_DEFAULT = CDAB
    PUSH_DEFAULT = DCBA
    WRITE_DEFAULT = DCBA

    DESC = {
        ABCD: "标准大端 (A B C D) — 协议文档示例",
        CDAB: "字交换 / 低字在前 (C D A B) — FC03 轮询实际",
        BADC: "字节交换 (B A D C)",
        DCBA: "标准小端 (D C B A) — 写入 / 主动推送实际",
    }


class StatusBit:
    """0x000F 系统状态字位定义（见协议 3.4.1）。"""
    EEPROM_FAIL = 0
    ADC_FAIL = 1
    CRC_ERR = 2
    OVERLOAD = 3
    PARAM_ERR = 4

    LABEL = {
        EEPROM_FAIL: "EEPROM 初始化失败",
        ADC_FAIL: "ADC 初始化失败",
        CRC_ERR: "通信校验失败",
        OVERLOAD: "过载告警",
        PARAM_ERR: "参数写入越界",
    }


AXIS_NAMES = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
AXIS_UNITS_FORCE = ("N", "N", "N", "N·m", "N·m", "N·m")


# =====================================================================
#  2. 异常
# =====================================================================
class ModbusError(Exception):
    """Modbus 通信层错误基类。"""


class ModbusTimeoutError(ModbusError):
    """等待响应超时。"""


class ModbusException(ModbusError):
    """设备返回异常码（FC | 0x80）。"""

    TEXT = {
        0x01: "非法功能码",
        0x02: "非法数据地址",
        0x03: "非法数据值",
        0x04: "从站设备故障",
    }

    def __init__(self, code: int):
        self.code = code
        super().__init__(f"设备异常 0x{code:02X} ({self.TEXT.get(code, '未知')})")


# =====================================================================
#  3. float32 字节序工具
# =====================================================================
def bytes_to_float(buf: bytes, order: str = ByteOrder.CDAB) -> float:
    """把 4 个字节按指定字节序解析为 float32。"""
    if len(buf) < 4:
        raise ValueError("float32 需要 4 个字节")
    b = bytes(buf[:4])
    if order == ByteOrder.ABCD:
        return struct.unpack(">f", b)[0]
    if order == ByteOrder.DCBA:
        return struct.unpack("<f", b)[0]
    if order == ByteOrder.BADC:
        return struct.unpack(">f", bytes((b[1], b[0], b[3], b[2])))[0]
    if order == ByteOrder.CDAB:
        return struct.unpack(">f", bytes((b[2], b[3], b[0], b[1])))[0]
    raise ValueError(f"未知字节序: {order}")


def float_to_bytes(value: float, order: str = ByteOrder.CDAB) -> bytes:
    """把 float32 按指定字节序打包为 4 个字节。"""
    b = struct.pack(">f", float(value))
    if order == ByteOrder.ABCD:
        return b
    if order == ByteOrder.DCBA:
        return b[::-1]
    if order == ByteOrder.BADC:
        return bytes((b[1], b[0], b[3], b[2]))
    if order == ByteOrder.CDAB:
        return bytes((b[2], b[3], b[0], b[1]))
    raise ValueError(f"未知字节序: {order}")


def hexdump(data: bytes, prefix: str = "") -> str:
    """把字节串转成 "AA BB CC" 形式的十六进制文本。"""
    return prefix + " ".join(f"{x:02X}" for x in data)


# =====================================================================
#  4. 客户端
# =====================================================================
class SensorClient:
    """
    六维力传感器 Modbus-TCP 客户端。

    线程模型
    --------
    * 一个后台接收线程持续 recv 并按 MBAP 长度拆帧；
    * 事务号（TID）非 0 的帧作为请求响应，唤醒等待中的调用方；
    * TID == 0 的帧是设备在“连续发送”模式下主动推送的数据帧
      （MBAP: 00 00 00 00 00 1B 01 03 18 + 24 字节 6×float32），
      写入波形缓冲并可选触发 on_push 回调。

    所有公开方法都是线程安全的。
    """

    PUSH_FRAME_LEN = 33          # 主动推送帧总长度
    PUSH_PAYLOAD_LEN = 24        # 6 × float32

    def __init__(self,
                 host: str = "192.168.1.12",
                 port: int = 502,
                 unit_id: int = 1,
                 byte_order: str = ByteOrder.POLL_DEFAULT,
                 push_byte_order: str = ByteOrder.PUSH_DEFAULT,
                 write_byte_order: str = ByteOrder.WRITE_DEFAULT,
                 timeout: float = 1.0,
                 wave_capacity: int = 20000):
        self.host = host
        self.port = int(port)
        self.unit_id = int(unit_id)
        # byte_order       —— FC03 轮询读路径的 float32 字节序（默认 CDAB）
        # write_byte_order —— FC10 写路径的 float32 字节序（默认 DCBA）
        # push_byte_order  —— 设备主动推流帧的 float32 字节序（默认 DCBA）
        self.byte_order = byte_order
        self.push_byte_order = push_byte_order
        self.write_byte_order = write_byte_order
        self.timeout = float(timeout)
        # 只读请求（FC03）超时后的自动重试次数。设备侧存在「两个请求紧挨着
        # 到达时后一个被丢弃」的固件缺陷（P0-4，见实现审查文档），重试一次
        # 能显著降低偶发超时；写请求不适用（见 _request 的说明）。
        self.read_retries = 1

        # 连接后由 probe_capabilities() 填充：真机实际可访问范围
        self.caps: Dict[str, object] = {
            "force_regs": Reg.FORCE_COUNT,
            "range_regs": Reg.RANGE_COUNT,
            "probed": False,
        }

        # ---- 回调（在接收线程中调用，注意不要直接操作界面）----
        self.on_push: Optional[Callable[[List[float]], None]] = None
        self.on_log: Optional[Callable[[str, str], None]] = None        # (方向, 文本)
        self.on_state: Optional[Callable[[bool, str], None]] = None     # (是否连接, 说明)
        self.on_error: Optional[Callable[[str], None]] = None

        # ---- 内部状态 ----
        self._sock: Optional[socket.socket] = None
        self._reader: Optional[threading.Thread] = None
        self._running = False
        self._tx_lock = threading.Lock()
        self._pending_lock = threading.RLock()
        self._pending: Dict[int, Tuple[threading.Event, dict]] = {}
        self._tid = 0
        self._stop_flag = threading.Event()

        # ---- 接收统计 ----
        self._data_lock = threading.Lock()
        self.latest: Optional[List[float]] = None
        self.wave: deque = deque(maxlen=wave_capacity)
        self.push_count = 0
        self.rx_frame_count = 0
        self.tx_frame_count = 0
        self.error_count = 0

        # ---- 单次转换 ----
        # last_single_src 记录上一次 single_shot() 实际走的路径："push" / "poll"
        # single_push_wait 是等待设备回送单次帧的秒数；一旦发现设备不推帧
        # （旧固件只往 RS485 发），自动缩短，避免每次都白等。
        self.last_single_src: Optional[str] = None
        self.single_push_wait = 0.35

    # ------------------------------------------------------------------
    #  连接管理
    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._running and self._sock is not None

    def connect(self) -> None:
        """建立 TCP 连接并启动接收线程。"""
        if self.connected:
            return
        sock = socket.create_connection((self.host, self.port), timeout=3.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(0.2)
        self._sock = sock
        self._running = True
        self._stop_flag.clear()
        self._reader = threading.Thread(target=self._read_loop,
                                        name="hs-modbus-rx", daemon=True)
        self._reader.start()
        self._emit_state(True, f"已连接 {self.host}:{self.port}")

    def close(self) -> None:
        """关闭连接并停止接收线程。"""
        if not self._running and self._sock is None:
            return
        self._running = False
        self._stop_flag.set()
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        # 唤醒所有等待中的请求
        with self._pending_lock:
            for ev, holder in self._pending.values():
                holder["closed"] = True
                ev.set()
            self._pending.clear()
        self._emit_state(False, "已断开")

    # 兼容写法
    disconnect = close

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # ------------------------------------------------------------------
    #  接收线程
    # ------------------------------------------------------------------
    def _read_loop(self) -> None:
        buf = bytearray()
        sock = self._sock
        while self._running and sock is not None:
            try:
                chunk = sock.recv(8192)
            except socket.timeout:
                continue
            except OSError as e:
                if self._running:
                    self._fail(f"接收失败: {e}")
                break
            if not chunk:
                if self._running:
                    self._fail("连接已被对端关闭")
                break
            buf.extend(chunk)
            self._extract_frames(buf)
        # 线程退出
        self._running = False

    def _extract_frames(self, buf: bytearray) -> None:
        """按 MBAP 长度字段从字节流中切出完整帧。"""
        while len(buf) >= 7:
            tid, pid, length = struct.unpack(">HHH", bytes(buf[:6]))
            if pid != 0:
                # 协议标识异常，丢弃首字节重新同步
                del buf[:1]
                continue
            total = 6 + length
            if length < 2 or total > 260:
                del buf[:1]
                continue
            if len(buf) < total:
                return
            frame = bytes(buf[:total])
            del buf[:total]
            self.rx_frame_count += 1
            try:
                self._dispatch(tid, frame)
            except Exception as e:                       # noqa: BLE001
                self.error_count += 1
                self._emit_error(f"帧处理异常: {e}")

    def _dispatch(self, tid: int, frame: bytes) -> None:
        if tid == 0:
            # 设备主动推送的数据帧（高频，不写入报文日志，避免刷屏）
            self._handle_push(frame)
            return

        if self.on_log:
            self.on_log("RX", hexdump(frame))

        with self._pending_lock:
            item = self._pending.get(tid)
        if item is None:
            return                                   # 已超时/未知响应，丢弃
        ev, holder = item
        holder["frame"] = frame
        ev.set()

    def _handle_push(self, frame: bytes) -> None:
        """解析主动推送帧：MBAP(7) + FC(1) + BYTE_CNT(1) + 24 字节数据。"""
        if len(frame) < 9:
            return
        fc = frame[7]
        if fc != 0x03:
            return
        length = frame[8]
        payload = frame[9:9 + length]
        if len(payload) < self.PUSH_PAYLOAD_LEN:
            return
        vals = [bytes_to_float(payload[i * 4:i * 4 + 4], self.push_byte_order)
                for i in range(6)]
        ts = time.time()
        with self._data_lock:
            self.latest = vals
            self.wave.append((ts, vals))
            self.push_count += 1
        cb = self.on_push
        if cb is not None:
            cb(vals)

    # ------------------------------------------------------------------
    #  收发基础
    # ------------------------------------------------------------------
    def _next_tid(self) -> int:
        self._tid = (self._tid % 0xFFFE) + 1        # 1..0xFFFF，跳过 0
        return self._tid

    def _request(self, pdu: bytes, timeout: Optional[float] = None,
                 retries: Optional[int] = None) -> bytes:
        """
        发送 PDU 并等待该事务号的响应帧，返回完整响应帧。

        只读请求（FC03）默认允许自动重试一次。原因：某些固件的 TCP 接收
        路径在「两个请求紧挨着到达」时会丢弃后一个 —— 既不处理也不应答
        （见 Docs/modbus-tcp-实现审查.md 的 P0-4）。重试能把这种偶发超时
        变成「稍慢但成功」。写请求（FC06/FC10）不重试，不对设备产生重复
        副作用。重试用新事务号，因此那个「迟到」的旧响应会因找不到等待者
        而被丢弃，不会污染后续请求。
        """
        if not pdu:
            raise ModbusError("空 PDU")
        if retries is None:
            retries = self.read_retries if pdu[0] == 0x03 else 0

        last: Optional[ModbusTimeoutError] = None
        for attempt in range(retries + 1):
            try:
                return self._request_once(pdu, timeout)
            except ModbusTimeoutError as e:
                last = e
                if attempt < retries:
                    self._emit_error(
                        f"{e}；只读请求自动重试第 {attempt + 1} 次")
        assert last is not None
        raise last

    def _request_once(self, pdu: bytes, timeout: Optional[float] = None) -> bytes:
        """发送 PDU 并等待该事务号的响应帧（不重试）。"""
        if self._sock is None:
            raise ModbusError("尚未连接设备")
        tid = self._next_tid()
        req = struct.pack(">HHHB", tid, 0, len(pdu) + 1, self.unit_id) + pdu
        self.tx_frame_count += 1

        ev = threading.Event()
        holder: dict = {}
        with self._pending_lock:
            self._pending[tid] = (ev, holder)
        try:
            with self._tx_lock:
                if self.on_log:
                    self.on_log("TX", hexdump(req))
                self._sock.sendall(req)
            if not ev.wait(timeout if timeout is not None else self.timeout):
                self.error_count += 1
                raise ModbusTimeoutError(
                    f"等待响应超时（TID=0x{tid:04X}）")
            if holder.get("closed"):
                raise ModbusError("连接已关闭")
            frame = holder["frame"]
            fc = frame[7]
            if fc & 0x80:
                raise ModbusException(frame[8] if len(frame) > 8 else 0)
            return frame
        finally:
            with self._pending_lock:
                self._pending.pop(tid, None)

    # ------------------------------------------------------------------
    #  原始寄存器读写
    # ------------------------------------------------------------------
    def read_registers(self, addr: int, count: int) -> List[int]:
        """FC 03 —— 读保持寄存器，返回 uint16 列表。"""
        if not 1 <= count <= 125:
            raise ValueError("寄存器数量需在 1..125 之间")
        frame = self._request(struct.pack(">BHH", 0x03, addr, count))
        byte_count = frame[8]
        data = frame[9:9 + byte_count]
        if len(data) < count * 2:
            raise ModbusError(f"响应数据长度不足：期望 {count * 2} 字节，"
                              f"实际 {len(data)}")
        return [struct.unpack(">H", data[i * 2:i * 2 + 2])[0]
                for i in range(count)]

    def read_bytes(self, addr: int, count: int) -> bytes:
        """FC 03 —— 读寄存器并返回原始字节串（不做 uint16 拆分）。"""
        frame = self._request(struct.pack(">BHH", 0x03, addr, count))
        byte_count = frame[8]
        return frame[9:9 + byte_count][:count * 2]

    def write_register(self, addr: int, value: int) -> None:
        """FC 06 —— 写单个寄存器。"""
        self._request(struct.pack(">BHH", 0x06, addr, value & 0xFFFF))

    def write_registers(self, addr: int, values: Sequence[int]) -> None:
        """FC 10 —— 写多个寄存器（values 为 uint16 序列）。"""
        values = list(values)
        if not values:
            raise ValueError("写入数据不能为空")
        payload = b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
        pdu = struct.pack(">BHHB", 0x10, addr, len(values), len(payload)) + payload
        self._request(pdu)

    # ------------------------------------------------------------------
    #  float32 便捷读写
    # ------------------------------------------------------------------
    def read_floats(self, addr: int, n: int, shrink: bool = False) -> List[float]:
        """
        读取 n 个 float32（共 2n 个寄存器）。

        shrink=True 时，若设备以「非法数据地址(0x02)」拒绝整块读取，则自动
        退到 n*2-1 个寄存器（即上一个完整 float），最后一个通道用 NaN 占位。
        用来兼容真机把块尾收敛到最后一个 float 起始地址的固件 bug。
        """
        try:
            raw = self.read_bytes(addr, n * 2)
        except ModbusException as e:
            if not (shrink and e.code == 0x02 and n > 1):
                raise
            raw = self.read_bytes(addr, n * 2 - 1)
            vals = [bytes_to_float(raw[i * 4:i * 4 + 4], self.byte_order)
                    for i in range((len(raw)) // 4)]
            while len(vals) < n:
                vals.append(float("nan"))
            return vals
        return [bytes_to_float(raw[i * 4:i * 4 + 4], self.byte_order)
                for i in range(n)]

    def write_floats(self, addr: int, values: Sequence[float]) -> None:
        """写入 n 个 float32（按 FC10 写路径的字节序打包，默认 DCBA）。"""
        raw = b"".join(float_to_bytes(v, self.write_byte_order) for v in values)
        regs = [struct.unpack(">H", raw[i * 2:i * 2 + 2])[0]
                for i in range(len(raw) // 2)]
        self.write_registers(addr, regs)

    # ------------------------------------------------------------------
    #  数据访问
    # ------------------------------------------------------------------
    def latest_values(self) -> Optional[List[float]]:
        """最近一次推送/读取的六维数据（线程安全）。"""
        with self._data_lock:
            return list(self.latest) if self.latest is not None else None

    def wave_snapshot(self, seconds: float = 10.0) -> List[Tuple[float, List[float]]]:
        """取最近 seconds 秒内的波形数据快照。"""
        with self._data_lock:
            if not self.wave:
                return []
            t_end = self.wave[-1][0]
            t_start = t_end - seconds
            return [(t, list(v)) for t, v in self.wave if t >= t_start]

    def clear_wave(self) -> None:
        with self._data_lock:
            self.wave.clear()

    def reset_stats(self) -> None:
        with self._data_lock:
            self.push_count = 0
            self.rx_frame_count = 0
            self.tx_frame_count = 0
            self.error_count = 0

    # ------------------------------------------------------------------
    #  六维力数据（兼容真机残块）
    # ------------------------------------------------------------------
    def get_force(self) -> List[Optional[float]]:
        """
        轮询读取六维力数据。

        会按 probe_capabilities() 实测到的长度裁剪请求；对于读不到的通道
        （真机固件的 Mz）返回 None，而不是抛异常或给出错值。
        """
        n = int(self.caps.get("force_regs", Reg.FORCE_COUNT))
        usable = n - (n % 2)                 # 只取完整 float 的寄存器数
        raw = self.read_bytes(Reg.FORCE, max(2, usable))
        vals: List[Optional[float]] = [
            bytes_to_float(raw[i * 4:i * 4 + 4], self.byte_order)
            for i in range(len(raw) // 4)]
        while len(vals) < 6:
            vals.append(None)
        return vals

    # ------------------------------------------------------------------
    #  控制指令
    # ------------------------------------------------------------------
    def start(self) -> None:
        """0x0002 —— 开始连续数据转换（设备开始主动推流）。"""
        self.write_register(Reg.START, 1)

    def stop(self) -> None:
        """0x0001 —— 停止发送。"""
        self.write_register(Reg.STOP, 1)

    def single_shot(self, delay: float = 0.01, wait_push: float = 0.35,
                    use_push: bool = True) -> List[Optional[float]]:
        """
        0x0003 —— 触发单次转换，返回 6 维数据（不可用通道为 None）。

        固件在 Modbus-TCP 通道上会把单次转换结果**主动回送一帧**（main.c 的
        `send_mode == 2` 分支，帧格式与连续推流完全相同：TID=0000 / LEN=001B /
        FC=03 / 6×float32），所以这里优先消费这一帧 —— 按 `push_byte_order`
        解析（默认 DCBA），并且能拿到完整的 Mz（不依赖轮询读）。

        取数顺序：

        1. 写 0x0003，然后等设备回送的帧，最多 `self.single_push_wait` 秒；
           等到则返回（`last_single_src == "push"`）；
        2. 超时则回退轮询读 `0x0033`（按 `byte_order`，默认 CDAB），
           力数据块读不全时相应通道返回 None（`last_single_src == "poll"`）。

        若设备从不推帧（旧固件只往 RS485 发），后续调用的等待会自动缩短到
        0.1 s，避免每次单次触发都白等。
        """
        with self._data_lock:
            base = self.push_count

        self.write_register(Reg.SINGLE, 1)

        if use_push and self.single_push_wait > 0:
            deadline = time.monotonic() + self.single_push_wait
            push_vals: Optional[List[float]] = None
            while time.monotonic() < deadline:
                time.sleep(0.004)
                with self._data_lock:
                    if self.push_count > base and self.latest is not None:
                        push_vals = list(self.latest)
                        break
            if push_vals is not None:
                self.last_single_src = "push"
                self.single_push_wait = max(wait_push, 0.05)   # 设备支持，保持长等待
                return [v if (v is not None and math.isfinite(v)) else None
                        for v in push_vals]
            self.single_push_wait = 0.1        # 设备不推帧，下次少等

        time.sleep(max(delay, 0.003))
        vals = self.get_force()
        self.last_single_src = "poll"
        with self._data_lock:
            self.latest = [v if v is not None else float("nan") for v in vals]
        return vals

    def zero(self, verify_delay: float = 0.3) -> List[float]:
        """0x0030 —— 触发自动清零（去皮），返回清零后的零点偏移值。"""
        self.write_register(Reg.ZERO_TRIG, 1)
        time.sleep(verify_delay)                     # 设备需采集 20 点求平均
        return self.get_zero_offsets()

    def unzero(self) -> None:
        """0x0031 —— 取消清零，零点偏移全部归零。"""
        self.write_register(Reg.UNZERO_TRIG, 1)
        time.sleep(0.05)

    # ------------------------------------------------------------------
    #  输出格式
    # ------------------------------------------------------------------
    def get_format(self) -> int:
        """读当前输出格式：0 = mV，1 = kg，2 = N。"""
        return self.read_registers(Reg.DATA_FORMAT, 1)[0]

    def set_format(self, fmt: int) -> None:
        if fmt not in (Format.MV, Format.KG, Format.N):
            raise ValueError("格式取值只能是 0(mV) / 1(kg) / 2(N)")
        self.write_register(Reg.DATA_FORMAT, fmt)

    def get_freq_mode(self) -> int:
        return self.read_registers(Reg.FREQ_MODE, 1)[0]

    def set_freq_mode(self, mode: int) -> None:
        if mode not in (FreqMode.F500, FreqMode.F1000):
            raise ValueError("频率模式只能是 0(500Hz) / 1(1000Hz)")
        self.write_register(Reg.FREQ_MODE, mode)

    # ------------------------------------------------------------------
    #  设备信息与状态
    # ------------------------------------------------------------------
    def get_sn(self) -> str:
        """读取序列号字符串（0x0005，8 个寄存器 / 16 字符）。"""
        raw = self.read_bytes(Reg.SN, Reg.SN_COUNT)
        text = raw.split(b"\x00")[0].decode("ascii", errors="replace")
        return text.strip()

    def get_fw_version(self) -> Tuple[int, int]:
        """读取固件版本 (major, minor)。"""
        regs = self.read_registers(Reg.FW_VERSION, 2)
        return regs[0] & 0xFF, regs[1] & 0xFF

    def get_fw_version_str(self) -> str:
        major, minor = self.get_fw_version()
        return f"v{major}.{minor}"

    def get_status(self) -> int:
        """读取系统状态字（0x000F）。"""
        return self.read_registers(Reg.STATUS, 1)[0]

    def describe_status(self, status: int) -> List[str]:
        """把状态字解析为可读文本列表。"""
        out = [f"{StatusBit.LABEL[b]} (bit{b})"
               for b in sorted(StatusBit.LABEL) if status & (1 << b)]
        return out or ["正常"]

    def get_comm_error(self) -> int:
        return self.read_registers(Reg.COMM_ERR, 1)[0]

    def get_self_test(self) -> Dict[str, int]:
        """读取扩展自检寄存器（0x0210~0x0215）。"""
        regs = self.read_registers(Reg.ST_ERROR, 6)
        return {
            "error_flags": (regs[1] << 16) | regs[0],
            "adc_id": (regs[2] >> 8) & 0xFF,
            "adc_error": regs[2] & 0xFF,
            "w5500_version": (regs[3] >> 8) & 0xFF,
            "w5500_phy_link": bool(regs[3] & 0x80),
            "w5500_sockets_ok": regs[3] & 0x0F,
            "rs485_uart_ok": bool(regs[4] & 0x80),
            "rs485_dma_ok": bool(regs[4] & 0x40),
            "flash_valid": bool(regs[5] & 0x80),
            "flash_matrix_ok": bool(regs[5] & 0x40),
            "flash_zero_ok": bool(regs[5] & 0x20),
            "flash_config_ok": bool(regs[5] & 0x10),
        }

    # ------------------------------------------------------------------
    #  零点偏移
    # ------------------------------------------------------------------
    def get_zero_offsets(self) -> List[float]:
        return self.read_floats(Reg.ZERO_OFFSET, 6)

    def set_zero_offsets(self, values: Sequence[float]) -> None:
        if len(values) != 6:
            raise ValueError("零点偏移必须是 6 个值")
        self.write_floats(Reg.ZERO_OFFSET, values)

    # ------------------------------------------------------------------
    #  过载阈值 / 阈值范围
    # ------------------------------------------------------------------
    def get_overload(self) -> List[float]:
        return self.read_floats(Reg.OVERLOAD, 6)

    def set_overload(self, values: Sequence[float]) -> None:
        if len(values) != 6:
            raise ValueError("过载阈值必须是 6 个值")
        self.write_floats(Reg.OVERLOAD, values)

    def get_ranges(self) -> List[Tuple[Optional[float], Optional[float]]]:
        """
        返回每个通道的 (最小值, 最大值)。

        兼容真机把范围块尾收敛到 0x0092 的情况：此时最后一项（Mz 最大值）
        读不到，对应位置返回 None。
        """
        n = int(self.caps.get("range_regs", Reg.RANGE_COUNT)) // 2
        flat = self.read_floats(Reg.RANGE, max(1, n), shrink=True)
        out: List[Tuple[Optional[float], Optional[float]]] = []
        for i in range(6):
            lo = flat[2 * i] if 2 * i < len(flat) else None
            hi = flat[2 * i + 1] if 2 * i + 1 < len(flat) else None
            if lo is not None and lo != lo:            # NaN 占位
                lo = None
            if hi is not None and hi != hi:
                hi = None
            out.append((lo, hi))
        return out

    def set_range(self, axis: int, lo: float, hi: float) -> None:
        """写入指定通道的阈值允许范围（axis: 0..5）。"""
        if not 0 <= axis < 6:
            raise ValueError("通道号必须是 0..5")
        addr = Reg.RANGE + axis * 4
        self.write_floats(addr, [lo, hi])

    # ------------------------------------------------------------------
    #  网络参数
    # ------------------------------------------------------------------
    def get_network(self) -> Dict[str, object]:
        """一次性读取 0x0100~0x010B（MAC + IP + 掩码 + 网关）。"""
        regs = self.read_registers(Reg.MAC, 12)
        mac = []
        for i in range(3):
            mac += [regs[i] >> 8, regs[i] & 0xFF]
        ip = [regs[6] >> 8, regs[6] & 0xFF, regs[7] >> 8, regs[7] & 0xFF]
        mask = [regs[8] >> 8, regs[8] & 0xFF, regs[9] >> 8, regs[9] & 0xFF]
        gw = [regs[10] >> 8, regs[10] & 0xFF, regs[11] >> 8, regs[11] & 0xFF]
        return {
            "mac": ":".join(f"{x:02X}" for x in mac),
            "ip": ".".join(str(x) for x in ip),
            "subnet": ".".join(str(x) for x in mask),
            "gateway": ".".join(str(x) for x in gw),
        }

    def set_ip(self, ip: str) -> None:
        self._write_ipv4(Reg.IP, ip)

    def set_subnet(self, ip: str) -> None:
        self._write_ipv4(Reg.SUBNET, ip)

    def set_gateway(self, ip: str) -> None:
        self._write_ipv4(Reg.GATEWAY, ip)

    def set_mac(self, mac: str) -> None:
        parts = mac.replace("-", ":").split(":")
        if len(parts) != 6:
            raise ValueError("MAC 地址格式应为 AA:BB:CC:DD:EE:FF")
        octets = [int(p, 16) for p in parts]
        regs = [(octets[0] << 8) | octets[1],
                (octets[2] << 8) | octets[3],
                (octets[4] << 8) | octets[5]]
        self.write_registers(Reg.MAC, regs)

    def _write_ipv4(self, addr: int, ip: str) -> None:
        parts = ip.split(".")
        if len(parts) != 4:
            raise ValueError("IPv4 地址格式应为 a.b.c.d")
        octets = [int(p) for p in parts]
        if any(not 0 <= o <= 255 for o in octets):
            raise ValueError("IPv4 每段取值需在 0..255")
        self.write_registers(addr, [(octets[0] << 8) | octets[1],
                                    (octets[2] << 8) | octets[3]])

    # ------------------------------------------------------------------
    #  设备能力探测（连接后调用一次）
    # ------------------------------------------------------------------
    def probe_capabilities(self) -> Dict[str, object]:
        """
        探测真机实际可访问的寄存器范围与轮询字节序。

        为什么必须实测：协议文档 v2.0 与 Core/Inc/mb_register.h 是权威定义，
        但设备里正在跑的固件版本不一定与之对齐 —— P0-1 之前的固件里
        mb_reg_read() 的 case 范围比块尾各少一格，于是按文档写的
        「0x0033 ×12」「0x007C ×24」通过 valid_range() 之后在末位落到
        default，被拒为异常 02（非法数据地址），且第 6 通道 Mz 读不全。
        此处实测后按实际能力裁剪请求，读不到的通道返回 None，不编造数值。

        返回 dict：
            force_regs / range_regs  实测可读寄存器数
            poll_order / push_order  两条路径的 float32 字节序
            notes                    可直接展示给用户的说明
        """
        notes: List[str] = []
        caps: Dict[str, object] = {"notes": notes}

        # ---- 1. 六维力数据块实际可读长度 ----
        force_regs = 0
        for n in (12, 11, 10, 6):
            try:
                self.read_registers(Reg.FORCE, n)
                force_regs = n
                break
            except ModbusException:
                continue
        caps["force_regs"] = force_regs or Reg.FORCE_COUNT
        if force_regs and force_regs < Reg.FORCE_COUNT:
            notes.append(
                f"力数据块实测只能读 {force_regs} 个寄存器（协议文档写 "
                f"{Reg.FORCE_COUNT} 个），第 6 通道 Mz 读不全 —— 设备固件"
                f"未包含 P0-1 修复（mb_reg_read 的 case 未覆盖到 0x003E）")

        # ---- 2. 阈值范围块实际可读长度 ----
        range_regs = 0
        for n in (24, 23, 22, 4):
            try:
                self.read_registers(Reg.RANGE, n)
                range_regs = n
                break
            except ModbusException:
                continue
        caps["range_regs"] = range_regs or Reg.RANGE_COUNT
        if range_regs and range_regs < Reg.RANGE_COUNT:
            notes.append(
                f"阈值范围块实测只能读 {range_regs} 个寄存器（协议文档写 "
                f"{Reg.RANGE_COUNT} 个），Mz 最大值不可读 —— 设备固件"
                f"未包含 P0-1 修复（mb_reg_read 的 case 未覆盖到 0x0093）")

        # ---- 3. 轮询字节序 ----
        poll, table = self.probe_byte_order()
        caps["poll_order"] = poll
        caps["poll_table"] = table
        self.byte_order = poll
        notes.append(f"轮询字节序判定为 {poll}（{ByteOrder.DESC.get(poll, '?')}）")
        caps["push_order"] = self.push_byte_order
        caps["write_order"] = self.write_byte_order

        caps["probed"] = True
        self.caps = caps
        return caps

    # ------------------------------------------------------------------
    #  写入字节序往返自检（会写设备，但自愈、不留副作用）
    # ------------------------------------------------------------------
    def verify_write_order(self, addr: int = Reg.OVERLOAD, n: int = 6,
                           tol: float = 1e-3) -> Tuple[str, List[Tuple[str, float]]]:
        """
        用「写入 → 回读」往返测试确定 FC10 写路径的 float32 字节序。

        判据设计（关键，两个坑都要绕开）：

        坑 1：不能写回当前值再回读相比 —— 那样「写入被设备拒绝（值越界）」与
              「写对了」回读结果完全相同，四种字节序会全部被判为正确。
              → 必须写一个与当前值明显不同的目标值。

        坑 2：各轮必须用**互不相同的**目标值 —— 否则某一轮写成功后，后面
              错误字节序的写入被越界拒绝，回读仍是已写入的那个目标值，
              依旧会被误判为正确。
              → 每轮目标值取 base 的不同比例。

        过程自愈：测完把原值写回，并回读校验；万一判定有误，逐个字节序重试
        直到确认原值已恢复，保证设备参数不变。

        返回 (判定结果, [(字节序, 回读偏差), ...])。
        """
        base = self.read_floats(addr, n)
        saved = self.write_byte_order
        factors = (0.50, 0.65, 0.80, 0.90)
        table: List[Tuple[str, float]] = []
        winner = ""
        for order, f in zip(ByteOrder.ALL, factors):
            target = [v * f if abs(v) >= 2.0 else (1.0 if v >= 0 else -1.0)
                      for v in base]
            self.write_byte_order = order
            try:
                self.write_floats(addr, target)
                got = self.read_floats(addr, n)
                err = max(abs(a - b) for a, b in zip(got, target))
            except ModbusError:
                err = float("inf")
            table.append((order, err))
            if err <= tol and not winner:
                winner = order

        # ---- 还原原值（自愈：直到回读确认与原值一致）----
        self.write_byte_order = winner or saved
        for order in ([winner] if winner else []) + [
                o for o in ByteOrder.ALL if o != winner]:
            if not order:
                continue
            self.write_byte_order = order
            try:
                self.write_floats(addr, base)
                if max(abs(a - b) for a, b in
                       zip(self.read_floats(addr, n), base)) <= tol:
                    self.write_byte_order = order
                    break
            except ModbusError:
                continue
        else:
            self.write_byte_order = winner or saved
        return self.write_byte_order, table

    # ------------------------------------------------------------------
    #  字节序自动探测
    # ------------------------------------------------------------------
    def probe_byte_order(self) -> Tuple[str, List[Tuple[str, float, float, bool]]]:
        """
        自动判定 FC03 轮询路径的 float32 字节序。

        用两组「出厂默认值」交叉判据（都能唯一确定字节序）：
          * 阈值范围 0x007C~0x007F → Fx 最小值 0.1、最大值 50000.0
          * 过载阈值 0x0070~0x007B → 2000, 2000, 5000, 200, 200, 200

        返回 (推荐字节序, [(字节序, Fx最小值, Fx最大值, 是否合理), ...])
        """
        raw = self.read_bytes(Reg.RANGE, 4)          # Fx Min + Fx Max
        try:
            ref = self.read_bytes(Reg.OVERLOAD, 12)
        except ModbusException:
            ref = b""
        expect = (2000.0, 2000.0, 5000.0, 200.0, 200.0, 200.0)

        results: List[Tuple[str, float, float, bool]] = []
        best, best_score = None, -1
        for order in ByteOrder.ALL:
            lo = bytes_to_float(raw[0:4], order)
            hi = bytes_to_float(raw[4:8], order)
            ok = _is_finite(lo) and _is_finite(hi) \
                and 0.0 < lo < 1000.0 and hi > lo > 0.0
            score = 0
            if ok:
                score += 10
                if lo < 10:
                    score += 5
                if 100 <= hi <= 1e7:
                    score += 5
            # 过载阈值命中出厂默认值 → 强证据
            if len(ref) >= 12:
                hits = sum(
                    1 for i in range(6)
                    if _is_finite(bytes_to_float(ref[i * 4:i * 4 + 4], order))
                    and abs(bytes_to_float(ref[i * 4:i * 4 + 4], order) - expect[i])
                    <= max(abs(expect[i]) * 1e-3, 1e-6))
                score += hits * 10
            results.append((order, lo, hi, ok))
            if score > best_score:
                best, best_score = order, score
        return (best or self.byte_order), results

    # ------------------------------------------------------------------
    #  内部工具
    # ------------------------------------------------------------------
    def _emit_state(self, connected: bool, msg: str) -> None:
        if self.on_state:
            self.on_state(connected, msg)

    def _emit_error(self, msg: str) -> None:
        if self.on_error:
            self.on_error(msg)

    def _fail(self, msg: str) -> None:
        self.error_count += 1
        self._running = False
        self._emit_error(msg)
        self._emit_state(False, msg)


def _is_finite(x: float) -> bool:
    return x == x and abs(x) != float("inf")


# =====================================================================
#  5. 简单自测入口
# =====================================================================
def _demo() -> None:                                  # pragma: no cover
    import argparse

    ap = argparse.ArgumentParser(description="六维力传感器 Modbus-TCP 命令行自测")
    ap.add_argument("--host", default="192.168.1.12")
    ap.add_argument("--port", type=int, default=502)
    ap.add_argument("--order", default=ByteOrder.POLL_DEFAULT,
                    choices=list(ByteOrder.ALL), help="FC03 轮询读字节序")
    ap.add_argument("--push-order", default=ByteOrder.PUSH_DEFAULT,
                    choices=list(ByteOrder.ALL), help="主动推流字节序")
    ap.add_argument("--write-order", default=ByteOrder.WRITE_DEFAULT,
                    choices=list(ByteOrder.ALL), help="FC10 写字节序")
    args = ap.parse_args()

    cli = SensorClient(args.host, args.port, byte_order=args.order,
                       push_byte_order=args.push_order,
                       write_byte_order=args.write_order)
    cli.on_log = lambda d, t: print(f"[{d}] {t}")
    cli.connect()
    try:
        print("SN       :", cli.get_sn())
        print("固件版本 :", cli.get_fw_version_str())
        print("状态     :", cli.describe_status(cli.get_status()))
        print("通信错误 :", cli.get_comm_error())
        print("网络参数 :", cli.get_network())

        caps = cli.probe_capabilities()
        print("\n--- 设备能力探测 ---")
        print("力数据可读寄存器数 :", caps["force_regs"])
        print("阈值范围可读寄存器数:", caps["range_regs"])
        print("轮询/推流/写入字节序:", caps["poll_order"], caps["push_order"],
              caps["write_order"])
        for n in caps["notes"]:
            print("  *", n)
        for o, lo, hi, ok in caps["poll_table"]:
            print(f"    {o}: min={lo:<12.6g} max={hi:<12.6g} {'合理' if ok else '-'}")

        print("\n阈值范围 :", cli.get_ranges())
        print("过载阈值 :", cli.get_overload())
        print("六维数据 :", cli.get_force())
        vals = cli.single_shot()
        src = (f"设备回送帧 {cli.push_byte_order}"
               if cli.last_single_src == "push"
               else f"回退轮询读 0x0033 {cli.byte_order}")
        print(f"单次采样 : {vals}   （来源：{src}）")
    finally:
        cli.close()


if __name__ == "__main__":                            # pragma: no cover
    _demo()
