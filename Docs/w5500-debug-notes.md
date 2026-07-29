# W5500 调试记录：Socket Init FAIL 问题排查与修复

> 日期：2026-07-19 | 版本：v1.0

---

## 1 故障现象

W5500 上电初始化后，芯片版本读取正常 (`ver=0x04`)，IP 配置正确，PHY Link UP，但在 Socket OPEN 阶段失败：

```
--- W5500 Init Start ---
Resetting W5500...
W5500 OK (ver=0x04)
Configuring network: IP=192.168.1.12:502
IP readback: 192.168.1.12
Waiting for PHY link...
PHY link UP
Socket init FAIL SR=A3          ← 失败：SR=0xA3 不是任何合法状态值
```

---

## 2 根因分析

`0xA3` 不是 W5500 合法的 Socket 状态寄存器值（合法值为 `0x00~0x15`），说明 **Sn_SR 读回的是未初始化内存块的垃圾值**。

经过排查，问题出在三个层面：

### 2.1 BS[4:0] 块选择位编码错误（核心 Bug）

W5500 的 VDM 模式 SPI 帧中，BS[4:0] 是 5 位区域选择码。其布局是 **交错排列** 的，不是分组排列：

```
BS[4:0] = 0x00   → Common Register 区
BS[4:0] = 0x01   → Socket 0 Register 区
BS[4:0] = 0x02   → Socket 0 TX Buffer 区
BS[4:0] = 0x03   → Socket 0 RX Buffer 区
BS[4:0] = 0x04   → Socket 1 Register 区
BS[4:0] = 0x05   → Socket 1 TX Buffer 区
BS[4:0] = 0x06   → Socket 1 RX Buffer 区
...
```

**通用公式**：Socket n 的 Reg=1+4n、TX=2+4n、RX=3+4n

原来的代码将 TX/RX 缓冲区的 BS 值错误地写成：
```c
// ❌ 错误写法
#define W5500_BS_TX_BUF(n) ((uint8_t)(0x10 + (n)))  // 0x10 = Socket 4 寄存器区！
#define W5500_BS_RX_BUF(n) ((uint8_t)(0x18 + (n)))  // 0x18 = Socket 6 寄存器区！
```

`0x10=16` 是 Socket 4 的寄存器区 BS，`0x18=24` 是 Socket 6 的寄存器区 BS —— 当代码用这些值去读 Socket 0 的 TX/RX Buffer 时，读回的是完全不相关的内存段。

**正确写法**：
```c
// ✅ 正确写法
#define W5500_BS_SOCK(n)   ((uint8_t)(1 + 4 * (n)))  // Reg 区
#define W5500_BS_TX_BUF(n) ((uint8_t)(2 + 4 * (n)))  // TX Buffer 区
#define W5500_BS_RX_BUF(n) ((uint8_t)(3 + 4 * (n)))  // RX Buffer 区
```

### 2.2 缓冲区分配必须批量写入全部 8 个 Socket

W5500 的 TX/RX 缓冲区内存分配是**批量触发机制**：

- 必须先向 **所有 8 个 Socket** 写入 `Sn_RXBUF_SIZE` 和 `Sn_TXBUF_SIZE`
- 等 8 个 Socket 全部写入完成后，硬件才会一次性完成内存分配
- **在分配完成之前，任何 Socket 的 OPEN 命令都不会生效**

原来的代码在 `w5500_socket_init()` 中只写了单个 Socket 的 buffer size 就执行 OPEN，此时内存尚未分配，OPEN 必然失败。

### 2.3 Sn_KPALVTR 必须在 OPEN 之前写入

W5500 Datasheet 要求 `Sn_KPALVTR`（TCP Keep-Alive 定时器）在 Socket 处于 CLOSED 状态时配置。OPEN 之后再写入是无效的。

---

## 3 修复内容

### 3.1 `w5500.h` — 修正 BS 宏

```c
// 文件：Core/Inc/w5500.h
#define W5500_BS_COMMON    0x00
#define W5500_BS_SOCK(n)   ((uint8_t)(1 + 4 * (n)))  /* Socket n Reg 区 */
#define W5500_BS_TX_BUF(n) ((uint8_t)(2 + 4 * (n)))  /* Socket n TX Buffer */
#define W5500_BS_RX_BUF(n) ((uint8_t)(3 + 4 * (n)))  /* Socket n RX Buffer */
```

### 3.2 `w5500.c` — 新增批量缓冲配置函数

```c
// 文件：Core/Src/w5500.c
void w5500_configure_buffers(void)
{
    for (uint8_t n = 0; n < 8; n++) {
        uint8_t block = W5500_BS_SOCK(n);
        uint8_t size  = (n < W5500_SOCK_COUNT) ? 0x02 : 0x00;
        w5500_write_byte(block, Sn_RXBUF_SIZE, size);
        w5500_write_byte(block, Sn_TXBUF_SIZE, size);
    }
}
```

**注意**：必须在 `main.c` 的初始化流程中，**在所有 `w5500_socket_init()` 调用之前** 先调用 `w5500_configure_buffers()`。

### 3.3 `w5500_socket_init()` — 调整寄存器写入顺序

```c
// 文件：Core/Src/w5500.c
uint8_t w5500_socket_init(uint8_t sock, uint16_t port)
{
    // ① Sn_MR = TCP
    // ② Sn_PORT = port
    // ③ Sn_KPALVTR = 0x03    ← 必须在 OPEN 之前
    // ④ Sn_CR = OPEN          ← 触发 OPEN
    // ⑤ HAL_Delay(2)          ← 等待命令生效
    // ⑥ 轮询 Sn_SR 直到 SOCK_INIT
}
```

### 3.4 `main.c` — 调用缓冲配置 + 改进错误日志

```c
// 在 PHY Link UP 之后，所有 socket init 之前：
w5500_configure_buffers();

// 初始化失败时打印具体 socket 编号：
static const char *init_fail[] = {"S0 init FAIL SR=", "S1 init FAIL SR=", ...};
```

---

## 4 W5500 初始化正确流程（总结）

```
1. w5500_hw_reset()
2. w5500_read_version()    → 验证芯片可通信
3. w5500_set_mac()
4. w5500_set_ip()
5. w5500_set_gateway()
6. w5500_set_subnet()
7. w5500_configure_buffers()   ← 批量分配全部 Buffer（NEW!）
8. 等待 PHY link UP
9. w5500_socket_init(0)        ← 每个 Socket: MR → PORT → KPALVTR → OPEN
10. w5500_socket_listen(0)     ← OPEN 成功 → LISTEN
11. 主循环: 轮询 Sn_IR/Sn_SR + 数据收发
```

---

## 5 关键经验教训

| # | 教训 |
|:---|:---|
| 1 | **BS[4:0] 是交错编码**，不是分组编码。官方 Datasheet 中 "Block Select Bits" 表格是唯一参考 |
| 2 | **不要假设** TX/RX Buffer 的 BS 值是连续排布的；对照 Datasheet 逐位验证 |
| 3 | W5500 Buffer 分配**必须一次写完所有 8 个 Socket** 才生效 |
| 4 | OPEN 之前需要写的寄存器：`Sn_MR` → `Sn_PORT` → `Sn_KPALVTR`，顺序不能错 |
| 5 | OPEN 命令发出后需要 **等待 1~2ms** 再读取 `Sn_SR`，否则可能读到旧状态 |
| 6 | 调试信息应包含 **Socket 编号**，方便多 Socket 场景定位问题 |
| 7 | 任何 `SR` 值不在合法范围 (0x00~0x15) 时，说明 **地址映射有问题**，而非 Socket 状态机异常 |

---

## 6 参考

- W5500 Datasheet v1.1.0 — Block Select Bits table, Socket n Register map
- 项目文件：`Core/Inc/w5500.h`, `Core/Src/w5500.c`, `Core/Src/main.c`
