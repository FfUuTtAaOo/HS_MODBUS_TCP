# 协议整合到开发需求里面去

## modbus-tcp 协议

见根目录 modbus-tcp-protocol-v2.0.md

## rs485 协议

见 中科米点六维力传感器力控通信协议RS485V1.4

## 协议开发目标

电路板有w5500 和 max13487 芯片，要实现modbus-tcp协议，利用485对电路进行部分配置

1. 完整实现modbus-tcp协议

2. 485 实现以下内容

    - 停止数据转换和发送

    - 以固定频率发送数据（频率暂时不用确认）

    - 单次转换，也就是一发一收

    - 清零 0x20

    - 取消清零 0x 21

    - debug模式（0x31: 退出模式；0x32: 进入模式）

    - 数据格式切换（0x33: kg单位；0x34: 原始mV；0x35: N单位）

    - 上传解耦矩阵指令：矩阵6*6，可以用6条指令来上传，比如0x22



### 补充解释

- debug模式下固定频率转换和单次转换数据由16进制切换成ascii模式

- 退出debug固定频率转换和单次转换数据由ascii模式切换成16进制

- 上传矩阵样例（6*6矩阵，6条指令，1条上传6个浮点数）

0xAA552201 DA0F4940DA0F49C0 DA0F4940DA0F49C0 DA0F4940DA0F49C 00D0A

0xAA552202 DA0F4940DA0F49C0 DA0F4940DA0F49C0 DA0F4940DA0F49C 00D0A

0xAA552203 DA0F4940DA0F49C0 DA0F4940DA0F49C0 DA0F4940DA0F49C 00D0A

0xAA552204 DA0F4940DA0F49C0 DA0F4940DA0F49C0 DA0F4940DA0F49C 00D0A

0xAA552205 DA0F4940DA0F49C0 DA0F4940DA0F49C0 DA0F4940DA0F49C 00D0A

0xAA552206 DA0F4940DA0F49C0 DA0F4940DA0F49C0 DA0F4940DA0F49C 00D0A