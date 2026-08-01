# 0121 - OLED 显示小车 UART 任务时间

## UART5 协议处理

- 按手册使用 128 字节 ASCII 行缓冲，以 `\r\n` 结束一帧。
- 收到 `START,Tn,Sxx` 后开始 MCU 侧计时。
- 收到 `STOP,Tn,MSxxxx` 后停止计时，并显示小车帧中的 `MS` 时间。
- 运行过程帧 `Tn,MS...` 被接收并丢弃，不影响闭环控制。

## OLED 显示

- OLED 由显示上电运行时间改为显示小车任务时间，格式为 `HH:MM:SS`。
- `START` 和 `STOP` 到达时立即刷新；任务运行中每秒刷新一次。

## 验证

- Keil `F407_STEP2_EmmV5` 全量编译通过，`0 Error(s), 0 Warning(s)`。
