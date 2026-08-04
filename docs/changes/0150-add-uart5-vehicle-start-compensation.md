# 0150 - UART5 小车起步前馈补偿

## 功能

- 配置 UART5 为 `115200, 8N1`，`PC12` 为 TX、`PD2` 为 RX。
- UART5 接收任意连续的 `START` 五个 ASCII 字符即触发，无需换行。
- 触发后进入 `X=0` 的中心闭环；若已在闭环中，则切换为中心目标。
- 起步后的 `600 ms` 额外叠加 `-35 pulse` 倾角，用于抵消小车向视觉 X 正方向起步时钢球向 X 负方向的惯性偏移。

## 调参

- `VEHICLE_START_COMPENSATION_PULSES`：若小车起步方向相反，改为正值；可调整其绝对值。
- `VEHICLE_START_COMPENSATION_DURATION_MS`：按小车实际加速时间调整。

## 验证

- Keil 全量编译：`0 Error(s), 0 Warning(s)`。
