# 0151 - UART5 START 接收诊断

## 修改

- UART5 在收到完整 `START` 或 `start` 后，通过 USART1 调试串口输出：
  `VEHICLE,START,RECEIVED`。
- UART5 的 `PD2` 接收引脚显式配置为 `GPIO_AF8_UART5`，避免依赖前一次 GPIO 结构体的残留设置。

## 判定方式

- 看不到 `VEHICLE,START,RECEIVED`：检查小车板 TX 是否接到本板 `PD2`、两板是否共地、双方是否均为 `115200 8N1`。
- 看到该行后紧跟 `CLOSED_LOOP,REJECT,...`：UART5 已正常收到命令，问题在视觉或电机反馈条件。
- 看到该行及 `CLOSED_LOOP,ENABLED`：中心闭环和起步前馈已生效。
