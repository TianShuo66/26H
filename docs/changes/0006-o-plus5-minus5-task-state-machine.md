# O 点至 +5 cm 再至 -5 cm 任务状态机

- 代码提交：本文件所在提交（提交说明：`Add O-to-plus5-to-minus5 task state machine`）
- 日期：2026-07-31

## 任务流程

1. 从中心附近启动。
2. 目标设为 `+5.0 cm`。
3. 钢球到达 `+4.5 cm` 后，立即将目标切换为 `-5.0 cm`。
4. 钢球处于 `-5.0 ±1.0 cm` 且速度不超过 `0.5 cm/s`，持续 `120 ms` 后输出完成。
5. 从任务启动起超过 `5 s` 未完成，立即停止电机并输出超时。

`+4.5 cm` 的折返阈值为 `+5 cm` 的 `0.5 cm` 容差内，可在高速时提前进入返程，避免等待完全静止而浪费任务时间。

## 按键

- `KEY1`：空闲时启动完整往返任务；任务或手动闭环中再次按下立即停止。
- `KEY2`：手动保持 `0 cm`。
- `KEY3`：手动保持 `+5 cm`。
- `KEY4`：手动保持 `-5 cm`。

任务启动前要求钢球距中心不超过 `1.5 cm` 且速度不超过 `0.5 cm/s`；不满足时串口输出 `TASK,REJECT_START_POSITION`。

## 串口事件

- `TASK,START,MS,0`
- `TASK,REVERSE,MS,<elapsed>`
- `TASK,COMPLETE,MS,<elapsed>`
- `TASK,TIMEOUT,MS,<elapsed>`

## 执行速度调整

- 单次短脉冲最大值由 `8` 提高到 `12`。
- 静态偏置由每 `200 ms` 增加 `2` 脉冲，改为每 `80 ms` 增加 `10` 脉冲。
- 这些调整用于消除日志中钢球在起动倾角下长时间不动的问题。

## 涉及文件

- `Core/Src/main.c`
- `Core/Src/usart.c`
- `Core/Inc/usart.h`

## 验证

- Keil 构建通过，`0 Error(s), 0 Warning(s)`。
- 尚未烧录到实物；是否在 `5 s` 内完成需以 `TASK,COMPLETE` 的实际串口时间为准。
