# 0122 - OLED 小车计时改为 MM:SS

## 修改

- OLED 显示格式由 `HH:MM:SS` 改为 `MM:SS`。
- 显示小车任务的分钟和秒，分钟到 `99` 后回绕。
- 数字改为 3 倍点阵，便于 0.96 寸 OLED 远距离查看。

## UART5 开始命令

```text
START,T1,S47\r\n
```

- 串口设置：115200、8N1、CRLF。
- 收到 `STOP,T1,MSxxxx\r\n` 后停止并显示最终时间。

## 验证

- Keil `F407_STEP2_EmmV5` 全量编译通过，`0 Error(s), 0 Warning(s)`。
