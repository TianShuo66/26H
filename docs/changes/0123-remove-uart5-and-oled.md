# 0123 - 移除 UART5 与 OLED 功能

## 修改

- 移除 UART5 的初始化、中断接收和小车 `START/STOP` 协议解析。
- 移除 I2C2、SSD1306 OLED 驱动以及时间显示刷新逻辑。
- 回收 `PB10`、`PB11`、`PC12`、`PD2` 的 Cube 引脚配置和 Keil I2C 源文件引用。

## 保留项

- 既有 UART1、USART2、UART4 功能和电机闭环控制不变。
- 既有 UART5/OLED 变更文档保留，便于以后恢复。

## 验证

- Keil `F407_STEP2_EmmV5` 全量编译通过，`0 Error(s), 0 Warning(s)`。
