# 初始闭环工程基线

- 代码提交：`75762ad`
- 日期：2026-07-31

## 修改内容

- 建立 STM32F407、Emm_V5 闭环步进电机和视觉串口控制工程。
- 纳入 CubeMX 配置、Keil 工程、UART/DMA 接收、电机位置反馈、视觉闭环和数据采集实现。
- 新增 `.gitignore`，排除 Keil 可重新生成的构建输出、个人配置和本地日志。

## 涉及文件

- `Core/`
- `F407_STEP2_EmmV5.ioc`
- `MDK-ARM/`
- `.gitignore`

## 验证

- Keil 构建通过，`0 Error(s), 0 Warning(s)`。
