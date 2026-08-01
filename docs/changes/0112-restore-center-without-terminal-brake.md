# 0112 - 中心点恢复为无末段反制动

## 修改

- `TASK_TO_CENTER` 不再调用 `ApplyTerminalCaptureBrake()`。
- 中心点恢复为原有的位置-速度级联、末段微调和静摩擦处理。
- `TASK_TO_NEGATIVE` 保留当前较低强度的末段反制动；`+5 cm` 保持原有独立逻辑与参数。

## 验证

- Keil `F407_STEP2_EmmV5` 全量编译通过，`0 Error(s), 0 Warning(s)`。
