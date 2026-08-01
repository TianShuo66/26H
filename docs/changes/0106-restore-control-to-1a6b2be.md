# 0106 - 恢复控制代码至 1a6b2be

## 修改

- `Core/Src/main.c` 的已提交控制逻辑恢复到 `1a6b2be` (`Speed up negative terminal capture braking`)。
- 后续中心点专属参数、静摩擦脱困与中心点反制动改动不再参与运行。
- 保留此前变更记录，Git 历史仍可用于恢复任意版本。

## 保留的本地参数

- 本地未提交的任务时长、反向助推时长和标定说明保持不变，未纳入本次恢复提交。

## 验证

- Keil `F407_STEP2_EmmV5` 全量编译通过，`0 Error(s), 0 Warning(s)`。
