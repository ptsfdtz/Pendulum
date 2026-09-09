# 原生 Simulink：已恢复的可运行版本

按用户要求，恢复到首次完成 **30 秒 / 3,000 周期一阶起摆及稳摆实测** 的版本。后续自动重试、启动流程调整和诊断改动已撤回；原有 `PhIO.m`、`ph_home.m` 已还原。

在项目根目录的 MATLAB 命令窗口运行：

```matlab
start_pendulum_simulink(1)
```

- `Pendulum_Native_1.slx`：一阶模型，`StopTime = inf`。
- `Pendulum_Native_2.slx`：二阶模型，`StopTime = inf`，仅完成离线验证。
- 起摆、稳摆、状态估计、积分及限位控制使用原生 Simulink 模块。
- 板卡 I/O、回中和下垂零点采集沿用 MATLAB 接口。
- 模型打开和编译时自动补齐本目录及 `matlab/hardware` 路径。打开 `Pendulum_Native_1.slx` 后直接点击 **Run**，即执行输入预检、回中、下垂标零、起摆和稳摆，时间为 `inf`；按 Stop 关闭输出。
- 只读测试仍可显式运行 `start_pendulum_simulink(1,'readonly',5)`。二阶模型直接 Run 仍保持只读模式。
- 回中和标零之后，首帧初始化期间保持伺服关闭；首个控制量就绪并复核位置、角度和限位后才使能并建立控制计时。运行中的 10 ms（一阶）计算周期和 50 ms 采样超时保护保留。
- 按 `Ctrl+C` 或 Simulink Stop 停止。此恢复版本没有后续新增的自动重试。

已通过实物运行的原始记录：
`../output/native_simulink/tpe03f1b50_7cd1_40ab_962a_a2a1a2df353a/`

该轮完成起摆、稳摆，3,000 个真实采样点与 MATLAB 原算法回放差异为 0。每阶离线对照包含 15,506 个采样点，一阶最大差异 0，二阶约 `3.55e-15`。

撤回前的文件已归档至：
`../output/native_simulink/rollback_archive_20260909_100216/`

此次恢复仅重建模型并进行离线编译/数值检查，不再操作硬件。
