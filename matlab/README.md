实物 MATLAB 入口及板卡验证请见 [实物迁移说明](hardware/README.md)。下文描述的是独立的离线模型。

# MATLAB 离线模拟与仿真

本目录保留一级、二级倒立摆的起摆与稳摆。二级来自原 `Pendulum` 工程；一级原始模型来自 `E:\直线倒立摆库`，另配可独立运行的离线数学模型。原始目录及其历史实验数据保留。

## 快速复现

在 MATLAB 中进入本 `matlab` 目录：

```matlab
single_swing = run_pendulum('single','swingup'); % 一级：下垂起摆→LQR 稳摆
single_hold  = run_pendulum('single','balance'); % 一级：直立附近单独稳摆
double_swing = run_pendulum('double','swingup'); % 二级：三阶段起摆→LQR 稳摆
double_hold  = run_pendulum('double','balance'); % 二级：直立附近单独稳摆
```

无界面仿真：

```matlab
result = run_pendulum('single','swingup',struct('show_ui',false));
assert(result.passed);
```

纯数值仿真（不运行 Simulink）：

```matlab
result = run_pendulum('double','balance',struct('engine','numeric'));
assert(result.passed);
```

四个场景均支持 `engine='numeric'` 和 `engine='simulink'`，默认基础 MATLAB 数值引擎（不需要工具箱）；显式 `engine='simulink'` 才运行 Simulink。运行 `report = verify_pendulum()` 仅用基础 MATLAB 检查四个数值场景；`verify_pendulum({'numeric','simulink'})` 才检查两种引擎，共八项。一级两个场景与二级起摆默认 50 秒，二级单独稳摆 15 秒。原有 `setup_pendulum; result = run_simulation;` 仍是二级完整起摆入口。

统一入口每次把参数、信号和结果保存到 `output/<模型>/` 下的独立运行目录，返回 `result.output_file`。一级 Simulink 显示状态 Scope，二级保留原动画和 Scope；数值入口返回信号数组。

需要 MATLAB；Simulink 入口需要 Simulink，LQR 设计和参数搜索需要 Control System Toolbox。无需 DAQ 或实时硬件工具箱。初始化后可在其他工作目录调用入口。

## 文件组织

| 路径 | 用途 |
| --- | --- |
| `setup_pendulum.m` | 添加源码路径，不递归加入输出及历史资料 |
| `run_pendulum.m`、`verify_pendulum.m` | 四场景统一入口与八项验证 |
| `single_pendulum/sp_config.m`、`sp_dynamics.m` | 一级参数与非线性数学模型 |
| `single_pendulum/sp_control.m` | 一级离线能量起摆和 LQR 稳摆 |
| `single_pendulum/sp_simulate.m`、`sp_build_model.m` | 一级数值仿真与 Simulink 生成脚本 |
| `single_pendulum/reference/` | 一级原始 SLX、ETlab 依赖库和 LQR 设计脚本，仅归档 |
| `single_pendulum/models/`、`double_pendulum/models/` | 四个离线场景的已验证 SLX 快照；运行入口可重新生成 |
| `double_pendulum/dp_config.m` | 物理参数、初始状态、采样周期和控制参数 |
| `double_pendulum/dp_scenario_config.m` | 二级起摆/单独稳摆场景及初始状态 |
| `double_pendulum/dp_dynamics.m` | 非线性动力学 |
| `double_pendulum/dp_controller.m`、`dp_swingup_control.m` | 控制器和三阶段起摆逻辑 |
| `double_pendulum/dp_best_lqr.m` | 当前保存的 LQR 参数 |
| `double_pendulum/dp_linearize.m`、`dp_lqr_design.m` | 线性化与 LQR 设计 |
| `double_pendulum/dp_simulate_*.m` | 数值仿真 |
| `double_pendulum/build_model.m`、`reference/Double_Pendulum_Project.slx` | 模型生成脚本与迁移时的模型快照 |
| `double_pendulum/run_simulation.m`、`dp_animation.m` | Simulink 运行、指标计算和动画 |
| `double_pendulum/dp_*optimize*.m`、`dp_evaluate_*.m` | 离线搜索和候选评估 |
| `double_pendulum/reference/` | 原优化报告和参考论文 |
| `output/double_pendulum/` | 重建模型、缓存、搜索结果及验证数据，Git 忽略 |
| `migration_record.json` | 原始源码哈希、迁移范围与验证记录 |
| `single_migration_record.json` | 一级原始资料来源及离线适配说明 |
| `validation_report.json` | 最近一次八场景验证的结果与数值指标 |

`run_simulation` 每次依据脚本重建模型，生成的 SLX 放到输出目录。需要永久修改模型结构时，请修改 `build_model.m`；只修改生成模型会在下次运行时被重建。

原文件夹较多是因为参数扫描为每个候选生成 CSV，同时保存了 MAT 轨迹、检查点、Simulink 缓存和临时文件。这些不是运行依赖，因此未整体复制。历史报告中的指标是原实验记录。

## 一级模型的来源与适配范围

`single_pendulum/reference/Copy_of_LQR_lp1_1.slx` 与 `ETlabLibrary.slx` 完整保留原起摆、稳摆及 ACC2VOL 逻辑。它们包含实时硬件接口，不是离线运行入口，也不自动加入 MATLAB 路径。

一级离线模型是新增的仿真适配：根据 `P_1_1.m` 使用 `m=0.134 kg`、`l=0.2285 m`、`g=9.8 m/s²`，关节惯量取 `4*m*l²/3`；状态顺序为 `[x theta xdot thetadot]`，控制输入为小车加速度。LQR 反馈保留原模型的 `10, -58.6, 12.23, -10.69` 系数。

起摆保留能量控制→LQR 的结构，但使用直立零点下的一致物理能量，以及离线速度门限、制动距离和切换角速度判据；它不是原硬件控制器逐采样的等价移植。速度门限 `0.30 m/s` 经当前理想模型验证。离线模型不模拟编码器量化、ACC2VOL、伺服或电压转换。原硬件参数与顺序可从归档模型及 C++ `ReferenceLqrVelocityController` 查阅。

一级阶段 `1=起摆, 2=LQR`；二级阶段 `1=第一杆起摆, 2=第二杆起摆, 3=双杆LQR`。起摆测试要求经过相应阶段并在末尾稳定；单独稳摆从小角度偏差开始，要求保持在轨道与输入限幅内并收敛。

一级 Simulink 通过解释执行适配块调用同一份 MATLAB 控制与动力学函数，并用零阶保持实现 1 ms 控制采样。此模型用于离线验证，不用于直接生成实时控制代码。二级保留原 MATLAB Function 模型结构。

## 重新优化

```matlab
setup_pendulum;
search = dp_lqr_optimize(struct('make_plots',false));
% 更耗时的第二阶段鲁棒性搜索：
% search = dp_optimize_stage2_robust();
```

LQR 搜索将候选 `dp_best_lqr.m` 写入输出目录。确认结果后，再将它复制替换 `double_pendulum/dp_best_lqr.m`，运行 `clear functions` 并重新仿真。输出目录不自动加入路径，搜索不会直接替换保存的基线参数。自定义候选评估的 `outputFile` 时，文件位置遵循调用者传入的路径。

## 应用于其他模型

1. 复制 `double_pendulum` 到新模型目录，为函数及 Simulink 模型改用独立名称，修改 `dp_output_dir.m` 中的输出目录名称，避免多模型函数冲突。
2. 在 `dp_config.m` 设置质量、惯量、几何尺寸、初态和限幅；在 `dp_dynamics.m` 替换动力学。当前参数并非通用默认值。
3. 当前状态顺序为 `[x theta1 theta2 xdot theta1dot theta2dot]`，角度以竖直向上为零，两个角度均为绝对角。位置单位 m，角度 rad，控制输入为小车加速度 m/s²。
4. 状态维度或输入定义改变时，同步调整线性化、控制器、数值积分器、Simulink 接线及动画。三阶段能量起摆逻辑针对当前双摆结构，需要重新推导。
5. 重新设计 LQR 并验证起摆、稳定性、行程和饱和。保留采样周期测试：当前不连续控制律对采样周期敏感。

新增 `hardware/` 已提供 MATLAB 实物传感器读取、相对编码器转换、加速度到电压映射及控制循环，验证范围见其说明。本目录复现的是离线模型，仿真参数不可直接等同于设备电压。
