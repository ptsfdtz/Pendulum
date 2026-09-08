# PendulumLab — MATLAB 实物控制迁移

在项目根目录的 MATLAB 命令窗口运行：

```matlab
start_pendulum       % 输入 1 或 2
% 或直接选择：
start_pendulum(1)    % 一阶：自动回中、下垂零点、起摆、稳摆
start_pendulum(2)    % 二阶：自动回中、下垂零点、三阶段起摆、稳摆
```

**上述入口会使能伺服并移动设备。当前已完成源码移植、只读板卡验证和离线测试，尚未完成 MATLAB 实物起摆验收。** 默认控制运行 60 秒（不含回中及零点等待）；运行中 `Ctrl+C` 触发停机清理。

运行需要 Windows 64 位 MATLAB（已测试 R2021a）、NI-DAQmx 的 .NET 组件、研华 DAQNavi 的 .NET 组件。实物入口只调用 `.m` 文件和厂商驱动，不需要 C++ 可执行程序、CMake、编译器、MEX、Simulink 或 Data Acquisition Toolbox。MATLAB 版本直接从 `start_pendulum.m` 启动，不使用 PowerShell 脚本。现有 `start_pendulum.ps1` 保持旧 C++ 启动方式。

- [实物迁移说明、依赖与验证边界](matlab/hardware/README.md)
- [原有 MATLAB 仿真](matlab/README.md)
- [接线定义](docs/引脚定义.md)

旧 C++ 源码和配置保留供对照，MATLAB 实物运行不读取它们。不要同时打开旧版控制程序和 MATLAB 控制任务。
