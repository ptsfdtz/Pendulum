# PendulumLab

一级、二级倒立摆共用一个启动脚本：

```powershell
powershell -ExecutionPolicy Bypass -File .\start_pendulum.ps1
```

- `1` + 回车：一级起摆并稳定。
- `2` + 回车：二级起摆并稳定。
- 运行中按 `Q` / `Esc`：停止并返回选择；选择界面输入 `Q` 退出。
- `Ctrl+C`：急停。

自动流程包含回中、下垂零点确认、起摆和稳定控制。界面保留模式、状态及必要错误，详细数据写日志。物理限位、行程保护和输出停止机制继续生效。

使用 NI PCI-6602 读取编码器、限位并控制 Servo，Advantech PCI-1723 AO0 输出速度指令。

[控制台说明](docs/CONSOLE.md) · [MATLAB 一级/二级离线复现](matlab/README.md) · [接线定义](docs/引脚定义.md)

## 开发验证

```powershell
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --preset vs2022-x64
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build --preset release --target pendulum_console pendulum_tests
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe' --preset release --output-on-failure
```