# PendulumLab

Windows C++ controller for a single-stage linear inverted pendulum using:

- NI PCI-6602 for the motor encoder, pendulum encoder, limits, and Servo enable
- Advantech PCI-1723 AO0 for the Yaskawa SGD7S velocity command

## Balance controller

The live balance path is the manual-upright branch of
`E:\直线倒立摆库\demo\Copy_of_LQR_lp1_1.slx` (model version 4.37).
The previous PD/LQR controllers and offline gain-learning tools have been removed.

The implementation preserves the reference model's signs, constants, unit-delay
initial conditions, saturation behavior, and update ordering:

```text
theta = wrap(-(pendulum_count - start_count) * 2*pi/8000)
x     = -(motor_count - start_count) * 0.163/8000

theta_dot = (theta - theta_previous) / 0.01
x_dot     = (x - x_previous) / 0.01

acc = clamp(
    -58.6*theta
    -10.69*theta_dot
    +10*x
    +12.23*x_dot,
    -10, +10)
```

The reference `ACC2VOL` block is also reproduced:

- control update: 0.01 s
- integrator multiplier `Ts`: 0.005 s, exactly as stored in the model
- velocity reference limit: ±0.6 m/s
- velocity PI: P=0.18, I=54
- AO0 limit: ±1 V
- AO0 zero command: 0 V

Automatic `Swing_up` is intentionally disabled for this stage. The operator
must hold the pendulum within ±30 degrees of upright before `balance start`.
The controller stops if the pendulum leaves that LQR region.

## Safety retained

The controller remains wrapped by the existing independent safety system:

- active-HIGH left and right physical limits with debounce
- AO0=0 V before Servo OFF on a limit, monitor fault, exception, Ctrl+C, or exit
- fresh two-limit `home center` required before every balance session
- balance start restricted to the configured center window
- software travel envelope stops balance before a physical limit
- manual Servo, homing, encoder inspection, calibration, and CSV logging remain available

## Manual console

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_manual_console.ps1
```

Normal commissioning sequence:

```text
home center
# Manually hold the pendulum near upright
balance start
```

Useful commands:

```text
status
limits
encoder
servo on
servo off
home measure
home center
home return
balance zero
balance start
balance stop
balance status
balance gains
quit
```

Reference-model gains are locked and cannot be changed at runtime.

## Build and test

```powershell
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --preset vs2022-x64
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build --preset release
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe' --preset release --output-on-failure
```

## Read-only hardware checks

```powershell
& '.\out\build\vs2022-x64\Release\pendulum_self_test.exe' --config '.\config\config.json' --enumerate-only
& '.\out\build\vs2022-x64\Release\pendulum_self_test.exe' --config '.\config\config.json' --input-probe
```

These checks do not enable Servo or command motion.
