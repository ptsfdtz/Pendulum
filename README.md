# PendulumLab

Windows C++ controller for a single-stage linear inverted pendulum using:

- NI PCI-6602 for the motor encoder, pendulum encoder, limits, and Servo enable
- Advantech PCI-1723 AO0 for the Yaskawa SGD7S velocity command

## Double-pendulum LQR experiment

The MATLAB-identified double-pendulum LQR is available through one command:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_double_balance.ps1
```

Alternatively, double-click `start_double_balance.cmd` in the project root.

The executable automatically measures both physical limits, returns the cart
to the measured center, switches AO0 to 0 V and Servo OFF, and waits. Hold both
links vertically upright and press Enter to start the 200 Hz LQR experiment.
The default duration is unlimited. Press `Q` or `Esc` to stop normally, disable the
servo, and finish writing the logs. `Ctrl+C` remains an emergency stop. Pass a positive
`-Duration` value to the PowerShell launcher for a timed test. Every run creates an immutable directory
under `experiments/double_balance` containing `config_snapshot.json`,
`home.json`, `telemetry.csv`, and `metrics.json`.

During balance, each physical limit must be active for the configured
`manual_console.limit_debounce_samples` consecutive 200 Hz samples before it stops the
experiment. The default is 3 samples; a low sample resets that limit's counter. The
termination reason records whether the stable input was the left or right limit.

The double-pendulum controller uses the measured cart conversion
`0.734 m / 33259 counts`, first encoder `Dev1/ctr1` at 8000 counts/rev, and
relative second encoder `Dev1/ctr2` at 4000 counts/rev. Both physical pendulum
encoders increase toward +x, while the MATLAB model angle is positive toward
-x, so both angle conversions are negative. The absolute second-link angle is
`theta2 = theta1 + relative_theta2`.

## Balance controller

The live balance path is the complete controller from
`E:\直线倒立摆库\demo\Copy_of_LQR_lp1_1.slx` (model version 4.37). The previous
PD/LQR controllers and offline gain-learning tools have been removed.

The implementation preserves the reference model's signs, constants,
unit-delay initial conditions, saturation behavior, and update ordering:

```text
theta = wrap(-(pendulum_count - upright_count) * 2*pi/8000)
x     = -(motor_count - center_count) * 0.163/8000

theta_dot = (theta - theta_previous) / 0.01
x_dot     = (x - x_previous) / 0.01

lqr_acc = clamp(
    -58.6*theta
    -10.69*theta_dot
    +10*x
    +12.23*x_dot,
    -10, +10)
```

The reference `ACC2VOL` block is also reproduced:

- control update: 0.01 s
- integrator multiplier `Ts`: 0.005 s, exactly as stored in the model
- velocity reference limit: +/-0.6 m/s
- velocity PI: P=0.18, I=54
- AO0 limit: +/-1 V
- AO0 zero command: 0 V

The first `balance auto` after program startup runs a fresh two-limit center
operation and remembers that center for the current process. Later
`balance auto` commands return directly to the remembered center without
probing both limits again. Each automatic run confirms the pendulum downward
zero before enabling the exact reference `Swing_up` branch. The model selects `Swing_up` while
`abs(theta) >= pi/6` and LQR inside that region; it returns to `Swing_up`
automatically if the pendulum leaves the LQR region. Copied swing-up constants:
`m=0.134`, `g=9.8`, `l=0.223`, `J=0.0089`, `Gain1=5`, `Gain2=6`, and
`PositionLimit=0.25`.

`balance start` remains the manual-upright mode. Hold the pendulum within
+/-30 degrees of upright before starting it. This mode stops when the pendulum
leaves the LQR region.

## Safety retained

- active-HIGH left and right physical limits with debounce
- AO0=0 V before Servo OFF on a limit, monitor fault, exception, Ctrl+C, or exit
- fresh two-limit `home center` on the first automatic run; later automatic
  runs return to the remembered in-process center
- balance start restricted to the configured center window
- automatic swing-up software travel limit at 85% of calibrated half-travel;
  outward commands are blocked without ending the run, and inward commands
  remain available so swing-up can recover and continue
- manual-upright balance still stops if it reaches the software travel limit
- manual Servo, homing, encoder inspection, calibration, and CSV logging remain

## Manual console

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_manual_console.ps1
```

Automatic sequence:

```text
balance auto
```

The first command performs `home center`; subsequent commands reuse the
remembered center. Each run confirms the downward zero, starts reference
swing-up, and switches to LQR automatically.

Manual-upright sequence:

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
balance auto
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
