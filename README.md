# PendulumLab

Windows C++ controller for a double linear inverted pendulum using:

- NI PCI-6602 for the cart encoder, both pendulum encoders, travel limits, and Servo enable
- Advantech PCI-1723 AO0 for the Yaskawa SGD7S velocity command

## Double-pendulum balance controller

The live balance path is ported from the archived
`reference/LQR_lp2/LQR_lp2.slx` model version 4.43. The copy has the same
SHA-256 hash as `E:\直线倒立摆库\demo\LQR_lp2.slx`:

```text
A1ED10DD4999DF36895BB2534E779F0ED9CA82DDD7776522F038F4F1AC2821F4
```

The model's exact numerical path is preserved:

```text
theta1 = wrap(-first_pendulum_delta * 2*pi/10000)
theta2 = wrap(-second_pendulum_delta * 2*pi/4000 + theta1)
x      = -cart_delta * 0.163/8000

theta1_dot = (theta1 - theta1_previous) / 0.005
theta2_dot = (theta2 - theta2_previous) / 0.005
x_dot      = (x - x_previous) / 0.005

acc = clamp(
     150.31*theta2 + 23.63*theta2_dot
    - 93.74*theta1 -  3.25*theta1_dot
    - 10*x          - 11.64*x_dot,
    -30, +30)
```

The copied `ACC2VOL` block uses:

- control/update period: 0.005 s (200 Hz)
- integrator multiplier `Ts`: 0.005 s
- velocity reference limit: ±0.6 m/s
- velocity PI: P=0.18, I=27
- AO0 limit: ±2 V
- AO0 initial/final value: 0 V
- velocity-reference integrator reset when either absolute link angle reaches 10°

`balance start` is intentionally the only balance-start mode. It captures the
current CTR1, CTR2, and cart counts as the upright/cart references, then starts
the exact LQR+ACC2VOL path. There is no automatic homing, downward-zero step, or
swing-up in this command.

## Wiring

| Function | PCI-6602 resource | Pin |
| --- | --- | ---: |
| Cart encoder A | CTR0 SOURCE | 2 |
| Cart encoder B | CTR0 AUX | 40 |
| First-pendulum encoder A | CTR1 SOURCE | 7 |
| First-pendulum encoder B | CTR1 AUX | 6 |
| Second-pendulum encoder A (black) | CTR2 SOURCE | 34 |
| Second-pendulum encoder B (white) | CTR2 AUX | 66 |
| Second-pendulum encoder Z (orange, optional) | CTR2 GATE | 67 |
| Second-pendulum encoder 0 V (blue) | D GND | 33 or 68 |
| Second-pendulum encoder +5 V (brown) | +5 V | 1 |
| Left limit | `port0/line0` | 10 |
| Right limit | `port0/line2` | 45 |
| Servo ON | `port0/line3` | 12 |
| Servo ON ground | D GND | 11 |

The second-pendulum encoder is configured for X4 decoding: 1000 PPR and 4000
effective counts/rev. CTR2's default A/B routing is pins 34/66. The optional Z
wire is documented but not used by the controller. All three encoder digital
minimum-pulse-width filters are disabled (`0 us`), matching the source model.

## Run

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_manual_console.ps1
```

Then:

```text
1. Place the cart at a safe position and manually hold both links upright.
2. Enter: balance start
3. To stop, enter: balance stop
```

The physical left/right limits, AO0=0 V before Servo OFF shutdown ordering,
fault handling, Ctrl+C handling, and CSV telemetry remain active.

Useful commands:

```text
status
limits
encoder
balance start
balance stop
balance status
balance gains
quit
```

## Build and test

```powershell
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --preset vs2022-x64
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build --preset release
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe' --preset release --output-on-failure
```

Read-only device enumeration:

```powershell
& '.\out\build\vs2022-x64\Release\pendulum_self_test.exe' --config '.\config\config.json' --enumerate-only
& '.\out\build\vs2022-x64\Release\pendulum_self_test.exe' --config '.\config\config.json' --encoder-probe
```

No unattended hardware-motion test is performed.
