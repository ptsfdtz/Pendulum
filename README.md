# PendulumLab

Phase 1 safety and hardware self-test for a single-stage inverted pendulum using:

- NI PCI-6602 through NI-DAQmx
- Advantech PCI-1723 through DAQNavi/BDaq

The manual console includes upright angle control plus normalized cart position, velocity, and
integral feedback. Cart coordinates come only from a fresh two-limit homing run in the current
process; saved absolute encoder positions are not reused.

The motor encoder calibration tool is read-only: it never creates AO or Servo output tasks. It
captures two stable raw counter values around a manually measured cart movement and atomically
stores `counts_per_mm` plus the raw calibration evidence in `config/config.json`.

## Safety model

The default command is enumeration-only and does not create output tasks. The safe-output test is
disabled by default and requires all of the following:

1. A confirmed `servo_enable_line` in `config/config.json`.
2. `allow_safe_output_test` set to `true`.
3. `output_test_confirmation` set to `I_CONFIRM_ONLY_SAFE_OUTPUTS_MAY_CHANGE`.
4. The explicit `--safe-output-test` command-line flag.

The safe-output test only writes AO0 to 0 V and Servo OFF. It never enables the servo.

## Manual commissioning console

Start the interactive console with:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\start_manual_console.ps1
```

In an interactive Windows terminal the console opens a continuously refreshed dashboard showing:

- Motor encoder `ctr0` signed accumulated X4 position and count rate
- Pendulum encoder `ctr1` signed accumulated X4 position and count rate
- Left/right raw and debounced limit states
- Servo state, AO0 voltage, inferred cart motion direction, and calibration state
- Recent hardware and command events plus an inline command prompt

The dashboard refresh rate is configured by `manual_console.dashboard_refresh_ms`. Redirected or
piped input automatically uses the plain line-oriented display instead of ANSI full-screen mode.

Use `status`, `limits`, `encoder`, `log`, and `help` to inspect the hardware. `servo on` writes
AO0=0 V and enables the Servo immediately; it stays enabled until `servo off` or a stop event.

At startup, the A-axis downward zero is captured only after a stable 1.5-second sample window. Home
the cart in the current console process, move the pendulum near upright, and start:

```text
home center
balance start +
```

Use `balance zero` only when you need to recapture the freely hanging reference during the same
console session.

The A-axis encoder uses a 10 us A/B minimum-pulse-width filter and is configured as 2000 PPR with
X4 decoding: 8000 counts/revolution,
0.045 degrees/count. The automatic startup capture (or `balance zero`) records the freely hanging
position. Each `balance start` captures the current encoder count as that trial's stabilization
reference. The controller applies no fixed encoder target or artificial angle bias. It combines
pendulum angle/rate error relative to the captured reference with cart
position, velocity, and position integral feedback. Cart position is normalized so each physical
limit is approximately one half-travel from the session center. Both physical limits still command
AO0=0 V and Servo OFF, and the balance loop stops at the configured software travel envelope before
reaching them.

Outside the configured `angle_relay_boost_deadband_counts` around the captured reference, the controller adds
a fixed signed angle relay boost. It reverses immediately after crossing the target, while the
ordinary PD term damps the motion and the cart feedback keeps the carriage near session center.
The boost and the combined command remain bounded by the configured rated-torque limits.

## Offline reinforcement learning

`tools/train_balance_policy.py` learns only from recorded `BalanceTelemetry` data. It identifies a
local dynamics model and searches for a bounded linear policy whose deterministic reward favors an
upright pendulum, a centered cart, low angular/cart speed, low actuator effort, and small command
changes. Training never connects to hardware:

```text
python tools/train_balance_policy.py --output experiments/rl_training_latest/policy_candidate.json
python -m unittest tests/test_rl_trainer.py -v
```

Generated candidates contain `"deployment_allowed": false`. They must not be copied into the live
controller until the timing issue is corrected, an independent replay/simulation gate passes, and a
short supervised hardware experiment is explicitly authorized.

## Operator-supervised experiment loop

After building the Release console, start the interactive slow-loop optimizer with:

```text
python tools/run_balance_experiments.py --duration 15 --polarity +
```

The manager launches the deterministic manual console, loads
`experiments/balance_optimizer/known_good.json`, asks for Enter before the one-time `home center`,
then asks for Enter before every balance trial. A trial ends on operator Enter, pendulum fall,
timeout, cart envelope, controller abort, or hardware safety stop. AO/Servo shutdown remains owned
by the C++ safety layer. Every run gets an immutable directory containing its parameters, telemetry,
and score. After each safely terminated trial, the manager issues `home return`: it reuses the
left/right boundaries and center measured once at session startup, moves directly to that known
center with the configured staged speed profile, then stops with AO0=0 and Servo OFF. It does not
probe both limits again.

The slow-loop learner uses bounded CEM episodic policy search. The complete
`Kp/Kd/Kx/Kv/Ki` vector is the policy: each trial samples a joint candidate from a persistent
distribution, stores the resulting reward and termination in replay, and updates the distribution
toward the highest-reward recent episodes. Exploration variance adapts as evidence accumulates; it
does not alternate fixed plus/minus steps. Up to 100 existing trials bootstrap the replay buffer.
All gains remain inside fixed bounds and the deterministic C++ safety limits remain authoritative.
A candidate must improve the score by at least 2% and pass three supervised trials before replacing
`known_good`. Every `balance start` captures the current pendulum encoder count as that trial's
balance reference; no fixed count or artificial angle bias is applied. Use `--dry-run` to inspect
the next joint candidate without launching hardware.


The first-stage controller commands the Yaskawa SGD7S-180A00A002 in analog torque mode. With
`Pn400=30`, 3.0 V represents 100% rated torque. The controller computes a signed fraction of rated
torque and converts it with:
`AO0 = torque_zero_voltage + rated_torque_fraction * 3.0 V`. Use `balance start +` or
`balance start -` to choose the relationship
between increasing A-axis counts and AO direction. If one polarity pushes the cart opposite the
direction of fall, stop and use the other polarity. `balance start` without a sign uses
`balance_control.default_polarity`. Initial gains and the AO clamp are configured under
`balance_control`.

The commissioned starting gains are `Kp=2.0` rated torque/radian and `Kd=0.1` rated
torque/(radian/second). Change them without restarting or recompiling while the console is running:

```text
balance kp 2.5
balance kd 0.12
balance kx 0.02
balance kv 0.01
balance ki 0.002
balance gains
```

Runtime gain changes take effect on the next A-axis sample and reset to `config.json` values when
the console restarts. Updating any gain resets the cart integral state. The cart correction is
independently limited by `maximum_absolute_cart_rated_torque_fraction`; balance can start only
inside `maximum_balance_start_position_fraction` and stops at
`maximum_balance_position_fraction`. The console requests 1 ms Windows timer resolution so the configured 2 ms
encoder monitor period does not degrade to the default approximately 16 ms scheduler interval.

Before tuning, hold AO0 at exactly 0 V and use the SGD7S `Fn009` function to automatically adjust
the analog speed/torque reference offset, or `Fn00B` for manual torque-reference offset adjustment.
After completing the drive-side adjustment, set `analog_torque_zero_calibrated` to `true`; use
`analog_torque_zero_voltage` only for a measured residual software trim. The previous velocity-mode
motor-zero result must not be reused as a torque-reference offset.
The commissioned residual torque-zero trim is `-0.00135 V`; `servo on` and balance control apply
this value while Servo OFF, limit stops, and process exit still force physical AO0 to `0 V`.

The commissioned maximum command is 100% rated torque, which is 3.0 V with Pn400=30. The balance
loop consumes each timestamped A-axis encoder sample exactly once, so its angular-rate estimate is
not calculated repeatedly from a stale count. `maximum_absolute_rated_torque_fraction` changes
actuator authority without recompiling.

The console supports `servo on`, `servo off`, `voltage <volts>` for a held output, and
`voltage <volts> <duration_ms>` for an optional timed test. Voltages may use the configured PCI-1723
range of -10 V to +10 V. Either stable limit, monitoring failure, Ctrl+C, and normal exit command
AO0 to 0 V before Servo OFF. A limit stop does not latch; after the limit clears, Servo can be
enabled again.

Run the motor zero-drift calibration from the same console:

```text
calibrate zero
```

The command follows the reference MATLAB sequence: Servo ON stabilization, coarse scan,
bidirectional DAC-code scan, deadband/hysteresis selection, adjacent-code refinement, and seven
long verification samples. A successful result is atomically saved as
`hardware.pci1723.calibrated_zero_voltage` and remains applied with Servo ON. A limit event aborts
calibration and commands AO0=0 V before Servo OFF. The full run takes roughly 2-3 minutes with the
default parameters in `motor_zero_calibration`.

Return the stage to its mechanical center from the console:

```text
home center
```

To measure the relative encoder span between the two physical limits without moving to the
calculated center, use:

```text
home measure
```

The measurement reports both refined boundary counts plus the forward and reverse travel. It
returns to the first limit for an independent reverse measurement, releases inward, and stops
without moving to center or writing the result to `config.json`.

Every invocation starts from the cart's current position, probes and refines both active-HIGH
limits, calculates the midpoint from this session's encoder counts, and returns to that midpoint.
The result is retained only in process memory for status display; it is never written to or reused
from `config.json`. The settled position is checked after outputs stop. Every path finishes at
AO0=0 V and Servo OFF. After a limit stop, the release phase accepts only motion away from that
active limit.

The initial limit search and the two long full-travel measurements use 0.20 V, while boundary
refinement continues to use the lower 0.02 V `fine_voltage`. After round-trip validation, the
return-to-center profile uses 0.12 V while farther than 15% of travel from center, 0.075 V from
15% to 3%, and 0.03 V for the final approach. Configuration validation caps every automatic
homing motion at 0.20 V.

The first boundary is intentionally not assumed to be LEFT or RIGHT. The controller accepts the
first limit actually reached, learns the AO direction from that event, and then requires the second
event to be the opposite limit before calculating the center. If the console starts while a limit
is already active, `home_center.away_direction_test_ms` controls the low-voltage release-direction
test.

There is no configured maximum encoder span. `home_center.minimum_travel_counts` rejects an
implausibly short measurement, and `home_center.maximum_travel_disagreement_fraction` requires the
forward and reverse spans to agree before centering. The motor encoder A/B inputs also use the
configurable NI-DAQmx minimum-pulse-width filter to reject narrow electrical glitches.

## Build

```powershell
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --preset vs2022-x64
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe' --build --preset release
& 'C:\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\ctest.exe' --preset release
```

## Run the non-output self-test

```powershell
& '.\out\build\vs2022-x64\Release\pendulum_self_test.exe' --config '.\config\config.json' --enumerate-only
```

Read and interpret the two active-HIGH limit inputs without creating output tasks:

```powershell
& '.\out\build\vs2022-x64\Release\pendulum_self_test.exe' --config '.\config\config.json' --input-probe
```

Do not enable the safe-output test until the Servo relay line and polarity have been verified from
the physical wiring and the AO0 voltage can be measured independently.

## Calibrate motor encoder distance

First set the confirmed NI counter and encoder A/B terminals in `config/config.json`. Keep Servo
OFF and move the cart manually during this procedure:

```powershell
& '.\out\build\vs2022-x64\Release\pendulum_encoder_calibrate.exe' --config '.\config\config.json'
```

To verify counter creation and inspect raw counts without entering calibration mode:

```powershell
& '.\out\build\vs2022-x64\Release\pendulum_encoder_calibrate.exe' --config '.\config\config.json' --probe --samples 20 --interval-ms 100
```

The tool uses X4 decoding with 2000 PPR (8000 counts/rev), handles 32-bit rollover, rejects unstable
endpoint samples, and requires the exact word `SAVE` before updating the configuration.
