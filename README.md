# PendulumLab

Phase 1 safety and hardware self-test for a single-stage inverted pendulum using:

- NI PCI-6602 through NI-DAQmx
- Advantech PCI-1723 through DAQNavi/BDaq

No closed-loop controller or homing routine is implemented in Phase 1.

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

Use `status`, `limits`, `encoder`, `log`, and `help` to inspect the hardware. `servo on` writes
AO0=0 V and enables the Servo immediately; it stays enabled until `servo off` or a stop event.

The console supports `servo on`, `servo off`, `voltage <volts>` for a held output, and
`voltage <volts> <duration_ms>` for an optional timed test. Voltages may use the configured PCI-1723
range of -10 V to +10 V. Either stable limit, monitoring failure, Ctrl+C, and normal exit command
AO0 to 0 V before Servo OFF. A limit stop does not latch; after the limit clears, Servo can be
enabled again.

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
