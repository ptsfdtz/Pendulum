# Phase 1 Commissioning Gate

The enumeration-only self-test is approved for unattended use because it does not create output
tasks. Do not authorize the safe-output test until every item below is confirmed from the wiring
diagram and, where applicable, measured independently.

## Required confirmations

- [x] NI PCI-6602 Servo ON physical line: `Dev1/port0/line3` (`P0.3`)
- [x] Servo relay active polarity: HIGH = ON, LOW = OFF
- [x] Relay de-energized state is Servo OFF
- [ ] Advantech PCI-1723 AO0 is physically connected to the servo command input
- [x] PCI-1723 configured AO range: -10 V to +10 V
- [x] A meter is connected to independently verify AO0 is 0 V
- [ ] Physical emergency stop removes actuator power independently of this program
- [ ] Mechanical travel is clear and both limit sensors are installed

## Safe-output test result

- Software result: passed on 2026-08-31
- Commanded AO0: 0.0 V
- Commanded Servo relay state: OFF (`P0.3` LOW)
- DAQ API errors: none
- [ ] Operator confirmed measured AO0 voltage
- [ ] Operator confirmed physical Servo relay remained OFF

The configuration authorization was disabled again immediately after the test.

## Limit and zero-offset facts

- [x] Left limit `Dev1/port0/line0` is active HIGH
- [x] Right limit `Dev1/port0/line2` is active HIGH
- [x] Servo ON at AO0 = 0 V causes cart drift
- [x] Positive AO0 voltage moves the cart RIGHT (confirmed from limit-event logs)
- [x] Negative AO0 voltage moves the cart LEFT (confirmed from limit-event logs)
- [x] Servo drive: Yaskawa `SGD7S-180A00A002`, switched to analog velocity mode for the reference Simulink controller
- [x] `Pn400 = 30`: 3.0 V torque reference equals 100% rated torque
- [ ] Analog torque-reference offset adjusted with drive function `Fn009` or `Fn00B`
- [ ] Stable motor zero voltage (`calibrated_zero_voltage`) measured and saved

`safe_voltage` remains 0 V for emergency shutdown together with Servo OFF. It must not be treated
as the stationary command while Servo is ON.

## Motor encoder distance calibration

- [x] Encoder resolution: 2000 PPR
- [x] Quadrature decoding: X4
- [x] Effective resolution: 8000 counts/rev
- [x] Transmission type: 3M timing belt
- [ ] Pulley tooth count (not required after direct distance calibration)
- [x] NI counter physical channel: `Dev1/ctr0`
- [x] Encoder A input: CTR0 SOURCE, connector pin 2 (DAQmx default routing)
- [x] Encoder B input: CTR0 AUX, connector pin 40 (DAQmx default routing)
- [x] A/B digital minimum-pulse-width filter: disabled (0 us), matching `LQR_lp2.slx`
- [x] Previously measured powered full travel: 33112 counts on 2026-09-01
- [ ] Manual distance calibration completed and `counts_per_mm` saved

The calibration tool intentionally refuses to start until the counter and both encoder terminals
are confirmed. It does not infer pulley geometry.

## Double-pendulum encoder inputs

- [x] First pendulum: `Dev1/ctr1`, CTR1 SOURCE pin 7, CTR1 AUX pin 6
- [x] First pendulum: X4, 2500 PPR, 10000 counts/rev
- [x] Second pendulum: `Dev1/ctr2`, CTR2 SOURCE pin 34, CTR2 AUX pin 66
- [x] Second pendulum: X4, 1000 PPR, 4000 counts/rev
- [x] Second-pendulum Z phase: optional CTR2 GATE pin 67, not used by control
- [x] Second-pendulum supply: +5 V pin 1; D GND pin 33 or 68
- [x] Both pendulum A/B digital minimum-pulse-width filters: disabled (0 us), matching `LQR_lp2.slx`
- [x] Read-only encoder probe created and read CTR0, CTR1, and CTR2 input tasks on 2026-09-03

## Output-test authorization

After the items above are confirmed, update only these fields in `config/config.json`:

```json
"servo_enable_line": "CONFIRMED_PHYSICAL_LINE",
"servo_active_high": true,
"allow_safe_output_test": true,
"output_test_confirmation": "I_CONFIRM_ONLY_SAFE_OUTPUTS_MAY_CHANGE"
```

Then run `pendulum_self_test --safe-output-test` while observing AO0 with a meter and confirming
that the Servo relay remains OFF. This test never commands Servo ON.
