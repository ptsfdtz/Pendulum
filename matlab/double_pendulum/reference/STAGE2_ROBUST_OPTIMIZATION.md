# Model-only swing-up optimization (2026-09-04)

Hardware logs were deliberately excluded because several runs included manual
uprighting and therefore do not represent autonomous controller behavior.

## Selected parameters

- Stage-1 energy gains: `2.1876`, `4.9156`, `8.2100` (2.0 times the
  paper-adapted gains)
- Stage-1 cart-speed gate: `0.12 m/s`
- Stage-2 far/near gains: `1.40`, `5.00`
- Second-link energy target: `0.005 J`
- Capture-assist second-link rate eligibility: `9 rad/s`

## Search and validation

The deterministic nonlinear search used six symmetric initial states and
three plant profiles: nominal, moderate actuator loss/friction, and severe
actuator loss/friction.  Both isolated stage-2 cases and full downward
stage-1-to-stage-3 runs were evaluated.  The search also checked 1 ms, 2 ms,
and 5 ms controller steps because the discontinuous energy switching law is
sample-time sensitive.

- Full model at 1 ms: 18/18 captured and settled, no 0.35 m hard-limit
  failure, mean capture time 41.200 s, worst travel 0.3081 m.
- Full model at 5 ms: 16/18 captured and settled, no hard-limit failure.
  The two misses were the exactly symmetric ideal downward states; perturbed
  downward starts completed.
- Isolated stage 2 at 5 ms: 16/18 captured and settled, no hard-limit
  failure, mean capture time 26.032 s, worst travel 0.3042 m.
- Generated Simulink model at 1 ms: stages 1-2-3, capture at 45.965 s,
  settled by 50 s, maximum travel 0.1279 m, maximum command 10.11 m/s^2.

The `0.30 m` boundary is modeled as a recoverable inward-drive boundary.
Only `0.35 m` is terminal in the numerical simulation. A tested
smooth replacement for the paper's `sign()` switch did not improve the
cross-sample-period results and was not selected.

These are simulation-selected parameters, not hardware-validated parameters.
Physical deployment is maintained separately in the PendulumLab C++ project.
