# Double-Pendulum LQR Optimization Report

## Implemented architecture

- `dp_config.m`: physical parameters, limits, search bounds, test cases, scoring weights, and budgets.
- `dp_linearize.m`: central-difference Jacobian of `dp_dynamics.m`, reference comparison, and controllability check.
- `dp_simulate_numeric.m`: saturated nonlinear RK4 simulation without GUI or Simulink.
- `dp_evaluate_controller.m`: five-case stability, rail, saturation, effort, and settling metrics.
- `dp_lqr_optimize.m`: deterministic Latin-hypercube-style log sampling, top-region refinement, checkpoints, reports, and best-controller generation.
- `dp_best_lqr.m`: generated code-generation-friendly fixed gain used by `dp_controller.m`.
- `build_model.m` and `run_simulation.m`: in-place model generation, scopes, machine-readable logging, animation, and final checks.

## Existing-project findings

1. The previous LQR file permanently hard-coded A, B, Q, and R.
2. The runtime controller called the offline `lqr` design path from a MATLAB Function block.
3. No multi-case nonlinear test, input saturation metric, rail failure metric, resume checkpoint, or optimizer report existed.
4. The nonlinear input sign disagreed with the requested paper B-matrix convention and was corrected.
5. With the supplied physical parameters, numerical A differs from the supplied reference by up to 8.266. B now has the requested signs and differs by up to 0.3259. The optimizer uses the numerical model, not the reference matrix.

## Formal 300+300 result

- Q diagonal: `[192.801703046, 34.3800101639, 801.989604216, 24.9635933446, 7.34648942262, 4.35527417231]`
- R: `0.447547556089`
- K for `u = -K*X`: `[20.7556260703, 260.364013081, -391.70878795, 25.5554002227, 3.67472398469, -49.3946784937]`
- Success: 4/5 configured stress cases
- Worst max absolute x: 0.300060165 m
- Max absolute u: 30 m/s^2
- Default final-validation initial state: converged numerically; max absolute x 0.084220956 m, max absolute u 24.769464 m/s^2, final angles below 9e-6 rad at 6 s.

The fifth stress case `[0, 0.20, -0.15, 0, 0, 0]` reaches the negative rail by 0.060 mm before recovery. This is reported as a failure rather than being hidden by a tolerance.

## Verification status

The 20+20 smoke optimization and the full 300+300 optimization both completed. Static MATLAB checks and nonlinear final-state checks pass. Simulink runtime verification is pending because the local R2021a Simulink plug-in loader reported insufficient page-file capacity while another MATLAB desktop process occupied about 3.3 GB; attempts through a batch process, a no-JVM process, and the existing desktop window all failed at the environment layer.
