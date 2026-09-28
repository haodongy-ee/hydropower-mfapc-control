# Offset-free fixed-model MPC

This branch adds a conventional, model-based MPC baseline without changing the
existing MFAPC or MFAC models.

## Why this is a distinct controller

MFAC and MFAPC update a pseudo-gradient online and do not require a fixed plant
model. The new MPC controller instead uses one fixed affine ARX model identified
before controller evaluation:

```text
y(k+1) = a1 y(k) + a2 y(k-1) + b1 u(k) + b2 u(k-1) + bias
```

The model is fitted from four dedicated MFAPC closed-loop identification
profiles. The first three profiles form the training set; the fourth is held
out for validation. None of these profiles is one of the six final benchmark
conditions.

The initial `0.01 s` model has poles at `0.99255277` and `-0.62948233`. On the
held-out profile it achieved a one-step RMSE of `0.00069734 pu` and an
R-squared value of `0.99998099`. Its free-run RMSE was `0.0128337 pu`, however,
which exposed model mismatch that was hidden by the one-step metric.

The implemented MPC uses the second-order model fitted at `0.05 s`:

```text
a1   =  1.7467169418
a2   = -0.7557718546
b1   =  0.0647657529
b2   = -0.0550777651
bias = -0.0002144255
```

Its poles are `0.95692380` and `0.78979314`; held-out one-step and free-run
RMSE values are `0.00079314 pu` and `0.0122045 pu`, respectively.

## MPC formulation

The controller augments the ARX state with the previous input and a constant
bias state. Simulink runs at `0.01 s`, while the optimizer executes every fifth
sample and holds its output between updates. At each `0.05 s` update it predicts
40 samples (`2.0 s`) and optimizes four future input increments. The objective
penalizes predicted tracking error and input movement; the final move weight is
`40`.

Hard constraints are projected into the optimization iterations:

- actuator range: `-1.15 <= u <= 1.15`;
- input movement: `|delta u| <= 0.015` per `0.05 s` MPC update.

To reject the residual offset caused by fixed-model mismatch, the reference
used by the predictor includes a bounded integral trim. Integration starts only
after the tracking error remains inside a `0.02 pu` gate for ten optimizer
updates, and it is suspended near actuator limits. The integral gain and trim
limit are both `0.03`. A first-order reference filter with coefficient `0.10`
reduces step overshoot. The trim is reset when the commanded target changes.

The small quadratic program is solved by 20 warm-started projected-gradient
iterations inside the MATLAB Function block. This avoids a run-time dependency
on Model Predictive Control Toolbox while retaining receding-horizon prediction,
move blocking, and explicit actuator constraints.

## Reproduction order

Run the scripts in MATLAB R2023a from the repository root:

```matlab
run('scripts/identify_mpc_model.m')
run('scripts/analyze_mpc_model_orders.m')
run('scripts/build_mpc_model.m')
run('scripts/run_mpc_offsetfree_tuning.m')
run('scripts/run_mpc_comparison.m')
```

Outputs:

- `results/mpc_identified_model.mat` and `.csv`;
- `results/mpc_identification_profiles.mat`;
- `results/mpc_multirate_model.mat`;
- `models/HT_ctrl_MPC_baseline.slx`;
- `results/mpc_offsetfree_final_refinement.csv` and `_summary.csv`;
- `results/mpc_offsetfree_selected.mat`;
- `results/three_controller_offsetfree_comparison.csv` and `.mat`.

`HT_ctrl_MPC_baseline.slx` deliberately uses a new name so it cannot overwrite
an earlier user model named `HT_ctrl_MPC.slx`.

## Tuning and six-condition result

Controller selection used three dedicated tuning profiles, separate from the
six benchmark conditions. A coarse search was followed by two refinements; the
retained setting is `[move_weight, du_limit, integral_gain,
reference_filter_alpha] = [40, 0.015, 0.03, 0.10]`.

All six final simulations completed successfully in MATLAB R2023a. The final
means are: IAE `0.545582`, ITAE `2.784530`, 20 s directional overshoot
`3.0753%`, 35 s directional overshoot `1.0516%`, settling time `1.6780 s`,
steady-state error `0.000136`, and steady-state fluctuation `0.004877`.

Relative to the original fixed-model MPC baseline, this reduces IAE by `4.65%`,
ITAE by `22.09%`, 20 s overshoot by `21.55%`, 35 s overshoot by `70.24%`,
settling time by `31.70%`, steady-state error by `95.63%`, and fluctuation by
`6.71%`. Mean 35 s overshoot is just above the aspirational `1%` target, while
the steady-error and fluctuation targets are satisfied.
