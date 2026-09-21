# Fixed-model MPC baseline

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
`20`.

Hard constraints are projected into the optimization iterations:

- actuator range: `-1.15 <= u <= 1.15`;
- input movement: `|delta u| <= 0.020` per `0.05 s` MPC update.

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
run('scripts/run_mpc_comparison.m')
```

Outputs:

- `results/mpc_identified_model.mat` and `.csv`;
- `results/mpc_identification_profiles.mat`;
- `results/mpc_multirate_model.mat`;
- `models/HT_ctrl_MPC_baseline.slx`;
- `results/three_controller_comparison.csv` and `.mat`.

`HT_ctrl_MPC_baseline.slx` deliberately uses a new name so it cannot overwrite
an earlier user model named `HT_ctrl_MPC.slx`.

## Six-condition result

All six MPC simulations completed successfully in MATLAB R2023a. MPC achieved
the lowest IAE in all six scenarios. Its mean IAE was `0.572218`, compared with
`0.643752` for MFAPC and `0.848029` for MFAC. Mean ITAE was `3.573921`, compared
with `3.608410` and `4.845817`. The robust move penalty reduced nominal
steady-state fluctuation from `0.08539 pu` in the aggressive prototype to
`0.00687 pu` in the retained controller.

The main limitation is offset/overshoot: mean steady-state error is `0.003122`
and mean 35 s directional overshoot is `3.5333%`, both worse than MFAPC. An
offset-free disturbance model or integral target calculation is therefore a
future refinement, but was not enabled in this baseline because the initial
high-gain disturbance estimator created a low-frequency limit cycle.
