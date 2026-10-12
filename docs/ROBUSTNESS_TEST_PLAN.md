# Phase 2 robustness test platform

## Scope

Phase 2 starts from the immutable phase-1 snapshot in
`baselines/phase1_offsetfree_mpc`. The robustness harness is generated as
`models/HT_ctrl_MPC_robustness.slx`; the frozen model is never edited.

The first screening matrix contains 23 deterministic cases:

- actuator/plant gain at -20%, -10%, +10%, and +20%;
- actuator time constant at -20%, -10%, +10%, and +20%;
- measurement-noise standard deviation of 0.001, 0.003, and 0.005 pu;
- positive load steps and load-rejection events at 30 s;
- tighter actuator-rate limits;
- measurement delays of 0.01, 0.02, and 0.05 s;
- low-, nominal-, and high-output reference profiles.

Noise is injected only into the MPC measurement path. The recorded plant
output therefore remains the physical output rather than the noisy feedback
signal. Load disturbances use the existing 13.8 kV load and breaker.

## Run order (MATLAB R2023a)

```matlab
addpath('scripts')
build_mpc_robustness_model
run_mpc_robustness_tests("smoke")
run_mpc_robustness_tests("screening")
```

The smoke mode runs the baseline and 0.003 pu noise cases. Screening mode runs
the complete matrix. The case matrix is always written to
`results/mpc_robustness_test_matrix.csv`.

## Recorded metrics

Each run records IAE, ITAE, directional overshoot, settling time, steady-state
error, steady-state fluctuation, total control variation, maximum absolute
control, input/move constraint violations, simulation wall time, run status,
and an acceptance flag.

A case passes when it has no input-limit violation, final steady-state error
below 0.001 pu, final directional overshoot below 5%, and steady-state
fluctuation below 0.008 pu. Category summaries include success and pass rates
plus 95th-percentile IAE, overshoot, and steady-state error.

This stage is a deterministic screening platform. Randomized Monte Carlo model
sampling and three-controller statistical comparison belong to the next stage.

## Smoke-test result

The generated harness was validated in MATLAB R2023a with the baseline and
0.003 pu measurement-noise cases. Both simulations completed successfully and
reported zero input-constraint and move-constraint violations.

| Case | IAE | Overshoot at 35 s | Steady error | Steady fluctuation | Pass |
|---|---:|---:|---:|---:|---|
| Baseline | 0.592841 | 1.6975% | 0.000021 | 0.004972 | Yes |
| Noise 0.003 pu | 0.608110 | 1.7312% | 0.000257 | 0.008711 | No |

The noisy case misses only the current `0.008 pu` fluctuation threshold. Its
IAE rises by 2.58%, while control total variation rises by 93.26%. This is an
actionable phase-2 finding: measurement filtering should be assessed before
the full Monte Carlo study.

## Full screening result

The complete 23-case deterministic screen was run in MATLAB R2023a. Every
simulation completed successfully, 21 cases passed all thresholds, and no case
reported an input-constraint or move-constraint violation.

| Category | Cases | Successful | Passed | Pass rate |
|---|---:|---:|---:|---:|
| Baseline | 1 | 1 | 1 | 100% |
| Plant gain | 4 | 4 | 4 | 100% |
| Time constant | 4 | 4 | 4 | 100% |
| Measurement noise | 3 | 3 | 1 | 33.3% |
| Load disturbance | 4 | 4 | 4 | 100% |
| Actuator limit | 2 | 2 | 2 | 100% |
| Measurement delay | 3 | 3 | 3 | 100% |
| Operating point | 2 | 2 | 2 | 100% |

The two failed cases were `noise_003` and `noise_005`; each missed only the
steady-fluctuation criterion. At `0.005 pu` noise, IAE rose by 5.57% relative
to baseline, but steady fluctuation rose to `0.011632 pu` and control total
variation rose by 178.13%. The most demanding passing case was the `0.050 pu`
actuator-rate limit, with IAE `0.975771` and settling time `2.9565 s`.

The results support proceeding to a noise-mitigation study before randomized
Monte Carlo evaluation. Candidate changes are a measurement low-pass filter,
an estimator/noise model, and higher move suppression, evaluated against the
same tracking and constraint criteria.
