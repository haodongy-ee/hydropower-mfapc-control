# Phase 2.4–2.5: randomized robustness study and presentation

## Study design

The upgraded MPC was tested with a reproducible 100-case joint-uncertainty
sample. Each continuous input is independently stratified into 100 equal
probability intervals and shuffled with MATLAB's `mt19937ar` stream (seed
`2401`). The same generated case table can be replayed with the phase-2 MPC
for a paired comparison. The plant and controller are simulated for 50 s in
MATLAB R2023a; no R2026a model conversion is used.

| Input | Sampling range / rule |
|---|---|
| Actuator gain scale | Uniform 0.80–1.20 |
| Actuator time constant | Uniform 0.056–0.084 s |
| Measurement-noise standard deviation | Uniform 0–0.005 pu; fixed per-case seed |
| Load event at 30 s | Uniform 1.5–6.0 MW; half additions, half rejections |
| Actuator rate limit | Uniform 0.05–0.10 pu per model interval |
| Measurement delay | Uniform 0–0.05 s |
| Reference profile | Balanced low, nominal, and high profiles |

The ranges extend the existing 23-case deterministic screen while combining
uncertainties rather than changing only one factor per run. The assumed
independent uniform distributions are a *test design*, not measured field
failure probabilities. Correlations, extreme tails, turbine parameter
uncertainty beyond this harness, and hardware effects remain untested.

## Reproduce and resume

From the repository root in MATLAB R2023a:

```matlab
addpath('scripts')
run_mpc_robustness_tests("montecarlo", "mpc23", 100, 2401)
% Optional paired comparison using exactly the same cases:
run_mpc_robustness_tests("montecarlo", "phase2", 100, 2401)
build_three_controller_scorecard
plot_phase24_results(100, 2401)
```

The runner checkpoints CSV and MAT results after every case. Repeating the
same command skips completed cases. A different sample count or seed gets
different filenames; the saved design is checked before resuming. A failed
simulation retains its failure status and error text rather than being
silently omitted. To retry a failed case, first make a reviewed copy of the
results; do not edit the original outcome in place.

The reproducibility key is `results/mpc_montecarlo_design_n100_seed2401.csv`.
For the upgraded controller, outputs use the prefix
`results/mpc_23_montecarlo_n100_seed2401_`; phase 2 uses
`results/mpc_robustness_montecarlo_n100_seed2401_`. Each variant writes
`results.csv`, `results.mat`, `summary.csv`, and one `trace.mat` for the first
successful case. The trace includes time, physical output, control, and
reference, and drives the load-disturbance response figure. The large trace
MAT is a local generated artifact, not part of the published result bundle;
rerun the command above to regenerate it. The per-case CSV/MAT, summary CSV,
and rendered figures are published.

## Metrics and statistics

IAE and ITAE integrate absolute tracking error from 20 to 50 s; ITAE uses
time measured from 20 s. Maximum overshoot is the larger directional
overshoot at the 20 and 35 s changes. Final steady error and fluctuation use
45–50 s. Settling time is measured from the 35 s change against a 2% final
target band. Control total variation is the sum of absolute consecutive
control changes. Input and move violations are counts from the sampled
trajectory using the same limits as the deterministic screen.

For every continuous metric, the summary reports the mean, median, and
linearly interpolated 95th percentile **among successful simulations with a
finite metric**; its `N` column shows that denominator. Failed simulations
contribute to `SimulationFailureRate`, not to the successful-run metric
distribution. Successful runs that never settle by 50 s are counted in
`UnsettledRate`; their settling time remains missing rather than being
misreported as zero. `AcceptancePassRate` uses all 100 attempted cases as its
denominator. Its criteria include zero hard input and move violations,
steady error below 0.001 pu, final-step overshoot below 5%, and final
steady fluctuation below 0.008 pu.

The gain/time-constant heatmap bins successful cases and labels each bin with
both its mean IAE and sample count. Empty bins stay marked as empty. Because
other sampled inputs also vary, these are descriptive conditional averages,
not isolated causal effects of gain and time constant. The boxplots use the
same successful-run populations as the summary. When phase-2 results exist,
they appear beside phase 2.3 for matched case IDs.

## R2023a full-run result

The `n100_seed2401` run finished all 100 cases: 100 successful simulations,
98 acceptance passes, zero hard input or move violations, and zero cases
unsettled at 50 s. The two acceptance misses, `mc_047` and `mc_098`, exceeded
only the 0.001 pu steady-error threshold, at 0.001073 and 0.001213 pu.

| Metric | Mean | Median | P95 |
|---|---:|---:|---:|
| IAE | 0.587560 | 0.636634 | 0.898506 |
| ITAE | 3.332964 | 3.076332 | 5.686627 |
| Maximum directional overshoot (%) | 2.766186 | 2.765927 | 3.500023 |
| Steady error (pu) | 0.000287 | 0.000237 | 0.000736 |
| Settling time after 35 s (s) | 1.883692 | 1.815289 | 2.775161 |
| Control total variation | 1.110222 | 1.211314 | 1.290973 |

The committed [per-case CSV](../results/mpc_23_montecarlo_n100_seed2401_results.csv),
[MAT result](../results/mpc_23_montecarlo_n100_seed2401_results.mat), and
[summary CSV](../results/mpc_23_montecarlo_n100_seed2401_summary.csv) retain
full precision. The [heatmap](../figures/montecarlo_parameter_heatmap.png),
[boxplots](../figures/montecarlo_boxplots.png), and
[representative disturbance response](../figures/montecarlo_disturbance_response.png)
are presentation views of this run. The disturbance response is one case,
not a percentile envelope. Heatmap cells contain only a few cases each;
differences between cells are not a causal estimate.

## Three-controller scorecard

`results/three_controller_scorecard.csv` is a separate presentation of the
existing six common reference-profile experiments. It uses mean IAE, ITAE,
per-profile maximum directional overshoot, 35 s settling time, and final
steady error. For each lower-is-better metric, the best of the three means
scores 100 and the worst scores 0; the five metric scores have equal weight.
The composite is relative to these three controllers and six profiles. It is
**not** a Monte Carlo score, constraint-safety rating, or statistical
significance test. MFAC/MFAPC do not have matched 100-case robustness runs in
this study, so no such comparison is claimed.

## Interpretation limits

One hundred stratified cases improve coverage beyond six demonstrations but
do not by themselves prove statistical significance or closed-loop stability.
Percentiles can be unstable in the tail at this sample size, and a zero
observed failure rate does not imply zero real-world failure probability.
The report should include the sample count, seed, unsuccessful-run count,
uncertainty ranges, and successful-run denominator beside any headline result.
