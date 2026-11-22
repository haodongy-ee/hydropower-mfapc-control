# Phase 2.4–2.5: randomized robustness study and presentation

## Study design

The upgraded MPC was tested with a reproducible 100-case joint-uncertainty
sample. Each continuous input is independently stratified into 100 equal
probability intervals and shuffled with MATLAB's `mt19937ar` stream (seed
`2401`). The same generated case table was replayed with the phase-2 MPC
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
run_mpc_robustness_tests("montecarlo", "phase2", 100, 2401)
analyze_paired_mpc_montecarlo(100, 2401)
build_three_controller_scorecard
plot_phase24_results(100, 2401)
```

The runner checkpoints CSV and MAT results after every case. Repeating the
same command skips completed cases. A different sample count or seed gets
different filenames; the saved design is checked before resuming. A failed
simulation retains its failure status and error text rather than being
silently omitted. To retry a failed case, first make a reviewed copy of the
results; do not edit the original outcome in place.

The case-design CSV is a readable index, but MATLAB's CSV formatting loses
some floating-point digits. For **exact trajectory replay**, use the `cases`
table in either variant's `results.mat` checkpoint. Reading rounded inputs
back from the CSV produced visibly different trajectories near acceptance
thresholds; replaying the MAT values reproduced the recorded metrics exactly.
The design CSV remains useful for inspection and matching case IDs.
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
same successful-run populations as the summary. Phase-2 results appear
beside phase 2.3 for matched case IDs.

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

## Matched phase-2 versus phase-2.3 comparison

The 100 phase-2 simulations also completed successfully on the identical
case IDs, input values, noise seeds, and reference profiles. The paired
analysis checks every design field before comparing outcomes. Both versions
had zero input/move violations and zero unsettled cases. Acceptance passes
were 68 for phase 2 and 98 for phase 2.3: 68 cases passed both, 30 passed
only phase 2.3, none passed only phase 2, and two passed neither.

| Lower-is-better metric | Phase-2 mean | Phase-2.3 mean | Mean paired change (new − old) | Relative mean change | Cases improved |
|---|---:|---:|---:|---:|---:|
| IAE | 0.580714 | 0.587560 | +0.006845 | +1.18% | 22/100 |
| ITAE | 3.319172 | 3.332964 | +0.013792 | +0.42% | 42/100 |
| Maximum overshoot (%) | 2.900232 | 2.766186 | −0.134046 | −4.62% | 79/100 |
| Steady error (pu) | 0.000262 | 0.000287 | +0.000026 | +9.82% | 49/100 |
| Steady fluctuation (pu) | 0.006979 | 0.004807 | −0.002172 | −31.12% | 93/100 |
| Settling time after 35 s (s) | 1.807672 | 1.883692 | +0.076020 | +4.21% | 18/100 |
| Control total variation | 2.194758 | 1.110222 | −1.084536 | −49.41% | 100/100 |

Negative paired change denotes improvement for these metrics. The old MPC
missed the 0.008 pu steady-fluctuation criterion in 32 cases; the upgrade
missed it in none. One old case also exceeded the steady-error criterion.
The upgraded version missed only steady error in `mc_047` and `mc_098`
(0.001073 and 0.001213 pu). Those same two cases failed the old controller's
fluctuation criterion, so they did not represent lost acceptance passes.

The [phase-2 raw CSV](../results/mpc_robustness_montecarlo_n100_seed2401_results.csv)
and [MAT result](../results/mpc_robustness_montecarlo_n100_seed2401_results.mat)
remain separate from the [paired case CSV](../results/mpc_paired_n100_seed2401_cases.csv),
[paired summary CSV](../results/mpc_paired_n100_seed2401_summary.csv), and
[paired MAT result](../results/mpc_paired_n100_seed2401.mat). The
[paired scatter](../figures/montecarlo_paired_scatter.png),
[two-version boxplots](../figures/montecarlo_boxplots.png), and
[example disturbance response](../figures/montecarlo_disturbance_response.png)
show complementary views. The mean tracking metrics and settling time are
slightly worse after the upgrade; the major gain is smoother output and
control with more accepted cases. One fixed seed and synthetic uncertainty
model do not establish a field failure probability or a significance claim.

## Independent-seed check protocol

Before inspecting any new outcomes, the next check is fixed at 50 new
stratified cases with seed `2402`. Both controller versions use the same
case design; neither controller is retuned. The primary descriptive
comparisons are acceptance passes, simulation failures, mean IAE, steady
fluctuation, and control total variation. Overshoot, steady error, and
settling time remain reported as tradeoffs. The original `n100_seed2401`
results stay unchanged. This 50-case check tests whether the direction of
the observed tradeoff repeats under another seed; it is not a field failure
probability or a formal significance test.

## Independent-seed result: 50 new matched cases

Both R2023a runs with `n=50`, seed `2402` completed with zero simulation
failures and zero recorded hard input or move-constraint violations. The old
MPC passed 32/50 acceptance checks and phase 2.3 passed 50/50: 32 passed both,
18 passed only the upgrade, and none passed only the old controller. The
results show the same directional tradeoff as the 100-case comparison.

| Lower-is-better mean metric | Phase 2 | Phase 2.3 | Relative change |
|---|---:|---:|---:|
| IAE | 0.577524 | 0.584635 | +1.23% |
| ITAE | 3.331231 | 3.345494 | +0.43% |
| Maximum directional overshoot | 2.887% | 2.728% | −5.52% |
| Steady fluctuation (pu) | 0.007209 | 0.004794 | −33.49% |
| Settling time after 35 s (s) | 1.859 | 1.895 | +1.93% |
| Control total variation | 2.178 | 1.110 | −49.05% |

The new sample supports the same descriptive tradeoff: less fluctuation,
overshoot, and control movement, with slightly worse mean tracking error and
settling time. It does not establish field reliability or a formal confidence
claim. The [design CSV](../results/mpc_montecarlo_design_n50_seed2402.csv),
[phase-2 results](../results/mpc_robustness_montecarlo_n50_seed2402_results.csv),
[phase-2.3 results](../results/mpc_23_montecarlo_n50_seed2402_results.csv),
[paired cases](../results/mpc_paired_n50_seed2402_cases.csv),
[paired summary](../results/mpc_paired_n50_seed2402_summary.csv), and
[paired scatter](../figures/montecarlo_paired_scatter_n50_seed2402.png) retain
the underlying values. Both variant MAT checkpoints contain the exact case
inputs for replay.

## Original acceptance misses: exact replay

`mc_047` and `mc_098` were the only phase-2.3 acceptance misses in the
original 100-case sample. Using the saved MAT inputs,
`diagnose_mpc_steady_error_cases(100,2401)` replayed both controllers on
both cases and reproduced every recorded steady-error and fluctuation metric
exactly. In `mc_047`, the upgraded controller's final-window mean was
0.001073 pu above the 0.9 pu target; in `mc_098`, it was 0.001213 pu below.
Both exceed the 0.001 pu steady-error criterion. The old controller's
fluctuation was 0.008098 and 0.008587 pu, respectively, above the 0.008 pu
criterion; the upgraded values were 0.005630 and 0.006626 pu. These replay
traces identify the acceptance tradeoff, without isolating a causal mechanism
or justifying controller retuning from two selected cases. See the
[diagnostic CSV](../results/mpc_failure_diagnostics_n100_seed2401.csv) and
[final-window plot](../figures/mpc_23_failure_case_final_windows.png).

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
