# Phase 1 frozen baseline

This directory is the immutable reference snapshot for the completed
offset-free MPC phase. Later robustness work must use a separate model and
must not overwrite these files.

- MATLAB/Simulink release: R2023a
- Local source commit: `e799752`
- Controller: fixed ARX MPC with conditional integral target trim
- Final mean IAE: `0.545581950032`
- Final mean 35 s directional overshoot: `1.05156145039%`
- Final mean steady-state error: `0.000136432410142 pu`

## Frozen files and SHA-256

| File | SHA-256 |
|---|---|
| `HT_ctrl_MPC_phase1.slx` | `4408FA4A404E3CEB655F0C1CDB6BB4A45322412C0D03DD9CFEAFF1159981ECAB` |
| `mpc_controller_parameters_phase1.mat` | `F9050D4F3B8087CE63714596D7E9B3BF916E495BDF3F57CC9B400E9E7C43D81D` |
| `phase1_average_summary.csv` | `325C3E4D91616B9B25AAC2350DC59098EEB1E0A9BB7321D888321C643DBB4927` |
| `phase1_six_scenario_results.csv` | `183D0F01D2F64540401F610E3845F8545F63BB2A7C577932132D33962332A638` |

The working robustness model is generated from `HT_ctrl_MPC_phase1.slx` by
`scripts/build_mpc_robustness_model.m`.
