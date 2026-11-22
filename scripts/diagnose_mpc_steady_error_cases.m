function diagnostic = diagnose_mpc_steady_error_cases(n, seed, case_ids)
%DIAGNOSE_MPC_STEADY_ERROR_CASES Replay selected cases without retuning.
%   Replays both MPC variants using the full-precision MAT case design.
if nargin < 1, n = 100; end
if nargin < 2, seed = 2401; end
if nargin < 3, case_ids = ["mc_047"; "mc_098"]; end
if ~strcmp(version('-release'), '2023a')
    error('This project requires MATLAB R2023a for these simulations.');
end
case_ids = string(case_ids);
case_ids = case_ids(:);
if numel(unique(case_ids)) ~= numel(case_ids)
    error('Diagnostic case IDs must be unique.');
end
root = fileparts(fileparts(mfilename('fullpath')));
result_dir = fullfile(root, 'results');
figure_dir = fullfile(root, 'figures');
design_file = fullfile(result_dir, sprintf( ...
    'mpc_robustness_montecarlo_n%d_seed%d_results.mat', n, seed));
design_source = load(design_file, 'cases');
design = design_source.cases;
if ~all(ismember(case_ids, design.CaseID))
    error('A requested case ID is absent from the saved design.');
end

diagnostic = table();
for variant = ["phase2", "mpc23"]
    records = replay_variant(root, result_dir, design, ...
        case_ids, variant, n, seed);
    if isempty(diagnostic)
        diagnostic = records;
    else
        diagnostic = [diagnostic; records]; %#ok<AGROW>
    end
end
output_file = fullfile(result_dir, ...
    sprintf('mpc_failure_diagnostics_n%d_seed%d.csv', n, seed));
writetable(diagnostic, output_file);
plot_final_windows(result_dir, figure_dir, case_ids, n, seed);
disp(diagnostic);
end

function records = replay_variant(root, result_dir, design, ...
        case_ids, variant, n, seed)
if variant == "mpc23"
    model_file = fullfile(root, 'models', 'HT_ctrl_MPC_23.slx');
    prefix = 'mpc_23';
else
    model_file = fullfile(root, 'models', 'HT_ctrl_MPC_robustness.slx');
    prefix = 'mpc_robustness';
end
saved_file = fullfile(result_dir, sprintf( ...
    '%s_montecarlo_n%d_seed%d_results.mat', prefix, n, seed));
saved_source = load(saved_file, 'results_table');
saved = saved_source.results_table;
if height(saved) ~= n || any(saved.Status == "")
    error('The recorded %s Monte Carlo run is incomplete.', variant);
end

load_system(model_file);
[~, model] = fileparts(model_file);
set_param(model, 'FastRestart', 'off', 'StopTime', '50');
abs_blocks = find_system(model, 'BlockType', 'Abs');
for k = 1:numel(abs_blocks)
    set_param(abs_blocks{k}, 'ZeroCross', 'off');
end
step_blocks = {[model '/wref step'], ...
    [model '/wref step1'], [model '/wref step2']};
for k = 1:numel(step_blocks)
    if getSimulinkBlockHandle(step_blocks{k}) == -1
        error('Required block not found: %s', step_blocks{k});
    end
end
cleanup_object = onCleanup(@() clean_up(model)); %#ok<NASGU>

records = table();
for k = 1:numel(case_ids)
    current = design(design.CaseID == case_ids(k), :);
    recorded = saved(saved.CaseID == case_ids(k), :);
    if height(current) ~= 1 || height(recorded) ~= 1 || ...
            recorded.Status ~= "success"
        error('Case %s has no unique successful saved result.', case_ids(k));
    end
    design_names = {'CaseID','Category','Description','Profile', ...
        'ActuatorGainScale','ActuatorTimeConstant','NoiseStd', ...
        'NoiseSeed','LoadPowerMW','LoadInitialState','LoadSwitchTime', ...
        'RateLimit','MeasurementDelay'};
    if ~isequaln(current(:, design_names), recorded(:, design_names))
        error('Saved result inputs differ from the design for %s.', ...
            case_ids(k));
    end
    steps = profile_values(current.Profile);
    for j = 1:numel(step_blocks)
        set_param(step_blocks{j}, 'After', num2str(steps(j), 17));
    end
    set_param([model '/Three-Phase Breaker1'], ...
        'InitialState', char(current.LoadInitialState));
    assignin('base', 'robust_actuator_gain_scale', ...
        current.ActuatorGainScale);
    assignin('base', 'robust_actuator_time_constant', ...
        current.ActuatorTimeConstant);
    assignin('base', 'robust_noise_std', current.NoiseStd);
    assignin('base', 'robust_noise_seed', current.NoiseSeed);
    assignin('base', 'robust_load_power_w', current.LoadPowerMW * 1e6);
    assignin('base', 'robust_load_switch_time', current.LoadSwitchTime);
    assignin('base', 'robust_actuator_rate_limit', current.RateLimit);
    assignin('base', 'robust_measurement_delay_s', ...
        current.MeasurementDelay);

    fprintf('%s diagnostic replay: %s\n', variant, case_ids(k));
    out = sim(model, 'ReturnWorkspaceOutputs', 'on');
    t = double(out.tout(:));
    y = double(out.y_mpc(:));
    u = double(out.u_mpc(:));
    target = sum(steps);
    window = (t >= 45) & (t <= 50);
    signed_bias = mean(y(window)) - target;
    fluctuation = max(y(window)) - min(y(window));
    error_delta = abs(abs(signed_bias) - recorded.SteadyError);
    fluctuation_delta = abs(fluctuation - recorded.SteadyFluctuation);
    if error_delta > 1e-10 || fluctuation_delta > 1e-10
        error('Replay differs from saved metrics for %s %s.', ...
            variant, case_ids(k));
    end
    reference = steps(1) * ones(size(t));
    reference(t >= 20) = steps(1) + steps(2);
    reference(t >= 35) = target;
    trace = struct('CaseID', case_ids(k), 'Variant', variant, ...
        't', t, 'y', y, 'u', u, 'reference', reference, ...
        'FinalTarget', target, 'RecordedSteadyError', ...
        recorded.SteadyError, 'ReplaySignedBias', signed_bias, ...
        'RecordedSteadyFluctuation', recorded.SteadyFluctuation, ...
        'ReplayFluctuation', fluctuation);
    trace_file = fullfile(result_dir, sprintf( ...
        '%s_montecarlo_n%d_seed%d_%s_trace.mat', ...
        prefix, n, seed, case_ids(k)));
    save(trace_file, 'trace');

    row = table(case_ids(k), variant, current.Profile, ...
        current.NoiseStd, current.MeasurementDelay, ...
        current.LoadPowerMW, current.LoadInitialState, ...
        target, recorded.SteadyError, abs(signed_bias), signed_bias, ...
        recorded.SteadyFluctuation, fluctuation, ...
        recorded.Pass, error_delta, fluctuation_delta, ...
        'VariableNames', {'CaseID','Variant','Profile','NoiseStd', ...
        'MeasurementDelay','LoadPowerMW','LoadInitialState', ...
        'FinalTarget','RecordedSteadyError','ReplaySteadyError', ...
        'ReplaySignedBias','RecordedSteadyFluctuation', ...
        'ReplaySteadyFluctuation','Pass','SteadyErrorReplayDifference', ...
        'FluctuationReplayDifference'});
    if isempty(records)
        records = row;
    else
        records = [records; row]; %#ok<AGROW>
    end
end
end

function steps = profile_values(profile)
switch string(profile)
    case "nominal"
        steps = [0.50, 0.25, 0.15];
    case "low"
        steps = [0.40, 0.15, 0.10];
    case "high"
        steps = [0.60, 0.25, 0.10];
    otherwise
        error('Unknown reference profile: %s', profile);
end
end

function plot_final_windows(result_dir, figure_dir, case_ids, n, seed)
f = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [80 80 1050 710]);
layout = tiledlayout(f, numel(case_ids), 1, ...
    'Padding', 'compact', 'TileSpacing', 'compact');
for k = 1:numel(case_ids)
    id = case_ids(k);
    old = load(fullfile(result_dir, sprintf( ...
        'mpc_robustness_montecarlo_n%d_seed%d_%s_trace.mat', ...
        n, seed, id)), 'trace');
    newer = load(fullfile(result_dir, sprintf( ...
        'mpc_23_montecarlo_n%d_seed%d_%s_trace.mat', ...
        n, seed, id)), 'trace');
    old = old.trace;
    newer = newer.trace;
    ax = nexttile(layout); hold(ax, 'on'); grid(ax, 'on');
    old_window = old.t >= 45 & old.t <= 50;
    new_window = newer.t >= 45 & newer.t <= 50;
    plot(ax, old.t(old_window), old.y(old_window), ...
        'Color', [0.57 0.62 0.68], 'LineWidth', 0.6);
    plot(ax, newer.t(new_window), newer.y(new_window), ...
        'Color', [0.04 0.52 0.55], 'LineWidth', 0.6);
    yline(ax, old.FinalTarget, '--', ...
        'Color', [0.20 0.24 0.28], 'LineWidth', 1.2);
    yline(ax, old.FinalTarget + old.ReplaySignedBias, ':', ...
        'Color', [0.38 0.43 0.49], 'LineWidth', 1.5);
    yline(ax, newer.FinalTarget + newer.ReplaySignedBias, ':', ...
        'Color', [0.02 0.38 0.40], 'LineWidth', 1.5);
    xlim(ax, [45 50]);
    ylabel(ax, 'Active power (pu)');
    title(ax, sprintf('%s: old fluctuation %.4f pu; upgraded bias %+.4f pu', ...
        id, old.ReplayFluctuation, newer.ReplaySignedBias), ...
        'Interpreter', 'none');
    if k == 1
        legend(ax, {'Phase 2 output','Phase 2.3 output', ...
            'Final target','Phase 2 mean','Phase 2.3 mean'}, ...
            'Location', 'best');
    end
end
xlabel(layout, 'Time (s)');
title(layout, 'Final five seconds of selected MPC cases');
if n == 100 && seed == 2401 && ...
        isequal(case_ids, ["mc_047"; "mc_098"])
    figure_name = 'mpc_23_failure_case_final_windows.png';
else
    figure_name = sprintf('mpc_diagnostic_n%d_seed%d.png', n, seed);
end
exportgraphics(f, fullfile(figure_dir, figure_name), ...
    'Resolution', 160);
close(f);
end

function clean_up(model)
variables = {'robust_actuator_gain_scale', ...
    'robust_actuator_time_constant', 'robust_noise_std', ...
    'robust_noise_seed', 'robust_load_power_w', ...
    'robust_load_switch_time', 'robust_actuator_rate_limit', ...
    'robust_measurement_delay_s'};
for k = 1:numel(variables)
    evalin('base', sprintf('clear %s', variables{k}));
end
if bdIsLoaded(model)
    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
end
