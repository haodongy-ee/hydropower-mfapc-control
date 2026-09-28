clear;
clc;

script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
results_dir = fullfile(project_root, 'results');
model_file = fullfile(project_root, 'models', 'HT_ctrl_MPC_baseline.slx');
parameter_file = fullfile(results_dir, 'mpc_controller_parameters.mat');

loaded = load(parameter_file, 'mpc_params');
base_params = loaded.mpc_params;

% Dedicated tuning profiles. These are deliberately different from the six
% final benchmark cases used by run_mpc_comparison.m.
scenario_names = ["tune_smooth_up"; "tune_reversal"; "tune_high_output"];
step_values = [ ...
    0.50, 0.20,  0.12; ...
    0.45, 0.28, -0.12; ...
    0.55, 0.28,  0.08];

% [move weight, delta-u limit, integral gain, reference-filter alpha]
candidates = [ ...
    40, 0.015, 0.030, 0.10; ...
    50, 0.012, 0.030, 0.08];

load_system(model_file);
[~, model] = fileparts(model_file);
set_param(model, 'FastRestart', 'off');
set_param(model, 'StopTime', '50');

abs_blocks = find_system(model, 'BlockType', 'Abs');
for block_index = 1:numel(abs_blocks)
    set_param(abs_blocks{block_index}, 'ZeroCross', 'off');
end

step_blocks = { ...
    [model '/wref step'], ...
    [model '/wref step1'], ...
    [model '/wref step2']};
parameter_block = [model '/Lambda'];

number_of_candidates = size(candidates, 1);
number_of_scenarios = size(step_values, 1);
rows = number_of_candidates * number_of_scenarios;

Candidate = zeros(rows, 1);
Scenario = strings(rows, 1);
MoveWeight = zeros(rows, 1);
DuLimit = zeros(rows, 1);
IntegralGain = zeros(rows, 1);
ReferenceAlpha = zeros(rows, 1);
IAE = NaN(rows, 1);
ITAE = NaN(rows, 1);
Overshoot20 = NaN(rows, 1);
Overshoot35 = NaN(rows, 1);
SteadyError = NaN(rows, 1);
SteadyFluctuation = NaN(rows, 1);
Score = Inf(rows, 1);

row = 0;

for candidate_index = 1:number_of_candidates
    params = base_params;
    params(7) = candidates(candidate_index, 1);
    params(8) = candidates(candidate_index, 2);
    params(11) = candidates(candidate_index, 3);
    params(14) = candidates(candidate_index, 4);
    set_param(parameter_block, 'Value', mat2str(params, 17));

    for scenario_index = 1:number_of_scenarios
        row = row + 1;
        values = step_values(scenario_index, :);

        for block_index = 1:3
            set_param(step_blocks{block_index}, 'After', ...
                num2str(values(block_index), 17));
        end

        fprintf('Candidate %d/%d, scenario %d/%d: %s\n', ...
            candidate_index, number_of_candidates, ...
            scenario_index, number_of_scenarios, ...
            scenario_names(scenario_index));

        out = sim(model, 'ReturnWorkspaceOutputs', 'on');
        t = double(out.tout(:));
        y = double(out.y_mpc(:));

        reference = values(1) * ones(size(t));
        reference(t >= 20) = values(1) + values(2);
        reference(t >= 35) = sum(values);
        metrics = calculate_metrics(t, y, reference, values);

        Candidate(row) = candidate_index;
        Scenario(row) = scenario_names(scenario_index);
        MoveWeight(row) = params(7);
        DuLimit(row) = params(8);
        IntegralGain(row) = params(11);
        ReferenceAlpha(row) = params(14);
        IAE(row) = metrics.IAE;
        ITAE(row) = metrics.ITAE;
        Overshoot20(row) = metrics.Overshoot20;
        Overshoot35(row) = metrics.Overshoot35;
        SteadyError(row) = metrics.SteadyError;
        SteadyFluctuation(row) = metrics.SteadyFluctuation;

        Score(row) = metrics.IAE + 0.05 * metrics.ITAE + ...
            0.02 * (metrics.Overshoot20 + metrics.Overshoot35) + ...
            30.0 * metrics.SteadyError + ...
            2.0 * metrics.SteadyFluctuation + ...
            0.20 * max(metrics.Overshoot35 - 1.0, 0.0) + ...
            100.0 * max(metrics.SteadyError - 0.0005, 0.0) + ...
            10.0 * max(metrics.SteadyFluctuation - 0.0060, 0.0);
    end
end

tuning_results = table( ...
    Candidate, Scenario, MoveWeight, DuLimit, IntegralGain, ReferenceAlpha, ...
    IAE, ITAE, Overshoot20, Overshoot35, SteadyError, ...
    SteadyFluctuation, Score);

candidate_summary = groupsummary( ...
    tuning_results, 'Candidate', 'mean', ...
    {'IAE', 'ITAE', 'Overshoot20', 'Overshoot35', ...
    'SteadyError', 'SteadyFluctuation', 'Score'});
[~, best_row] = min(candidate_summary.mean_Score);
selected_candidate = candidate_summary.Candidate(best_row);
selected_values = candidates(selected_candidate, :);

selected_params = base_params;
selected_params(7) = selected_values(1);
selected_params(8) = selected_values(2);
selected_params(11) = selected_values(3);
selected_params(14) = selected_values(4);

writetable(tuning_results, ...
    fullfile(results_dir, 'mpc_offsetfree_final_refinement.csv'));
writetable(candidate_summary, ...
    fullfile(results_dir, 'mpc_offsetfree_final_refinement_summary.csv'));
save(fullfile(results_dir, 'mpc_offsetfree_selected.mat'), ...
    'selected_params', 'selected_candidate', 'candidate_summary');

disp(candidate_summary);
fprintf('Selected candidate: %d\n', selected_candidate);
fprintf('Selected parameters: %s\n', mat2str(selected_params, 10));
close_system(model, 0);

function metrics = calculate_metrics(t, y, reference, values)
index_metrics = t >= 20;
metrics.IAE = trapz(t(index_metrics), ...
    abs(reference(index_metrics) - y(index_metrics)));
metrics.ITAE = trapz(t(index_metrics), ...
    (t(index_metrics) - 20) .* ...
    abs(reference(index_metrics) - y(index_metrics)));

target20 = values(1) + values(2);
target35 = sum(values);
local20 = y(t >= 20 & t < 35);
local35 = y(t >= 35);

metrics.Overshoot20 = directional_overshoot(local20, target20, values(2));
metrics.Overshoot35 = directional_overshoot(local35, target35, values(3));
steady = y(t >= 45);
metrics.SteadyError = abs(target35 - mean(steady));
metrics.SteadyFluctuation = max(steady) - min(steady);
end

function value = directional_overshoot(y, target, step)
if step >= 0
    excursion = max(y) - target;
else
    excursion = target - min(y);
end
value = max(excursion, 0) / max(abs(target), eps) * 100;
end
