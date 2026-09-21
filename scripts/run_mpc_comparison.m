clear;
clc;

%% Project paths
script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
results_dir = fullfile(project_root, 'results');
model_file = fullfile(project_root, 'models', 'HT_ctrl_MPC_baseline.slx');
prior_csv = fullfile(results_dir, 'mfapc_vs_mfac_comparison.csv');

if ~isfile(model_file)
    error([ ...
        'MPC model not found: %s\nRun scripts/build_mpc_model.m first.'], ...
        model_file);
end

if ~isfile(prior_csv)
    error('MFAPC/MFAC comparison file not found: %s', prior_csv);
end

prior_results = readtable(prior_csv, 'TextType', 'string');

if height(prior_results) ~= 12
    error('Expected 12 existing MFAPC/MFAC rows, found %d.', height(prior_results));
end

%% Six benchmark scenarios used by the existing study
scenario_names = [ ...
    "nominal_up"; ...
    "small_up"; ...
    "high_output_up"; ...
    "large_middle_small_final"; ...
    "rise_then_drop"; ...
    "small_rise_then_drop"];

step_values = [ ...
     0.50, 0.25,  0.15; ...
     0.50, 0.15,  0.10; ...
     0.50, 0.30,  0.15; ...
     0.50, 0.35,  0.05; ...
     0.50, 0.25, -0.15; ...
     0.50, 0.15, -0.10];

number_of_cases = size(step_values, 1);

%% Load the independently generated MPC model
load_system(model_file);
[~, model] = fileparts(model_file);
set_param(model, 'FastRestart', 'off');
set_param(model, 'StopTime', '50');
disable_abs_zero_crossing(model);

step_blocks = { ...
    [model '/wref step'], ...
    [model '/wref step1'], ...
    [model '/wref step2']};
validate_blocks(step_blocks);

original_step_values = cellfun( ...
    @(block) get_param(block, 'After'), ...
    step_blocks, ...
    'UniformOutput', false);

cleanup_object = onCleanup(@() clean_up_model( ...
    model, step_blocks, original_step_values)); %#ok<NASGU>

%% Prepare MPC result rows
Scenario = scenario_names;
Controller = repmat("MPC_baseline", number_of_cases, 1);
Step0 = step_values(:, 1);
Step20 = step_values(:, 2);
Step35 = step_values(:, 3);
Target20 = Step0 + Step20;
Target35 = sum(step_values, 2);
IAE = NaN(number_of_cases, 1);
ITAE = NaN(number_of_cases, 1);
DirectionalOvershoot20 = NaN(number_of_cases, 1);
DirectionalOvershoot35 = NaN(number_of_cases, 1);
SettlingTime35 = NaN(number_of_cases, 1);
SteadyMean = NaN(number_of_cases, 1);
SteadyError = NaN(number_of_cases, 1);
SteadyFluctuation = NaN(number_of_cases, 1);
Status = strings(number_of_cases, 1);
ErrorMessage = strings(number_of_cases, 1);

%% Run MPC on the same six operating conditions
for case_number = 1:number_of_cases
    set_step_values(step_blocks, step_values(case_number, :));

    fprintf( ...
        'Running MPC case %d/%d: %s\n', ...
        case_number, ...
        number_of_cases, ...
        scenario_names(case_number));

    try
        out = sim(model, 'ReturnWorkspaceOutputs', 'on');
        t = double(out.tout(:));
        y = double(out.y_mpc(:));

        reference = Step0(case_number) * ones(size(t));
        reference(t >= 20) = Target20(case_number);
        reference(t >= 35) = Target35(case_number);

        metrics = calculate_metrics( ...
            t, ...
            y, ...
            reference, ...
            Target20(case_number), ...
            Target35(case_number), ...
            Step20(case_number), ...
            Step35(case_number));

        IAE(case_number) = metrics.IAE;
        ITAE(case_number) = metrics.ITAE;
        DirectionalOvershoot20(case_number) = metrics.Overshoot20;
        DirectionalOvershoot35(case_number) = metrics.Overshoot35;
        SettlingTime35(case_number) = metrics.SettlingTime35;
        SteadyMean(case_number) = metrics.SteadyMean;
        SteadyError(case_number) = metrics.SteadyError;
        SteadyFluctuation(case_number) = metrics.SteadyFluctuation;
        Status(case_number) = "success";
        ErrorMessage(case_number) = "";

    catch simulation_error
        Status(case_number) = "failed";
        ErrorMessage(case_number) = string(simulation_error.message);

        warning( ...
            'MPC case %s failed: %s', ...
            scenario_names(case_number), ...
            simulation_error.message);
    end
end

mpc_rows = table( ...
    Scenario, ...
    Controller, ...
    Step0, ...
    Step20, ...
    Step35, ...
    Target20, ...
    Target35, ...
    IAE, ...
    ITAE, ...
    DirectionalOvershoot20, ...
    DirectionalOvershoot35, ...
    SettlingTime35, ...
    SteadyMean, ...
    SteadyError, ...
    SteadyFluctuation, ...
    Status, ...
    ErrorMessage);

%% Interleave the three controllers scenario by scenario
results_table = prior_results([],:);

for case_number = 1:number_of_cases
    scenario = scenario_names(case_number);
    existing_rows = prior_results(prior_results.Scenario == scenario, :);

    if height(existing_rows) ~= 2
        error('Expected two existing rows for scenario %s.', scenario);
    end

    results_table = [ ...
        results_table; ...
        existing_rows; ...
        mpc_rows(mpc_rows.Scenario == scenario, :)]; %#ok<AGROW>
end

csv_file = fullfile(results_dir, 'three_controller_comparison.csv');
mat_file = fullfile(results_dir, 'three_controller_comparison.mat');
writetable(results_table, csv_file);
save(mat_file, 'results_table', 'step_values');

disp(results_table);
fprintf('\nSuccessful MPC cases: %d/%d\n', sum(Status == "success"), number_of_cases);
fprintf('Three-controller CSV saved to:\n%s\n', csv_file);

%% Local functions
function validate_blocks(blocks)
    for block_index = 1:numel(blocks)
        if getSimulinkBlockHandle(blocks{block_index}) == -1
            error('Required block not found: %s', blocks{block_index});
        end
    end
end

function set_step_values(blocks, values)
    for block_index = 1:numel(blocks)
        set_param( ...
            blocks{block_index}, ...
            'After', ...
            num2str(values(block_index), 17));
    end
end

function disable_abs_zero_crossing(model)
    abs_blocks = find_system(model, 'BlockType', 'Abs');

    for block_index = 1:numel(abs_blocks)
        set_param(abs_blocks{block_index}, 'ZeroCross', 'off');
    end
end

function metrics = calculate_metrics( ...
    t, y, reference, target_20, target_35, step_20, step_35)

    index_metrics = (t >= 20) & (t <= 50);
    metric_t = t(index_metrics);
    absolute_error = abs(reference(index_metrics) - y(index_metrics));

    metrics.IAE = trapz(metric_t, absolute_error);
    metrics.ITAE = trapz( ...
        metric_t, ...
        (metric_t - 20) .* absolute_error);

    index_20 = (t >= 20) & (t < 35);
    index_35 = (t >= 35) & (t <= 50);
    local_y_20 = y(index_20);
    local_t_35 = t(index_35);
    local_y_35 = y(index_35);

    metrics.Overshoot20 = directional_overshoot( ...
        local_y_20, target_20, step_20);
    metrics.Overshoot35 = directional_overshoot( ...
        local_y_35, target_35, step_35);

    tolerance = 0.02 * max(abs(target_35), eps);
    last_bad = find(abs(local_y_35 - target_35) > tolerance, 1, 'last');

    if isempty(last_bad)
        metrics.SettlingTime35 = 0;
    elseif last_bad < numel(local_t_35)
        metrics.SettlingTime35 = local_t_35(last_bad + 1) - 35;
    else
        metrics.SettlingTime35 = NaN;
    end

    index_steady = (t >= 45) & (t <= 50);
    steady_y = y(index_steady);
    metrics.SteadyMean = mean(steady_y);
    metrics.SteadyError = abs(metrics.SteadyMean - target_35);
    metrics.SteadyFluctuation = max(steady_y) - min(steady_y);
end

function value = directional_overshoot(y, target, step_change)
    denominator = max(abs(target), eps);

    if step_change >= 0
        value = max(0, (max(y) - target) / denominator * 100);
    else
        value = max(0, (target - min(y)) / denominator * 100);
    end
end

function clean_up_model(model, step_blocks, original_step_values)
    if ~bdIsLoaded(model)
        return;
    end

    for block_index = 1:numel(step_blocks)
        set_param( ...
            step_blocks{block_index}, ...
            'After', ...
            original_step_values{block_index});
    end

    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
