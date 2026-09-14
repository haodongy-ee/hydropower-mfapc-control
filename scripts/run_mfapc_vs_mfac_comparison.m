clear;
clc;

%% Project paths
% Place this script in the project's scripts folder.
script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
results_dir = fullfile(project_root, 'results');

prior_csv = fullfile( ...
    results_dir, ...
    'mfapc_vs_mfac_comparison_v3.csv');

if ~isfile(prior_csv)
    error('The existing v3 CSV was not found: %s', prior_csv);
end

mfac_candidates = { ...
    fullfile(project_root, '02_working', 'HT_ctrl_MFAC.slx'), ...
    fullfile(project_root, '01_original', 'HT_ctrl_MFAC.slx')};

mfac_file = '';

for candidate_index = 1:numel(mfac_candidates)
    if isfile(mfac_candidates{candidate_index})
        mfac_file = mfac_candidates{candidate_index};
        break;
    end
end

if isempty(mfac_file)
    error([ ...
        'HT_ctrl_MFAC.slx was not found. Place it in either ', ...
        '02_working or 01_original.']);
end

%% Retain the six valid MFAPC rows from the v3 result
prior_results = readtable(prior_csv, 'TextType', 'string');
mfapc_rows = prior_results(prior_results.Controller == "MFAPC_final", :);

if height(mfapc_rows) ~= 6
    error('Expected six MFAPC_final rows in the v3 CSV.');
end

mfapc_rows.Status(:) = "success";
mfapc_rows.ErrorMessage(:) = "";

%% Load independent MFAC model
[~, mfac_model] = fileparts(mfac_file);
mfac_was_loaded = bdIsLoaded(mfac_model);
load_system(mfac_file);

set_param(mfac_model, 'FastRestart', 'off');

original_stop_time = get_param(mfac_model, 'StopTime');
set_param(mfac_model, 'StopTime', '50');

disable_abs_zero_crossing(mfac_model);

%% Add a temporary output logger
mfac_logger = [mfac_model '/Comparison Output Logger'];

if getSimulinkBlockHandle(mfac_logger) ~= -1
    delete_block(mfac_logger);
end

add_block( ...
    'simulink/Sinks/To Workspace', ...
    mfac_logger, ...
    'VariableName', ...
    'y_mfac_compare', ...
    'SaveFormat', ...
    'Array', ...
    'SampleTime', ...
    '-1', ...
    'Position', ...
    [1020, 690, 1150, 720]);

add_line( ...
    mfac_model, ...
    'From4/1', ...
    'Comparison Output Logger/1', ...
    'autorouting', ...
    'on');

%% Step blocks and automatic cleanup
mfac_steps = { ...
    [mfac_model '/wref step'], ...
    [mfac_model '/wref step1'], ...
    [mfac_model '/wref step2']};

validate_blocks(mfac_steps);

original_step_values = cellfun( ...
    @(block) get_param(block, 'After'), ...
    mfac_steps, ...
    'UniformOutput', ...
    false);

cleanup_object = onCleanup(@() clean_up_model( ...
    mfac_model, ...
    mfac_steps, ...
    original_step_values, ...
    original_stop_time, ...
    mfac_logger, ...
    mfac_was_loaded)); %#ok<NASGU>

%% Use the same six operating conditions as the v3 experiment
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

%% Prepare six replacement MFAC rows
Scenario = scenario_names;
Controller = repmat("MFAC_baseline", number_of_cases, 1);
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

%% Run only the six missing MFAC simulations
for case_number = 1:number_of_cases

    set_step_values(mfac_steps, step_values(case_number, :));

    fprintf( ...
        '\nRunning MFAC completion case %d of %d: %s\n', ...
        case_number, ...
        number_of_cases, ...
        scenario_names(case_number));

    try
        out = sim(mfac_model, 'ReturnWorkspaceOutputs', 'on');

        t = double(reshape(out.tout, [], 1));
        y = double(reshape(squeeze(out.y_mfac_compare), [], 1));

        if numel(y) ~= numel(t)
            error('The logged MFAC output length does not match tout.');
        end

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
            'MFAC completion failed for %s: %s', ...
            scenario_names(case_number), ...
            simulation_error.message);
    end
end

%% Build completed 12-row comparison table
mfac_rows = table( ...
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

results_table = mfapc_rows([],:);

for case_number = 1:number_of_cases
    current_name = scenario_names(case_number);
    results_table = [ ...
        results_table; ...
        mfapc_rows(mfapc_rows.Scenario == current_name, :); ...
        mfac_rows(mfac_rows.Scenario == current_name, :)]; %#ok<AGROW>
end

%% Save final completed results
csv_file = fullfile( ...
    results_dir, ...
    'mfapc_vs_mfac_comparison_v4.csv');

mat_file = fullfile( ...
    results_dir, ...
    'mfapc_vs_mfac_comparison_v4.mat');

writetable(results_table, csv_file);
save(mat_file, 'results_table', 'step_values');

disp(results_table);

fprintf('\nCompleted six additional MFAC simulations.\n');
fprintf('Final successful rows: %d of %d\n', ...
    sum(results_table.Status == "success"), ...
    height(results_table));
fprintf('CSV saved to:\n%s\n', csv_file);
fprintf('MAT file saved to:\n%s\n', mat_file);

%% Local functions
function validate_blocks(blocks)
    for block_index = 1:numel(blocks)
        if getSimulinkBlockHandle(blocks{block_index}) == -1
            error('Required block was not found: %s', blocks{block_index});
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
    elseif last_bad < length(local_t_35)
        metrics.SettlingTime35 = local_t_35(last_bad + 1) - 35;
    else
        metrics.SettlingTime35 = NaN;
    end

    index_steady = (t >= 45) & (t <= 50);
    steady_y = double(y(index_steady));
    steady_y = steady_y(:);

    if isempty(steady_y)
        error('No MFAC samples were recorded in the 45-50 s window.');
    end

    metrics.SteadyMean = sum(steady_y) / numel(steady_y);
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

function clean_up_model( ...
    model, ...
    step_blocks, ...
    original_step_values, ...
    original_stop_time, ...
    logger_block, ...
    model_was_loaded)

    if ~bdIsLoaded(model)
        return;
    end

    for block_index = 1:numel(step_blocks)
        if getSimulinkBlockHandle(step_blocks{block_index}) ~= -1
            set_param( ...
                step_blocks{block_index}, ...
                'After', ...
                original_step_values{block_index});
        end
    end

    set_param(model, 'StopTime', original_stop_time);

    if getSimulinkBlockHandle(logger_block) ~= -1
        delete_block(logger_block);
    end

    if ~model_was_loaded
        close_system(model, 0);
    end
end
