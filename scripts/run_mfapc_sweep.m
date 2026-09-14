clear;
clc;

%% Project paths
script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);

model_file = fullfile( ...
    project_root, ...
    '02_working', ...
    'HT_ctrl_MFAPC_sweep.slx');

results_dir = fullfile(project_root, 'results');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

%% Load model
load_system(model_file);
model = 'HT_ctrl_MFAPC_sweep';

set_param(model, 'FastRestart', 'off');

%% Parameter order
% [lambda, rho, alpha_r, du_limit, eta, mu, alpha_d]

base_params = [ ...
    2.00, ...  % lambda
    0.32, ...  % rho
    0.10, ...  % alpha_r
    0.030, ... % du_limit
    0.06, ...  % eta
    0.08, ...  % mu
    0.08];     % alpha_d

%% Parameters to test
parameter_indices = [1, 2, 3, 4];

parameter_names = [ ...
    "lambda", ...
    "rho", ...
    "alpha_r", ...
    "du_limit"];

parameter_values = { ...
    [1.40, 1.70, 2.00, 2.30, 2.60], ...
    [0.24, 0.28, 0.32, 0.36, 0.40], ...
    [0.06, 0.08, 0.10, 0.12, 0.15], ...
    [0.020, 0.025, 0.030, 0.035]};

%% Construct experiment cases
parameter_matrix = base_params;
changed_parameter = "baseline";
changed_value = NaN;

for group = 1:length(parameter_indices)

    parameter_index = parameter_indices(group);
    values = parameter_values{group};

    for value = values

        % Avoid repeating the baseline case
        if abs(value - base_params(parameter_index)) < 1e-12
            continue;
        end

        new_params = base_params;
        new_params(parameter_index) = value;

        parameter_matrix(end + 1, :) = new_params;
        changed_parameter(end + 1, 1) = parameter_names(group);
        changed_value(end + 1, 1) = value;
    end
end

number_of_cases = size(parameter_matrix, 1);

%% Prepare result variables
IAE = NaN(number_of_cases, 1);
ITAE = NaN(number_of_cases, 1);
Overshoot20 = NaN(number_of_cases, 1);
Overshoot35 = NaN(number_of_cases, 1);
SettlingTime = NaN(number_of_cases, 1);
SteadyMean = NaN(number_of_cases, 1);
SteadyError = NaN(number_of_cases, 1);
SteadyFluctuation = NaN(number_of_cases, 1);
Status = strings(number_of_cases, 1);

%% Run all experiments
for case_number = 1:number_of_cases

    controller_params = parameter_matrix(case_number, :);

    fprintf( ...
        '\nRunning case %d of %d: %s = %.4f\n', ...
        case_number, ...
        number_of_cases, ...
        changed_parameter(case_number), ...
        changed_value(case_number));

    try
        out = sim(model, 'ReturnWorkspaceOutputs', 'on');

        t = out.tout;
        y = squeeze(out.y_mfapc);

        iae_data = out.IAE_mfapc_20_50;
        itae_data = out.ITAE_mfapc_20_50;

        IAE(case_number) = iae_data(end);
        ITAE(case_number) = itae_data(end);

        %% Overshoot after the 20-second step
        index_20 = (t >= 20) & (t < 35);
        peak_20 = max(y(index_20));

        Overshoot20(case_number) = ...
            max(0, (peak_20 - 0.75) / 0.75 * 100);

        %% Overshoot after the 35-second step
        index_35 = (t >= 35) & (t <= 50);
        peak_35 = max(y(index_35));

        Overshoot35(case_number) = ...
            max(0, (peak_35 - 0.90) / 0.90 * 100);

        %% Two-percent settling time after 35 seconds
        target = 0.90;
        tolerance = 0.02 * target;

        local_t = t(index_35);
        local_y = y(index_35);

        last_bad = find( ...
            abs(local_y - target) > tolerance, ...
            1, ...
            'last');

        if isempty(last_bad)
            SettlingTime(case_number) = 0;
        elseif last_bad < length(local_t)
            SettlingTime(case_number) = ...
                local_t(last_bad + 1) - 35;
        else
            SettlingTime(case_number) = NaN;
        end

        %% Steady-state performance from 45 to 50 seconds
        index_steady = (t >= 45) & (t <= 50);
        steady_y = y(index_steady);

        SteadyMean(case_number) = mean(steady_y);
        SteadyError(case_number) = ...
            abs(SteadyMean(case_number) - target);

        SteadyFluctuation(case_number) = ...
            max(steady_y) - min(steady_y);

        Status(case_number) = "success";

    catch simulation_error

        Status(case_number) = "failed";

        warning( ...
            'Case %d failed: %s', ...
            case_number, ...
            simulation_error.message);
    end
end

%% Create results table
results_table = table( ...
    changed_parameter, ...
    changed_value, ...
    parameter_matrix(:, 1), ...
    parameter_matrix(:, 2), ...
    parameter_matrix(:, 3), ...
    parameter_matrix(:, 4), ...
    parameter_matrix(:, 5), ...
    parameter_matrix(:, 6), ...
    parameter_matrix(:, 7), ...
    IAE, ...
    ITAE, ...
    Overshoot20, ...
    Overshoot35, ...
    SettlingTime, ...
    SteadyMean, ...
    SteadyError, ...
    SteadyFluctuation, ...
    Status, ...
    'VariableNames', { ...
    'ChangedParameter', ...
    'ChangedValue', ...
    'Lambda', ...
    'Rho', ...
    'AlphaR', ...
    'DuLimit', ...
    'Eta', ...
    'Mu', ...
    'AlphaD', ...
    'IAE', ...
    'ITAE', ...
    'Overshoot20', ...
    'Overshoot35', ...
    'SettlingTime', ...
    'SteadyMean', ...
    'SteadyError', ...
    'SteadyFluctuation', ...
    'Status'});

%% Save results
csv_file = fullfile( ...
    results_dir, ...
    'mfapc_parameter_sweep.csv');

mat_file = fullfile( ...
    results_dir, ...
    'mfapc_parameter_sweep.mat');

writetable(results_table, csv_file);
save(mat_file, 'results_table', 'parameter_matrix');

%% Reset parameters to current best candidate
controller_params = base_params;

%% Show results
disp(results_table);

fprintf('\nFinished %d experiments.\n', number_of_cases);
fprintf('CSV saved to:\n%s\n', csv_file);
fprintf('MAT file saved to:\n%s\n', mat_file);