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
set_param([model '/Abs1'], 'ZeroCross', 'off');

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

%% Joint-search ranges
lambda_values  = [2.15, 2.25, 2.35, 2.45];
rho_values     = [0.29, 0.31, 0.33];
alpha_r_values = 0.15;

du_limit = 0.030;
eta = 0.06;
mu = 0.08;
alpha_d = 0.08;

%% Construct all parameter combinations
number_of_cases = ...
    length(lambda_values) * ...
    length(rho_values) * ...
    length(alpha_r_values);

parameter_matrix = zeros(number_of_cases, 7);
changed_parameter = strings(number_of_cases, 1);
changed_value = zeros(number_of_cases, 1);

case_index = 0;

for lambda = lambda_values
    for rho = rho_values
        for alpha_r = alpha_r_values

            case_index = case_index + 1;

            parameter_matrix(case_index, :) = [ ...
                lambda, ...
                rho, ...
                alpha_r, ...
                du_limit, ...
                eta, ...
                mu, ...
                alpha_d];

            changed_parameter(case_index) = "joint_grid";
            changed_value(case_index) = case_index;
        end
    end
end

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
    'mfapc_local_search.csv');

mat_file = fullfile( ...
    results_dir, ...
    'mfapc_local_search.mat');

writetable(results_table, csv_file);
save(mat_file, 'results_table', 'parameter_matrix');

%% Reset parameters to current best candidate
controller_params = base_params;

%% Show results
disp(results_table);

fprintf('\nFinished %d experiments.\n', number_of_cases);
fprintf('CSV saved to:\n%s\n', csv_file);
fprintf('MAT file saved to:\n%s\n', mat_file);