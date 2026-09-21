clear;
clc;

%% Project paths and fixed experiment settings
script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
model_file = fullfile(project_root, 'models', 'HT_ctrl_MFAPC_final.slx');
results_dir = fullfile(project_root, 'results');

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

if ~isfile(model_file)
    error('MFAPC model not found: %s', model_file);
end

sample_time = 0.01;
controller_params = [2.40, 0.32, 0.18, 0.030, 0.06, 0.08, 0.08];

% These profiles are reserved for model identification. None is one of the
% six benchmark profiles used by run_mpc_comparison.m.
identification_targets = [ ...
    0.50, 0.68, 0.82; ...
    0.50, 0.72, 0.58; ...
    0.50, 0.82, 0.68; ...
    0.50, 0.70, 0.78];

%% Load the R2023a Simulink model
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

%% Generate closed-loop identification trajectories
profile_data = cell(size(identification_targets, 1), 1);
profile_data_file = fullfile(results_dir, 'mpc_identification_profiles.mat');

for profile_index = 1:size(identification_targets, 1)
    targets = identification_targets(profile_index, :);
    steps = [targets(1), diff(targets)];
    set_step_values(step_blocks, steps);

    fprintf( ...
        'Identification profile %d/%d: %.2f -> %.2f -> %.2f pu\n', ...
        profile_index, ...
        size(identification_targets, 1), ...
        targets(1), ...
        targets(2), ...
        targets(3));

    out = sim(model, 'ReturnWorkspaceOutputs', 'on');
    t_solver = double(out.tout(:));
    y_solver = double(out.y_mfapc(:));
    u = double(out.u_mf(:));
    t = (0:numel(u) - 1)' * sample_time;
    y = interp1(t_solver, y_solver, t, 'linear', 'extrap');

    % Discard startup initialization. The remaining data span both upward
    % and downward movements around the intended operating region.
    keep = t >= 5;
    profile_data{profile_index} = struct( ...
        't', t(keep), ...
        'y', y(keep), ...
        'u', u(keep));

    save( ...
        profile_data_file, ...
        'profile_data', ...
        'identification_targets', ...
        'sample_time');
end

%% Fit a fixed second-order affine ARX model
% y(k) = a1*y(k-1) + a2*y(k-2) + b1*u(k-1) + b2*u(k-2) + c
% The first three profiles are the training set. The fourth is held out for
% validation so the reported fit is not an in-sample number.
[x_train, y_train] = build_regression(profile_data(1:3));
[x_validation, y_validation] = build_regression(profile_data(4));

ridge = 1e-8;
penalty = eye(size(x_train, 2));
penalty(end, end) = 0;
theta = (x_train' * x_train + ridge * penalty) \ (x_train' * y_train);

validation_prediction = x_validation * theta;
validation_error = y_validation - validation_prediction;
validation_rmse = sqrt(mean(validation_error .^ 2));
validation_span = max(y_validation) - min(y_validation);
validation_nrmse = validation_rmse / max(validation_span, eps);
validation_r2 = 1 - sum(validation_error .^ 2) / ...
    max(sum((y_validation - mean(y_validation)) .^ 2), eps);

ar_roots = roots([1, -theta(1), -theta(2)]);
stable_model = all(abs(ar_roots) < 1);

if ~stable_model
    error( ...
        'Identified ARX model is unstable (poles: %s).', ...
        mat2str(ar_roots, 6));
end

mpc_model = struct( ...
    'sample_time', sample_time, ...
    'a1', theta(1), ...
    'a2', theta(2), ...
    'b1', theta(3), ...
    'b2', theta(4), ...
    'bias', theta(5), ...
    'validation_rmse', validation_rmse, ...
    'validation_nrmse', validation_nrmse, ...
    'validation_r2', validation_r2, ...
    'poles', ar_roots, ...
    'identification_targets', identification_targets);

save(fullfile(results_dir, 'mpc_identified_model.mat'), 'mpc_model');

coefficient = ["a1"; "a2"; "b1"; "b2"; "bias"];
value = theta;
coefficient_table = table(coefficient, value);
writetable( ...
    coefficient_table, ...
    fullfile(results_dir, 'mpc_identified_model.csv'));

fprintf('\nIdentified model:\n');
fprintf( ...
    'y(k) = %.10f y(k-1) %+ .10f y(k-2) %+ .10f u(k-1) %+ .10f u(k-2) %+ .10f\n', ...
    theta(1), theta(2), theta(3), theta(4), theta(5));
fprintf('Poles: %s\n', mat2str(ar_roots, 8));
fprintf('Held-out one-step RMSE: %.8g pu\n', validation_rmse);
fprintf('Held-out normalized RMSE: %.4f%%\n', 100 * validation_nrmse);
fprintf('Held-out R^2: %.8f\n', validation_r2);

%% Local functions
function [x, target] = build_regression(profiles)
    x = zeros(0, 5);
    target = zeros(0, 1);

    for profile_index = 1:numel(profiles)
        y = profiles{profile_index}.y;
        u = profiles{profile_index}.u;

        local_x = [ ...
            y(2:end-1), ...
            y(1:end-2), ...
            u(2:end-1), ...
            u(1:end-2), ...
            ones(numel(y) - 2, 1)];
        local_target = y(3:end);

        x = [x; local_x]; %#ok<AGROW>
        target = [target; local_target]; %#ok<AGROW>
    end
end

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
