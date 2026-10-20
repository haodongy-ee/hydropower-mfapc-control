function build_mpc_23_model()
%BUILD_MPC_23_MODEL Build a separate R2023a phase-2.3 robustness model.
script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
source_file = fullfile(project_root, 'models', ...
    'HT_ctrl_MPC_robustness.slx');
target_file = fullfile(project_root, 'models', ...
    'HT_ctrl_MPC_23.slx');
controller_file = fullfile(scripts_dir, 'mpc_23_controller_block.m');
parameter_file = fullfile(project_root, 'baselines', ...
    'phase1_offsetfree_mpc', 'mpc_controller_parameters_phase1.mat');
if ~isfile(source_file) || ~isfile(controller_file) || ...
        ~isfile(parameter_file)
    error('Phase-2.3 build requires the robustness model, controller and frozen parameters.');
end

loaded = load(parameter_file, 'mpc_params');
if ~isfield(loaded, 'mpc_params') || numel(loaded.mpc_params) ~= 14
    error('Frozen phase-1 MPC parameter vector must have 14 values.');
end
mpc23_params = [loaded.mpc_params(:).', ...
    0.05, ...  % steady measurement low-pass coefficient per 0.05 s update
    0.02, ...  % additive-output-disturbance observer gain
    0.35, ...  % lower soft output bound (pu)
    1.00, ...  % upper soft output bound (pu)
    20.0, ...  % quadratic soft-output violation penalty
    0.10];    % saturation unwind factor per MPC update

copyfile(source_file, target_file, 'f');
[~, model] = fileparts(target_file);
load_system(target_file);
cleanup_object = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>
controller_block = [model '/MPC_Controller'];
if getSimulinkBlockHandle(controller_block) == -1
    error('MPC controller block missing: %s', controller_block);
end
root = sfroot;
chart = root.find('-isa', 'Stateflow.EMChart', 'Path', controller_block);
if numel(chart) ~= 1
    error('Expected one MATLAB Function chart at %s.', controller_block);
end
chart.Script = fileread(controller_file);
set_param([model '/Lambda'], 'Value', mat2str(mpc23_params, 17));
set_param(model, 'Description', [ ...
    'Phase-2.3 filtered, disturbance-estimating MPC with soft output ', ...
    'constraints and anti-windup; generated from phase-2 harness.']);
save_system(model);
save(fullfile(project_root, 'results', 'mpc_23_parameters.mat'), ...
    'mpc23_params');
fprintf('Created phase-2.3 MPC model:\n%s\n', target_file);
end

function close_if_loaded(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end
