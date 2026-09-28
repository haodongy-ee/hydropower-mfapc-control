clear;
clc;

%% Project paths
script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
source_file = fullfile(project_root, 'models', 'HT_ctrl_MFAPC_final.slx');
target_file = fullfile(project_root, 'models', 'HT_ctrl_MPC_baseline.slx');
identified_model_file = fullfile(project_root, 'results', 'mpc_multirate_model.mat');
controller_source_file = fullfile(scripts_dir, 'mpc_controller_block.m');

required_files = { ...
    source_file, ...
    identified_model_file, ...
    controller_source_file};

for file_index = 1:numel(required_files)
    if ~isfile(required_files{file_index})
        error('Required file not found: %s', required_files{file_index});
    end
end

%% Load the identified fixed prediction model
loaded = load(identified_model_file, 'selected_model');
identified = loaded.selected_model;

% Controller tuning and hard constraints. The first five elements are the
% fixed ARX model; only weights and constraints should be tuned here.
mpc_params = [ ...
    identified.a1, ...
    identified.a2, ...
    identified.b1, ...
    identified.b2, ...
    identified.bias, ...
    1.0, ...   % output tracking weight
   40.00, ...  % selected input-move weight
    0.015, ... % selected |delta u| limit per 0.05 s update
   -1.15, ...  % lower actuator limit
    1.15, ...  % upper actuator limit
    0.030, ... % selected conditional integral gain
    0.030, ... % integral trim limit
    0.020, ... % error gate for integral action
    0.10];     % selected reference-filter coefficient per MPC update

%% Copy the source model at the file-system level before making changes.
% R2023a can block while performing save-as over an existing SLX file. A
% normal file copy followed by an in-place Simulink save avoids that path.
copyfile(source_file, target_file, 'f');
[~, model] = fileparts(target_file);
load_system(target_file);

controller_block = [model '/MFAPC_Controller'];

if getSimulinkBlockHandle(controller_block) == -1
    error('Expected controller block not found: %s', controller_block);
end

root = sfroot;
chart = root.find( ...
    '-isa', ...
    'Stateflow.EMChart', ...
    'Path', ...
    controller_block);

if numel(chart) ~= 1
    error('Expected one MATLAB Function chart at %s.', controller_block);
end

chart.Script = fileread(controller_source_file);
set_param(controller_block, 'Name', 'MPC_Controller');
set_param([model '/Lambda'], 'Value', mat2str(mpc_params, 17));

%% Rename logged variables so MPC results cannot overwrite MFAPC results
workspace_blocks = find_system(model, 'BlockType', 'ToWorkspace');

for block_index = 1:numel(workspace_blocks)
    block = workspace_blocks{block_index};
    variable_name = get_param(block, 'VariableName');

    switch variable_name
        case 'y_mfapc'
            set_param(block, 'VariableName', 'y_mpc');
        case 'u_mf'
            set_param(block, 'VariableName', 'u_mpc');
        case 'IAE_mfapc_20_50'
            set_param(block, 'VariableName', 'IAE_mpc_20_50');
        case 'ITAE_mfapc_20_50'
            set_param(block, 'VariableName', 'ITAE_mpc_20_50');
        case 'y_mf'
            set_param(block, 'VariableName', 'y_mpc_aux');
        case 'r_mf'
            set_param(block, 'VariableName', 'r_mpc');
    end
end

if getSimulinkBlockHandle([model '/MFAPC']) ~= -1
    set_param([model '/MFAPC'], 'Name', 'MPC');
end

if getSimulinkBlockHandle([model '/mfapc']) ~= -1
    set_param([model '/mfapc'], 'Name', 'mpc');
end

set_param(model, 'Description', [ ...
    'Offset-free multirate constrained MPC generated from ', ...
    'HT_ctrl_MFAPC_final.slx by scripts/build_mpc_model.m.']);

save_system(model);
close_system(model, 0);

save( ...
    fullfile(project_root, 'results', 'mpc_controller_parameters.mat'), ...
    'mpc_params');

fprintf('Created MPC model:\n%s\n', target_file);
fprintf('MPC parameter vector:\n%s\n', mat2str(mpc_params, 10));
