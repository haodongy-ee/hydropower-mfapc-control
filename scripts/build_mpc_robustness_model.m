function build_mpc_robustness_model()
%BUILD_MPC_ROBUSTNESS_MODEL Create a non-destructive phase-2 test model.
% The frozen phase-1 model is copied before adding measurement-noise and
% delay injection. Plant, load, and actuator perturbations remain variables
% so one model can be reused for the complete screening matrix.

script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
source_file = fullfile( ...
    project_root, 'baselines', 'phase1_offsetfree_mpc', ...
    'HT_ctrl_MPC_phase1.slx');
target_file = fullfile( ...
    project_root, 'models', 'HT_ctrl_MPC_robustness.slx');

if ~isfile(source_file)
    error('Frozen phase-1 model not found: %s', source_file);
end

copyfile(source_file, target_file, 'f');
[~, model] = fileparts(target_file);
load_system(target_file);

cleanup_object = onCleanup(@() close_if_loaded(model)); %#ok<NASGU>

required_blocks = { ...
    [model '/Transfer Fcn'], ...
    [model '/Saturation'], ...
    [model '/5 MW1'], ...
    [model '/Three-Phase Breaker1'], ...
    [model '/From4'], ...
    [model '/MPC_Controller']};
validate_blocks(required_blocks);

% Expose the principal plant/load/actuator quantities to the runner.
set_param([model '/Transfer Fcn'], ...
    'Numerator', '[robust_actuator_gain_scale*10/3]', ...
    'Denominator', '[robust_actuator_time_constant 1]');
set_param([model '/Saturation'], ...
    'UpperLimit', 'robust_actuator_rate_limit', ...
    'LowerLimit', '-robust_actuator_rate_limit');
set_param([model '/5 MW1'], ...
    'ActivePower', 'robust_load_power_w');
set_param([model '/Three-Phase Breaker1'], ...
    'SwitchTimes', '[robust_load_switch_time]');

noise_block = [model '/Robustness Measurement Noise'];
sum_block = [model '/Robustness Measurement Sum'];
delay_block = [model '/Robustness Measurement Delay'];

add_block('simulink/Sources/Random Number', noise_block, ...
    'Position', [-1780, -300, -1680, -260], ...
    'Mean', '0', ...
    'Variance', 'robust_noise_std^2', ...
    'Seed', 'robust_noise_seed', ...
    'SampleTime', '0.01');
add_block('simulink/Math Operations/Add', sum_block, ...
    'Position', [-1635, -250, -1605, -210], ...
    'Inputs', '++');
add_block('simulink/Continuous/Transport Delay', delay_block, ...
    'Position', [-1570, -245, -1470, -215], ...
    'DelayTime', 'robust_measurement_delay_s', ...
    'InitialOutput', '0.5');

source_ports = get_param([model '/From4'], 'PortHandles');
controller_ports = get_param([model '/MPC_Controller'], 'PortHandles');

if get_param(controller_ports.Inport(1), 'Line') == -1
    error('MPC measurement input is not connected.');
end

% Delete only the branch entering the controller; the same source also
% feeds plant-output logging and must remain connected.
delete_line(model, source_ports.Outport(1), controller_ports.Inport(1));
add_line(model, source_ports.Outport(1), ...
    get_param(sum_block, 'PortHandles').Inport(1), 'autorouting', 'on');
add_line(model, get_param(noise_block, 'PortHandles').Outport(1), ...
    get_param(sum_block, 'PortHandles').Inport(2), 'autorouting', 'on');
add_line(model, get_param(sum_block, 'PortHandles').Outport(1), ...
    get_param(delay_block, 'PortHandles').Inport(1), 'autorouting', 'on');
add_line(model, get_param(delay_block, 'PortHandles').Outport(1), ...
    controller_ports.Inport(1), 'autorouting', 'on');

set_param(model, 'Description', [ ...
    'Phase-2 robustness harness generated from the frozen phase-1 ', ...
    'offset-free MPC baseline.']);
save_system(model);

fprintf('Created robustness model:\n%s\n', target_file);
end

function validate_blocks(blocks)
for block_index = 1:numel(blocks)
    if getSimulinkBlockHandle(blocks{block_index}) == -1
        error('Required block not found: %s', blocks{block_index});
    end
end
end

function close_if_loaded(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end
