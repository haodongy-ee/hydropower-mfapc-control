clear;
clc;

script_path = mfilename('fullpath');
project_root = fileparts(fileparts(script_path));
model_file = fullfile(project_root, 'models', 'HT_ctrl_MPC_baseline.slx');

load_system(model_file);
[~, model] = fileparts(model_file);
set_param(model, 'StopTime', '50');

abs_blocks = find_system(model, 'BlockType', 'Abs');
for block_index = 1:numel(abs_blocks)
    set_param(abs_blocks{block_index}, 'ZeroCross', 'off');
end

out = sim(model, 'ReturnWorkspaceOutputs', 'on');
t = double(out.tout(:));
y = double(out.y_mpc(:));
u = double(out.u_mpc(:));

reference = 0.5 * ones(size(t));
reference(t >= 20) = 0.75;
reference(t >= 35) = 0.90;
metric_index = t >= 20;
steady_index = t >= 45;

iae = trapz(t(metric_index), abs(reference(metric_index) - y(metric_index)));
itae = trapz( ...
    t(metric_index), ...
    (t(metric_index) - 20) .* abs(reference(metric_index) - y(metric_index)));
steady_error = abs(0.90 - mean(y(steady_index)));
steady_fluctuation = max(y(steady_index)) - min(y(steady_index));
local20 = y(t >= 20 & t < 35);
local35 = y(t >= 35);
overshoot20 = max(max(local20) - 0.75, 0) / 0.75 * 100;
overshoot35 = max(max(local35) - 0.90, 0) / 0.90 * 100;

fprintf('final_y=%.8f\n', y(end));
fprintf('final_u=%.8f\n', u(end));
fprintf('IAE=%.8f\n', iae);
fprintf('ITAE=%.8f\n', itae);
fprintf('steady_error=%.8f\n', steady_error);
fprintf('steady_fluctuation=%.8f\n', steady_fluctuation);
fprintf('overshoot20=%.8f%%\n', overshoot20);
fprintf('overshoot35=%.8f%%\n', overshoot35);
fprintf('u_range=[%.8f,%.8f]\n', min(u), max(u));

close_system(model, 0);
