function [results_table, summary_table] = run_mpc_robustness_tests(mode)
%RUN_MPC_ROBUSTNESS_TESTS Execute deterministic phase-2 screening cases.
%   run_mpc_robustness_tests("smoke") runs two fast validation cases.
%   run_mpc_robustness_tests("screening") runs the full one-factor matrix.

if nargin < 1
    mode = "screening";
end

mode = lower(string(mode));

if ~ismember(mode, ["smoke", "screening"])
    error('Mode must be "smoke" or "screening".');
end

script_path = mfilename('fullpath');
scripts_dir = fileparts(script_path);
project_root = fileparts(scripts_dir);
results_dir = fullfile(project_root, 'results');
model_file = fullfile(project_root, 'models', 'HT_ctrl_MPC_robustness.slx');

if ~isfile(model_file)
    build_mpc_robustness_model();
end

all_cases = robustness_case_matrix();
writetable(all_cases, fullfile(results_dir, 'mpc_robustness_test_matrix.csv'));

if mode == "smoke"
    selected_ids = ["baseline", "noise_003"];
    cases = all_cases(ismember(all_cases.CaseID, selected_ids), :);
else
    cases = all_cases;
end

load_system(model_file);
[~, model] = fileparts(model_file);
set_param(model, 'FastRestart', 'off', 'StopTime', '50');
disable_abs_zero_crossing(model);

step_blocks = { ...
    [model '/wref step'], ...
    [model '/wref step1'], ...
    [model '/wref step2']};
validate_blocks(step_blocks);

cleanup_object = onCleanup(@() clean_up(model)); %#ok<NASGU>
number_of_cases = height(cases);

CaseID = cases.CaseID;
Category = cases.Category;
Description = cases.Description;
Status = strings(number_of_cases, 1);
ErrorMessage = strings(number_of_cases, 1);
IAE = NaN(number_of_cases, 1);
ITAE = NaN(number_of_cases, 1);
DirectionalOvershoot20 = NaN(number_of_cases, 1);
DirectionalOvershoot35 = NaN(number_of_cases, 1);
SettlingTime35 = NaN(number_of_cases, 1);
SteadyError = NaN(number_of_cases, 1);
SteadyFluctuation = NaN(number_of_cases, 1);
ControlTotalVariation = NaN(number_of_cases, 1);
MaximumAbsoluteControl = NaN(number_of_cases, 1);
InputConstraintViolations = NaN(number_of_cases, 1);
MoveConstraintViolations = NaN(number_of_cases, 1);
SimulationWallTime = NaN(number_of_cases, 1);
Pass = false(number_of_cases, 1);

for case_number = 1:number_of_cases
    current = cases(case_number, :);
    step_values = profile_values(current.Profile);
    set_step_values(step_blocks, step_values);
    set_param([model '/Three-Phase Breaker1'], ...
        'InitialState', char(current.LoadInitialState));

    assignin('base', 'robust_actuator_gain_scale', ...
        current.ActuatorGainScale);
    assignin('base', 'robust_actuator_time_constant', ...
        current.ActuatorTimeConstant);
    assignin('base', 'robust_noise_std', current.NoiseStd);
    assignin('base', 'robust_noise_seed', current.NoiseSeed);
    assignin('base', 'robust_load_power_w', current.LoadPowerMW * 1e6);
    assignin('base', 'robust_load_switch_time', current.LoadSwitchTime);
    assignin('base', 'robust_actuator_rate_limit', current.RateLimit);
    assignin('base', 'robust_measurement_delay_s', ...
        current.MeasurementDelay);

    fprintf('Robustness case %d/%d: %s\n', ...
        case_number, number_of_cases, current.CaseID);
    timer = tic;

    try
        out = sim(model, 'ReturnWorkspaceOutputs', 'on');
        SimulationWallTime(case_number) = toc(timer);
        t = double(out.tout(:));
        y = double(out.y_mpc(:));
        u = double(out.u_mpc(:));
        reference = build_reference(t, step_values);
        metrics = calculate_metrics(t, y, u, reference, step_values);

        IAE(case_number) = metrics.IAE;
        ITAE(case_number) = metrics.ITAE;
        DirectionalOvershoot20(case_number) = metrics.Overshoot20;
        DirectionalOvershoot35(case_number) = metrics.Overshoot35;
        SettlingTime35(case_number) = metrics.SettlingTime35;
        SteadyError(case_number) = metrics.SteadyError;
        SteadyFluctuation(case_number) = metrics.SteadyFluctuation;
        ControlTotalVariation(case_number) = metrics.ControlTotalVariation;
        MaximumAbsoluteControl(case_number) = metrics.MaximumAbsoluteControl;
        InputConstraintViolations(case_number) = ...
            metrics.InputConstraintViolations;
        MoveConstraintViolations(case_number) = ...
            metrics.MoveConstraintViolations;
        Pass(case_number) = metrics.Pass;
        Status(case_number) = "success";
        ErrorMessage(case_number) = "";
    catch simulation_error
        SimulationWallTime(case_number) = toc(timer);
        Status(case_number) = "failed";
        ErrorMessage(case_number) = string(simulation_error.message);
        warning('Case %s failed: %s', ...
            current.CaseID, simulation_error.message);
    end
end

results_table = [cases, table( ...
    Status, ErrorMessage, IAE, ITAE, DirectionalOvershoot20, ...
    DirectionalOvershoot35, SettlingTime35, SteadyError, ...
    SteadyFluctuation, ControlTotalVariation, MaximumAbsoluteControl, ...
    InputConstraintViolations, MoveConstraintViolations, ...
    SimulationWallTime, Pass)];
summary_table = summarize_results(results_table);

csv_file = fullfile(results_dir, ...
    sprintf('mpc_robustness_%s_results.csv', mode));
summary_file = fullfile(results_dir, ...
    sprintf('mpc_robustness_%s_summary.csv', mode));
mat_file = fullfile(results_dir, ...
    sprintf('mpc_robustness_%s_results.mat', mode));
writetable(results_table, csv_file);
writetable(summary_table, summary_file);
save(mat_file, 'results_table', 'summary_table', 'all_cases');

disp(results_table);
disp(summary_table);
fprintf('Robustness results saved to:\n%s\n', csv_file);
end

function cases = robustness_case_matrix()
rows = { ...
"baseline","baseline","Frozen phase-1 nominal case","nominal",1.00,.070,0,101,3.0,"open",30,.100,0; ...
"gain_m20","plant_gain","Actuator gain -20%","nominal",.80,.070,0,102,3.0,"open",30,.100,0; ...
"gain_m10","plant_gain","Actuator gain -10%","nominal",.90,.070,0,103,3.0,"open",30,.100,0; ...
"gain_p10","plant_gain","Actuator gain +10%","nominal",1.10,.070,0,104,3.0,"open",30,.100,0; ...
"gain_p20","plant_gain","Actuator gain +20%","nominal",1.20,.070,0,105,3.0,"open",30,.100,0; ...
"tau_m20","time_constant","Actuator time constant -20%","nominal",1.00,.056,0,106,3.0,"open",30,.100,0; ...
"tau_m10","time_constant","Actuator time constant -10%","nominal",1.00,.063,0,107,3.0,"open",30,.100,0; ...
"tau_p10","time_constant","Actuator time constant +10%","nominal",1.00,.077,0,108,3.0,"open",30,.100,0; ...
"tau_p20","time_constant","Actuator time constant +20%","nominal",1.00,.084,0,109,3.0,"open",30,.100,0; ...
"noise_001","measurement_noise","Measurement noise 0.001 pu","nominal",1.00,.070,.001,201,3.0,"open",30,.100,0; ...
"noise_003","measurement_noise","Measurement noise 0.003 pu","nominal",1.00,.070,.003,202,3.0,"open",30,.100,0; ...
"noise_005","measurement_noise","Measurement noise 0.005 pu","nominal",1.00,.070,.005,203,3.0,"open",30,.100,0; ...
"load_add_15","load_disturbance","Add 1.5 MW load at 30 s","nominal",1.00,.070,0,301,1.5,"open",30,.100,0; ...
"load_add_60","load_disturbance","Add 6 MW load at 30 s","nominal",1.00,.070,0,302,6.0,"open",30,.100,0; ...
"load_reject_15","load_disturbance","Reject 1.5 MW load at 30 s","nominal",1.00,.070,0,303,1.5,"closed",30,.100,0; ...
"load_reject_30","load_disturbance","Reject 3 MW load at 30 s","nominal",1.00,.070,0,304,3.0,"closed",30,.100,0; ...
"rate_075","actuator_limit","Actuator rate limit 0.075 pu","nominal",1.00,.070,0,401,3.0,"open",30,.075,0; ...
"rate_050","actuator_limit","Actuator rate limit 0.050 pu","nominal",1.00,.070,0,402,3.0,"open",30,.050,0; ...
"delay_010","measurement_delay","Measurement delay 0.01 s","nominal",1.00,.070,0,501,3.0,"open",30,.100,.01; ...
"delay_020","measurement_delay","Measurement delay 0.02 s","nominal",1.00,.070,0,502,3.0,"open",30,.100,.02; ...
"delay_050","measurement_delay","Measurement delay 0.05 s","nominal",1.00,.070,0,503,3.0,"open",30,.100,.05; ...
"low_output","operating_point","Low-output reference profile","low",1.00,.070,0,601,3.0,"open",30,.100,0; ...
"high_output","operating_point","High-output reference profile","high",1.00,.070,0,602,3.0,"open",30,.100,0};

cases = cell2table(rows, 'VariableNames', { ...
    'CaseID','Category','Description','Profile','ActuatorGainScale', ...
    'ActuatorTimeConstant','NoiseStd','NoiseSeed','LoadPowerMW', ...
    'LoadInitialState','LoadSwitchTime','RateLimit','MeasurementDelay'});
cases.CaseID = string(cases.CaseID);
cases.Category = string(cases.Category);
cases.Description = string(cases.Description);
cases.Profile = string(cases.Profile);
cases.LoadInitialState = string(cases.LoadInitialState);
end

function values = profile_values(profile)
switch string(profile)
    case "nominal"
        values = [0.50, 0.25, 0.15];
    case "low"
        values = [0.40, 0.15, 0.10];
    case "high"
        values = [0.60, 0.25, 0.10];
    otherwise
        error('Unknown operating profile: %s', profile);
end
end

function reference = build_reference(t, values)
reference = values(1) * ones(size(t));
reference(t >= 20) = values(1) + values(2);
reference(t >= 35) = sum(values);
end

function metrics = calculate_metrics(t, y, u, reference, values)
target_20 = values(1) + values(2);
target_35 = sum(values);
index_metrics = (t >= 20) & (t <= 50);
metric_t = t(index_metrics);
absolute_error = abs(reference(index_metrics) - y(index_metrics));
metrics.IAE = trapz(metric_t, absolute_error);
metrics.ITAE = trapz(metric_t, (metric_t - 20) .* absolute_error);

index_20 = (t >= 20) & (t < 35);
index_35 = (t >= 35) & (t <= 50);
local_y_20 = y(index_20);
local_t_35 = t(index_35);
local_y_35 = y(index_35);
metrics.Overshoot20 = directional_overshoot( ...
    local_y_20, target_20, values(2));
metrics.Overshoot35 = directional_overshoot( ...
    local_y_35, target_35, values(3));

tolerance = 0.02 * max(abs(target_35), eps);
last_bad = find(abs(local_y_35 - target_35) > tolerance, 1, 'last');
if isempty(last_bad)
    metrics.SettlingTime35 = 0;
elseif last_bad < numel(local_t_35)
    metrics.SettlingTime35 = local_t_35(last_bad + 1) - 35;
else
    metrics.SettlingTime35 = NaN;
end

steady_y = y((t >= 45) & (t <= 50));
metrics.SteadyError = abs(mean(steady_y) - target_35);
metrics.SteadyFluctuation = max(steady_y) - min(steady_y);
metrics.ControlTotalVariation = sum(abs(diff(u)));
metrics.MaximumAbsoluteControl = max(abs(u));
metrics.InputConstraintViolations = sum(u < -1.15 - 1e-9 | u > 1.15 + 1e-9);
metrics.MoveConstraintViolations = sum(abs(diff(u)) > 0.015 + 1e-8);
metrics.Pass = metrics.InputConstraintViolations == 0 && ...
    metrics.SteadyError < 0.001 && ...
    metrics.Overshoot35 < 5 && ...
    metrics.SteadyFluctuation < 0.008;
end

function value = directional_overshoot(y, target, step_change)
denominator = max(abs(target), eps);
if step_change >= 0
    value = max(0, (max(y) - target) / denominator * 100);
else
    value = max(0, (target - min(y)) / denominator * 100);
end
end

function summary = summarize_results(results)
categories = unique(results.Category, 'stable');
Category = categories;
Cases = zeros(numel(categories), 1);
SuccessfulCases = zeros(numel(categories), 1);
PassedCases = zeros(numel(categories), 1);
SuccessRate = zeros(numel(categories), 1);
PassRate = zeros(numel(categories), 1);
P95IAE = NaN(numel(categories), 1);
P95Overshoot35 = NaN(numel(categories), 1);
P95SteadyError = NaN(numel(categories), 1);

for index = 1:numel(categories)
    rows = results(results.Category == categories(index), :);
    successful = rows.Status == "success";
    Cases(index) = height(rows);
    SuccessfulCases(index) = sum(successful);
    PassedCases(index) = sum(rows.Pass);
    SuccessRate(index) = SuccessfulCases(index) / Cases(index);
    PassRate(index) = PassedCases(index) / Cases(index);
    P95IAE(index) = percentile95(rows.IAE(successful));
    P95Overshoot35(index) = percentile95( ...
        rows.DirectionalOvershoot35(successful));
    P95SteadyError(index) = percentile95(rows.SteadyError(successful));
end

summary = table(Category, Cases, SuccessfulCases, PassedCases, ...
    SuccessRate, PassRate, P95IAE, P95Overshoot35, P95SteadyError);
end

function value = percentile95(data)
data = sort(data(isfinite(data)));
if isempty(data)
    value = NaN;
    return;
end
position = 1 + 0.95 * (numel(data) - 1);
lower_index = floor(position);
upper_index = ceil(position);
if lower_index == upper_index
    value = data(lower_index);
else
    weight = position - lower_index;
    value = data(lower_index) * (1 - weight) + data(upper_index) * weight;
end
end

function set_step_values(blocks, values)
for index = 1:numel(blocks)
    set_param(blocks{index}, 'After', num2str(values(index), 17));
end
end

function validate_blocks(blocks)
for index = 1:numel(blocks)
    if getSimulinkBlockHandle(blocks{index}) == -1
        error('Required block not found: %s', blocks{index});
    end
end
end

function disable_abs_zero_crossing(model)
blocks = find_system(model, 'BlockType', 'Abs');
for index = 1:numel(blocks)
    set_param(blocks{index}, 'ZeroCross', 'off');
end
end

function clean_up(model)
variables = { ...
    'robust_actuator_gain_scale', 'robust_actuator_time_constant', ...
    'robust_noise_std', 'robust_noise_seed', 'robust_load_power_w', ...
    'robust_load_switch_time', 'robust_actuator_rate_limit', ...
    'robust_measurement_delay_s'};
for index = 1:numel(variables)
    evalin('base', sprintf('clear %s', variables{index}));
end
if bdIsLoaded(model)
    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
end
