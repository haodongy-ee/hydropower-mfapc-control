function scorecard = build_three_controller_scorecard()
%BUILD_THREE_CONTROLLER_SCORECARD Descriptive score on six common profiles.
% The score uses equal weights for five lower-is-better mean metrics.
% It is a within-table comparison, not a safety or robustness certificate.
project_root = fileparts(fileparts(mfilename('fullpath')));
source = fullfile(project_root, 'results', ...
    'three_controller_offsetfree_comparison.csv');
data = readtable(source, 'TextType', 'string');
controllers = ["MFAC_baseline"; "MFAPC_final"; "MPC_offsetfree"];
if height(data) ~= 18 || numel(unique(data.Scenario)) ~= 6
    error('Expected 18 successful rows from six common scenarios.');
end
if any(data.Status ~= "success")
    error('Scorecard cannot include failed simulations.');
end

IAE = zeros(3, 1);
ITAE = zeros(3, 1);
MaximumOvershootPct = zeros(3, 1);
SettlingTime35 = zeros(3, 1);
SteadyError = zeros(3, 1);
for index = 1:3
    rows = data(data.Controller == controllers(index), :);
    if height(rows) ~= 6 || numel(unique(rows.Scenario)) ~= 6
        error('Controller %s lacks one or more paired scenarios.', controllers(index));
    end
    IAE(index) = mean(rows.IAE);
    ITAE(index) = mean(rows.ITAE);
    MaximumOvershootPct(index) = mean(max( ...
        rows.DirectionalOvershoot20, rows.DirectionalOvershoot35));
    SettlingTime35(index) = mean(rows.SettlingTime35);
    SteadyError(index) = mean(rows.SteadyError);
end
metric_values = [IAE, ITAE, MaximumOvershootPct, ...
    SettlingTime35, SteadyError];
if any(~isfinite(metric_values), 'all')
    error('Scorecard contains a missing or nonfinite metric.');
end
metric_scores = zeros(size(metric_values));
for column = 1:size(metric_values, 2)
    lo = min(metric_values(:, column));
    hi = max(metric_values(:, column));
    if hi == lo
        metric_scores(:, column) = 100;
    else
        metric_scores(:, column) = 100 * ...
            (hi - metric_values(:, column)) / (hi - lo);
    end
end
CompositeScore = mean(metric_scores, 2);
scorecard = table(controllers, IAE, ITAE, MaximumOvershootPct, ...
    SettlingTime35, SteadyError, CompositeScore, ...
    'VariableNames', {'Controller', 'IAE', 'ITAE', ...
    'MaximumOvershootPct', 'SettlingTime35', 'SteadyError', ...
    'CompositeScore'});
writetable(scorecard, fullfile(project_root, 'results', ...
    'three_controller_scorecard.csv'));
disp(scorecard);
end
