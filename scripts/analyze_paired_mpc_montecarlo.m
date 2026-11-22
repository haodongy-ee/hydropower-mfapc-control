function [paired, summary] = analyze_paired_mpc_montecarlo(n, seed)
%ANALYZE_PAIRED_MPC_MONTECARLO Compare phase-2 and phase-2.3 on identical cases.
%   Run after both Monte Carlo variants finish in MATLAB R2023a.
if nargin < 1, n = 100; end
if nargin < 2, seed = 2401; end
if ~strcmp(version('-release'), '2023a')
    error('This project uses MATLAB R2023a for Monte Carlo analysis.');
end

root = fileparts(fileparts(mfilename('fullpath')));
results_dir = fullfile(root, 'results');
figures_dir = fullfile(root, 'figures');
suffix = sprintf('montecarlo_n%d_seed%d', n, seed);
base_file = fullfile(results_dir, ['mpc_robustness_' suffix '_results.csv']);
upgrade_file = fullfile(results_dir, ['mpc_23_' suffix '_results.csv']);
base = readtable(base_file, 'TextType', 'string');
upgrade = readtable(upgrade_file, 'TextType', 'string');
if height(base) ~= n || height(upgrade) ~= n || ...
        any(base.Status == "") || any(upgrade.Status == "")
    error('Both variants need %d completed cases.', n);
end
design_names = {'CaseID', 'Category', 'Description', 'Profile', ...
    'ActuatorGainScale', 'ActuatorTimeConstant', 'NoiseStd', ...
    'NoiseSeed', 'LoadPowerMW', 'LoadInitialState', 'LoadSwitchTime', ...
    'RateLimit', 'MeasurementDelay'};
if ~isequaln(base(:, design_names), upgrade(:, design_names)) || ...
        numel(unique(base.CaseID)) ~= n
    error('Case IDs or experimental inputs differ between variants.');
end

paired = table(base.CaseID, base.Profile, base.Status, upgrade.Status, ...
    base.Pass, upgrade.Pass, 'VariableNames', ...
    {'CaseID', 'Profile', 'Phase2Status', 'Phase23Status', ...
    'Phase2Pass', 'Phase23Pass'});
old_pass = logical(base.Pass);
new_pass = logical(upgrade.Pass);
transition = repmat("neither_pass", n, 1);
transition(old_pass & new_pass) = "both_pass";
transition(~old_pass & new_pass) = "upgrade_only";
transition(old_pass & ~new_pass) = "phase2_only";
paired.PassTransition = transition;
valid = base.Status == "success" & upgrade.Status == "success";
metric_names = {'IAE', 'ITAE', 'MaximumOvershootPct', ...
    'SteadyError', 'SteadyFluctuation', 'SettlingTime35', ...
    'ControlTotalVariation'};
metric_labels = ["IAE", "ITAE", "MaximumOvershootPct", ...
    "SteadyError", "SteadyFluctuation", "SettlingTime35", ...
    "ControlTotalVariation"].';
count = numel(metric_names);
PairedN = zeros(count, 1);
Phase2Mean = NaN(count, 1);
Phase23Mean = NaN(count, 1);
MeanDelta = NaN(count, 1);
MedianDelta = NaN(count, 1);
P95Delta = NaN(count, 1);
RelativeMeanChangePct = NaN(count, 1);
ImprovedCount = zeros(count, 1);
WorsenedCount = zeros(count, 1);
TieCount = zeros(count, 1);

for k = 1:count
    name = metric_names{k};
    if strcmp(name, 'MaximumOvershootPct')
        before = max(base.DirectionalOvershoot20, ...
            base.DirectionalOvershoot35);
        after = max(upgrade.DirectionalOvershoot20, ...
            upgrade.DirectionalOvershoot35);
    else
        before = base.(name);
        after = upgrade.(name);
    end
    before(~valid) = NaN;
    after(~valid) = NaN;
    difference = after - before; % Negative means phase 2.3 is better.
    paired.(['Phase2' name]) = before;
    paired.(['Phase23' name]) = after;
    paired.(['Delta' name]) = difference;
    good = isfinite(before) & isfinite(after);
    PairedN(k) = sum(good);
    if PairedN(k) == 0, continue; end
    a = before(good);
    b = after(good);
    d = difference(good);
    Phase2Mean(k) = mean(a);
    Phase23Mean(k) = mean(b);
    MeanDelta(k) = mean(d);
    MedianDelta(k) = linear_percentile(d, 0.5);
    P95Delta(k) = linear_percentile(d, 0.95);
    if Phase2Mean(k) ~= 0
        RelativeMeanChangePct(k) = 100 * MeanDelta(k) / Phase2Mean(k);
    end
    tolerance = 1e-10 * max(1, max(abs([a; b])));
    ImprovedCount(k) = sum(d < -tolerance);
    WorsenedCount(k) = sum(d > tolerance);
    TieCount(k) = PairedN(k) - ImprovedCount(k) - WorsenedCount(k);
end

Metric = metric_labels;
summary = table(Metric, PairedN, Phase2Mean, Phase23Mean, ...
    MeanDelta, MedianDelta, P95Delta, RelativeMeanChangePct, ...
    ImprovedCount, WorsenedCount, TieCount);
prefix = fullfile(results_dir, sprintf('mpc_paired_n%d_seed%d', n, seed));
writetable(paired, [prefix '_cases.csv']);
writetable(summary, [prefix '_summary.csv']);
save([prefix '.mat'], 'paired', 'summary', 'n', 'seed');
make_paired_scatter(paired, figures_dir, n, seed);

fprintf('Paired successful cases: %d/%d\n', sum(valid), n);
fprintf('Acceptance passes, phase 2: %d; phase 2.3: %d\n', ...
    sum(base.Pass), sum(upgrade.Pass));
disp(summary);
end

function value = linear_percentile(values, p)
values = sort(values(:));
position = 1 + p * (numel(values)-1);
lo = floor(position);
hi = ceil(position);
value = values(lo) + (position-lo) * (values(hi)-values(lo));
end

function make_paired_scatter(paired, directory, n, seed)
metrics = {'IAE', 'ITAE', 'MaximumOvershootPct', 'SteadyError'};
labels = {'IAE (pu s)', 'ITAE (pu s^2)', ...
    'Maximum overshoot (%)', 'Final steady error (milli-pu)'};
short_names = {'IAE', 'ITAE', 'Maximum overshoot', 'Steady error'};
f = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [80 80 1050 780]);
layout = tiledlayout(f, 2, 2, 'Padding', 'compact', ...
    'TileSpacing', 'compact');
for k = 1:numel(metrics)
    name = metrics{k};
    before = paired.(['Phase2' name]);
    after = paired.(['Phase23' name]);
    if strcmp(name, 'SteadyError')
        before = 1000 * before;
        after = 1000 * after;
    end
    good = isfinite(before) & isfinite(after);
    ax = nexttile(layout); hold(ax, 'on'); grid(ax, 'on');
    values = [before(good); after(good)];
    upper = max(values);
    lower = min(0, min(values));
    if upper <= lower, upper = lower + 1; end
    padding = 0.05 * (upper - lower);
    bounds = [lower-padding, upper+padding];
    plot(ax, bounds, bounds, '--', 'Color', [0.45 0.48 0.52], ...
        'LineWidth', 1.2);
    scatter(ax, before(good), after(good), 20, [0.04 0.52 0.55], ...
        'filled', 'MarkerFaceAlpha', 0.55);
    xlim(ax, bounds); ylim(ax, bounds); axis(ax, 'square');
    xlabel(ax, ['Phase 2 ' labels{k}]);
    ylabel(ax, ['Phase 2.3 ' labels{k}]);
    title(ax, sprintf('%s: %d/%d cases improved', short_names{k}, ...
        sum(after(good) < before(good)), sum(good)), ...
        'Interpreter', 'none');
end
title(layout, sprintf('Matched Monte Carlo cases (n=%d; below line is better)', n));
if n == 100 && seed == 2401
    figure_name = 'montecarlo_paired_scatter.png';
else
    figure_name = sprintf('montecarlo_paired_scatter_n%d_seed%d.png', n, seed);
end
exportgraphics(f, fullfile(directory, figure_name), ...
    'Resolution', 160);
close(f);
end
