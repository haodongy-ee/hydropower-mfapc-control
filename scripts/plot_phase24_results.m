function plot_phase24_results(sample_count, sample_seed)
%PLOT_PHASE24_RESULTS Export publication-ready Monte Carlo figures in R2023a.
if nargin < 1, sample_count = 100; end
if nargin < 2, sample_seed = 2401; end
project_root = fileparts(fileparts(mfilename('fullpath')));
results_dir = fullfile(project_root, 'results');
figures_dir = fullfile(project_root, 'figures');
if ~isfolder(figures_dir), mkdir(figures_dir); end
suffix = sprintf('montecarlo_n%d_seed%d', sample_count, sample_seed);
new_prefix = fullfile(results_dir, ['mpc_23_' suffix]);
old_prefix = fullfile(results_dir, ['mpc_robustness_' suffix]);
new = readtable([new_prefix '_results.csv'], 'TextType', 'string');
if height(new) ~= sample_count || any(new.Status == "")
    error('Upgraded MPC Monte Carlo results are not complete.');
end
if isfile([old_prefix '_results.csv'])
    old = readtable([old_prefix '_results.csv'], 'TextType', 'string');
    if height(old) ~= sample_count || any(old.Status == "") || ...
            ~isequal(old.CaseID, new.CaseID)
        error('Phase-2 results are incomplete or not paired to phase 2.3.');
    end
else
    old = table();
end

make_heatmap(new, figures_dir, sample_count);
make_boxplots(old, new, figures_dir, sample_count);
make_disturbance_plot(old_prefix, new_prefix, figures_dir);
fprintf('Phase 2.4/2.5 figures saved under %s\n', figures_dir);
end

function make_heatmap(data, directory, count)
successful = data(data.Status == "success" & isfinite(data.IAE), :);
gain_edges = linspace(0.8, 1.2, 6);
tau_edges = linspace(0.056, 0.084, 6);
means = NaN(5, 5);
counts = zeros(5, 5);
for i = 1:5
    for j = 1:5
        g = successful.ActuatorGainScale >= gain_edges(j) & ...
            successful.ActuatorGainScale < gain_edges(j+1);
        t = successful.ActuatorTimeConstant >= tau_edges(i) & ...
            successful.ActuatorTimeConstant < tau_edges(i+1);
        if j == 5
            g = successful.ActuatorGainScale >= gain_edges(j) & ...
                successful.ActuatorGainScale <= gain_edges(j+1);
        end
        if i == 5
            t = successful.ActuatorTimeConstant >= tau_edges(i) & ...
                successful.ActuatorTimeConstant <= tau_edges(i+1);
        end
        values = successful.IAE(g & t);
        counts(i, j) = numel(values);
        if ~isempty(values), means(i, j) = mean(values); end
    end
end
f = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [80 80 820 650]);
ax = axes(f);
imagesc(ax, means, 'AlphaData', ~isnan(means));
ax.Color = [0.93 0.94 0.95];
colormap(ax, parula);
cb = colorbar(ax);
cb.Label.String = 'Mean IAE (pu s)';
ax.XTick = 1:5; ax.YTick = 1:5;
ax.XTickLabel = compose('%.2f', (gain_edges(1:5)+gain_edges(2:6))/2);
ax.YTickLabel = compose('%.3f', (tau_edges(1:5)+tau_edges(2:6))/2);
xlabel(ax, 'Actuator gain scale');
ylabel(ax, 'Actuator time constant (s)');
title(ax, sprintf('Joint-uncertainty IAE by gain and time constant (n=%d)', count));
for i = 1:5
    for j = 1:5
        if counts(i, j) > 0
            text_color = [0.05 0.10 0.16];
            finite_means = means(isfinite(means));
            if means(i, j) < (min(finite_means) + max(finite_means))/2
                text_color = [1 1 1];
            end
            text(ax, j, i, sprintf('%.3f\nn=%d', means(i,j), counts(i,j)), ...
                'HorizontalAlignment', 'center', 'FontSize', 10, ...
                'Color', text_color);
        else
            text(ax, j, i, 'no samples', 'HorizontalAlignment', 'center', ...
                'FontSize', 9, 'Color', [0.35 0.38 0.42]);
        end
    end
end
exportgraphics(f, fullfile(directory, 'montecarlo_parameter_heatmap.png'), ...
    'Resolution', 160);
close(f);
end

function make_boxplots(old, new, directory, count)
metrics = {'IAE', 'ITAE', 'MaximumOvershootPct', 'SteadyError'};
titles = {'IAE (pu s)', 'ITAE (pu s^2)', ...
    'Maximum directional overshoot (%)', 'Final steady error (pu)'};
f = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [80 80 1050 760]);
tiledlayout(f, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
for k = 1:4
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on');
    if ~isempty(old)
        draw_box(ax, select_values(old, metrics{k}), 1, [0.48 0.56 0.64]);
        draw_box(ax, select_values(new, metrics{k}), 2, [0.04 0.52 0.55]);
        ax.XTick = [1 2]; ax.XTickLabel = {'Phase 2', 'Phase 2.3'};
        xlim(ax, [0.4 2.6]);
    else
        draw_box(ax, select_values(new, metrics{k}), 1, [0.04 0.52 0.55]);
        ax.XTick = 1; ax.XTickLabel = {'Phase 2.3'};
        xlim(ax, [0.4 1.6]);
    end
    ylabel(ax, titles{k});
    if k == 1
        title(ax, sprintf('Monte Carlo distributions (%d attempted per variant)', count));
    end
end
exportgraphics(f, fullfile(directory, 'montecarlo_boxplots.png'), ...
    'Resolution', 160);
close(f);
end

function values = select_values(data, name)
successful = data.Status == "success";
if strcmp(name, 'MaximumOvershootPct')
    values = max(data.DirectionalOvershoot20(successful), ...
        data.DirectionalOvershoot35(successful));
else
    column = data.(name);
    values = column(successful);
end
values = values(isfinite(values));
end

function draw_box(ax, values, x, color)
if isempty(values), return; end
q1 = linear_percentile(values, 0.25);
q2 = linear_percentile(values, 0.50);
q3 = linear_percentile(values, 0.75);
iqr = q3 - q1;
whisker = values(values >= q1 - 1.5*iqr & values <= q3 + 1.5*iqr);
lo = min(whisker); hi = max(whisker);
patch(ax, [x-.27 x+.27 x+.27 x-.27], [q1 q1 q3 q3], color, ...
    'FaceAlpha', .32, 'EdgeColor', color, 'LineWidth', 1.5);
plot(ax, [x-.27 x+.27], [q2 q2], 'Color', color, 'LineWidth', 2.2);
plot(ax, [x x], [lo q1], 'Color', color, 'LineWidth', 1.4);
plot(ax, [x x], [q3 hi], 'Color', color, 'LineWidth', 1.4);
plot(ax, [x-.13 x+.13], [lo lo], 'Color', color, 'LineWidth', 1.4);
plot(ax, [x-.13 x+.13], [hi hi], 'Color', color, 'LineWidth', 1.4);
outliers = values(values < lo | values > hi);
if ~isempty(outliers)
    plot(ax, x*ones(size(outliers)), outliers, '.', 'Color', color, ...
        'MarkerSize', 8);
end
end

function value = linear_percentile(values, p)
values = sort(values(:));
position = 1 + p * (numel(values)-1);
a = floor(position); b = ceil(position);
value = values(a) + (position-a) * (values(b)-values(a));
end

function make_disturbance_plot(old_prefix, new_prefix, directory)
new_file = [new_prefix '_trace.mat'];
if ~isfile(new_file), error('No upgraded-MPC trace is available.'); end
loaded = load(new_file, 'trace');
updated = loaded.trace;
f = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [80 80 1000 570]);
plot(updated.t, updated.reference, '--', 'Color', [0.20 0.24 0.28], ...
    'LineWidth', 1.5); hold on;
if isfile([old_prefix '_trace.mat'])
    prior = load([old_prefix '_trace.mat'], 'trace');
    if string(prior.trace.CaseID) == string(updated.CaseID)
        plot(prior.trace.t, prior.trace.y, 'Color', [0.48 0.56 0.64], ...
            'LineWidth', 1.2);
        label_names = {'Reference', 'Phase 2', 'Phase 2.3', 'Load switch'};
    else
        label_names = {'Reference', 'Phase 2.3', 'Load switch'};
    end
else
    label_names = {'Reference', 'Phase 2.3', 'Load switch'};
end
plot(updated.t, updated.y, 'Color', [0.04 0.52 0.55], ...
    'LineWidth', 1.6);
xline(updated.LoadSwitchTime, ':', 'Color', [0.75 0.37 0.24], ...
    'LineWidth', 1.3);
grid on; xlim([0 50]);
xlabel('Time (s)'); ylabel('Active power (pu)');
title(sprintf('Disturbance response: %s, %.2f MW load, breaker initially %s', ...
    string(updated.CaseID), updated.LoadPowerMW, ...
    string(updated.LoadInitialState)), 'Interpreter', 'none');
legend(label_names, 'Location', 'best');
exportgraphics(f, fullfile(directory, 'montecarlo_disturbance_response.png'), ...
    'Resolution', 160);
close(f);
end
