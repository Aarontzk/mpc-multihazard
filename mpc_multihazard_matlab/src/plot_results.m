function plot_results(scenario, runs, sys, savepath)
% PLOT_RESULTS  Render comparative time-history plots.
%   runs : struct array with fields {label, x_log, u_log, t}
%   sys  : system struct (uses sys.n)
%   savepath : optional .png path (folder must exist)

n = sys.n;
fig = figure('Position', [80 80 1280 820], 'Color', 'w');

% --- Panel 1: floor displacements (one subplot per floor) ---
for i = 1:n
    subplot(n+1, 2, 2*(i-1)+1); hold on; grid on;
    for r = 1:numel(runs)
        plot(runs(r).t, runs(r).x_log(i,:), 'DisplayName', runs(r).label, ...
            'LineWidth', 1.2);
    end
    ylabel(sprintf('q_%d [m]', i));
    if i == 1
        title(sprintf('Floor displacements - %s', scenario), 'FontSize', 12);
        legend('Location','best');
    end
end
xlabel('time [s]');

% --- Panel 2: inter-story drifts ---
for i = 1:n
    subplot(n+1, 2, 2*(i-1)+2); hold on; grid on;
    for r = 1:numel(runs)
        if i == 1
            d = runs(r).x_log(i,:);
        else
            d = runs(r).x_log(i,:) - runs(r).x_log(i-1,:);
        end
        plot(runs(r).t, d, 'DisplayName', runs(r).label, 'LineWidth', 1.2);
    end
    ylabel(sprintf('drift_%d [m]', i));
    if i == 1
        title('Inter-story drifts', 'FontSize', 12);
    end
end
xlabel('time [s]');

% --- Bottom row: control forces + cumulative energy ---
subplot(n+1, 2, 2*n+1); hold on; grid on;
for r = 1:numel(runs)
    if isempty(runs(r).u_log); continue; end
    for i = 1:size(runs(r).u_log,1)
        plot(runs(r).t(1:end-1), runs(r).u_log(i,:), ...
            'DisplayName', sprintf('%s u_%d', runs(r).label, i), ...
            'LineWidth', 1.0);
    end
end
ylabel('u [N]');
xlabel('time [s]');
title('Hybrid actuator forces (MLFS+FAHFS)');
legend('Location','best','NumColumns',2);

subplot(n+1, 2, 2*n+2); hold on; grid on;
for r = 1:numel(runs)
    if isempty(runs(r).u_log); continue; end
    e = cumsum(sum(runs(r).u_log.^2, 1)) * (runs(r).t(2)-runs(r).t(1));
    plot(runs(r).t(1:end-1), e, 'DisplayName', runs(r).label, 'LineWidth',1.2);
end
ylabel('cum. energy [J s.u.]');
xlabel('time [s]');
title('Cumulative control energy');

if nargin >= 4 && ~isempty(savepath)
    if exist('exportgraphics', 'file') == 2
        exportgraphics(fig, savepath, 'Resolution', 150);
    else
        % Older MATLAB fallback
        print(fig, savepath, '-dpng', '-r150');
    end
end
end
