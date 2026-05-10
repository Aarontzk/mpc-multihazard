function print_metrics(label, m, sys)
% PRINT_METRICS  Pretty-print metric summary for one run.
n = sys.n;
fprintf('\n--- %s ---\n', label);
fprintf('  Peak displacement [mm] : ');
fprintf('%7.2f ', 1e3*m.peak_disp);
fprintf('\n  Peak drift        [mm] : ');
fprintf('%7.2f ', 1e3*m.peak_drift);
fprintf('\n  Peak acceleration [m/s^2]: ');
fprintf('%7.2f ', m.peak_acc);
fprintf('\n  Settling time      [s] : ');
fprintf('%7.2f ', m.settling);
fprintf('\n  Peak control force [N] : ');
if isempty(m.peak_force) || all(m.peak_force == 0)
    fprintf('  (uncontrolled)');
else
    fprintf('%7.1f ', m.peak_force);
end
fprintf('\n  Control energy         : %.3e\n', m.ctrl_energy);
fprintf('  Jain fairness (drifts) : %.4f\n', m.fairness);
if ~isnan(m.reduction_avg)
    fprintf('  Avg peak reduction (%%) : %.2f\n', m.reduction_avg);
    fprintf('  Per-floor reduction[%%] : ');
    fprintf('%7.2f ', m.reduction);
    fprintf('\n');
end
end
