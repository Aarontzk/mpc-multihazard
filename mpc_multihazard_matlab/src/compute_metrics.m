function m = compute_metrics(out, sys, baseline, ref_traj)
% COMPUTE_METRICS  KPIs including FAHFS elevation tracking + inundation.
%
%   m = compute_metrics(out, sys [, baseline [, ref_traj]])

if nargin < 3; baseline = []; end
if nargin < 4; ref_traj = []; end

n  = sys.n;
Ts = sys.Ts;

q  = out.x_log(sys.idx_q,  :);
qd = out.x_log(sys.idx_qd, :);
zb = out.x_log(sys.idx_z,  :);
zd = out.x_log(sys.idx_zd, :);

m.peak_disp  = max(abs(q), [], 2);
drift = q;  drift(2:end,:) = q(2:end,:) - q(1:end-1,:);
m.peak_drift = max(abs(drift), [], 2);
qdd = [zeros(n,1), diff(qd,1,2)/Ts];
m.peak_acc = max(abs(qdd), [], 2);
m.rms_disp = sqrt(mean(q.^2, 2));

m.settling = zeros(n,1);
for i = 1:n
    thr = 0.05 * m.peak_disp(i);
    idx = find(abs(q(i,:)) > thr, 1, 'last');
    if isempty(idx); m.settling(i) = 0; else; m.settling(i) = idx*Ts; end
end

if isempty(out.u_log)
    m.ctrl_energy = 0;
    m.peak_force  = zeros(sys.nu,1);
else
    m.ctrl_energy = sum(out.u_log(:).^2) * Ts;
    m.peak_force  = max(abs(out.u_log), [], 2);
end

m.fairness = (sum(m.peak_drift))^2 / (n * sum(m.peak_drift.^2) + eps);

% Elevation / FAHFS metrics
m.peak_z_base = max(zb);
m.final_z_base = zb(end);
if ~isempty(ref_traj)
    z_ref = ref_traj(sys.idx_z, 1:size(zb,2));
    m.z_track_rmse = sqrt(mean((zb - z_ref).^2));
    m.z_track_peak_err = max(abs(zb - z_ref));
else
    m.z_track_rmse = NaN;
    m.z_track_peak_err = NaN;
end

% Inundation: max depth water exceeds the BOTTOM of floor 1 (= z_base + 0)
if isfield(out, 'inundation')
    m.peak_inundation = max(out.inundation);
else
    m.peak_inundation = NaN;
end

if ~isempty(baseline)
    base_peak = max(abs(baseline.x_log(sys.idx_q,:)), [], 2);
    m.reduction = 100 * (base_peak - m.peak_disp) ./ max(base_peak, eps);
    m.reduction_avg = mean(m.reduction);
else
    m.reduction = nan(n,1);
    m.reduction_avg = NaN;
end
end
